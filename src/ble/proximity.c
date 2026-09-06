/*
 * BLE Smart Lock - Proximity Tracking
 * Phase 3 Lesson 1: phone-tracking registry + state machine skeleton
 *
 * See proximity.h for top-level documentation.
 *
 * ch1 scope:
 *   - phones[] static array of MAX_TRACKED_PHONES slots
 *   - proximity_init: zero array + schedule first tick
 *   - proximity_on_match: lookup-or-alloc + update fields + ABSENT->APPROACHING
 *   - proximity_tick: 30s timeout cleanup + DBG state log
 *   - All transitions logged at INF for audit
 *
 * Phase 3 ch2 changes (search for "Phase 3 ch2" to find them):
 *   1. kalman_update(): time-normalized scalar Kalman filter
 *      (design addendum 3-L1-A point (ii)):
 *        - dt from k_uptime timestamps, q_step = Q_PER_SEC * dt
 *        - first sample after alloc: x0 = z0, p0 = r (no convergence wait)
 *   2. fmt_float2(): fixed-point float formatting for logs (no
 *      CONFIG_CBPRINTF_FP_SUPPORT, no libm -- no roundf/fabsf anywhere)
 *   3. proximity_on_match(): feeds each MATCH through kalman_update
 *      BEFORE updating last_match_time/match_count (ordering matters!)
 *   4. proximity_tick(): DBG snapshot now includes kalman_x / kalman_p
 *
 * Phase 3 ch3 changes (search for "Phase 3 ch3"):
 *   1. trend_push(): decimated (x, t) insertion on every MATCH
 *      (>= TREND_DECIMATE_MS apart; stores kalman_x -- the ESTIMATE
 *      trajectory, which is the Issue #3 fix: never raw RSSI)
 *   2. trend_compute(): LSQ slope in dB/s over the 5 s time window,
 *      guards (>= TREND_MIN_POINTS, >= TREND_MIN_SPAN_MS) else INVALID;
 *      runs in the 1 s tick; window anchored at NOW so a silent phone
 *      self-invalidates within ~5 s (honesty, not a bug)
 *   3. 3-sigma innovation gate in kalman_update (PROX_KALMAN_GATE_SIGMA;
 *      squared comparison, no sqrtf; rejection keeps p = p_pred so the
 *      gate self-opens on genuine persistent level shifts)
 *   4. struct phone_track re-laid-out: raw rssi_buf[32] and
 *      smoothed_buf[5] DELETED (superseded by the decimated ring);
 *      zero-padding field order, 112 -> 136 bytes/slot; BUILD_ASSERT
 *      verifies the documented layout
 *   5. tick DBG now includes slope (or INVALID) + gated count
 *
 * Phase 3 ch3 lab changes (3-L3-C, after the walk-lab analysis):
 *   6. Circuit breaker in kalman_update: after N consecutive gate
 *      rejections the sample is force-accepted through the normal
 *      update (bounded staleness invariant -- see
 *      PROX_KALMAN_GATE_MAX_REJECTS in proximity.h). Driven by the
 *      new kalman_reject_streak field.
 *   7. kalman_gated uint16_t -> uint8_t, saturating at 255 (pays for
 *      the streak byte; sizeof(phone_track) stays 136).
 *
 * Phase 3 ch4 changes (search for "Phase 3 ch4"):
 *   1. evaluate_phone(): the evaluator -- threshold condition + dwell
 *      credit + trend modulation, per the decision table in
 *      proximity.h (contradiction pauses the credit; INVALID or
 *      below-floor is neutral; the nominal target runs. Confirm
 *      exists ONLY on the relock path, 5 s -> 3 s, fail-secure).
 *      DELIBERATELY no confirm on the unlock path: the L4 host
 *      simulation falsified a 2 s confirmed unlock (pass-through
 *      unlocked on the trend's direction lag).
 *   2. transition_phone(): fires UNLOCK/RELOCK events (registered
 *      callback + INF audit lines carrying the decision evidence:
 *      x, slope, dwell reached) and resets the dwell credit on every
 *      transition (the credit belongs to the old state's exit).
 *   3. proximity_tick_handler(): DEPARTING exits (5 s silence or
 *      x < -90 -> slot freed, distinct GONE audit lines) + the
 *      evidence-freshness gate (dwell FREEZES, never resets, while
 *      the newest MATCH is older than PROX_EVIDENCE_FRESH_MS) +
 *      evaluate_phone() right after trend_compute (rule R5).
 *   4. proximity_on_match(): first-sight floor (-ERANGE for a new
 *      phone at/below PROX_TRACK_FLOOR_RSSI) -- registry hygiene,
 *      allocation-time only.
 *   5. struct phone_track: + uint32_t dwell_ms; 136 -> 144 bytes/slot
 *      (4 documented tail-padding bytes; BUILD_ASSERT + offset
 *      table updated).
 *   6. tick DBG snapshot: + dwell=%ums (the lab's modulation view:
 *      watch the credit climb, pause, and reset).
 *   7. fmt_slope(): the only sanctioned way to print trend_slope --
 *      makes rule R1 structural (never read the slope without
 *      checking trend_valid).
 *
 * Float discipline (Cortex-M4F, single-precision FPv4 FPU):
 *   - float only, never double; ALL float constants f-suffixed
 *   - CONFIG_FPU=y in prj.conf (hard-float ABI for the whole build)
 *   - FP_SHARING: Zephyr 4.4 turns it on automatically when FPU=y (verified
 *     in the student's generated .config -- L2 review catch). That is good:
 *     FPU registers are saved across context switches with LAZY stacking,
 *     so only threads that actually execute FP pay the ~132-byte context
 *     cost. Leave it enabled; do not force it off.
 *   - No FP in ISRs (unsupported on ARMv7-M in Zephyr) -- both FP paths
 *     run in thread (workqueue) context, verified.
 *
 * Threading (carried from Phase 2 ch4):
 *   - proximity_on_match runs in scan_work_handler (system workqueue)
 *   - proximity_tick runs in its own k_work_delayable (also system workqueue)
 *   - System workqueue is single-threaded -> no locking on phones[]
 *   - Phase 4+ will move heavy work (HMAC, NVS) to a dedicated workqueue
 */

#include "proximity.h"
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>   /* snprintk */
#include <string.h>             /* memset, memcmp, memcpy */

/*
 * Pin this module to DBG (Phase 3 L3-E: "quiet world, loud pins").
 *
 * This is the TELEMETRY module: kalman:/trend:/tick:/gate: lines are the
 * tuning view we want in every Phase 3 lab, permanently. Volume is
 * bounded by design -- ~1 line/s per active phone per stream (tick,
 * slope, push, kalman = ~4-5 lines/s total at 1 Hz adv + 100% duty),
 * far under what the CDC ACM console drains. Contrast scan.c: pinned
 * to INF because its only DBG site fires for EVERY non-matching adv
 * in the air (the L3-B firehose).
 *
 * Requires CONFIG_LOG_MAX_LEVEL=4 in prj.conf: a module's effective
 * compiled level is min(pin, MAX), and MAX defaults to DEFAULT_LEVEL.
 * With DEFAULT=3 and no explicit MAX, this DBG pin would be silently
 * clamped to INF and the data lines would never appear.
 */
LOG_MODULE_REGISTER(proximity, LOG_LEVEL_DBG);
/* --- The registry: static array of phone_track records ---------------- */

static struct phone_track phones[MAX_TRACKED_PHONES];

/* Phase 3 ch4: verify the documented layout (offset table in proximity.h).
 * If this fires after you edit the struct, update the table -- the docs
 * and the code must not drift apart. BUILD_ASSERT is compile-time only,
 * zero runtime cost (<zephyr/toolchain.h>, pulled in by kernel.h).
 */
BUILD_ASSERT(sizeof(struct phone_track) == 144);

/*
 * Phase 3 ch4: actuation event callback (registered by main via
 * proximity_register_event_cb). NULL until then. Fired from
 * transition_phone -- see the contract in proximity.h: system
 * workqueue context, must not block.
 */
static proximity_event_cb_t event_cb;

/* Phase 3 ch4: forward declarations -- transition_phone (below) formats
 * the UNLOCK/RELOCK audit lines with helpers defined later in this file.
 */
static void fmt_float2(char *out, size_t len, float v);
static void fmt_slope(char *out, size_t len, const struct phone_track *ph);

/* Indexed by enum phone_state. Must stay in sync with the enum. */
const char *const phone_state_name[4] = {
    "ABSENT",
    "APPROACHING",
    "PRESENT",
    "DEPARTING",
};

/* --- Internal helpers (not in public API) ----------------------------- */

/*
 * Format 16-byte UUID as 32 lowercase hex chars + null terminator.
 * Output buffer must be >= 33 bytes. Matches the format used in scan.c.
 *
 * Duplicated from scan.c (which has its own static format_uuid_hex) to keep
 * the proximity module self-contained. L2/L3 can refactor into a shared
 * util if a third consumer appears.
 */
static void format_uuid_hex(const uint8_t uuid[MSD_LEN_UUID],
                           char *out, size_t out_len)
{
    if (out_len < (size_t)(MSD_LEN_UUID * 2U) + 1U) {
        return;  /* buffer too small -- caller error, silently bail */
    }
    for (uint8_t i = 0U; i < MSD_LEN_UUID; i++) {
        (void)snprintk(&out[i * 2U], 3U, "%02x", uuid[i]);
    }
    out[MSD_LEN_UUID * 2U] = '\0';
}

/*
 * Find a phone by UUID. Linear scan of in_use slots.
 *
 * Returns: slot index 0..MAX_TRACKED_PHONES-1 if found, -1 if not found.
 *
 * Cost: at most 8 memcmp(16-byte) = 128 bytes compared.
 * On Cortex-M4 with optimized memcmp: ~50 cycles. Trivial.
 *
 * Why not a hash table: 8 entries is below the crossover where hashing's
 * constant overhead beats linear scan's O(n). Plus linear scan is simpler,
 * auditable, and has no hash collisions to debug.
 */
static int find_phone(const uint8_t *uuid)
{
    for (int i = 0; i < MAX_TRACKED_PHONES; i++) {
        if (phones[i].in_use &&
            memcmp(phones[i].uuid, uuid, MSD_LEN_UUID) == 0) {
            return i;
        }
    }
    return -1;
}

/*
 * Find a free slot. Linear scan for in_use == false.
 *
 * Returns: slot index 0..MAX_TRACKED_PHONES-1 if free slot found,
 *          -1 if all slots in use.
 *
 * Note: we do NOT evict. If the registry is full, the caller
 * (proximity_on_match) logs a WRN and drops the MATCH. The phone will be
 * tracked on a future MATCH after a slot frees up (30s timeout of an
 * inactive phone).
 *
 * Eviction policy (LRU, oldest-last_match_time) was considered and rejected:
 *   - Adds complexity (time comparison on every alloc)
 *   - Risk: evicting a phone that's about to send a MATCH (resident
 *     momentarily in a multipath null, then comes back strong)
 *   - 8 slots is enough for residential (15-25 residents, at most ~half
 *     near a gate at once). Registry full should be vanishingly rare.
 *   - If it becomes a problem in Phase 6 (false negatives during parties),
 *     bump MAX_TRACKED_PHONES to 16 or 32 -- RAM cost is trivial.
 */
static int find_free_slot(void)
{
    for (int i = 0; i < MAX_TRACKED_PHONES; i++) {
        if (!phones[i].in_use) {
            return i;
        }
    }
    return -1;
}

/*
 * Allocate a slot for a new phone UUID.
 *
 * Returns: slot index on success, -1 if registry full.
 *
 * Initializes all fields to safe defaults:
 *   - uuid: copied from input
 *   - in_use: true
 *   - state: ABSENT (proximity_on_match will immediately transition to
 *     APPROACHING)
 *   - last_rssi, match_count, last_match_time, state_enter_time: 0 from
 *     memset (will be set by caller)
 *   - kalman_x, kalman_p: 0.0 from memset -- kalman_update initializes
 *     them to (x0=z0, p0=r) on the first MATCH (match_count==0 sentinel)
 *   - trend ring / slope / valid / gated count: zeroed -- trend_push
 *     inserts the first (x, t) point on that same first MATCH
 *
 * We memset the whole struct first (cheap, gets all padding too) then set
 * the fields we need. This is safer than partial init (no uninitialized
 * padding bytes that could leak info if we ever dump the struct).
 */
static int alloc_phone(const uint8_t *uuid)
{
    int slot = find_free_slot();
    if (slot < 0) {
        return -1;
    }

    memset(&phones[slot], 0, sizeof(phones[slot]));
    memcpy(phones[slot].uuid, uuid, MSD_LEN_UUID);
    phones[slot].in_use = true;
    phones[slot].state  = PHONE_STATE_ABSENT;
    /* last_rssi, match_count, last_match_time, state_enter_time all 0 from
     * memset. Kalman/trend fields also 0 -- kalman_update/trend_push
     * initialize them on the first MATCH.
     */

    return slot;
}

/*
 * Free a slot. Sets in_use=false. Other fields are left as-is (will be
 * re-zeroed on next alloc_phone via memset).
 *
 * Why not memset on free: if we memset on free, a concurrent reader
 * (proximity_get_slot from main thread) might see a half-zeroed struct.
 * With in_use=false set first, the reader checks in_use and bails out if
 * false. Memory ordering on Cortex-M4 is strong enough (no store-store
 * reordering) that this works without a barrier.
 *
 * Actually, single-threaded system workqueue means NO concurrent readers
 * during state mutation. The only "concurrent" access is proximity_get_slot
 * from main thread (for status display), which is best-effort anyway.
 * So memset-on-free would also be fine. We choose in_use=false-first because
 * it's defensive (works even if we later add a dedicated workqueue).
 */
static void free_phone(int slot)
{
    if (slot < 0 || slot >= MAX_TRACKED_PHONES) {
        return;
    }
    phones[slot].in_use = false;
    /* Other fields left as-is. alloc_phone will memset on reuse. */
}

/*
 * Transition a phone to a new state.
 *
 * Logs the transition at INF for audit. Updates state_enter_time.
 *
 * In L1, the only callers are:
 *   - proximity_on_match: ABSENT -> APPROACHING (on first MATCH for a new
 *     phone)
 *   - proximity_tick: Any -> ABSENT (on 30s timeout, via free_phone which
 *     sets in_use=false -- transition_phone is NOT called for timeout, the
 *     TIMEOUT log line covers it)
 *
 * Wait -- re-reading the L1 code, timeout does NOT call transition_phone.
 * It calls free_phone directly. So transition_phone is only called for
 * ABSENT -> APPROACHING in L1. That's correct -- timeout frees the slot
 * (in_use=false) without a state transition log (the TIMEOUT log line is
 * the audit record).
 *
 * ch4 delivered: transition_phone is called by evaluate_phone (the
 * tick's evaluator) for APPROACHING -> PRESENT, DEPARTING -> PRESENT
 * (the return edge) and PRESENT -> DEPARTING; the UNLOCK/RELOCK
 * events + audit lines fire below on the actuation-relevant entries,
 * and the dwell credit is reset here on EVERY transition. DEPARTING
 * exits to "ABSENT" are NOT transitions -- they free the slot directly
 * (see proximity_tick_handler, the GONE lines): a freed slot IS the
 * absent phone.
 *
 * Why a separate function (vs. inline assignment in callers):
 *   - Single point of truth for "state transition happened"
 *   - Single point to add logging, metrics, side effects
 *   - Easier to audit (grep for transition_phone)
 */
static void transition_phone(int slot, uint8_t new_state)
{
    uint8_t old_state = phones[slot].state;
    if (old_state == new_state) {
        return;  /* no-op, don't log */
    }

    /* Decision evidence, captured BEFORE the mutation: the audit lines
     * below must show what the evaluator SAW, and dwell_ms resets as
     * part of the transition. (Phase 3 ch4.)
     */
    uint32_t dwell_at_fire = phones[slot].dwell_ms;

    phones[slot].state = new_state;
    phones[slot].state_enter_time = k_uptime_get();
    /* The dwell credit belongs to the OLD state's exit condition; the
     * new state starts its own from zero. Centralized here so no
     * caller can forget the reset. (Phase 3 ch4.)
     */
    phones[slot].dwell_ms = 0U;

    char uuid_str[MSD_LEN_UUID * 2U + 1U];
    format_uuid_hex(phones[slot].uuid, uuid_str, sizeof(uuid_str));

    LOG_INF("STATE: uuid=%s slot=%d %s -> %s",
            uuid_str, slot,
            phone_state_name[old_state],
            phone_state_name[new_state]);

    /*
     * Phase 3 ch4 side effects. UNLOCK fires on every ENTRY to PRESENT:
     * from APPROACHING (first arrival) or from DEPARTING (the user came
     * back) -- both mean "person at the door, release the strike".
     * RELOCK fires only on PRESENT -> DEPARTING: the only entry path to
     * DEPARTING in this machine, and the only context where "relock"
     * means anything (the lock was open).
     *
     * The audit line carries the decision evidence (x, slope, dwell
     * reached) -- the "why did the gate unlock at 14:23:05" record
     * promised in this file's ch1 header.
     */
    if (new_state == PHONE_STATE_PRESENT) {
        char xs[12], sl[12];
        fmt_float2(xs, sizeof(xs), phones[slot].kalman_x);
        fmt_slope(sl, sizeof(sl), &phones[slot]);
        LOG_INF("UNLOCK: uuid=%s slot=%d x=%s slope=%s dwell=%ums",
                uuid_str, slot, xs, sl, (unsigned)dwell_at_fire);
        if (event_cb != NULL) {
            event_cb(PROX_EVENT_UNLOCK, slot, phones[slot].uuid);
        }
    } else if (new_state == PHONE_STATE_DEPARTING &&
               old_state == PHONE_STATE_PRESENT) {
        char xs[12], sl[12];
        fmt_float2(xs, sizeof(xs), phones[slot].kalman_x);
        fmt_slope(sl, sizeof(sl), &phones[slot]);
        LOG_INF("RELOCK: uuid=%s slot=%d x=%s slope=%s dwell=%ums",
                uuid_str, slot, xs, sl, (unsigned)dwell_at_fire);
        if (event_cb != NULL) {
            event_cb(PROX_EVENT_RELOCK, slot, phones[slot].uuid);
        }
    }
}

/* --- Kalman filter (Phase 3 ch2) ---------------------------------------- */

/*
 * fmt_float2: format a float as "[-]d.dd" into a caller buffer.
 * (Phase 3 ch2)
 *
 * Why not LOG_DBG("... %f ..."): %f in Zephyr logging requires
 * CONFIG_CBPRINTF_FP_SUPPORT=y, which pulls in substantial code. Why not
 * roundf()/fabsf(): those are libm calls (dependency + call overhead).
 * This helper is pure integer formatting after ONE manual rounding step
 * -- production-free, zero dependencies.
 *
 * Method: round to hundredths with sign-adjusted truncation (adding +0.5
 * for v>=0 and -0.5 for v<0 makes C's toward-zero truncation round to
 * nearest instead of flooring negatives), then print integer parts.
 * Examples: -58.37 -> "-58.37", 0.678 -> "0.67", -0.5 -> "-0.50".
 *
 * Inputs are bounded (|x| <= 128 dBm, p <= ~64 dB^2, k in [0,1]), so
 * cents fits comfortably in int32_t. No overflow possible.
 */
static void fmt_float2(char *out, size_t len, float v)
{
    if (out == NULL || len < 9U) {
        return;  /* "-100.00" + NUL = 8; 9 gives margin. Caller error. */
    }

    int32_t cents = (int32_t)(v * 100.0f + (v >= 0.0f ? 0.5f : -0.5f));
    int32_t whole = cents / 100;
    int32_t frac  = cents % 100;

    if (cents < 0) {
        (void)snprintk(out, len, "-%d.%02d", (int)(-whole), (int)(-frac));
    } else {
        (void)snprintk(out, len, "%d.%02d", (int)whole, (int)frac);
    }
}

/*
 * fmt_slope: "1.25" or "INVALID" for the audit/tick lines.
 * (Phase 3 chL4.)
 *
 * Rule R1 made STRUCTURAL: trend_slope is never read anywhere in
 * this module without checking trend_valid first -- this helper is
 * the only sanctioned way to render the slope, so the rule cannot
 * be forgotten at a call site.
 */
static void fmt_slope(char *out, size_t len, const struct phone_track *ph)
{
    if (ph->trend_valid) {
        fmt_float2(out, len, ph->trend_slope);
    } else {
        (void)snprintk(out, len, "INVALID");
    }
}

/*
 * kalman_update: feed one MATCH's RSSI into the slot's scalar Kalman filter.
 * (Phase 3 ch2; implements design addendum 3-L1-A point (ii).)
 *
 * Called from proximity_on_match ONLY (system workqueue thread), and it
 * must run BEFORE last_match_time / match_count are updated -- it needs
 * the PREVIOUS values for dt and for first-sample detection.
 *
 * Model (deliberately the simplest useful one -- "random walk + noise"):
 *
 *   true state : x_k = x_{k-1} + w_k,  w ~ N(0, Q_PER_SEC * dt)   [predict]
 *   measurement: z_k = x_k + v_k,      v ~ N(0, R)                [update]
 *
 * This IS the textbook Kalman with F=1, H=1 (scalar, no dynamics) -- no
 * matrices needed. No motion model on purpose: at 1-4 m gate range,
 * multipath makes RSSI-vs-distance too ambiguous to "predict where the
 * RSSI is going"; we only model "it drifts, and drift accumulates with
 * TIME".
 *
 * Units: x in dBm, p in dB^2, dt in seconds.
 *
 * Why no NaN/overflow guards: z is a bounded int8_t, all parameters are
 * positive finite floats, and the recursion only uses +,-,*,/ -- with
 * finite inputs the state cannot become NaN or infinite. Guards-by-design
 * beats defensive runtime checks (nothing to check, nothing to branch on).
 */
static void kalman_update(int slot, int8_t z, int64_t now_ms)
{
    struct phone_track *ph = &phones[slot];

    /*
     * First sample after alloc: start the filter AT the measurement
     * (addendum 3-L1-A: x0 = z0, p0 = r). The estimate is exactly as good
     * as one raw sample -- because it IS one raw sample. No convergence
     * wait on the critical path.
     *
     * match_count == 0 is a reliable "not yet initialized" sentinel:
     * alloc_phone memsets the slot (kalman_reject_streak = 0 too), and
     * alloc + first feed happen inside the same proximity_on_match call,
     * so this branch fires exactly once per slot lifetime (match_count
     * is only 0 between memset and the first increment, which brackets
     * this call).
     */
    if (ph->match_count == 0U) {
        ph->kalman_x = (float)z;
        ph->kalman_p = PROX_KALMAN_R;
        return;
    }

    /*
     * dt from wall-clock timestamps -- NEVER from sample counts (the
     * whole point of addendum 3-L1-A). Range: 0 .. ~31 s (30 s timeout
     * + up to 1 s tick latency). Even a grossly larger dt stays graceful:
     * p grows, k -> 1, next sample snaps the estimate -- correct
     * semantics for "phone was gone a long time".
     *
     * dt == 0 (two MATCHes in the same millisecond -- possible during
     * scan-window bursts) is legal: predict adds zero process noise, and
     * the update still validly averages two same-instant measurements.
     * Nothing anywhere divides by dt, so there is nothing to blow up.
     *
     * Precision: |dt| < 2^24 ms is exact in float; our max is ~31000 ms.
     * (dt_s values like 2.857 are inexact by <1 ulp -- irrelevant at
     * dB^2/s scale.)
     */
    float dt_s = (float)(now_ms - ph->last_match_time) / 1000.0f;

    /* --- Predict: time advanced, no new information yet ---------------- */
    /* x_pred = x          (random walk: no direction to extrapolate)      */
    /* p_pred = p + q*dt   (uncertainty grows linearly with elapsed time)  */
    float p_pred = ph->kalman_p + PROX_KALMAN_Q_PER_SEC * dt_s;

    /* --- 3-sigma innovation gate (Phase 3 ch3) + circuit breaker -------------
     *
     * The innovation (z - x) is the filter's own prediction residual; its
     * variance is p_pred + r (estimate uncertainty + measurement noise).
     * A residual beyond 3 sigma is either (a) a multipath outlier -- one
     * channel's faded path, quantization stack-up, body micro-motion (the
     * -76-among--65s spike in the ch2 desk capture; the -87..-76 deep
     * fades of the ch3 gate-hunt run) -- or (b) a genuine fast level
     * change.
     *
     * For (a) the gate is exactly right: fades never persist more than
     * 2-3 samples (validated: run5's 6 rejections, x std 1.46 dB vs
     * z std 6.79 dB -- the estimate stayed clean through swings).
     *
     * For (b) the L3 lab falsified the original claim that the gate
     * "self-opens within ~3 samples": that is true for a persistent
     * LEVEL STEP, but on a sustained RAMP (walking) the innovation KEEPS
     * GROWING while the threshold grows only ~sqrt(q*t) -- the gate can
     * stay shut for 9-12+ s and the estimate goes blind (observed:
     * walk-in frozen at -70.9 while the phone reached -53).
     *
     * The circuit breaker (PROX_KALMAN_GATE_MAX_REJECTS) restores the
     * missing invariant: after N consecutive rejections the sample is
     * force-accepted through the NORMAL update below. No special-case
     * math: N rejections have already inflated p, so k is naturally
     * ~0.7-0.9 and the filter mostly trusts the measurement -- which is
     * the correct Bayesian attitude after N samples of acknowledged
     * blindness. The gate can therefore never freeze the filter for
     * longer than N-1 consecutive samples, at any Q, at any walk speed.
     *
     * Squared comparison: |innov| >= k*sigma <=> innov^2 >= k^2*sigma^2.
     * Comparing squares avoids sqrtf (libm dependency; see fmt_float2).
     * PROX_KALMAN_GATE_SIGMA == 0.0f disables (constant-folded).
     */
    bool force_accept = false;

    if (PROX_KALMAN_GATE_SIGMA > 0.0f) {
        float innov = (float)z - ph->kalman_x;
        float thresh2 = PROX_KALMAN_GATE_SIGMA * PROX_KALMAN_GATE_SIGMA
                        * (p_pred + PROX_KALMAN_R);
        if (innov * innov > thresh2) {
            if (PROX_KALMAN_GATE_MAX_REJECTS > 0U &&
                ph->kalman_reject_streak + 1U >=
                PROX_KALMAN_GATE_MAX_REJECTS) {
                /* Nth consecutive rejection: break the freeze. p is
                 * already inflated (p_pred); fall through to the normal
                 * update with the wide gain. Streak resets below.
                 */
                force_accept = true;
            } else {
                ph->kalman_reject_streak++;
                ph->kalman_p = p_pred;   /* time passed; no info gained */
                if (ph->kalman_gated < 255U) {
                    ph->kalman_gated++;  /* saturating diagnostic total */
                }
                LOG_DBG("kalman: slot=%d gate: z=%d rejected, too far from x "
                        "(streak=%u gated=%u)", slot, z,
                        (unsigned)ph->kalman_reject_streak,
                        (unsigned)ph->kalman_gated);
                return;                  /* x unchanged; caller stamps time */
            }
        }
    }

    /* --- Update: blend prediction with measurement ----------------------
     *
     * Optimal gain for combining two unbiased estimates (prediction with
     * variance p_pred, measurement with variance r): the minimum-variance
     * blend weight. k -> 1: trust the measurement (prediction is vague).
     * k -> 0: trust the prediction (measurement is noisy).
     *
     * All-float with f-suffixed constants -- a single bare `2.0`/`4.0`
     * anywhere here promotes that expression to software double (ch2
     * lesson part 2; objdump evidence in the lab section).
     */
    float k = p_pred / (p_pred + PROX_KALMAN_R);

    ph->kalman_x += k * ((float)z - ph->kalman_x);
    ph->kalman_p  = (1.0f - k) * p_pred;

    /* Every acceptance (normal or breaker) resets the streak: a single
     * good sample proves the estimate is live again. (Phase 3 L3-C.)
     */
    ph->kalman_reject_streak = 0U;

    if (force_accept) {
        LOG_DBG("kalman: slot=%d gate breaker: forced accept z=%d "
                "after %u consecutive rejections", slot, z,
                (unsigned)(PROX_KALMAN_GATE_MAX_REJECTS - 1U));
    }

    /*
     * Per-MATCH tuning view (DBG only): the five numbers you stare at
     * while tuning q/r. Enable with CONFIG_LOG_DEFAULT_LEVEL=4.
     * (dt printed as int ms: bounded by ~31000, fits comfortably.)
     */
    char xs[12], ps[12], ks[12];
    fmt_float2(xs, sizeof(xs), ph->kalman_x);
    fmt_float2(ps, sizeof(ps), ph->kalman_p);
    fmt_float2(ks, sizeof(ks), k);
    LOG_DBG("kalman: slot=%d z=%d dt=%dms k=%s x=%s p=%s",
            slot, z, (int)(now_ms - ph->last_match_time), ks, xs, ps);
}

/* --- Trend: decimated (x,t) ring + LSQ slope (Phase 3 ch3) --------------- */

/*
 * trend_push: decimated insertion of the current Kalman output.
 * (Phase 3 L3; design addendum 3-L1-A point (iii): one entry per
 * >= TREND_DECIMATE_MS.)
 *
 * Called from proximity_on_match AFTER kalman_update (so we store the
 * post-update estimate) and AFTER the field updates.
 *
 * Runs even when the innovation gate just rejected the sample: a gated
 * sample still legitimately timestamps "our best estimate was x at time
 * t" -- that IS the estimate trajectory, which is what the regression
 * fits.
 *
 * Why decimation is safe: skipped samples are NOT lost -- the Kalman
 * already fused them into kalman_x. The ring stores the trajectory of
 * the ESTIMATE, not raw observations, so decimation loses no trend
 * information while bounding ring churn and making the structure
 * rate-independent (10 entries always span >= 5 s, at any MATCH rate).
 */
static void trend_push(int slot, int64_t now_ms)
{
    struct phone_track *ph = &phones[slot];
    uint32_t now32 = (uint32_t)now_ms;

    if (ph->trend_count > 0U) {
        /* newest entry lives at (head - 1 + N) % N */
        uint8_t last = (uint8_t)((ph->trend_head + TREND_RING_N - 1U)
                                 % TREND_RING_N);
        int32_t since = (int32_t)(now32 - ph->trend_t[last]);
        if (since < TREND_DECIMATE_MS) {
            return;  /* too soon; the Kalman already used this sample */
        }
    }

    ph->trend_x[ph->trend_head] = ph->kalman_x;
    ph->trend_t[ph->trend_head] = now32;
    ph->trend_head = (uint8_t)((ph->trend_head + 1U) % TREND_RING_N);
    if (ph->trend_count < TREND_RING_N) {
        ph->trend_count++;
    }

    char xs[12];
    fmt_float2(xs, sizeof(xs), ph->kalman_x);
    LOG_DBG("trend: slot=%d push x=%s (n=%u)",
            slot, xs, (unsigned)ph->trend_count);
}

/*
 * trend_compute: refresh the LSQ slope over the 5 s time window.
 * (Phase 3 L3; addendum 3-L1-A point (iii).)
 *
 * Runs in the 1 s tick, not per MATCH: the slope is a 5 s-window average
 * that changes slowly, 1 s cadence is plenty, and L4's state machine
 * will read the stored fields right here in the same tick.
 *
 * Method:
 *   1. Walk the ring newest -> oldest (insertion order IS time order).
 *   2. Keep entries with age <= TREND_WINDOW_MS. The window is anchored
 *      at NOW, not at the newest entry -- so when the phone goes silent,
 *      entries age out and the trend self-invalidates within ~5 s. That
 *      is the desired honesty (no direction claim without fresh data),
 *      not a bug. The walk breaks at the first too-old entry: everything
 *      behind it is older still.
 *   3. Guards: >= TREND_MIN_POINTS points AND >= TREND_MIN_SPAN_MS span,
 *      else trend_valid=0 / slope=0.0f.
 *   4. Mean-centered least squares. Time axis = age in seconds, NEGATIVE
 *      and bounded [-5, 0]: no large numbers (absolute uptime in seconds
 *      would waste float precision at ~0.008 s/ulp after one day), and
 *      mean-centering before products avoids cancellation.
 *
 * Numerical safety: with the guards, the denominator sum (t - tbar)^2 is
 * >= ~1 s^2 (4+ points, 2+ s span, >= 500 ms apart), so the division is
 * well-conditioned; the explicit check below covers the impossible case.
 *
 * Stack: ts[10] + xs[10] = 80 bytes, reused per phone. sysworkq had
 * ~1300 bytes free at last check.
 */
static void trend_compute(int slot, int64_t now_ms)
{
    struct phone_track *ph = &phones[slot];
    uint32_t now32 = (uint32_t)now_ms;

    float ts[TREND_RING_N];   /* time axis: age in s (negative = past) */
    float xs[TREND_RING_N];
    uint8_t n = 0;
    int32_t age_first = 0;    /* age of newest kept entry, ms */
    int32_t age_last = 0;     /* age of oldest kept entry, ms */

    for (uint8_t i = 0; i < ph->trend_count; i++) {
        uint8_t idx = (uint8_t)((ph->trend_head + TREND_RING_N - 1U - i)
                                % TREND_RING_N);
        int32_t age = (int32_t)(now32 - ph->trend_t[idx]);
        if (age > TREND_WINDOW_MS) {
            break;   /* ring is time-ordered: all older entries are out */
        }
        if (n == 0U) {
            age_first = age;
        }
        age_last = age;
        ts[n] = -(float)age / 1000.0f;
        xs[n] = ph->trend_x[idx];
        n++;
    }

    int32_t span_ms = age_last - age_first;

    if ((n < TREND_MIN_POINTS) || (span_ms < TREND_MIN_SPAN_MS)) {
        ph->trend_valid = 0;
        ph->trend_slope = 0.0f;
        LOG_DBG("trend: slot=%d INVALID (n=%u span=%dms)",
                slot, (unsigned)n, (int)span_ms);
        return;
    }

    /* Mean-centered LSQ: slope = sum((t-tbar)*(x-xbar)) / sum((t-tbar)^2) */
    float mt = 0.0f, mx = 0.0f;
    for (uint8_t i = 0; i < n; i++) {
        mt += ts[i];
        mx += xs[i];
    }
    mt /= (float)n;
    mx /= (float)n;

    float s_tt = 0.0f, s_tx = 0.0f;
    for (uint8_t i = 0; i < n; i++) {
        float dtt = ts[i] - mt;
        float dxx = xs[i] - mx;
        s_tt += dtt * dtt;
        s_tx += dtt * dxx;
    }

    if (s_tt < 0.0001f) {  /* impossible with the guards; cheap insurance */
        ph->trend_valid = 0;
        ph->trend_slope = 0.0f;
        return;
    }

    ph->trend_slope = s_tx / s_tt;   /* dB/s; + = approaching, - = receding */
    ph->trend_valid = 1;

    char sb[12];
    fmt_float2(sb, sizeof(sb), ph->trend_slope);
    LOG_DBG("trend: slot=%d slope=%s dB/s (n=%u span=%dms)",
            slot, sb, (unsigned)n, (int)span_ms);
}

/* --- Evaluator: thresholds + dwell + trend modulation (Phase 3 ch4) ---- */

/*
 * evaluate_phone: run the current state's exit condition for one phone.
 * (Phase 3 L4; the decision table and the dwell invariant are documented
 * with the PROX_* ch4 tunables in proximity.h -- read that first.)
 *
 * Runs in the 1 s tick, IMMEDIATELY AFTER trend_compute for the same
 * phone: rule R5 -- level and trend are consumed in the same tick that
 * computed them, one consistent snapshot, serialized on the system
 * workqueue.
 *
 * The structure is two mirrored branches:
 *
 *   unlock direction (APPROACHING or DEPARTING): condition x > UNLOCK
 *     -> PRESENT + UNLOCK. In DEPARTING this is the RETURN edge (the
 *     user walked back); same condition, same 3 s target as
 *     APPROACHING. Trend's role here: pause only, never confirm.
 *   relock direction (PRESENT): condition x < RELOCK -> DEPARTING +
 *     RELOCK.
 *
 * Each branch is three moves:
 *   1. condition broken -> dwell = 0 and out. THE ONLY RESET (the
 *      invariant). Everything else modulates.
 *   2. trend contradicts -> credit frozen this tick (pass-through /
 *      returner protection). On the RELOCK path only, a confirming
 *      trend also shrinks the target (5 s -> 3 s, fail-secure). The
 *      unlock path has NO confirm -- the host simulation falsified it
 *      (pass-through unlock on the trend's direction lag; see
 *      proximity.h). Neutral (INVALID or |slope| < floor) -> nominal.
 *   3. credit >= target -> transition (transition_phone resets the
 *      credit, fires the event, writes the audit lines).
 *
 * Why the pauses are self-limiting (no stuck states): a PAUSE on the
 * unlock path requires slope <= -floor sustained -- but a sustained
 * negative slope pulls kalman_x down through the threshold within
 * ~2-4 s, which RESETS the credit via move 1. Same argument, mirrored,
 * for the relock pause. A dwell can only be paused, never deadlocked.
 */
static void evaluate_phone(int slot)
{
    struct phone_track *ph = &phones[slot];

    /* Classify the trend (rules R1, R4, R6): INVALID or below-floor
     * slopes classify as NEUTRAL -- both booleans false.
     */
    bool trend_up   = ph->trend_valid &&
                      (ph->trend_slope >=  PROX_TREND_FLOOR_DB_S);
    bool trend_down = ph->trend_valid &&
                      (ph->trend_slope <= -PROX_TREND_FLOOR_DB_S);

    if (ph->state == PHONE_STATE_PRESENT) {
        /* --- relock direction: PRESENT -> DEPARTING --- */
        if (ph->kalman_x >= (float)PROX_RELOCK_RSSI) {
            ph->dwell_ms = 0U;   /* condition broken: the only reset */
            return;
        }
        if (!trend_up) {
            /* trend_up = below the relock line but RISING: a returner
             * walking back through the band -- pause so they can pass
             * back above UNLOCK without the gate locking behind them.
             */
            ph->dwell_ms += PROX_TICK_MS;
        }
        uint32_t target = trend_down ? PROX_RELOCK_DWELL_CONFIRM_MS
                                     : PROX_RELOCK_DWELL_MS;
        if (ph->dwell_ms >= target) {
            transition_phone(slot, PHONE_STATE_DEPARTING);
        }
    } else {
        /* --- unlock direction: APPROACHING/DEPARTING -> PRESENT --- */
        if (ph->kalman_x <= (float)PROX_UNLOCK_RSSI) {
            ph->dwell_ms = 0U;   /* condition broken: the only reset */
            return;
        }
        if (!trend_down) {
            /* trend_down = above the unlock line but RECEDING: a
             * pass-through -- pause the credit; the level condition
             * will break on its own within seconds and do the reset.
             */
            ph->dwell_ms += PROX_TICK_MS;
        }
        /* Nominal target only -- the unlock path deliberately has NO
         * confirmed shortcut (the ch4 host simulation falsified it: a
         * brisk pass-through unlocked on the trend's ~W/2 direction
         * lag; see the confirmed-dwell comment in proximity.h). The
         * trend's entire job on THIS path is the pause above.
         */
        if (ph->dwell_ms >= PROX_UNLOCK_DWELL_MS) {
            transition_phone(slot, PHONE_STATE_PRESENT);
        }
    }
}

/* --- Periodic tick ---------------------------------------------------- */

static void proximity_tick_handler(struct k_work *work);

K_WORK_DELAYABLE_DEFINE(proximity_tick_work, proximity_tick_handler);

/*
 * proximity_tick_handler: runs every PROX_TICK_MS (1s) on the system
 * workqueue.
 *
 * For each tracked phone:
 *   - Compute time since last MATCH (now - last_match_time)
 *   - If > PROX_TIMEOUT_MS (30s): free the slot (state -> ABSENT, in_use=false)
 *   - ch4 delivered: DEPARTING exits, trend refresh, freshness gate,
 *     and evaluate_phone (dwell timers + transitions) below
 *   - Log state + time-in-state at DBG level (for dev observability)
 *
 * Threading: runs in system workqueue context.
 * Work budget: 8 phones * O(1) per phone = ~50us total. Trivial.
 *
 * Why the tick (vs. purely event-driven on MATCH):
 *   - The 30s timeout MUST fire even if no MATCHes arrive (that's the whole
 *     point -- phone went away, no MATCHes, we need to detect that).
 *   - L4's dwell timers also need time-driven checks (e.g., "RSSI > -58 for
 *     3s" requires checking the time-since-condition-met, even if no new
 *     MATCH arrives during that 3s).
 *   - Pure event-driven would miss timeouts when the phone disappears.
 */
static void proximity_tick_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    int64_t now = k_uptime_get();

    for (int i = 0; i < MAX_TRACKED_PHONES; i++) {
        if (!phones[i].in_use) {
            continue;
        }

        int64_t since_match = now - phones[i].last_match_time;

        /* 30s timeout: phone is gone, free the slot */
        if (since_match > PROX_TIMEOUT_MS) {
            char uuid_str[MSD_LEN_UUID * 2U + 1U];
            format_uuid_hex(phones[i].uuid, uuid_str, sizeof(uuid_str));
            LOG_INF("TIMEOUT: uuid=%s slot=%d (no MATCH for %lldms, freeing)",
                    uuid_str, i, (long long)since_match);
            free_phone(i);
            continue;
        }

        /* Phase 3 ch4: DEPARTING exits (handoff 7.1: "no MATCH for >5 s
         * in DEPARTING, OR RSSI below -90 dBm"). Implemented as FREEING
         * the slot: ABSENT is slot-freedom in this design (a freed slot
         * IS the absent phone -- there is no live ABSENT state to
         * transition into). Two distinct GONE lines so the log tells you
         * WHICH exit fired -- same audit principle as TIMEOUT.
         *
         * Checked BEFORE trend_compute/evaluator: both exits decide on
         * raw fields (since_match, kalman_x), no trend needed, and a
         * gone phone must not spend a tick evaluating.
         */
        if (phones[i].state == PHONE_STATE_DEPARTING) {
            if (since_match > PROX_DEPARTING_SILENCE_MS) {
                char uuid_str[MSD_LEN_UUID * 2U + 1U];
                format_uuid_hex(phones[i].uuid, uuid_str, sizeof(uuid_str));
                LOG_INF("GONE: uuid=%s slot=%d (departing, no MATCH for %lldms)",
                        uuid_str, i, (long long)since_match);
                free_phone(i);
                continue;
            }
            if (phones[i].kalman_x < (float)PROX_TRACK_FLOOR_RSSI) {
                char uuid_str[MSD_LEN_UUID * 2U + 1U];
                char xs[12];
                format_uuid_hex(phones[i].uuid, uuid_str, sizeof(uuid_str));
                fmt_float2(xs, sizeof(xs), phones[i].kalman_x);
                LOG_INF("GONE: uuid=%s slot=%d (departing, x=%s below %d floor)",
                        uuid_str, i, xs, PROX_TRACK_FLOOR_RSSI);
                free_phone(i);
                continue;
            }
        }

        /* Phase 3 ch3: refresh the trend slope (LSQ over the 5 s window).
         * 1 s cadence is plenty -- the slope is a 5 s-window average that
         * changes slowly, and L4 will read the stored fields right here.
         */
        trend_compute(i, now);

        /* Phase 3 ch4: evidence-freshness gate. If the newest MATCH is
         * older than the evidence horizon (PROX_EVIDENCE_FRESH_MS, an
         * alias of TREND_WINDOW_MS -- deliberately the same 5 s), the
         * level estimate is a 5 s old photograph and the trend has
         * already self-invalidated: FREEZE the dwell credit (do NOT
         * reset -- R3: silence is absent evidence, not contrary
         * evidence; a pocket-dozed phone that resumes above the
         * threshold picks its credit back up) and skip the evaluator.
         */
        if (since_match <= PROX_EVIDENCE_FRESH_MS) {
            evaluate_phone(i);
        }

        /* DBG: per-tick per-phone snapshot for dev observability.
         * ALWAYS visible: this module is pinned to LOG_LEVEL_DBG
         * (Phase 3 L3-E, "quiet world, loud pins") -- the kalman:/trend:/
         * tick: lines are the permanent telemetry view. No Kconfig
         * toggling needed; volume is bounded (~1 line/s per phone).
         */
        int64_t time_in_state = now - phones[i].state_enter_time;

        /* Phase 3 ch2: Kalman estimate + variance in the per-tick snapshot.
         * fmt_float2 avoids CONFIG_CBPRINTF_FP_SUPPORT.
         */
        char xs[12], ps[12], sl[12];
        fmt_float2(xs, sizeof(xs), phones[i].kalman_x);
        fmt_float2(ps, sizeof(ps), phones[i].kalman_p);
        fmt_slope(sl, sizeof(sl), &phones[i]);

        LOG_DBG("tick: slot=%d state=%s rssi=%d x=%s p=%s slope=%s "
                "matches=%u gated=%u dwell=%ums age=%lldms in_state=%lldms",
                i, phone_state_name[phones[i].state],
                phones[i].last_rssi, xs, ps, sl, phones[i].match_count,
                (unsigned)phones[i].kalman_gated,
                (unsigned)phones[i].dwell_ms,
                (long long)since_match, (long long)time_in_state);
    }

    /* Reschedule ourselves. Canonical periodic pattern (same as scan.c
     * stats_work_handler).
     */
    (void)k_work_reschedule(&proximity_tick_work, K_MSEC(PROX_TICK_MS));
}

/* --- Public API ------------------------------------------------------- */

int proximity_init(void)
{
    /* Zero the entire phones[] array. All slots in_use=false, state=ABSENT. */
    memset(phones, 0, sizeof(phones));

    /* Schedule first tick. k_work_reschedule is idempotent: if already
     * scheduled, it reschedules to the new time. Safe to call multiple times.
     *
     * Return contract (Zephyr Quirk #11, learned from the student's boot
     * log): the delayable-work scheduling functions return 0 or 1 on
     * SUCCESS (0 = newly scheduled; positive = scheduling-state info,
     * e.g. already scheduled / deadline updated) and NEGATIVE errno on
     * failure. The old `rc != 0` check raised a false ERR on first boot
     * with rc=1 while the schedule actually SUCCEEDED (the registry and
     * transitions demonstrably worked). Only rc < 0 is an error.
     *
     */
    int rc = k_work_reschedule(&proximity_tick_work, K_MSEC(PROX_TICK_MS));
    if (rc < 0) {
        LOG_ERR("proximity_init: k_work_reschedule failed: %d", rc);
        return rc;
    }

    LOG_INF("proximity: init OK (%d slots, tick=%ums, timeout=%us)",
            MAX_TRACKED_PHONES, PROX_TICK_MS, PROX_TIMEOUT_MS / 1000);
    return 0;
}

int proximity_on_match(const uint8_t *uuid, int8_t rssi)
{
    if (uuid == NULL) {
        return -EINVAL;
    }

    /* 1. Find existing slot for this UUID, or allocate a new one */
    int slot = find_phone(uuid);
    bool is_new = (slot < 0);

    if (is_new) {
        /*
         * Phase 3 ch4: first-sight floor (handoff 7.1: "ABSENT ->
         * APPROACHING: first MATCH (RSSI > -90)"). Allocation-time
         * ONLY: a phone already tracked that fades deep is handled by
         * the DEPARTING exits / 30 s timeout -- evicting mid-track
         * would throw away live Kalman state on every deep fade.
         *
         * This is REGISTRY HYGIENE, not proximity logic: it keeps
         * far-away phones (the one in the bedroom at -85) from
         * squatting on slots for 30 s each. Honest limits: any of OUR
         * phones already above the floor keeps its slot until timeout
         * no matter how far it then fades; the real filter for
         * foreign traffic is the CID+version check in scan.c (and,
         * in Phase 4, authentication).
         *
         * DBG, not WRN: routine, not exceptional -- every distant
         * compatible phone logs here on each catch.
         */
        if (rssi <= PROX_TRACK_FLOOR_RSSI) {
            char uuid_str[MSD_LEN_UUID * 2U + 1U];
            format_uuid_hex(uuid, uuid_str, sizeof(uuid_str));
            LOG_DBG("far phone: uuid=%s rssi=%d <= %d, not tracked",
                    uuid_str, rssi, PROX_TRACK_FLOOR_RSSI);
            return -ERANGE;
        }
        slot = alloc_phone(uuid);
        if (slot < 0) {
            /* Registry full. Log at WRN. Phone will be tracked on a future
             * MATCH after a slot frees up (30s timeout of an inactive phone).
             */
            char uuid_str[MSD_LEN_UUID * 2U + 1U];
            format_uuid_hex(uuid, uuid_str, sizeof(uuid_str));
            LOG_WRN("registry full (%d slots), ignoring uuid=%s rssi=%d",
                    MAX_TRACKED_PHONES, uuid_str, rssi);
            return -ENOSPC;
        }
    }

    /* 2. Kalman update (Phase 3 ch2)
     *
     * ORDER MATTERS: kalman_update must run BEFORE last_match_time and
     * match_count are overwritten below -- it computes dt from the
     * PREVIOUS last_match_time, and uses match_count==0 to detect the
     * first sample after alloc. (Swapping this order is ch2 exercise Q4.)
     *
     * One k_uptime_get() for the whole MATCH: dt, last_match_time, and
     * state_enter_time (via transition_phone below) all see the same
     * "now", so logs and math stay mutually consistent.
     */
    int64_t now = k_uptime_get();
    kalman_update(slot, rssi, now);

    /* 3. Update per-MATCH fields */
    phones[slot].last_rssi        = rssi;
    phones[slot].last_match_time = now;
    phones[slot].match_count++;

    /* 4. Trend decimation (Phase 3 ch3): store the post-update estimate
     * into the (x, t) ring. Runs AFTER kalman_update so we push the
     * freshly-blended x; runs regardless of whether this sample was
     * gated (a gated sample still timestamps the estimate trajectory).
     */
    trend_push(slot, now);

    /* 5. State machine: ABSENT -> APPROACHING on first MATCH
     *
     * For a newly-allocated slot, state is ABSENT (set in alloc_phone).
     * For an existing slot that timed out and was freed, alloc_phone also
     * sets ABSENT. So this transition fires correctly in both cases.
     *
     * If the phone was already in APPROACHING/PRESENT/DEPARTING (we've seen
     * it recently), this is a no-op (transition_phone checks for same-state).
     *
     * Phase 3 L4 delivered: the -90 dBm first-sight floor lives HERE
     * (the far-phone check above); the threshold/dwell/trend
     * transitions live in the TICK's evaluate_phone, not here -- rule
     * R5 (worklog 3-L3-A): level and trend are consumed in the same
     * tick that computed them, one consistent snapshot.
     */
    if (phones[slot].state == PHONE_STATE_ABSENT) {
        char uuid_str[MSD_LEN_UUID * 2U + 1U];
        format_uuid_hex(phones[slot].uuid, uuid_str, sizeof(uuid_str));
        LOG_INF("NEW PHONE: uuid=%s slot=%d rssi=%d",
                uuid_str, slot, rssi);
        transition_phone(slot, PHONE_STATE_APPROACHING);
    }

    return slot;
}

int proximity_get_slot(uint8_t index, struct phone_track *out)
{
    if (out == NULL || index >= MAX_TRACKED_PHONES || !phones[index].in_use) {
        return -EINVAL;
    }
    *out = phones[index];  /* struct copy */
    return 0;
}

uint8_t proximity_get_count(void)
{
    uint8_t count = 0;
    for (int i = 0; i < MAX_TRACKED_PHONES; i++) {
        if (phones[i].in_use) {
            count++;
        }
    }
    return count;
}

int proximity_register_event_cb(proximity_event_cb_t cb)
{
    /*
     * Phase 3 L4. Plain pointer store -- safe by the ordering contract
     * documented in proximity.h: register from main() BEFORE
     * bt_enable(); the evaluator that fires events only ever runs on
     * the system workqueue, which is only fed after BLE comes up.
     * No locking needed, cannot fail.
     */
    event_cb = cb;
    return 0;
}