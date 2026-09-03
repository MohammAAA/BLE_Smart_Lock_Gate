/*
 * BLE Smart Lock - Proximity Tracking
 * Phase 3 Lesson 1: phone-tracking registry + state machine skeleton
 *
 * See proximity.h for top-level documentation.
 *
 * L1 scope:
 *   - phones[] static array of MAX_TRACKED_PHONES slots
 *   - proximity_init: zero array + schedule first tick
 *   - proximity_on_match: lookup-or-alloc + update fields + ABSENT->APPROACHING
 *   - proximity_tick: 30s timeout cleanup + DBG state log
 *   - All transitions logged at INF for audit
 *
 * Phase 3 ch2 changes:
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

LOG_MODULE_REGISTER(proximity);

/* --- The registry: static array of phone_track records ---------------- */

static struct phone_track phones[MAX_TRACKED_PHONES];

/* Phase 3 ch3: verify the documented layout (offset table in proximity.h).
 * If this fires after you edit the struct, update the table -- the docs
 * and the code must not drift apart. BUILD_ASSERT is compile-time only,
 * zero runtime cost (<zephyr/toolchain.h>, pulled in by kernel.h).
 */
BUILD_ASSERT(sizeof(struct phone_track) == 136);

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
 * L4 will add calls for:
 *   - APPROACHING -> PRESENT (on RSSI + dwell) -- with unlock side effect
 *   - PRESENT -> DEPARTING (on RSSI + dwell) -- with relock side effect
 *   - DEPARTING -> ABSENT (on extended timeout)
 *
 * The state machine is intentionally simple: no guards, no actions beyond
 * logging. L4 will add unlock/relock side effects on PRESENT/DEPARTING entry.
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

    phones[slot].state = new_state;
    phones[slot].state_enter_time = k_uptime_get();

    char uuid_str[MSD_LEN_UUID * 2U + 1U];
    format_uuid_hex(phones[slot].uuid, uuid_str, sizeof(uuid_str));

    LOG_INF("STATE: uuid=%s slot=%d %s -> %s",
            uuid_str, slot,
            phone_state_name[old_state],
            phone_state_name[new_state]);
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
     * alloc_phone memsets the slot, and alloc + first feed happen inside
     * the same proximity_on_match call, so this branch fires exactly once
     * per slot lifetime (match_count is only 0 between memset and the
     * first increment, which brackets this call).
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

    /* --- 3-sigma innovation gate (Phase 3 ch3) -----------------------------
     *
     * The innovation (z - x) is the filter's own prediction residual; its
     * variance is p_pred + r (estimate uncertainty + measurement noise).
     * A residual beyond 3 sigma is either (a) a multipath outlier -- one
     * channel's faded path, quantization stack-up, body micro-motion (the
     * -76-among--65s spike in the L2 desk capture) -- or (b) a genuine
     * fast level change.
     *
     * The gate cannot tell (a) from (b), and does not need to:
     *   - (a) never persists: the next sample lands back inside the gate,
     *     and the outlier never enters x or the trend ring. One-sample
     *     spikes are exactly what LSQ slopes are most vulnerable to.
     *   - (b) persists: each rejection leaves p = p_pred (time still
     *     passed, no information gained), so the threshold WIDENS every
     *     sample -- the gate self-opens within ~3 samples at 1 Hz.
     *
     * Squared comparison: |innov| >= k*sigma <=> innov^2 >= k^2*sigma^2.
     * Comparing squares avoids sqrtf (libm dependency; see fmt_float2).
     * PROX_KALMAN_GATE_SIGMA == 0.0f disables (constant-folded).
     */
    if (PROX_KALMAN_GATE_SIGMA > 0.0f) {
        float innov = (float)z - ph->kalman_x;
        float thresh2 = PROX_KALMAN_GATE_SIGMA * PROX_KALMAN_GATE_SIGMA
                        * (p_pred + PROX_KALMAN_R);
        if (innov * innov > thresh2) {
            ph->kalman_p = p_pred;   /* time passed; no info gained */
            ph->kalman_gated++;
            LOG_DBG("kalman: slot=%d gate: z=%d rejected, too far from x "
                    "(gated=%u)", slot, z, (unsigned)ph->kalman_gated);
            return;                  /* x unchanged; caller still stamps time */
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

    /*
     * Per-MATCH tuning view (DBG only): the five numbers you stare at
     * while tuning q/r. Enable with CONFIG_LOG_DEFAULT_LEVEL=4.
     * (dt printed as int ms: bounded by ~31000, fits comfortably.)
     */
    char xs[12], ps[12], ks[12];
    fmt_float2(xs, sizeof(xs), ph->kalman_x);
    fmt_float2(ps, sizeof(ps), ph->kalman_p);
    fmt_float2(ks, sizeof(ks), k);
    LOG_INF("kalman: slot=%d z=%d dt=%dms k=%s x=%s p=%s",
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
 *   - (L4 will add: check dwell timers for APPROACHING->PRESENT etc.)
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

        /* L4 will add: dwell timer checks for APPROACHING->PRESENT etc. */

        /* Phase 3 ch3: refresh the trend slope (LSQ over the 5 s window).
         * 1 s cadence is plenty -- the slope is a 5 s-window average that
         * changes slowly, and L4 will read the stored fields right here.
         */
        trend_compute(i, now);

        /* DBG: per-tick per-phone snapshot for dev observability.
         * Visible only when CONFIG_LOG_DEFAULT_LEVEL=4 (DBG).
         * Off in production (CONFIG_LOG_DEFAULT_LEVEL=3 = INF).
         *
         * To see ticks during algorithm tuning, temporarily set:
         *   CONFIG_LOG_DEFAULT_LEVEL=4
         * in prj.conf, rebuild, flash. You'll see ~8 lines/second when 8
         * phones are tracked. Useful for verifying the state machine is
         * stable. Turn back to 3 for normal runs.
         */
        int64_t time_in_state = now - phones[i].state_enter_time;

        /* Phase 3 ch2: Kalman estimate + variance in the per-tick snapshot.
         * fmt_float2 avoids CONFIG_CBPRINTF_FP_SUPPORT.
         */
        char xs[12], ps[12], sl[12];
        fmt_float2(xs, sizeof(xs), phones[i].kalman_x);
        fmt_float2(ps, sizeof(ps), phones[i].kalman_p);
        if (phones[i].trend_valid) {
            fmt_float2(sl, sizeof(sl), phones[i].trend_slope);
        } else {
            (void)snprintk(sl, sizeof(sl), "INVALID");
        }

        LOG_DBG("tick: slot=%d state=%s rssi=%d x=%s p=%s slope=%s "
                "matches=%u gated=%u age=%lldms in_state=%lldms",
                i, phone_state_name[phones[i].state],
                phones[i].last_rssi, xs, ps, sl, phones[i].match_count,
                (unsigned)phones[i].kalman_gated,
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
     * L4 will add:
     *   - if state == APPROACHING and kalman_x > UNLOCK_RSSI and dwell >= 3s:
     *       transition to PRESENT, fire unlock
     *   - if state == PRESENT and kalman_x < RELOCK_RSSI and dwell >= 5s:
     *       transition to DEPARTING, fire relock
     *   - if state == DEPARTING and no MATCH for >DEPARTING_TIMEOUT:
     *       transition to ABSENT
     *
     * Note: the -90 dBm floor for ABSENT->APPROACHING (from handoff section 7.1)
     * is deferred to L4. In L1, any MATCH transitions to APPROACHING. The
     * scan.c filter (CID + version) is enough filtering for L1. L4 adds the
     * RSSI floor as part of the threshold logic.
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