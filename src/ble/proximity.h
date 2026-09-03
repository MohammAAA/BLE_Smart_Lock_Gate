#ifndef BLE_PROXIMITY_H_
#define BLE_PROXIMITY_H_

/*
 * BLE Smart Lock - Proximity Tracking
 * Phase 3 Lesson 1: phone-tracking registry + state machine skeleton
 *
 * ch1 scope:
 *   - phone_track struct (per-tracked-phone state)
 *   - Registry: array of MAX_TRACKED_PHONES slots, alloc/free/lookup-by-UUID
 *   - State machine enum: ABSENT -> APPROACHING -> PRESENT -> DEPARTING -> ABSENT
 *   - Real transitions in ch1: ABSENT->APPROACHING (on first MATCH), Any->ABSENT
 *     (30s timeout)
 *   - Stub transitions (ch4 will fill): APPROACHING->PRESENT (dwell+RSSI),
 *     PRESENT->DEPARTING, DEPARTING->ABSENT
 *   - 1-second proximity_tick via k_work_delayable (state evolution + cleanup)
 *   - All state transitions logged at INF for audit
 *
 * Lesson status:
 *   ch1: registry + state machine skeleton (delivered)
 *   ch2: time-normalized scalar Kalman filter (delivered -- PROX_KALMAN_*
 *       tunables below; kalman_x/kalman_p populated on every MATCH).
 *       Design addendum 3-L1-A rule: ALL filters are TIME-based, never
 *       sample-index-based.
 *   ch3: trend (delivered -- TREND_* tunables below): decimated (x,t) ring
 *       of Kalman outputs + 5 s time-window LSQ slope with evidence
 *       guards + 3-sigma innovation gate on the Kalman. Fixes Issue #3
 *       (trend input is smoothed values, never raw RSSI). The 12 s MA
 *       from the original plan: DROPPED (the LSQ is itself a smoother;
 *       a series MA would add ~5.5 s slope-onset lag -- analysis and
 *       falsification criterion in the L3 lesson).
 *   - ch4 (pending): RSSI threshold + dwell timers driving
 *         APPROACHING->PRESENT, PRESENT->DEPARTING, DEPARTING->ABSENT
 *   - ch5 (pending): asymmetric scan (modifies scan_start to accept
 *         per-state params; scan duty is a latency/energy/spectrum knob
 *         only -- never a correctness knob, addendum 3-L1-A (vi))
 *
 * Threading model (carried from Phase 2 L4):
 *   - proximity_on_match() runs in scan_work_handler context (system workqueue)
 *   - proximity_tick() runs in its own k_work_delayable, also on system workqueue
 *   - Both serialized by the single-threaded system workqueue -> no locking
 *     needed for the phones[] array
 *   - Phase 4+ will need a dedicated workqueue for heavy work (HMAC, NVS)
 *
 * Zephyr 4.4 API verification greps (student should run on their tree):
 *   grep -rn "int64_t k_uptime_get" \
 *       /media/rimo/Technical/zephyrproject/zephyr/include/zephyr/kernel.h
 *   grep -rn "K_WORK_DELAYABLE_DEFINE" \
 *       /media/rimo/Technical/zephyrproject/zephyr/include/zephyr/kernel.h
 *   grep -rn "k_work_reschedule" \
 *       /media/rimo/Technical/zephyrproject/zephyr/include/zephyr/kernel.h
 */

#include <zephyr/kernel.h>
#include "scan.h"                    /* MSD_LEN_UUID */

/* --- Configuration ----------------------------------------------------- */

/*
 * MAX_TRACKED_PHONES: fixes Issue #8 from original guide (referenced but
 * never #defined).
 *
 * Why 8: residential deployment is 15-25 residents. At any given moment, at
 * most ~half are near a gate (the other half are at work / asleep / elsewhere).
 * 8 slots covers the worst case (party, all residents near one gate) with
 * headroom.
 *
 * RAM cost: 8 * sizeof(struct phone_track) = 8 * ~112 bytes (incl. padding)
 *         = ~896 bytes. Trivial vs nRF52840's 256 KB RAM.
 *
 * What happens if a 9th phone appears while all 8 slots are full?
 *   - The 9th phone's MATCH is logged at WRN ("registry full, ignoring uuid=...")
 *   - The phone is NOT tracked until a slot frees up (30s timeout of an
 *     inactive phone)
 *   - For residential use this is vanishingly rare (8 simultaneous near-gate
 *     phones)
 *   - For commercial deployment (office building), bump to 32 or 64
 *
 * Alternatives considered:
 *   - Hash table: overkill for 8 entries. Linear scan is 8 memcmps of 16
 *     bytes = 128 bytes compared per MATCH, ~50 cycles on Cortex-M4. Trivial.
 *   - Linked list: more complex (pointer maintenance, fragmentation), no
 *     benefit at this size.
 */
#define MAX_TRACKED_PHONES 8

/*
 * Phone disappearance timeout (milliseconds).
 *
 * If no MATCH is received from a tracked phone for this long, the phone is
 * considered gone: state -> ABSENT, slot freed, Kalman state discarded.
 *
 * Why 30 seconds (math corrected in the L1 addendum review, worklog 3-L1-A;
 * the original comment used the wrong exponent and mixed up models):
 *   - Model: the phone advertises ~1 event/s; each adv event is caught
 *     with probability ~0.35 (Phase 2 ch4 empirical: 3-4 matches per 10s
 *     with window=160/interval=400 = 100/250 ms = 40% duty, minus losses).
 *     Over 30s that is ~30 independent catch opportunities, ~10-12 expected
 *     matches.
 *   - P(0 matches in 30s) = (1 - 0.35)^30 = 2.4e-6, about 1 in 400,000.
 *     The statistical false-positive of the timeout (phone still
 *     advertising but we declare it gone) is negligible.
 *   - P(0 matches in 10s) = 0.65^10 = 1.4% (the old comment said 1.3% for
 *     30s and 3.5% for 10s -- both wrong: 1.3% IS the 10s figure).
 *   - So 30s is NOT a statistics-driven choice; it is a UX/robustness
 *     choice: bridge brief disappearances (pocket fades, Android doze
 *     throttling advertising below 1 Hz, app hiccups) while still freeing
 *     slots reasonably fast. 60s would hold slots twice as long for a
 *     statistical benefit we do not need.
 *
 * Trade-off: when a phone leaves and comes back within 30s, the slot is still
 * reserved and Kalman state is preserved (fast re-convergence, good UX).
 * When a phone leaves for >30s, we treat it as "new" on return (fresh Kalman,
 * ~6s convergence, slightly worse UX but acceptable -- phone was genuinely
 * gone).
 *
 * Production revisit (Phase 6): if residents complain about slow re-unlock
 * after being away for 30-60s, bump to 60s. RAM cost: same (slots are reused,
 * not added).
 */
#define PROX_TIMEOUT_MS (30 * 1000)

/*
 * Proximity tick period (milliseconds).
 *
 * proximity_tick() runs every PROX_TICK_MS via k_work_delayable.
 * It scans all tracked phones and:
 *   - Checks 30s timeout (frees gone phones)
 *   - Logs state + time-in-state at DBG level (so we can see the state
 *     machine working when DBG is enabled)
 *   - In L4: will also check dwell timers for state transitions
 *
 * Why 1 second:
 *   - Fine enough to catch 30s timeout within 1s of expiry (good enough for
 *     cleanup)
 *   - Fine enough for 3s/5s dwell timers (L4) -- within 1s of dwell expiry
 *   - Coarse enough to not waste workqueue CPU (8 phones * O(1) = ~50us/tick)
 *   - Matches the 1 Hz phone advertising rate (one tick per expected MATCH)
 *
 * Alternatives:
 *   - 100ms: 10x more CPU, no benefit at 1 Hz input. Wasted.
 *   - 5s: too coarse for 3s dwell (would trigger 2s late). Bad for
 *         responsiveness.
 *   - 1s: sweet spot. Will revisit if asymmetric scan (L5) changes input rate.
 */
#define PROX_TICK_MS 1000

/* --- Kalman filter tunables (Phase 3 ch2) ------------------------------- */

/*
 * Process noise: how much the "true RSSI" may drift per second, dB^2/s.
 * This is the TIME-NORMALIZED q of design addendum 3-L1-A: per-step
 * process noise is q_step = Q_PER_SEC * dt, with dt computed from
 * k_uptime timestamps of consecutive MATCHes -- NEVER from sample counts.
 * Consequence: the filter behaves consistently in TIME at any observed
 * sample rate (0.35 Hz pocket-at-40%-duty, 1 Hz pocket-at-100%-duty,
 * 10 Hz Phase-6 app) with zero retuning.
 *
 * Physical grounding (calm indoor walk):
 *   - person walks ~1 m/s; at 1-4 m gate range the log-distance path-loss
 *     slope is ~2.5 dB per meter (20*log10(d2/d1), exponent n ~ 2-3)
 *   - so true RSSI drifts ~1-2.5 dB/s -> variance ~1-6 dB^2/s
 *   - Q_PER_SEC = 2.0 (sigma ~1.4 dB/s) sits mid-band for a calm walk
 *
 * Tune UP (4-8): DBG logs show kalman_x lagging a walking person by >3 dB.
 * Tune DOWN (0.5-1): DBG logs show kalman_x jumping on every multipath
 * outlier while the phone is stationary.
 *
 * Steady state for constant dt (per-update q_step = Q_PER_SEC*dt, r):
 *   p_ss = (-q_step + sqrt(q_step^2 + 4*q_step*r)) / 2
 *   K_ss = (p_ss + q_step) / (p_ss + q_step + r)
 * Derivation + operating-point table in the ch2 lesson.
 */
#define PROX_KALMAN_Q_PER_SEC 1.0f

/*
 * Measurement noise: variance of ONE RSSI reading, dB^2. r = 4.0 means
 * sigma_z = 2 dB -- the empirical jitter of a single BLE adv RSSI report
 * from a stationary phone (Phase 2 logs showed +/-3-5 dB swings, which
 * for roughly-Gaussian noise is ~2 sigma).
 *
 * Per-OBSERVATION, not per-second: a noisy measurement is noisy no matter
 * when it arrives. Only the process noise is time-normalized.
 */
#define PROX_KALMAN_R 4.0f

/* --- Trend tunables (Phase 3 ch3) ---------------------------------------- */

/*
 * TREND_RING_N: decimated (x, t) entries kept per phone.
 *
 * Sizing: the slope uses a TREND_WINDOW_MS window, decimation inserts at
 * most one entry per TREND_DECIMATE_MS, so the window can hold at most
 * 5000/500 + 1 = 11 boundary points; the ring keeps the last 10, which
 * spans the whole window at the fastest legal decimated rate (2 Hz).
 * Slower input rates leave the ring partially full -- older entries are
 * simply filtered out at compute time by the age check.
 *
 * RAM: (4 + 4) bytes * 10 = 80 bytes/slot.
 */
#define TREND_RING_N 10

/*
 * TREND_DECIMATE_MS: minimum spacing between ring entries (500 ms).
 *
 * One entry per >= 500 ms, per design addendum 3-L1-A (iii). Skipping
 * faster samples loses NOTHING: the Kalman has already fused them into
 * kalman_x; the ring stores the trajectory of the ESTIMATE, not raw
 * observations. Decimation makes the structure rate-independent: 10
 * entries always span >= 5 s, whether MATCHes arrive at 0.35 Hz (pocket
 * at 40% scan duty) or 10 Hz (Phase 6 app).
 *
 * 500 ms = TREND_WINDOW_MS / TREND_RING_N: the minimum spacing that lets
 * 10 entries cover the 5 s window.
 */
#define TREND_DECIMATE_MS 500

/*
 * TREND_WINDOW_MS: the LSQ slope is fitted over entries younger than
 * this. Rolling window anchored at NOW (not at the newest entry!), so
 * stale entries age out and a silent phone self-invalidates its trend
 * within ~5 s.
 *
 * Why 5 s (addendum (iii)): matches the dwell timescale (3 s unlock /
 * 5 s relock). A longer window lowers slope noise but slows the
 * response to a direction CHANGE (full refresh takes ~W/2 after the
 * change). At this window: sigma_slope ~ 0.25-0.4 dB/s vs a walking
 * person's ~2.5 dB/s (SNR ~7-10; numbers in the L3 lesson Q1).
 */
#define TREND_WINDOW_MS 5000

/*
 * Trend guards: the slope is claimed VALID only with enough evidence.
 * Fewer than TREND_MIN_POINTS points in the window, or a span shorter
 * than TREND_MIN_SPAN_MS, yields trend_valid=0 (i.e.: INVALID) / slope=0.0f -- "no
 * direction claim", NOT "stationary". ALWAYS check trend_valid before
 * using trend_slope.
 *
 * Why guards matter: with 2 points the "slope" is a single difference
 * (pure noise); with 3 close points the regression denominator is tiny
 * and the slope explodes. The guards are an evidence threshold, not an
 * edge-case patch.
 */
#define TREND_MIN_POINTS 4
#define TREND_MIN_SPAN_MS 2000

/*
 * PROX_KALMAN_GATE_SIGMA: innovation (outlier) gate width, in sigmas.
 *
 * The innovation (z - x) is the filter's own prediction residual; its
 * variance is p_pred + r. A residual beyond PROX_KALMAN_GATE_SIGMA
 * sigmas is REJECTED: x keeps the predict step only (p = p_pred, time
 * still passed). The caller still stamps last_match_time, so time
 * accounting stays honest.
 *
 * Why: single-sample multipath spikes (the -76-among--65s excursion in
 * the L2 desk capture) are the #1 corruptor of LSQ slopes -- one 13 dB
 * point among 5 can fake ~1-2 dB/s of slope. A spike never persists, so
 * it never gets in. A GENUINE persistent level shift self-opens the
 * gate: each rejection leaves p = p_pred, so the threshold widens every
 * sample (accept after ~3 rejections at 1 Hz).
 *
 * Set to 0.0f to disable (e.g. if walk tests show lag on fast genuine
 * changes). Full trace in the L3 lesson Q3.
 */
#define PROX_KALMAN_GATE_SIGMA 3.0f

/* --- State machine ---------------------------------------------------- */

/*
 * Phone proximity state machine.
 *
 * Lifecycle:
 *
 *   ABSENT      Phone not being tracked (slot is free, or slot is allocated
 *               but no MATCH received in 30s -> freed).
 *               This is the "ground state" -- no Kalman, no ring buffer,
 *               no timers.
 *
 *   APPROACHING Phone has been seen (at least one MATCH). Kalman is converging.
 *               Scan switches to 100% duty (ch5): catch EVERY advertisement.
 *               (Normalization note, worklog 3-L1-A: sample rate is set by
 *               the phone's adv rate -- 1 Hz in pocket, ~10 Hz with the
 *               Phase 6 app. 100% duty removes OUR 60% drop rate; it does
 *               not add samples.)
 *               In ch1: we ENTER this state on first MATCH and stay here.
 *               In ch4: we EXIT to PRESENT when RSSI > -58 dBm sustained >3s.
 *
 *   PRESENT     Phone is close (RSSI > -58 dBm sustained). UNLOCK fires on
 *               entry to this state. Scan returns to slow duty (ch5; current
 *               params 160/400 units = 100/250 ms = 40% duty, not 25% --
 *               the old label divided a ms window by a unit interval).
 *               In L1: never entered (transition is L4 work).
 *               In L4: we EXIT to DEPARTING when RSSI < -75 dBm sustained >5s.
 *
 *   DEPARTING   Phone is leaving (RSSI < -75 dBm sustained). RELOCK fires on
 *               entry to this state. Scan switches to 100% duty (L5) to confirm
 *               departure.
 *               In L1: never entered (transition is L4 work).
 *               In L4: we EXIT to ABSENT when no MATCH for >5s while in
 *                      DEPARTING, OR when RSSI drops below -90 dBm (out of range).
 *
 * State diagram:
 *
 *   ABSENT --first MATCH--> APPROACHING --RSSI>-58 + 3s dwell--> PRESENT
 *                                                              |
 *                                                              v
 *   ABSENT <--no MATCH 30s-- DEPARTING <--RSSI<-75 + 5s dwell-- PRESENT
 *
 * The L1 implementation only fires: ABSENT -> APPROACHING (on MATCH) and
 * Any -> ABSENT (on 30s timeout). The intermediate transitions are L4 work.
 *
 * Why a state machine (vs. just thresholding RSSI directly):
 *   - Thresholding alone causes flicker: multipath nulls dip RSSI below -75
 *     for 100ms, falsely triggering relock. Dwell (sustained for N seconds)
 *     filters this.
 *   - State machine makes the unlock/relock events EXPLICIT and AUDITABLE.
 *     Each transition is logged. Security review can trace "why did the gate
 *     unlock at 14:23:05?".
 *   - State machine enables asymmetric scan (L5): different scan duty per
 *     state.
 *
 * AUTOSAR analogy (genuine fit): this is like an EcuM mode transition. EcuM
 * has RUN/POST_RUN/SLEEP/RESET states with explicit transition rules and
 * dwell timers (e.g., "stay in POST_RUN for 2s before SLEEP"). Our state
 * machine follows the same pattern: explicit states, explicit transitions,
 * explicit dwell. The BswM mode arbitration ("given current mode + inputs,
 * decide next mode") is similar to what the proximity evaluator will do
 * in L4: "given current state + current RSSI + dwell timer, decide next
 * state".
 */
enum phone_state {
    PHONE_STATE_ABSENT      = 0,
    PHONE_STATE_APPROACHING = 1,
    PHONE_STATE_PRESENT     = 2,
    PHONE_STATE_DEPARTING   = 3,
};

/* String names for logging. Index by enum value. Declared in proximity.c. */
extern const char *const phone_state_name[4];

/* --- Per-phone tracking record ---------------------------------------- */

/*
 * One of these per tracked phone. Lives in the static phones[] array.
 *
 * Lifecycle:
 *   - Allocated by proximity_on_match() when a new UUID is seen
 *   - Updated by proximity_on_match() on each subsequent MATCH from same UUID
 *   - Inspected by proximity_tick() every 1s for timeout/transition logic
 *   - Freed (in_use=false) when 30s timeout expires
 *
 * Threading: only the system workqueue touches this struct (via
 * proximity_on_match and proximity_tick). No locking needed.
 *
 * Layout (Phase 3 ch3): fields are ordered largest-alignment-first, which
 * eliminates ALL padding: sizeof = 136 bytes/slot exactly (was 112 with
 * the ch1 fields; the growth is the 80-byte trend ring). 8 slots = 1088
 * bytes total -- still trivial vs 256 KB RAM. Offset table inside the
 * struct; verified by BUILD_ASSERT in proximity.c.
 *
 * Note on last_match_time vs. handoff's last_match_age:
 *   The handoff document (section 7.4) specified last_match_age as a uint32_t
 *   "in stats windows (10s each)". This L1 implementation uses last_match_time
 *   as int64_t milliseconds (k_uptime_get() value) instead. Rationale:
 *     - More precise (1ms resolution vs 10s)
 *     - Serves multiple uses: 30s timeout check, reappear gap, DBG log
 *     - Single source of truth (no separate age counter to maintain)
 *     - Cost: 4 extra bytes per slot (8 * 4 = 32 bytes total). Trivial.
 *   This is a deliberate deviation from the handoff, documented here for
 *   traceability.
 */
struct phone_track {
    /*
     * Offset table (Phase 3 ch3 ordering, zero padding -- we verify with the
     * BUILD_ASSERT in proximity.c):
     *
     *   0   last_match_time   int64_t  (8)
     *   8   state_enter_time  int64_t  (8)
     *   16  match_count       uint32_t (4)
     *   20  kalman_x          float    (4)
     *   24  kalman_p          float    (4)
     *   28  trend_x[10]       float    (40)
     *   68  trend_t[10]       uint32_t (40)
     *   108 trend_slope       float    (4)
     *   112 kalman_gated      uint16_t (2)
     *   114 uuid[16]          uint8_t  (16)
     *   130 in_use            bool     (1)
     *   131 state             uint8_t  (1)
     *   132 last_rssi         int8_t   (1)
     *   133 trend_head        uint8_t  (1)
     *   134 trend_count       uint8_t  (1)
     *   135 trend_valid       uint8_t  (1)
     *   ---- sizeof = 136 (alignment 8, multiple of 8) ----
     */

    /* --- Match + state timing --- */
    int64_t  last_match_time;      /* k_uptime_get() at last MATCH, ms */
    int64_t  state_enter_time;     /* k_uptime_get() at state entry, ms */

    /* --- Kalman filter (populated since L2, on every MATCH) --- */
    uint32_t match_count;          /* total MATCHes since alloc (debug) */
    float    kalman_x;             /* estimate of true RSSI now, dBm */
    float    kalman_p;             /* variance of estimate, dB^2 */

    /* --- Trend (L3): decimated (x, t) ring + LSQ slope --- */
    /* trend_x/trend_t: trajectory of the Kalman ESTIMATE (Issue #3 fix:
     * never raw RSSI), one entry per >= TREND_DECIMATE_MS. trend_t holds
     * (uint32_t)k_uptime ms: truncation wraps every 49.7 days, but every
     * use computes spans as (int32_t)(t_new - t_old), which is exact for
     * true spans < 24.8 days -- ours are <= 5 s, so the wrap is invisible.
     */
    float    trend_x[TREND_RING_N];
    uint32_t trend_t[TREND_RING_N];
    /* Slope in dB/s: positive = RSSI rising = approaching, negative =
     * receding. Meaningful ONLY when trend_valid == 1: slope == 0.0f with
     * trend_valid == 0 means "no direction claim", NOT "stationary".
     */
    float    trend_slope;
    uint16_t kalman_gated;         /* MATCHes rejected by the 3-sigma gate */

    /* --- Identity + small fields --- */
    uint8_t  uuid[MSD_LEN_UUID];   /* 16 bytes, network order (RFC 4122) */
    bool     in_use;               /* slot occupancy flag */
    uint8_t  state;                /* enum phone_state, current */
    int8_t   last_rssi;            /* most recent RSSI from MATCH, dBm */
    uint8_t  trend_head;           /* next write index in the trend rings */
    uint8_t  trend_count;          /* valid entries, 0..TREND_RING_N */
    uint8_t  trend_valid;          /* 1 = slope passed the evidence guards */
};

/* --- Public API ------------------------------------------------------- */

/*
 * Initialize the proximity tracking module.
 *
 * Call this ONCE from bt_ready_cb(), after scan_start().
 * Idempotent: safe to call multiple times (subsequent calls zero the array
 * and reschedule the tick).
 *
 * What it does:
 *   - Zeros the phones[] array (all slots in_use=false, state=ABSENT)
 *   - Schedules the first proximity_tick() for PROX_TICK_MS later
 *
 * Returns 0 on success, negative errno on failure (only failure is
 * k_work_reschedule failing, which is vanishingly rare).
 */
int proximity_init(void);

/*
 * Handle a MATCH from the scanner.
 *
 * Called from scan_work_handler for each MATCH (CID+version OK, UUID
 * extracted). This is the ONLY input to the proximity module from the BLE
 * side.
 *
 * What it does:
 *   1. Look up phone by UUID in phones[] (linear scan, 8 entries max)
 *   2. If not found, allocate a free slot, copy UUID, set state=ABSENT
 *   3. Update last_rssi, last_match_time, match_count
 *   4. If state == ABSENT, transition to APPROACHING (log at INF)
 *   5. (L4 will add: if RSSI threshold met, transition to PRESENT, fire unlock)
 *
 * Threading: runs in system workqueue context (scan_work_handler).
 * NEVER call this from scan_cb (BT RX context) -- it does too much work.
 *
 * Parameters:
 *   uuid   - 16-byte device UUID (pointer to msd_data[MSD_OFFSET_UUID])
 *   rssi   - RSSI in dBm (signed, e.g., -65)
 *
 * Returns: slot index (0..MAX_TRACKED_PHONES-1) on success,
 *          -ENOSPC if registry full (all 8 slots in use),
 *          -EINVAL on bad input (uuid NULL).
 */
int proximity_on_match(const uint8_t *uuid, int8_t rssi);

/*
 * Get a snapshot of a phone_track slot (for debug/status commands).
 *
 * Copies the slot into caller-provided struct. Returns 0 on success,
 * -EINVAL if index out of range or slot not in use.
 *
 * Threading: caller may be any thread. Snapshot may be slightly stale
 * (read-during-write race on individual fields), but each field is <=8 bytes
 * so reads are atomic on Cortex-M4. Acceptable for diagnostics.
 */
int proximity_get_slot(uint8_t index, struct phone_track *out);

/*
 * Get the current count of tracked phones (slots in use).
 *
 * Returns count 0..MAX_TRACKED_PHONES. For debug/status display.
 */
uint8_t proximity_get_count(void);

#endif /* BLE_PROXIMITY_H_ */