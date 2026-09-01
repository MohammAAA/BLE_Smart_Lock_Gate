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
 *   - ch3 (pending): ring buffer + moving average + trend slope
 *         (rssi_buf / smoothed_buf fields are in struct but not populated
 *         yet; trend = time-window LSQ on decimated (x,t) pairs)
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

/* --- Kalman filter tunables (Phase 3 cg2) ------------------------------- */

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
 * Fields marked "(L2)" or "(L3)" are declared now to avoid struct refactoring
 * later, but are not populated until those lessons.
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
    /* --- Identity (set on alloc, never changed) --- */
    uint8_t  uuid[MSD_LEN_UUID];   /* 16 bytes, network order (RFC 4122) */
    bool     in_use;               /* slot occupancy flag */

    /* --- Current state (updated on transitions) --- */
    uint8_t  state;                /* enum phone_state, current */
    int8_t   last_rssi;            /* most recent RSSI from MATCH, dBm */
    uint32_t match_count;          /* total MATCHes since alloc (for debug) */
    int64_t  last_match_time;      /* k_uptime_get() at last MATCH, ms */
    int64_t  state_enter_time;     /* k_uptime_get() at state entry, ms */

    /* --- Kalman state (populated since ch2, on every MATCH) --- */
    /* x: estimate of the phone's true RSSI right now, dBm.
     * p: variance of that estimate, dB^2 (NOT sigma -- sigma = sqrt(p)).
     * Populated by kalman_update() in proximity.c; see PROX_KALMAN_*
     * tunables above for the model and units.
     */
    float    kalman_x;             /* current estimate (smoothed RSSI) */
    float    kalman_p;             /* current uncertainty (variance) */

    /* --- Ring buffer of raw RSSI (L3 populates) --- */
    int8_t   rssi_buf[32];         /* 32 samples = 32s history at 1 Hz */
    uint8_t  rssi_buf_head;        /* next write index */
    uint8_t  rssi_buf_count;      /* valid entries (0..32) */

    /* --- Smoothed-value ring for trend (L3 populates, fixes Issue #3) --- */
    float    smoothed_buf[5];      /* last 5 Kalman outputs for trend slope */
    uint8_t  smoothed_head;
    uint8_t  smoothed_count;
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