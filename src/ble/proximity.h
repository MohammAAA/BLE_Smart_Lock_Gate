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
 *   - Stub transitions in ch1 (now filled by the ch4 evaluator in the tick):
 *     APPROACHING->PRESENT (dwell+RSSI), PRESENT->DEPARTING,
 *     DEPARTING->ABSENT (implemented as slot-freedom)
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
 *       CH3 LAB (3-L3-C): After testing, the gate FREEZES on sustained genuine ramps
 *       (walk-in/-out) -- rejection freezes x, innovation grows while
 *       the threshold grows only as sqrt(q*t), and the filter goes
 *       blind for 9-12+ s until motion stops. Fix: restore Q=2.0 (we ran Q=1.0, which armed the freeze) + consecutive-
 *       rejection circuit breaker (PROX_KALMAN_GATE_MAX_REJECTS).
 *       Validated in the same lab: R=4.0 is exact in the new open
 *       environment at 1 m (measured sigma_z=2.01 -> R_emp=4.04);
 *       stationary sigma_b=0.28 dB/s -> trust floor 0.8 dB/s; the
 *       deep-fade gate rejections in run5 worked exactly as designed.
 *   ch4 (delivered): the evaluator -- RSSI thresholds on kalman_x +
 *         dwell credit + trend modulation (decision table below, with
 *         the Phase 3 L4 tunables) + UNLOCK/RELOCK events via
 *         registered callback + DEPARTING exits (silence / floor) +
 *         first-sight -90 floor in proximity_on_match.
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
 * RAM cost: 8 * sizeof(struct phone_track) = 8 * 144 bytes = 1152
 *         bytes (Phase 3 ch4 layout, see the offset table at the struct).
 *         Trivial vs nRF52840's 256 KB RAM.
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
 *   - ch4 delivered: refreshes the trend (trend_compute) then runs the
 *     evaluator (evaluate_phone) per phone -- thresholds, dwell
 *     credit, trend modulation, transitions, UNLOCK/RELOCK events
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
 * LAB NOTE (3-L3-C): the value matters beyond tuning -- it is also the
 * gate-opening rate. Each gate rejection leaves p = p_pred = p + q*dt,
 * so the 3-sigma threshold widens ~sqrt(q*t) while a rejected genuine
 * WALK keeps growing its innovation ~s*t. Q too low -> the gate wins
 * the race too often and the filter freezes on ramps (initially, we
 * ran 1.0f and froze for 12+ s on a 7-8 m walk-in test; the Q=2.0
 * replay of the same data tracks the whole walk with zero rejections).
 * Recommendation: Do not tune below 1.5 without re-running the walk lab.
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
#define PROX_KALMAN_Q_PER_SEC 2.0f

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

/*
 * PROX_KALMAN_GATE_MAX_REJECTS: consecutive-rejection circuit breaker.
 * (Phase 3 ch3 lab fix, 3-L3-C.)
 *
 * THE FAILURE THIS CAPS: on a sustained genuine RAMP (a person walking
 * in or out), the first fast samples get rejected; x freezes; the
 * innovation then measures the FULL accumulated drift and grows at the
 * walk rate while the gate threshold grows only ~sqrt(q*t). The gate
 * can stay shut for 9-12+ s (observed in our walk-in test capture:
 * x frozen at -70.9 while the phone walked to -53; it only reopened
 * after motion STOPPED). The L3 lesson's claim "the gate never bites
 * on physical walking" was wrong -- it bit on exactly that.
 *
 * THE INVARIANT: the gate may reject outliers for at most N-1
 * consecutive samples; on the Nth consecutive rejection the sample is
 * force-ACCEPTED through the normal update (no re-init hack). This is
 * honest Bayesian mechanics, not a workaround: N consecutive
 * rejections have already inflated p to ~p0+N*q*dt, so the forced
 * update runs with k ~ 0.7-0.9 -- it mostly trusts the measurement,
 * exactly as it should after ~N seconds of acknowledged blindness.
 * Worst-case estimate staleness: N samples (4 s at 1 Hz, 0.4 s at
 * 10 Hz) -- bounded forever, at any Q, at any walk speed.
 *
 * Why 4: survives the observed 2-3-sample multipath fade BURSTS
 * (Phase 2 L2-A desk capture; the test run5's fades never exceeded 3 in a row)
 * while capping freeze latency at ~4 s. If a lab ever shows 4+
 * consecutive bad samples, the estimate snaps toward them for one
 * sample and recovers on the next -- asymmetric risk: persistent fades
 * pull x DOWN (delayed unlock, safe direction); persistent positive
 * innovations are approaching phones (correct direction). There is no
 * physical mechanism that holds RSSI 10+ dB ABOVE the path-loss truth
 * for 4+ consecutive seconds.
 *
 * Set to 0 to disable (gate alone, pre-lab behavior).
 */
#define PROX_KALMAN_GATE_MAX_REJECTS 4

/* --- Evaluator tunables (Phase 3 ch4) ------------------------------------ */

/*
 * THE EVALUATOR DECISION TABLE. Everything below is consumed in the
 * 1 s tick, in evaluate_phone(), immediately after trend_compute() for
 * the same phone (rule R5: level and trend are read in the same tick
 * that computed them -- one consistent snapshot, serialized on the
 * system workqueue).
 *
 *   state          condition         trend >= +floor    trend <= -floor
 *   -------------  ----------------  -----------------  -----------------
 *   APPROACHING /  kalman_x >        NEUTRAL: nominal   credit PAUSED
 *   DEPARTING      PROX_UNLOCK_RSSI  target 3 s         (pass-through
 *   (return user)  -> PRESENT +      (no confirm on      protection)
 *                  UNLOCK event      the unlock path --
 *                                   see the confirmed-
 *                                   dwell comment)
 *
 *   PRESENT        kalman_x <        target 3 s         credit PAUSED
 *                  PROX_RELOCK_RSSI  (confirmed)        (returner
 *                  -> DEPARTING +                       protection)
 *                  RELOCK event
 *
 * And when the trend is INVALID or |slope| < floor: NEUTRAL -- the
 * nominal targets (3 s unlock / 5 s relock) run unchanged. That is
 * rule R2 made structural: no transition ever REQUIRES a valid trend
 * (at 40% scan duty the trend is mostly INVALID; the evaluator must
 * still work there -- worklog 3-L3-A Q4).
 *
 * THE DWELL INVARIANT (rule R3 extended): the dwell credit RESETS TO
 * ZERO only when the threshold condition itself breaks. Trend
 * evidence can lower the target or pause the credit -- it can NEVER
 * erase accumulated progress. Rationale: the LEVEL evidence (Kalman
 * estimate sustained over seconds) is the primary channel; the trend
 * at 1 Hz is advisory-grade (walker SNR ~3, worklog 3-L3-F); a
 * one-tick slope flicker must not throw away seconds of level
 * evidence.
 */

/*
 * PROX_UNLOCK_RSSI / PROX_RELOCK_RSSI: the two threshold lines, dBm.
 *
 *   unlock: kalman_x > PROX_UNLOCK_RSSI, sustained (dwell) -> PRESENT.
 *           The UNLOCK event fires on ENTRY to PRESENT.
 *   relock: kalman_x < PROX_RELOCK_RSSI, sustained (dwell) -> DEPARTING.
 *           The RELOCK event fires on that transition (from PRESENT).
 *
 * The evaluator reads kalman_x (the filtered estimate), NEVER
 * last_rssi (raw): thresholds on raw RSSI would flicker with every
 * multipath swing -- the entire L2/L3 pipeline exists to make this
 * one comparison honest.
 *
 * The gap between the two lines is a Schmitt-trigger DEAD BAND:
 * between them the state HOLDS (no transition either way). A phone
 * hovering inside the band keeps whatever state it has. Without the
 * band, a phone parked exactly at one threshold would toggle
 * unlock/relock at the noise rate; the dwell alone would slow the
 * chatter, not stop it. (Same construct as BswM mode-request
 * hysteresis; the dwell is the hysteresis TIME, the band is the
 * hysteresis LEVEL.)
 *
 * CALIBRATION IS LOAD-BEARING (L3 lab, 3-L3-F): RSSI at a given
 * distance is installation-specific -- the old desk read x=-56 at
 * 1 m, the new spot reads x=-72 at 1 m. With these defaults at the
 * new spot, -58 only fires at 20-30 cm and the evaluator would never
 * unlock. Per installation:
 *   1. 60 s capture, phone at the door position IN THE POCKET (body
 *      absorption is 5-15 dB and is part of the real number), note
 *      the mean kalman_x = X_door.
 *   2. PROX_UNLOCK_RSSI = round(X_door) - 2 (clears the threshold
 *      with margin while standing at the door).
 *   3. 30 s capture at the "definitely leaving" mark (3-4 m), note
 *      mean x = X_away. PROX_RELOCK_RSSI = round(X_away) + 2.
 *   4. PROX_TREND_FLOOR_DB_S = 3 * sigma_b from the SAME stationary
 *      capture (spread of the slope lines).
 * The defaults below are the original guide's numbers, kept only as
 * placeholders so the constants exist -- expect to replace all
 * three at every install.
 *
 * Security posture: these thresholds are UX TIMING, not security.
 * Any attacker with a relay/spoofer beats any RSSI threshold (the
 * Phase 0 relay-attack discussion). The security boundary is Phase
 * 4's HMAC. Tuning them loose costs seconds of latency, not safety.
 */
#define PROX_UNLOCK_RSSI (-58)
#define PROX_RELOCK_RSSI (-75)

/*
 * Nominal dwell: how long the threshold condition must hold (on
 * kalman_x) before the transition fires, milliseconds.
 *
 * "Sustained" is a CREDIT accumulated in the 1 s tick, not a
 * stopwatch started on condition-true: the credit grows while the
 * condition holds, FREEZES while the trend contradicts, and resets
 * only when the condition breaks (the invariant above).
 *
 * Why 3 s unlock / 5 s relock (handoff 7.1):
 *   - unlock 3 s: at 1 Hz that is ~3 MATCHes of above-threshold
 *     evidence -- enough for the Kalman to have absorbed the crossing
 *     (not just one lucky fade) and for a pass-through to have shown
 *     its falling slope. This is temporal debounce, scaled to the
 *     physics of the channel: a tactile switch bounces for ~10 ms so
 *     we debounce 20 ms; BLE RSSI "bounces" (multipath/body) for
 *     ~1-3 s, so we debounce 3-5 s.
 *   - relock 5 s > unlock 3 s: fail-OPEN on approach, fail-CLOSED on
 *     departure -- the lock stays released a bit long (user experience)
 *     rather than snapping shut on a deep pocket fade (user locked
 *     out). Asymmetric by design; see the lesson's risk table.
 * Rate independence: dwell is TIME-based, deliberately -- the same
 * 3 s at 0.35 Hz and 10 Hz input (addendum 3-L1-A: filters are
 * time-based; the dwell is the last filter in the chain and follows
 * the same law).
 */
#define PROX_UNLOCK_DWELL_MS (3 * 1000)
#define PROX_RELOCK_DWELL_MS (5 * 1000)

/*
 * Confirmed dwell (RELOCK path only): the target shrinks from 5 s to
 * 3 s when the trend CONFIRMS departure (slope <= -floor). Advisory
 * evidence buys 2 s of latency, never correctness -- the nominal
 * target above remains the fallback. Locking sooner is the safe
 * direction (fail-secure), so this is the one place advisory evidence
 * is allowed to ACCELERATE a transition.
 *
 * REMOVED from the UNLOCK path (design iteration, verified in the L4
 * host simulation before delivery): a 2 s confirmed unlock was tried
 * first, and the harness falsified it -- a brisk pass-through (4 dB/s,
 * peak 4 dB above threshold) UNLOCKED, because the 5 s trend window
 * still showed "approaching" for ~2.5 s after the direction flip, so
 * the confirmation fired on stale positive slope exactly when it
 * should not. The trend lags direction changes by ~W/2; a target
 * reachable inside that lag is reachable by a passer-by. Cost of the
 * removal: ~1 s of unlock latency on the return edge. Resurrection
 * criterion: only if the Phase 6 (10 Hz) lab shows slope latency well
 * under 1 s AND pass-through immunity re-validated at that rate.
 */
#define PROX_RELOCK_DWELL_CONFIRM_MS (3 * 1000)

/*
 * PROX_TREND_FLOOR_DB_S: direction trust floor, dB/s. |slope| below
 * the floor is NOISE, not direction -- treated as neutral even when
 * trend_valid == 1 (rule R4).
 *
 * Provisional 1.5: calibrated from the L3 labs as 3*sigma_b
 * (stationary: 0.8 dB/s at the old desk spot, 1.76 at the new open
 * spot -- the floor is ENVIRONMENT-dependent; calibrate per install,
 * procedure above). Walker peaks measured +2.8/-3.0 dB/s stand
 * ~1.6-2x above the calibrated floor; stroll pace ~1.1 dB/s is
 * below it -- the trend honestly abstains there and the nominal
 * dwell carries the decision (advisory grade, by design).
 *
 * Phase 6 (10 Hz app): n=10-11 per window, sigma_b ~0.35-0.4,
 * floor ~1.0-1.2 -- the trend hardens from advisory to reliable.
 */
#define PROX_TREND_FLOOR_DB_S 1.5f

/*
 * PROX_TRACK_FLOOR_RSSI: first-sight allocation floor, dBm. A NEW
 * phone whose first MATCH is at/below this is not even allocated a
 * slot (proximity_on_match returns -ERANGE). Handoff 7.1's "ABSENT ->
 * APPROACHING: first MATCH (RSSI > -90)".
 *
 * This is REGISTRY HYGIENE, not proximity logic: it keeps far-away
 * phones (the bedroom at -85, the neighbor's install at -88) from
 * squatting on 8 slots for 30 s each. Allocation-time ONLY: a phone
 * already tracked that fades deep is NOT evicted -- evicting
 * mid-track would throw away live Kalman state on every deep fade;
 * the DEPARTING exits and the 30 s timeout handle genuine departure.
 *
 * It also doubles as the DEPARTING "out of range" exit: kalman_x <
 * this floor in DEPARTING frees the slot (handoff 7.1's "OR when RSSI
 * drops below -90 dBm"). Two roles, one number -- both mean "the
 * link is not worth acting on".
 */
#define PROX_TRACK_FLOOR_RSSI (-90)

/*
 * PROX_DEPARTING_SILENCE_MS: in DEPARTING, no MATCH for this long ->
 * slot freed (the GONE log line). Handoff 7.1: "no MATCH for >5s
 * while in DEPARTING".
 *
 * Why shorter than the 30 s timeout: DEPARTING means the phone is
 * leaving -- RSSI falling, catch rate collapsing with the link
 * budget. Silence in DEPARTING is departure confirmation, not
 * pocket doze. 5 s at the L4 lab's 100% scan duty is decisive; in
 * L5 this state runs 100% duty too (handoff 7.3) for exactly this
 * reason -- at 40% duty a catch gap > 5 s happens ~12% of the time
 * (0.65^5) and would false-fire this exit (lesson exercise Q3).
 */
#define PROX_DEPARTING_SILENCE_MS (5 * 1000)

/*
 * PROX_EVIDENCE_FRESH_MS: the evidence horizon. A MATCH older than
 * this and the evaluator does not run at all: the dwell credit
 * FREEZES (no reset -- silence is absent evidence, not contrary
 * evidence; a pocket-dozed phone that resumes above the threshold
 * picks its credit back up).
 *
 * Deliberately an ALIAS of TREND_WINDOW_MS -- one number, one
 * meaning: the horizon within which we hold BOTH a direction claim
 * (the trend window) and a level claim worth acting on. Beyond 5 s
 * of silence the trend has already self-invalidated; the Kalman x
 * is a 5 s old photograph. Splitting the two constants apart is
 * allowed but understand they are tied philosophically: freshness
 * of level and freshness of direction are the same freshness.
 */
#define PROX_EVIDENCE_FRESH_MS TREND_WINDOW_MS

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
 *               ch4 delivered: we EXIT to PRESENT when kalman_x >
 *               PROX_UNLOCK_RSSI and the dwell credit reaches 3 s
 *               (nominal only -- the trend can PAUSE the credit but
 *               never accelerates the unlock; see the confirmed-dwell
 *               comment for why).
 *
 *   PRESENT     Phone is close (RSSI > -58 dBm sustained). UNLOCK fires on
 *               entry to this state. Scan returns to slow duty (ch5; current
 *               params 160/400 units = 100/250 ms = 40% duty, not 25% --
 *               the old label divided a ms window by a unit interval).
 *               In L1: never entered (transition is L4 work).
 *               ch4 delivered: we EXIT to DEPARTING when kalman_x <
 *               PROX_RELOCK_RSSI and the dwell credit reaches its target
 *               (3 s if the trend confirms departure / 5 s nominal).
 *               UNLOCK fires on ENTRY to this state.
 *
 *   DEPARTING   Phone is leaving (RSSI < -75 dBm sustained). RELOCK fires on
 *               entry to this state. Scan switches to 100% duty (L5) to confirm
 *               departure.
 *               In L1: never entered (transition is L4 work).
 *               ch4 delivered: EXIT to PRESENT if the user walks back in
 *                      (x > unlock threshold + dwell -- the RETURN EDGE,
 *                      added in L4: the handoff 7.1 machine had no way
 *                      back except full re-discovery as a new phone).
 *                      EXIT the slot (freed) when silent for
 *                      PROX_DEPARTING_SILENCE_MS or when kalman_x <
 *                      PROX_TRACK_FLOOR_RSSI. "Exit to ABSENT" is
 *                      implemented as FREEING the slot: in this design
 *                      ABSENT is slot-freedom, not a live state value --
 *                      a freed slot IS the absent phone. The GONE log
 *                      line is the audit record.
 *
 * State diagram (ch4 delivered):
 *
 *   ABSENT --first MATCH (rssi > -90)--> APPROACHING
 *
 *   APPROACHING --x > UNLOCK + dwell 3s--> PRESENT         [UNLOCK event]
 *   DEPARTING   --x > UNLOCK + dwell 3s--> PRESENT         [UNLOCK event,
 *                                                            the return edge]
 *   PRESENT     --x < RELOCK + dwell(3s|5s)--> DEPARTING   [RELOCK event]
 *   DEPARTING   --silent > PROX_DEPARTING_SILENCE_MS or x < -90-->
 *                                                        (slot freed; GONE)
 *   any state   --no MATCH 30 s--> (slot freed; TIMEOUT)
 *
 * dwell(A|B): B nominal, A when the trend CONFIRMS departure (relock
 * path only). The trend can also PAUSE the credit on either path
 * (contradiction) -- see the evaluator decision table with the Phase 3
 * ch4 tunables, and the dwell invariant: the trend never resets the
 * credit; only the threshold condition breaking does. The L1 skeleton
 * fired only ABSENT -> APPROACHING and the 30 s timeout; ch4 fills in
 * the rest.
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
 * Layout (Phase 3 ch4): fields are ordered largest-alignment-first.
 * sizeof = 144 bytes/slot (L3 was 136; +4 for dwell_ms, and 140 rounds
 * up to the struct's 8-byte alignment -- the 4 tail padding bytes are
 * DOCUMENTED, not accidental). 8 slots = 1152 bytes total -- still
 * trivial vs 256 KB RAM. Offset table inside the struct; verified by
 * BUILD_ASSERT in proximity.c.
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
     * Offset table (Phase 3 ch4 ordering, largest-alignment-first -- verify
     * with the BUILD_ASSERT in proximity.c):
     *
     *   0   last_match_time   int64_t  (8)
     *   8   state_enter_time  int64_t  (8)
     *   16  match_count       uint32_t (4)
     *   20  dwell_ms          uint32_t (4)
     *   24  kalman_x          float    (4)
     *   28  kalman_p          float    (4)
     *   32  trend_x[10]       float    (40)
     *   72  trend_t[10]       uint32_t (40)
     *   112 trend_slope       float    (4)
     *   116 kalman_gated      uint8_t  (1)
     *   117 kalman_reject_str  uint8_t  (1)
     *   118 uuid[16]          uint8_t  (16)
     *   134 in_use            bool     (1)
     *   135 state             uint8_t  (1)
     *   136 last_rssi         int8_t   (1)
     *   137 trend_head        uint8_t  (1)
     *   138 trend_count       uint8_t  (1)
     *   139 trend_valid       uint8_t  (1)
     *   140 (4 bytes tail padding to the 8-byte struct alignment)
     *   ---- sizeof = 144 (multiple of 8) ----
     */

    /* --- Match + state timing --- */
    int64_t  last_match_time;      /* k_uptime_get() at last MATCH, ms */
    int64_t  state_enter_time;     /* k_uptime_get() at state entry, ms */

    /* --- Kalman filter (populated since L2, on every MATCH) --- */
    uint32_t match_count;          /* total MATCHes since alloc (debug) */
    /*
     * Dwell credit toward the CURRENT state's exit condition, ms
     * (Phase 3 ch4). Grows by PROX_TICK_MS each tick the threshold
     * condition holds; freezes while the trend contradicts (the
     * invariant: only the condition breaking resets it); zeroed on
     * every state transition (transition_phone owns that -- the
     * credit belongs to the OLD state's exit).
     *
     * Bounded in practice by the largest target (5 s) + pause slack;
     * uint32 ms wraps only after 49.7 days of CONTINUOUS hold, which
     * no physical RSSI trajectory can sustain (a sustained +1.5 dB/s
     * walk exhausts the dynamic range in ~40 s). No overflow guard
     * needed -- guards-by-design, same policy as kalman_update.
     */
    uint32_t dwell_ms;             /* Phase 3 ch4 evaluator accumulator */
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
    /* MATCHes rejected by the 3-sigma gate since alloc. Saturating at
     * 255 (was uint16_t; the shrink pays for the streak counter below
     * without changing sizeof -- 8 slots x 2 bytes saved). A diagnostic
     * total, not logic: saturation just parks it at 255.
     */
    uint8_t  kalman_gated;
    /* Consecutive gate rejections, reset on every acceptance. Drives
     * the circuit breaker (PROX_KALMAN_GATE_MAX_REJECTS): the gate may
     * never freeze the filter for more than N-1 consecutive samples.
     */
    uint8_t  kalman_reject_streak;

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
 * Phase 3 ch4: proximity events.
 *
 * UNLOCK fires on every ENTRY to PRESENT: from APPROACHING (first
 * arrival) or from DEPARTING (the user came back). RELOCK fires on
 * PRESENT -> DEPARTING (the only entry path to DEPARTING in this
 * machine -- and the only context where "relock" means anything:
 * the lock was open).
 *
 * Callback contract:
 *   - Called in SYSTEM WORKQUEUE context (the proximity tick). Must
 *     not block, must not call back into the proximity module.
 *   - uuid points into the slot's storage: valid for the duration of
 *     the call ONLY. Copy it if you need it later.
 *   - The rich audit lines (uuid, x, slope, dwell reached) are logged
 *     by proximity.c itself at INF; the callback is for ACTUATION
 *     (relay, LEDs, and later the Phase 5 gate state machine).
 *     Layering: detection emits events, the owner of the hardware
 *     consumes them.
 *
 * Register ONCE from main() BEFORE bt_enable() (so no event can fire
 * unregistered); passing NULL unregisters. The pointer is plain
 * state -- safe because registration completes before the system
 * workqueue ever runs the evaluator that fires events.
 *
 * Phase 4 note: this is the authentication interception point. A
 * future auth gate sits between these events and the relay, and the
 * evaluator itself will consume the phone's auth status.
 */
enum proximity_event {
    PROX_EVENT_UNLOCK = 0,   /* entered PRESENT: person at the door */
    PROX_EVENT_RELOCK = 1,   /* PRESENT -> DEPARTING: person confirmed leaving */
};

typedef void (*proximity_event_cb_t)(enum proximity_event evt,
                                     uint8_t slot,
                                     const uint8_t *uuid);

/*
 * Register the event callback (see the contract above).
 * Returns 0 (plain pointer store; cannot fail).
 */
int proximity_register_event_cb(proximity_event_cb_t cb);

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
 *   5. (ch4 delivered: the threshold/dwell transitions live in the TICK's
 *      evaluator, evaluate_phone -- NOT here. Rule R5: level and trend
 *      are consumed in the same tick that computed them. The one thing
 *      added HERE is the first-sight floor below.)
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
 *          -ERANGE if a NEW phone's first RSSI is <=
 *                  PROX_TRACK_FLOOR_RSSI (not tracked -- registry
 *                  hygiene, Phase 3 L4),
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