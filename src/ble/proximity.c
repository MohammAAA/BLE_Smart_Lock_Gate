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
 * Threading (carried from Phase 2 L4):
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
 *   - kalman_x, kalman_p: 0.0 (L2 will initialize properly)
 *   - rssi_buf: zeroed (L3 will populate)
 *   - smoothed_buf: zeroed (L3 will populate)
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
     * memset. Kalman/ring/smoothed fields also 0 -- L2/L3 will initialize.
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
        LOG_DBG("tick: slot=%d state=%s rssi=%d matches=%u age=%lldms in_state=%lldms",
                i, phone_state_name[phones[i].state],
                phones[i].last_rssi, phones[i].match_count,
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
    if (rc != 0) {
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

    /* 2. Update per-MATCH fields */
    phones[slot].last_rssi        = rssi;
    phones[slot].last_match_time = k_uptime_get();
    phones[slot].match_count++;

    /* 3. State machine: ABSENT -> APPROACHING on first MATCH
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