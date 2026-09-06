/*
 * BLE Smart Lock - Gate Scanner
 * Phase 2 Ch4: threading model + stats + drop counters + stack monitor
 * Phase 3 Ch1 addition: feed MATCH into proximity tracking module.
 *
 * Ch4 scope:
 *   - stats counters (struct scan_stats)
 *   - drop counter in scan_cb (queue_drops)
 *   - guard counters in scan_cb (addr/rssi/buf NULL)
 *   - matched/non_matched/trunc/ver_mismatch counters in workqueue
 *   - periodic stats dump via k_work_delayable
 *   - stack high-water mark via k_thread_stack_space_get
 *
 * Phase 3 ch1 changes (search for "Phase 3 ch1" to find them):
 *   1. Added #include "proximity.h" near the other includes
 *   2. Added proximity_on_match() call in scan_work_handler, right after
 *      the MATCH log line. This feeds each matched adv into the proximity
 *      tracking module (phone registry + state machine).
 *
 * Threading model:
 *
*   [BT RX thread]                  [System workqueue]            [Log proc thread]
 *   prio ~-7                         prio -1                        prio 0 (low)
 *   --------                         ---------                      -------------
 *   scan_cb  ----k_msgq_put---->   scan_work_handler  ----LOG---->  ring buf -> UART
 *                K_NO_WAIT          k_msgq_get K_NO_WAIT
 *                                    ~50-100us per msg
 *                                   |
 *                                   +-- proximity_on_match() (Phase 3 ch1)
 *                                       updates phone registry + state machine
 *                                   |
 *                                   +-- proximity_tick_handler (every 1s, Phase 3 ch1)
 *                                       timeout cleanup + state evolution
 *                                   |
 *                                   +-- stats_work_handler (every 10s)
 *                                       counter dump + stack monitor
 *
 *                                   stats_work_handler (delayable)
 *                                   every 10s: dump counters + stack
*/

#include "scan.h"
#include "proximity.h"                    /* Phase 3 ch1: phone registry + state machine */
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>   /* bt_le_scan_start, BT_LE_SCAN_PARAM, BT_DATA_MANUFACTURER_DATA, struct bt_data, bt_data_parse */
#include <zephyr/bluetooth/addr.h>        /* bt_addr_le_t, bt_addr_le_to_str     */
#include <zephyr/bluetooth/gap.h>         /* BT_GAP_ADV_TYPE_* (currently this is for reference only)  */
#include <zephyr/logging/log.h>
#include <zephyr/net_buf.h>               /* struct net_buf_simple, net_buf_simple_init_with_data       */
#include <zephyr/sys/printk.h>            /* snprintk */

/*
 * Pin the ble_scan module to the log level (INF) (Phase 3 ch3 lab fix, 3-L3-B).
 *
 * Why: scan_work_handler emits one DBG line per NON-MATCHING adv
 * ("non-match: addr=...") -- in a residential environment that is maybe tens
 * of lines per second. The ch3 lab sets
 * CONFIG_LOG_DEFAULT_LEVEL=4 to expose proximity's DBG tuning lines,
 * which used to arm this firehose too: the deferred-mode log pool
 * (CONFIG_LOG_BUFFER_SIZE) filled, messages were DROPPED, and the
 * kalman:/trend:/tick: lines we actually wanted drowned in noise.
 *
 * Registering with an explicit level makes this module IGNORE
 * CONFIG_LOG_DEFAULT_LEVEL: with max level INF, the LOG_DBG sites in
 * this file compile to nothing -- zero code, zero RAM, cannot flood.
 * (Same intent as CONFIG_BT_LOG_LEVEL_INF / CONFIG_USBD_LOG_LEVEL_WRN
 * already in prj.conf; for an app module the pin lives here because it
 * has no Kconfig plumbing.)
 *
 * To debug adv parsing again (Phase 2 style): temporarily change to
 * LOG_LEVEL_DBG, rebuild, revert.
 
 */
LOG_MODULE_REGISTER(ble_scan, LOG_LEVEL_INF);

/* --- Stats (single producer in BT RX thread, single consumer (reader) in stats work) */
static struct scan_stats stats;

/* k_msgq is a fixed-size ring buffer. Single-producer (BT RX thread) and
 * single-consumer (system workqueue) -> no locking needed internally.
 *
 * K_NO_WAIT (put's) behavior when full: returns -ENOMSG, newest adv is DROPPED.
 * We accept that drop. Phase 3 will add a drop counter and will tune the SCAN_MSGQ_DEPTH.
 *
 * Alignment 4 is conservative; our struct's natural alignment is 1 (all element sizes are 1 byte) but
 * the queue buffer is allocated separately so any alignment >=1 is safe.
 * 
 * macro declaration: #define K_MSGQ_DEFINE(q_name,q_msg_size,q_max_msgs,q_align)
 */
K_MSGQ_DEFINE(scan_msgq,
	      sizeof(struct scan_msg),
	      SCAN_MSGQ_DEPTH,
	      4);

/* parse_cb()
 *
 * bt_data_parse() calls this once per AD structure in the adv payload.
 *
 * Return value semantics:
 *   true  = keep parsing (look at the next AD structure)
 *   false = stop parsing (we found what we need, OR the payload is malformed)
 *
 * Edge cases handled:
 *   - Empty MSD (data_len < 2): can't even read Company ID, return false
 *     to stop. This adv is bogus - don't try to parse further structures.
 *   - Multiple MSDs in one adv: spec says only one MSD per adv. We stop at
 *     the first match. If there's another MSD with our CID later (shouldn't
 *     happen), we'd miss it.
 *   - MSD with our CID but truncated payload: we mark found_msd=true and
 *     stop, then the workqueue handler checks msd_len < MSD_LEN_PAYLOAD_MIN
 *     and rejects.
 */
static bool parse_cb(struct bt_data *data, void *user_data)
{
	struct parse_ctx *ctx = (struct parse_ctx *)user_data;

	if (data->type != BT_DATA_MANUFACTURER_DATA) {
		return true;  /* not MSD, keep looking */
	}

	/* MSD layout: [Company ID: 2 bytes LE] [payload: data_len - 2 bytes] */
	if (data->data_len < 2) {
		/* Malformed: Company ID alone requires 2 bytes. Stop parsing. */
		return false;
	}

	uint16_t cid = (uint16_t)data->data[0]
		     | ((uint16_t)data->data[1] << 8);

	if (cid != SMART_LOCK_COMPANY_ID) {
		return true;  /* MSD but not ours - keep looking (rare) */
	}

	/* Match. Capture pointer + length, stop parsing. */
	ctx->found_msd = true;
	ctx->msd_data = data->data + 2;          /* skip Company ID */
	ctx->msd_len  = (uint8_t)(data->data_len - 2);
	return false;  /* we have what we need */
}

/* Format a 16-byte UUID as 32 lowercase hex chars (no dashes).
 * Uses snprintk for size-safety. Output buffer must be >= 33 bytes.
 *
 * Why no dashes in Ch3:
 *   The dashed RFC 4122 format ("01234567-89ab-cdef-...") is human-friendly
 *   but adds 4 bytes to the log line and 4 format-string evaluations per
 *   adv. In Ch3 we just want to verify parsing works - hex-only is enough.
 *   Phase 4 will switch to a structured log (LOG_INF("uuid", LOG_HEX(...)))
 *   that doesn't need string formatting at all.
 */
static void format_uuid_hex(const uint8_t uuid[MSD_LEN_UUID], char *out, size_t out_len)
{
	if (out_len < (MSD_LEN_UUID * 2U) + 1U) {
		return;  /* buffer too small - caller error, silently bail */
	}

	for (uint8_t i = 0U; i < MSD_LEN_UUID; i++) {
		(void)snprintk(out + (i * 2U), 3U, "%02x", uuid[i]);
	}
	out[MSD_LEN_UUID * 2U] = '\0';
}

/* --- Workqueue handler (drains the queue, parses payload, logs) ------------- */
static void scan_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	struct scan_msg msg;
	char addr_str[BT_ADDR_LE_STR_LEN];  /* 30 bytes: "XX:XX:XX:XX:XX:XX (random)\0" */

	/* Drain everything pending. K_NO_WAIT: we are the consumer, if the
	 * queue is empty we just return - the workqueue thread goes back to
	 * sleep until scan_cb submits us again.
	 */
	while (k_msgq_get(&scan_msgq, &msg, K_NO_WAIT) == 0) {
		bt_addr_le_to_str(&msg.addr, addr_str, sizeof(addr_str));

		/* Wrap msg.payload in a net_buf_simple so bt_data_parse can
		 * iterate it. net_buf_simple_init_with_data does NOT copy -
		 * it just sets pointers into our local msg.payload. bt_data_parse
		 * will advance those pointers as it walks the AD structures,
		 * modifying msg.payload's view of itself - but since msg is
		 * a stack-local copy owned by this function, that's fine.
		 *
		 * Passing msg.payload_len (not BT_ADV_PAYLOAD_MAX) means
		 * bt_data_parse won't try to read past actual data, so trailing
		 * zero-padding (if any) in the controller's report is ignored.
		 */
		struct net_buf_simple parse_buf;
		net_buf_simple_init_with_data(&parse_buf,
		                              msg.payload,
		                              msg.payload_len);

		struct parse_ctx ctx = {
			.found_msd = false,
			.msd_data  = NULL,
			.msd_len   = 0U,
		};
		bt_data_parse(&parse_buf, parse_cb, &ctx);

		if (!ctx.found_msd) {
		
			/* Not our device, increment the non_matched_advs counter */
			stats.non_matched_advs++;
			
			 
			 /* Log at DBG level so INF log doesn't
			 * flood with every unwanter in range. (DBG is off
			 * in prj.conf - these lines won't appear in normal runs.)
			 *
			 * Phase 3 will move these to a per-address drop counter
			 * so we can log "ignored N advs from M non-matching
			 * devices in the last 10s" rather than per-adv spam.
			 */
			LOG_DBG("non-match: addr=%s rssi=%d type=%u len=%u",
				addr_str, msg.rssi, msg.adv_type, msg.payload_len);
			continue;
		}

		/* MSD with our Company ID found. Validate payload length. */
		if (ctx.msd_len < MSD_LEN_PAYLOAD_MIN) {
			stats.trunc_msd++;
			LOG_WRN("trunc MSD: addr=%s rssi=%d msd_len=%u (need %u)",
				addr_str, msg.rssi, ctx.msd_len,
				MSD_LEN_PAYLOAD_MIN);
			continue;
		}

		uint8_t version = ctx.msd_data[MSD_OFFSET_VERSION];
		if (version != SMART_LOCK_PACKET_VERSION) {
			stats.version_mismatch++;
			LOG_WRN("ver mismatch: addr=%s rssi=%d got=%u exp=%u",
				addr_str, msg.rssi, version,
				SMART_LOCK_PACKET_VERSION);
			continue;
		}
	
		stats.matched_advs++;
		/* UUID bytes are in network order (MSB first) per RFC 4122.
		 * The phone app generates a UUID and writes the 16 raw bytes
		 * into the MSD payload in the same order they appear in the
		 * UUID string. So uuid[0] is the first hex pair, etc.
		 */
		char uuid_str[MSD_LEN_UUID * 2U + 1U];  /* 33 bytes */
		format_uuid_hex(&ctx.msd_data[MSD_OFFSET_UUID],
				uuid_str, sizeof(uuid_str));

		/* adv_type is the raw 3-bit PDU Type field from the adv PDU
		 * header. Zephyr passes it as a uint8_t.
		 * It maps to BT_GAP_ADV_TYPE_* constants:
		 *
		 *   0  ADV_IND               connectable, scannable, undirected
		 *   1  ADV_DIRECT_IND        connectable, directed (high duty)
		 *   2  ADV_SCAN_IND          non-conn, scannable (a.k.a. ADV_IND w/o conn)
		 *   3  ADV_NONCONN_IND        non-conn, non-scannable
		 *   4  ADV_DIRECT_IND_LOW_DUTY directed, low duty (BLE 4.1+)
		 *
		 * We expect mostly type 0 (ADV_IND) from phones. Apple devices
		 * sometimes use type 3 (NONCONN) for beaconing.
		 */

                LOG_INF("MATCH: addr=%s rssi=%4d type=%u ver=%u uuid=%s",
                        addr_str, msg.rssi, msg.adv_type, version, uuid_str);

                /* Phase 3 ch1: feed this MATCH into the proximity tracking
                 * module. The proximity module handles:
                 *   - Lookup-or-alloc a phone_track slot by UUID
                 *   - Update last_rssi, last_match_time, match_count
                 *   - State machine: ABSENT -> APPROACHING (on first MATCH)
                 *   - (ch4 delivered: the threshold/dwell transitions live
                 *     in the proximity tick's evaluate_phone, not here)
                 *
                 * We pass a pointer to the UUID bytes within the MSD payload
                 * (ctx.msd_data + MSD_OFFSET_UUID). The proximity module
                 * copies the UUID into its own phone_track.uuid field, so
                 * the pointer does not need to remain valid after this call.
                 *
                 * Return value is the slot index (0..7) on success, or
                 * -ENOSPC if registry full (logged at WRN inside the call).
                 * We don't act on the return value here -- the proximity
                 * module handles all state logic. We just feed it data.
                 *
                 * Threading: this runs in scan_work_handler (system workqueue).
                 * proximity_on_match also runs in system workqueue. No
                 * locking needed.
                 */
                (void)proximity_on_match(&ctx.msd_data[MSD_OFFSET_UUID],
                                         msg.rssi);
        }
}

/* K_WORK_DEFINE declares a static k_work item bound to scan_work_handler.
 * k_work_submit(&scan_work) is idempotent: if already pending or running,
 * it returns without re-queuing. That means even if scan_cb fires 5 times
 * before the workqueue picks it up, scan_work_handler runs ONCE and drains
 * ALL 5 messages in the loop above.
 */
K_WORK_DEFINE(scan_work, scan_work_handler);

/* --- Scan callback (BT RX thread context - DO NOT BLOCK) -------------
 *
 * THIS FUNCTION RUNS IN THE BLUETOOTH RX THREAD.
 *
 * The BT RX thread is a high-priority Zephyr thread (typically priority
 * -8 to -6, i.e. above the system workqueue at -1 and the main thread at 0).
 * It is the ONLY thread that processes Rx LL events: adv reports, connection
 * events, encryption PDUs, HCI command completions. Blocking here stalls
 * the entire BLE stack (i.e.: no more advs are received, no connection events
 * are serviced, encryption handshakes time out, the controller's event
 * queue overflows and we start seeing "BT_EVT RX overflow" warnings).
 *
 * Hard rules:
 *
 *   NO k_sleep / k_msleep / k_busy_wait        - obviously
 *   NO blocking k_msgq_get / k_sem_take        - use K_NO_WAIT
 *   NO mutex_lock (k_mutex)
 *   NO heavy compute (target: < 100 us total in this function)
 *   NO k_malloc (heap may be locked by another thread)
 *   NO LOG_INF on critical hot path *if* LOG is in IMMEDIATE mode.
 *     We are in DEFERRED mode, so LOG_INF here would be a bit
 *     safer - it just copies ~120 bytes into the log ring buffer. But by
 *     convention we still offload logging to the workqueue, because:
 *       (a) it keeps scan_cb minimal and auditable,
 *       (b) the workqueue is on a sensible thread priority where a long
 *           LOG_INF format-string evaluation won't impact BLE timing,
 *
 * Safe operations in scan_cb:
 *   1. Copy fields out of `addr` (bt_addr_le_t is 7 bytes; cheap copy)
 *   2. Copy bytes out of `buf` IF we need them
 *   3. k_msgq_put(&q, &msg, K_NO_WAIT) - lock-free if no contention
 *   4. k_work_submit(&work) - sets a bit and signals the workqueue
 *
 * IMPORTANT: `buf` (struct net_buf_simple *) points to memory owned by the
 * BT stack. The buffer is cleared AFTER this callback returns. If we
 * need any bytes from it, we copy them out NOW. We don't in Ch2; Ch3 uses
 * bt_data_parse() on a stack-local copy.
 *
 * Defensive: rssi == 0 is the Controller's "unknown" sentinel (defined in
 * the HCI Adv Report event spec). We drop those. addr == NULL should never
 * happen but we guard anyway - a NULL deref in BT RX context would
 * hard-fault the whole BT stack and we'd see a K_FATAL panic.
 * 
 * Ch3 changes: we now copy the adv payload out of `buf` into msg.payload
 * before enqueuing. Total work in scan_cb grows from ~10 us (Ch2) to
 * ~20 us (Ch3): one 7-byte addr copy + one 9-byte metadata copy + one
 * up-to-31-byte payload memcpy + ring buffer put + work submit.
 *
 * Still well under the 100 us target. The 31-byte memcpy is ~12 cycles
 * on Cortex-M4 with optimized memcpy (32-byte aligned loop).
 *
 * The payload copy MUST happen here, not in the workqueue. The `buf`
 * pointer is owned by the BT stack and is recycled (returned to its
 * pool) the moment scan_cb returns. If we passed `buf` itself into the
 * queue, by the time the workqueue looked at it, the buffer would
 * contain random other BT event data - silent corruption.
 *
 * Recall Phase 0 Ch4 - the adv payload is up to 31 bytes (legacy ADV_IND)
 * or up to 254 bytes (extended adv with Adv Extensions, BLE 5.0+). We
 * only support legacy in Ch3 (CONFIG_BT_CTLR_ADV_EXT not enabled for
 * scanning). If extended adv support is added later, this copy needs to
 * be guarded against payload_len > 31.
 *
 */
static void scan_cb(const bt_addr_le_t *addr,
		    int8_t rssi,
		    uint8_t adv_type,
		    struct net_buf_simple *buf)
{
	stats.total_advs++;

	/* Defensive guards. Each branch increments its own counter so we
	 * can see which guard is firing (or none). In normal operation all
	 * three should be 0. Non-zero means either a controller bug or
	 * (more likely) a misbehaving device.
	 */
	if (addr == NULL) {
		stats.guard_addr_null++;
		return;
	}
	if (rssi == 0) {
		stats.guard_rssi_zero++;
		return;
	}
	if (buf == NULL) {
		stats.guard_buf_null++;
		return;
	}
	/* Defensive: clamp payload to our buffer size. Should never trigger
	 * for legacy ADV (max 31 bytes), but extended adv reports can be up
	 * to 254 bytes. If we ever enable BT_LE_SCAN_OPT_EXT_ADV, this guard
	 * prevents a buffer overflow (we'd just truncate, not crash).
	 */
	uint8_t copy_len = (uint8_t)buf->len;
	if (copy_len > BT_ADV_PAYLOAD_MAX) {
		copy_len = BT_ADV_PAYLOAD_MAX;
	}
	
	struct scan_msg msg;
	/* Copy out the 3 fields we need. Total 9 bytes. */
	msg.addr     = *addr;
	msg.rssi     = rssi;
	msg.adv_type = adv_type;
	msg.payload_len = copy_len;
	memcpy(msg.payload, buf->data, copy_len);

	/* Enqueue. If the queue is full, k_msgq_put returns -ENOMSG, the
	 * newest adv is dropped, and the queue_drops counter increments.
	 */
	int rc = k_msgq_put(&scan_msgq, &msg, K_NO_WAIT);
	if (rc == -ENOMSG) {
		stats.queue_drops++;
		/* We don't submit the work here, we'd just reschedule a
		 * saturated workqueue. The workqueue will pick up the
		 * next adv when it gets CPU time.
		 */
		return;
	}

	/* Schedule the workqueue to drain. Cheap (set bit + signal) if
	 * already pending; this is the canonical producer/consumer pattern.
	 */
	k_work_submit(&scan_work);
}

/* ---------------------------- Public APIs ----------------------------- */

static void stats_work_handler(struct k_work *work);

K_WORK_DELAYABLE_DEFINE(stats_work, stats_work_handler);

static void stats_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	/* Snapshot under no lock - 32-bit reads are atomic on Cortex-M4.
	 * Reader may see slightly stale values from one counter while
	 * another is fresh - acceptable for diagnostics.
	 */
	uint32_t total       = stats.total_advs;
	uint32_t matched     = stats.matched_advs;
	uint32_t non_match   = stats.non_matched_advs;
	uint32_t trunc       = stats.trunc_msd;
	uint32_t ver_mis     = stats.version_mismatch;
	uint32_t drops       = stats.queue_drops;
	uint32_t g_addr      = stats.guard_addr_null;
	uint32_t g_rssi      = stats.guard_rssi_zero;
	uint32_t g_buf       = stats.guard_buf_null;

	LOG_INF("--- stats (last %us) ---", SCAN_STATS_PERIOD_SEC);
	LOG_INF("  advs received:    %u", total);
	LOG_INF("  matched:          %u", matched);
	LOG_INF("  non-match:        %u", non_match);
	LOG_INF("  trunc MSD:        %u", trunc);
	LOG_INF("  ver mismatch:     %u", ver_mis);
	LOG_INF("  queue drops:      %u", drops);
	LOG_INF("  guards (a/r/b):   %u / %u / %u", g_addr, g_rssi, g_buf);

	/* Sanity check: total should equal sum of all workqueue-side
	 * counters + drops (advs that didn't make it to workqueue).
	 * If not, there's a bug somewhere.
	 */
	uint32_t accounted = matched + non_match + trunc + ver_mis + drops;
	if (accounted != total) {
		LOG_WRN("stats mismatch: total=%u accounted=%u (diff=%u)",
			total, accounted, total - accounted);
	}

	/* Stack monitoring for the system workqueue thread.
	 *
	 * k_thread_stack_space_get: returns 0 on success, -EINVAL if
	 * CONFIG_THREAD_STACK_INFO is not enabled, errno otherwise.
	 * free_bytes is the minimum free stack observed since thread
	 * start (high-water mark). It does NOT go down on allocation,
	 * only up on stack unwinding - so it's the worst-case usage.
	 *
	 * Safe headroom: >256 bytes free is comfortable.
	 * Caution: 128-256 bytes free - monitor, may need bump.
	 * Danger: <128 bytes free - bump CONFIG_SYSTEM_WORKQUEUE_STACK_SIZE
	 *         before adding more work to the workqueue.
	 */
	size_t stack_free = 0;
	int src = k_thread_stack_space_get(&k_sys_work_q.thread, &stack_free);
	if (src == 0) {
		LOG_INF("  sysworkq stack:   %u / %u bytes free",
			stack_free, CONFIG_SYSTEM_WORKQUEUE_STACK_SIZE);
	} else {
		LOG_WRN("  stack info unavailable (err=%d) - enable CONFIG_THREAD_STACK_INFO", src);
	}

	/* Reset counters for next window */
	stats.total_advs       = 0;
	stats.matched_advs     = 0;
	stats.non_matched_advs = 0;
	stats.trunc_msd        = 0;
	stats.version_mismatch = 0;
	stats.queue_drops      = 0;
	stats.guard_addr_null  = 0;
	stats.guard_rssi_zero  = 0;
	stats.guard_buf_null   = 0;

	/* Reschedule ourselves - this is the periodic pattern */
	k_work_reschedule(&stats_work, K_SECONDS(SCAN_STATS_PERIOD_SEC));
}

int scan_start(void)
{
	int err;

	/* BT_LE_SCAN_PARAM(type, options, interval, window) is the official
	 * initializer macro from <zephyr/bluetooth/bluetooth.h>. It expands to
	 * a pointer to a (struct bt_le_scan_param[]) compound literal with
	 *   .type     = BT_LE_SCAN_TYPE_PASSIVE
	 *   .options  = BT_LE_SCAN_OPT_NONE
	 *   .interval = 400
	 *   .window   = 80
	 * and leaves timeout=0 (scan forever) and scan_coded=0 (1M PHY) at
	 * their defaults (the macro handles conditional fields via IF_ENABLED).
	 *
	 * If your tree does NOT have BT_LE_SCAN_OPT_NONE defined (very old
	 * Zephyr), use 0 - the enum value of BT_LE_SCAN_OPT_NONE is literally 0.
	 *
	 */
	err = bt_le_scan_start(BT_LE_SCAN_PARAM(BT_LE_SCAN_TYPE_PASSIVE,
								BT_LE_SCAN_OPT_NONE,
								SCAN_INTERVAL,
								SCAN_WINDOW),
			       			scan_cb);
	if (err) {
		LOG_ERR("bt_le_scan_start failed: %d", err);
		return err;
	}

	/* 0.625 ms = 5/8 ms. Integer-only conversion: ms = units * 5 / 8. */
	LOG_INF("scan started: interval=%u (%u ms) window=%u (%u ms) "
		"type=passive opt=none cid=0x%04X",
		SCAN_INTERVAL, (SCAN_INTERVAL * 5U / 8U),
		SCAN_WINDOW,   (SCAN_WINDOW   * 5U / 8U),
		SMART_LOCK_COMPANY_ID);

	/* Schedule first stats dump after the full period - so the first
	 * dump shows stats accumulated over a full window, not a partial one.
	 */
	k_work_reschedule(&stats_work, K_SECONDS(SCAN_STATS_PERIOD_SEC));
	return 0;
}

int scan_stop(void)
{
	int err = bt_le_scan_stop();

	/* -EALREADY means we weren't scanning. Not an error from the caller's
	 * perspective (idempotent stop). Anything else is logged.
	 */
	if (err && (err != -EALREADY)) {
		LOG_ERR("bt_le_scan_stop failed: %d", err);
		return err;
	}

	/* Cancel the periodic stats work so it doesn't keep firing after stop */
	(void)k_work_cancel_delayable(&stats_work);

	LOG_INF("scan stopped");
	return 0;
}

void scan_get_stats(struct scan_stats *out)
{
	if (out != NULL) {
		*out = stats;
	}
}
