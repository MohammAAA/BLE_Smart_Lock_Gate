/*
 * BLE Smart Lock - Gate Scanner
 * Phase 2 Lesson 2: parse adv payload + extract device_uuid from MSD.
 *
 * L2 scope: log every received adv (addr, RSSI, adv_type). No parsing yet.
 *
 * Threading model (high level - deep dive in L4):
 *
 *   [BT RX thread]                 [System workqueue]      [Log proc thread]
 *   scan_cb  ----k_msgq_put---->  scan_work_handler  ---->  LOG_INF
 *                K_NO_WAIT        k_msgq_get K_NO_WAIT     (deferred UART)
 *
 * Hard rule: scan_cb runs in BT RX thread context. we shall NEVER block there.
 */

#include "scan.h"
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>   /* bt_le_scan_start, BT_LE_SCAN_PARAM, BT_DATA_MANUFACTURER_DATA, struct bt_data, bt_data_parse */
#include <zephyr/bluetooth/addr.h>        /* bt_addr_le_t, bt_addr_le_to_str     */
#include <zephyr/bluetooth/gap.h>         /* BT_GAP_ADV_TYPE_* (currently this is for reference only)  */
#include <zephyr/logging/log.h>
#include <zephyr/net_buf.h>               /* struct net_buf_simple, net_buf_simple_init_with_data       */
#include <zephyr/sys/printk.h>            /* snprintk */

LOG_MODULE_REGISTER(ble_scan);



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
 * Why no dashes in L3:
 *   The dashed RFC 4122 format ("01234567-89ab-cdef-...") is human-friendly
 *   but adds 4 bytes to the log line and 4 format-string evaluations per
 *   adv. In L3 we just want to verify parsing works - hex-only is enough.
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
			/* Not our device. Log at DBG level so INF log doesn't
			 * flood with every smart bulb in range. (DBG is off
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
			LOG_WRN("trunc MSD: addr=%s rssi=%d msd_len=%u (need %u)",
				addr_str, msg.rssi, ctx.msd_len,
				MSD_LEN_PAYLOAD_MIN);
			continue;
		}

		uint8_t version = ctx.msd_data[MSD_OFFSET_VERSION];
		if (version != SMART_LOCK_PACKET_VERSION) {
			LOG_WRN("ver mismatch: addr=%s rssi=%d got=%u exp=%u",
				addr_str, msg.rssi, version,
				SMART_LOCK_PACKET_VERSION);
			continue;
		}

		/* UUID bytes are in network order (MSB first) per RFC 4122.
		 * The phone app generates a UUID and writes the 16 raw bytes
		 * into the MSD payload in the same order they appear in the
		 * UUID string. So uuid[0] is the first hex pair, etc.
		 */
		char uuid_str[MSD_LEN_UUID * 2U + 1U];  /* 33 bytes */
		format_uuid_hex(&ctx.msd_data[MSD_OFFSET_UUID],
				uuid_str, sizeof(uuid_str));

		/* adv_type is the raw 3-bit PDU Type field from the adv PDU
		 * header (recall Phase 0 L4). Zephyr passes it as a uint8_t.
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
 * Hard rules (will be deepened in L4):
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
 *       (c) we'll add parsing in L3 and want a stable structure now (we won't update this callback in L3,
 *           instead we will update the workqueue handler (scan_work_handler())).
 *
 * Safe operations in scan_cb:
 *   1. Copy fields out of `addr` (bt_addr_le_t is 7 bytes; cheap copy)
 *   2. Copy bytes out of `buf` IF we need them (we don't in L2)
 *   3. k_msgq_put(&q, &msg, K_NO_WAIT) - lock-free if no contention
 *   4. k_work_submit(&work) - sets a bit and signals the workqueue
 *
 * IMPORTANT: `buf` (struct net_buf_simple *) points to memory owned by the
 * BT stack. The buffer is cleared AFTER this callback returns. If we
 * need any bytes from it, we copy them out NOW. We don't in L2; L3 will use
 * bt_data_parse() on a stack-local copy.
 *
 * Defensive: rssi == 0 is the Controller's "unknown" sentinel (defined in
 * the HCI Adv Report event spec). We drop those. addr == NULL should never
 * happen but we guard anyway - a NULL deref in BT RX context would
 * hard-fault the whole BT stack and we'd see a K_FATAL panic.
 * 
 * L3 changes: we now copy the adv payload out of `buf` into msg.payload
 * before enqueuing. Total work in scan_cb grows from ~10 us (L2) to
 * ~20 us (L3): one 7-byte addr copy + one 9-byte metadata copy + one
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
 * Recall Phase 0 L4 - the adv payload is up to 31 bytes (legacy ADV_IND)
 * or up to 254 bytes (extended adv with Adv Extensions, BLE 5.0+). We
 * only support legacy in L3 (CONFIG_BT_CTLR_ADV_EXT not enabled for
 * scanning). If extended adv support is added later, this copy needs to
 * be guarded against payload_len > 31.
 *
 */
static void scan_cb(const bt_addr_le_t *addr,
		    int8_t rssi,
		    uint8_t adv_type,
		    struct net_buf_simple *buf)
{

	struct scan_msg msg;

	if ((addr == NULL) || (rssi == 0) || (buf == NULL)) {
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

	/* Copy out the 3 fields we need. Total 9 bytes. */
	msg.addr     = *addr;
	msg.rssi     = rssi;
	msg.adv_type = adv_type;
	msg.payload_len = copy_len;
	memcpy(msg.payload, buf->data, copy_len);

	/* Enqueue. If the queue is full, k_msgq_put returns -ENOMSG and the
	 * newest adv is silently dropped. We'll measure the drop rate in L4
	 * and tune SCAN_MSGQ_DEPTH accordingly. For L2 (just logging) the
	 * drop rate will be ~0 unless we stuff ~25 phones in front of the
	 * gate at once.
	 */
	(void)k_msgq_put(&scan_msgq, &msg, K_NO_WAIT);

	/* Schedule the workqueue to drain. Cheap (set bit + signal) if
	 * already pending; this is the canonical producer/consumer pattern.
	 */
	k_work_submit(&scan_work);
}

/* ---------------------------- Public APIs ----------------------------- */

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
	 * Verify on your machine:
	 *   grep -rn "BT_LE_SCAN_OPT_NONE" /media/rimo/Technical/zephyrproject/zephyr/include/zephyr/bluetooth/
	 *   grep -rn "bt_le_scan_start"     /media/rimo/Technical/zephyrproject/zephyr/include/zephyr/bluetooth/bluetooth.h
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

	LOG_INF("scan stopped");
	return 0;
}