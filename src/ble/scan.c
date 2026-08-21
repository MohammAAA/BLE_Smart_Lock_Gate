/*
 * BLE Smart Lock - Gate Scanner
 * Phase 2 Lesson 2: scan start + callback + workqueue logging
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
#include <zephyr/bluetooth/bluetooth.h>   /* bt_le_scan_start, BT_LE_SCAN_PARAM */
#include <zephyr/bluetooth/addr.h>        /* bt_addr_le_t, bt_addr_le_to_str     */
#include <zephyr/bluetooth/gap.h>         /* BT_GAP_ADV_TYPE_* (currently this is for reference only)  */
#include <zephyr/logging/log.h>

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


/* --- Workqueue handler (drains the queue, logs each adv) ------------- */
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
		LOG_INF("adv: addr=%-30s rssi=%4d type=%u",
			addr_str, msg.rssi, msg.adv_type);
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
 */
static void scan_cb(const bt_addr_le_t *addr,
		    int8_t rssi,
		    uint8_t adv_type,
		    struct net_buf_simple *buf)
{
	ARG_UNUSED(buf);  /* L3 will replace this with bt_data_parse */

	struct scan_msg msg;

	if ((addr == NULL) || (rssi == 0)) {
		return;
	}

	/* Copy out the 3 fields we need. Total 9 bytes. */
	msg.addr     = *addr;
	msg.rssi     = rssi;
	msg.adv_type = adv_type;

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
	LOG_INF("scan started: interval=%u (%u ms) window=%u (%u ms) , type=passive, opt=none",
		SCAN_INTERVAL, (SCAN_INTERVAL * 5 / 8),
		SCAN_WINDOW,   (SCAN_WINDOW   * 5 / 8));
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