#ifndef BLE_SCAN_H_
#define BLE_SCAN_H_

/*
 * BLE Smart Lock - Gate Scanner
 * Phase 2 Lesson 2: scan start + callback + workqueue logging
 *
 * L2 scope: log every received adv (addr, RSSI, type). No parsing yet.
 * L3 will add bt_data_parse + MSD extraction.
 * L4 will formalize the threading model and add a dedicated workqueue.
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/addr.h>        /* bt_addr_le_t     */


/* --- Scan parameters (locked in Phase 0 L5) ---------------------------
 *
 * scan_interval = 400 * 0.625 ms = 250 ms  (one full cycle over 3 adv ch)
 * scan_window   =  80 * 0.625 ms =  50 ms  (RX-on time per cycle)
 * duty cycle    = 80/400 = 20%
 * scan_options = BT_LE_SCAN_OPT_NONE
 */

#define SCAN_INTERVAL     400   /* 250 ms */
#define SCAN_WINDOW        80   /* 50 ms */
#define SCAN_MSGQ_DEPTH    16   /* advs queued before workqueue drains */

/* --- Adv message queue (callback -> workqueue) ----------------------- */
struct scan_msg {
	bt_addr_le_t addr;          /* 7 bytes: type(1 byte) + MAC Address (6 bytes)   */
	int8_t       rssi;          /* signed: -127:+20 typical  */
	uint8_t      adv_type;      /* raw PDU header type */
};

/**
 * Start BLE passive scan.
 *
 * Must be called AFTER bt_ready_cb fires (i.e. the BT host+controller are
 * fully initialized). Calling before bt_enable completes returns -EAGAIN.
 *
 * Scan parameters are hard-coded to Phase 0 L5 design:
 *   - interval  = 400 * 0.625 ms = 250 ms
 *   - window    =  80 * 0.625 ms =  50 ms
 *   - type      = BT_LE_SCAN_TYPE_PASSIVE
 *   - options   = BT_LE_SCAN_OPT_NONE  (no dup filter, no whitelist)
 *
 * @return 0 on success, negative errno-style code on failure.
 */
int scan_start(void);

/**
 * Stop BLE scan. Safe to call when not scanning (returns -EALREADY, logged).
 *
 * @return 0 on success, negative on failure.
 */
int scan_stop(void);

#endif /* BLE_SCAN_H_ */