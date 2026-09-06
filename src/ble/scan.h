#ifndef BLE_SCAN_H_
#define BLE_SCAN_H_

/*
 * BLE Smart Lock - Gate Scanner
 * Phase 2 Checkpoint 4: threading model + stats + drop counters + stack monitor.
 *
 * Ch4 scope:
 *   - Bump SCAN_MSGQ_DEPTH (16 -> 32, more burst tolerance)
 *   - Bump SCAN_WINDOW (80 -> 160, 50 ms -> 100 ms)
 *     (Rationale: 1 Hz phone advertisers need wider window for <1s discovery)
 *   - Stats counters: track advs matched/non-matched/truncated/version-mismatched
 *   - Drop counters: track k_msgq_put failures and scan_cb guard rejects
 *   - Periodic stats dump every 10 s (k_work_delayable)
 *   - Stack high-water mark monitoring for the system workqueue
 *
 * Ch2 scope: logged every received adv (addr, RSSI, type). No parsing yet.
 * Ch3 scope: added bt_data_parse + MSD extraction + version for matched advs
 * Ch4 scope: formalized the threading model and add a dedicated workqueue.
 *
 * Phase 3 ch1 note: this file is UNCHANGED from L4. The proximity module
 * (proximity.h / proximity.c) is a new, separate module. scan.h only needs
 * to provide MSD_LEN_UUID to proximity.h, which it already does.
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/addr.h>        /* bt_addr_le_t     */

/* --- Scan parameters ------------------------------------------------ *
 *
 * Ch4 CHANGES:
 *     Observation after testing: phone at 1 Hz advertising gives ~5 s
 *     discovery latency at window=50 ms.
 *     So we tune SCAN_WINDOW:  80 -> 160  (50 ms -> 100 ms)
 *     Trade-off: duty cycle 20% -> 40%. Power irrelevant (wired). WiFi
 *     coexistence surface ~2x - acceptable for residential application.
 *     Production revisit: Phase 6, when phone app actually drives advertising rate.
 *
 *   - SCAN_MSGQ_DEPTH: 16 -> 32
 *     Bumping to 32 = 1.6 s buffer, handles ~1 s workqueue stalls
 *     (USB reconnect, LOG backpressure). RAM cost: 32 * 41 bytes = 1312
 *     bytes, still 0.5% of nRF52840's 256 KB RAM.
 */

#define SCAN_INTERVAL          400   /* 250 ms */
#define SCAN_WINDOW            SCAN_INTERVAL //160    /* 100 ms */
#define SCAN_MSGQ_DEPTH        32    /* advs queued before workqueue drains */
#define SCAN_STATS_PERIOD_SEC   10   /* periodic stats dump interval */
#define BT_ADV_PAYLOAD_MAX     31U   /* legacy ADV_IND / NONCONN / SCAN_IND */

/* --- Ch3: Our Manufacturer ID ----------------------------------------
 *
 * Use 0xFFFF for development. The Bluetooth SIG reserves this value for
 * testing - no production device is supposed to use it. For our private
 * 2-gate deployment that's fine: we won't collide with anything real in
 * a residential environment.
 *
 * Before shipping this firmware to anyone outside the developer's home,
 * apply for an official Company ID at bluetooth.com (small fee, takes
 * a few days). Replace this #define with the official ID (e.g., 0x1234).
 *
 * Why we filter on CID at all:
 *   the MSD AD structure carries 2-byte Company ID +
 *   payload. Residential environments can have ~10-50 BLE advs constantly
 *   flying around (smart bulbs, TVs, earbuds, fitness trackers, ...). Filtering
 *   on CID lets the gate ignore ~99% of them at the parse layer, before
 *   they reach the proximity algorithm in Phase 3.
 */
#define SMART_LOCK_COMPANY_ID     0xFFFFU

/* --- Ch3: Our MSD payload layout -----------------------------
 *
 * Wire format of the full MSD AD structure:
 *
 *   [Len:1] [Type:1] [Company ID:2 LE] [Payload:22]
 *      |        |            |               |
 *value= 0x19   0xFF        0xFFFF      version+uuid+reservedBytesForSecurity
 *
 *   Total on-wire size: 1 + 1 + 2 + 22 = 26 bytes (fits in 31-byte legacy adv)
 *
 * Length byte value = 25 (0x19):
 *   Length includes the Type byte but NOT itself.
 *   25 = 1 (Type) + 2 (CID) + 22 (Payload).
 *
 * bt_data_parse sees this AD structure as:
 *   data->type      = 0xFF (BT_DATA_MANUFACTURER_DATA)
 *   data->data_len  = 24   (Length - 1 = 25 - 1)
 *   data->data      -> 24-byte buffer: [CID_lo][CID_hi][version][uuid0..15][reserved 0..4]
 *
 * Within data->data (after bt_data_parse strips the Type byte and Length byte):
 *   Offset 0:  cID_lo  (1 byte)
 *   Offset 1:  cID_hi  (1 byte)
 *   Offset 2:  version (1 byte)  - payload format version
 *   Offset 3:  uuid[0] (1 byte) - first byte of device_uuid (MSB / network order)
 *   ...
 *   Offset 18: uuid[15] (1 byte) - last byte of device_uuid
 *   Offset 19-23: reserved (5 bytes) - future use (truncated HMAC in P4)
 *
 * After we strip the Company ID (parse_cb sets msd_data = data->data + 2):
 *   msd_data[0]    = version
 *   msd_data[1..16] = device_uuid (16 bytes)
 *   msd_data[17..21] = reserved (5 bytes)
 *
 * Versioning policy:
 *   Bump SMART_LOCK_PACKET_VERSION on ANY change to this layout. Gate firmware
 *   must REJECT (not silently reinterpret) any adv whose version != current.
 *   This prevents old phones (running an outdated app version) from being
 *   misidentified by new gate firmware.
 */
#define SMART_LOCK_PACKET_VERSION    0x01U

/* Offsets within the MSD payload AFTER Company ID is stripped
 * (i.e: relative to msd_data = data->data + 2). */
#define MSD_OFFSET_VERSION        0U
#define MSD_OFFSET_UUID           1U
#define MSD_LEN_UUID              16U
#define MSD_LEN_PAYLOAD_MIN       (1U + MSD_LEN_UUID)   /* 17: ver + uuid */
#define MSD_LEN_PAYLOAD_FULL      22U                   /* ver + uuid + 5 rsv */

/* --- Adv message queue (callback -> workqueue) -----------------------
 *
 * Ch3 change: scan_msg now carries the adv payload alongside addr/rssi/type.
 * Size grows from 9 bytes (Ch2) to 41 bytes. K_MSGQ_DEFINE buffer grows
 * from 144 bytes to 656 bytes - still trivial on our nRF52840's 256 KB RAM.
 *
 * Why payload is copied inline (not via a pointer):
 *   Option A (inline, chosen): payload bytes live in scan_msg. Simple, no
 *     ownership concerns, no allocation/free. Cost: 656 bytes of static RAM.
 *   Option B (pointer): scan_msg carries a pointer to a payload buffer.
 *     Requires a static ring of payload buffers + index management. More
 *     complex, only worth it if payload were >64 bytes (which it isn't for
 *     legacy adv). Phase 4 may revisit if we add Adv Extensions / extended
 *     payload (up to 254 bytes) - then 16-slot ring * 254 = 4 KB, still
 *     trivial, so probably never worth switching.
 *
 * Threading: this struct is filled by scan_cb (BT RX thread) and consumed
 * by scan_work_handler (system workqueue). k_msgq handles the SPSC transfer
 * safely. The consumer may modify the payload in place (e.g., bt_data_parse
 * advances internal pointers into it) - it owns the copy after k_msgq_get.
 */
struct scan_msg {
	bt_addr_le_t addr;                        /* 7 bytes  */
	int8_t       rssi;                         /* 1 byte   */
	uint8_t      adv_type;                     /* 1 byte   */
	uint8_t      payload_len;                  /* 1 byte, actual bytes copied */
	uint8_t      payload[BT_ADV_PAYLOAD_MAX];   /* 31 bytes */
	/* Total: 41 bytes, natural alignment 1 (no padding) */
};

/* Context passed into bt_data_parse() via 'user_data'. The parse callback
 * fills it with a pointer to our MSD payload (if found) or leaves it
 * at default (found_msd=false).
 *
 * IMPORTANT lifetime: msd_data points INTO msg->payload (the consumer's
 * local copy). Valid only while scan_work_handler is running. Don't stash.
 */
struct parse_ctx {
	bool           found_msd;       /* true if we found an MSD with our CID */
	const uint8_t *msd_data;        /* points into msg->payload (after CID) */
	uint8_t        msd_len;         /* payload length after CID           */
};

/* --- Ch4: Stats counters -------------------------------------------- *
 *
 * Single-producer (scan_cb in BT RX thread) + single-reader (stats_work
 * in system workqueue). 32-bit aligned reads/writes are atomic on
 * Cortex-M4, so no lock needed. Reader may see slightly stale values
 * (eventual consistency) - acceptable for diagnostic metrics.
 *
 * Counter semantics:
 *   - Counted at scan_cb: total_advs, guard_*, queue_drops
 *   - Counted at scan_work_handler: matched, non_matched, trunc_msd, ver_mismatch
 *
 * Why some counters live in scan_cb vs scan_work_handler:
 *   - Counters in scan_cb catch EVERY adv (including ones dropped before
 *     reaching the workqueue due to queue full).
 *   - Counters in scan_work_handler only catch advs that made it through
 *     the queue (so queue_drops is what's missing).
 *
 *   For diagnosis, you want both views. If total_advs - matched - non_matched
 *   - trunc_msd - ver_mismatch != queue_drops, you have a bug.
 */
struct scan_stats {
	/* Counted in scan_cb (BT RX thread) */
	uint32_t total_advs;        /* every adv that reached scan_cb        */
	uint32_t guard_addr_null;  /* addr == NULL (defensive guard)        */
	uint32_t guard_rssi_zero;  /* rssi == 0 (Controller unknown)       */
	uint32_t guard_buf_null;   /* buf == NULL (defensive guard)         */
	uint32_t queue_drops;       /* k_msgq_put returned -ENOMSG (full)     */

	/* Counted in scan_work_handler (system workqueue) */
	uint32_t matched_advs;       /* CID matched, version OK              */
	uint32_t non_matched_advs;   /* CID didn't match (smart bulb etc.)   */
	uint32_t trunc_msd;          /* CID matched but payload < 17 bytes    */
	uint32_t version_mismatch;   /* CID matched but version wrong        */
};


/**
 * Start BLE passive scan.
 *
 * Must be called AFTER bt_ready_cb fires (i.e. the BT host+controller are
 * fully initialized). Calling before bt_enable completes returns -EAGAIN.
 *
 * Scan parameters:
 *   - interval  = 400 * 0.625 ms = 250 ms
 *   - window    = 160 * 0.625 ms =  100 ms
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

/**
 * Get a snapshot of scan stats (copies into caller-provided struct).
 * Useful for caller-side reporting (e.g., status command in main.c).
 */
void scan_get_stats(struct scan_stats *out);
#endif /* BLE_SCAN_H_ */