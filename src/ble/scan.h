#ifndef BLE_SCAN_H_
#define BLE_SCAN_H_

/*
 * BLE Smart Lock - Gate Scanner
 * Phase 2 Lesson 3: parse adv payload, extract device_uuid from MSD
 *
 *
 * L2 scope: log every received adv (addr, RSSI, type). No parsing yet.
 * L3 will add bt_data_parse + MSD extraction + version for matched advs
 * L4 will formalize the threading model and add a dedicated workqueue.
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/addr.h>        /* bt_addr_le_t     */


/* --- Scan parameters (locked in Phase 0 L5) ---------------------------
 *
 * scan_interval = 400 * 0.625 ms = 250 ms  (one full cycle over 3 adv ch)
 * scan_window   =  160 * 0.625 (i.e.: 5/8) ms =  100 ms  (RX-on time per cycle)
 * duty cycle    = 160/400 = 40%
 * scan_options = BT_LE_SCAN_OPT_NONE
 */

#define SCAN_INTERVAL          400   /* 250 ms */
#define SCAN_WINDOW            160    /* 100 ms */
#define SCAN_MSGQ_DEPTH        16    /* advs queued before workqueue drains */
#define BT_ADV_PAYLOAD_MAX     31U   /* legacy ADV_IND / NONCONN / SCAN_IND */

/* --- L3: Our Manufacturer ID ----------------------------------------
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

/* --- L3: Our MSD payload layout -----------------------------
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
 * L3 change: scan_msg now carries the adv payload alongside addr/rssi/type.
 * Size grows from 9 bytes (L2) to 41 bytes. K_MSGQ_DEFINE buffer grows
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


/**
 * Start BLE passive scan.
 *
 * Must be called AFTER bt_ready_cb fires (i.e. the BT host+controller are
 * fully initialized). Calling before bt_enable completes returns -EAGAIN.
 *
 * Scan parameters are hard-coded to Phase 0 L5 design:
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

#endif /* BLE_SCAN_H_ */