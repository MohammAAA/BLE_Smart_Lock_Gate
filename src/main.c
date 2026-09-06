// src/main.c
//
// BLE:
//   bt_enable() with async callback pattern 
//   bt_ready_cb() that logs BLE readiness
//
// Phase 3 ch1 changes (search for "Phase 3 ch1" to find them):
//   1. Added #include "ble/proximity.h"
//   2. Added proximity_init() call in bt_ready_cb, right after scan_start()
//      This initializes the phone-tracking registry + schedules the 1s tick.
//
// Phase 3 ch4 changes (search for "Phase 3 ch4" to find them):
//   1. pulse_relay() -> pulse_relay_async(): the 500 ms OFF-step is now a
//      k_work_delayable. The old k_sleep(500) was fine from main-thread
//      context (the 'u' command) but is a BUG from the proximity callback,
//      which runs on the SYSTEM WORKQUEUE -- a sleeping handler there
//      stalls scan_work, proximity_tick and every other sysworkq user.
//   2. proximity_event_handler: UNLOCK -> relay pulse; RELOCK -> logged
//      hook point for the Phase 5 gate state machine.
//   3. proximity_register_event_cb() called before bt_enable() (ordering
//      IS the thread-safety story; see proximity.h).
//   4. 'u' command uses the async pulse too (same code path as the real
//      unlock -- one pulse implementation, not two).
//
// LED mappings:
//   LED 1 ON  = app started
//   LED 2 ON  = survived SYS_INIT (I am currently using this pin for relay triggering)
//   LED 4 ON  = DTR wait complete (i.e.: host is connected)
//   LED 5 BLINK = main loop running
//
// UART commands:
//   'u' + Enter = unlock gate (pulse the relay)
//   's' + Enter = show status

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <zephyr/version.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <stdio.h>
#include "ble/scan.h"
#include "ble/proximity.h"           /* Phase 3 ch1: phone registry + state machine */

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF); // LOG_INF("TEXT"); --> becomes "INF main: TEXT"

/* 5 diagnostic LEDs */
static const struct gpio_dt_spec led1 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(diag_led_1), gpios);
static const struct gpio_dt_spec led2 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(diag_led_2), gpios);
static const struct gpio_dt_spec led3 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(diag_led_3), gpios);
static const struct gpio_dt_spec led4 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(diag_led_4), gpios);
static const struct gpio_dt_spec led5 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(diag_led_5), gpios);

/* Relay driver */
static const struct gpio_dt_spec relay =
    GPIO_DT_SPEC_GET(DT_NODELABEL(diag_led_2), gpios);

/* Reed switch (gate state) */
static const struct gpio_dt_spec reed =
    GPIO_DT_SPEC_GET(DT_NODELABEL(gate_reed), gpios);

static int config_led(const struct gpio_dt_spec *led)
{
    if (!device_is_ready(led->port)) {
        return -ENODEV;
    }
    return gpio_pin_configure_dt(led, GPIO_OUTPUT_INACTIVE);
}

/* ============================================================
 * Phase 3 ch4: non-blocking relay pulse
 * ============================================================ */

/*
 * Why the old pulse_relay() had to die: it called k_sleep(500) between
 * ON and OFF. From main-thread context (the 'u' command) that was
 * merely ugly. From the proximity callback it would be a BUG: the
 * callback runs on the system workqueue, and a sleeping handler there
 * stalls EVERYTHING that shares the queue -- scan_work_handler stops
 * draining (the 32-deep msgq backs up and starts dropping advs within
 * ~1-2 s at ambient traffic), proximity_tick is delayed (dwell credits
 * inflate by the stall), the stats dump slips. Zephyr rule: system
 * workqueue handlers must not block.
 *
 * Fix: split the pulse. ON is a direct GPIO write; the OFF 500 ms later
 * is a k_work_delayable scheduled onto the same (system) workqueue.
 * Re-pulse while active just reschedules the OFF work -- worst case the
 * relay stays ON slightly longer (two residents unlocking at once);
 * harmless for a strike pulse. (One residual, documented: the manual
 * 'u' command runs in MAIN-thread context and can race the OFF work --
 * worst case a manual pulse gets cut short. Dev-only command, ignored.)
 *
 * k_work_delayable vs k_timer: the OFF step toggles a GPIO, legal from
 * a timer callback (ISR) too, but a work item keeps the actuation in
 * THREAD context -- consistent with everything else this project does
 * from workqueues, and the exact pattern already used by
 * proximity_tick and the scan stats dump. No new API to learn.
 */
static void relay_off_handler(struct k_work *work);

K_WORK_DELAYABLE_DEFINE(relay_off_work, relay_off_handler);

static void relay_off_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    gpio_pin_set_dt(&relay, 0);
    /* LOG_INF, not printk: printk is SYNCHRONOUS (it writes through the
     * console driver directly -- can block if the USB host stalls);
     * LOG is deferred to the log thread. From workqueue context only
     * the deferred form is safe. (The GATE / status printks below stay
     * printk: they run in main-thread context.)
     */
    LOG_INF("relay pulse complete (OFF)");
}

static void pulse_relay_async(void)
{
    gpio_pin_set_dt(&relay, 1);
    LOG_INF("UNLOCK: relay ON, off in 500ms");
    (void)k_work_reschedule(&relay_off_work, K_MSEC(500));
}

/*
 * Phase 3 L4: proximity events -> hardware.
 *
 * Runs in SYSTEM WORKQUEUE context (the proximity tick): no blocking,
 * no printk. The audit lines (uuid, x, slope, dwell) are already logged
 * by proximity.c -- this handler is pure actuation.
 *
 * Phase 4: the authentication gate goes here (unlock events from
 * unauthenticated phones must not reach the relay). Phase 5: the gate
 * state machine consumes RELOCK.
 */
static void proximity_event_handler(enum proximity_event evt, uint8_t slot,
                                    const uint8_t *uuid)
{
    ARG_UNUSED(slot);
    ARG_UNUSED(uuid);

    switch (evt) {
    case PROX_EVENT_UNLOCK:
        pulse_relay_async();
        break;
    case PROX_EVENT_RELOCK:
        /* Informational today: the strike is spring-return, there is
         * nothing to actuate. Phase 5 hooks the gate state machine
         * here (relock + reed = gate secured).
         */
        break;
    }
}

/* ============================================================
 * BLE initialization
 * ============================================================ */

/* Called by the BLE stack when init completes (1-2 s after bt_enable()).
 * Runs in the (BT RX) thread context. We must keep it short, and do NOT block. */
static void bt_ready_cb(int err)
{
    if (err) {
        LOG_ERR("BLE init failed: %d", err);
        return;
    }

    LOG_INF("BLE ready");

    /* Log our own BLE MAC address so we can verify identity */
    bt_addr_le_t addr;
    size_t count = 1; /* Request 1 address */
    char addr_str[BT_ADDR_LE_STR_LEN];
    
    /* Fetch the local identity address */
    bt_id_get(&addr, &count);

    /* Verify we actually got an address back */
    if (count > 0) {
        bt_addr_le_to_str(&addr, addr_str, sizeof(addr_str));
        LOG_INF("Gate BLE address: %s", addr_str);
    }

    /* Kick off passive scan */
    LOG_INF("BLE ready, starting scan ...");
    scan_start();

    /* Phase 3 ch1: initialize the phone-tracking registry + schedule the
     * 1-second proximity tick. Must come AFTER scan_start() so that if a
     * phone happens to be advertising the moment BLE comes up, the scan is
     * already running and MATCHes will flow into proximity_on_match.
     *
     * proximity_init() zeros the phones[] array and schedules the first
     * proximity_tick for 1 second from now. Idempotent -- safe to call
     * multiple times.
     *
     * Threading note: this runs in BT RX context (bt_ready_cb is called by
     * the BT stack on the BT RX thread). proximity_init just calls memset +
     * k_work_reschedule, both of which are safe in BT RX context. The
     * actual proximity_tick runs in the system workqueue.
     */
    proximity_init();
}

int main(void)
{
    /* Configure all 5 LEDs */
    if (config_led(&led1) < 0 ||
        config_led(&led2) < 0 ||
        config_led(&led3) < 0 ||
        config_led(&led4) < 0 ||
        config_led(&led5) < 0) {
        while (1)
        {
            k_sleep(K_SECONDS(1));
        }
    }

    /* Configure relay as output, initially OFF */
    if (!device_is_ready(relay.port)) {
        while (1)
        {
            k_sleep(K_SECONDS(1));
        }
    }
    gpio_pin_configure_dt(&relay, GPIO_OUTPUT_INACTIVE);

    /* Configure reed switch as input */
    if (!device_is_ready(reed.port)) {
        while (1) 
        { 
            k_sleep(K_SECONDS(1));
        }
    }
    gpio_pin_configure_dt(&reed, GPIO_INPUT);

    /* === Stage 1: app started === */
    gpio_pin_set_dt(&led1, 1);
    k_busy_wait(300 * 1000);

    /* === Stage 2: survived SYS_INIT ===
     * SYS_INIT includes USB auto-init. If we get here, USB init
     * completed without crashing. */
    //gpio_pin_set_dt(&led2, 1);
    //k_busy_wait(300 * 1000);

    /* === Stage 3: USB init complete ===
     * At this point, the USB CDC ACM device should be enumerating.
     * Check lsusb and dmesg on the host. */
    //gpio_pin_set_dt(&led3, 1);

    /* === Stage 4: DTR wait ===
     * Wait for the host to open the serial port (gives USB time to enumerate before we start logging)
     * 10-second timeout, if host doesn't connect, proceed anyway. */
    const struct device *const console_dev =
        DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    if (device_is_ready(console_dev)) {
        uint32_t dtr = 0;
        for (int i = 0; i < 100 && !dtr; i++) {
            uart_line_ctrl_get(console_dev, UART_LINE_CTRL_DTR, &dtr);
            k_sleep(K_MSEC(100));
        }
        if (dtr) {
            k_sleep(K_MSEC(500));  /* let terminal settle */
        }
    }

    gpio_pin_set_dt(&led4, 1);

    /* Phase 3 ch4: register the actuation callback BEFORE BLE starts
     * (bt_enable -> bt_ready_cb -> scan + proximity). Ordering IS the
     * thread-safety story: the evaluator that fires events only runs
     * on the system workqueue, which is only fed after this point.
     */
    proximity_register_event_cb(proximity_event_handler);

    /* === Start BLE init (async; returns immediately) === */
    int err = bt_enable(bt_ready_cb);
    if (err) {
        LOG_ERR("bt_enable failed: %d", err);
        /* In production, we will return .. but in development phase we don't return to keep the smoke-test loop running so we can debug */
    } else {
        printk("BLE init started, waiting for ready callback...\r\n");
    }

    /* === Main loop .. This is identical to the previous commit === */
    printk("\r\n=== Smart Lock Gate ===\r\n");
    printk("Board: %s\r\n", CONFIG_BOARD);
    printk("Zephyr version: %s\r\n", KERNEL_VERSION_STRING);
    printk("Commands: 'u' + Enter = unlock gate (pulse the relay)\r\n");
    printk("          's' + Enter = show status\r\n");
    printk("\r\n");

    // Monitor the last state of the gate
    // If reed pin is LOW, then the gate is CLOSED.
    uint8_t last_reed_closed = (gpio_pin_get_dt(&reed) == 0);
    printk("Initial gate state: %s\r\n\r\n",
           last_reed_closed ? "CLOSED" : "OPEN");

    /* Serial command buffer */
    char rx_buf[16];
    int rx_idx = 0;

    while (1) {
        gpio_pin_toggle_dt(&led5);

        /* Check reed switch for state changes */
        uint8_t reed_closed = (gpio_pin_get_dt(&reed) == 0);
        if (reed_closed != last_reed_closed) 
        {
            printk(">>> GATE %s\r\n", reed_closed ? "CLOSED" : "OPEN");
            last_reed_closed = reed_closed;
        }

        /* Check for serial input (non-blocking poll) */
        if (device_is_ready(console_dev)) {
            // input character (i.e.: command) that is received by the console
            uint8_t ch;
            while (uart_poll_in(console_dev, &ch) >= 0) 
            {
                if (ch == '\r' || ch == '\n') 
                {
                    if (rx_idx > 0) 
                    {
                        rx_buf[rx_idx] = 0;
                        if (rx_buf[0] == 'u' || rx_buf[0] == 'U') {
                            // unlock the gate (Phase 3 ch4: async pulse,
                            // same code path as the proximity unlock)
                            pulse_relay_async();
                        } else if (rx_buf[0] == 's' || rx_buf[0] == 'S') {
                            printk("Status: gate=%s relay=%d\r\n",
                                   reed_closed ? "CLOSED" : "OPEN",
                                   gpio_pin_get_dt(&relay));
                        } else {
                            printk("Unknown command: '%s'\r\n", rx_buf);
                        }
                        rx_idx = 0;
                    }
                }
                else if (rx_idx < (int)sizeof(rx_buf) - 1) 
                {
                    rx_buf[rx_idx++] = ch;
                }
            }
        }

        k_sleep(K_MSEC(100));  /* 10Hz poll rate — LED 5 toggles every 100ms */
    }
    return 0;
}