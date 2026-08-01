// src/main.c — Chapter 11: Intercom Integration Test
//
// Accepts serial commands: 'u'+Enter = unlock, 's'+Enter = status.
// Monitors reed switch and reports gate state changes.
//
// LED progress:
//   LED 1 ON  = app started
//   LED 2 ON  = survived SYS_INIT
//   LED 3 ON  = USB init complete
//   LED 4 ON  = DTR wait complete (i.e.: host is connected)
//   LED 5 BLINK = main loop running

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <zephyr/version.h>
#include <stdio.h>

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

/* Busy-wait delay; does NOT depend on the kernel tick interrupt.
 * Use during boot before the scheduler is guaranteed to be running. */
static void delay_ms(uint32_t ms)
{
    k_busy_wait(ms * 1000);
}

/* Pulse the relay for 500ms to unlock the gate */
static void pulse_relay(void)
{
    printk(">>> UNLOCK: relay is ON for 500ms ..\r\n");
    gpio_pin_set_dt(&relay, 1);
    k_sleep(K_MSEC(500));
    gpio_pin_set_dt(&relay, 0);
    printk(">>> RELOCK: relay is OFF ... \r\n");
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
    delay_ms(300);

    /* === Stage 2: survived SYS_INIT ===
     * SYS_INIT includes USB auto-init. If we get here, USB init
     * completed without crashing. */
    //gpio_pin_set_dt(&led2, 1);
    //delay_ms(300);

    /* === Stage 3: USB init complete ===
     * At this point, the USB CDC ACM device should be enumerating.
     * Check lsusb and dmesg on the host. */
    //gpio_pin_set_dt(&led3, 1);

    /* === Stage 4: DTR wait ===
     * Wait for the host to open the serial port.
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

    /* === Main loop === */
    printk("\r\n=== Smart Lock Gate — Intercom Integration ===\r\n");
    printk("Board: %s\r\n", CONFIG_BOARD);
    printk("Zephyr version: %s\r\n", KERNEL_VERSION_STRING);
    printk("Commands: 'u' + Enter = unlock (pulse relay)\r\n");
    printk("          's' + Enter = status\r\n");
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
                            // unlock the gate
                            pulse_relay();
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