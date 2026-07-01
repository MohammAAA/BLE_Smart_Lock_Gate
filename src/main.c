// src/main.c — 5-LED diagnostic + USB CDC ACM, NO LOG (preventing deadlock)
//
// Uses k_busy_wait instead of k_sleep (doesn't need tick interrupt (in case a deadlock occurs again)).
// Uses printk instead of LOG_INF (non-blocking, no log backend required).
//
// LED progress:
//   LED 1 ON  = app started
//   LED 2 ON  = survived SYS_INIT
//   LED 3 ON  = USB init complete
//   LED 4 ON  = DTR wait complete
//   LED 5 BLINK = main loop running

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <zephyr/version.h>

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

static int config_led(const struct gpio_dt_spec *led)
{
    if (!device_is_ready(led->port)) {
        return -ENODEV;
    }
    return gpio_pin_configure_dt(led, GPIO_OUTPUT_INACTIVE);
}

/* Busy-wait delay; does NOT depend on the kernel tick interrupt (unlike the k_sleep()).
 * Use this instead of k_sleep when the tick might be starved. */
static void delay_ms(uint32_t ms)
{
    k_busy_wait(ms * 1000);
}

int main(void)
{
    /* Configure all 5 LEDs */
    if (config_led(&led1) < 0 ||
        config_led(&led2) < 0 ||
        config_led(&led3) < 0 ||
        config_led(&led4) < 0 ||
        config_led(&led5) < 0) {
        while (1) { k_sleep(K_SECONDS(1)); }
    }

    /* === Stage 1: app started === */
    gpio_pin_set_dt(&led1, 1);
    delay_ms(300);

    /* === Stage 2: survived SYS_INIT ===
     * SYS_INIT includes USB auto-init. If we get here, USB init
     * completed without crashing. */
    gpio_pin_set_dt(&led2, 1);
    delay_ms(300);

    /* === Stage 3: USB init complete ===
     * At this point, the USB CDC ACM device should be enumerating.
     * Check lsusb and dmesg on the host. */
    gpio_pin_set_dt(&led3, 1);

    /* === Stage 4: DTR wait ===
     * Wait for the host to open the serial port.
     * Use k_sleep here (1-second granularity) with a 10-second timeout.
     * If k_sleep still hangs, the deadlock isn't fully resolved. */
    const struct device *const console_dev =
        DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    if (device_is_ready(console_dev)) {
        uint32_t dtr = 0;
        for (int i = 0; i < 100 && !dtr; i++) {
            uart_line_ctrl_get(console_dev, UART_LINE_CTRL_DTR, &dtr);
            k_sleep(K_MSEC(100));
        }
    }

    gpio_pin_set_dt(&led4, 1);

    /* === Main loop === */
    printk("=== smart-lock-gate booting ===\n");
    printk("Board: %s\n", CONFIG_BOARD);
    printk("Zephyr version: %s\n", KERNEL_VERSION_STRING);
    printk("USB CDC ACM console ready\n");

    int counter = 0;
    while (1) {
        gpio_pin_toggle_dt(&led5);
        printk("alive counter=%d\n", counter++);
        k_sleep(K_SECONDS(2));
    }
    return 0;
}