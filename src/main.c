// src/main.c — 5-LED diagnostic + USB CDC ACM
//
// LED progress indicator:
//   LED 1 (P0.20) ON  = app started, GPIO configured
//   LED 2 (P0.22) ON  = survived SYS_INIT (pre-main init complete)
//   LED 3 (P0.09) ON  = USB init complete (CDC ACM stack initialized)
//   LED 4 (P1.00) ON  = DTR wait complete (host connected or 5s timeout)
//   LED 5 (P0.11) BLINK = main loop running, logging active
//
// If the app crashes, the LEDs that are ON tell us exactly where.

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/version.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* Reference all 5 LEDs via their device tree node labels */
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

/* Configure a GPIO as output, initially OFF. Returns 0 on success. */
static int config_led(const struct gpio_dt_spec *led)
{
    if (!device_is_ready(led->port)) {
        return -ENODEV;
    }
    return gpio_pin_configure_dt(led, GPIO_OUTPUT_INACTIVE);
}

int main(void)
{
    /* Configure all 5 LEDs. If any fails, we can't signal — spin forever. */
    if (config_led(&led1) < 0 ||
        config_led(&led2) < 0 ||
        config_led(&led3) < 0 ||
        config_led(&led4) < 0 ||
        config_led(&led5) < 0) {
        while (1) { k_sleep(K_SECONDS(1)); }
    }

    /* === Stage 1: app started, GPIO configured === */
     gpio_pin_set_dt(&led1, 1);
     //k_sleep(K_MSEC(300));
     // sob7an allah hya msh sha3'ala ela hena !!!!!!!!!!!!!!
     // ana tested el 8 LEDs w kollohom sha3'aleen HW bs msh bynwro SW
     // ana 3ayz bokra a5ally &led1 dy ta5od el pins eltanya w ashof htsht3'l wla la2 !!
     // ana nfsy afhm eh elly by7sl :((

    /* === Stage 2: survived SYS_INIT ===
     * SYS_INIT runs before main() and includes USB auto-initialization
     * (CONFIG_CDC_ACM_SERIAL_INITIALIZE_AT_BOOT). If we get here,
     * the USB stack initialized without crashing. */
    gpio_pin_set_dt(&led2, 1);
    //k_sleep(K_MSEC(300));

          // investigating if the issue in pinout or software, I know LED1 is HW correct so I'm using it as my benchmark
          //gpio_pin_set_dt(&led1, 1);
          //k_sleep(K_MSEC(4000));

    /* === Stage 3: USB init complete ===
     * The USB CDC ACM device should now be enumerated (or enumerating)
     * on the host. Check `lsusb` and `dmesg` at this point. */
    gpio_pin_set_dt(&led3, 1);

          // investigating if the issue in pinout or software, I know LED1 is HW correct so I'm using it as my benchmark
            //gpio_pin_set_dt(&led1, 1);
            //k_sleep(K_MSEC(4000));

    /* === Stage 4: DTR wait ===
     * Wait for the host to open the serial port (asserts DTR).
     * 5-second timeout — if the host doesn't connect, we proceed anyway. */
    const struct device *const console_dev =
        DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
            // investigating if the issue in pinout or software, I know LED1 is HW correct so I'm using it as my benchmark
            //gpio_pin_set_dt(&led1, 1);
            //k_sleep(K_MSEC(4000));
    if (device_is_ready(console_dev)) {
                  // investigating if the issue in pinout or software, I know LED1 is HW correct so I'm using it as my benchmark
                //gpio_pin_set_dt(&led1, 1);
                //k_sleep(K_MSEC(4000));
        uint32_t dtr = 0;
        for (int i = 0; i < 50 && !dtr; i++) {
            uart_line_ctrl_get(console_dev, UART_LINE_CTRL_DTR, &dtr);
            //k_sleep(K_MSEC(100));
        }
        if (dtr) {
            /* Host connected — give the terminal a moment to settle */
            //k_sleep(K_MSEC(200));
        }
    }

    /* === Stage 4 complete === */
    gpio_pin_set_dt(&led4, 1);

      // investigating if the issue in pinout or software, I know LED1 is HW correct so I'm using it as my benchmark
     //gpio_pin_set_dt(&led1, 1);
     //k_sleep(K_MSEC(4000));


    /* === Main loop === */
    LOG_INF("=== smart-lock-gate booting ===");
    LOG_INF("Board: %s", CONFIG_BOARD);
    LOG_INF("Zephyr version: %s", KERNEL_VERSION_STRING);
    LOG_INF("5 diagnostic LEDs configured");
    LOG_INF("USB CDC ACM console ready");

    int counter = 0;
    while (1) {
        gpio_pin_toggle_dt(&led5);
           //gpio_pin_toggle_dt(&led1);

        LOG_INF("alive counter=%d", counter++);
        k_sleep(K_SECONDS(2));
    }
    return 0;
}