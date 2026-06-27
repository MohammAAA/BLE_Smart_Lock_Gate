// sanity check
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/version.h>


LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* DT_ALIAS(led0) resolves to the board's "led0" alias in the device tree.
 * For sparkfun_pro_nrf52840_mini, that's P0.15 (the blue status LED).
 * No hardcoded pin — the board DTS handles it. */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

int main(void)
{
    int ret;

    if (!device_is_ready(led.port)) {
        LOG_ERR("LED device not ready — check device tree");
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure LED: %d", ret);
        return ret;
    }

    LOG_INF("smart-lock-gate booting...");
    LOG_INF("Board: %s", CONFIG_BOARD);
    LOG_INF("Zephyr version: %s", KERNEL_VERSION_STRING);
    LOG_INF("LED on %s pin %d (active %s)",
            led.port->name, led.pin,
            led.dt_flags & GPIO_ACTIVE_LOW ? "LOW" : "HIGH");

    int counter = 0;
    while (1) {
        gpio_pin_toggle_dt(&led);
        LOG_INF("alive counter=%d led_state=%d", counter++,
                gpio_pin_get_dt(&led));
        k_sleep(K_SECONDS(3));
    }
    return 0;
}