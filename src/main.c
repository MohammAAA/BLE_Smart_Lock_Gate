// src/main.c — external LED blinky on P0.20
//
// This test bypasses the on-board LED (which has confusing wiring
// on the Tenstar/nice!nano clone) and blinks an external LED on P0.20.
//
// Wiring:
//   P0.20 → LED anode (long leg) → 330Ω resistor → GND
//   GND   → LED cathode (short leg, via resistor)
//
// Expected behavior:
//   - External LED blinks on/off every 500ms (clear, visible)
//   - On-board LEDs may or may not do anything (ignore them)
//   - No USB device enumerates (we didn't enable USB)
//
// If the external LED blinks: your toolchain, board definition,
// and GPIO subsystem all work. The project is unblocked.
// If it doesn't blink: check wiring, polarity, resistor value.

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

/* Reference the external LED via its node label in app.overlay.
 * DT_NODELABEL(ext_led) resolves to the ext_led node we defined. */
static const struct gpio_dt_spec ext_led =
    GPIO_DT_SPEC_GET(DT_NODELABEL(ext_led), gpios);

int main(void)
{
    int ret;

    /* Verify the GPIO device is ready */
    if (!device_is_ready(ext_led.port)) {
        /* Cannot signal error without USB or LED — spin forever */
        while (1) {
            k_sleep(K_SECONDS(1));
        }
    }

    /* Configure the pin as output, initially OFF (active-high = 0 = off) */
    ret = gpio_pin_configure_dt(&ext_led, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        while (1) {
            k_sleep(K_SECONDS(1));
        }
    }

    /* Blink the external LED every 500ms */
    while (1) {
        ret = gpio_pin_toggle_dt(&ext_led);
        if (ret < 0) {
            /* Toggle failed — pin may have been reconfigured by another driver */
            while (1) {
                k_sleep(K_SECONDS(1));
            }
        }
        k_sleep(K_MSEC(3000));
    }

    return 0;
}


// // sanity check
// #include <zephyr/kernel.h>
// #include <zephyr/drivers/gpio.h>
// #include <zephyr/logging/log.h>
// #include <zephyr/version.h>

// /* For USB and Serial monitoring */
// #include <zephyr/kernel.h>
// #include <zephyr/drivers/uart.h>
// #include <zephyr/usb/usbd.h>


// LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

// /* DT_ALIAS(led0) resolves to the board's "led0" alias in the device tree.
//  * For sparkfun_pro_nrf52840_mini, that's P0.15 (the blue status LED).
//  * No hardcoded pin — the board DTS handles it. */
// static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

// // /* USB Declarations */
// // /* 1. Define mandatory configuration string descriptor */
// // USBD_DESC_CONFIG_DEFINE(my_usbd_fs_cfg_desc, "FS Configuration");

// // /* 2. Instantiate configuration framework (Exactly 4 arguments for Zephyr 4.4) */
// // USBD_CONFIGURATION_DEFINE(my_usbd_config, USB_SCD_SELF_POWERED, 100, &my_usbd_fs_cfg_desc);

// // /* 3. Define the main hardware device structure mapping */
// // USBD_DEVICE_DEFINE(my_usbd, DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)), 0x1209, 0x0001);



// int main(void)
// {
//     int ret;

//     /* Unlike the Arduino framework, Zephyr requires us to programmatically trigger the USB device stack initialization
//      * in our software before it starts operating. */

//     // /* Initialize base stack framework */
//     // if (usbd_init(&my_usbd) != 0) {
//     //     return 0;
//     // }

//     //  /* Add Full-Speed (FS) configuration parameter (Requires speed enum) */
//     // if (usbd_add_configuration(&my_usbd, USBD_SPEED_FS, &my_usbd_config) != 0) {
//     //     return 0;
//     // }

//     // /* Bind the automated CDC ACM instance using its string node label */
//     // if (usbd_register_class(&my_usbd, "cdc_acm_0", USBD_SPEED_FS, 1) != 0) {
//     //     return 0;
//     // }

//     // /* Enable and start transceiver hardware */
//     // if (usbd_enable(&my_usbd) != 0) {
//     //     return 0;
//     // }


//      /* Block execution block until screen/minicom connects on the host PC */
//     // const struct device *const dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
//     // uint32_t dtr = 0;
//     // while (!dtr) {
//     //     uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
//     //     k_sleep(K_MSEC(100));
//     // }

//     /* The LED toggling Logic starts here */

//     if (!device_is_ready(led.port)) {
//         LOG_ERR("LED device not ready — check device tree");
//         return -ENODEV;
//     }

//     ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
//     if (ret < 0) {
//         LOG_ERR("Failed to configure LED: %d", ret);
//         return ret;
//     }

//     /* Start blinking immediately so we know the app booted */
//     for (int i = 0; i < 5; i++) {
//         gpio_pin_toggle_dt(&led);
//         k_sleep(K_MSEC(100));
//     }

//     LOG_INF("smart-lock-gate booting...");
//     LOG_INF("Board: %s", CONFIG_BOARD);
//     LOG_INF("Zephyr version: %s", KERNEL_VERSION_STRING);
//     LOG_INF("LED on %s pin %d (active %s)",
//             led.port->name, led.pin,
//             led.dt_flags & GPIO_ACTIVE_LOW ? "LOW" : "HIGH");

//     int counter = 0;
//     while (1) {
//         gpio_pin_toggle_dt(&led);
//         LOG_INF("alive counter=%d led_state=%d", counter++,
//                 gpio_pin_get_dt(&led));
//         k_sleep(K_SECONDS(3));
//     }
//     return 0;
// }

