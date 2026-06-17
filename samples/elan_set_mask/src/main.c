/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * EM32F967 GPIO masked write test sample.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(elan_set_mask, LOG_LEVEL_INF);

#define GPIOA_NODE DT_NODELABEL(gpioa)
#define TEST_PIN 14

int main(void)
{
    const struct device *gpio_dev = DEVICE_DT_GET(GPIOA_NODE);
    int ret;
    uint32_t port_value;

    if (!device_is_ready(gpio_dev)) {
        LOG_ERR("GPIOA device is not ready");
        return 0;
    }

    ret = gpio_pin_configure(gpio_dev, TEST_PIN, GPIO_OUTPUT_LOW);
    if (ret < 0) {
        LOG_ERR("Failed to configure PA%u: %d", TEST_PIN, ret);
        return 0;
    }

    LOG_INF("Starting masked write test on %s pin %u", gpio_dev->name, TEST_PIN);

    while (1) {
        LOG_INF("Setting pin with gpio_port_set_masked_raw()");
        gpio_port_set_masked_raw(gpio_dev, BIT(TEST_PIN), BIT(TEST_PIN));
        k_msleep(500);
        gpio_port_get_raw(gpio_dev, &port_value);
        LOG_INF("Port value after set: 0x%08x", port_value);

        LOG_INF("Clearing pin with gpio_port_set_masked_raw()");
        gpio_port_set_masked_raw(gpio_dev, BIT(TEST_PIN), 0);
        k_msleep(500);
        gpio_port_get_raw(gpio_dev, &port_value);
        LOG_INF("Port value after clear: 0x%08x", port_value);
    }
}
