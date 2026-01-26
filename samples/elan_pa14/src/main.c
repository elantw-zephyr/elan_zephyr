#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

#define PA14_PIN 14

int main(void)
{
    const struct device *gpioa = DEVICE_DT_GET(DT_NODELABEL(gpioa));
    if (!device_is_ready(gpioa)) {
        printk("elan_pa14: gpioa device not ready\n");
        return 0;
    }

    /* Initial state: PA14 = HIGH for 1 second */
    if (gpio_pin_configure(gpioa, PA14_PIN, GPIO_OUTPUT) != 0) {
        printk("elan_pa14: failed to configure PA14\n");
        return 0;
    }
    gpio_pin_set(gpioa, PA14_PIN, 1);
    printk("elan_pa14: PA14 set HIGH (initial)\n");
    k_msleep(1000);

    /* Reset: set LOW for ~1 second, then HIGH again */
    printk("elan_pa14: asserting reset (LOW)\n");
    gpio_pin_set(gpioa, PA14_PIN, 0);
    k_msleep(1000);
    gpio_pin_set(gpioa, PA14_PIN, 1);
    printk("elan_pa14: deasserted reset (HIGH)\n");

    /* Keep running; sleep to avoid exiting */
    while (1) {
        k_sleep(K_SECONDS(60));
    }

    return 0;
}
