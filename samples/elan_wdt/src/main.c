/*
 * Copyright (c) 2024 ELAN Microelectronics Corp.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief EM32F967 Watchdog Timer Sample Application
 * 
 * This sample demonstrates the EM32F967 watchdog timer functionality:
 * - Basic watchdog setup and feeding
 * - Watchdog timeout handling with interrupt callback
 * - Reset functionality on timeout
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

LOG_MODULE_REGISTER(wdt_sample, LOG_LEVEL_DBG);

/* Device bindings */
#if DT_NODE_HAS_STATUS(DT_ALIAS(watchdog0), okay)
#define WDT_NODE        DT_ALIAS(watchdog0)
#else
#error "No watchdog device found in device tree"
#endif

#if DT_NODE_HAS_STATUS(DT_ALIAS(led0), okay)
#define LED0_NODE       DT_ALIAS(led0)
#define HAS_LED0        1
#else
#define HAS_LED0        0
#endif

/* Device instances */
static const struct device *const wdt_dev = DEVICE_DT_GET(WDT_NODE);

#if HAS_LED0
static const struct device *const led0_dev = DEVICE_DT_GET(DT_GPIO_CTLR(LED0_NODE, gpios));
static const gpio_pin_t led0_pin = DT_GPIO_PIN(LED0_NODE, gpios);
static const gpio_flags_t led0_flags = DT_GPIO_FLAGS(LED0_NODE, gpios);
#endif

/* Watchdog configuration */
#define WDT_TIMEOUT_MS          5000    /* 5 second timeout */
#define WDT_FEED_INTERVAL_MS    2000    /* Feed every 2 seconds */

/* Application state */
static bool wdt_enabled = false;
static int wdt_channel_id = -1;
static struct k_work_delayable wdt_feed_work;

/* Statistics */
static uint32_t feed_count = 0;
static uint32_t timeout_count = 0;

/**
 * @brief Watchdog timeout callback
 * 
 * This function is called when the watchdog timer expires.
 * It provides the last chance to take action before reset.
 */
static void wdt_timeout_callback(const struct device *wdt_dev, int channel_id)
{
    timeout_count++;
    LOG_WRN("Watchdog timeout! Channel: %d, Count: %u", channel_id, timeout_count);
    
    printk("*** WATCHDOG TIMEOUT - SYSTEM WILL RESET ***\n");
    
    /* The system will reset after this callback returns */
}

/**
 * @brief Feed the watchdog timer
 */
static void feed_watchdog(void)
{
    if (!wdt_enabled) {
        return;
    }
    
    int ret = wdt_feed(wdt_dev, wdt_channel_id);
    if (ret == 0) {
        feed_count++;
        LOG_DBG("Watchdog fed successfully, count: %u", feed_count);
        
#if HAS_LED0
        /* Toggle LED0 to show activity */
        gpio_pin_toggle(led0_dev, led0_pin);
#endif
    } else {
        LOG_ERR("Failed to feed watchdog: %d", ret);
    }
}

/**
 * @brief Delayed work handler for automatic watchdog feeding
 */
static void wdt_feed_work_handler(struct k_work *work)
{
    feed_watchdog();
    
    /* Schedule next feed */
    k_work_schedule(&wdt_feed_work, K_MSEC(WDT_FEED_INTERVAL_MS));
}

/**
 * @brief Setup and start the watchdog timer
 */
static int setup_watchdog(void)
{
    struct wdt_timeout_cfg wdt_config = {
        .flags = WDT_FLAG_RESET_SOC,
        .window.min = 0,
        .window.max = WDT_TIMEOUT_MS,
        .callback = wdt_timeout_callback,
    };
    
    if (!device_is_ready(wdt_dev)) {
        LOG_ERR("Watchdog device not ready");
        return -ENODEV;
    }
    
    wdt_channel_id = wdt_install_timeout(wdt_dev, &wdt_config);
    if (wdt_channel_id < 0) {
        LOG_ERR("Failed to install watchdog timeout: %d", wdt_channel_id);
        return wdt_channel_id;
    }
    
    int ret = wdt_setup(wdt_dev, WDT_OPT_PAUSE_HALTED_BY_DBG);
    if (ret < 0) {
        LOG_ERR("Failed to setup watchdog: %d", ret);
        return ret;
    }
    
    wdt_enabled = true;
    LOG_INF("Watchdog setup complete - timeout: %d ms, channel: %d", 
            WDT_TIMEOUT_MS, wdt_channel_id);
    
    return 0;
}

/**
 * @brief Initialize GPIOs
 */
static int init_gpio(void)
{
#if HAS_LED0
    int ret;
    
    /* Initialize LED */
    if (!device_is_ready(led0_dev)) {
        LOG_ERR("LED GPIO device not ready");
        return -ENODEV;
    }
    
    ret = gpio_pin_configure(led0_dev, led0_pin, GPIO_OUTPUT_INACTIVE | led0_flags);
    if (ret < 0) {
        LOG_ERR("Failed to configure LED0: %d", ret);
        return ret;
    }
    
    LOG_INF("GPIO initialization complete");
#endif
    return 0;
}

/**
 * @brief Print status information
 */
static void print_status(void)
{
    printk("\n=== Watchdog Status ===\n");
    printk("Enabled: %s\n", wdt_enabled ? "YES" : "NO");
    printk("Channel ID: %d\n", wdt_channel_id);
    printk("Timeout: %d ms\n", WDT_TIMEOUT_MS);
    printk("Feed interval: %d ms\n", WDT_FEED_INTERVAL_MS);
    printk("Feed count: %u\n", feed_count);
    printk("Timeout count: %u\n", timeout_count);
    printk("Device ready: %s\n", device_is_ready(wdt_dev) ? "YES" : "NO");
}

/**
 * @brief Main application entry point
 */
int main(void)
{
    int ret;
    
    printk("\n*** EM32F967 Watchdog Timer Sample ***\n");
    printk("Version: 1.0\n");
    printk("Build time: " __DATE__ " " __TIME__ "\n");
    
    /* Initialize GPIO */
    ret = init_gpio();
    if (ret < 0) {
        LOG_ERR("GPIO initialization failed: %d", ret);
        return ret;
    }
    
    /* Initialize delayed work for watchdog feeding */
    k_work_init_delayable(&wdt_feed_work, wdt_feed_work_handler);
    
    /* Check watchdog device */
    if (!device_is_ready(wdt_dev)) {
        LOG_ERR("Watchdog device not ready");
        return -ENODEV;
    }
    
    LOG_INF("Watchdog device ready: %s", wdt_dev->name);
    
    /* Setup and start watchdog */
    ret = setup_watchdog();
    if (ret < 0) {
        LOG_ERR("Failed to setup watchdog: %d", ret);
        return ret;
    }
    
    /* Start automatic feeding */
    k_work_schedule(&wdt_feed_work, K_MSEC(WDT_FEED_INTERVAL_MS));
    
    printk("Watchdog started successfully!\n");
    printk("- Timeout: %d ms\n", WDT_TIMEOUT_MS);
    printk("- Feed interval: %d ms\n", WDT_FEED_INTERVAL_MS);
#if HAS_LED0
    printk("- LED0 will blink on each feed\n");
#endif
    printk("- System will reset if feeding stops\n");
    
    /* Show initial status */
    print_status();
    
    /* Main loop - just print status periodically */
    while (1) {
        k_msleep(10000); /* 10 seconds */
        print_status();
        
        /* Test timeout after 30 seconds */
        static int loop_count = 0;
        loop_count++;
        if (loop_count >= 3) {
            printk("\n*** STOPPING WATCHDOG FEEDING - SYSTEM WILL RESET ***\n");
            k_work_cancel_delayable(&wdt_feed_work);
            /* System will reset when watchdog times out */
            break;
        }
    }
    
    /* Wait for reset */
    while (1) {
        k_msleep(1000);
        printk("Waiting for watchdog timeout...\n");
    }
    
    return 0;
}