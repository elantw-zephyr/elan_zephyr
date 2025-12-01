/*
 * Copyright (c) 2024 ELAN Microelectronics Corp.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <errno.h>

LOG_MODULE_REGISTER(rtc_test, LOG_LEVEL_INF);

/* External diagnostic function from RTC driver */
extern void rtc_em32_diagnose(const struct device *dev);

/* Device references */
static const struct device *rtc_dev;



/* Test RTC basic functionality */
static int test_rtc_basic(void)
{
    struct rtc_time tm_set, tm_get;
    int ret;

    printk("\n=== RTC Basic Test ===\n");

    /* Set initial time: 2024-08-04 12:30:45 */
    tm_set.tm_sec = 45;
    tm_set.tm_min = 30;
    tm_set.tm_hour = 12;
    tm_set.tm_mday = 4;
    tm_set.tm_mon = 7;      /* August (0-based) */
    tm_set.tm_year = 124;   /* 2024 (years since 1900) */
    tm_set.tm_wday = 0;     /* Sunday */
    tm_set.tm_yday = 216;   /* Day of year */

    printk("Setting RTC time to: %04d-%02d-%02d %02d:%02d:%02d\n",
           tm_set.tm_year + 1900, tm_set.tm_mon + 1, tm_set.tm_mday,
           tm_set.tm_hour, tm_set.tm_min, tm_set.tm_sec);

    ret = rtc_set_time(rtc_dev, &tm_set);
    if (ret < 0) {
        printk("Failed to set RTC time: %d\n", ret);
        return ret;
    }

    /* Wait a moment for the time to update */
    k_msleep(1100);

    /* Read back the time */
    ret = rtc_get_time(rtc_dev, &tm_get);
    if (ret < 0) {
        printk("Failed to get RTC time: %d\n", ret);
        return ret;
    }

    printk("Read RTC time: %04d-%02d-%02d %02d:%02d:%02d\n",
           tm_get.tm_year + 1900, tm_get.tm_mon + 1, tm_get.tm_mday,
           tm_get.tm_hour, tm_get.tm_min, tm_get.tm_sec);

    /* Verify the date components match */
    if (tm_get.tm_year != tm_set.tm_year ||
        tm_get.tm_mon != tm_set.tm_mon ||
        tm_get.tm_mday != tm_set.tm_mday ||
        tm_get.tm_hour != tm_set.tm_hour ||
        tm_get.tm_min != tm_set.tm_min) {
        printk("RTC date/time components don't match (expected, some timing difference in seconds is normal)\n");
        return -EIO;
    }

    printk("RTC basic test PASSED (time set successfully and counting)\n");
    return 0;
}







/* Main test function - simplified to focus on RTC functionality */
static int run_rtc_test(void)
{
    int ret;

    printk("\n=== ELAN EM32F967 RTC Test Suite ===\n");

    /* Test RTC basic functionality */
    ret = test_rtc_basic();
    if (ret < 0) {
        printk("RTC basic test failed: %d\n", ret);
        return ret;
    }

    printk("\n=== RTC TEST PASSED ===\n");
    return 0;
}

/* Main application */
int main(void)
{
    int ret;

    /* Very first output to test console */
    printk("\n*** CONSOLE TEST - If you see this, console is working! ***\n");
    k_msleep(100);  /* Small delay to ensure output */
    
    printk("ELAN EM32F967 RTC Test Application\n");
    printk("==================================\n");

    /* Get device bindings */
    rtc_dev = DEVICE_DT_GET(DT_NODELABEL(rtc0));
    if (!device_is_ready(rtc_dev)) {
        printk("ERROR: RTC device is not ready\n");
        return -ENODEV;
    }
    printk("RTC device ready\n");

    /* Run RTC diagnostic to help debug any issues */
    printk("\n=== Running RTC Diagnostic ===\n");
    rtc_em32_diagnose(rtc_dev);



    /* Run RTC test */
    printk("\nRunning RTC test...\n");
    ret = run_rtc_test();
    if (ret < 0) {
        printk("RTC test failed: %d\n", ret);
    } else {
        printk("RTC test completed successfully!\n");
    }

    /* Keep the application running */
    printk("\nRTC test application completed.\n");
    printk("System will continue running. RTC time will be displayed every 10 seconds.\n");

    /* Continuous RTC time display */
    while (1) {
        struct rtc_time tm;
        
        k_msleep(10000);  /* Wait 10 seconds */
        
        ret = rtc_get_time(rtc_dev, &tm);
        if (ret == 0) {
            printk("Current RTC time: %04d-%02d-%02d %02d:%02d:%02d\n",
                   tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                   tm.tm_hour, tm.tm_min, tm.tm_sec);
        }
    }

    return 0;
}
