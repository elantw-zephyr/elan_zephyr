/*
 * Copyright (c) 2024 ELAN Microelectronics Corp.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT elan_em32_wdt

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>
#include <soc.h>
#include "../../include/zephyr/drivers/clock_control/clock_control_em32_ahb.h"
#include <elan_em32.h> //TODO: remove elan_em32.h

LOG_MODULE_REGISTER(wdt_em32, CONFIG_WDT_LOG_LEVEL);

/* EM32 WDT register offsets - corrected per spec */
#define WDT_WDOGLOAD_OFFSET     0x00    /* Watchdog Load Register */
#define WDT_WDOGVALUE_OFFSET    0x04    /* Watchdog Value Register (Read-only) */
#define WDT_WDOGCONTROL_OFFSET  0x08    /* Watchdog Control Register */
#define WDT_WDOGINTCLR_OFFSET   0x0C    /* Watchdog Interrupt Clear Register */
#define WDT_WDOGRIS_OFFSET      0x10    /* Watchdog Raw Interrupt Status */
#define WDT_WDOGMIS_OFFSET      0x14    /* Watchdog Masked Interrupt Status */
#define WDT_WDOGLOCK_OFFSET     0xC00   /* Watchdog Lock Register */

/* WDOGCONTROL register bits - corrected per EM32 spec */
#define WDT_WDOGCONTROL_RESERVED_MASK   (0x3FFFFFFF << 2)  /* Reserved bits 31:2 */
#define WDT_WDOGCONTROL_RESEN           BIT(1)              /* Reset enable */
#define WDT_WDOGCONTROL_INTEN           BIT(0)              /* Interrupt enable */

/* WDOGINTCLR register bits */
#define WDT_WDOGINTCLR_RESERVED_MASK    (0x7FFFFFFF << 1)   /* Reserved bits 31:1 */
#define WDT_WDOGINTCLR_CLEAR            BIT(0)              /* Write any value to clear */

/* WDOGRIS register bits */
#define WDT_WDOGRIS_RESERVED_MASK       (0x7FFFFFFF << 1)   /* Reserved bits 31:1 */
#define WDT_WDOGRIS_RAWINT              BIT(0)              /* Raw interrupt status */

/* WDOGMIS register bits */
#define WDT_WDOGMIS_RESERVED_MASK       (0x7FFFFFFF << 1)   /* Reserved bits 31:1 */
#define WDT_WDOGMIS_MASKEDINT           BIT(0)              /* Masked interrupt status */

/* WDOGLOCK register - corrected per EM32 spec */
#define WDT_WDOGLOCK_UNLOCK_VALUE       0x1ACCE551          /* Unlock value */
#define WDT_WDOGLOCK_LOCKED_STATE       0x00000001          /* Any other value locks */

/* Hardware clock gating register */
#define EM32_CLKGATEREG                 0x40030100          /* Clock Gating Control Register */

struct wdt_em32_config {
    uint32_t base;
    const struct device *clock_dev;
    uint32_t clock_group_id;
};

struct wdt_em32_data {
    wdt_callback_t callback;
    uint32_t timeout_ms;
    bool timeout_installed;
};

static inline uint32_t wdt_em32_read(const struct device *dev, uint32_t offset)
{
    const struct wdt_em32_config *config = dev->config;
    return sys_read32(config->base + offset);
}

static inline void wdt_em32_write(const struct device *dev, uint32_t offset, uint32_t value)
{
    const struct wdt_em32_config *config = dev->config;
    sys_write32(value, config->base + offset);
    /* Update again for cdc error 20240216 notice*/
    sys_write32(value, config->base + offset);
}

static void wdt_em32_unlock(const struct device *dev)
{
    /* Write unlock value to WDOGLOCK register */
    wdt_em32_write(dev, WDT_WDOGLOCK_OFFSET, WDT_WDOGLOCK_UNLOCK_VALUE);
}

static void wdt_em32_lock(const struct device *dev)
{
    /* Write any other value to lock (using 1 as specified) */
    wdt_em32_write(dev, WDT_WDOGLOCK_OFFSET, WDT_WDOGLOCK_LOCKED_STATE);
}

static bool wdt_em32_is_locked(const struct device *dev)
{
    uint32_t lock_status = wdt_em32_read(dev, WDT_WDOGLOCK_OFFSET);
    /* When WDOGLOCK_PASSWORD = 0x1ACCE551, bit[0] = 0 (unlocked) */
    /* When else, bit[0] = 1 (locked) */
    return (lock_status & BIT(0)) != 0;
}

static void wdt_em32_disable_clkgate(void)
{
    /* Disable clock gating for watchdog (following hardware sample pattern) */
    volatile uint32_t *clkgate_reg = (volatile uint32_t *)EM32_CLKGATEREG;
    *clkgate_reg &= ~(1U << PCLKG_DWG);  /* Clear bit to enable clock */
}

static void wdt_em32_debug_registers(const struct device *dev)
{
    LOG_DBG("=== WDT Register Dump ===");
    LOG_DBG("WDOGLOAD:    0x%08x", wdt_em32_read(dev, WDT_WDOGLOAD_OFFSET));
    LOG_DBG("WDOGVALUE:   0x%08x", wdt_em32_read(dev, WDT_WDOGVALUE_OFFSET));
    LOG_DBG("WDOGCONTROL: 0x%08x", wdt_em32_read(dev, WDT_WDOGCONTROL_OFFSET));
    LOG_DBG("WDOGRIS:     0x%08x", wdt_em32_read(dev, WDT_WDOGRIS_OFFSET));
    LOG_DBG("WDOGMIS:     0x%08x", wdt_em32_read(dev, WDT_WDOGMIS_OFFSET));
    LOG_DBG("WDOGLOCK:    0x%08x", wdt_em32_read(dev, WDT_WDOGLOCK_OFFSET));
}

static int wdt_em32_setup(const struct device *dev, uint8_t options)
{
    struct wdt_em32_data *data = dev->data;
    uint32_t load_value;
    uint32_t control = 0;

    if (!data->timeout_installed) {
        return -EINVAL;
    }

    /* Calculate load value based on timeout and clock frequency */
    /* Use hardware sample method: 32 * milliseconds for 32kHz clock */
    load_value = 32 * data->timeout_ms;  /* Simplified calculation for 32kHz */
    
    LOG_DBG("WDT Setup: timeout_ms=%u, load_value=0x%08x", data->timeout_ms, load_value);

    /* Ensure hardware clock gating is disabled */
    wdt_em32_disable_clkgate();

    /* Unlock watchdog for configuration */
    wdt_em32_unlock(dev);

    /* Verify unlock was successful */
    if (wdt_em32_is_locked(dev)) {
        LOG_ERR("WDT: Failed to unlock");
        return -EIO;
    }

    /* Disable watchdog first (like hardware sample) */
    wdt_em32_write(dev, WDT_WDOGCONTROL_OFFSET, 0);

    /* Set load value */
    wdt_em32_write(dev, WDT_WDOGLOAD_OFFSET, load_value);

    /* Configure control register */
    if (options & WDT_OPT_PAUSE_IN_SLEEP) {
        /* WDT continues in sleep mode by default - no specific bit to control this */
    }

    if (options & WDT_OPT_PAUSE_HALTED_BY_DBG) {
        /* Debug pause not supported in EM32 implementation */
    }

    /* Enable interrupt  */
    control |= WDT_WDOGCONTROL_INTEN;
    LOG_INF("WDT: Interrupt enabled");


    /* Enable reset - this is the primary function */
    control |= WDT_WDOGCONTROL_RESEN;
    LOG_INF("WDT: Reset enabled");

    /* Write control register */
    wdt_em32_write(dev, WDT_WDOGCONTROL_OFFSET, control);

    /* Debug: Read back registers */
    LOG_DBG("WDT: Setup complete, dumping registers:");
    wdt_em32_debug_registers(dev);

    /* Lock watchdog after configuration */
    wdt_em32_lock(dev);

    return 0;
}

static int wdt_em32_disable(const struct device *dev)
{
    struct wdt_em32_data *data = dev->data;

    /* Unlock watchdog */
    wdt_em32_unlock(dev);

    /* Verify unlock was successful */
    if (wdt_em32_is_locked(dev)) {
        return -EIO;
    }

    /* Disable watchdog by clearing control register */
    wdt_em32_write(dev, WDT_WDOGCONTROL_OFFSET, 0);

    /* Lock watchdog */
    wdt_em32_lock(dev);

    /* Clear software state so timeouts can be re-installed later */
    data->timeout_installed = false;
    data->callback = NULL;
    data->timeout_ms = 0;

    return 0;
}

static int wdt_em32_install_timeout(const struct device *dev,
                       const struct wdt_timeout_cfg *cfg)
{
    struct wdt_em32_data *data = dev->data;

    if (data->timeout_installed) {
        LOG_ERR("WDT: install_timeout failed - timeout already installed");
        return -ENOMEM;
    }

    /* Accept either explicit WDT_FLAG_RESET_SOC or 0 as shorthand */
    if (cfg->flags != 0 && cfg->flags != WDT_FLAG_RESET_SOC) {
        LOG_ERR("WDT: install_timeout failed - unsupported flags: 0x%x", cfg->flags);
        return -ENOTSUP;
    }

    if (cfg->window.min != 0) {
        LOG_ERR("WDT: install_timeout failed - window.min must be 0 (got %u)", cfg->window.min);
        return -EINVAL;
    }

    if (cfg->window.max == 0) {
        LOG_ERR("WDT: install_timeout failed - window.max must be non-zero");
        return -EINVAL;
    }

    data->callback = cfg->callback;
    data->timeout_ms = cfg->window.max;
    data->timeout_installed = true;

    /* Single-channel driver: return channel id 0 on success */
    return 0;
}

static int wdt_em32_feed(const struct device *dev, int channel_id)
{
    struct wdt_em32_data *data = dev->data;
    uint32_t load_value;

    ARG_UNUSED(channel_id);

    if (!data->timeout_installed) {
        return -EINVAL;
    }

    /* Calculate load value using hardware sample method */
    load_value = 32 * data->timeout_ms;  /* 32kHz clock */

    /* Unlock watchdog */
    wdt_em32_unlock(dev);

    /* Verify unlock was successful */
    if (wdt_em32_is_locked(dev)) {
        return -EIO;
    }

    /* Clear any pending interrupt first */
    wdt_em32_write(dev, WDT_WDOGINTCLR_OFFSET, 1);

    /* Reload the watchdog counter */
    wdt_em32_write(dev, WDT_WDOGLOAD_OFFSET, load_value);

    /* Lock watchdog */
    wdt_em32_lock(dev);

    return 0;
}

static void wdt_em32_isr(const struct device *dev)
{
    struct wdt_em32_data *data = dev->data;
    uint32_t int_status;

    /* Read interrupt status */
    int_status = wdt_em32_read(dev, WDT_WDOGRIS_OFFSET);

    if (int_status & WDT_WDOGRIS_RAWINT) {
        /* Clear interrupt by writing any value to WDOGINTCLR */
        wdt_em32_write(dev, WDT_WDOGINTCLR_OFFSET, 1);

        /* Call user callback if registered */
        if (data->callback) {
            data->callback(dev, 0);
        }
    }
}

static const struct wdt_driver_api wdt_em32_api = {
    .setup = wdt_em32_setup,
    .disable = wdt_em32_disable,
    .install_timeout = wdt_em32_install_timeout,
    .feed = wdt_em32_feed,
};

static int wdt_em32_init(const struct device *dev)
{
    const struct wdt_em32_config *config = dev->config;
    struct wdt_em32_data *data = dev->data;

    LOG_INF("WDT: Initializing at base 0x%08x", config->base);

    /* Disable hardware clock gating first */
    wdt_em32_disable_clkgate();

    /* Enable clock using the same pattern as TRNG driver */
    if (config->clock_dev) {
        if (!device_is_ready(config->clock_dev)) {
            LOG_ERR("WDT: Clock device not ready");
            return -ENODEV;
        }
        
        struct elan_em32_clock_control_subsys clk_subsys = {
            .clock_group = config->clock_group_id
        };
        
        int ret = clock_control_on(config->clock_dev, &clk_subsys);
        if (ret < 0) {
            LOG_ERR("WDT: Failed to enable clock: %d", ret);
            return ret;
        }
        LOG_INF("WDT: Clock enabled successfully (group %u)", config->clock_group_id);
    }

    /* Initialize data */
    data->timeout_installed = false;
    data->callback = NULL;

    /* Small delay to ensure clocks are stable */
    k_msleep(10);

    /* Initial register dump */
    LOG_DBG("WDT: Initial register state:");
    wdt_em32_debug_registers(dev);

    /* Disable watchdog initially */
    wdt_em32_disable(dev);

    LOG_INF("WDT: Initialization complete");
    return 0;
}

#define WDT_EM32_INIT(n)							\
    static void wdt_em32_irq_config_func_##n(const struct device *dev)	\
    {									\
        IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),		\
                wdt_em32_isr, DEVICE_DT_INST_GET(n), 0);	\
        irq_enable(DT_INST_IRQN(n));					\
    }									\
                                        \
    static const struct wdt_em32_config wdt_em32_config_##n = {	\
        .base = DT_INST_REG_ADDR(n),					\
        .clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),		\
        .clock_group_id = PCLKG_DWG, \
    };									\
                                        \
    static struct wdt_em32_data wdt_em32_data_##n;			\
                                        \
    DEVICE_DT_INST_DEFINE(n, wdt_em32_init, NULL,			\
                  &wdt_em32_data_##n,				\
                  &wdt_em32_config_##n,				\
                  POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	\
                  &wdt_em32_api);				\
                                        \
    static int wdt_em32_init_##n(void)					\
    {									\
        wdt_em32_irq_config_func_##n(DEVICE_DT_INST_GET(n));	\
        return 0;							\
    }									\
    SYS_INIT(wdt_em32_init_##n, POST_KERNEL,				\
         CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

DT_INST_FOREACH_STATUS_OKAY(WDT_EM32_INIT)
