/*
 * Copyright (c) 2024 ELAN Microelectronics Corp.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief EM32 Battery-Backed RAM (BBRAM) driver
 *
 * This driver provides BBRAM functionality for the EM32 microcontroller family
 * using the 512-bit (64-byte) backup registers. The implementation follows
 * EM32-style backup domain patterns while adapting to EM32 hardware.
 *
 * Key Features:
 * - 64 bytes of battery-backed storage (16 × 32-bit registers)
 * - Power domain management and write protection handling
 * - Data integrity checking and power failure detection
 * - Zephyr BBRAM API compliance
 */

#define DT_DRV_COMPAT elan_em32_bbram

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/bbram.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <soc.h>

LOG_MODULE_REGISTER(bbram_em32, CONFIG_BBRAM_LOG_LEVEL);

/* EM32 Backup Register Layout */
#define EM32_BACKUP_BASE     0x40033000
#define EM32_BACKUP_SIZE     64         /* 64 bytes */
#define EM32_BACKUP_REG_COUNT 16        /* 16 × 32-bit registers */

/* EM32 Clock Gating Control */
#define EM32_CLKGATE_REG     0x40030100  /* Clock Gating Control Register */
#define HCLKG_BKP                24          /* BKP Clock Enable (bit 24) */

/* Power Management Registers - EM32-style */
#define EM32_PWR_BASE        0x40031000
#define PWR_CR_OFFSET            0x00       /* em32 PD_REG */
#define PWR_CSR_OFFSET           0x04       /* em32 WAKEUP_REG*/

/* Power Control Register bits */
#define PWR_CR_DBP_BIT           BIT(8)     /* Disable Backup Protection */
#define PWR_CR_PVDE_BIT          BIT(4)     /* Power Voltage Detector Enable */

/* Power Control Status Register bits */
#define PWR_CSR_BRE_BIT          BIT(9)     /* Backup Regulator Enable */
#define PWR_CSR_BRR_BIT          BIT(3)     /* Backup Regulator Ready */
#define PWR_CSR_EWUP_BIT         BIT(8)     /* Enable WKUP pin */

/* RTC Backup Domain Control Register */
#define EM32_RCC_BASE        0x40030000
#define RCC_BDCR_OFFSET          0x20
#define RCC_BDCR_BDRST_BIT       BIT(16)    /* Backup Domain Reset */
#define RCC_BDCR_RTCEN_BIT       BIT(15)    /* RTC Enable */

/* BBRAM status flags (stored in first register) */
#define BBRAM_STATUS_MAGIC       0xBBCC9967 /* Magic number for validity */
#define BBRAM_STATUS_VALID_BIT   BIT(0)     /* Data valid flag */
#define BBRAM_STATUS_POWER_BIT   BIT(1)     /* Power failure flag */
#define BBRAM_STATUS_STANDBY_BIT BIT(2)     /* Standby power failure */

/* Register access macros */
#define REG32(addr) (*((volatile uint32_t *)(addr)))

/**
 * @brief Enable BBRAM (BKP) clock using EM32 clock gating
 */
static int em32_bbram_enable_clock(void)
{
    uint32_t clkgate_reg;

    LOG_DBG("Enabling BKP clock (bit %d)", HCLKG_BKP);

    /* Read current clock gating register */
    clkgate_reg = REG32(EM32_CLKGATE_REG);

    /* Clear the BKP clock gating bit to enable the clock */
    clkgate_reg &= ~(1UL << HCLKG_BKP);
    REG32(EM32_CLKGATE_REG) = clkgate_reg;

    /* Verify clock is enabled */
    clkgate_reg = REG32(EM32_CLKGATE_REG);
    if (clkgate_reg & (1UL << HCLKG_BKP)) {
        LOG_ERR("Failed to enable BKP clock - bit still set");
        return -EIO;
    }

    LOG_DBG("BKP clock enabled successfully");
    return 0;
}

/* BBRAM configuration structure */
struct em32_bbram_config {
    uint32_t base_addr;
    uint32_t size;
    uint32_t reg_count;
    const struct device *clock_dev;
};

/* BBRAM runtime data */
struct em32_bbram_data {
    bool backup_domain_enabled;
    bool write_protection_disabled;
    uint32_t status_flags;
};

/**
 * @brief Read from backup register
 */
static inline uint32_t bbram_read_reg(uint32_t base, uint32_t offset)
{
    return REG32(base + offset);
}

/**
 * @brief Write to backup register  
 */
static inline void bbram_write_reg(uint32_t base, uint32_t offset, uint32_t value)
{
    REG32(base + offset) = value;
}

/**
 * @brief Enable backup domain write access (EM32-style)
 */
static int em32_bbram_enable_write_access(const struct device *dev)
{
    struct em32_bbram_data *data = dev->data;
    
    if (data->write_protection_disabled) {
        return 0; /* Already enabled */
    }

    /* EM32 specific: Check if backup registers are directly accessible */
    /* EM32 may not require backup domain write protection */
    
    /* Test write access by trying to read/write to a backup register */
    uint32_t test_addr = EM32_BACKUP_BASE + (15 * 4); /* Use last register for test */
    uint32_t original_value = REG32(test_addr);
    uint32_t test_value = 0x5A5A5A5A;
    
    LOG_DBG("Testing backup register access at 0x%08X", test_addr);
    LOG_DBG("Original value: 0x%08X", original_value);
    
    /* Try to write test value */
    REG32(test_addr) = test_value;
    uint32_t read_back = REG32(test_addr);
    
    LOG_DBG("After write test value 0x%08X, read back: 0x%08X", test_value, read_back);
    
    /* Restore original value */
    REG32(test_addr) = original_value;
    
    if (read_back == test_value) {
        /* Write access is working */
        data->write_protection_disabled = true;
        LOG_DBG("Backup domain write access confirmed (no protection needed)");
        return 0;
    } else {
        /* Try enabling power domain control if needed */
        volatile uint32_t *pwr_cr = (uint32_t *)(EM32_PWR_BASE + PWR_CR_OFFSET);
        uint32_t initial_value, current_value;
        int timeout = 1000;

        initial_value = *pwr_cr;
        LOG_DBG("Backup write failed, trying PWR_CR. Initial value: 0x%08X", initial_value);

        /* Enable backup domain write access */
        *pwr_cr |= PWR_CR_DBP_BIT;
        
        /* Read back to confirm write */
        current_value = *pwr_cr;
        LOG_DBG("PWR_CR after write: 0x%08X (bit %d should be set)", current_value, 8);

        /* Wait for write protection to be disabled */
        while (timeout-- > 0) {
            current_value = *pwr_cr;
            if (current_value & PWR_CR_DBP_BIT) {
                data->write_protection_disabled = true;
                LOG_DBG("Backup domain write access enabled after %d us", 1000-timeout);
                return 0;
            }
            k_busy_wait(1);
        }

        LOG_ERR("Failed to enable backup domain write access (final PWR_CR: 0x%08X)", current_value);
        return -ETIMEDOUT;
    }
}

/**
 * @brief Disable backup domain write access (for security)
 */
static void em32_bbram_disable_write_access(const struct device *dev)
{
    struct em32_bbram_data *data = dev->data;
    volatile uint32_t *pwr_cr = (uint32_t *)(EM32_PWR_BASE + PWR_CR_OFFSET);

    if (!data->write_protection_disabled) {
        return; /* Already disabled */
    }

    /* Disable backup domain write access */
    *pwr_cr &= ~PWR_CR_DBP_BIT;
    data->write_protection_disabled = false;

    LOG_DBG("Backup domain write access disabled");
}

/**
 * @brief Check if backup domain reset occurred
 */
static bool em32_bbram_check_reset_flag(void)
{
    volatile uint32_t *rcc_bdcr = (uint32_t *)(EM32_RCC_BASE + RCC_BDCR_OFFSET);
    uint32_t bdcr_val = *rcc_bdcr;

    if (bdcr_val & RCC_BDCR_BDRST_BIT) {
        /* Clear reset flag */
        *rcc_bdcr = bdcr_val & ~RCC_BDCR_BDRST_BIT;
        LOG_INF("Backup domain reset detected and cleared");
        return true;
    }

    return false;
}

/**
 * @brief Initialize backup domain status
 */
static int em32_bbram_init_status(const struct device *dev)
{
    const struct em32_bbram_config *config = dev->config;
    struct em32_bbram_data *data = dev->data;
    uint32_t status_reg;
    bool domain_reset;

    /* Check for backup domain reset */
    domain_reset = em32_bbram_check_reset_flag();

    /* Enable write access for status initialization */
    int ret = em32_bbram_enable_write_access(dev);
    if (ret != 0) {
        return ret;
    }

    /* Read current status from first backup register */
    status_reg = bbram_read_reg(config->base_addr, 0);

    if (domain_reset || (status_reg >> 16) != (BBRAM_STATUS_MAGIC >> 16)) {
        /* Initialize status register */
        LOG_INF("Initializing BBRAM status (reset=%d, magic=0x%08X)", 
                domain_reset, status_reg);
        
        data->status_flags = BBRAM_STATUS_VALID_BIT;
        if (domain_reset) {
            data->status_flags |= BBRAM_STATUS_POWER_BIT;
        }
        
        status_reg = BBRAM_STATUS_MAGIC | data->status_flags;
        bbram_write_reg(config->base_addr, 0, status_reg);
    } else {
        /* Extract status flags */
        data->status_flags = status_reg & 0xFFFF;
        LOG_DBG("BBRAM status restored: 0x%04X", data->status_flags);
    }

    return 0;
}

/**
 * @brief Update status flags in backup register
 */
static void em32_bbram_update_status(const struct device *dev)
{
    const struct em32_bbram_config *config = dev->config;
    struct em32_bbram_data *data = dev->data;
    uint32_t status_reg;

    if (!data->write_protection_disabled) {
        return; /* Cannot write without write access */
    }

    status_reg = BBRAM_STATUS_MAGIC | data->status_flags;
    bbram_write_reg(config->base_addr, 0, status_reg);
}

/**
 * @brief Check if BBRAM data is invalid
 */
static int em32_bbram_check_invalid(const struct device *dev)
{
    struct em32_bbram_data *data = dev->data;

    if (!(data->status_flags & BBRAM_STATUS_VALID_BIT)) {
        /* Clear invalid flag after reading */
        data->status_flags |= BBRAM_STATUS_VALID_BIT;
        em32_bbram_update_status(dev);
        return -EFAULT;
    }

    return 0;
}

/**
 * @brief Check for standby power failure
 */
static int em32_bbram_check_standby_power(const struct device *dev)
{
    struct em32_bbram_data *data = dev->data;

    if (data->status_flags & BBRAM_STATUS_STANDBY_BIT) {
        /* Clear standby failure flag after reading */
        data->status_flags &= ~BBRAM_STATUS_STANDBY_BIT;
        em32_bbram_update_status(dev);
        return -EFAULT;
    }

    return 0;
}

/**
 * @brief Check for VCC power failure
 */
static int em32_bbram_check_power(const struct device *dev)
{
    struct em32_bbram_data *data = dev->data;

    if (data->status_flags & BBRAM_STATUS_POWER_BIT) {
        /* Clear power failure flag after reading */
        data->status_flags &= ~BBRAM_STATUS_POWER_BIT;
        em32_bbram_update_status(dev);
        return -EFAULT;
    }

    return 0;
}

/**
 * @brief Get BBRAM size
 */
static int em32_bbram_get_size(const struct device *dev, size_t *size)
{
    const struct em32_bbram_config *config = dev->config;

    /* Reserve first 4 bytes for status, return usable size */
    *size = config->size - 4;
    return 0;
}

/**
 * @brief Read data from BBRAM
 */
static int em32_bbram_read(const struct device *dev, size_t offset, 
                              size_t size, uint8_t *data)
{
    const struct em32_bbram_config *config = dev->config;
    size_t usable_size = config->size - 4; /* Reserve 4 bytes for status */
    size_t i;

    if (!data) {
        return -EINVAL;
    }

    if (offset + size > usable_size) {
        LOG_ERR("Read beyond BBRAM bounds: offset=%zu, size=%zu, max=%zu",
                offset, size, usable_size);
        return -EFAULT;
    }

    LOG_DBG("BBRAM read: offset=%zu, size=%zu", offset, size);

    /* Read data byte by byte, starting from offset 4 (after status) */
    for (i = 0; i < size; i++) {
        uint32_t reg_offset = ((offset + i + 4) / 4) * 4;
        uint32_t byte_offset = (offset + i + 4) % 4;
        uint32_t reg_value = bbram_read_reg(config->base_addr, reg_offset);
        
        data[i] = (reg_value >> (byte_offset * 8)) & 0xFF;
    }

    return 0;
}

/**
 * @brief Write data to BBRAM
 */
static int em32_bbram_write(const struct device *dev, size_t offset,
                               size_t size, const uint8_t *data)
{
    const struct em32_bbram_config *config = dev->config;
    size_t usable_size = config->size - 4; /* Reserve 4 bytes for status */
    size_t i;
    int ret;

    if (!data) {
        return -EINVAL;
    }

    if (offset + size > usable_size) {
        LOG_ERR("Write beyond BBRAM bounds: offset=%zu, size=%zu, max=%zu",
                offset, size, usable_size);
        return -EFAULT;
    }

    LOG_DBG("BBRAM write: offset=%zu, size=%zu", offset, size);

    /* Enable write access */
    ret = em32_bbram_enable_write_access(dev);
    if (ret != 0) {
        return ret;
    }

    /* Write data byte by byte, starting from offset 4 (after status) */
    for (i = 0; i < size; i++) {
        uint32_t reg_offset = ((offset + i + 4) / 4) * 4;
        uint32_t byte_offset = (offset + i + 4) % 4;
        uint32_t reg_value = bbram_read_reg(config->base_addr, reg_offset);
        
        /* Update the specific byte */
        reg_value &= ~(0xFF << (byte_offset * 8));
        reg_value |= (data[i] << (byte_offset * 8));
        
        bbram_write_reg(config->base_addr, reg_offset, reg_value);
    }

    /* Disable write access for security */
    em32_bbram_disable_write_access(dev);

    return 0;
}

/* BBRAM driver API */
static const struct bbram_driver_api em32_bbram_api = {
    .check_invalid = em32_bbram_check_invalid,
    .check_standby_power = em32_bbram_check_standby_power,
    .check_power = em32_bbram_check_power,
    .get_size = em32_bbram_get_size,
    .read = em32_bbram_read,
    .write = em32_bbram_write,
};

/**
 * @brief Initialize BBRAM device
 */
static int em32_bbram_init(const struct device *dev)
{
    const struct em32_bbram_config *config = dev->config;
    struct em32_bbram_data *data = dev->data;
    int ret;

    LOG_INF("Initializing EM32 BBRAM at 0x%08X, size=%u bytes",
            config->base_addr, config->size);

    /* Initialize runtime data */
    data->backup_domain_enabled = false;
    data->write_protection_disabled = false;
    data->status_flags = 0;

    /* Enable BBRAM hardware clock first */
    ret = em32_bbram_enable_clock();
    if (ret != 0) {
        LOG_ERR("Failed to enable BBRAM hardware clock: %d", ret);
        return ret;
    }

    /* Note: BKP hardware clock is already enabled above.
     * EM32 BBRAM uses direct hardware clock gating rather than 
     * the generic APB clock control framework. */

    /* Initialize backup domain status */
    ret = em32_bbram_init_status(dev);
    if (ret != 0) {
        LOG_ERR("Failed to initialize BBRAM status: %d", ret);
        return ret;
    }

    LOG_INF("EM32 BBRAM initialized successfully");
    return 0;
}

/* Device instantiation macro */
#define EM32_BBRAM_INIT(n)                                                \
    static const struct em32_bbram_config em32_bbram_config_##n = {  \
        .base_addr = DT_INST_REG_ADDR(n),                                     \
        .size = DT_INST_REG_SIZE(n),                                          \
        .reg_count = DT_INST_PROP(n, backup_regs_count),                     \
        .clock_dev = DEVICE_DT_GET_OR_NULL(DT_INST_CLOCKS_CTLR(n)),          \
    };                                                                        \
                                                                              \
    static struct em32_bbram_data em32_bbram_data_##n;               \
                                                                              \
    DEVICE_DT_INST_DEFINE(n, em32_bbram_init, NULL,                      \
                          &em32_bbram_data_##n,                           \
                          &em32_bbram_config_##n,                         \
                          POST_KERNEL, CONFIG_BBRAM_INIT_PRIORITY,            \
                          &em32_bbram_api);

DT_INST_FOREACH_STATUS_OKAY(EM32_BBRAM_INIT)
