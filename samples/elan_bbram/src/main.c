/*
 * Copyright (c) 2024 ELAN Microelectronics Corp.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief EM32 BBRAM Test Suite
 *
 * Comprehensive test application for the EM32 Battery-Backed RAM driver.
 * Tests all aspects of BBRAM functionality including basic operations,
 * power management, data integrity, and persistence across resets.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/bbram.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/crc.h>
#include <zephyr/random/random.h>
#include <errno.h>
#include <string.h>

/* Test patterns */
#define BBRAM_TEST_PATTERN_1    0xDEADBEEF
#define BBRAM_TEST_PATTERN_2    0xCAFEBABE
#define BBRAM_TEST_PATTERN_3    0x55AA55AA
#define BBRAM_MAGIC_HEADER      0xBBAAA967

/* Test data structures */
struct bbram_test_header {
    uint32_t magic;
    uint32_t test_counter;
    uint32_t checksum;
    uint32_t timestamp;
};

struct bbram_config_data {
    uint32_t boot_count;
    uint32_t error_count;
    uint32_t test_flags;
    uint32_t calibration_data[4];
};

struct bbram_event_log {
    uint32_t timestamp;
    uint16_t event_id;
    uint16_t data;
};

#define MAX_EVENT_LOG_ENTRIES   8
#define EVENT_LOG_START_OFFSET  (sizeof(struct bbram_test_header) + sizeof(struct bbram_config_data))

/* Global variables */
static const struct device *bbram_dev;
static size_t bbram_size;
static uint32_t test_run_count = 0;

/* Event IDs for logging */
#define EVENT_SYSTEM_BOOT       0x0001
#define EVENT_TEST_START        0x0002
#define EVENT_TEST_COMPLETE     0x0003
#define EVENT_POWER_FAILURE     0x0004
#define EVENT_DATA_CORRUPTION   0x0005

/**
 * @brief Print test banner
 */
static void print_banner(void)
{
    printk("\n");
    printk("=========================================\n");
    printk("   EM32 BBRAM Test Suite v1.0\n");
    printk("   Build: %s %s\n", __DATE__, __TIME__);
    printk("=========================================\n");
}

/**
 * @brief Log an event to BBRAM
 */
static int log_event(uint16_t event_id, uint16_t data)
{
    struct bbram_event_log entry;
    uint32_t log_index = 0;
    size_t offset;
    int ret;

    if (bbram_size < EVENT_LOG_START_OFFSET + sizeof(entry)) {
        return -ENOSPC;
    }

    /* Read current log index from last 4 bytes */
    ret = bbram_read(bbram_dev, bbram_size - 4, sizeof(log_index), (uint8_t *)&log_index);
    if (ret != 0) {
        log_index = 0;
    }

    /* Prepare log entry */
    entry.timestamp = k_uptime_get_32();
    entry.event_id = event_id;
    entry.data = data;

    /* Calculate offset for circular buffer */
    offset = EVENT_LOG_START_OFFSET + (log_index % MAX_EVENT_LOG_ENTRIES) * sizeof(entry);
    
    /* Write log entry */
    ret = bbram_write(bbram_dev, offset, sizeof(entry), (uint8_t *)&entry);
    if (ret != 0) {
        printk("Failed to write event log: %d\n", ret);
        return ret;
    }

    /* Update log index */
    log_index++;
    ret = bbram_write(bbram_dev, bbram_size - 4, sizeof(log_index), (uint8_t *)&log_index);
    if (ret != 0) {
        printk("Failed to update log index: %d\n", ret);
        return ret;
    }

    printk("Event logged: ID=0x%04X, data=0x%04X, index=%u\n", event_id, data, log_index);
    return 0;
}

/**
 * @brief Test: Basic read/write operations
 */
static int test_bbram_basic_rw(void)
{
    uint32_t write_data = BBRAM_TEST_PATTERN_1;
    uint32_t read_data = 0;
    uint8_t byte_data[4] = {0x11, 0x22, 0x33, 0x44};
    uint8_t read_bytes[4] = {0};
    int ret;
    
    printk("\n=== BBRAM Basic Read/Write Test ===\n");
    
    /* Test 32-bit word write/read */
    ret = bbram_write(bbram_dev, 0, sizeof(write_data), (uint8_t *)&write_data);
    if (ret != 0) {
        printk("BBRAM 32-bit write failed: %d\n", ret);
        return ret;
    }
    
    ret = bbram_read(bbram_dev, 0, sizeof(read_data), (uint8_t *)&read_data);
    if (ret != 0) {
        printk("BBRAM 32-bit read failed: %d\n", ret);
        return ret;
    }
    
    if (read_data != write_data) {
        printk("BBRAM 32-bit data mismatch: wrote 0x%08X, read 0x%08X\n",
               write_data, read_data);
        return -EIO;
    }
    printk("32-bit R/W test: PASSED (0x%08X)\n", read_data);

    /* Test byte-level write/read */
    ret = bbram_write(bbram_dev, 8, sizeof(byte_data), byte_data);
    if (ret != 0) {
        printk("BBRAM byte write failed: %d\n", ret);
        return ret;
    }

    ret = bbram_read(bbram_dev, 8, sizeof(read_bytes), read_bytes);
    if (ret != 0) {
        printk("BBRAM byte read failed: %d\n", ret);
        return ret;
    }

    if (memcmp(byte_data, read_bytes, sizeof(byte_data)) != 0) {
        printk("BBRAM byte data mismatch\n");
        return -EIO;
    }
    printk("Byte-level R/W test: PASSED\n");
    
    return 0;
}

/**
 * @brief Test: Size validation
 */
static int test_bbram_size(void)
{
    int ret;
    
    printk("\n=== BBRAM Size Validation Test ===\n");
    
    ret = bbram_get_size(bbram_dev, &bbram_size);
    if (ret != 0) {
        printk("BBRAM get_size failed: %d\n", ret);
        return ret;
    }
    
    printk("BBRAM size: %zu bytes\n", bbram_size);
    
    /* EM32 has 64 bytes total, 4 bytes reserved for status = 60 bytes usable */
    if (bbram_size != 60) {
        printk("Unexpected BBRAM size: expected 60, got %zu\n", bbram_size);
        return -EINVAL;
    }
    
    printk("Size validation: PASSED\n");
    return 0;
}

/**
 * @brief Test: Power domain status checking
 */
static int test_bbram_power_status(void)
{
    int ret;
    
    printk("\n=== BBRAM Power Status Test ===\n");
    
    /* Check data validity */
    ret = bbram_check_invalid(bbram_dev);
    printk("BBRAM data validity: %s\n", (ret == 0) ? "VALID" : "INVALID");
    if (ret != 0) {
        log_event(EVENT_DATA_CORRUPTION, ret);
    }
    
    /* Check standby power */
    ret = bbram_check_standby_power(bbram_dev);
    printk("BBRAM standby power: %s\n", (ret == 0) ? "NORMAL" : "FAILURE");
    if (ret != 0) {
        log_event(EVENT_POWER_FAILURE, 0x0001);
    }
    
    /* Check main power */
    ret = bbram_check_power(bbram_dev);
    printk("BBRAM main power: %s\n", (ret == 0) ? "NORMAL" : "FAILURE");
    if (ret != 0) {
        log_event(EVENT_POWER_FAILURE, 0x0002);
    }
    
    printk("Power status check: COMPLETED\n");
    return 0;
}

/**
 * @brief Test: Data persistence across resets
 */
static int test_bbram_persistence(void)
{
    struct bbram_test_header header;
    struct bbram_config_data config;
    uint32_t expected_checksum;
    int ret;
    
    printk("\n=== BBRAM Data Persistence Test ===\n");
    
    /* Read existing header */
    ret = bbram_read(bbram_dev, 0, sizeof(header), (uint8_t *)&header);
    if (ret == 0 && header.magic == BBRAM_MAGIC_HEADER) {
        /* Verify checksum */
        expected_checksum = crc32_ieee((uint8_t *)&header, sizeof(header) - sizeof(header.checksum));
        if (expected_checksum == header.checksum) {
            test_run_count = header.test_counter + 1;
            printk("Previous boot detected, counter: %u\n", header.test_counter);
            printk("Last boot timestamp: %u ms\n", header.timestamp);
        } else {
            printk("Header checksum mismatch: expected 0x%08X, got 0x%08X\n",
                   expected_checksum, header.checksum);
            test_run_count = 1;
        }
    } else {
        printk("First boot or invalid data, initializing counter\n");
        test_run_count = 1;
        
        /* Initialize config data */
        memset(&config, 0, sizeof(config));
        config.boot_count = 1;
        config.test_flags = 0x12345678;
        config.calibration_data[0] = 0x1000;
        config.calibration_data[1] = 0x2000;
        config.calibration_data[2] = 0x3000;
        config.calibration_data[3] = 0x4000;
        
        ret = bbram_write(bbram_dev, sizeof(header), sizeof(config), (uint8_t *)&config);
        if (ret != 0) {
            printk("Failed to initialize config data: %d\n", ret);
            return ret;
        }
    }
    
    /* Prepare new header */
    header.magic = BBRAM_MAGIC_HEADER;
    header.test_counter = test_run_count;
    header.timestamp = k_uptime_get_32();
    header.checksum = crc32_ieee((uint8_t *)&header, sizeof(header) - sizeof(header.checksum));
    
    /* Write updated header */
    ret = bbram_write(bbram_dev, 0, sizeof(header), (uint8_t *)&header);
    if (ret != 0) {
        printk("BBRAM header write failed: %d\n", ret);
        return ret;
    }
    
    /* Update boot count in config */
    ret = bbram_read(bbram_dev, sizeof(header), sizeof(config), (uint8_t *)&config);
    if (ret == 0) {
        config.boot_count++;
        ret = bbram_write(bbram_dev, sizeof(header), sizeof(config), (uint8_t *)&config);
        if (ret == 0) {
            printk("Boot count updated to: %u\n", config.boot_count);
        }
    }
    
    printk("Test run count updated to: %u\n", test_run_count);
    log_event(EVENT_TEST_START, test_run_count);
    
    return 0;
}

/**
 * @brief Test: Boundary conditions
 */
static int test_bbram_boundary(void)
{
    uint8_t test_byte = 0x55;
    uint8_t read_byte;
    int ret;
    
    printk("\n=== BBRAM Boundary Condition Test ===\n");
    
    /* Test write at last valid address */
    ret = bbram_write(bbram_dev, bbram_size - 1, 1, &test_byte);
    if (ret != 0) {
        printk("BBRAM boundary write failed: %d\n", ret);
        return ret;
    }
    
    /* Test read at last valid address */
    ret = bbram_read(bbram_dev, bbram_size - 1, 1, &read_byte);
    if (ret != 0) {
        printk("BBRAM boundary read failed: %d\n", ret);
        return ret;
    }
    
    if (read_byte != test_byte) {
        printk("BBRAM boundary data mismatch: wrote 0x%02X, read 0x%02X\n",
               test_byte, read_byte);
        return -EIO;
    }
    printk("Boundary write/read: PASSED\n");
    
    /* Test invalid address (should fail) */
    ret = bbram_write(bbram_dev, bbram_size, 1, &test_byte);
    if (ret == 0) {
        printk("BBRAM should have rejected out-of-bounds write\n");
        return -EIO;
    }
    printk("Out-of-bounds write rejection: PASSED\n");
    
    /* Test invalid read (should fail) */
    ret = bbram_read(bbram_dev, bbram_size, 1, &read_byte);
    if (ret == 0) {
        printk("BBRAM should have rejected out-of-bounds read\n");
        return -EIO;
    }
    printk("Out-of-bounds read rejection: PASSED\n");
    
    return 0;
}

/**
 * @brief Test: Data integrity with patterns
 */
static int test_bbram_data_integrity(void)
{
    uint32_t patterns[] = {
        BBRAM_TEST_PATTERN_1,
        BBRAM_TEST_PATTERN_2,
        BBRAM_TEST_PATTERN_3,
        0x00000000,
        0xFFFFFFFF,
        0x55555555,
        0xAAAAAAAA
    };
    uint32_t read_value;
    size_t offset = 16; /* Start after header */
    int ret;
    
    printk("\n=== BBRAM Data Integrity Test ===\n");
    
    for (int i = 0; i < ARRAY_SIZE(patterns); i++) {
        /* Write pattern */
        ret = bbram_write(bbram_dev, offset, sizeof(patterns[i]), (uint8_t *)&patterns[i]);
        if (ret != 0) {
            printk("Pattern %d write failed: %d\n", i, ret);
            return ret;
        }
        
        /* Read back */
        ret = bbram_read(bbram_dev, offset, sizeof(read_value), (uint8_t *)&read_value);
        if (ret != 0) {
            printk("Pattern %d read failed: %d\n", i, ret);
            return ret;
        }
        
        /* Verify */
        if (read_value != patterns[i]) {
            printk("Pattern %d mismatch: wrote 0x%08X, read 0x%08X\n",
                   i, patterns[i], read_value);
            return -EIO;
        }
        
        printk("Pattern %d (0x%08X): PASSED\n", i, patterns[i]);
        offset += sizeof(uint32_t);
        
        if (offset + sizeof(uint32_t) >= bbram_size) {
            break; /* Avoid overflow */
        }
    }
    
    return 0;
}

/**
 * @brief Test: Random data stress test
 */
static int test_bbram_stress(void)
{
    uint8_t write_buffer[32];
    uint8_t read_buffer[32];
    size_t offset = 20; /* Avoid header area */
    int ret;
    
    printk("\n=== BBRAM Stress Test ===\n");
    
    if (offset + sizeof(write_buffer) >= bbram_size) {
        printk("Not enough space for stress test\n");
        return -ENOSPC;
    }
    
    /* Generate random data */
    for (int i = 0; i < sizeof(write_buffer); i++) {
        write_buffer[i] = sys_rand32_get() & 0xFF;
    }
    
    /* Multiple write/read cycles */
    for (int cycle = 0; cycle < 10; cycle++) {
        /* Write random data */
        ret = bbram_write(bbram_dev, offset, sizeof(write_buffer), write_buffer);
        if (ret != 0) {
            printk("Stress test cycle %d write failed: %d\n", cycle, ret);
            return ret;
        }
        
        /* Read back */
        ret = bbram_read(bbram_dev, offset, sizeof(read_buffer), read_buffer);
        if (ret != 0) {
            printk("Stress test cycle %d read failed: %d\n", cycle, ret);
            return ret;
        }
        
        /* Verify */
        if (memcmp(write_buffer, read_buffer, sizeof(write_buffer)) != 0) {
            printk("Stress test cycle %d data mismatch\n", cycle);
            return -EIO;
        }
        
        /* Generate new random data for next cycle */
        for (int i = 0; i < sizeof(write_buffer); i++) {
            write_buffer[i] = sys_rand32_get() & 0xFF;
        }
        
        printk("Stress test cycle %d: PASSED\n", cycle);
        k_msleep(10); /* Small delay between cycles */
    }
    
    return 0;
}

/**
 * @brief Display event log
 */
static void display_event_log(void)
{
    struct bbram_event_log entry;
    uint32_t log_index = 0;
    size_t offset;
    int ret;
    
    printk("\n=== BBRAM Event Log ===\n");
    
    /* Read log index */
    ret = bbram_read(bbram_dev, bbram_size - 4, sizeof(log_index), (uint8_t *)&log_index);
    if (ret != 0 || log_index == 0) {
        printk("No events logged\n");
        return;
    }
    
    printk("Total events logged: %u\n", log_index);
    
    /* Display last few events */
    uint32_t start_idx = (log_index > MAX_EVENT_LOG_ENTRIES) ? 
                         log_index - MAX_EVENT_LOG_ENTRIES : 0;
    
    for (uint32_t i = start_idx; i < log_index; i++) {
        offset = EVENT_LOG_START_OFFSET + (i % MAX_EVENT_LOG_ENTRIES) * sizeof(entry);
        
        ret = bbram_read(bbram_dev, offset, sizeof(entry), (uint8_t *)&entry);
        if (ret == 0) {
            printk("Event #%u: ID=0x%04X, data=0x%04X, time=%u ms\n",
                   i + 1, entry.event_id, entry.data, entry.timestamp);
        }
    }
}

/**
 * @brief Main test execution
 */
int main(void)
{
    int ret;
    
    k_msleep(100);  
    print_banner();
    
    /* Get BBRAM device */
    bbram_dev = DEVICE_DT_GET(DT_NODELABEL(bbram0));
    if (!device_is_ready(bbram_dev)) {
        printk("ERROR: BBRAM device not ready\n");
        return -ENODEV;
    }
    
    k_msleep(100);  
    printk("BBRAM device ready: %s\n", bbram_dev->name);
    log_event(EVENT_SYSTEM_BOOT, 0);
    
    k_msleep(100);  
    /* Run test suite */
    printk("\nStarting BBRAM test suite...\n");
    
    ret = test_bbram_size();
    if (ret != 0) {
        printk("Size test FAILED: %d\n", ret);
        return ret;
    }
    
    k_msleep(10);  
    ret = test_bbram_power_status();
    if (ret != 0) {
        printk("Power status test FAILED: %d\n", ret);
        return ret;
    }
    
    k_msleep(10);  
    ret = test_bbram_persistence();
    if (ret != 0) {
        printk("Persistence test FAILED: %d\n", ret);
        return ret;
    }
    
    k_msleep(10);  
    ret = test_bbram_basic_rw();
    if (ret != 0) {
        printk("Basic R/W test FAILED: %d\n", ret);
        return ret;
    }
    k_msleep(10);  
    ret = test_bbram_boundary();
    if (ret != 0) {
        printk("Boundary test FAILED: %d\n", ret);
        return ret;
    }
    k_msleep(10);  
    ret = test_bbram_data_integrity();
    if (ret != 0) {
        printk("Data integrity test FAILED: %d\n", ret);
        return ret;
    }
    
    k_msleep(10);  
    ret = test_bbram_stress();
    if (ret != 0) {
        printk("Stress test FAILED: %d\n", ret);
        return ret;
    }
    
    k_msleep(10);  
    log_event(EVENT_TEST_COMPLETE, test_run_count);
    
    printk("\n=== All BBRAM Tests COMPLETED SUCCESSFULLY ===\n");
    printk("Test run #%u completed at %u ms uptime\n", test_run_count, k_uptime_get_32());
    
    display_event_log();
    
    /* Continuous monitoring loop */
    printk("\nEntering monitoring mode...\n");
    while (1) {
        k_msleep(1000); /* 10 second intervals */
        
        /* Periodic power status check */
        if (bbram_check_invalid(bbram_dev) != 0) {
            printk("WARNING: BBRAM data became invalid!\n");
            log_event(EVENT_DATA_CORRUPTION, k_uptime_get_32());
        }
        
        printk("BBRAM monitoring... uptime: %u ms, run: #%u\n", 
               k_uptime_get_32(), test_run_count);
    }
    
    return 0;
}
