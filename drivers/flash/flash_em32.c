/*
 * Copyright (c) 2025 Elan Microelectronics Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Flash controller driver for Elan eM32 microcontroller
 *
 * This driver provides Flash memory operations including:
 * - Page erase and chip erase
 * - Single word and burst write operations
 * - Dual key protection system
 * - LVD (Low Voltage Detection) control
 * - Cache control for performance optimization
 */

#define DT_DRV_COMPAT elan_em32_flash_controller

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/flash.h>
#include <errno.h>
#include <elan_em32.h> // Part of soc.h, only

//#include <flash_em32.h>
#include "../../include/zephyr/drivers/flash/flash_em32.h"

//#include <clock_control_em32_ahb.h>
#include "../../include/zephyr/drivers/clock_control/clock_control_em32_ahb.h" // delay_10us() & delay_100us()

/* Log configuration */
#include <zephyr/logging/log.h>
//LOG_MODULE_REGISTER(flash_em32, LOG_LEVEL_DBG);
LOG_MODULE_REGISTER(flash_em32, CONFIG_FLASH_LOG_LEVEL);

/* Driver data structure */
struct flash_em32_dev_data {
    struct k_sem mutex;
};

static struct flash_em32_dev_data flash_em32_data;

/* Static flash parameters */
static const struct flash_parameters flash_em32_parameters = {
    .write_block_size = EM32_NV_FLASH_WRITE_BLOCK_SIZE,
    .erase_value = 0xff,
};

/* Flash Mode Enumeration */
typedef enum
{ 
    USERMODE    = 0,
    PAGEERASE   = 1
} FlashMode;

/**
 * @brief Validate range of flash read
 */
bool flash_em32_valid_read_range(off_t offset, size_t len)
{
    bool ret = true;

    // Base Address
    if ((offset < 0) || (offset >= EM32_NV_FLASH_SIZE))
    {
        LOG_ERR("offset (0x%lx) not in a valid range.", offset);
        ret = false;
    }

    // Data Length
    if ((len == 0) || (len > EM32_NV_FLASH_SIZE))
    {
        LOG_ERR("len (0x%x) not a valid length.", len);
        ret = false;
    }

    return ret;
}

/**
 * @brief Read data from Flash
 */
static int flash_em32_read(const struct device *dev, off_t offset, void *data, size_t len)
{
    struct flash_em32_dev_data *dev_data = dev->data;
    uint32_t address_base = EM32_NV_FLASH_ADDR, \
             address = address_base + offset;
    int ret = 0;

    // Validate Range
    if (flash_em32_valid_read_range(offset, len) == false) 
    {
        ret = -EINVAL;
        goto FLASH_EM32_READ_EXIT;
    }
    LOG_DBG("Read flash from address 0x%x, length %d.", address, len);

    k_sem_take(&dev_data->mutex, K_FOREVER);

    memcpy(data, (uint8_t *)address, len);

    k_sem_give(&dev_data->mutex);

FLASH_EM32_READ_EXIT:
    return ret;
}

/**
 * @brief Validate range of flash erase
 */
bool flash_em32_valid_erase_range(off_t offset, size_t len)
{
    bool ret = true;

    if (flash_em32_valid_read_range(offset, len) == false) 
    {
        ret = false;
        goto FLASH_EM32_VALID_ERASE_RANGE_EXIT;
    }
	
    if ((offset % EM32_NV_FLASH_PAGE_SIZE) != 0) 
    {
        LOG_ERR("offset (0x%lx) not on a page boundary.", offset);
        ret = false;
        goto FLASH_EM32_VALID_ERASE_RANGE_EXIT;
    }

    if ((len % EM32_NV_FLASH_PAGE_SIZE) != 0) 
    {
        LOG_ERR("len (%zu) not multiple of page size (%d).", len, EM32_NV_FLASH_PAGE_SIZE);
        ret = false;
        goto FLASH_EM32_VALID_ERASE_RANGE_EXIT;
    }

FLASH_EM32_VALID_ERASE_RANGE_EXIT:
    return ret;
}

/**
 * @brief Set Flash Key
 */
void flash_em32_set_flash_key( FlashMode flash_mode, uint32_t pre_key1, uint32_t pre_key2 )
{
    const uint32_t key_array[8] = { 0xC433,   0x4E4D,  0xC04D,   0x8481,
                                    0x1bdf4, 0x213aee, 0x15189,  0x4ade7};
    __IO uint32_t key1 = 0, \
                  key2 = 0;

    LOG_DBG("flash_mode=%d, pre_key1=0x%x, pre_key2=0x%x.", flash_mode, pre_key1, pre_key2);

    // Flash Key 1
    switch( flash_mode )
    {
        case USERMODE:  
            key1 = key_array[0] * pre_key1 * pre_key2 + key_array[1]; 
            break;

        case PAGEERASE: 
            key1 = key_array[4] * pre_key1 * pre_key2 + key_array[5]; 
            break;
				
        default: 
            break;
    }
    FLASHKEY1 = key1;
    delay_10us();
	
    // Flash Key 2
    switch( flash_mode )
    {
        case USERMODE:
            key2 = key_array[2] * pre_key1 * pre_key2 + key_array[3];
            break;

        case PAGEERASE:
            key2 = key_array[6] * pre_key1 * pre_key2 + key_array[7]; 
            break;

        default:
            break;
    }
    FLASHKEY2 = key2;
    delay_10us();
	
    LOG_DBG("key1=0x%x, key2=0x%x.", key1, key2);

    return;
}

/**
 * @brief Clear Flash Key
 */
void flash_em32_clear_flash_key(void)
{
    LOG_DBG("Clear flash key.");
    FLASHKEY1 = 0;
    FLASHKEY2 = 0;
    delay_10us();
    return;
}

/**
 * @brief Erase Flash Page
 */
void flash_em32_erase_page(__IO uint32_t page_index)
{
    LOG_DBG("Erase page 0x%x.", page_index);

    FLASH_SR0 = 0; 
    FLASHSR0_Ctrl->PageAddr = page_index;
    FLASHSR0_Ctrl->EraseEnable = 1;

    while( FLASHSR0_Ctrl->Flash_Idle == 0 )
        ;

    return;
}

/**
 * @brief Erase Flash Region
 */
static int flash_em32_erase(const struct device *dev, off_t offset, size_t len)
{
    struct flash_em32_dev_data *dev_data = dev->data;
    int ret = 0;
    uint32_t pre_key1 = 0xea, \
             pre_key2 = 0x93, \
             start_mem_addr = offset, \
             end_mem_addr = (offset + len), \
             start_page = 0, \
             end_page = 0, \
             page_index = 0;

    // Validate Range
    if (flash_em32_valid_erase_range(offset, len) == false) 
    {
        ret = -EINVAL;
        goto FLASH_EM32_ERASE_EXIT;
    }
    LOG_DBG("Erase flash from memory address 0x%x, length %d.", start_mem_addr, len);

    k_sem_take(&dev_data->mutex, K_FOREVER);

    // Set Flash Key
    flash_em32_set_flash_key(PAGEERASE, pre_key1, pre_key2);

    // Erase Flash Pages
    start_page = (start_mem_addr / EM32_NV_FLASH_PAGE_SIZE);
    LOG_DBG("start_mem_addr=0x%x, start_page=0x%x.", start_mem_addr, start_page);
    end_page = (end_mem_addr / EM32_NV_FLASH_PAGE_SIZE) - 1;
    LOG_DBG("end_mem_addr=0x%x, end_page=0x%x.", end_mem_addr, end_page);
    for ( page_index = start_page; page_index <= end_page; page_index++ )
        flash_em32_erase_page(page_index);

    // Clear Flash Key
    flash_em32_clear_flash_key();

    k_sem_give(&dev_data->mutex);

FLASH_EM32_ERASE_EXIT:
    return ret;
}

/**
 * @brief Validate range of flash write
 */
bool flash_em32_valid_write_range(off_t offset, size_t len)
{
    bool ret = true;

    if (flash_em32_valid_read_range(offset, len) == false) 
    {
        ret = false;
        goto FLASH_EM32_VALID_WRITE_RANGE_EXIT;
    }

    if ((offset % sizeof(uint32_t)) != 0)
    {
        LOG_ERR("offset (%lu) not multiple of size of uint32 (%d).", offset, sizeof(uint32_t));
        ret = false;
        goto FLASH_EM32_VALID_WRITE_RANGE_EXIT;
    }

    if ((len % EM32_NV_FLASH_WRITE_BLOCK_SIZE) != 0) 
    {
        LOG_ERR("len (%zu) not multiple of write block size (%d).", len, EM32_NV_FLASH_WRITE_BLOCK_SIZE);
        ret = false;
        goto FLASH_EM32_VALID_WRITE_RANGE_EXIT;
    }

FLASH_EM32_VALID_WRITE_RANGE_EXIT:
    return ret;
}

/**
 * @brief Write data to Flash
 */
static int flash_em32_write(const struct device *dev, off_t offset, const void *data,
				size_t len)
{
    struct flash_em32_dev_data *dev_data = dev->data;
    int ret = 0;
    uint32_t pre_key1 = 0xb8, \
             pre_key2 = 0xf0, \
             address_base = EM32_NV_FLASH_ADDR, \
             address = address_base + offset, \
             *flash_mem = (uint32_t *)address, \
             boudary_addr = address + len; // End boundary of current partition (i.e. start of next partition)
    const uint32_t *write_data = (const uint32_t *)data;
    size_t write_block_size = EM32_NV_FLASH_WRITE_BLOCK_SIZE, // ex: 16
           write_block_count = len / write_block_size, \
           write_block_index = 0, \
           words_per_block = write_block_size / sizeof(uint32_t), \
           word_index_in_block = 0;

    // Validate Range
    if (flash_em32_valid_write_range(offset, len) == false) 
    {
        ret = -EINVAL;
        goto FLASH_EM32_WRITE_EXIT;
    }
    LOG_DBG("Write flash from address 0x%x, length %d.", address, len);

    k_sem_take(&dev_data->mutex, K_FOREVER);

    // Set Flash Key
    flash_em32_set_flash_key(USERMODE, pre_key1, pre_key2);

    // Write Flash Blocks (with word / uint32_t pointer)
    for (write_block_index = 0; write_block_index < write_block_count; write_block_index++)
    {
        for (word_index_in_block = 0; word_index_in_block < words_per_block; word_index_in_block++)
        {
            /* ex:
             * flash_mem[block_index * 4 + 0] = write_data[block_index * 4 + 0];
             * flash_mem[block_index * 4 + 1] = write_data[block_index * 4 + 1];
             * flash_mem[block_index * 4 + 2] = write_data[block_index * 4 + 2];
             * flash_mem[block_index * 4 + 3] = write_data[block_index * 4 + 3];
             */
            flash_mem[write_block_index * words_per_block + word_index_in_block] = write_data[write_block_index * words_per_block + word_index_in_block];
        }
    }

    // Clear Flash Key only at the end boundary of Rollback0, Rollback1, or EC-RW partitions
    if((boudary_addr == /* rollback0_boundary_addr */ EM32_ROLLBACK1_PARTITION_ADDR) || \
       (boudary_addr == /* rollback1_boundary_addr */ EM32_EC_RW_PARTITION_ADDR) || \
       (boudary_addr == /* ec_rw_boundary_addr */ (EM32_EC_RW_PARTITION_ADDR + EM32_EC_RW_PARTITION_SIZE)))
    {
        flash_em32_clear_flash_key();
    }

    k_sem_give(&dev_data->mutex);

FLASH_EM32_WRITE_EXIT:
    return ret;
}

/**
 * @brief Get flash parameters
 */
static const struct flash_parameters*
flash_em32_get_parameters(const struct device *dev)
{
    ARG_UNUSED(dev);

    return &flash_em32_parameters;
}

#if CONFIG_FLASH_PAGE_LAYOUT
static const struct flash_pages_layout flash_em32_pages_layout = {
    .pages_count = ( EM32_NV_FLASH_SIZE / EM32_NV_FLASH_PAGE_SIZE ),
    .pages_size  = EM32_NV_FLASH_PAGE_SIZE,
};

/**
 * @brief Get Flash page layout
 */
static void flash_em32_page_layout(const struct device *dev,
				       const struct flash_pages_layout **layout,
				       size_t *layout_size)
{
    ARG_UNUSED(dev);

    *layout = &flash_em32_pages_layout;
    *layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

/* Flash driver API */
static DEVICE_API(flash, flash_em32_driver_api) = {
    .read = flash_em32_read,
    .write = flash_em32_write,
    .erase = flash_em32_erase,
    .get_parameters = flash_em32_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
    .page_layout = flash_em32_page_layout,
#endif
};

/**
 * @brief Initialize Flash controller
 */
static int flash_em32_init(const struct device *dev)
{
    struct flash_em32_dev_data *data = dev->data;
    LOG_DBG("Initializing Elan eM32 flash controller");

    k_sem_init(&data->mutex, 1, 1);

    return 0;
}

/* Create device instances */
DEVICE_DT_INST_DEFINE(0, flash_em32_init, NULL,
		      &flash_em32_data, NULL, POST_KERNEL,
		      CONFIG_FLASH_INIT_PRIORITY, &flash_em32_driver_api);
