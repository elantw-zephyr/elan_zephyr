/*
 * Copyright (c) 2025 Elan Microelectronics Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __ZEPHYR_INCLUDE_DRIVERS_FLASH_EM32_H__
#define __ZEPHYR_INCLUDE_DRIVERS_FLASH_EM32_H__

#include <zephyr/devicetree.h>
#include <zephyr/storage/flash_map.h>

/*
 * Definitions
 */

// Flash
#define EM32_NV_FLASH_NODE             DT_INST(0, soc_nv_flash)
#define EM32_NV_FLASH_ADDR             DT_REG_ADDR(EM32_NV_FLASH_NODE)
#define EM32_NV_FLASH_SIZE             DT_REG_SIZE(EM32_NV_FLASH_NODE)
#define EM32_NV_FLASH_PAGE_SIZE        DT_PROP(EM32_NV_FLASH_NODE, erase_block_size)
#define EM32_NV_FLASH_WRITE_BLOCK_SIZE DT_PROP(EM32_NV_FLASH_NODE, write_block_size)

//
// Partitions
//

#if 0
// WP-RO
#define EM32_WP_RO_PARTITION_ADDR      FIXED_PARTITION_OFFSET(wp_ro)
#define EM32_WP_RO_PARTITION_SIZE      FIXED_PARTITION_SIZE(wp_ro)

// Rollback 0
#define EM32_ROLLBACK0_PARTITION_ADDR  FIXED_PARTITION_OFFSET(rollback_0)
#define EM32_ROLLBACK0_PARTITION_SIZE  FIXED_PARTITION_SIZE(rollback_0)

// Rollback 1
#define EM32_ROLLBACK1_PARTITION_ADDR  FIXED_PARTITION_OFFSET(rollback_1)
#define EM32_ROLLBACK1_PARTITION_SIZE  FIXED_PARTITION_SIZE(rollback_1)

// EC-RW
#define EM32_EC_RW_PARTITION_ADDR      FIXED_PARTITION_OFFSET(ec_rw)
#define EM32_EC_RW_PARTITION_SIZE      FIXED_PARTITION_SIZE(ec_rw)
#endif

#define EM32_ROLLBACK1_PARTITION_ADDR  0x10022000

#define EM32_EC_RW_PARTITION_ADDR      0x10024000
#define EM32_EC_RW_PARTITION_SIZE      0x00062000

#endif //__ZEPHYR_INCLUDE_DRIVERS_FLASH_EM32_H__
