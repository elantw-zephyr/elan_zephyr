/*
 * Copyright (c) 2024 Silicon Laboratories Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_EM32_AHB_H__
#define __ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_EM32_AHB_H__

#include <stdint.h>

#include <zephyr/sys/sys_io.h>    /* mm_reg_t, sys_read32/sys_write32 */

// #include <soc.h>
// #include "../../../../soc/elan/em32f967/soc.h"
// #include <elan_em32.h>
// #include "../../../../include/zephyr/drivers/clock_control/elan_em32.h" // Part of soc.h, only
// definititions & structures

/*
 * Enumeration
 */

typedef enum {
	HCLKG_DMA = 0x00,
	HCLKG_GPIOA = 0x01,
	HCLKG_GPIOB = 0x02,
	PCLKG_LPC = 0x03,
	HCLKG_7816_1 = 0x04,
	HCLKG_7816_2 = 0x05,
	HCLKG_ENCRYPT = 0x06,
	PCLKG_USART = 0x07,
	PCLKG_TMR1 = 0x08,
	PCLKG_TMR2 = 0x09,
	PCLKG_TMR3 = 0x0a,
	PCLKG_TMR4 = 0x0b,
	PCLKG_UART1 = 0x0c,
	PCLKG_UART2 = 0x0d,
	PCLKG_RVD1 = 0x0e,
	HCLKG_ESPI1 = 0x0f,
	PCLKG_SSP2 = 0x10,
	PCLKG_I2C1 = 0x11,
	PCLKG_I2C2 = 0x12,
	PCLKG_PWM = 0x13,
	PCLKG_RVD2 = 0x14,
	PCLKG_UDC = 0x15,
	PCLKG_ATRIM = 0x16,
	PCLKG_RTC = 0x17,
	PCLKG_BKP = 0x18,
	PCLKG_DWG = 0x19,
	PCLKG_PWR = 0x1a,
	PCLKG_CACHE = 0x1b,
	PCLKG_AIP = 0x1c,
	PCLKG_ECC = 0x1d,
	PCLKG_TRNG = 0x1e,
	HCLKG_EXTSPI = 0x1f,
	HCLKG_GHM_ACC1 = 0x20,
	HCLKG_GHM_ACC2 = 0x21,
	HCLKG_GHM_ACC3 = 0x22,
	HCLKF_GHM_IP = 0x23,
	HCLKF_FLASH_BIST = 0x24,
	HCLKF_GHM_RANSAC = 0x25,
	HCLKF_SWSPI = 0x26,
	HCLKF_GHM_DOUBLE = 0x27,
	HCLKF_GHM_DISTINGUISH = 0x28,
	HCLKF_GHM_LSE = 0x29,
	HCLKF_GHM_SAD = 0x2a,
	HCLKF_GHM_M2D = 0x2b,
	PCLKG_SSP1 = 0x30,
	PCLKG_ALL = 0xffff
} CLKGatingSwitch;
/*
 * Data Structure
 */

struct elan_em32_clock_control_subsys {
	uint32_t clock_group;
};

#endif //__ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_EM32_AHB_H__
