/*
 * Copyright (c) 2024 Silicon Laboratories Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_EM32_AHB_H__
#define __ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_EM32_AHB_H__

#include <stdint.h>

// #include <soc.h>
// #include "../../../../soc/elan/em32f967/soc.h"
#include <elan_em32.h>
// #include "../../../../include/zephyr/drivers/clock_control/elan_em32.h" // Part of soc.h, only
// definititions & structures

/*
 * Enumeration
 */

typedef enum {
	IRCLOW = 0x00,
	IRCHIGH = 0x01,
	External1 = 0x20
} ClockSource;

typedef enum {
	IRCLOW12 = 0x00,
	IRCLOW16 = 0x01,
	IRCLOW20 = 0x02,
	IRCLOW24 = 0x03,
	IRCLOW28 = 0x04,
	IRCLOW32 = 0x05,
	IRCHIGH64 = 0x11,
	IRCHIGH80 = 0x12,
	IRCHIGH96 = 0x13,
	IRCHIGH112 = 0x14,
	IRCHIGH128 = 0x15,
	IRCHIGH96Q = 0x16
} Freq1Source;

typedef enum {
	DIV1 = 0x00,
	DIV2 = 0x01,
	DIV4 = 0x02,
	DIV8 = 0x03,
	DIV16 = 0x04,
	DIV32 = 0x05,
	DIV64 = 0x06,
	DIV128 = 0x07
} AHBPreScaler;

/*
 * Data Structure
 */

struct elan_em32_ahb_clock_control_config {
	uint32_t clock_source;
	uint32_t clock_frequency;
	uint32_t clock_divider;
};

struct elan_em32_clock_control_subsys {
	uint32_t clock_group;
};

/*
 * Function Prototype
 */
 
void delay_10us(void);
void delay_100us(void);

#endif //__ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_EM32_AHB_H__
