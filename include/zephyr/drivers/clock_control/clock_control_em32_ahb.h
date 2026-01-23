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
//#include <elan_em32.h>
// #include "../../../../include/zephyr/drivers/clock_control/elan_em32.h" // Part of soc.h, only
// definititions & structures

/*
 * Data Structure
 */

struct elan_em32_clock_control_subsys {
	uint32_t clock_group;
};

#endif //__ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_EM32_AHB_H__
