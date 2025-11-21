/*
 * Copyright (c) 2024 Silicon Laboratories Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_EM32_APB_H__
#define __ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_EM32_APB_H__

#include <stdint.h>
#include "clock_control_em32_ahb.h"

/*
 * Data Structure
 */

struct elan_em32_apb_clock_control_config {
	const struct device *clock_device;
};

#endif //__ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_EM32_APB_H__
