/*
 * Copyright (c) 2025 Elan Microelectronics Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __ZEPHYR_INCLUDE_DRIVERS_HWINFO_EM32_H__
#define __ZEPHYR_INCLUDE_DRIVERS_HWINFO_EM32_H__

#include <stdint.h>

// #include <soc.h>
// #include "../../../../soc/elan/em32f967/soc.h"
#include <elan_em32.h> // Part of soc.h, only

/*
 * Data Structure
 */

struct em32_hwinfo {
	uint32_t chip_id;
	uint32_t device_id;
	uint32_t ic_version;
};

#endif //__ZEPHYR_INCLUDE_DRIVERS_HWINFO_EM32_H__
