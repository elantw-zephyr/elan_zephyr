/*
 * Copyright (c) 2025 Elan Microelectronics Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT elan_em32_uid

#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/logging/log.h>
#include <stdint.h>
#include <string.h>

// #include <soc.h>
// #include "../../../../soc/elan/em32f967/soc.h"
#include <elan_em32.h>
// #include "../../../../include/zephyr/drivers/clock_control/elan_em32.h" // Part of soc.h, only
//#include <hwinfo_em32.h>
#include "../../../include/zephyr/drivers/hwinfo/hwinfo_em32.h"

//LOG_MODULE_REGISTER(em32_uid, LOG_LEVEL_DBG);
LOG_MODULE_REGISTER(em32_uid, CONFIG_HWINFO_LOG_LEVEL);

ssize_t z_impl_hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
	mm_reg_t chip_id_addr = (mm_reg_t) DT_INST_REG_ADDR_BY_NAME(0, chip_id);
	mm_reg_t dev_id_addr  = (mm_reg_t) DT_INST_REG_ADDR_BY_NAME(0, device_id);
	mm_reg_t ic_ver_addr  = (mm_reg_t) DT_INST_REG_ADDR_BY_NAME(0, ic_version);
	size_t len = 0;
	struct em32_hwinfo dev_hwinfo = {0};

	// Chip ID
	dev_hwinfo.chip_id = sys_read32(chip_id_addr);
	LOG_DBG("chip_id_addr=0x%lx, chip_id=0x%08x.", chip_id_addr, dev_hwinfo.chip_id);
	
	// Device ID
	dev_hwinfo.device_id = sys_read32(dev_id_addr);
	LOG_DBG("dev_id_addr=0x%lx, device_id=0x%08x.", dev_id_addr, dev_hwinfo.device_id);

	// IC Version
	dev_hwinfo.ic_version = sys_read32(ic_ver_addr);
	LOG_DBG("ic_ver_addr=0x%lx, ic_version=0x%08x.", ic_ver_addr, dev_hwinfo.ic_version);

	// Load Device UID to Input Buffer
	len = MIN(length, sizeof(dev_hwinfo));
	memcpy(buffer, &dev_hwinfo, len);
	LOG_DBG("len=%d.", len);

	return len;
}

int z_impl_hwinfo_get_supported_reset_cause(uint32_t *supported)
{
	*supported = (RESET_PIN | RESET_SOFTWARE | RESET_BROWNOUT | RESET_WATCHDOG |
		      RESET_LOW_POWER_WAKE);

	return 0;
}

int z_impl_hwinfo_get_reset_cause(uint32_t *cause)
{
	if (SYSSTATUSCTRL->SWRESETS) // SW Reset
	{
		*cause = RESET_SOFTWARE;
	} else if (SYSSTATUSCTRL->BORRESETS) // Brownout Reset
	{
		*cause = RESET_BROWNOUT;
	} else if (SYSSTATUSCTRL->WDTRESETS) // Watchdog Reset
	{
		*cause = RESET_WATCHDOG;
	} else // Low Power Wake Reset or Reset Pin
	{
		SYSREGCTRL->POWEN = 1; // Enable power domain

		if (POWERSWCTRL->StandBy1_S) // Low Power Wake Reset
		{
			*cause = RESET_LOW_POWER_WAKE;
		} else // Reset Pin
		{
			*cause = RESET_PIN;
		}
	}

	LOG_DBG("reset case: 0x%x.", *cause);

	return 0;
}
