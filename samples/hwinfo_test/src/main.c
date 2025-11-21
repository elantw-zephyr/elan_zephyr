/*
 * Copyright (c) 2025 Elan Microelectronics Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/sys/sys_io.h> // mm_reg_t & sys_read32
#include <zephyr/logging/log.h>

//#include <hwinfo_em32.h>
#include "../../../include/zephyr/drivers/hwinfo/hwinfo_em32.h"

LOG_MODULE_REGISTER(hwinfo_test, LOG_LEVEL_DBG);
//LOG_MODULE_REGISTER(hwinfo_test, CONFIG_LOG_DEFAULT_LEVEL);

ssize_t main(void)
{
	ssize_t ret = 0;
	uint32_t em32_supported_reset_cause = 0;
	uint32_t em32_reset_cause = 0;
    struct em32_hwinfo dev_hwinfo = {0};

	//LOG_DBG("hwinfo_test start.");

	// Get Device UID
	ret = hwinfo_get_device_id((uint8_t *)&dev_hwinfo, sizeof(dev_hwinfo));
	if (ret < 0) {
		LOG_ERR("hwinfo_get_device_id fail, err=%d.", ret);
		goto EXIT;
	}
	LOG_DBG("chip_id: 0x%x.", dev_hwinfo.chip_id);
	LOG_DBG("device_id: 0x%x.", dev_hwinfo.device_id);
	LOG_DBG("ic_version: 0x%x.", dev_hwinfo.ic_version);

	// Get Supported Reset Cause
	ret = hwinfo_get_supported_reset_cause(&em32_supported_reset_cause);
	if (ret < 0) {
		LOG_ERR("get_supported_reset_cause fail, err=%d.", ret);
		goto EXIT;
	}
	LOG_DBG("hwid_test: em32_supported_reset_cause=0x%x.", em32_supported_reset_cause);

	// Get Reset Cause
	ret = hwinfo_get_reset_cause(&em32_reset_cause);
	if (ret < 0) {
		LOG_ERR("get_reset_cause fail, err=%d.", ret);
		goto EXIT;
	}
	LOG_DBG("em32_reset_cause=0x%x.", em32_reset_cause);

EXIT:
	return ret;
}
