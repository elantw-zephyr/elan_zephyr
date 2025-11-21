/*
 * Copyright (c) 2025 Elan Microelectronics Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT elan_em32_apb

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
// #include <zephyr/drivers/clock_control/clock_control_em32_ahb.h>
#include "../../include/zephyr/drivers/clock_control/clock_control_em32_ahb.h"
// #include <zephyr/drivers/clock_control/clock_control_em32_apb.h>
#include "../../include/zephyr/drivers/clock_control/clock_control_em32_apb.h"

#define LOG_LEVEL LOG_LEVEL_DBG
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(em32_apb);

static int elan_em32_apb_clock_control_on(const struct device *dev, clock_control_subsys_t sys)
{
	const struct elan_em32_apb_clock_control_config *config = dev->config;

	return clock_control_on(config->clock_device, sys);
}

static int elan_em32_apb_clock_control_off(const struct device *dev, clock_control_subsys_t sys)
{
	return -ENOTSUP;
}

static int elan_em32_apb_clock_control_get_rate(const struct device *dev,
						clock_control_subsys_t sys, uint32_t *rate)
{
	const struct elan_em32_apb_clock_control_config *config = dev->config;
	int ret = 0;
	uint32_t ahb_clk_rate = 0;
	uint32_t apb_clk_rate = 0;

	// Get AHB Clock Rate
	ret = clock_control_get_rate(config->clock_device, NULL, &ahb_clk_rate);
	if (ret) {
		LOG_ERR("Fail to Get AHB Clock Rate, err=%d.", ret);
		return ret;
	}

	apb_clk_rate = ahb_clk_rate / 2; // Fix value of clock divider (2)
	// LOG_DBG("apb_clk_rate=%d (Hz).", apb_clk_rate);

	*rate = apb_clk_rate;

	return ret;
}

static DEVICE_API(clock_control, elan_em32_apb_clock_control_api) = {
	.on = elan_em32_apb_clock_control_on,
	.off = elan_em32_apb_clock_control_off,
	.get_rate = elan_em32_apb_clock_control_get_rate,
};

static int elan_em32_apb_clock_control_init(const struct device *dev)
{
	const struct elan_em32_apb_clock_control_config *config = dev->config;

	if (!device_is_ready(config->clock_device)) {
		LOG_ERR("Clock source not ready!");
		return -ENODEV;
	}

	LOG_DBG("Initialized.");

	return 0;
}

static const struct elan_em32_apb_clock_control_config em32_apb_config = {
	.clock_device = DEVICE_DT_GET(DT_NODELABEL(clk_ahb)),
	/*.clock_device = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(0)), */
};

DEVICE_DT_INST_DEFINE(0, &elan_em32_apb_clock_control_init, NULL, NULL, &em32_apb_config,
		      PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
		      &elan_em32_apb_clock_control_api);
