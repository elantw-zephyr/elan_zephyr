/*
 * Copyright (c) 2025 Elan Microelectronics Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT elan_em32_ahb

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
// #include <zephyr/drivers/clock_control/clock_control_em32_ahb.h>
#include "../../include/zephyr/drivers/clock_control/clock_control_em32_ahb.h"

#define LOG_LEVEL LOG_LEVEL_DBG
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(em32_ahb);

/*
 * Configurations
 */
__IO static uint32_t irc_freq_src = IRCLOW12;
static uint32_t ahb_count = 12000; // 12M Hz

static void _z_nop_delay(uint32_t cnt)
{
	for (uint32_t i = 0; i < cnt; i++) {
		__asm volatile("nop");
	}
}

void delay_10us(void)
{
	_z_nop_delay(ahb_count / 1000);
}

void delay_100us(void)
{
	_z_nop_delay(ahb_count / 100);
}

uint32_t elan_em32_get_ahb_freq(void)
{
	uint32_t irc_freq = 0;
	uint32_t irc_pll_freq = 0;
	uint32_t main_freq = 0;
	uint32_t ahb_freq = 0;

	switch (MIRCCTRL->MIRCRCM) {
	case 0x00:
		irc_freq = 12000;
		irc_pll_freq = 12000 * 16 / 2;
		break; // 12M/120M
	case 0x01:
		irc_freq = 16000;
		irc_pll_freq = 16000 * 16 / 4;
		break; // 16M/80M
	case 0x02:
		irc_freq = 20000;
		irc_pll_freq = 20000 * 16 / 4;
		break; // 20M/100M
	case 0x03:
		irc_freq = 24000;
		irc_pll_freq = 24000 * 16 / 4;
		break; // 24M/120M
	case 0x04:
		irc_freq = 28000;
		irc_pll_freq = 28000 * 16 / 6;
		break; // 28M/93M
	case 0x05:
		irc_freq = 32000;
		irc_pll_freq = 32000 * 16 / 6;
		break; // 32M/107M
	default:
		break;
	}

	switch (SYSREGCTRL->HCLKSEL & 0x03) {
	case 0x00: {
		main_freq = irc_freq;
	} break;

	case 0x01: {
		if (SYSREGCTRL->XTALHIRCSEL) {
			main_freq = 24000 * 5;
		} else {
			main_freq = irc_pll_freq;
		}
	} break;

	case 0x02: {
		main_freq = 0xffffffff;
	} break;

	default: {
		main_freq = 0;
	} break;
	}

	main_freq = main_freq >> (SYSREGCTRL->HCLKDIV);
	ahb_freq = main_freq;

	return ahb_freq;
}

void elan_em32_clk_gating_disable(CLKGatingSwitch GatingN)
{
	if (GatingN == PCLKG_ALL) {
		CLKGATEREG = 0;
		CLKGATEREG2 = 0;
	} else if (GatingN <= 31) {
		CLKGATEREG &= ~(0x01 << GatingN);
	} else {
		CLKGATEREG2 &= ~(0x01 << (GatingN - 32));
	}

	return;
}

void elan_em32_set_ahb_freq(ClockSource clk_src, Freq1Source freq_src, AHBPreScaler pre_div)
{
	bool bPLL = 0;

	elan_em32_clk_gating_disable(PCLKG_AIP);

	if (freq_src == IRCLOW12 /* irc_freq_src */) {
		SYSREGCTRL->HCLKDIV = pre_div;
		return;
	}

	MISCREGCTRL->WaitCountPass = 0x0a;
	MISCREGCTRL->WaitCount = 3;
	MISCREGCTRL->WaitCountSet = 1;

	if (SYSREGCTRL->HCLKSEL == 0x01) {
		SYSREGCTRL->HCLKSEL = 0x00;
		delay_100us();
		SYSPLLCTRL->SYSPLLPD = 1;
		_z_nop_delay(10);
	}

	if (clk_src == External1) {
		SYSREGCTRL->HCLKSEL = 0x02;
	} else {
		if (freq_src >> 4) {
			bPLL = 1;
		} else {
			bPLL = 0;
		}

		switch (freq_src) {
		case IRCLOW12:
			MIRCCTRL_2->MIRCTall = MIRC12M_R_2->MIRC_Tall;
			MIRCCTRL_2->MIRCTV12 = ~MIRC12M_R_2->MIRC_TV12;
			break;

		case IRCLOW16:
		case IRCHIGH64:
			MIRCCTRL_2->MIRCTall = MIRC16M_2->MIRC_Tall;
			MIRCCTRL_2->MIRCTV12 = ~MIRC16M_2->MIRC_TV12;
			break;

		case IRCLOW20:
		case IRCHIGH80:
			MIRCCTRL_2->MIRCTall = MIRC20M_2->MIRC_Tall;
			MIRCCTRL_2->MIRCTV12 = ~MIRC20M_2->MIRC_TV12;
			break;

		case IRCLOW24:
		case IRCHIGH96:
			MIRCCTRL_2->MIRCTall = MIRC24M_2->MIRC_Tall;
			MIRCCTRL_2->MIRCTV12 = ~MIRC24M_2->MIRC_TV12;
			break;

		case IRCLOW28:
		case IRCHIGH112:
			MIRCCTRL_2->MIRCTall = MIRC28M_2->MIRC_Tall;
			MIRCCTRL_2->MIRCTV12 = ~MIRC28M_2->MIRC_TV12;
			break;

		case IRCLOW32:
		case IRCHIGH128:
			MIRCCTRL_2->MIRCTall = MIRC32M_2->MIRC_Tall;
			MIRCCTRL_2->MIRCTV12 = ~MIRC32M_2->MIRC_TV12;
			break;

		default:
			break;
		}

		delay_100us();
		MIRCCTRL->MIRCRCM = (freq_src & 0x0f);
		SYSREGCTRL->XTALHIRCSEL = 0;

		if (bPLL) {
			switch (freq_src) {
			case IRCHIGH64:
				SYSPLLCTRL->SYSPLLFSET = 0;
				break;
			case IRCHIGH80:
				SYSPLLCTRL->SYSPLLFSET = 1;
				break;
			case IRCHIGH96:
				SYSPLLCTRL->SYSPLLFSET = 2;
				break;
			case IRCHIGH112:
				SYSPLLCTRL->SYSPLLFSET = 3;
				break;
			case IRCHIGH128:
				SYSPLLCTRL->SYSPLLFSET = 3;
				break;
			default:
				break;
			}

			LDOPLL->PLLLDO_PD = 0;
			_z_nop_delay(10);
			_z_nop_delay(10);
			LDOPLL->PLLLDO_VP_SEL = 0;
			delay_10us();
			delay_10us();
			SYSPLLCTRL->SYSPLLPD = 0;
			_z_nop_delay(10);
			while (SYSPLLCTRL->SYSPLLSTABLE == 0)
				;
			_z_nop_delay(10);
			SYSREGCTRL->HCLKSEL = 0x01;
			_z_nop_delay(10);
		} else {
			SYSREGCTRL->HCLKSEL = 0x00;
			delay_100us();
			SYSPLLCTRL->SYSPLLPD = 1;
		}

		irc_freq_src = freq_src;
	}

	if (pre_div == DIV128) {
		SYSREGCTRL->HCLKDIV = (pre_div - 1);
	} else {
		SYSREGCTRL->HCLKDIV = (pre_div + 1);
	}

	MISCREGCTRL->WaitCountSet = 0;
	MISCREGCTRL->WaitCountPass = 0;
	SYSREGCTRL->HCLKDIV = pre_div;

	ahb_count = elan_em32_get_ahb_freq();
	LOG_DBG("ahb_count=%d.", ahb_count);

	return;
}

static int elan_em32_ahb_clock_control_on(const struct device *dev, clock_control_subsys_t sys)
{
	struct elan_em32_clock_control_subsys *subsys =
		(struct elan_em32_clock_control_subsys *)sys;
	int ret = 0;
	uint32_t clk_grp = subsys->clock_group;
	// LOG_DBG("clock_group=%d.", clk_grp);

	if (((clk_grp >= HCLKG_DMA) &&
	     (clk_grp <= PCLKG_SSP1) /* HCLKG_DMA <= clk_grp <= PCLKG_SSP1 */) ||
	    (clk_grp == PCLKG_ALL)) {
		elan_em32_clk_gating_disable((CLKGatingSwitch)clk_grp);
	} else {
		LOG_ERR("Unknown clock group #%d", clk_grp);
		ret = -EINVAL;
	}

	return ret;
}

static int elan_em32_ahb_clock_control_off(const struct device *dev, clock_control_subsys_t sys)
{
	return -ENOTSUP;
}

static int elan_em32_ahb_clock_control_get_rate(const struct device *dev,
						clock_control_subsys_t sys, uint32_t *rate)
{
	int ahb_clk_rate = 0;

	ahb_clk_rate = elan_em32_get_ahb_freq() * 1000; // unit: 1000 Hz
	// LOG_DBG("ahb_clk_rate=%d (Hz).", ahb_clk_rate);

	*rate = ahb_clk_rate;

	return 0;
}

static DEVICE_API(clock_control, elan_em32_ahb_clock_control_api) = {
	.on = elan_em32_ahb_clock_control_on,
	.off = elan_em32_ahb_clock_control_off,
	.get_rate = elan_em32_ahb_clock_control_get_rate,
};

static int elan_em32_ahb_clock_control_init(const struct device *dev)
{
	const struct elan_em32_ahb_clock_control_config *config = dev->config;
	ClockSource clk_src = config->clock_source;
	Freq1Source clk_freq = config->clock_frequency;
	AHBPreScaler clk_div = config->clock_divider;
	LOG_DBG("clock_source=0x%x, clock_frequency=0x%x, clock_divider=0x%x.", clk_src, clk_freq,
		clk_div);

	// Set AHB Clock
	elan_em32_set_ahb_freq(clk_src, clk_freq, clk_div);

	return 0;
}

static const struct elan_em32_ahb_clock_control_config em32_ahb_config = {
	.clock_source = DT_PROP(DT_NODELABEL(clk_ahb), clock_source),
	.clock_frequency = DT_PROP(DT_NODELABEL(clk_ahb), clock_frequency),
	.clock_divider = DT_PROP(DT_NODELABEL(clk_ahb), clock_divider),
};

DEVICE_DT_INST_DEFINE(0, &elan_em32_ahb_clock_control_init, NULL, NULL, &em32_ahb_config,
		      PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
		      &elan_em32_ahb_clock_control_api);
