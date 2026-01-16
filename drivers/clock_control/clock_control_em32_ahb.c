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

#include "soc_sysctrl.h"
#include "soc_clkctrl.h"

#define LOG_LEVEL LOG_LEVEL_DBG
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(em32_ahb);

#if 1 /* temp, will move to include file */
/* Register Base */
#define MIRC_BASE		0x100a0000

/* Register Offsets */
#define MIRC_12M_R_2_OFF 0x7f60
#define MIRC_16M_2_OFF   0x6070
#define MIRC_20M_2_OFF   0x6074
#define MIRC_24M_2_OFF   0x6078
#define MIRC_28M_2_OFF   0x607c
#define MIRC_32M_2_OFF   0x6080

/* Field Masks for MIRC_CTRL */
#define MIRC_TALL_MASK  GENMASK(9, 0)     /* [9:0]    MIRC_Tall */
#define MIRC_TV12_MASK  GENMASK(12, 10)   /* [12:10]  MIRC_TV12 */
#endif /* temp, will move to include file */

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

static inline uint32_t ahb_em32_read_field(uint32_t base, uint32_t offset, uint32_t mask)
{
	uint32_t reg = 0;

	reg = sys_read32(base + offset);

	return FIELD_GET(mask, reg);
}

static inline void ahb_em32_write_field(uint32_t base, uint32_t offset,
					uint32_t mask, uint32_t value)
{
	uint32_t reg = 0;

	//LOG_DBG("base=0x%x offset=0x%x mask=0x%x value=0x%x", base, offset, mask, value);

	/* Optional: check value range */
	if ((value << __builtin_ctz(mask)) & ~mask) {
		LOG_ERR("Value 0x%x exceeds field mask 0x%x", value, mask);
		return;
	}

	reg = sys_read32(base + offset);
	//LOG_DBG("reg-read=0x%x", reg);
	reg &= ~mask;
	//LOG_DBG("reg-~mask=0x%x", reg);
	reg |= FIELD_PREP(mask, value);
	//LOG_DBG("reg-FIELD_PREP=0x%x", reg);
	sys_write32(reg, base + offset);
}

uint32_t elan_em32_get_ahb_freq(const struct device *dev)
{
	const struct elan_em32_ahb_clock_control_config *config = dev->config;
	mm_reg_t sysctrl_base = config->sysctrl_base;
	mm_reg_t clkctrl_base = config->clkctrl_base;
	uint32_t irc_freq = 0;
	uint32_t irc_pll_freq = 0;
	uint32_t main_freq = 0;
	uint32_t ahb_freq = 0;

	uint32_t mirc_rcm = ahb_em32_read_field(clkctrl_base,
					CLKCTRL_MIRC_CTRL_OFF, CLKCTRL_MIRC_RCM_MASK);
	switch (mirc_rcm) {
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

	uint32_t hclk_sel = ahb_em32_read_field(sysctrl_base,
					SYSCTRL_SYS_REG_CTRL_OFF, SYSCTRL_HCLK_SEL_MASK);
	switch (hclk_sel) {
	case 0x00: {
		main_freq = irc_freq;
	} break;

	case 0x01: {
		uint32_t xtal_hirc_sel = ahb_em32_read_field(sysctrl_base,
						SYSCTRL_SYS_REG_CTRL_OFF, SYSCTRL_XTAL_HIRC_SEL);
		if (xtal_hirc_sel) {
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

	uint32_t hclk_div = ahb_em32_read_field(sysctrl_base,
					SYSCTRL_SYS_REG_CTRL_OFF, SYSCTRL_HCLK_DIV_MASK);
	main_freq = main_freq >> (hclk_div);
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

void elan_em32_set_ahb_freq(const struct device *dev)
{
	const struct elan_em32_ahb_clock_control_config *config = dev->config;
	mm_reg_t sysctrl_base = config->sysctrl_base;
	mm_reg_t clkctrl_base = config->clkctrl_base;
	ClockSource clk_src = config->clock_source;
	Freq1Source freq_src = config->clock_frequency;
	AHBPreScaler pre_div = config->clock_divider;
	bool bPLL = 0;

	LOG_DBG("clock_source=0x%x, clock_frequency=0x%x, clock_divider=0x%x.", clk_src, freq_src,
		pre_div);

	elan_em32_clk_gating_disable(PCLKG_AIP);

	if (freq_src == IRCLOW12 /* irc_freq_src */) {
		ahb_em32_write_field(sysctrl_base, SYSCTRL_SYS_REG_CTRL_OFF,
					SYSCTRL_HCLK_DIV_MASK, pre_div);
		return;
	}

	ahb_em32_write_field(sysctrl_base, SYSCTRL_MISC_REG_CTRL_OFF,
				SYSCTRL_WAIT_COUNT_PASS_MASK, 0x0a);
	ahb_em32_write_field(sysctrl_base, SYSCTRL_MISC_REG_CTRL_OFF,
				SYSCTRL_WAIT_COUNT_MASK, 0x03);
	ahb_em32_write_field(sysctrl_base, SYSCTRL_MISC_REG_CTRL_OFF,
				SYSCTRL_WAIT_COUNT_SET, 0x01);

	uint32_t hclk_sel = ahb_em32_read_field(sysctrl_base,
					SYSCTRL_SYS_REG_CTRL_OFF, SYSCTRL_HCLK_SEL_MASK);
	if (hclk_sel == 0x01) {
		ahb_em32_write_field(sysctrl_base, SYSCTRL_SYS_REG_CTRL_OFF,
					SYSCTRL_HCLK_SEL_MASK, 0x00);
		delay_100us();
		ahb_em32_write_field(clkctrl_base, CLKCTRL_SYS_PLL_CTRL_OFF,
					CLKCTRL_SYS_PLL_PD, 0x01);
		_z_nop_delay(10);
	}

	if (clk_src == External1) {
		ahb_em32_write_field(sysctrl_base, SYSCTRL_SYS_REG_CTRL_OFF,
					SYSCTRL_HCLK_SEL_MASK, 0x02);
	} else {
		if (freq_src >> 4) {
			bPLL = 1;
		} else {
			bPLL = 0;
		}

		uint32_t mirc_tall, mirc_tv12;
		switch (freq_src) {
		case IRCLOW12:
			mirc_tall = ahb_em32_read_field(MIRC_BASE,
							MIRC_12M_R_2_OFF, MIRC_TALL_MASK);
			mirc_tv12 = ahb_em32_read_field(MIRC_BASE,
							MIRC_12M_R_2_OFF, MIRC_TV12_MASK);
			ahb_em32_write_field(clkctrl_base, CLKCTRL_MIRC_CTRL2_OFF,
						CLKCTRL_MIRC2_TALL_MASK, (mirc_tall & 0x3FF));
			ahb_em32_write_field(clkctrl_base, CLKCTRL_MIRC_CTRL2_OFF,
						CLKCTRL_MIRC2_TV12_MASK, (~mirc_tv12 & 0x7));
			break;

		case IRCLOW16:
		case IRCHIGH64:
			mirc_tall = ahb_em32_read_field(MIRC_BASE,
							MIRC_16M_2_OFF, MIRC_TALL_MASK);
			mirc_tv12 = ahb_em32_read_field(MIRC_BASE,
							MIRC_16M_2_OFF, MIRC_TV12_MASK);
			ahb_em32_write_field(clkctrl_base, CLKCTRL_MIRC_CTRL2_OFF,
						CLKCTRL_MIRC2_TALL_MASK, (mirc_tall & 0x3FF));
			ahb_em32_write_field(clkctrl_base, CLKCTRL_MIRC_CTRL2_OFF,
						CLKCTRL_MIRC2_TV12_MASK, (~mirc_tv12 & 0x7));
			break;

		case IRCLOW20:
		case IRCHIGH80:
			mirc_tall = ahb_em32_read_field(MIRC_BASE,
							MIRC_20M_2_OFF, MIRC_TALL_MASK);
			mirc_tv12 = ahb_em32_read_field(MIRC_BASE,
							MIRC_20M_2_OFF, MIRC_TV12_MASK);
			ahb_em32_write_field(clkctrl_base, CLKCTRL_MIRC_CTRL2_OFF,
						CLKCTRL_MIRC2_TALL_MASK, (mirc_tall & 0x3FF));
			ahb_em32_write_field(clkctrl_base, CLKCTRL_MIRC_CTRL2_OFF,
						CLKCTRL_MIRC2_TV12_MASK, (~mirc_tv12 & 0x7));
			break;

		case IRCLOW24:
		case IRCHIGH96:
			mirc_tall = ahb_em32_read_field(MIRC_BASE,
							MIRC_24M_2_OFF, MIRC_TALL_MASK);
			mirc_tv12 = ahb_em32_read_field(MIRC_BASE,
							MIRC_24M_2_OFF, MIRC_TV12_MASK);
			ahb_em32_write_field(clkctrl_base, CLKCTRL_MIRC_CTRL2_OFF,
						CLKCTRL_MIRC2_TALL_MASK, (mirc_tall & 0x3FF));
			ahb_em32_write_field(clkctrl_base, CLKCTRL_MIRC_CTRL2_OFF,
						CLKCTRL_MIRC2_TV12_MASK, (~mirc_tv12 & 0x7));
			break;

		case IRCLOW28:
		case IRCHIGH112:
			mirc_tall = ahb_em32_read_field(MIRC_BASE,
							MIRC_28M_2_OFF, MIRC_TALL_MASK);
			mirc_tv12 = ahb_em32_read_field(MIRC_BASE,
							MIRC_28M_2_OFF, MIRC_TV12_MASK);
			ahb_em32_write_field(clkctrl_base, CLKCTRL_MIRC_CTRL2_OFF,
						CLKCTRL_MIRC2_TALL_MASK, (mirc_tall & 0x3FF));
			ahb_em32_write_field(clkctrl_base, CLKCTRL_MIRC_CTRL2_OFF,
						CLKCTRL_MIRC2_TV12_MASK, (~mirc_tv12 & 0x7));
			break;

		case IRCLOW32:
		case IRCHIGH128:
			mirc_tall = ahb_em32_read_field(MIRC_BASE,
							MIRC_32M_2_OFF, MIRC_TALL_MASK);
			mirc_tv12 = ahb_em32_read_field(MIRC_BASE,
							MIRC_32M_2_OFF, MIRC_TV12_MASK);
			ahb_em32_write_field(clkctrl_base, CLKCTRL_MIRC_CTRL2_OFF,
						CLKCTRL_MIRC2_TALL_MASK, (mirc_tall & 0x3FF));
			ahb_em32_write_field(clkctrl_base, CLKCTRL_MIRC_CTRL2_OFF,
						CLKCTRL_MIRC2_TV12_MASK, (~mirc_tv12 & 0x7));
			break;

		default:
			break;
		}

		delay_100us();
		ahb_em32_write_field(clkctrl_base, CLKCTRL_MIRC_CTRL_OFF,
					CLKCTRL_MIRC_RCM_MASK, (freq_src & 0x0f));
		ahb_em32_write_field(sysctrl_base, SYSCTRL_SYS_REG_CTRL_OFF,
					SYSCTRL_XTAL_HIRC_SEL, 0x00);

		if (bPLL) {
			switch (freq_src) {
			case IRCHIGH64:
				ahb_em32_write_field(clkctrl_base, CLKCTRL_SYS_PLL_CTRL_OFF,
							CLKCTRL_SYS_PLL_FSET_MASK, 0x00);
				break;
			case IRCHIGH80:
				ahb_em32_write_field(clkctrl_base, CLKCTRL_SYS_PLL_CTRL_OFF,
							CLKCTRL_SYS_PLL_FSET_MASK, 0x01);
				break;
			case IRCHIGH96:
				ahb_em32_write_field(clkctrl_base, CLKCTRL_SYS_PLL_CTRL_OFF,
							CLKCTRL_SYS_PLL_FSET_MASK, 0x02);
				break;
			case IRCHIGH112:
				ahb_em32_write_field(clkctrl_base, CLKCTRL_SYS_PLL_CTRL_OFF,
							CLKCTRL_SYS_PLL_FSET_MASK, 0x03);
				break;
			case IRCHIGH128:
				ahb_em32_write_field(clkctrl_base, CLKCTRL_SYS_PLL_CTRL_OFF,
							CLKCTRL_SYS_PLL_FSET_MASK, 0x03);
				break;
			default:
				break;
			}

			ahb_em32_write_field(clkctrl_base, CLKCTRL_LDO_PLL_OFF,
						CLKCTRL_PLL_LDO_PD, 0x00);
			_z_nop_delay(10);
			_z_nop_delay(10);
			ahb_em32_write_field(clkctrl_base, CLKCTRL_LDO_PLL_OFF,
						CLKCTRL_PLL_LDO_VP_SEL, 0x00);
			delay_10us();
			delay_10us();
			ahb_em32_write_field(clkctrl_base, CLKCTRL_SYS_PLL_CTRL_OFF,
						CLKCTRL_SYS_PLL_PD, 0x00);
			_z_nop_delay(10);
			while (ahb_em32_read_field(clkctrl_base,
						   CLKCTRL_SYS_PLL_CTRL_OFF,
						   CLKCTRL_SYS_PLL_STABLE) == 0)
				;
			_z_nop_delay(10);
			ahb_em32_write_field(sysctrl_base, SYSCTRL_SYS_REG_CTRL_OFF,
						SYSCTRL_HCLK_SEL_MASK, 0x01);
			_z_nop_delay(10);
		} else {
			ahb_em32_write_field(sysctrl_base, SYSCTRL_SYS_REG_CTRL_OFF,
						SYSCTRL_HCLK_SEL_MASK, 0x00);
			delay_100us();
			ahb_em32_write_field(clkctrl_base, CLKCTRL_SYS_PLL_CTRL_OFF,
						CLKCTRL_SYS_PLL_PD, 0x01);
		}

		irc_freq_src = freq_src;
	}

	if (pre_div == DIV128) {
		ahb_em32_write_field(sysctrl_base, SYSCTRL_SYS_REG_CTRL_OFF,
					SYSCTRL_HCLK_DIV_MASK, (pre_div - 1));
	} else {
		ahb_em32_write_field(sysctrl_base, SYSCTRL_SYS_REG_CTRL_OFF,
					SYSCTRL_HCLK_DIV_MASK, (pre_div + 1));
	}

	ahb_em32_write_field(sysctrl_base, SYSCTRL_MISC_REG_CTRL_OFF,
				SYSCTRL_WAIT_COUNT_SET, 0x00);
	ahb_em32_write_field(sysctrl_base, SYSCTRL_MISC_REG_CTRL_OFF,
				SYSCTRL_WAIT_COUNT_PASS_MASK, 0x00);
	ahb_em32_write_field(sysctrl_base, SYSCTRL_SYS_REG_CTRL_OFF,
				SYSCTRL_HCLK_DIV_MASK, pre_div);

	ahb_count = elan_em32_get_ahb_freq(dev);
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

	ahb_clk_rate = elan_em32_get_ahb_freq(dev) * 1000; // unit: 1000 Hz
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
	elan_em32_set_ahb_freq(dev);

	return 0;
}

static const struct elan_em32_ahb_clock_control_config em32_ahb_config = {
	.sysctrl_base = DT_REG_ADDR(SYSCTRL_DT_NODE),
	.clkctrl_base = DT_REG_ADDR(CLKCTRL_DT_NODE),
	.clock_source = DT_PROP(DT_NODELABEL(clk_ahb), clock_source),
	.clock_frequency = DT_PROP(DT_NODELABEL(clk_ahb), clock_frequency),
	.clock_divider = DT_PROP(DT_NODELABEL(clk_ahb), clock_divider),
};

DEVICE_DT_INST_DEFINE(0, &elan_em32_ahb_clock_control_init, NULL, NULL, &em32_ahb_config,
		      PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
		      &elan_em32_ahb_clock_control_api);
