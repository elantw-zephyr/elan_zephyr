/*
 * Copyright (c) 2024 ELAN Microelectronics Corp.
 * SPDX-License-Identifier: Apache-2.0
 *
 * EM32F967 PWM Controller Driver
 *
 * Based on tested pwm.c sample code from EM32F967 SDK.
 * Implements Zephyr PWM driver API for EM32F967 6-channel PWM controller.
 */

#define DT_DRV_COMPAT elan_em32_pwm

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/util.h>
#include <soc.h>

#include "pwm_em32.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pwm_em32, CONFIG_PWM_LOG_LEVEL);

struct pwm_em32_config {
	uint32_t base;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	const struct pinctrl_dev_config *pcfg;
	bool use_port_a;    /* true = PA0-5 (PWM_S=0), false = PB10-15 (PWM_S=1) */
	uint8_t output_type; /* PWM_EM32_OUTPUT_P, PWM_EM32_OUTPUT_N, or PWM_EM32_OUTPUT_BOTH */
};

struct pwm_em32_data {
	uint32_t clock_freq;
	struct k_spinlock lock;
};

/* Register access helpers */
static inline uint32_t pwm_em32_read(const struct pwm_em32_config *cfg, uint32_t offset)
{
	return sys_read32(cfg->base + offset);
}

static inline void pwm_em32_write(const struct pwm_em32_config *cfg, uint32_t offset,
				  uint32_t value)
{
	sys_write32(value, cfg->base + offset);
}

/* Get channel register base offset */
static inline uint32_t pwm_em32_channel_offset(uint32_t channel)
{
	if (channel >= PWM_EM32_NUM_CHANNELS) {
		return 0;
	}
	return pwm_channel_offsets[channel];
}

/* Enable clock gating for PWM */
static void pwm_em32_clock_enable(void)
{
	uint32_t reg = sys_read32(CLK_GATE_REG_ADDR);
	reg &= ~BIT(PCLKG_PWM);  /* Clear bit to enable clock (active low) */
	sys_write32(reg, CLK_GATE_REG_ADDR);
}

/* Configure IP Share for PWM pin selection and N output routing
 *
 * For N (complementary) outputs on Port A:
 * - PA1 (PWMA_N): Requires IP_Share[15]=1 (PWM_SW1=1) to enable PWMA1 mode
 * - PA3 (PWMB_N): Requires IP_Share[16]=1 (PWM_SW2=1) to enable PWMB1 mode
 * - PA5 (PWMC_N): Requires IP_Share[17]=1 (PWM_SW3=1) to enable PWMC1 mode
 *
 * For Port B N outputs (PB11, PB13, PB15), the same switch bits apply.
 */
static void pwm_em32_configure_pin_select(bool use_port_b, uint8_t output_type)
{
	uint32_t reg = sys_read32(IP_SHARE_CTRL_ADDR);

	if (use_port_b) {
		reg |= IP_SHARE_PWM_S;  /* Set bit 18 for PB10-15 */
	} else {
		reg &= ~IP_SHARE_PWM_S; /* Clear bit 18 for PA0-5 */
	}

	/* If using N output (or both P and N), set the PWM switch bits
	 * to enable the Bx mode (PWMx1) for N output routing.
	 * This enables PWMA1, PWMB1, PWMC1 modes for the N outputs.
	 */
	if (output_type == PWM_EM32_OUTPUT_N || output_type == PWM_EM32_OUTPUT_BOTH) {
		/* Enable all N output routing switches for flexibility */
		reg |= IP_SHARE_PWM_D_A1_S;  /* PWM_SW1: Enable PWMA1 for PA1/PB11 */
		reg |= IP_SHARE_PWM_E_B1_S;  /* PWM_SW2: Enable PWMB1 for PA3/PB13 */
		reg |= IP_SHARE_PWM_F_C1_S;  /* PWM_SW3: Enable PWMC1 for PA5/PB15 */
		LOG_DBG("Enabled N output routing (PWM_SWx=1)");
	} else {
		/* Clear switch bits for standard D/E/F channel mode */
		reg &= ~IP_SHARE_PWM_D_A1_S;
		reg &= ~IP_SHARE_PWM_E_B1_S;
		reg &= ~IP_SHARE_PWM_F_C1_S;
	}

	sys_write32(reg, IP_SHARE_CTRL_ADDR);
}

/* Enable a PWM channel */
static void pwm_em32_channel_enable(const struct pwm_em32_config *cfg, uint32_t channel)
{
	uint32_t encr = pwm_em32_read(cfg, PWM_ENCR_OFFSET);
	encr |= pwm_channel_enable_bits[channel];
	pwm_em32_write(cfg, PWM_ENCR_OFFSET, encr);
}

/* Disable a PWM channel */
static void pwm_em32_channel_disable(const struct pwm_em32_config *cfg, uint32_t channel)
{
	uint32_t encr = pwm_em32_read(cfg, PWM_ENCR_OFFSET);
	encr &= ~pwm_channel_enable_bits[channel];
	pwm_em32_write(cfg, PWM_ENCR_OFFSET, encr);
}

/* Set PWM period and duty cycle for a channel */
static int pwm_em32_set_cycles(const struct device *dev, uint32_t channel,
			       uint32_t period_cycles, uint32_t pulse_cycles,
			       pwm_flags_t flags)
{
	const struct pwm_em32_config *cfg = dev->config;
	struct pwm_em32_data *data = dev->data;
	uint32_t ch_offset;
	uint32_t cr_val;
	k_spinlock_key_t key;

	if (channel >= PWM_EM32_NUM_CHANNELS) {
		LOG_ERR("Invalid channel %u (max %d)", channel, PWM_EM32_NUM_CHANNELS - 1);
		return -EINVAL;
	}

	if (period_cycles == 0) {
		/* Disable channel if period is 0 */
		key = k_spin_lock(&data->lock);
		pwm_em32_channel_disable(cfg, channel);
		k_spin_unlock(&data->lock, key);
		return 0;
	}

	if (pulse_cycles > period_cycles) {
		LOG_ERR("Pulse cycles (%u) > period cycles (%u)", pulse_cycles, period_cycles);
		return -EINVAL;
	}

	ch_offset = pwm_em32_channel_offset(channel);

	key = k_spin_lock(&data->lock);

	/*
	 * Configure control register FIRST (before setting period/duty)
	 *
	 * Hardware behavior (verified by testing):
	 * - PWMAE (bit 15): Enable P output
	 * - PWMAA (bit 13): P output polarity - 1=normal (HIGH during duty), 0=inverted
	 * - IPWMAE (bit 14): Enable N output
	 * - IPWMAA (bit 12): N output polarity - 0=complementary to P, 1=same as P
	 */
	cr_val = 0;

	/* Configure output based on output_type setting */
	switch (cfg->output_type) {
	case PWM_EM32_OUTPUT_N:
		/* N output only (e.g., for PA3 PWMB_N)
		 * The N output is "complementary to P", so we must enable the P output
		 * logic internally (PWMAE=1) even though P is not routed to a pin.
		 * SDK reference: PWMEnable() always sets both P and N mode.
		 */
		cr_val |= PWMCR_PWMAE;   /* Enable P output logic (required for N to work) */
		cr_val |= PWMCR_IPWMAE;  /* Enable N output */
		if (flags & PWM_POLARITY_INVERTED) {
			/* Inverted: Set PWMAA=1 (P=HIGH during duty), so N=LOW during duty */
			cr_val |= PWMCR_PWMAA;
			/* IPWMAA=0 for complementary behavior */
		} else {
			/* Normal: Set PWMAA=0 (P=LOW during duty), so N=HIGH during duty */
			/* PWMAA=0, IPWMAA=0 */
		}
		break;

	case PWM_EM32_OUTPUT_BOTH:
		/* Both P and N outputs (complementary pair) */
		cr_val |= PWMCR_PWMAE;   /* Enable P output */
		cr_val |= PWMCR_IPWMAE;  /* Enable N output */
		if (flags & PWM_POLARITY_INVERTED) {
			/* Inverted polarity */
			/* PWMAA=0 for inverted P, IPWMAA=0 for complementary N */
		} else {
			/* Normal polarity */
			cr_val |= PWMCR_PWMAA;   /* PWMAA=1 for normal P */
			/* IPWMAA=0 for complementary N (opposite of P) */
		}
		break;

	case PWM_EM32_OUTPUT_P:
	default:
		/* P output only (default) */
		cr_val |= PWMCR_PWMAE;   /* Enable P output */
		if (flags & PWM_POLARITY_INVERTED) {
			/* Inverted: P output LOW during duty portion */
			/* PWMAA=0 for inverted P output */
		} else {
			/* Normal: P output HIGH during duty portion */
			cr_val |= PWMCR_PWMAA;   /* PWMAA=1 for normal/positive P output */
		}
		break;
	}

	pwm_em32_write(cfg, ch_offset + PWM_CR_OFFSET, cr_val);

	/* Set duty register FIRST (as per SDK order) */
	pwm_em32_write(cfg, ch_offset + PWM_DTR_OFFSET, pulse_cycles);

	/* Set period register (period_cycles - 1 as per SDK) */
	pwm_em32_write(cfg, ch_offset + PWM_PRDR_OFFSET, period_cycles - 1);

	/* Enable channel timer */
	pwm_em32_channel_enable(cfg, channel);

	k_spin_unlock(&data->lock, key);

	LOG_DBG("Ch%u: CR@0x%03x=0x%04x, DTR@0x%03x=%u, PRDR@0x%03x=%u, ENCR=0x%x",
		channel,
		ch_offset + PWM_CR_OFFSET, cr_val,
		ch_offset + PWM_DTR_OFFSET, pulse_cycles,
		ch_offset + PWM_PRDR_OFFSET, period_cycles - 1,
		pwm_em32_read(cfg, PWM_ENCR_OFFSET));

	return 0;
}

/* Get cycles per second (clock frequency) */
static int pwm_em32_get_cycles_per_sec(const struct device *dev, uint32_t channel,
				       uint64_t *cycles)
{
	struct pwm_em32_data *data = dev->data;

	if (channel >= PWM_EM32_NUM_CHANNELS) {
		return -EINVAL;
	}

	*cycles = (uint64_t)data->clock_freq;
	return 0;
}

/* Set dead-time for complementary outputs (for future use) */
static int __maybe_unused pwm_em32_set_dead_time(const struct device *dev, uint32_t dead_time_ns)
{
	const struct pwm_em32_config *cfg = dev->config;
	struct pwm_em32_data *data = dev->data;
	uint32_t dead_time_cycles;
	uint32_t prescaler = 0;
	uint32_t reg_val;

	/* Calculate dead-time in clock cycles */
	dead_time_cycles = (uint64_t)dead_time_ns * data->clock_freq / 1000000000ULL;

	/* Find appropriate prescaler */
	while (dead_time_cycles > 0xFFFF && prescaler < 3) {
		prescaler++;
		dead_time_cycles >>= 1;
	}

	if (dead_time_cycles > 0xFFFF) {
		LOG_ERR("Dead-time too large");
		return -EINVAL;
	}

	reg_val = (dead_time_cycles & PWMDEADTR_VALUE_MASK) |
		  ((prescaler & PWMDEADTR_TP_MASK) << PWMDEADTR_TP_SHIFT);

	pwm_em32_write(cfg, PWM_DEADTR_OFFSET, reg_val);

	LOG_DBG("Dead-time set: %u ns (%u cycles, prescaler %u)",
		dead_time_ns, dead_time_cycles, prescaler);

	return 0;
}

/* PWM driver API */
static DEVICE_API(pwm, pwm_em32_driver_api) = {
	.set_cycles = pwm_em32_set_cycles,
	.get_cycles_per_sec = pwm_em32_get_cycles_per_sec,
};

/* Device initialization */
static int pwm_em32_init(const struct device *dev)
{
	const struct pwm_em32_config *cfg = dev->config;
	struct pwm_em32_data *data = dev->data;
	int ret;

	/* Enable PWM clock */
	pwm_em32_clock_enable();

	/* Get clock frequency */
	if (cfg->clock_dev != NULL) {
		ret = clock_control_get_rate(cfg->clock_dev, cfg->clock_subsys,
					     &data->clock_freq);
		if (ret < 0) {
			LOG_ERR("Failed to get clock rate: %d", ret);
			/* Use default 48MHz */
			data->clock_freq = 48000000;
		}
	} else {
		/* Default to 48MHz APB clock */
		data->clock_freq = 48000000;
	}

	LOG_DBG("PWM clock frequency: %u Hz", data->clock_freq);

	/* Configure pin control */
	if (cfg->pcfg != NULL) {
		ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret < 0) {
			LOG_ERR("Failed to apply pinctrl: %d", ret);
			return ret;
		}
	}

	/* Configure IP Share for PWM pin selection and N output routing */
	pwm_em32_configure_pin_select(!cfg->use_port_a, cfg->output_type);

	LOG_DBG("PWM using Port %c pins, output_type=%u",
		cfg->use_port_a ? 'A' : 'B', cfg->output_type);

	/* Disable all channels initially */
	pwm_em32_write(cfg, PWM_ENCR_OFFSET, 0);

	/* Clear dead-time register */
	pwm_em32_write(cfg, PWM_DEADTR_OFFSET, 0);

	LOG_INF("EM32 PWM controller initialized at 0x%08x", cfg->base);
	LOG_DBG("IP_SHARE=0x%08x, CLK_GATE=0x%08x",
		sys_read32(IP_SHARE_CTRL_ADDR), sys_read32(CLK_GATE_REG_ADDR));

	return 0;
}

/* Device instantiation macros */
#define PWM_EM32_INIT(n)                                                       \
	PINCTRL_DT_INST_DEFINE(n);                                             \
                                                                               \
	static const struct pwm_em32_config pwm_em32_config_##n = {            \
		.base = DT_INST_REG_ADDR(n),                                   \
		.clock_dev = DEVICE_DT_GET_OR_NULL(DT_INST_CLOCKS_CTLR(n)),    \
		.clock_subsys = NULL,                                          \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                     \
		.use_port_a = DT_INST_PROP_OR(n, use_port_a, false),           \
		.output_type = DT_INST_PROP_OR(n, output_type, 0),             \
	};                                                                     \
                                                                               \
	static struct pwm_em32_data pwm_em32_data_##n;                         \
                                                                               \
	DEVICE_DT_INST_DEFINE(n, pwm_em32_init, NULL,                          \
			      &pwm_em32_data_##n, &pwm_em32_config_##n,        \
			      POST_KERNEL, CONFIG_PWM_INIT_PRIORITY,           \
			      &pwm_em32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_EM32_INIT)

