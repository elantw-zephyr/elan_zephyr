/*
 * Copyright (c) 2026 ELAN Microelectronics Corp.
 * SPDX-License-Identifier: Apache-2.0
 *
 * EM32F967 PWM Sample - PWMB_N on PA3 with high-drive set via DTS
 *
 * This sample is identical in behaviour to `elan_pwm`, but PA3 high-drive is
 * NOT configured by an application register write. Instead it is requested
 * declaratively in the device tree:
 *
 *     &pwmb_pa3_n {
 *         drive-strength = <1>;   // 1 = high drive
 *     };
 *     &pwm0 {
 *         pinctrl-0 = <&pwmb_pa3_n>;
 *         pinctrl-names = "default";
 *     };
 *
 * The em32 pinctrl driver applies this during pinctrl_apply_state() at PWM
 * init, setting bit 3 of IO_HD_PA_CTRL (0x4003021C) for us. Compare with the
 * elan_pwm sample, which performed:
 *
 *     uint32_t hd = sys_read32(EM32_IO_HD_PA_CTRL_REG);
 *     hd |= BIT(3);
 *     sys_write32(hd, EM32_IO_HD_PA_CTRL_REG);
 *
 * Hardware Setup:
 * - PA3: PWMB_N output (connect oscilloscope/logic analyzer here)
 * - PWM frequency: 1kHz
 * - Duty cycle: Ramps from 0% to 100% and back
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pwm_pa3_hd_sample, LOG_LEVEL_INF);

/* PWM device from devicetree */
#define PWM_NODE DT_NODELABEL(pwm0)

/* PWM parameters */
#define PWM_PERIOD_NS   1000000U  /* 1ms period = 1kHz frequency */
#define PWM_STEP_NS     50000U    /* 50us step for duty cycle ramping (5% step) */
#define RAMP_DELAY_MS   50        /* Delay between duty cycle changes */

/* PWM channel definitions
 * Channel B (index 1) is used for PWMB_N on PA3
 */
#define PWM_CHANNEL_B   1

int main(void)
{
	const struct device *pwm_dev;
	uint32_t pulse_ns;
	int ret;
	bool increasing = true;

	LOG_INF("EM32F967 PWM Sample - PWMB_N on PA3 (high-drive via DTS)");
	LOG_INF("========================================================");

	/* Get PWM device */
	pwm_dev = DEVICE_DT_GET(PWM_NODE);
	if (!device_is_ready(pwm_dev)) {
		LOG_ERR("PWM device %s is not ready", pwm_dev->name);
		return -ENODEV;
	}

	/*
	 * No manual high-drive register write here.
	 * PA3 high-drive is set by the pinctrl driver from the
	 * `drive-strength = <1>` property on the pwmb_pa3_n DTS node.
	 */
	LOG_INF("PA3 high-drive configured via DTS (drive-strength = <1>)");

	LOG_INF("PWM device %s is ready", pwm_dev->name);
	LOG_INF("Testing PWMB channel N output on PA3");
	LOG_INF("Expected: 1kHz PWM signal with ramping duty cycle");

	/* Start with 50% duty cycle */
	ret = pwm_set(pwm_dev, PWM_CHANNEL_B, PWM_PERIOD_NS, PWM_PERIOD_NS / 2, 0);
	if (ret < 0) {
		LOG_ERR("Failed to set PWM channel B: %d", ret);
		return ret;
	}
	LOG_INF("Channel B (PA3): Initial 50%% duty cycle at 1kHz");

	/* Wait a bit to allow measurement of initial state */
	k_msleep(2000);

	LOG_INF("Starting duty cycle ramping on PWMB (PA3)...");

	/* Ramp duty cycle on Channel B (PWMB_N on PA3) */
	pulse_ns = 0;

	while (1) {
		ret = pwm_set(pwm_dev, PWM_CHANNEL_B, PWM_PERIOD_NS, pulse_ns, 0);
		if (ret < 0) {
			LOG_ERR("Failed to update PWM: %d", ret);
			break;
		}

		/* Update pulse width */
		if (increasing) {
			pulse_ns += PWM_STEP_NS;
			if (pulse_ns >= PWM_PERIOD_NS) {
				pulse_ns = PWM_PERIOD_NS;
				increasing = false;
				LOG_INF("Duty cycle reached 100%%, reversing");
			}
		} else {
			if (pulse_ns >= PWM_STEP_NS) {
				pulse_ns -= PWM_STEP_NS;
			} else {
				pulse_ns = 0;
				increasing = true;
				LOG_INF("Duty cycle reached 0%%, reversing");
			}
		}

		k_msleep(RAMP_DELAY_MS);
	}

	return 0;
}
