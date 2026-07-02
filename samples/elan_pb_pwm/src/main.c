/*
 * SPDX-FileCopyrightText: 2026 ELAN Microelectronics Corp.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief EM32F967 All-Port-B PWM Sample
 *
 * Drives all 6 PWM channels (PWMA..PWMF) simultaneously on the Port-B
 * "alternative" pin assignment (PWM_S = 1):
 *
 *   Channel | Pad  | Output
 *   --------+------+-------
 *    PWMA   | PB11 | N
 *    PWMB   | PB13 | N
 *    PWMC   | PB15 | N
 *    PWMD   | PB10 | P
 *    PWME   | PB12 | P
 *    PWMF   | PB14 | P
 *
 * Each channel runs at the same 1 kHz period but with a distinct duty
 * cycle so the six pads can be told apart on a scope. The driver does
 * the register programming (PWM_SW1/2/3 + IP_Share[18] + per-channel
 * PWMAE/IPWMAE/PWMAA/IPWMAA) based on the overlay's `output-type = <2>`
 * and the absence of `use-port-a`.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

LOG_MODULE_REGISTER(pb_pwm_sample, LOG_LEVEL_INF);

#define PWM_NODE       DT_NODELABEL(pwm0)
#define PWM_PERIOD_NS  1000000U  /* 1 ms = 1 kHz */

struct pwm_ch_info {
	uint32_t channel;   /* 0..5  → PWMA..PWMF */
	const char *name;
	const char *pad;
	const char *role;   /* "P" or "N" */
	uint32_t duty_pct;  /* Fixed duty cycle to make scopes easy */
};

/*
 * Distinct duty cycles so every pad is visibly different on a scope.
 * Order matches the alternative pin assignment requested by the sample.
 */
static const struct pwm_ch_info channels[] = {
	{ 0, "PWMA", "PB11", "N", 10 },
	{ 1, "PWMB", "PB13", "N", 25 },
	{ 2, "PWMC", "PB15", "N", 40 },
	{ 3, "PWMD", "PB10", "P", 55 },
	{ 4, "PWME", "PB12", "P", 70 },
	{ 5, "PWMF", "PB14", "P", 85 },
};

static int arm_channel(const struct device *pwm, const struct pwm_ch_info *ch)
{
	uint32_t pulse_ns = (PWM_PERIOD_NS / 100U) * ch->duty_pct;
	int ret = pwm_set(pwm, ch->channel, PWM_PERIOD_NS, pulse_ns, 0);

	if (ret < 0) {
		printk("  [ch%u %s] pwm_set failed: %d\n", ch->channel, ch->name, ret);
		return ret;
	}

	printk("  [ch%u %s] %s@%s : duty %u%% (pulse=%uns / period=%uns)\n",
	       ch->channel, ch->name, ch->role, ch->pad,
	       ch->duty_pct, pulse_ns, PWM_PERIOD_NS);
	return 0;
}

int main(void)
{
	const struct device *pwm = DEVICE_DT_GET(PWM_NODE);
	int ok = 0;

	printk("\n");
	printk("================================================================\n");
	printk("  EM32F967 All-Port-B PWM Sample (elan_pb_pwm)\n");
	printk("  Build : " __DATE__ " " __TIME__ "\n");
	printk("  Board : 32f967_dv\n");
	printk("  Target: PWMA..PWMF on PB11/PB13/PB15/PB10/PB12/PB14 @ 1 kHz\n");
	printk("================================================================\n");

	if (!device_is_ready(pwm)) {
		printk("[ERR] PWM device %s not ready\n", pwm->name);
		return -ENODEV;
	}
	printk("\n[1/2] PWM device %s ready.\n", pwm->name);

	printk("\n[2/2] Arming all 6 channels at 1 kHz with distinct duty cycles ...\n");
	for (size_t i = 0; i < ARRAY_SIZE(channels); i++) {
		if (arm_channel(pwm, &channels[i]) == 0) {
			ok++;
		}
	}
	printk("      %d / %d channels running\n", ok, (int)ARRAY_SIZE(channels));

	if (ok != ARRAY_SIZE(channels)) {
		printk("[WARN] Not all channels armed — waveform check will be partial.\n");
	}

	printk("\nAll PWM channels are now free-running in hardware.\n");
	printk("Probe PB10..PB15 on the 32f967_dv board to observe 6 simultaneous 1 kHz signals.\n");

	while (1) {
		k_msleep(10000);
		printk("[heartbeat] PWMA..PWMF still driving PB10..PB15 ...\n");
	}

	return 0;
}
