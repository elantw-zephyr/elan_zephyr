/*
 * SPDX-FileCopyrightText: 2026 ELAN Microelectronics Corp.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief EM32F967 — wake from PDSW2 (Standby1, RTC off) on any WKUP1..WKUP7 pin.
 *
 * Flow:
 *   1. Boot banner.
 *   2. For each WKUP node declared in the overlay, call
 *      gpio_pin_configure_dt(spec, GPIO_INPUT | EM32_GPIO_WKUP).
 *      The EM32 GPIO driver (drivers/gpio/gpio_em32.c :: gpio_em32_pin_configure)
 *      forwards each request to em32_pwr_wkup_pin_cfg_gpio() which stores
 *      the (port, pin, flags) tuple in a private registry.
 *   3. Wait 3 seconds with all PM policy states locked (so k_msleep really
 *      sleeps and no deeper state is auto-selected by the policy).
 *   4. Release the locks and call pm_state_force(PM_STATE_STANDBY, substate=2).
 *      Substate 2 maps to PDSW2 with rtc_enable=false (SIRC32 domain off) in
 *      soc/elan/em32f967/power.c :: pm_handle_standby_modes().
 *   5. pm_handle_standby_modes() calls em32_pwr_wkup_pin_cfg_pupd() which
 *      applies AF5 mux, WAKEUP_REG WAKEUPEN/WAKETYPE bits, and enables
 *      WakeUp_Int_IRQn (IRQ 3).
 *   6. A button press on any of the 7 wakeup pins toggles the pin LOW →
 *      falling-edge event → WakeUp controller asserts system RESET → the
 *      CPU restarts from the reset vector and main() prints the banner again.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/policy.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#include <zephyr/dt-bindings/gpio/em32-gpio.h>

LOG_MODULE_REGISTER(all_wkup_pin, LOG_LEVEL_INF);

#define WKUP_NODE(n)	DT_NODELABEL(wkup##n)

BUILD_ASSERT(DT_NODE_EXISTS(WKUP_NODE(1)), "wkup1 missing from overlay");
BUILD_ASSERT(DT_NODE_EXISTS(WKUP_NODE(2)), "wkup2 missing from overlay");
BUILD_ASSERT(DT_NODE_EXISTS(WKUP_NODE(3)), "wkup3 missing from overlay");
BUILD_ASSERT(DT_NODE_EXISTS(WKUP_NODE(4)), "wkup4 missing from overlay");
BUILD_ASSERT(DT_NODE_EXISTS(WKUP_NODE(5)), "wkup5 missing from overlay");
BUILD_ASSERT(DT_NODE_EXISTS(WKUP_NODE(6)), "wkup6 missing from overlay");
BUILD_ASSERT(DT_NODE_EXISTS(WKUP_NODE(7)), "wkup7 missing from overlay");

struct wkup_desc {
	const char *name;
	struct gpio_dt_spec spec;
};

static const struct wkup_desc wkup_pins[] = {
	{ "WKUP1 (PA6)",  GPIO_DT_SPEC_GET(WKUP_NODE(1), gpios) },
	{ "WKUP2 (PA14)", GPIO_DT_SPEC_GET(WKUP_NODE(2), gpios) },
	{ "WKUP3 (PA15)", GPIO_DT_SPEC_GET(WKUP_NODE(3), gpios) },
	{ "WKUP4 (PB3)",  GPIO_DT_SPEC_GET(WKUP_NODE(4), gpios) },
	{ "WKUP5 (PB7)",  GPIO_DT_SPEC_GET(WKUP_NODE(5), gpios) },
	{ "WKUP6 (PB8)",  GPIO_DT_SPEC_GET(WKUP_NODE(6), gpios) },
	{ "WKUP7 (PB9)",  GPIO_DT_SPEC_GET(WKUP_NODE(7), gpios) },
};

/*
 * PDSW2 + RTC disabled.  substate-id=2 is already declared as `standby1`
 * in dts/arm/elan/32f967_dv_priv.dtsi, so pm_state_force() will accept it.
 */
static const struct pm_state_info pdsw2_rtc_off = {
	.state = PM_STATE_STANDBY,
	.substate_id = 2,
	.min_residency_us = 0,
	.exit_latency_us = 0,
};

static int register_all_wakeup_pins(void)
{
	int registered = 0;

	for (size_t i = 0; i < ARRAY_SIZE(wkup_pins); i++) {
		const struct gpio_dt_spec *spec = &wkup_pins[i].spec;
		int ret;

		if (!gpio_is_ready_dt(spec)) {
			printk("  [%s] port %s not ready — skipped\n",
				wkup_pins[i].name, spec->port->name);
			continue;
		}

		ret = gpio_pin_configure_dt(spec, GPIO_INPUT | EM32_GPIO_WKUP);
		if (ret < 0) {
			printk("  [%s] gpio_pin_configure_dt failed (%d) — skipped\n",
				wkup_pins[i].name, ret);
			continue;
		}

		printk("  [%s] registered (port=%s pin=%d flags=0x%x)\n",
			wkup_pins[i].name, spec->port->name,
			spec->pin, spec->dt_flags);
		registered++;
	}

	return registered;
}

static void lock_all_pm_states(void)
{
	pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
	pm_policy_state_lock_get(PM_STATE_STANDBY,        PM_ALL_SUBSTATES);
	pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_RAM, PM_ALL_SUBSTATES);
	pm_policy_state_lock_get(PM_STATE_SOFT_OFF,       PM_ALL_SUBSTATES);
}

static void unlock_all_pm_states(void)
{
	pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
	pm_policy_state_lock_put(PM_STATE_STANDBY,        PM_ALL_SUBSTATES);
	pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_RAM, PM_ALL_SUBSTATES);
	pm_policy_state_lock_put(PM_STATE_SOFT_OFF,       PM_ALL_SUBSTATES);
}

int main(void)
{
	int registered;

	printk("\n");
	printk("================================================================\n");
	printk("  EM32F967 All-WKUP-Pin Sample (elan_all_wkup_pin)\n");
	printk("  Build : " __DATE__ " " __TIME__ "\n");
	printk("  Board : 32f967_dv\n");
	printk("  Target: PDSW2 (Standby1), RTC off — substate_id = 2\n");
	printk("  Wake  : any of WKUP1..WKUP7 (PA6/PA14/PA15/PB3/PB7/PB8/PB9)\n");
	printk("================================================================\n");

	printk("\n[1/4] Registering all 7 WKUP sources with the EM32 WakeUp controller ...\n");
	registered = register_all_wakeup_pins();
	printk("      %d / %d WKUP sources registered\n",
		registered, (int)ARRAY_SIZE(wkup_pins));

	if (registered == 0) {
		printk("[ERR] no wakeup pins registered — aborting\n");
		return -ENODEV;
	}

	printk("\n[2/4] Locking PM states during 3-second pre-sleep window ...\n");
	lock_all_pm_states();
	for (int i = 3; i > 0; i--) {
		printk("      %d ...\n", i);
		k_msleep(1000);
	}

	printk("\n[3/4] Releasing locks and forcing PDSW2 (substate 2, RTC off) ...\n");
	unlock_all_pm_states();

	if (!pm_state_force(0U, &pdsw2_rtc_off)) {
		printk("[ERR] pm_state_force(PDSW2, substate=2) rejected — check DTS power-states\n");
		return -EIO;
	}
	printk("      PDSW2 requested. power.c will now:\n");
	printk("        - call em32_pwr_wkup_pin_cfg_pupd() (AF5 mux + WAKEUP_REG + IRQ3)\n");
	printk("        - power down the 32 kHz oscillator (SIRC32PD=1)\n");
	printk("        - enter Standby1 (PDSW2)\n");
	printk("      Press any of SW/WKUP buttons to trigger a SYSTEM RESET wake.\n");

	printk("\n[4/4] Yielding to idle. Waiting for button-press wakeup ...\n");
	while (1) {
		k_msleep(1000);
		printk("  (still awake — PDSW2 entry pending / post-wake loop)\n");
	}

	return 0;
}
