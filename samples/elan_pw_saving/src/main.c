/*
 * Copyright (c) 2026 ELAN Microelectronics Corp.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Power saving test application for EM32F967 (32f967_dv)
 *
 * Uses pm_state_force() to enter PDSW2 (Standby1) mode automatically.
 * Wakeup source: PA15 (WKUP3, rising edge)
 *
 * Flow:
 *   1. Configure PA15 as input with pull-down + EM32_GPIO_WKUP flag
 *   2. Configure PB14 as LED output
 *   3. Wait 4 seconds
 *   4. Force PDSW2 (standby1) mode with RTC off
 *   5. Zephyr PM enters PDSW2 → WFI → system resets on PA15 rising edge
 *   6. Boot banner appears again
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/state.h>
#include <zephyr/dt-bindings/gpio/em32-gpio.h>

/* PA15 button (WKUP3, rising edge) */
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);

int main(void)
{
	int ret;

	if (!gpio_is_ready_dt(&button)) {
		printk("ERROR: button device %s is not ready\n", button.port->name);
		return -ENODEV;
	}

	printk("\nWake-up button is connected to %s pin %d (PA15, WKUP3, rising edge)\n",
	       button.port->name, button.pin);

	/*
	 * Configure PA15 as input + pull-down + EM32_GPIO_WKUP.
	 * The EM32_GPIO_WKUP flag (bit 8) registers PA15 as a wakeup source.
	 * GPIO_ACTIVE_HIGH → rising edge wakeup (PA15 goes HIGH on button press)
	 * GPIO_PULL_DOWN   → default state is LOW, button pulls HIGH
	 */
	ret = gpio_pin_configure_dt(&button, GPIO_INPUT | EM32_GPIO_WKUP);
	if (ret < 0) {
		printk("ERROR: failed to configure button: %d\n", ret);
		return ret;
	}

	printk("Will wait 4s before entering power saving (PDSW2)\n");

	k_busy_wait(4000000); /* 4 seconds */
	
	printk("Entering power saving mode (PDSW2 / Standby1)\n");
	printk("Press PA15 button (rising edge) to wake up and reset the system\n");

	/*
	 * Force PDSW2 (Standby1) mode with RTC off.
	 *
	 * pm_state_force() tells the Zephyr PM subsystem to enter the
	 * specified state on the next idle. When the kernel becomes idle,
	 * pm_system_suspend() → pm_state_set() → pm_handle_standby_modes()
	 * → pm_enter_standby1(false) → SLEEPDEEP + WFI.
	 *
	 * The WakeUp controller asserts SYSTEM RESET on PA15 rising edge.
	 * No IRQ handler needed — same as STM32 SHUTDOWN mode.
	 *
	 * substate_id = 2 → PDSW2 (Standby1, RTC off)
	 *   Matches power.c pm_handle_standby_modes() switch case:
	 *     case 2: mode = PDSW2; rtc_enable = false;
	 */
	struct pm_state_info pdsw2_rtc_off = {
		.state = PM_STATE_STANDBY,
		.substate_id = 2,
		.min_residency_us = 0,
		.exit_latency_us = 0,
	};

	if (!pm_state_force(0U, &pdsw2_rtc_off)) {
		printk("ERROR: pm_state_force() failed; PDSW2 not available\n");
		return -ENOTSUP;
	}

	printk("pm_state_force(PDSW2) succeeded — system will enter deep sleep on next idle\n");

	/*
	 * Let the kernel become idle so that pm_system_suspend() runs.
	 * k_sleep(K_FOREVER) puts the calling thread to sleep, making
	 * the system idle → PM kicks in → PDSW2 entered → WFI → RESET.
	 */
	k_sleep(K_FOREVER);

	/* Never reached — system resets on wakeup */
	CODE_UNREACHABLE;
	return 0;
}
