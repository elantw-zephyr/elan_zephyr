/*
 * SPDX-FileCopyrightText: 2026 ELAN Microelectronics Corp.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief EM32F967 PDSW2 (RTC off) Power Management Sample Application
 *
 * This sample demonstrates entering the EM32F967 PDSW2 Standby1 mode with
 * RTC (32 kHz) turned off through Zephyr's power management subsystem:
 *
 * - PDSW2 with RTC disabled is mapped to PM_STATE_STANDBY with substate-id 2
 * - pm_state_force() is used to request that state for CPU0
 * - The idle thread then calls into soc/elan/em32f967/power.c
 *
 * The low-level register programming for PDSW2 follows the EM32F967
 * specification and the reference implementation in 0203pm/em32f967/power.c.
 */

#include <zephyr/kernel.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/policy.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

LOG_MODULE_REGISTER(pdsw2_sample, LOG_LEVEL_DBG);

/*
 * Power state description for PDSW2 with RTC off.
 *
 * The mapping between (state, substate_id) and the EM32F967 power
 * switch settings is implemented in soc/elan/em32f967/power.c. For
 * (PM_STATE_STANDBY, substate_id = 2) the driver enters PDSW2 with the
 * RTC 32 kHz domain powered down (equivalent to PowerDownSwitch(PDSW2,0)).
 */
static const struct pm_state_info pdsw2_state = {
	.state = PM_STATE_STANDBY,
	.substate_id = 2,
	.min_residency_us = 0,
	.exit_latency_us = 0,
};

int main(void)
{
		printk("\n*** EM32F967 PDSW2 (RTC off) Power Management Sample ***\n");
	printk("Build time: " __DATE__ " " __TIME__ "\n");

	/*
	 * IMPORTANT:
	 *
	 * The reference 0203pm PDSW4 test locks *all* PM states during its
	 * countdown period to prevent the default PM policy from entering
	 * any low-power mode automatically while k_msleep() is running.
	 * Otherwise, the policy may pick a deep state (e.g. PDSW2/PDSW4),
	 * shut down the timer clock, and k_msleep() will never return.
	 *
	 * Mirror that behavior here: lock every PM state before the 3-second
	 * delay, then unlock them again immediately before we explicitly
	 * force PDSW4 using pm_state_force(). Forced states bypass policy
	 * decisions, but we still drop the locks to match the reference
	 * design and avoid surprising interactions.
	 */
	printk("Locking PM states during initial 3-second delay...\n");
	pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
	pm_policy_state_lock_get(PM_STATE_STANDBY, PM_ALL_SUBSTATES);
	pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_RAM, PM_ALL_SUBSTATES);
	pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_DISK, PM_ALL_SUBSTATES);
	pm_policy_state_lock_get(PM_STATE_SOFT_OFF, PM_ALL_SUBSTATES);

	/*
	 * Give the user a short window to see the banner and attach a
	 * console before the system attempts to enter deep standby.
	 */
		printk("About to sleep for 3 seconds before requesting PDSW2 (RTC off)...\n");
	k_msleep(3000);

	/* Now drop the locks so that the explicit PDSW4 request can proceed */
		printk("3-second delay complete, unlocking PM policy locks...\n");
	pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
	pm_policy_state_lock_put(PM_STATE_STANDBY, PM_ALL_SUBSTATES);
	pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_RAM, PM_ALL_SUBSTATES);
	pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_DISK, PM_ALL_SUBSTATES);
	pm_policy_state_lock_put(PM_STATE_SOFT_OFF, PM_ALL_SUBSTATES);

	/*
	 * Use printk instead of LOG_INF/LOG_ERR here so that messages are
	 * emitted immediately on the UART console. With deferred logging,
	 * log messages from this sample might not flush before the system
	 * enters deep standby (PDSW4) and resets.
	 */
		printk("Requesting PDSW2 (RTC off) (PM_STATE_STANDBY, substate-id=%d)...\n",
		       pdsw2_state.substate_id);

		if (!pm_state_force(0U, &pdsw2_state)) {
		printk("pm_state_force() failed; power state not available\n");
		return -1;
	}

		printk("PDSW2 forced. CPU will enter Standby1 on next idle.\n");
		printk("System will now attempt to enter PDSW2 (RTC off)...\n");

	/*
	 * Yield to the idle thread so that the PM core can see the forced
	 * state and call pm_state_set(). On real hardware, PDSW4 usually
	 * implies no RAM retention, so execution after resume may start from
	 * reset rather than returning here.
	 */
		while (1) {
			k_msleep(1000);
			printk("Running... (system may have resumed from PDSW2, RTC off)\n");
		}

	return 0;
}
