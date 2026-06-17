# EM32F967 PDSW2 (RTC off) Power Management Sample

## Overview

This sample exercises the EM32F967 Standby1 power mode **PDSW2** with the
RTC (32 kHz) domain turned off, using Zephyr's power management subsystem.
PDSW2 with RTC off is mapped to the Zephyr power state
`PM_STATE_STANDBY` with `substate-id = 2` via the `cpus/power-states`
definitions in `em32fxxx.dtsi` and the SoC power driver.

The application forces this power state using `pm_state_force()` and then
lets the CPU go idle so that the Zephyr PM core can enter the SoC-specific
`pm_state_set()` implementation in `soc/elan/em32f967/power.c`.

## Building and Running

```bash
west build -b 32f967_dv samples/elan_pdsw4
west flash
```

After reset, open the UART console. You should see a banner followed by a
short delay and log messages indicating that PDSW2 (RTC off) is being
requested.

On real hardware, entering PDSW2 (Standby1 with RAM retention) and RTC
disabled allows wake-up via configured wake sources such as GPIO PA15 as
described in the EM32F967 specification and the SoC power driver.

## Files

- `CMakeLists.txt` – Standard Zephyr sample CMake file, mirroring
  `samples/elan_wdt`.
- `prj.conf` – Enables `CONFIG_PM` and logging/console support.
- `32f967_dv.overlay` – Includes the shared private board DTSI
  (`elan/32f967_dv_priv.dtsi`).
	- `src/main.c` – Application code which forces PDSW2 (RTC off) using
	  Zephyr PM APIs.
