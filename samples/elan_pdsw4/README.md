# EM32F967 PDSW4 Power Management Sample

## Overview

This sample exercises the EM32F967 deep standby power mode **PDSW4** using
Zephyr's power management subsystem. PDSW4 is mapped to the Zephyr power
state `PM_STATE_SUSPEND_TO_RAM` with `substate-id = 4` via the
`cpus/power-states` definitions in `em32fxxx.dtsi`.

The application forces this power state using `pm_state_force()` and then
lets the CPU go idle so that the Zephyr PM core can enter the SoC-specific
`pm_state_set()` implementation in `soc/elan/em32f967/power.c`.

## Building and Running

```bash
west build -b 32f967_dv samples/elan_pdsw4
west flash
```

After reset, open the UART console. You should see a banner followed by a
short delay and log messages indicating that PDSW4 is being requested.

On real hardware, entering PDSW4 (deep standby, no RAM retention) typically
results in a cold or warm reset on wake-up, depending on the configured
wakeup source, as described in the EM32F967 specification.

## Files

- `CMakeLists.txt` – Standard Zephyr sample CMake file, mirroring
  `samples/elan_wdt`.
- `prj.conf` – Enables `CONFIG_PM` and logging/console support.
- `32f967_dv.overlay` – Includes the shared private board DTSI
  (`elan/32f967_dv_priv.dtsi`).
- `src/main.c` – Application code which forces PDSW4 using Zephyr PM APIs.
