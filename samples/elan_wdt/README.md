# EM32F967 Watchdog Sample

This sample demonstrates the watchdog timer functionality on the EM32F967 microcontroller.

## Overview

The watchdog timer (WDT) is a safety mechanism that resets the system if the application becomes unresponsive. This sample shows how to:

- Configure and start the watchdog timer
- Feed the watchdog to prevent timeout
- Handle watchdog timeout interrupts
- Test different scenarios including forced timeout

## Features Demonstrated

1. **Basic Watchdog Operation**
   - Setup with 5-second timeout
   - Automatic feeding every 2 seconds
   - Manual feeding via button or command

2. **Interrupt Handling**
   - Timeout callback function
   - LED indication of watchdog activity
   - Status reporting

3. **User Interaction**
   - Console-based command interface
   - Button input for manual feeding
   - LED feedback for visual indication

## Hardware Requirements

- EM32F967 development board (32f967_dv)
- LEDs connected to PB14 (LED0) and PB15 (LED1)
- Button connected to PA6 (SW0)
- UART console for interaction

## Usage

### Building and Running

```bash
west build -b 32f967_dv samples/elan_wdt
west flash
```

### Commands

- `s` - Start watchdog timer
- `t` - Stop watchdog timer (if supported)
- `f` - Feed watchdog manually
- `a` - Toggle auto-feed mode
- `r` - Reset system (trigger timeout)
- `i` - Show status information
- `h` - Show help menu

### LED Indicators

- **LED0 (PB14)**: Toggles when watchdog is fed
- **LED1 (PB15)**: Rapid flash during timeout

### Button Input

- **SW0 (PA6)**: Manual watchdog feed

## Expected Behavior

1. **Normal Operation**: LED0 blinks every 2 seconds as watchdog is fed
2. **Manual Feed**: Press SW0 button to feed watchdog manually
3. **Timeout Test**: Use 'r' command to disable auto-feed and trigger reset
4. **Status Display**: Use 'i' command to see current watchdog status

## Watchdog Configuration

- **Timeout**: 5000 ms (5 seconds)
- **Feed Interval**: 2000 ms (2 seconds) in auto mode
- **Clock Source**: APB clock (32 kHz typical)
- **Reset Type**: System reset on timeout

## Troubleshooting

If the watchdog doesn't work as expected:

1. Check device tree configuration in board files
2. Verify clock source is properly configured
3. Ensure watchdog is enabled in device tree (`status = "okay"`)
4. Check console output for error messages
5. Verify LED connections for visual feedback