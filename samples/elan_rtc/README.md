# ELAN EM32F967 RTC & Watchdog Test Application

This sample application demonstrates and tests the Real-Time Clock (RTC) and Watchdog Timer (WDT) functionality on the ELAN EM32F967 microcontroller.

## Features

### RTC Testing
- **Basic Time Operations**: Set and get current time
- **Time Accuracy**: Verify time counting functionality
- **Alarm Functionality**: Set alarms and test callback mechanisms
- **Interrupt Handling**: Test RTC interrupt generation

### Watchdog Testing
- **Basic Watchdog**: Configure timeout and feed operations
- **Reset Functionality**: Test system reset on timeout
- **Callback Mode**: Test watchdog callback before reset
- **Channel Management**: Multi-channel watchdog support

### Interactive Shell
- Real-time control of RTC and watchdog via shell commands
- Manual testing capabilities
- Status monitoring and debugging

## Requirements

### Hardware
- ELAN EM32F967 development board
- 32.768kHz crystal for RTC (if external crystal is used)
- UART connection for console output

### Software
- Zephyr RTOS
- ELAN EM32F967 HAL drivers
- RTC and Watchdog device drivers

## Building and Running

### Build the application:
```bash
cd samples/elan_rtc_wdt
west build -b elan_32f967_dv
```

### Flash to the board:
```bash
west flash
```

### Monitor the output:
```bash
west build -t monitor
```

## Test Scenarios

### Automatic Tests
The application runs comprehensive automatic tests on startup:

1. **RTC Basic Test**
   - Sets initial time (2024-08-04 12:30:45)
   - Reads back time after 1 second
   - Verifies time increment accuracy

2. **RTC Alarm Test**
   - Sets alarm for 5 seconds from current time
   - Waits for alarm callback
   - Verifies alarm functionality

3. **Watchdog Basic Test**
   - Configures 5-second timeout
   - Tests feeding mechanism
   - Verifies watchdog setup

4. **Watchdog Callback Test**
   - Sets 3-second timeout with callback
   - Waits without feeding
   - Tests callback trigger before reset

### Interactive Shell Commands

#### RTC Commands
```bash
# Get current RTC time
rtc get

# Set RTC time (year month day hour minute second)
rtc set 2024 8 4 14 30 0
```

#### Watchdog Commands
```bash
# Start watchdog with default 5-second timeout
wdt start

# Start watchdog with custom timeout (in milliseconds)
wdt start 10000

# Feed the watchdog to prevent timeout
wdt feed

# Stop/disable the watchdog
wdt stop
```

#### Test Commands
```bash
# Run all automatic tests
test
```

## Expected Output

### Successful Test Run
```
ELAN EM32F967 RTC & Watchdog Test Application
============================================
RTC device ready
Watchdog device ready

Running automatic test suite...

=== ELAN EM32F967 RTC & Watchdog Test Suite ===

=== RTC Basic Test ===
Setting RTC time to: 2024-08-04 12:30:45
Read RTC time: 2024-08-04 12:30:46
RTC basic test PASSED

=== RTC Alarm Test ===
Supported alarm fields: 0x0007
Setting alarm for: 12:30:51
Waiting for alarm...
RTC Alarm triggered! ID: 0
RTC alarm test PASSED

=== Watchdog Basic Test ===
Installing watchdog timeout: 5000 ms
Watchdog channel ID: 0
Watchdog started successfully
Feeding watchdog (iteration 1)
Feeding watchdog (iteration 2)
Feeding watchdog (iteration 3)
Watchdog basic test PASSED

=== Watchdog Callback Test ===
Installing watchdog with callback, timeout: 3000 ms
Waiting for watchdog callback (not feeding)...
Watchdog callback triggered! Channel: 0
Watchdog callback test PASSED

=== ALL TESTS PASSED ===

Shell commands available:
  rtc get                        - Get current RTC time
  rtc set <y> <m> <d> <h> <m> <s> - Set RTC time
  wdt start [timeout_ms]         - Start watchdog
  wdt feed                       - Feed watchdog
  wdt stop                       - Stop watchdog
  test                           - Run all tests

Type 'help' for more commands
uart:~$
```

## Troubleshooting

### Common Issues

1. **RTC not working**
   - Check if 32.768kHz crystal is properly connected
   - Verify RTC power supply
   - Check device tree configuration

2. **Watchdog reset occurring**
   - This is expected behavior if watchdog is not fed
   - Use `wdt feed` command to prevent reset
   - Adjust timeout values if needed

3. **Shell not responding**
   - Check UART connection and baud rate
   - Verify console configuration in prj.conf

### Debug Information
Enable additional debugging by modifying `prj.conf`:
```bash
CONFIG_LOG_LEVEL_DBG=y
CONFIG_RTC_LOG_LEVEL_DBG=y
CONFIG_WDT_LOG_LEVEL_DBG=y
```

## Hardware Configuration

### RTC Configuration
- Base Address: 0x40032000
- Clock Source: 32.768kHz crystal or internal oscillator
- Interrupt: IRQ 21
- Power: Always-on domain

### Watchdog Configuration
- Base Address: 0x40035000
- Clock Source: APB clock
- Interrupt: IRQ 20
- Timeout Range: 1ms to ~134 seconds (at 32MHz)

## Device Tree Nodes

The application uses the following device tree nodes:
```dts
rtc0: rtc@40032000 {
    compatible = "elan,em32f902-rtc";
    reg = <0x40032000 0x1000>;
    clocks = <&clk_apb 0>;
    interrupts = <21 0>;
    clock-frequency = <32768>;
    status = "okay";
};

wdt0: watchdog@40035000 {
    compatible = "elan,em32f902-wdt";
    reg = <0x40035000 0x1000>;
    clocks = <&clk_apb 0>;
    interrupts = <20 0>;
    timeout-sec = <30>;
    status = "okay";
};
```

## Driver Features Tested

### RTC Driver (rtc_em32f902.c)
- ✅ Time setting and getting
- ✅ Alarm configuration
- ✅ Alarm callbacks
- ✅ Interrupt handling
- ✅ Field mask support

### Watchdog Driver (wdt_em32f902.c)
- ✅ Timeout configuration
- ✅ Feed operation
- ✅ Reset on timeout
- ✅ Callback before reset
- ✅ Channel management
- ✅ Lock/unlock mechanism

## Performance Notes

- RTC accuracy depends on crystal frequency stability
- Watchdog resolution is limited by APB clock frequency
- Shell commands have minimal impact on real-time operations
- Interrupt latency is typically < 10μs

## Extension Ideas

1. **Power Management**: Test RTC operation in low-power modes
2. **Calendar Functions**: Add date calculation and validation
3. **Multiple Alarms**: Test multiple concurrent alarms
4. **Watchdog Windows**: Test window watchdog functionality
5. **Stress Testing**: Long-duration reliability tests
