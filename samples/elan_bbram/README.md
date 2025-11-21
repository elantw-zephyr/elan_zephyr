# EM32 BBRAM Test Application

This comprehensive test application validates the Battery-Backed RAM (BBRAM) functionality of the EM32 microcontroller. It exercises all aspects of the BBRAM driver including basic operations, power management, data integrity, and persistence across resets.

## Overview

The EM32 provides 64 bytes (16 × 32-bit registers) of backup registers at address `0x40033000` that retain data when powered by backup battery (VBAT). This test suite validates the BBRAM driver implementation and demonstrates practical usage patterns.

## Features Tested

### 1. Basic Functionality
- **Read/Write Operations**: 32-bit word and byte-level access
- **Size Validation**: Confirms 60 bytes usable space (4 bytes reserved for status)
- **Boundary Conditions**: Tests address limits and out-of-bounds rejection

### 2. Power Management
- **Write Protection**: STM32-style backup domain write access control
- **Power Status Monitoring**: Main power, standby power, and data validity checks
- **Reset Detection**: Backup domain reset flag handling

### 3. Data Integrity
- **Pattern Testing**: Multiple test patterns including edge cases
- **Checksum Validation**: CRC32 verification of critical data
- **Stress Testing**: Random data with multiple read/write cycles

### 4. Persistence Features
- **Boot Counter**: Tracks system restarts across power cycles
- **Event Logging**: Circular buffer for system events
- **Configuration Storage**: Calibration data and system settings

## Hardware Requirements

- **Board**: EM32 Development Board (32f967_dv)
- **Power**: VBAT backup power connection (optional, for full persistence testing)
- **Console**: UART console for test output monitoring

## Build Instructions

```bash
# Navigate to BBRAM test directory
cd samples/elan_bbram

# Build for EM32 target
west build -b 32f967_dv -p always

# Flash to device
west flash

# Monitor console output
west attach
```

## Expected Output

```
=========================================
   EM32 BBRAM Test Suite v1.0
   Build: Aug 13 2025 15:30:00
=========================================
BBRAM device ready: bbram@40033000

=== BBRAM Size Validation Test ===
BBRAM size: 60 bytes
Size validation: PASSED

=== BBRAM Power Status Test ===
BBRAM data validity: VALID
BBRAM standby power: NORMAL
BBRAM main power: NORMAL
Power status check: COMPLETED

=== BBRAM Data Persistence Test ===
First boot or invalid data, initializing counter
Boot count updated to: 1
Test run count updated to: 1

=== BBRAM Basic Read/Write Test ===
32-bit R/W test: PASSED (0xDEADBEEF)
Byte-level R/W test: PASSED

=== BBRAM Boundary Condition Test ===
Boundary write/read: PASSED
Out-of-bounds write rejection: PASSED
Out-of-bounds read rejection: PASSED

=== BBRAM Data Integrity Test ===
Pattern 0 (0xDEADBEEF): PASSED
Pattern 1 (0xCAFEBABE): PASSED
Pattern 2 (0x55AA55AA): PASSED
Pattern 3 (0x00000000): PASSED
Pattern 4 (0xFFFFFFFF): PASSED
Pattern 5 (0x55555555): PASSED
Pattern 6 (0xAAAAAAAA): PASSED

=== BBRAM Stress Test ===
Stress test cycle 0: PASSED
Stress test cycle 1: PASSED
...
Stress test cycle 9: PASSED

=== All BBRAM Tests COMPLETED SUCCESSFULLY ===
Test run #1 completed at 2847 ms uptime

=== BBRAM Event Log ===
Total events logged: 3
Event #1: ID=0x0001, data=0x0000, time=234 ms
Event #2: ID=0x0002, data=0x0001, time=1456 ms
Event #3: ID=0x0003, data=0x0001, time=2847 ms

Entering monitoring mode...
BBRAM monitoring... uptime: 12847 ms, run: #1
```

## Test Sequence Details

### 1. Initialization Phase
- Device readiness verification
- BBRAM size detection and validation
- Power domain status checking
- Event logging initialization

### 2. Persistence Testing
- Boot counter recovery from previous sessions
- Data integrity verification with checksums
- Configuration data persistence validation

### 3. Functional Testing
- Basic read/write operations (32-bit and byte-level)
- Boundary condition testing
- Data pattern validation
- Random data stress testing

### 4. Monitoring Phase
- Continuous power status monitoring
- Periodic data integrity checks
- Event logging for anomalies

## Data Layout in BBRAM

```
Offset  Size  Description
------  ----  -----------
0x00    16    Test Header (magic, counter, checksum, timestamp)
0x10    16    Configuration Data (boot count, errors, calibration)
0x20    32    Event Log (8 × 4-byte entries, circular buffer)
0x40    8     Reserved for additional data
0x48    4     Log Index Counter
0x4C    4     Available for user data
```

## Power Cycle Testing

To test data persistence across power cycles:

1. **Run Initial Test**: Flash and run the application once
2. **Power Cycle**: Remove power completely, then restore
3. **Verify Persistence**: Check that boot counter increments and data is retained
4. **VBAT Testing**: Connect backup battery and test persistence during main power loss

## Event IDs Reference

| Event ID | Description | Data Field |
|----------|-------------|------------|
| 0x0001   | System Boot | 0x0000 |
| 0x0002   | Test Start  | Test run number |
| 0x0003   | Test Complete | Test run number |
| 0x0004   | Power Failure | Failure type (1=standby, 2=main) |
| 0x0005   | Data Corruption | Error code |

## Configuration Options

### Project Configuration (prj.conf)
- `CONFIG_BBRAM=y`: Enable BBRAM subsystem
- `CONFIG_BBRAM_EM32=y`: Enable EM32 BBRAM driver
- `CONFIG_BBRAM_LOG_LEVEL_DBG=y`: Enable debug logging
- `CONFIG_CRC=y`: Enable CRC for data validation
- `CONFIG_TEST_RANDOM_GENERATOR=y`: Enable random number generation

### Device Tree
The BBRAM device is automatically enabled in the board device tree:
```dts
&bbram0 {
    status = "okay";
};
```

## Troubleshooting

### Common Issues

1. **Device Not Ready**
   - Check device tree configuration
   - Verify BBRAM driver is enabled in Kconfig
   - Ensure proper clock configuration

2. **Write Failures**
   - Backup domain write protection may be active
   - Check power management register access
   - Verify sufficient stack size for operations

3. **Data Corruption**
   - Check VBAT power connection
   - Verify backup domain reset handling
   - Monitor for power supply issues

### Debug Output

Enable debug logging to see detailed driver operation:
```
CONFIG_BBRAM_LOG_LEVEL_DBG=y
CONFIG_LOG=y
```

## Driver Architecture

The BBRAM driver follows STM32-style backup domain patterns:
- **Write Protection**: Automatic backup domain write enable/disable
- **Power Management**: Integration with PWR and RCC registers  
- **Status Tracking**: Magic number and flags for data validity
- **Memory Layout**: 4-byte status header + 60 bytes user data

## Performance Characteristics

- **Read Latency**: ~1μs (direct register access)
- **Write Latency**: ~10μs (including write protection handling)
- **Data Retention**: Indefinite with VBAT power
- **Power Consumption**: <1μA in backup mode

## Related Documentation

- [EM32 BBRAM Implementation Analysis](../../ai_doc/0813_1507_bbram_em32f967_Implementation_Analysis.md)
- [GPIO_IPType Analysis Report](../../ai_doc/GPIO_IPType_Analysis_Report.md)
- [BBRAM Functionality Analysis Report](../../ai_doc/0813_1440_BBRAM_Functionality_Analysis_Report.md)
