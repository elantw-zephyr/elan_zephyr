.. zephyr:code-sample:: elan-gpio-pinctrl-spi-test
   :name: ELAN GPIO/Pinctrl/SPI2 Test Application
   :relevant-api: gpio_interface spi_interface pinctrl_interface

   Comprehensive test application for EM32F967 GPIO, Pinctrl, and SPI2 functionality.

Overview
********

This sample application provides comprehensive testing for the ELAN EM32F967 development board's
GPIO, Pinctrl, and SPI2 functionality. It performs the following tests:

1. **GPIO Port Readiness Test**: Verifies that GPIO ports PA and PB are properly initialized and functional
2. **Pinctrl Functionality Test**: Tests pin multiplexing and configuration capabilities
3. **SPI2 Loopback Test**: Tests SPI2 interface with MOSI to MISO loopback verification

Features Tested
***************

GPIO Testing
============

* GPIO Port A (PA) readiness and basic I/O operations
* GPIO Port B (PB) readiness and basic I/O operations  
* Pin configuration as input/output
* Pin state reading and writing
* Pull-up/pull-down configuration

Pinctrl Testing
===============

* Button pin configuration (PA6 with pull-up)
* LED pin configuration (PB8 as output)
* Pin multiplexing functionality
* Hardware-specific pin control features

SPI2 Testing
============

* SPI2 device readiness verification
* SPI2 pin configuration (PB4-PB7)
* Manual MOSI to MISO loopback test
* SPI transaction functionality
* Data integrity verification

Hardware Requirements
*********************

Board Support
=============

* ELAN EM32F967 Development Board (32f967_dv)
* GPIO ports PA and PB must be available
* SPI2 interface must be configured

Pin Connections
===============

The application uses the following pins:

**GPIO Test Pins:**
* PA0: Test output pin
* PA6: Button input (with internal pull-up)
* PB0: Test output pin  
* PB8: LED output

**SPI2 Pins:**
* PB4: SPI2 Chip Select (CS)
* PB5: SPI2 Serial Clock (SCK)
* PB6: SPI2 Master In Slave Out (MISO)
* PB7: SPI2 Master Out Slave In (MOSI)

**Optional Loopback Connection:**
For complete SPI2 loopback testing, physically connect:
* PB7 (MOSI) to PB6 (MISO)

This connection allows verification that data written to MOSI is correctly received on MISO.

Building and Running
********************

Building
========

To build the sample application:

.. zephyr-app-commands::
   :zephyr-app: samples/elan_gpio_pinctrl_spi_test
   :board: 32f967_dv
   :goals: build
   :compact:

Or using west directly:

.. code-block:: console

   cd zephyrproject/elan-zephyr
   west build -b 32f967_dv -p always ./samples/elan_gpio_pinctrl_spi_test

Flashing
========

Flash the application to the board:

.. code-block:: console

   west flash

Running
=======

After flashing, the application will automatically start and display test results
via the console UART (typically UART0 on PA1/PA2 at 115200 baud).

Expected Output
***************

Successful Test Run
===================

When all tests pass, you should see output similar to:

.. code-block:: console

   =====================================
   ELAN GPIO/Pinctrl/SPI2 Test Application
   =====================================
   Board: EM32F967 Development Board
   Testing: GPIO PA/PB, Pinctrl, SPI2
   =====================================

   === Testing GPIO PA Readiness ===
   ✓ GPIO PA device is ready
   ✓ GPIO PA pin 0 configured as output
   ✓ GPIO PA pin 0 toggle test passed
   ✓ GPIO PA pin 0 configured as input

   === Testing GPIO PB Readiness ===
   ✓ GPIO PB device is ready
   ✓ GPIO PB pin 0 configured as output
   ✓ GPIO PB pin 0 toggle test passed
   ✓ GPIO PB pin 0 configured as input

   === Testing Pinctrl Functionality ===
   ✓ PA6 button pin configured with pull-up
   ✓ PA6 button state: Released (raw value: 1)
   ✓ PB8 LED pin configured as output
   ✓ PB8 LED blink test completed

   === Configuring SPI2 Pins for Loopback Test ===
   ✓ PB6 (MISO) configured as input with pull-up
   ✓ PB7 (MOSI) configured as output

   === Testing Manual MOSI to MISO Loopback ===
   NOTE: For this test to pass, physically connect PB7 (MOSI) to PB6 (MISO)
   MOSI=HIGH, MISO=HIGH (expected: HIGH if loopback connected)
   MOSI=LOW, MISO=LOW (expected: LOW if loopback connected)
   ✓ Manual MOSI to MISO loopback test completed

   === Testing SPI2 Device Functionality ===
   ✓ SPI2 device is ready
   Performing SPI2 transceive test...
   TX data: 0x55 0xAA 0x33 0xCC 0x0F 0xF0 0x5A 0xA5
   ✓ SPI2 transceive completed successfully
   RX data: 0x55 0xAA 0x33 0xCC 0x0F 0xF0 0x5A 0xA5
   ✓ Perfect loopback detected - MOSI to MISO connection verified!

   =====================================
   ✓ ALL TESTS COMPLETED SUCCESSFULLY!
   =====================================
   Summary:
   - GPIO PA: Ready and functional
   - GPIO PB: Ready and functional
   - Pinctrl: Functional (button, LED tested)
   - SPI2: Ready and functional
   - MOSI/MISO: Tested (connect PB7 to PB6 for loopback)
   =====================================

Troubleshooting
***************

Common Issues
=============

**GPIO Device Not Ready:**
* Check that GPIO drivers are enabled in prj.conf
* Verify device tree configuration includes GPIO nodes
* Ensure clocks are properly configured

**SPI2 Device Not Ready:**
* Verify SPI2 is enabled in device tree
* Check pinctrl configuration for SPI2 pins
* Ensure SPI driver is enabled in prj.conf

**Loopback Test Fails:**
* Verify physical connection between PB7 (MOSI) and PB6 (MISO)
* Check that pins are not conflicting with other functions
* Ensure proper signal levels and timing

**Button/LED Tests Fail:**
* Check physical connections to PA6 (button) and PB8 (LED)
* Verify pull-up/pull-down configurations
* Ensure pins are not used by other peripherals

Debug Options
=============

Enable additional debugging by modifying prj.conf:

.. code-block:: kconfig

   CONFIG_GPIO_LOG_LEVEL_DBG=y
   CONFIG_SPI_LOG_LEVEL_DBG=y
   CONFIG_PINCTRL_LOG_LEVEL_DBG=y
   CONFIG_LOG_MODE_IMMEDIATE=y

This will provide detailed driver-level debugging information.

References
**********

* :ref:`gpio_api`
* :ref:`spi_api`  
* :ref:`pinctrl_guide`
* ELAN EM32F967 Development Board Documentation
* EM32F967 Complete Specification
