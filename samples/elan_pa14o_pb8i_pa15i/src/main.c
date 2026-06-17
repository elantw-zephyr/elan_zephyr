/*
 * Copyright (c) 2026 ELAN Microelectronics Corp.
 * SPDX-License-Identifier: Apache-2.0
 *
 * EM32F967 GPIO + SPI2 loopback sample
 *
 * GPIO (DTS "zephyr,user" method):
 *   - PA14 is configured as an output and toggled 1/0 every second.
 *   - PB8 and PA15 are configured as inputs; their levels are read and printed
 *     to the console every second.
 *
 * SPI2 loopback (PB4-PB7):
 *   - PB4 (CS)   : SPI2_CS   (GPIO-controlled chip-select, see cs-gpios)
 *   - PB5 (SCK)  : SPI2_SCK  (pinctrl AF2)
 *   - PB6 (MISO) : SPI2_MISO (pinctrl AF2)
 *   - PB7 (MOSI) : SPI2_MOSI (pinctrl AF2)
 *   - A fixed pattern is sent every second; with MOSI (PB7) wired to MISO
 *     (PB6) the received bytes should match the transmitted bytes. The
 *     PASS/FAIL result is printed to the console once per second.
 *
 * The spi2 node, its pinctrl group and cs-gpios all come from the device tree
 * (board DTS + 32f967_dv_priv.dtsi); no register address is hard-coded in C.
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/printk.h>

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)
#define SPI2_NODE        DT_CHOSEN(zephyr_spi2)

/* Fail the build early if spi2 is not enabled in the device tree. */
BUILD_ASSERT(DT_NODE_HAS_STATUS(SPI2_NODE, okay), "spi2 node is not enabled");

/* PA14 = output (toggled); PB8 = in[0], PA15 = in[1] (read) */
static const struct gpio_dt_spec out_pa14 =
	GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, out_gpios);
static const struct gpio_dt_spec in_pb8 =
	GPIO_DT_SPEC_GET_BY_IDX(ZEPHYR_USER_NODE, in_gpios, 0);
static const struct gpio_dt_spec in_pa15 =
	GPIO_DT_SPEC_GET_BY_IDX(ZEPHYR_USER_NODE, in_gpios, 1);

/* SPI2 device + GPIO chip-select (PB4) taken from the device tree. */
static const struct device *const spi2_dev = DEVICE_DT_GET(SPI2_NODE);

static const struct spi_config spi2_cfg = {
	.frequency = 1000000U, /* 1 MHz */
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
	.slave = 0,
	.cs = {
		.gpio = GPIO_DT_SPEC_GET(SPI2_NODE, cs_gpios),
		.delay = 0,
	},
};

/* Initial data sent on MOSI/MISO for the loopback test. */
#define SPI_TEST_BUF_SIZE 8
static const uint8_t spi_tx_buf[SPI_TEST_BUF_SIZE] = {0x55, 0xAA, 0x33, 0xCC,
						     0x0F, 0xF0, 0x5A, 0xA5};
static uint8_t spi_rx_buf[SPI_TEST_BUF_SIZE];

static void describe_spi2_pins(void)
{
	printk("\n=== Configuring SPI2 Pins for Loopback Test ===\n");
	printk("SPI2 pins (PB4-PB7) are configured by pinctrl system as AF2 (SPI function)\n");
	printk("PB4 (CS): Configured as SPI2_CS by pinctrl\n");
	printk("PB5 (SCK): Configured as SPI2_SCK by pinctrl\n");
	printk("PB6 (MISO): Configured as SPI2_MISO by pinctrl\n");
	printk("PB7 (MOSI): Configured as SPI2_MOSI by pinctrl\n");
	printk("INFO: For loopback, wire PB7 (MOSI) to PB6 (MISO)\n");
}

/*
 * Run one SPI2 transceive of the test pattern and report whether the received
 * bytes match the transmitted bytes (i.e. MOSI looped back to MISO).
 * Returns 0 on a matching loopback, 1 on mismatch, negative on transfer error.
 */
static int spi2_loopback_once(void)
{
	const struct spi_buf tx = {.buf = (void *)spi_tx_buf, .len = SPI_TEST_BUF_SIZE};
	const struct spi_buf rx = {.buf = spi_rx_buf, .len = SPI_TEST_BUF_SIZE};
	const struct spi_buf_set tx_set = {.buffers = &tx, .count = 1};
	const struct spi_buf_set rx_set = {.buffers = &rx, .count = 1};
	int ret;

	memset(spi_rx_buf, 0, sizeof(spi_rx_buf));

	ret = spi_transceive(spi2_dev, &spi2_cfg, &tx_set, &rx_set);
	if (ret < 0) {
		printk("SPI2 loopback: transceive error %d\n", ret);
		return ret;
	}

	printk("SPI2 TX:");
	for (int i = 0; i < SPI_TEST_BUF_SIZE; i++) {
		printk(" 0x%02X", spi_tx_buf[i]);
	}
	printk(" | RX:");
	for (int i = 0; i < SPI_TEST_BUF_SIZE; i++) {
		printk(" 0x%02X", spi_rx_buf[i]);
	}

	if (memcmp(spi_tx_buf, spi_rx_buf, SPI_TEST_BUF_SIZE) == 0) {
		printk(" | RESULT: PASS (loopback OK)\n");
		return 0;
	}

	printk(" | RESULT: FAIL (RX != TX)\n");
	return 1;
}

int main(void)
{
	int ret;
	int level = 0;

	printk("EM32F967 GPIO + SPI2 loopback sample\n");
	printk("=========================================================\n");

	/* Verify all GPIO controllers and the SPI2 device are ready. */
	if (!gpio_is_ready_dt(&out_pa14) || !gpio_is_ready_dt(&in_pb8) ||
	    !gpio_is_ready_dt(&in_pa15)) {
		printk("Error: a GPIO controller is not ready\n");
		return -ENODEV;
	}

	if (!device_is_ready(spi2_dev)) {
		printk("Error: SPI2 device not ready\n");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&spi2_cfg.cs.gpio)) {
		printk("Error: SPI2 CS GPIO (PB4) not ready\n");
		return -ENODEV;
	}

	/* PA14 as output, start low. */
	ret = gpio_pin_configure_dt(&out_pa14, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		printk("Error %d: failed to configure PA14 (%s pin %d) as output\n", ret,
		       out_pa14.port->name, out_pa14.pin);
		return ret;
	}

	/* PB8 and PA15 as inputs. */
	ret = gpio_pin_configure_dt(&in_pb8, GPIO_INPUT);
	if (ret < 0) {
		printk("Error %d: failed to configure PB8 (%s pin %d) as input\n", ret,
		       in_pb8.port->name, in_pb8.pin);
		return ret;
	}

	ret = gpio_pin_configure_dt(&in_pa15, GPIO_INPUT);
	if (ret < 0) {
		printk("Error %d: failed to configure PA15 (%s pin %d) as input\n", ret,
		       in_pa15.port->name, in_pa15.pin);
		return ret;
	}

	printk("Setup OK: PA14=%s.%d (out), PB8=%s.%d (in), PA15=%s.%d (in)\n",
	       out_pa14.port->name, out_pa14.pin, in_pb8.port->name, in_pb8.pin,
	       in_pa15.port->name, in_pa15.pin);

	describe_spi2_pins();
	printk("SPI2 ready: %s, CS=%s.%d, %u Hz, 8-bit MSB-first\n\n", spi2_dev->name,
	       spi2_cfg.cs.gpio.port->name, spi2_cfg.cs.gpio.pin, spi2_cfg.frequency);

	while (1) {
		/* 1) Toggle PA14. */
		level = !level;
		ret = gpio_pin_set_dt(&out_pa14, level);
		if (ret < 0) {
			printk("Error %d: failed to set PA14\n", ret);
			break;
		}

		/* 2) Read the two inputs. */
		int v_pb8 = gpio_pin_get_dt(&in_pb8);
		int v_pa15 = gpio_pin_get_dt(&in_pa15);

		if (v_pb8 < 0 || v_pa15 < 0) {
			printk("Error reading inputs (PB8=%d, PA15=%d)\n", v_pb8, v_pa15);
			break;
		}

		printk("PA14(out)=%d | PB8(in)=%d | PA15(in)=%d\n", level, v_pb8, v_pa15);

		/* 3) Run one SPI2 loopback transaction and print the result. */
		(void)spi2_loopback_once();

		k_msleep(1000);
	}

	return 0;
}
