/*
 * Copyright (c) 2026 ELAN Microelectronics Corp.
 * SPDX-License-Identifier: Apache-2.0
 *
 * EM32F967 HP Gaston merged sample (32f967_dv).
 *
 * Merges three samples into one application:
 *   1. elan_pa14o_pb8i_pa15i (+ SPI2 loopback)
 *      - PA14 output toggled 1/0 every second
 *      - PB8 / PA15 inputs, read every second
 *      - SPI2 loopback on PB4-PB7 (wire PB7 MOSI -> PB6 MISO)
 *   2. elan_pwm_pa3_hd
 *      - PWMB_N on PA3, 1kHz, duty ramps 0->100->0; high-drive via DTS
 *   3. elan_pb3i_pb9o
 *      - PB3 input (strong PU1, low drive), PB9 output (LOW, high drive)
 *
 * All pins, the SPI2 node, PWM controller, pull and drive strengths come from
 * the device tree (board DTS + 32f967_dv_priv.dtsi + overlay); no register
 * address is hard-coded in C.
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/printk.h>

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)
#define SPI2_NODE        DT_CHOSEN(zephyr_spi2)
#define PWM_NODE         DT_NODELABEL(pwm0)

/* Fail the build early if spi2 is not enabled in the device tree. */
BUILD_ASSERT(DT_NODE_HAS_STATUS(SPI2_NODE, okay), "spi2 node is not enabled");

/* Pinctrl config (PB3 PU1 + PB9 high-drive + GPIO mux). */
PINCTRL_DT_DEFINE(ZEPHYR_USER_NODE);
static const struct pinctrl_dev_config *pin_cfg =
	PINCTRL_DT_DEV_CONFIG_GET(ZEPHYR_USER_NODE);

/* Outputs: PA14 (idx0, toggled), PB9 (idx1, driven low). */
static const struct gpio_dt_spec out_pa14 =
	GPIO_DT_SPEC_GET_BY_IDX(ZEPHYR_USER_NODE, out_gpios, 0);
static const struct gpio_dt_spec out_pb9 =
	GPIO_DT_SPEC_GET_BY_IDX(ZEPHYR_USER_NODE, out_gpios, 1);

/* Inputs: PB8 (idx0), PA15 (idx1), PB3 (idx2). */
static const struct gpio_dt_spec in_pb8 =
	GPIO_DT_SPEC_GET_BY_IDX(ZEPHYR_USER_NODE, in_gpios, 0);
static const struct gpio_dt_spec in_pa15 =
	GPIO_DT_SPEC_GET_BY_IDX(ZEPHYR_USER_NODE, in_gpios, 1);
static const struct gpio_dt_spec in_pb3 =
	GPIO_DT_SPEC_GET_BY_IDX(ZEPHYR_USER_NODE, in_gpios, 2);

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

/* PWMB_N on PA3: 1kHz, duty ramps 0% -> 100% -> 0% (high-drive via DTS). */
#define PWM_NODE_DEV    DEVICE_DT_GET(PWM_NODE)
#define PWM_PERIOD_NS   1000000U  /* 1ms period = 1kHz frequency */
#define PWM_STEP_NS     50000U    /* 50us step (5% per second) */
#define PWM_CHANNEL_B   1         /* Channel B (index 1) -> PWMB_N on PA3 */

static const struct device *const pwm_dev = PWM_NODE_DEV;
static uint32_t pwm_pulse_ns;
static bool pwm_increasing = true;

static void describe_spi2_pins(void)
{
	printk("\n=== SPI2 Pins (PB4-PB7) ===\n");
	printk("PB4 (CS): SPI2_CS  PB5 (SCK): SPI2_SCK  "
	       "PB6 (MISO): SPI2_MISO  PB7 (MOSI): SPI2_MOSI\n");
	printk("INFO: For loopback, wire PB7 (MOSI) to PB6 (MISO)\n");
}

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

static void pwm_ramp_once(void)
{
	if (pwm_set(pwm_dev, PWM_CHANNEL_B, PWM_PERIOD_NS, pwm_pulse_ns, 0) < 0) {
		printk("PWM: failed to update duty\n");
		return;
	}

	if (pwm_increasing) {
		pwm_pulse_ns += PWM_STEP_NS;
		if (pwm_pulse_ns >= PWM_PERIOD_NS) {
			pwm_pulse_ns = PWM_PERIOD_NS;
			pwm_increasing = false;
		}
	} else {
		if (pwm_pulse_ns >= PWM_STEP_NS) {
			pwm_pulse_ns -= PWM_STEP_NS;
		} else {
			pwm_pulse_ns = 0;
			pwm_increasing = true;
		}
	}
}

int main(void)
{
	int ret;
	int level = 0;

	printk("EM32F967 HP Gaston merged sample\n");
	printk("=========================================================\n");

	/* Verify GPIO controllers, SPI2 and PWM are ready. */
	if (!gpio_is_ready_dt(&out_pa14) || !gpio_is_ready_dt(&out_pb9) ||
	    !gpio_is_ready_dt(&in_pb8) || !gpio_is_ready_dt(&in_pa15) ||
	    !gpio_is_ready_dt(&in_pb3)) {
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

	if (!device_is_ready(pwm_dev)) {
		printk("Error: PWM device not ready\n");
		return -ENODEV;
	}

	/* PA14 output, start low; PB9 output, driven low. */
	ret = gpio_pin_configure_dt(&out_pa14, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		printk("Error %d: failed to configure PA14 as output\n", ret);
		return ret;
	}

	ret = gpio_pin_configure_dt(&out_pb9, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		printk("Error %d: failed to configure PB9 as output\n", ret);
		return ret;
	}

	/* PB8, PA15 and PB3 inputs. */
	ret = gpio_pin_configure_dt(&in_pb8, GPIO_INPUT);
	if (ret < 0) {
		printk("Error %d: failed to configure PB8 as input\n", ret);
		return ret;
	}

	ret = gpio_pin_configure_dt(&in_pa15, GPIO_INPUT);
	if (ret < 0) {
		printk("Error %d: failed to configure PA15 as input\n", ret);
		return ret;
	}

	ret = gpio_pin_configure_dt(&in_pb3, GPIO_INPUT);
	if (ret < 0) {
		printk("Error %d: failed to configure PB3 as input\n", ret);
		return ret;
	}

	/* Apply pull (PB3 PU1) + drive (PB9 high) via pinctrl after GPIO config. */
	ret = pinctrl_apply_state(pin_cfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		printk("Error %d: failed to apply pinctrl state\n", ret);
		return ret;
	}

	ret = gpio_pin_set_dt(&out_pb9, 0);
	if (ret < 0) {
		printk("Error %d: failed to drive PB9 low\n", ret);
		return ret;
	}

	printk("Setup OK:\n");
	printk("  PA14=%s.%d (out)  PB9=%s.%d (out,low,HD)\n",
	       out_pa14.port->name, out_pa14.pin, out_pb9.port->name, out_pb9.pin);
	printk("  PB8=%s.%d (in)  PA15=%s.%d (in)  PB3=%s.%d (in,PU1)\n",
	       in_pb8.port->name, in_pb8.pin, in_pa15.port->name, in_pa15.pin,
	       in_pb3.port->name, in_pb3.pin);

	describe_spi2_pins();
	printk("SPI2 ready: %s, CS=%s.%d, %u Hz, 8-bit MSB-first\n",
	       spi2_dev->name, spi2_cfg.cs.gpio.port->name, spi2_cfg.cs.gpio.pin,
	       spi2_cfg.frequency);

	/* Start PWMB_N at 50% duty so PA3 shows a clear 1kHz signal, then ramp. */
	pwm_pulse_ns = PWM_PERIOD_NS / 2;
	if (pwm_set(pwm_dev, PWM_CHANNEL_B, PWM_PERIOD_NS, pwm_pulse_ns, 0) < 0) {
		printk("PWM: failed to set initial 50%% duty on PA3\n");
	}
	printk("PWMB_N on PA3 ready: 1kHz, 50%% start, high-drive via DTS\n\n");

	while (1) {
		/* 1) Toggle PA14. */
		level = !level;
		ret = gpio_pin_set_dt(&out_pa14, level);
		if (ret < 0) {
			printk("Error %d: failed to set PA14\n", ret);
			break;
		}

		/* 2) Read inputs. */
		int v_pb8 = gpio_pin_get_dt(&in_pb8);
		int v_pa15 = gpio_pin_get_dt(&in_pa15);
		int v_pb3 = gpio_pin_get_dt(&in_pb3);
		int v_pb9 = gpio_pin_get_dt(&out_pb9);

		printk("PA14(out)=%d | PB8=%d | PA15=%d | PB3(PU1)=%d | PB9(out,low,HD)=%d\n",
		       level, v_pb8, v_pa15, v_pb3, v_pb9);

		/* 3) SPI2 loopback. */
		(void)spi2_loopback_once();

		/* 4) PWMB_N duty ramp on PA3. */
		pwm_ramp_once();

		k_msleep(1000);
	}

	return 0;
}
