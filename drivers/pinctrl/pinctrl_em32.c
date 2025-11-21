/*
 * Copyright (c) 2025 Elan Microelectronics Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief EM32F967 Pin Control Driver
 *
 * This driver provides pin multiplexing and configuration support for the
 * EM32F967 microcontroller, following STM32-style design patterns for
 * clean and maintainable code.
 */

#include <zephyr/init.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
#include "../../../include/zephyr/dt-bindings/pinctrl/em32f967-pinctrl.h"
#include "../../../include/zephyr/drivers/pinctrl/pinctrl_em32.h"
#include <soc.h>

LOG_MODULE_REGISTER(pinctrl_em32, LOG_LEVEL_INF);

/* ============================================================================
 * Hardware Register Definitions
 * ============================================================================ */

/* Base addresses */
#define EM32_PINCTRL_BASE 0x40030200U
#define EM32_IOSHARE_BASE 0x4003023CU

/* IOMUX register offsets */
#define EM32_IOMUX_PA_LOW_OFFSET  0x00U /* PA[7:0] multiplexing */
#define EM32_IOMUX_PA_HIGH_OFFSET 0x04U /* PA[15:8] multiplexing */
#define EM32_IOMUX_PB_LOW_OFFSET  0x08U /* PB[7:0] multiplexing */
#define EM32_IOMUX_PB_HIGH_OFFSET 0x0CU /* PB[15:8] multiplexing */

/* Pull-up/Pull-down register offsets */
#define EM32_PUPD_PA_OFFSET 0x14U /* PA pull control */
#define EM32_PUPD_PB_OFFSET 0x18U /* PB pull control */

/* Open-drain register offsets */
#define EM32_OD_PA_OFFSET 0x2CU /* PA open-drain control */
#define EM32_OD_PB_OFFSET 0x30U /* PB open-drain control */

/* ============================================================================
 * Hardware Constants
 * ============================================================================ */

/* Pin configuration limits */
#define EM32_MAX_PORTS          2U
#define EM32_MAX_PINS_PER_PORT  16U
#define EM32_PINS_PER_IOMUX_REG 8U
#define EM32_MAX_ALT_FUNC       7U

/* Bit field sizes */
#define EM32_IOMUX_BITS_PER_PIN 4U
#define EM32_PUPD_BITS_PER_PIN  2U
#define EM32_OD_BITS_PER_PIN    1U

/* Port identifiers */
#define EM32_PORT_A 0U
#define EM32_PORT_B 1U

/* ============================================================================
 * Pin Configuration Macros
 * ============================================================================ */

/* Extract pin information from encoded values */
#define EM32_GET_PORT(pin)         (((pin) >> 4) & 0xFU)
#define EM32_GET_PIN_NUM(pin)      ((pin) & 0xFU)
#define EM32_ENCODE_PIN(port, num) (((port) << 4) | (num))

/* Extract device tree pinmux information */
#define EM32_DT_GET_PORT(pinmux) EM32F967_DT_PINMUX_PORT(pinmux)
#define EM32_DT_GET_PIN(pinmux)  EM32F967_DT_PINMUX_PIN(pinmux)
#define EM32_DT_GET_FUNC(pinmux) EM32F967_DT_PINMUX_MUX(pinmux)

/* ============================================================================
 * Register Access Helpers
 * ============================================================================ */

/**
 * @brief Get IOMUX register address for a specific port and pin
 *
 * @param port GPIO port (0=PA, 1=PB)
 * @param pin_num Pin number (0-15)
 * @return Register address or 0 if invalid
 */
static inline uint32_t em32_get_iomux_reg_addr(uint8_t port, uint8_t pin_num)
{
	const uint32_t base = EM32_PINCTRL_BASE;

	if (port == EM32_PORT_A) {
		return base + ((pin_num < EM32_PINS_PER_IOMUX_REG) ? EM32_IOMUX_PA_LOW_OFFSET
								   : EM32_IOMUX_PA_HIGH_OFFSET);
	} else if (port == EM32_PORT_B) {
		return base + ((pin_num < EM32_PINS_PER_IOMUX_REG) ? EM32_IOMUX_PB_LOW_OFFSET
								   : EM32_IOMUX_PB_HIGH_OFFSET);
	}

	return 0;
}

/**
 * @brief Get pull-up/pull-down register address for a specific port
 *
 * @param port GPIO port (0=PA, 1=PB)
 * @return Register address or 0 if invalid
 */
static inline uint32_t em32_get_pupd_reg_addr(uint8_t port)
{
	const uint32_t base = EM32_PINCTRL_BASE;

	switch (port) {
	case EM32_PORT_A:
		return base + EM32_PUPD_PA_OFFSET;
	case EM32_PORT_B:
		return base + EM32_PUPD_PB_OFFSET;
	default:
		return 0;
	}
}

/**
 * @brief Get open-drain register address for a specific port
 *
 * @param port GPIO port (0=PA, 1=PB)
 * @return Register address or 0 if invalid
 */
static inline uint32_t em32_get_od_reg_addr(uint8_t port)
{
	const uint32_t base = EM32_PINCTRL_BASE;

	switch (port) {
	case EM32_PORT_A:
		return base + EM32_OD_PA_OFFSET;
	case EM32_PORT_B:
		return base + EM32_OD_PB_OFFSET;
	default:
		return 0;
	}
}

/* ============================================================================
 * IOShare Configuration (Peripheral Multiplexing)
 * ============================================================================ */

/**
 * @brief IOShare configuration table entry
 */
struct em32_ioshare_config {
	uint8_t port;
	uint8_t pin_start;
	uint8_t pin_end;
	uint32_t alt_func;
	uint32_t bit_pos;
	uint32_t bit_mask;
	uint32_t bit_value;
	const char *peripheral;
};

/* IOShare configuration lookup table */
static const struct em32_ioshare_config em32_ioshare_table[] = {
	/* UART configurations */
	{EM32_PORT_A, 1, 2, EM32F967_AF2, EM32_IP_SHARE_UART1, BIT(EM32_IP_SHARE_UART1),
	 BIT(EM32_IP_SHARE_UART1), "UART1"},
	{EM32_PORT_A, 4, 5, EM32F967_AF2, EM32_IP_SHARE_UART2, BIT(EM32_IP_SHARE_UART2),
	 BIT(EM32_IP_SHARE_UART2), "UART2"},
	{EM32_PORT_B, 8, 9, EM32F967_AF2, EM32_IP_SHARE_UART1, BIT(EM32_IP_SHARE_UART1), 0,
	 "UART1_ALT"},

	/* SPI configurations */
	{EM32_PORT_B, 0, 3, EM32F967_AF1, EM32_IP_SHARE_SPI1_SHIFT,
	 0x3U << EM32_IP_SHARE_SPI1_SHIFT, 0x0U << EM32_IP_SHARE_SPI1_SHIFT, "SPI1"},
	{EM32_PORT_B, 4, 7, EM32F967_AF6, EM32_IP_SHARE_SSP2_SHIFT,
	 0x3U << EM32_IP_SHARE_SSP2_SHIFT, 0x0U << EM32_IP_SHARE_SSP2_SHIFT, "SSP2"},

	/* I2C configurations */
	{EM32_PORT_A, 4, 5, EM32F967_AF4, EM32_IP_SHARE_I2C2, BIT(EM32_IP_SHARE_I2C2), 0, "I2C2"},
	{EM32_PORT_B, 0, 1, EM32F967_AF5, EM32_IP_SHARE_I2C1, BIT(EM32_IP_SHARE_I2C1),
	 BIT(EM32_IP_SHARE_I2C1), "I2C1"},
};

/**
 * @brief Configure IOShare register for peripheral multiplexing
 *
 * @param port GPIO port (0=PA, 1=PB)
 * @param pin_num Pin number (0-15)
 * @param alt_func Alternate function (EM32F967_AF1-AF7)
 * @return 0 on success, negative errno on failure
 */
static int em32_configure_ioshare(uint8_t port, uint8_t pin_num, uint32_t alt_func)
{
	uint32_t ioshare_val;
	bool config_found = false;

	if (port >= EM32_MAX_PORTS) {
		LOG_ERR("Invalid port: %d", port);
		return -EINVAL;
	}

	/* Search for matching IOShare configuration */
	for (size_t i = 0; i < ARRAY_SIZE(em32_ioshare_table); i++) {
		const struct em32_ioshare_config *cfg = &em32_ioshare_table[i];

		if (cfg->port == port && pin_num >= cfg->pin_start && pin_num <= cfg->pin_end &&
		    cfg->alt_func == alt_func) {

			ioshare_val = sys_read32(EM32_IOSHARE_BASE);
			ioshare_val &= ~cfg->bit_mask;
			ioshare_val |= cfg->bit_value;
			sys_write32(ioshare_val, EM32_IOSHARE_BASE);

			LOG_DBG("Configured %s on P%c%d (IOShare: 0x%08X)", cfg->peripheral,
				'A' + port, pin_num, ioshare_val);
			config_found = true;
			break;
		}
	}

	if (!config_found) {
		LOG_DBG("No IOShare config needed for P%c%d AF%d", 'A' + port, pin_num, alt_func);
	}

	return 0;
}

/* ============================================================================
 * Pin Configuration Functions
 * ============================================================================ */

/**
 * @brief Validate pin parameters
 *
 * @param port GPIO port
 * @param pin_num Pin number
 * @param alt_func Alternate function
 * @return 0 if valid, negative errno if invalid
 */
static inline int em32_validate_pin_params(uint8_t port, uint8_t pin_num, uint32_t alt_func)
{
	if (port >= EM32_MAX_PORTS) {
		LOG_ERR("Invalid port: %d (max: %d)", port, EM32_MAX_PORTS - 1);
		return -EINVAL;
	}

	if (pin_num >= EM32_MAX_PINS_PER_PORT) {
		LOG_ERR("Invalid pin: %d (max: %d)", pin_num, EM32_MAX_PINS_PER_PORT - 1);
		return -EINVAL;
	}

	if (alt_func > EM32_MAX_ALT_FUNC) {
		LOG_ERR("Invalid alternate function: %d (max: %d)", alt_func, EM32_MAX_ALT_FUNC);
		return -EINVAL;
	}

	return 0;
}

/**
 * @brief Configure pin multiplexing function
 *
 * @param port GPIO port (0=PA, 1=PB)
 * @param pin_num Pin number (0-15)
 * @param alt_func Alternate function (0=GPIO, 1-7=AF1-AF7)
 * @return 0 on success, negative errno on failure
 */
static int em32_configure_pin_mux(uint8_t port, uint8_t pin_num, uint32_t alt_func)
{
	uint32_t reg_addr;
	uint32_t reg_val;
	uint32_t pin_offset;
	uint32_t mux_mask;

	/* Validate parameters */
	int ret = em32_validate_pin_params(port, pin_num, alt_func);
	if (ret < 0) {
		return ret;
	}

	/* Get IOMUX register address */
	reg_addr = em32_get_iomux_reg_addr(port, pin_num);
	if (reg_addr == 0) {
		LOG_ERR("Failed to get IOMUX register for P%c%d", 'A' + port, pin_num);
		return -EINVAL;
	}

	/* Calculate bit position and mask */
	pin_offset = (pin_num % EM32_PINS_PER_IOMUX_REG) * EM32_IOMUX_BITS_PER_PIN;
	mux_mask = ((1U << EM32_IOMUX_BITS_PER_PIN) - 1) << pin_offset;

	/* Read-modify-write the IOMUX register */
	reg_val = sys_read32(reg_addr);
	reg_val &= ~mux_mask;
	reg_val |= (alt_func << pin_offset);
	sys_write32(reg_val, reg_addr);

	LOG_DBG("P%c%d: mux=AF%d, reg=0x%08X, val=0x%08X", 'A' + port, pin_num, alt_func, reg_addr,
		reg_val);

	return 0;
}

/**
 * @brief Configure pin bias (pull-up/pull-down resistors)
 *
 * @param port GPIO port (0=PA, 1=PB)
 * @param pin_num Pin number (0-15)
 * @param pin_cfg Pin configuration flags from device tree
 * @return 0 on success, negative errno on failure
 */
static int em32_configure_pin_bias(uint8_t port, uint8_t pin_num, uint32_t pin_cfg)
{
	uint32_t reg_addr;
	uint32_t reg_val;
	uint32_t pin_offset;
	uint32_t bias_mask;
	uint32_t bias_config;

	/* Get pull-up/pull-down register address */
	reg_addr = em32_get_pupd_reg_addr(port);
	if (reg_addr == 0) {
		LOG_ERR("Failed to get PUPD register for port %d", port);
		return -EINVAL;
	}

	/* Calculate bit position and mask */
	pin_offset = pin_num * EM32_PUPD_BITS_PER_PIN;
	bias_mask = ((1U << EM32_PUPD_BITS_PER_PIN) - 1) << pin_offset;

	/* Extract bias configuration from device tree */
	bias_config = (pin_cfg >> EM32_PUPDR_SHIFT) & ((1U << EM32_PUPD_BITS_PER_PIN) - 1);

	/* Read-modify-write the PUPD register */
	reg_val = sys_read32(reg_addr);
	reg_val &= ~bias_mask;

	switch (bias_config) {
	case EM32_PULL_UP:
		reg_val |= (0x01U << pin_offset);
		LOG_DBG("P%c%d: pull-up enabled", 'A' + port, pin_num);
		break;
	case EM32_PULL_DOWN:
		reg_val |= (0x02U << pin_offset);
		LOG_DBG("P%c%d: pull-down enabled", 'A' + port, pin_num);
		break;
	case EM32_NO_PULL:
	default:
		/* Floating - already cleared by mask operation */
		LOG_DBG("P%c%d: no pull resistor", 'A' + port, pin_num);
		break;
	}

	sys_write32(reg_val, reg_addr);

	return 0;
}

/**
 * @brief Configure pin drive characteristics (open-drain/push-pull)
 *
 * @param port GPIO port (0=PA, 1=PB)
 * @param pin_num Pin number (0-15)
 * @param pin_cfg Pin configuration flags from device tree
 * @return 0 on success, negative errno on failure
 */
static int em32_configure_pin_drive(uint8_t port, uint8_t pin_num, uint32_t pin_cfg)
{
	uint32_t reg_addr;
	uint32_t reg_val;
	uint32_t pin_mask;
	bool open_drain;

	/* Get open-drain register address */
	reg_addr = em32_get_od_reg_addr(port);
	if (reg_addr == 0) {
		LOG_ERR("Failed to get OD register for port %d", port);
		return -EINVAL;
	}

	/* Calculate pin mask */
	pin_mask = BIT(pin_num);

	/* Extract drive configuration from device tree */
	open_drain = (pin_cfg & (EM32_OPEN_DRAIN << EM32_OTYPER_SHIFT)) != 0;

	/* Read-modify-write the open-drain register */
	reg_val = sys_read32(reg_addr);
	if (open_drain) {
		reg_val |= pin_mask;
		LOG_DBG("P%c%d: open-drain enabled", 'A' + port, pin_num);
	} else {
		reg_val &= ~pin_mask;
		LOG_DBG("P%c%d: push-pull enabled", 'A' + port, pin_num);
	}
	sys_write32(reg_val, reg_addr);

	return 0;
}
/**
 * @brief Configure a single pin with all its properties
 *
 * @param port GPIO port (0=PA, 1=PB)
 * @param pin_num Pin number (0-15)
 * @param alt_func Alternate function (0=GPIO, 1-7=AF1-AF7)
 * @param pin_cfg Pin configuration flags from device tree
 * @return 0 on success, negative errno on failure
 */
static int em32_configure_single_pin(uint8_t port, uint8_t pin_num, uint32_t alt_func,
				     uint32_t pin_cfg)
{
	int ret;

	LOG_DBG("Configuring P%c%d: AF%d, cfg=0x%08X", 'A' + port, pin_num, alt_func, pin_cfg);

	/* Step 1: Configure pin multiplexing */
	ret = em32_configure_pin_mux(port, pin_num, alt_func);
	if (ret < 0) {
		LOG_ERR("Failed to configure mux for P%c%d: %d", 'A' + port, pin_num, ret);
		return ret;
	}

	/* Step 2: Configure pin bias (pull resistors) */
	ret = em32_configure_pin_bias(port, pin_num, pin_cfg);
	if (ret < 0) {
		LOG_ERR("Failed to configure bias for P%c%d: %d", 'A' + port, pin_num, ret);
		return ret;
	}

	/* Step 3: Configure pin drive characteristics */
	ret = em32_configure_pin_drive(port, pin_num, pin_cfg);
	if (ret < 0) {
		LOG_ERR("Failed to configure drive for P%c%d: %d", 'A' + port, pin_num, ret);
		return ret;
	}

	/* Step 4: Configure peripheral IOShare multiplexing */
	ret = em32_configure_ioshare(port, pin_num, alt_func);
	if (ret < 0) {
		LOG_ERR("Failed to configure IOShare for P%c%d: %d", 'A' + port, pin_num, ret);
		return ret;
	}

	LOG_DBG("P%c%d configuration complete", 'A' + port, pin_num);
	return 0;
}

/* ============================================================================
 * Public API Implementation
 * ============================================================================ */

/**
 * @brief Configure multiple pins according to pinctrl state
 *
 * This is the main entry point called by the Zephyr pinctrl subsystem
 * to configure pins according to device tree specifications.
 *
 * @param pins Array of pin configurations
 * @param pin_cnt Number of pins to configure
 * @param reg Unused parameter (for API compatibility)
 * @return 0 on success, negative errno on failure
 */
int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	ARG_UNUSED(reg);

	if (pins == NULL) {
		LOG_ERR("Invalid pins array");
		return -EINVAL;
	}

	if (pin_cnt == 0) {
		LOG_WRN("No pins to configure");
		return 0;
	}

	LOG_INF("Configuring %d pins", pin_cnt);

	/* Configure each pin in the array */
	for (uint8_t i = 0; i < pin_cnt; i++) {
		uint32_t pinmux = pins[i].pinmux;
		uint32_t pin_cfg = pins[i].pincfg;

		uint8_t port = EM32_DT_GET_PORT(pinmux);
		uint8_t pin_num = EM32_DT_GET_PIN(pinmux);
		uint32_t alt_func = EM32_DT_GET_FUNC(pinmux);

		LOG_DBG("Pin %d: P%c%d AF%d (pinmux=0x%08X, cfg=0x%08X)", i, 'A' + port, pin_num,
			alt_func, pinmux, pin_cfg);

		/* Validate alternate function range */
		if (alt_func > EM32_MAX_ALT_FUNC) {
			LOG_ERR("Pin %d: Invalid alternate function %d", i, alt_func);
			return -EINVAL;
		}

		/* Configure the pin */
		int ret = em32_configure_single_pin(port, pin_num, alt_func, pin_cfg);
		if (ret < 0) {
			LOG_ERR("Pin %d configuration failed: %d", i, ret);
			return ret;
		}
	}

	LOG_INF("Successfully configured %d pins", pin_cnt);
	return 0;
}

/* ============================================================================
 * Driver Initialization and Utilities
 * ============================================================================ */

/**
 * @brief Initialize the EM32F967 pinctrl driver
 *
 * This function performs early initialization of the pinctrl subsystem,
 * setting up default configurations for essential peripherals.
 *
 * @return 0 on success, negative errno on failure
 */
static int em32_pinctrl_driver_init(void)
{
	uint32_t ioshare_val;

	LOG_INF("EM32F967 pinctrl driver initializing");

	/* Read current IOShare register state */
	ioshare_val = sys_read32(EM32_IOSHARE_BASE);
	LOG_DBG("Initial IOShare register: 0x%08X", ioshare_val);

	/* Set up default peripheral configurations
	 * NOTE: Do not force UART1 IOShare here. The final IOShare for UART1
	 * should be set according to the pinctrl state applied for the
	 * UART device (via pinctrl_apply_state). Forcing a default mapping
	 * here prevents the board DT selection from taking effect.
	 */

	/* SSP2 on PB4-PB7 for SPI functionality */
	ioshare_val &= ~(0x3U << EM32_IP_SHARE_SSP2_SHIFT);
	ioshare_val |= (0x0U << EM32_IP_SHARE_SSP2_SHIFT);

	/* Apply the configuration */
	sys_write32(ioshare_val, EM32_IOSHARE_BASE);

	LOG_INF("EM32F967 pinctrl driver initialized (IOShare: 0x%08X)", ioshare_val);
	return 0;
}

/* Initialize pinctrl driver during system startup */
SYS_INIT(em32_pinctrl_driver_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

/* ============================================================================
 * Debug and Verification Functions
 * ============================================================================ */

/**
 * @brief Verify and correct SSP2 pin configuration
 *
 * This function checks if SSP2 pins (PB4-PB7) are correctly configured
 * and corrects any conflicts caused by manual GPIO operations.
 *
 * @return 0 if no corrections needed, 1 if corrections made, negative on error
 */
int em32_pinctrl_verify_ssp2(void)
{
	const uint8_t ssp2_port = EM32_PORT_B;
	const uint8_t ssp2_pin_start = 4;
	const uint8_t ssp2_pin_end = 7;
	const uint32_t expected_func = EM32F967_AF6;

	uint32_t reg_addr = em32_get_iomux_reg_addr(ssp2_port, ssp2_pin_start);
	if (reg_addr == 0) {
		LOG_ERR("Failed to get IOMUX register for SSP2 pins");
		return -EINVAL;
	}

	uint32_t reg_val = sys_read32(reg_addr);
	bool corrections_made = false;

	/* Check each SSP2 pin */
	for (uint8_t pin = ssp2_pin_start; pin <= ssp2_pin_end; pin++) {
		uint32_t pin_offset = pin * EM32_IOMUX_BITS_PER_PIN;
		uint32_t pin_mask = ((1U << EM32_IOMUX_BITS_PER_PIN) - 1) << pin_offset;
		uint32_t actual_func =
			(reg_val >> pin_offset) & ((1U << EM32_IOMUX_BITS_PER_PIN) - 1);

		if (actual_func != expected_func) {
			LOG_WRN("SSP2 pin PB%d: incorrect AF%d, correcting to AF%d", pin,
				actual_func, expected_func);

			reg_val &= ~pin_mask;
			reg_val |= (expected_func << pin_offset);
			corrections_made = true;
		}
	}

	if (corrections_made) {
		sys_write32(reg_val, reg_addr);
		LOG_INF("SSP2 pin configuration corrected");
		return 1;
	}

	LOG_DBG("SSP2 pins correctly configured");
	return 0;
}
