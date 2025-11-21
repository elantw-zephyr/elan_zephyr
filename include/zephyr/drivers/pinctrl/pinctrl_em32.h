/*
 * Copyright (c) 2024 ELAN Microelectronics Corp.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_PINCTRL_PINCTRL_EM32_H_
#define ZEPHYR_INCLUDE_DRIVERS_PINCTRL_PINCTRL_EM32_H_

/**
 * @file
 * @brief EM32F967 Pin Control Driver API
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Verify and correct SSP2 pin configuration
 *
 * This function checks if SSP2 pins (PB4-PB7) are correctly configured
 * and corrects any conflicts caused by manual GPIO configuration.
 * Should be called after any manual GPIO operations that might affect SSP2 pins.
 *
 * @return 0 if no corrections needed, 1 if corrections were made, negative on error
 */
int em32_pinctrl_verify_ssp2(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_PINCTRL_PINCTRL_EM32_H_ */
