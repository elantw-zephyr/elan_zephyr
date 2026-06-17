/*
 * Copyright (c) 2026 ELAN Microelectronics Corp.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file header for EM32 GPIO
 *
 * Internal GPIO driver header. Pinctrl now owns IOMUX / ALTFUNC /
 * drive-strength directly (see pinctrl_em32.c) so there is no longer any
 * cross-driver surface — only GPIO-driver-internal declarations live here.
 */

#ifndef ZEPHYR_DRIVERS_GPIO_GPIO_EM32_H_
#define ZEPHYR_DRIVERS_GPIO_GPIO_EM32_H_

#include <zephyr/drivers/gpio.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_EM32_WKUP_PINS
/**
 * @brief Register a GPIO pin as an EM32 WakeUp source.
 *
 * Called from gpio_em32_pin_configure() when the EM32_GPIO_WKUP flag is set.
 * Mirrors STM32's stm32_pwr_wkup_pin_cfg_gpio().
 *
 * This function only REGISTERS the pin (logs + stores port/pin/flags). The
 * hardware configuration (AF5 IOMUX + WAKEUP_REG WAKEUPEN + IRQ 3) is
 * deferred to em32_pwr_wkup_pin_cfg_pupd(), called just before poweroff.
 *
 * @param dev   GPIO port device (gpioa or gpiob)
 * @param pin   GPIO pin number (0-15)
 * @param flags GPIO configuration flags (GPIO_ACTIVE_LOW → falling edge)
 * @return 0 on success, -EINVAL if pin not mapped to a WKUP source
 */
int em32_pwr_wkup_pin_cfg_gpio(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags);

/**
 * @brief Activate hardware wakeup configuration for all registered pins.
 *
 * Called from z_sys_poweroff() in poweroff.c just before entering deep sleep.
 * Mirrors STM32's stm32_pwr_wkup_pin_cfg_pupd().
 *
 * For each pin registered by em32_pwr_wkup_pin_cfg_gpio():
 *   1. Configures IOMUX to AF5 (WakeUp controller routing).
 *   2. Sets WAKEUP_REG WAKEUPEN bit for the corresponding WKUP source.
 *   3. Sets trigger polarity (FALLINGEDGE for GPIO_ACTIVE_LOW).
 *   4. Clears pending wakeup status (WSTATUSCLR).
 *   5. Enables WakeUp_Int_IRQn (IRQ 3) in NVIC.
 */
void em32_pwr_wkup_pin_cfg_pupd(void);
#endif /* CONFIG_EM32_WKUP_PINS */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_GPIO_GPIO_EM32_H_ */
