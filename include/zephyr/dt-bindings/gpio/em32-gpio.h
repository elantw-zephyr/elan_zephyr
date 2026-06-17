/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * EM32 GPIO DT-binding helper macros for C code.
 */

#ifndef ZEPHYR_DT_BINDINGS_GPIO_EM32_GPIO_H
#define ZEPHYR_DT_BINDINGS_GPIO_EM32_GPIO_H

/* EM32 GPIO wake-up flag (bit 8) */
#define EM32_GPIO_WKUP          0x0100

/* Pull-up / pull-down hardware values (EM32 spec)
 * Note: values must match driver expectations.
 */
#define EM32_GPIO_PUPD_FLOATING 0x00
#define EM32_GPIO_PUPD_PULLUP   0x01
#define EM32_GPIO_PUPD_PULLDOWN 0x03

#endif /* ZEPHYR_DT_BINDINGS_GPIO_EM32_GPIO_H */
