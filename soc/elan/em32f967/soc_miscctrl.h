/*
 * Copyright (c) 2025 Elan Microelectronics Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_ELAN_EM32F967_SOC_MISCCTRL_H_
#define ZEPHYR_SOC_ELAN_EM32F967_SOC_MISCCTRL_H_

#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>

/* Devicetree node label */
#define MISCCTRL_DT_NODE DT_NODELABEL(miscctrl)

/* Register Offsets */
#define MIRC_12M_R_2_OFF 0x7f60
#define MIRC_16M_2_OFF   0x6070
#define MIRC_20M_2_OFF   0x6074
#define MIRC_24M_2_OFF   0x6078
#define MIRC_28M_2_OFF   0x607c
#define MIRC_32M_2_OFF   0x6080

#define CLKCTRL_CLK_GATE_REG_OFF  0x0100 /* TODO: move to sysctrl.h when upstream */
#define CLKCTRL_CLK_GATE_REG2_OFF 0x0104 /* TODO: move to sysctrl.h when upstream */

/* Field Masks for MIRC_CTRL */
#define MIRC_TALL_MASK GENMASK(9, 0)     /* [9:0]    MIRC_Tall */
#define MIRC_TV12_MASK GENMASK(12, 10)   /* [12:10]  MIRC_TV12 */

#endif /* ZEPHYR_SOC_ELAN_EM32F967_SOC_MISCCTRL_H_ */
