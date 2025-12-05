/*
 * Copyright (c) 2024 ELAN Microelectronics Corp.
 * SPDX-License-Identifier: Apache-2.0
 *
 * EM32F967 PWM Controller Driver Header
 *
 * Based on tested pwm.h and pwm.c sample code from EM32F967 SDK.
 * This header defines all registers and types for the EM32F967 PWM controller.
 */

#ifndef ZEPHYR_DRIVERS_PWM_PWM_EM32_H_
#define ZEPHYR_DRIVERS_PWM_PWM_EM32_H_

#include <stdint.h>
#include <zephyr/sys/util.h>

/*
 * EM32F967 PWM Controller
 * Base Address: 0x40006000
 * Bus: APB1
 * Channels: 6 (PWMA, PWMB, PWMC, PWMD, PWME, PWMF)
 */

/* PWM Base Address */
#define PWM_BASE_ADDR           0x40006000

/* PWM Register Offsets */
#define PWM_DEADTR_OFFSET       0x000   /* Dead-Time Register */
#define PWM_ENCR_OFFSET         0x004   /* Enable Control Register */

/*
 * Channel Register Offsets (EM32F967 layout)
 * Based on SDK pwm.h for EM32967:
 *   Dummy0[62] = 248 bytes (0xF8) after PWMENCR
 *   Each channel has 4 registers (16 bytes) + Dummy[60] = 256 bytes (0x100) spacing
 *
 * Structure: PWMDEADTR(0x00), PWMENCR(0x04), Dummy0[62],
 * PWMACR, PWMAPRDR, PWMADTR, PWMATMR, Dummy1[60], ...
 *
 * So channel offsets from base are:
 *   PWMA: 0x008 + 0x0F8 = 0x100
 *   PWMB: 0x100 + 0x100 = 0x200
 *   etc.
 */
#define PWM_CHANNEL_A_OFFSET    0x100   /* PWMA registers start */
#define PWM_CHANNEL_B_OFFSET    0x200   /* PWMB registers start */
#define PWM_CHANNEL_C_OFFSET    0x300   /* PWMC registers start */
#define PWM_CHANNEL_D_OFFSET    0x400   /* PWMD registers start */
#define PWM_CHANNEL_E_OFFSET    0x500   /* PWME registers start */
#define PWM_CHANNEL_F_OFFSET    0x600   /* PWMF registers start */

/* Per-channel register offsets (relative to channel base) */
#define PWM_CR_OFFSET           0x00    /* Control Register (PWMxCR) */
#define PWM_PRDR_OFFSET         0x04    /* Period Data Register (PWMxPRDR) */
#define PWM_DTR_OFFSET          0x08    /* Duty Time Register (PWMxDTR) */
#define PWM_TMR_OFFSET          0x0C    /* Timer Register (PWMxTMR, read-only) */

/* IP Share Control Register for PWM pin selection */
#define IP_SHARE_CTRL_ADDR      0x4003023C

/* Clock Gating Register */
#define CLK_GATE_REG_ADDR       0x40030100
#define PCLKG_PWM               0x13    /* PWM clock gating bit position */

/* PWM Source Channel Definitions (bit positions for enable) */
#define PWM_CH_A                BIT(0)
#define PWM_CH_B                BIT(1)
#define PWM_CH_C                BIT(2)
#define PWM_CH_D                BIT(3)
#define PWM_CH_E                BIT(4)
#define PWM_CH_F                BIT(5)
#define PWM_CH_ALL              0x3F

/* Number of PWM channels */
#define PWM_EM32_NUM_CHANNELS   6

/* PWM Enable Control Register (PWMENCR) bit definitions */
#define PWMENCR_TCAEN           BIT(0)  /* Timer Channel A Enable */
#define PWMENCR_TCBEN           BIT(1)  /* Timer Channel B Enable */
#define PWMENCR_TCCEN           BIT(2)  /* Timer Channel C Enable */
#define PWMENCR_TCDEN           BIT(3)  /* Timer Channel D Enable */
#define PWMENCR_TCEEN           BIT(4)  /* Timer Channel E Enable */
#define PWMENCR_TCFEN           BIT(5)  /* Timer Channel F Enable */
#define PWMENCR_PWMADTIE        BIT(10) /* PWMA Dead-Time Interrupt Enable */
#define PWMENCR_PWMAPRDIE       BIT(11) /* PWMA Period Interrupt Enable */
#define PWMENCR_PWMBDTIE        BIT(12) /* PWMB Dead-Time Interrupt Enable */
#define PWMENCR_PWMBPRDIE       BIT(13) /* PWMB Period Interrupt Enable */
#define PWMENCR_PWMCDTIE        BIT(14) /* PWMC Dead-Time Interrupt Enable */
#define PWMENCR_PWMCPRDIE       BIT(15) /* PWMC Period Interrupt Enable */
#define PWMENCR_PWMDDTIE        BIT(16) /* PWMD Dead-Time Interrupt Enable */
#define PWMENCR_PWMDPRDIE       BIT(17) /* PWMD Period Interrupt Enable */
#define PWMENCR_PWMEDTIE        BIT(18) /* PWME Dead-Time Interrupt Enable */
#define PWMENCR_PWMEPRDIE       BIT(19) /* PWME Period Interrupt Enable */
#define PWMENCR_PWMFDTIE        BIT(20) /* PWMF Dead-Time Interrupt Enable */
#define PWMENCR_PWMFPRDIE       BIT(21) /* PWMF Period Interrupt Enable */

/* PWM Channel Control Register (PWMxCR) bit definitions */
#define PWMCR_DEADTAE           BIT(0)  /* Dead-Time Auto Enable */
#define PWMCR_PWMAS             BIT(1)  /* PWM Auto Start */
#define PWMCR_PWMAOSM           BIT(3)  /* PWM One Shot Mode */
#define PWMCR_PWMASWAP          BIT(4)  /* PWM Output Swap */
#define PWMCR_PWMADSFCL         BIT(6)  /* PWM Duty Status Flag Clear */
#define PWMCR_PWMAPSFCL         BIT(7)  /* PWM Period Status Flag Clear */
#define PWMCR_TAP_SHIFT         8       /* Timer Auto Prescaler shift */
#define PWMCR_TAP_MASK          0x7     /* Timer Auto Prescaler mask (3 bits) */
#define PWMCR_IPWMAA            BIT(12) /* N output polarity: 0=normal, 1=same as P */
#define PWMCR_PWMAA             BIT(13) /* P output polarity: 0=inverted, 1=normal */
#define PWMCR_IPWMAE            BIT(14) /* N output enable */
#define PWMCR_PWMAE             BIT(15) /* P output enable */
#define PWMCR_PWMADSF           BIT(16) /* PWM Duty Status Flag */
#define PWMCR_PWMAPSF           BIT(17) /* PWM Period Status Flag */

/*
 * PWM Output Type Flags (for driver use)
 * These are used in device tree or runtime configuration to select output type.
 * Use PWM_POLARITY_INVERTED flag in Zephyr PWM API for polarity control.
 */
#define PWM_EM32_OUTPUT_P       0x00    /* Use P (positive) output only */
#define PWM_EM32_OUTPUT_N       0x01    /* Use N (negative/complementary) output only */
#define PWM_EM32_OUTPUT_BOTH    0x02    /* Use both P and N outputs (complementary pair) */

/* IP Share Control Register bit definitions for PWM */
#define IP_SHARE_PWM_D_A1_S     BIT(15) /* PWM D/A1 Select */
#define IP_SHARE_PWM_E_B1_S     BIT(16) /* PWM E/B1 Select */
#define IP_SHARE_PWM_F_C1_S     BIT(17) /* PWM F/C1 Select */
#define IP_SHARE_PWM_S          BIT(18) /* PWM Pin Select: 0=PA0-5, 1=PB10-15 */

/* Dead-Time Register bit definitions */
#define PWMDEADTR_VALUE_MASK    0xFFFF  /* Dead-time value mask [15:0] */
#define PWMDEADTR_TP_SHIFT      16      /* Dead-time prescaler shift */
#define PWMDEADTR_TP_MASK       0x3     /* Dead-time prescaler mask (2 bits) */

/* Dead-time prescaler values */
#define PWMDEADTR_TP_DIV1       0       /* Division by 1 */
#define PWMDEADTR_TP_DIV2       1       /* Division by 2 */
#define PWMDEADTR_TP_DIV4       2       /* Division by 4 */
#define PWMDEADTR_TP_DIV8       3       /* Division by 8 */

/* PWM Output Mode Enumerations (from original pwm.h) */
typedef enum {
	PWM_P_OUT_DISABLE  = 0x0,       /* P output disabled */
	PWM_P_OUT_POSITIVE = 0x1,       /* P output positive (normal) */
	PWM_P_OUT_NEGATIVE = 0x2,       /* P output negative (inverted) */
} pwm_p_mode_t;

typedef enum {
	PWM_N_OUT_DISABLE  = 0x0,       /* N output disabled */
	PWM_N_OUT_POSITIVE = 0x1,       /* N output positive */
	PWM_N_OUT_NEGATIVE = 0x2,       /* N output negative */
} pwm_n_mode_t;

/* PWM Channel Index */
typedef enum {
	PWM_CHANNEL_A = 0,
	PWM_CHANNEL_B = 1,
	PWM_CHANNEL_C = 2,
	PWM_CHANNEL_D = 3,
	PWM_CHANNEL_E = 4,
	PWM_CHANNEL_F = 5,
} pwm_channel_t;

/* Channel offset lookup table */
static const uint32_t pwm_channel_offsets[PWM_EM32_NUM_CHANNELS] = {
	PWM_CHANNEL_A_OFFSET,
	PWM_CHANNEL_B_OFFSET,
	PWM_CHANNEL_C_OFFSET,
	PWM_CHANNEL_D_OFFSET,
	PWM_CHANNEL_E_OFFSET,
	PWM_CHANNEL_F_OFFSET,
};

/* Channel enable bit lookup table */
static const uint32_t pwm_channel_enable_bits[PWM_EM32_NUM_CHANNELS] = {
	PWM_CH_A,
	PWM_CH_B,
	PWM_CH_C,
	PWM_CH_D,
	PWM_CH_E,
	PWM_CH_F,
};

#endif /* ZEPHYR_DRIVERS_PWM_PWM_EM32_H_ */

