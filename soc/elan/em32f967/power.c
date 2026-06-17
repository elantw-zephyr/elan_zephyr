/*
 * Copyright (c) 2025 ELAN Microelectronics Corp.
 * SPDX-License-Identifier: Apache-2.0
 *
 * EM32F967 Power Management Driver
 *
 * This driver implements power state transitions for the EM32F967 MCU,
 * supporting multiple power modes including:
 * - PDSW0: Active Mode
 * - PDSW1: CPU Power Down (suspend-to-idle)
 * - PDSW2: Standby1 (standby with selective RAM retention)
 * - PDSW3: GHM Mode
 * - PDSW4: Standby2 (deep standby, no RAM retention)
 *
 * Based on reference implementation from PowerDownSwitch.c
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/policy.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/uart.h>
#include <cmsis_core.h>
#include <soc.h>
#include "elan_em32.h"
#include "soc_967.h"
#include "soc_pm.h"
#include "em32f967.h"
#include "../../../drivers/usb/udc/udc_e967.h"

LOG_MODULE_REGISTER(pm_em32, CONFIG_PM_LOG_LEVEL);

/* ======================= Forward Declarations ============================ */
/* External functions from power_deep_sleep.c, soc_wakeup_config.c, pm_handlers.c */
/* These functions are implemented in separate source files and compiled together */
extern int em32f967_set_deep_sleep_config(int sensor_mode, int rtc_enabled);
extern void em32f967_enter_deep_sleep(void);
extern void em32f967_exit_deep_sleep(void);
extern int em32f967_enter_standby1(int rtc_enabled);
extern int em32f967_validate_deep_sleep_config(void);

extern void soc_setup_external_wakeup(int edge_type);
extern void soc_disable_external_wakeup(void);
extern void soc_configure_gpio_pulldown(void);
extern int soc_get_power_state(void);
extern void soc_enable_usb_remote_wakeup(void);
extern void soc_configure_clock_gates(int mode);
extern int soc_prepare_for_deep_sleep(void);
extern void soc_handle_wakeup(void);
extern void soc_setup_startup_configuration(void);

#ifdef CONFIG_EM32_WKUP_PINS
extern void em32_pwr_wkup_pin_cfg_pupd(void);
#endif

/* Clock controller helper from drivers/clock_control/clock_control_em32_ahb.c.
 * Re-applies the devicetree AHB clock configuration (96MHz PLL on this board)
 * and is used to restore the original system clock after PDSW1 light-sleep
 * recovery. Reusing the boot-time routine guarantees the resumed clock matches
 * the pre-suspend state exactly, so the unchanged UART baud divider keeps the
 * same (correct) baud rate. */
extern void elan_em32_set_ahb_freq(const struct device *dev);

/* ======================= Register Base Addresses ========================= */
#define EM32_SYSCFG_BASE        0x40030000
#define EM32_PWR_BASE           0x40031000
#define EM32_CLKGATE_BASE       0x40030100
#define EM32_RTC_BASE           0x40032000
#define EM32_AIP_BASE           0x40036000
#define EM32_BOOTLVE_BASE       0x4003025C
#define EM32_PIN_CTRL           0x40034118
#define EM32_WAKEUP_REG         0x40031004

/* WAKEUP_REG (0x40031004) bit definitions (see EM32F967 spec)
 *
 * - EXT_WAKEUP_EN[6:0]   : enable Wkup7..Wkup1
 * - EXT_WAKEUP_TYPE[20:7]: 2 bits per Wkup (00=fall, 01=rise, 10/11=both)
 * - EXT_WAKEUP_STATUS[27:21]: latched status of wakeup events (read-only)
 * - EXT_WAKEUP_STATUS_CLR: bit 28,wakeup status
 * - EXT_WAKEUP_STATUS[31:29]: latched status of wakeup events (read-only)
 * On the 32f967_dv board PA15 is mapped to WKUP3, and is used as the
 * only external GPIO wakeup source.
 */
#define WAKEUP_EXT_WAKEUP_STATUS_CLR   (1U << 28)
#define WAKEUP_EXT_EN_WKUP3            (1U << 2)
#define WAKEUP_EXT_TYPE_SHIFT          7U
#define WAKEUP_EXT_TYPE_BITS_PER_PIN   2U
#define WAKEUP_EXT_TYPE_FALLING        0x0U
#define WAKEUP_EXT_TYPE_RISING         0x1U
#define WAKEUP_EXT_TYPE_BOTH           0x2U

/* ======================= Additional Register Definitions ================= */
/* Boot LVE Power Down Pull Up Register (0x4003025C) */
typedef struct {
	__IO uint32_t BOOT_LVE_PSWD: 30; /* [29:0] Password */
	__IO uint32_t LVE_PDPU: 1;       /* [30] LVE Power Down/Pull Up */
	__IO uint32_t BOOT_PDPU: 1;      /* [31] Boot Power Down/Pull Up */
} BOOTLVEPDPU_TypeDef;

/* LJIRC Control Register */
typedef struct {
	__IO uint32_t LJIRCPD: 1;       /* [0] Power Down */
	__IO uint32_t LJIRCRCM: 2;      /* [2:1] Frequency Select */
	__IO uint32_t Reserved: 29;
} LJIRC_TypeDef;

/* USB PLL Control Register */
typedef struct {
	__IO uint32_t USBPLLPD: 1;      /* [0] Power Down */
	__IO uint32_t Reserved: 31;
} USBPLL_TypeDef;

/* BOR Control Register */
typedef struct {
	__IO uint32_t BOR_BOREN: 1;     /* [0] BOR Enable */
	__IO uint32_t BOR_EN_LTCH: 1;   /* [1] */
	__IO uint32_t BOR_PD_VREF: 1;   /* [2] */
	__IO uint32_t BOR_R_TRIM: 4;    /* [6:3] */
	__IO uint32_t BOR_reset: 1;     /* [7] */
	__IO uint32_t Reserved: 24;
} BOR_TypeDef;

/* LDO2 Control Register */
typedef struct {
	__IO uint32_t LDO2_IDLE: 1;     /* [0] LDO2 Idle Mode */
	__IO uint32_t LDO2_PD: 1;       /* [1] LDO2 Power Down */
	__IO uint32_t LDO2_VREF_EN: 1;  /* [2] */
	__IO uint32_t LDO2_VT: 4;       /* [6:3] */
	__IO uint32_t LDO2_VSEL: 2;     /* [8:7] */
	__IO uint32_t Reserved: 23;
} LDO2_TypeDef;

/* LDO1 Change Control Register */
typedef struct {
	__IO uint32_t LDO1_CHG_EN: 1;   /* [0] */
	__IO uint32_t CHGCounter: 9;    /* [9:1] */
	__IO uint32_t Reserved: 22;
} LDOSwitch_TypeDef;

/* ======================= Register Pointers =============================== */
#define PM_SYSREGCTRL    ((volatile SysReg_Type *)EM32_SYSCFG_BASE)
#define PM_MISCREGCTRL   ((volatile MISCReg_Type *)(EM32_SYSCFG_BASE + 0x08))
#define PM_POWERSWCTRL   ((volatile PowerSW_Type *)EM32_PWR_BASE)
#define PM_BOOTLVEPDPU   ((volatile BOOTLVEPDPU_TypeDef *)EM32_BOOTLVE_BASE)
#define PM_LJIRCCTRL     ((volatile LJIRC_TypeDef *)(EM32_AIP_BASE + 0x004))
#define PM_USBPLLCTRL    ((volatile USBPLL_TypeDef *)(EM32_AIP_BASE + 0x400))
#define PM_LDOPLL        ((volatile LDOPLL_Type *)(EM32_AIP_BASE + 0x30C))
#define PM_BORCTRL       ((volatile BOR_TypeDef *)(EM32_AIP_BASE + 0x500))
#define PM_LDO2CTRL      ((volatile LDO2_TypeDef *)(EM32_AIP_BASE + 0x304))
#define PM_LDOChange     ((volatile LDOSwitch_TypeDef *)(EM32_AIP_BASE + 0x310))
#define PM_PIN_CTRL      (*((volatile uint32_t *)EM32_PIN_CTRL))

/* Direct memory access for RAM Save Control and AIP Password */
#define RAMSave70CTRL_CS  (*((volatile uint32_t *)(EM32_AIP_BASE + 0xE08)))
#define RAMSave90CTRL_CS  (*((volatile uint32_t *)(EM32_AIP_BASE + 0xE0C)))
#define AIP_Password_CS   (*((volatile uint32_t *)(EM32_AIP_BASE + 0xF00)))

/* Note: The following are already defined in elan_em32.h or soc_967.h:
   - MIRCCTRL_2: MIRC oscillator control register (type MIRC_Type2)
   - MIRC12M_R_2, MIRC16M_2, ... MIRC32M_2: Factory trim values in ROM */

/* ======================= Power State Definitions ========================= */
#define PDSW0   0x00    /* Active Mode */
#define PDSW1   0x01    /* CPU Power Down */
#define PDSW2   0x02    /* Standby1 Mode */
#define PDSW3   0x03    /* GHM Mode */
#define PDSW4   0x04    /* Standby2 Mode */

/* ======================= PDSW1 A/B Test Switch =========================== */
/* A/B test for PDSW1 wakeup debugging.
 *
 * Set to 1 to BYPASS the PDSW1 power-switch and clock downshift, executing
 * only a plain __WFI() at the active clock. This isolates whether the USB
 * resume interrupt can wake the CPU from WFI at all:
 *   - If the CPU wakes in this mode but NOT with the full PDSW1 sequence,
 *     the fault is in the PDSW1 power-switch/clock sequence.
 *   - If the CPU still does not wake in this mode, the fault is in the
 *     WFI/interrupt path (USB resume IRQ, NVIC, or PM policy).
 *
 * Set to 0 for the normal PDSW1 power-down behavior.
 */
#define PDSW1_AB_TEST_PLAIN_WFI   1

/* USB interrupt index used by the current EM32F967 USB device controller.
 * Keep this aligned with SoC IRQ mapping (log currently reports IRQ 6). */
#define EM32_USB_RESUME_IRQN       ((IRQn_Type)6)

/* Password for Boot LVE register access */
#define BOOT_LVE_PASSWORD   0x38888192
/* Password for AIP register access */
#define AIP_PASSWORD        0x81948434

/* RAM Save Control Values from reference code */
#define RAMSAVE70_VALUE     0x1FFFF   /* All RAM regions 0-7 alive */
#define RAMSAVE90_VALUE     0x1d9C7   /* Only SRAM_128K_136K & UDC RAM alive */

/* For PDSW4 - no RAM retention */
#define RAMSAVE70_ALL_OFF   0x00000   /* All RAM powered down */
#define RAMSAVE90_ALL_OFF   0x00000   /* All RAM powered down */

/* ======================= Clock Gating Definitions ======================== */
#define PCLKG_PWR   0x1A
#define PCLKG_BKP   0x18
#define PCLKG_AIP   0x1C
#define PCLKG_GPIOA 0x01
#define PCLKG_GPIOB 0x02
#define PCLKG_LPC	0x03

/* All clock gating bits for complete power down */
#define CLKGATE_ALL_GATED   0xFFFFFFFF
#define CLKGATE2_ALL_GATED  0x00000FFF

/* ======================= UART Register Definitions ======================== */
/* UART register offsets (used for baudrate recalculation) */
#define UART_CTRL_OFFSET      0x08
#define UART_INTSTACLR_OFFSET 0x0C
#define UART_BAUDDIV_OFFSET   0x10
/* Standard UART baudrate (typically 115200 baud) */
#define UART_STANDARD_BAUDRATE 115200

/* ======================= Register Logging Functions ====================== */
/**
 * @brief Log all power management related registers
 * @param prefix String prefix for log messages (e.g., "BEFORE", "AFTER")
 */
static void pm_log_registers(const char *prefix)
{
#if 0
	volatile uint32_t *sys_reg = (volatile uint32_t *)EM32_SYSCFG_BASE;
	volatile uint32_t *pwr_reg = (volatile uint32_t *)EM32_PWR_BASE;
	volatile uint32_t *wakeup_reg = (volatile uint32_t *)EM32_WAKEUP_REG;
	volatile uint32_t *powersw_reg = (volatile uint32_t *)(EM32_PWR_BASE + 0x08);

	LOG_INF("===== PM Register Dump: %s =====", prefix);

	/* 1. SYS_REG (0x40030000) - System Register */
	LOG_INF("SYS_REG (0x40030000): 0x%08X", *sys_reg);
	LOG_INF("  HCLKSEL=%d, HCLKDIV=%d, POWEN=%d",
		PM_SYSREGCTRL->HCLKSEL, PM_SYSREGCTRL->HCLKDIV, PM_SYSREGCTRL->POWEN);
	LOG_INF("  USBReset_SEL=%d, DEEPSLPCLKOFF=%d",
		PM_SYSREGCTRL->USBReset_SEL, PM_SYSREGCTRL->DEEPSLPCLKOFF);

	/* 2. PD_REG (0x40031000) - Power Domain Register */
	LOG_INF("PD_REG (0x40031000): 0x%08X", *pwr_reg);
	LOG_INF("  POWERSW=%d (0=Active,1=CPU,2=Standby1,3=GHM,4=Standby2)",
		PM_POWERSWCTRL->POWERSW);
	LOG_INF("  SIPPDEnable=%d, BORPD=%d, HIRCPD=%d, LDO2PD=%d",
		PM_POWERSWCTRL->SIPPDEnable, PM_POWERSWCTRL->BORPD,
		PM_POWERSWCTRL->HIRCPD, PM_POWERSWCTRL->LDO2PD);
	LOG_INF("  LDOIdle=%d, SIRC32PD=%d, RAMPDEnable=%d",
		PM_POWERSWCTRL->LDOIdle, PM_POWERSWCTRL->SIRC32PD,
		PM_POWERSWCTRL->RAMPDEnable);

	/* 3. CLKGATE_REG (0x40030100) - Clock Gating Register */
	LOG_INF("CLKGATE_REG (0x40030100): 0x%08X", CLKGATEREG);
	LOG_INF("CLKGATE2_REG: 0x%08X", CLKGATEREG2);

	/* 4. POWERSW_CTRL (0x40031008) - Power Switch Control Register */
	LOG_INF("POWERSW_CTRL (0x40031008): 0x%08X", *powersw_reg);

	/* 5. ARM SCB->SCR (System Control Register) */
	LOG_INF("SCB->SCR (0xE000ED10): 0x%08X", SCB->SCR);
	LOG_INF("  SLEEPDEEP=%d, SLEEPONEXIT=%d, SEVONPEND=%d",
		(SCB->SCR & SCB_SCR_SLEEPDEEP_Msk) ? 1 : 0,
		(SCB->SCR & SCB_SCR_SLEEPONEXIT_Msk) ? 1 : 0,
		(SCB->SCR & SCB_SCR_SEVONPEND_Msk) ? 1 : 0);

	/* 6. WAKEUP_REG (0x40031004) */
	LOG_INF("WAKEUP_REG (0x40031004): 0x%08X", *wakeup_reg);

	/* Additional AIP registers */
	LOG_INF("AIP_Password_CS: 0x%08X", AIP_Password_CS);
	LOG_INF("RAMSave70CTRL_CS: 0x%08X, RAMSave90CTRL_CS: 0x%08X",
		RAMSave70CTRL_CS, RAMSave90CTRL_CS);
	LOG_INF("BOOTLVEPDPU (0x4003025C): PSWD=0x%08X, BOOT_PDPU=%d, LVE_PDPU=%d",
		PM_BOOTLVEPDPU->BOOT_LVE_PSWD, PM_BOOTLVEPDPU->BOOT_PDPU,
		PM_BOOTLVEPDPU->LVE_PDPU);

	/* PLL Status */
	LOG_INF("LJIRCCTRL: LJIRCPD=%d", PM_LJIRCCTRL->LJIRCPD);
	LOG_INF("USBPLLCTRL: USBPLLPD=%d", PM_USBPLLCTRL->USBPLLPD);
	LOG_INF("SYSPLLCTRL: SYSPLLPD=%d", SYSPLLCTRL->SYSPLLPD);
	LOG_INF("LDOPLL: PLLLDO_PD=%d", PM_LDOPLL->PLLLDO_PD);
	LOG_INF("========================================");
#else
	LOG_INF("===== PM Register Dump: %s =====", prefix);
#endif
}

/* ======================= Helper Functions ================================ */
void normal_toggle_pb8_power(uint32_t times, uint32_t delay)
{
    CLKGatingDisable(HCLKG_GPIOB);
    GPIO_SetOutput(GPIOIPB, GPIO_PINSOURCE8, GPIO_PuPd_Floating);
    //GPIO_WriteBit(GPIOIPB, GPIO_PIN_8, (BitAction)1);
    for (int i=0; i < times; i++) {
           GPIO_ToggleBits(GPIOIPB, GPIO_PIN_8);
           for (int j=0; j<delay; j++) {
                   k_busy_wait(10);
           }
    }
}

void normal_toggle_pb9_power(uint32_t times, uint32_t delay)
{
    CLKGatingDisable(HCLKG_GPIOB);
    GPIO_SetOutput(GPIOIPB, GPIO_PINSOURCE9, GPIO_PuPd_Floating);
    //GPIO_WriteBit(GPIOIPB, GPIO_PIN_9, (BitAction)1);
    for (int i=0; i < times; i++) {
           GPIO_ToggleBits(GPIOIPB, GPIO_PIN_9);
           for (int j=0; j<delay; j++) {
                   k_busy_wait(10);
           }
    }
}

/**
 * @brief Re-initialize UART after frequency change in PDSW1 recovery
 *
 * When system wakes from PDSW1 and frequency switches from 12MHz to 24MHz,
 * the APB clock changes, requiring UART baudrate divider recalculation.
 *
 * This function recalculates the baudrate divider for UART1 based on the new frequency.
 */
#if 0 /* Superseded: PDSW1 recovery now restores the original 96MHz PLL via
       * elan_em32_set_ahb_freq(), so the unchanged UART divider is already
       * correct and no baud recalculation is needed. Kept for reference. */
static void pm_uart_recalc_baudrate_pdsw1(void)
{
	/* Access UART1 registers directly */
	volatile uint32_t *uart1_base = (volatile uint32_t *)UART1_BASE;

	/* Get current APB clock frequency (should now be 48MHz after frequency switch) */
	/* For PDSW1 recovery, APB = 48MHz */
	uint32_t apb_clk_rate = 48000000;  /* 48MHz after frequency switch */
	uint32_t baudrate = UART_STANDARD_BAUDRATE;
	uint32_t bauddiv;

	/* Recalculate baudrate divider: bauddiv = (apb_clk + baudrate/2) / baudrate */
	bauddiv = (apb_clk_rate + (baudrate / 2)) / baudrate;

	if (bauddiv < 16) {
		bauddiv = 16;  /* Minimum divider value */
	}

	/* Write new baudrate divider to UART1_BAUDDIV register */
	*(uart1_base + (UART_BAUDDIV_OFFSET / sizeof(uint32_t))) = bauddiv;
	/* Clear UART1 interrupt status to prevent spurious interrupts after wakeup */
	*(uart1_base + (UART_INTSTACLR_OFFSET / sizeof(uint32_t))) = 0xF;  /* Clear all interrupt status bits */
	*(uart1_base + (UART_CTRL_OFFSET / sizeof(uint32_t))) = 0x3;  /* Re-enable UART with new baudrate */

	LOG_INF("[PDSW1_RESUME] UART1 baudrate divider recalculated: 0x%08X (APB: 48MHz, Baudrate: %u)",
		bauddiv, baudrate);
}
#endif

/**
 * @brief Disable clock gating for a peripheral
 */
static inline void pm_clk_gating_disable(uint32_t gating_n)
{
	if (gating_n <= 31) {
		CLKGATEREG &= ~(0x01 << gating_n);
	} else {
		CLKGATEREG2 &= ~(0x01 << (gating_n - 32));
	}
}

/**
 * @brief Enable clock gating for a peripheral
 */
static inline void pm_clk_gating_enable(uint32_t gating_n)
{
	if (gating_n <= 31) {
		CLKGATEREG |= (0x01 << gating_n);
	} else {
		CLKGATEREG2 |= (0x01 << (gating_n - 32));
	}
}

/**
 * @brief Short delay using NOP instructions
 */
static inline void pm_nop_delay(uint32_t count)
{
	for (volatile uint32_t i = 0; i < count; i++) {
		__asm volatile("nop");
	}
}

#if 0
static inline bool pm_usb_resume_pending(void)
{
	return NVIC_GetPendingIRQ(EM32_USB_RESUME_IRQN) != 0U;
	//return 1;
}
#endif

/**
 * @brief Disable watchdog timer before entering any deep sleep state
 *
 * Must be called before WFI in PDSW1, PDSW2, and PDSW4. The CPU cannot
 * execute the watchdog feed sequence while in sleep, so the watchdog MUST
 * be disabled. Failure causes a hard system reset ~1-2 s after sleep entry.
 *
 * The watchdog control register requires unlock key 0x1acce551.
 * WDOGCONTROL = 0x00 disables both interrupt and reset functions.
 */
static inline void pm_watchdog_disable_pdsw1(void)
{
	/* Unlock watchdog register */
	WDOGLOCK = 0x1acce551;

	/* Disable watchdog (0x00 = disabled, 0x01 = enable int, 0x03 = enable reset) */
	WDOGCONTROL = 0x00;

	/* Lock watchdog register to prevent accidental modification */
	WDOGLOCK = 0;

	LOG_INF("[WATCHDOG] Disabled for deep sleep entry");
}

/**
 * @brief Re-enable watchdog timer after waking from any deep sleep state
 *
 * Restores watchdog protection after PDSW1, PDSW2, or PDSW4 wakeup.
 * Re-enabled with WDOGCONTROL = 0x03 (interrupt + reset enabled).
 */
static inline void pm_watchdog_enable_pdsw1(void)
{
	/* Unlock watchdog register */
	WDOGLOCK = 0x1acce551;

	/* Re-enable watchdog with reset enabled (0x03 = enable reset) */
	WDOGCONTROL = 0x03;

	/* Lock watchdog register */
	WDOGLOCK = 0;

	LOG_INF("[WATCHDOG] Re-enabled after deep sleep wakeup");
}

/**
 * @brief Set LVE to High (from reference SetLVE_High function)
 */
static void pm_set_lve_high(void)
{
	LOG_INF("Setting LVE to High for deep sleep entry");
	PM_LDOChange->CHGCounter = 7;
	PM_LDOChange->LDO1_CHG_EN = 1;
	PM_MISCREGCTRL->CPUReady_SkipArbiter = 1;
	PM_PIN_CTRL |= (0x01 << 4);
	PM_PIN_CTRL &= ~(0x01 << 5);
	PM_MISCREGCTRL->CPUReady_SkipArbiter = 0;
	pm_nop_delay(40);
	PM_LDOChange->LDO1_CHG_EN = 0;
}

/**
 * @brief Set LVE to Low (from reference SetLVE_Low function)
 */
static void pm_set_lve_low(void)
{
	LOG_INF("Setting LVE to Low for deep sleep entry");
	PM_LDOChange->LDO1_CHG_EN = 1;
	PM_MISCREGCTRL->CPUReady_SkipArbiter = 1;
	PM_PIN_CTRL &= ~(0x11 << 4);
	PM_PIN_CTRL &= ~(0x01 << 5);
	PM_MISCREGCTRL->CPUReady_SkipArbiter = 0;
	pm_nop_delay(40);
	PM_LDOChange->LDO1_CHG_EN = 0;
}

/**
 * @brief Switch to low frequency clock (IRCLOW 12MHz)
 * Corrected implementation of SetMainFreq2(IRCLOW,IRCLOW12,DIV1)
 *
 * Based on reference PowerDownSwitch.c, this function properly:
 * 1. Disables clock gating for AIP module
 * 2. Sets up bus synchronization (Wait Count)
 * 3. Loads IRC trim values for frequency accuracy
 * 4. Actually changes the frequency via MIRCRCM
 * 5. Powers down PLL and sets clock dividers
 *
 * Critical for reducing standby power from 1.8mA to ~200µA
 */
static void pm_switch_to_low_freq(void)
{
	LOG_INF("Switching to low frequency clock (IRCLOW 12MHz) for standby mode");
	/* STEP 1: Disable clock gating for AIP module */
	/* AIP needs clock for register access during configuration */
	pm_clk_gating_disable(PCLKG_AIP);

	/* STEP 2: Configure Wait Count Register for bus synchronization */
	/* This synchronizes clock changes with system bus to prevent corruption */
	PM_MISCREGCTRL->WaitCountPass = 0x0a;  /* Password (added 20210119) */
	PM_MISCREGCTRL->WaitCount = 3;         /* Max wait cycles */
	PM_MISCREGCTRL->WaitCountSet = 1;      /* Enable Wait Count */

	/* STEP 3: If currently on PLL, switch to IRC first */
	/* HCLKSEL = 0x01 means PLL is active, need to switch out first */
	if (PM_SYSREGCTRL->HCLKSEL == 0x01) {
		PM_SYSREGCTRL->HCLKSEL = 0x00;    /* Select normal speed IRC */
		pm_nop_delay(100);                 /* Wait for switch > 50µs */
		SYSPLLCTRL->SYSPLLPD = 1;          /* Power down PLL */
		pm_nop_delay(10);
	}

	/* STEP 4: Load IRC Trim Values for target frequency */
	/* CRITICAL: Factory-calibrated trim values optimize oscillator accuracy/stability */
	/* Using IRCLOW12 (12MHz) for standby mode */
	MIRCCTRL_2->MIRCTall = MIRC12M_R_2->MIRC_Tall;
	MIRCCTRL_2->MIRCTV12 = ~MIRC12M_R_2->MIRC_TV12;

	/* STEP 5: Wait for trim values to settle */
	/* CRITICAL: Must wait > 50µs after loading trim before changing frequency */
	/* This allows the oscillator to stabilize with new trim values */
	pm_nop_delay(100);  /* ~100µs at current frequency */

	/* STEP 6: Set the actual IRC frequency via MIRCRCM */
	/* THIS IS THE KEY MISSING STEP IN THE ORIGINAL CODE */
	/* MIRCRCM[2:0] controls which IRC frequency is active */
	/* 0x00 = IRCLOW12 (12 MHz) - used for PDSW2 standby */
	/* 0x01 = IRCLOW16 (16 MHz) */
	/* 0x02 = IRCLOW20 (20 MHz) */
	/* 0x03 = IRCLOW24 (24 MHz) */
	/* 0x04 = IRCLOW28 (28 MHz) */
	/* 0x05 = IRCLOW32 (32 MHz) */
	MIRCCTRL->MIRCRCM = 0x00;  /* Select 12 MHz IRC */

	/* STEP 7: Ensure IRC mode selected (not XTAL) */
	/* Explicitly select IRC source instead of XTAL for consistency */
	PM_SYSREGCTRL->XTALHIRCSEL = 0;  /* Select IRC (0), not XTAL (1) */

	/* STEP 8: Ensure PLL remains powered down */
	/* Final check: keep PLL off to prevent accidental re-activation */
	SYSPLLCTRL->SYSPLLPD = 1;
	pm_nop_delay(10);

	/* STEP 9: Set HCLK divider to DIV1 (no division) */
	/* HCLKDIV[2:0] controls AHB clock divider */
	/* 0x00 = DIV1 (no division) - system clock = IRC frequency */
	/* 0x01 = DIV2, 0x02 = DIV4, 0x03 = DIV8, etc. */
	PM_SYSREGCTRL->HCLKDIV = 0;

	/* STEP 10: Disable Wait Count Register */
	/* Now that frequency is set, disable synchronization */
	PM_MISCREGCTRL->WaitCountSet = 0;
	PM_MISCREGCTRL->WaitCountPass = 0;

		/* STEP 11: In the original implementation, the UART baud rate was
		 * recalculated here using uart_elandev_recalc_baudrate(). That helper
		 * is not available in the current tree, so we skip the UART update.
		 *
		 * Console output may not be accurate if the APB clock frequency
		 * changes, but USB-based control from the EC is unaffected.
		 */
}

#if 0 /* Superseded: PDSW1 recovery now restores the original 96MHz PLL via
       * elan_em32_set_ahb_freq() instead of switching only to 24MHz IRC.
       * Kept for reference. */
/**
 * @brief Switch to high frequency clock for PDSW1 recovery (no PLL)
 *
 * For PDSW1 (light sleep), we only need to switch from 12MHz IRC to 24MHz IRC.
 * We do NOT enable PLL to avoid complex PLL lock timing issues.
 * This is much faster and simpler than the full pm_switch_to_high_freq() path.
 *
 * @return 0 on success
 */
static int pm_switch_to_high_freq_pdsw1(void)
{
	LOG_INF("[PDSW1_RESUME] Switching to high frequency IRC (24MHz) - no PLL");

	/* CRITICAL: First, restore basic power domain access before accessing AIP registers */
	/* During PDSW1 entry, POWEN was set to 0 and clock gating was modified */
	/* We need to restore this state first */
	PM_SYSREGCTRL->POWEN = 1;
	pm_nop_delay(10);

	/* STEP 1: Disable clock gating for AIP module to access registers */
	pm_clk_gating_disable(PCLKG_AIP);
	pm_nop_delay(10);

	/* STEP 2: Configure Wait Count for synchronization */
	PM_MISCREGCTRL->WaitCountPass = 0x0a;
	PM_MISCREGCTRL->WaitCount = 3;
	PM_MISCREGCTRL->WaitCountSet = 1;

	/* STEP 3: Load IRC trim values for 24MHz */
	MIRCCTRL_2->MIRCTall = MIRC24M_2->MIRC_Tall;
	MIRCCTRL_2->MIRCTV12 = ~MIRC24M_2->MIRC_TV12;

	/* STEP 4: Wait for trim values to settle */
	pm_nop_delay(100);  /* ~100µs */

	/* STEP 5: Set IRC frequency to 24MHz (without PLL) */
	MIRCCTRL->MIRCRCM = 0x03;  /* Select 24 MHz IRC */

	/* STEP 6: Ensure IRC mode (not XTAL) */
	PM_SYSREGCTRL->XTALHIRCSEL = 0;  /* Select IRC */

	/* STEP 7: Keep PLL powered down for PDSW1 */
	SYSPLLCTRL->SYSPLLPD = 1;

	/* STEP 8: Set HCLK divider to DIV1 (no division) */
	PM_SYSREGCTRL->HCLKDIV = 0;

	/* STEP 9: Disable Wait Count */
	PM_MISCREGCTRL->WaitCountSet = 0;
	PM_MISCREGCTRL->WaitCountPass = 0;

	pm_nop_delay(10);
	LOG_INF("[PDSW1_RESUME] Successfully switched to 24MHz IRC");

	return 0;
}
#endif

#if 0
/**
 * @brief Switch to high frequency clock (IRCHIGH 96MHz)
 * Corrected implementation of SetMainFreq2(IRCLOW,IRCHIGH96,DIV1)
 *
 * Based on reference PowerDownSwitch.c, this function properly:
 * 1. Disables clock gating for AIP module
 * 2. Sets up bus synchronization (Wait Count)
 * 3. Loads IRC trim values for frequency accuracy
 * 4. Actually changes the frequency via MIRCRCM
 * 5. Powers down PLL and sets clock dividers
 *
 * Critical for reducing standby power from 1.8mA to ~200µA
 *
 * NOTE: This function enables PLL and may have long stabilization times.
 * For PDSW1 recovery, use pm_switch_to_high_freq_pdsw1() instead.
 */
 static void pm_switch_to_high_freq(void)
{
	LOG_INF("Switching to high frequency clock (IRCHIGH 96MHz) for active mode");
	/* STEP 1: Disable clock gating for AIP module */
	/* AIP needs clock for register access during configuration */
	pm_clk_gating_disable(PCLKG_AIP);

	/* STEP 2: Configure Wait Count Register for bus synchronization */
	/* This synchronizes clock changes with system bus to prevent corruption */
	PM_MISCREGCTRL->WaitCountPass = 0x0a;  /* Password (added 20210119) */
	PM_MISCREGCTRL->WaitCount = 3;         /* Max wait cycles */
	PM_MISCREGCTRL->WaitCountSet = 1;      /* Enable Wait Count */

	/* STEP 3: If currently on PLL, switch to IRC first */
	/* HCLKSEL = 0x01 means PLL is active, need to switch out first */
	if (PM_SYSREGCTRL->HCLKSEL == 0x01) {
		PM_SYSREGCTRL->HCLKSEL = 0x00;    /* Select normal speed IRC */
		pm_nop_delay(100);                 /* Wait for switch > 50µs */
		SYSPLLCTRL->SYSPLLPD = 1;          /* Power down PLL */
		pm_nop_delay(10);
	}

	/* STEP 4: Load IRC Trim Values for target frequency */
	/* CRITICAL: Factory-calibrated trim values optimize oscillator accuracy/stability */
	/* Using IRCHIGH96 (96MHz) for active mode */
	MIRCCTRL_2->MIRCTall = MIRC24M_2->MIRC_Tall;
	MIRCCTRL_2->MIRCTV12 = ~MIRC24M_2->MIRC_TV12;

	/* STEP 5: Wait for trim values to settle */
	/* CRITICAL: Must wait > 50µs after loading trim before changing frequency */
	/* This allows the oscillator to stabilize with new trim values */
	pm_nop_delay(100);  /* ~100µs at current frequency */

	/* STEP 6: Set the actual IRC frequency via MIRCRCM */
	/* THIS IS THE KEY MISSING STEP IN THE ORIGINAL CODE */
	/* MIRCRCM[2:0] controls which IRC frequency is active */
	/* 0x00 = IRCLOW12 (12 MHz) - used for PDSW2 standby */
	/* 0x01 = IRCLOW16 (16 MHz) */
	/* 0x02 = IRCLOW20 (20 MHz) */
	/* 0x03 = IRCLOW24 (24 MHz) */
	/* 0x04 = IRCLOW28 (28 MHz) */
	/* 0x05 = IRCLOW32 (32 MHz) */
	MIRCCTRL->MIRCRCM = 0x03;  /* Select 24 MHz IRC */

	/* STEP 7: Ensure IRC mode selected (not XTAL) */
	/* Explicitly select IRC source instead of XTAL for consistency */
	PM_SYSREGCTRL->XTALHIRCSEL = 0;  /* Select IRC (0), not XTAL (1) */

	/* enable PLL */
	SYSPLLCTRL->SYSPLLFSET = 0x02;
	LDOPLL->PLLLDO_PD = 0;
	k_busy_wait(1);
	LDOPLL->PLLLDO_VP_SEL = 0;
	k_busy_wait(10);
	SYSPLLCTRL->SYSPLLPD = 0;
	k_busy_wait(1);
	while (SYSPLLCTRL->SYSPLLSTABLECNT == 0) {
		/* Wait for PLL to lock */
	}
	k_busy_wait(1);
	PM_SYSREGCTRL->HCLKSEL = 0x01;    /* Select PLL as system clock */
	k_busy_wait(1);

	/* STEP 8: Ensure PLL remains powered down */
	/* Final check: keep PLL off to prevent accidental re-activation */
	//SYSPLLCTRL->SYSPLLPD = 1;
	//pm_nop_delay(10);

	/* STEP 9: Set HCLK divider to DIV1 (no division) */
	/* HCLKDIV[2:0] controls AHB clock divider */
	/* 0x00 = DIV1 (no division) - system clock = IRC frequency */
	/* 0x01 = DIV2, 0x02 = DIV4, 0x03 = DIV8, etc. */
	PM_SYSREGCTRL->HCLKDIV = 1;

	/* STEP 10: Disable Wait Count Register */
	/* Now that frequency is set, disable synchronization */
	PM_MISCREGCTRL->WaitCountSet = 0;
	PM_MISCREGCTRL->WaitCountPass = 0;

	PM_SYSREGCTRL->HCLKDIV = 0;

		/* STEP 11: In the original implementation, the UART baud rate was
		 * recalculated here using uart_elandev_recalc_baudrate(). That helper
		 * is not available in the current tree, so we skip the UART update.
		 *
		 * Console output may not be accurate if the APB clock frequency
		 * changes, but USB-based control from the EC is unaffected.
		 */
}
#endif
#if 1 //Johnny standby1 enhancements
#define QUADSPI_DLR		            		(*( __IO uint32_t *)0x40024000)

typedef struct
{
	__IO uint32_t INSTRUCTION	  : 8;
	__IO uint32_t IMODE				  : 2;
	__IO uint32_t ADMODE			  : 2;
	__IO uint32_t ADSIZE			  : 2;
	__IO uint32_t ABMODE			  : 2;
	__IO uint32_t ABSIZE		 	  : 2;
	__IO uint32_t DCYC					: 6;
	__IO uint32_t CMD_EN				: 1;
	__IO uint32_t DMODE			    : 2;
	__IO uint32_t FMODE				  : 2;
	__IO uint32_t QSPI_EN		    : 1;
	__IO uint32_t RESERVED	    : 2;
} QuadSpi_Type;
#define QUADSPI_CCR1      ((QuadSpi_Type *)0x40024004)

static int32_t flash_powerdown()
{
	//QUADSPI_CCR1 = 0x048001B9;
	QUADSPI_DLR = 0x00;	//write enable
	QUADSPI_CCR1->INSTRUCTION=0xB9;
	QUADSPI_CCR1->IMODE=0x1;
	QUADSPI_CCR1->ADMODE=0x0;
	QUADSPI_CCR1->ADSIZE=0x0;
	QUADSPI_CCR1->ABMODE=0x0;
	QUADSPI_CCR1->ABSIZE=0x0;
	QUADSPI_CCR1->DCYC=0x0;
	QUADSPI_CCR1->DMODE=0x0;
	QUADSPI_CCR1->FMODE=0x0;
	QUADSPI_CCR1->CMD_EN=0x1;

	/*Johnny 0415 skip wait_qspi temporary*/
	//if (wait_qspi(1) < 0)
	//	return WaitQSPITimeOut;

	return 0;
}

static void pm_external_flash_power_control(void)
{
	//pm_log_registers("Johnny: Configuring external flash power control for standby mode");
	/*Johnny 0415 skip backup*/
	//uint32_t IOPUPDPBCTRL_bk = IOPUPDPBCTRL;
	//uint32_t IOPUPDPACTRL_bk = IOPUPDPACTRL;
	flash_powerdown();
	// QSPI CS pull up
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_PINSOURCE11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_PullUp66K;
	GPIO_Init( GPIOIPB, &GPIO_InitStructure );
	IOPUPDPBCTRL = (IOPUPDPBCTRL&0x007FFFFF)|(0xFF700000); //PB11 pull up, PB12~15 pull down
	IOPUPDPACTRL = (IOPUPDPACTRL&0xFF000000)|(0x0DFFFFFF); //PA0~11 Pull-down from bell
}

#if 1 //DGD suggestion for output GPIO power down
void GPIO_Ouput_PowerDown(GPIO_IPType* GPIOx, GPIOPinBitDef GPIOPin, BitAction BitVal)
{
    if((uint64_t)GPIOx == GPIOA_BASE )
	{
		PDPAOE |= GPIOPin;

		if( BitVal != Bit_RESET )
			PDPAOUT |= GPIOPin;
		else
			PDPAOUT &= (~GPIOPin);

		PDGPIOCTRL |= 0x01;	// PA OUT enable
	}
    else if((uint64_t)GPIOx == GPIOB_BASE )
	{
		PDPBOE |= GPIOPin;

		if( BitVal != Bit_RESET )
			PDPBOUT |= GPIOPin;
		else
			PDPBOUT &= (~GPIOPin);

		PDGPIOCTRL |= 0x02;	// PB OUT enable
	}
}
#endif

static void pm_led_pa3_output_high(void)
{
	//pm_log_registers("Johnny: Setting PA3 high for LED indication");
	/* Initial Output GPIO as floating before enter standby1 mode */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_PINSOURCE3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_Floating;
	GPIO_Init(GPIOIPA, &GPIO_InitStructure );
	GPIO_WriteBit(GPIOIPA, GPIO_PIN_3, (BitAction)1);

	/*
	 * Power down the output driver by DGD suggestion.
	 * PA3 is used as an LED indicator, so we set it high
	 * before entering standby1 mode to indicate the device is in low power state.

	 */
	GPIO_Ouput_PowerDown(GPIOIPA , GPIO_PIN_3, 1);}
#endif //Johnny standby1 enhancements

/**
 * @brief Enter Standby1 (PDSW2) power mode
 *
 * This function implements the PDSW2 power state transition exactly as per
 * the reference PowerDownSwitch function in system.c
 *
 * @param rtc_enable Enable RTC as wakeup source
 */
static void pm_enter_standby1(bool rtc_enable)
{
	//pm_log_registers("=========================== Johnny v4 =============================");
	/*
	 * v2 Changes:
	 * - Added pm_external_flash_power_control from Simon's code
	 * - Setup external flash power before enter standby1 mode
	 * v3 Changes:
	 * - Added pm_led_pa3_output_high to set PA3 high for LED indication before entering standby1 mode
	 * v4 Changes:
	 * - Added GPIO_Ouput_PowerDown() to power down the output driver for PA3, as per DGD suggestion.
	 * - DGD suggested for output GPIO pins, it should be set to floating and output driver H or L by using case.
	 */

	//volatile uint32_t *pwr_reg = (volatile uint32_t *)EM32_PWR_BASE;
	//LOG_INF("PD_REG (0x40031000): 0x%08X", *pwr_reg);
	/* Step 6: Unlock AIP registers */
	//AIP_Password_CS = AIP_PASSWORD;

	/* Step 7: Configure RAM retention for PDSW2 */
	//LOG_INF("PM_POWERSWCTRL->RAMPDEnable (0x40031000): 0x%X", PM_POWERSWCTRL->RAMPDEnable);
	//LOG_INF("RAMSave70CTRL_CS (0x40036E08): 0x%08X", RAMSave70CTRL_CS);
	//LOG_INF("RAMSave90CTRL_CS (0x40036E0C): 0x%08X", RAMSave90CTRL_CS);
	pm_external_flash_power_control();
	pm_led_pa3_output_high();

	//LOG_INF("[ENTER_PDSW2] Entering Standby1 (PDSW2) mode, RTC %s", rtc_enable ? "enabled" : "disabled");

	//pm_log_registers("BEFORE PDSW2 Entry");

	/* Disable watchdog before any clock or power changes.
	 * The watchdog cannot be fed while the CPU is in deep sleep; failure to
	 * disable it will cause a reset ~1-2 s after entering PDSW2.
	 * Follows the same approach used for PDSW1. */
	pm_watchdog_disable_pdsw1();

	/* Step 1: Disable clock gating for power management peripherals */
	pm_clk_gating_disable(PCLKG_PWR);
	pm_clk_gating_disable(PCLKG_BKP);
	pm_clk_gating_disable(PCLKG_AIP);

	/* Step 2: Enable power domain */
	PM_SYSREGCTRL->POWEN = 1;

	/* Step 3: Switch to low frequency clock (IRCLOW 12MHz) */
	pm_switch_to_low_freq();

	/* Step 4: Set LVE High */
	pm_set_lve_high();

	/* Step 5: Unlock Boot LVE register and configure */
	PM_BOOTLVEPDPU->BOOT_LVE_PSWD = BOOT_LVE_PASSWORD;
	PM_BOOTLVEPDPU->BOOT_PDPU = 1;
	PM_BOOTLVEPDPU->LVE_PDPU = 0;

	/* Step 6: Unlock AIP registers */
	AIP_Password_CS = AIP_PASSWORD;

	/* Step 7: Configure RAM retention for PDSW2 */
	RAMSave70CTRL_CS = RAMSAVE70_ALL_OFF;
	RAMSave90CTRL_CS = RAMSAVE90_ALL_OFF;

	/* Step 8: Set USB reset to power-on reset only */
	PM_SYSREGCTRL->USBReset_SEL = 1;

	/* Step 9: Configure power switch control based on RTC setting */
	if (!rtc_enable) {
		/* RTC disabled - full power down */
		PM_POWERSWCTRL->SIPPDEnable = 1;
		PM_POWERSWCTRL->BORPD = 1;
		PM_POWERSWCTRL->HIRCPD = 1;
		PM_POWERSWCTRL->LDO2PD = 1;
		PM_POWERSWCTRL->LDOIdle = 1;
		PM_POWERSWCTRL->SIRC32PD = 1;
		PM_POWERSWCTRL->RAMPDEnable = 1;
	} else {
		/* RTC enabled - keep 32K oscillator running */
		PM_BORCTRL->BOR_BOREN = 0;
		PM_LDO2CTRL->LDO2_IDLE = 1;
		PM_POWERSWCTRL->SIPPDEnable = 1;
		PM_POWERSWCTRL->BORPD = 1;
		PM_POWERSWCTRL->HIRCPD = 1;
		PM_POWERSWCTRL->LDO2PD = 1;
		PM_POWERSWCTRL->LDOIdle = 1;
		PM_POWERSWCTRL->SIRC32PD = 0;  /* Keep 32K running for RTC */
		PM_POWERSWCTRL->RAMPDEnable = 1;
	}

	/* Step 10: Power down LJIRC */
	PM_LJIRCCTRL->LJIRCPD = 1;

	/* Step 11: Power down USB PLL */
	PM_USBPLLCTRL->USBPLLPD = 1;

	/* Step 12: Power down PLL LDO */
	PM_LDOPLL->PLLLDO_PD = 1;

	/* Step 13: Switch to 32K clock if RTC enabled */
	if (rtc_enable) {
		PM_SYSREGCTRL->HCLKSEL = 0x03;  /* Select 32K CLK */
			pm_clk_gating_enable(PCLKG_AIP);
			/* UART baud adjustment for 32 KHz clock is omitted; see comment in
			 * pm_switch_to_low_freq().
			 */
	}
	normal_toggle_pb8_power(14, 1);
	/* Step 14: Set power domain switch to PDSW2 */
	//pm_log_registers("Step 14: BEFORE Set POWERSW to PDSW2");
	PM_POWERSWCTRL->POWERSW = PDSW2;

	/* Step 15: Wait for power switch to complete */
	while (PM_POWERSWCTRL->POWERSW != PDSW2) {
		pm_nop_delay(10);
	}

	/* Step 16: Small delay for stabilization */
	pm_nop_delay(1000);

	/* Step 17: Disable power domain */
	PM_SYSREGCTRL->POWEN = 0;

	/* Log registers before entering WFI */
	//pm_log_registers("BEFORE WFI (PDSW2)");

	/* Step 18: Set SLEEPDEEP bit and enter WFI */
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	__DSB();
// james 0414	__WFI();
	__ISB();
	//pm_log_registers("AFTER PDSW2 Wakeup");
}

/**
 * @brief Enter Standby1 (PDSW2) for sys_poweroff — RESET-based wakeup.
 *
 * Called from z_sys_poweroff() in poweroff.c. Uses the full PDSW2
 * power-down sequence (same as pm_enter_standby1).
 *
 * The EM32F967 WakeUp controller works the SAME as STM32 SHUTDOWN mode:
 *   - WakeUp controller asserts SYSTEM RESET on wakeup (not an interrupt)
 *   - CPU never returns from WFI — reboots from reset vector
 *   - No IRQ handler (IRQ 3) fires during wakeup
 *   - No NVIC_SystemReset() needed — hardware does the reset
 *
 * Wakeup sources (both trigger hardware RESET):
 *   - WKUP1 (PA6 falling-edge) — button press
 *   - RTC alarm (5 s timeout)  — armed in poweroff.c before this call
 *
 * @param rtc_enable Keep the 32K oscillator running for RTC alarm wakeup.
 */
void pm_enter_standby1_for_poweroff(bool rtc_enable)
{
	LOG_INF("[POWEROFF_PDSW2] Entering Standby1 via sys_poweroff, RTC %s",
		rtc_enable ? "enabled" : "disabled");

	/*
	 * Perform the full PDSW2 entry sequence:
	 *   1. Watchdog disable
	 *   2. Clock gating disable (PWR, BKP, AIP)
	 *   3. POWEN=1
	 *   4. Switch to 12 MHz IRC (UART garbles after this)
	 *   5. LVE config, Boot-LVE unlock
	 *   6. AIP password
	 *   7. RAM retention
	 *   8. POWERSWCTRL bitfields (SIPPDEnable, BORPD, HIRCPD, etc.)
	 *   9.
	 *  10. POWERSW = PDSW2
	 *  11. POWEN=0
	 *  12. SLEEPDEEP=1 (set inside pm_enter_standby1)
	 *
	 * Note: __WFI() in pm_enter_standby1 may be commented out for
	 * development. We execute WFI below to actually enter deep sleep.
	 */
	pm_enter_standby1(rtc_enable);

	/*
	 * Execute WFI — enter PDSW2 deep sleep.
	 *
	 * SLEEPDEEP is already set by pm_enter_standby1() (Step 18).
	 * After WFI, the WakeUp controller (WKUP1 or RTC alarm) asserts
	 * SYSTEM RESET — same as STM32 SHUTDOWN mode. The CPU reboots
	 * from the reset vector. This code NEVER executes past __WFI().
	 *
	 * On the console, the next output will be:
	 *   "*** Booting Zephyr OS build v4.3.0-... ***"
	 */
	LOG_INF("[POWEROFF_PDSW2] SLEEPDEEP=1, entering WFI — system will RESET on wakeup");
	__DSB();
	__WFI();

	/* WakeUp controller asserts RESET — CPU never reaches here */
	CODE_UNREACHABLE;
}

/**
 * @brief Exit from Standby1 mode
 */
static void pm_exit_standby1(void)
{
	LOG_INF("[EXIT_PDSW2] Exiting Standby1 (PDSW2) mode");

	/* Re-enable watchdog now that the CPU is running again */
	pm_watchdog_enable_pdsw1();

	/* Clear SLEEPDEEP bit */
	SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

	/* Restore power domain settings */
	PM_SYSREGCTRL->POWEN = 1;

	/* Restore USB reset selection */
	PM_SYSREGCTRL->USBReset_SEL = 0;

	/* Clear AIP password */
	AIP_Password_CS = 0;

	/* Clear Boot LVE password */
	PM_BOOTLVEPDPU->BOOT_LVE_PSWD = 0;

	/* Set LVE Low */
	pm_set_lve_low();

	/* Restore power switch to active mode */
	PM_POWERSWCTRL->POWERSW = PDSW0;
	while (PM_POWERSWCTRL->POWERSW != PDSW0) {
		pm_nop_delay(10);
	}

	/* Disable power-down settings */
	PM_POWERSWCTRL->LDOIdle = 0;
	PM_POWERSWCTRL->SIRC32PD = 0;
	PM_POWERSWCTRL->RAMPDEnable = 0;
	PM_POWERSWCTRL->BORPD = 0;
	PM_POWERSWCTRL->HIRCPD = 0;
	PM_POWERSWCTRL->SIPPDEnable = 0;
	PM_POWERSWCTRL->LDO2PD = 0;

	/* Re-enable PLLs */
	PM_LDOPLL->PLLLDO_PD = 0;
	pm_nop_delay(100);

	PM_USBPLLCTRL->USBPLLPD = 0;
	pm_nop_delay(100);

	PM_LJIRCCTRL->LJIRCPD = 0;
	pm_nop_delay(10);

	/* Disable power domain */
	PM_SYSREGCTRL->POWEN = 0;
}

/**
 * @brief Configure GPIOA and GPIOB for lowest-power prior to deep standby
 *
 * Actions performed:
 * - Drive all GPIOA/PB outputs low
 * - Configure PA and PB pull-up/pull-down registers to "Floating" (00)
 * - Disable high-drive, open-drain and open-source controls
 */
static void pm_gpio_low_power_config(void)
{
	LOG_INF("[GPIO_CONFIG] Configuring GPIOA/PB for low power");
	/* Drive output registers low */
	IPGPIOADATAOUT = 0x00000000;
	IPGPIOBDATAOUT = 0x00000000;

	/* Set all PA/PB pull-up/pull-down fields to 00 (Floating) */
//	IOPUPDPACTRL = 0x00000000;
//	IOPUPDPBCTRL = 0x00000000;
	/* Keep PA15 pull-up enabled because PA15 is used as WKUP3 on 32f967_dv. */
	IOPUPDPACTRL = 0x7dffffff;
	IOPUPDPBCTRL = 0xffffffff;

	/* Disable high drive and open-drain/open-source features */
	IOHDPACRTL = 0x00000000;
	IOHDPBCRTL = 0x00000000;
	IOODEPACTRL = 0x00000000;
	IOODEPBCTRL = 0x00000000;
	IOOSEPACTRL = 0x00000000;
	IOOSEPBCTRL = 0x00000000;

	/* Disable PD GPIO controls to avoid leakage sources */
	PDGPIOCTRL = 0x00000000;

	pm_nop_delay(10);
}

/**
 * @brief Enter Standby2 (PDSW4) power mode - Deepest power down
 *
 * This function implements the PDSW4 power state transition for maximum
 * power savings. Unlike PDSW2, this mode:
 * - Powers down ALL RAM (no retention)
 * - Powers down ALL clocks
 * - Powers down ALL LDOs
 * - Only external wakeup or RTC can wake the system
 * - After wakeup, system performs a full reset
 *
 * @param rtc_enable Enable RTC as wakeup source
 */
static void pm_enter_standby2(bool rtc_enable)
{
	pm_log_registers("BEFORE PDSW4 Entry");

	/* Disable watchdog before any power/clock changes for PDSW4.
	 * Same requirement as PDSW2: the CPU cannot feed the watchdog during
	 * deep sleep, so a watchdog reset would occur ~1-2 s after entry. */
	pm_watchdog_disable_pdsw1();

	/* Step 1: Disable clock gating for power management peripherals */
	pm_clk_gating_disable(PCLKG_PWR);
	pm_clk_gating_disable(PCLKG_BKP);
	pm_clk_gating_disable(PCLKG_AIP);

	/* Step 2: Enable power domain control */
	PM_SYSREGCTRL->POWEN = 1;

	/* Step 3: Switch to low frequency clock */
// 0210 james	pm_switch_to_low_freq();

	/* Step 4: Set LVE High for low voltage operation */
// 0210 james	pm_set_lve_high();

	/* Step 5: Unlock Boot LVE register */
	PM_BOOTLVEPDPU->BOOT_LVE_PSWD = BOOT_LVE_PASSWORD;
	PM_BOOTLVEPDPU->BOOT_PDPU = 1;
	PM_BOOTLVEPDPU->LVE_PDPU = 0;

	/* Step 6: Unlock AIP registers */
	AIP_Password_CS = AIP_PASSWORD;

	/* Step 7: PDSW4 - Power down ALL RAM (no retention) */
	RAMSave70CTRL_CS = RAMSAVE70_ALL_OFF;
	RAMSave90CTRL_CS = RAMSAVE90_ALL_OFF
	;

	/* Step 8: USB reset configuration */
	PM_SYSREGCTRL->USBReset_SEL = 1;

	/* Step 9: Configure ALL power domains to power down */
	if (!rtc_enable) {
		/* RTC disabled - absolute maximum power down */
		PM_POWERSWCTRL->SIPPDEnable = 1;  /* SIP power down */
		PM_POWERSWCTRL->BORPD = 1;        /* BOR power down */
		PM_POWERSWCTRL->HIRCPD = 1;       /* HIRC power down */
		PM_POWERSWCTRL->LDO2PD = 1;       /* LDO2 power down */
		PM_POWERSWCTRL->LDOIdle = 1;      /* LDO idle mode */
		PM_POWERSWCTRL->SIRC32PD = 1;     /* 32K IRC power down */
		PM_POWERSWCTRL->RAMPDEnable = 1;  /* RAM power down */
	} else {
		/* RTC enabled - keep 32K oscillator for RTC wakeup */
		PM_BORCTRL->BOR_BOREN = 0;        /* Disable BOR */
		PM_LDO2CTRL->LDO2_IDLE = 1;       /* LDO2 idle mode */
		PM_POWERSWCTRL->SIPPDEnable = 1;
		PM_POWERSWCTRL->BORPD = 1;
		PM_POWERSWCTRL->HIRCPD = 1;
		PM_POWERSWCTRL->LDO2PD = 1;
		PM_POWERSWCTRL->LDOIdle = 1;
		PM_POWERSWCTRL->SIRC32PD = 0;     /* Keep 32K running for RTC */
		PM_POWERSWCTRL->RAMPDEnable = 1;
	}

	/* Step 10: Power down LJIRC */
	PM_LJIRCCTRL->LJIRCPD = 1;

	/* Step 11: Power down USB PLL */
	PM_USBPLLCTRL->USBPLLPD = 1;

	/* Step 12: Power down PLL LDO */
// 0210 james	PM_LDOPLL->PLLLDO_PD = 1;

	/* Step 13: Gate ALL clocks for maximum power savings */
	pm_log_registers("Step 13: BEFORE CLKGAT_ALL and PDSW4 Entry");
	//CLKGATEREG = CLKGATE_ALL_GATED;
	//CLKGATEREG2 = CLKGATE2_ALL_GATED;
	/* Keep AIP enabled for power control if needed */
	pm_clk_gating_disable(PCLKG_AIP);

	/* Step 14: Switch to 32K clock if RTC enabled */
	if (rtc_enable) {
		PM_SYSREGCTRL->HCLKSEL = 0x03;
			pm_clk_gating_enable(PCLKG_AIP);
			/* In the original implementation, the UART baud rate was
			 * recalculated here for the 32 KHz clock domain using
			 * uart_elandev_recalc_baudrate(). That helper is not
			 * available in this tree, so we skip the UART update.
			 *
			 * Console output may not be accurate when running from the
			 * 32 KHz clock, but the EM32F967 power switch and RAM
			 * retention behavior follow the specification.
			 */
	}

	/* Step 14.1: Configure GPIOs for lowest-power before final PDSW transition */
	pm_gpio_low_power_config();

	/* Step 15: Set power domain switch to PDSW4 */
#if 1  /* for james debug usage */
//CLKGATEREG = 0xfbffeffd; /* Remain PWR and UART and GPIOA */
//CLKGATEREG = 0xfbffefff; /* Remain PWR and UART also have log  */
CLKGATEREG = 0xfbffefff;
CLKGATEREG2 = CLKGATE2_ALL_GATED;
PM_BOOTLVEPDPU->BOOT_LVE_PSWD = BOOT_LVE_PASSWORD;
PM_BOOTLVEPDPU->BOOT_PDPU = 0;
PM_BOOTLVEPDPU->LVE_PDPU = 0;
pm_clk_gating_disable(PCLKG_AIP);
AIP_Password_CS = AIP_PASSWORD;
PM_BORCTRL->BOR_BOREN = 0;
PM_POWERSWCTRL->LDO2PD = 1;

MIRCCTRL->MIRCPD = 1;  /* IRC power down */
PM_POWERSWCTRL->SIPPDEnable = 1;
PM_POWERSWCTRL->LDOIdle = 1;
PM_POWERSWCTRL->POWERSW = PDSW4;  /* root-cause no log  */

#else
CLKGATEREG = 0xffffefff;
CLKGATEREG2 = CLKGATE2_ALL_GATED;
PM_BOOTLVEPDPU->BOOT_LVE_PSWD = BOOT_LVE_PASSWORD;
PM_BOOTLVEPDPU->BOOT_PDPU = 1;
PM_BOOTLVEPDPU->LVE_PDPU = 1;

#endif
	pm_log_registers("Step 15: BEFORE PDSW4_2 Entry");
	pm_nop_delay(1000);

	/* Step 17.5: Configure wakeup sources (RTC / GPIO PA15) while
	 * the power domain control is still enabled. WAKEUP_REG is part
	 * of the same power domain block (0x4003_1000 + 0x004), so it
	 * must be programmed before POWEN is cleared.
	 */

	PM_POWERSWCTRL->POWERSW = PDSW4;  /* PDSW1 the same as PDSW2 */

	/* Step 16: Wait for power switch to complete */
	int retry_count = 0;
	while (PM_POWERSWCTRL->POWERSW != PDSW4) {
		pm_nop_delay(100);
		retry_count++;
		PM_POWERSWCTRL->POWERSW = PDSW4;
		if (retry_count > 1000) {
			break;
		}
	}

	/* Step 17: Small delay for stabilization */
	pm_nop_delay(1000);

		/* Step 18: Disable power domain control */
		PM_SYSREGCTRL->POWEN = 0;

		/* Log registers before entering WFI */
		pm_log_registers("BEFORE WFI after PDSW4 setting)");

	/* Step 19: Set SLEEPDEEP bit and enter WFI */
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	__DSB();
	__WFI();
	__ISB();

	/* Note: For PDSW4, we typically won't reach here as the system
	 * will reset on wakeup. But if we do wake up without reset: */
	pm_log_registers("AFTER PDSW4 Wakeup");
}

/**
 * @brief Exit from Standby2 mode
 * Note: In most cases, PDSW4 wakeup causes a system reset,
 * so this function may not be called.
 */
static void pm_exit_standby2(void)
{
	LOG_INF("pm_exit_standby2: System woke up from PDSW4 (deep standby) mode");

	/* Re-enable watchdog now that the CPU is running again.
	 * Note: PDSW4 typically causes a full reset on wakeup, so this
	 * path may not be reached. Re-enabling is safe if it is reached. */
	pm_watchdog_enable_pdsw1();

	/* Clear SLEEPDEEP bit */
	SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

	/* Restore power domain settings */
	PM_SYSREGCTRL->POWEN = 1;

	/* Restore USB reset selection */
	PM_SYSREGCTRL->USBReset_SEL = 0;

	/* Clear AIP password */
	AIP_Password_CS = 0;

	/* Clear Boot LVE password */
	PM_BOOTLVEPDPU->BOOT_LVE_PSWD = 0;

	/* Set LVE Low */
	pm_set_lve_low();

	/* Restore power switch to active mode */
	PM_POWERSWCTRL->POWERSW = PDSW0;
	while (PM_POWERSWCTRL->POWERSW != PDSW0) {
		pm_nop_delay(10);
	}

	/* Disable power-down settings */
	PM_POWERSWCTRL->LDOIdle = 0;
	PM_POWERSWCTRL->SIRC32PD = 0;
	PM_POWERSWCTRL->RAMPDEnable = 0;
	PM_POWERSWCTRL->BORPD = 0;
	PM_POWERSWCTRL->HIRCPD = 0;
	PM_POWERSWCTRL->SIPPDEnable = 0;
	PM_POWERSWCTRL->LDO2PD = 0;

	/* Re-enable PLLs */
	PM_LDOPLL->PLLLDO_PD = 0;
	pm_nop_delay(100);

	PM_USBPLLCTRL->USBPLLPD = 0;
	pm_nop_delay(100);

	PM_LJIRCCTRL->LJIRCPD = 0;
	pm_nop_delay(10);

	/* Restore clock gating */
	CLKGATEREG = 0;
	CLKGATEREG2 = 0;

	/* Disable power domain */
	PM_SYSREGCTRL->POWEN = 0;
	pm_log_registers("AFTER PDSW4 Exit");
}

/* ======================= Zephyr PM Interface ============================= */

/**
 * @brief Helper to map Zephyr PM state/substate to EM32F967 PDSW mode
 *
 * We support four deep standby combinations, matching the legacy
 * PowerDownSwitch(PDSW2/4, RTC_enable) behavior:
 *
 *   substate_id = 2 : PDSW2, RTC disabled  (RTC_enable = 0)
 *   substate_id = 3 : PDSW2, RTC enabled   (RTC_enable = 1)
 *   substate_id = 4 : PDSW4, RTC disabled  (RTC_enable = 0)
 *   substate_id = 5 : PDSW4, RTC enabled   (RTC_enable = 1)
 *
 * For any other substate_id we fall back to the original behavior:
 * - PM_STATE_STANDBY      -> PDSW2,  rtc_enable = IS_ENABLED(CONFIG_RTC)
 * - PM_STATE_SUSPEND_TO_RAM -> PDSW4, rtc_enable = IS_ENABLED(CONFIG_RTC)
 */

/* ========== RTC alarm helper (moved from poweroff.c) ====================== */

#define EM32_RTC_RTSC       (*(volatile uint32_t *)(EM32_RTC_BASE + 0x00U))
#define EM32_RTC_RTMNC      (*(volatile uint32_t *)(EM32_RTC_BASE + 0x04U))
#define EM32_RTC_RTHRC      (*(volatile uint32_t *)(EM32_RTC_BASE + 0x08U))
#define EM32_RTC_ARSC       (*(volatile uint32_t *)(EM32_RTC_BASE + 0x10U))
#define EM32_RTC_ARMN       (*(volatile uint32_t *)(EM32_RTC_BASE + 0x14U))
#define EM32_RTC_ARHR       (*(volatile uint32_t *)(EM32_RTC_BASE + 0x18U))
#define EM32_RTC_RTCR       (*(volatile uint32_t *)(EM32_RTC_BASE + 0x20U))
#define EM32_RTC_RTITS      (*(volatile uint32_t *)(EM32_RTC_BASE + 0x34U))

#define EM32_RTC_RTCR_ENABLE     BIT(0)
#define EM32_RTC_RTCR_ALARMEN    BIT(5)
#define EM32_RTC_RTCR_RELOAD     BIT(6)

#define EM32_CLKGATE_RTC_BIT     BIT(0x17U) /* PCLKG_RTC = bit 23 */

/**
 * @brief Arm the RTC alarm to trigger a system RESET in @p seconds.
 *
 * No ISR needed — the RTC alarm causes a hardware reset during PDSW2.
 * Reference: system.c RTCAlarm() — EnableRTC() + set ARSC/ARMN/ARHR.
 */
static void em32_rtc_arm_alarm(uint32_t seconds)
{
	uint32_t sc, mn, hr;

	/* Enable RTC clock gate (active-low: clear bit to enable) */
	uint32_t cg = sys_read32(EM32_CLKGATE_BASE);

	cg &= ~EM32_CLKGATE_RTC_BIT;
	sys_write32(cg, EM32_CLKGATE_BASE);
	__DSB();

	/* Disable RTC while reconfiguring */
	EM32_RTC_RTCR = 0;
	__DSB();

	/* Enable and reload RTC */
	EM32_RTC_RTCR = EM32_RTC_RTCR_ENABLE | EM32_RTC_RTCR_RELOAD;
	__DSB();
	k_busy_wait(100);

	/* Read current time */
	sc = EM32_RTC_RTSC + seconds;
	mn = EM32_RTC_RTMNC;
	hr = EM32_RTC_RTHRC;

	/* Carry seconds → minutes → hours */
	if (sc > 59U) {
		mn += sc / 60U;
		sc  = sc % 60U;
	}
	if (mn > 59U) {
		hr += mn / 60U;
		mn  = mn % 60U;
	}
	hr = hr % 24U;

	/* Set alarm registers */
	EM32_RTC_ARSC = sc;
	EM32_RTC_ARMN = mn;
	EM32_RTC_ARHR = hr;

	/* Clear any pending alarm status */
	EM32_RTC_RTITS = 0;

	/* Enable alarm in RTCR — no IRQ needed, alarm triggers RESET */
	uint32_t cr = EM32_RTC_RTCR;

	cr |= EM32_RTC_RTCR_ALARMEN | EM32_RTC_RTCR_ENABLE;
	EM32_RTC_RTCR = cr;
	__DSB();

	LOG_DBG("RTC alarm armed: current %02d:%02d:%02d, alarm %02d:%02d:%02d (+%ds)",
		(int)EM32_RTC_RTHRC, (int)EM32_RTC_RTMNC, (int)EM32_RTC_RTSC,
		(int)hr, (int)mn, (int)sc, (int)seconds);
}

/* ========================================================================== */

static void pm_handle_standby_modes(enum pm_state state, uint8_t substate_id)
{
	bool default_rtc = IS_ENABLED(CONFIG_RTC);
	bool rtc_enable = default_rtc;
	uint8_t mode = 0U;

	//LOG_INF("pm_handle_standby_modes: state=%d, substate_id=%d", state, substate_id);

	/*
	 * Configure wake-up pins and RTC alarm before entering deep sleep.
	 * Moved from poweroff.c — these are needed for both sys_poweroff()
	 * and pm_state_force() entry paths.
	 */
	LOG_DBG("System Off (EM32F967 PDSW2 / Standby1) — RESET-based wakeup");
#ifdef CONFIG_EM32_WKUP_PINS
	LOG_DBG("Configure wake-up pins (em32_pwr_wkup_pin_cfg_pupd)");
	em32_pwr_wkup_pin_cfg_pupd();
#endif
	em32_rtc_arm_alarm(5);
	LOG_DBG("RTC alarm set — system will RESET in 5 s if button not pressed");
	switch (substate_id) {
	case 2U:
		mode = PDSW2;
		rtc_enable = false; /* RTC disabled (32K off) */
		break;
	case 3U:
		mode = PDSW2;
		rtc_enable = true;  /* RTC enabled (32K kept on) */
		break;
	case 4U:
		mode = PDSW4;
		rtc_enable = false; /* RTC disabled (32K off) */
		break;
	case 5U:
		mode = PDSW4;
		rtc_enable = true;  /* RTC enabled (32K kept on) */
		break;
	default:
		/* Backwards-compatible default mapping */
		if (state == PM_STATE_STANDBY) {
			mode = PDSW2;
		} else {
			mode = PDSW4;
		}
		rtc_enable = default_rtc;
		break;
	}

	if (mode == PDSW2) {
		pm_enter_standby1(rtc_enable);
	} else if (mode == PDSW4) {
		pm_enter_standby2(rtc_enable);
	}
}

/**
 * @brief Initialize all configured wakeup sources for 32f967_dv board
 *
 * Configures the following wakeup sources:
 *  WKUP1: PA6  (SW0) → EXTWAKEUP1
 *  WKUP3: PA15 (SW3) → EXTWAKEUP3
 *  WKUP6: PB8  (SW4) → EXTWAKEUP6
 *
 * Called at APPLICATION level to ensure GPIO devices are ready.
 * This allows GPIO interrupts to work normally, and enables deep sleep
 * wakeup functionality when needed.
 *
 * @return 0 on success
 */
static bool init_enable = false;
static int em32_wakeup_sources_init(void)
{
#if 0  /* for james debug usage */
	//CheckWDTEvent part
	LOG_INF("E967_SYSREGCTRL->USBReset_SEL: %d", PM_SYSREGCTRL->USBReset_SEL);
	LOG_INF("UDCINTSTA->UDCINT_STA.UDC_INT_STABIT.RESUMEINTSF: %d", UDCINTSTA->UDCINT_STA.UDC_INT_STABIT.RESUMEINTSF);
	LOG_INF("SYSSTATUSCTRL->WDTRESETS: %d", SYSSTATUSCTRL->WDTRESETS);
	volatile uint32_t *wakeup_reg = (volatile uint32_t *)EM32_WAKEUP_REG;
	LOG_INF("WAKEUP_REG (0x40031004): 0x%08X", *wakeup_reg);
	volatile uint32_t *pwr_reg = (volatile uint32_t *)EM32_PWR_BASE;
	LOG_INF("PD_REG (0x40031000): 0x%08X", *pwr_reg);
	LOG_INF("PM_POWERSWCTRL->RAMPDEnable (0x40031000): 0x%X", PM_POWERSWCTRL->RAMPDEnable);
	AIP_Password_CS = AIP_PASSWORD;
	LOG_INF("RAMSave70CTRL_CS (0x40036E08): 0x%08X", RAMSave70CTRL_CS);
	LOG_INF("RAMSave90CTRL_CS (0x40036E0C): 0x%08X", RAMSave90CTRL_CS);
#endif

	if (POWERSWCTRL->StandBy1_S) // Low Power Wake Reset
	{
		LOG_INF("Low Power Wake Reset.");
#if 0 /* for james debug usage */
		LOG_INF("UDCCTRL1->UDC_CTRL1_.UDCCTRL1BIT.SUSPENDSTA: %d", UDCCTRL1->UDC_CTRL1_.UDCCTRL1BIT.SUSPENDSTA);
		LOG_INF("UDCCTRL->UDC_CTRL.UDC_CTRLBIT.USBSLPRESUME: %d", UDCCTRL->UDC_CTRL.UDC_CTRLBIT.USBSLPRESUME);
		LOG_INF("UDCINTSTA->UDCINT_STA.UDC_INT_STABIT.USBWAKEUP_SF_CLR: %d", UDCINTSTA->UDCINT_STA.UDC_INT_STABIT.USBWAKEUP_SF_CLR);
		UDCCTRL->UDC_CTRL.UDC_CTRLBIT.USBSLPRESUME = 0;
		UDCINTSTA->UDCINT_STA.UDC_INT_STABIT.USBWAKEUP_SF_CLR = 1;
		LOG_INF("USB Resume force clear.");
		LOG_INF("UDCCTRL->UDC_CTRL.UDC_CTRLBIT.USBSLPRESUME: %d", UDCCTRL->UDC_CTRL.UDC_CTRLBIT.USBSLPRESUME);
		LOG_INF("UDCINTSTA->UDCINT_STA.UDC_INT_STABIT.USBWAKEUP_SF_CLR: %d", UDCINTSTA->UDCINT_STA.UDC_INT_STABIT.USBWAKEUP_SF_CLR);
		if(UDCCTRL1->UDC_CTRL1_.UDCCTRL1BIT.SUSPENDSTA == 0){ // USB resume
			UDCCTRL->UDC_CTRL.UDC_CTRLBIT.USBSLPRESUME = 0;
			UDCINTSTA->UDCINT_STA.UDC_INT_STABIT.USBWAKEUP_SF_CLR = 1;
			LOG_INF("USB Resume.");
		}
		pm_exit_standby1();
#endif
	} else // Reset Pin
	{
		LOG_INF("Reset Pin.");
	}
	//pm_exit_standby1();
	init_enable = true;
	LOG_INF("SYS_INIT for Power Management: em32_wakeup_sources_init .");
#if 0
	const struct pm_state_info *states;
	uint8_t count = pm_state_cpu_get_all(0, &states);
	for (int i = 0; i < count; i++) {
		pm_policy_state_lock_put(states[i].state, PM_ALL_SUBSTATES);
	}
	LOG_INF("SYS_INIT for Power Management: em32_wakeup_sources_init release policy.");
#endif

	return 0;
}

#if 1 //Johnny debug

/* dump stage */
typedef enum {
    PM_DUMP_BEFORE = 0,
    PM_DUMP_AFTER  = 1,
} pm_dump_stage_t;

/* Base address */
#define J_UART_BASE      0x40002000
#define J_SPI_BASE       0x40013000
#define J_GPIOA_BASE     0x40020000
#define J_GPIOB_BASE     0x40021000
#define J_PINCTRL_BASE   0x40030200

/* UART offsets */
#define J_UART_CTRL_OFFSET      0x08
#define J_UART_INTSTACLR_OFFSET 0x0C
#define J_UART_BAUDDIV_OFFSET   0x10

/* SPI offsets (PL022) */
#define J_SSP_CR0_OFFSET   0x000
#define J_SSP_CR1_OFFSET   0x004
#define J_SSP_DR_OFFSET    0x008
#define J_SSP_SR_OFFSET    0x00C
#define J_SSP_CPSR_OFFSET  0x010
#define J_SSP_IMSC_OFFSET  0x014
#define J_SSP_RIS_OFFSET   0x018
#define J_SSP_MIS_OFFSET   0x01C
#define J_SSP_ICR_OFFSET   0x020
#define J_SSP_DMACR_OFFSET 0x024

/* GPIO offsets */
#define J_GPIO_DATA_OFFSET            0x00
#define J_GPIO_DATAOUT_OFFSET         0x04
#define J_GPIO_ALTFUNCSET_OFFSET      0x18
#define J_GPIO_INTENSET_OFFSET        0x20
#define J_GPIO_INTSTATUS_OFFSET       0x38

/* PINCTRL offsets */
#define J_EM32_IOMUXPACTRL_OFFSET   0x200
#define J_EM32_IOMUXPBCTRL_OFFSET   0x208
#define J_EM32_IOPUPACTRL_OFFSET    0x214
#define J_EM32_IOPUPBCTRL_OFFSET    0x218

/* helper read */
static inline uint32_t reg_read(uint32_t addr)
{
    return sys_read32(addr);
}

/* prefix */
static const char *stage_str(pm_dump_stage_t stage)
{
    return (stage == PM_DUMP_BEFORE) ? "BEFORE_LP" : "AFTER_LP";
}

void dump_peripheral_regs(pm_dump_stage_t stage)
{
    const char *tag = stage_str(stage);

    LOG_INF("========== %s REGISTER DUMP ==========", tag);

    /* ================= UART ================= */
    LOG_INF("[%s][UART]", tag);
    LOG_INF("CTRL      @0x%08X = 0x%08X", J_UART_BASE + J_UART_CTRL_OFFSET,
            reg_read(J_UART_BASE + J_UART_CTRL_OFFSET));
    LOG_INF("INTCLR    @0x%08X = 0x%08X", J_UART_BASE + J_UART_INTSTACLR_OFFSET,
            reg_read(J_UART_BASE + J_UART_INTSTACLR_OFFSET));
    LOG_INF("BAUDDIV   @0x%08X = 0x%08X", J_UART_BASE + J_UART_BAUDDIV_OFFSET,
            reg_read(J_UART_BASE + J_UART_BAUDDIV_OFFSET));

    /* ================= SPI ================= */
    LOG_INF("[%s][SPI]", tag);
    LOG_INF("CR0   = 0x%08X", reg_read(J_SPI_BASE + J_SSP_CR0_OFFSET));
    LOG_INF("CR1   = 0x%08X", reg_read(J_SPI_BASE + J_SSP_CR1_OFFSET));
    LOG_INF("SR    = 0x%08X", reg_read(J_SPI_BASE + J_SSP_SR_OFFSET));
    LOG_INF("CPSR  = 0x%08X", reg_read(J_SPI_BASE + J_SSP_CPSR_OFFSET));
    LOG_INF("IMSC  = 0x%08X", reg_read(J_SPI_BASE + J_SSP_IMSC_OFFSET));
    LOG_INF("RIS   = 0x%08X", reg_read(J_SPI_BASE + J_SSP_RIS_OFFSET));
    LOG_INF("MIS   = 0x%08X", reg_read(J_SPI_BASE + J_SSP_MIS_OFFSET));
    LOG_INF("DMACR = 0x%08X", reg_read(J_SPI_BASE + J_SSP_DMACR_OFFSET));

    /* ================= GPIO A ================= */
    LOG_INF("[%s][GPIOA]", tag);
    LOG_INF("DATA      = 0x%08X", reg_read(J_GPIOA_BASE + J_GPIO_DATA_OFFSET));
    LOG_INF("DATAOUT   = 0x%08X", reg_read(J_GPIOA_BASE + J_GPIO_DATAOUT_OFFSET));
    LOG_INF("ALTFUNC   = 0x%08X", reg_read(J_GPIOA_BASE + J_GPIO_ALTFUNCSET_OFFSET));
    LOG_INF("INTEN     = 0x%08X", reg_read(J_GPIOA_BASE + J_GPIO_INTENSET_OFFSET));
    LOG_INF("INTSTATUS = 0x%08X", reg_read(J_GPIOA_BASE + J_GPIO_INTSTATUS_OFFSET));

    /* ================= GPIO B ================= */
    LOG_INF("[%s][GPIOB]", tag);
    LOG_INF("DATA      = 0x%08X", reg_read(J_GPIOB_BASE + J_GPIO_DATA_OFFSET));
    LOG_INF("DATAOUT   = 0x%08X", reg_read(J_GPIOB_BASE + J_GPIO_DATAOUT_OFFSET));
    LOG_INF("ALTFUNC   = 0x%08X", reg_read(J_GPIOB_BASE + J_GPIO_ALTFUNCSET_OFFSET));
    LOG_INF("INTEN     = 0x%08X", reg_read(J_GPIOB_BASE + J_GPIO_INTENSET_OFFSET));
    LOG_INF("INTSTATUS = 0x%08X", reg_read(J_GPIOB_BASE + J_GPIO_INTSTATUS_OFFSET));

    /* ================= PINCTRL ================= */
    LOG_INF("[%s][PINCTRL]", tag);
    LOG_INF("PA MUX  = 0x%08X", reg_read(J_PINCTRL_BASE + J_EM32_IOMUXPACTRL_OFFSET));
    LOG_INF("PB MUX  = 0x%08X", reg_read(J_PINCTRL_BASE + J_EM32_IOMUXPBCTRL_OFFSET));
    LOG_INF("PA PULL = 0x%08X", reg_read(J_PINCTRL_BASE + J_EM32_IOPUPACTRL_OFFSET));
    LOG_INF("PB PULL = 0x%08X", reg_read(J_PINCTRL_BASE + J_EM32_IOPUPBCTRL_OFFSET));

    LOG_INF("=========================================");
}

/* write helper */
static inline void reg_write(uint32_t addr, uint32_t val)
{
    sys_write32(val, addr);
}

void restore_peripheral_regs_after_lp(void)
{
    LOG_INF("====== RESTORE REGISTERS AFTER LOW POWER ======");

    /* ===================================================== */
    /* ====================== SPI ========================== */
    /* ===================================================== */

    /* BEFORE:
     * CR0  = 0x00000207
     * CR1  = 0x00000002
     * CPSR = 0x00000002
     */

    LOG_INF("[RESTORE][SPI]");

    reg_write(0x40013000 + 0x004, 0x00000000); /* Disable first (CR1 = 0) */
    reg_write(0x40013000 + 0x010, 0x00000002); /* CPSR */
    reg_write(0x40013000 + 0x000, 0x00000207); /* CR0 */
    reg_write(0x40013000 + 0x004, 0x00000002); /* CR1 (enable SPI) */

    /* ===================================================== */
    /* ====================== GPIOA ======================== */
    /* ===================================================== */

    /* BEFORE:
     * DATAOUT = 0x00004000
     * ALTFUNC = 0x0000000E
     */

    LOG_INF("[RESTORE][GPIOA]");

    reg_write(0x40020000 + 0x04, 0x00004000); /* DATAOUT */
    reg_write(0x40020000 + 0x18, 0x0000000E); /* ALTFUNC */

    /* ===================================================== */
    /* ====================== GPIOB ======================== */
    /* ===================================================== */

    /* BEFORE:
     * DATAOUT = 0x00000010
     * ALTFUNC = 0x000000E0
     */

    LOG_INF("[RESTORE][GPIOB]");

    reg_write(0x40021000 + 0x04, 0x00000010); /* DATAOUT */
    reg_write(0x40021000 + 0x18, 0x000000E0); /* ALTFUNC */

    LOG_INF("================================================");
}

void lp_auto_checklist(void)
{
    uint32_t gpioa_out = sys_read32(0x40020000 + 0x04);
    uint32_t gpioa_alt = sys_read32(0x40020000 + 0x18);
    uint32_t gpiob_out = sys_read32(0x40021000 + 0x04);
    uint32_t gpiob_alt = sys_read32(0x40021000 + 0x18);

    uint32_t spi_cr1 = sys_read32(0x40013000 + 0x004);
    uint32_t spi_sr  = sys_read32(0x40013000 + 0x00C);

    uint32_t scr     = SCB->SCR;
    uint32_t icsr    = SCB->ICSR;

    LOG_INF("========== LP AUTO CHECK ==========");

    /* ========================= */
    /* 1. Wake storm detection   */
    /* ========================= */
    if (icsr & SCB_ICSR_PENDSTSET_Msk) {
        LOG_WRN("[WAKE] SysTick pending -> frequent wake");
    }

    if (icsr & SCB_ICSR_PENDSVSET_Msk) {
        LOG_WRN("[WAKE] PendSV pending -> kernel activity");
    }

    for (int i = 0; i < 1; i++) {
        if (NVIC->ISPR[i]) {
            LOG_WRN("[WAKE] IRQ pending bitmap: 0x%08X", NVIC->ISPR[i]);
        }
    }

    /* ========================= */
    /* 2. SPI state check        */
    /* ========================= */
    if (spi_cr1 & 0x2) {
        LOG_WRN("[SPI] still ENABLED -> clock active");
    }

    if (spi_sr & 0x1) {
        LOG_WRN("[SPI] BUSY -> clock not gated");
    }

    /* ⚠️ 特別針對你 observaton */
    if (!(spi_cr1 & 0x2) && (spi_sr != 0)) {
        LOG_WRN("[SPI] disabled but status active -> possible IO contention (worse power!)");
    }

    /* ========================= */
    /* 3. GPIO leakage patterns  */
    /* ========================= */

    /* GPIOA */
    if (gpioa_out) {
        LOG_WRN("[GPIOA] DATAOUT non-zero: 0x%08X -> possible external leakage", gpioa_out);
    }

    if (gpioa_alt) {
        LOG_WRN("[GPIOA] ALT FUNC enabled: 0x%08X -> peripheral still driving", gpioa_alt);
    }

    /* GPIOB */
    if (gpiob_out) {
        LOG_WRN("[GPIOB] DATAOUT non-zero: 0x%08X -> possible external leakage", gpiob_out);
    }

    if (gpiob_alt) {
        LOG_WRN("[GPIOB] ALT FUNC enabled: 0x%08X -> peripheral still driving", gpiob_alt);
    }

    /* ========================= */
    /* 4. SCB sanity             */
    /* ========================= */
    if (scr & SCB_SCR_SLEEPDEEP_Msk) {
        LOG_WRN("[CPU] still in SLEEPDEEP (should be cleared after wake)");
    }

    if (scr & SCB_SCR_SEVONPEND_Msk) {
        LOG_WRN("[CPU] SEVONPEND enabled -> causes frequent wakeups");
    }

    /* ========================= */
    /* 5. Debug block            */
    /* ========================= */
    if (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) {
        LOG_WRN("[DBG] Trace enabled -> ~20-50uA leakage");
    }

    /* ========================= */
    /* 6. Heuristic scoring      */
    /* ========================= */

    if (NVIC->ISPR[0]) {
        LOG_WRN("[ROOT CAUSE] Interrupt storm likely dominating power");
    }

    if (gpioa_out || gpiob_out) {
        LOG_WRN("[ROOT CAUSE] GPIO driving external load likely");
    }

    if (!(spi_cr1 & 0x2) && spi_sr) {
        LOG_WRN("[ROOT CAUSE] SPI clock off but pins still active -> BAD state");
    }

    LOG_INF("===================================");
}
#endif //Johnny debug

/**
 * @brief Set power state callback for Zephyr PM subsystem
 *
 * Power state mapping:
 * - PM_STATE_SUSPEND_TO_IDLE: PDSW1 (CPU power down, light sleep)
 * - PM_STATE_STANDBY:         PDSW2 / PDSW4 with RAM retention option
 * - PM_STATE_SUSPEND_TO_RAM:  PDSW4 (Standby2, no RAM retention)
 *
 * Deep standby combinations (substate_id):
 *   2: PDSW2, RTC disabled
 *   3: PDSW2, RTC enabled
 *   4: PDSW4, RTC disabled
 *   5: PDSW4, RTC enabled
 */
//static int first_suspend_to_idle = 0;
//static int first_standby1 = 0;
//#include <zephyr/kernel.h>
void pm_state_set(enum pm_state state, uint8_t substate_id)
{
	if (!init_enable) {
		LOG_WRN("pm_state_set called before wakeup sources initialized");
		return;
	}
	//pm_log_registers("PM STATE SET Entry");

	switch (state) {
	case PM_STATE_SUSPEND_TO_IDLE:
		normal_toggle_pb8_power(16, 1);
		pm_log_registers("BEFORE PDSW1 Entry");
		dump_peripheral_regs(PM_DUMP_BEFORE);

		/* Disable watchdog before entering WFI to prevent timeout during sleep */
		pm_watchdog_disable_pdsw1();

		pm_log_registers("BEFORE PDSW1 setting");
#if PDSW1_AB_TEST_PLAIN_WFI
		LOG_INF("PDSW1 A/B TEST: plain WFI without power switch or clock changes");
		uint32_t old_primask = __get_PRIMASK();
		uint32_t old_basepri;
		__asm volatile ("mrs %0, basepri" : "=r" (old_basepri));
		LOG_INF("Old BASEPRI: 0x%02X, PRIMASK: 0x%02X", old_basepri, old_primask);

		//normal_toggle_pb8_power(18, 1);
		//SCB->SCR &= ~SCB_SCR_SEVONPEND_Msk;
		SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk; /* Deep sleep, not Normal deep */
		/* pm_state_set() runs from the idle thread with interrupts locked
		 * via BASEPRI. On ARMv7-M (Cortex-M4) a BASEPRI-masked interrupt
		 * cannot generate a WFI wake-up event, so a raw __WFI() would never
		 * return. Mirror Zephyr's arch_cpu_idle(): set PRIMASK to defer ISR
		 * entry, clear BASEPRI so the resume IRQ can wake the core, run WFI,
		 * then re-enable so the pending ISR is serviced.
		 */

		uint32_t systick_ctrl = SysTick->CTRL;

		/* For case1 power measurement, fully pause SysTick while sleeping.
		 * Masking TICKINT alone still leaves the counter running. */
		SysTick->CTRL &= ~(SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk);
		/* Clear stale system pending state before first WFI. If PENDSTSET
		 * or PENDSVSET is already latched, WFI may return immediately. */
		SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;
		lp_auto_checklist();

		/* Keep PRIMASK asserted while checking wake source so non-USB
		 * interrupts do not force a full PM exit/re-entry cycle. */
		__disable_irq();
		__set_BASEPRI(0);
		__ISB();
		__DSB();
		__WFI();
		__set_BASEPRI(old_basepri);

		SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
		/* Allow the pending USB ISR to run and complete resume path. */
		__enable_irq();
		__ISB();

		/* Restore the SysTick interrupt-enable state. */
		SysTick->CTRL = systick_ctrl;
		normal_toggle_pb8_power(20, 1);
		pm_log_registers("AFTER PDSW1 plain-WFI (A/B test)");
#else
		uint32_t old_primask = __get_PRIMASK();
		uint32_t old_basepri;
		__asm volatile ("mrs %0, basepri" : "=r" (old_basepri));
		LOG_INF("Old BASEPRI: 0x%02X, PRIMASK: 0x%02X", old_basepri, old_primask);
		uint32_t usb_prio = NVIC_GetPriority(6);
		LOG_INF("USB IRQ priority: %d", usb_prio);
		pm_external_flash_power_control();
		pm_led_pa3_output_high();
		SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk; /* Normal sleep, not deep */
		__DSB();
		pm_clk_gating_disable(PCLKG_PWR);
		PM_SYSREGCTRL->POWEN = 1;
		pm_switch_to_low_freq();
		normal_toggle_pb8_power(18, 1);
		PM_POWERSWCTRL->POWERSW = PDSW1;
		while (PM_POWERSWCTRL->POWERSW != PDSW1) {
			//pm_log_registers("Waiting for PDSW1 to take effect...");
			//pm_nop_delay(10);
		}
		//pm_log_registers("BEFORE WFI after PDSW1 setting");

#if 0
		LOG_INF("SCB->SCR (0xE000ED10): 0x%08X", SCB->SCR);
		LOG_INF("  SLEEPDEEP=%d, SLEEPONEXIT=%d, SEVONPEND=%d",
		(SCB->SCR & SCB_SCR_SLEEPDEEP_Msk) ? 1 : 0,
		(SCB->SCR & SCB_SCR_SLEEPONEXIT_Msk) ? 1 : 0,
		(SCB->SCR & SCB_SCR_SEVONPEND_Msk) ? 1 : 0);

		/* 6. WAKEUP_REG (0x40031004) */
		volatile uint32_t *wakeup_reg = (volatile uint32_t *)EM32_WAKEUP_REG;
		LOG_INF("WAKEUP_REG (0x40031004): 0x%08X", *wakeup_reg);
#endif
		/* pm_state_set() runs from the idle thread with interrupts locked
		 * via BASEPRI. On ARMv7-M (Cortex-M4) a BASEPRI-masked interrupt
		 * cannot generate a WFI wake-up event, so a raw __WFI() would never
		 * return. Mirror Zephyr's arch_cpu_idle(): set PRIMASK to defer ISR
		 * entry, clear BASEPRI so the resume IRQ can wake the core, run WFI,
		 * then re-enable so the pending ISR is serviced.
		 */
		{
			/* Hold PDSW1 until a real device interrupt (USB resume IRQ)
			 * wakes the core. The tickless kernel programs SysTick for the
			 * next scheduled timeout, which would otherwise wake PDSW1 every
			 * few tens of milliseconds and produce a suspend/resume loop.
			 * Mask the SysTick interrupt so only device IRQs can wake WFI.
			 *
			 * Note: kernel timekeeping pauses during this sleep window
			 * (SysTick is the system timer and is downshifted to 12MHz here),
			 * which is acceptable for the suspended state. The previous CTRL
			 * value is restored on resume.
			 */
			uint32_t systick_ctrl = SysTick->CTRL;

			SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
			/* Clear stale system pending state before first WFI. If PENDSTSET
			 * or PENDSVSET is already latched, WFI may return immediately. */
			SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;

			while (true) {
				/* Keep PRIMASK asserted while checking wake source so non-USB
				 * interrupts do not force a full PM exit/re-entry cycle. */
				__disable_irq();
				__set_BASEPRI(0);
				__ISB();
				__DSB();
				__WFI();
				__set_BASEPRI(old_basepri);

				//if (pm_usb_resume_pending()) {
					/* Allow the pending USB ISR to run and complete resume path. */
					__enable_irq();
					__ISB();
					break;
				//}

				/* Non-USB wake source: scrub common spurious system pendings and
				 * return to WFI without leaving the PDSW1 state machine. */
				SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;
				LOG_DBG("[PDSW1] Spurious wake ignored: ICSR=0x%08X ISPR0=0x%08X",
					SCB->ICSR, NVIC->ISPR[0]);
			}

			/* Restore the SysTick interrupt-enable state. */
			SysTick->CTRL = systick_ctrl;
		}

		/* PDSW1 Recovery - restore to active mode */
		/* NOTE: GPIO is powered down during PDSW1, so no GPIO signals are possible until GPIO is re-powered */

		/* Restore power domain first (this powers GPIO back on) */
		PM_POWERSWCTRL->POWERSW = PDSW0;
		while (PM_POWERSWCTRL->POWERSW != PDSW0) {
			pm_nop_delay(10);
		}
		PM_SYSREGCTRL->POWEN = 0;
		pm_clk_gating_enable(PCLKG_PWR);

		LOG_INF("[PDSW1_RESUME] CPU woke from light sleep, restoring system state");

		/* Restore the original system clock (96MHz PLL) by re-applying the
		 * devicetree AHB clock configuration. This matches the pre-suspend
		 * state exactly, so the UART baud divider (which pm_switch_to_low_freq()
		 * never modified) once again produces the correct console baud rate -
		 * no UART recalculation is required.
		 */
		elan_em32_set_ahb_freq(DEVICE_DT_GET(DT_NODELABEL(clk_ahb_v2)));
		normal_toggle_pb9_power(6, 1);
		pm_uart_recalc_baudrate_pdsw1();
		restore_peripheral_regs_after_lp();
		GPIO_SetOutput( GPIOIPB, GPIO_PINSOURCE4, GPIO_PuPd_Floating );
		GPIO_WriteBit( GPIOIPB, GPIO_PIN_4, (BitAction)1);
		dump_peripheral_regs(PM_DUMP_AFTER);
		normal_toggle_pb8_power(10, 1);
		pm_log_registers("AFTER WFI - PDSW1 Recovery Complete");
#endif

		/* Re-enable watchdog after waking from sleep */
		pm_watchdog_enable_pdsw1();
		normal_toggle_pb9_power(10, 1);

		LOG_INF("[PDSW1_RESUME] System fully restored to active mode");
		break;

	case PM_STATE_STANDBY:
		//if (first_standby1 < 10) {
			//printk("First PM_STATE_STANDBY entry - skipping WFI for debug (%d)\n", first_standby1);
			//k_usleep(1);
			//first_standby1++;
			//break;
		//}
		/* Standby modes (PDSW2/PDSW4) with optional RTC */
		normal_toggle_pb8_power(12, 1);
		pm_handle_standby_modes(state, substate_id);
		break;

	case PM_STATE_SUSPEND_TO_RAM:
		/* Deep standby modes are also selectable here via substate_id */
		pm_handle_standby_modes(state, substate_id);
		break;

	default:
		break;
	}
}

/**
 * @brief Exit power state callback for Zephyr PM subsystem
 */
void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
	if (!init_enable) {
		LOG_WRN("pm_state_exit_post_ops called before wakeup sources initialized");
		return;
	}

	switch (state) {
	case PM_STATE_SUSPEND_TO_IDLE:
		/* Nothing special needed for light sleep */
		normal_toggle_pb8_power(22, 1);
		pm_log_registers("pm_state_exit_post_ops: Woke up from SUSPEND_TO_IDLE (PDSW1)");

		break;

		case PM_STATE_STANDBY:
			/* Mirror the entry mapping for deep standby substates */
			switch (substate_id) {
			case 2U: /* PDSW2, RTC disabled */
			case 3U: /* PDSW2, RTC enabled */
				pm_exit_standby1();
				break;
			case 4U: /* PDSW4, RTC disabled */
			case 5U: /* PDSW4, RTC enabled */
				pm_exit_standby2();
				break;
			default:
				/* Backwards-compatible default: STANDBY -> PDSW2 */
				pm_exit_standby1();
				break;
			}
			break;

		case PM_STATE_SUSPEND_TO_RAM:
			/* Default for SUSPEND_TO_RAM is PDSW4; honor substates if used */
			switch (substate_id) {
			case 2U:
			case 3U:
				pm_exit_standby1();
				break;
			case 4U:
			case 5U:
				pm_exit_standby2();
				break;
			default:
				pm_exit_standby2();
				break;
			}
			break;

	default:
		break;
	}

	/* Re-enable interrupts */
	//irq_unlock(0);

	pm_log_registers("PM STATE EXIT Complete");
}

#if 0
static int em32_pre_init(void)
{
	LOG_INF("SYS_INIT for Power Management: em32_pre_init ./n");
	const struct pm_state_info *states;
	uint8_t count = pm_state_cpu_get_all(0, &states);
	for (int i = 0; i < count; i++) {
		pm_policy_state_lock_get(states[i].state, PM_ALL_SUBSTATES);
	}
	LOG_INF("SYS_INIT for Power Management: em32_pre_init lock policy ./n");
	return 0;
}

SYS_INIT(em32_pre_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
#endif
SYS_INIT(em32_wakeup_sources_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
