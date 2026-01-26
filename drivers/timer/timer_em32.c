/*
 * Copyright (c) 2024 ELAN Microelectronics Corp.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT elan_em32_timer

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/sys_clock.h>
#include <zephyr/spinlock.h>
#include <soc.h>

/* EM32F967 Timer register offsets - corrected per spec */
#define TIMER_CTRL_OFFSET       0x00    /* CTRL1 */
#define TIMER_VALUE_OFFSET      0x04    /* VALUE1 */
#define TIMER_RELOAD_OFFSET     0x08    /* RELOAD1 */
#define TIMER_INTSTATUS_OFFSET  0x0C    /* INTSTATUS_INTCLEAR1 */

/* TIMER_CTRL register bits - updated per spec */
#define TIMER_CTRL_EN           BIT(0)  /* Enable */
#define TIMER_CTRL_SELEXTIN     BIT(1)  /* Select external input as input */
#define TIMER_CTRL_SELEXTCLK    BIT(2)  /* Select external input as clock */
#define TIMER_CTRL_INTEN        BIT(3)  /* Timer interrupt enable */

struct timer_em32_config {
    uint32_t base;
    uint32_t clock_freq;
    const struct device *clock_dev;
	uint32_t clock_gate;
    void (*irq_config_func)(const struct device *dev);
};

struct timer_em32_data {
    uint32_t ticks_per_cycle;
    volatile uint64_t cycle_count;
    struct k_spinlock lock;
};

static inline uint32_t timer_em32_read(const struct device *dev, uint32_t offset)
{
    const struct timer_em32_config *config = dev->config;
    return sys_read32(config->base + offset);
}

static inline void timer_em32_write(const struct device *dev, uint32_t offset, uint32_t value)
{
    const struct timer_em32_config *config = dev->config;
    sys_write32(value, config->base + offset);
}

static void timer_em32_isr(const struct device *dev)
{
    struct timer_em32_data *data = dev->data;

    /* Clear interrupt by writing 1 (RW1C per spec) */
    timer_em32_write(dev, TIMER_INTSTATUS_OFFSET, BIT(0));

    /* Increment cycle count */
    data->cycle_count++;

    /* Announce tick to kernel */
    sys_clock_announce(1);
}

static int timer_em32_start(const struct device *dev)
{
    uint32_t ctrl = timer_em32_read(dev, TIMER_CTRL_OFFSET);
    ctrl |= TIMER_CTRL_EN;
    timer_em32_write(dev, TIMER_CTRL_OFFSET, ctrl);
    return 0;
}

static int timer_em32_stop(const struct device *dev)
{
    uint32_t ctrl = timer_em32_read(dev, TIMER_CTRL_OFFSET);
    ctrl &= ~TIMER_CTRL_EN;
    timer_em32_write(dev, TIMER_CTRL_OFFSET, ctrl);
    return 0;
}

static uint32_t timer_em32_get_value(const struct device *dev)
{
    return timer_em32_read(dev, TIMER_VALUE_OFFSET);
}

static int timer_em32_set_reload(const struct device *dev, uint32_t reload)
{
    timer_em32_write(dev, TIMER_RELOAD_OFFSET, reload);
    return 0;
}

/* System timer interface */
void sys_clock_set_timeout(int32_t ticks, bool idle)
{
    /* EM32F967 timer runs continuously, timeout handled by kernel */
    ARG_UNUSED(ticks);
    ARG_UNUSED(idle);
}

uint32_t sys_clock_elapsed(void)
{
    return 0; /* Continuous timer, no elapsed calculation needed */
}

uint32_t sys_clock_cycle_get_32(void)
{
    const struct device *dev = DEVICE_DT_INST_GET(0);
    struct timer_em32_data *data = dev->data;
    k_spinlock_key_t key;
    uint32_t cycles;

    key = k_spin_lock(&data->lock);
    cycles = (uint32_t)(data->cycle_count * data->ticks_per_cycle +
               (data->ticks_per_cycle - timer_em32_get_value(dev)));
    k_spin_unlock(&data->lock, key);

    return cycles;
}

uint64_t sys_clock_cycle_get_64(void)
{
    const struct device *dev = DEVICE_DT_INST_GET(0);
    struct timer_em32_data *data = dev->data;
    k_spinlock_key_t key;
    uint64_t cycles;

    key = k_spin_lock(&data->lock);
    cycles = data->cycle_count * data->ticks_per_cycle +
         (data->ticks_per_cycle - timer_em32_get_value(dev));
    k_spin_unlock(&data->lock, key);

    return cycles;
}

static int timer_em32_init(const struct device *dev)
{
    const struct timer_em32_config *config = dev->config;
    struct timer_em32_data *data = dev->data;

    /* Enable clock */
    if (config->clock_dev) {
        clock_control_on(config->clock_dev, UINT_TO_POINTER(config->clock_gate));
    }

    /* Calculate ticks per system clock cycle */
    data->ticks_per_cycle = config->clock_freq / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    data->cycle_count = 0;

    /* Configure timer */
    timer_em32_write(dev, TIMER_RELOAD_OFFSET, data->ticks_per_cycle - 1);
    timer_em32_write(dev, TIMER_CTRL_OFFSET, TIMER_CTRL_INTEN);

    /* Configure interrupt */
    config->irq_config_func(dev);

    /* Start timer */
    timer_em32_start(dev);

    return 0;
}

#define TIMER_EM32_INIT(n)							\
    static void timer_em32_irq_config_func_##n(const struct device *dev)	\
    {									\
        IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),		\
                timer_em32_isr, DEVICE_DT_INST_GET(n), 0);	\
        irq_enable(DT_INST_IRQN(n));					\
    }									\
                                        \
    static const struct timer_em32_config timer_em32_config_##n = {\
        .base = DT_INST_REG_ADDR(n),					\
        .clock_freq = DT_PROP(DT_INST_CLOCKS_CTLR(n), clock_frequency),\
        .clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),		\
        .clock_gate = DT_INST_CLOCKS_CELL_BY_IDX(n, 0, gate), \
        .irq_config_func = timer_em32_irq_config_func_##n,		\
    };									\
                                        \
    static struct timer_em32_data timer_em32_data_##n;		\
                                        \
    DEVICE_DT_INST_DEFINE(n, timer_em32_init, NULL,			\
                  &timer_em32_data_##n,				\
                  &timer_em32_config_##n,			\
                  PRE_KERNEL_2, CONFIG_SYSTEM_TIMER_INIT_PRIORITY,	\
                  NULL);

DT_INST_FOREACH_STATUS_OKAY(TIMER_EM32_INIT)
