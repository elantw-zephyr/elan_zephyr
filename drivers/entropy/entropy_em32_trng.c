/*
 * EM32F967 True Random Number Generator (TRNG) entropy driver
 *
 * Copyright (c) 2025 Elan Microelectronics
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT elan_em32_trng

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <soc.h>
#include "../../include/zephyr/drivers/clock_control/clock_control_em32_apb.h"

LOG_MODULE_REGISTER(entropy_em32_trng, CONFIG_ENTROPY_LOG_LEVEL);

/* Original initialization: keep default early device init priority. */
#define EM32_CLKGATEREG 0x40030100 /* Clock Gating Control Register */
#define HCLKG_TRNG  0x1E           /* 0: TRNG Clock Enable (bit 30) */

/* Register offsets (from hardware specification) */
#define TRNG_CONTROL_OFFSET    0x00  /* R/W Control register */
#define TRNG_STATUS_OFFSET     0x0C  /* R/W Status register */
#define TRNG_DATA_START_OFFSET 0x10  /* Random data start (256 bits from 0x10 to 0x2C) */
#define TRNG_DATA_END_OFFSET   0x2C  /* Random data end */
#define TRNG_DATA_SIZE_BYTES   32    /* 256 bits = 32 bytes */

/* CONTROL register configuration */
#define TRNG_CONTROL_VALUE     0x0B    /* Correct control value for TRNG operation */

/* STATUS register bit definitions */
#define TRNG_STATUS_DATA_RDY     BIT(0)  /* Data ready status bit */
#define TRNG_STATUS_BUSY         BIT(1)  /* TRNG busy generating data */
#define TRNG_STATUS_ERROR        BIT(2)  /* Error status bit */

/* Convenience accessors */
#define TRNG_REG(base, off) (*((volatile uint32_t *)((base) + (off))))
#define TRNG_READ(base, off) (TRNG_REG(base, off))
#define TRNG_WRITE(base, off, val) (TRNG_REG(base, off) = (uint32_t)(val))

struct em32_trng_config {
	uintptr_t base;
	const struct device *clock_dev;
};

struct em32_trng_data {
	bool ready;
};

/* Forward declarations */
static int em32_trng_collect_cycle(const struct device *dev, uint8_t *buffer);
static int em32_trng_start_generation(const struct device *dev);
static void em32_disable_clkgate(void);







static int em32_trng_get_entropy(const struct device *dev, uint8_t *buffer, uint16_t length)
{
	struct em32_trng_data *data = dev->data;
	uint16_t collected = 0;

	if (!data->ready) {
		return -ENODEV;
	}

	LOG_DBG("Requesting %u bytes of entropy", length);

	while (collected < length) {
		/* Polling mode - use the unified collection function */
		uint8_t cycle_data[32];
		int ret = em32_trng_collect_cycle(dev, cycle_data);
		if (ret < 0) {
			return ret;
		}

		/* Copy what we need */
		uint32_t to_copy = MIN((uint32_t)(length - collected), (uint32_t)ret);
		memcpy(&buffer[collected], cycle_data, to_copy);
		collected += to_copy;
	}

	LOG_DBG("Successfully collected %u bytes of entropy", collected);
	return 0;
}



static void em32_disable_clkgate(void)
{
	/* Read current clock gating register value */
	uint32_t clkgate_reg = *((volatile uint32_t *)EM32_CLKGATEREG);

	/* Clear the TRNG clock gate bit (bit 30) to enable TRNG clock */
	clkgate_reg &= ~(1U << HCLKG_TRNG);

	/* Write back the modified value */
	*((volatile uint32_t *)EM32_CLKGATEREG) = clkgate_reg;

	LOG_DBG("TRNG clock gate disabled (clock enabled): clkgate_reg=0x%08x", clkgate_reg);
}



static int em32_trng_init(const struct device *dev)
{
	const struct em32_trng_config *cfg = dev->config;
	struct em32_trng_data *data = dev->data;
	const struct device *apb_clk_dev = cfg->clock_dev;
	struct elan_em32_clock_control_subsys apb_clk_subsys;
	int ret = 0;

	/* Disable TRNG clock gate to enable TRNG clock */
	em32_disable_clkgate();

	/* Enable clock to specified peripheral if clock device is available */
	if (apb_clk_dev != NULL) {
		/* Check if clock control device is ready */
		if (!device_is_ready(apb_clk_dev)) {
			LOG_ERR("APB clock control device not ready");
			return -ENODEV;
		}

		apb_clk_subsys.clock_group = PCLKG_TRNG;
		LOG_DBG("clock_group=%d.", apb_clk_subsys.clock_group);
		ret = clock_control_on(apb_clk_dev, &apb_clk_subsys);
		if (ret < 0) {
			LOG_ERR("Turn on apb clock fail %d.", ret);
			return ret;
		}
		LOG_INF("TRNG clock enabled successfully");
	} else {
		LOG_WRN("APB clock device is NULL, proceeding without clock control");
	}

	/* Add basic diagnostic logging */
	LOG_INF("Initializing TRNG at base address 0x%08x", (uint32_t)cfg->base);

	/* Small delay to ensure clocks are stable */
	k_msleep(10);

	/* Reset TRNG first */
	TRNG_WRITE(cfg->base, TRNG_CONTROL_OFFSET, 0);
	k_msleep(1); /* Allow reset to complete */

	/* Verify reset completed */
	LOG_DBG("TRNG reset completed");

	/* Initialize TRNG with correct control value */
	uint32_t ctrl = TRNG_CONTROL_VALUE;  /* Use 0x0B as specified */

	LOG_INF("TRNG polling mode enabled");

	/* Write TRNG configuration */
	TRNG_WRITE(cfg->base, TRNG_CONTROL_OFFSET, ctrl);

	/* Verify we can read back the configuration */
	uint32_t readback = TRNG_READ(cfg->base, TRNG_CONTROL_OFFSET);
	LOG_INF("TRNG configured: wrote 0x%08x, read 0x%08x", ctrl, readback);

	data->ready = true;
	return 0;
}

static int em32_trng_start_generation(const struct device *dev)
{
	const struct em32_trng_config *cfg = dev->config;

	/* Write the control value to start TRNG operation */
	TRNG_WRITE(cfg->base, TRNG_CONTROL_OFFSET, TRNG_CONTROL_VALUE);

	LOG_DBG("Started TRNG generation, control=0x%08x", TRNG_CONTROL_VALUE);

	return 0;
}



static const struct entropy_driver_api em32_trng_api = {
	.get_entropy = em32_trng_get_entropy,
};

/* Use the defined constants from register layout */
#define TRNG_BITS_PER_CYCLE    256
#define TRNG_BYTES_PER_CYCLE   TRNG_DATA_SIZE_BYTES  /* 32 bytes */

static int em32_trng_collect_cycle(const struct device *dev, uint8_t *buffer)
{
	const struct em32_trng_config *cfg = dev->config;
	int ret;

	LOG_DBG("Starting TRNG collection cycle");

	/* Start new generation cycle */
	ret = em32_trng_start_generation(dev);
	if (ret < 0) {
		return ret;
	}

	/* Wait for data ready with timeout */
	uint32_t start = k_uptime_get_32();
	uint32_t status;
	int poll_count = 0;

	while ((k_uptime_get_32() - start) < CONFIG_ENTROPY_EM32_TRNG_STARTUP_TIMEOUT_MS) {
		/* Check status register for data ready */
		status = TRNG_READ(cfg->base, TRNG_STATUS_OFFSET);
		poll_count++;

		LOG_DBG("Poll %d: status=0x%08x, data_ready=%s", poll_count, status,
			(status & TRNG_STATUS_DATA_RDY) ? "YES" : "NO");

		if (status & TRNG_STATUS_DATA_RDY) {
			/* Data ready - read 256 bits from 0x10 to 0x2C */
			LOG_DBG("TRNG data ready after %d polls", poll_count);

			/* Read 32 bytes (256 bits) from data registers 0x10-0x2C */
			int i;
			for (i = 0; i < TRNG_DATA_SIZE_BYTES / 4; i++) {
				uint32_t offset = TRNG_DATA_START_OFFSET + (i * 4);
				uint32_t word = TRNG_READ(cfg->base, offset);
				LOG_DBG("Read word %d: 0x%08x from offset 0x%02x", i, word, offset);

				/* Store in little-endian format */
				buffer[i * 4 + 0] = (word >> 0) & 0xFF;
				buffer[i * 4 + 1] = (word >> 8) & 0xFF;
				buffer[i * 4 + 2] = (word >> 16) & 0xFF;
				buffer[i * 4 + 3] = (word >> 24) & 0xFF;
			}

			/* Log first few bytes for debugging */
			LOG_DBG("Generated data: %02x %02x %02x %02x ...",
				buffer[0], buffer[1], buffer[2], buffer[3]);

			return TRNG_DATA_SIZE_BYTES;
		}

		/* Check for error status */
		if (status & TRNG_STATUS_ERROR) {
			LOG_ERR("TRNG error detected in status register: 0x%08x", status);
			return -EIO;
		}

		k_busy_wait(10);
	}
	
	return -ETIMEDOUT;
}

#define EM32_TRNG_INIT(inst) \
	static struct em32_trng_data em32_trng_data_##inst; \
	static const struct em32_trng_config em32_trng_config_##inst = { \
		.base = DT_INST_REG_ADDR(inst), \
		.clock_dev = DEVICE_DT_GET(DT_INST_PHANDLE(inst, clocks)), \
	}; \
    DEVICE_DT_INST_DEFINE(inst, em32_trng_init, NULL, &em32_trng_data_##inst, \
    &em32_trng_config_##inst, POST_KERNEL, CONFIG_ENTROPY_EM32_TRNG_INIT_PRIORITY, &em32_trng_api);

DT_INST_FOREACH_STATUS_OKAY(EM32_TRNG_INIT)
