#define DT_DRV_COMPAT elan_elandev_spi2

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/toolchain.h>
#include <zephyr/logging/log.h>
#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
LOG_MODULE_REGISTER(spi_elandev, CONFIG_SPI_LOG_LEVEL);
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/gpio.h>
#include <../drivers/spi/spi_context.h>
#include <zephyr/drivers/clock_control.h>
#include "../../include/zephyr/drivers/clock_control/clock_control_em32_apb.h"

#define SSP_MASK(regname, name) GENMASK(SSP_##regname##_##name##_MSB, SSP_##regname##_##name##_LSB)

/* PL022 Register definitions */

/*
 * Macros to access SSP Registers with their offsets
 */
#define SSP_CR0(r)   (r + 0x000)
#define SSP_CR1(r)   (r + 0x004)
#define SSP_DR(r)    (r + 0x008)
#define SSP_SR(r)    (r + 0x00C)
#define SSP_CPSR(r)  (r + 0x010)
#define SSP_IMSC(r)  (r + 0x014)
#define SSP_RIS(r)   (r + 0x018)
#define SSP_MIS(r)   (r + 0x01C)
#define SSP_ICR(r)   (r + 0x020)
#define SSP_DMACR(r) (r + 0x024)

/*
 * Control Register 0
 */
#define SSP_CR0_SCR_MSB 15
#define SSP_CR0_SCR_LSB 8
#define SSP_CR0_SPH_MSB 7
#define SSP_CR0_SPH_LSB 7
#define SSP_CR0_SPO_MSB 6
#define SSP_CR0_SPO_LSB 6
#define SSP_CR0_FRF_MSB 5
#define SSP_CR0_FRF_LSB 4
#define SSP_CR0_DSS_MSB 3
#define SSP_CR0_DSS_LSB 0

/* Data size select */
#define SSP_CR0_MASK_DSS SSP_MASK(CR0, DSS)
/* Frame format */
#define SSP_CR0_MASK_FRF SSP_MASK(CR0, FRF)
/* Polarity */
#define SSP_CR0_MASK_SPO SSP_MASK(CR0, SPO)
/* Phase */
#define SSP_CR0_MASK_SPH SSP_MASK(CR0, SPH)
/* Serial Clock Rate */
#define SSP_CR0_MASK_SCR SSP_MASK(CR0, SCR)

/*
 * Control Register 1
 */
#define SSP_CR1_SOD_MSB 3
#define SSP_CR1_SOD_LSB 3
#define SSP_CR1_MS_MSB  2
#define SSP_CR1_MS_LSB  2
#define SSP_CR1_SSE_MSB 1
#define SSP_CR1_SSE_LSB 1
#define SSP_CR1_LBM_MSB 0
#define SSP_CR1_LBM_LSB 0

/* Loopback Mode */
#define SSP_CR1_MASK_LBM SSP_MASK(CR1, LBM)
/* Port Enable */
#define SSP_CR1_MASK_SSE SSP_MASK(CR1, SSE)
/* Controller/Peripheral (Master/Slave) select */
#define SSP_CR1_MASK_MS  SSP_MASK(CR1, MS)
/* Peripheral (Slave) mode output disabled */
#define SSP_CR1_MASK_SOD SSP_MASK(CR1, SOD)

/*
 * Status Register
 */
#define SSP_SR_BSY_MSB 4
#define SSP_SR_BSY_LSB 4
#define SSP_SR_RFF_MSB 3
#define SSP_SR_RFF_LSB 3
#define SSP_SR_RNE_MSB 2
#define SSP_SR_RNE_LSB 2
#define SSP_SR_TNF_MSB 1
#define SSP_SR_TNF_LSB 1
#define SSP_SR_TFE_MSB 0
#define SSP_SR_TFE_LSB 0

/* TX FIFO empty */
#define SSP_SR_MASK_TFE SSP_MASK(SR, TFE)
/* TX FIFO not full */
#define SSP_SR_MASK_TNF SSP_MASK(SR, TNF)
/* RX FIFO not empty */
#define SSP_SR_MASK_RNE SSP_MASK(SR, RNE)
/* RX FIFO full */
#define SSP_SR_MASK_RFF SSP_MASK(SR, RFF)
/* Busy Flag */
#define SSP_SR_MASK_BSY SSP_MASK(SR, BSY)

/*
 * Clock Parameter ranges
 */
#define CPSDVR_MIN 0x02
#define CPSDVR_MAX 0xFE

#define SCR_MIN 0x00
#define SCR_MAX 0xFF

/* Fifo depth */
#define SSP_FIFO_DEPTH 8

/*
 * Register READ/WRITE macros
 */
#define SSP_READ_REG(reg)       (*((volatile uint32_t *)reg))
#define SSP_WRITE_REG(reg, val) (*((volatile uint32_t *)reg) = val)
#define SSP_CLEAR_REG(reg, val) (*((volatile uint32_t *)reg) &= ~(val))

/*
 * Status check macros
 */
#define SSP_BUSY(reg)              (SSP_READ_REG(SSP_SR(reg)) & SSP_SR_MASK_BSY)
#define SSP_RX_FIFO_NOT_EMPTY(reg) (SSP_READ_REG(SSP_SR(reg)) & SSP_SR_MASK_RNE)
#define SSP_TX_FIFO_EMPTY(reg)     (SSP_READ_REG(SSP_SR(reg)) & SSP_SR_MASK_TFE)
#define SSP_TX_FIFO_NOT_FULL(reg)  (SSP_READ_REG(SSP_SR(reg)) & SSP_SR_MASK_TNF)

/*
 * Max frequency
 */
#define MAX_FREQ_CONTROLLER_MODE(pclk) ((pclk) / 2)
#define MAX_FREQ_PERIPHERAL_MODE(pclk) ((pclk) / 12)

struct spi_elandev_config {
	uintptr_t base;                 // base address from DTS `reg`
	const struct device *clock_dev; // clock device reference from DTS "clocks" property
	const struct pinctrl_dev_config *pcfg;
};

struct spi_elandev_data {
	struct spi_context ctx;
	uint32_t tx_count;
	uint32_t rx_count;
};

/* Helper Functions */

static inline uint32_t spi_pl022_calc_prescale(const uint32_t pclk, const uint32_t baud)
{
	uint32_t prescale;

	/* prescale only can take even number */
	for (prescale = CPSDVR_MIN; prescale < CPSDVR_MAX; prescale += 2) {
		if (pclk < (prescale + 2) * CPSDVR_MAX * baud) {
			break;
		}
	}

	return prescale;
}

static inline uint32_t spi_pl022_calc_postdiv(const uint32_t pclk, const uint32_t baud,
					      const uint32_t prescale)
{
	uint32_t postdiv;

	for (postdiv = SCR_MAX + 1; postdiv > SCR_MIN + 1; --postdiv) {
		if (pclk / (prescale * (postdiv - 1)) > baud) {
			break;
		}
	}
	return postdiv - 1;
}

static int spi_pl022_configure(const struct device *dev, const struct spi_config *spicfg)
{
	const struct spi_elandev_config *cfg = dev->config;
	struct spi_elandev_data *data = dev->data;
	const struct device *apb_clk_dev = cfg->clock_dev;
	struct elan_em32_clock_control_subsys apb_clk_subsys;
	const uint16_t op = spicfg->operation;
	uint32_t apb_clk_rate = 0;
	uint32_t prescale;
	uint32_t postdiv;
	uint32_t pclk = 0;
	uint32_t cr0;
	uint32_t cr1;
	int ret = 0;

	if (spi_context_configured(&data->ctx, spicfg)) {
		return 0;
	}

	/* Enable clock to specified peripheral if clock device is available */
	if (apb_clk_dev != NULL) {
		/* Enable clock to specified peripheral */
		apb_clk_subsys.clock_group = PCLKG_SSP2;
		LOG_DBG("clock_group=%d.", apb_clk_subsys.clock_group);
		ret = clock_control_on(apb_clk_dev, &apb_clk_subsys);
		if (ret < 0) {
			LOG_ERR("Turn on apb clock fail %d.", ret);
			return ret;
		}

		// Get APB Clock Rate
		ret = clock_control_get_rate(apb_clk_dev, NULL, &apb_clk_rate);
		LOG_DBG("apb_clk_rate=%d.", apb_clk_rate);
		pclk = apb_clk_rate;
	}
	if (ret < 0 || pclk == 0) {
		LOG_ERR("Fail to get apb clock rate, err=%d.", ret);
		return -EINVAL;
	}

	if (spicfg->frequency > MAX_FREQ_CONTROLLER_MODE(pclk)) {
		LOG_ERR("Frequency is up to %u in controller mode.",
			MAX_FREQ_CONTROLLER_MODE(pclk));
		return -ENOTSUP;
	}

	if (op & SPI_TRANSFER_LSB) {
		LOG_ERR("LSB-first not supported");
		return -ENOTSUP;
	}

	/* Half-duplex mode has not been implemented */
	if (op & SPI_HALF_DUPLEX) {
		LOG_ERR("Half-duplex not supported");
		return -ENOTSUP;
	}

	/* Peripheral mode has not been implemented */
	if (SPI_OP_MODE_GET(op) != SPI_OP_MODE_MASTER) {
		LOG_ERR("Peripheral mode is not supported");
		return -ENOTSUP;
	}

	/* Word sizes other than 8 bits has not been implemented */
	if (SPI_WORD_SIZE_GET(op) != 8) {
		LOG_ERR("Word sizes other than 8 bits are not supported");
		return -ENOTSUP;
	}

	/* configure registers */

	prescale = spi_pl022_calc_prescale(pclk, spicfg->frequency);
	postdiv = spi_pl022_calc_postdiv(pclk, spicfg->frequency, prescale);

	cr0 = 0;
	cr0 |= (postdiv << SSP_CR0_SCR_LSB);
	cr0 |= (SPI_WORD_SIZE_GET(op) - 1);
	cr0 |= (op & SPI_MODE_CPOL) ? SSP_CR0_MASK_SPO : 0;
	cr0 |= (op & SPI_MODE_CPHA) ? SSP_CR0_MASK_SPH : 0;

	cr1 = 0;
	cr1 |= SSP_CR1_MASK_SSE; /* Always enable SPI */
	cr1 |= (op & SPI_MODE_LOOP) ? SSP_CR1_MASK_LBM : 0;

	/* Disable the SSP before it is reconfigured */
	SSP_WRITE_REG(SSP_CR1(cfg->base), 0);
	SSP_WRITE_REG(SSP_CPSR(cfg->base), prescale);
	SSP_WRITE_REG(SSP_CR0(cfg->base), cr0);
	SSP_WRITE_REG(SSP_CR1(cfg->base), cr1);

	data->ctx.config = spicfg;

	return 0;
}

static inline bool spi_pl022_transfer_ongoing(struct spi_elandev_data *data)
{
	return spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx);
}

static void spi_pl022_xfer(const struct device *dev)
{
	const struct spi_elandev_config *cfg = dev->config;
	struct spi_elandev_data *data = dev->data;
	const size_t chunk_len = spi_context_max_continuous_chunk(&data->ctx);
	const void *txbuf = data->ctx.tx_buf;
	void *rxbuf = data->ctx.rx_buf;
	uint32_t txrx;
	size_t fifo_cnt = 0;

	data->tx_count = 0;
	data->rx_count = 0;

	/* Ensure writable */
	while (!SSP_TX_FIFO_EMPTY(cfg->base)) {
		;
	}
	/* Drain RX FIFO */
	while (SSP_RX_FIFO_NOT_EMPTY(cfg->base)) {
		SSP_READ_REG(SSP_DR(cfg->base));
	}

	while (data->rx_count < chunk_len || data->tx_count < chunk_len) {
		/* Fill up fifo with available TX data */
		while (SSP_TX_FIFO_NOT_FULL(cfg->base) && data->tx_count < chunk_len &&
		       fifo_cnt < SSP_FIFO_DEPTH) {
			/* Send 0 in the case of read only operation */
			txrx = 0;

			if (txbuf) {
				txrx = ((uint8_t *)txbuf)[data->tx_count];
			}
			SSP_WRITE_REG(SSP_DR(cfg->base), txrx);
			data->tx_count++;
			fifo_cnt++;
		}
		while (data->rx_count < chunk_len && fifo_cnt > 0) {
			if (!SSP_RX_FIFO_NOT_EMPTY(cfg->base)) {
				continue;
			}

			txrx = SSP_READ_REG(SSP_DR(cfg->base));

			/* Discard received data if rx buffer not assigned */
			if (rxbuf) {
				((uint8_t *)rxbuf)[data->rx_count] = (uint8_t)txrx;
			}
			data->rx_count++;
			fifo_cnt--;
		}
	}
}

static int transceive_polling(const struct device *dev, const struct spi_config *config,
			      const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs)
{
	struct spi_elandev_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	int ret;

	/* Lock the SPI Context */
	spi_context_lock(&data->ctx, false /*asynchronous*/, NULL /*cb*/, NULL /*userdata*/,
			 config);

	ret = spi_pl022_configure(dev, config);
	if (ret < 0) {
		LOG_ERR("spi_pl022_configure ERROR, ret=%d", ret);
		goto error;
	}

	spi_context_buffers_setup(ctx, tx_bufs, rx_bufs, 1);

	spi_context_cs_control(ctx, true);

	do {
		spi_pl022_xfer(dev);
		spi_context_update_tx(ctx, 1, data->tx_count);
		spi_context_update_rx(ctx, 1, data->rx_count);
	} while (spi_pl022_transfer_ongoing(data));

	spi_context_cs_control(&data->ctx, false);

error:
	spi_context_release(&data->ctx, ret);

	return 0;
}

static int spi_e967_transceive(const struct device *dev, const struct spi_config *config,
			       const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs)
{
	int ret = 0;

	ret = transceive_polling(dev, config, tx_bufs, rx_bufs);

	return ret;
}

static int spi_e967_release(const struct device *dev, const struct spi_config *config)
{
	struct spi_elandev_data *data = dev->data;

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static const struct spi_driver_api spi_elan967_api = {.transceive = spi_e967_transceive,
#ifdef CONFIG_SPI_ASYNC
						      .transceive_async = NULL,
#endif /* CONFIG_SPI_ASYNC */
#ifdef CONFIG_SPI_RTIO
						      .submit iodev_submit = NULL,
#endif /* CONFIG_SPI_RTIO */
						      .release = spi_e967_release};

static int spi_e967_init(const struct device *dev)
{
	/* Initialize with lowest frequency */
	const struct spi_config spicfg = {
		.frequency = 0,
		.operation = SPI_WORD_SET(8),
		.slave = 0,
	};
	const struct spi_elandev_config *cfg = dev->config;
	struct spi_elandev_data *data = dev->data;
	int ret = 0;

	ret = spi_pl022_configure(dev, &spicfg);
	if (ret < 0) {
		LOG_ERR("Failed to configure spi");
		return ret;
	}

	ret = spi_context_cs_configure_all(&data->ctx);
	if (ret < 0) {
		LOG_ERR("spi_context_cs_configure_all fail, ret=%d", ret);
		return ret;
	}

	/* Make sure the context is unlocked */
	spi_context_unlock_unconditionally(&data->ctx);

#if 1 // TODO how to handle sensor reset config
	/* Pull high sensor reset for test app*/
	GPIO_SetOutput(GPIOIPA, GPIO_PINSOURCE14, GPIO_PuPd_Floating);
	GPIO_WriteBit(GPIOIPA, GPIO_PIN_14, (BitAction)1);
#endif

	/* Apply pinctrl configuration if available */
	if (cfg->pcfg != NULL) {
		ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret < 0) {
			LOG_ERR("Failed to apply pinctrl state, err=%d.", ret);
			return ret;
		}
	} else {
		LOG_WRN("No pinctrl configuration available, using manual pin setup");
		/* Manual pin configuration for KEIL mode or when pinctrl is not available */
		/* This would typically be done by hardware abstraction layer */
	}

	return 0;
}

#define ELAN_SPI_INIT(index)                                                                       \
	PINCTRL_DT_INST_DEFINE(index);                                                             \
	static struct spi_elandev_data spi_elandev_data_##index = {                                \
		SPI_CONTEXT_INIT_LOCK(spi_elandev_data_##index, ctx),                              \
		SPI_CONTEXT_INIT_SYNC(spi_elandev_data_##index, ctx),                              \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(index), ctx)};                         \
                                                                                                   \
	static const struct spi_elandev_config spi_elandev_config_##index = {                      \
		.base = DT_INST_REG_ADDR(index),                                                   \
		.clock_dev = DEVICE_DT_GET(DT_INST_PHANDLE(index, clocks)),                        \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),                                     \
	};                                                                                         \
                                                                                                   \
	SPI_DEVICE_DT_INST_DEFINE(index, spi_e967_init, NULL, /* PM control */                     \
				  &spi_elandev_data_##index,  /* mutable runtime data */           \
				  &spi_elandev_config_##index, POST_KERNEL,                        \
				  CONFIG_SPI_INIT_PRIORITY, &spi_elan967_api);

DT_INST_FOREACH_STATUS_OKAY(ELAN_SPI_INIT)
