#define DT_DRV_COMPAT elan_elandev_uart

#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include "../../include/zephyr/drivers/clock_control/clock_control_em32_apb.h"
#include <soc.h>

#define _DEVICE_ID "elan967_uart_dev"

#define UART_STATE_TX_BUSY_MASK    BIT(0)
#define UART_STATE_RX_RDY_MASK     BIT(1)

/* EM32 UART register offsets - corrected per spec */
#define UART_DATA_OFFSET           0x00
#define UART_STATE_OFFSET          0x04
#define UART_CTRL_OFFSET           0x08
#define UART_INTSTACLR_OFFSET      0x0C
#define UART_BAUDDIV_OFFSET        0x10

LOG_MODULE_REGISTER(elan967_uart_dev, CONFIG_UART_LOG_LEVEL);

struct uart_elandev_config {
	uintptr_t base;                 // base address from DTS `reg`
	const struct device *clock_dev; // clock device reference from DTS "clocks" property
	const struct pinctrl_dev_config *pcfg;
};

struct uart_elandev_data {
	uint32_t baudrate;
};

static inline uint32_t uart_em32_read(const struct device *dev, uint32_t offset)
{
	const struct uart_elandev_config *config = dev->config;
	return sys_read32(config->base + offset);
}

static inline void uart_em32_write(const struct device *dev, uint32_t offset, uint32_t value)
{
	const struct uart_elandev_config *config = dev->config;
	sys_write32(value, config->base + offset);
}

static int _uart_poll_in(const struct device *dev, unsigned char *p_char)
{
	/* Check if RX data is available */
	if (!(uart_em32_read(dev, UART_STATE_OFFSET) & UART_STATE_RX_RDY_MASK)) {
		return -1;
	}

	*p_char = uart_em32_read(dev, UART_DATA_OFFSET) & 0xFF;
	return 0;
}

static void _uart_poll_out(const struct device *dev, unsigned char out_char)
{
	/* Wait until TX is not busy */
	while (uart_em32_read(dev, UART_STATE_OFFSET) & UART_STATE_TX_BUSY_MASK) {
		/* spin */
		continue;
	}

	uart_em32_write(dev, UART_DATA_OFFSET, out_char);
}

static int _uart_err_check(const struct device *dev)
{
	uint32_t status = uart_em32_read(dev, UART_STATE_OFFSET);
	int err = 0;

	if (status & BIT(3)) { // RXBUFOVERRUN
		err |= UART_ERROR_OVERRUN;
	}

	return err;
}

static const struct uart_driver_api uart_elandev_api = {
	.poll_in = _uart_poll_in,
	.poll_out = _uart_poll_out,
	.err_check = _uart_err_check,
};

static int uart_elandev_init(const struct device *dev)
{
	const struct uart_elandev_config *cfg = dev->config;
	const struct device *apb_clk_dev = cfg->clock_dev;
	struct uart_elandev_data *data = dev->data;
	struct elan_em32_clock_control_subsys apb_clk_subsys;
	uint32_t apb_clk_rate = 0;
	uint32_t bauddiv;
	uint32_t baudrate = data->baudrate;
	int ret = 0;

	/* Apply pinctrl configuration first so IOShare and IOMUX are set
	 * before configuring UART registers. Some hardware requires the
	 * pin routing to be in place before the peripheral is initialized.
	 */
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("Failed to apply pinctrl state, err=%d.", ret);
		return ret;
	}

	/* Enable clock to specified peripheral */
	apb_clk_subsys.clock_group = PCLKG_UART1;
	LOG_DBG("clock_group=%d.", apb_clk_subsys.clock_group);
	ret = clock_control_on(apb_clk_dev, &apb_clk_subsys);
	if (ret < 0) {
		LOG_ERR("Turn on apb clock fail %d.", ret);
		return ret;
	}

	// Get APB Clock Rate
	ret = clock_control_get_rate(apb_clk_dev, NULL, &apb_clk_rate);
	if (ret < 0) {
		LOG_ERR("Fail to get apb clock rate, err=%d.", ret);
		return ret;
	}
	LOG_DBG("apb_clk_rate=%d.", apb_clk_rate);

	bauddiv = (apb_clk_rate + (baudrate / 2)) / baudrate;
	LOG_DBG("baudrate=%d, bauddiv=%d.", baudrate, bauddiv);

	if (bauddiv < 16) {
		bauddiv = 16;
	}

	uart_em32_write(dev, UART_BAUDDIV_OFFSET, bauddiv);
	uart_em32_write(dev, UART_INTSTACLR_OFFSET, 0xF);
	uart_em32_write(dev, UART_CTRL_OFFSET, 0x3);

	return 0;
}

#define UART_ELANDEV_INIT(index)                                                                   \
	PINCTRL_DT_INST_DEFINE(index);                                                             \
	static struct uart_elandev_data uart_elandev_data_##index = {                              \
		.baudrate = DT_INST_PROP(index, current_speed),                                    \
	};                                                                                         \
                                                                                                   \
	static const struct uart_elandev_config uart_elandev_config_##index = {                    \
		.base = DT_INST_REG_ADDR(index),                                                   \
		.clock_dev = DEVICE_DT_GET(DT_INST_PHANDLE(index, clocks)),                        \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),                                     \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(index, uart_elandev_init, NULL, /* PM control */                     \
			      &uart_elandev_data_##index, &uart_elandev_config_##index,            \
			      PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY, &uart_elandev_api);

DT_INST_FOREACH_STATUS_OKAY(UART_ELANDEV_INIT)
