#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

#define ELAN_SPI_TX_BUF_SIZE 8
#define ELAN_SPI_RX_BUF_SIZE 8
#define READ_REG_HEAD        0x40

uint8_t tx_buf[ELAN_SPI_TX_BUF_SIZE];
uint8_t rx_buf[ELAN_SPI_RX_BUF_SIZE];

static const struct device *spi_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_spi2));

/* 取得 spi2 的 node */
#define SPI_NODE DT_NODELABEL(spi2)

/* 驗證 spi2 node 是否存在並啟用 */
BUILD_ASSERT(DT_NODE_HAS_STATUS(SPI_NODE, okay), "spi2 not okay");

/* 建立 spi_cs_control struct */
static struct spi_cs_control spi_cs = {
	.gpio = GPIO_DT_SPEC_GET(SPI_NODE, cs_gpios),
	.delay = 0,
};

int spi_transaction(const struct device *spi_dev, uint8_t *tx_buf, int tx_len, uint8_t *rx_buf,
		    int rx_len)
{
	struct spi_config spi_cfg = {0};
	struct spi_buf_set tx_bufs;
	struct spi_buf_set rx_bufs;
	struct spi_buf buf_in[1];
	struct spi_buf buf_out[1];

	/* Configure SPI parameters */
	spi_cfg.frequency = 4000000U; /* 4 MHz */
	spi_cfg.operation = (SPI_WORD_SET(8) | SPI_TRANSFER_MSB);
	spi_cfg.slave = 0;
	spi_cfg.cs = spi_cs;
	// printk("spi_cs.pin=%u flags=0x%x \n\r", spi_cs.gpio.pin, spi_cs.gpio.dt_flags);

	buf_out[0].buf = tx_buf;
	buf_out[0].len = tx_len;
	buf_in[0].buf = rx_buf;
	buf_in[0].len = rx_len;

	tx_bufs.count = 1;
	tx_bufs.buffers = buf_out;
	rx_bufs.count = 1;
	rx_bufs.buffers = buf_in;
	spi_transceive(spi_dev, &spi_cfg, &tx_bufs, &rx_bufs);
	return 0;
}

int elan_read_cmd(uint8_t fp_cmd, uint8_t *regdata)
{
	int ret = 0;

	memset(tx_buf, 0, ELAN_SPI_TX_BUF_SIZE);
	memset(rx_buf, 0, ELAN_SPI_RX_BUF_SIZE);

	tx_buf[0] = fp_cmd; /* one byte data read */
	ret = spi_transaction(&spi_dev[0], tx_buf, 2, rx_buf, 2);
	*regdata = rx_buf[1];

	return ret;
}

int elan_read_register(uint8_t regaddr, uint8_t *regdata)
{
	return elan_read_cmd(READ_REG_HEAD + regaddr, regdata);
}

int elan_get_hwid(uint16_t *id)
{
	int rc;
	uint8_t id_hi = 0;
	if (id == NULL) {
		return -1;
	}
	rc = elan_read_register(0x01, &id_hi);
	if (rc) {
		printk("ELAN HW ID read failed %d \n\r", rc);
		return -1;
	}
	*id = id_hi;
	return 0;
}

int main(void)
{
	uint16_t id = 0;
	int status;

	printk("Elan SPI test! \n\r");
	status = elan_get_hwid(&id);
	if (id == 0xb1) {
		printk("ELAN Read Success 0x%02x \n\r", id);
	} else {
		printk("SPI Read fail 0x%02x \n\r", id);
	}

#if 0  // Test Code of Clock Control Driver, Paul@20250717
	printk("Set Clock Out: Group 1, HCLK, 3M.\n");
	// Set Clock Out (CLKO2: PA15/GPIOA15)
	SetCLKOut(1 /* GPIO Group 1 */, 6 /* HCLK */, 31 /* DIV 32*/);
#endif // 0

	return 0;
}
