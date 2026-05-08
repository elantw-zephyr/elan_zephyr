

#define DT_DRV_COMPAT elan_elandev_usbd

#define zephyr_official
#ifndef zephyr_official
#include "bsp_config.h"
#endif
#include <../drivers/usb/udc/udc_common.h>
#include <string.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/logging/log.h>
#include "udc_e967.h"

#include "em32f967.h" /* TODO: remove em32f967.h */
#include "soc_967.h" /* TODO: remove soc_967.h and elan_em32.h */

#define __IO_ENQUEUE_LOG__	 0
#define __RESET_SUSPEND_RESUME_LOG__ 0
#define __PATCH_LOG__		 0
#define	__DEBUG_LOG__		 0
#define __GLOBAL_DEBUG_LOG__ 0
#define __EP0_OUT_LOG__      0
#define __EP0_LOG__          0
#define __EPX_OUT_LOG__      0
#define __EPX_IN_LOG__       0

LOG_MODULE_REGISTER(udc_e967, CONFIG_UDC_DRIVER_LOG_LEVEL);

#define USB_NUM_BIDIR_ENDPOINTS 5
#define EP0_MPS                 8
#define EP_MPS                  64

#define _IS_SET_CLEAR_FEATURE_PATCH 1
#define _IS_IO_ROUTE 1

enum UDC_E967_MSG_TYPE {
	UDC_E967_MSG_TYPE_SETUP,
	UDC_E967_MSG_TYPE_XFER,
	UDC_E967_MSG_TYPE_SW_RECONN,
#if (_IS_SET_CLEAR_FEATURE_PATCH)
	UDC_E967_MSG_TYPE_PWR,
#endif
};

struct udc_e967_msg {
	enum UDC_E967_MSG_TYPE type;
	union {
		struct {
			enum udc_event_type type;
		} udc_bus_event;
		struct {
			uint8_t ep;
		} setup;
		struct {
			uint8_t ep;
		} out;
		struct {
			uint8_t ep;
		} in;
		struct {
			uint8_t ep;
		} xfer;
		struct {
			uint8_t sus;
		} pwr;
	};
};

struct udc_e967_config {
	size_t num_of_eps;
	struct udc_ep_config *ep_cfg_in;
	struct udc_ep_config *ep_cfg_out;
	uint32_t ep_cfg_out_size;
	uint32_t ep_cfg_in_size;
	int speed_idx;
	void (*irq_enable_func)(const struct device *dev);
	void (*irq_disable_func)(const struct device *dev);
	void (*make_thread)(const struct device *dev);
};

struct e967_usbd_ep {
	uint8_t idx;
	uint32_t is_in_pkg;
	uint32_t is_out_pkg;
	UDC_EPx_INT_EN *reg_ep_int_en;
	UDC_EPx_INT_STA *reg_ep_int_sta;
	volatile uint32_t *reg_data_cnt;
	volatile uint32_t *reg_data_buf;
};

struct udc_e967_data {
	uint8_t setup_pkg[8];
	uint8_t pending_setup_pkg[8];
	const struct device *dev;
	uint8_t addr;
	struct k_msgq *msgq;
	struct k_thread thread_data;
	uint8_t ep_out_num;
	uint8_t ep_out_num_new;
	
	UDC_EP0_INT_EN *reg_ep0_int_en;
	UDC_EP0_INT_STA *reg_ep0_int_sts;
	uint32_t ep0_out_size;
	uint32_t is_pending_pkg;
	uint32_t is_ep0_in_en;
	uint32_t ep0_xfer_size;
	uint32_t is_addressed_state;
	uint32_t is_configured_state;
	uint32_t is_proc_remote_wakeup;
	volatile uint32_t *reg_ep0_data_buf;
	struct e967_usbd_ep epx_ctrl[USB_NUM_BIDIR_ENDPOINTS - 1];

	EP_BUF_STA *reg_ep_buf_sta;

	UDCCTRL_ *reg_udc_ctrl;
	UDC_CTRL1 *reg_udc_ctrl1;
	UDC_INT_EN *reg_udc_int_en;
	UDC_INT_STA *reg_udc_int_sta;
	UDC_CF_DATA *reg_udc_cf_data;
	E967_PHY_Type *reg_usb_phy;
	E967_LJIRC_Type *reg_ljirc_ctrl;
	E967_USBPLL_Type *reg_usbpll_ctrl;
	E967_XTAL_Type *reg_xtal_ctrl;
	E967_SysReg_Type *reg_sysreg;
};

static int _e967_usbd_xfer_out(const struct device *dev, uint8_t ep);
static int _e967_usbd_xfer_in(const struct device *dev, uint8_t ep);
static int _usbd_ctrl_in(const struct device *dev, uint8_t ep);

#if ((__GLOBAL_DEBUG_LOG__ > 1) || (__DEBUG_LOG__))
void HexDump(const void *data, size_t size)
{
	char ascii[17];
	size_t i, j;
	ascii[16] = '\0';
	printk("[DUMP] %p %i\n", data, size);
	for (i = 0; i < size; ++i) {
		printk("%02X ", ((unsigned char *)data)[i]);
		if (((unsigned char *)data)[i] >= ' ' && ((unsigned char *)data)[i] <= '~') {
			ascii[i % 16] = ((unsigned char *)data)[i];
		} else {
			ascii[i % 16] = '.';
		}
		if ((i + 1) % 8 == 0 || i + 1 == size) {
			printk(" ");
			if ((i + 1) % 16 == 0) {
				printk("|  %s \n", ascii);
			} else if (i + 1 == size) {
				ascii[(i + 1) % 16] = '\0';
				if ((i + 1) % 16 <= 8) {
					printk(" ");
				}
				for (j = (i + 1) % 16; j < 16; ++j) {
					printk("   ");
				}
				printk("|  %s \n", ascii);
			}
		}
	}
}
#endif

static struct e967_usbd_ep *_e967_get_ep(struct udc_e967_data *priv, uint8_t ep_addr)
{
	uint8_t ep_idx;

	ep_idx = USB_EP_GET_IDX(ep_addr);

	if (ep_idx >= USB_NUM_BIDIR_ENDPOINTS || ep_idx == 0) {
		return NULL;
	}

	return &priv->epx_ctrl[ep_idx - 1];
}

static inline void _e967_usbd_sw_disconnect(const struct device *dev)
{
	struct udc_e967_data *priv = udc_get_private(dev);
	priv->reg_usb_phy->USBPHYRSW = 0;
}

static inline void _e967_usbd_sw_connect(const struct device *dev)
{
	struct udc_e967_data *priv = udc_get_private(dev);
	priv->reg_usb_phy->USBPHYRSW = 1;
}

static void _e967_usb_clock_set(struct udc_e967_data *priv, uint8_t Usb_Clk_Sel)
{
	volatile uint32_t reg;
	uint32_t _trim_Code;

	(void)_trim_Code;

	CLKGatingDisable(PCLKG_AIP);

	if ((Usb_Clk_Sel == USB_XTAL_12M) || (Usb_Clk_Sel == USB_XTAL_24M)) {
		// xtal_ljirc_sel  Select : 0 = XTAL, 1= LJIRC
		priv->reg_sysreg->XTALLJIRCSEL = 0;

		// Xtal S1S0 = 01 : 24MHz, 11 : 12MHz
		if (Usb_Clk_Sel == USB_XTAL_12M) {
			priv->reg_xtal_ctrl->XTALFREQSEL = 0x03;
		}

		if (Usb_Clk_Sel == USB_XTAL_24M) {
			priv->reg_xtal_ctrl->XTALFREQSEL = 0x01;
		}

		// Xtal Power on
		priv->reg_xtal_ctrl->XTALPD = 0;

		k_busy_wait(2000);
		do {
			reg = priv->reg_xtal_ctrl->XTALSTABLE;
		} while (reg == 0);
		k_busy_wait(12000);
	} else if (Usb_Clk_Sel == USB_IRC) {
		// Load LJIRC Trim Code : Info 0x100A6090
		_trim_Code = *((uint32_t *)0x100A6090);
		priv->reg_ljirc_ctrl->LJIRCFR = _trim_Code & 0x0000000F;
		priv->reg_ljirc_ctrl->LJIRCCA = (_trim_Code & 0x000001F0) >> 4;
		priv->reg_ljirc_ctrl->LJIRCFC = (_trim_Code & 0x00000E00) >> 9;
		priv->reg_ljirc_ctrl->LJIRCTMV10 = (_trim_Code & 0x00003000) >> 12;

		// Load PHY R Trim Code : Info 0x100A60F0
		_trim_Code = *((uint32_t *)0x100A60F0);
		priv->reg_usb_phy->PHYRTRIM = _trim_Code;

		// xtal_ljirc_sel  Select : 0 = XTAL, 1= LJIRC
		priv->reg_sysreg->XTALLJIRCSEL = 1;
	}

	// LJIRC power down
	priv->reg_ljirc_ctrl->LJIRCPD = 0;

	k_busy_wait(2000);

	// USB Clock Select
	priv->reg_sysreg->USBCLKSEL = 0;

	// UDC Clock Enable
	CLKGatingDisable(PCLKG_UDC);

	// USB PLL PD Disable
	priv->reg_usbpll_ctrl->USBPLLPD = 0;

	// Wait PLL Stable
	while (priv->reg_usbpll_ctrl->USBPLLSTABLE == 0)
		;

	// USB PHY PD Disable
	priv->reg_usb_phy->USBPHYPDB = 1;
}

static const unsigned char _ep_conf_data[6] = {0x43, 0x43, 0x42, 0x42, 0xFA, 0x00};
static const uint32_t ep1_size = 64;
static const uint32_t ep2_size = 64;
static const uint32_t ep3_size = 64;
static const uint32_t ep4_size = 64;

void _ep_internal_setup(struct udc_e967_data *priv)
{
	int index;
	for (index = 0; index < 4; index++) {
		priv->reg_udc_cf_data->UDC_CFDATA.UDC_CF_DATABIT.CONFIG_DATA = _ep_conf_data[index];
		while (priv->reg_udc_cf_data->UDC_CFDATA.UDC_CF_DATABIT.EP_CONFIG_RDY == 0)
			;
	}

	priv->reg_udc_cf_data->UDC_CFDATA.UDC_CF_DATABIT.CONFIG_DATA = _ep_conf_data[4];
	while (priv->reg_udc_cf_data->UDC_CFDATA.UDC_CF_DATABIT.EP_CONFIG_DONE == 0)
		;

	// Endpoint FIFO Depth Setting
	E967_EPBUFDEPTH0 = (ep2_size << 16) + ep1_size;
	E967_EPBUFDEPTH1 = (ep4_size << 16) + ep3_size;
}

void _e967_phy_setup(struct udc_e967_data *priv)
{
	priv->reg_udc_ctrl->UDC_CTRL.UDC_CTRLBIT.UDCEN = 1;
	while (priv->reg_udc_ctrl->UDC_CTRL.UDC_CTRLBIT.UDCRSTRDY == 0)
		;

	priv->reg_usb_phy->USBPHYRSW = 0;

	_ep_internal_setup(priv);
}
void _e967_usb_init(struct udc_e967_data *priv)
{
	_e967_phy_setup(priv);

	priv->reg_udc_int_en->UDCINT_EN.UDC_INTENBIT.RSTINTEN = 1;
	priv->reg_udc_int_en->UDCINT_EN.UDC_INTENBIT.SUSPENDINTEN = 1;
	priv->reg_udc_int_en->UDCINT_EN.UDC_INTENBIT.RESUMEINTEN = 1;

	priv->reg_ep0_int_en->UDCEP0INT_EN.UDC_EP0_INT_ENBIT.SETUPINTEN = 1;
	priv->reg_ep0_int_en->UDCEP0INT_EN.UDC_EP0_INT_ENBIT.EP0ININTEN = 1;
	priv->reg_ep0_int_en->UDCEP0INT_EN.UDC_EP0_INT_ENBIT.EP0OUTINTEN = 1;

	priv->reg_udc_ctrl->UDC_CTRL.UDC_CTRLBIT.EP1EN = 0;
	priv->reg_udc_ctrl->UDC_CTRL.UDC_CTRLBIT.EP2EN = 0;
	priv->reg_udc_ctrl->UDC_CTRL.UDC_CTRLBIT.EP3EN = 0;
	priv->reg_udc_ctrl->UDC_CTRL.UDC_CTRLBIT.EP4EN = 0;

	PHYTEST->PHY_TEST_.PHY_TESTBIT.USBWAKEUPEN = 1;
	CLKGatingDisable(PCLKG_ATRIM);
}
void _e967_epx_init(const struct device *dev)
{
	struct udc_e967_data *priv = udc_get_private(dev);
	struct e967_usbd_ep *pepx;

	pepx = _e967_get_ep(priv, 1);
	if (pepx == NULL) {
		return;
	}

	pepx->idx = 1;
	pepx->is_in_pkg = 0;
	pepx->is_out_pkg = 0;
	pepx->reg_ep_int_en = (UDC_EPx_INT_EN *)UDCEP1INTEN;
	pepx->reg_ep_int_sta = (UDC_EPx_INT_STA *)UDCEP1INTSTA;
	pepx->reg_data_cnt = (volatile uint32_t *)(E967_USB_BASE + 0x50);
	pepx->reg_data_buf = (volatile uint32_t *)(E967_USB_BASE + 0x3C);

	pepx = _e967_get_ep(priv, 2);
	if (pepx == NULL) {
		return;
	}

	pepx->idx = 2;
	pepx->is_in_pkg = 0;
	pepx->is_out_pkg = 0;
	pepx->reg_ep_int_en = (UDC_EPx_INT_EN *)UDCEP2INTEN;
	pepx->reg_ep_int_sta = (UDC_EPx_INT_STA *)UDCEP2INTSTA;
	pepx->reg_data_cnt = (volatile uint32_t *)(E967_USB_BASE + 0x54);
	pepx->reg_data_buf = (volatile uint32_t *)(E967_USB_BASE + 0x40);

	pepx = _e967_get_ep(priv, 3);
	if (pepx == NULL) {
		return;
	}

	pepx->idx = 3;
	pepx->is_in_pkg = 0;
	pepx->is_out_pkg = 0;
	pepx->reg_ep_int_en = (UDC_EPx_INT_EN *)UDCEP3INTEN;
	pepx->reg_ep_int_sta = (UDC_EPx_INT_STA *)UDCEP3INTSTA;
	pepx->reg_data_cnt = (volatile uint32_t *)(E967_USB_BASE + 0x58);
	pepx->reg_data_buf = (volatile uint32_t *)(E967_USB_BASE + 0x44);

	pepx = _e967_get_ep(priv, 4);
	if (pepx == NULL) {
		return;
	}

	pepx->idx = 4;
	pepx->is_in_pkg = 0;
	pepx->is_out_pkg = 0;
	pepx->reg_ep_int_en = (UDC_EPx_INT_EN *)UDCEP4INTEN;
	pepx->reg_ep_int_sta = (UDC_EPx_INT_STA *)UDCEP4INTSTA;
	pepx->reg_data_cnt = (volatile uint32_t *)(E967_USB_BASE + 0x5C);
	pepx->reg_data_buf = (volatile uint32_t *)(E967_USB_BASE + 0x48);

	return;
}

static int _udc_e967_send_msg(const struct device *dev, const struct udc_e967_msg *msg)
{
	struct udc_e967_data *priv = udc_get_private(dev);
	int err;

	err = k_msgq_put(priv->msgq, msg, K_NO_WAIT);
	if (err < 0) {
		k_msgq_purge(priv->msgq);
	}

	return err;
}

#if _IS_IO_ROUTE
void _get_out_pipe_num( const struct device *dev, struct net_buf *buf)
{
	struct udc_e967_data *priv = udc_get_private(dev);
	uint8_t *ptr;
	uint8_t *pEnd;
	uint32_t len;
	
	ptr = buf->data;
	len = buf->len;
	
	if( len <= 9)
		return;
	
	if( ptr[0] == 0x09 && ptr[1] == 0x02) {
		
		pEnd = ptr+len;
		ptr = ptr + ptr[0];
		while( ptr < pEnd )
		{
			if( ptr[0] == 7 && ptr[1] == 5) {
				if( (ptr[2] & 0x80) == 0)
				{
					priv->ep_out_num = ptr[2];
					priv->ep_out_num_new = 3;
					ptr[2] = priv->ep_out_num_new;
				}
			}
			ptr = ptr + ptr[0];
		}
	}
}
#endif

static int udc_e967_ep_enqueue(const struct device *dev, struct udc_ep_config *const cfg,
			       struct net_buf *buf)
{
#if _IS_IO_ROUTE
	struct udc_e967_data *priv = udc_get_private(dev);
	struct udc_ep_config *new_cfg;
#endif
	unsigned int lock_key;
	struct udc_e967_msg msg = {0};
	uint32_t isHalt;
	uint8_t ep = cfg->addr;
	//uint8_t *pData;


#if (__DEBUG_LOG__)
	printk("[INFO] enqueue ep:0x%2x, buf=%p\n", ep, buf);
#endif

#if (__IO_ENQUEUE_LOG__)
	uint16_t len;
	const struct udc_buf_info *bi;
	if(buf) {
		bi = udc_get_buf_info(buf);
		if( bi) {
			len = buf->len;
			if( (ep & 0x80) != 0x80) {
				len = buf->size;
			}
			if( bi->setup) {
				printk("[INFO] enqueue:setup ep:0x%2x, buf=%p, buf_len=%i\n", ep, buf, len);
			}else if( bi->data) {
				printk("[INFO] enqueue:data ep:0x%2x, buf=%p, buf_len=%i\n", ep, buf, len);
			}else if( bi->status) {
				printk("[INFO] enqueue:status ep:0x%2x, buf=%p, buf_len=%i\n", ep, buf, len);
			}else{
				printk("[INFO] enqueue ep:0x%2x, buf=%p, buf_len=%i\n", ep, buf, len);
			}
		}
	}
#endif

#if _IS_IO_ROUTE
	if( ep == USB_CONTROL_EP_IN) {
		_get_out_pipe_num( dev, buf);
	}

	if( (priv->ep_out_num != 0) && (ep == priv->ep_out_num) ) {
		new_cfg = udc_get_ep_cfg( dev, priv->ep_out_num_new);
		udc_buf_put( new_cfg, buf);
		ep = priv->ep_out_num_new;
	}else {
		udc_buf_put( cfg, buf);
	}
#else
	udc_buf_put( cfg, buf);
#endif

	lock_key = irq_lock();
	isHalt = cfg->stat.halted;
	irq_unlock(lock_key);

	if (!isHalt) {
		msg.type = UDC_E967_MSG_TYPE_XFER;
		msg.xfer.ep = ep;
		_udc_e967_send_msg(dev, &msg);
	} else {
#if (__GLOBAL_DEBUG_LOG__ > 0)
		printk("[INFO] ep:0x%2x is halted, enqueue buf=%p fail\n", ep, buf);
#endif
	}

	return 0;
}

static int udc_e967_ep_dequeue(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	udc_ep_cancel_queued(dev, ep_cfg);

	return 0;
}

void _udc_ep_set_halt(struct udc_e967_data *priv, struct udc_ep_config *const cfg, bool isHalt)
{
	uint8_t ep_idx;

	ep_idx = USB_EP_GET_IDX(cfg->addr);
	cfg->stat.halted = isHalt;

#if (__DEBUG_LOG__)
	if( isHalt) {
		printk("[INFO] ep:0x%2x is halted\n", cfg->addr);
	}else{
		printk("[INFO] ep:0x%2x is active\n", cfg->addr);
	}
#endif

	if (isHalt) {
		if (ep_idx == 0) {
			priv->reg_udc_ctrl1->UDC_CTRL1_.UDCCTRL1BIT.STALL = 1;
		} else if (ep_idx == 1) {
			priv->reg_udc_ctrl1->UDC_CTRL1_.UDCCTRL1BIT.EP1STALL = 1;
		} else if (ep_idx == 2) {
			priv->reg_udc_ctrl1->UDC_CTRL1_.UDCCTRL1BIT.EP2STALL = 1;
		} else if (ep_idx == 3) {
			priv->reg_udc_ctrl1->UDC_CTRL1_.UDCCTRL1BIT.EP3STALL = 1;
		} else if (ep_idx == 4) {
			priv->reg_udc_ctrl1->UDC_CTRL1_.UDCCTRL1BIT.EP4STALL = 1;
		}
	} else {
		if (ep_idx == 0) {
			priv->reg_udc_ctrl1->UDC_CTRL1_.UDCCTRL1BIT.STALL = 0;
		} else if (ep_idx == 1) {
			priv->reg_udc_ctrl1->UDC_CTRL1_.UDCCTRL1BIT.EP1STALL = 0;
		} else if (ep_idx == 2) {
			priv->reg_udc_ctrl1->UDC_CTRL1_.UDCCTRL1BIT.EP2STALL = 0;
		} else if (ep_idx == 3) {
			priv->reg_udc_ctrl1->UDC_CTRL1_.UDCCTRL1BIT.EP3STALL = 0;
		} else if (ep_idx == 4) {
			priv->reg_udc_ctrl1->UDC_CTRL1_.UDCCTRL1BIT.EP4STALL = 0;
		}
	}
}

/* Halt endpoint. Halted endpoint should respond with a STALL handshake. */
static int udc_e967_ep_set_halt(const struct device *dev, struct udc_ep_config *const cfg)
{
	struct udc_e967_data *priv = udc_get_private(dev);
	_udc_ep_set_halt(priv, cfg, true);
	return 0;
}

/*
 * Opposite to halt endpoint. If there are requests in the endpoint queue,
 * the next transfer should be prepared.
 */
static int udc_e967_ep_clear_halt(const struct device *dev, struct udc_ep_config *const cfg)
{
	struct udc_e967_data *priv = udc_get_private(dev);
	_udc_ep_set_halt(priv, cfg, false);
	return 0;
}

static int udc_e967_host_wakeup(const struct device *dev)
{
	struct udc_e967_data *priv = udc_get_private(dev);

	priv->reg_udc_ctrl1->UDC_CTRL1_.UDCCTRL1BIT.DEVRESUME = 1;
	k_busy_wait( 10000);
	priv->reg_udc_ctrl1->UDC_CTRL1_.UDCCTRL1BIT.DEVRESUME = 0;

	return 0;
}

static void re_issue_pending_pkg(const struct device *dev)
{
	struct udc_e967_data *priv = udc_get_private(dev);
	struct udc_e967_msg msg;
	int i;

	if( priv->is_pending_pkg) {

#if ( __EP0_LOG__ > 0)
		uint8_t *ptr;
		ptr = priv->pending_setup_pkg;
		printk("[INFO] re issue pkg: %x %x %x %x %x %x %x %x \n", 
				ptr[0], ptr[1], ptr[2], ptr[3], ptr[4], ptr[5], ptr[6], ptr[7]
			);
#endif

		for( i=0; i<8; i++) {
			priv->setup_pkg[i] = priv->pending_setup_pkg[i];
		}
		priv->is_pending_pkg = 0;
		
		msg.type = UDC_E967_MSG_TYPE_SETUP;
		k_msgq_put(priv->msgq, &msg, K_NO_WAIT);
		
		return;
	}
	
	return;
}

static int do_patch_proc(const struct device *dev) {
	struct udc_e967_data *priv = udc_get_private(dev);
	int i;

	if (priv->is_addressed_state == 0) {
		if (*((uint32_t *)(priv->setup_pkg)) == 0x01000680 &&
		    *((uint16_t *)(priv->setup_pkg + 6)) >= 18) {

#if (__PATCH_LOG__)
			printk("[SUCCESS] pending set-address-op \n");
#endif
			priv->is_addressed_state = 1;

			return 0;
		}
	}else if (priv->is_addressed_state == 1) {
		
#if (__PATCH_LOG__)
		printk("[SUCCESS] issue set-address-op \n");
#endif
		
		priv->is_addressed_state = 2;
		
		for(i=0; i<8; i++) {
			priv->pending_setup_pkg[i] = priv->setup_pkg[i];
		}
		priv->is_pending_pkg = 1;
		
		priv->setup_pkg[0] = 0x00;
		priv->setup_pkg[1] = 0x05;
		priv->setup_pkg[2] = 0x0f;
		priv->setup_pkg[3] = 0x00;
		priv->setup_pkg[4] = 0x00;
		priv->setup_pkg[5] = 0x00;
		priv->setup_pkg[6] = 0x00;
		priv->setup_pkg[7] = 0x00;

		udc_setup_received( dev, priv->setup_pkg);
		return 1;
	}


	if (priv->is_configured_state == 0) {
		if( *((uint32_t*)(priv->setup_pkg)) == 0x02000680 && *((uint16_t*)(priv->setup_pkg+6)) > 18) {

#if (__PATCH_LOG__)
			printk("[SUCCESS] pending set-configuration-op \n");
#endif
			priv->is_configured_state = 1;

			return 0;
		}
	}else if (priv->is_configured_state == 1) {

#if (__PATCH_LOG__)
			printk("[SUCCESS] issue set-configuration-op \n");
#endif
		
			priv->is_configured_state = 2;

			for(i=0; i<8; i++) {
				priv->pending_setup_pkg[i] = priv->setup_pkg[i];
			}
			priv->is_pending_pkg = 1;

			priv->setup_pkg[0] = 0x00;
			priv->setup_pkg[1] = 0x09;
			priv->setup_pkg[2] = 0x01;
			priv->setup_pkg[3] = 0x00;
			priv->setup_pkg[4] = 0x00;
			priv->setup_pkg[5] = 0x00;
			priv->setup_pkg[6] = 0x00;
			priv->setup_pkg[7] = 0x00;

			udc_setup_received( dev, priv->setup_pkg);

			return 1;
	}

	return 0;
}



#if (_IS_SET_CLEAR_FEATURE_PATCH)
int _handle_set_feature_remote_wakeup( const struct device *dev, uint32_t isSet)
{
	struct udc_e967_data *priv = udc_get_private(dev);
	struct udc_e967_msg msg = {0};
	int i;

	priv->is_pending_pkg = 0;
	for( i=0; i<8; i++) {
		priv->pending_setup_pkg[i] = 0x0;
		priv->setup_pkg[i] = 0x0;
	}

	if( priv->is_configured_state < 3) {
		return 0;
	}

#if (__PATCH_LOG__)
	if(isSet)
		printk("[SUCCESS] issue set-feature-op \n");
	else
		printk("[SUCCESS] issue clear-feature-op \n");
#endif


	msg.type = UDC_E967_MSG_TYPE_PWR;
	
	if(isSet) {
		msg.pwr.sus = 1;
	}else{
		msg.pwr.sus = 0;
	}

	k_msgq_put(priv->msgq, &msg, K_NO_WAIT);

	return 1;
}
#endif


static int udc_e967_msg_handler_setup(const struct device *dev, const struct udc_e967_msg *msg)
{
	struct udc_e967_data *priv = udc_get_private(dev);
	uint8_t *pSetupPkg;
	uint16_t xfer_size;
	struct udc_ep_config *ep_ctrl_in;
	struct udc_ep_config *ep_ctrl_out;

	pSetupPkg = priv->setup_pkg;
	xfer_size = *((uint16_t *)(pSetupPkg + 6));

#if (__EP0_LOG__ > 1)
	uint8_t *ptr;
	ptr = pSetupPkg;
	printk("[INFO] setup handler\n");
	printk("[INFO]: %x %x %x %x %x %x %x %x, xfer_size=%d\n", ptr[0], ptr[1], ptr[2], ptr[3], ptr[4],
	       ptr[5], ptr[6], ptr[7], xfer_size);
#endif

	if( do_patch_proc(dev)) {
		return 0;
	}

	ep_ctrl_in = udc_get_ep_cfg(dev, USB_CONTROL_EP_IN);
	ep_ctrl_out = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);

	udc_ep_set_busy(ep_ctrl_in, false);
	udc_ep_set_busy(ep_ctrl_out, false);

	_udc_ep_set_halt(priv, ep_ctrl_in, false);
	_udc_ep_set_halt(priv, ep_ctrl_out, false);

	udc_setup_received( dev, pSetupPkg);
	priv->ep0_xfer_size = xfer_size;
	priv->is_ep0_in_en = 1;

	return 0;
}

/* Message handler for S/W reconnect */
static int _e967_usbd_msg_handle_sw_reconn(const struct device *dev, struct udc_e967_msg *msg)
{
	int err = 0;
	return err;
}

int _usbd_ctrl_out(const struct device *dev, uint8_t ep)
{
	struct udc_ep_config *ep_cfg;
	struct net_buf *buf;
	struct udc_buf_info *bi;

	ep_cfg = udc_get_ep_cfg(dev, ep);
	buf = udc_buf_peek(ep_cfg);
	
	if( buf == NULL) 
		return 0;
		
	bi = udc_get_buf_info(buf);
	if( bi->setup) {
		return 0;
	}
		
		
	if (bi->status) {
#if (__EP0_OUT_LOG__)
		printk("[INFO] _usbd_ctrl_out: this is status pkg \n");
#endif
		buf = udc_buf_get(ep_cfg);
		udc_submit_ep_event(dev, buf, 0);
		return 0;
	}

#if (__EP0_OUT_LOG__)
	printk("[ERROR] _usbd_ctrl_out: this is data pkg \n");
#endif

	return 0;
}

int _usbd_ctrl_handler(const struct device *dev, uint8_t ep)
{
	int ret;
	if (USB_EP_DIR_IS_OUT(ep)) {
		ret = _usbd_ctrl_out(dev, ep);
	} else {
		ret = _usbd_ctrl_in(dev, ep);
	}
	return ret;
}

#ifndef zephyr_official
static int _epx_check_buffer_empty(struct udc_e967_data *priv, uint8_t ep)
{
	uint8_t ep_idx;
	uint8_t ep_dir;

	ep_idx = USB_EP_GET_IDX(ep);
	ep_dir = USB_EP_GET_DIR(ep);

	if (ep_dir == USB_EP_DIR_OUT) {
		if (ep_idx == 0) {
			return priv->reg_ep_buf_sta->EP_BUFSTA.UDC_EPX_INT_STABIT.EP0OUTBUFEMPTY;
		} else if (ep_idx == 1) {
			return priv->reg_ep_buf_sta->EP_BUFSTA.UDC_EPX_INT_STABIT.EP1OUTBUFEMPTY;
		} else if (ep_idx == 2) {
			return priv->reg_ep_buf_sta->EP_BUFSTA.UDC_EPX_INT_STABIT.EP2OUTBUFEMPTY;
		} else if (ep_idx == 3) {
			return priv->reg_ep_buf_sta->EP_BUFSTA.UDC_EPX_INT_STABIT.EP3OUTBUFEMPTY;
		} else if (ep_idx == 4) {
			return priv->reg_ep_buf_sta->EP_BUFSTA.UDC_EPX_INT_STABIT.EP4OUTBUFEMPTY;
		}
	} else {
		if (ep_idx == 0) {
			return priv->reg_ep_buf_sta->EP_BUFSTA.UDC_EPX_INT_STABIT.EP0INBUFEMPTY;
		} else if (ep_idx == 1) {
			return priv->reg_ep_buf_sta->EP_BUFSTA.UDC_EPX_INT_STABIT.EP1INBUFEMPTY;
		} else if (ep_idx == 2) {
			return priv->reg_ep_buf_sta->EP_BUFSTA.UDC_EPX_INT_STABIT.EP2INBUFEMPTY;
		} else if (ep_idx == 3) {
			return priv->reg_ep_buf_sta->EP_BUFSTA.UDC_EPX_INT_STABIT.EP3INBUFEMPTY;
		} else if (ep_idx == 4) {
			return priv->reg_ep_buf_sta->EP_BUFSTA.UDC_EPX_INT_STABIT.EP4INBUFEMPTY;
		}
	}

	return -1;
}
#endif

static int _e967_usbd_msg_handle_xfer(const struct device *dev, struct udc_e967_msg *msg)
{
	uint8_t ep;

	ep = msg->xfer.ep;

	if (USB_EP_GET_IDX(ep) == 0) {
		_usbd_ctrl_handler(dev, ep);
		return 0;
	}

	if (USB_EP_DIR_IS_OUT(ep)) {
		_e967_usbd_xfer_out(dev, ep);
	} else {
		_e967_usbd_xfer_in(dev, ep);
	}

	return 0;
}

#if (_IS_SET_CLEAR_FEATURE_PATCH)
static int _e967_usbd_msg_handle_pwr(const struct device *dev, struct udc_e967_msg *msg)
{

	struct udc_e967_data *priv = udc_get_private(dev);
	struct udc_e967_msg setup_msg;
	uint8_t sus;

	sus = msg->pwr.sus;

#if (__PATCH_LOG__)
	if(sus)
		printk("[SUCCESS] proc set-feature-op msg, sus=%i\n", sus);
	else
		printk("[SUCCESS] proc clear-feature-op msg, sus=%i\n", sus);
#endif

	priv->setup_pkg[0] = 0x00;
	if(sus) {
		priv->is_proc_remote_wakeup = 1;
		priv->setup_pkg[1] = 0x03;
	}else{
		priv->is_proc_remote_wakeup = 2;
		priv->setup_pkg[1] = 0x01;
	}
	priv->setup_pkg[2] = 0x01;
	priv->setup_pkg[3] = 0x00;
	priv->setup_pkg[4] = 0x00;
	priv->setup_pkg[5] = 0x00;
	priv->setup_pkg[6] = 0x00;
	priv->setup_pkg[7] = 0x00;

	setup_msg.type = UDC_E967_MSG_TYPE_SETUP;
	k_msgq_put(priv->msgq, &setup_msg, K_NO_WAIT);
	//udc_setup_received( dev, priv->setup_pkg);

	return 0;
}
#endif


static void e967_usbd_msg_handler(const struct device *dev)
{
	struct udc_e967_data *priv = udc_get_private(dev);
	struct udc_e967_msg msg;
	int err;

	while (true) {
		if (k_msgq_get(priv->msgq, &msg, K_FOREVER)) {
			continue;
		}

		err = 0;

		udc_lock_internal(dev, K_FOREVER);

		switch (msg.type) {
		case UDC_E967_MSG_TYPE_SETUP:
			err = udc_e967_msg_handler_setup(dev, &msg);
			break;
		case UDC_E967_MSG_TYPE_XFER:
			err = _e967_usbd_msg_handle_xfer(dev, &msg);
			break;
#if (_IS_SET_CLEAR_FEATURE_PATCH)			
		case UDC_E967_MSG_TYPE_PWR:
			err = _e967_usbd_msg_handle_pwr(dev, &msg);
			break;
#endif
		case UDC_E967_MSG_TYPE_SW_RECONN:
			err = _e967_usbd_msg_handle_sw_reconn(dev, &msg);
			break;

		default:
			__ASSERT_NO_MSG(false);
		}

		udc_unlock_internal(dev);

		if (err) {
			udc_submit_event(dev, UDC_EVT_ERROR, err);
		}
	}
}

static void e967_usb_suspend_isr(const struct device *dev)
{
	struct udc_e967_data *priv = udc_get_private(dev);

	if (priv->reg_udc_int_sta->UDCINT_STA.UDC_INT_STABIT.SUSPENDINTSF == 1) {
		priv->reg_udc_int_sta->UDCINT_STA.UDC_INT_STABIT.SUSPENDINTSFCLR = 1;
	}

#if (__RESET_SUSPEND_RESUME_LOG__)
	printk("[INFO][ISR] >>> usb [suspend] signal\n");
#endif

#if (_IS_SET_CLEAR_FEATURE_PATCH)
	if (_handle_set_feature_remote_wakeup( dev, 1))
	{
		return;
	}

	udc_set_suspended(dev, true);
	udc_submit_event(dev, UDC_EVT_SUSPEND, 0);
#else
	udc_set_suspended(dev, true);
	udc_submit_event(dev, UDC_EVT_SUSPEND, 0);
#endif

	return;
}
static void e967_usb_resume_isr(const struct device *dev)
{
	struct udc_e967_data *priv = udc_get_private(dev);
	if (priv->reg_udc_int_sta->UDCINT_STA.UDC_INT_STABIT.RESUMEINTSF == 1) {
	
#if (__RESET_SUSPEND_RESUME_LOG__)
		printk("[INFO][ISR] >>> usb [resume] signal\n");
#endif		
	
		udc_set_suspended(dev, false);
		udc_submit_event(dev, UDC_EVT_RESUME, 0);

#if (_IS_SET_CLEAR_FEATURE_PATCH)
		_handle_set_feature_remote_wakeup( dev, 0);
#endif
		priv->reg_udc_int_sta->UDCINT_STA.UDC_INT_STABIT.RESUMEINTSFCLR = 1;
	}

	return;
}
static void e967_usb_reset_isr(const struct device *dev)
{
	struct udc_e967_data *priv = udc_get_private(dev);
	int len, i;
	uint8_t tmp;
	
	if (priv->reg_udc_int_sta->UDCINT_STA.UDC_INT_STABIT.RSTINTSF == 1) {
		priv->reg_udc_int_sta->UDCINT_STA.UDC_INT_STABIT.RSTINTSFCLR = 1;
	}
	
#if (__RESET_SUSPEND_RESUME_LOG__)
	printk("[INFO][ISR] >>> usb [reset] signal\n");
#endif


	// EP1 Buffer Clear
	UDCEP1INTEN->UDCEP1INT_EN.UDC_EP1_INT_ENBIT.EP1BUFCLR = 1;
	UDCEP1INTEN->UDCEP1INT_EN.UDC_EP1_INT_ENBIT.EP1BUFCLR = 0;

	// EP2 Buffer Clear
	UDCEP2INTEN->UDCEP2INT_EN.UDC_EP2_INT_ENBIT.EP2BUFCLR = 1;
	UDCEP2INTEN->UDCEP2INT_EN.UDC_EP2_INT_ENBIT.EP2BUFCLR = 0;

	// EP3 Buffer Clear
	UDCEP3INTEN->UDCEP3INT_EN.UDC_EP3_INT_ENBIT.EP3BUFCLR = 1;
	UDCEP3INTEN->UDCEP3INT_EN.UDC_EP3_INT_ENBIT.EP3BUFCLR = 0;

	len = (EP1DATINOUTCNT >> 16);
	for( i = 0 ; i < len ; i++) {
		tmp = EP1BUFDATA;
	}

	len = (EP2DATINOUTCNT >> 16);
	for( i = 0 ; i < len ; i++) {
		tmp = EP2BUFDATA;
	}

	len = (EP3DATINOUTCNT >> 16);
	for( i = 0 ; i < len ; i++) {
		tmp = EP3BUFDATA;
	}

	priv->addr = 0;
	priv->is_addressed_state = 0;
	priv->is_configured_state = 0;
	priv->ep_out_num = 0;
	priv->ep_out_num_new = 0;

	udc_submit_event( dev, UDC_EVT_RESET, 0);
	
	return;
}

static void e967_usb_setup_isr(const struct device *dev)
{
	struct udc_e967_data *priv = udc_get_private(dev);
	struct udc_e967_msg msg = {0};
	uint32_t index;
	int err;

	priv->is_pending_pkg = 0;
	priv->is_ep0_in_en = 0;
	priv->ep0_xfer_size = 0;
	priv->ep0_out_size = 0;

	for (index = 0; index < 8; index++) {
		priv->setup_pkg[index] = (uint8_t)*(priv->reg_ep0_data_buf);
	}

#if (( __EP0_LOG__ > 0)||(__DEBUG_LOG__))
	uint8_t *ptr;
	ptr = priv->setup_pkg;
	printk("\n[INFO][ISR][SETUP]: %x %x %x %x %x %x %x %x\n", ptr[0], ptr[1], ptr[2], ptr[3], ptr[4],
	       ptr[5], ptr[6], ptr[7]);
#endif

	msg.type = UDC_E967_MSG_TYPE_SETUP;
	err = k_msgq_put(priv->msgq, &msg, K_NO_WAIT);
	if (err < 0) {
		goto _exit;
	}

_exit:
	// Clear Setup Interrupt Flag
	UDCEP0INTSTA->UDC_EP0_INTSTA.UDC_EP0_INT_STABIT.SETUPINTSFCLR = 1;
	return;
}

void _e967_proc_ep0_h2d(const struct device *dev)
{
	struct udc_e967_data *priv = udc_get_private(dev);
	struct udc_ep_config *ep_cfg;
	struct net_buf *buf;
	struct udc_buf_info *bi;
	
#if (__EP0_OUT_LOG__)
	printk("[INFO][ISR] EP0-OUT-ISR\n");
#endif

	ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
	buf = udc_buf_peek(ep_cfg);

	if (buf == NULL) {
		goto _exit;
	}

	bi = udc_get_buf_info(buf);
	if( bi->setup) {
#if (__EP0_OUT_LOG__)
		printk("[INFO][ISR] buf is setup pkg\n");
#endif
		return;
	}


	if (priv->ep0_out_size == 1) {
		goto _exit;
	}

	priv->ep0_out_size = 1;

_exit:
	priv->reg_ep0_int_sts->UDC_EP0_INTSTA.UDC_EP0_INT_STABIT.EP0OUTINTSFCLR = 1;

	return;
}

static int _usbd_ctrl_in(const struct device *dev, uint8_t ep)
{
	struct udc_e967_data *priv = udc_get_private(dev);
	struct udc_ep_config *ep_cfg;
	struct net_buf *buf;
	struct udc_buf_info *bi;

	ep_cfg = udc_get_ep_cfg(dev, ep);
	buf = udc_buf_peek(ep_cfg);
	
	if( buf == NULL)
		return 0;

	bi = udc_get_buf_info(buf);

	if (bi->status) {
		if (priv->is_addressed_state == 2) {
			buf = udc_buf_get(ep_cfg);
			udc_submit_ep_event(dev, buf, 0);
			priv->is_addressed_state = 3;
			re_issue_pending_pkg(dev);
#if (__PATCH_LOG__)
			printk("[SUCCESS] dev is addressed\n");
#endif
			return 0;
		} else if (priv->is_configured_state == 2) {
			buf = udc_buf_get(ep_cfg);
			udc_submit_ep_event(dev, buf, 0);
			priv->is_configured_state = 3;
			re_issue_pending_pkg(dev);
#if (__PATCH_LOG__)
			printk("[SUCCESS] dev is configured\n");
#endif
			return 0;
		}
#if (_IS_SET_CLEAR_FEATURE_PATCH)
		else if ( priv->is_proc_remote_wakeup)
		{
			buf = udc_buf_get(ep_cfg);
			udc_submit_ep_event(dev, buf, 0);

			if( priv->is_proc_remote_wakeup == 1) {
#if (__PATCH_LOG__)
				printk("[SUCCSSS] issue UDC_EVT_SUSPEND evt\n");
#endif
				udc_set_suspended(dev, true);
				udc_submit_event(dev, UDC_EVT_SUSPEND, 0);
			}

			priv->is_proc_remote_wakeup = 0;

#if (__PATCH_LOG__)
			printk("[SUCCSSS] set/clear remote-wake pkg is completed\n");
#endif
		}
#endif
		else{

#if (__DEBUG_LOG__)
			printk("[INFO] status buf %p is submit\n", buf);
#endif
			buf = udc_buf_get(ep_cfg);
			udc_submit_ep_event(dev, buf, 0);
		}

	}

	return 0;
}

void _e967_proc_ep0_d2h_isr(const struct device *dev)
{
	struct udc_e967_data *priv = udc_get_private(dev);
	struct udc_ep_config *ep_cfg;
	struct net_buf *nbuf;
	uint8_t *data_ptr;
	uint32_t data_len;
	uint32_t len, i;

	if ( !(priv->is_ep0_in_en)) {
		goto _exit;
	}

	ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_IN);
	nbuf = udc_buf_peek(ep_cfg);

	if (nbuf == NULL) {
		goto _exit;
	}

#if (__EP0_LOG__>2)
	printk("[INFO][ISR] EP0-IN-ISR\n");
#endif

	data_ptr = nbuf->data;
	data_len = nbuf->len;

	len = EP0_MPS;
	if (len > data_len) {
		len = data_len;
	}

	for (i = 0; i < len; i++) {
		*(priv->reg_ep0_data_buf) = *data_ptr;
		data_ptr++;
	}
	priv->reg_ep0_int_en->UDCEP0INT_EN.UDC_EP0_INT_ENBIT.EP0DATREADY = 1;

	net_buf_pull(nbuf, len);
	
	if( priv->ep0_xfer_size > len)
		priv->ep0_xfer_size = priv->ep0_xfer_size - len;
	else
		priv->ep0_xfer_size = 0;

	data_len = nbuf->len;

#if ((__EP0_LOG__ > 1)||(__DEBUG_LOG__))
	printk("[INFO] ep0_d2h : len_of_buf=%d, sent=%d, xfer=%d\n", data_len, len, priv->ep0_xfer_size);
#endif

	if (data_len != 0) {
		goto _exit;
	}

	if( priv->ep0_xfer_size != 0) {
		if( len == EP0_MPS)
			goto _exit;
	}

	nbuf = udc_buf_get(ep_cfg);
	udc_submit_ep_event(dev, nbuf, 0);
#if (__EP0_LOG__ > 0)		
	printk("[INFO][ISR] ep0_d2h : %p is submit\n", nbuf);
#endif

_exit:
	priv->reg_ep0_int_sts->UDC_EP0_INTSTA.UDC_EP0_INT_STABIT.EP0ININTSFCLR = 1;
	return;
}

static int _e967_usbd_xfer_in(const struct device *dev, uint8_t ep)
{
	struct udc_e967_data *priv = udc_get_private(dev);
	struct udc_ep_config *ep_cfg;
	struct e967_usbd_ep *ep_ctrl;

	ep_ctrl = _e967_get_ep(priv, ep);
	ep_cfg = udc_get_ep_cfg(dev, ep);

#if (__EPX_IN_LOG__ > 1)
	printk("[INFO] _e967_usbd_xfer_in, ep=0x%02x\n", ep);
#endif
	if (ep_ctrl->is_in_pkg) {
		ep_ctrl->reg_ep_int_en->UDCEPx_INT_EN.UDC_EPx_INT_ENBIT.EPxININTEN = 0;
		ep_ctrl->is_in_pkg = 0;
		ep_ctrl->reg_ep_int_en->UDCEPx_INT_EN.UDC_EPx_INT_ENBIT.EPxININTEN = 1;
	}

	return 0;
}

static void _e967_proc_epx_d2h(const struct device *dev, uint8_t ep_addr)
{
	struct udc_e967_data *priv = udc_get_private(dev);
	struct udc_ep_config *ep_cfg;
	struct e967_usbd_ep *ep_ctrl;
	struct net_buf *nbuf;
	uint8_t *data_ptr;
	uint32_t data_len, len, i;
	int err;

	ep_ctrl = _e967_get_ep(priv, ep_addr);
	ep_cfg = udc_get_ep_cfg(dev, ep_addr);
	nbuf = udc_buf_peek(ep_cfg);

#if (__EPX_IN_LOG__ > 2)
	printk("[INFO] epx_d2h + ep:0x%02x\n", ep_addr);
#endif

	if (ep_ctrl->is_in_pkg) {
		goto _exit;
	}

	if (nbuf == NULL) {
#if (__EPX_IN_LOG__ > 0)
		printk("[ERROR] epx_d2h ep:0x%02x, no bufer available\n", ep_addr);
#endif
		ep_ctrl->is_in_pkg = 1;
		ep_ctrl->reg_ep_int_en->UDCEPx_INT_EN.UDC_EPx_INT_ENBIT.EPxININTEN = 0;
		goto _exit;
	}

	data_ptr = nbuf->data;
	data_len = nbuf->len;

	do {
		priv->reg_udc_ctrl1->UDC_CTRL1_.UDCCTRL1BIT.EPINPREHOLD = 1;
	} while (priv->reg_udc_ctrl1->UDC_CTRL1_.UDCCTRL1BIT.EPINPREHOLD != 1);

	len = data_len;
	if (len > EP_MPS) {
		len = EP_MPS;
	}
	
	*(ep_ctrl->reg_data_cnt) = len;
	if (len > 0) {
		for (i = 0; i < len; i++) {
			*(ep_ctrl->reg_data_buf) = *data_ptr;
			data_ptr++;
		}
	}
	ep_ctrl->reg_ep_int_en->UDCEPx_INT_EN.UDC_EPx_INT_ENBIT.EPxDATREADY = 1;

	priv->reg_udc_ctrl1->UDC_CTRL1_.UDCCTRL1BIT.EPINPREHOLD = 0;

	net_buf_pull(nbuf, len);
	data_len = nbuf->len;

#if (__EPX_IN_LOG__ > 1)
	printk("[INFO] epx_d2h done, ep=0x%02x, size=%i\n", ep_addr, data_len);
#endif

	if (data_len == 0) {
		nbuf = udc_buf_get(ep_cfg);
		err = udc_submit_ep_event(dev, nbuf, 0);

#if (__EPX_IN_LOG__ > 2)
		printk("[INFO] epx_d2h done, request next pkg\n");
#endif
		if (err != 0) {
#if (__EPX_IN_LOG__ > 0)
			printk("[ERROR] _e967_proc_epx_d2h:udc_submit_ep_event fail, err=%i\n",
			       err);
#endif
		}
	}

_exit:
	ep_ctrl->reg_ep_int_sta->UDC_EPx_INTSTA.UDC_EPx_INT_STABIT.EPx_IN_INT_SF_CLR = 1;
}

static void e967_usb_ep_d2h_isr(const struct device *dev)
{
	struct udc_e967_data *priv = udc_get_private(dev);
	struct e967_usbd_ep *ep_ctrl;

	if (priv->reg_ep0_int_sts->UDC_EP0_INTSTA.UDC_EP0_INT_STABIT.EP0ININTSF == 1) {
		_e967_proc_ep0_d2h_isr(dev);
		return;
	}

	ep_ctrl = _e967_get_ep(priv, USB_EP_DIR_IN | 1);
	if (ep_ctrl->reg_ep_int_sta->UDC_EPx_INTSTA.UDC_EPx_INT_STABIT.EPx_IN_INT_SF == 1) {
		_e967_proc_epx_d2h(dev, USB_EP_DIR_IN | 1);
		return;
	}

	ep_ctrl = _e967_get_ep(priv, USB_EP_DIR_IN | 2);
	if (ep_ctrl->reg_ep_int_sta->UDC_EPx_INTSTA.UDC_EPx_INT_STABIT.EPx_IN_INT_SF == 1) {
		_e967_proc_epx_d2h(dev, USB_EP_DIR_IN | 2);
		return;
	}

	ep_ctrl = _e967_get_ep(priv, USB_EP_DIR_IN | 3);
	if (ep_ctrl->reg_ep_int_sta->UDC_EPx_INTSTA.UDC_EPx_INT_STABIT.EPx_IN_INT_SF == 1) {
		_e967_proc_epx_d2h(dev, USB_EP_DIR_IN | 3);
		return;
	}

	ep_ctrl = _e967_get_ep(priv, USB_EP_DIR_IN | 4);
	if (ep_ctrl->reg_ep_int_sta->UDC_EPx_INTSTA.UDC_EPx_INT_STABIT.EPx_IN_INT_SF == 1) {
		_e967_proc_epx_d2h(dev, USB_EP_DIR_IN | 4);
		return;
	}

	return;
}

static int _e967_usbd_xfer_out(const struct device *dev, uint8_t ep)
{
	struct udc_e967_data *priv = udc_get_private(dev);
	struct udc_ep_config *ep_cfg;
	struct e967_usbd_ep *ep_ctrl;
	struct net_buf *buf;
	uint8_t *data_ptr;
	uint32_t data_len, len, i;
	uint32_t isDataOut;
	uint32_t lock_key;

	lock_key = irq_lock();

	ep_ctrl = _e967_get_ep(priv, ep);
	ep_cfg = udc_get_ep_cfg(dev, ep);

	buf = udc_buf_peek(ep_cfg);
	if (buf == NULL) {
		goto __exit;
	}


	data_len = net_buf_tailroom(buf);
	data_ptr = net_buf_tail(buf);

#if (__EPX_OUT_LOG__ > 1)
	printk("[INFO] _e967_usbd_xfer_out, ep=0x%2x data, size=%i\n", ep, data_len);
#endif
	isDataOut = 0;

	if (ep_ctrl->is_out_pkg) {
		isDataOut = 1;
	}
	
	if (!isDataOut) {
		goto __exit;
	}

#if (__EPX_OUT_LOG__ > 1)
	printk("[INFO] process ep=0x%2x data, buf=%p\n", ep, buf);
#endif

	do {
		priv->reg_udc_ctrl1->UDC_CTRL1_.UDCCTRL1BIT.EPINPREHOLD = 1;
	} while (priv->reg_udc_ctrl1->UDC_CTRL1_.UDCCTRL1BIT.EPINPREHOLD != 1);

	data_len = net_buf_tailroom(buf);
	data_ptr = net_buf_tail(buf);

	len = *(ep_ctrl->reg_data_cnt);
	len = len >> 16;
	if (len > EP_MPS) {
		len = EP_MPS;
	}
	if (len > data_len) {
		len = data_len;
	}

	for (i = 0; i < len; i++) {
		*data_ptr = *(ep_ctrl->reg_data_buf);
		data_ptr++;
	}

	priv->reg_udc_ctrl1->UDC_CTRL1_.UDCCTRL1BIT.EPINPREHOLD = 0;
	net_buf_add(buf, len);

	data_len = net_buf_tailroom(buf);
	// if( data_len == 0) {
	if (data_len < EP_MPS) {
		buf = udc_buf_get(ep_cfg);
		udc_submit_ep_event(dev, buf, 0);
	}

	ep_ctrl->is_out_pkg = 0;

#if (__EPX_OUT_LOG__ > 1)
	printk("[INFO] _e967_usbd_xfer_out done, ep=0x%2x \n", ep);
#endif

__exit:
	irq_unlock(lock_key);
	return 0;
}

static void _e967_proc_epx_h2d(const struct device *dev, uint8_t ep_addr)
{
	struct udc_e967_data *priv = udc_get_private(dev);
	struct udc_ep_config *ep_cfg;
	struct e967_usbd_ep *ep_ctrl;
	struct net_buf *nbuf;
	uint8_t *data_ptr;
	uint32_t data_len, len, i;

#if (__EPX_OUT_LOG__ > 1)
	printk("[INFO] epx_h2d + ep:0x%02x\n", ep_addr);
#endif

	ep_ctrl = _e967_get_ep(priv, ep_addr);
	ep_cfg = udc_get_ep_cfg(dev, ep_addr);
	nbuf = udc_buf_peek(ep_cfg);

	ep_ctrl->reg_ep_int_sta->UDC_EPx_INTSTA.UDC_EPx_INT_STABIT.EPx_OUT_INT_SF_CLR = 1;

	if (ep_ctrl->is_out_pkg) {
#if (__EPX_OUT_LOG__ > 1)
		printk("[INFO] epx_h2d ep:0x%02x, data should be process in normal code first\n",
		       ep_addr);
#endif
		return;
	}

	data_ptr = NULL;
	data_len = 0;

	if (nbuf == NULL) {
#if (__EPX_OUT_LOG__ > 1)
		printk("[ERROR] epx_h2d ep:0x%02x, no bufer available\n", ep_addr);
#endif
		ep_ctrl->is_out_pkg = 1;
		return;
	}

	data_ptr = net_buf_tail(nbuf);
	data_len = net_buf_tailroom(nbuf);

#if (__EPX_OUT_LOG__ > 1)
	printk("[INFO] epx_h2d ep:0x%02x, data=%p, size=%i\n", ep_addr, data_ptr, data_len);
#endif

	if (data_ptr == NULL) {
		if (data_len != 0) {
#if (__EPX_OUT_LOG__ > 1)
			printk("[ERROR] epx_h2d ep:0x%02x, invalid parameters, buf=NULL, size=%i\n",
			       ep_addr, data_len);
#endif
			return;
		}
	}

	len = 0;

	do {
		priv->reg_udc_ctrl1->UDC_CTRL1_.UDCCTRL1BIT.EPINPREHOLD = 1;
	} while (priv->reg_udc_ctrl1->UDC_CTRL1_.UDCCTRL1BIT.EPINPREHOLD != 1);

	len = *(ep_ctrl->reg_data_cnt);
	len = len >> 16;
	if (len > EP_MPS) {
		len = EP_MPS;
	}
	if (len > data_len) {
		len = data_len;
	}

	if (len > 0) {
		for (i = 0; i < len; i++) {
			*data_ptr = *(ep_ctrl->reg_data_buf);
			data_ptr++;
		}
	}

	priv->reg_udc_ctrl1->UDC_CTRL1_.UDCCTRL1BIT.EPINPREHOLD = 0;

	net_buf_add(nbuf, len);

	data_ptr = net_buf_tail(nbuf);
	data_len = net_buf_tailroom(nbuf);

#if (__EPX_OUT_LOG__ > 1)
	printk("[INFO] epx_h2d done, ep=0x%02x, size=%i\n", ep_addr, data_len);
#endif

	if (data_len < EP_MPS) {
		nbuf = udc_buf_get(ep_cfg);
		udc_submit_ep_event(dev, nbuf, 0);
	}

	return;
}

static void e967_usb_ep_h2d_isr(const struct device *dev)
{
	struct udc_e967_data *priv = udc_get_private(dev);
	struct e967_usbd_ep *ep_ctrl;

	if (priv->reg_ep0_int_sts->UDC_EP0_INTSTA.UDC_EP0_INT_STABIT.EP0OUTINTSF == 1) {
		_e967_proc_ep0_h2d(dev);
	} else {

		ep_ctrl = _e967_get_ep(priv, USB_EP_DIR_OUT | 1);
		if (ep_ctrl->reg_ep_int_sta->UDC_EPx_INTSTA.UDC_EPx_INT_STABIT.EPx_OUT_INT_SF ==
		    1) {
			_e967_proc_epx_h2d(dev, USB_EP_DIR_OUT | 1);
			return;
		}

		ep_ctrl = _e967_get_ep(priv, USB_EP_DIR_OUT | 2);
		if (ep_ctrl->reg_ep_int_sta->UDC_EPx_INTSTA.UDC_EPx_INT_STABIT.EPx_OUT_INT_SF ==
		    1) {
			_e967_proc_epx_h2d(dev, USB_EP_DIR_OUT | 2);
			return;
		}

		ep_ctrl = _e967_get_ep(priv, USB_EP_DIR_OUT | 3);
		if (ep_ctrl->reg_ep_int_sta->UDC_EPx_INTSTA.UDC_EPx_INT_STABIT.EPx_OUT_INT_SF ==
		    1) {
			_e967_proc_epx_h2d(dev, USB_EP_DIR_OUT | 3);
			return;
		}

		ep_ctrl = _e967_get_ep(priv, USB_EP_DIR_OUT | 4);
		if (ep_ctrl->reg_ep_int_sta->UDC_EPx_INTSTA.UDC_EPx_INT_STABIT.EPx_OUT_INT_SF ==
		    1) {
			_e967_proc_epx_h2d(dev, USB_EP_DIR_OUT | 4);
			return;
		}
	}
}

static int udc_e967_ep_enable(const struct device *dev, struct udc_ep_config *const cfg)
{
	struct udc_e967_data *priv = udc_get_private(dev);
	struct e967_usbd_ep *ep_ctrl;
	uint32_t lock_key;
	uint8_t ep_dir;
	uint8_t ep_idx;

#if (__EPX_OUT_LOG__ > 1)
	printk("[INFO] udc ep enabled %p\n", dev);
#endif


	ep_ctrl = _e967_get_ep(priv, cfg->addr);
	ep_dir = USB_EP_GET_DIR(cfg->addr);
	ep_idx = USB_EP_GET_IDX(cfg->addr);

	if (ep_idx == 0) {
		return 0;
	}

	if (ep_idx > 4) {
		return -1;
	}

	if (ep_dir == USB_EP_DIR_IN) {
		lock_key = irq_lock();
		ep_ctrl->is_in_pkg = 0;
		irq_unlock(lock_key);
		ep_ctrl->reg_ep_int_sta->UDC_EPx_INTSTA.UDC_EPx_INT_STABIT.EPx_IN_INT_SF_CLR = 1;
		ep_ctrl->reg_ep_int_en->UDCEPx_INT_EN.UDC_EPx_INT_ENBIT.EPxININTEN = 1;
	} else {
		lock_key = irq_lock();
		ep_ctrl->is_out_pkg = 0;
		irq_unlock(lock_key);
		ep_ctrl->reg_ep_int_sta->UDC_EPx_INTSTA.UDC_EPx_INT_STABIT.EPx_OUT_INT_SF_CLR = 1;
		ep_ctrl->reg_ep_int_en->UDCEPx_INT_EN.UDC_EPx_INT_ENBIT.EPxOUTINTEN = 1;
	}

	if (ep_idx == 1) {
		priv->reg_udc_ctrl1->UDC_CTRL1_.UDCCTRL1BIT.EP1STALL = 0;
		priv->reg_udc_ctrl->UDC_CTRL.UDC_CTRLBIT.EP1EN = 1;
	} else if (ep_idx == 2) {
		priv->reg_udc_ctrl1->UDC_CTRL1_.UDCCTRL1BIT.EP2STALL = 0;
		priv->reg_udc_ctrl->UDC_CTRL.UDC_CTRLBIT.EP2EN = 1;
	} else if (ep_idx == 3) {
		priv->reg_udc_ctrl1->UDC_CTRL1_.UDCCTRL1BIT.EP3STALL = 0;
		priv->reg_udc_ctrl->UDC_CTRL.UDC_CTRLBIT.EP3EN = 1;
	} else if (ep_idx == 4) {
		priv->reg_udc_ctrl1->UDC_CTRL1_.UDCCTRL1BIT.EP4STALL = 0;
		priv->reg_udc_ctrl->UDC_CTRL.UDC_CTRLBIT.EP4EN = 1;
	}

	return 0;
}

static int udc_e967_ep_disable(const struct device *dev, struct udc_ep_config *const cfg)
{
	struct udc_e967_data *priv = udc_get_private(dev);
	struct e967_usbd_ep *ep_ctrl;
	uint32_t lock_key;
	uint8_t ep_dir;
	uint8_t ep_idx;

#if (__EPX_OUT_LOG__ > 1)
	printk("[INFO] udc ep disabled %p\n", dev);
#endif

	ep_ctrl = _e967_get_ep(priv, cfg->addr);
	ep_dir = USB_EP_GET_DIR(cfg->addr);
	ep_idx = USB_EP_GET_IDX(cfg->addr);

	if (ep_idx == 0) {
		return 0;
	}

	if (ep_idx > 4) {
		return -1;
	}

	if (ep_idx == 1) {
		priv->reg_udc_ctrl->UDC_CTRL.UDC_CTRLBIT.EP1EN = 0;
	} else if (ep_idx == 2) {
		priv->reg_udc_ctrl->UDC_CTRL.UDC_CTRLBIT.EP2EN = 0;
	} else if (ep_idx == 3) {
		priv->reg_udc_ctrl->UDC_CTRL.UDC_CTRLBIT.EP3EN = 0;
	} else if (ep_idx == 4) {
		priv->reg_udc_ctrl->UDC_CTRL.UDC_CTRLBIT.EP4EN = 0;
	}

	if (ep_dir == USB_EP_DIR_IN) {
		lock_key = irq_lock();
		ep_ctrl->is_in_pkg = 0;
		irq_unlock(lock_key);
		ep_ctrl->reg_ep_int_en->UDCEPx_INT_EN.UDC_EPx_INT_ENBIT.EPxININTEN = 0;
		ep_ctrl->reg_ep_int_sta->UDC_EPx_INTSTA.UDC_EPx_INT_STABIT.EPx_IN_INT_SF_CLR = 1;
	} else {
		lock_key = irq_lock();
		ep_ctrl->is_out_pkg = 0;
		irq_unlock(lock_key);
		ep_ctrl->reg_ep_int_en->UDCEPx_INT_EN.UDC_EPx_INT_ENBIT.EPxOUTINTEN = 0;
		ep_ctrl->reg_ep_int_sta->UDC_EPx_INTSTA.UDC_EPx_INT_STABIT.EPx_OUT_INT_SF_CLR = 1;
	}

	return 0;
}

static int udc_e967_set_address(const struct device *dev, const uint8_t addr)
{
	struct udc_e967_data *priv = udc_get_private(dev);

	priv->addr = addr;

	return 0;
}

static int udc_e967_enable(const struct device *dev)
{
	/* S/W connect */
#if (__EPX_OUT_LOG__ > 1)
	printk("[INFO] Enable device %p\n", dev);
#endif

	_e967_usbd_sw_connect(dev);

	return 0;
}

static int udc_e967_disable(const struct device *dev)
{
	/* S/W disconnect */

#if (__EPX_OUT_LOG__ > 1)
	printk("[INFO] Disable device %p\n", dev);
#endif

	_e967_usbd_sw_disconnect(dev);

	return 0;
}

void _enable_all_ep(const struct device *dev)
{
	struct udc_ep_config *cfg;
	cfg = udc_get_ep_cfg(dev, USB_EP_DIR_IN | 1);
	udc_e967_ep_enable(dev, cfg);

	cfg = udc_get_ep_cfg(dev, USB_EP_DIR_IN | 2);
	udc_e967_ep_enable(dev, cfg);

	cfg = udc_get_ep_cfg(dev, USB_EP_DIR_IN | 3);
	udc_e967_ep_enable(dev, cfg);

	cfg = udc_get_ep_cfg(dev, USB_EP_DIR_IN | 4);
	udc_e967_ep_enable(dev, cfg);

	cfg = udc_get_ep_cfg(dev, USB_EP_DIR_OUT | 1);
	udc_e967_ep_enable(dev, cfg);

	cfg = udc_get_ep_cfg(dev, USB_EP_DIR_OUT | 2);
	udc_e967_ep_enable(dev, cfg);

	cfg = udc_get_ep_cfg(dev, USB_EP_DIR_OUT | 3);
	udc_e967_ep_enable(dev, cfg);

	cfg = udc_get_ep_cfg(dev, USB_EP_DIR_OUT | 4);
	udc_e967_ep_enable(dev, cfg);
}

static int udc_e967_init(const struct device *dev)
{
	const struct udc_e967_config *config = dev->config;
	struct udc_e967_data *priv = udc_get_private(dev);


#if (__EPX_OUT_LOG__ > 1)
	printk("[INFO] udc init for %p\n", dev);
#endif

	/* Initialize USBD H/W */
	_e967_usb_clock_set(priv, USB_IRC);
	_e967_usb_init(priv);

	_e967_usbd_sw_disconnect(dev);

	priv->addr = 0;
	priv->ep_out_num = 0;
	priv->ep_out_num_new = 0;
	
	_e967_epx_init(dev);

	_enable_all_ep(dev);

	config->irq_enable_func(dev);
	if (udc_ep_enable_internal(dev, USB_CONTROL_EP_OUT, USB_EP_TYPE_CONTROL, 8, 0)) {
		return -EIO;
	}

	if (udc_ep_enable_internal(dev, USB_CONTROL_EP_IN, USB_EP_TYPE_CONTROL, 8, 0)) {
		return -EIO;
	}

	return 0;
}

static int udc_e967_shutdown(const struct device *dev)
{
	const struct udc_e967_config *config = dev->config;
	struct udc_e967_data *priv = udc_get_private(dev);

	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_OUT)) {
		return -EIO;
	}

	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_IN)) {
		return -EIO;
	}

	/* Uninitialize IRQ */
	config->irq_disable_func(dev);

	/* Set SE0 for S/W disconnect */
	_e967_usbd_sw_disconnect(dev);

	/* Disable USB PHY */
	priv->reg_usb_phy->USBPHYPDB = 0;

	/* Equivalent to CLK_DisableModuleClock() */
	CLKGatingEnable(PCLKG_UDC);

	/* Purge message queue */
	k_msgq_purge(priv->msgq);

	return 0;
}

static int udc_e967_driver_preinit(const struct device *dev)
{
	const struct udc_e967_config *config = dev->config;
	struct udc_data *data = dev->data;
	int err;
	int i;

	data->caps.hs = false;
	data->caps.rwup = true;
	data->caps.addr_before_status = true;
	data->caps.mps0 = UDC_MPS0_8;
	data->caps.out_ack = true;
	data->caps.can_detect_vbus = false;

	config->ep_cfg_out[0].caps.out = 1;
	config->ep_cfg_out[0].caps.control = 1;
	config->ep_cfg_out[0].caps.mps = 8;
	config->ep_cfg_out[0].addr = USB_EP_DIR_OUT | 0;
	err = udc_register_ep(dev, &config->ep_cfg_out[0]);
	if (err != 0) {
		LOG_ERR("Failed to register endpoint");
		return err;
	}

	config->ep_cfg_in[0].caps.in = 1;
	config->ep_cfg_in[0].caps.control = 1;
	config->ep_cfg_in[0].caps.mps = 8;
	config->ep_cfg_in[0].addr = USB_EP_DIR_IN | 0;
	err = udc_register_ep(dev, &config->ep_cfg_in[0]);
	if (err != 0) {
		LOG_ERR("Failed to register endpoint");
		return err;
	}

	for (i = 1; i <= 4; i++) {
		config->ep_cfg_out[i].caps.out = 1;
		config->ep_cfg_out[i].caps.interrupt = 1;
		config->ep_cfg_out[i].caps.bulk = 1;
		config->ep_cfg_out[i].caps.iso = 1;
		config->ep_cfg_out[i].caps.mps = 1023;
		config->ep_cfg_out[i].addr = USB_EP_DIR_OUT | i;
		err = udc_register_ep(dev, &config->ep_cfg_out[i]);
		if (err != 0) {
			LOG_ERR("Failed to register endpoint");
			return err;
		}
	}

	for (i = 1; i <= 4; i++) {
		config->ep_cfg_in[i].caps.in = 1;
		config->ep_cfg_in[i].caps.interrupt = 1;
		config->ep_cfg_in[i].caps.bulk = 1;
		config->ep_cfg_in[i].caps.iso = 1;
		config->ep_cfg_in[i].caps.mps = 1023;
		config->ep_cfg_in[i].addr = USB_EP_DIR_IN | i;
		err = udc_register_ep(dev, &config->ep_cfg_in[i]);
		if (err != 0) {
			LOG_ERR("Failed to register endpoint");
			return err;
		}
	}

	config->make_thread(dev);
	LOG_INF("Device %p (max. speed %d)", dev, config->speed_idx);

	return 0;
}

static void udc_e967_lock(const struct device *dev)
{
	udc_lock_internal(dev, K_FOREVER);
}

static void udc_e967_unlock(const struct device *dev)
{
	udc_unlock_internal(dev);
}

static enum udc_bus_speed udc_e967_device_speed(const struct device *dev)
{
	struct udc_data *data = dev->data;
	return data->caps.hs ? UDC_BUS_SPEED_HS : UDC_BUS_SPEED_FS;
}

static const struct udc_api udc_e967_api = {
	.device_speed = udc_e967_device_speed,
	.ep_enqueue = udc_e967_ep_enqueue,
	.ep_dequeue = udc_e967_ep_dequeue,
	.ep_set_halt = udc_e967_ep_set_halt,
	.ep_clear_halt = udc_e967_ep_clear_halt,
	.ep_enable = udc_e967_ep_enable,
	.ep_disable = udc_e967_ep_disable,
	.host_wakeup = udc_e967_host_wakeup,
	.set_address = udc_e967_set_address,
	.enable = udc_e967_enable,
	.disable = udc_e967_disable,
	.init = udc_e967_init,
	.shutdown = udc_e967_shutdown,
	.lock = udc_e967_lock,
	.unlock = udc_e967_unlock,
	.test_mode = NULL,
};

#ifdef zephyr_official
#define UDC_E967_DEVICE_DEFINE(inst)                                                               \
	static void udc_e967_irq_enable_func(const struct device *dev)                             \
	{                                                                                          \
		irq_connect_dynamic(E967_USB_Setup_IRQn, 0,                                        \
				    (void (*)(const void *))e967_usb_setup_isr,                    \
				    DEVICE_DT_INST_GET(inst), 0);                                  \
		irq_connect_dynamic(E967_USB_Suspend_IRQn, 0,                                      \
				    (void (*)(const void *))e967_usb_suspend_isr,                  \
				    DEVICE_DT_INST_GET(inst), 0);                                  \
		irq_connect_dynamic(E967_USB_Resume_IRQn, 0,                                       \
				    (void (*)(const void *))e967_usb_resume_isr,                   \
				    DEVICE_DT_INST_GET(inst), 0);                                  \
		irq_connect_dynamic(E967_USB_Reset_IRQn, 0,                                        \
				    (void (*)(const void *))e967_usb_reset_isr,                    \
				    DEVICE_DT_INST_GET(inst), 0);                                  \
		irq_connect_dynamic(E967_USB_EPx_In_EPx_Empty_IRQn, 0,                             \
				    (void (*)(const void *))e967_usb_ep_d2h_isr,                   \
				    DEVICE_DT_INST_GET(inst), 0);                                  \
		irq_connect_dynamic(E967_USB_EPx_Out_IRQn, 0,                                      \
				    (void (*)(const void *))e967_usb_ep_h2d_isr,                   \
				    DEVICE_DT_INST_GET(inst), 0);                                  \
		irq_enable(E967_USB_Setup_IRQn);                                                   \
		irq_enable(E967_USB_Suspend_IRQn);                                                 \
		irq_enable(E967_USB_Resume_IRQn);                                                  \
		irq_enable(E967_USB_Reset_IRQn);                                                   \
		irq_enable(E967_USB_EPx_In_EPx_Empty_IRQn);                                        \
		irq_enable(E967_USB_EPx_Out_IRQn);                                                 \
	}                                                                                          \
	static void udc_e967_irq_disable_func(const struct device *dev)                            \
	{                                                                                          \
		irq_disable(E967_USB_Setup_IRQn);                                                  \
		irq_disable(E967_USB_Suspend_IRQn);                                                \
		irq_disable(E967_USB_Resume_IRQn);                                                 \
		irq_disable(E967_USB_Reset_IRQn);                                                  \
		irq_disable(E967_USB_EPx_In_EPx_Empty_IRQn);                                       \
		irq_disable(E967_USB_EPx_Out_IRQn);                                                \
	}                                                                                          \
                                                                                                   \
	K_THREAD_STACK_DEFINE(udc_e967_stack_##inst, CONFIG_UDC_E967_STACK_SIZE);                  \
                                                                                                   \
	static void udc_e967_thread_##inst(void *dev, void *arg1, void *arg2)                      \
	{                                                                                          \
		ARG_UNUSED(arg1);                                                                  \
		ARG_UNUSED(arg2);                                                                  \
		e967_usbd_msg_handler(dev);                                                        \
	}                                                                                          \
                                                                                                   \
	static void udc_e967_make_thread(const struct device *dev)                                 \
	{                                                                                          \
		struct udc_e967_data *priv = udc_get_private(dev);                                 \
                                                                                                   \
		k_thread_create(&priv->thread_data, udc_e967_stack_##inst,                         \
				K_THREAD_STACK_SIZEOF(udc_e967_stack_##inst),                      \
				udc_e967_thread_##inst, (void *)dev, NULL, NULL,                   \
				K_PRIO_COOP(CONFIG_UDC_E967_THREAD_PRIORITY), K_ESSENTIAL,         \
				K_NO_WAIT);                                                        \
		k_thread_name_set(&priv->thread_data, dev->name);                                  \
	}                                                                                          \
                                                                                                   \
	static struct udc_ep_config ep_cfg_out_##inst[USB_NUM_BIDIR_ENDPOINTS];                    \
	static struct udc_ep_config ep_cfg_in_##inst[USB_NUM_BIDIR_ENDPOINTS];                     \
                                                                                                   \
	static const struct udc_e967_config udc_e967_config_##inst = {                             \
		.num_of_eps = USB_NUM_BIDIR_ENDPOINTS,                                             \
		.ep_cfg_in = ep_cfg_in_##inst,                                                     \
		.ep_cfg_out = ep_cfg_out_##inst,                                                   \
		.ep_cfg_out_size = ARRAY_SIZE(ep_cfg_out_##inst),                                  \
		.ep_cfg_in_size = ARRAY_SIZE(ep_cfg_in_##inst),                                    \
		.make_thread = udc_e967_make_thread,                                               \
		.speed_idx = UDC_BUS_SPEED_FS,                                                     \
		.irq_enable_func = udc_e967_irq_enable_func,                                       \
		.irq_disable_func = udc_e967_irq_disable_func};                                    \
                                                                                                   \
	K_MSGQ_DEFINE(e967_usbd_msgq_##inst, sizeof(struct udc_e967_msg),                          \
		      CONFIG_UDC_E967_MSG_QUEUE_SIZE, 4);                                          \
	static struct udc_e967_data e967_udc_priv_##inst = {                                       \
		.msgq = &e967_usbd_msgq_##inst,                                                    \
		.reg_ep0_data_buf = (volatile uint32_t *)(E967_USB_BASE + 0x38),                   \
		.reg_ep0_int_sts = UDCEP0INTSTA,                                                   \
		.reg_ep0_int_en = UDCEP0INTEN,                                                     \
		.ep0_out_size = 0,                                                                 \
		.is_pending_pkg = 0,                                                               \
		.is_ep0_in_en = 0,                                                                  \
		.ep0_xfer_size = 0,                                                                \
		.is_configured_state = 0,                                                          \
		.is_addressed_state = 0,                                                           \
		.is_proc_remote_wakeup = 0,                                                        \
		.reg_ep_buf_sta = EPBUFSTA,                                                        \
		.reg_udc_ctrl = UDCCTRL,                                                           \
		.reg_udc_ctrl1 = UDCCTRL1,                                                         \
		.reg_udc_int_en = UDCINTEN,                                                        \
		.reg_udc_int_sta = UDCINTSTA,                                                      \
		.reg_udc_cf_data = UDCCFDATA,                                                      \
		.reg_usb_phy = E967_PHYCTRL,                                                       \
		.reg_ljirc_ctrl = E967_LJIRCCTRL,                                                  \
		.reg_usbpll_ctrl = E967_USBPLLCTRL,                                                \
		.reg_xtal_ctrl = E967_XTALCTRL,                                                    \
		.reg_sysreg = E967_SYSREGCTRL};                                                    \
                                                                                                   \
	static struct udc_data _e967_udc_data_##inst = {                                           \
		.mutex = Z_MUTEX_INITIALIZER(_e967_udc_data_##inst.mutex),                         \
		.priv = &e967_udc_priv_##inst,                                                     \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, udc_e967_driver_preinit, NULL, &_e967_udc_data_##inst,         \
			      &udc_e967_config_##inst, POST_KERNEL,                                \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &udc_e967_api);

DT_INST_FOREACH_STATUS_OKAY(UDC_E967_DEVICE_DEFINE)
#else
struct device __device_dts_e967_udc_dev_0;
static void udc_e967_irq_enable_func(const struct device *dev)
{
	irq_connect_dynamic(E967_USB_Setup_IRQn, 0, (void (*)(const void *))e967_usb_setup_isr,
			    &__device_dts_e967_udc_dev_0, 0);
	irq_connect_dynamic(E967_USB_Suspend_IRQn, 0, (void (*)(const void *))e967_usb_suspend_isr,
			    &__device_dts_e967_udc_dev_0, 0);
	irq_connect_dynamic(E967_USB_Resume_IRQn, 0, (void (*)(const void *))e967_usb_resume_isr,
			    &__device_dts_e967_udc_dev_0, 0);
	irq_connect_dynamic(E967_USB_Reset_IRQn, 0, (void (*)(const void *))e967_usb_reset_isr,
			    &__device_dts_e967_udc_dev_0, 0);
	irq_connect_dynamic(E967_USB_EPx_In_EPx_Empty_IRQn, 0,
			    (void (*)(const void *))e967_usb_ep_d2h_isr,
			    &__device_dts_e967_udc_dev_0, 0);
	irq_connect_dynamic(E967_USB_EPx_Out_IRQn, 0, (void (*)(const void *))e967_usb_ep_h2d_isr,
			    &__device_dts_e967_udc_dev_0, 0);
	irq_enable(E967_USB_Setup_IRQn);
	irq_enable(E967_USB_Suspend_IRQn);
	irq_enable(E967_USB_Resume_IRQn);
	irq_enable(E967_USB_Reset_IRQn);
	irq_enable(E967_USB_EPx_In_EPx_Empty_IRQn);
	irq_enable(E967_USB_EPx_Out_IRQn);
}
static void udc_e967_irq_disable_func(const struct device *dev)
{
	irq_disable(E967_USB_Setup_IRQn);
	irq_disable(E967_USB_Suspend_IRQn);
	irq_disable(E967_USB_Resume_IRQn);
	irq_disable(E967_USB_Reset_IRQn);
	irq_disable(E967_USB_EPx_In_EPx_Empty_IRQn);
	irq_disable(E967_USB_EPx_Out_IRQn);
}

K_THREAD_STACK_DEFINE(udc_e967_stack_0, CONFIG_UDC_SKELETON_STACK_SIZE);

static void udc_e967_thread_0(void *dev, void *arg1, void *arg2)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	e967_usbd_msg_handler(dev);
}

static void udc_e967_make_thread(const struct device *dev)
{
	struct udc_e967_data *priv = udc_get_private(dev);

	k_thread_create(&priv->thread_data, udc_e967_stack_0,
			K_THREAD_STACK_SIZEOF(udc_e967_stack_0), udc_e967_thread_0, (void *)dev,
			NULL, NULL, K_PRIO_COOP(CONFIG_UDC_SKELETON_THREAD_PRIORITY), K_ESSENTIAL,
			K_NO_WAIT);
	k_thread_name_set(&priv->thread_data, dev->name);
}

static struct udc_ep_config ep_cfg_out_0[USB_NUM_BIDIR_ENDPOINTS];
static struct udc_ep_config ep_cfg_in_0[USB_NUM_BIDIR_ENDPOINTS];

static const struct udc_e967_config udc_e967_config_0 = {
	.num_of_eps = USB_NUM_BIDIR_ENDPOINTS,
	.ep_cfg_in = ep_cfg_in_0,
	.ep_cfg_out = ep_cfg_out_0,
	.ep_cfg_out_size = ARRAY_SIZE(ep_cfg_out_0),
	.ep_cfg_in_size = ARRAY_SIZE(ep_cfg_in_0),
	.make_thread = udc_e967_make_thread,
	.speed_idx = UDC_BUS_SPEED_FS,
	.irq_enable_func = udc_e967_irq_enable_func,
	.irq_disable_func = udc_e967_irq_disable_func};

K_MSGQ_DEFINE(e967_usbd_msgq_0, sizeof(struct udc_e967_msg), CONFIG_UDC_E967_MSG_QUEUE_SIZE, 4);
static struct udc_e967_data e967_udc_priv_0 = { .msgq = &e967_usbd_msgq_0,
					       .reg_ep0_data_buf =
						       (volatile uint32_t *)(E967_USB_BASE + 0x38),
					       .reg_ep0_int_sts = UDCEP0INTSTA,
					       .reg_ep0_int_en = UDCEP0INTEN,
					       .ep0_out_size = 0,
						   .is_pending_pkg = 0,
					       .is_ep0_in_en = 0,
					       .ep0_xfer_size = 0,
					       .is_configured_state = 0,
					       .is_addressed_state = 0,
					       .is_proc_remote_wakeup = 0,
					       .reg_ep_buf_sta = EPBUFSTA,
					       .reg_udc_ctrl = UDCCTRL,
					       .reg_udc_ctrl1 = UDCCTRL1,
					       .reg_udc_int_en = UDCINTEN,
					       .reg_udc_int_sta = UDCINTSTA,
					       .reg_udc_cf_data = UDCCFDATA,
					       .reg_usb_phy = E967_PHYCTRL,
					       .reg_ljirc_ctrl = E967_LJIRCCTRL,
					       .reg_usbpll_ctrl = E967_USBPLLCTRL,
					       .reg_xtal_ctrl = E967_XTALCTRL,
					       .reg_sysreg = E967_SYSREGCTRL};

static struct udc_data _e967_udc_data_0 = {
	.mutex = Z_MUTEX_INITIALIZER(_e967_udc_data_0.mutex),
	.priv = &e967_udc_priv_0,
};

static struct device_state __devstate_e967_usb_dev = {.init_res = 0, .initialized = true};

struct device __device_dts_e967_udc_dev_0 = {
	/** Name of the device instance */
	// const char *name;
	.name = "Elan 967 udc device",
	/** Address of device instance config information */
	// const void *config;
	.config = &udc_e967_config_0,
	/** Address of the API structure exposed by the device instance */
	// const void *api;
	.api = &udc_e967_api,
	/** Address of the common device state */
	// struct device_state *state;
	.state = &__devstate_e967_usb_dev,
	/** Address of the device instance private data */
	// void *data;
	.data = &_e967_udc_data_0,
	/** Device operations */
	// struct device_ops ops;
	/** Initialization function */
	// int (*init)(const struct device *dev);
	/** De-initialization function */
	// int (*deinit)(const struct device *dev);
	.ops = {.init = udc_e967_driver_preinit, .deinit = NULL},
	/** Device flags */
	// device_flags_t flags;
	.flags = 0};

// PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY
static const struct init_entry __init_e967_usb_dev = {.init_fn = NULL,
						      .dev = &__device_dts_e967_udc_dev_0};
const struct init_entry *__get_dev_entry_usb_e967()
{
	return &__init_e967_usb_dev;
}

struct device *__get_dev_inst_usb_e967()
{
	return &__device_dts_e967_udc_dev_0;
}
#endif
