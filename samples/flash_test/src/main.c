/*
 * Copyright (c) 2025 Elan Microelectronics Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>

//#include <flash_em32.h>
#include "../../../../include/zephyr/drivers/flash/flash_em32.h"

/* Log configuration */
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(flash_test, LOG_LEVEL_DBG);
//LOG_MODULE_REGISTER(flash_test, CONFIG_LOG_DEFAULT_LEVEL);

int main(void)
{
    int ret = 0;
    const struct device *flash_device;
    off_t address_base = EM32_NV_FLASH_ADDR, \
          address = 0, \
          offset = 0;
    size_t len = 0;
    //size_t index = 0;
    //uint32_t device_id = 0;
    //uint8_t buffer[32] = {0};
    //uint8_t buffer8192[8192] = {0};
    //static uint8_t buffer8192[8192] = {0};
    //uint8_t *p_buffer8192 = NULL;
    //uint8_t *p_page_buf = NULL;
    uint8_t buffer[256] = {0};
    //uint8_t input_buf[256] = {0};
    //uint8_t output_buf[256] = {0};
	
    flash_device = DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller));
    if (!device_is_ready(flash_device)) {
        printk("%s: device not ready.\n", flash_device->name);
        return 0;
    }
    LOG_DBG("Flash controller (%s) ready.", flash_device->name);

#if 0
    /* Read Device ID (Need to disable check function of em32-flash driver read()) */

    // Read using flash API
    address = 0x100A6020; // Device ID
    offset = address - address_base;
    len = 4;
    memset(buffer, 0, sizeof(buffer));
    LOG_DBG("Read flash from address 0x%lx (offset 0x%lx), len %d.", address, offset, len);
    ret = flash_read(flash_device, offset, buffer, len);
    if(ret < 0)
    {
        LOG_ERR("Read flash from address 0x%lx (len %d) fail, ret = %d.", address, len, ret);
        goto EXIT;
    }
    LOG_HEXDUMP_DBG(buffer, len, "Read flash using flash API");
    memcpy(&device_id, buffer, len);
    LOG_DBG("device_id=0x%04x.", device_id);
#endif //0

#if 0
    /* Read a small amount of data from EC-RW */

    // Read using flash API
    address = 0x10024000; // EC-RW
    offset = address - address_base;
    len = sizeof(buffer);
    memset(buffer, 0, sizeof(buffer));
    LOG_DBG("Read flash from address 0x%lx (offset 0x%lx), len %d.", address, offset, len);
    ret = flash_read(flash_device, offset, buffer, len);
    if(ret < 0)
    {
        LOG_ERR("Read flash from address 0x%lx (len %d) fail, ret = %d.", address, len, ret);
        goto EXIT;
    }
    LOG_HEXDUMP_DBG(buffer, sizeof(buffer), "Read flash using flash API");
#endif //0

#if 0
    /* Erase flash content */
	
    // Erase using flash API
    address = 0x10024000; // EC-RW
    offset = address - address_base;
    len = EM32_NV_FLASH_PAGE_SIZE;
    LOG_DBG("Erase flash from address 0x%lx (offset 0x%lx), len %d.", address, offset, len);
    ret = flash_erase(flash_device, offset, len);
    if(ret < 0)
    {
        LOG_ERR("Erase flash fail, ret = %d.", ret);
        goto EXIT;
    }
    LOG_DBG("Erase flash ok.");

    /* Read a small amount of data from EC-RW */

    // Read using flash API
    address = 0x10024000; // EC-RW
    offset = address - address_base;
    len = sizeof(buffer);
    memset(buffer, 0, sizeof(buffer));
    LOG_DBG("Read flash from address 0x%lx (offset 0x%lx), len %d.", address, offset, len);
    ret = flash_read(flash_device, offset, buffer, len);
    if(ret < 0)
    {
        LOG_ERR("Read flash from address 0x%lx (len %d) fail, ret = %d.", address, len, ret);
        goto EXIT;
    }
	LOG_HEXDUMP_DBG(buffer, sizeof(buffer), "Read flash using flash API");
#endif //0

#if 0
    /* Write a small amount of data to EC-RW */

    // Set Write Buffer Content
    for (size_t k = 0; k < len; k++) {
        buffer[k] = k;
    }
    LOG_HEXDUMP_DBG(buffer, sizeof(buffer), "Write flash using flash API");

    // Write using flash API
    address = 0x10024000; // EC-RW
    offset = address - address_base;
    len = sizeof(buffer);
    ret = flash_write(flash_device, offset, buffer, len);
    if(ret < 0)
    {
        LOG_ERR("Write flash from address 0x%lx (len %d) fail, ret = %d.", address, len, ret);
        goto EXIT;
    }
    LOG_DBG("Write flash ok.");

    /* Read a small amount of data from EC-RW */

    // Read using flash API
    address = 0x10024000; // EC-RW
    offset = address - address_base;
    len = sizeof(buffer);
    memset(buffer, 0, sizeof(buffer));
    LOG_DBG("Read flash from address 0x%lx (offset 0x%lx), len %d.", address, offset, len);
    ret = flash_read(flash_device, offset, buffer, len);
    if(ret < 0)
    {
        LOG_ERR("Read flash from address 0x%lx (len %d) fail, ret = %d.", address, len, ret);
        goto EXIT;
    }
    LOG_HEXDUMP_DBG(buffer, sizeof(buffer), "Read flash using flash API");
#endif //0

#if 0
    /* Write 8192 bytes of data to EC-RW with static buffer */
    address = 0x10024000 + sizeof(buffer8192); // EC-RW[1024 * index]
    offset = address - address_base;
    len = sizeof(buffer8192);
    LOG_DBG("Write flash from address 0x%lx, len %d.", address, len);
    ret = flash_write(flash_device, offset, buffer8192, len);
    if(ret < 0)
    {
        LOG_ERR("Write flash from address 0x%lx (len %d) fail, ret = %d.", address, len, ret);
        goto EXIT;
    }
    LOG_DBG("Write flash address 0x%lx (len %d) ok.", address, len);
#endif //0

#if 0
    /* Write 8192 bytes of data to EC-RW with dynamic memory allocation */
    p_buffer8192 = k_malloc(8192);
    if (p_buffer8192 == NULL) {
        LOG_ERR("Fail to allocate memory for 8K memory buffer!");
        goto EXIT;
    }
    memset(p_buffer8192, 0, 8192);
	
    /* Write a page of data to EC-RW with static buffer */
    address = 0x10024000; // EC-RW[1024]
    offset = address - address_base;
    len = 8192;
    LOG_DBG("Write flash from address 0x%lx, len %d.", address, len);
    ret = flash_write(flash_device, offset, p_buffer8192, len);
    if(ret < 0)
    {
        LOG_ERR("Write flash from address 0x%lx (len %d) fail, ret = %d.", address, len, ret);
		
        // Release allocated buffer
        k_free(p_buffer8192);
		
        goto EXIT;
    }
    LOG_DBG("Write flash address 0x%lx (len %d) ok.", address, len);

    // Release allocated buffer
    k_free(p_buffer8192);
#endif //0

#if 0
    /* Write a page of data to EC-RW with dynamic memory allocation */
    p_page_buf = k_malloc(EM32_NV_FLASH_PAGE_SIZE);
    if (p_page_buf == NULL) {
        LOG_ERR("Fail to allocate memory for 8K memory buffer!");
        goto EXIT;
    }
    memset(p_page_buf, 0, EM32_NV_FLASH_PAGE_SIZE);
	
    /* Write a page of data to EC-RW with static buffer */
    address = 0x10024000; // EC-RW[1024]
    offset = address - address_base;
    len = EM32_NV_FLASH_PAGE_SIZE;
    LOG_DBG("Write flash from address 0x%lx, len %d.", address, len);
    ret = flash_write(flash_device, offset, p_page_buf, len);
    if(ret < 0)
    {
        LOG_ERR("Write flash from address 0x%lx (len %d) fail, ret = %d.", address, len, ret);
		
        // Release allocated buffer
        k_free(p_page_buf);
		
        goto EXIT;
    }
    LOG_DBG("Write flash address 0x%lx (len %d) ok.", address, len);

    // Release allocated buffer
    k_free(p_page_buf);
#endif //0

#if 0
    /* Write a small amount of data to EC-RW */

    // Set Write Buffer Content
    for (size_t k = 0; k < sizeof(buffer); k++) {
        buffer[k] = k;
    }
    LOG_HEXDUMP_DBG(buffer, sizeof(buffer), "Write flash using flash API");

    // Write using flash API
    address = 0x10024000; // EC-RW
    offset = address - address_base;
    len = 240;
    ret = flash_write(flash_device, offset, buffer, len);
    if(ret < 0)
    {
        LOG_ERR("Write flash from address 0x%lx (len %d) fail, ret = %d.", address, len, ret);
        goto EXIT;
    }
    LOG_DBG("Write flash address 0x%lx (offset 0x%lx) len 0x%x ok.", address, offset, len);
	
    address = 0x10024000 + 240; // EC-RW
    offset = address - address_base;
    len = sizeof(buffer) - 240;
    ret = flash_write(flash_device, offset, &buffer[240], len);
    if(ret < 0)
    {
        LOG_ERR("Write flash from address 0x%lx (len %d) fail, ret = %d.", address, len, ret);
        goto EXIT;
    }
    LOG_DBG("Write flash address 0x%lx (offset 0x%lx) len 0x%x ok.", address, offset, len);

    /* Read a small amount of data from EC-RW */

    // Read using flash API
    address = 0x10024000; // EC-RW
    offset = address - address_base;
    len = sizeof(buffer);
    memset(buffer, 0, sizeof(buffer));
    LOG_DBG("Read flash from address 0x%lx (offset 0x%lx), len %d.", address, offset, len);
    ret = flash_read(flash_device, offset, buffer, len);
    if(ret < 0)
    {
        LOG_ERR("Read flash from address 0x%lx (len %d) fail, ret = %d.", address, len, ret);
        goto EXIT;
    }
    LOG_HEXDUMP_DBG(buffer, sizeof(buffer), "Read flash using flash API");
#endif //0

#if 0
    // Set Write Buffer Content
    for (size_t k = 0; k < sizeof(output_buf); k++) {
        output_buf[k] = k;
    }
    LOG_HEXDUMP_DBG(output_buf, sizeof(output_buf), "Write flash using flash API");

    /* Write 256-byte of data (from 0-th to 255-th) to EC-RW */

    // Write using flash API
    address = 0x10024000; // EC-RW
    offset = address - address_base;
    len = sizeof(output_buf);
    ret = flash_write(flash_device, offset, output_buf, len);
    if(ret < 0)
    {
        LOG_ERR("Write flash from address 0x%lx (len %d) fail, ret = %d.", address, len, ret);
        goto EXIT;
    }
    LOG_DBG("Write flash address 0x%lx (offset 0x%lx) len 0x%x ok.", address, offset, len);

    /* Read a small amount of data from EC-RW */

    // Read using flash API
    address = 0x10024000; // EC-RW
    offset = address - address_base;
    len = sizeof(input_buf);
    memset(input_buf, 0, sizeof(input_buf));
    LOG_DBG("Read flash from address 0x%lx (offset 0x%lx), len %d.", address, offset, len);
    ret = flash_read(flash_device, offset, input_buf, len);
    if(ret < 0)
    {
        LOG_ERR("Read flash from address 0x%lx (len %d) fail, ret = %d.", address, len, ret);
        goto EXIT;
    }
    LOG_HEXDUMP_DBG(input_buf, sizeof(input_buf), "Read flash using flash API");
#endif //0

#if 0
    /* Write 112-byte of data (from 112-th to 223-th) to EC-RW */
	
    address = 0x10024000 + 112; // EC-RW
    offset = address - address_base;
    len = 112;
    ret = flash_write(flash_device, offset, &output_buf[112], len);
    if(ret < 0)
    {
        LOG_ERR("Write flash from address 0x%lx (len %d) fail, ret = %d.", address, len, ret);
        goto EXIT;
    }
    LOG_DBG("Write flash address 0x%lx (offset 0x%lx) len 0x%x ok.", address, offset, len);

    /* Read a small amount of data from EC-RW */

    // Read using flash API
    address = 0x10024000; // EC-RW
    offset = address - address_base;
    len = sizeof(input_buf);
    memset(input_buf, 0, sizeof(input_buf));
    LOG_DBG("Read flash from address 0x%lx (offset 0x%lx), len %d.", address, offset, len);
    ret = flash_read(flash_device, offset, input_buf, len);
    if(ret < 0)
    {
        LOG_ERR("Read flash from address 0x%lx (len %d) fail, ret = %d.", address, len, ret);
        goto EXIT;
    }
    LOG_HEXDUMP_DBG(input_buf, sizeof(input_buf), "Read flash using flash API");

    /* Write 112-byte of data (from 0-th to 111-th) to EC-RW */

    // Write using flash API
    address = 0x10024000; // EC-RW
    offset = address - address_base;
    len = 112;
    ret = flash_write(flash_device, offset, output_buf, len);
    if(ret < 0)
    {
        LOG_ERR("Write flash from address 0x%lx (len %d) fail, ret = %d.", address, len, ret);
        goto EXIT;
    }
    LOG_DBG("Write flash address 0x%lx (offset 0x%lx) len 0x%x ok.", address, offset, len);

    /* Read a small amount of data from EC-RW */

    // Read using flash API
    address = 0x10024000; // EC-RW
    offset = address - address_base;
    len = sizeof(input_buf);
    memset(input_buf, 0, sizeof(input_buf));
    LOG_DBG("Read flash from address 0x%lx (offset 0x%lx), len %d.", address, offset, len);
    ret = flash_read(flash_device, offset, input_buf, len);
    if(ret < 0)
    {
        LOG_ERR("Read flash from address 0x%lx (len %d) fail, ret = %d.", address, len, ret);
        goto EXIT;
    }
    LOG_HEXDUMP_DBG(input_buf, sizeof(input_buf), "Read flash using flash API");

    /* Write 16-byte of data (from 224-th to 255-th) to EC-RW */

    address = 0x10024000 + 224; // EC-RW
    offset = address - address_base;
    len = sizeof(output_buf) - 224;
    ret = flash_write(flash_device, offset, &output_buf[224], len);
    if(ret < 0)
    {
        LOG_ERR("Write flash from address 0x%lx (len %d) fail, ret = %d.", address, len, ret);
        goto EXIT;
    }
    LOG_DBG("Write flash address 0x%lx (offset 0x%lx) len 0x%x ok.", address, offset, len);

    /* Read a small amount of data from EC-RW */

    // Read using flash API
    address = 0x10024000; // EC-RW
    offset = address - address_base;
    len = sizeof(input_buf);
    memset(input_buf, 0, sizeof(input_buf));
    LOG_DBG("Read flash from address 0x%lx (offset 0x%lx), len %d.", address, offset, len);
    ret = flash_read(flash_device, offset, input_buf, len);
    if(ret < 0)
    {
        LOG_ERR("Read flash from address 0x%lx (len %d) fail, ret = %d.", address, len, ret);
        goto EXIT;
    }
    LOG_HEXDUMP_DBG(input_buf, sizeof(input_buf), "Read flash using flash API");
#endif //0

    // Write using flash API
    address = 0x10085E00;
    offset = address - address_base;
    len = sizeof(buffer);
    ret = flash_write(flash_device, offset, buffer, len);
    if(ret < 0)
    {
        LOG_ERR("Write flash from address 0x%lx (len %d) fail, ret = %d.", address, len, ret);
        goto EXIT;
    }

    // Write using flash API
    address = 0x10085E00 + sizeof(buffer);
    offset = address - address_base;
    len = sizeof(buffer);
    ret = flash_write(flash_device, offset, buffer, len);
    if(ret < 0)
    {
        LOG_ERR("Write flash from address 0x%lx (len %d) fail, ret = %d.", address, len, ret);
        goto EXIT;
    }

EXIT:
    return ret;
}
