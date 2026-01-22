/*
 * Copyright (c) 2024 Elan Microelectronics Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT elan_em32_crypto

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/clock_control.h>
#include <errno.h>
#include <string.h>
#include <soc.h>
#include "../../include/zephyr/drivers/clock_control/clock_control_em32_ahb.h"

LOG_MODULE_REGISTER(crypto_em32_sha, CONFIG_CRYPTO_LOG_LEVEL);

/* EM32F967 SHA256 Hardware Registers - Based on Hardware Specification */
#define SHA_CTR_OFFSET      0x00
#define SHA_IN_OFFSET       0x04
#define SHA_OUT_OFFSET      0x08
#define SHA_DATALEN_5832_OFFSET 0x28  /* Data Length Upper [58:32] */
#define SHA_DATALEN_OFFSET  0x2C      /* Data Length Lower [31:0] */
#define SHA_PAD_CTR_OFFSET  0x30      /* Padding Control */

/* SHA Control Register Bits */
#define SHA_STR_BIT         BIT(0)  /* Start */
#define SHA_INT_CLR_BIT     BIT(1)  /* Interrupt Clear */
#define SHA_RST_BIT         BIT(2)  /* Reset */
#define SHA_READY_BIT       BIT(3)  /* Ready */
#define SHA_STA_BIT         BIT(4)  /* Status */
#define SHA_INT_MASK_BIT    BIT(5)  /* Interrupt Mask */
#define SHA_WR_REV_BIT      BIT(8)  /* Write Reverse */
#define SHA_RD_REV_BIT      BIT(9)  /* Read Reverse */

/* SHA Padding Control Register Bits */
#define SHA_PAD_PACKET_MASK 0x1F    /* Padding packet count (bits 4:0) */
#define SHA_VALID_BYTE_SHIFT 8      /* Valid byte count (bits 9:8) */
#define SHA_VALID_BYTE_MASK 0x3

/* SHA256 constants */
#define SHA256_DIGEST_SIZE  32
#define SHA256_BLOCK_SIZE   64

/* Large data processing constants */
#define SHA256_CHUNK_SIZE   (64 * 1024)   /* 64KB chunks for large data (fits in 112KB RAM) */
#define SHA256_MAX_DATA_LEN (2ULL << 59)  /* 2^59 bits max per hardware spec */

/* SHA256 state continuation support */
#define SHA256_STATE_WORDS  8             /* 256 bits / 32 bits per word */
#define SHA256_INITIAL_H0   0x6a09e667UL
#define SHA256_INITIAL_H1   0xbb67ae85UL
#define SHA256_INITIAL_H2   0x3c6ef372UL
#define SHA256_INITIAL_H3   0xa54ff53aUL
#define SHA256_INITIAL_H4   0x510e527fUL
#define SHA256_INITIAL_H5   0x9b05688cUL
#define SHA256_INITIAL_H6   0x1f83d9abUL
#define SHA256_INITIAL_H7   0x5be0cd19UL



/* Clock gating constants */
#define EM32_CLKGATEREG     0x40030100  /* Clock Gating Control Register */

enum sha_operation_state {
    SHA_STATE_IDLE,
    SHA_STATE_BUSY,
    SHA_STATE_ERROR
};

struct crypto_em32_config {
    uint32_t base;
    const struct device *clock_dev;
    uint32_t clock_group_id;
#ifdef CONFIG_CRYPTO_EM32_SHA_INTERRUPT
    void (*irq_config_func)(const struct device *dev);
#endif
};

struct crypto_em32_data {
    struct hash_ctx *ctx;
    enum sha_operation_state state;
    hash_completion_cb callback;

    /* Legacy small-data buffer (<256B; larger inputs use chunked path) */
    uint8_t buffer[256];              /* Backwards compatibility */
    uint32_t total_len;               /* Legacy total length */
    uint16_t buffer_len;              /* Legacy buffer length (avoid overflow) */



    /* Chunked processing for >64KB data with state continuation */
    bool use_chunked;                 /* True if processing in chunks */
    uint64_t total_bytes_processed;   /* Total bytes processed across chunks */
    uint32_t chunk_state[8];          /* SHA256 state between chunks (H0-H7) */
    bool chunk_state_valid;           /* Whether chunk_state is valid */
    uint64_t chunk_message_bits;      /* Total message bits processed so far */
    size_t expected_total_bytes;        /* Total message bytes if known */
    bool have_expected_total;           /* True if expected_total_bytes is valid */


    /* Chunk buffer for intermediate processing */
    uint8_t *chunk_buf;               /* Buffer for current chunk */
    size_t chunk_buf_len;             /* Current chunk data length */
    size_t chunk_buf_cap;             /* Chunk buffer capacity (64KB) */

    /* Input buffer reference for chunked mode (no copy) */
    const uint8_t *last_input_buf;    /* Reference to last input buffer */
    size_t last_input_len;            /* Length of last input */

    /* Final partial-word handling for chunked mode */
    uint8_t final_rem_buf[4];
    uint8_t final_rem_len;

    bool session_active;
#ifdef CONFIG_CRYPTO_EM32_SHA_INTERRUPT
    struct k_sem op_complete;
#endif
};

/* Register access macros */
#define SHA_REG(dev, offset) \
    ((volatile uint32_t *)(((const struct crypto_em32_config *)dev->config)->base + offset))

#define SHA_CTR(dev)        SHA_REG(dev, SHA_CTR_OFFSET)
#define SHA_IN(dev)         SHA_REG(dev, SHA_IN_OFFSET)
#define SHA_OUT(dev)        SHA_REG(dev, SHA_OUT_OFFSET)
#define SHA_DATALEN_5832(dev) SHA_REG(dev, SHA_DATALEN_5832_OFFSET)
#define SHA_DATALEN(dev)    SHA_REG(dev, SHA_DATALEN_OFFSET)
#define SHA_PAD_CTR(dev)    SHA_REG(dev, SHA_PAD_CTR_OFFSET)

static inline void sha_write_reg(const struct device *dev, uint32_t offset, uint32_t value)
{
    *SHA_REG(dev, offset) = value;
}

static inline uint32_t sha_read_reg(const struct device *dev, uint32_t offset)
{
    return *SHA_REG(dev, offset);
}

/* Forward declarations */
static void sha_reset(const struct device *dev);
static void sha_configure(const struct device *dev);

/* Initialize hash state with SHA256 initial values */
/* Helpers for dynamic accumulation buffer */




/* Initialize SHA256 state to standard initial values */
static void sha_init_state(uint32_t *state)
{
    state[0] = SHA256_INITIAL_H0;
    state[1] = SHA256_INITIAL_H1;
    state[2] = SHA256_INITIAL_H2;
    state[3] = SHA256_INITIAL_H3;
    state[4] = SHA256_INITIAL_H4;
    state[5] = SHA256_INITIAL_H5;
    state[6] = SHA256_INITIAL_H6;
    state[7] = SHA256_INITIAL_H7;
}

/* Process a single chunk through hardware with state save/restore */
static int process_sha256_hardware(const struct device *dev,
                                   const uint8_t *data_buf,
                                   size_t data_len,
                                   uint32_t *state,
                                   uint64_t total_message_bits,
                                   bool is_first_chunk)
{
    struct crypto_em32_data *data = dev->data;
    const struct crypto_em32_config *config = dev->config;

    if (!data_buf || data_len == 0) {
        return 0;
    }

    /* Step 1: Reset and configure hardware (only for first chunk) */
    uint32_t ctrl_reg = SHA_WR_REV_BIT | SHA_RD_REV_BIT;

    if (is_first_chunk) {
        sha_reset(dev);
        sha_configure(dev);

        /* Step 2: Configure byte order and set data length (only for first chunk) */
        sys_write32(ctrl_reg, config->base + SHA_CTR_OFFSET);

        /* Set data length to TOTAL message size (all chunks combined) */
        uint32_t words_lo = (uint32_t)((total_message_bits / 8 + 3U) / 4U);
        sys_write32(words_lo, config->base + SHA_DATALEN_OFFSET);
        sys_write32(0, config->base + SHA_DATALEN_5832_OFFSET);

        /* Readback DATALEN registers for debug */
        {
            uint32_t dlo = sys_read32(config->base + SHA_DATALEN_OFFSET);
            uint32_t dhi = sys_read32(config->base + SHA_DATALEN_5832_OFFSET);
            LOG_DBG("First chunk: DATALEN registers low=0x%08x high=0x%08x", dlo, dhi);
        }
        LOG_DBG("First chunk: DATALEN set to %u words (%llu bits)", words_lo, total_message_bits);

        /* Step 3: If total length is known (common case), program PAD_CTR now. */
        if (data->have_expected_total) {
            LOG_DBG("First chunk: have expected total length %zu bytes, setting PAD_CTR early",
                    data->expected_total_bytes);
            uint64_t total_bytes_first = total_message_bits / 8ULL;
            uint32_t rem_bytes_first = (uint32_t)(total_bytes_first % 4ULL);
            uint32_t valid_enc_first = rem_bytes_first & 0x3U; /* 0..3 */
            uint32_t bmod_first = (uint32_t)(total_message_bits % 512ULL);
            uint32_t pad_packet_first = (bmod_first < 448U) ? ((512U - bmod_first - 64U) / 32U)
                                                            : ((512U - bmod_first + 448U) / 32U);
            uint32_t pad_ctrl_first = (valid_enc_first << 8) | (pad_packet_first & 0x1F);
        sys_write32(pad_ctrl_first, config->base + SHA_PAD_CTR_OFFSET);
        /* Read back PAD_CTR for verification */
        {
        uint32_t pad_rb = sys_read32(config->base + SHA_PAD_CTR_OFFSET);
        LOG_DBG("First chunk: PAD_CTR set early (bmod=%u, pad_packet=%u, pad_ctrl=0x%08x) PAD_CTR_REG=0x%08x",
            bmod_first, pad_packet_first, pad_ctrl_first, pad_rb);
        }
        } else {
            LOG_DBG("First chunk: expected total length unknown, PAD_CTR will be set later");
        }

        /* Step 4: Start operation BEFORE writing data */
        ctrl_reg |= SHA_STR_BIT;
        sys_write32(ctrl_reg, config->base + SHA_CTR_OFFSET);

        LOG_DBG("First chunk: SHA_STR set, starting operation");
    } else {
        /* For subsequent chunks: do NOT write CTR; only stream data */
        LOG_DBG("Writing data for subsequent chunk (no CTR write)");
    }

    /* Step 5: Write input data to hardware with READY checks */
    uint32_t words_to_write = (uint32_t)((data_len + 3U) / 4U);
    uint32_t words_written = 0;
    size_t bytes_written = 0;

    while (words_written < words_to_write) {
        uint32_t w = 0;
        for (int j = 0; j < 4; j++) {
            if (bytes_written < data_len) {
                w |= ((uint32_t)data_buf[bytes_written]) << (j * 8);
                bytes_written++;
            }
        }
        sys_write32(w, config->base + SHA_IN_OFFSET);

        words_written++;

        /* Check READY bit every 16 words */
        if ((words_written % 16U) == 0U) {
            for (int j = 0; j < 6; j++) {
                __asm__ volatile ("nop");
            }
            while (!(sys_read32(config->base + SHA_CTR_OFFSET) & SHA_READY_BIT)) {
                k_busy_wait(1);
            }
        }
    }

    /* After writing all data for this chunk, wait for READY bit to indicate hardware is ready */
    uint32_t timeout = 0;
    uint32_t ctrl_before = sys_read32(config->base + SHA_CTR_OFFSET);
    LOG_DBG("Before READY wait: CTR=0x%08x (READY=%d, STA=%d)",
            ctrl_before,
            (ctrl_before & SHA_READY_BIT) ? 1 : 0,
            (ctrl_before & SHA_STA_BIT) ? 1 : 0);

    while (!(sys_read32(config->base + SHA_CTR_OFFSET) & SHA_READY_BIT)) {
        if (timeout++ > CONFIG_CRYPTO_EM32_SHA_TIMEOUT_USEC) {
            LOG_ERR("Timeout waiting for READY bit after chunk data");
            return -ETIMEDOUT;
        }
        k_busy_wait(1);
    }

    uint32_t ctrl_after = sys_read32(config->base + SHA_CTR_OFFSET);
    LOG_DBG("After READY wait: CTR=0x%08x (READY=%d, STA=%d)",
            ctrl_after,
            (ctrl_after & SHA_READY_BIT) ? 1 : 0,
            (ctrl_after & SHA_STA_BIT) ? 1 : 0);

    /* For subsequent chunks: just return after writing data */
    if (!is_first_chunk) {
        LOG_DBG("Chunk processed, returning");
        return 0;
    }

    /* For first chunk: don't wait for STA bit here, it will be set after all chunks */
    LOG_DBG("First chunk processed, waiting for more chunks");
    return 0;
}

static void sha_disable_clkgate(void)
{
    /* Read current clock gating register value */
    uint32_t clkgate_reg = *((volatile uint32_t *)EM32_CLKGATEREG);

    /* Clear the ENCRYPT clock gate bit (bit 6) to enable crypto clock */
    clkgate_reg &= ~(1U << HCLKG_ENCRYPT);

    /* Write back the modified value */
    *((volatile uint32_t *)EM32_CLKGATEREG) = clkgate_reg;


}

static void sha_reset(const struct device *dev)
{
    uint32_t ctrl;

    /* Reset SHA engine */
    ctrl = sha_read_reg(dev, SHA_CTR_OFFSET);
    ctrl |= SHA_RST_BIT;
    sha_write_reg(dev, SHA_CTR_OFFSET, ctrl);

    /* Wait for reset completion */
    while (sha_read_reg(dev, SHA_CTR_OFFSET) & SHA_RST_BIT) {
        k_busy_wait(1);
    }
}

static void sha_configure(const struct device *dev)
{
    uint32_t ctrl = 0;

    /* Configure byte reversal for input and output */
    ctrl |= SHA_WR_REV_BIT | SHA_RD_REV_BIT;

#ifdef CONFIG_CRYPTO_EM32_SHA_INTERRUPT
    ctrl |= SHA_INT_MASK_BIT;
#endif

    sha_write_reg(dev, SHA_CTR_OFFSET, ctrl);
}

/* Zephyr Crypto API Implementation */

static int crypto_em32_query_hw_caps(const struct device *dev)
{
    return CAP_SEPARATE_IO_BUFS | CAP_SYNC_OPS;
}

/* Application helper: set total message length for chunked processing.
 * This ensures the first chunk programs DATALEN to the TOTAL byte count.
 */
int crypto_em32_sha_set_total_length(const struct device *dev, size_t total_bytes)
{
    LOG_DBG("Set expected total length: %zu bytes", total_bytes);
    if (!dev) {
        return -EINVAL;
    }
    struct crypto_em32_data *data = dev->data;
    data->expected_total_bytes = total_bytes;
    data->have_expected_total = true;
    LOG_DBG("Expected total length set: %zu bytes", data->expected_total_bytes);
    LOG_DBG("have_expected_total=%d", data->have_expected_total);
    return 0;
}

static int em32_sha256_handler(struct hash_ctx *ctx, struct hash_pkt *pkt, bool finish)
{
    const struct device *dev = ctx->device;
    struct crypto_em32_data *data = dev->data;
    const struct crypto_em32_config *config = dev->config;

    if (!data->session_active || data->ctx != ctx) {
        return -EINVAL;
    }

    if (data->state == SHA_STATE_ERROR) {
        return -EIO;
    }

    /* Handle data input (non-finish calls) */
    if (!finish && pkt->in_len > 0) {
        if (!pkt->in_buf) {
            LOG_ERR("Null input buffer pointer");
            return -EINVAL;
        }
        /* Check if we need to switch to chunked processing FIRST */
        /* Switch to chunked if:
         * 1. Single input chunk is 64KB or larger (chunk-sized input), OR
         * 2. Total data exceeds max accumulation size
         */
        if (!data->use_chunked && pkt->in_len >= SHA256_CHUNK_SIZE) {
            LOG_INF("Switching to chunked processing for large input (input=%zu bytes >= %u bytes)",
                    pkt->in_len, SHA256_CHUNK_SIZE);
            data->use_chunked = true;
            data->total_bytes_processed = 0;
            data->chunk_state_valid = false;
            /* Inform driver of full message size so DATALEN is programmed once. */
            (void)crypto_em32_sha_set_total_length(dev, pkt->in_len);
        }

        /* Prefer legacy small-buffer path if it fits and we haven't switched */
        if (!data->use_chunked &&
            (data->buffer_len + pkt->in_len) < sizeof(data->buffer)) {
            memcpy(&data->buffer[data->buffer_len], pkt->in_buf, pkt->in_len);
            data->buffer_len += pkt->in_len;
            data->total_len = data->buffer_len;
            return 0;
        }

        if (data->use_chunked) {
            if (pkt->in_len > 0) {
                /* Check if this is the first chunk */
                bool is_first = !data->chunk_state_valid;

                /* Initialize state if this is the first chunk */
                if (is_first) {
                    sha_init_state(data->chunk_state);
                    data->chunk_state_valid = true;
                    data->chunk_message_bits = 0;
                }

                /* For chunked mode, if total length is known we pre-program PAD_CTR
                 * on the first chunk. We stream all bytes across chunks; the hardware
                 * will interpret the final partial word using valid_byte.
                 */
                size_t write_len = pkt->in_len;

                /* Determine total message bits for this hardware op */
                uint64_t total_bits = (is_first && data->have_expected_total)
                                        ? ((uint64_t)data->expected_total_bytes * 8ULL)
                                        : ((data->total_bytes_processed + write_len) * 8ULL);

                /* Process chunk with state continuation */
                int ret = 0;
                if (write_len > 0) {
                    ret = process_sha256_hardware(dev, pkt->in_buf, write_len,
                                                  data->chunk_state, total_bits, is_first);
                    if (ret) {
                        return ret;
                    }
                }

                /* Update tracking */
                data->total_bytes_processed += write_len;
                if (data->have_expected_total) {
                    data->chunk_message_bits = (uint64_t)data->expected_total_bytes * 8ULL;
                } else {
                    data->chunk_message_bits = (uint64_t)data->total_bytes_processed * 8ULL;
                }
            }
        } else {
            /* Still fits in small-buffer path: already appended above. */
        }
        return 0;
    }

    /* Handle finalization */
    if (finish) {
        data->state = SHA_STATE_BUSY;

        /* Validate output buffer */
        if (!pkt->out_buf) {
            LOG_ERR("Null output buffer");
            data->state = SHA_STATE_ERROR;
            return -EINVAL;
        }

        /* Determine final source and size for hardware processing */
        const uint8_t *src;
        size_t total_bytes;
        uint64_t total_message_bits;

        if (data->use_chunked) {
            /* For chunked mode: if total length was known, PAD_CTR was set on the first
             * chunk. Otherwise, set PAD_CTR now based on the bytes we've processed.
             */
            if (!data->have_expected_total) {
                uint64_t total_bits = (uint64_t)data->total_bytes_processed * 8ULL;
                uint32_t rem = (uint32_t)(data->total_bytes_processed % 4U);
                uint32_t valid_enc = rem & 0x3U; /* 0..3 */
                uint32_t bmod = (uint32_t)(total_bits % 512ULL);
                uint32_t pad_packet = (bmod < 448U) ? ((512U - bmod - 64U) / 32U)
                                                    : ((512U - bmod + 448U) / 32U);
        uint32_t pad_ctrl = (valid_enc << 8) | (pad_packet & 0x1F);
        sys_write32(pad_ctrl, config->base + SHA_PAD_CTR_OFFSET);
        /* Read back PAD_CTR and DATALEN for additional debug */
        {
            uint32_t pad_rb = sys_read32(config->base + SHA_PAD_CTR_OFFSET);
            uint32_t dlo = sys_read32(config->base + SHA_DATALEN_OFFSET);
            uint32_t dhi = sys_read32(config->base + SHA_DATALEN_5832_OFFSET);
            LOG_DBG("Finalize (chunked): PAD_CTR set late (bytes=%zu, bmod=%u, pad_packet=%u, pad_ctrl=0x%08x) PAD_CTR_REG=0x%08x DATALEN_LO=0x%08x DATALEN_HI=0x%08x",
                (size_t)data->total_bytes_processed, bmod, pad_packet, pad_ctrl,
                pad_rb, dlo, dhi);
        }
        LOG_DBG("Finalize context: have_expected_total=%d expected_total_bytes=%zu total_bytes_processed=%zu",
            data->have_expected_total, data->expected_total_bytes, (size_t)data->total_bytes_processed);
            }
            uint32_t timeout = 0;
            uint32_t ctrl_val;
            while (1) {
                ctrl_val = sys_read32(config->base + SHA_CTR_OFFSET);
                if (ctrl_val & SHA_STA_BIT) {
                    LOG_DBG("Finalize (chunked): STA_BIT set after %u iterations", timeout);
                    break;
                }
                if (timeout++ > CONFIG_CRYPTO_EM32_SHA_TIMEOUT_USEC) {
                    LOG_ERR("Timeout waiting for SHA256 completion at finalization (timeout=%u)", timeout);
                    LOG_ERR("Final CTR=0x%08x (READY=%d, STA=%d)",
                            ctrl_val,
                            (ctrl_val & SHA_READY_BIT) ? 1 : 0,
                            (ctrl_val & SHA_STA_BIT) ? 1 : 0);
                    data->state = SHA_STATE_ERROR;
                    return -ETIMEDOUT;
                }
                k_busy_wait(1);
            }

            /* Clear interrupt and read final result */
            sys_write32(sys_read32(config->base + SHA_CTR_OFFSET) | SHA_INT_CLR_BIT,
                        config->base + SHA_CTR_OFFSET);

            /* Read final hash from hardware registers */
            uint32_t *output32 = (uint32_t *)pkt->out_buf;
            for (int i = 0; i < 8; i++) {
                output32[i] = sys_read32(config->base + SHA_OUT_OFFSET + i * 4);
            }
            data->state = SHA_STATE_IDLE;
            return 0;
        } else {
            src = data->buffer;
            total_bytes = data->buffer_len;
            total_message_bits = (uint64_t)total_bytes * 8ULL;
        }

        /* Step 1: Configure byte order */
        uint32_t ctrl_reg = SHA_WR_REV_BIT | SHA_RD_REV_BIT;
        sys_write32(ctrl_reg, config->base + SHA_CTR_OFFSET);

        /* Step 2: Program data length (words) and padding */
        uint32_t words_lo = (uint32_t)((total_bytes + 3U) / 4U);
        sys_write32(words_lo, config->base + SHA_DATALEN_OFFSET);
        sys_write32(0,       config->base + SHA_DATALEN_5832_OFFSET);

        /* Valid byte encoding per spec [9:8]:
         * 0: all 4 bytes valid (rem=0), 1: [31:24] valid (rem=1),
         * 2: [31:16] valid (rem=2), 3: [31:8] valid (rem=3)
         */
        uint32_t rem = (uint32_t)(total_bytes % 4U);
        uint32_t valid_enc = rem & 0x3U; /* direct mapping: 0..3 */
        uint32_t bmod = (uint32_t)(total_message_bits % 512ULL);
        uint32_t pad_packet = (bmod < 448U) ? ((512U - bmod - 64U) / 32U)
                                            : ((512U - bmod + 448U) / 32U);
        uint32_t pad_ctrl = (valid_enc << 8) | (pad_packet & 0x1F);
        sys_write32(pad_ctrl, config->base + SHA_PAD_CTR_OFFSET);

        /* Step 3: Start operation and feed all input words */
        ctrl_reg |= SHA_STR_BIT;
        sys_write32(ctrl_reg, config->base + SHA_CTR_OFFSET);

        uint32_t words_to_write = (uint32_t)((total_bytes + 3U) / 4U);
        uint32_t words_written = 0;
        size_t bytes_written = 0;
        while (words_written < words_to_write) {
            uint32_t w = 0;
            for (int j = 0; j < 4; j++) {
                if (bytes_written < total_bytes) {
                    w |= ((uint32_t)src[bytes_written]) << (j * 8);
                    bytes_written++;
                }
            }
            sys_write32(w, config->base + SHA_IN_OFFSET);

            words_written++;
            if ((words_written % 16U) == 0U) {
                for (int j = 0; j < 6; j++) {
                    __asm__ volatile ("nop");
                }
                while (!(sys_read32(config->base + SHA_CTR_OFFSET) & SHA_READY_BIT)) {}
            }
        }

        /* Step 4: Wait for completion with timeout */
        uint32_t timeout = 0;
        while (!(sys_read32(config->base + SHA_CTR_OFFSET) & SHA_STA_BIT)) {
            if (timeout++ > CONFIG_CRYPTO_EM32_SHA_TIMEOUT_USEC) {
                LOG_ERR("Timeout");
                data->state = SHA_STATE_ERROR;
                return -ETIMEDOUT;
            }
            k_busy_wait(1);
        }

        /* Step 5: Clear interrupt and read result */
        sys_write32(sys_read32(config->base + SHA_CTR_OFFSET) | SHA_INT_CLR_BIT,
                    config->base + SHA_CTR_OFFSET);

        uint32_t *output32 = (uint32_t *)pkt->out_buf;
        for (int i = 0; i < 8; i++) {
            output32[i] = sys_read32(config->base + SHA_OUT_OFFSET + i * 4);
        }

        data->state = SHA_STATE_IDLE;
    }

    return 0;
}

static int crypto_em32_hash_begin_session(const struct device *dev,
                                         struct hash_ctx *ctx,
                                         enum hash_algo algo)
{
    struct crypto_em32_data *data = dev->data;

    if (data->session_active) {
        return -EBUSY;
    }

    if (algo != CRYPTO_HASH_ALGO_SHA256) {
        return -ENOTSUP;
    }

    data->ctx = ctx;
    data->state = SHA_STATE_IDLE;

    /* Initialize legacy small-data buffer */
    data->total_len = 0;
    data->buffer_len = 0;



    /* Initialize chunked processing state */
    data->use_chunked = false;
    data->total_bytes_processed = 0;
    data->chunk_state_valid = false;
    data->chunk_message_bits = 0;
    memset(data->chunk_state, 0, sizeof(data->chunk_state));

    data->expected_total_bytes = 0;
    data->have_expected_total = false;

    /* Initialize chunk buffer */
    data->chunk_buf = NULL;
    data->chunk_buf_len = 0;
    data->chunk_buf_cap = 0;

    /* Final partial-word defaults */
    data->final_rem_len = 0;
    memset(data->final_rem_buf, 0, sizeof(data->final_rem_buf));

    data->session_active = true;

    /* Bind the driver's hash handler into the session context.
     * This is the critical wiring point where Zephyr's higher-level
     * hash API (hash_begin_session/hash_update/hash_compute) is
     * connected to this driver's implementation. After this assignment
     * calls like `hash_update(&ctx, &pkt)` will invoke
     * `em32_sha256_handler(ctx, &pkt, false)` and finalization via
     * `hash_compute()` or `ctx.hash_hndlr(&ctx, &pkt, true)` will call
     * `em32_sha256_handler(..., true)` which in turn drives the
     * hardware via `process_sha256_hardware()` and the SHA registers.
     */
    ctx->hash_hndlr = em32_sha256_handler;

    /* Reset and configure hardware */
    sha_reset(dev);
    sha_configure(dev);



    return 0;
}

/* Convenience helper: hash a memory region specified by base + offset.
 * This implements the new EC control pattern where the EC only specifies
 * the memory region on the EM32F967 to be hashed. Data remains in SoC memory.
 */
int crypto_em32_sha_hash_region(const struct device *dev,
                                uintptr_t base_addr,
                                size_t offset,
                                size_t length,
                                uint8_t out_hash[SHA256_DIGEST_SIZE])
{
    if (!dev || !out_hash || length == 0) {
        return -EINVAL;
    }

    const uint8_t *ptr = (const uint8_t *)(base_addr + offset);

    /* Begin a synchronous SHA256 session and stream from mapped memory. */
    struct hash_ctx ctx = {0};
    struct hash_pkt pkt = {
        .in_buf = (uint8_t *)ptr,
        .in_len = length,
        .out_buf = out_hash,
    };

    int ret;

    ctx.flags = CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;

    ret = hash_begin_session(dev, &ctx, CRYPTO_HASH_ALGO_SHA256);
    if (ret) {
        LOG_ERR("hash_begin_session failed: %d", ret);
        return ret;
    }

    /* Inform driver of full message size so DATALEN is programmed once. */
    (void)crypto_em32_sha_set_total_length(dev, length);

    /* Single update referencing the memory-mapped region. */
    ret = hash_update(&ctx, &pkt);
    if (ret) {
        LOG_ERR("hash_update failed: %d", ret);
        (void)hash_free_session(dev, &ctx);
        return ret;
    }

    /* Finalize: per Zephyr API, call compute() with zero input to get digest. */
    pkt.in_buf = NULL;
    pkt.in_len = 0;
    ret = hash_compute(&ctx, &pkt);
    if (ret) {
        LOG_ERR("hash_compute (final) failed: %d", ret);
    }

    (void)hash_free_session(dev, &ctx);
    return ret;
}

static int crypto_em32_hash_free_session(const struct device *dev,
                                        struct hash_ctx *ctx)
{
    struct crypto_em32_data *data = dev->data;

    if (!data->session_active || data->ctx != ctx) {
        return -EINVAL;
    }



    /* Clear chunked processing state */
    data->use_chunked = false;
    data->total_bytes_processed = 0;
    data->chunk_state_valid = false;
    data->chunk_message_bits = 0;
    memset(data->chunk_state, 0, sizeof(data->chunk_state));

    /* Free chunk buffer */
    if (data->chunk_buf) {
        if (data->chunk_buf_len) {
            memset(data->chunk_buf, 0, data->chunk_buf_len);
        }
        k_free(data->chunk_buf);
        data->chunk_buf = NULL;
    }
    data->chunk_buf_len = 0;
    data->chunk_buf_cap = 0;

    /* Clear small buffer and final remainder */
    memset(data->buffer, 0, sizeof(data->buffer));
    data->expected_total_bytes = 0;
    data->have_expected_total = false;
    data->final_rem_len = 0;
    memset(data->final_rem_buf, 0, sizeof(data->final_rem_buf));

    data->session_active = false;
    data->ctx = NULL;
    data->state = SHA_STATE_IDLE;



    return 0;
}

#ifdef CONFIG_CRYPTO_EM32_SHA_INTERRUPT
static int crypto_em32_hash_async_callback_set(const struct device *dev,
                                              hash_completion_cb cb)
{
    struct crypto_em32_data *data = dev->data;
    data->callback = cb;
    return 0;
}

static void crypto_em32_isr(const struct device *dev)
{
    struct crypto_em32_data *data = dev->data;
    uint32_t status;

    status = sha_read_reg(dev, SHA_CTR_OFFSET);

    if (status & SHA_STA_BIT) {
        /* Clear interrupt */
        uint32_t ctrl = status | SHA_INT_CLR_BIT;
        sha_write_reg(dev, SHA_CTR_OFFSET, ctrl);

        /* Signal completion */
        k_sem_give(&data->op_complete);

        /* Call callback if set */
        if (data->callback) {
            struct hash_pkt pkt = { .ctx = data->ctx };
            data->callback(&pkt, 0);
        }
    }
}
#endif

static const struct crypto_driver_api crypto_em32_api = {
    .query_hw_caps = crypto_em32_query_hw_caps,
    .hash_begin_session = crypto_em32_hash_begin_session,
    .hash_free_session = crypto_em32_hash_free_session,
#ifdef CONFIG_CRYPTO_EM32_SHA_INTERRUPT
    .hash_async_callback_set = crypto_em32_hash_async_callback_set,
#endif
};

static int crypto_em32_init(const struct device *dev)
{
    const struct crypto_em32_config *cfg = dev->config;
    struct crypto_em32_data *data = dev->data;
    int ret;

    /* Disable crypto clock gate first to enable crypto clock */
    sha_disable_clkgate();

    /* Enable clock using the same pattern as TRNG driver */
    if (cfg->clock_dev) {
        if (!device_is_ready(cfg->clock_dev)) {
            LOG_ERR("Crypto clock device not ready");
            return -ENODEV;
        }

        struct elan_em32_clock_control_subsys clk_subsys = {
            .clock_group = cfg->clock_group_id
        };

        ret = clock_control_on(cfg->clock_dev, &clk_subsys);
        if (ret < 0) {
            LOG_ERR("Failed to enable clock: %d", ret);
            return ret;
        }

    }

    /* Initialize data structure */
    data->session_active = false;
    data->state = SHA_STATE_IDLE;
    data->ctx = NULL;
    data->callback = NULL;

#ifdef CONFIG_CRYPTO_EM32_SHA_INTERRUPT
    k_sem_init(&data->op_complete, 0, 1);

    /* Configure interrupts */
    if (cfg->irq_config_func) {
        cfg->irq_config_func(dev);
    }
#endif

    /* Small delay to ensure clocks are stable */
    k_msleep(10);

    /* Reset hardware */
    sha_reset(dev);



    return 0;
}

#define CRYPTO_EM32_INIT(n)                                                    \
    IF_ENABLED(CONFIG_CRYPTO_EM32_SHA_INTERRUPT,                              \
              (static void crypto_em32_irq_config_##n(const struct device *dev);)) \
                                                                               \
    static const struct crypto_em32_config crypto_em32_config_##n = {         \
        .base = DT_INST_REG_ADDR(n),                                         \
        .clock_dev = DEVICE_DT_GET_OR_NULL(DT_INST_CLOCKS_CTLR(n)),          \
        .clock_group_id = HCLKG_ENCRYPT, \
        IF_ENABLED(CONFIG_CRYPTO_EM32_SHA_INTERRUPT,                          \
                  (.irq_config_func = crypto_em32_irq_config_##n,))           \
    };                                                                         \
                                                                               \
    static struct crypto_em32_data crypto_em32_data_##n;                      \
                                                                               \
    DEVICE_DT_INST_DEFINE(n, crypto_em32_init, NULL,                         \
                         &crypto_em32_data_##n, &crypto_em32_config_##n,      \
                         POST_KERNEL, CONFIG_CRYPTO_INIT_PRIORITY,             \
                         &crypto_em32_api);                                    \
                                                                               \
    IF_ENABLED(CONFIG_CRYPTO_EM32_SHA_INTERRUPT,                              \
              (static void crypto_em32_irq_config_##n(const struct device *dev) \
               {                                                               \
                   IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),    \
                              crypto_em32_isr, DEVICE_DT_INST_GET(n), 0);     \
                   irq_enable(DT_INST_IRQN(n));                              \
               }))

DT_INST_FOREACH_STATUS_OKAY(CRYPTO_EM32_INIT)
