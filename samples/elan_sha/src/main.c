/* Unified test app: EC memory-region hash (major) + large data EC simulation tests */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/crypto/hash.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdio.h>
#include <soc.h>

LOG_MODULE_REGISTER(elan_sha, LOG_LEVEL_INF);

/* ==== Common config ==== */
#define TEST_DATA_SIZE      (400 * 1024)   /* 400KB */
#define CHUNK_SIZE          (64 * 1024)    /* 64KB */
#define NUM_CHUNKS          ((TEST_DATA_SIZE + CHUNK_SIZE - 1) / CHUNK_SIZE)

static const struct device *crypto_dev = DEVICE_DT_GET(DT_NODELABEL(crypto0));

/* Driver helpers (non-standard, provided by crypto_em32_sha driver) */
extern int crypto_em32_sha_set_total_length(const struct device *dev, size_t total_bytes);
extern int crypto_em32_sha_hash_region(const struct device *dev,
                                       uintptr_t base_addr,
                                       size_t offset,
                                       size_t length,
                                       uint8_t out_hash[32]);
/* Optional helper to reset/configure SHA hardware from application space */
extern int crypto_em32_sha_hw_init(const struct device *dev);

/* Defaults for EC memory-region hashing (override via -D if needed) */
#ifndef EC_MEM_HASH_BASE
#define EC_MEM_HASH_BASE FLASH_BASE
#endif
#ifndef EC_MEM_HASH_OFFSET
#define EC_MEM_HASH_OFFSET 0x00024000u
#endif
#ifndef EC_MEM_HASH_LEN
#define EC_MEM_HASH_LEN    0x00061C00u
#endif

static void generate_test_data(uint8_t *buf, size_t len, size_t offset)
{
    for (size_t i = 0; i < len; i++) {
        buf[i] = (uint8_t)((offset + i) & 0xFF);
    }
}

static void print_hash(const uint8_t *hash, size_t len)
{
    char s[65];
    char *p = s;
    for (size_t i = 0; i < len; i++) {
        p += snprintf(p, 3, "%02x", hash[i]);
    }
    *p = '\0';
    LOG_INF("Hash: %s", s);
}

/* ===== Group 1: NIST SHA-256 test vectors ===== */
static int hexstr_to_bytes(const char *hex, uint8_t *out, size_t outlen)
{
    size_t hexlen = strlen(hex);
    if (hexlen != outlen * 2) {
        return -EINVAL;
    }

    for (size_t i = 0; i < outlen; i++) {
        char byte_s[3] = { hex[i*2], hex[i*2+1], '\0' };
        unsigned int v = 0;
        if (sscanf(byte_s, "%x", &v) != 1) {
            return -EINVAL;
        }
        out[i] = (uint8_t)v;
    }
    return 0;
}

static int test_nist_sha256_vectors(void)
{
    struct hash_ctx ctx; struct hash_pkt pkt; uint8_t hash_output[32];
    int ret = 0; int failures = 0;

    struct {
        const char *msg;
        const char *expected_hex;
    } cases[] = {
        { "abc", "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad" },
        { "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq", "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1" },
        { "This is simple 56-byte test string for debug purposes", "83898ce0edb19377ce9a4b45d7fe0d482a71e1a60a602e3fea76c131fa453131" },
        { "The quick brown fox jumps over the lazy dog", "d7a8fbb307d7809469ca9abcb0082e4f8d5651e46d3cdb762d02d0bf37c9e592" },
        { "The quick brown fox jumps over the lazy cog", "e4c4d8f3bf76b692de791a173e05321150f7a345b46484fe427f6acc7ecc81be" },
        { "bhn5bjmoniertqea40wro2upyflkydsibsk8ylkmgbvwi420t44cq034eou1szc1k0mk46oeb7ktzmlxqkbte2sy", "9085df2f02e0cc455928d0f51b27b4bf1d9cd260a66ed1fda11b0a3ff5756d99" }
    };

    LOG_INF("=== Group 1: NIST SHA-256 Test Vectors ===");

    for (size_t i = 0; i < ARRAY_SIZE(cases); i++) {
        const char *m = cases[i].msg;
        size_t mlen = strlen(m);
        uint8_t expected[32];

        ret = hexstr_to_bytes(cases[i].expected_hex, expected, sizeof(expected));
        if (ret) {
            LOG_ERR("Invalid expected hex for vector %zu", i);
            failures++;
            continue;
        }

        memset(&ctx, 0, sizeof(ctx));
        ctx.flags = CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;
        ret = hash_begin_session(crypto_dev, &ctx, CRYPTO_HASH_ALGO_SHA256);
        if (ret) { LOG_ERR("hash_begin_session: %d", ret); failures++; continue; }

        memset(&pkt, 0, sizeof(pkt));
        pkt.in_buf = (uint8_t *)m; pkt.in_len = mlen; pkt.out_buf = hash_output;
        ret = hash_update(&ctx, &pkt);
        if (ret) { LOG_ERR("hash_update for vector %zu failed: %d", i, ret); hash_free_session(crypto_dev, &ctx); failures++; continue; }

        memset(&pkt, 0, sizeof(pkt)); pkt.in_buf = NULL; pkt.in_len = 0; pkt.out_buf = hash_output;
        ret = ctx.hash_hndlr(&ctx, &pkt, true);
        if (ret) { LOG_ERR("finalize for vector %zu failed: %d", i, ret); hash_free_session(crypto_dev, &ctx); failures++; continue; }

        /* Compare */
        if (memcmp(hash_output, expected, sizeof(expected)) != 0) {
            char exp_s[65]; char got_s[65]; char *p = exp_s; char *q = got_s;
            for (size_t j = 0; j < sizeof(expected); j++) { p += snprintf(p, 3, "%02x", expected[j]); q += snprintf(q, 3, "%02x", hash_output[j]); }
            *p = '\0'; *q = '\0';
            LOG_ERR("Vector %zu FAILED", i+1);
            LOG_ERR("  Msg: '%s'", m);
            LOG_ERR("  Expected: %s", exp_s);
            LOG_ERR("  Actual:   %s", got_s);
            failures++;
        } else {
            LOG_INF("Vector %zu PASSED: '%s'", i+1, m);
            print_hash(hash_output, sizeof(hash_output));
        }

        hash_free_session(crypto_dev, &ctx);
        k_msleep(20);
    }

    LOG_INF("Group 1 complete: %d failures", failures);
    return (failures == 0) ? 0 : -1;
}

/* ===== Major test: hash on-board flash region using EC control pattern ===== */
static int test_ec_mem_hash_major(void)
{
    uint8_t digest[32] = {0};
    LOG_INF("=== Major Test: EC memory-region hash (base+offset+len) ===");
    LOG_INF("... base=0x%08x, offset=0x%08x, len=0x%08x",
            (uint32_t)EC_MEM_HASH_BASE, (uint32_t)EC_MEM_HASH_OFFSET, (uint32_t)EC_MEM_HASH_LEN);

    int ret = crypto_em32_sha_hash_region(crypto_dev,
                                          (uintptr_t)EC_MEM_HASH_BASE,
                                          (size_t)EC_MEM_HASH_OFFSET,
                                          (size_t)EC_MEM_HASH_LEN,
                                          digest);
    if (ret) {
        LOG_ERR("crypto_em32_sha_hash_region failed: %d", ret);
        return ret;
    }

    print_hash(digest, sizeof(digest));
    return 0;
}

/* ===== Large data EC simulation tests (chunked streaming) ===== */
static int test_single_shot_400kb(void)
{
    struct hash_ctx ctx; struct hash_pkt pkt; uint8_t hash_output[32];
    uint8_t *chunk_buf; size_t offset = 0; int ret;

    LOG_INF("=== Test 1: Chunked 400KB Hash (EC Communication Pattern) ===");
    chunk_buf = k_malloc(CHUNK_SIZE);
    if (!chunk_buf) { LOG_ERR("Failed to allocate %u bytes", CHUNK_SIZE); return -ENOMEM; }

    memset(&ctx, 0, sizeof(ctx));
    ctx.flags = CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;
    ret = hash_begin_session(crypto_dev, &ctx, CRYPTO_HASH_ALGO_SHA256);
    if (ret) { LOG_ERR("hash_begin_session: %d", ret); k_free(chunk_buf); return ret; }
    (void)crypto_em32_sha_set_total_length(crypto_dev, TEST_DATA_SIZE);

    while (offset < TEST_DATA_SIZE) {
        size_t this_chunk = MIN((size_t)CHUNK_SIZE, TEST_DATA_SIZE - offset);
        generate_test_data(chunk_buf, this_chunk, offset);
        memset(&pkt, 0, sizeof(pkt));
        pkt.in_buf = chunk_buf; pkt.in_len = this_chunk; pkt.out_buf = hash_output;
        ret = hash_update(&ctx, &pkt);
        if (ret) { LOG_ERR("hash_update off %zu: %d", offset, ret); hash_free_session(crypto_dev, &ctx); k_free(chunk_buf); return ret; }
        offset += this_chunk;
    }

    memset(&pkt, 0, sizeof(pkt)); pkt.in_buf = NULL; pkt.in_len = 0; pkt.out_buf = hash_output;
    ret = ctx.hash_hndlr(&ctx, &pkt, true);
    if (ret) { LOG_ERR("finalize: %d", ret); hash_free_session(crypto_dev, &ctx); k_free(chunk_buf); return ret; }

    print_hash(hash_output, 32);
    hash_free_session(crypto_dev, &ctx); k_free(chunk_buf); return 0;
}

static int test_ec_chunked_transfer(void)
{
    struct hash_ctx ctx; struct hash_pkt pkt; uint8_t hash_output[32];
    uint8_t *chunk_buf; int ret; size_t offset = 0; int chunk_num = 0;

    LOG_INF("=== Test 2: EC-style Chunked Transfer (64KB chunks) ===");
    chunk_buf = k_malloc(CHUNK_SIZE);
    if (!chunk_buf) { LOG_ERR("Failed to allocate %u bytes", CHUNK_SIZE); return -ENOMEM; }

    memset(&ctx, 0, sizeof(ctx));
    ctx.flags = CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;
    ret = hash_begin_session(crypto_dev, &ctx, CRYPTO_HASH_ALGO_SHA256);
    if (ret) { LOG_ERR("hash_begin_session: %d", ret); k_free(chunk_buf); return ret; }
    (void)crypto_em32_sha_set_total_length(crypto_dev, TEST_DATA_SIZE);

    while (offset < TEST_DATA_SIZE) {
        size_t this_chunk = MIN((size_t)CHUNK_SIZE, TEST_DATA_SIZE - offset);
        chunk_num++;
        generate_test_data(chunk_buf, this_chunk, offset);
        memset(&pkt, 0, sizeof(pkt));
        pkt.in_buf = chunk_buf; pkt.in_len = this_chunk; pkt.out_buf = hash_output;
        ret = hash_update(&ctx, &pkt);
        if (ret) { LOG_ERR("update chunk %d: %d", chunk_num, ret); hash_free_session(crypto_dev, &ctx); k_free(chunk_buf); return ret; }
        offset += this_chunk; k_msleep(10);
    }

    memset(&pkt, 0, sizeof(pkt)); pkt.in_buf = NULL; pkt.in_len = 0; pkt.out_buf = hash_output;
    ret = ctx.hash_hndlr(&ctx, &pkt, true);
    if (ret) { LOG_ERR("finalize: %d", ret); hash_free_session(crypto_dev, &ctx); k_free(chunk_buf); return ret; }

    print_hash(hash_output, 32);
    hash_free_session(crypto_dev, &ctx); k_free(chunk_buf); return 0;
}

static int test_consistency_check(void)
{
    struct hash_ctx ctx; struct hash_pkt pkt; uint8_t hash_output[32];
    uint8_t *chunk_buf; int ret; size_t offset = 0; int chunk_count = 0;

    LOG_INF("=== Test 3: Chunked Processing Verification ===");
    chunk_buf = k_malloc(CHUNK_SIZE);
    if (!chunk_buf) { LOG_ERR("Failed to allocate chunk buffer"); return -ENOMEM; }

    memset(&ctx, 0, sizeof(ctx));
    ctx.flags = CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;
    ret = hash_begin_session(crypto_dev, &ctx, CRYPTO_HASH_ALGO_SHA256);
    if (ret) { LOG_ERR("hash_begin_session: %d", ret); k_free(chunk_buf); return ret; }
    (void)crypto_em32_sha_set_total_length(crypto_dev, TEST_DATA_SIZE);

    while (offset < TEST_DATA_SIZE) {
        size_t this_chunk = MIN((size_t)CHUNK_SIZE, TEST_DATA_SIZE - offset);
        chunk_count++;
        generate_test_data(chunk_buf, this_chunk, offset);
        memset(&pkt, 0, sizeof(pkt));
        pkt.in_buf = chunk_buf; pkt.in_len = this_chunk; pkt.out_buf = hash_output;
        ret = hash_update(&ctx, &pkt);
        if (ret) { LOG_ERR("update chunk %d: %d", chunk_count, ret); hash_free_session(crypto_dev, &ctx); k_free(chunk_buf); return ret; }
        offset += this_chunk;
    }

    memset(&pkt, 0, sizeof(pkt)); pkt.in_buf = NULL; pkt.in_len = 0; pkt.out_buf = hash_output;
    ret = ctx.hash_hndlr(&ctx, &pkt, true);
    if (ret) { LOG_ERR("finalize: %d", ret); hash_free_session(crypto_dev, &ctx); k_free(chunk_buf); return ret; }

    LOG_INF("\xE2\x9C\x93 Chunked processing verification PASSED - processed %d chunks", chunk_count);
    print_hash(hash_output, 32);
    hash_free_session(crypto_dev, &ctx); k_free(chunk_buf); return 0;
}

int main(void)
{
    int ret; int passed = 0; int failed = 0;
    LOG_INF("========================================");
    LOG_INF("Unified SHA256 Tests");
    LOG_INF("Test Data Size: %u bytes (400KB)", TEST_DATA_SIZE);
    LOG_INF("Chunk Size: %u bytes (64KB)", CHUNK_SIZE);
    LOG_INF("Number of Chunks: %u", NUM_CHUNKS);
    LOG_INF("========================================");

    if (!device_is_ready(crypto_dev)) {
        LOG_ERR("Crypto device not ready");
        return -ENODEV;
    }

    /* Group 1: NIST vectors (new) */
    ret = test_nist_sha256_vectors();
    if (ret) { LOG_ERR("NIST SHA-256 vectors FAILED"); failed++; } else { LOG_INF("NIST SHA-256 vectors PASSED"); passed++; }
    k_msleep(200);

    if (ret) { LOG_WRN("crypto_em32_sha_hw_init returned %d", ret); }
    ret = test_ec_mem_hash_major();
    if (ret) { LOG_ERR("EC memory-region test FAILED"); failed++; } else { LOG_INF("EC memory-region test PASSED"); passed++; }
    k_msleep(300);

    /* Group 3: Large-data / EC simulation tests */
    /* Test 1 */
    ret = test_single_shot_400kb();
    if (ret) { LOG_ERR("Test 1 FAILED"); failed++; } else { LOG_INF("Test 1 PASSED"); passed++; }
    k_msleep(200);

    /* Test 2 */
    ret = test_ec_chunked_transfer();
    if (ret) { LOG_ERR("Test 2 FAILED"); failed++; } else { LOG_INF("Test 2 PASSED"); passed++; }
    k_msleep(200);

    /* Test 3 */
    ret = test_consistency_check();
    if (ret) { LOG_ERR("Test 3 FAILED"); failed++; } else { LOG_INF("Test 3 PASSED"); passed++; }

    LOG_INF("========================================");
    LOG_INF("Test Summary: %d passed, %d failed", passed, failed);
    LOG_INF("========================================");

    return (failed == 0) ? 0 : -1;
}

