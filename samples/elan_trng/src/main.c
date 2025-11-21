#include <zephyr/kernel.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(elan_trng_sample, LOG_LEVEL_INF);

#define ENTROPY_BUFFER_SIZE 32
#define TEST_ITERATIONS 10

/* Simple entropy quality checks */
static void analyze_entropy_quality(uint8_t *data, size_t len)
{
    uint32_t bit_count = 0;
    uint32_t byte_histogram[256] = {0};
    uint32_t transitions = 0;
    
    /* Count 1-bits and byte frequency */
    for (size_t i = 0; i < len; i++) {
        byte_histogram[data[i]]++;
        
        /* Count bit transitions */
        if (i > 0) {
            uint8_t xor_result = data[i] ^ data[i-1];
            while (xor_result) {
                if (xor_result & 1) transitions++;
                xor_result >>= 1;
            }
        }
        
        /* Count 1-bits */
        uint8_t byte = data[i];
        while (byte) {
            if (byte & 1) bit_count++;
            byte >>= 1;
        }
    }
    
    /* Calculate statistics (use double to avoid float -> double promotion warnings with variadic logging) */
    double bit_bias = (double)bit_count / (double)(len * 8);
    double transition_rate = (double)transitions / (double)(len * 8 - 1);
    
    /* Find most/least frequent bytes */
    uint32_t max_freq = 0, min_freq = UINT32_MAX;
    for (int i = 0; i < 256; i++) {
        if (byte_histogram[i] > max_freq) max_freq = byte_histogram[i];
        if (byte_histogram[i] < min_freq) min_freq = byte_histogram[i];
    }
    
    LOG_INF("Entropy Quality Analysis:");
    LOG_INF("  Bit bias: %.3f (ideal: 0.5)", bit_bias);
    LOG_INF("  Transition rate: %.3f (ideal: 0.5)", transition_rate);
    LOG_INF("  Byte frequency range: %u - %u (ideal: uniform)", min_freq, max_freq);
    
    /* Simple quality assessment */
    bool quality_good = (bit_bias > 0.4 && bit_bias < 0.6) &&
                        (transition_rate > 0.4 && transition_rate < 0.6) &&
                        (max_freq - min_freq < len / 4);
    
    LOG_INF("  Overall quality: %s", quality_good ? "GOOD" : "POOR");
}

static void print_hex_data(const char *label, uint8_t *data, size_t len)
{
    printk("%s: ", label);
    for (size_t i = 0; i < len; i++) {
        printk("%02x", data[i]);
        if ((i + 1) % 16 == 0) {
            printk("\n");
            if (i + 1 < len) printk("      ");
        } else if ((i + 1) % 4 == 0) {
            printk(" ");
        }
    }
    if (len % 16 != 0) printk("\n");
}

static int test_entropy_performance(const struct device *dev)
{
    uint8_t buffer[ENTROPY_BUFFER_SIZE];
    uint32_t start_time, end_time;
    int total_bytes = 0;

    LOG_INF("=== Performance Test ===");

    start_time = k_uptime_get_32();

    for (int i = 0; i < TEST_ITERATIONS; i++) {
        int rc = entropy_get_entropy(dev, buffer, sizeof(buffer));
        if (rc != 0) {
            LOG_ERR("entropy_get_entropy failed on iteration %d: %d", i, rc);
            return rc;
        }
        total_bytes += sizeof(buffer);
    }

    end_time = k_uptime_get_32();
    uint32_t duration_ms = end_time - start_time;

    if (duration_ms > 0) {
        uint32_t throughput = (total_bytes * 1000) / duration_ms;
        LOG_INF("Generated %d bytes in %u ms", total_bytes, duration_ms);
        LOG_INF("Throughput: %u bytes/sec", throughput);
    }

    return 0;
}





static int test_entropy_repeatability(const struct device *dev)
{
    uint8_t buffer1[ENTROPY_BUFFER_SIZE];
    uint8_t buffer2[ENTROPY_BUFFER_SIZE];
    int rc;
    
    LOG_INF("=== Repeatability Test ===");
    
    rc = entropy_get_entropy(dev, buffer1, sizeof(buffer1));
    if (rc != 0) {
        LOG_ERR("First entropy_get_entropy failed: %d", rc);
        return rc;
    }
    
    k_sleep(K_MSEC(10)); /* Small delay */
    
    rc = entropy_get_entropy(dev, buffer2, sizeof(buffer2));
    if (rc != 0) {
        LOG_ERR("Second entropy_get_entropy failed: %d", rc);
        return rc;
    }
    
    /* Check if buffers are identical (they shouldn't be) */
    bool identical = (memcmp(buffer1, buffer2, sizeof(buffer1)) == 0);
    
    LOG_INF("Two consecutive reads are %s", identical ? "IDENTICAL (BAD!)" : "DIFFERENT (GOOD)");
    
    if (identical) {
        LOG_WRN("TRNG may not be working properly - identical outputs detected");
        print_hex_data("Buffer 1", buffer1, sizeof(buffer1));
        print_hex_data("Buffer 2", buffer2, sizeof(buffer2));
    }
    
    return identical ? -1 : 0;
}

int main(void)
{
    const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(trng0));
    uint8_t buffer[ENTROPY_BUFFER_SIZE];
    int rc;

    /* Early console test to ensure system stability */
    printk("\n*** CONSOLE TEST - EM32F967 TRNG Sample Starting ***\n");
    k_msleep(100);
    printk("Console is working properly!\n");

	k_sleep(K_SECONDS(1));
    LOG_INF("=== EM32F967 TRNG Test Application ===");

    LOG_INF("Running in POLLING mode");

    if (!device_is_ready(dev)) {
        LOG_ERR("TRNG device not ready");
        return -ENODEV;
    }

    LOG_INF("TRNG device ready");
	k_sleep(K_SECONDS(1));

    /* Basic functionality test */
    LOG_INF("=== Basic Functionality Test ===");
    rc = entropy_get_entropy(dev, buffer, sizeof(buffer));
    if (rc != 0) {
        LOG_ERR("entropy_get_entropy failed: %d", rc);
        return rc;
    }
	k_sleep(K_SECONDS(1));
    print_hex_data("Random data", buffer, sizeof(buffer));

	k_sleep(K_SECONDS(1));
	/* Analyze entropy quality */
    analyze_entropy_quality(buffer, sizeof(buffer));

    /* Test repeatability */
    test_entropy_repeatability(dev);

    /* Performance test */
    test_entropy_performance(dev);



    /* Continuous operation test */
    LOG_INF("=== Continuous Operation Test ===");
    int iteration = 0;
    while (1) {
        k_sleep(K_SECONDS(5));

        rc = entropy_get_entropy(dev, buffer, sizeof(buffer));
        if (rc == 0) {
            LOG_INF("Iteration %d: %02x %02x %02x %02x ... %02x %02x %02x %02x",
                    iteration++,
                    buffer[0], buffer[1], buffer[2], buffer[3],
                    buffer[28], buffer[29], buffer[30], buffer[31]);
        } else {
            LOG_ERR("entropy_get_entropy failed: %d", rc);
        }

        /* Every 10 iterations, do a quality check */
        if (iteration % 10 == 0) {
            analyze_entropy_quality(buffer, sizeof(buffer));
        }
    }

    /* Not reached */
    return 0;
}
