/* bms_hist.c（BMSStateHistory 编码器）离线验证：
 * 用 mqttagent 参考实现（air780e/bms_codec.lua）的黄金向量做字节级比对，
 * 确保 ESP32 端编码结果与 Go 服务端 internal/bms 解码兼容。
 *
 * 向量生成：lua tools/golden_gen.lua <mqttagent 根目录> > tools/golden_vectors.h
 *
 * 运行（宿主，WSL/Linux）：
 *   gcc -fno-builtin -O1 -Wall -I main tools/bms_hist_test.c main/bms_hist.c \
 *       -o /tmp/bms_hist_test && /tmp/bms_hist_test
 * 退出码 0 = 全部通过，非 0 编码了失败向量/字节位置。 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "../main/bms_hist.h"
#include "golden_vectors.h"

#ifndef BMS_TEST_FREESTANDING
#include <stdio.h>
#include <string.h>
#define TEST_LOG(...) printf(__VA_ARGS__)
#else
#define TEST_LOG(...) ((void)0)
/* freestanding（无 CRT）：自带 memcpy/memset */
void *memcpy(void *dst, const void *src, size_t n)
{
    uint8_t *d = (uint8_t *)dst;
    const uint8_t *s = (const uint8_t *)src;
    for (size_t i = 0; i < n; i++) {
        d[i] = s[i];
    }
    return dst;
}

void *memset(void *dst, int c, size_t n)
{
    uint8_t *d = (uint8_t *)dst;
    for (size_t i = 0; i < n; i++) {
        d[i] = (uint8_t)c;
    }
    return dst;
}
#endif

static int hex_val(char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    return -1;
}

static size_t hex_to_bytes(const char *hex, uint8_t *out, size_t cap)
{
    size_t n = 0;
    while (hex[0] != '\0' && hex[1] != '\0' && n < cap) {
        int hi = hex_val(hex[0]);
        int lo = hex_val(hex[1]);
        if (hi < 0 || lo < 0) {
            break;
        }
        out[n++] = (uint8_t)((hi << 4) | lo);
        hex += 2;
    }
    return n;
}

static void set_sample(bms_hist_t *h, size_t i, int16_t temp1, int16_t temp2,
                       int16_t temp_mos, uint16_t balan, uint32_t vol,
                       int32_t cur, uint32_t cycle_cap, uint32_t cap_remain,
                       uint32_t time_s)
{
    h->temp1[i] = temp1;
    h->temp2[i] = temp2;
    h->temp_mos[i] = temp_mos;
    h->balan_current[i] = balan;
    h->bat_vol[i] = vol;
    h->bat_current[i] = cur;
    h->cycle_cap[i] = cycle_cap;
    h->cap_remain[i] = cap_remain;
    h->time_s[i] = time_s;
}

/* mode 0 = 运行态序列；1 = golden1；2 = golden2 静态 */
static void set_cells(bms_hist_t *h, size_t i, uint8_t cell_count, int mode)
{
    uint16_t *row = &h->cells[i * BMS_MAX_CELLS];
    for (uint8_t j = 0; j < cell_count; j++) {
        if (mode == 0) {
            row[j] = (uint16_t)(3300 + j * 2 + (uint8_t)((i + j) % 11));
        } else if (mode == 1) {
            row[j] = (uint16_t)(3300 + j + (i > 0 ? 1 : 0));
        } else {
            row[j] = (uint16_t)(3300 + j);
        }
    }
}

static int check_vector(const bms_hist_t *h, const char *expect_hex)
{
    static uint8_t got[BMS_HIST_MAX_SIZE];
    static uint8_t want[2048];

    size_t got_len = bms_hist_encode(h, got, sizeof(got));
    size_t want_len = hex_to_bytes(expect_hex, want, sizeof(want));
    if (got_len != want_len) {
        TEST_LOG("  size mismatch: got=%u want=%u\n", (unsigned)got_len,
                 (unsigned)want_len);
        return (int)(got_len > want_len ? got_len - want_len : want_len - got_len) + 1;
    }
    for (size_t i = 0; i < got_len; i++) {
        if (got[i] != want[i]) {
            TEST_LOG("  first diff at byte %u: got=%02x want=%02x (len=%u)\n",
                     (unsigned)i, got[i], want[i], (unsigned)got_len);
            return (int)i + 1;
        }
    }
    TEST_LOG("  OK (%u bytes)\n", (unsigned)got_len);
    return 0;
}

static int run_tests(void)
{
    static bms_hist_t h;

    /* 向量 1：golden1（两样本三电芯） */
    memset(&h, 0, sizeof(h));
    h.count = 2;
    h.cell_count = 3;
    set_sample(&h, 0, -125, 231, 450, 12, 52340, -1800, 123456, 98765, 1700000000);
    set_cells(&h, 0, 3, 1);
    set_sample(&h, 1, -124, 232, 451, 13, 52341, -1799, 123457, 98766, 1700000001);
    set_cells(&h, 1, 3, 1);
    TEST_LOG("golden1 (2 samples x 3 cells)\n");
    int rc = check_vector(&h, GOLDEN1);
    if (rc != 0) {
        return 1;
    }

    /* 向量 2：golden2（三样本两电芯，全列 Constant） */
    memset(&h, 0, sizeof(h));
    h.count = 3;
    h.cell_count = 2;
    for (size_t i = 0; i < 3; i++) {
        set_sample(&h, i, 250, 248, 300, 10, 52000, 0, 100000, 90000, 1700000000);
        set_cells(&h, i, 2, 2);
    }
    TEST_LOG("golden2 (3 samples x 2 cells, constant)\n");
    rc = check_vector(&h, GOLDEN2);
    if (rc != 0) {
        return 10 + rc;
    }

    /* 向量 3：30 样本 × 20 电芯，时间递增 */
    memset(&h, 0, sizeof(h));
    h.count = 30;
    h.cell_count = 20;
    for (size_t i = 0; i < 30; i++) {
        set_sample(&h, i, (int16_t)(250 + i % 3), (int16_t)(248 + i % 2),
                   (int16_t)(300 + i % 4), (uint16_t)(10 + i % 2),
                   (uint32_t)(52000 + i), (int32_t)(-20000 + (int32_t)((i * 3700) % 90001)),
                   (uint32_t)(100000 + i), (uint32_t)(90000 - i),
                   (uint32_t)(1800000000 + i));
        set_cells(&h, i, 20, 0);
    }
    TEST_LOG("active30 (30 samples x 20 cells, 时间递增)\n");
    rc = check_vector(&h, ACTIVE30);
    if (rc != 0) {
        return 50 + rc;
    }

    /* 向量 4：同上但时间列恒定（fake 模式，420 字节） */
    memset(&h, 0, sizeof(h));
    h.count = 30;
    h.cell_count = 20;
    for (size_t i = 0; i < 30; i++) {
        set_sample(&h, i, (int16_t)(250 + i % 3), (int16_t)(248 + i % 2),
                   (int16_t)(300 + i % 4), (uint16_t)(10 + i % 2),
                   (uint32_t)(52000 + i), (int32_t)(-20000 + (int32_t)((i * 3700) % 90001)),
                   (uint32_t)(100000 + i), (uint32_t)(90000 - i),
                   (uint32_t)1800000000);
        set_cells(&h, i, 20, 0);
    }
    TEST_LOG("fake30 (30 samples x 20 cells, 时间恒定)\n");
    rc = check_vector(&h, FAKE30);
    if (rc != 0) {
        return 100 + rc;
    }

    return 0;
}

/* 宿主（WSL/Linux gcc）入口 */
int main(void)
{
    return run_tests();
}

/* freestanding（Windows -Wl,-e,bms_test_entry）入口 */
int bms_test_entry(void)
{
    return run_tests();
}
