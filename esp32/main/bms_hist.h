#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bms_interface.h"

/* BMS 历史批数据（移植自 mqttagent：air780e/bms.lua + bms_codec.lua）
 *   - 每秒采样一条，攒满 batch_samples 条（默认 30）作为一个批次；
 *   - 按 docs/bms.md 的 BMSStateHistory 压缩格式编码（与 Go 端 internal/bms 字节级兼容）；
 *   - 由 bms_uplink 通过 MQTT 发布到 /bms/<device>/status。 */

#define BMS_HIST_MAX_SAMPLES 30
#define BMS_HIST_MAX_SIZE    4096

typedef struct {
    uint8_t batch_samples; /* 批大小（编码时的样本数） */
    uint8_t count;         /* 当前已累计样本数 */
    uint8_t cell_count;    /* 本批电芯数（0 = 无电芯列） */
    uint32_t total;        /* 累计采样次数（诊断用） */

    int16_t temp1[BMS_HIST_MAX_SAMPLES];         /* 0.1℃ */
    int16_t temp2[BMS_HIST_MAX_SAMPLES];         /* 0.1℃ */
    int16_t temp_mos[BMS_HIST_MAX_SAMPLES];      /* 0.1℃ */
    uint16_t balan_current[BMS_HIST_MAX_SAMPLES]; /* mA */
    uint32_t bat_vol[BMS_HIST_MAX_SAMPLES];      /* mV */
    int32_t bat_current[BMS_HIST_MAX_SAMPLES];   /* mA */
    uint32_t cycle_cap[BMS_HIST_MAX_SAMPLES];    /* mAh（SOCCycleCap） */
    uint32_t cap_remain[BMS_HIST_MAX_SAMPLES];   /* mAh（SOCCapRemain） */
    uint32_t time_s[BMS_HIST_MAX_SAMPLES];       /* Unix 秒 */
    uint16_t cells[BMS_HIST_MAX_SAMPLES * BMS_MAX_CELLS]; /* mV，按样本连续 */
} bms_hist_t;

void bms_hist_init(bms_hist_t *h, uint8_t batch_samples);

/* 追加一条采样；返回 true 表示本批已满，可调用 bms_hist_encode 上报。 */
bool bms_hist_add(bms_hist_t *h, const bms_snapshot_t *snap, uint32_t unix_time);

/* 编码当前批次（BMSStateHistory）。返回编码字节数，0 = 失败/缓冲不足。 */
size_t bms_hist_encode(const bms_hist_t *h, uint8_t *out, size_t cap);

/* 上报完成后重置批次（保留累计计数与电芯数配置）。 */
void bms_hist_reset(bms_hist_t *h);
