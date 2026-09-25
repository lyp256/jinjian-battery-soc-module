/* BMSStateHistory 压缩编码器（C 版，等价移植 air780e/bms_codec.lua）。
 * 规范：mqttagent/docs/bms.md
 *   uvarint(count-1) + uvarint((cellCount<<2)|flags) + [modeTable] + 各列
 * 列：temp1/temp2/tempMos(int16 0.1℃)、balanCurrent(uint16 mA)、batVol(uint32 mV)、
 *     batCurrent(int32 mA)、socCycleCap(uint32 mAh)、socCapRemain(uint32 mAh)、
 *     time(uint32 s)、电芯列（offset / spatial 两种布局取较短者）。 */

#include "bms_hist.h"

#include <string.h>

typedef struct {
    uint8_t *buf;
    size_t cap;
    size_t len;
    bool overflow;
} writer_t;

static void w_byte(writer_t *w, uint8_t b)
{
    if (w->buf != NULL) {
        if (w->len < w->cap) {
            w->buf[w->len] = b;
        } else {
            w->overflow = true;
        }
    }
    w->len++;
}

static void w_uvarint(writer_t *w, uint64_t v)
{
    while (v >= 0x80) {
        w_byte(w, (uint8_t)((v & 0x7F) | 0x80));
        v >>= 7;
    }
    w_byte(w, (uint8_t)v);
}

static size_t uvarint_len(uint64_t v)
{
    size_t n = 1;
    while (v >= 0x80) {
        v >>= 7;
        n++;
    }
    return n;
}

static uint64_t zigzag(int64_t n)
{
    if (n >= 0) {
        return (uint64_t)n << 1;
    }
    return (((uint64_t)(-n)) << 1) - 1;
}

static void w_delta_code(writer_t *w, int64_t delta)
{
    if (delta == 0) {
        w_uvarint(w, 1);
        return;
    }
    w_uvarint(w, zigzag(delta) + 2);
}

/* 零/非零 Delta 游程编码（与 Lua encode_delta_sequence 一致） */
static void encode_delta_sequence(writer_t *w, const int64_t *deltas, size_t count)
{
    size_t i = 0;
    size_t remaining = count;

    while (remaining > 0) {
        int64_t delta = deltas[i];
        size_t run = 1;
        while (run < remaining && deltas[i + run] == delta) {
            run++;
        }

        if (delta == 0) {
            if (run == 1) {
                w_uvarint(w, 1);
            } else {
                w_uvarint(w, 0);
                w_uvarint(w, run);
            }
        } else {
            uint64_t ed = zigzag(delta);
            size_t single_size = uvarint_len(ed + 2);
            size_t run_size = 1 + uvarint_len(ed) + uvarint_len(run);
            if (run_size < run * single_size) {
                w_uvarint(w, 2);
                w_uvarint(w, ed);
                w_uvarint(w, run);
            } else {
                uint64_t code = ed + 2;
                for (size_t k = 0; k < run; k++) {
                    w_uvarint(w, code);
                }
            }
        }

        i += run;
        remaining -= run;
    }
}

/* 单列编码：mode 0=Delta 1=Delta2 2=Constant 3=Linear */
static void encode_column(writer_t *w, const int64_t *v, size_t n,
                          bool is_signed, uint8_t mode)
{
    if (is_signed) {
        w_uvarint(w, zigzag(v[0]));
    } else {
        w_uvarint(w, (uint64_t)v[0]);
    }

    if (mode == 2) {
        return;
    }
    if (mode == 3) {
        if (n > 1) {
            w_delta_code(w, v[1] - v[0]);
        }
        return;
    }
    if (n <= 1) {
        return;
    }

    int64_t d1[BMS_HIST_MAX_SAMPLES];
    size_t m = n - 1;
    for (size_t i = 0; i < m; i++) {
        d1[i] = v[i + 1] - v[i];
    }
    if (mode == 1) {
        int64_t d2[BMS_HIST_MAX_SAMPLES];
        d2[0] = d1[0];
        for (size_t i = 1; i < m; i++) {
            d2[i] = d1[i] - d1[i - 1];
        }
        encode_delta_sequence(w, d2, m);
        return;
    }
    encode_delta_sequence(w, d1, m);
}

static size_t column_length(const int64_t *v, size_t n, bool is_signed, uint8_t mode)
{
    writer_t w = {.buf = NULL, .cap = 0, .len = 0, .overflow = false};
    encode_column(&w, v, n, is_signed, mode);
    return w.len;
}

/* 与 Lua choose_column_mode 完全一致：仅当更短才换模式 */
static uint8_t choose_column_mode(const int64_t *v, size_t n, bool is_signed,
                                  size_t *best_len)
{
    uint8_t best_mode = 0;
    size_t best = column_length(v, n, is_signed, 0);

    bool is_const = true;
    for (size_t i = 1; i < n; i++) {
        if (v[i] != v[0]) {
            is_const = false;
            break;
        }
    }
    bool is_linear = true;
    if (n > 1) {
        int64_t prev = v[1] - v[0];
        for (size_t i = 2; i < n; i++) {
            int64_t d = v[i] - v[i - 1];
            if (d != prev) {
                is_linear = false;
                break;
            }
            prev = d;
        }
    }

    if (is_const) {
        size_t len = column_length(v, n, is_signed, 2);
        if (len < best) {
            best = len;
            best_mode = 2;
        }
    }
    if (is_linear) {
        size_t len = column_length(v, n, is_signed, 3);
        if (len < best) {
            best = len;
            best_mode = 3;
        }
    }
    size_t len = column_length(v, n, is_signed, 1);
    if (len < best) {
        best = len;
        best_mode = 1;
    }

    *best_len = best;
    return best_mode;
}

/* 取第 idx 列数据：0..8 为固定列，>=9 为电芯列 */
static size_t build_column(const bms_hist_t *h, size_t idx, bool spatial,
                          int64_t *out, bool *is_signed)
{
    size_t n = h->count;
    *is_signed = false;

    switch (idx) {
    case 0:
        *is_signed = true;
        for (size_t i = 0; i < n; i++) {
            out[i] = h->temp1[i];
        }
        break;
    case 1:
        *is_signed = true;
        for (size_t i = 0; i < n; i++) {
            out[i] = h->temp2[i];
        }
        break;
    case 2:
        *is_signed = true;
        for (size_t i = 0; i < n; i++) {
            out[i] = h->temp_mos[i];
        }
        break;
    case 3:
        for (size_t i = 0; i < n; i++) {
            out[i] = h->balan_current[i];
        }
        break;
    case 4:
        for (size_t i = 0; i < n; i++) {
            out[i] = h->bat_vol[i];
        }
        break;
    case 5:
        *is_signed = true;
        for (size_t i = 0; i < n; i++) {
            out[i] = h->bat_current[i];
        }
        break;
    case 6:
        for (size_t i = 0; i < n; i++) {
            out[i] = h->cycle_cap[i];
        }
        break;
    case 7:
        for (size_t i = 0; i < n; i++) {
            out[i] = h->cap_remain[i];
        }
        break;
    case 8:
        for (size_t i = 0; i < n; i++) {
            out[i] = h->time_s[i];
        }
        break;
    default: {
        size_t c = h->cell_count;
        size_t j = idx - 9;
        if (c == 0 || j >= c) {
            return 0;
        }
        for (size_t s = 0; s < n; s++) {
            const uint16_t *row = &h->cells[s * BMS_MAX_CELLS];
            if (j == 0) {
                out[s] = row[0];
            } else if (spatial) {
                out[s] = (int64_t)row[j] - (int64_t)row[j - 1];
                *is_signed = true;
            } else {
                out[s] = (int64_t)row[j] - (int64_t)row[0];
                *is_signed = true;
            }
        }
        break;
    }
    }
    return n;
}

/* 编码一种电芯布局；out 为 NULL 时只计算长度 */
static size_t encode_layout(const bms_hist_t *h, bool spatial,
                            uint8_t *out, size_t cap)
{
    size_t n = h->count;
    size_t c = h->cell_count;
    size_t cols = 9 + c;
    if (n == 0 || n > BMS_HIST_MAX_SAMPLES || cols > 9 + BMS_MAX_CELLS) {
        return 0;
    }

    uint8_t modes[9 + BMS_MAX_CELLS];
    size_t saved = 0;
    int64_t values[BMS_HIST_MAX_SAMPLES];

    for (size_t i = 0; i < cols; i++) {
        bool is_signed = false;
        size_t len = build_column(h, i, spatial, values, &is_signed);
        if (len == 0) {
            return 0;
        }
        size_t default_len = column_length(values, len, is_signed, 0);
        size_t best_len = 0;
        uint8_t mode = choose_column_mode(values, len, is_signed, &best_len);
        if (mode != 0 && best_len < default_len) {
            modes[i] = mode;
            saved += default_len - best_len;
        } else {
            modes[i] = 0;
        }
    }

    size_t mode_size = (cols * 2 + 7) / 8;
    uint64_t header_without = ((uint64_t)c << 2) | (spatial ? 2u : 0u);
    uint64_t header_with = header_without | 1u;
    bool use_modes =
        saved > mode_size + uvarint_len(header_with) - uvarint_len(header_without);

    writer_t w = {.buf = out, .cap = cap, .len = 0, .overflow = false};
    w_uvarint(&w, (uint64_t)(n - 1));
    w_uvarint(&w, use_modes ? header_with : header_without);

    if (use_modes) {
        uint8_t mode_table[(9 + BMS_MAX_CELLS) * 2 / 8 + 1];
        memset(mode_table, 0, sizeof(mode_table));
        for (size_t i = 0; i < cols; i++) {
            size_t bit = i * 2;
            mode_table[bit / 8] |= (uint8_t)(modes[i] << (bit % 8));
        }
        for (size_t i = 0; i < mode_size; i++) {
            w_byte(&w, mode_table[i]);
        }
    }

    for (size_t i = 0; i < cols; i++) {
        bool is_signed = false;
        size_t len = build_column(h, i, spatial, values, &is_signed);
        encode_column(&w, values, len, is_signed, use_modes ? modes[i] : 0);
    }

    if (w.overflow) {
        return 0;
    }
    return w.len;
}

void bms_hist_init(bms_hist_t *h, uint8_t batch_samples)
{
    memset(h, 0, sizeof(*h));
    if (batch_samples < 1) {
        batch_samples = 1;
    }
    if (batch_samples > BMS_HIST_MAX_SAMPLES) {
        batch_samples = BMS_HIST_MAX_SAMPLES;
    }
    h->batch_samples = batch_samples;
}

bool bms_hist_add(bms_hist_t *h, const bms_snapshot_t *snap, uint32_t unix_time)
{
    if (h->count >= h->batch_samples) {
        return true; /* 上一批还没被取走 */
    }

    uint8_t cells = snap->cell_count;
    if (cells > BMS_MAX_CELLS) {
        cells = BMS_MAX_CELLS;
    }
    if (h->cell_count == 0) {
        h->cell_count = cells; /* 首批确定电芯数，批内固定 */
    }
    uint8_t cc = h->cell_count;

    size_t i = h->count;
    h->temp1[i] = snap->temp1_tenths;
    h->temp2[i] = snap->temp2_tenths;
    h->temp_mos[i] = snap->board_temp_tenths;
    h->balan_current[i] = snap->balan_current_ma;
    h->bat_vol[i] = snap->total_voltage_mv;
    h->bat_current[i] = snap->charge_current_ma;
    h->cycle_cap[i] = snap->cycle_capacity_mah;
    h->cap_remain[i] = (uint32_t)(snap->capacity_remain_mah < 0 ? 0 : snap->capacity_remain_mah);
    h->time_s[i] = unix_time;

    uint16_t *row = &h->cells[i * BMS_MAX_CELLS];
    for (uint8_t j = 0; j < cc; j++) {
        row[j] = j < cells ? snap->cell_mv[j] : 0;
    }
    for (uint8_t j = cc; j < BMS_MAX_CELLS; j++) {
        row[j] = 0;
    }

    h->count++;
    h->total++;
    return h->count >= h->batch_samples;
}

size_t bms_hist_encode(const bms_hist_t *h, uint8_t *out, size_t cap)
{
    if (h->count == 0) {
        return 0;
    }
    size_t offset_len = encode_layout(h, false, out, cap);
    if (h->cell_count == 0) {
        return offset_len;
    }
    /* 单线程调用（上报任务），用静态暂存比较 spatial 布局 */
    static uint8_t alt[BMS_HIST_MAX_SIZE];
    size_t spatial_len = encode_layout(h, true, alt, sizeof(alt));
    if (spatial_len != 0 && (offset_len == 0 || spatial_len < offset_len)) {
        if (spatial_len <= cap) {
            memcpy(out, alt, spatial_len);
            offset_len = spatial_len;
        }
    }
    return offset_len;
}

void bms_hist_reset(bms_hist_t *h)
{
    h->count = 0;
    h->cell_count = 0;
}
