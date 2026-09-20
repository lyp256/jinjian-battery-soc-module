#include "jk_bms.h"

#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "led_ctrl.h"
#include "modbus_rtu.h"

static const char *TAG = "jk_bms";

/* 极空只读实时数据区 */
#define JK_REG_CELLS_BASE     0x1200
#define JK_REG_CELLS_NUM      32
#define JK_REG_CELL_STATS     0x1240
#define JK_REG_CELL_STATS_NUM 4
#define JK_REG_STATS_BASE     0x128A   /* TempMos 起，含 0x1290 汇总 */
#define JK_REG_STATS_NUM      58       /* 0x128A .. 0x12C3 */

/* 信息/版本区与配置区 */
#define JK_REG_INFO_BASE 0x1300
#define JK_REG_INFO_NUM  20
#define JK_REG_CELL_COUNT_CFG 0x106C
#define JK_REG_DESIGN_CAP_CFG 0x107C
#define JK_REG_RCV_TIME_CFG   0x1104

/* 0x128A 起的统计块索引 */
#define STAT_TEMP_MOS      0
#define STAT_BATVOL_HI     3
#define STAT_BATVOL_LO     4
#define STAT_CURRENT_HI    7
#define STAT_CURRENT_LO    8
#define STAT_TEMP_BAT1     9
#define STAT_TEMP_BAT2     10
#define STAT_ALARM_HI      11
#define STAT_ALARM_LO      12
#define STAT_BAL_SOC       14
#define STAT_CAP_REM_HI    15
#define STAT_CAP_REM_LO    16
#define STAT_FULL_CAP_HI   17
#define STAT_FULL_CAP_LO   18
#define STAT_CYCLE_HI      19
#define STAT_CYCLE_LO      20
#define STAT_SOH_PREC      23
#define STAT_USER_ALARM    24
#define STAT_RUNTIME_HI    25
#define STAT_RUNTIME_LO    26
#define STAT_CHARGE_STATE  27
#define STAT_USER_ALARM2   28

static jk_bms_config_t s_cfg;
static SemaphoreHandle_t s_mutex;
static bms_snapshot_t s_snapshot;

static uint16_t s_design_cell_count;
static uint32_t s_design_capacity_mah;
static uint32_t s_poll_count;
static bool s_charge_time_overridden;

static uint32_t u32_from(const uint16_t *r, size_t hi, size_t lo)
{
    return ((uint32_t)r[hi] << 16) | r[lo];
}

static int32_t s32_from(const uint16_t *r, size_t hi, size_t lo)
{
    return (int32_t)u32_from(r, hi, lo);
}

static int16_t tenths_to_c(int16_t v)
{
    if (v >= 0) {
        return (int16_t)((v + 5) / 10);
    }
    return (int16_t)(-((-v + 5) / 10));
}

static uint16_t clamp_u16(int32_t v)
{
    if (v < 0) {
        return 0;
    }
    if (v > 0xFFFF) {
        return 0xFFFF;
    }
    return (uint16_t)v;
}

static bool jk_read_regs(uint16_t addr, uint16_t num, uint16_t *regs, size_t max_regs)
{
    if (num == 0 || num > 125 || num > max_regs) {
        return false;
    }

    uint8_t frame[8];
    size_t frame_len = modbus_build_read_regs(frame, s_cfg.slave_addr, addr, num);
    uint8_t resp[256];
    size_t need = 5 + (size_t)num * 2;
    size_t got = 0;

    uart_flush_input(s_cfg.uart_num);
    int written = uart_write_bytes(s_cfg.uart_num, frame, frame_len);
    if (written != (int)frame_len) {
        ESP_LOGW(TAG, "JK write failed (%d)", written);
        return false;
    }

    while (got < need) {
        int n = uart_read_bytes(s_cfg.uart_num, resp + got,
                                (uint32_t)(need - got),
                                pdMS_TO_TICKS(s_cfg.response_timeout_ms));
        if (n <= 0) {
            break;
        }
        got += (size_t)n;
        if (got >= 3 && (resp[1] & 0x80)) {
            need = 5;
        }
        if (got >= need) {
            break;
        }
    }

    if (got < 5 || !modbus_check_crc(resp, got)) {
        return false;
    }
    if (resp[0] != s_cfg.slave_addr || resp[1] != MODBUS_FUNC_READ_REGS) {
        return false;
    }
    if (resp[2] != (uint8_t)(num * 2)) {
        return false;
    }
    for (size_t i = 0; i < num; i++) {
        regs[i] = ((uint16_t)resp[3 + i * 2] << 8) | resp[4 + i * 2];
    }
    return true;
}

static void parse_version(const uint8_t *text, size_t len, uint16_t out[2])
{
    uint8_t comp[4] = {0, 0, 0, 0};
    size_t idx = 0;
    uint16_t val = 0;
    bool have = false;

    for (size_t i = 0; i < len && idx < 4; i++) {
        if (text[i] >= '0' && text[i] <= '9') {
            val = val * 10 + (uint16_t)(text[i] - '0');
            if (val > 255) {
                val = 255;
            }
            have = true;
        } else if (have) {
            comp[idx++] = (uint8_t)val;
            val = 0;
            have = false;
        }
    }
    if (have && idx < 4) {
        comp[idx++] = (uint8_t)val;
    }
    if (idx == 0) {
        comp[0] = 1;
        comp[1] = 0;
        comp[2] = 0;
        comp[3] = 0;
    }
    out[0] = ((uint16_t)comp[0] << 8) | comp[1];
    out[1] = ((uint16_t)comp[2] << 8) | comp[3];
}

static void refresh_info(void)
{
    uint16_t regs[JK_REG_INFO_NUM];
    if (jk_read_regs(JK_REG_INFO_BASE, JK_REG_INFO_NUM, regs, JK_REG_INFO_NUM)) {
        char pn[17] = {0};
        for (size_t i = 0; i < 8; i++) {
            pn[i * 2] = (char)(regs[i] >> 8);
            pn[i * 2 + 1] = (char)(regs[i] & 0xFF);
        }
        for (size_t i = 0; i < 16; i++) {
            if (pn[i] < 0x20 || pn[i] > 0x7E) {
                pn[i] = '0';
            }
        }
        pn[16] = '\0';

        uint8_t sw[8];
        for (size_t i = 0; i < 4; i++) {
            sw[i * 2] = (uint8_t)(regs[12 + i] >> 8);
            sw[i * 2 + 1] = (uint8_t)(regs[12 + i] & 0xFF);
        }

        xSemaphoreTake(s_mutex, portMAX_DELAY);
        /* 模块自身 PN 由 MAC 派生，不覆盖为极空的 PN */
        parse_version(sw, sizeof(sw), s_snapshot.version_regs);
        xSemaphoreGive(s_mutex);
        ESP_LOGI(TAG, "JK info: pn=%s sw=%02X%02X/%02X%02X",
                 pn, sw[0], sw[1], sw[2], sw[3]);
    }

    uint16_t tmp[2];
    if (jk_read_regs(JK_REG_CELL_COUNT_CFG, 2, tmp, 2)) {
        s_design_cell_count = clamp_u16((int32_t)u32_from(tmp, 0, 1));
    }
    if (jk_read_regs(JK_REG_DESIGN_CAP_CFG, 2, tmp, 2)) {
        s_design_capacity_mah = u32_from(tmp, 0, 1);
    }
    if (!s_charge_time_overridden &&
        jk_read_regs(JK_REG_RCV_TIME_CFG, 1, tmp, 1)) {
        uint16_t rcv_tenth_h = (tmp[0] >> 8) & 0xFF;
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_snapshot.charge_time_min = (uint16_t)(rcv_tenth_h * 6U);
        xSemaphoreGive(s_mutex);
    }
}

static uint8_t count_cells(uint32_t cell_sta, const uint16_t *cells, size_t n)
{
    uint32_t mask = cell_sta & 0xFFFFFFFFu;
    int count = 0;
    while (mask) {
        mask &= mask - 1;
        count++;
    }
    if (count > 0 && count <= BMS_MAX_CELLS) {
        return (uint8_t)count;
    }
    count = 0;
    for (size_t i = 0; i < n; i++) {
        if (cells[i] > 0) {
            count++;
        }
    }
    if (count > 0) {
        return (uint8_t)count;
    }
    if (s_design_cell_count > 0 && s_design_cell_count <= BMS_MAX_CELLS) {
        return (uint8_t)s_design_cell_count;
    }
    return 0;
}

static uint16_t max_cell_diff(const uint16_t *cells, size_t n)
{
    uint16_t minv = 0xFFFF;
    uint16_t maxv = 0;
    for (size_t i = 0; i < n; i++) {
        if (cells[i] == 0) {
            continue;
        }
        if (cells[i] < minv) {
            minv = cells[i];
        }
        if (cells[i] > maxv) {
            maxv = cells[i];
        }
    }
    if (minv == 0xFFFF) {
        return 0;
    }
    return maxv - minv;
}

static bool bit_is_set(uint32_t v, unsigned bit)
{
    return ((v >> bit) & 1U) != 0;
}

static void apply_stats(const uint16_t *cells, size_t cell_n,
                        const uint16_t *cell_stats,
                        const uint16_t *stats, size_t stats_n)
{
    (void)stats_n;
    uint32_t cell_sta = u32_from(cell_stats, 0, 1);
    uint32_t bat_vol_mv = u32_from(stats, STAT_BATVOL_HI, STAT_BATVOL_LO);
    int32_t current_ma = s32_from(stats, STAT_CURRENT_HI, STAT_CURRENT_LO);
    int16_t temp1_raw = (int16_t)stats[STAT_TEMP_BAT1];
    int16_t temp2_raw = (int16_t)stats[STAT_TEMP_BAT2];
    int16_t temp_mos_raw = (int16_t)stats[STAT_TEMP_MOS];
    uint32_t alarm = u32_from(stats, STAT_ALARM_HI, STAT_ALARM_LO);
    uint32_t full_cap_mah = u32_from(stats, STAT_FULL_CAP_HI, STAT_FULL_CAP_LO);
    uint32_t cycle_count = u32_from(stats, STAT_CYCLE_HI, STAT_CYCLE_LO);
    uint8_t balance = (uint8_t)(stats[STAT_BAL_SOC] >> 8);
    uint8_t soc = (uint8_t)(stats[STAT_BAL_SOC] & 0xFF);
    uint8_t charge_state = (uint8_t)(stats[STAT_CHARGE_STATE] >> 8);
    uint8_t discharge_state = (uint8_t)(stats[STAT_CHARGE_STATE] & 0xFF);
    uint16_t user_alarm2 = stats[STAT_USER_ALARM2];

    (void)discharge_state;

    if (full_cap_mah == 0) {
        full_cap_mah = s_design_capacity_mah;
    }

    uint32_t capacity_ah32 = (full_cap_mah + 500) / 1000;
    uint16_t capacity_ah = capacity_ah32 > 0xFFFF ? 0xFFFF : (uint16_t)capacity_ah32;
    uint8_t cell_count = count_cells(cell_sta, cells, cell_n);
    if (cell_count > cell_n) {
        cell_count = (uint8_t)cell_n;
    }

    int32_t voltage_raw = (int32_t)(bat_vol_mv / 10U);
    int32_t current_raw = current_ma / 10;
    if (current_raw > 32767) {
        current_raw = 32767;
    } else if (current_raw < -32768) {
        current_raw = -32768;
    }

    uint16_t short_circuit = (bit_is_set(alarm, 7) || bit_is_set(alarm, 14)) ? 1 : 0;
    uint16_t over_temp_charge = bit_is_set(alarm, 8) ? 1 : 0;
    uint16_t over_temp_discharge = bit_is_set(alarm, 15) ? 1 : 0;
    uint16_t low_temp_charge = bit_is_set(alarm, 9) ? 1 : 0;
    /* 极空公开协议没有放电低温保护位；保留用户报警2低字节，方便后续扩展。 */
    uint16_t low_temp_discharge = (user_alarm2 & 0x0001) ? 1 : 0;

    int16_t temp1 = tenths_to_c(temp1_raw);
    int16_t temp2 = tenths_to_c(temp2_raw);
    if (!low_temp_charge &&
        (temp1 < s_cfg.low_temp_charge_c || temp2 < s_cfg.low_temp_charge_c)) {
        low_temp_charge = 1;
    }
    if (!low_temp_discharge &&
        (temp1 < s_cfg.low_temp_discharge_c || temp2 < s_cfg.low_temp_discharge_c)) {
        low_temp_discharge = 1;
    }

    uint16_t fast_charging =
        (charge_state != 0 && current_ma >= (int32_t)s_cfg.fast_charge_current_ma) ? 1 : 0;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_snapshot.have_data = true;
    s_snapshot.fresh = true;
    s_snapshot.poll_count++;
    s_snapshot.last_ok_ms = (uint32_t)(esp_timer_get_time() / 1000);
    s_snapshot.total_voltage_raw = clamp_u16(voltage_raw);
    s_snapshot.cell_count = cell_count;
    s_snapshot.soc = soc > 100 ? 100 : soc;
    s_snapshot.capacity_ah = capacity_ah;
    s_snapshot.charge_current_raw = (int16_t)current_raw;
    s_snapshot.temp1 = temp1;
    s_snapshot.temp2 = temp2;
    s_snapshot.board_temp = tenths_to_c(temp_mos_raw);
    memcpy(s_snapshot.cell_mv, cells, cell_n * sizeof(uint16_t));
    if (cell_n < BMS_MAX_CELLS) {
        memset(s_snapshot.cell_mv + cell_n, 0,
               (BMS_MAX_CELLS - cell_n) * sizeof(uint16_t));
    }
    s_snapshot.max_cell_diff_mv = max_cell_diff(cells, cell_n);
    s_snapshot.battery_type = 0;
    s_snapshot.cycle_count = cycle_count > 0xFFFF ? 0xFFFF : (uint16_t)cycle_count;
    s_snapshot.balance_status = balance != 0 ? 1 : 0;
    s_snapshot.nominal_capacity_ah = capacity_ah;
    s_snapshot.fast_charging = fast_charging;
    memcpy(s_snapshot.protection, (uint16_t[]){
               short_circuit, over_temp_charge, over_temp_discharge,
               low_temp_charge, low_temp_discharge},
           sizeof(s_snapshot.protection));
    xSemaphoreGive(s_mutex);
}

static void jk_task(void *arg)
{
    (void)arg;
    refresh_info();

    while (1) {
        uint16_t cells[JK_REG_CELLS_NUM];
        uint16_t cell_stats[JK_REG_CELL_STATS_NUM];
        uint16_t stats[JK_REG_STATS_NUM];

        bool ok_cells = jk_read_regs(JK_REG_CELLS_BASE, JK_REG_CELLS_NUM,
                                     cells, JK_REG_CELLS_NUM);
        bool ok_stats = jk_read_regs(JK_REG_CELL_STATS, JK_REG_CELL_STATS_NUM,
                                     cell_stats, JK_REG_CELL_STATS_NUM);
        bool ok_main = jk_read_regs(JK_REG_STATS_BASE, JK_REG_STATS_NUM,
                                    stats, JK_REG_STATS_NUM);

        s_poll_count++;
        if (s_poll_count % 20 == 0) {
            refresh_info();
        }

        bool ok_all = ok_cells && ok_stats && ok_main;
        if (ok_all) {
            apply_stats(cells, JK_REG_CELLS_NUM, cell_stats, stats, JK_REG_STATS_NUM);
            ESP_LOGD(TAG, "JK OK: %.2fV SOC=%u I=%dmA cells=%u",
                     (double)(s_snapshot.total_voltage_raw) / 100.0,
                     s_snapshot.soc, s_snapshot.charge_current_raw * 10,
                     s_snapshot.cell_count);
        } else {
            xSemaphoreTake(s_mutex, portMAX_DELAY);
            s_snapshot.fresh = false;
            s_snapshot.poll_failures++;
            xSemaphoreGive(s_mutex);
            ESP_LOGW(TAG, "JK poll failed: cells=%d stats=%d main=%d",
                     ok_cells, ok_stats, ok_main);
        }
        /* 绿色闪烁 = 读取 BMS；成功获取数据后单色 LED 再闪一次 */
        led_ctrl_notify_event(LED_EVENT_BMS, ok_all);

        if (s_poll_count % 20 == 0) {
            xSemaphoreTake(s_mutex, portMAX_DELAY);
            uint32_t ok = s_snapshot.poll_count;
            uint32_t fail = s_snapshot.poll_failures;
            uint32_t last_ms = s_snapshot.last_ok_ms;
            xSemaphoreGive(s_mutex);
            ESP_LOGI(TAG, "UART poll summary: ok=%lu fail=%lu lastOkMs=%lu",
                     (unsigned long)ok, (unsigned long)fail,
                     (unsigned long)last_ms);
        }

        vTaskDelay(pdMS_TO_TICKS(s_cfg.poll_interval_ms));
    }
}

int jk_bms_uart_init(void *config)
{
    const jk_bms_config_t *cfg = (const jk_bms_config_t *)config;
    s_cfg = *cfg;
    s_mutex = xSemaphoreCreateMutex();
    memset(&s_snapshot, 0, sizeof(s_snapshot));
    s_snapshot.charge_time_min = 60;
    s_snapshot.charge_target_soc = cfg->charge_target_soc;
    s_snapshot.version_regs[0] = 0x0100;
    bms_mac_pn(s_snapshot.pn, sizeof(s_snapshot.pn));
    s_charge_time_overridden = false;

    uart_config_t uart_cfg = {
        .baud_rate = cfg->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(cfg->uart_num, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(cfg->uart_num, cfg->tx_gpio, cfg->rx_gpio, -1, -1));
    ESP_ERROR_CHECK(uart_driver_install(cfg->uart_num, 1024, 0, 0, NULL, 0));

    xTaskCreate(jk_task, "jk_bms", 4096, NULL, 5, NULL);
    return 0;
}

void jk_bms_uart_deinit(void)
{
    /* 保持简单：UART 驱动启动后常驻；切换驱动时由上层决定是否重启系统。 */
}

bool jk_bms_uart_get_snapshot(bms_snapshot_t *out)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *out = s_snapshot;
    xSemaphoreGive(s_mutex);
    return true;
}

void jk_bms_uart_set_charge_time_min(uint16_t minutes)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_snapshot.charge_time_min = minutes > 120 ? 120 : minutes;
    xSemaphoreGive(s_mutex);
    s_charge_time_overridden = true;
}

void jk_bms_uart_set_charge_target_soc(uint16_t soc)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_snapshot.charge_target_soc = soc > 100 ? 100 : soc;
    xSemaphoreGive(s_mutex);
}

void jk_bms_uart_end_fast_charge(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_snapshot.fast_charging = 0;
    xSemaphoreGive(s_mutex);
}
