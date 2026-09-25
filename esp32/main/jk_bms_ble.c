#include "jk_bms_ble.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatt_defs.h"
#include "esp_gattc_api.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "led_ctrl.h"

static const char *TAG = "jk_ble";

#define JK_BLE_SERVICE_UUID 0xFFE0
#define JK_BLE_CHAR_UUID    0xFFE1
#define JK_BLE_APP_ID       0
#define JK_FRAME_SIZE       300
#define JK_FRAME_MAX        320

#define CMD_DEVICE_INFO 0x97
#define CMD_CELL_INFO   0x96

#define WEB_SCAN_MAX_DEVICES 24
#define WEB_SCAN_NAME_LEN    33
#define WEB_SCAN_MAC_LEN     19

typedef struct {
    bool used;
    char name[WEB_SCAN_NAME_LEN];
    char mac[WEB_SCAN_MAC_LEN];
} web_scan_dev_t;

static jk_bms_ble_config_t s_cfg;
static SemaphoreHandle_t s_mutex;
static bms_snapshot_t s_snapshot;
static TaskHandle_t s_task;
static bool s_bt_ready;
static bool s_stop_requested;

/* GATT / 扫描状态 */
static esp_gatt_if_t s_gattc_if = ESP_GATT_IF_NONE;
static uint16_t s_conn_id;
static uint16_t s_service_start;
static uint16_t s_service_end;
static uint16_t s_char_handle;
static uint16_t s_notify_handle;
static bool s_connected;
static bool s_notify_enabled;
static bool s_scanning;
static bool s_target_found;
static esp_bd_addr_t s_target_bda;

static uint8_t s_frame_buf[JK_FRAME_MAX];
static size_t s_frame_len;

static uint64_t s_last_scan_ms;
static uint64_t s_last_request_ms;

static web_scan_dev_t s_web_scan_devs[WEB_SCAN_MAX_DEVICES];
static size_t s_web_scan_count;
static bool s_web_scan_active;
static uint64_t s_web_scan_deadline_ms;

static esp_bt_uuid_t s_svc_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = JK_BLE_SERVICE_UUID},
};
static esp_bt_uuid_t s_char_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = JK_BLE_CHAR_UUID},
};
static esp_bt_uuid_t s_cccd_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG},
};

static uint64_t now_ms(void)
{
    return (uint64_t)(esp_timer_get_time() / 1000);
}

static uint16_t u16le(const uint8_t *d, size_t i)
{
    return (uint16_t)((uint16_t)d[i] | ((uint16_t)d[i + 1] << 8));
}

static uint32_t u32le(const uint8_t *d, size_t i)
{
    return (uint32_t)u16le(d, i) | ((uint32_t)u16le(d, i + 2) << 16);
}

static int32_t s32le(const uint8_t *d, size_t i)
{
    return (int32_t)u32le(d, i);
}

static int16_t tenths_to_c(int16_t v)
{
    if (v >= 0) {
        return (int16_t)((v + 5) / 10);
    }
    return (int16_t)(-((-v + 5) / 10));
}

static uint16_t clamp_u16(uint32_t v)
{
    return v > 0xFFFF ? 0xFFFF : (uint16_t)v;
}

static bool bit_set(uint32_t v, unsigned b)
{
    return ((v >> b) & 1U) != 0;
}

static bool parse_mac(const char *s, esp_bd_addr_t out)
{
    unsigned b[6];
    if (sscanf(s, "%02x:%02x:%02x:%02x:%02x:%02x",
               &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6) {
        return false;
    }
    for (int i = 0; i < 6; i++) {
        out[i] = (uint8_t)b[i];
    }
    return true;
}

static bool addr_matches(const esp_bd_addr_t a, const esp_bd_addr_t b)
{
    return memcmp(a, b, ESP_BD_ADDR_LEN) == 0;
}

static bool contains_ci(const char *hay, const char *needle)
{
    if (hay == NULL || needle == NULL || *needle == '\0') {
        return true;
    }
    size_t nlen = strlen(needle);
    for (const char *p = hay; *p; p++) {
        size_t i = 0;
        while (i < nlen && tolower((unsigned char)p[i]) == tolower((unsigned char)needle[i])) {
            i++;
        }
        if (i == nlen) {
            return true;
        }
    }
    return false;
}

static bool device_matches(const esp_bd_addr_t bda, const uint8_t *name, uint8_t name_len)
{
    if (s_cfg.target_addr[0] != '\0') {
        esp_bd_addr_t wanted;
        return parse_mac(s_cfg.target_addr, wanted) && addr_matches(bda, wanted);
    }
    if (s_cfg.target_name[0] != '\0') {
        char buf[33];
        size_t n = name_len < sizeof(buf) - 1 ? name_len : sizeof(buf) - 1;
        memcpy(buf, name, n);
        buf[n] = '\0';
        return contains_ci(buf, s_cfg.target_name);
    }
    return true; /* 未配置过滤条件时连接第一个扫描到的设备 */
}

static void build_command(uint8_t *frame, uint8_t command)
{
    memset(frame, 0, 20);
    frame[0] = 0xAA;
    frame[1] = 0x55;
    frame[2] = 0x90;
    frame[3] = 0xEB;
    frame[4] = command;
    frame[5] = 0x00;
    uint8_t sum = 0;
    for (int i = 0; i < 19; i++) {
        sum = (uint8_t)(sum + frame[i]);
    }
    frame[19] = sum;
}

static void ble_send_cmd(uint8_t command)
{
    if (s_gattc_if == ESP_GATT_IF_NONE || s_char_handle == 0 || !s_connected) {
        return;
    }
    uint8_t frame[20];
    build_command(frame, command);
    esp_ble_gattc_write_char(s_gattc_if, s_conn_id, s_char_handle,
                             sizeof(frame), frame,
                             ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE);
}

static void reset_link_state(void)
{
    s_connected = false;
    s_notify_enabled = false;
    s_char_handle = 0;
    s_notify_handle = 0;
    s_service_start = 0;
    s_service_end = 0;
    s_frame_len = 0;
    s_target_found = false;
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

static void text_to_pn(const uint8_t *src, size_t len, char *pn, size_t pn_cap)
{
    memset(pn, '0', pn_cap - 1);
    size_t n = len < pn_cap - 1 ? len : pn_cap - 1;
    for (size_t i = 0; i < n; i++) {
        if (src[i] >= 0x20 && src[i] <= 0x7E) {
            pn[i] = (char)src[i];
        }
    }
    pn[pn_cap - 1] = '\0';
}

static void decode_device_info(const uint8_t *d, size_t len)
{
    if (len < JK_FRAME_SIZE) {
        return;
    }
    char model[17], sw[9];
    text_to_pn(d + 6, 16, model, sizeof(model));
    text_to_pn(d + 30, 8, sw, sizeof(sw));

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    /* 模块自身 PN 由 MAC 派生，不覆盖为极空的型号 */
    parse_version(d + 30, 8, s_snapshot.version_regs);
    xSemaphoreGive(s_mutex);
    ESP_LOGI(TAG, "JK BLE device: model=%s sw=%s", model, sw);
}

static uint16_t max_diff_mv(const uint16_t *cells, size_t n)
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
    return minv == 0xFFFF ? 0 : maxv - minv;
}

static void decode_jk02_cell_info(const uint8_t *d, size_t len)
{
    if (len < JK_FRAME_SIZE) {
        return;
    }
    int is_32s = (s_cfg.protocol_version == JK_BLE_PROTOCOL_32S);
    int shift = is_32s ? 32 : 0;
    int cells = is_32s ? 32 : 24;

    uint16_t cell_mv[BMS_MAX_CELLS] = {0};
    uint8_t count = 0;
    for (int i = 0; i < cells && i < BMS_MAX_CELLS; i++) {
        cell_mv[i] = u16le(d, 6 + (size_t)i * 2);
        if (cell_mv[i] > 0) {
            count++;
        }
    }
    if (count == 0) {
        return;
    }

    uint32_t total_mv = u32le(d, 118 + shift);
    int32_t current_ma = s32le(d, 126 + shift);
    int16_t temp1_raw = (int16_t)u16le(d, 130 + shift);
    int16_t temp2_raw = (int16_t)u16le(d, 132 + shift);
    int16_t temp1 = tenths_to_c(temp1_raw);
    int16_t temp2 = tenths_to_c(temp2_raw);
    int16_t mos_raw = is_32s ? (int16_t)u16le(d, 112 + shift)
                             : (int16_t)u16le(d, 134 + shift);
    int16_t board_temp = tenths_to_c(mos_raw);
    uint32_t alarm = is_32s ? u32le(d, 134 + shift) : u16le(d, 136 + shift);

    int16_t balan_current_ma = (int16_t)u16le(d, 138 + shift);
    uint8_t balance = d[140 + shift];
    uint8_t soc = d[141 + shift];
    uint32_t cap_remain_mah = u32le(d, 142 + shift);
    uint32_t full_cap_mah = u32le(d, 146 + shift);
    uint32_t cycles = u32le(d, 150 + shift);
    uint32_t cycle_cap_mah = u32le(d, 154 + shift);
    uint8_t soh = d[158 + shift];
    uint8_t charge_state = d[166 + shift];
    uint8_t discharge_state = d[167 + shift];
    uint16_t battery_type = is_32s ? d[243 + shift] : 0;

    (void)soh;
    (void)discharge_state;

    uint16_t short_circuit = (bit_set(alarm, 7) || bit_set(alarm, 14)) ? 1 : 0;
    uint16_t over_temp_charge = bit_set(alarm, 8) ? 1 : 0;
    uint16_t over_temp_discharge = bit_set(alarm, 15) ? 1 : 0;
    uint16_t low_temp_charge = bit_set(alarm, 9) ? 1 : 0;
    uint16_t low_temp_discharge = 0;
    if (!low_temp_charge &&
        (temp1 < s_cfg.low_temp_charge_c || temp2 < s_cfg.low_temp_charge_c)) {
        low_temp_charge = 1;
    }
    if (temp1 < s_cfg.low_temp_discharge_c || temp2 < s_cfg.low_temp_discharge_c) {
        low_temp_discharge = 1;
    }

    uint16_t fast_charging =
        (charge_state != 0 && current_ma >= (int32_t)s_cfg.fast_charge_current_ma) ? 1 : 0;
    uint16_t capacity_ah = full_cap_mah ? clamp_u16((full_cap_mah + 500) / 1000) : 0;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_snapshot.have_data = true;
    s_snapshot.fresh = true;
    s_snapshot.poll_count++;
    s_snapshot.last_ok_ms = (uint32_t)(esp_timer_get_time() / 1000);
    s_snapshot.total_voltage_raw = clamp_u16(total_mv / 10);
    s_snapshot.total_voltage_mv = total_mv;
    s_snapshot.cell_count = count;
    s_snapshot.soc = soc > 100 ? 100 : soc;
    s_snapshot.capacity_ah = capacity_ah;
    if (current_ma > 32767) {
        current_ma = 32767;
    } else if (current_ma < -32768) {
        current_ma = -32768;
    }
    s_snapshot.charge_current_raw = (int16_t)(current_ma / 10);
    s_snapshot.charge_current_ma = current_ma;
    s_snapshot.temp1 = temp1;
    s_snapshot.temp2 = temp2;
    s_snapshot.board_temp = board_temp;
    s_snapshot.temp1_tenths = temp1_raw;
    s_snapshot.temp2_tenths = temp2_raw;
    s_snapshot.board_temp_tenths = mos_raw;
    s_snapshot.balan_current_ma = (uint16_t)balan_current_ma;
    s_snapshot.capacity_remain_mah = (int32_t)cap_remain_mah;
    s_snapshot.cycle_capacity_mah = cycle_cap_mah;
    memcpy(s_snapshot.cell_mv, cell_mv, sizeof(cell_mv));
    s_snapshot.max_cell_diff_mv = max_diff_mv(cell_mv, BMS_MAX_CELLS);
    s_snapshot.battery_type = battery_type;
    s_snapshot.cycle_count = clamp_u16(cycles);
    s_snapshot.balance_status = balance != 0 ? 1 : 0;
    s_snapshot.nominal_capacity_ah = capacity_ah;
    s_snapshot.fast_charging = fast_charging;
    memcpy(s_snapshot.protection, (uint16_t[]){
               short_circuit, over_temp_charge, over_temp_discharge,
               low_temp_charge, low_temp_discharge},
           sizeof(s_snapshot.protection));
    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "JK BLE cell info: %.2fV SOC=%u%% I=%dmA cells=%u",
             (double)(total_mv / 10) / 100.0, soc, current_ma, count);
}

static void decode_frame(const uint8_t *d, size_t len)
{
    if (len < 5) {
        return;
    }
    switch (d[4]) {
    case 0x02:
        decode_jk02_cell_info(d, len);
        break;
    case 0x03:
        decode_device_info(d, len);
        break;
    default:
        ESP_LOGD(TAG, "JK frame type 0x%02X ignored", d[4]);
        break;
    }
}

static void assemble_notify(const uint8_t *data, uint16_t len)
{
    const uint8_t preamble[4] = {0x55, 0xAA, 0xEB, 0x90};
    if (len >= 4 && memcmp(data, preamble, 4) == 0) {
        s_frame_len = 0;
    }
    if (s_frame_len + len > sizeof(s_frame_buf)) {
        s_frame_len = 0;
        return;
    }
    memcpy(s_frame_buf + s_frame_len, data, len);
    s_frame_len += len;
    if (s_frame_len < JK_FRAME_SIZE) {
        return;
    }

    uint8_t sum = 0;
    for (size_t i = 0; i < JK_FRAME_SIZE - 1; i++) {
        sum = (uint8_t)(sum + s_frame_buf[i]);
    }
    if (sum != s_frame_buf[JK_FRAME_SIZE - 1]) {
        ESP_LOGW(TAG, "JK BLE CRC failed: 0x%02X != 0x%02X",
                 sum, s_frame_buf[JK_FRAME_SIZE - 1]);
        s_frame_len = 0;
        return;
    }

    decode_frame(s_frame_buf, JK_FRAME_SIZE);
    s_frame_len = 0;
}

static void record_scan_device(const esp_bd_addr_t bda, const uint8_t *name,
                               uint8_t name_len)
{
    char mac[WEB_SCAN_MAC_LEN];
    snprintf(mac, sizeof(mac), "%02X:%02X:%02X:%02X:%02X:%02X",
             bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (size_t i = 0; i < s_web_scan_count; i++) {
        if (strcmp(s_web_scan_devs[i].mac, mac) == 0) {
            xSemaphoreGive(s_mutex);
            return;
        }
    }
    if (s_web_scan_count < WEB_SCAN_MAX_DEVICES) {
        web_scan_dev_t *d = &s_web_scan_devs[s_web_scan_count++];
        d->used = true;
        snprintf(d->mac, sizeof(d->mac), "%s", mac);
        size_t n = name_len < sizeof(d->name) - 1 ? name_len : sizeof(d->name) - 1;
        memcpy(d->name, name, n);
        d->name[n] = '\0';
        if (n == 0) {
            snprintf(d->name, sizeof(d->name), "(unknown)");
        }
    }
    xSemaphoreGive(s_mutex);
}

static void start_scan(void)
{
    if (s_scanning || s_target_found || s_web_scan_active) {
        return;
    }
    esp_ble_scan_params_t params = {
        .scan_type = BLE_SCAN_TYPE_ACTIVE,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval = 0x50,
        .scan_window = 0x30,
        .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE,
    };
    esp_ble_gap_set_scan_params(&params);
}

static void gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        uint32_t duration = 0;
        if (s_web_scan_active) {
            uint64_t remain = s_web_scan_deadline_ms > now_ms()
                                  ? s_web_scan_deadline_ms - now_ms()
                                  : 0;
            duration = (uint32_t)((remain + 999) / 1000);
            if (duration == 0) {
                duration = 1;
            }
        }
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        if (param->scan_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            s_scanning = true;
            s_last_scan_ms = now_ms();
        }
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
        if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_CMPL_EVT) {
            s_scanning = false;
            if (s_web_scan_active) {
                s_web_scan_active = false;
                ESP_LOGI(TAG, "web scan finished: %u devices",
                         (unsigned)s_web_scan_count);
            }
            break;
        }
        if (param->scan_rst.search_evt != ESP_GAP_SEARCH_INQ_RES_EVT) {
            break;
        }
        if (s_web_scan_active) {
            uint8_t name_len = 0;
            uint8_t *name = esp_ble_resolve_adv_data_by_type(
                param->scan_rst.ble_adv,
                param->scan_rst.adv_data_len + param->scan_rst.scan_rsp_len,
                ESP_BLE_AD_TYPE_NAME_CMPL, &name_len);
            record_scan_device(param->scan_rst.bda, name, name_len);
            break;
        }
        if (s_target_found) {
            break;
        }
        {
            uint8_t name_len = 0;
            uint8_t *name = esp_ble_resolve_adv_data_by_type(
                param->scan_rst.ble_adv,
                param->scan_rst.adv_data_len + param->scan_rst.scan_rsp_len,
                ESP_BLE_AD_TYPE_NAME_CMPL, &name_len);
            if (device_matches(param->scan_rst.bda, name, name_len)) {
                ESP_LOGI(TAG, "JK BMS found, connecting");
                memcpy(s_target_bda, param->scan_rst.bda, ESP_BD_ADDR_LEN);
                s_target_found = true;
                esp_ble_gap_stop_scanning();
                esp_ble_gatt_creat_conn_params_t cp = {0};
                memcpy(cp.remote_bda, param->scan_rst.bda, ESP_BD_ADDR_LEN);
                cp.remote_addr_type = param->scan_rst.ble_addr_type;
                cp.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
                cp.is_direct = true;
                cp.is_aux = false;
                cp.phy_mask = 0x0;
                if (s_gattc_if != ESP_GATT_IF_NONE) {
                    esp_ble_gattc_enh_open(s_gattc_if, &cp);
                }
            }
        }
        break;
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        s_scanning = false;
        if (s_web_scan_active) {
            s_web_scan_active = false;
        }
        break;
    default:
        break;
    }
}

static void gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                     esp_ble_gattc_cb_param_t *param)
{
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "GATTC register failed: %d", param->reg.status);
            return;
        }
        s_gattc_if = gattc_if;
        start_scan();
        return;
    }
    if (gattc_if != s_gattc_if) {
        return;
    }

    switch (event) {
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK) {
            ESP_LOGW(TAG, "BLE open failed: %d", param->open.status);
            s_connected = false;
            s_target_found = false;
            break;
        }
        s_conn_id = param->open.conn_id;
        s_connected = true;
        ESP_LOGI(TAG, "BLE connected, conn_id=%u", param->open.conn_id);
        esp_ble_gattc_send_mtu_req(s_gattc_if, s_conn_id);
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        ESP_LOGD(TAG, "MTU %u", param->cfg_mtu.mtu);
        break;
    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
        if (param->dis_srvc_cmpl.status == ESP_GATT_OK) {
            esp_ble_gattc_search_service(s_gattc_if, s_conn_id, &s_svc_uuid);
        }
        break;
    case ESP_GATTC_SEARCH_RES_EVT:
        if (param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 &&
            param->search_res.srvc_id.uuid.uuid.uuid16 == JK_BLE_SERVICE_UUID) {
            s_service_start = param->search_res.start_handle;
            s_service_end = param->search_res.end_handle;
        }
        break;
    case ESP_GATTC_SEARCH_CMPL_EVT: {
        if (param->search_cmpl.status != ESP_GATT_OK || s_service_start == 0) {
            ESP_LOGW(TAG, "JK BLE service not found");
            break;
        }
        uint16_t count = 0;
        if (esp_ble_gattc_get_attr_count(s_gattc_if, s_conn_id, ESP_GATT_DB_CHARACTERISTIC,
                                         s_service_start, s_service_end, 0, &count) != ESP_GATT_OK ||
            count == 0) {
            break;
        }
        esp_gattc_char_elem_t *chars = calloc(count, sizeof(esp_gattc_char_elem_t));
        if (chars == NULL) {
            break;
        }
        if (esp_ble_gattc_get_char_by_uuid(s_gattc_if, s_conn_id, s_service_start,
                                           s_service_end, s_char_uuid, chars, &count) == ESP_GATT_OK) {
            for (uint16_t i = 0; i < count; i++) {
                if ((chars[i].properties & ESP_GATT_CHAR_PROP_BIT_WRITE_NR) ||
                    (chars[i].properties & ESP_GATT_CHAR_PROP_BIT_WRITE)) {
                    s_char_handle = chars[i].char_handle;
                }
                if (chars[i].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY) {
                    s_notify_handle = chars[i].char_handle;
                }
            }
        }
        free(chars);
        if (s_notify_handle != 0) {
            ESP_LOGI(TAG, "BLE char handle=%u notify=%u", s_char_handle, s_notify_handle);
            esp_ble_gattc_register_for_notify(s_gattc_if, s_target_bda,
                                              s_notify_handle);
        }
        break;
    }
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        if (param->reg_for_notify.status != ESP_GATT_OK) {
            break;
        }
        uint16_t count = 0;
        esp_ble_gattc_get_attr_count(s_gattc_if, s_conn_id, ESP_GATT_DB_DESCRIPTOR,
                                     s_service_start, s_service_end, s_notify_handle, &count);
        if (count > 0) {
            esp_gattc_descr_elem_t *descs = calloc(count, sizeof(esp_gattc_descr_elem_t));
            if (descs != NULL) {
                if (esp_ble_gattc_get_descr_by_char_handle(s_gattc_if, s_conn_id,
                                                           s_notify_handle, s_cccd_uuid,
                                                           descs, &count) == ESP_GATT_OK && count > 0) {
                    uint16_t enable = 0x0001;
                    esp_ble_gattc_write_char_descr(s_gattc_if, s_conn_id, descs[0].handle,
                                                   sizeof(enable), (uint8_t *)&enable,
                                                   ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
                } else {
                    s_notify_enabled = true;
                    s_last_request_ms = 0;
                    ESP_LOGI(TAG, "BLE notify enabled");
                }
                free(descs);
            }
        } else {
            s_notify_enabled = true;
            s_last_request_ms = 0;
            ESP_LOGI(TAG, "BLE notify enabled (no CCCD)");
        }
        break;
    }
    case ESP_GATTC_WRITE_DESCR_EVT:
        if (param->write.status == ESP_GATT_OK) {
            s_notify_enabled = true;
            s_last_request_ms = 0;
            ESP_LOGI(TAG, "BLE notify enabled via CCCD");
        }
        break;
    case ESP_GATTC_NOTIFY_EVT:
        if (param->notify.handle == s_notify_handle) {
            assemble_notify(param->notify.value, param->notify.value_len);
            led_ctrl_notify_event(LED_EVENT_BMS, true);
        }
        break;
    case ESP_GATTC_DISCONNECT_EVT:
        ESP_LOGW(TAG, "JK BLE disconnected, reason=0x%02x", param->disconnect.reason);
        led_ctrl_notify_event(LED_EVENT_BMS, false);
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_snapshot.fresh = false;
        s_snapshot.poll_failures++;
        xSemaphoreGive(s_mutex);
        reset_link_state();
        break;
    default:
        break;
    }
}

static void jk_ble_task(void *arg)
{
    (void)arg;
    while (!s_stop_requested) {
        if (s_web_scan_active) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        uint64_t now = now_ms();
        if (!s_connected && !s_scanning && s_gattc_if != ESP_GATT_IF_NONE &&
            now - s_last_scan_ms >= s_cfg.reconnect_interval_ms) {
            start_scan();
        }
        if (s_scanning && s_cfg.scan_timeout_ms > 0 &&
            now - s_last_scan_ms >= s_cfg.scan_timeout_ms) {
            esp_ble_gap_stop_scanning();
        }
        if (s_connected && s_notify_enabled &&
            now - s_last_request_ms >= s_cfg.poll_interval_ms) {
            s_last_request_ms = now;
            ble_send_cmd(CMD_CELL_INFO);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelete(NULL);
}

static int jk_ble_stack_init(void)
{
    if (s_bt_ready) {
        return 0;
    }
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bluedroid_init_with_cfg(&bluedroid_cfg));
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_cb));
    ESP_ERROR_CHECK(esp_ble_gattc_register_callback(gattc_cb));
    ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(512));
    ESP_ERROR_CHECK(esp_ble_gattc_app_register(JK_BLE_APP_ID));
    s_bt_ready = true;
    return 0;
}

int jk_bms_ble_init(void *config)
{
    const jk_bms_ble_config_t *cfg = (const jk_bms_ble_config_t *)config;
    s_cfg = *cfg;
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateMutex();
    }
    memset(&s_snapshot, 0, sizeof(s_snapshot));
    s_snapshot.charge_time_min = 60;
    s_snapshot.charge_target_soc = cfg->charge_target_soc;
    s_snapshot.version_regs[0] = 0x0100;
    bms_mac_pn(s_snapshot.pn, sizeof(s_snapshot.pn));
    s_stop_requested = false;

    jk_ble_stack_init();

    reset_link_state();
    s_last_scan_ms = 0;
    s_last_request_ms = 0;
    if (s_task == NULL) {
        xTaskCreate(jk_ble_task, "jk_ble", 4096, NULL, 5, &s_task);
    }
    return 0;
}

void jk_bms_ble_deinit(void)
{
    /* 蓝牙控制器保持运行，避免与 WiFi 的时序冲突；切换驱动时由上层重启系统。 */
}

bool jk_bms_ble_get_snapshot(bms_snapshot_t *out)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *out = s_snapshot;
    xSemaphoreGive(s_mutex);
    return true;
}

void jk_bms_ble_set_charge_time_min(uint16_t minutes)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_snapshot.charge_time_min = minutes > 120 ? 120 : minutes;
    xSemaphoreGive(s_mutex);
}

void jk_bms_ble_set_charge_target_soc(uint16_t soc)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_snapshot.charge_target_soc = soc > 100 ? 100 : soc;
    xSemaphoreGive(s_mutex);
}

void jk_bms_ble_end_fast_charge(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_snapshot.fast_charging = 0;
    xSemaphoreGive(s_mutex);
}

int jk_bms_ble_web_scan_start(uint32_t duration_ms)
{
    if (duration_ms < 1000) {
        duration_ms = 3000;
    }
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateMutex();
    }
    jk_ble_stack_init();

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_web_scan_count = 0;
    s_web_scan_active = true;
    s_web_scan_deadline_ms = now_ms() + duration_ms;
    xSemaphoreGive(s_mutex);
    ESP_LOGI(TAG, "web scan start: %u ms", (unsigned)duration_ms);

    if (s_scanning) {
        esp_ble_gap_stop_scanning();
    }
    esp_ble_scan_params_t params = {
        .scan_type = BLE_SCAN_TYPE_ACTIVE,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval = 0x50,
        .scan_window = 0x30,
        .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE,
    };
    esp_ble_gap_set_scan_params(&params);
    return 0;
}

bool jk_bms_ble_web_scan_active(void)
{
    bool active = false;
    if (s_mutex != NULL) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        active = s_web_scan_active;
        xSemaphoreGive(s_mutex);
    }
    return active;
}

size_t jk_bms_ble_web_scan_count(void)
{
    size_t count = 0;
    if (s_mutex != NULL) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        count = s_web_scan_count;
        xSemaphoreGive(s_mutex);
    }
    return count;
}

int jk_bms_ble_web_scan_get(size_t index, char *name, size_t name_cap,
                            char *mac, size_t mac_cap)
{
    if (s_mutex == NULL) {
        return -1;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (index >= s_web_scan_count) {
        xSemaphoreGive(s_mutex);
        return -1;
    }
    snprintf(name, name_cap, "%s", s_web_scan_devs[index].name);
    snprintf(mac, mac_cap, "%s", s_web_scan_devs[index].mac);
    xSemaphoreGive(s_mutex);
    return 0;
}
