#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "bms_uplink.h"
#include "bms_interface.h"
#include "bms_uplink.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "factory_reset.h"
#include "nvs_flash.h"
#include "jinjian_bms.h"
#include "jk_bms_ble.h"
#include "jk_bms.h"
#include "led_ctrl.h"
#include "ml307_4g.h"
#include "power_mgr.h"
#include "json_util.h"
#include "rpc_server.h"
#include "sdkconfig.h"
#include "web_server.h"
#include "log_stream.h"

static const char *TAG = "app";

static jk_bms_config_t s_jk_cfg;
static jk_bms_ble_config_t s_ble_cfg;
static ml307_4g_config_t s_lte_cfg;
static bms_uplink_config_t s_uplink_cfg;
static app_config_t s_app_cfg;

/* ---------------- RPC（MQTT + Web 共用）依赖 ---------------- */

static void rpc_reboot_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(300));
    esp_restart();
}

static void rpc_reboot_soon(void)
{
    xTaskCreate(rpc_reboot_task, "rpc_reboot", 2048, NULL, 5, NULL);
}

static void rpc_get_info(rpc_device_info_t *out)
{
    memset(out, 0, sizeof(*out));
    bms_uplink_status_t up;
    bms_uplink_get_status(&up);
    snprintf(out->device_id, sizeof(out->device_id), "%s",
             up.device_id[0] ? up.device_id : "unknown");
    snprintf(out->version, sizeof(out->version), "%s", "1.0.0");
    snprintf(out->mode, sizeof(out->mode), "%s", bms_manager_name());
    out->uptime_s = (uint32_t)(esp_timer_get_time() / 1000000);
    uint32_t epoch = 0;
    if (ml307_4g_get_epoch(&epoch)) {
        out->epoch = epoch;
    }
    out->mqtt_connected = up.mqtt_connected;
}

static int rpc_get_config_json(char *out, size_t cap)
{
    app_config_t c;
    app_config_get(&c);
    bms_uplink_status_t up;
    bms_uplink_get_status(&up);
    ml307_4g_status_t lte;
    ml307_4g_get_status(&lte);

    sbuf_t sb;
    sb_init(&sb, out, cap);
    sb_puts(&sb, "{\"transport\":");
    sb_json_quoted(&sb, c.bms_transport);
    sb_printf(&sb, ",\"jkPollMs\":%u,\"uartPollMs\":%u,\"bleName\":",
              (unsigned)c.jk_uart_poll_ms, (unsigned)c.jk_uart_poll_ms);
    sb_json_quoted(&sb, c.ble_target_name);
    sb_puts(&sb, ",\"bleAddr\":");
    sb_json_quoted(&sb, c.ble_target_addr);
    sb_printf(&sb, ",\"bleProtocol\":%d,\"lteEnable\":%s,\"lteApn\":",
              c.ble_protocol, c.lte_enable ? "true" : "false");
    sb_json_quoted(&sb, c.lte_apn);
    sb_printf(&sb, ",\"mqttEnable\":%s,\"mqttHost\":",
              c.mqtt_enable ? "true" : "false");
    sb_json_quoted(&sb, c.mqtt_host);
    sb_printf(&sb, ",\"mqttPort\":%u,\"mqttUser\":", (unsigned)c.mqtt_port);
    sb_json_quoted(&sb, c.mqtt_user);
    sb_puts(&sb, ",\"mqttPrefix\":");
    sb_json_quoted(&sb, c.mqtt_prefix);
    sb_printf(&sb, ",\"sampleIntervalMs\":%u,\"batchSamples\":%u,\"keepaliveS\":%u,"
                   "\"logLevel\":%d,\"ledEnable\":%s,\"deviceId\":",
              (unsigned)c.mqtt_sample_ms, (unsigned)c.mqtt_batch,
              (unsigned)c.mqtt_keepalive_s, c.log_level,
              c.led_enable ? "true" : "false");
    sb_json_quoted(&sb, up.device_id);
    sb_printf(&sb, ",\"imei\":");
    sb_json_quoted(&sb, lte.imei);
    sb_printf(&sb, ",\"rssi\":%d,\"mqttConnected\":%s,\"samples\":%u,"
                   "\"batchesOk\":%u,\"batchesFail\":%u}",
              lte.rssi_dbm, up.mqtt_connected ? "true" : "false",
              (unsigned)up.samples, (unsigned)up.batches_ok,
              (unsigned)up.batches_fail);
    return sb.overflow ? -1 : 0;
}

static int rpc_set_config_json(const char *json, size_t len, char *err, size_t err_cap)
{
    static char buf[512];
    static json_pool_t pool;
    if (len >= sizeof(buf)) {
        snprintf(err, err_cap, "配置过长");
        return -1;
    }
    memcpy(buf, json, len);
    buf[len] = '\0';
    const char *perr = NULL;
    json_node_t *root = json_parse(buf, len, &pool, &perr);
    if (root == NULL || root->type != JSON_OBJ) {
        snprintf(err, err_cap, "配置必须是 JSON 对象");
        return -1;
    }

    app_config_t c;
    app_config_get(&c);
    bool restart = false;
    long v = 0;
    if (json_int(json_obj_get(root, "jkPollMs"), &v) && v >= 100 && v <= 60000) {
        c.jk_uart_poll_ms = (uint32_t)v;
    }
    if (json_int(json_obj_get(root, "sampleIntervalMs"), &v)) {
        if (v < 200 || v > 60000) {
            snprintf(err, err_cap, "sampleIntervalMs 超出 200..60000");
            return -1;
        }
        c.mqtt_sample_ms = (uint32_t)v;
        restart = true;
    }
    if (json_int(json_obj_get(root, "batchSamples"), &v)) {
        if (v < 1 || v > 30) {
            snprintf(err, err_cap, "batchSamples 超出 1..30");
            return -1;
        }
        c.mqtt_batch = (uint8_t)v;
        restart = true;
    }
    if (json_int(json_obj_get(root, "keepaliveS"), &v)) {
        if (v < 15 || v > 600) {
            snprintf(err, err_cap, "keepaliveS 超出 15..600");
            return -1;
        }
        c.mqtt_keepalive_s = (uint16_t)v;
        restart = true;
    }
    if (json_int(json_obj_get(root, "logLevel"), &v)) {
        if (v < 0 || v > 5) {
            snprintf(err, err_cap, "logLevel 超出 0..5");
            return -1;
        }
        c.log_level = (int)v;
    }
    bool b = false;
    if (json_bool(json_obj_get(root, "ledEnable"), &b)) {
        c.led_enable = b;
    }
    if (json_bool(json_obj_get(root, "mqttEnable"), &b)) {
        c.mqtt_enable = b;
        restart = true;
    }
    if (json_bool(json_obj_get(root, "lteEnable"), &b)) {
        c.lte_enable = b;
        restart = true;
    }
    const char *s = json_str(json_obj_get(root, "mqttHost"), NULL);
    if (s != NULL) {
        snprintf(c.mqtt_host, sizeof(c.mqtt_host), "%s", s);
        restart = true;
    }
    s = json_str(json_obj_get(root, "mqttPrefix"), NULL);
    if (s != NULL) {
        snprintf(c.mqtt_prefix, sizeof(c.mqtt_prefix), "%s", s);
        restart = true;
    }
    s = json_str(json_obj_get(root, "mqttUser"), NULL);
    if (s != NULL) {
        snprintf(c.mqtt_user, sizeof(c.mqtt_user), "%s", s);
        restart = true;
    }
    s = json_str(json_obj_get(root, "mqttPassword"), NULL);
    if (s != NULL) {
        snprintf(c.mqtt_password, sizeof(c.mqtt_password), "%s", s);
        restart = true;
    }
    s = json_str(json_obj_get(root, "lteApn"), NULL);
    if (s != NULL) {
        snprintf(c.lte_apn, sizeof(c.lte_apn), "%s", s);
        restart = true;
    }
    if (json_int(json_obj_get(root, "mqttPort"), &v) && v > 0 && v <= 65535) {
        c.mqtt_port = (uint16_t)v;
        restart = true;
    }

    app_config_save(&c);
    app_config_apply_log_level();
    ESP_LOGI(TAG, "RPC 更新配置：restartRequired=%d", restart);
    return restart ? 1 : 0;
}

static bool rpc_get_snapshot(void *snap)
{
    return bms_manager_get_snapshot((bms_snapshot_t *)snap);
}

static int rpc_read_regs(uint16_t start, uint16_t count, uint16_t *out,
                         char *err, size_t err_cap)
{
    if (strcmp(bms_manager_name(), "uart") != 0) {
        snprintf(err, err_cap, "当前采集通道 %s 不支持原始寄存器访问", bms_manager_name());
        return -1;
    }
    return jk_bms_uart_read_regs(start, count, out, err, err_cap);
}

static int rpc_write_regs(uint16_t start, const uint16_t *vals, size_t count,
                          char *err, size_t err_cap)
{
    if (strcmp(bms_manager_name(), "uart") != 0) {
        snprintf(err, err_cap, "当前采集通道 %s 不支持原始寄存器写入", bms_manager_name());
        return -1;
    }
    return jk_bms_uart_write_regs(start, vals, count, err, err_cap);
}

void app_main(void)
{
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);

    /* lwIP/TCP-IP 栈必须先于任何 socket 使用（log_stream 任务会立即创建 socket） */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    log_stream_init();
    app_config_init();
    app_config_get(&s_app_cfg);
    app_config_apply_log_level();
    ESP_LOGI(TAG, "app config: transport=%s logLevel=%d",
             s_app_cfg.bms_transport, s_app_cfg.log_level);
    led_ctrl_init();

    /* 放在 LED 初始化之后：按住 BOOT 时 WS2812 显示紫色，触发恢复出厂变红色 */
    factory_reset_check();

    s_jk_cfg = (jk_bms_config_t){
        .uart_num = CONFIG_JINJIAN_JK_UART_NUM,
        .tx_gpio = CONFIG_JINJIAN_JK_TX_GPIO,
        .rx_gpio = CONFIG_JINJIAN_JK_RX_GPIO,
        .baud_rate = CONFIG_JINJIAN_JK_BAUD,
        .slave_addr = CONFIG_JINJIAN_JK_SLAVE_ADDR,
        .poll_interval_ms = s_app_cfg.jk_uart_poll_ms,
        .response_timeout_ms = CONFIG_JINJIAN_JK_RESPONSE_TIMEOUT_MS,
        .fast_charge_current_ma = CONFIG_JINJIAN_JK_FAST_CHARGE_CURRENT_MA,
        .low_temp_charge_c = CONFIG_JINJIAN_LOW_TEMP_CHARGE_C,
        .low_temp_discharge_c = CONFIG_JINJIAN_LOW_TEMP_DISCHARGE_C,
        .charge_target_soc = CONFIG_JINJIAN_CHARGE_TARGET_SOC_DEFAULT,
    };

    bms_driver_t uart_driver = {
        .name = "uart",
        .config = &s_jk_cfg,
        .init = jk_bms_uart_init,
        .deinit = jk_bms_uart_deinit,
        .get_snapshot = jk_bms_uart_get_snapshot,
        .set_charge_time_min = jk_bms_uart_set_charge_time_min,
        .set_charge_target_soc = jk_bms_uart_set_charge_target_soc,
        .end_fast_charge = jk_bms_uart_end_fast_charge,
    };

    s_ble_cfg = (jk_bms_ble_config_t){
        .target_name = {0},
        .target_addr = {0},
        .poll_interval_ms = s_app_cfg.ble_poll_ms,
        .scan_timeout_ms = s_app_cfg.ble_scan_timeout_ms,
        .reconnect_interval_ms = s_app_cfg.ble_reconnect_ms,
        .protocol_version = s_app_cfg.ble_protocol,
        .fast_charge_current_ma = CONFIG_JINJIAN_JK_FAST_CHARGE_CURRENT_MA,
        .low_temp_charge_c = CONFIG_JINJIAN_LOW_TEMP_CHARGE_C,
        .low_temp_discharge_c = CONFIG_JINJIAN_LOW_TEMP_DISCHARGE_C,
        .charge_target_soc = CONFIG_JINJIAN_CHARGE_TARGET_SOC_DEFAULT,
    };
    snprintf(s_ble_cfg.target_name, sizeof(s_ble_cfg.target_name), "%s", s_app_cfg.ble_target_name);
    snprintf(s_ble_cfg.target_addr, sizeof(s_ble_cfg.target_addr), "%s", s_app_cfg.ble_target_addr);

    bms_driver_t ble_driver = {
        .name = "ble",
        .config = &s_ble_cfg,
        .init = jk_bms_ble_init,
        .deinit = jk_bms_ble_deinit,
        .get_snapshot = jk_bms_ble_get_snapshot,
        .set_charge_time_min = jk_bms_ble_set_charge_time_min,
        .set_charge_target_soc = jk_bms_ble_set_charge_target_soc,
        .end_fast_charge = jk_bms_ble_end_fast_charge,
    };

    if (strcmp(s_app_cfg.bms_transport, "ble") == 0) {
        ESP_LOGI(TAG, "BMS transport: BLE");
        bms_manager_set_driver(&ble_driver);
    } else {
        ESP_LOGI(TAG, "BMS transport: UART");
        bms_manager_set_driver(&uart_driver);
    }

    jinjian_bms_config_t vehicle = {
        .uart_num = CONFIG_JINJIAN_VEH_UART_NUM,
        .tx_gpio = CONFIG_JINJIAN_VEH_TX_GPIO,
        .rx_gpio = CONFIG_JINJIAN_VEH_RX_GPIO,
        .baud_rate = CONFIG_JINJIAN_VEH_BAUD,
        .slave_addr = CONFIG_JINJIAN_VEH_SLAVE_ADDR,
    };
    jinjian_bms_init(&vehicle);

    /* ML307-NL 4G：EN 上电 + AT 指令入网 + 周期上报电池数据 */
#ifdef CONFIG_JINJIAN_LTE_EN_ACTIVE_LOW
    const bool lte_en_active_low = true;
#else
    const bool lte_en_active_low = false;
#endif
    s_lte_cfg = (ml307_4g_config_t){
        .enable = s_app_cfg.lte_enable,
        .uart_num = CONFIG_JINJIAN_LTE_UART_NUM,
        .tx_gpio = CONFIG_JINJIAN_LTE_TX_GPIO,
        .rx_gpio = CONFIG_JINJIAN_LTE_RX_GPIO,
        .en_gpio = CONFIG_JINJIAN_LTE_EN_GPIO,
        .en_active_low = lte_en_active_low,
        .en_pulse_ms = CONFIG_JINJIAN_LTE_EN_PULSE_MS,
        .baud_rate = CONFIG_JINJIAN_LTE_BAUD,
        .apn = {0},
        .host = {0},
        .port = s_app_cfg.mqtt_port,
    };
    snprintf(s_lte_cfg.apn, sizeof(s_lte_cfg.apn), "%s", s_app_cfg.lte_apn);
    /* 4G 的 TCP 目标就是 MQTT broker */
    snprintf(s_lte_cfg.host, sizeof(s_lte_cfg.host), "%s", s_app_cfg.mqtt_host);
    ml307_4g_start(&s_lte_cfg);

    /* 采集上报：BMS → 压缩编码 → MQTT（device_id 用 IMEI，取不到回退 PN） */
    s_uplink_cfg = (bms_uplink_config_t){
        .enable = s_app_cfg.mqtt_enable && s_app_cfg.lte_enable,
        .host = {0},
        .port = s_app_cfg.mqtt_port,
        .user = {0},
        .password = {0},
        .topic_prefix = {0},
        .sample_interval_ms = s_app_cfg.mqtt_sample_ms,
        .batch_samples = s_app_cfg.mqtt_batch,
        .keepalive_s = s_app_cfg.mqtt_keepalive_s,
    };
    snprintf(s_uplink_cfg.host, sizeof(s_uplink_cfg.host), "%s", s_app_cfg.mqtt_host);
    snprintf(s_uplink_cfg.user, sizeof(s_uplink_cfg.user), "%s", s_app_cfg.mqtt_user);
    snprintf(s_uplink_cfg.password, sizeof(s_uplink_cfg.password), "%s",
             s_app_cfg.mqtt_password);
    snprintf(s_uplink_cfg.topic_prefix, sizeof(s_uplink_cfg.topic_prefix), "%s",
             s_app_cfg.mqtt_prefix);
    bms_uplink_start(&s_uplink_cfg);

    /* JSON-RPC：MQTT(<prefix>/<device>/call) 与 Web(/api/rpc) 共用同一套方法 */
    rpc_deps_t rpc_deps = {
        .get_info = rpc_get_info,
        .get_config_json = rpc_get_config_json,
        .set_config_json = rpc_set_config_json,
        .reboot = rpc_reboot_soon,
        .get_snapshot = rpc_get_snapshot,
        .read_regs = rpc_read_regs,
        .write_regs = rpc_write_regs,
    };
    rpc_server_init(&rpc_deps);

    web_server_start();
    power_mgr_init();
    ESP_LOGI(TAG, "Jinjian battery SOC module started");
}
