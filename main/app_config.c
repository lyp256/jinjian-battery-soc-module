#include "app_config.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_log_level.h"
#include "esp_mac.h"
#include "nvs.h"
#include "sdkconfig.h"

static const char *TAG = "app_cfg";
static const char *NVS_NS = "jinjian";

static app_config_t s_cfg;

static void build_unique_ssid(char *out, size_t cap)
{
    uint8_t mac[6] = {0};
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) != ESP_OK) {
        snprintf(out, cap, "%s", CONFIG_JINJIAN_AP_SSID);
        return;
    }
    snprintf(out, cap, "SOC-Module-%02X%02X%02X", mac[3], mac[4], mac[5]);
}

static void set_defaults(void)
{
    memset(&s_cfg, 0, sizeof(s_cfg));
    build_unique_ssid(s_cfg.ap_ssid, sizeof(s_cfg.ap_ssid));
    snprintf(s_cfg.ap_password, sizeof(s_cfg.ap_password), "%s", CONFIG_JINJIAN_AP_PASSWORD);
    snprintf(s_cfg.bms_transport, sizeof(s_cfg.bms_transport), "%s", CONFIG_JINJIAN_BMS_TRANSPORT);
    snprintf(s_cfg.ble_target_name, sizeof(s_cfg.ble_target_name), "%s", CONFIG_JINJIAN_BLE_TARGET_NAME);
    snprintf(s_cfg.ble_target_addr, sizeof(s_cfg.ble_target_addr), "%s", CONFIG_JINJIAN_BLE_TARGET_ADDR);
    s_cfg.ble_protocol = CONFIG_JINJIAN_BLE_PROTOCOL;
    s_cfg.ble_poll_ms = CONFIG_JINJIAN_BLE_POLL_INTERVAL_MS;
    s_cfg.ble_scan_timeout_ms = CONFIG_JINJIAN_BLE_SCAN_TIMEOUT_MS;
    s_cfg.ble_reconnect_ms = CONFIG_JINJIAN_BLE_RECONNECT_INTERVAL_MS;
    s_cfg.jk_uart_poll_ms = CONFIG_JINJIAN_JK_POLL_INTERVAL_MS;
    s_cfg.log_level = CONFIG_JINJIAN_LOG_LEVEL_DEFAULT;
}

void app_config_init(void)
{
    set_defaults();
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    size_t len;
    if (nvs_get_str(h, "ap_ssid", NULL, &len) == ESP_OK && len <= sizeof(s_cfg.ap_ssid)) {
        nvs_get_str(h, "ap_ssid", s_cfg.ap_ssid, &len);
    }
    if (nvs_get_str(h, "ap_pwd", NULL, &len) == ESP_OK && len <= sizeof(s_cfg.ap_password)) {
        nvs_get_str(h, "ap_pwd", s_cfg.ap_password, &len);
    }
    if (nvs_get_str(h, "transport", NULL, &len) == ESP_OK && len <= sizeof(s_cfg.bms_transport)) {
        nvs_get_str(h, "transport", s_cfg.bms_transport, &len);
    }
    if (nvs_get_str(h, "ble_name", NULL, &len) == ESP_OK && len <= sizeof(s_cfg.ble_target_name)) {
        nvs_get_str(h, "ble_name", s_cfg.ble_target_name, &len);
    }
    if (nvs_get_str(h, "ble_addr", NULL, &len) == ESP_OK && len <= sizeof(s_cfg.ble_target_addr)) {
        nvs_get_str(h, "ble_addr", s_cfg.ble_target_addr, &len);
    }
    int32_t proto = 0;
    if (nvs_get_i32(h, "ble_proto", &proto) == ESP_OK) {
        s_cfg.ble_protocol = (int)proto;
    }
    nvs_get_u32(h, "ble_poll", &s_cfg.ble_poll_ms);
    nvs_get_u32(h, "ble_scan", &s_cfg.ble_scan_timeout_ms);
    nvs_get_u32(h, "ble_rec", &s_cfg.ble_reconnect_ms);
    nvs_get_u32(h, "uart_poll", &s_cfg.jk_uart_poll_ms);
    int32_t lvl = 0;
    if (nvs_get_i32(h, "log_level", &lvl) == ESP_OK) {
        s_cfg.log_level = (int)lvl;
    }
    nvs_close(h);

    if (s_cfg.ble_protocol < 2 || s_cfg.ble_protocol > 3) {
        s_cfg.ble_protocol = CONFIG_JINJIAN_BLE_PROTOCOL;
    }
    if (strcmp(s_cfg.bms_transport, "uart") != 0 && strcmp(s_cfg.bms_transport, "ble") != 0) {
        snprintf(s_cfg.bms_transport, sizeof(s_cfg.bms_transport), "uart");
    }
    if (s_cfg.log_level < 0 || s_cfg.log_level > 5) {
        s_cfg.log_level = CONFIG_JINJIAN_LOG_LEVEL_DEFAULT;
    }
    if (s_cfg.ap_ssid[0] == '\0' ||
        strcmp(s_cfg.ap_ssid, CONFIG_JINJIAN_AP_SSID) == 0) {
        build_unique_ssid(s_cfg.ap_ssid, sizeof(s_cfg.ap_ssid));
    }
    ESP_LOGI(TAG, "config: transport=%s ap=%s ble=%s/%s",
             s_cfg.bms_transport, s_cfg.ap_ssid, s_cfg.ble_target_name, s_cfg.ble_target_addr);
}

void app_config_get(app_config_t *out)
{
    *out = s_cfg;
}

void app_config_save(const app_config_t *cfg)
{
    s_cfg = *cfg;
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_str(h, "ap_ssid", s_cfg.ap_ssid);
    nvs_set_str(h, "ap_pwd", s_cfg.ap_password);
    nvs_set_str(h, "transport", s_cfg.bms_transport);
    nvs_set_str(h, "ble_name", s_cfg.ble_target_name);
    nvs_set_str(h, "ble_addr", s_cfg.ble_target_addr);
    nvs_set_i32(h, "ble_proto", s_cfg.ble_protocol);
    nvs_set_u32(h, "ble_poll", s_cfg.ble_poll_ms);
    nvs_set_u32(h, "ble_scan", s_cfg.ble_scan_timeout_ms);
    nvs_set_u32(h, "ble_rec", s_cfg.ble_reconnect_ms);
    nvs_set_u32(h, "uart_poll", s_cfg.jk_uart_poll_ms);
    nvs_set_i32(h, "log_level", s_cfg.log_level);
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "config saved: transport=%s", s_cfg.bms_transport);
}

void app_config_apply_log_level(void)
{
    if (s_cfg.log_level < 0 || s_cfg.log_level > 5) {
        s_cfg.log_level = CONFIG_JINJIAN_LOG_LEVEL_DEFAULT;
    }
    esp_log_level_set("*", (esp_log_level_t)s_cfg.log_level);
    ESP_LOGI(TAG, "applied log level=%d", s_cfg.log_level);
}
