#pragma once

#include <stdint.h>

#define APP_CFG_SSID_LEN  33
#define APP_CFG_PASS_LEN  65
#define APP_CFG_NAME_LEN  33
#define APP_CFG_ADDR_LEN  19
#define APP_CFG_TRANSPORT_LEN 8

typedef struct {
    char ap_ssid[APP_CFG_SSID_LEN];
    char ap_password[APP_CFG_PASS_LEN];
    char bms_transport[APP_CFG_TRANSPORT_LEN]; /* "uart" | "ble" */
    char ble_target_name[APP_CFG_NAME_LEN];
    char ble_target_addr[APP_CFG_ADDR_LEN];
    int ble_protocol;
    uint32_t ble_poll_ms;
    uint32_t ble_scan_timeout_ms;
    uint32_t ble_reconnect_ms;
    uint32_t jk_uart_poll_ms;
    int log_level;                 /* 0..5，对应 esp_log_level_t */
} app_config_t;

void app_config_init(void);
void app_config_get(app_config_t *out);
void app_config_save(const app_config_t *cfg);
void app_config_apply_log_level(void);
