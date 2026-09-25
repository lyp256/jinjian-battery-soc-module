#pragma once

#include <stdbool.h>
#include <stdint.h>

#define APP_CFG_SSID_LEN  33
#define APP_CFG_PASS_LEN  65
#define APP_CFG_NAME_LEN  33
#define APP_CFG_ADDR_LEN  19
#define APP_CFG_TRANSPORT_LEN 8
#define APP_CFG_HOST_LEN  64
#define APP_CFG_APN_LEN   33
#define APP_CFG_USER_LEN  33
#define APP_CFG_MQTTPASS_LEN 33
#define APP_CFG_PREFIX_LEN 24

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
    bool lte_enable;                       /* 是否启用 ML307 4G 模块 */
    char lte_apn[APP_CFG_APN_LEN];         /* 留空 = 模块/SIM 默认 APN */
    bool mqtt_enable;                      /* 是否启用采集上报（MQTT） */
    char mqtt_host[APP_CFG_HOST_LEN];      /* MQTT broker，留空 = 不上报 */
    uint16_t mqtt_port;
    char mqtt_user[APP_CFG_USER_LEN];      /* 留空 = 匿名 */
    char mqtt_password[APP_CFG_MQTTPASS_LEN];
    char mqtt_prefix[APP_CFG_PREFIX_LEN];  /* 主题前缀，默认 /bms */
    uint32_t mqtt_sample_ms;               /* 采样周期 */
    uint8_t mqtt_batch;                    /* 每批样本数 */
    uint16_t mqtt_keepalive_s;             /* MQTT 保活 */
    int log_level;                 /* 0..5，对应 esp_log_level_t */
    bool led_enable;               /* 是否开启 LED 状态指示灯 */
} app_config_t;

void app_config_init(void);
void app_config_get(app_config_t *out);
void app_config_save(const app_config_t *cfg);
void app_config_apply_log_level(void);
