#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "bms_interface.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "factory_reset.h"
#include "nvs_flash.h"
#include "jinjian_bms.h"
#include "jk_bms_ble.h"
#include "jk_bms.h"
#include "led_ctrl.h"
#include "power_mgr.h"
#include "sdkconfig.h"
#include "web_server.h"
#include "log_stream.h"

static const char *TAG = "app";

static jk_bms_config_t s_jk_cfg;
static jk_bms_ble_config_t s_ble_cfg;
static app_config_t s_app_cfg;

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
        .de_gpio = CONFIG_JINJIAN_VEH_DE_GPIO,
        .baud_rate = CONFIG_JINJIAN_VEH_BAUD,
        .slave_addr = CONFIG_JINJIAN_VEH_SLAVE_ADDR,
    };
    jinjian_bms_init(&vehicle);

    web_server_start();
    power_mgr_init();
    ESP_LOGI(TAG, "Jinjian battery SOC module started");
}
