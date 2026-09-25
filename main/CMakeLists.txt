idf_component_register(SRCS "app_config.c" "bms_interface.c" "jinjian_bms.c"
                        "jk_bms.c" "jk_bms_ble.c" "led_ctrl.c" "log_stream.c" "modbus_rtu.c"
                        "factory_reset.c" "ota_update.c" "web_server.c" "ml307_4g.c" "main.c"
                        "power_mgr.c" "mqtt_client.c" "bms_hist.c" "bms_uplink.c"
                        "json_util.c" "rpc_server.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES bt driver esp_driver_rmt esp_driver_uart esp_event esp_http_server
                                  esp_netif esp_wifi app_update nvs_flash freertos)
