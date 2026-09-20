idf_component_register(SRCS "app_config.c" "bms_interface.c" "jinjian_bms.c"
                        "jk_bms.c" "jk_bms_ble.c" "log_stream.c" "modbus_rtu.c"
                        "factory_reset.c" "ota_update.c" "web_server.c" "main.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES bt driver esp_driver_uart esp_event esp_http_server
                                  esp_netif esp_wifi app_update nvs_flash freertos)
