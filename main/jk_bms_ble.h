#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "bms_interface.h"

#define JK_BLE_PROTOCOL_24S 2
#define JK_BLE_PROTOCOL_32S 3

typedef struct {
    char target_name[33];        /* 广播名子串，大小写不敏感；空串表示只按 MAC */
    char target_addr[19];        /* "aa:bb:cc:dd:ee:ff"，空串表示按名称扫描 */
    uint32_t poll_interval_ms;
    uint32_t scan_timeout_ms;
    uint32_t reconnect_interval_ms;
    int protocol_version;        /* JK_BLE_PROTOCOL_24S / JK_BLE_PROTOCOL_32S */
    uint16_t fast_charge_current_ma;
    int16_t low_temp_charge_c;
    int16_t low_temp_discharge_c;
    uint16_t charge_target_soc;
} jk_bms_ble_config_t;

int jk_bms_ble_init(void *config);
void jk_bms_ble_deinit(void);
bool jk_bms_ble_get_snapshot(bms_snapshot_t *out);
void jk_bms_ble_set_charge_time_min(uint16_t minutes);
void jk_bms_ble_set_charge_target_soc(uint16_t soc);
void jk_bms_ble_end_fast_charge(void);

/* Web 页面蓝牙扫描：不影响正常连接的独立发现流程 */
int jk_bms_ble_web_scan_start(uint32_t duration_ms);
bool jk_bms_ble_web_scan_active(void);
size_t jk_bms_ble_web_scan_count(void);
int jk_bms_ble_web_scan_get(size_t index, char *name, size_t name_cap,
                            char *mac, size_t mac_cap);
