#pragma once

#include <stdbool.h>
#include <stdint.h>

#define BMS_MAX_CELLS 32

/* 与具体保护板协议无关的电池快照，供金箭从机 / Web 页面统一消费。 */
typedef struct {
    bool have_data;      /* 是否至少成功读取过一次 */
    bool fresh;          /* 最近一轮轮询是否成功 */
    uint32_t poll_count;
    uint32_t poll_failures;
    uint32_t last_ok_ms; /* esp_timer 单调时钟 */

    uint16_t total_voltage_raw;   /* 0.01V */
    uint8_t cell_count;
    uint8_t soc;
    uint16_t capacity_ah;
    int16_t charge_current_raw;   /* 0.01A，充正放负 */
    int16_t temp1;
    int16_t temp2;
    int16_t board_temp;

    uint16_t cell_mv[BMS_MAX_CELLS];
    uint16_t max_cell_diff_mv;

    uint16_t battery_type;
    uint16_t cycle_count;
    uint16_t balance_status;
    uint16_t nominal_capacity_ah;

    char pn[17];
    uint16_t version_regs[2];

    uint16_t charge_time_min;
    uint16_t charge_target_soc;
    uint16_t fast_charging;
    uint16_t protection[5];       /* 短路/过温充/过温放/低温充/低温放 */
} bms_snapshot_t;

typedef struct bms_driver bms_driver_t;

struct bms_driver {
    const char *name;
    void *config;                       /* 指向各驱动自己的配置结构 */
    int (*init)(void *config);
    void (*deinit)(void);
    bool (*get_snapshot)(bms_snapshot_t *out);
    void (*set_charge_time_min)(uint16_t minutes);
    void (*set_charge_target_soc)(uint16_t soc);
    void (*end_fast_charge)(void);
};

void bms_manager_set_driver(const bms_driver_t *drv);
const char *bms_manager_name(void);
bool bms_manager_get_snapshot(bms_snapshot_t *out);
void bms_manager_set_charge_time_min(uint16_t minutes);
void bms_manager_set_charge_target_soc(uint16_t soc);
void bms_manager_end_fast_charge(void);

/* 根据本机 WiFi MAC 生成 16 位唯一 PN（形如 SOC-9888E072BD78），
 * 供金箭从机 / Web 页面作为模块自身序列号使用。 */
void bms_mac_pn(char *pn, size_t cap);
