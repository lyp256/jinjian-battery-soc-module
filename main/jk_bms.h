#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "bms_interface.h"

typedef struct {
    int uart_num;
    int tx_gpio;
    int rx_gpio;
    uint32_t baud_rate;
    uint8_t slave_addr;
    uint32_t poll_interval_ms;
    uint32_t response_timeout_ms;
    uint16_t fast_charge_current_ma;
    int16_t low_temp_charge_c;     /* 用于补充判断低温充电保护 */
    int16_t low_temp_discharge_c;  /* 极空无对应位时由温度推导 */
    uint16_t charge_target_soc;
} jk_bms_config_t;

int jk_bms_uart_init(void *config);
void jk_bms_uart_deinit(void);
bool jk_bms_uart_get_snapshot(bms_snapshot_t *out);

/* 车端写入项：先更新模块缓存，后续轮询会尝试从极空配置区回读。 */
void jk_bms_uart_set_charge_time_min(uint16_t minutes);
void jk_bms_uart_set_charge_target_soc(uint16_t soc);
void jk_bms_uart_end_fast_charge(void);
