#pragma once

#include <stdint.h>

typedef struct {
    int uart_num;
    int tx_gpio;               /* 第二路 THVD1406 的 D（DI） */
    int rx_gpio;               /* 第二路 THVD1406 的 R（RO） */
    uint32_t baud_rate;
    uint8_t slave_addr;
} jinjian_bms_config_t;

void jinjian_bms_init(const jinjian_bms_config_t *cfg);
