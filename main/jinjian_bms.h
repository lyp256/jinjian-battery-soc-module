#pragma once

#include <stdint.h>

typedef struct {
    int uart_num;
    int tx_gpio;
    int rx_gpio;
    int de_gpio;               /* MAX3485 DE/RE 共用脚，接到 UART RTS */
    uint32_t baud_rate;
    uint8_t slave_addr;
} jinjian_bms_config_t;

void jinjian_bms_init(const jinjian_bms_config_t *cfg);
