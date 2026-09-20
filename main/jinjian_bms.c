#include "jinjian_bms.h"

#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bms_interface.h"
#include "modbus_rtu.h"

static const char *TAG = "jinjian";

static jinjian_bms_config_t s_cfg;
static uint8_t s_rx_buf[512];
static size_t s_rx_len;
static QueueHandle_t s_uart_queue;

static void log_frame(const char *dir, const uint8_t *buf, size_t len)
{
    char hex[128];
    size_t n = len < sizeof(hex) / 2 - 1 ? len : sizeof(hex) / 2 - 1;
    for (size_t i = 0; i < n; i++) {
        snprintf(hex + i * 2, 3, "%02X", buf[i]);
    }
    ESP_LOGD(TAG, "%s %s", dir, hex);
}

static bool read_reg_value(const bms_snapshot_t *s, uint16_t addr,
                           uint16_t *out)
{
    if (addr == 0) {
        *out = s->total_voltage_raw;
        return true;
    }
    if (addr == 1) {
        *out = s->cell_count;
        return true;
    }
    if (addr == 2) {
        *out = s->soc;
        return true;
    }
    if (addr == 3) {
        *out = s->capacity_ah;
        return true;
    }
    if (addr == 4) {
        *out = s->protection[0];
        return true;
    }
    if (addr == 5) {
        *out = (uint16_t)(int16_t)s->charge_current_raw;
        return true;
    }
    if (addr == 6) {
        *out = (uint16_t)(int16_t)s->temp2;
        return true;
    }
    if (addr == 7) {
        *out = (uint16_t)(int16_t)s->temp1;
        return true;
    }
    if (addr == 8) {
        *out = (uint16_t)(int16_t)s->board_temp;
        return true;
    }
    if (addr >= 9 && addr <= 32) {
        uint8_t i = (uint8_t)(addr - 9);
        *out = i < s->cell_count ? s->cell_mv[i] : 0;
        return true;
    }
    if (addr == 103) {
        *out = s->battery_type;
        return true;
    }
    if (addr == 104) {
        *out = s->cycle_count;
        return true;
    }
    if ((addr >= 105 && addr <= 109) || addr == 111 || addr == 112) {
        *out = 0;
        return true;
    }
    if (addr == 110) {
        *out = s->balance_status;
        return true;
    }
    if (addr == 113) {
        *out = s->nominal_capacity_ah;
        return true;
    }
    if (addr >= 1000 && addr <= 1007) {
        size_t i = (size_t)(addr - 1000) * 2;
        *out = ((uint16_t)(uint8_t)s->pn[i] << 8) | (uint8_t)s->pn[i + 1];
        return true;
    }
    if (addr == 1016) {
        *out = s->version_regs[0];
        return true;
    }
    if (addr == 1017) {
        *out = s->version_regs[1];
        return true;
    }
    if (addr == 1089) {
        *out = s->charge_time_min;
        return true;
    }
    if (addr == 1090) {
        *out = s->charge_target_soc;
        return true;
    }
    return false;
}

static size_t handle_read_regs(const modbus_request_t *req, uint8_t *resp)
{
    if (req->num == 0 || req->num > 125) {
        return modbus_build_exception(resp, s_cfg.slave_addr, req->func,
                                      MODBUS_EXC_ILLEGAL_VALUE);
    }

    bms_snapshot_t snap;
    bms_manager_get_snapshot(&snap);

    uint16_t values[125];
    for (uint16_t i = 0; i < req->num; i++) {
        if (!read_reg_value(&snap, (uint16_t)(req->addr + i), &values[i])) {
            return modbus_build_exception(resp, s_cfg.slave_addr, req->func,
                                          MODBUS_EXC_ILLEGAL_ADDRESS);
        }
    }

    uint8_t data[250];
    size_t data_len = modbus_pack_regs(data, values, req->num);
    return modbus_build_read_response(resp, s_cfg.slave_addr, req->func,
                                      data, data_len);
}

static size_t handle_read_coils(const modbus_request_t *req, uint8_t *resp)
{
    if (req->num == 0 || req->num > 64) {
        return modbus_build_exception(resp, s_cfg.slave_addr, req->func,
                                      MODBUS_EXC_ILLEGAL_VALUE);
    }

    bms_snapshot_t snap;
    bms_manager_get_snapshot(&snap);

    uint8_t bits[64] = {0};
    for (uint16_t i = 0; i < req->num; i++) {
        uint16_t addr = (uint16_t)(req->addr + i);
        if (addr >= 4 && addr <= 8) {
            bits[i] = (uint8_t)snap.protection[addr - 4];
        } else if (addr == 63) {
            bits[i] = (uint8_t)snap.fast_charging;
        } else {
            bits[i] = 0;
        }
    }

    size_t nbytes = (req->num + 7) / 8;
    uint8_t data[8] = {0};
    for (uint16_t i = 0; i < req->num; i++) {
        if (bits[i]) {
            data[i >> 3] |= (uint8_t)(1U << (i & 7));
        }
    }
    return modbus_build_read_response(resp, s_cfg.slave_addr, req->func,
                                      data, nbytes);
}

static size_t handle_write_reg(const modbus_request_t *req, uint8_t *resp)
{
    if (req->addr == 0x0441) {
        if (req->value > 120) {
            return modbus_build_exception(resp, s_cfg.slave_addr, req->func,
                                          MODBUS_EXC_ILLEGAL_VALUE);
        }
        bms_manager_set_charge_time_min(req->value);
        ESP_LOGI(TAG, "write charge time -> %u min", req->value);
        return modbus_build_write_response(resp, req->raw);
    }
    if (req->addr == 0x0442) {
        if (req->value > 100) {
            return modbus_build_exception(resp, s_cfg.slave_addr, req->func,
                                          MODBUS_EXC_ILLEGAL_VALUE);
        }
        bms_manager_set_charge_target_soc(req->value);
        ESP_LOGI(TAG, "write charge target SOC -> %u%%", req->value);
        return modbus_build_write_response(resp, req->raw);
    }
    return modbus_build_exception(resp, s_cfg.slave_addr, req->func,
                                  MODBUS_EXC_ILLEGAL_ADDRESS);
}

static size_t handle_write_coil(const modbus_request_t *req, uint8_t *resp)
{
    if (req->addr == 0x0040 &&
        (req->value == 0xFF00 || req->value == 0x0000)) {
        if (req->value == 0xFF00) {
            bms_manager_end_fast_charge();
            ESP_LOGI(TAG, "fast charge stop requested");
        }
        return modbus_build_write_response(resp, req->raw);
    }
    return modbus_build_exception(resp, s_cfg.slave_addr, req->func,
                                  MODBUS_EXC_ILLEGAL_ADDRESS);
}

static size_t handle_request(const modbus_request_t *req, uint8_t *resp)
{
    switch (req->func) {
    case MODBUS_FUNC_READ_REGS:
        return handle_read_regs(req, resp);
    case MODBUS_FUNC_READ_COILS:
        return handle_read_coils(req, resp);
    case MODBUS_FUNC_WRITE_REG:
        return handle_write_reg(req, resp);
    case MODBUS_FUNC_WRITE_COIL:
        return handle_write_coil(req, resp);
    default:
        return modbus_build_exception(resp, s_cfg.slave_addr, req->func,
                                      MODBUS_EXC_ILLEGAL_FUNCTION);
    }
}

static void process_rx(void)
{
    while (s_rx_len > 0) {
        modbus_request_t req;
        size_t frame_len = modbus_scan_request(s_rx_buf, s_rx_len, &req);
        if (frame_len == 0) {
            break;
        }
        memmove(s_rx_buf, s_rx_buf + frame_len, s_rx_len - frame_len);
        s_rx_len -= frame_len;

        if (req.slave != s_cfg.slave_addr) {
            continue;
        }

        uint8_t resp[256];
        size_t resp_len = handle_request(&req, resp);
        if (resp_len > 0) {
            log_frame("RX", req.raw, req.raw_len);
            uart_write_bytes(s_cfg.uart_num, resp, resp_len);
            log_frame("TX", resp, resp_len);
        }
    }
}

static void jinjian_task(void *arg)
{
    (void)arg;
    uart_event_t event;
    while (1) {
        if (xQueueReceive(s_uart_queue, &event, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (event.type == UART_DATA) {
                uint8_t tmp[256];
                size_t want = event.size;
                if (want > sizeof(tmp)) {
                    want = sizeof(tmp);
                }
                int n = uart_read_bytes(s_cfg.uart_num, tmp, (uint32_t)want, 0);
                if (n > 0) {
                    if (s_rx_len + (size_t)n > sizeof(s_rx_buf)) {
                        ESP_LOGW(TAG, "RX buffer overflow, dropping %u bytes",
                                 s_rx_len);
                        s_rx_len = 0;
                    }
                    memcpy(s_rx_buf + s_rx_len, tmp, (size_t)n);
                    s_rx_len += (size_t)n;
                    process_rx();
                }
            } else if (event.type == UART_BUFFER_FULL || event.type == UART_FIFO_OVF ||
                       event.type == UART_FRAME_ERR || event.type == UART_PARITY_ERR) {
                uart_flush_input(s_cfg.uart_num);
                s_rx_len = 0;
            }
        }
    }
}

void jinjian_bms_init(const jinjian_bms_config_t *cfg)
{
    s_cfg = *cfg;

    uart_config_t uart_cfg = {
        .baud_rate = cfg->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(cfg->uart_num, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(cfg->uart_num, cfg->tx_gpio, cfg->rx_gpio,
                                 cfg->de_gpio, -1));
    ESP_ERROR_CHECK(uart_driver_install(cfg->uart_num, 1024, 0, 16,
                                        &s_uart_queue, 0));
    ESP_ERROR_CHECK(uart_set_mode(cfg->uart_num, UART_MODE_RS485_HALF_DUPLEX));
    ESP_ERROR_CHECK(uart_set_rx_timeout(cfg->uart_num, 2));

    xTaskCreate(jinjian_task, "jinjian_rs485", 4096, NULL, 6, NULL);
}
