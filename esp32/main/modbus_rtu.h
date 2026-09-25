#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MODBUS_FUNC_READ_COILS   0x01
#define MODBUS_FUNC_READ_REGS    0x03
#define MODBUS_FUNC_WRITE_COIL   0x05
#define MODBUS_FUNC_WRITE_REG    0x06
#define MODBUS_FUNC_WRITE_COILS  0x0F
#define MODBUS_FUNC_WRITE_REGS   0x10

#define MODBUS_EXC_ILLEGAL_FUNCTION 0x01
#define MODBUS_EXC_ILLEGAL_ADDRESS  0x02
#define MODBUS_EXC_ILLEGAL_VALUE    0x03
#define MODBUS_EXC_SLAVE_BUSY       0x06

typedef struct {
    uint8_t slave;
    uint8_t func;
    uint16_t addr;
    uint16_t num;
    uint16_t value;
    uint8_t data[256];
    size_t data_len;
    uint8_t raw[256];
    size_t raw_len;
} modbus_request_t;

uint16_t modbus_crc16(const uint8_t *data, size_t len);
void modbus_append_crc(uint8_t *frame, size_t body_len);
bool modbus_check_crc(const uint8_t *frame, size_t len);

size_t modbus_build_read_regs(uint8_t *dst, uint8_t slave, uint16_t addr, uint16_t num);
size_t modbus_build_read_coils(uint8_t *dst, uint8_t slave, uint16_t addr, uint16_t num);
size_t modbus_build_write_reg(uint8_t *dst, uint8_t slave, uint16_t addr, uint16_t value);
size_t modbus_build_write_regs(uint8_t *dst, uint8_t slave, uint16_t addr,
                               const uint16_t *values, size_t count);
size_t modbus_build_write_coil(uint8_t *dst, uint8_t slave, uint16_t addr, bool on);

/* Scan the buffer from every offset and return the first valid Modbus RTU
 * request frame. Returns 0 when no complete request is found yet. */
size_t modbus_scan_request(const uint8_t *buf, size_t len, modbus_request_t *req);

size_t modbus_build_read_response(uint8_t *dst, uint8_t slave, uint8_t func,
                                  const uint8_t *data, size_t data_len);
size_t modbus_build_write_response(uint8_t *dst, const uint8_t *request);
size_t modbus_build_exception(uint8_t *dst, uint8_t slave, uint8_t func, uint8_t code);

size_t modbus_pack_regs(uint8_t *dst, const uint16_t *values, size_t count);
