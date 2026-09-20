#include "modbus_rtu.h"

#include <string.h>

uint16_t modbus_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void modbus_append_crc(uint8_t *frame, size_t body_len)
{
    uint16_t crc = modbus_crc16(frame, body_len);
    frame[body_len] = (uint8_t)(crc & 0xFF);
    frame[body_len + 1] = (uint8_t)(crc >> 8);
}

bool modbus_check_crc(const uint8_t *frame, size_t len)
{
    if (len < 4) {
        return false;
    }
    uint16_t crc = modbus_crc16(frame, len - 2);
    return frame[len - 2] == (uint8_t)(crc & 0xFF) &&
           frame[len - 1] == (uint8_t)(crc >> 8);
}

size_t modbus_build_read_regs(uint8_t *dst, uint8_t slave, uint16_t addr, uint16_t num)
{
    dst[0] = slave;
    dst[1] = MODBUS_FUNC_READ_REGS;
    dst[2] = (uint8_t)(addr >> 8);
    dst[3] = (uint8_t)(addr & 0xFF);
    dst[4] = (uint8_t)(num >> 8);
    dst[5] = (uint8_t)(num & 0xFF);
    modbus_append_crc(dst, 6);
    return 8;
}

size_t modbus_build_read_coils(uint8_t *dst, uint8_t slave, uint16_t addr, uint16_t num)
{
    dst[0] = slave;
    dst[1] = MODBUS_FUNC_READ_COILS;
    dst[2] = (uint8_t)(addr >> 8);
    dst[3] = (uint8_t)(addr & 0xFF);
    dst[4] = (uint8_t)(num >> 8);
    dst[5] = (uint8_t)(num & 0xFF);
    modbus_append_crc(dst, 6);
    return 8;
}

size_t modbus_build_write_reg(uint8_t *dst, uint8_t slave, uint16_t addr, uint16_t value)
{
    dst[0] = slave;
    dst[1] = MODBUS_FUNC_WRITE_REG;
    dst[2] = (uint8_t)(addr >> 8);
    dst[3] = (uint8_t)(addr & 0xFF);
    dst[4] = (uint8_t)(value >> 8);
    dst[5] = (uint8_t)(value & 0xFF);
    modbus_append_crc(dst, 6);
    return 8;
}

size_t modbus_build_write_coil(uint8_t *dst, uint8_t slave, uint16_t addr, bool on)
{
    dst[0] = slave;
    dst[1] = MODBUS_FUNC_WRITE_COIL;
    dst[2] = (uint8_t)(addr >> 8);
    dst[3] = (uint8_t)(addr & 0xFF);
    dst[4] = on ? 0xFF : 0x00;
    dst[5] = 0x00;
    modbus_append_crc(dst, 6);
    return 8;
}

static size_t request_len_at(const uint8_t *buf, size_t off, size_t len)
{
    if (len - off < 2) {
        return 0;
    }
    uint8_t func = buf[off + 1];
    switch (func) {
    case MODBUS_FUNC_READ_COILS:
    case MODBUS_FUNC_READ_REGS:
    case MODBUS_FUNC_WRITE_COIL:
    case MODBUS_FUNC_WRITE_REG:
        return 8;
    case MODBUS_FUNC_WRITE_COILS:
    case MODBUS_FUNC_WRITE_REGS:
        if (len - off < 7) {
            return 0;
        }
        return 9 + buf[off + 6];
    default:
        /* Unknown function codes still have an 8-byte Modbus RTU shape. */
        return 8;
    }
}

static void parse_request_at(const uint8_t *buf, size_t len, modbus_request_t *req)
{
    memset(req, 0, sizeof(*req));
    req->slave = buf[0];
    req->func = buf[1];
    req->addr = ((uint16_t)buf[2] << 8) | buf[3];
    req->num = ((uint16_t)buf[4] << 8) | buf[5];
    req->value = ((uint16_t)buf[4] << 8) | buf[5];
    if (req->func == MODBUS_FUNC_WRITE_COILS || req->func == MODBUS_FUNC_WRITE_REGS) {
        if (len >= 7) {
            req->data_len = buf[6];
            if (req->data_len <= len - 9) {
                memcpy(req->data, buf + 7, req->data_len);
            }
        }
    }
    req->raw_len = len;
    memcpy(req->raw, buf, len);
}

size_t modbus_scan_request(const uint8_t *buf, size_t len, modbus_request_t *req)
{
    for (size_t i = 0; i + 4 <= len; i++) {
        size_t need = request_len_at(buf, i, len);
        if (need == 0 || i + need > len) {
            continue;
        }
        if (modbus_check_crc(buf + i, need)) {
            parse_request_at(buf + i, need, req);
            return need;
        }
    }
    return 0;
}

size_t modbus_build_read_response(uint8_t *dst, uint8_t slave, uint8_t func,
                                  const uint8_t *data, size_t data_len)
{
    dst[0] = slave;
    dst[1] = func;
    dst[2] = (uint8_t)data_len;
    memcpy(dst + 3, data, data_len);
    modbus_append_crc(dst, 3 + data_len);
    return 3 + data_len + 2;
}

size_t modbus_build_write_response(uint8_t *dst, const uint8_t *request)
{
    memcpy(dst, request, 6);
    modbus_append_crc(dst, 6);
    return 8;
}

size_t modbus_build_exception(uint8_t *dst, uint8_t slave, uint8_t func, uint8_t code)
{
    dst[0] = slave;
    dst[1] = (uint8_t)(func | 0x80);
    dst[2] = code;
    modbus_append_crc(dst, 3);
    return 5;
}

size_t modbus_pack_regs(uint8_t *dst, const uint16_t *values, size_t count)
{
    for (size_t i = 0; i < count; i++) {
        dst[i * 2] = (uint8_t)(values[i] >> 8);
        dst[i * 2 + 1] = (uint8_t)(values[i] & 0xFF);
    }
    return count * 2;
}
