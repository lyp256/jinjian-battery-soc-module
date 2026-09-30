--[[
@module  jinjian_slave
@summary 金箭中控 Modbus RTU 从机（RS4852 / 串口2）
@version 1.0
@date    2026.09.28
@usage
金箭中控/仪表作为 Modbus 主站，9600 8N1 周期轮询本模块（从站地址 1）。
协议依据 docs/jinjian/02_BMS_PROTOCOL.md 与 docs/jinjian/03_BMS_COLLECTION_REPORT.md，
寄存器 ← 极空数据的映射与 ESP32 侧实现保持一致：

  寄存器(03)：
    0     总电压     极空总压 mV / 10          → 0.01V
    1     电芯串数   极空广播识别出的串数
    2     SOC        极空 SOC，clamp 0-100
    3     容量       估算总容量 mAh / 1000      → Ah
    4     短路保护   极空短路报警位
    5     充电电流   极空电流 mA / 10           → 0.01A，充电为正
    6/7   温度2/温度1 极空电池温度（整数 ℃）
    8     板温       极空 MOS 温度（整数 ℃）
    9-32  单体电压   极空单体电压 mV
    103   电池类型   固定 0（锂电）
    104   循环次数   本地缓存（极空广播无此数据）
    110   均衡状态   极空均衡开关状态
    113   标称容量   估算总容量 Ah
    1000-1007 PN     模块自身 PN（SOC-XXXXXXXXXXXX，16 ASCII）
    1016/1017 版本   脚本 VERSION 解析为 a高.a低 / b高.b低
    1089  充电时间   本地缓存（默认 120 分钟），可被主站写入
    1090  目标SOC    本地缓存（默认 90%），可被主站写入
  开关量(01)：4 短路 / 5 过温充电 / 6 过温放电 / 7 低温充电 / 8 低温放电 / 63 快充状态
  写入：06 写 0x0441(充电时间) / 0x0442(目标SOC)；05 写 0x0040 FF00(结束快充)

对外接口：
  jinjian_slave.handle_frame(frame)  离线测试用：喂入请求帧返回响应帧（或 nil）
  jinjian_slave.get_stats()          从机统计
  jinjian_slave.get_cache()          本地缓存（充电设置等）
]]

local cfg = require "config"
local jk = require "jk_display"
local board = require "board"

local M = {}

local SLAVE = cfg.JJ_SLAVE_ADDR

-- 金箭寄存器 9~32 只有 24 路单体；极空若为 ≥25 串，车端链路只报前 24 节
-- （第 25 节仍会进入 MQTT 上报的压缩数据）
local JJ_MAX_CELLS = 24

local function d_info(...) if cfg.LOG_ENABLE then log.info("jj", ...) end end
local function d_warn(...) if cfg.LOG_ENABLE then log.warn("jj", ...) end end
local function d_debug(...) if cfg.LOG_RAW_FRAMES then log.info("jj.raw", ...) end end

--=============================================================================
-- Modbus CRC16（0xA001 多项式，低字节在前）
--=============================================================================
local function crc16(s)
    local crc = 0xFFFF
    for i = 1, #s do
        crc = crc ~ string.byte(s, i)
        for _ = 1, 8 do
            if (crc & 1) ~= 0 then
                crc = (crc >> 1) ~ 0xA001
            else
                crc = crc >> 1
            end
        end
    end
    return crc & 0xFFFF
end

local function append_crc(body)
    local crc = crc16(body)
    return body .. string.char(crc & 0xFF, (crc >> 8) & 0xFF)
end

local function crc_ok(frame)
    if #frame < 4 then
        return false
    end
    local calc = crc16(frame:sub(1, #frame - 2))
    return string.byte(frame, #frame - 1) == (calc & 0xFF)
       and string.byte(frame, #frame) == ((calc >> 8) & 0xFF)
end

-- 十六进制字符串（LuatOS 的 string:toHex() 在 PC Lua 上不存在，这里自备）
local function to_hex(s)
    return (s:gsub(".", function(c) return string.format("%02x", c:byte()) end))
end

--=============================================================================
-- 本地缓存（极空广播里没有的配置项）
--=============================================================================
local cache = {
    chargeTimeMin = cfg.JJ_DEFAULT_CHARGE_TIME_MIN,
    chargeTargetSoc = cfg.JJ_DEFAULT_CHARGE_TARGET_SOC,
    cycleCount = 0,          -- 104 循环次数（无数据源，保持 0，可后续扩展）
    fastChargeEnded = false, -- 主站写过"结束快充"后置位，放电/停充后自动清除
}

local stats = {
    rx_frames = 0,   -- 收到且 CRC 正确的请求帧数
    tx_frames = 0,   -- 发出的响应帧数
    ignored = 0,     -- 从站地址不匹配而忽略的帧数
    errors = {},     -- 各功能码异常次数
    last_error = nil,
}

--=============================================================================
-- 设备标识（PN / 版本寄存器）
--=============================================================================
local pn = "SOC-000000000000"

local function build_pn()
    if board and board.pn then
        pn = board.pn()
    end
end

local version_regs = { 0x0100, 0x0000 }

local function build_version()
    local text = tostring(_G.VERSION or "1.0.0")
    local comp, val, have = {}, 0, false
    for i = 1, #text do
        local c = string.byte(text, i)
        if c >= 0x30 and c <= 0x39 then
            val = val * 10 + (c - 0x30)
            if val > 255 then val = 255 end
            have = true
        elseif have then
            comp[#comp + 1] = val
            val, have = 0, false
        end
        if #comp >= 4 then break end
    end
    if have and #comp < 4 then comp[#comp + 1] = val end
    while #comp < 4 do comp[#comp + 1] = 0 end
    version_regs = { (comp[1] << 8) | comp[2], (comp[3] << 8) | comp[4] }
end

--=============================================================================
-- 金箭寄存器 ← 极空数据
--=============================================================================
local function to_i16(v)
    v = math.floor(v + 0.5) & 0xFFFF
    if v >= 0x8000 then
        return v - 0x10000
    end
    return v
end

-- sysAlarm 位表（手册 §11.3，与 BLE 错误位域互证）
local SA_CHG_SHORT  = 0x0080 -- bit7  充电短路
local SA_DCH_SHORT  = 0x4000 -- bit14 放电短路
local SA_CHG_OVER_T = 0x0100 -- bit8  充电过温
local SA_CHG_LOW_T  = 0x0200 -- bit9  充电低温
local SA_DCH_OVER_T = 0x8000 -- bit15 放电过温
local SA_MOS_OVER_T = 0x0002 -- bit1  MOS 过温

-- 保护状态：[1] 短路 [2] 过温充电 [3] 过温放电 [4] 低温充电 [5] 低温放电
-- 优先用完整 sysAlarm 位图；本轮缺扩展块时退回 9 个图标字段 + 温度阈值
local function protection_bits()
    local s = jk.get_state()
    local bits = { 0, 0, 0, 0, 0 }
    if not s then
        return bits
    end
    local a = s.alarms or {}
    local short_circuit, over_temp_charge, over_temp_discharge, low_temp_charge
    if s.sysAlarmValid then
        local alarm = s.sysAlarm or 0
        local function has(mask)
            return (alarm & mask) ~= 0
        end
        short_circuit = has(SA_CHG_SHORT) or has(SA_DCH_SHORT)
        over_temp_charge = has(SA_CHG_OVER_T) or has(SA_MOS_OVER_T)
        over_temp_discharge = has(SA_DCH_OVER_T) or has(SA_MOS_OVER_T)
        low_temp_charge = has(SA_CHG_LOW_T)
    else
        -- 图标字段：偏移 78=MOS 过温、80=电池温度类、82=短路
        short_circuit = (a[6] == 1)
        over_temp_charge = (a[4] == 1) or (a[5] == 1)
        over_temp_discharge = over_temp_charge
        low_temp_charge = false
    end
    bits[1] = short_circuit and 1 or 0
    bits[2] = over_temp_charge and 1 or 0
    bits[3] = over_temp_discharge and 1 or 0
    -- 低温充电：位图有专用位；无位图时用温度阈值兜底
    bits[4] = (low_temp_charge or s.tempBatC <= cfg.JJ_LOW_TEMP_CHARGE_C) and 1 or 0
    -- 低温放电：位表里没有专用位，仍由温度阈值推导
    bits[5] = (s.tempBatC <= cfg.JJ_LOW_TEMP_DISCHARGE_C) and 1 or 0
    return bits
end

-- 是否处于快充（充电中且电流达到阈值，且主站没有下过"结束快充"）
local function fast_charging()
    local s = jk.get_state()
    if not s then
        return 0
    end
    if s.batCurrentMa <= 0 then
        cache.fastChargeEnded = false -- 停充/放电后允许下一次充电重新判定快充
        return 0
    end
    if cache.fastChargeEnded then
        return 0
    end
    return (s.batCurrentMa >= cfg.JJ_FAST_CHARGE_CURRENT_MA) and 1 or 0
end

-- 读单个寄存器；返回 nil 表示地址非法（异常码 02）
local function reg_value(addr)
    local s = jk.get_state()
    local cellCount = math.min(s and s.cellCount or 0, JJ_MAX_CELLS)

    if addr == 0 then                                     -- 总电压 0.01V
        return s and math.floor(s.batVolMv / 10 + 0.5) or 0
    elseif addr == 1 then                                 -- 串数
        return cellCount
    elseif addr == 2 then                                 -- SOC
        local soc = s and s.soc or 0
        if soc < 0 then soc = 0 elseif soc > 100 then soc = 100 end
        return soc
    elseif addr == 3 then                                 -- 容量 Ah
        return s and math.floor(s.fullCapMah / 1000 + 0.5) or 0
    elseif addr == 4 then                                 -- 短路保护（模拟量块）
        return protection_bits()[1]
    elseif addr == 5 then                                 -- 充电电流 0.01A
        if not s then return 0 end
        return to_i16(s.batCurrentMa / 10)
    elseif addr == 6 or addr == 7 then                    -- 电池温度 2 / 1
        return to_i16(s and s.tempBatC or 0)
    elseif addr == 8 then                                 -- 板温
        return to_i16(s and s.tempMosC or 0)
    elseif addr >= 9 and addr <= 32 then                  -- 单体电压 mV
        local i = addr - 8
        if s and i <= cellCount then
            return s.cells[i] or 0
        end
        return 0
    elseif addr == 103 then                               -- 电池类型
        return cfg.JJ_BATTERY_TYPE
    elseif addr == 104 then                               -- 循环次数
        return cache.cycleCount
    elseif addr == 110 then                               -- 均衡状态
        return (s and s.balanceSw == 1) and 1 or 0
    elseif addr == 113 then                               -- 标称容量 Ah
        return s and math.floor(s.fullCapMah / 1000 + 0.5) or 0
    elseif addr >= 105 and addr <= 109 or addr == 111 or addr == 112 then
        return 0
    elseif addr >= 1000 and addr <= 1007 then             -- PN（每寄存器 2 ASCII）
        local i = (addr - 1000) * 2 + 1
        return (string.byte(pn, i) << 8) | string.byte(pn, i + 1)
    elseif addr == 1016 then
        return version_regs[1]
    elseif addr == 1017 then
        return version_regs[2]
    elseif addr == 1089 then                              -- 充电时间 min
        return cache.chargeTimeMin
    elseif addr == 1090 then                              -- 目标 SOC %
        return cache.chargeTargetSoc
    end
    return nil
end

-- 读开关量：[addr] -> 0/1
local function coil_value(addr)
    if addr >= 4 and addr <= 8 then
        return protection_bits()[addr - 3]
    elseif addr == 63 then
        return fast_charging()
    end
    return 0
end

--=============================================================================
-- 响应组帧
--=============================================================================
local function build_read_response(func, payload)
    return append_crc(string.char(SLAVE, func, #payload) .. payload)
end

local function build_exception(func, code)
    local key = string.format("%02X/%02X", func, code)
    stats.errors[key] = (stats.errors[key] or 0) + 1
    stats.last_error = string.format("func 0x%02X exception 0x%02X", func, code)
    return append_crc(string.char(SLAVE, (func | 0x80) & 0xFF, code))
end

local function build_write_echo(frame)
    return append_crc(frame:sub(1, 6))
end

--=============================================================================
-- 请求处理
--=============================================================================
local FUNC_READ_COILS = 0x01
local FUNC_READ_REGS = 0x03
local FUNC_WRITE_COIL = 0x05
local FUNC_WRITE_REG = 0x06
local FUNC_WRITE_COILS = 0x0F
local FUNC_WRITE_REGS = 0x10

local EXC_ILLEGAL_FUNCTION = 0x01
local EXC_ILLEGAL_ADDRESS = 0x02
local EXC_ILLEGAL_VALUE = 0x03

local function handle_read_regs(func, start, num)
    if num == 0 or num > 125 then
        return build_exception(func, EXC_ILLEGAL_VALUE)
    end
    local parts = {}
    for i = 0, num - 1 do
        local v = reg_value(start + i)
        if v == nil then
            return build_exception(func, EXC_ILLEGAL_ADDRESS)
        end
        v = v & 0xFFFF
        parts[#parts + 1] = string.char((v >> 8) & 0xFF, v & 0xFF)
    end
    return build_read_response(func, table.concat(parts))
end

local function handle_read_coils(func, start, num)
    if num == 0 or num > 64 then
        return build_exception(func, EXC_ILLEGAL_VALUE)
    end
    local bytes = {}
    for i = 0, num - 1 do
        if coil_value(start + i) == 1 then
            local idx = (i >> 3) + 1
            bytes[idx] = (bytes[idx] or 0) | (1 << (i & 7))
        end
    end
    local payload = {}
    for i = 1, (num + 7) // 8 do
        payload[i] = string.char(bytes[i] or 0)
    end
    return build_read_response(func, table.concat(payload))
end

local function handle_write_reg(func, addr, value)
    if addr == 0x0441 then                      -- 充电时间 0-120 分钟
        if value > 120 then
            return build_exception(func, EXC_ILLEGAL_VALUE)
        end
        cache.chargeTimeMin = value
        d_info("write charge time", value, "min")
        return true
    elseif addr == 0x0442 then                  -- 目标 SOC 0-100
        if value > 100 then
            return build_exception(func, EXC_ILLEGAL_VALUE)
        end
        cache.chargeTargetSoc = value
        d_info("write charge target soc", value, "%")
        return true
    end
    return build_exception(func, EXC_ILLEGAL_ADDRESS)
end

local function handle_write_coil(func, addr, value)
    if addr == 0x0040 and (value == 0xFF00 or value == 0x0000) then
        if value == 0xFF00 then
            cache.fastChargeEnded = true
            d_info("end fast charge requested")
        else
            cache.fastChargeEnded = false
        end
        return true
    end
    return build_exception(func, EXC_ILLEGAL_ADDRESS)
end

-- 处理一帧请求，返回响应帧字符串；nil 表示无需响应
local function handle_request(frame)
    stats.rx_frames = stats.rx_frames + 1
    local slave = string.byte(frame, 1)
    if slave ~= SLAVE then
        stats.ignored = stats.ignored + 1
        return nil
    end

    local func = string.byte(frame, 2)
    d_debug("RX", to_hex(frame))

    local resp
    if func == FUNC_READ_REGS then
        local start = (string.byte(frame, 3) << 8) | string.byte(frame, 4)
        local num = (string.byte(frame, 5) << 8) | string.byte(frame, 6)
        resp = handle_read_regs(func, start, num)
    elseif func == FUNC_READ_COILS then
        local start = (string.byte(frame, 3) << 8) | string.byte(frame, 4)
        local num = (string.byte(frame, 5) << 8) | string.byte(frame, 6)
        resp = handle_read_coils(func, start, num)
    elseif func == FUNC_WRITE_REG then
        local addr = (string.byte(frame, 3) << 8) | string.byte(frame, 4)
        local value = (string.byte(frame, 5) << 8) | string.byte(frame, 6)
        resp = handle_write_reg(func, addr, value)
        if resp == true then resp = build_write_echo(frame) end
    elseif func == FUNC_WRITE_COIL then
        local addr = (string.byte(frame, 3) << 8) | string.byte(frame, 4)
        local value = (string.byte(frame, 5) << 8) | string.byte(frame, 6)
        resp = handle_write_coil(func, addr, value)
        if resp == true then resp = build_write_echo(frame) end
    else
        resp = build_exception(func, EXC_ILLEGAL_FUNCTION)
    end

    if resp then
        stats.tx_frames = stats.tx_frames + 1
    end
    return resp
end

--=============================================================================
-- 帧扫描：从缓冲区里找 CRC 正确的完整请求帧（与 ESP32 侧逻辑一致）
--=============================================================================
local function request_len_at(buf, off) -- off 为 1 基下标
    if #buf - off + 1 < 2 then
        return 0
    end
    local func = string.byte(buf, off + 1)
    if func == FUNC_READ_COILS or func == FUNC_READ_REGS or
       func == FUNC_WRITE_COIL or func == FUNC_WRITE_REG then
        return 8
    elseif func == FUNC_WRITE_COILS or func == FUNC_WRITE_REGS then
        if #buf - off + 1 < 7 then
            return 0
        end
        return 9 + string.byte(buf, off + 6)
    end
    -- 未知功能码也按最短 8 字节尝试（会被功能码异常分支拒绝）
    return 8
end

local function scan_frame(buf)
    for off = 1, #buf - 3 do
        local need = request_len_at(buf, off)
        if need > 0 and off + need - 1 <= #buf then
            local frame = buf:sub(off, off + need - 1)
            if crc_ok(frame) then
                return frame, off, need
            end
        end
    end
    return nil
end

--=============================================================================
-- 串口接收
--=============================================================================
local rx = ""

local function on_receive(id)
    local s = ""
    repeat
        s = uart.read(id, 128)
        if s and #s > 0 then
            rx = rx .. s
        end
    until not s or #s == 0

    if #rx > cfg.JJ_RX_BUFF_SIZE then
        d_warn("rx overflow, drop", #rx)
        rx = rx:sub(-cfg.JJ_MAX_FRAME_LEN)
    end

    while #rx >= 4 do
        local frame, off, need = scan_frame(rx)
        if not frame then
            -- 没有完整帧：只保留可能构成帧的尾部
            if #rx > cfg.JJ_MAX_FRAME_LEN then
                rx = rx:sub(-cfg.JJ_MAX_FRAME_LEN)
            end
            return
        end
        if off > 1 then
            d_debug("drop", off - 1, "bytes before frame")
        end
        rx = rx:sub(off + need)
        local resp = handle_request(frame)
        if resp then
            uart.write(cfg.JJ_UART_ID, resp)
            d_debug("TX", to_hex(resp))
        end
    end
end

local opened = false

build_pn()
build_version()

sys.taskInit(function()
    local dir = cfg.JJ_RS485_DIR_GPIO or 0xFFFFFFFF
    while true do
        if not opened and uart and uart.setup then
            if uart.setup(cfg.JJ_UART_ID, cfg.JJ_BAUD, 8, 1, uart.NONE,
                    uart.LSB, cfg.JJ_RX_BUFF_SIZE, dir,
                    cfg.JJ_RS485_RX_LEVEL, cfg.JJ_RS485_DELAY_US) then
                uart.on(cfg.JJ_UART_ID, "receive", on_receive)
                opened = true
                d_info("modbus rtu slave ready, uart", cfg.JJ_UART_ID,
                    cfg.JJ_BAUD, "8N1, addr", SLAVE, "PN", pn,
                    string.format("version %04X %04X", version_regs[1], version_regs[2]))
            else
                d_warn("uart", cfg.JJ_UART_ID, "open failed, retry")
            end
        end
        sys.wait(1000)
    end
end)

--=============================================================================
-- 对外接口
--=============================================================================

-- 离线测试：直接处理一帧请求（不经过串口）
function M.handle_frame(frame)
    return handle_request(frame)
end

function M.get_stats()
    return stats
end

function M.get_cache()
    return cache
end

M.crc16 = crc16 -- 供测试比对

return M
