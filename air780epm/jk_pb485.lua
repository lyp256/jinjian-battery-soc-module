--[[
@module  jk_pb485
@summary 极空 PB/BD 系列 RS485 私有 300 字节实时帧采集（RS485-3 / UART3）

部分较新极空固件在 115200 RS485 上不再返回完整的 0x128A 汇总 Modbus 块，
而是由主站写 0x1620 选择 BMS 后，主动发送 300 字节：

  55 AA EB 90 | 类型 | 0x00 | 数据... | SUM8

类型 0x02 为实时电芯信息，字段地址与 Modbus 的 0x1200 区一致：

  数据区偏移 0x00   单体电压（UINT16 LE，mV）
  数据区偏移 0x8A   MOS 温度（INT16 LE，0.1℃）
  数据区偏移 0x90   总压（UINT32 LE，mV）
  数据区偏移 0x98   电流（INT32 LE，mA，充电为正）
  数据区偏移 0x9C/9E 电池温度 1/2（INT16 LE，0.1℃）
  数据区偏移 0xA0   报警位图（UINT32 LE）
  数据区偏移 0xA6   [高字节]SOC / [低字节]均衡状态
  数据区偏移 0xA8   剩余容量（INT32 LE，mAh）
  数据区偏移 0xAC   实际容量（UINT32 LE，mAh）
  数据区偏移 0xB0   循环次数（UINT32 LE）
  数据区偏移 0xB4   循环总容量（UINT32 LE，mAh）
  数据区偏移 0xB8   [低字节]SOH / [高字节]预充状态
  数据区偏移 0xBC   运行时间（UINT32 LE，秒）
  数据区偏移 0xC0   [低字节]充电状态 / [高字节]放电状态
  数据区偏移 0xC2   用户报警 2（UINT16 LE）

其中“数据区偏移”从帧的第 7 个字节开始。校验是前 299 字节逐字节求和取低 8 位。
]]

local cfg = require "config"
local uart_pins = require "uart_pins"
local M = {}

local MAX_CELLS = 32
M.MAX_CELLS = MAX_CELLS

local SLAVE = cfg.JK_MODBUS_SLAVE_ADDR or 1
local UART_ID = cfg.JK_UART_ID
local FRAME_SIZE = 300
local PREAMBLE = "\85\170\235\144" -- 55 AA EB 90

local function d_info(...) if cfg.LOG_ENABLE then log.info("jkpb", ...) end end
local function d_warn(...) if cfg.LOG_ENABLE then log.warn("jkpb", ...) end end

local function now_ms()
    if mcu and mcu.ticks and mcu.hz then
        return math.floor(mcu.ticks() * 1000 / mcu.hz())
    end
    return os.time() * 1000
end

local function u16le(s, pos)
    return (s:byte(pos) or 0) | ((s:byte(pos + 1) or 0) << 8)
end

local function s16le(s, pos)
    local v = u16le(s, pos)
    return v >= 0x8000 and v - 0x10000 or v
end

local function u32le(s, pos)
    return u16le(s, pos) | (u16le(s, pos + 2) << 16)
end

local function s32le(s, pos)
    local v = u32le(s, pos)
    return v >= 0x80000000 and v - 0x100000000 or v
end

local function tenths_to_c(v)
    if v >= 0 then return (v + 5) // 10 end
    return -((-v + 5) // 10)
end

local function crc16(s)
    local crc = 0xFFFF
    for i = 1, #s do
        crc = crc ~ s:byte(i)
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
M.crc16 = crc16

-- 写 0x1620 选择 0 号 BMS；设备随后在 RS485 上发送 55AAEB90 实时帧。
function M.build_select_request()
    local body = string.char(SLAVE, 0x10, 0x16, 0x20, 0x00, 0x01, 0x02, 0x00, 0x00)
    local crc = crc16(body)
    return body .. string.char(crc & 0xFF, (crc >> 8) & 0xFF)
end

local A_WIRE       = 0x000001
local A_MOS_OT     = 0x000002
local A_CELL_COUNT = 0x000004
local A_CELL_OV    = 0x000010
local A_BAT_OV     = 0x000020
local A_CHG_OCP    = 0x000040
local A_CHG_SCP    = 0x000080
local A_CHG_OTP    = 0x000100
local A_CHG_UTP    = 0x000200
local A_AUX_COMM   = 0x000400
local A_CELL_UV    = 0x000800
local A_BAT_UV     = 0x001000
local A_DCH_OCP    = 0x002000
local A_DCH_SCP    = 0x004000
local A_DCH_OTP    = 0x008000

local function field_pos(reg, shift)
    -- 帧头 6 字节；reg 相对实时数据区 0x1200
    return 7 + shift + reg
end

local function read_cells(frame, shift, slots)
    local cells, count, sum, max_mv, min_mv = {}, 0, 0, 0, 0xFFFF
    for i = 1, MAX_CELLS do
        local mv = i <= slots and u16le(frame, 7 + shift + (i - 1) * 2) or 0
        if mv >= cfg.JK_CELL_MIN_MV and mv <= 6000 then
            cells[i] = mv
            count = i
            sum = sum + mv
            if mv > max_mv then max_mv = mv end
            if mv < min_mv then min_mv = mv end
        else
            cells[i] = 0
        end
    end
    return cells, count, sum, max_mv, min_mv
end

local function decode_layout(frame, shift, slots)
    local cells, count, sum, max_mv, min_mv = read_cells(frame, shift, slots)
    local total_mv = u32le(frame, field_pos(0x90, shift))
    if count == 0 or total_mv < 1000 or total_mv > 200000 then
        return nil
    end
    local tolerance = math.max(2000, count * 150)
    if math.abs(total_mv - sum) > tolerance then
        return nil
    end
    if min_mv == 0xFFFF then min_mv = 0 end
    return cells, count, sum, total_mv, max_mv, min_mv
end

local stats = {
    frames = 0,
    rounds = 0,
    requests = 0,
    timeouts = 0,
    rx_bytes = 0,
    tx_bytes = 0,
    bad_checksum = 0,
    resync = 0,
    ignored_frames = 0,
    last_error = nil,
    last_round_ms = nil,
}

local state = nil
local rxb = ""

local function build_state(frame)
    local layout
    local cells, count, sum, total_mv, max_mv, min_mv
    for _, candidate in ipairs({ { shift = 0, slots = 24 }, { shift = 32, slots = 32 } }) do
        local c, n, s, v, hi, lo = decode_layout(frame, candidate.shift, candidate.slots)
        if c then
            layout, cells, count, sum, total_mv, max_mv, min_mv =
                candidate, c, n, s, v, hi, lo
            break
        end
    end
    if not layout then
        stats.last_error = "invalid frame layout/total voltage"
        return nil
    end

    local shift = layout.shift
    local current_ma = s32le(frame, field_pos(0x98, shift))
    local temp_mos = s16le(frame, field_pos(0x8A, shift))
    local temp1 = s16le(frame, field_pos(0x9C, shift))
    local temp2 = s16le(frame, field_pos(0x9E, shift))
    local alarm = u32le(frame, field_pos(0xA0, shift))
    local balan_ma = s16le(frame, field_pos(0xA4, shift))
    local packed_soc = u16le(frame, field_pos(0xA6, shift))
    local soc = (packed_soc >> 8) & 0xFF
    local balan = packed_soc & 0xFF
    local cap_remain = s32le(frame, field_pos(0xA8, shift))
    local full_cap = u32le(frame, field_pos(0xAC, shift))
    local cycles = u32le(frame, field_pos(0xB0, shift))
    local cycle_cap = u32le(frame, field_pos(0xB4, shift))
    local packed_soh = u16le(frame, field_pos(0xB8, shift))
    local soh = packed_soh & 0xFF
    local run_time = u32le(frame, field_pos(0xBC, shift))
    local packed_mos = u16le(frame, field_pos(0xC0, shift))
    local charge_mos = packed_mos & 0xFF
    local dis_mos = (packed_mos >> 8) & 0xFF
    local user_alarm2 = u16le(frame, field_pos(0xC2, shift))

    local function has(mask) return (alarm & mask) ~= 0 end
    local s = {
        ts = os.time(),
        valid = true,
        stale = false,
        source = "pb485",
        modbusComplete = true,
        cellCount = count,
        cells = cells,
        batVolMv = total_mv,
        batCurrentMa = current_ma,
        soc = soc > 100 and 100 or soc,
        maxDiffMv = max_mv - min_mv,
        tempMosC = tenths_to_c(temp_mos),
        tempBatC = tenths_to_c(temp1),
        tempBat2C = tenths_to_c(temp2),
        warn = alarm ~= 0 and 1 or 0,
        balanceSw = balan ~= 0 and 1 or 0,
        chargeMos = charge_mos ~= 0 and 1 or 0,
        dischargeMos = dis_mos ~= 0 and 1 or 0,
        alarms = {
            has(A_CELL_UV) and 1 or 0, has(A_CELL_OV) and 1 or 0,
            (has(A_CHG_OCP) or has(A_DCH_OCP)) and 1 or 0,
            has(A_MOS_OT) and 1 or 0,
            (has(A_CHG_OTP) or has(A_DCH_OTP)) and 1 or 0,
            (has(A_CHG_SCP) or has(A_DCH_SCP)) and 1 or 0,
            has(A_AUX_COMM) and 1 or 0, has(A_WIRE) and 1 or 0,
            has(A_CELL_COUNT) and 1 or 0,
        },
        sysAlarm = alarm,
        sysAlarmValid = true,
        capRemainMah = cap_remain,
        fullCapMah = full_cap,
        balanCurrentMa = balan_ma,
        cycleCount = cycles > 0xFFFF and 0xFFFF or cycles,
        cycleCapMah = cycle_cap,
        bmsUpTimeS = run_time,
        soh = soh,
        cellMaxMv = max_mv,
        cellMinMv = min_mv,
        userAlarm2 = user_alarm2,
        powerW01 = math.floor(total_mv * current_ma / 100000 + 0.5),
    }
    state = s
    stats.rounds = stats.rounds + 1
    stats.last_round_ms = now_ms()
    stats.last_error = nil
    d_info("round", stats.rounds, "cell", count,
        string.format("%.3fV %+.3fA soc=%d%%", total_mv / 1000, current_ma / 1000, soc),
        "tBat", s.tempBatC, "tMos", s.tempMosC,
        "cap", full_cap, "cyc", s.cycleCount,
        alarm ~= 0 and string.format("alarm=0x%08X", alarm) or "alarm=none")
    sys.publish("JK_ROUND", s)
    return s
end

local function extract_frames()
    while true do
        local start = rxb:find(PREAMBLE, 1, true)
        if not start then
            if #rxb > 3 then
                stats.resync = stats.resync + #rxb - 3
                rxb = rxb:sub(-3)
            end
            return
        end
        if start > 1 then
            stats.resync = stats.resync + start - 1
            rxb = rxb:sub(start)
        end
        if #rxb < FRAME_SIZE then return end

        local frame = rxb:sub(1, FRAME_SIZE)
        local sum = 0
        for i = 1, FRAME_SIZE - 1 do sum = (sum + frame:byte(i)) & 0xFF end
        if sum ~= frame:byte(FRAME_SIZE) then
            stats.bad_checksum = stats.bad_checksum + 1
            stats.last_error = string.format("checksum 0x%02X != 0x%02X",
                sum, frame:byte(FRAME_SIZE))
            rxb = rxb:sub(2)
        else
            rxb = rxb:sub(FRAME_SIZE + 1)
            local frame_type = frame:byte(5)
            if frame_type == 0x02 then
                stats.frames = stats.frames + 1
                build_state(frame)
                sys.publish("JK_PB_RX", state)
            else
                stats.ignored_frames = stats.ignored_frames + 1
            end
        end
    end
end

local function on_receive(id)
    local data = ""
    repeat
        local s = uart.read(id, 512)
        if s and #s > 0 then
            stats.rx_bytes = stats.rx_bytes + #s
            data = data .. s
        end
    until not s or #s == 0
    rxb = rxb .. data
    local limit = cfg.JK_PB485_RX_BUFF_SIZE or 1024
    if #rxb > limit then
        stats.resync = stats.resync + #rxb - limit
        rxb = rxb:sub(-limit)
    end
    extract_frames()
end

local opened = false
local running = true

local function open_uart()
    if opened then return true end
    if not uart or not uart.setup then return false end
    uart_pins.mux(UART_ID, cfg.JK_PIN_RX, cfg.JK_PIN_TX, "jkpb")
    local dir = cfg.JK_RS485_DIR_GPIO or 0xFFFFFFFF
    if not uart.setup(UART_ID, cfg.JK_MODBUS_BAUD, 8, 1, uart.NONE,
            uart.LSB, cfg.JK_PB485_RX_BUFF_SIZE or 1024, dir,
            cfg.JK_RS485_RX_LEVEL, cfg.JK_RS485_DELAY_US) then
        return false
    end
    uart.on(UART_ID, "receive", on_receive)
    opened = true
    d_info("uart", UART_ID, "opened", cfg.JK_MODBUS_BAUD,
        "8N1 (PB485 stream), slave addr", SLAVE)
    return true
end

function M.stop() running = false end
function M.feed(data) rxb = rxb .. data; extract_frames() end
function M.get_state() return state end
function M.get_stats() return stats end
function M.is_fresh() return state ~= nil and not state.stale end

sys.taskInit(function()
    while running do
        if not opened and not open_uart() then
            d_warn("uart", UART_ID, "open failed, retry")
            sys.wait(1000)
        else
            local poll_ms = cfg.JK_PB485_POLL_MS or 800
            local timeout_ms = cfg.JK_PB485_TIMEOUT_MS or 500
            local retry_ms = cfg.JK_MODBUS_RETRY_MS or 2000
            local req = M.build_select_request()
            rxb = ""
            stats.requests = stats.requests + 1
            stats.tx_bytes = stats.tx_bytes + #req
            uart.write(UART_ID, req)
            local ok = sys.waitUntil("JK_PB_RX", timeout_ms)
            if ok then
                if state and not state.stale and stats.last_round_ms and
                   now_ms() - stats.last_round_ms > cfg.JK_DATA_TIMEOUT_MS then
                    state.stale = true
                end
                sys.wait(poll_ms)
            else
                stats.timeouts = stats.timeouts + 1
                stats.last_error = "no 0x55AAEB90 type 0x02 frame"
                if state and not state.stale and stats.last_round_ms and
                   now_ms() - stats.last_round_ms > cfg.JK_DATA_TIMEOUT_MS then
                    state.stale = true
                    d_warn("PB485 data timeout, last round", state.ts)
                end
                d_warn("trigger timeout, retry in", retry_ms, "ms")
                sys.wait(retry_ms)
            end
        end
    end
end)

return M
