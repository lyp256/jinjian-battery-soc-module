--[[
@module  jk_modbus
@summary 极空(JK) BMS RS485 Modbus 通用协议主站采集（RS485-3 / UART3）
@version 1.0
@date    2026.10.08
@usage
协议依据 docs/jikong/JK-BMS-RS485.md（极空 BMS RS485 Modbus 通用协议 V1.1）：
  115200 8N1、半双工、主从应答；本模块是主机（Master），极空保护板是从机（Slave）。
  请求帧：地址码(1) 功能码(1) 起始寄存器(2) 寄存器数量(2) CRC(2)
  响应帧：地址码(1) 功能码(1) 字节数(1) 数据(N) CRC(2)，CRC 低字节在前
  功能码 03H 读保持寄存器（本模块只读，不改保护板配置）。

  寄存器地址约定（与 esp32/main/jk_bms.c 保持一致，两套固件取到同一份数据）：
    只读实时区 基地址 0x1200：
      0x1200        单体电压 ×32（UINT16，mV，连续 32 个寄存器）
      0x1240        CellSta 电池状态位图（UINT32，BIT[n]=1 表示第 n+1 节存在）
      0x128A        TempMos 功率板温度（INT16，0.1℃）
      0x1290        BatVol 电池总电压（UINT32，mV）
      0x1298        BatCurrent 电池电流（INT32，mA，充电为正）
      0x129C/0x129E TempBat1/TempBat2 电池温度（INT16，0.1℃）
      0x12A0        报警位域 Alarm（UINT32）
      0x12A4        BalanCurrent 均衡电流（INT16，mA）
      0x12A6        [高字节]BalanSta 均衡状态 / [低字节]SOC 剩余电量
      0x12A8        SOCCapRemain 剩余容量（INT32，mAh）
      0x12AC        SOCFullChargeCap 电池实际容量（UINT32，mAh）
      0x12B0        SOCCycleCount 循环次数（UINT32）
      0x12B4        SOCCycleCap 循环总容量（UINT32，mAh）
      0x12B8        [高字节]SOCSOH / [低字节]Precharge
      0x12BC        RunTime BMS 运行时间（UINT32，秒）
      0x12C0        [高字节]充电状态 / [低字节]放电状态

  注：协议手册 §6.3 的“单体电压”表把偏移写成了字节偏移（0/2/4…），其余表是寄存器偏移，
      统一按“实际地址 = 基地址 + 偏移（寄存器）”解释，故单体电压是 0x1200 起连续的 32 个
      寄存器，CellSta 在 0x1240；与 esp32 侧实现、社区 esphome-jk-bms 实测一致。

对外接口（与 jk_display 一致，由 jk.lua 按 config.JK_PROTOCOL 选择其一）：
  jk_modbus.get_state()   最近一轮解析结果（table），从未收到时返回 nil
  jk_modbus.is_fresh()    最近一轮是否在超时时间内
  jk_modbus.get_stats()   统计信息（rounds/frames/requests/…）
  jk_modbus.MAX_CELLS     协议里单体的最大槽位数（32）
  jk_modbus.build_read_request(start, num)  组读寄存器请求帧（离线测试用）
  jk_modbus.feed(bytes)   注入一段从机响应字节流（离线测试用）
  jk_modbus.stop()        停止轮询（离线测试用）

每完成一轮轮询（单体电压 + 电池状态 + 汇总必需字段全部成功）发布一次 "JK_ROUND"，
汇总可选字段失败时仍发布核心样本，并在 state.modbusComplete 标记不完整；
携带本轮 state，供 bms_uplink 按“一轮一条样本”的粒度入队；state 字段与 jk_display
保持同名字段（batVolMv/batCurrentMa/soc/cells/tempBatC/tempMosC/fullCapMah/…），
并额外提供 modbus 独有字段（tempBat2C/balanCurrentMa/cycleCount/soh/bmsUpTimeS/…）。
]]

local cfg = require "config"
local uart_pins = require "uart_pins"
local M = {}

-- 单体槽位上限（0x1200 起连续 32 个寄存器）
local MAX_CELLS = 32
M.MAX_CELLS = MAX_CELLS

local SLAVE   = cfg.JK_MODBUS_SLAVE_ADDR or 1
local UART_ID = cfg.JK_UART_ID

--=============================================================================
-- 寄存器地址与汇总块下标（与 esp32/main/jk_bms.c 的字段映射一致）
--=============================================================================
local REG_CELLS,     NUM_CELLS     = 0x1200, 32 -- 单体电压 mV
local REG_CELL_STA,  NUM_CELL_STA  = 0x1240, 2  -- CellSta 在位位图 u32

-- 极空的“寄存器地址”实际按字节偏移组织，同一固件只保证单个实体按其自身宽度可读。
-- 实测 0x128A 即使用 x32 也会被静默丢弃；社区 esphome-jk-bms 同样按实体逐个请求
-- （0x128A x1、0x1290 x2、0x1298 x2…），不能把汇总区合并成一个大块。
-- base 仍是相对 0x128A 的 1 基 16-bit 寄存器下标，用来兼容 build_state 的既有映射。
local STAT_READS = {
    { reg = 0x128A, num = 1, base = 1,  required = false, field = "TempMos" },
    { reg = 0x1290, num = 2, base = 7,  required = true,  field = "BatVol" },
    { reg = 0x1298, num = 2, base = 15, required = true,  field = "BatCurrent" },
    { reg = 0x129C, num = 1, base = 19, required = false, field = "TempBat1" },
    { reg = 0x129E, num = 1, base = 21, required = false, field = "TempBat2" },
    { reg = 0x12A0, num = 2, base = 23, required = false, field = "Alarm" },
    { reg = 0x12A4, num = 1, base = 27, required = false, field = "BalanCurrent" },
    { reg = 0x12A6, num = 1, base = 29, required = true,  field = "SOC" },
    { reg = 0x12A8, num = 2, base = 31, required = false, field = "SOCCapRemain" },
    { reg = 0x12AC, num = 2, base = 35, required = false, field = "SOCFullChargeCap" },
    { reg = 0x12B0, num = 2, base = 39, required = false, field = "SOCCycleCount" },
    { reg = 0x12B4, num = 2, base = 43, required = false, field = "SOCCycleCap" },
    { reg = 0x12B8, num = 1, base = 47, required = false, field = "SOCSOH" },
    { reg = 0x12BC, num = 2, base = 51, required = false, field = "RunTime" },
    { reg = 0x12C0, num = 1, base = 55, required = false, field = "ChargeState" },
    { reg = 0x12C2, num = 1, base = 57, required = false, field = "UserAlarm2" },
}

-- 汇总块（0x128A 起）的 1 基寄存器下标
local ST_TEMP_MOS                      = 1      -- 0x128A
local ST_BATVOL_HI,  ST_BATVOL_LO      = 7, 8   -- 0x1290
local ST_CUR_HI,     ST_CUR_LO         = 15, 16 -- 0x1298
local ST_TEMP_BAT1                     = 19     -- 0x129C
local ST_TEMP_BAT2                     = 21     -- 0x129E
local ST_ALARM_HI,   ST_ALARM_LO       = 23, 24 -- 0x12A0
local ST_BALAN_CUR                     = 27     -- 0x12A4
local ST_BAL_SOC                       = 29     -- 0x12A6
local ST_CAPREM_HI,  ST_CAPREM_LO      = 31, 32 -- 0x12A8
local ST_FULLCAP_HI, ST_FULLCAP_LO     = 35, 36 -- 0x12AC
local ST_CYCLE_HI,   ST_CYCLE_LO       = 39, 40 -- 0x12B0
local ST_CYCLE_CAP_HI, ST_CYCLE_CAP_LO = 43, 44 -- 0x12B4
local ST_SOH_PREC                      = 47     -- 0x12B8
local ST_RUNTIME_HI, ST_RUNTIME_LO     = 51, 52 -- 0x12BC BMS 运行时间（秒，u32）
local ST_CHARGE_STA                    = 55     -- 0x12C0
local ST_USER_ALARM2                   = 57     -- 0x12C2

-- 报警位域（手册 §6.3 报警标志位域，UINT32）
local A_WIRE       = 0x000001 -- bit0  均衡线电阻过大
local A_MOS_OT     = 0x000002 -- bit1  MOS 过温保护
local A_CELL_COUNT = 0x000004 -- bit2  单体数量与设置不符
local A_CUR_SENS   = 0x000008 -- bit3  电流传感器异常
local A_CELL_OV    = 0x000010 -- bit4  单体过压保护
local A_BAT_OV     = 0x000020 -- bit5  电池过压保护
local A_CHG_OCP    = 0x000040 -- bit6  充电过流保护
local A_CHG_SCP    = 0x000080 -- bit7  充电短路保护
local A_CHG_OTP    = 0x000100 -- bit8  充电过温保护
local A_CHG_UTP    = 0x000200 -- bit9  充电低温保护
local A_AUX_COMM   = 0x000400 -- bit10 内部通信异常
local A_CELL_UV    = 0x000800 -- bit11 单体欠压保护
local A_BAT_UV     = 0x001000 -- bit12 电池欠压保护
local A_DCH_OCP    = 0x002000 -- bit13 放电过流保护
local A_DCH_SCP    = 0x004000 -- bit14 放电短路保护
local A_DCH_OTP    = 0x008000 -- bit15 放电过温保护

local FUNC_READ_REGS     = 0x03
local FUNC_READ_REGS_ERR = 0x83

local function d_info(...) if cfg.LOG_ENABLE then log.info("jkmb", ...) end end
local function d_warn(...) if cfg.LOG_ENABLE then log.warn("jkmb", ...) end end
local function d_debug(...) if cfg.LOG_RAW_FRAMES then log.info("jkmb.raw", ...) end end

local function to_hex(s)
    return (s:gsub(".", function(c) return string.format("%02x", c:byte()) end))
end

-- 原始帧十六进制限频打印：LOG_RAW_FRAMES 打开时每 LOG_HEX_EVERY_N 帧打一次。
-- 汇总区按实体读取，一轮最多 18 帧；单帧最多 69 字节（138 字符），仍应限频。
local hex_dumps = 0
local function d_hex(tag, s)
    if not cfg.LOG_RAW_FRAMES then
        return
    end
    hex_dumps = hex_dumps + 1
    local every = cfg.LOG_HEX_EVERY_N or 20
    if every > 1 and (hex_dumps - 1) % every ~= 0 then
        return
    end
    log.info("jkmb.raw", tag, to_hex(s))
end

local function now_ms()
    if mcu and mcu.ticks and mcu.hz then
        return math.floor(mcu.ticks() * 1000 / mcu.hz())
    end
    return os.time() * 1000
end

--=============================================================================
-- Modbus CRC16（0xA001 多项式，低字节在前，与 jinjian_slave 一致）
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
M.crc16 = crc16

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

-- 03H 读保持寄存器请求帧（离线测试/调试可复用）
function M.build_read_request(start, num)
    local body = string.char(SLAVE, FUNC_READ_REGS,
        (start >> 8) & 0xFF, start & 0xFF, (num >> 8) & 0xFF, num & 0xFF)
    return append_crc(body)
end

-- 解析 03H 响应帧 → 寄存器数组（1 基）；返回 nil, 原因 表示失败
local function parse_read_response(frame, num)
    if not crc_ok(frame) then
        return nil, "crc"
    end
    if string.byte(frame, 1) ~= SLAVE then
        return nil, "addr"
    end
    local func = string.byte(frame, 2)
    if func == FUNC_READ_REGS_ERR then
        return nil, string.format("exception 0x%02X", string.byte(frame, 3) or 0)
    end
    if func ~= FUNC_READ_REGS then
        return nil, string.format("func 0x%02X", func)
    end
    if string.byte(frame, 3) ~= num * 2 then
        return nil, string.format("bytecount %d", string.byte(frame, 3) or -1)
    end
    local regs = {}
    for i = 0, num - 1 do
        regs[i + 1] = (string.byte(frame, 4 + i * 2) << 8) | string.byte(frame, 5 + i * 2)
    end
    return regs
end

--=============================================================================
-- 统计信息
--=============================================================================
local stats = {
    frames = 0,        -- 收到的合法响应帧数
    rounds = 0,        -- 成功完成的轮询轮数（= 收到极空数据的次数）
    degraded = 0,      -- 成功但缺少可选尾部状态块的轮数
    degraded_streak = 0,
    cycles = 0,        -- 发起过的轮询轮数（含失败）
    cycles_failed = 0, -- 有任一数据块读取失败的轮数
    fail_streak = 0,   -- 连续失败轮数（用于退避与现场判断）
    requests = 0,      -- 发出的读寄存器请求数
    rx_bytes = 0,      -- 串口收到的原始字节数（0 = 保护板没有任何回传）
    tx_bytes = 0,      -- 串口发出的原始字节数
    timeouts = 0,      -- 请求无响应的次数
    late_frames = 0,   -- 没有任何请求在途时收到的帧（迟到/噪声，直接丢弃）
    bad_response = 0,  -- 响应 CRC/地址/字节数非法的次数
    bad_crc = 0,       -- 收帧过程中 CRC 错误次数
    bytecount_mismatch = 0, -- 地址/功能码像响应，但字节数不符合当前请求（常见于 TX 回显）
    resync = 0,        -- 因地址/功能码/字节数不匹配而丢弃的字节数（重新同步）
    last_error = nil,
    last_round_ms = nil, -- 最近一轮成功的 tick(ms)
}

--=============================================================================
-- 串口收发（半双工主站：发请求 → 等应答）
--=============================================================================
local rxb = ""
local awaiting = false -- 只有请求在途时收到的帧才予受理，避免迟到帧被配给下一个请求
local expected_num = nil -- 当前请求期望的寄存器数；用来拒绝上一请求的回显/迟到帧

local function extract_frames()
    local pos = 1
    local hold = nil -- 尾部不完整候选帧的起点；不能在噪声后把它截掉

    while pos <= #rxb do
        local addr = string.byte(rxb, pos)
        local func = string.byte(rxb, pos + 1)

        if addr ~= SLAVE or (func ~= FUNC_READ_REGS and func ~= FUNC_READ_REGS_ERR) then
            pos = pos + 1
        elseif func == FUNC_READ_REGS_ERR then
            if #rxb - pos + 1 < 5 then
                hold = awaiting and pos or nil
                break
            end
            local frame = rxb:sub(pos, pos + 4)
            if crc_ok(frame) then
                stats.resync = stats.resync + (pos - 1)
                rxb = rxb:sub(pos + 5)
                pos = 1
                if awaiting then
                    stats.frames = stats.frames + 1
                    sys.publish("JK_MB_RX", frame)
                else
                    stats.late_frames = stats.late_frames + 1
                end
            else
                stats.bad_crc = stats.bad_crc + 1
                pos = pos + 1
            end
        else
            local byte_count = string.byte(rxb, pos + 2)
            if not byte_count then
                hold = awaiting and pos or nil
                break
            end
            -- 115200 半双工总线有时会把刚发出的请求回显到 RX。请求帧的第三字节
            -- 是起始地址高字节，绝不是当前响应的字节数；按期望长度过滤后再扫，
            -- 可以跨过回显/噪声找到真正的响应，而不是把回显当帧头等超时。
            if expected_num and byte_count ~= expected_num * 2 then
                stats.bytecount_mismatch = stats.bytecount_mismatch + 1
                pos = pos + 1
            elseif byte_count == 0 or (byte_count & 1) ~= 0 or byte_count > 250 then
                stats.resync = stats.resync + 1
                pos = pos + 1
            else
                local total = 5 + byte_count
                if #rxb - pos + 1 < total then
                    hold = awaiting and pos or nil
                    break
                end
                local frame = rxb:sub(pos, pos + total - 1)
                if crc_ok(frame) then
                    stats.resync = stats.resync + (pos - 1)
                    rxb = rxb:sub(pos + total)
                    pos = 1
                    if awaiting then
                        stats.frames = stats.frames + 1
                        sys.publish("JK_MB_RX", frame)
                    else
                        stats.late_frames = stats.late_frames + 1
                    end
                else
                    stats.bad_crc = stats.bad_crc + 1
                    pos = pos + 1
                end
            end
        end
    end

    if hold then
        stats.resync = stats.resync + (hold - 1)
        rxb = rxb:sub(hold)
    elseif pos > #rxb then
        stats.resync = stats.resync + (pos - 1)
        rxb = ""
    end
end

-- 串口接收回调：搬字节 + 切出完整响应帧（只做定界与 CRC，单帧 ≤ 245 字节、可在
-- 常数时间内完成）。这里保留在回调内是有意为之——响应帧紧接着请求到达，多一次
-- 任务唤醒会直接影响请求/应答时序；而重活（建表、日志、publish）都在轮询任务里。
local function on_receive(id)
    local s = ""
    repeat
        s = uart.read(id, 256)
        if s and #s > 0 then
            stats.rx_bytes = stats.rx_bytes + #s
            rxb = rxb .. s
        end
    until not s or #s == 0
    if #rxb > cfg.JK_MODBUS_RX_BUFF_SIZE then
        stats.resync = stats.resync + (#rxb - cfg.JK_MODBUS_RX_BUFF_SIZE)
        rxb = rxb:sub(-cfg.JK_MODBUS_RX_BUFF_SIZE)
    end
    extract_frames()
end

-- 发一帧读请求并同步等待应答；成功返回寄存器数组（1 基），失败返回 nil, 原因
local function request_read(start, num)
    rxb = ""
    local req = M.build_read_request(start, num)
    d_hex("TX", req)
    expected_num = num
    awaiting = true
    uart.write(UART_ID, req)
    stats.requests = stats.requests + 1
    stats.tx_bytes = stats.tx_bytes + #req

    local ok, frame = sys.waitUntil("JK_MB_RX", cfg.JK_MODBUS_TIMEOUT_MS)
    awaiting = false
    expected_num = nil
    if not ok or not frame then
        stats.timeouts = stats.timeouts + 1
        stats.last_error = string.format("read 0x%04X x%d timeout", start, num)
        return nil, "timeout"
    end
    d_hex("RX", frame)
    local regs, err = parse_read_response(frame, num)
    if not regs then
        stats.bad_response = stats.bad_response + 1
        stats.last_error = string.format("read 0x%04X x%d: %s", start, num, tostring(err))
    end
    return regs, err
end

--=============================================================================
-- 解析状态
--=============================================================================
local state = nil          -- 最近一轮的完整快照
local last_cell_count = 0  -- 已确认的串数（抗单节瞬时 0）
local shrink_hits = 0      -- 连续“串数变小”的轮数

--=============================================================================
-- 字节读取 / 换算
--=============================================================================
local function u32(regs, hi, lo)
    return (((regs[hi] or 0) << 16) | (regs[lo] or 0)) & 0xFFFFFFFF
end

local function s32(regs, hi, lo)
    local v = u32(regs, hi, lo)
    if v >= 0x80000000 then
        v = v - 0x100000000
    end
    return v
end

local function s16(v)
    v = (v or 0) & 0xFFFF
    if v >= 0x8000 then
        v = v - 0x10000
    end
    return v
end

-- 0.1℃ → 整数 ℃（四舍五入，负数对称）
local function tenths_to_c(v)
    if v >= 0 then
        return (v + 5) // 10
    end
    return -((-v + 5) // 10)
end

local function popcount32(v)
    local n = 0
    for _ = 1, 32 do
        if (v & 1) ~= 0 then
            n = n + 1
        end
        v = v >> 1
    end
    return n
end

--=============================================================================
-- 一轮数据 → 快照（字段与 jk_display 同名，另带 modbus 独有字段）
--=============================================================================
local function build_state(cells, cell_stats, st, complete)
    local cell_sta    = u32(cell_stats, 1, 2)
    local bat_vol_mv  = u32(st, ST_BATVOL_HI, ST_BATVOL_LO)
    local current_ma  = s32(st, ST_CUR_HI, ST_CUR_LO)
    local temp1_raw   = s16(st[ST_TEMP_BAT1])
    local temp2_raw   = s16(st[ST_TEMP_BAT2])
    local temp_mos    = s16(st[ST_TEMP_MOS])
    local alarm       = u32(st, ST_ALARM_HI, ST_ALARM_LO)
    local balan_ma    = s16(st[ST_BALAN_CUR])
    local full_cap    = u32(st, ST_FULLCAP_HI, ST_FULLCAP_LO)
    local cap_remain  = s32(st, ST_CAPREM_HI, ST_CAPREM_LO)
    local cycle_cnt   = u32(st, ST_CYCLE_HI, ST_CYCLE_LO)
    local cycle_cap   = u32(st, ST_CYCLE_CAP_HI, ST_CYCLE_CAP_LO)
    local run_time    = u32(st, ST_RUNTIME_HI, ST_RUNTIME_LO)
    local balan       = ((st[ST_BAL_SOC] or 0) >> 8) & 0xFF
    local soc         = (st[ST_BAL_SOC] or 0) & 0xFF
    local soh         = ((st[ST_SOH_PREC] or 0) >> 8) & 0xFF
    local charge_mos  = ((st[ST_CHARGE_STA] or 0) >> 8) & 0xFF
    local dis_mos     = (st[ST_CHARGE_STA] or 0) & 0xFF
    local user_alarm2 = st[ST_USER_ALARM2] or 0

    -- 1) 串数：优先用 CellSta 在位位图（popcount）；为 0 时退回“最后一个有效槽位”
    local detected = popcount32(cell_sta)
    if detected == 0 or detected > MAX_CELLS then
        detected = 0
        for i = 1, MAX_CELLS do
            if (cells[i] or 0) >= cfg.JK_CELL_MIN_MV then
                detected = i
            end
        end
    end
    if detected > last_cell_count then
        last_cell_count = detected
        shrink_hits = 0
    elseif detected < last_cell_count then
        shrink_hits = shrink_hits + 1
        if shrink_hits >= cfg.JK_CELL_CHANGE_DEBOUNCE then
            d_warn("cell count change", last_cell_count, "->", detected)
            last_cell_count = detected
            shrink_hits = 0
        end
    else
        shrink_hits = 0
    end
    local cellCount = last_cell_count

    -- 2) 单体电压表 + 最高/最低/压差
    local out = {}
    local max_mv, min_mv = 0, 0xFFFF
    for i = 1, MAX_CELLS do
        local mv = i <= cellCount and (cells[i] or 0) or 0
        out[i] = mv
        if mv > 0 then
            if mv > max_mv then max_mv = mv end
            if mv < min_mv then min_mv = mv end
        end
    end
    if min_mv == 0xFFFF then min_mv = 0 end

    -- 3) 容量：极空直接给“实际容量”，为 0 时用 剩余容量 ÷ SOC 兜底
    local fullCapMah = full_cap
    if fullCapMah <= 0 then
        if soc >= 5 and cap_remain > 0 then
            fullCapMah = math.floor(cap_remain * 100 / soc + 0.5)
        elseif state then
            fullCapMah = state.fullCapMah or 0
        end
    end

    -- 4) 9 个诊断位（与 jk_display.alarms 同序：欠压/过压/过流/MOS过温/电池过温/
    --    短路/内部通信/线阻/串数），供金箭从机在缺 sysAlarm 时兜底
    local function has(mask) return (alarm & mask) ~= 0 end
    local alarms = {
        has(A_CELL_UV) and 1 or 0,
        has(A_CELL_OV) and 1 or 0,
        (has(A_CHG_OCP) or has(A_DCH_OCP)) and 1 or 0,
        has(A_MOS_OT) and 1 or 0,
        (has(A_CHG_OTP) or has(A_DCH_OTP)) and 1 or 0,
        (has(A_CHG_SCP) or has(A_DCH_SCP)) and 1 or 0,
        has(A_AUX_COMM) and 1 or 0,
        has(A_WIRE) and 1 or 0,
        has(A_CELL_COUNT) and 1 or 0,
    }

    -- 5) 快照（新表，之后不再修改）
    local s = {
        ts              = os.time(),
        valid           = true,
        stale           = false,
        source          = "modbus",
        modbusComplete  = complete and true or false,
        cellCount       = cellCount,
        cells           = out,            -- [1..32] mV
        batVolMv        = bat_vol_mv,
        batCurrentMa    = current_ma,     -- 充电为正
        soc             = soc > 100 and 100 or soc,
        maxDiffMv       = max_mv - min_mv,
        tempMosC        = tenths_to_c(temp_mos),
        tempBatC        = tenths_to_c(temp1_raw),
        tempBat2C       = tenths_to_c(temp2_raw),
        warn            = (alarm ~= 0) and 1 or 0,
        balanceSw       = (balan ~= 0) and 1 or 0,
        chargeMos       = (charge_mos ~= 0) and 1 or 0,
        dischargeMos    = (dis_mos ~= 0) and 1 or 0,
        alarms          = alarms,         -- 9 个诊断位
        sysAlarm        = alarm,          -- 完整 32 位报警位图
        sysAlarmValid   = true,
        capRemainMah    = cap_remain,
        fullCapMah      = fullCapMah,
        balanCurrentMa  = balan_ma,       -- 均衡电流 mA（显示屏协议没有）
        cycleCount      = cycle_cnt > 0xFFFF and 0xFFFF or cycle_cnt,
        cycleCapMah     = cycle_cap,
        bmsUpTimeS      = run_time,       -- 0x12BC RunTime（BMS 上电秒数，显示屏协议没有）
        soh             = soh,
        cellMaxMv       = max_mv,
        cellMinMv       = min_mv,
        userAlarm2      = user_alarm2,
        powerW01        = math.floor(bat_vol_mv * current_ma / 100000 + 0.5), -- 0.1W
    }
    state = s
    stats.rounds = stats.rounds + 1
    stats.last_round_ms = now_ms()

    d_info("round", stats.rounds, "cell", cellCount,
        string.format("%.2fV %+.2fA soc=%d%%", bat_vol_mv / 1000, current_ma / 1000, soc),
        "tBat", s.tempBatC, "tMos", s.tempMosC,
        "cap", fullCapMah, "cyc", s.cycleCount,
        alarm ~= 0 and string.format("alarm=0x%08X", alarm) or "alarm=none")

    -- 一轮一条样本：sys.publish 只保留最后一次事件值，消费端需及时消费（见 bms_uplink）
    sys.publish("JK_ROUND", s)
    return s
end

--=============================================================================
-- 轮询一轮：单体电压 + 电池状态 + 汇总（任一块失败即整轮失败）
--=============================================================================
local function poll_once()
    local cells      = request_read(REG_CELLS, NUM_CELLS)
    local cell_stats = request_read(REG_CELL_STA, NUM_CELL_STA)

    local st, complete = {}, true
    local optional_error
    local required_ok = true
    for _, read in ipairs(STAT_READS) do
        local regs, err = request_read(read.reg, read.num)
        if regs then
            for i = 1, read.num do
                st[read.base + i - 1] = regs[i]
            end
        else
            complete = false
            if read.required then
                required_ok = false
            end
            optional_error = optional_error or string.format("0x%04X %s: %s",
                read.reg, read.field, tostring(err))
        end
    end

    stats.cycles = stats.cycles + 1
    if cells and cell_stats and required_ok then
        if stats.fail_streak > 0 then
            d_info("poll recovered after", stats.fail_streak, "failed rounds")
        end
        stats.fail_streak = 0
        if complete then
            if stats.degraded_streak > 0 then
                d_info("poll complete after", stats.degraded_streak, "degraded rounds")
            end
            stats.degraded_streak = 0
        else
            stats.degraded = stats.degraded + 1
            stats.degraded_streak = stats.degraded_streak + 1
            if stats.degraded_streak == 1 or stats.degraded_streak % 60 == 0 then
                d_warn("poll degraded", stats.degraded_streak, "times:", optional_error)
            end
        end
        build_state(cells, cell_stats, st, complete)
        return true
    end

    stats.cycles_failed = stats.cycles_failed + 1
    stats.fail_streak = (stats.fail_streak or 0) + 1
    if state and not state.stale and stats.last_round_ms and
       now_ms() - stats.last_round_ms > cfg.JK_DATA_TIMEOUT_MS then
        state.stale = true
        d_warn("modbus poll timeout, last round", state.ts)
    end
    return false
end
M.poll_once = poll_once

--=============================================================================
-- 串口打开 + 轮询任务
--=============================================================================
local opened = false
local running = true

local function open_uart()
    if opened then
        return true
    end
    if not uart or not uart.setup then
        return false
    end
    -- UART3 不是默认复用功能：先把 PIN53/PIN52 设成 UART3_RXD/TXD 再开串口（见 uart_pins.lua）
    uart_pins.mux(cfg.JK_UART_ID, cfg.JK_PIN_RX, cfg.JK_PIN_TX, "jk")
    local dir = cfg.JK_RS485_DIR_GPIO or 0xFFFFFFFF
    if not uart.setup(UART_ID, cfg.JK_MODBUS_BAUD, 8, 1, uart.NONE,
            uart.LSB, cfg.JK_MODBUS_RX_BUFF_SIZE, dir,
            cfg.JK_RS485_RX_LEVEL, cfg.JK_RS485_DELAY_US) then
        return false
    end
    uart.on(UART_ID, "receive", on_receive)
    opened = true
    d_info("uart", UART_ID, "opened", cfg.JK_MODBUS_BAUD,
        "8N1 (master), slave addr", SLAVE)
    return true
end

-- 离线测试用：停止轮询任务
function M.stop()
    running = false
end

-- 离线测试用：注入一段从机响应字节流
function M.feed(data)
    rxb = rxb .. data
    extract_frames()
end

sys.taskInit(function()
    local poll_ms = cfg.JK_MODBUS_POLL_MS or 1000
    local retry_ms = cfg.JK_MODBUS_RETRY_MS or (poll_ms * 2)
    while running do
        if not opened then
            if not open_uart() then
                d_warn("uart", UART_ID, "open failed, retry")
                sys.wait(1000)
            end
        else
            if poll_once() then
                sys.wait(poll_ms)
            else
                -- 失败退避：单轮最多 3 次超时（3 × JK_MODBUS_TIMEOUT_MS），
                -- 保护板掉线时不要以 1s 节奏反复重试并刷日志
                if stats.fail_streak == 1 or stats.fail_streak % 10 == 0 then
                    d_warn("poll failed", stats.fail_streak, "times, last_error:",
                        stats.last_error or "unknown", "retry in", retry_ms, "ms")
                end
                sys.wait(retry_ms)
            end
        end
    end
    d_info("poll task stopped")
end)

--=============================================================================
-- 对外接口
--=============================================================================

-- 最近一轮解析结果；从未收到有效数据时返回 nil
function M.get_state()
    return state
end

-- 数据是否新鲜（超时内完成过一轮轮询）
function M.is_fresh()
    return state ~= nil and not state.stale
end

function M.get_stats()
    return stats
end

return M
