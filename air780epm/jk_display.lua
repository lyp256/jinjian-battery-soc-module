--[[
@module  jk_display
@summary 极空(JK) BMS 显示屏协议接收与解析（RS4851 / 串口1）
@version 1.1
@date    2026.09.30
@usage
协议依据 docs/jikong/极空显示屏协议.md：
  2400 8N1、单向广播、一轮 ≈ 2.17s，帧格式：
    A5 5A | LEN | CMD | ADDR_H | ADDR_L | DATA(N)，其中 LEN = N + 3，帧总长 = LEN + 3
  命令码：
    0x80 = 写寄存器（1B 寄存器号 + 1B 值）：寄存器 0x01 = 每轮第一个帧（背光/刷新，
           作为轮起点）；寄存器 0x02 = 蜂鸣器帧（报警时出现在轮尾），不能当轮起点
    0x82 = 写变量区（2B 大端地址 + 数据）
  一轮顺序：写寄存器(0x80) → 主数据块(0x82/0x1000, 45×u16)
           → [仅 ≥25 串] 第25节电压(0x82/0x2011, u16)
           → 单体颜色帧×27(0x82/0x20xx, RGB565) → 扩展块(0x82/0x1300, 20×u16)
           → [报警时] 蜂鸣器帧(0x80/0x02)
  轮字节数随固件/串数变化（20 串 = 364 字节，≥25 串多 9 字节），本模块一律按
  LEN 定界、不写死轮长。

  单体颜色帧（旧文档误称"记录块/均衡线电阻表"）：
    0x07E0 绿 = 普通单体，0x001F 蓝 = 最高单体，0xF800 红 = 最低单体，0x0000 黑 = 未用槽位

对外接口：
  jk_display.get_state()   最近一轮解析结果（table），从未收到时返回 nil
  jk_display.is_fresh()    最近一轮是否在超时时间内
  jk_display.get_stats()   统计信息（轮数、帧数、错误数、最近错误）
                           rounds = 收到极空保护板的数据次数（完整广播轮数，一轮一份数据）
                           frames = 已解析帧数
  jk_display.MAX_CELLS     协议里单体的最大槽位数（主块 24 + 0x2011 补发的第 25 节）

每解析完一轮广播（扩展块结束）发布一次 "JK_ROUND" 事件，携带本轮 state，
供采集上报 task 按"一轮一条样本"的粒度入队。
]]

local cfg = require "config"
local M = {}

-- 主数据块 24 槽 + 0x2011 补发的第 25 节（≥25 串才发）
local MAX_CELLS = 25
M.MAX_CELLS = MAX_CELLS

local function d_info(...) if cfg.LOG_ENABLE then log.info("jk", ...) end end
local function d_warn(...) if cfg.LOG_ENABLE then log.warn("jk", ...) end end
local function d_debug(...) if cfg.LOG_RAW_FRAMES then log.info("jk.raw", ...) end end

--=============================================================================
-- 统计信息
--=============================================================================
local stats = {
    frames = 0,          -- 已解析帧数
    rounds = 0,          -- 已完成轮数（= 收到极空保护板的数据次数，一轮一份数据）
    sync_lost = 0,       -- 因帧头/长度非法丢弃的字节数（重新同步）
    bad_len = 0,         -- 长度与命令码不匹配的帧数
    cell_mismatch = 0,   -- Σ单体电压 与 总电压 明显不符的次数
    reg_writes = 0,      -- 0x80 写寄存器帧（轮起点）计数
    buzzer = 0,          -- 0x80/0x02 蜂鸣器帧计数
    other_writes = 0,    -- 其它寄存器写帧（不作为轮起点）
    color_frames = 0,    -- 单体颜色帧计数
    special_frames = 0,  -- 0x2203/0x2213 特殊状态帧计数
    cell25_frames = 0,   -- 第 25 节电压帧（≥25 串）计数
    crosscheck_mismatch = 0, -- 扩展块偏移 28/30 与主块 8/10 不一致的次数
    last_error = nil,
    last_round_ms = nil, -- 最近一轮完成的 tick(ms)
}

local function now_ms()
    if mcu and mcu.ticks and mcu.hz then
        return math.floor(mcu.ticks() * 1000 / mcu.hz())
    end
    return os.time() * 1000
end

--=============================================================================
-- 字节读取（全部大端）
--=============================================================================
local function u16(data, off) -- off 为数据块内 0 基字节偏移
    local hi = string.byte(data, off + 1)
    local lo = string.byte(data, off + 2)
    if not hi or not lo then
        return 0
    end
    return ((hi << 8) | lo) & 0xFFFF
end

local function i16(data, off)
    local v = u16(data, off)
    if v >= 0x8000 then
        v = v - 0x10000
    end
    return v
end

local function u32(data, off)
    return ((u16(data, off) << 16) | u16(data, off + 2)) & 0xFFFFFFFF
end

--=============================================================================
-- 解析状态
--=============================================================================
local state = nil          -- 最近一轮的完整快照（每轮新建，不原地修改）
local last_cell_count = 0  -- 已确认的串数（抗单节瞬时 0mV）
local shrink_hits = 0      -- 连续"串数变小"的轮数

-- 本轮累积缓冲
local function new_work()
    return {
        main = nil,      -- 主数据块
        ext = nil,       -- 扩展块
        records = 0,     -- 0x20xx 帧数（颜色帧 + 特殊帧 + 第25节帧）
        colors = nil,    -- [1..25] 单体颜色码（RGB565）
        cell25Mv = nil,  -- 第 25 节电压 mV（仅 ≥25 串）
        special = 0,     -- 0x2203/0x2213 特殊状态帧数
    }
end

local work = new_work()

local ALARM_NAMES = {
    "cell_under", "cell_over", "over_current", "mos_over_temp", "bat_over_temp",
    "short_circuit", "co_proc_comm", "wire_res", "cell_count",
}

local function alarm_list()
    local list = {}
    for i = 1, #ALARM_NAMES do
        if state and state.alarms[i] == 1 then
            list[#list + 1] = ALARM_NAMES[i]
        end
    end
    return list
end

-- 主数据块（45×u16）
local function handle_main(data)
    local m = {
        batVol10mV    = u16(data, 0),   -- 10mV
        batCurrent01A = i16(data, 2),   -- 0.1A，充电为正
        soc           = u16(data, 6),   -- %
        maxDiffMv     = u16(data, 8),   -- mV
        tempMosC      = i16(data, 10),  -- ℃
        tempBatC      = i16(data, 12),  -- ℃
        warn          = u16(data, 14),
        cellAvgMv     = u16(data, 16),
        balanceSw     = u16(data, 18),
        chargeMos     = u16(data, 20),
        dischargeMos  = u16(data, 22),
        cells         = {},             -- [1..24] mV，未使用槽位 = 0（第 25 节见 0x2011 帧）
        alarms        = {},             -- [1..9] 0/1
    }
    for i = 0, 23 do
        m.cells[i + 1] = u16(data, 24 + i * 2)
    end
    for i = 0, 8 do
        m.alarms[i + 1] = u16(data, 72 + i * 2)
    end
    work.main = m
end

-- 扩展块（20×u16，字段依据 2026-09-29 固件逆向版手册 §3.3 / §11.4）
local function handle_ext(data)
    work.ext = {
        socIcon        = u16(data, 0),   -- SOC 电池图标格数（5 格图标）
        capRemain01Ah  = u16(data, 2),   -- 折算剩余容量 0.1Ah
        chargeLeftH    = u16(data, 4),   -- 剩余时间·小时
        chargeLeftM    = u16(data, 6),   -- 剩余时间·分
        chargeLeftSec  = u16(data, 8),   -- 剩余时间·秒
        power01W       = u32(data, 10),  -- 功率 0.1W（u32，起点在偏移 10）
        cellMaxMv      = u16(data, 14),  -- 最高单体 mV
        cellMinMv      = u16(data, 16),  -- 最低单体 mV
        cellAvgMv      = u16(data, 18),  -- 单体平均 mV
        sysAlarm       = u32(data, 20),  -- 完整 sysAlarm 32 位位图
        animCount      = u16(data, 24),  -- 动画/刷新计数（3 态循环）
        chargeAnim     = u16(data, 26),  -- 充电动画状态
        maxDiffMv      = u16(data, 28),  -- 最大压差 mV（与主块偏移 8 同源）
        tempMosC       = i16(data, 30),  -- MOS 温度整数℃（与主块偏移 10 同源）
        sysWarn        = u16(data, 32),  -- 是否有系统报警（同主块偏移 14）
        balanceSw      = u16(data, 34),  -- 均衡开关（同主块偏移 18）
        statusA        = u16(data, 36),  -- 内部状态字节 A（含义未定）
        statusB        = u16(data, 38),  -- 内部状态字节 B（含义未定）
    }
end

-- 0x20xx 帧：单体颜色帧 / 第 25 节电压帧 / 特殊状态帧
local function handle_record(data, addr)
    work.records = work.records + 1
    if addr == 0x2011 then
        -- 仅 ≥25 串的电池才发：第 25 节电压（u16 BE，mV）
        work.cell25Mv = u16(data, 0)
        stats.cell25_frames = stats.cell25_frames + 1
    elseif addr >= 0x2003 and addr <= 0x2183 and ((addr - 0x2003) % 0x10) == 0 then
        -- 第 n+1 串的 RGB565 颜色码（不是测量值）：
        --   0x07E0 绿=普通单体 / 0x001F 蓝=最高单体 / 0xF800 红=最低单体 / 0x0000 黑=未用槽位
        local n = (addr - 0x2003) // 0x10 + 1
        work.colors = work.colors or {}
        work.colors[n] = u16(data, 0)
        stats.color_frames = stats.color_frames + 1
    else
        -- 0x2203 / 0x2213：两个特殊状态帧（固件内部哨兵相关），本项目不使用
        work.special = work.special + 1
        stats.special_frames = stats.special_frames + 1
    end
end

-- 一轮结束：组装并发布本轮样本
local function finalize_round()
    local m = work.main
    local e = work.ext or {}
    if not m then
        work = new_work()
        return
    end

    -- 1) 槽位电压：主块 24 槽 +（≥25 串时）0x2011 补发的第 25 节
    local slots = {}
    for i = 1, 24 do
        slots[i] = m.cells[i]
    end
    slots[25] = work.cell25Mv or 0

    -- 2) 串数：取最后一个电压 >= 阈值的槽位；变小需连续多轮确认
    local detected = 0
    for i = 1, MAX_CELLS do
        if slots[i] >= cfg.JK_CELL_MIN_MV then
            detected = i
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

    -- 3) 换算成工程单位
    local cellCount = last_cell_count
    local cells = {}
    local sumMv = 0
    for i = 1, MAX_CELLS do
        cells[i] = slots[i] or 0
        if i <= cellCount then
            sumMv = sumMv + cells[i]
        end
    end
    local batVolMv = m.batVol10mV * 10
    local batCurrentMa = m.batCurrent01A * 100

    -- 4) 自检：Σ单体电压 ≈ 总电压（协议手册 §6 的三条自检之一）
    if cellCount > 0 and batVolMv > 0 then
        local diff = sumMv - batVolMv
        if diff < 0 then diff = -diff end
        if diff > math.max(500, batVolMv // 20) then
            stats.cell_mismatch = stats.cell_mismatch + 1
            d_warn("cell sum mismatch, sum", sumMv, "total", batVolMv)
        end
    end

    -- 5) 容量估算：极空广播没有总容量，用 剩余容量 ÷ SOC 推算（SOC 太低时不更新）
    local capRemainMah = (e.capRemain01Ah or 0) * 100
    local fullCapMah = state and state.fullCapMah or 0
    if m.soc >= 5 and capRemainMah > 0 then
        fullCapMah = math.floor(capRemainMah * 100 / m.soc + 0.5)
    end

    -- 6) 单体颜色帧：蓝=最高单体、红=最低单体、非黑=有效槽位（诊断用，不参与串数判定）
    local colors = work.colors or {}
    local cellMaxIdx, cellMinIdx, colorCells = 0, 0, 0
    for i = 1, MAX_CELLS do
        local c = colors[i]
        if c == 0x001F then
            cellMaxIdx = i
        end
        if c == 0xF800 then
            cellMinIdx = i
        end
        if c ~= nil and c ~= 0x0000 then
            colorCells = colorCells + 1
        end
    end

    -- 7) 交叉校验：扩展块偏移 28/30 与主块偏移 8/10 同源（手册 §9.1）
    if e.maxDiffMv ~= nil and e.maxDiffMv ~= m.maxDiffMv then
        stats.crosscheck_mismatch = stats.crosscheck_mismatch + 1
    end
    if e.tempMosC ~= nil and e.tempMosC ~= m.tempMosC then
        stats.crosscheck_mismatch = stats.crosscheck_mismatch + 1
    end

    -- 8) 生成本轮快照（新表，之后不再修改）
    local s = {
        ts             = os.time(),
        valid          = true,
        stale          = false,
        cellCount      = cellCount,
        cells          = cells,          -- [1..25] mV（25 仅 ≥25 串时有效）
        batVolMv       = batVolMv,
        batCurrentMa   = batCurrentMa,   -- 充电为正
        soc            = m.soc,
        socIcon        = e.socIcon or 0, -- SOC 电池图标格数
        maxDiffMv      = m.maxDiffMv,
        tempMosC       = m.tempMosC,
        tempBatC       = m.tempBatC,
        warn           = m.warn,
        cellAvgMv      = m.cellAvgMv,
        balanceSw      = m.balanceSw,
        chargeMos      = m.chargeMos,
        dischargeMos   = m.dischargeMos,
        alarms         = m.alarms,       -- 9 个图标位（sysAlarm 的位组合）
        sysAlarm       = e.sysAlarm,     -- 完整 32 位报警位图（nil = 本轮缺扩展块）
        sysAlarmValid  = (e.sysAlarm ~= nil),
        capRemainMah   = capRemainMah,   -- 折算剩余容量 mAh
        fullCapMah     = fullCapMah,     -- 估算总容量 mAh
        chargeLeftH    = e.chargeLeftH or 0,
        chargeLeftS    = (e.chargeLeftH or 0) * 3600 + (e.chargeLeftM or 0) * 60
                         + (e.chargeLeftSec or 0),
        powerW01       = e.power01W or 0,
        cellMaxMv      = e.cellMaxMv or 0,
        cellMinMv      = e.cellMinMv or 0,
        cellColors     = colors,         -- [1..25] RGB565 颜色码
        cellMaxIdx     = cellMaxIdx,     -- 最高单体序号（蓝），0 = 未标出
        cellMinIdx     = cellMinIdx,     -- 最低单体序号（红），0 = 未标出
        colorCells     = colorCells,     -- 颜色帧里的有效槽位数（诊断）
        records        = work.records,
    }
    state = s
    stats.rounds = stats.rounds + 1
    stats.last_round_ms = now_ms()

    local alarms = alarm_list()
    d_info("round", stats.rounds, "cell", cellCount,
        string.format("%.2fV %+.2fA soc=%d%%", batVolMv / 1000, batCurrentMa / 1000, m.soc),
        "tBat", m.tempBatC, "tMos", m.tempMosC,
        string.format("left=%d:%02d:%02d", s.chargeLeftH,
            (e.chargeLeftM or 0), (e.chargeLeftSec or 0)),
        #alarms > 0 and ("alarm=" .. table.concat(alarms, ",")) or "alarm=none")

    work = new_work()
    -- 一轮一条样本：注意 sys.publish 只保留最后一次事件值，
    -- 消费端（bms_uplink 采样任务）必须能及时消费，否则会丢轮（见其 stats.rounds_lost）。
    sys.publish("JK_ROUND", s)
end

--=============================================================================
-- 帧解析状态机（按 LEN 定界，靠帧头 + 合法长度自同步）
--=============================================================================
local LEN_SYNC, LEN_RECORD, LEN_EXT, LEN_MAIN = 0x03, 0x05, 0x2B, 0x5D

local function handle_frame(cmd, addr, data)
    stats.frames = stats.frames + 1
    if cmd == 0x80 then
        -- 0x80 = 写寄存器：第 4 字节 = 寄存器号，第 5 字节 = 值（不是 16 位地址）
        local reg = addr >> 8
        local val = addr & 0xFF
        if reg == 0x01 then
            -- 每轮第一个帧（背光/刷新，值 0x00 或 0x40）：轮起点，重置轮内计数
            stats.reg_writes = stats.reg_writes + 1
            if work.main then
                d_debug("reg write without ext, finalize previous round")
                finalize_round()
            end
            work = new_work()
            d_debug(string.format("round start, reg 0x01 <- 0x%02X", val))
        elseif reg == 0x02 then
            -- 蜂鸣器帧（报警时出现在轮尾）：不能当轮起点，否则会把一轮截断
            stats.buzzer = stats.buzzer + 1
            d_debug("buzzer frame, value", val)
        else
            -- 其它寄存器写：仅计数，轮结束仍以扩展块为准
            stats.other_writes = stats.other_writes + 1
            d_debug(string.format("register write 0x%02X <- 0x%02X (ignored)", reg, val))
        end
    elseif cmd == 0x82 then
        if addr == 0x1000 then
            handle_main(data)
        elseif addr == 0x1300 then
            handle_ext(data)
            finalize_round()
        elseif addr >= 0x2000 and addr <= 0x22FF then
            handle_record(data, addr)
        else
            d_debug("unknown block addr", string.format("0x%04X", addr))
        end
    else
        d_debug("unknown cmd", string.format("0x%02X", cmd))
    end
end

local buf = ""

local function parse_buffer()
    while true do
        -- 同步帧头 A5 5A
        local pos = string.find(buf, "\165\90", 1, true)
        if not pos then
            -- 没有帧头，保留最后一个字节以防它正好是 0xA5
            if #buf > 1 then
                stats.sync_lost = stats.sync_lost + #buf - 1
                buf = buf:sub(-1)
            end
            return
        end
        if pos > 1 then
            stats.sync_lost = stats.sync_lost + pos - 1
            buf = buf:sub(pos)
        end

        if #buf < 6 then
            return -- 帧头/LEN/CMD/ADDR 还不齐
        end

        local len = string.byte(buf, 3)
        local cmd = string.byte(buf, 4)
        local addr = (string.byte(buf, 5) << 8) | string.byte(buf, 6)

        -- 长度合法性校验：只有 4 种固定块，非法说明当前 A5 5A 是数据里的假帧头
        local valid = (cmd == 0x80 and len == LEN_SYNC) or
                      (cmd == 0x82 and (len == LEN_MAIN or len == LEN_EXT or
                                        (len == LEN_RECORD and addr >= 0x2000 and addr <= 0x22FF)))
        if not valid then
            stats.bad_len = stats.bad_len + 1
            stats.last_error = string.format("bad frame cmd=0x%02X len=%d", cmd, len)
            stats.sync_lost = stats.sync_lost + 1
            buf = buf:sub(2) -- 跳过 1 字节重新同步
        else
            local total = len + 3
            if #buf < total then
                return -- 数据还不全，等下一批
            end
            local data = buf:sub(7, 6 + len - 3)
            buf = buf:sub(total + 1)
            handle_frame(cmd, addr, data)
        end
    end
end

--=============================================================================
-- 串口接收
--=============================================================================
local opened = false

-- 串口接收回调：只把字节搬进缓冲并打一个"有新数据"标记，解析交给独立的解析任务。
-- 这样回调本身很快返回，不会长时间占住调度器（一轮广播 364 字节、周期 0.5-2.2s）。
local function on_receive(id)
    local s = ""
    repeat
        s = uart.read(id, 256)
        if s and #s > 0 then
            buf = buf .. s
        end
    until not s or #s == 0
    -- 防止异常情况下缓冲区无限增长
    if #buf > cfg.JK_RX_BUFF_SIZE then
        stats.sync_lost = stats.sync_lost + (#buf - cfg.JK_RX_BUFF_SIZE)
        buf = buf:sub(-cfg.JK_RX_BUFF_SIZE)
    end
    sys.publish("JK_RX")
end

local function open_uart()
    if opened then
        return true
    end
    if not uart or not uart.setup then
        return false
    end
    local dir = cfg.JK_RS485_DIR_GPIO or 0xFFFFFFFF
    local ok = uart.setup(cfg.JK_UART_ID, cfg.JK_BAUD, 8, 1, uart.NONE,
        uart.LSB, cfg.JK_RX_BUFF_SIZE, dir, cfg.JK_RS485_RX_LEVEL, cfg.JK_RS485_DELAY_US)
    if not ok then
        return false
    end
    uart.on(cfg.JK_UART_ID, "receive", on_receive)
    opened = true
    d_info("uart", cfg.JK_UART_ID, "opened", cfg.JK_BAUD, "8N1 (RX only)")
    return true
end

sys.taskInit(function()
    while not open_uart() do
        d_warn("uart", cfg.JK_UART_ID, "open failed, retry")
        sys.wait(1000)
    end
    -- 解析任务 + 数据失效看门狗：
    --   · 收到 JK_RX（串口回调只做搬字节）后解析，避免在回调里做重活；
    --   · 没有数据时每 2s 醒一次，检查广播是否中断超过 JK_DATA_TIMEOUT_MS → 标记 stale。
    while true do
        if sys.waitUntil("JK_RX", 2000) then
            parse_buffer()
        end
        if state and stats.last_round_ms then
            if now_ms() - stats.last_round_ms > cfg.JK_DATA_TIMEOUT_MS and not state.stale then
                state.stale = true
                d_warn("broadcast timeout, last round", state.ts)
            end
        end
    end
end)

--=============================================================================
-- 对外接口
--=============================================================================

-- 最近一轮解析结果；从未收到有效广播时返回 nil
function M.get_state()
    return state
end

-- 数据是否新鲜（超时内收到过完整一轮）
function M.is_fresh()
    return state ~= nil and not state.stale
end

function M.get_stats()
    return stats
end

-- 供离线测试/调试使用：注入一段广播字节流
function M.feed(data)
    buf = buf .. data
    parse_buffer()
end

return M
