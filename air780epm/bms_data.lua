--[[
@module  bms_data
@summary 采集适配层：把极空两套协议的快照归一成同一份「规范样本」写入 bms_cache
@version 1.0
@date    2026.10.12
@usage
数据流（与 README §1 一致）：

    jk_display / jk_modbus ──"JK_ROUND"──► bms_data ──normalize──► bms_cache.push

本模块只做「字段/单位映射」，不做插值、不补零、不改协议解析：
  · display（显示屏广播）：tempBat2C = tempBat1C；balanCurrent / socCycleCap / cycleCount /
    bmsUpTimeS 无数据源记 0；socFullChargeCapMah 沿用 jk_display 的「剩余容量 ÷ SOC」估计；
  · modbus（RS485 主站）：字段直通（两路温度、均衡电流、循环次数/循环容量、SOH、
    0x12BC RunTime → bmsUpTimeS）。

规范样本字段（同时满足金箭寄存器映射与 MessageV1 编码；固定 32 路单体槽位）：
    seq(缓存分配), ts(Unix 秒), cellCount, cells[1..MAX_CELLS](mV), batVolMv, batCurrentMa,
    soc, tempBat1C, tempBat2C, tempMosC, balanCurrentMa, socFullChargeCapMah,
    socCapRemainMah, socCycleCapMah, cycleCount, soh, sysAlarm, sysAlarmValid, alarms(9 图标位),
    balanceSw, chargeMos, dischargeMos, bmsUpTimeS, source

对外接口：
  bms_data.normalize(snapshot)   快照 → 规范样本（离线自测可直接调用，不依赖事件）
  bms_data.module_uptime_s()     模块上电秒数（mcu.ticks 单调计时，供 ModuleUPTime）
  bms_data.get_stats()           采集统计（rounds/skipped/pushed）
  bms_data.MAX_CELLS             规范样本的单体槽位数（32，两套协议取较大者）
]]

local cfg = require "config"
local jk = require "jk"
local cache = require "bms_cache"

local M = {}

-- 规范样本固定 32 路槽位：display 只填前 25 路（其余为 0），modbus 填满 32 路。
-- 编码/寄存器读取都只看样本的 cellCount，多余槽位不参与运算。
local MAX_CELLS = 32
M.MAX_CELLS = MAX_CELLS

local stats = {
    rounds = 0,  -- 收到的 JK_ROUND 次数
    pushed = 0,  -- 成功归一并入缓存的样本数
    skipped = 0, -- 归一失败（快照非法）而被跳过的轮数
}

local function d_warn(...) if cfg.LOG_ENABLE then log.warn("bmsdata", ...) end end

local function num(v, default)
    v = tonumber(v)
    if v == nil then
        return default or 0
    end
    return v
end

local function clamp(v, lo, hi)
    if v < lo then
        return lo
    end
    if v > hi then
        return hi
    end
    return v
end

-- 4G 上报的 ModuleUPTime：模组上电秒数（mcu.ticks 单调，不受 NTP 跳变影响）
function M.module_uptime_s()
    if mcu and mcu.ticks and mcu.hz then
        local hz = mcu.hz()
        if hz and hz > 0 then
            return math.floor(mcu.ticks() / hz)
        end
    end
    return 0
end

-- 极空两套协议的快照 → 规范样本；s 非法时返回 nil
function M.normalize(s)
    if type(s) ~= "table" then
        return nil
    end

    local source = s.source or "display"
    local modbus = (source == "modbus")

    local cellCount = math.floor(num(s.cellCount, 0))
    if cellCount < 0 then
        cellCount = 0
    elseif cellCount > MAX_CELLS then
        cellCount = MAX_CELLS
    end
    -- config.CELL_COUNT > 0 时固定串数（覆盖自动识别），与旧实现的后门保持一致
    -- （固定值大于实际串数时缺失槽位为 0；大于 MAX_CELLS 时按 MAX_CELLS 截断）
    local fixed = math.floor(num(cfg.CELL_COUNT, 0))
    if fixed > 0 then
        cellCount = math.min(fixed, MAX_CELLS)
    end

    local cells = {}
    for i = 1, MAX_CELLS do
        local mv = num(s.cells and s.cells[i], 0)
        if mv < 0 then
            mv = 0
        end
        cells[i] = math.floor(mv)
    end

    -- 电池温度：display 只有一路（固件按工况在 T1/T2 间取一个填入），两路写同值
    local tempBat1C = math.floor(num(s.tempBatC, 0))
    local tempBat2C = modbus and math.floor(num(s.tempBat2C, tempBat1C)) or tempBat1C

    return {
        ts = math.floor(num(s.ts, os.time())),
        cellCount = cellCount,
        cells = cells,
        batVolMv = math.floor(num(s.batVolMv, 0)),
        batCurrentMa = math.floor(num(s.batCurrentMa, 0)),
        soc = clamp(math.floor(num(s.soc, 0)), 0, 100),
        tempBat1C = tempBat1C,
        tempBat2C = tempBat2C,
        tempMosC = math.floor(num(s.tempMosC, 0)),
        -- display 广播没有均衡电流
        balanCurrentMa = modbus and math.floor(num(s.balanCurrentMa, 0)) or 0,
        -- 满电总容量：display 用「剩余容量 ÷ SOC」估算（jk_display.fullCapMah），modbus 直读 0x12AC
        socFullChargeCapMah = math.floor(num(s.fullCapMah, 0)),
        socCapRemainMah = math.floor(num(s.capRemainMah, 0)),
        -- 循环总容量：display 没有该字段（写 0），modbus 取 0x12B4
        socCycleCapMah = modbus and math.floor(num(s.cycleCapMah, 0)) or 0,
        cycleCount = modbus and math.floor(num(s.cycleCount, 0)) or 0,
        soh = modbus and math.floor(num(s.soh, 0)) or 0,
        sysAlarm = math.floor(num(s.sysAlarm, 0)),
        sysAlarmValid = s.sysAlarmValid == true,
        -- 9 个图标位：display 缺扩展块时金箭保护位回退要用（见 jinjian_slave.protection_bits）
        alarms = s.alarms,
        balanceSw = (num(s.balanceSw, 0) ~= 0) and 1 or 0,
        chargeMos = (num(s.chargeMos, 0) ~= 0) and 1 or 0,
        dischargeMos = (num(s.dischargeMos, 0) ~= 0) and 1 or 0,
        -- BMS 上电时间：modbus 取 0x12BC RunTime，display 无来源记 0
        bmsUpTimeS = modbus and math.floor(num(s.bmsUpTimeS, 0)) or 0,
        source = source,
    }
end

--=============================================================================
-- 采集任务：JK_ROUND（一轮一条）→ 规范样本 → 缓存
--   · 只做字段映射（微秒级），不会长时间占住调度器；
--   · sys.publish 只保留最后一次事件值，本任务每条都及时消费。
--=============================================================================
sys.taskInit(function()
    while true do
        if sys.waitUntil("JK_ROUND") then
            stats.rounds = stats.rounds + 1
            local ok, s = pcall(jk.get_state)
            local sample = ok and M.normalize(s) or nil
            if sample then
                cache.push(sample)
                stats.pushed = stats.pushed + 1
            else
                stats.skipped = stats.skipped + 1
                d_warn("JK_ROUND 快照非法，跳过")
            end
        end
    end
end)

function M.get_stats()
    return stats
end

return M
