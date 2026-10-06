--[[
@module  selftest
@summary 离线自测（PC 端 Lua 5.4）
@version 1.1
@date    2026.09.28
@usage
在 air780epm 目录下执行：
    lua tools/selftest.lua

测试内容：
1. bms_codec 黄金向量字节级比对（2×3 = 41B、3×2 全静态 = 30B）；
2. jk_display 解析整轮广播样本（docs/jikong/极空显示屏协议.md §7）并核对全部物理量；
3. jinjian_slave 对 01 03 / 01 01 / 01 06 / 01 05 的应答、异常码与文档 CRC 样例；
4. 端到端：用协程调度器真跑 bms_uplink 的"采样→批次→编码→发布"链路，
   并用独立实现的解码器解回数据做逐条核对。
]]

--=============================================================================
-- 运行环境打桩：让业务模块能在 PC 的 Lua 上加载并真正跑起来
--=============================================================================
local here = (arg and arg[0] or ""):match("^(.*)[/\\][^/\\]*$") or "."
package.path = here .. "/../?.lua;" .. here .. "/?.lua;" .. package.path

_G.log = {
    info = function(tag, ...) io.write("[info] ", tostring(tag), " ", table.concat({...}, " "), "\n") end,
    warn = function(tag, ...) io.write("[warn] ", tostring(tag), " ", table.concat({...}, " "), "\n") end,
    error = function(tag, ...) io.write("[error] ", tostring(tag), " ", table.concat({...}, " "), "\n") end,
    debug = function() end,
}

-- 极简协作式调度器：sys.taskInit / sys.wait / sys.waitUntil / sys.publish
local tasks, events = {}, {}
local scheduler = {}

_G.sys = {}

function _G.sys.taskInit(fn)
    tasks[#tasks + 1] = {
        co = coroutine.create(fn), event = false, sleeping = false, dead = false,
    }
end

function _G.sys.wait(ms)
    coroutine.yield({ sleep = ms or 1 })
end

function _G.sys.waitUntil(name, _timeout)
    local data = coroutine.yield({ event = name })
    if data == nil then
        return false
    end
    return true, data
end

function _G.sys.publish(name, data)
    events[name] = (data == nil) and true or data
end

_G.sys.subscribe = function() end
_G.sys.timerStart = function() end
_G.sys.timerLoopStart = function() end

local function pump(max_steps)
    max_steps = max_steps or 500
    local steps = 0
    while steps < max_steps do
        local ran = false
        for _, t in ipairs(tasks) do
            if not t.dead and not t.sleeping and (not t.event or events[t.event] ~= nil) then
                local arg
                if t.event then
                    arg = events[t.event]
                    events[t.event] = nil
                end
                local ok, y = coroutine.resume(t.co, arg)
                steps = steps + 1
                ran = true
                if not ok then
                    io.write("[sched] task error: ", tostring(y), "\n")
                    t.dead = true
                elseif coroutine.status(t.co) == "dead" then
                    t.dead = true
                elseif type(y) == "table" then
                    t.event = y.event or false
                    t.sleeping = y.sleep ~= nil
                end
            end
        end
        if not ran then
            break
        end
    end
end

function scheduler.wake()
    for _, t in ipairs(tasks) do
        t.sleeping = false
    end
    pump()
end

function scheduler.publish(name, data)
    _G.sys.publish(name, data)
    pump()
end

-- 丢弃尚未被消费的事件（LuatOS 的 sys.publish 不会排队投递给后来才等待的任务，
-- 测试里把前面章节遗留的 JK_ROUND 清掉，保证第 4 节从干净状态开始计数）
function scheduler.clear(name)
    events[name] = nil
end

_G.uart = {
    setup = function() return false end, -- PC 上串口必然打开失败，业务 task 会走重试分支
    on = function() end, read = function() return "" end,
    write = function() end, close = function() end, LSB = 0, NONE = 0,
}
_G.gpio = { setup = function() end, set = function() end, get = function() return 1 end, PULLUP = 1 }
_G.adc = {
    open = function() return false end, get = function() return nil end,
    close = function() end, setRange = function() end, ADC_RANGE_MAX = 1,
}
_G.mcu = {
    ticks = function() return os.clock() * 1000 end,
    hz = function() return 1000 end,
    unique_id = function() return { toHex = function() return "9888E072BD78AABB" end } end,
}
_G.mobile = { imei = function() return "861234567890123" end, simid = function() end }
_G.rtos = { reboot = function() end, bsp = function() return "PC" end }
_G.air153C_wtd = nil
_G.VERSION = "1.0.0"

--=============================================================================
-- 断言与工具
--=============================================================================
local pass, fail = 0, 0

local function check(name, got, want)
    if got == want then
        pass = pass + 1
        io.write(string.format("  ok   %s\n", name))
    else
        fail = fail + 1
        io.write(string.format("  FAIL %s\n       got:  %s\n       want: %s\n",
            name, tostring(got), tostring(want)))
    end
end

local function to_hex(s)
    return (s:gsub(".", function(c) return string.format("%02x", c:byte()) end))
end

local function from_hex(s)
    s = s:gsub("%s+", "")
    local out = {}
    for i = 1, #s, 2 do
        out[#out + 1] = string.char(tonumber(s:sub(i, i + 1), 16))
    end
    return table.concat(out)
end

local function build_frame(hex)
    local body = from_hex(hex)
    local crc = require("jinjian_slave").crc16(body)
    return body .. string.char(crc & 0xFF, (crc >> 8) & 0xFF)
end

--=============================================================================
-- 1. 压缩编码黄金向量
--=============================================================================
io.write("== bms_codec 黄金向量 ==\n")
local codec = require "bms_codec"

do
    local hist = {
        count = 2, cellCount = 3,
        temp1 = { -125, -124 }, temp2 = { 231, 232 }, tempMos = { 450, 451 },
        balanCurrent = { 12, 13 },
        batVol = { 52340, 52341 }, batCurrent = { -1800, -1799 },
        socCycleCap = { 123456, 123457 }, socCapRemain = { 98765, 98766 },
        time = { 1700000000, 1700000001 },
        cellVols = { 3300, 3301, 3302, 3301, 3302, 3303 },
    }
    local want = "010cf90104ce03048407040c04f49803048f1c04c0c40704cd83060480e2cfaa0604e4190402010401"
    check("vector1 (2x3) 字节一致", to_hex(codec.encode(hist)), want)
end

do
    local hist = {
        count = 3, cellCount = 2,
        temp1 = { 250, 250, 250 }, temp2 = { 248, 248, 248 }, tempMos = { 300, 300, 300 },
        balanCurrent = { 10, 10, 10 },
        batVol = { 52000, 52000, 52000 }, batCurrent = { 0, 0, 0 },
        socCycleCap = { 100000, 100000, 100000 }, socCapRemain = { 90000, 90000, 90000 },
        time = { 1700000000, 1700000000, 1700000000 },
        cellVols = { 3300, 3301, 3300, 3301, 3300, 3301 },
    }
    local want = "0209aaaa2af403f003d8040aa0960300a08d0690bf0580e2cfaa06e41902"
    check("vector2 (3x2 静态) 字节一致", to_hex(codec.encode(hist)), want)
end

--=============================================================================
-- 2. 极空广播整轮样本解析（协议手册 §7）
--=============================================================================
io.write("== jk_display 整轮广播解析 ==\n")
local jk = require "jk_display"

local ROUND_HEX = [[
A5 5A 03 80 01 00 A5 5A 5D 82 10 00 19 CF 00 22
00 01 00 33 00 02 00 21 00 20 00 00 0C E8 00 01
00 01 00 01 0C E8 0C E8 0C E8 0C E8 0C E8 0C E8
0C E6 0C E6 0C E8 0C E8 0C E8 0C E8 0C E8 0C E8
0C E8 0C E8 0C E8 0C E8 0C E8 0C E8 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 A5 5A 05 82 20 03 00 1F A5 5A
05 82 20 13 07 E0 A5 5A 05 82 20 23 07 E0 A5 5A
05 82 20 33 07 E0 A5 5A 05 82 20 43 07 E0 A5 5A
05 82 20 53 07 E0 A5 5A 05 82 20 63 F8 00 A5 5A
05 82 20 73 07 E0 A5 5A 05 82 20 83 07 E0 A5 5A
05 82 20 93 07 E0 A5 5A 05 82 20 A3 07 E0 A5 5A
05 82 20 B3 07 E0 A5 5A 05 82 20 C3 07 E0 A5 5A
05 82 20 D3 07 E0 A5 5A 05 82 20 E3 07 E0 A5 5A
05 82 20 F3 07 E0 A5 5A 05 82 21 03 07 E0 A5 5A
05 82 21 13 07 E0 A5 5A 05 82 21 23 07 E0 A5 5A
05 82 21 33 07 E0 A5 5A 05 82 21 43 00 00 A5 5A
05 82 21 53 00 00 A5 5A 05 82 21 63 00 00 A5 5A
05 82 21 73 00 00 A5 5A 05 82 21 83 00 00 A5 5A
05 82 22 03 07 E0 A5 5A 05 82 22 13 07 E0 A5 5A
2B 82 13 00 00 03 01 13 00 07 00 27 00 25 00 00
08 EB 0C E8 0C E6 0C E8 00 00 00 00 00 35 00 24
00 02 00 21 00 00 00 01 00 01 00 01
]]

local round = from_hex(ROUND_HEX)
check("样本轮字节数 = 364", #round, 364)

jk.feed(round)
local s = jk.get_state()
check("收到有效轮次", s ~= nil and s.valid, true)
check("总电压 mV", s.batVolMv, 66070)
check("电流 mA(充电为正)", s.batCurrentMa, 3400)
check("SOC", s.soc, 51)
check("最大压差 mV", s.maxDiffMv, 2)
check("MOS 温度", s.tempMosC, 33)
check("电池温度", s.tempBatC, 32)
check("系统警告", s.warn, 0)
check("单体平均 mV", s.cellAvgMv, 3304)
check("均衡开关", s.balanceSw, 1)
check("充电 MOS", s.chargeMos, 1)
check("放电 MOS", s.dischargeMos, 1)
check("串数", s.cellCount, 20)
check("单体1 mV", s.cells[1], 3304)
check("单体2 mV", s.cells[2], 3304)
check("单体7 mV", s.cells[7], 3302)
check("单体8 mV", s.cells[8], 3302)
check("单体20 mV", s.cells[20], 3304)
check("单体21 mV(空槽位)", s.cells[21], 0)
for i = 1, 9 do
    check("报警位 " .. i, s.alarms[i], 0)
end
check("剩余容量 mAh", s.capRemainMah, 27500)
check("SOC 图标格数", s.socIcon, 3)
check("剩余时间·时", s.chargeLeftH, 7)
check("剩余时间·总秒(7:39:37)", s.chargeLeftS, 7 * 3600 + 39 * 60 + 37)
check("功率 0.1W", s.powerW01, 2283)
check("sysAlarm 位图", s.sysAlarm, 0)
check("sysAlarm 已解析", s.sysAlarmValid, true)
check("最高单体 mV", s.cellMaxMv, 3304)
check("最低单体 mV", s.cellMinMv, 3302)
check("颜色=蓝(最高)", s.cellColors[1], 0x001F)
check("颜色=绿(普通)", s.cellColors[2], 0x07E0)
check("颜色=红(最低)", s.cellColors[7], 0xF800)
check("颜色=黑(空槽)", s.cellColors[21], 0x0000)
check("最高单体序号", s.cellMaxIdx, 1)
check("最低单体序号", s.cellMinIdx, 7)
check("颜色帧有效槽位数", s.colorCells, 20)
check("记录块数", s.records, 27)
check("估算总容量 mAh", s.fullCapMah, math.floor(27500 * 100 / 51 + 0.5))
check("统计轮数", jk.get_stats().rounds, 1)
check("收到极空数据次数(=轮数)", jk.get_stats().rounds, 1)
check("扩展块/主块交叉校验无失配", jk.get_stats().crosscheck_mismatch, 0)
check("轮起点(写寄存器)计数", jk.get_stats().reg_writes, 1)

-- 拆包（模拟串口分片）也必须能解析出同一结果
local jk2 = dofile(here .. "/../jk_display.lua")
for i = 1, #round, 37 do
    jk2.feed(round:sub(i, i + 36))
end
local s2 = jk2.get_state()
check("分片接收后串数", s2 and s2.cellCount, 20)
check("分片接收后总压", s2 and s2.batVolMv, 66070)

-- 2.1 0x80 = 写寄存器：寄存器 0x02 的蜂鸣器帧不能当作轮起点
--     （旧实现按"任何 0x80 都是轮起点"处理，会把一轮在轮尾截断）
local jk3 = dofile(here .. "/../jk_display.lua")
jk3.feed(round)
check("基准轮串数", jk3.get_state().cellCount, 20)
local rw0, rounds0 = jk3.get_stats().reg_writes, jk3.get_stats().rounds
local with_buzzer = round:sub(1, 318) .. from_hex("A5 5A 03 80 02 64") .. round:sub(319)
jk3.feed(with_buzzer)
check("蜂鸣器帧后轮数 +1", jk3.get_stats().rounds, rounds0 + 1)
check("蜂鸣器帧不计入轮起点", jk3.get_stats().reg_writes, rw0 + 1)
check("蜂鸣器帧计数", jk3.get_stats().buzzer, 1)
check("蜂鸣器帧后数据完整", jk3.get_state().cellCount, 20)
check("蜂鸣器帧后扩展块有效", jk3.get_state().sysAlarmValid, true)
check("蜂鸣器帧后剩余时间保持", jk3.get_state().chargeLeftH, 7)

-- 2.2 ≥25 串：主块只有 24 槽，第 25 节由 0x2011 帧补发（每轮多 9 字节）
local jk4 = dofile(here .. "/../jk_display.lua")
local function patch_u16(str, off1, value)
    return str:sub(1, off1 - 1)
        .. string.char((value >> 8) & 0xFF, value & 0xFF)
        .. str:sub(off1 + 2)
end
local r25 = round
for i = 21, 24 do
    -- 主块单体 i 的高字节 1 基偏移 = 37 + 2*(i-1)（数据块起点在帧内第 12 字节）
    r25 = patch_u16(r25, 37 + 2 * (i - 1), 3300)
end
r25 = r25:sub(1, 102) .. from_hex("A5 5A 05 82 20 11 0C F7") .. r25:sub(103)
jk4.feed(r25)
local s25 = jk4.get_state()
check("25 串识别", s25 and s25.cellCount, 25)
check("第21节电压 mV", s25 and s25.cells[21], 3300)
check("第25节电压 mV", s25 and s25.cells[25], 3319)
check("第25节帧计数", jk4.get_stats().cell25_frames, 1)

--=============================================================================
-- 3. 金箭 Modbus 从机
--=============================================================================
io.write("== jinjian_slave 应答 ==\n")
local jj = require "jinjian_slave"

-- 3.1 CRC 与文档样例比对（文档 CRC 按 低字节 在前 排列）
check("CRC 01 03 0000 0009 = CC85", jj.crc16(from_hex("01 03 00 00 00 09")), 0xCC85)
check("CRC 01 01 0004 0005 = C8BD", jj.crc16(from_hex("01 01 00 04 00 05")), 0xC8BD)

-- 3.2 周期轮询 0-8
do
    local resp = jj.handle_frame(from_hex("01 03 00 00 00 09 85 CC"))
    check("周期轮询有应答", resp ~= nil, true)
    check("应答字节数 = 3+18+2", #resp, 23)
    check("应答从站/功能码", string.byte(resp, 1) * 256 + string.byte(resp, 2), 0x0103)
    check("应答字节计数", string.byte(resp, 3), 18)
    local regs = {}
    for i = 0, 8 do
        regs[i] = (string.byte(resp, 4 + i * 2) << 8) | string.byte(resp, 5 + i * 2)
    end
    check("reg0 总电压 0.01V", regs[0], 6607)
    check("reg1 串数", regs[1], 20)
    check("reg2 SOC", regs[2], 51)
    check("reg3 容量 Ah", regs[3], 54)
    check("reg4 短路保护", regs[4], 0)
    check("reg5 电流 0.01A", regs[5], 340)
    check("reg6 温度2", regs[6], 32)
    check("reg7 温度1", regs[7], 32)
    check("reg8 板温", regs[8], 33)
end

-- 3.3 详情 0-32（含单体电压）
do
    local resp = jj.handle_frame(from_hex("01 03 00 00 00 21 85 D2"))
    check("详情应答长度 = 3+66+2", #resp, 71)
    local function reg(a) return (string.byte(resp, 4 + a * 2) << 8) | string.byte(resp, 5 + a * 2) end
    check("单体1 mV", reg(9), 3304)
    check("单体7 mV", reg(15), 3302)
    check("单体20 mV", reg(28), 3304)
    check("单体21 mV", reg(29), 0)
end

-- 3.4 电池状态 103-113
do
    local resp = jj.handle_frame(from_hex("01 03 00 67 00 0B B5 D2"))
    check("状态应答长度 = 3+22+2", #resp, 27)
    local function reg(a) return (string.byte(resp, 4 + (a - 103) * 2) << 8) | string.byte(resp, 5 + (a - 103) * 2) end
    check("reg103 电池类型", reg(103), 0)
    check("reg104 循环次数", reg(104), 0)
    check("reg110 均衡状态", reg(110), 1)
    check("reg113 标称容量", reg(113), 54)
end

-- 3.5 PN（1000-1007）为 16 ASCII，形如 SOC-XXXXXXXXXXXX
do
    local resp = jj.handle_frame(from_hex("01 03 03 E8 00 08 C4 7C"))
    local sb = {}
    for i = 0, 7 do
        sb[#sb + 1] = string.char(string.byte(resp, 4 + i * 2))
        sb[#sb + 1] = string.char(string.byte(resp, 5 + i * 2))
    end
    local pn = table.concat(sb)
    check("PN 长度 16", #pn, 16)
    check("PN 前缀", pn:sub(1, 4), "SOC-")
    -- 本用例 stub 的 IMEI = 861234567890123，取后 12 位
    check("PN 内容", pn, "SOC-234567890123")
end

-- 3.6 版本 1016-1017（VERSION = 1.0.0 → 0x0100 / 0x0000）
do
    local resp = jj.handle_frame(from_hex("01 03 03 F8 00 02 45 BE"))
    local v1 = (string.byte(resp, 4) << 8) | string.byte(resp, 5)
    local v2 = (string.byte(resp, 6) << 8) | string.byte(resp, 7)
    check("版本寄存器1", v1, 0x0100)
    check("版本寄存器2", v2, 0x0000)
end

-- 3.7 保护开关量 4-8
do
    local resp = jj.handle_frame(from_hex("01 01 00 04 00 05 BD C8"))
    check("开关量应答长度 = 3+1+2", #resp, 6)
    check("开关量数据 = 0", string.byte(resp, 4), 0)
end

-- 3.8 快充状态（充电中 → 1）
do
    local resp = jj.handle_frame(from_hex("01 01 00 3F 00 01 CD C6"))
    check("快充状态 = 1", string.byte(resp, 4), 1)
end

-- 3.9 写充电时间 / 目标 SOC
do
    local resp = jj.handle_frame(from_hex("01 06 04 41 00 3C D8 FF"))
    check("写充电时间回显", resp and to_hex(resp), "01060441003cd8ff")
    check("充电时间缓存", jj.get_cache().chargeTimeMin, 60)

    local resp2 = jj.handle_frame(from_hex("01 06 04 42 00 5A A8 D5"))
    check("写目标SOC回显", resp2 and #resp2, 8)
    check("目标SOC缓存", jj.get_cache().chargeTargetSoc, 90)

    local resp3 = jj.handle_frame(build_frame("01 06 04 42 00 7B"))
    check("越界值异常码 0x03", resp3 and string.byte(resp3, 3), 0x03)
end

-- 3.10 结束快充 → 快充状态转为 0
do
    -- 注意：文档 §5 该行 CRC(5FA5) 与标准 Modbus CRC 不符，实际应为 8D EE；
    -- 本用例按标准 CRC 组帧（真实主站也是标准 CRC）。
    local req = build_frame("01 05 00 40 FF 00")
    check("结束快充帧 CRC = 8DEE", to_hex(req:sub(-2)), "8dee")
    local resp = jj.handle_frame(req)
    check("结束快充回显", resp and #resp, 8)
    local resp2 = jj.handle_frame(from_hex("01 01 00 3F 00 01 CD C6"))
    check("结束快充后状态 = 0", string.byte(resp2, 4), 0)
end

-- 3.11 非法地址 → 异常码 02；其它从站地址 → 不应答
do
    local resp = jj.handle_frame(build_frame("01 03 00 90 00 01"))
    check("非法地址异常码", resp and string.byte(resp, 3), 0x02)
    local resp2 = jj.handle_frame(from_hex("02 03 00 00 00 09 85 CC"))
    check("其它从站不应答", resp2, nil)
end

-- 3.12 计数统计：中控 BMS 查询次数 / 应答次数
do
    local st = jj.get_stats()
    check("查询计数已累加", st.queries > 0, true)
    check("查询次数 = 应答次数", st.queries, st.responses)
    check("忽略非本机地址帧计数", st.ignored, 1)
    check("收帧数 = 查询 + 忽略", st.rx_frames, st.queries + st.ignored)
    check("最近查询时间已记录", st.last_query_ms ~= nil, true)
end

--=============================================================================
-- 4. 端到端：采样 → 批次 → 压缩编码 → MQTT 发布（用独立解码器核对）
--=============================================================================
io.write("== 端到端上报链路（14×20） ==\n")

-- 4.0 未配置 MQTT 地址 → 关闭 4G 上报（独立加载一份禁用版模块，不影响后面的启用版）
do
    local cfg = require "config"
    cfg.MQTT_HOST = ""
    local off = dofile(here .. "/../bms_uplink.lua")
    pump()
    for _ = 1, 3 do
        jk.feed(round)
    end
    pump()
    check("未配置 MQTT 地址: 上报功能关闭", off.is_enabled(), false)
    check("未配置 MQTT 地址: 不采样", off.get_stats().samples, 0)
    check("未配置 MQTT 地址: 无上报主题", off.topic(), nil)
    check("未配置 MQTT 地址: MQTT 未就绪", off.is_mqtt_ready(), false)
    check("未配置 MQTT 地址: flush 拒绝", (select(2, off.flush())), "mqtt disabled")
    scheduler.clear("JK_ROUND")
    cfg.MQTT_HOST = "mqtt.test.local" -- 场景二：配置了地址 → 正常上报
end

-- 4.1 MQTT 客户端打桩
local published = {}
_G.mqtt = {
    create = function()
        local client = { _cb = nil, _ready = true }
        function client:auth() return true end
        function client:autoreconn() return true end
        function client:keepalive() return true end
        function client:on(cb) self._cb = cb end
        function client:connect()
            if self._cb then self._cb(self, "conack", nil, nil) end
            return true
        end
        function client:ready() return self._ready end
        function client:publish(topic, payload, qos)
            published[#published + 1] = { topic = topic, payload = payload, qos = qos }
            return true
        end
        return client
    end,
}

local uplink = require "bms_uplink"
pump()

-- 丢弃第 2/3 节遗留的 JK_ROUND，保证从干净状态开始统计（LuatOS 的 sys.publish
-- 只投递给当时正在等待的任务，测试里显式清掉遗留事件）
scheduler.clear("JK_ROUND")

-- 4.2 联网 / 对时事件（与 main.lua、ntp_sync.lua 的行为一致）
scheduler.publish("ntp_synced", os.time())
scheduler.publish("net_ready", "TESTDEV123")
scheduler.wake()
check("MQTT 已连接", uplink.is_mqtt_ready(), true)
check("上报主题", uplink.topic(), "/bms/TESTDEV123/status")

-- 4.2.1 上报与采集解耦：尚无任何采集数据时，也会按周期上报兜底批次
check("无采集数据也上报", #published, 1)
check("兜底批次占满一批样本", uplink.get_stats().placeholders, 14)
check("此时真实样本仍为 0", uplink.get_stats().samples, 0)

-- 4.3 喂 14 轮广播（SOC 每轮 +1，其余字段与手册样本一致）
local function set_round_soc(hex_round, soc)
    -- 主数据块 SOC 位于数据块偏移 6，数据块起点为帧内第 12 字节 → 第 19 字节
    return hex_round:sub(1, 18) .. string.char(0, soc) .. hex_round:sub(21)
end

for i = 1, 14 do
    local r = set_round_soc(round, 40 + i)
    jk.feed(r)
    pump()
end
-- 注意：不再调用 scheduler.wake()，否则会额外触发一次兜底上报，破坏下面的计数断言

check("已发布 2 包(兜底+真实)", #published, 2)
local msg = published[2]
check("发布主题", msg and msg.topic, "/bms/TESTDEV123/status")
check("发布 QoS", msg and msg.qos, 0)
check("批次计数(兜底+真实)", uplink.get_stats().batches, 2)
check("真实样本计数", uplink.get_stats().samples, 14)

-- 4.4 独立实现解码器（对照格式规范），解回数据逐条核对
local function get_uvarint(data, pos)
    local value, shift = 0, 0
    while true do
        local b = string.byte(data, pos)
        if not b then return nil, pos, "truncated varint" end
        pos = pos + 1
        value = value | ((b & 0x7f) << shift)
        if (b & 0x80) == 0 then return value, pos end
        shift = shift + 7
        if shift >= 64 then return nil, pos, "varint overflow" end
    end
end

local function unzigzag(u) return (u >> 1) ~ -(u & 1) end

local function decode_one_delta(data, pos)
    local code
    code, pos = get_uvarint(data, pos)
    if code == 1 then return 0, pos end
    if code == 0 or code == 2 then error("run marker invalid for one delta") end
    return unzigzag(code - 2), pos
end

local function decode_delta_sequence(data, pos, expected)
    local deltas, index = {}, 0
    while index < expected do
        local code
        code, pos = get_uvarint(data, pos)
        if code == 1 then
            index = index + 1
            deltas[index] = 0
        elseif code == 0 then
            local run
            run, pos = get_uvarint(data, pos)
            if run < 2 or run > expected - index then error("invalid zero run") end
            for _ = 1, run do index = index + 1; deltas[index] = 0 end
        elseif code == 2 then
            local ed, run
            ed, pos = get_uvarint(data, pos)
            run, pos = get_uvarint(data, pos)
            if ed == 0 or run < 2 or run > expected - index then error("invalid repeat run") end
            local d = unzigzag(ed)
            for _ = 1, run do index = index + 1; deltas[index] = d end
        else
            index = index + 1
            deltas[index] = unzigzag(code - 2)
        end
    end
    return deltas, pos
end

local function decode_column(data, pos, count, signed, mode)
    local values = {}
    local first
    first, pos = get_uvarint(data, pos)
    values[1] = signed and unzigzag(first) or first
    if mode == 2 then
        for i = 2, count do values[i] = values[1] end
        return values, pos
    end
    if mode == 3 then
        if count > 1 then
            local d
            d, pos = decode_one_delta(data, pos)
            for i = 2, count do values[i] = values[i - 1] + d end
        end
        return values, pos
    end
    local deltas
    deltas, pos = decode_delta_sequence(data, pos, count - 1)
    if mode == 0 then
        for i = 2, count do values[i] = values[i - 1] + deltas[i - 1] end
    else
        if count > 1 then
            local d = deltas[1]
            values[2] = values[1] + d
            for i = 3, count do
                d = d + deltas[i - 1]
                values[i] = values[i - 1] + d
            end
        end
    end
    return values, pos
end

local function decode_payload(data)
    local pos = 1
    local n1
    n1, pos = get_uvarint(data, pos)
    local count = n1 + 1
    local header
    header, pos = get_uvarint(data, pos)
    local cellCount = header >> 2
    local hasModes = (header & 1) == 1
    local spatial = (header & 2) == 2
    local columnCount = 9 + cellCount

    local modes = {}
    if hasModes then
        for i = 1, columnCount do
            local bit = (i - 1) * 2
            local byte = string.byte(data, pos + (bit // 8))
            modes[i] = (byte >> (bit % 8)) & 3
        end
        pos = pos + (columnCount * 2 + 7) // 8
    end

    -- 列 1..9 的有符号性（格式规定）；列 10 为无符号基准电压，
    -- 列 11.. 为有符号压差/偏移
    local signed_cols = { true, true, true, false, false, true, false, false, false, false }
    local cols = {}
    for i = 1, columnCount do
        local signed = signed_cols[i] or (i > 10)
        local mode = hasModes and (modes[i] or 0) or 0
        cols[i], pos = decode_column(data, pos, count, signed, mode)
    end
    if pos ~= #data + 1 then
        error("trailing bytes after payload: pos=" .. pos .. " len=" .. #data)
    end

    local cells = {}
    for t = 1, count do
        cells[t] = { cols[10][t] }
        for j = 2, cellCount do
            if spatial then
                cells[t][j] = cells[t][j - 1] + cols[9 + j][t]
            else
                cells[t][j] = cells[t][1] + cols[9 + j][t]
            end
        end
    end
    return {
        count = count, cellCount = cellCount, spatial = spatial,
        temp1 = cols[1], temp2 = cols[2], tempMos = cols[3],
        balanCurrent = cols[4], batVol = cols[5], batCurrent = cols[6],
        socCycleCap = cols[7], socCapRemain = cols[8], time = cols[9],
        cells = cells,
    }
end

local ok_dec, dec = pcall(decode_payload, msg.payload)
check("payload 可被独立解码器解开", ok_dec, true)
if ok_dec then
    check("解码样本数", dec.count, 14)
    check("解码串数", dec.cellCount, 20)
    check("温度列(0.1℃)", dec.temp1[1], 320)
    check("MOS 温度列(0.1℃)", dec.tempMos[14], 330)
    check("总压列 mV", dec.batVol[7], 66070)
    check("电流列 mA", dec.batCurrent[14], 3400)
    check("剩余容量列 mAh", dec.socCapRemain[1], 27500)
    check("单体1 mV", dec.cells[1][1], 3304)
    check("单体7 mV", dec.cells[14][7], 3302)
    check("单体20 mV", dec.cells[14][20], 3304)
    -- 每轮 SOC = 40+i → 估算总容量 = 剩余容量/SOC
    local ok_cap = true
    for i = 1, 14 do
        local want = math.floor(27500 * 100 / (40 + i) + 0.5)
        if dec.socCycleCap[i] ~= want then
            ok_cap = false
            io.write(string.format("       sample %d socCycleCap got %d want %d\n",
                i, dec.socCycleCap[i], want))
        end
    end
    check("总容量列 = 剩余容量/SOC", ok_cap, true)
    io.write(string.format("[info] 报文长度 %d 字节, 电芯布局 %s\n",
        #msg.payload, dec.spatial and "spatial" or "offset"))
else
    io.write("[error] ", tostring(dec), "\n")
end

-- 4.5 兜底批次（无采集数据）同样可被解码：样本数不变，串数沿用最近一次极空快照
do
    local ok_fb, fb = pcall(decode_payload, published[1].payload)
    check("兜底批次可被解码", ok_fb, true)
    if ok_fb then
        check("兜底批次样本数", fb.count, 14)
        check("兜底批次串数(沿用最近)", fb.cellCount, 20)
    end
end

--=============================================================================
-- 汇总
--=============================================================================
io.write(string.format("\n通过 %d 项, 失败 %d 项\n", pass, fail))
if fail > 0 then
    os.exit(1)
end
