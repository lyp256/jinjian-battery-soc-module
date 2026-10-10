-- 离线虚拟时钟仿真：直接跑 air780epm 的真实链路
--     jk_display ──JK_ROUND──► bms_data ──► bms_cache ──► bms_uplink（MessageV1）──► MQTT 打桩
-- 观察两件事：
--   1) 每 REPORT_MS 只上报「本窗口新到达的真实样本」（一批里 Time = BaseTimestamp - ts）；
--   2) 保护板掉线（FEED_END_MS 之后不再喂广播）时只发 13 字节左右的 0 样本空帧，
--      不补零、不插值、不沿用旧值，服务端时间轴不会倒退、不会被伪数据覆盖。
-- 用法: lua hwsim.lua   (在仓库根目录, air780epm 作为 package.path)
--   环境变量：ROUND_MS / REPORT_MS / T_END / FEED_END_MS / HOST
--   DUMP=<file>：把每个上报 payload 的 hex 逐行写文件，便于用服务端解码器（cmd/mock、
--                cmd/agent 或 mqttagent/pkg/bms）再解一遍核对（见 README §9）

local here = (arg and arg[0] or ""):match("^(.*)[/\\][^/\\]*$") or "."
package.path = here .. "/air780epm/?.lua;" .. package.path

local ROUND_MS = tonumber(os.getenv("ROUND_MS") or "") or 2170   -- 一圈广播周期
local REPORT_MS = tonumber(os.getenv("REPORT_MS") or "") or 30000
local T_END = tonumber(os.getenv("T_END") or "") or 900000        -- 仿真时长
local FEED_END = tonumber(os.getenv("FEED_END_MS") or "") or 300000 -- 之后模拟保护板掉线
local HOST = os.getenv("HOST") or "mqtt.test.local"

--=============================================================================
-- 虚拟时钟
--=============================================================================
local V = 0            -- 毫秒
local BASE_EPOCH = 1700000000
os.time = function() return BASE_EPOCH + math.floor(V / 1000) end

_G.mcu = { ticks = function() return V end, hz = function() return 1000 end }
_G.rtos = { bsp = function() return "SIM" end, reboot = function() end }

_G.log = {
    info  = function(...) end,
    warn  = function(...) end,
    error = function(...) end,
    debug = function() end,
}

-- 串口打桩：PC 上打不开（业务任务走重试分支，不参与本仿真）
_G.uart = {
    setup = function() return false end, on = function() end,
    read = function() return "" end, write = function() end, close = function() end,
    LSB = 0, NONE = 0,
}

--=============================================================================
-- 极简协作式调度器（离散事件 + 虚拟时钟）
--=============================================================================
local INF = 1e18
local tasks, published = {}, {}

_G.sys = {}
function _G.sys.taskInit(fn)
    tasks[#tasks + 1] = { co = coroutine.create(fn), resume = V, ev = nil }
end
function _G.sys.wait(ms) coroutine.yield({ wait = ms or 1 }) end
function _G.sys.waitUntil(name, timeout)
    local v = coroutine.yield({ until_ = name, timeout = timeout })
    if v == nil then return false end
    return true, v
end
function _G.sys.publish(name, v)
    if v == nil then v = true end
    published[name] = v
    for _, t in ipairs(tasks) do
        if t.ev == name then t.resume = V end
    end
end
_G.sys.subscribe = function() end
_G.sys.timerStart = function() end
_G.sys.timerLoopStart = function() end

local function run_scheduler()
    local steps = 0
    while true do
        local next_t
        for _, t in ipairs(tasks) do
            if not t.dead and t.resume and (next_t == nil or t.resume < next_t) then
                next_t = t.resume
            end
        end
        if next_t == nil or next_t >= INF or next_t > T_END then
            V = math.min(next_t or T_END, T_END)
            break
        end
        V = next_t
        local ran = true
        while ran do
            ran = false
            for _, t in ipairs(tasks) do
                if not t.dead and t.resume and t.resume <= V then
                    ran = true
                    steps = steps + 1
                    if steps % 200000 == 0 then
                        io.write(string.format("[sched] steps=%d V=%.0fms\n", steps, V))
                        io.flush()
                    end
                    local arg
                    if t.ev then
                        if published[t.ev] ~= nil then
                            arg = published[t.ev]
                            published[t.ev] = nil
                        end
                        t.ev = nil
                    end
                    local ok, y = coroutine.resume(t.co, arg)
                    if not ok then
                        io.write(string.format("[sched] task error @%.0fms: %s\n", V, tostring(y)))
                        t.dead, t.resume = true, nil
                    elseif coroutine.status(t.co) == "dead" then
                        t.dead, t.resume = true, nil
                    elseif y.wait then
                        t.resume = V + y.wait
                    elseif y.until_ then
                        t.ev = y.until_
                        if published[t.ev] ~= nil then
                            t.resume = V
                        else
                            t.resume = y.timeout and (V + y.timeout) or INF
                        end
                    else
                        t.resume = V
                    end
                end
            end
        end
    end
end

--=============================================================================
-- 外设 / 网络打桩
--=============================================================================
_G.board = { set_net_led = function() end, pn = function() return "SOC-000000000000" end }
package.loaded["board"] = _G.board

local published_pkts, auth_args = {}, nil
_G.mqtt = {
    create = function(_, host, port)
        local c = { _cb = nil, host = host, port = port }
        function c:auth(a, b, pw) auth_args = { a, b, pw } return true end
        function c:autoreconn() return true end
        function c:keepalive() return true end
        function c:on(cb) self._cb = cb end
        function c:connect()
            if self._cb then self._cb(self, "conack", nil, nil) end
            return true
        end
        function c:ready() return true end
        function c:publish(topic, payload, qos)
            published_pkts[#published_pkts + 1] = {
                t = V, topic = topic, payload = payload, qos = qos,
            }
            return true
        end
        return c
    end,
}

-- config
local cfg = require "config"
cfg.MQTT_HOST = HOST
cfg.REPORT_PERIOD_MS = REPORT_MS
cfg.LOG_ENABLE = false

-- 极空广播样本（从 tools/selftest.lua 里取 ROUND_HEX）+ 每轮改 SOC 以便区分样本
local jk = require "jk_display"
local ROUND_HEX
do
    local f = assert(io.open(here .. "/air780epm/tools/selftest.lua", "r"))
    local src = f:read("a")
    f:close()
    ROUND_HEX = src:match("ROUND_HEX = %[%[(.-)%]%]")
    assert(ROUND_HEX, "ROUND_HEX not found")
end
local function from_hex(s)
    s = s:gsub("%s+", "")
    return (s:gsub("%x%x", function(h) return string.char(tonumber(h, 16)) end))
end
local round = from_hex(ROUND_HEX)
local function set_soc(h, soc)
    return h:sub(1, 18) .. string.char(0, soc) .. h:sub(21)
end

-- bms_uplink 会连带 require bms_data（JK_ROUND → 规范样本 → bms_cache）
local bms_cache = require "bms_cache"
local uplink = require "bms_uplink"

-- 广播源任务：每 ROUND_MS 喂一轮（SOC 每轮 +1），FEED_END 之后停喂 = 保护板掉线
local soc = 40
sys.taskInit(function()
    while V <= T_END do
        sys.wait(ROUND_MS)
        if V > FEED_END then
            break
        end
        soc = soc + 1
        if soc > 99 then soc = 40 end
        jk.feed(set_soc(round, soc))
    end
end)

-- 启动时序：2s 联网，3s 对时（模拟真实开机）
sys.taskInit(function()
    sys.wait(2000)
    sys.publish("net_ready", "SIMDEVICE")
    sys.wait(1000)
    sys.publish("ntp_synced", os.time())
end)

run_scheduler()

-- DUMP=<file>：把上报 payload 的 hex 逐行写文件（给服务端解码器再验一遍）
local dump_path = os.getenv("DUMP")
if dump_path then
    local df = assert(io.open(dump_path, "w"))
    for _, p in ipairs(published_pkts) do
        df:write((p.payload:gsub(".", function(c) return string.format("%02x", c:byte()) end)) .. "\n")
    end
    df:close()
    io.write(string.format("[dump] %d payloads -> %s\n", #published_pkts, dump_path))
end

--=============================================================================
-- 从 tools/selftest.lua 抽出独立解码器（与上报格式规范对照的实现）
--=============================================================================
local decode_payload = (function()
    local f = assert(io.open(here .. "/air780epm/tools/selftest.lua", "r"))
    local src = f:read("a")
    f:close()
    -- 注意：这里要容忍 CRLF（Windows 上 core.autocrlf=true 时检出的就是 \r\n）。
    -- 原来的写法写死了 "\nend\n"，在 CRLF 的工作区里永远匹配不到，hwsim 直接报
    -- "decoder not found in selftest.lua"；改成 \r?\n 后 LF/CRLF 两种检出都能用。
    local block = src:match("(local function get_uvarint.-\r?\nlocal function decode_payload.-\r?\nend\r?\n)")
    assert(block, "decoder not found in selftest.lua")
    local chunk = assert(load(block .. "\nreturn decode_payload", "decoder", "t",
        { string = string, error = error, math = math, ipairs = ipairs, table = table }))
    return chunk()
end)()

--=============================================================================
-- 结果统计
--=============================================================================
local st = uplink.get_stats()
local cs = bms_cache.stats()
io.write(string.format(
    "\n=== ROUND_MS=%d REPORT_MS=%d 仿真 %.0fs（%.0fs 前有广播, 之后模拟掉线）===\n",
    ROUND_MS, REPORT_MS, T_END / 1000, FEED_END / 1000))
io.write(string.format("MQTT  clientId=%s username=%s password=%s topic=%s\n",
    tostring(auth_args and auth_args[1]), tostring(auth_args and auth_args[2]),
    tostring(auth_args and auth_args[3]), tostring(uplink.topic())))
io.write(string.format(
    "payloads=%d  batches=%d  真实样本=%d  空帧=%d  丢失=%d  published=%d dropped=%d\n",
    #published_pkts, st.batches, st.samples, st.empty_frames, st.lost, st.published, st.dropped))
io.write(string.format("共享缓存 %d/%d 条, 累计写入 %d 条, 覆盖丢弃 %d 条\n",
    cs.count, cs.capacity, cs.pushed, cs.overwritten))

io.write("\n--- 前 20 包 ---\n")
for i = 1, math.min(20, #published_pkts) do
    local p = published_pkts[i]
    local ok, d = pcall(decode_payload, p.payload)
    if not ok then
        io.write(string.format("  t=%9.3fs  DECODE FAIL: %s\n", p.t / 1000, tostring(d)))
    elseif d.count == 0 then
        io.write(string.format("  t=%9.3fs  EMPTY(0 样本)  %4d bytes\n", p.t / 1000, #p.payload))
    else
        local newest = d.baseTimestamp - d.time[d.count]
        local oldest = d.baseTimestamp - d.time[1]
        io.write(string.format(
            "  t=%9.3fs  %3d samples  cell=%d  ts=%d..%d  %s  %4d bytes\n",
            p.t / 1000, d.count, d.cellCount, oldest, newest,
            d.spatial and "spatial" or "offset", #p.payload))
    end
end

--=============================================================================
-- 服务端视角：按真实时间戳（BaseTimestamp - Time）归并，看时间轴是否倒退/有空洞
--=============================================================================
do
    local by_ts = {}       -- ts -> true（后到的覆盖先到的：服务端 upsert 策略）
    local accepted, empties, regressed, overlap = 0, 0, 0, 0
    local last_ts = -1
    for _, p in ipairs(published_pkts) do
        local ok, d = pcall(decode_payload, p.payload)
        if ok and d.count == 0 then
            empties = empties + 1
        elseif ok then
            for k = 1, d.count do
                local ts = d.baseTimestamp - d.time[k]
                if by_ts[ts] ~= nil then
                    overlap = overlap + 1
                end
                by_ts[ts] = true
            end
            local batch_newest = d.baseTimestamp - d.time[d.count]
            if batch_newest <= last_ts then
                regressed = regressed + 1
            end
            last_ts = math.max(last_ts, batch_newest)
            accepted = accepted + 1
        end
    end
    local n_ts, t0, t1 = 0, math.huge, 0
    for ts in pairs(by_ts) do
        n_ts = n_ts + 1
        if ts < t0 then t0 = ts end
        if ts > t1 then t1 = ts end
    end
    local gap_total, gap_max = 0, 0
    if n_ts > 0 then
        t0, t1 = math.floor(t0), math.floor(t1)
        local ts = t0
        while ts <= t1 do
            if by_ts[ts] == nil then
                local n = 1
                while ts + n <= t1 and by_ts[ts + n] == nil do n = n + 1 end
                gap_total = gap_total + n
                if n > gap_max then gap_max = n end
                ts = ts + n
            else
                ts = ts + 1
            end
        end
    end
    io.write(string.format(
        "\n--- 服务端(按 ts 归并) ---\n  非空包 %d（空帧 %d），去重后时间点 %d 个\n",
        accepted, empties, n_ts))
    io.write(string.format(
        "  重复时间戳 %d 个，时间戳倒退的包 %d 个（都应为 0）\n", overlap, regressed))
    if n_ts > 0 then
        io.write(string.format("  时间轴 %d..%d，1s 粒度空洞合计 %ds，最大连续空洞 %ds\n",
            t0, t1, gap_total, gap_max))
    end
end

-- 每个上报周期内发出的包数分布（固定周期上报，应当恒为 1）
local buckets = {}
for _, p in ipairs(published_pkts) do
    local k = math.floor(p.t / REPORT_MS)
    buckets[k] = (buckets[k] or 0) + 1
end
local hist = {}
for _, c in pairs(buckets) do hist[c] = (hist[c] or 0) + 1 end
io.write("\n--- 每个上报周期内发出的包数分布 ---\n")
for c, n in pairs(hist) do
    io.write(string.format("  %d 包/周期 : %d 个周期\n", c, n))
end
