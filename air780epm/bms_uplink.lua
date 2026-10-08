--[[
@module  bms_uplink
@summary 电池数据采集聚合 + MQTT 压缩上报（4G 通道）
@version 1.0
@date    2026.09.28
@usage
采集与上报流程（只做压缩上报、无 RPC）：

  极空 BMS 广播一轮 (≈2.17s) ──► jk_display 解析 ──► "JK_ROUND" 事件
        │  一轮 = 一条样本（采集时间粒度以极空广播间隔为准）
        ▼
  环形批次缓冲（cfg.BATCH_SAMPLES 条，默认 14 ≈ 30s）
        │  攒满一批 → 复制到双缓冲快照 → "BMS_BATCH_READY"
        ▼
  bms_codec 列式 Delta+ZigZag+Varint 压缩编码（与 Go 服务端字节级兼容）
        │
        ▼
  MQTT PUBLISH(QoS0) → <prefix>/<device_id>/status

上报与采集解耦：除"攒满一批立即上报"外，另有 REPORT_PERIOD_MS（默认 30s）
  定时冲刷——无论极空是否采集到数据都上报：
    · 周期内有真实样本 → 按实际条数上报（可不足 BATCH_SAMPLES）；
    · 周期内完全没有极空数据 → 用兜底样本填满一批后上报
      （REPORT_HOLD_LAST=true 沿用最近一次极空快照，false 则全 0）。

上报主题：<MQTT_TOPIC_PREFIX>/<IMEI>/status。

4G 上报开关：config.MQTT_HOST 留空（nil 或 ""）表示本机不用 4G 上报，
  此时本模块不启动采样/组批/MQTT 任务，只打印一条提示，其余模块不受影响。

对外接口：
  bms_uplink.is_enabled()  4G 上报是否启用（是否配置了 MQTT 地址）
  bms_uplink.get_stats()  采集/上报统计
  bms_uplink.flush()      立即把当前未满批次上报（调试用）
]]

local cfg = require "config"
local jk = require "jk"          -- 极空数据源（显示屏广播 / RS485 Modbus，按 config 选择）
local codec = require "bms_codec"
local board = require "board"

local M = {}

local BATCH = cfg.BATCH_SAMPLES

--=============================================================================
-- 4G 上报开关：config.MQTT_HOST 为空（nil / "" / 全空白）即视为未配置地址，
-- 此时整个上报链路（采样 → 组批 → 编码 → MQTT）都不启动。
--=============================================================================
local function host_configured(host)
    return type(host) == "string" and host:match("%S") ~= nil
end

local MQTT_ENABLE = host_configured(cfg.MQTT_HOST)
local MQTT_PORT   = tonumber(cfg.MQTT_PORT) or 1883

local function d_info(...) if cfg.LOG_ENABLE then log.info("uplink", ...) end end
local function d_warn(...) if cfg.LOG_ENABLE then log.warn("uplink", ...) end end
local function d_error(...) if cfg.LOG_ENABLE then log.error("uplink", ...) end end

local function to_hex(s)
    return (s:gsub(".", function(c) return string.format("%02x", c:byte()) end))
end

--=============================================================================
-- 采样环形缓冲：扁平数组，避免热路径上产生大量小 table
--   cellVols[(sampleIndex-1)*cellCount + cellIndex]
--=============================================================================
local ring = {
    count = 0,
    time = {}, temp1 = {}, temp2 = {}, tempMos = {}, balanCurrent = {},
    batVol = {}, batCurrent = {}, socCycleCap = {}, socCapRemain = {},
    cellVols = {},
}

-- 双缓冲快照：采样 task 只写 ring，写满后整批复制到 hists[1|2]，
-- 上报 task 读另一个缓冲，互不干扰。
local function new_hist()
    return {
        count = BATCH, cellCount = 0,
        time = {}, temp1 = {}, temp2 = {}, tempMos = {}, balanCurrent = {},
        batVol = {}, batCurrent = {}, socCycleCap = {}, socCapRemain = {},
        cellVols = {},
    }
end
local hists = { new_hist(), new_hist() }
local hist_index = 0

local batch_cell_count = 0 -- 当前批次固定的串数

local stats = {
    samples = 0,        -- 真实采集样本数（一轮极空广播 = 一条）
    placeholders = 0,   -- 无采集数据时用兜底值填充的样本数
    rounds_lost = 0,    -- 采集任务来不及消费而丢掉的极空轮数（sys.publish 只保留最后一次）
    batches = 0,
    published = 0,
    dropped = 0,
    encode_fail = 0,
    last_payload_len = 0,
    last_publish_ms = nil,
}

local function now_ms()
    if mcu and mcu.ticks and mcu.hz then
        return math.floor(mcu.ticks() * 1000 / mcu.hz())
    end
    return os.time() * 1000
end

--=============================================================================
-- MQTT 连接
--=============================================================================
local mqttc = nil
local device_id = nil
local pub_topic = nil
local mqtt_ready = false

local function connect_mqtt()
    if not MQTT_ENABLE then
        return
    end
    -- 等待 4G 拨号完成：限时等待 + 周期提示（sys.waitUntil 挂起协程，不占 CPU；
    -- 加超时只是让现场日志能看出"卡在等网络"，不是阻塞其它任务）
    local ok, id
    repeat
        ok, id = sys.waitUntil("net_ready", 60000)
        if not ok then
            d_warn("waiting for net_ready ...")
        end
    until ok
    device_id = id or "unknown"
    pub_topic = cfg.MQTT_TOPIC_PREFIX .. "/" .. device_id .. "/status"
    d_info("topic", pub_topic)

    if mqtt == nil then
        d_error("本固件未集成 mqtt 库, 无法上报")
        return
    end

    mqttc = mqtt.create(nil, cfg.MQTT_HOST, MQTT_PORT, cfg.MQTT_IS_SSL)
    if not mqttc then
        d_error("mqtt.create failed")
        return
    end
    mqttc:auth(device_id, cfg.MQTT_USER, cfg.MQTT_PASSWORD)
    mqttc:autoreconn(true, cfg.MQTT_RECONN_MS)
    if mqttc.keepalive then
        mqttc:keepalive(cfg.MQTT_KEEPALIVE_S)
    end

    mqttc:on(function(_, event, data)
        if event == "conack" then
            mqtt_ready = true
            d_info("mqtt connected", cfg.MQTT_HOST, MQTT_PORT)
            if board then board.set_net_led("on") end
            sys.publish("mqtt_conack")
        elseif event == "disconnect" then
            mqtt_ready = false
            d_warn("mqtt disconnect, autoreconn in", cfg.MQTT_RECONN_MS, "ms")
            if board then board.set_net_led("blink") end
        elseif event == "error" then
            mqtt_ready = false
            d_warn("mqtt error", data)
        end
    end)

    mqttc:connect()
    -- 最多等 60s CONNACK（只用于日志/串灯提示）。未连上时批次按"未就绪丢弃"处理；
    -- 重连由 mqtt 的 autoreconn 与事件回调负责，本任务到这里即可结束，不留空转协程。
    sys.waitUntil("mqtt_conack", cfg.MQTT_CONNACK_TIMEOUT_MS)
end

--=============================================================================
-- 采样：一轮广播 = 一条样本
--   上报与采集解耦：由 task_report 按 REPORT_PERIOD_MS 定时冲刷，
--   采集不到数据时用兜底样本补齐，保证"无论采集成功与否都按时上报"。
--=============================================================================
local function snapshot_and_notify(n)
    n = n or ring.count
    if n < 1 then
        return
    end
    hist_index = (hist_index % 2) + 1
    local h = hists[hist_index]
    local C = batch_cell_count
    h.count = n
    h.cellCount = C
    for i = 1, n do
        h.time[i] = ring.time[i]
        h.temp1[i] = ring.temp1[i]
        h.temp2[i] = ring.temp2[i]
        h.tempMos[i] = ring.tempMos[i]
        h.balanCurrent[i] = ring.balanCurrent[i]
        h.batVol[i] = ring.batVol[i]
        h.batCurrent[i] = ring.batCurrent[i]
        h.socCycleCap[i] = ring.socCycleCap[i]
        h.socCapRemain[i] = ring.socCapRemain[i]
        local base = (i - 1) * C
        for j = 1, C do
            h.cellVols[base + j] = ring.cellVols[base + j]
        end
    end
    ring.count = 0
    stats.batches = stats.batches + 1
    d_info("batch ready", stats.batches, "samples", n, "cells", C)
    sys.publish("BMS_BATCH_READY", hist_index)
end

-- is_fallback = true 表示本条没有新采集数据（兜底值），单独计数且不改批次串数
local function add_sample(s, is_fallback)
    local C
    if is_fallback and batch_cell_count > 0 then
        C = batch_cell_count   -- 兜底样本沿用当前批次串数，避免打断已有批次
    else
        C = cfg.CELL_COUNT > 0 and cfg.CELL_COUNT or s.cellCount
    end
    if C < 0 then
        C = 0
    end
    if C > jk.MAX_CELLS then
        C = jk.MAX_CELLS
    end
    if batch_cell_count == 0 then
        batch_cell_count = C
    elseif C ~= batch_cell_count then
        -- 串数变化：丢弃未满批次重新开始，保证一批内串数固定
        d_warn("cell count changed", batch_cell_count, "->", C, ", restart batch")
        batch_cell_count = C
        ring.count = 0
    end

    local pos = ring.count + 1
    ring.time[pos] = s.ts
    ring.temp1[pos] = s.tempBatC * 10      -- 0.1℃
    -- 显示屏广播只有一路电池温度（temp1 = temp2）；Modbus 数据源有温度传感器 2
    ring.temp2[pos] = (s.tempBat2C or s.tempBatC) * 10
    ring.tempMos[pos] = s.tempMosC * 10
    ring.balanCurrent[pos] = s.balanCurrentMa or 0 -- 广播里没有均衡电流，Modbus 有
    ring.batVol[pos] = s.batVolMv
    ring.batCurrent[pos] = s.batCurrentMa  -- 充电为正
    ring.socCycleCap[pos] = s.fullCapMah   -- 估算总容量 mAh
    ring.socCapRemain[pos] = s.capRemainMah
    local base = (pos - 1) * batch_cell_count
    for j = 1, batch_cell_count do
        ring.cellVols[base + j] = s.cells[j] or 0
    end
    ring.count = pos
    if is_fallback then
        stats.placeholders = stats.placeholders + 1
    else
        stats.samples = stats.samples + 1
    end

    if ring.count >= BATCH then
        snapshot_and_notify(BATCH)
    end
end

-- 兜底样本：默认沿用最近一次极空快照（保持数值连续），从未收到过则全 0；
-- 时间戳用当前时间，保证上报时间轴连续。
local function fallback_sample(ts)
    local s = jk.get_state()
    if cfg.REPORT_HOLD_LAST ~= false and s and s.cellCount and s.cellCount > 0 then
        return {
            ts = ts,
            cellCount = s.cellCount,
            cells = s.cells,
            batVolMv = s.batVolMv,
            batCurrentMa = s.batCurrentMa,
            soc = s.soc,
            tempBatC = s.tempBatC,
            tempBat2C = s.tempBat2C,
            tempMosC = s.tempMosC,
            balanCurrentMa = s.balanCurrentMa,
            fullCapMah = s.fullCapMah or 0,
            capRemainMah = s.capRemainMah or 0,
        }
    end
    return {
        ts = ts,
        cellCount = 0,
        cells = {},
        batVolMv = 0,
        batCurrentMa = 0,
        soc = 0,
        tempBatC = 0,
        tempMosC = 0,
        fullCapMah = 0,
        capRemainMah = 0,
    }
end

-- 采样任务：首样本前等待 NTP 对时（最多 60s），保证 Time 列准确。
-- 说明（LuatOS 单线程协作式调度）：
--   · sys.waitUntil 只是挂起本协程，事件不来时本任务休眠、不占 CPU，不影响其它任务；
--   · sys.publish 只保留"最后一次"事件值：若本任务未及时消费，中间的轮次会被覆盖丢失。
--     本任务每条样本只做几次表复制（微秒级），正常不会丢；这里统计丢轮数便于现场判断。
local function task_sample()
    sys.waitUntil("ntp_synced", 60000)
    local consumed = jk.get_stats().rounds or 0 -- 以开始消费时的轮数为基准，避免统计开机前的轮次
    while true do
        local ok, s = sys.waitUntil("JK_ROUND")
        if ok and s then
            local total = jk.get_stats().rounds or 0
            if total - consumed > 1 then
                stats.rounds_lost = stats.rounds_lost + (total - consumed - 1)
            end
            consumed = total
            add_sample(s)
        end
    end
end

-- 上报任务：与采集解耦，无论是否采集到数据，每个 REPORT_PERIOD_MS 都冲刷/上报一次
--   ring 里有真实样本 → 按实际条数上报（可不足 BATCH_SAMPLES）；
--   ring 为空（整个周期没有极空数据）→ 用兜底样本填满一批后上报。
local function task_report()
    local period_ms = cfg.REPORT_PERIOD_MS or 30000
    local step_s = math.max(1, math.floor(period_ms / BATCH / 1000))
    while true do
        sys.wait(period_ms)
        if ring.count > 0 then
            snapshot_and_notify(ring.count)
        else
            d_warn("no collection data, report fallback batch")
            local now = os.time()
            for i = 1, BATCH do
                add_sample(fallback_sample(now - (BATCH - i) * step_s), true)
            end
        end
    end
end

--=============================================================================
-- 上报：编码 + MQTT 发布
--=============================================================================
-- 编码分片让出：codec 是纯 CPU 计算（无 IO），长批次/多电芯时同步跑会长时间占住
-- 调度器（实测 14×20≈0.55ms、30×32≈1.3ms 于 PC Lua，模组上还要慢数倍），
-- 这里每编码若干列主动让出一次（cfg.ENCODE_YIELD_MS = 0/nil 时不让出）。
local function encode_yield()
    local ms = cfg.ENCODE_YIELD_MS
    if ms and ms > 0 then
        -- 让出只在协程（任务）里可行：flush() 等调试入口若在非任务上下文被调用，
        -- sys.wait 会报错，这里吞掉异常，退化成同步编码（不影响结果）
        pcall(sys.wait, ms)
    end
end

local function task_publish()
    while true do
        local ok, idx = sys.waitUntil("BMS_BATCH_READY")
        if ok and idx and hists[idx] then
            local payload = codec.encode(hists[idx], encode_yield)
            if payload then
                stats.last_payload_len = #payload
                d_info("batch encoded", #payload, "bytes")
                if cfg.LOG_HEX_PAYLOAD then
                    log.info("uplink.hex", to_hex(payload))
                end
                if mqttc and mqtt_ready and mqttc:ready() then
                    mqttc:publish(pub_topic, payload, cfg.MQTT_QOS)
                    stats.published = stats.published + 1
                    stats.last_publish_ms = now_ms()
                    d_info("published", pub_topic, #payload, "bytes qos", cfg.MQTT_QOS)
                else
                    stats.dropped = stats.dropped + 1
                    d_warn("mqtt not ready, drop batch")
                end
            else
                stats.encode_fail = stats.encode_fail + 1
                d_error("encode failed")
            end
        end
    end
end

--=============================================================================
-- 启动
--=============================================================================
if MQTT_ENABLE then
    sys.taskInit(task_sample)
    sys.taskInit(task_report)
    sys.taskInit(task_publish)
    sys.taskInit(connect_mqtt)
else
    log.info("uplink", "MQTT_HOST 未配置, 关闭 4G 上报（不采样/不组批/不连 MQTT）")
end

--=============================================================================
-- 对外接口
--=============================================================================
function M.is_enabled()
    return MQTT_ENABLE
end

function M.get_stats()
    return stats
end

function M.topic()
    return pub_topic
end

function M.is_mqtt_ready()
    return mqtt_ready
end

-- 立即上报当前未满批次（不足 BATCH 条时按实际条数编码，调试用）
function M.flush()
    if not MQTT_ENABLE then
        return false, "mqtt disabled"
    end
    if ring.count == 0 then
        return false, "no sample"
    end
    local h = new_hist()
    local n = ring.count
    local C = batch_cell_count
    h.count = n
    h.cellCount = C
    for i = 1, n do
        h.time[i] = ring.time[i]
        h.temp1[i] = ring.temp1[i]
        h.temp2[i] = ring.temp2[i]
        h.tempMos[i] = ring.tempMos[i]
        h.balanCurrent[i] = ring.balanCurrent[i]
        h.batVol[i] = ring.batVol[i]
        h.batCurrent[i] = ring.batCurrent[i]
        h.socCycleCap[i] = ring.socCycleCap[i]
        h.socCapRemain[i] = ring.socCapRemain[i]
        local base = (i - 1) * C
        for j = 1, C do
            h.cellVols[base + j] = ring.cellVols[base + j]
        end
    end
    local payload = codec.encode(h, encode_yield)
    if not payload then
        return false, "encode failed"
    end
    if mqttc and mqtt_ready and mqttc:ready() then
        mqttc:publish(pub_topic, payload, cfg.MQTT_QOS)
        stats.published = stats.published + 1
        stats.last_payload_len = #payload
        return true, #payload
    end
    return false, "mqtt not ready"
end

return M
