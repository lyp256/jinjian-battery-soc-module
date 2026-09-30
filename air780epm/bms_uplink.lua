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

上报主题：<MQTT_TOPIC_PREFIX>/<IMEI>/status。

4G 上报开关：config.MQTT_HOST 留空（nil 或 ""）表示本机不用 4G 上报，
  此时本模块不启动采样/组批/MQTT 任务，只打印一条提示，其余模块不受影响。

对外接口：
  bms_uplink.is_enabled()  4G 上报是否启用（是否配置了 MQTT 地址）
  bms_uplink.get_stats()  采集/上报统计
  bms_uplink.flush()      立即把当前未满批次上报（调试用）
]]

local cfg = require "config"
local jk = require "jk_display"
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
    samples = 0,
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
    local _, id = sys.waitUntil("net_ready")
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
    -- 最多等 60s CONNACK；未连上时批次按"未就绪丢弃"处理，autoreconn 会继续重连
    sys.waitUntil("mqtt_conack", cfg.MQTT_CONNACK_TIMEOUT_MS)
    while true do
        sys.wait(60000)
    end
end

--=============================================================================
-- 采样：一轮广播 = 一条样本
--=============================================================================
local function snapshot_and_notify()
    hist_index = (hist_index % 2) + 1
    local h = hists[hist_index]
    local C = batch_cell_count
    h.count = BATCH
    h.cellCount = C
    for i = 1, BATCH do
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
    d_info("batch ready", stats.batches, "samples", BATCH, "cells", C)
    sys.publish("BMS_BATCH_READY", hist_index)
end

local function add_sample(s)
    local C = cfg.CELL_COUNT > 0 and cfg.CELL_COUNT or s.cellCount
    if C <= 0 then
        return
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
    ring.temp2[pos] = s.tempBatC * 10      -- 极空广播只有一路电池温度
    ring.tempMos[pos] = s.tempMosC * 10
    ring.balanCurrent[pos] = 0             -- 广播里没有均衡电流
    ring.batVol[pos] = s.batVolMv
    ring.batCurrent[pos] = s.batCurrentMa  -- 充电为正
    ring.socCycleCap[pos] = s.fullCapMah   -- 估算总容量 mAh
    ring.socCapRemain[pos] = s.capRemainMah
    local base = (pos - 1) * batch_cell_count
    for j = 1, batch_cell_count do
        ring.cellVols[base + j] = s.cells[j] or 0
    end
    ring.count = pos
    stats.samples = stats.samples + 1

    if ring.count >= BATCH then
        snapshot_and_notify()
    end
end

-- 采样任务：首样本前等待 NTP 对时（最多 60s），保证 Time 列准确
local function task_sample()
    sys.waitUntil("ntp_synced", 60000)
    while true do
        local ok, s = sys.waitUntil("JK_ROUND")
        if ok and s then
            add_sample(s)
        end
    end
end

--=============================================================================
-- 上报：编码 + MQTT 发布
--=============================================================================
local function task_publish()
    while true do
        local ok, idx = sys.waitUntil("BMS_BATCH_READY")
        if ok and idx and hists[idx] then
            local payload = codec.encode(hists[idx])
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

-- 周期状态日志（便于现场观察采集/上报是否正常）
sys.taskInit(function()
    while true do
        sys.wait(60000)
        local jks = jk.get_stats()
        d_info("status", "rounds", jks.rounds, "samples", stats.samples,
            "batches", stats.batches, "published", stats.published,
            "dropped", stats.dropped,
            "mqtt", (not MQTT_ENABLE) and "disabled" or (mqtt_ready and "up" or "down"))
    end
end)

--=============================================================================
-- 启动
--=============================================================================
if MQTT_ENABLE then
    sys.taskInit(task_sample)
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
    local payload = codec.encode(h)
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
