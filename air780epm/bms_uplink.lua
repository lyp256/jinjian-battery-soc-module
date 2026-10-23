--[[
@module  bms_uplink
@summary 4G MQTT 上报：从共享缓存取「真实样本」编码成 MessageV1，0 样本发空帧
@version 2.0
@date    2026.10.12
@usage
数据流（与 README §1 一致）：

    极空一轮（JK_ROUND）──► bms_data 归一 ──► bms_cache ──since(游标)──► 本模块
                                                                              │
                                            bms_codec(MessageV1) ──► MQTT QoS0
                                                <prefix>/<IMEI>/status

关键约定（见 mqttagent/pkg/bms/bms.md）：
  · 固定 REPORT_PERIOD_MS（默认 30s）一个周期：只上报「本轮窗口内新到达」的真实样本，
    窗口为空则发 13 字节 0 样本帧（flags bit10），**绝不补零/插值/沿用旧值冒充采集数据**；
  · 窗口游标是 bms_cache 的样本序号（不是时间戳），因此同一秒多条、NTP 跳变都不会
    让窗口重叠或丢样；被环形缓冲覆盖而没来得及上报的样本计入 stats.lost；
  · BaseTimestamp = 本批最新样本的 ts，Time = BaseTimestamp - ts（uint8 秒，窗口 ≤30s 远小于
    255s；超出 255s 的样本本批跳过并计入 lost）；一批内串数以最新样本为准，串数不同的
    样本本批跳过（重配串数时丢极少量样本）；
  · 消息级字段：SOCFullChargeCap = 最新样本满电总容量，ModuleUPTime = 模组上电秒数，
    BMSUPTime = 最新样本 BMS 上电秒数（modbus 的 0x12BC RunTime，display 恒 0），
    JinjianModbusRequestCount / JikongModbusRequestCount = 两条从机/主站链路的请求计数；
  · 鉴权：clientId = username = IMEI，password = HMAC-SHA256(key="soc", msg=IMEI) 小写 hex
    （LuatOS 的 mqtt 签名是 auth(client_id, username, password[, clean_session])）；
  · 未 NTP 对时前不发布（时间列与 BaseTimestamp 不可信）；MQTT 未就绪时丢弃并计数；
  · config.MQTT_HOST 留空 = 整条 4G 链路关闭（只连 MQTT/上报，采集与金箭从机照常）。

对外接口：
  bms_uplink.is_enabled()     是否配置了 MQTT 地址
  bms_uplink.get_stats()      上报统计
  bms_uplink.topic()          上报主题（未连接时为 nil）
  bms_uplink.is_mqtt_ready()  MQTT 是否已 CONNACK
  bms_uplink.flush()          立即按当前游标上报一次（调试用）

MQTT 总线（把这一条连接借给第二条传输，见 mgmt_mqtt.lua）：
  bms_uplink.subscribe(topic, fn)        订阅主题，fn(topic, payload) 由下行任务调用
  bms_uplink.publish(topic, payload, qos) 借这条连接发布（未就绪返回 false）
  bms_uplink.client() / device_id()      取连接对象 / 设备 ID（IMEI）
  本机 clientId = username = IMEI，同 broker 同 clientId 只允许一条连接，
  所以第二条传输必须共用这条连接，不能自己 create/connect。
]]

local cfg = require "config"
local cache = require "bms_cache"
local codec = require "bms_codec"
local hmac = require "hmac"
local board = require "board"
local data = require "bms_data" -- ModuleUPTime（mcu.ticks 单调计时）

local M = {}

local REPORT_PERIOD_MS = tonumber(cfg.REPORT_PERIOD_MS) or 30000
local MQTT_QOS = cfg.MQTT_QOS or 0
local HMAC_SECRET = cfg.MQTT_HMAC_SECRET or "soc"

--=============================================================================
-- 4G 上报开关：config.MQTT_HOST 为空（nil / "" / 全空白）即视为未配置地址，
-- 此时不建 MQTT 连接、不上报；采集（bms_data）与金箭从机不受影响。
--=============================================================================
local function host_configured(host)
    return type(host) == "string" and host:match("%S") ~= nil
end

local MQTT_ENABLE = host_configured(cfg.MQTT_HOST)
local MQTT_PORT = tonumber(cfg.MQTT_PORT) or 1883

local function d_info(...) if cfg.LOG_ENABLE then log.info("uplink", ...) end end
local function d_warn(...) if cfg.LOG_ENABLE then log.warn("uplink", ...) end end
local function d_error(...) if cfg.LOG_ENABLE then log.error("uplink", ...) end end

local function to_hex(s)
    return (s:gsub(".", function(c) return string.format("%02x", c:byte()) end))
end

local function now_ms()
    if mcu and mcu.ticks and mcu.hz then
        return math.floor(mcu.ticks() * 1000 / mcu.hz())
    end
    return os.time() * 1000
end

--=============================================================================
-- 统计（stat 日志与现场排查用）
--=============================================================================
local stats = {
    samples = 0,       -- 已纳入上报窗口的真实样本数
    empty_frames = 0,  -- 0 样本空帧数（窗口内没有新样本）
    lost = 0,          -- 未上报而丢失的样本数（缓存覆盖 / 超出 255s / MQTT 未就绪丢弃）
    batches = 0,       -- 生成并尝试上报的报文数（含空帧）
    published = 0,
    dropped = 0,       -- MQTT 未就绪被丢弃的报文数
    encode_fail = 0,
    last_payload_len = 0,
    last_publish_ms = nil,
    last_ts = 0,       -- 最近一次非空报文里的 BaseTimestamp（诊断）
    rx_drop = 0,       -- 下行队列满而丢弃的 MQTT 载荷数（见 deliver）
}

--=============================================================================
-- MQTT 连接（clientId = username = IMEI；password = HMAC-SHA256(key, IMEI)）
--=============================================================================
local mqttc = nil
local device_id = nil
local pub_topic = nil
local mqtt_ready = false
local last_reported_seq = 0  -- 上报游标：只上报该序号之后的样本，窗口不重叠
local last_bms_uptime = 0    -- 最近已知的 BMS 上电秒数（空窗沿用，未知为 0）

--=============================================================================
-- MQTT 总线：把这一条连接借给第二条传输用（管理 MQTT JSON-RPC，见 mgmt_mqtt.lua）
--   本机 clientId = username = device_id（IMEI），同一个 broker 上同一 clientId 只允许一条
--   连接（另开一条会把上报这条踢下线），所以管理 MQTT 不自己建连，而是**共用这一条**：
--     bms_uplink.subscribe(topic, fn)        订阅主题；fn(topic, payload) 由下行任务（task_downlink）调用
--     bms_uplink.publish(topic, payload, qos) 借这条连接发布（MQTT 未就绪返回 false）
--     bms_uplink.client() / device_id() / is_mqtt_ready()
--   订阅在每个 CONNACK 后**重放**一遍：autoreconn 重连后订阅不会丢（见 connect_mqtt）。
--   ⚠ recv 回调（libemqtt 经 msgbus 调起，跑在 rtos 消息泵里、不是协程）只做入队；
--     真正的订阅回调在任务里跑：可以用 sys.wait，但别长时间占住队列（队列上限 16 条，
--     满了丢最旧并计入 stats.rx_drop）。回调异常只丢这一条。
--=============================================================================
local subs = {}                 -- { { topic = string, fn = function } }

-- 下行队列：mqttc:on 的 recv 回调由 libemqtt 经 msgbus 直接调起（跑在 rtos.receive
-- 的消息泵里，**不是协程**）——在同一段里做 json.decode / 状态快照 / 编码会整段占住
-- 调度器，串口接收回调也在同一个消息泵里跑（jk_modbus 512B@115200 只有 ≈44ms 余量）。
-- 所以 recv 里只把「主题 + 载荷」入队并发一个事件，订阅回调统一在 task_downlink 里执行：
--   · 传输层/管理服务的耗时工作不再占用消息泵；
--   · 回调跑在任务里，因此内部可以安全使用 sys.wait / sys.waitUntil；
--   · 先进先出，同一主题的请求顺序不变；队列有上限，满了丢最旧（主站超时重试兜底）。
local RX_QUEUE_MAX = 16         -- 下行队列上限（条）；满则丢最旧，计入 stats.rx_drop
local rx_queue = {}

local function deliver(topic, payload)
    if #rx_queue >= RX_QUEUE_MAX then
        table.remove(rx_queue, 1)
        stats.rx_drop = stats.rx_drop + 1
        d_warn("MQTT 下行队列满（", RX_QUEUE_MAX, "条），丢弃最旧一条")
    end
    rx_queue[#rx_queue + 1] = { topic = topic, payload = payload }
    sys.publish("MQTT_DOWNLINK")
end

-- 把队列里的下行载荷依次交给订阅者（在任务上下文里跑；异常只丢这一条）
local function deliver_pending()
    while #rx_queue > 0 do
        local item = table.remove(rx_queue, 1)
        for _, s in ipairs(subs) do
            if s.topic == item.topic then
                local ok, err = pcall(s.fn, item.topic, item.payload)
                if not ok then
                    d_warn("订阅回调异常，已丢弃该条:", err)
                end
            end
        end
    end
end

-- 下行投递任务：等事件（带超时兜底，事件丢了也不会把队列卡住）
local function task_downlink()
    while true do
        sys.waitUntil("MQTT_DOWNLINK", 10000)
        deliver_pending()
    end
end

-- 逐个订阅已登记的主题（CONNACK 后调用；订阅动作本身失败只记日志，重连后还会再来一遍）
local function replay_subs()
    if mqttc == nil or not mqtt_ready or type(mqttc.subscribe) ~= "function" then
        return
    end
    for _, s in ipairs(subs) do
        pcall(mqttc.subscribe, mqttc, s.topic, cfg.MQTT_QOS or 0)
    end
end

local function connect_mqtt()
    if not MQTT_ENABLE then
        return
    end
    -- 等待 4G 拨号完成：限时等待 + 周期提示（sys.waitUntil 挂起协程，不占 CPU）
    local ok, id
    repeat
        ok, id = sys.waitUntil("net_ready", 60000)
        if not ok then
            d_warn("waiting for net_ready ...")
        end
    until ok

    device_id = tostring(id or (board and board.device_id and board.device_id()) or "unknown")
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

    -- LuatOS mqtt:auth(client_id, username, password[, clean_session])
    -- 本项目的约定：用户名 = clientId = IMEI，密码 = HMAC-SHA256("soc", IMEI) 小写 hex
    local password = hmac.sha256_hex(HMAC_SECRET, device_id)
    mqttc:auth(device_id, device_id, password)
    mqttc:autoreconn(true, cfg.MQTT_RECONN_MS)
    if mqttc.keepalive then
        mqttc:keepalive(cfg.MQTT_KEEPALIVE_S)
    end

    -- LuatOS mqtt 事件回调：function(client, event, data, payload)
    --   conack 后 data = nil；recv 时 data = 主题、payload = 载荷（管理 MQTT JSON-RPC 走这条路进来）
    mqttc:on(function(_, event, data, payload)
        if event == "conack" then
            mqtt_ready = true
            d_info("mqtt connected", cfg.MQTT_HOST, MQTT_PORT, "client", device_id)
            if board then board.set_net_led("on") end
            replay_subs()          -- 重连后订阅要重放（autoreconn 不会替我们记住）
            sys.publish("mqtt_conack")
        elseif event == "recv" then
            deliver(data, payload)
        elseif event == "disconnect" then
            mqtt_ready = false
            d_warn("mqtt disconnect, autoreconn in", cfg.MQTT_RECONN_MS, "ms")
            if board then board.set_net_led("blink") end
        elseif event == "error" then
            mqtt_ready = false
            if data == "conack" then
                d_error("mqtt CONNACK rejected: broker authentication/authorization failed "
                    .. "(MQTT 3.1.1 code 0x05 = not authorized); check clientId, username, "
                    .. "password and broker ACL")
            else
                d_warn("mqtt error", data)
            end
        end
    end)

    -- 连接尝试与超时都打日志：现场"网络起来了但没有任何 mqtt 日志"时，能区分
    -- 「卡在 connect/等 CONNACK」还是「根本没有发起连接」
    d_info("mqtt connecting", cfg.MQTT_HOST, MQTT_PORT,
        (cfg.MQTT_IS_SSL and "ssl" or "tcp"), "client", device_id)
    mqttc:connect()
    -- 最多等 60s CONNACK（只用于日志/串灯提示）。重连由 autoreconn 与事件回调负责。
    if not sys.waitUntil("mqtt_conack", cfg.MQTT_CONNACK_TIMEOUT_MS) then
        d_warn("mqtt conack timeout", cfg.MQTT_CONNACK_TIMEOUT_MS,
            "ms, autoreconn 仍在按", cfg.MQTT_RECONN_MS, "ms 重连")
    end
end

--=============================================================================
-- 上报窗口 → MessageV1
--=============================================================================
-- 编码分片让出：codec 是纯 CPU 计算（无 IO），多电芯/多样本时同步跑会长时间占住调度器，
-- 这里每编码若干列主动让出一次（cfg.ENCODE_YIELD_MS = 0/nil 时不让出）。
local function encode_yield()
    local ms = cfg.ENCODE_YIELD_MS
    if ms and ms > 0 then
        -- flush() 等调试入口若在非任务上下文被调用，sys.wait 会报错，这里吞掉异常，
        -- 退化成同步编码（不影响结果）
        pcall(sys.wait, ms)
    end
end

-- 惰性取对端计数，避免与 jinjian_slave / jk_modbus 的加载顺序耦合；
-- display 模式不加载 jk_modbus（否则会去抢占极空那条 RS485-3 做 115200 轮询）。
local function module_stat(module_name, field)
    local ok, mod = pcall(require, module_name)
    if not ok or type(mod) ~= "table" or type(mod.get_stats) ~= "function" then
        return 0
    end
    local st = mod.get_stats()
    local v = type(st) == "table" and tonumber(st[field]) or nil
    return v or 0
end

local function clamp_u16(v)
    v = math.floor(tonumber(v) or 0)
    if v < 0 then return 0 end
    if v > 0xFFFF then return 0xFFFF end
    return v
end

-- u32 列的上限。不能写成 `v > 0xFFFFFFFF`：Air780EPM 固件的 Lua 是 32 位整数
-- （LUA_32BITS），该字面量会回绕成 -1，比较恒为真 → 每个样本的总压/容量都被钳成
-- -1，无符号列写出 0。32 位固件的整数上限是 2^31-1，远高于真实电压 mV / 容量 mAh。
local U32_MAX = 0x7FFFFFFF

local function clamp_u32(v)
    v = math.floor(tonumber(v) or 0)
    if v < 0 then return 0 end
    if v > U32_MAX then return U32_MAX end
    return v
end

local function clamp_i16(v)
    v = math.floor(tonumber(v) or 0)
    if v < -0x8000 then return -0x8000 end
    if v > 0x7FFF then return 0x7FFF end
    return v
end

local function clamp_i32(v)
    v = math.floor(tonumber(v) or 0)
    if v < -0x80000000 then return -0x80000000 end
    if v > 0x7FFFFFFF then return 0x7FFFFFFF end
    return v
end

-- 把窗口内样本装进编码器要的扁平结构；返回 hist, meta, n, latest_seq
-- 只做「真实样本」的搬运与钳位，不补零、不插值。
local function build_batch()
    local samples, latest_seq, lost = cache.since(last_reported_seq)
    if lost > 0 then
        stats.lost = stats.lost + lost
    end
    if latest_seq < last_reported_seq then
        latest_seq = last_reported_seq
    end

    local newest = cache.latest()
    local cellCount = (newest and newest.cellCount) or 0
    local count = #samples

    -- BaseTimestamp = 本批时序数据里**最新**的采集时间（与 pkg/bms 的定义一致：
    -- "本批时序数据中最新的记录时间"）。正常 ts 单调，就是最后一条样本的 ts；
    -- NTP 微调导致时钟回跳时取窗口内最大值，避免个别样本算出负偏移被丢掉。
    local base_ts = 0
    local full_cap = 0
    if count > 0 then
        for i = 1, count do
            local ts = samples[i].ts or 0
            if ts > base_ts then
                base_ts = ts
            end
        end
        -- SOCFullChargeCap 取最新一条样本（按采集顺序）的满电容量
        full_cap = samples[count].socFullChargeCapMah or 0
    elseif newest then
        base_ts = newest.ts or 0
        full_cap = newest.socFullChargeCapMah or 0
    end
    if newest and (newest.bmsUpTimeS or 0) > 0 then
        last_bms_uptime = newest.bmsUpTimeS
    end

    local hist, n = nil, 0
    if count > 0 then
        hist = {
            count = 0, cellCount = cellCount,
            temp1 = {}, temp2 = {}, tempMos = {}, balanCurrent = {},
            batVol = {}, batCurrent = {}, socCycleCap = {}, socCapRemain = {},
            time = {}, cellVols = {},
        }
        for i = 1, count do
            local s = samples[i]
            if (s.cellCount or 0) == cellCount then
                local off = base_ts - (s.ts or 0)
                if off >= 0 and off <= 255 then
                    n = n + 1
                    hist.temp1[n] = clamp_i16((s.tempBat1C or 0) * 10)   -- int16, 0.1℃
                    hist.temp2[n] = clamp_i16((s.tempBat2C or 0) * 10)
                    hist.tempMos[n] = clamp_i16((s.tempMosC or 0) * 10)
                    hist.balanCurrent[n] = clamp_u16(s.balanCurrentMa or 0) -- uint16, mA
                    hist.batVol[n] = clamp_u32(s.batVolMv or 0)             -- uint32, mV
                    hist.batCurrent[n] = clamp_i32(s.batCurrentMa or 0)     -- int32, mA
                    hist.socCycleCap[n] = clamp_u32(s.socCycleCapMah or 0)  -- uint32, mAh
                    hist.socCapRemain[n] = clamp_u32(s.socCapRemainMah or 0)
                    hist.time[n] = off                                     -- uint8, 距 BaseTimestamp 秒
                    local base = (n - 1) * cellCount
                    for j = 1, cellCount do
                        hist.cellVols[base + j] = clamp_u16(s.cells and s.cells[j] or 0)
                    end
                else
                    -- 超过 uint8 时间窗（正常 30s 窗口不会发生）：只跳过该条，不伪造时间
                    stats.lost = stats.lost + 1
                end
            else
                -- 串数与本批基准不同：本批跳过（下一批按新串数上报）
                stats.lost = stats.lost + 1
            end
        end
        hist.count = n
        if n == 0 then
            hist = nil
        end
    end

    local meta = {
        socFullChargeCap = full_cap,
        baseTimestamp = base_ts,
        moduleUpTime = data.module_uptime_s(),
        bmsUpTime = last_bms_uptime,
        jinjianModbusRequestCount = module_stat("jinjian_slave", "queries"),
        jikongModbusRequestCount = (cfg.JK_PROTOCOL == "modbus")
            and module_stat("jk_modbus", "requests") or 0,
    }
    return hist, meta, n, latest_seq
end

-- 上报一次：窗口非空 → encode；窗口为空 → 13 字节 0 样本帧
local function report_once()
    local hist, meta, n, latest_seq = build_batch()

    local payload
    if n > 0 and hist then
        local ok_enc, res = pcall(codec.encode, hist, meta, encode_yield)
        if ok_enc then
            payload = res
        else
            d_error("encode error", res)
        end
    else
        payload = codec.encode_empty(meta)
    end

    if not payload then
        stats.encode_fail = stats.encode_fail + 1
        d_error("encode failed")
        return false, "encode failed"
    end

    -- 本窗口的真实样本只上报这一次：游标先推进，保证窗口永不重叠
    last_reported_seq = latest_seq
    stats.batches = stats.batches + 1
    stats.last_payload_len = #payload
    if n > 0 then
        stats.samples = stats.samples + n
        stats.last_ts = meta.baseTimestamp or 0
    else
        stats.empty_frames = stats.empty_frames + 1
    end

    if cfg.LOG_HEX_PAYLOAD then
        log.info("uplink.hex", to_hex(payload))
    end

    if mqttc and mqtt_ready and mqttc:ready() then
        mqttc:publish(pub_topic, payload, MQTT_QOS)
        stats.published = stats.published + 1
        stats.last_publish_ms = now_ms()
        d_info("published", pub_topic, #payload, "bytes qos", MQTT_QOS, "samples", n)
        return true, #payload
    end

    stats.dropped = stats.dropped + 1
    if n > 0 then
        stats.lost = stats.lost + n
    end
    d_warn("mqtt not ready, drop report", #payload, "bytes, samples", n)
    return false, "mqtt not ready"
end

--=============================================================================
-- 启动
--=============================================================================
local ntp_ready = false

-- 时间是否已可信：优先看 ntp_synced 事件，其次看 os.time() 是否已是合理 Unix 时间
-- （NTP 成功后系统 RTC 会被更新；事件错过时靠这条兜底，最多等 60s 起步）
local function time_looks_synced()
    local t = os.time()
    return t ~= nil and t > 1600000000
end

-- 上报任务：等 NTP 对时 → 每 REPORT_PERIOD_MS 上报一次当前窗口
local function task_report()
    while not ntp_ready do
        if time_looks_synced() then
            ntp_ready = true
        elseif sys.waitUntil("ntp_synced", 60000) then
            ntp_ready = true
        end
    end
    while true do
        sys.wait(REPORT_PERIOD_MS)
        report_once()
    end
end

if MQTT_ENABLE then
    sys.taskInit(connect_mqtt)
    sys.taskInit(task_report)
else
    log.info("uplink", "MQTT_HOST 未配置, 关闭 4G 上报（采集与金箭从机不受影响）")
end

-- 下行投递任务：订阅回调统一在这里执行（不占 MQTT 的 rtos 消息泵）
sys.taskInit(task_downlink)

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

--=============================================================================
-- MQTT 总线对外接口（管理 MQTT JSON-RPC 共用这条连接，见 mgmt_mqtt.lua）
--=============================================================================
-- 订阅一个主题：fn(topic, payload) 在 recv 事件里被同步调用。
-- 连接已就绪时立即订阅，否则记下主题、CONNACK 后由 replay_subs() 重放（断线重连也不丢）。
function M.subscribe(topic, fn)
    if type(topic) ~= "string" or topic == "" or type(fn) ~= "function" then
        return false
    end
    subs[#subs + 1] = { topic = topic, fn = fn }
    if mqttc and mqtt_ready and type(mqttc.subscribe) == "function" then
        pcall(mqttc.subscribe, mqttc, topic, cfg.MQTT_QOS or 0)
    end
    return true
end

-- 借这条连接发布（MQTT 未就绪返回 false，调用方自己计数/重试）
function M.publish(topic, payload, qos)
    if mqttc == nil or not mqtt_ready then
        return false
    end
    if type(mqttc.ready) == "function" and not mqttc:ready() then
        return false
    end
    return mqttc:publish(topic, payload, qos or MQTT_QOS)
end

function M.client()
    return mqttc
end

function M.device_id()
    return device_id
end

-- 立即按当前游标上报一次（调试用）
function M.flush()
    if not MQTT_ENABLE then
        return false, "mqtt disabled"
    end
    if not ntp_ready and not time_looks_synced() then
        return false, "time not synced"
    end
    return report_once()
end

return M
