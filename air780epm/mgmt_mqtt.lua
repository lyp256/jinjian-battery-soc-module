--[[
@module  mgmt_mqtt
@summary 管理链路 MQTT JSON-RPC 传输：通道 = 主题后缀 rpc（与 4G 上报共用同一条连接）
@version 1.0
@date    2026.10.14
@usage
与管理链路 UART1（RS485-1 裸帧 RPC）并列的**第二条管理传输**：主站不接 485，而是把
JSON-RPC 请求经 MQTT 发下来，本机把响应经 MQTT 发回。方法表、字段、错误码与 UART 传输
完全一致（由同一个 `mgmt` 服务实现），本模块只做「收 MQTT 报文 → 交给 mgmt → 把响应发回」。

通道（主题后缀）约定 —— 与上报通道 `status` 并列，管理通道的后缀就是 `rpc`：
  下行（主站 → 设备）：`<MGMT_MQTT_SUFFIX>` = `<MQTT_TOPIC_PREFIX>/<device_id>/rpc`
  上行（设备 → 主站）：`<MQTT_TOPIC_PREFIX>/<device_id>/rpc/resp`
后缀由 MGMT_MQTT_SUFFIX 配置（默认 "rpc"）；响应主题固定 = 订阅主题 + "/resp"。
每消息一条请求、回一条响应，事务号用 JSON 里的 id（设备原样回带）。**MQTT 纯请求-应答、不发事件**。

连接：与 4G 上报（bms_uplink）**共用同一条 MQTT 连接**——本机 clientId = username = device_id
（IMEI），同一个 broker 上同一 clientId 只允许一条连接，另开一条会把上报那条踢掉。所以本模块
**不自己 create/connect**，只向 bms_uplink 注册「这个主题的回调」，断线重连后的订阅由
bms_uplink 在每个 CONNACK 后重放（见 bms_uplink.subscribe）。

开关：MGMT_MQTT_ENABLE（默认 true）**且** MQTT_HOST 已配置才启动；任一不满足都只打日志，
不影响 UART1 裸帧 RPC，也不影响上报。broker 需放行 `.../rpc`（订阅）与 `.../rpc/resp`（发布）。

对外接口：
  mgmt_mqtt.is_enabled()                      是否启用（MGMT_MQTT_ENABLE）
  mgmt_mqtt.start(opts)                       接线：等 net_ready 后注册订阅（main.lua 用）
  mgmt_mqtt.get_stats()                       单例统计（未 start 时为 nil）
  mgmt_mqtt.is_open()                         单例是否已订阅
  mgmt_mqtt.new(opts)                         造传输对象（opts 可注入 link/mgmt/board，便于自测）
  mgmt_mqtt.new_transport(opts)               同 new，但省略的 link/board 自动取 bms_uplink / board
  mgmt_mqtt.topics(device_id)                 返回 (下行主题, 上行主题)
  mgmt_mqtt.suffix()                          当前通道后缀
  mgmt_mqtt.b64_decode(s) / b64_encode(s)     base64 编解码（fota.chunk 的 data 字段）
  transport:open([device_id])                 注册订阅（未启用/无链路/取不到设备 ID → false + 原因）
  transport:on_message(payload)               收到一条载荷：交给 mgmt + 把响应发回（返回最后响应表）
  transport:send(bytes) / transport:get_stats() / transport:close()
]]

local M = {}

--=============================================================================
-- base64（纯 Lua，避免依赖固件裁剪情况；url-safe 也兼容）
--=============================================================================
local B64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"
local B64_MAP = {}
for i = 1, #B64 do
    B64_MAP[B64:sub(i, i)] = i - 1
end

function M.b64_decode(s)
    if type(s) ~= "string" then
        return nil
    end
    s = s:gsub("%s+", ""):gsub("-", "+"):gsub("_", "/"):gsub("=+$", "")
    if #s % 4 == 1 then
        return nil
    end
    local out = {}
    local acc, bits = 0, 0
    for i = 1, #s do
        local v = B64_MAP[s:sub(i, i)]
        if not v then
            return nil
        end
        acc = (acc << 6) | v
        bits = bits + 6
        if bits >= 8 then
            bits = bits - 8
            out[#out + 1] = string.char((acc >> bits) & 0xFF)
            acc = acc & ((1 << bits) - 1)
        end
    end
    return table.concat(out)
end

function M.b64_encode(s)
    if type(s) ~= "string" then
        return nil
    end
    local out = {}
    local i = 1
    while i <= #s do
        local b1 = string.byte(s, i)
        local b2 = string.byte(s, i + 1)
        local b3 = string.byte(s, i + 2)
        local n = (b1 << 16) | ((b2 or 0) << 8) | (b3 or 0)
        out[#out + 1] = B64:sub(((n >> 18) & 0x3F) + 1, ((n >> 18) & 0x3F) + 1)
        out[#out + 1] = B64:sub(((n >> 12) & 0x3F) + 1, ((n >> 12) & 0x3F) + 1)
        out[#out + 1] = b2 and B64:sub(((n >> 6) & 0x3F) + 1, ((n >> 6) & 0x3F) + 1) or "="
        out[#out + 1] = b3 and B64:sub((n & 0x3F) + 1, (n & 0x3F) + 1) or "="
        i = i + 3
    end
    return table.concat(out)
end

--=============================================================================
-- 传输对象
--=============================================================================
local function json_ok()
    return json ~= nil and type(json.encode) == "function" and type(json.decode) == "function"
end

-- 通道后缀：空/缺省都回退 rpc，并去掉前后多余的斜杠
local function norm_suffix(v)
    v = tostring(v or ""):gsub("^/+", ""):gsub("/+$", "")
    if v == "" then
        v = "rpc"
    end
    return v
end

function M.new(opts)
    opts = opts or {}
    local cfg = opts.cfg or require "config"
    local mgmt = opts.mgmt or require("mgmt").instance

    local ENABLED = (cfg.MGMT_MQTT_ENABLE == true)
    local SUFFIX = norm_suffix(cfg.MGMT_MQTT_SUFFIX)
    local QOS = cfg.MQTT_QOS or 0
    local MAX_FRAMES = tonumber(cfg.MGMT_MQTT_MAX_FRAMES) or 8

    local stats = {
        opened = false,
        rx = 0,          -- 收到的 MQTT 载荷数
        reply = 0,       -- 成功回发的响应数
        ignored = 0,     -- 未回发的载荷数（解码失败 / 无 id / 编码失败）
        bad = 0,         -- 无法解析（非 JSON / 非对象）的载荷数
        errors = 0,      -- 内部异常（mgmt 抛异常 / 无引擎）
        dropped = 0,     -- MQTT 未就绪被丢弃的响应数
        last_error = nil,
    }

    local self = {}
    self.stats = stats
    self.link = opts.link
    self.board = opts.board
    self.device_id = nil
    self.sub_topic, self.resp_topic = nil, nil
    self.registered_topic = nil
    self.ENABLED = ENABLED

    local function d_info(...) if cfg.LOG_ENABLE then log.info("mgmqtt", ...) end end
    local function d_warn(...) if cfg.LOG_ENABLE then log.warn("mgmqtt", ...) end end

    function self:is_enabled() return ENABLED end
    function self:suffix() return SUFFIX end
    function self:topics(device_id)
        if device_id == nil then
            device_id = self.device_id
        end
        local base = (cfg.MQTT_TOPIC_PREFIX or "/bms") .. "/" .. tostring(device_id) .. "/" .. SUFFIX
        return base, base .. "/resp"
    end
    function self:get_stats() return stats end
    function self:is_open() return stats.opened end

    -- 注册订阅（不建连：连接与重连后的订阅重放都由 bms_uplink 负责）
    function self:open(device_id)
        if not ENABLED then
            d_info("管理 MQTT RPC 未启用（MGMT_MQTT_ENABLE = false），不订阅")
            return false, "disabled"
        end
        local link = self.link
        if link == nil or type(link.subscribe) ~= "function" or type(link.publish) ~= "function" then
            d_warn("管理 MQTT RPC 缺少可用的 MQTT 链路（MQTT_HOST 未配置？）")
            return false, "no link"
        end
        local id = device_id
        if (id == nil or id == "") and type(link.device_id) == "function" then
            id = link.device_id()
        end
        if (id == nil or id == "") and self.board and type(self.board.device_id) == "function" then
            id = self.board.device_id()
        end
        id = tostring(id or "")
        if id == "" or id == "unknown" then
            d_warn("管理 MQTT RPC 取不到设备 ID，暂不订阅")
            return false, "no device id"
        end
        self.device_id = id
        self.sub_topic, self.resp_topic = self:topics(id)
        if self.sub_topic == self.registered_topic then
            stats.opened = true
            return true
        end
        link.subscribe(self.sub_topic, function(_topic, payload) return self:on_message(payload) end)
        self.registered_topic = self.sub_topic
        stats.opened = true
        d_info("管理 MQTT RPC 已订阅", self.sub_topic, "→", self.resp_topic)
        return true
    end

    -- 一条 MQTT 载荷 = 一条 JSON-RPC 请求；返回响应表（或 nil）
    function self:on_message(payload)
        stats.rx = stats.rx + 1
        if not json_ok() then
            stats.errors = stats.errors + 1
            stats.last_error = "json missing"
            return nil
        end
        if type(payload) ~= "string" or #payload == 0 then
            stats.bad = stats.bad + 1
            return nil
        end
        local ok, req = pcall(json.decode, payload)
        if not ok or type(req) ~= "table" then
            stats.bad = stats.bad + 1
            return nil
        end
        local resp
        local hok, r = pcall(mgmt.handle, mgmt, req, req.id)
        if not hok or type(r) ~= "table" then
            stats.errors = stats.errors + 1
            stats.last_error = tostring(r)
            return nil
        end
        resp = r
        -- 事务号原样回带（id 可为数字/字符串；缺省就不带）
        if req.id ~= nil then
            local out = { id = req.id }
            for k, v in pairs(resp) do
                out[k] = v
            end
            resp = out
        end
        self:send(resp)
        return resp
    end

    -- 把响应表编码成 JSON 发到上行主题（借 bms_uplink 的那条连接发布）
    function self:send(resp)
        if not json_ok() then
            stats.ignored = stats.ignored + 1
            return false
        end
        local ok, bytes = pcall(json.encode, resp)
        if not ok or type(bytes) ~= "string" then
            stats.ignored = stats.ignored + 1
            stats.last_error = tostring(bytes)
            return false
        end
        local link = self.link
        if link and type(link.publish) == "function" and self.resp_topic then
            local pok, res = pcall(link.publish, self.resp_topic, bytes, QOS)
            if pok and res ~= false then
                stats.reply = stats.reply + 1
                return true
            end
        end
        stats.dropped = stats.dropped + 1
        d_warn("MQTT 未就绪，管理 RPC 响应无法回发")
        return false
    end

    function self:close()
        stats.opened = false
        return true
    end

    return self
end

--=============================================================================
-- 单例接线：main.lua 里 mgmt_mqtt.start() 接一次
--=============================================================================
local function default_link()
    local ok, m = pcall(require, "bms_uplink")
    if ok and type(m) == "table" then
        return m
    end
    return nil
end

function M.new_transport(opts)
    opts = opts or {}
    if opts.link == nil then
        opts.link = default_link()
    end
    if opts.board == nil then
        local ok, b = pcall(require, "board")
        opts.board = ok and b or nil
    end
    return M.new(opts)
end

-- 等 4G 就绪（net_ready 带设备 ID）后注册订阅；未启用/未配置 MQTT 时只打日志
function M.start(opts)
    if M.instance then
        return M.instance
    end
    opts = opts or {}
    local t = M.new_transport(opts)
    M.instance = t
    local cfg = opts.cfg or require "config"
    if not t:is_enabled() then
        if cfg.LOG_ENABLE then log.info("mgmqtt", "管理 MQTT RPC 未启用（MGMT_MQTT_ENABLE = false）") end
        return t
    end
    if t.link == nil or (type(t.link.is_enabled) == "function" and not t.link.is_enabled()) then
        if cfg.LOG_ENABLE then
            log.warn("mgmqtt", "MGMT_MQTT_ENABLE = true，但 MQTT_HOST 未配置：管理 MQTT RPC 不启动")
        end
        return t
    end
    sys.taskInit(function()
        local ok, id
        repeat
            ok, id = sys.waitUntil("net_ready", 60000)
            if not ok and cfg.LOG_ENABLE then
                log.warn("mgmqtt", "管理 MQTT RPC 等待 net_ready ...")
            end
        until ok
        t:open(id)
    end)
    return t
end

function M.get_stats()
    return M.instance and M.instance.get_stats() or nil
end

function M.is_enabled()
    return M.instance and M.instance.is_enabled() or (require("config").MGMT_MQTT_ENABLE == true)
end

function M.is_open()
    return (M.instance ~= nil) and M.instance.stats.opened or false
end

function M.suffix()
    if M.instance then
        return M.instance:suffix()
    end
    return norm_suffix(require("config").MGMT_MQTT_SUFFIX)
end

function M.topics(device_id)
    local t = M.instance or M.new{ link = {}, board = {} }
    return t:topics(device_id)
end

return M
