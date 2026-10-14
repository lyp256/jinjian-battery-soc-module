--[[
@module  cfg_store
@summary 运行期配置持久化：把管理链路（UART1 RPC / MQTT RPC）写入的配置项落到 fskv，重启后仍生效
@version 1.0
@date    2026.10.14
@usage
数据流：

    main.lua  ──require "config"──► require("cfg_store").load()  ──► 覆盖 config 表
                                                  ▲    │
                          mgmt（config.set / 恢复出厂）│    ▼
                                          fskv（key = CFG_STORE_FSKV_KEY，JSON 文本）

    · load()  开机在**任何业务模块 require 之前**调用一次：把 fskv 里的覆盖项按白名单
              校验后写回 config 表，非法项跳过并打日志；
    · set()   写透：改 config + 立即落盘 + 置「需重启」（只对 RESTART_KEYS 里的键）；
    · stage() 只改 config + 记脏标记，不落盘；随后的 save() 一次落盘（一次 config.set
              批量写只碰一次 fskv，避免逐键重复写）；
    · reset() 恢复出厂：清掉 fskv 覆盖、置「恢复出厂待生效」标记（重启后回默认值）。

白名单 / 类型 / 范围（`SPEC`）是唯一的校验入口：不在表里的键永远不会被写入 config，
类型或范围非法的项被跳过（返回 false 并打日志）。

fskv 不可用（老固件没带核心库 / 写失败）时只告警一次，退化为「不持久化」：
本次运行的配置写入仍然生效，只是重启后丢失，不会抛异常、不影响其它链路。

对外接口：
  cfg_store.load()                 开机加载覆盖项，返回成功应用的项数
  cfg_store.set(key, value)        写透（校验 + 改 config + 落盘 + 需重启标记）
  cfg_store.stage(key, value)      只改 config + 记脏（批量写用，需 save()）
  cfg_store.save()                 把当前覆盖项落盘一次（无脏标记时是空操作）
  cfg_store.reset()                恢复出厂（清覆盖 + 置待生效标记）
  cfg_store.info()                 状态快照（供 mgmt.status.get）
  cfg_store.has_override()         是否存在持久化覆盖
  cfg_store.needs_restart()        是否有需重启才生效的改动
  cfg_store.factory_pending()      是否有恢复出厂待生效
  cfg_store.get(key)               当前覆盖值（无覆盖返回 nil）
  cfg_store.SPEC / RESTART_KEYS    白名单与「需重启」键集合（供自测/文档核对）
]]

local cfg = require "config"

local M = {}

local KEY     = cfg.CFG_STORE_FSKV_KEY or "jjcfg"
local ENABLED = (cfg.CFG_STORE_ENABLE ~= false)

local function d_info(...) if cfg.LOG_ENABLE ~= false then log.info("cfg", ...) end end
local function d_warn(...) if cfg.LOG_ENABLE ~= false then log.warn("cfg", ...) end end

--=============================================================================
-- 白名单：键 → 类型 / 范围（不在表里的键一律拒绝）
--   类型：bool / int（取整） / float（保留小数，如快充倍率 0.5C） / enum / string
--=============================================================================
local BAUD_ENUM = { 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200 }

M.SPEC = {
    -- 管理链路自身（[R]）
    MGMT_BAUD                  = { type = "enum", values = BAUD_ENUM },
    -- 实时生效
    LOG_ENABLE                 = { type = "bool" },
    JK_MODBUS_POLL_MS          = { type = "int", min = 100, max = 60000 },
    JJ_LOW_TEMP_CHARGE_C       = { type = "int", min = -100, max = 100 },
    JJ_LOW_TEMP_DISCHARGE_C    = { type = "int", min = -100, max = 100 },
    -- 快充判定倍率（C）：判定电流 mA = 倍率 × 电池容量(Ah)，1 = 1C；小数（0.5C）用 float
    JJ_FAST_CHARGE_C_RATE      = { type = "float", min = 0, max = 10 },
    -- [R] 重启后生效
    REPORT_PERIOD_MS           = { type = "int", min = 1000, max = 3600000 },
    STATUS_REPORT_MS           = { type = "int", min = 1000, max = 3600000 },
    JK_PROTOCOL                = { type = "enum", values = { "display", "modbus" } },
    JK_MODBUS_SLAVE_ADDR       = { type = "int", min = 1, max = 247 },
    MQTT_PORT                  = { type = "int", min = 1, max = 65535 },
    MQTT_IS_SSL                = { type = "bool" },
    MQTT_QOS                   = { type = "enum", values = { 0, 1 } },
    MQTT_KEEPALIVE_S           = { type = "int", min = 1, max = 65535 },
    MQTT_HOST                  = { type = "string", maxlen = 64 },
    MQTT_TOPIC_PREFIX          = { type = "string", maxlen = 64 },
}

-- 需要重启才生效的键（其余白名单键写入即生效）
M.RESTART_KEYS = {
    MGMT_BAUD = true,
    REPORT_PERIOD_MS = true,
    STATUS_REPORT_MS = true,
    JK_PROTOCOL = true,
    JK_MODBUS_SLAVE_ADDR = true,
    MQTT_PORT = true,
    MQTT_IS_SSL = true,
    MQTT_QOS = true,
    MQTT_KEEPALIVE_S = true,
    MQTT_HOST = true,
    MQTT_TOPIC_PREFIX = true,
}

--=============================================================================
-- 运行状态
--=============================================================================
local overrides = {}      -- 已持久化的覆盖项（键 → 值）
local dirty = false       -- 有未落盘的覆盖项
local needs_restart = false
local factory_pending = false
local available = (fskv ~= nil) -- fskv 是否可用（首次写失败会置 false）
local warned = false

local function json_ok()
    return json ~= nil and type(json.encode) == "function" and type(json.decode) == "function"
end

-- 首次使用时初始化 fskv（部分固件需要显式 init；失败不致命）
local function fskv_init()
    if fskv and type(fskv.init) == "function" then
        pcall(fskv.init)
    end
end

local function warn_once(...)
    if warned then
        return
    end
    warned = true
    d_warn(...)
end

--=============================================================================
-- 校验
--=============================================================================
local function validate(spec, value)
    if spec.type == "bool" then
        if type(value) == "boolean" then
            return true, value
        end
        if type(value) == "number" then
            return true, (value ~= 0)
        end
        return false
    elseif spec.type == "int" then
        local n = tonumber(value)
        if n == nil then
            return false
        end
        n = math.floor(n)
        if (spec.min ~= nil and n < spec.min) or (spec.max ~= nil and n > spec.max) then
            return false
        end
        return true, n
    elseif spec.type == "float" then
        local n = tonumber(value)
        if n == nil then
            return false
        end
        if (spec.min ~= nil and n < spec.min) or (spec.max ~= nil and n > spec.max) then
            return false
        end
        return true, n
    elseif spec.type == "enum" then
        for _, v in ipairs(spec.values) do
            if v == value then
                return true, v
            end
        end
        -- 允许数字枚举用字符串传来（JSON 往返）
        if type(value) == "string" then
            local n = tonumber(value)
            if n ~= nil then
                for _, v in ipairs(spec.values) do
                    if v == n then
                        return true, v
                    end
                end
            end
        end
        return false
    elseif spec.type == "string" then
        if type(value) ~= "string" then
            return false
        end
        if spec.maxlen ~= nil and #value > spec.maxlen then
            return false
        end
        return true, value
    end
    return false
end

-- 覆盖项 → JSON 文本（失败返回 nil）
local function encode_overrides()
    if not json_ok() then
        return nil
    end
    local ok, text = pcall(json.encode, overrides)
    if ok and type(text) == "string" then
        return text
    end
    return nil
end

--=============================================================================
-- 持久化
--=============================================================================
-- 落盘（有脏标记时才真正写）；返回 true = 已落盘或本来就不需要落盘
function M.save()
    if not ENABLED or not dirty then
        return true
    end
    if not (fskv and type(fskv.set) == "function") then
        available = false
        dirty = false
        warn_once("fskv 不可用，配置改动本次运行有效但重启后丢失（不持久化）")
        return false
    end
    fskv_init()
    local text = encode_overrides()
    if text == nil then
        available = false
        dirty = false
        warn_once("json 库不可用，配置改动无法持久化（本次运行仍生效）")
        return false
    end
    local ok, err = pcall(fskv.set, KEY, text)
    if not ok or err == false then
        dirty = false
        available = false
        warn_once("fskv.set 失败，配置改动无法持久化（本次运行仍生效）:", err)
        return false
    end
    dirty = false
    return true
end

--=============================================================================
-- 写配置
--=============================================================================
-- 只改 config + 记覆盖 + 记脏（批量写用；由 save() 统一落盘）
-- 返回 true / false（非法键或非法值）
function M.stage(key, value)
    local spec = M.SPEC[key]
    if not spec then
        d_warn("拒绝写入未列入白名单的配置项", tostring(key))
        return false
    end
    local ok, val = validate(spec, value)
    if not ok then
        d_warn("配置项取值非法，拒绝写入", tostring(key), tostring(value))
        return false
    end
    cfg[key] = val
    overrides[key] = val
    dirty = true
    if M.RESTART_KEYS[key] then
        needs_restart = true
    end
    d_info("配置写入", key, "=", tostring(val), M.RESTART_KEYS[key] and "(需重启)" or "")
    return true
end

-- 只校验、不改状态（引擎的 dry 校验用）；返回 (true, 规范化后的值) / false
function M.check(key, value)
    local spec = M.SPEC[key]
    if not spec then
        return false
    end
    return validate(spec, value)
end

-- 写透：改 config + 立即落盘（单点写入用）
function M.set(key, value)
    if not M.stage(key, value) then
        return false
    end
    M.save()
    return true
end

--=============================================================================
-- 开机加载 / 恢复出厂
--=============================================================================
local function decode_stored(raw)
    if type(raw) == "table" then
        return raw
    end
    if type(raw) ~= "string" or #raw == 0 then
        return nil
    end
    if not json_ok() then
        return nil
    end
    local ok, dec = pcall(json.decode, raw)
    if ok and type(dec) == "table" then
        return dec
    end
    return nil
end

-- 开机加载：把 fskv 覆盖项按白名单写回 config；返回应用成功的项数
function M.load()
    if not ENABLED then
        d_info("配置持久化未启用（CFG_STORE_ENABLE = false）")
        return 0
    end
    if not (fskv and type(fskv.get) == "function") then
        available = false
        d_warn("本固件没有 fskv，配置持久化关闭（始终用 config.lua 的默认值）")
        return 0
    end
    fskv_init()
    local ok, raw = pcall(fskv.get, KEY)
    if not ok then
        available = false
        d_warn("fskv.get 失败，配置持久化关闭:", raw)
        return 0
    end
    if raw == nil then
        d_info("无持久化配置，使用 config.lua 默认值")
        return 0
    end
    local data = decode_stored(raw)
    if type(data) ~= "table" then
        d_warn("持久化配置格式非法，忽略:", tostring(raw))
        return 0
    end
    local applied, skipped = 0, 0
    local keys = {}
    for k in pairs(data) do
        keys[#keys + 1] = k
    end
    table.sort(keys)
    for _, k in ipairs(keys) do
        local spec = M.SPEC[k]
        if spec then
            local okv, val = validate(spec, data[k])
            if okv then
                cfg[k] = val
                overrides[k] = val
                applied = applied + 1
            else
                skipped = skipped + 1
                d_warn("持久化配置项非法，跳过:", tostring(k), tostring(data[k]))
            end
        else
            skipped = skipped + 1
            d_warn("持久化配置项不在白名单，跳过:", tostring(k))
        end
    end
    d_info("已应用持久化配置项", applied, "项（跳过", skipped, "项）")
    return applied
end

-- 恢复出厂：清 fskv 覆盖 + 置「恢复出厂待生效」标记（重启后回 config.lua 默认值）
function M.reset()
    overrides = {}
    dirty = true
    factory_pending = true
    local ok = true
    if fskv and type(fskv.del) == "function" then
        local pok, perr = pcall(fskv.del, KEY)
        if not pok or perr == false then
            ok = M.save()
        else
            dirty = false
        end
    else
        ok = M.save()
    end
    d_info("恢复出厂：已清除持久化覆盖，重启后回默认值（del ok =", tostring(ok), ")")
    return true
end

--=============================================================================
-- 状态查询
--=============================================================================
function M.info()
    local n = 0
    for _ in pairs(overrides) do
        n = n + 1
    end
    return {
        available = available and true or false,
        enabled = ENABLED and true or false,
        has_override = n > 0,
        override_count = n,
        needs_restart = needs_restart,
        factory_pending = factory_pending,
        fskv_available = available and true or false,
    }
end

function M.has_override()
    return next(overrides) ~= nil
end

function M.needs_restart()
    return needs_restart
end

function M.set_needs_restart(v)
    needs_restart = v and true or false
end

function M.factory_pending()
    return factory_pending
end

function M.set_factory_pending(v)
    factory_pending = v and true or false
end

function M.get(key)
    return overrides[key]
end

function M.key()
    return KEY
end

-- 离线自测用：清空内存状态（不清 fskv）
function M.reset_memory()
    overrides = {}
    dirty = false
    needs_restart = false
    factory_pending = false
    warned = false
    available = (fskv ~= nil)
end

return M
