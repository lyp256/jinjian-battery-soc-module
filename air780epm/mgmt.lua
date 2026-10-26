--[[
@module  mgmt
@summary 统一管理服务（传输无关）：方法分发 / 状态快照 / 配置读写 / 命令 / FOTA / ID 去重
@version 1.0
@date    2026.10.14
@usage
一个**统一管理服务** + 两条传输（`mgmt_uart.lua` 裸帧 / `mgmt_mqtt.lua` JSON-RPC）。本模块只做
「方法分发 + 业务动作」，不碰任何串口/MQTT：传输层把收到的请求交给 handle()，拿到响应信封后
自己编码写回。两条传输共用同一套方法、字段与错误码，**没有寄存器表/从站地址**。

    local mgmt = require "mgmt"
    local resp = mgmt.instance:handle('{"method":"sys.ping"}', 7)   -- 7 = UART 帧 ID（可省略）
    -- resp = { ok = true, result = { time = ..., uptime_s = ... } }

方法表（UART 与 MQTT 完全一致）：
    sys.ping          无                -> { time, uptime_s }
    status.get        无                -> 状态大对象（见 build_status）
    config.get        无                -> { values{KEY:val}, restart_keys[], needs_restart, has_override }
    config.set        {KEY:val,...}     -> { applied{...}, restart_required }（全量校验、原子写入）
    cmd.reboot        无                -> { rebooting=true }（回帧后 200ms 重启）
    cmd.factory_reset 无                -> { factory_reset=true }（清 fskv 覆盖 + 置待生效位）
    fota.url          { url }           -> { started=true }
    fota.begin        { total, crc32?}  -> { started=true, total=... }
    fota.chunk        { seq, data }     -> { seq, rx, total, progress }（data = base64，仅 MQTT；UART 走 BLOB）
    fota.abort        无                -> { aborted=bool }
    fota.status       无                -> { mode, state, stage, result, progress, rx, total, last_seq, caps }

错误码（字符串，见 M.ERR）：EBADJSON / EBADFRAME / EBADMETHOD / EBADPARAM / EBADKEY /
    EBADVALUE / EBUSY / ENOLIB / EFLASH / ECRC32 / EBADSEQ / EFOTA / EINTERNAL

事件（仅 UART 传输消费）：本模块通过 `sys.publish("MGMT_EVT", {evt=..., data=...})` 推送
    hello / fota。hello 由传输在串口打开后调用 hello() 主动发；fota 事件在 FOTA 阶段/状态变化时
    推送（失败也发一次）。MQTT 纯请求-应答、不发事件。

ID 去重：同一 ID 在 MGMT_DEDUP_WINDOW_MS（默认 10s）内重复到达且方法相同 → 直接重放缓存响应、
    不重复执行（防止丢应答后主站重试造成二次重启/二次 FOTA）。写入类命令靠「回帧先于动作 +
    去重窗口 + 帧 CRC」保证安全，不做额外布防握手。

对外接口：
    mgmt.instance                      单例（main.lua 直接用；传输层默认用它）
    mgmt.new(opts)                     造独立实例（离线自测 / 多实例）
    instance:handle(payload[, id])     请求 → 响应信封（payload 是 JSON 字符串或已解码的表）
    instance:call(method, params)      直接调用方法 → 响应信封
    instance:fota_blob(seq, data)      UART BLOB 帧 → FOTA 数据块 → 响应信封
    instance:hello()                   evt.hello 的 data
    instance:set_net_ready(bool)       离线自测注入「已联网」（真机由 net_ready 事件设置）
    mgmt.PROTOCOL_VERSION / mgmt.ERR / mgmt.FOTA_BLOB_MAX
]]

local M = {}

M.PROTOCOL_VERSION = 1

-- UART BLOB 帧 DATA 上限（BLOB 载荷 = SEQ(2) + LEN(2) + DATA，载荷 ≤ 242 → DATA ≤ 238）
M.FOTA_BLOB_MAX = 238

--=============================================================================
-- 错误码（字符串）
--=============================================================================
M.ERR = {
    EBADJSON   = "EBADJSON",   -- 载荷不是合法 JSON / 不是对象
    EBADFRAME  = "EBADFRAME",  -- 帧结构非法（长度/类型/CRC 等）
    EBADMETHOD = "EBADMETHOD", -- 未知方法
    EBADPARAM  = "EBADPARAM",  -- 参数缺失 / 类型不符
    EBADKEY    = "EBADKEY",    -- config.set 里有不在白名单的键
    EBADVALUE  = "EBADVALUE",  -- config.set 里有取值非法的键
    EBUSY      = "EBUSY",      -- FOTA 通道正忙
    ENOLIB     = "ENOLIB",     -- 需要的库缺失（libfota2 / 核心库 fota / json）
    EFLASH     = "EFLASH",     -- 写 Flash 失败（fota.run/finish）
    ECRC32     = "ECRC32",     -- 整包 CRC32 不匹配
    EBADSEQ    = "EBADSEQ",    -- 数据块序号/偏移/长度/会话状态不对
    EFOTA      = "EFOTA",      -- FOTA 其它失败 / 通道关闭
    EINTERNAL  = "EINTERNAL",  -- 方法内部异常
}

-- 重启命令的回帧延时：先把响应帧发完（115200 下几十字节 <5ms）再重启
local REBOOT_DELAY_MS = 200

--=============================================================================
-- 小工具
--=============================================================================
local function default_now_ms()
    if mcu and mcu.ticks and mcu.hz then
        local hz = mcu.hz()
        if hz and hz > 0 then
            return math.floor(mcu.ticks() * 1000 / hz)
        end
    end
    if os and os.time then
        return os.time() * 1000
    end
    return 0
end

-- 延时执行：默认开一个任务等 ms 后跑；离线自测里用打桩的 post 观察「回帧先于动作」
local function default_post(fn, ms)
    if sys and sys.taskInit then
        sys.taskInit(function()
            if ms and ms > 0 then
                sys.wait(ms)
            end
            fn()
        end)
    else
        fn()
    end
end

-- 惰性取模块（避免加载顺序耦合；取不到返回 nil）
local function lazy(name)
    local ok, m = pcall(require, name)
    if ok and type(m) == "table" then
        return m
    end
    return nil
end

local function safe_call(obj, method, ...)
    if type(obj) ~= "table" or type(obj[method]) ~= "function" then
        return nil
    end
    local ok, v = pcall(obj[method], ...)
    if ok then
        return v
    end
    return nil
end

local function json_ok()
    return json ~= nil and type(json.encode) == "function" and type(json.decode) == "function"
end

--=============================================================================
-- 实例
--=============================================================================
-- opts（都可省略，真机用默认值；离线自测用来打桩）：
--   cfg/store/fota   注入模块
--   board            注入 board 模块
--   post(fn, ms)     延时执行（默认 sys.taskInit + sys.wait）
--   now_ms()         单调毫秒（默认 mcu.ticks）
--   snapshot()       注入状态大对象（默认惰性 require 各业务模块）
--   no_watch         true = 不订阅 net_ready / fota 事件（离线自测）
--   slow_refresh     true = 起后台任务刷新慢外设（供电电压/PN）
function M.new(opts)
    opts = opts or {}
    local cfg = opts.cfg or require "config"
    local store = opts.store or require "cfg_store"
    local fota = opts.fota or require "fota_mgr"
    local board = opts.board or lazy("board")
    local now_ms = opts.now_ms or default_now_ms
    local post = opts.post or default_post

    local dedup_window_ms = tonumber(cfg.MGMT_DEDUP_WINDOW_MS) or 10000
    local dedup_cap = tonumber(cfg.MGMT_DEDUP_MAX) or 256   -- 去重条目硬上限（防高频唯一 ID 撑大内存）
    local snap_ttl_ms = tonumber(cfg.MGMT_SNAP_TTL_MS) or 1000
    local refresh_ms = tonumber(cfg.MGMT_SLOW_REFRESH_MS) or 5000

    --======================= 运行状态 =======================
    local net_up = false             -- 由 net_ready 事件置位（真机）
    local counters = { requests = 0, errors = 0, dedup_hits = 0, commands = 0, fota_chunks = 0 }
    local dedup = {}                 -- id(字符串) → { method, at, resp }
    local dedup_n = 0

    local function watch_net()
        net_up = true
    end

    if not opts.no_watch and sys and sys.subscribe then
        pcall(sys.subscribe, "net_ready", watch_net)
    end

    --======================= 慢外设（不进请求路径） =======================
    -- board.read_supply_voltage() 里是分次采样 + sys.wait；board.pn() 是 AT 查询。
    -- 一律不进请求路径（否则会把管理服务拖死），改成后台任务定时刷新，方法只读缓存值。
    local supply_mv, pn_cached = nil, nil

    local function refresh_slow_fields()
        if not board then
            return
        end
        -- 供电电压：优先读 board 侧缓存（board 自己的周期任务在刷新），不要在这里
        -- 再开关一次 ADC —— 两个任务并发 open/get/close 同一个 ADC 会交错，读数会抖。
        -- 只有 board 没提供缓存接口（老版本 / 打桩）时才退回自己读。
        if board.supply_voltage then
            local mv = board.supply_voltage()
            if mv then
                supply_mv = mv
            end
        end
        -- 缓存还没填上（开机瞬间 board 的周期任务正在读）时才自己读一次；
        -- board.read_supply_voltage 已做防重入，两个任务并发也不会交错开关 ADC。
        if supply_mv == nil and board.read_supply_voltage then
            local mv = board.read_supply_voltage()   -- 允许挂起：只跑在后台任务里
            if mv then
                supply_mv = mv
            end
        end
        -- PN 是开机常量（IMEI 派生的 SOC-xxxxxxxxxxxx）：取到一次就不要再问了
        -- （board.pn() 会走 mobile.imei()，不同 BSP 可能是缓存也可能是 AT 查询）
        if pn_cached == nil and board.pn then
            local p = board.pn()
            if p and p ~= "" then
                pn_cached = p
            end
        end
    end

    if opts.slow_refresh and sys and sys.taskInit then
        sys.taskInit(function()
            while true do
                refresh_slow_fields()
                sys.wait(refresh_ms)
            end
        end)
    end

    local function socket_ready()
        if socket and socket.dft and socket.adapter then
            local ok, ad = pcall(socket.adapter, socket.dft())
            return ok and ad ~= nil
        end
        return false
    end

    local function mod_stats(name)
        local m = lazy(name)
        if m then
            return safe_call(m, "get_stats")
        end
        return nil
    end

    --======================= 状态快照 =======================
    local snap_cache, snap_at = nil, nil

    local function fota_caps()
        local fi = safe_call(fota, "info")
        return {
            core = (type(fi) == "table" and fi.core) and true or false,
            -- 核心库 fota 的 Lua 类型：nil（没有）/ userdata（LuatOS rotable，正常）/ table
            -- —— 现场用它一眼分清固件到底有没有这个库（见 fota_mgr.lua get_core）
            core_type = (type(fi) == "table" and fi.core_type) or nil,
            lib  = (type(fi) == "table" and fi.lib) and true or false,
            local_enabled = (cfg.MGMT_FOTA_LOCAL_ENABLE ~= false),
            url_enabled   = (cfg.MGMT_FOTA_ENABLE ~= false),
            fskv = (fskv ~= nil),
            json = json_ok(),
        }
    end

    local function fota_status()
        local fi = safe_call(fota, "info") or {}
        return {
            mode = tonumber(fi.mode) or 0,
            state = tonumber(fi.state) or 0,
            stage = tonumber(fi.stage) or 0,
            result = tonumber(fi.result) or 0,
            progress = tonumber(fi.progress) or 0,
            rx = math.max(0, math.floor(tonumber(fi.rx) or 0)),
            total = math.max(0, math.floor(tonumber(fi.total) or 0)),
            last_seq = math.max(0, math.floor(tonumber(fi.last_seq) or 0)),
            caps = fota_caps(),
        }
    end

    local function ident_text(name)
        local v = _G[name]
        if v == nil then
            return ""
        end
        return tostring(v)
    end

    local function fota_busy()
        local fi = safe_call(fota, "info")
        return (type(fi) == "table" and fi.state == 1) and true or false
    end

    local function build_status()
        if opts.snapshot then
            return opts.snapshot()
        end
        local t = now_ms()
        if snap_cache and snap_at and (t - snap_at) < snap_ttl_ms then
            return snap_cache
        end

        local jk = lazy("jk")
        local upl = lazy("bms_uplink")
        local mqtt_state = "disabled"
        if upl and safe_call(upl, "is_enabled") then
            mqtt_state = safe_call(upl, "is_mqtt_ready") and "up" or "down"
        end

        local jk_links = { rounds = 0, frames = 0, rx = 0, sync_lost = 0,
                           valid = (jk and safe_call(jk, "is_fresh")) and true or false }
        local jks = mod_stats("jk")
        if jks then
            jk_links.rounds = tonumber(jks.rounds) or 0
            jk_links.frames = tonumber(jks.frames) or 0
            jk_links.rx = tonumber(jks.rx_bytes) or 0
            jk_links.sync_lost = tonumber(jks.sync_lost) or 0
        end

        local jj_links = { queries = 0, responses = 0, ignored = 0, rx = 0 }
        local jjs = mod_stats("jinjian_slave")
        if jjs then
            jj_links.queries = tonumber(jjs.queries) or 0
            jj_links.responses = tonumber(jjs.responses) or 0
            jj_links.ignored = tonumber(jjs.ignored) or 0
            jj_links.rx = tonumber(jjs.rx_bytes) or 0
        end

        local cache = { count = 0, capacity = 0 }
        local cache_mod = lazy("bms_cache")
        local cs = (cache_mod and safe_call(cache_mod, "stats")) or mod_stats("bms_cache")
        if cs then
            cache.count = tonumber(cs.count) or 0
            cache.capacity = tonumber(cs.capacity) or 0
        end

        local uplink = { published = 0, samples = 0, empty = 0, lost = 0, batches = 0, dropped = 0 }
        local ups = mod_stats("bms_uplink")
        if ups then
            uplink.published = tonumber(ups.published) or 0
            uplink.samples = tonumber(ups.samples) or 0
            uplink.empty = tonumber(ups.empty_frames) or 0
            uplink.lost = tonumber(ups.lost) or 0
            uplink.batches = tonumber(ups.batches) or 0
            uplink.dropped = tonumber(ups.dropped) or 0
        end

        local mgmt_uart = lazy("mgmt_uart")
        local mgmt_mqtt = lazy("mgmt_mqtt")

        local reset_reason = 0
        if pm and pm.lastReson then
            local ok, _, _, r3 = pcall(pm.lastReson)
            if ok then
                reset_reason = tonumber(r3) or 0
            end
        end
        local uptime_s = 0
        if mcu and mcu.ticks and mcu.hz then
            local hz = mcu.hz()
            if hz and hz > 0 then
                uptime_s = math.floor(mcu.ticks() / hz)
            end
        end

        local s = {
            net_ready = (net_up or socket_ready()) and true or false,
            mqtt = mqtt_state,
            jk_valid = jk_links.valid,
            power_on = (board and safe_call(board, "power_output_state")) and true or false,
            needs_restart = (store.needs_restart and store.needs_restart()) and true or false,
            has_override = (store.has_override and store.has_override()) and true or false,
            factory_pending = (store.factory_pending and store.factory_pending()) and true or false,
            fota_busy = fota_busy(),
            reset_reason = reset_reason,
            uptime_s = uptime_s,
            supply_mv = math.max(0, math.floor(tonumber(supply_mv) or 0)),
            device = {
                project = ident_text("PROJECT"),
                model = ident_text("MODEL"),
                version = ident_text("VERSION"),
                pn = pn_cached or "SOC-000000000000",
                proto = M.PROTOCOL_VERSION,
            },
            links = {
                jk = { rounds = jk_links.rounds, frames = jk_links.frames,
                       rx = jk_links.rx, sync = jk_links.sync_lost },
                jinjian = { queries = jj_links.queries, responses = jj_links.responses,
                            ignored = jj_links.ignored, rx = jj_links.rx },
                -- 传输统计只报关键计数：status.get 的整体 JSON 必须 ≤ MGMT_MAX_PAYLOAD(1024)
                mgmt_uart = (function()
                    local s = (mgmt_uart and safe_call(mgmt_uart, "get_stats")) or {}
                    return { rx = tonumber(s.rx_bytes) or 0, frames = tonumber(s.frames) or 0,
                             reply = tonumber(s.replies) or 0, crc = tonumber(s.crc) or 0,
                             sync = tonumber(s.sync) or 0, drop = tonumber(s.drop) or 0,
                             tx = tonumber(s.tx_bytes) or 0, evt = tonumber(s.evt) or 0 }
                end)(),
                mgmt_mqtt = (function()
                    local s = (mgmt_mqtt and safe_call(mgmt_mqtt, "get_stats")) or {}
                    return { rx = tonumber(s.rx) or 0, reply = tonumber(s.reply) or 0,
                             ignored = tonumber(s.ignored) or 0, bad = tonumber(s.bad) or 0 }
                end)(),
            },
            cache = cache,
            uplink = uplink,
            fota = fota_status(),
        }
        snap_cache, snap_at = s, t
        return s
    end

    --======================= 方法实现 =======================
    -- 每个方法返回 result（成功）或 result, error（失败）
    local methods = {}

    local function err(code, msg)
        return nil, { code = code, msg = msg or code }
    end

    methods["sys.ping"] = function()
        return { time = os.time(), uptime_s = build_status().uptime_s }
    end

    methods["status.get"] = function()
        return build_status()
    end

    methods["config.get"] = function()
        local values = {}
        for k in pairs(store.SPEC or {}) do
            if cfg[k] ~= nil then
                values[k] = cfg[k]
            else
                local ov = store.get and store.get(k)
                if ov ~= nil then
                    values[k] = ov
                end
            end
        end
        local restart = {}
        for k in pairs(store.RESTART_KEYS or {}) do
            restart[#restart + 1] = k
        end
        table.sort(restart)
        return {
            values = values,
            restart_keys = restart,
            needs_restart = (store.needs_restart and store.needs_restart()) and true or false,
            has_override = (store.has_override and store.has_override()) and true or false,
        }
    end

    methods["config.set"] = function(params)
        if type(params) ~= "table" then
            return err(M.ERR.EBADPARAM, "params must be an object")
        end
        local n = 0
        for _ in pairs(params) do n = n + 1 end
        if n == 0 then
            return err(M.ERR.EBADPARAM, "empty config.set")
        end
        -- 先全量校验，再原子写入（任一项非法 → 整批不落盘）
        local pending = {}
        for k, v in pairs(params) do
            if not (store.SPEC and store.SPEC[k]) then
                return err(M.ERR.EBADKEY, tostring(k))
            end
            local ok, norm = store.check(k, v)
            if not ok then
                return err(M.ERR.EBADVALUE, tostring(k) .. "=" .. tostring(v))
            end
            pending[k] = norm
        end
        local applied = {}
        for k, v in pairs(pending) do
            store.stage(k, v)
            applied[k] = v
        end
        store.save()
        return {
            applied = applied,
            restart_required = (store.needs_restart and store.needs_restart()) and true or false,
        }
    end

    methods["cmd.reboot"] = function()
        counters.commands = counters.commands + 1
        if cfg.LOG_ENABLE then
            log.warn("mg", "收到重启命令，", REBOOT_DELAY_MS, "ms 后重启模组")
        end
        -- 回帧先于动作：调用方把响应发完之后，延时 200ms 再重启
        post(function()
            if rtos and rtos.reboot then
                rtos.reboot()
            end
        end, REBOOT_DELAY_MS)
        return { rebooting = true }
    end

    methods["cmd.factory_reset"] = function()
        counters.commands = counters.commands + 1
        if cfg.LOG_ENABLE then
            log.warn("mg", "恢复出厂：清持久化覆盖，重启后回默认值")
        end
        if store.reset then
            store.reset()
        end
        return { factory_reset = true }
    end

    local function map_fota_err(why)
        why = tostring(why or "")
        if why == "busy" then return M.ERR.EBUSY end
        if why == "lib missing" then return M.ERR.ENOLIB end
        if why == "run" then return M.ERR.EFLASH end
        if why == "seq" or why == "offset" or why == "length" or why == "state" then
            return M.ERR.EBADSEQ
        end
        if why == "empty url" or why == "bad total" or why == "bad flags" then
            return M.ERR.EBADPARAM
        end
        return M.ERR.EFOTA
    end

    methods["fota.url"] = function(params)
        local url = type(params) == "table" and params.url or nil
        if type(url) ~= "string" or url == "" then
            return err(M.ERR.EBADPARAM, "url required")
        end
        if not fota.start then
            return err(M.ERR.ENOLIB, "fota_mgr missing")
        end
        local ok, why = fota.start(url)
        if not ok then
            return err(map_fota_err(why), tostring(why))
        end
        return { started = true }
    end

    methods["fota.begin"] = function(params)
        local total = type(params) == "table" and tonumber(params.total) or nil
        local crc = type(params) == "table" and tonumber(params.crc32) or 0
        if not total or total <= 0 then
            return err(M.ERR.EBADPARAM, "total required")
        end
        if not fota.local_begin then
            return err(M.ERR.ENOLIB, "fota_mgr missing")
        end
        local ok, why = fota.local_begin(total, crc or 0, 0)
        if not ok then
            return err(map_fota_err(why), tostring(why))
        end
        return { started = true, total = total }
    end

    -- 数据块写入公共路径：UART BLOB 与 MQTT fota.chunk（base64）都走这里。
    -- 偏移由设备「按已收字节自动推得」（主站不传 offset）。
    local function do_fota_chunk(seq, data)
        seq = tonumber(seq)
        if not seq or seq < 1 then
            return err(M.ERR.EBADSEQ, "seq required")
        end
        if type(data) ~= "string" or #data == 0 then
            return err(M.ERR.EBADPARAM, "data required")
        end
        if #data > M.FOTA_BLOB_MAX then
            return err(M.ERR.EBADPARAM, "chunk too long")
        end
        if not fota.local_write then
            return err(M.ERR.ENOLIB, "fota_mgr missing")
        end
        local fi = safe_call(fota, "info") or {}
        local offset = math.max(0, math.floor(tonumber(fi.rx) or 0))
        local ok, why = fota.local_write(seq, offset, data)
        if not ok then
            return err(map_fota_err(why), tostring(why))
        end
        counters.fota_chunks = counters.fota_chunks + 1
        local fi2 = safe_call(fota, "info") or {}
        return {
            seq = seq,
            rx = math.max(0, math.floor(tonumber(fi2.rx) or 0)),
            total = math.max(0, math.floor(tonumber(fi2.total) or 0)),
            progress = tonumber(fi2.progress) or 0,
        }
    end

    methods["fota.chunk"] = function(params)
        if type(params) ~= "table" then
            return err(M.ERR.EBADPARAM, "params required")
        end
        local data = params.data
        if type(data) ~= "string" then
            return err(M.ERR.EBADPARAM, "data (base64) required")
        end
        -- base64 解码由 mgmt_mqtt 提供（仅 MQTT 通道用 fota.chunk；UART 用 BLOB 帧）
        local mq = lazy("mgmt_mqtt")
        local bin
        if type(data) == "string" and mq and type(mq.b64_decode) == "function" then
            bin = mq.b64_decode(data)
        end
        if type(bin) ~= "string" then
            return err(M.ERR.EBADPARAM, "bad base64")
        end
        return do_fota_chunk(params.seq, bin)
    end

    methods["fota.abort"] = function()
        if not fota.local_abort then
            return err(M.ERR.ENOLIB, "fota_mgr missing")
        end
        local ok = fota.local_abort("rpc")
        return { aborted = ok and true or false }
    end

    methods["fota.status"] = function()
        return fota_status()
    end

    local function invoke(method, params)
        local fn = methods[method]
        if not fn then
            return nil, { code = M.ERR.EBADMETHOD, msg = "unknown method: " .. tostring(method) }
        end
        local ok, res, e = pcall(fn, params)
        if not ok then
            return nil, { code = M.ERR.EINTERNAL, msg = tostring(res) }
        end
        if e then
            return nil, e
        end
        return res or {}, nil
    end

    local function envelope(method, params)
        local res, e = invoke(method, params)
        if e then
            counters.errors = counters.errors + 1
            return { ok = false, error = e }
        end
        return { ok = true, result = res }
    end

    --======================= ID 去重 =======================
    local function dedup_prune(t)
        for k, v in pairs(dedup) do
            if (t - v.at) >= dedup_window_ms then
                dedup[k] = nil
                dedup_n = dedup_n - 1
            end
        end
    end

    -- 按容量淘汰最旧：主站用递增 ID 高频请求时，窗口内可能有几百条响应（每条都握着
    -- 一份 status 大表的引用）。窗口清理不够用时，再按条数硬砍，保证内存有硬上限。
    local function dedup_trim(cap)
        while dedup_n > cap do
            local oldest_key, oldest_at = nil, nil
            for k, v in pairs(dedup) do
                if oldest_at == nil or v.at < oldest_at then
                    oldest_key, oldest_at = k, v.at
                end
            end
            if oldest_key == nil then
                dedup_n = 0
                break
            end
            dedup[oldest_key] = nil
            dedup_n = dedup_n - 1
        end
    end

    --======================= 对外：请求 → 响应信封 =======================
    -- payload：JSON 字符串（UART）或已解码的表（MQTT）；id：帧 ID（UART）或 JSON 里的 id（MQTT）
    local function handle(payload, id)
        counters.requests = counters.requests + 1
        local req = payload
        if type(payload) == "string" then
            if not json_ok() then
                return { ok = false, error = { code = M.ERR.ENOLIB, msg = "json missing" } }
            end
            local ok, dec = pcall(json.decode, payload)
            if not ok or type(dec) ~= "table" then
                counters.errors = counters.errors + 1
                return { ok = false, error = { code = M.ERR.EBADJSON, msg = "bad json" } }
            end
            req = dec
        end
        if type(req) ~= "table" then
            counters.errors = counters.errors + 1
            return { ok = false, error = { code = M.ERR.EBADJSON, msg = "bad payload" } }
        end
        local method = req.method
        if type(method) ~= "string" or method == "" then
            counters.errors = counters.errors + 1
            return { ok = false, error = { code = M.ERR.EBADMETHOD, msg = "method required" } }
        end
        local rid = id
        if rid == nil then
            rid = req.id
        end
        local t = now_ms()
        if rid ~= nil then
            local key = tostring(rid)
            local d = dedup[key]
            if d and d.method == method and (t - d.at) < dedup_window_ms then
                counters.dedup_hits = counters.dedup_hits + 1
                if cfg.LOG_ENABLE then
                    log.info("mg", "重复请求 ID", key, method, "→ 重放缓存响应")
                end
                return d.resp
            end
            local resp = envelope(method, req.params)
            if dedup_n > 64 then
                dedup_prune(t)
            end
            if dedup[key] == nil then
                dedup_n = dedup_n + 1
            end
            dedup[key] = { method = method, at = t, resp = resp }
            dedup_trim(dedup_cap)     -- 插完再裁：条目数严格 <= cap（满了丢最旧）
            return resp
        end
        return envelope(method, req.params)
    end

    --======================= 事件 =======================
    local function publish_evt(evt, data)
        if sys and sys.publish then
            pcall(sys.publish, "MGMT_EVT", { evt = evt, data = data })
        end
    end

    local function on_fota_event()
        publish_evt("fota", fota_status())
    end

    if not opts.no_watch and sys and sys.subscribe then
        pcall(sys.subscribe, "fota_started", on_fota_event)
        pcall(sys.subscribe, "fota_done", on_fota_event)
        pcall(sys.subscribe, "fota_local_ready", on_fota_event)
        pcall(sys.subscribe, "fota_local_failed", on_fota_event)
    end

    --======================= hello =======================
    local function hello()
        local st = build_status()
        return {
            project = st.device.project,
            model = st.device.model,
            version = st.device.version,
            pn = st.device.pn,
            reset_reason = st.reset_reason,
            uptime_s = st.uptime_s,
            proto = M.PROTOCOL_VERSION,
            caps = fota_caps(),
        }
    end

    --======================= 组装实例 =======================
    local self = {}
    self.cfg = cfg
    self.store = store
    self.fota = fota
    self.stats = counters
    self.STATE = M.ERR
    self.PROTOCOL_VERSION = M.PROTOCOL_VERSION
    -- 公开方法统一用冒号调用（inst:handle / inst:hello / inst:fota_blob）
    self.handle = function(_, payload, id) return handle(payload, id) end
    self.call = function(_, method, params) return envelope(method, params) end
    -- UART BLOB 帧：SEQ + DATA（偏移由设备按已收字节推得）→ fota_mgr.local_write
    self.fota_blob = function(_, seq, data)
        local res, e = do_fota_chunk(seq, data)
        if e then
            return { ok = false, error = e }
        end
        return { ok = true, result = res }
    end
    self.hello = function(_) return hello() end
    self.build_status = function(_) return build_status() end
    self.refresh_slow_fields = function(_) return refresh_slow_fields() end
    self.set_net_ready = function(_, v) net_up = v and true or false end
    self.is_net_ready = function(_) return net_up or socket_ready() end
    self.emit_fota_evt = function(_) return on_fota_event() end
    self.publish_evt = function(_, evt, data) return publish_evt(evt, data) end
    self.dedup_clear = function(_) dedup = {}; dedup_n = 0 end
    return self
end

--=============================================================================
-- 单例：main.lua / 传输层默认用它
--=============================================================================
M.instance = M.new{ slow_refresh = true }

return M
