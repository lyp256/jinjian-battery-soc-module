--[[
@module  mgmt_uart
@summary 管理链路 UART1（RS485-1）裸帧 RPC 传输层
@version 1.0
@date    2026.10.14
@usage
RS485-1/UART1 上的管理链路：主站（USB-RS485 + `tools/mgmt_probe.ps1`）发**自定义裸帧**，
本机把 mgmt 的响应按同一套帧格式写回。点对点专用线，**没有从站地址/广播**，一主一从、
同一时刻只允许一个请求在飞。

帧格式（双向同一套）：
    SOF(2)=A5 5A | VER(1)=01 | TYPE(1: 1 REQ / 2 RESP / 3 EVT / 4 BLOB) | ID(1) |
    LEN(2, 小端) | PAYLOAD | CRC16(2, 0xA001 低字节在前，覆盖 VER..PAYLOAD)
    REQ  = {"method","params"}；RESP = {"ok":true,"result":{…}} / {"ok":false,"error":{…}}
    EVT  = {"evt":"hello|fota","data":{…}}（ID 固定 0）；BLOB = SEQ(2 LE) | LEN(2 LE) | DATA
    LEN 上限：JSON ≤ MGMT_MAX_PAYLOAD（1024）；BLOB 载荷 ≤ 242（DATA ≤ 238）。

收发规则：
  · 扫描器遇到非 SOF 丢弃并重新同步；CRC 错 / VER 不符 / 超长分别计数丢弃；
  · 回调只 `uart.read` 搬字节 + `sys.publish`，帧扫描/解析/应答都在任务里（见 README §11）；
    突发帧流按 MGMT_PARSE_MAX_FRAMES 分批让出；
  · 回帧先于动作：命令类动作（重启/FOTA）由 mgmt 在响应之后才排期；
  · 事件（hello / fota）只在 UART 传输发，排队后由任务写回（MQTT 纯请求-应答，不发事件）；
  · 单帧处理整体包一层 pcall：坏帧/异常只丢这一帧，绝不把串口任务打死。

引脚：本板 RS485-1 接 UART1（PIN18=UART1_TXD、PIN17=UART1_RXD，默认复用，不用 pins.setup）；
方向脚 EN = GPIO25（MGMT_RS485_DIR），必须显式交给 uart.setup 控制 —— 少了它完全收发不了。

诊断模式（MGMT_ECHO_TEST = true，生产必须 false）：UART1 不跑 RPC，改成"收到什么原样回什么 +
每 N ms 一行心跳"（`[mg echo] 2026-10-14 16:20:00 #12`），只测电气层。

对外接口：
  mgmt_uart.start(opts)              启动单例（opts 可注入 mgmt / cfg；main.lua 用）
  mgmt_uart.new(opts)                造独立传输对象（离线自测）
  transport:start()/:stop()/:handle_frame(frame)/:feed(bytes)/:get_stats()/:echo_mode()
  mgmt_uart.get_stats() / is_enabled() / echo_mode() / stop()   单例快捷方式
  mgmt_uart.build(ftype, id, payload)  组帧（离线自测/工具复用）
  mgmt_uart.crc16(s)                   帧 CRC16（0xA001）
  mgmt_uart.scan(buf, stats, max_payload)  扫帧 → frames, rest
  mgmt_uart.TYPE_REQ / TYPE_RESP / TYPE_EVT / TYPE_BLOB / SOF / VER
]]

local M = {}

local SOF1, SOF2 = 0xA5, 0x5A
M.SOF1, M.SOF2 = SOF1, SOF2
M.VER = 0x01
M.TYPE_REQ, M.TYPE_RESP, M.TYPE_EVT, M.TYPE_BLOB = 0x01, 0x02, 0x03, 0x04
M.BLOB_MAX_DATA = 238
M.BLOB_MAX_PAYLOAD = 4 + M.BLOB_MAX_DATA   -- SEQ(2) + LEN(2) + DATA(≤238) = 242

--=============================================================================
-- CRC16（Modbus 0xA001 多项式，低字节在前）
--=============================================================================
local function crc16(s)
    local crc = 0xFFFF
    for i = 1, #s do
        crc = crc ~ string.byte(s, i)
        for _ = 1, 8 do
            if (crc & 1) ~= 0 then
                crc = (crc >> 1) ~ 0xA001
            else
                crc = crc >> 1
            end
        end
    end
    return crc & 0xFFFF
end
M.crc16 = crc16

local function pack_u16le(v)
    v = v & 0xFFFF
    return string.char(v & 0xFF, (v >> 8) & 0xFF)
end

local function u16le(s, i)
    return (string.byte(s, i) or 0) | ((string.byte(s, i + 1) or 0) << 8)
end

-- 组一帧：TYPE + ID + PAYLOAD（CRC 覆盖 VER..PAYLOAD，低字节在前）
function M.build(ftype, id, payload)
    payload = payload or ""
    local body = string.char(M.VER, ftype & 0xFF, id & 0xFF) .. pack_u16le(#payload) .. payload
    local crc = crc16(body)
    return string.char(SOF1, SOF2) .. body .. pack_u16le(crc)
end

--=============================================================================
-- 帧扫描：从缓冲里切出完整帧；返回 frames, rest（未消费的尾部）
--   frames[i] = { type, id, payload }
--   统计：stats.sync（重同步丢弃的字节）/ stats.crc / stats.ver / stats.overlong
--=============================================================================
local function type_cap(ftype, max_payload)
    if ftype == M.TYPE_BLOB then
        return M.BLOB_MAX_PAYLOAD
    end
    return max_payload
end

function M.scan(buf, stats, max_payload)
    max_payload = max_payload or 1024
    stats = stats or {}
    local frames = {}
    local n = #buf
    local sof_pat = string.char(SOF1, SOF2)
    local i = 1
    while true do
        local sof = string.find(buf, sof_pat, i, true)
        if not sof then
            if i > n then
                return frames, ""            -- 已到末尾，无残留
            end
            -- 没找到 SOF：丢掉除最后一个字节外的全部（末字节可能是 SOF 前半）
            local drop = n - i + 1
            if drop > 1 then
                stats.sync = (stats.sync or 0) + (drop - 1)
            end
            return frames, buf:sub(n)
        end
        if sof > i then
            stats.sync = (stats.sync or 0) + (sof - i)
        end
        if n - sof + 1 < 9 then
            return frames, buf:sub(sof)       -- 头都没齐，等更多字节
        end
        local ftype = string.byte(buf, sof + 3)
        local len = u16le(buf, sof + 5)
        local cap = type_cap(ftype, max_payload)
        if len > cap then
            stats.overlong = (stats.overlong or 0) + 1
            i = sof + 1                       -- 重新同步
        else
            local total = len + 9
            if n - sof + 1 < total then
                return frames, buf:sub(sof)   -- 载荷/CRC 还没齐，等更多字节
            end
            local ver = string.byte(buf, sof + 2)
            if ver ~= M.VER then
                stats.ver = (stats.ver or 0) + 1
                i = sof + 1
            else
                local body = buf:sub(sof + 2, sof + 6 + len)
                local got = u16le(buf, sof + 7 + len)
                if crc16(body) ~= got then
                    stats.crc = (stats.crc or 0) + 1
                    i = sof + 1
                else
                    frames[#frames + 1] = {
                        type = ftype,
                        id = string.byte(buf, sof + 4),
                        payload = buf:sub(sof + 7, sof + 6 + len),
                    }
                    i = sof + total
                end
            end
        end
    end
end

--=============================================================================
-- 传输对象
--=============================================================================
local function json_ok()
    return json ~= nil and type(json.encode) == "function" and type(json.decode) == "function"
end

function M.new(opts)
    opts = opts or {}
    local cfg = opts.cfg or require "config"
    local mgmt = opts.mgmt or require("mgmt").instance

    local UART_ID  = tonumber(cfg.MGMT_UART_ID) or 1
    local BAUD     = tonumber(cfg.MGMT_BAUD) or 115200
    local BUFF     = tonumber(cfg.MGMT_RX_BUFF_SIZE) or 2048
    local MAXFRAME = tonumber(cfg.MGMT_MAX_FRAME_LEN) or 1200
    local MAXPAY   = tonumber(cfg.MGMT_MAX_PAYLOAD) or 1024
    local BUDGET   = tonumber(cfg.MGMT_PARSE_MAX_FRAMES) or 8
    local PIN_RX   = tonumber(cfg.MGMT_PIN_RX)
    local PIN_TX   = tonumber(cfg.MGMT_PIN_TX)
    local DIR      = cfg.MGMT_RS485_DIR or 0xFFFFFFFF
    local RX_LEVEL = tonumber(cfg.MGMT_RS485_RX_LEVEL) or 0
    local DELAY_US = tonumber(cfg.MGMT_RS485_DELAY_US) or 20000
    local ECHO     = (cfg.MGMT_ECHO_TEST == true)
    local HB_MS    = tonumber(cfg.MGMT_ECHO_HEARTBEAT_MS) or 5000

    local function d_info(...) if cfg.LOG_ENABLE then log.info("mg", ...) end end
    local function d_warn(...) if cfg.LOG_ENABLE then log.warn("mg", ...) end end

    local stats = {
        rx_bytes = 0,     -- 串口收到的原始字节数（0 = 主站一个字节都没进来）
        frames = 0,       -- 收到并交给 mgmt 的请求帧数（REQ + BLOB）
        replies = 0,      -- 写回的响应帧数
        evt = 0,          -- 写回的事件帧数（hello / fota）
        tx_bytes = 0,     -- 写回串口的字节数
        crc = 0,          -- CRC 校验失败的帧数
        ver = 0,          -- VER 不符的帧数
        overlong = 0,     -- 超长帧数
        sync = 0,         -- 重同步丢弃的字节数
        drop = 0,         -- 接收缓冲溢出丢弃的字节数
        errors = 0,       -- 处理异常/编码失败次数
        opened = false,
        echo_chunks = 0,  -- 诊断模式回显次数
        heartbeats = 0,   -- 诊断模式心跳个数
        last_error = nil,
        reasons = {},     -- 计数：badjson / badblob / encode / mgmt
    }

    local self = {}
    self.stats = stats
    self.UART_ID = UART_ID
    self.ECHO_MODE = ECHO

    local rx = ""
    local evt_queue = {}
    local running, stopped = false, false

    local function count(reason)
        stats.reasons[reason] = (stats.reasons[reason] or 0) + 1
    end

    local function write_back(bytes)
        if type(bytes) ~= "string" or #bytes == 0 then
            return false
        end
        uart.write(UART_ID, bytes)
        stats.tx_bytes = stats.tx_bytes + #bytes
        return true
    end

    -- 把响应信封编码成 RESP 帧（同一 ID 原样回带）
    local function resp_frame(id, resp)
        if not json_ok() then
            count("encode")
            return nil
        end
        local ok, text = pcall(json.encode, resp)
        if not ok or type(text) ~= "string" then
            count("encode")
            stats.errors = stats.errors + 1
            return nil
        end
        if #text > MAXPAY then
            count("encode")
            stats.errors = stats.errors + 1
            return nil
        end
        return M.build(M.TYPE_RESP, id & 0xFF, text)
    end

    local function evt_frame(evt, data)
        if not json_ok() then
            return nil
        end
        local ok, text = pcall(json.encode, { evt = evt, data = data })
        if not ok or type(text) ~= "string" then
            return nil
        end
        return M.build(M.TYPE_EVT, 0, text)
    end

    local function send_evt(evt, data)
        local f = evt_frame(evt, data)
        if f then
            write_back(f)
            stats.evt = stats.evt + 1
        end
    end

    -- 一帧请求 → 响应帧（UART 与离线测试共用）
    local function dispatch(frame)
        if frame.type == M.TYPE_BLOB then
            local payload = frame.payload
            if #payload < 4 then
                count("badblob")
                return resp_frame(frame.id, { ok = false,
                    error = { code = "EBADFRAME", msg = "blob too short" } })
            end
            local seq = u16le(payload, 1)
            local len = u16le(payload, 3)
            if len < 1 or len > M.BLOB_MAX_DATA or #payload < 4 + len then
                count("badblob")
                return resp_frame(frame.id, { ok = false,
                    error = { code = "EBADFRAME", msg = "bad blob len" } })
            end
            local data = payload:sub(5, 4 + len)
            return resp_frame(frame.id, mgmt:fota_blob(seq, data))
        end
        if frame.type ~= M.TYPE_REQ then
            return nil                        -- 对端发来的 RESP/EVT：忽略
        end
        return resp_frame(frame.id, mgmt:handle(frame.payload, frame.id))
    end

    local function safe_dispatch(frame)
        stats.frames = stats.frames + 1
        local ok, resp = pcall(dispatch, frame)
        if not ok then
            stats.errors = stats.errors + 1
            stats.last_error = tostring(resp)
            count("mgmt")
            d_warn("请求处理异常，已忽略该帧:", resp)
            return nil
        end
        if resp then
            stats.replies = stats.replies + 1
        end
        return resp
    end

    -- 解析缓冲里的完整请求帧并应答；每 BUDGET 帧让出一次
    local function process_rx()
        local budget = BUDGET
        local n = 0
        while #rx >= 9 do
            local frames, rest = M.scan(rx, stats, MAXPAY)
            rx = rest
            if #frames == 0 then
                if #rx > MAXFRAME then
                    local over = #rx - MAXFRAME
                    stats.drop = stats.drop + over
                    rx = rx:sub(-MAXFRAME)
                end
                return
            end
            for _, frame in ipairs(frames) do
                local resp = safe_dispatch(frame)
                if resp then
                    write_back(resp)
                end
                n = n + 1
                if n >= budget then
                    n = 0
                    sys.wait(1)
                end
            end
        end
    end

    --======================= 诊断模式：回环 + 心跳 ========================
    local function echo_drain()
        if #rx == 0 then
            return
        end
        local chunk = rx
        rx = ""
        write_back(chunk)
        stats.echo_chunks = stats.echo_chunks + 1
    end

    local function heartbeat_task()
        while not stopped do
            sys.wait(HB_MS)
            if stopped then return end
            stats.heartbeats = stats.heartbeats + 1
            local ts = os.date("%Y-%m-%d %H:%M:%S", os.time())
            write_back(string.format("[mg echo] %s #%d\r\n", ts, stats.heartbeats))
            if cfg.LOG_ENABLE then
                log.info("mg", "heartbeat", ts, "#" .. stats.heartbeats)
            end
        end
    end

    --======================= 接收回调 ========================
    local on_receive = function(id)
        local got = 0
        while true do
            local s = uart.read(id, 256)
            if not s or #s == 0 then
                break
            end
            got = got + #s
            rx = rx .. s
        end
        if got > 0 then
            stats.rx_bytes = stats.rx_bytes + got
            if #rx > BUFF then
                local over = #rx - BUFF
                stats.drop = stats.drop + over
                d_warn("接收缓冲溢出，丢弃", over, "字节")
                rx = rx:sub(-BUFF)
            end
            sys.publish("MGMT_RX")
        end
    end

    --======================= 事件队列 =======================
    local function on_mgmt_evt(e)
        if type(e) == "table" and e.evt then
            evt_queue[#evt_queue + 1] = e
            -- 唤醒解析任务把事件写出去（否则要等 waitUntil 的 1s 超时）
            if sys and sys.publish then
                pcall(sys.publish, "MGMT_RX")
            end
        end
    end

    local function flush_evts()
        while #evt_queue > 0 do
            local e = table.remove(evt_queue, 1)
            send_evt(e.evt, e.data)
        end
    end

    --======================= 启动 ========================
    self.start = function(_)
        if running then
            return true
        end
        running = true
        stopped = false
        if sys and sys.subscribe then
            pcall(sys.subscribe, "MGMT_EVT", on_mgmt_evt)
        end
        sys.taskInit(function()
            if not (uart and uart.setup) then
                d_warn("uart 库不可用，管理 UART 传输退出")
                return
            end
            local uart_pins = require "uart_pins"
            uart_pins.mux(UART_ID, PIN_RX, PIN_TX, "mg")
            local opened = false
            while not opened and not stopped do
                if uart.setup(UART_ID, BAUD, 8, 1, uart.NONE, uart.LSB,
                        BUFF, DIR, RX_LEVEL, DELAY_US) then
                    uart.on(UART_ID, "receive", on_receive)
                    opened = true
                    stats.opened = true
                    if ECHO then
                        d_info("uart", UART_ID, "opened", BAUD, "8N1 [诊断] 回环+心跳模式",
                            "(MGMT_ECHO_TEST=true)，buf", BUFF, "heartbeat", HB_MS, "ms")
                        sys.taskInit(heartbeat_task)
                    else
                        d_info("uart", UART_ID, "opened", BAUD, "8N1 (mgmt rpc, raw frame), buf", BUFF)
                        -- evt.hello：串口打开后约 1s 发一次（走事件队列，由任务写回）
                        sys.taskInit(function()
                            sys.wait(1000)
                            if not stopped then
                                evt_queue[#evt_queue + 1] = { evt = "hello", data = mgmt:hello() }
                                if sys and sys.publish then
                                    pcall(sys.publish, "MGMT_RX")
                                end
                            end
                        end)
                    end
                else
                    stats.errors = stats.errors + 1
                    d_warn("uart", UART_ID, "open failed, retry")
                    sys.wait(1000)
                end
            end
            while opened and not stopped do
                flush_evts()
                if sys.waitUntil("MGMT_RX", 1000) then
                    if ECHO then echo_drain() else process_rx() end
                elseif ECHO then
                    echo_drain()
                end
            end
        end)
        return true
    end

    self.stop = function(_)
        stopped = true
        running = false
        if sys and sys.unsubscribe then
            pcall(sys.unsubscribe, "MGMT_EVT", on_mgmt_evt)
        end
    end

    -- 离线测试：喂一段字节（走真实扫描/应答路径，但不碰串口）
    self.feed = function(_, bytes)
        rx = rx .. (bytes or "")
        local out = {}
        while true do
            local frames, rest = M.scan(rx, stats, MAXPAY)
            rx = rest
            if #frames == 0 then
                break
            end
            for _, frame in ipairs(frames) do
                local resp = safe_dispatch(frame)
                if resp then
                    out[#out + 1] = resp
                end
            end
        end
        return out
    end

    -- 离线测试：直接处理一帧，返回响应帧（或 nil）
    self.handle_frame = function(_, frame)
        return safe_dispatch({ type = string.byte(frame, 4), id = string.byte(frame, 5),
                               payload = frame:sub(8) })
    end

    self.get_stats = function(_) return stats end
    self.echo_mode = function(_) return ECHO end
    self.reset_stats = function(_)
        for k in pairs(stats) do
            if k ~= "reasons" then stats[k] = 0 end
        end
        stats.opened = false
        stats.last_error = nil
        stats.reasons = {}
        rx = ""
    end
    self.push_evt = function(_, evt, data)
        evt_queue[#evt_queue + 1] = { evt = evt, data = data }
        if sys and sys.publish then
            pcall(sys.publish, "MGMT_RX")
        end
    end
    self.flush_evts = function(_) flush_evts() end
    return self
end

--=============================================================================
-- 单例：main.lua 里 mgmt_uart.start() 接一次
--=============================================================================
function M.start(opts)
    if M.instance then
        return M.instance
    end
    M.instance = M.new(opts)
    M.instance.start()
    return M.instance
end

function M.get_stats()
    return M.instance and M.instance.get_stats() or nil
end

function M.is_enabled()
    return M.instance ~= nil
end

function M.echo_mode()
    return M.instance and M.instance.echo_mode() or false
end

function M.stop()
    if M.instance then
        M.instance.stop()
    end
end

function M.handle_frame(frame)
    if not M.instance then
        return nil
    end
    return M.instance.handle_frame(frame)
end

return M
