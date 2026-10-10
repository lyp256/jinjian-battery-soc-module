--[[
@module  fota_mgr
@summary FOTA 升级管理：① 主站下发 URL（libfota2 下载）② 主站经管理链路分段上传本地升级包
@version 1.0
@date    2026.10.14
@usage
本模块管理两种互斥的升级通道（原模块名 `fota.lua`，**改名**是因为 LuatOS 核心库里也有一个
全局 `fota`（本地升级/差分写入），同名会让 require 与核心库互相遮蔽）：

  ① URL 通道：主站下发 fota.url（URL）→ 封装官方扩展库
     `libfota2.request(cb, { url = <主站给的 URL>, progress = <可选进度回调> })`
     → 下载 + **成功不自动重启**（置状态 2 + 「需重启」，等主站发重启命令）；
  ② 本地通道：主站用 fota.begin + fota.chunk（MQTT）/ BLOB 帧（UART）把 LuatOS `.bin`
     升级载荷**分段流式**写进设备：核心库 `fota.init()` → 反复 `fota.run(buf)` →
     `fota.isDone()` → `fota.finish(ok)` 直接写 Flash，**不联网、不落文件**；收齐且
     （可选）CRC32 校验通过后同样只置「成功待重启」，由主站下发现有重启命令 0x0001。
     · 用法照官方 demo（`demo/fota/fota(使用fota核心库)/fota_uart.lua`、
       `demo/fota/fota(使用fota核心库)/fota_file.lua`、`demo/fota/ble_5101_fota/`）：
       `fota.init()` 后**轮询 `fota.wait()` 等底层就绪**（`while not fota.wait() do sys.wait(100) end`）。
       本模块把它做成**有上限**的等待（MGMT_FOTA_LOCAL_WAIT_MS，默认 3s）：就绪就进 ready，
       超时只告警仍继续（避免个别固件 wait() 语义不同把升级卡死）；自测里设 0 = 跳过；
     · `fota.run(buf)` 的 buf 可以是字符串（BLE 分段 demo 就是传字符串，UART demo 传 zbuff），
       返回 `(无异常, 收到最后一块, 未写 Flash 的缓存字节)` —— 第二值作为收齐信号
       （另加"已收字节 ≥ 声明总长"兜底），第三值作为未落盘缓存上报；
     · `fota.isDone()` 返回 `(无异常, 写到最后一块)`，以第二个值为准。
     · ⚠ 核心库 `fota` 在 Lua 里**不是 table**：LuatOS 的 C 库都通过 `rotable2_newlib()` 注册成
       「userdata + 元表」的**只读表**（Lua 侧 `type(fota) == "userdata"`，`__name == "rotable"`），
       方法从元表 `__index` 取，`fota.init()` 这种点号调用照常可用。因此**判「库在不在」只能看
       `fota.init` 是否可调用**；早期写法 `type(fota) == "table"` 会把「有库」误判成「缺库」，
       现象是 `caps.core=false` + `fota.begin` 回 `ENOLIB`（本机实测踩过一次）。

统一状态（fota.status / mgmt.status.get 的 fota 字段）：
    0 idle / 1 进行中 / 2 成功待重启 / 3 失败
来源 mode：0 无 / 1 URL / 2 本地分段
本地阶段 stage：0 idle / 1 preparing / 2 ready / 3 receiving / 4 verifying /
                5 success / 6 failed

约定与取舍：
  · 两条通道**互斥**：任一条在忙时另一条的启动请求返回 false, "busy"；
  · URL 通道只在联网后执行（等 `net_ready`，限时 + adapter 判据兜底），
    `libfota2` 缺失时优雅降级；本地通道**完全不依赖联网与 libfota2**；
  · 本地通道收齐数据后**不自动重启**：置阶段 success、进度 100、`cfg_store.needs_restart(true)`；
  · 本地通道空闲 `MGMT_FOTA_LOCAL_TIMEOUT_MS`（默认 60s）没有新数据块 → 自动
    `fota.finish(false)` 取消；`local_abort()` 同理；失败/取消后阶段 = failed（可被主站观察），
    下一次 `local_begin()` 会重新开始；
  · `fota.run` 收到的**每一块都必须与主站的 offset 顺序一致**（由 mgmt 按已收字节推得 offset 后调进来），
    同一块重发（SEQ/OFFSET/LEN 全同）按幂等重试处理，不重复写 Flash；
  · 收尾（等 `isDone()` + CRC32 + `finish(true)`）在**后台任务**里做，不阻塞 RPC 应答。

对外接口：
  fota_mgr.start(url)                  启动 URL 升级（true / false + 原因）
  fota_mgr.local_begin(total, crc32, flags)  开始本地升级（total > 0；crc32 = 0 表示不校验）
  fota_mgr.local_write(seq, offset, data)    写一个数据块（顺序/偏移由调用方校验）
  fota_mgr.local_abort(reason)         主动中止本地升级
  fota_mgr.info()                      状态快照 { state, result, progress, url, enabled, lib,
                                                   mode, stage, rx, total, last_seq, cache }
  fota_mgr.is_busy()                   URL 通道是否下载中
  fota_mgr.is_local_busy()             本地通道是否进行中
  fota_mgr.state_name(s) / stage_name(s)  状态码 → 文本（日志用）
  fota_mgr.STATE / MODE / STAGE / NAME 状态码表
  fota_mgr.crc32(data)                 标准 CRC32（IEEE，0xEDB88320，与主站一致）
  fota_mgr.set_clock(fn) / reset()     离线自测用：注入时钟 / 复位状态机
]]

local cfg = require "config"

local M = {}

-- 统一状态（mgmt.status.get 的 fota.state / fota_busy）
M.STATE = { IDLE = 0, ACTIVE = 1, SUCCESS = 2, FAILED = 3 }
local NAME = { [0] = "idle", [1] = "active", [2] = "success", [3] = "failed" }

-- 升级来源（fota.status 的 mode）
M.MODE = { NONE = 0, URL = 1, LOCAL = 2 }

-- 本地升级阶段（fota.status 的 stage）
M.STAGE = { IDLE = 0, PREPARING = 1, READY = 2, RECEIVING = 3, VERIFYING = 4,
            SUCCESS = 5, FAILED = 6 }
local STAGE_NAME = { [0] = "idle", [1] = "preparing", [2] = "ready", [3] = "receiving",
                     [4] = "verifying", [5] = "success", [6] = "failed" }
M.STAGE_NAME = STAGE_NAME

-- 本地数据块上限（与 UART BLOB 帧 DATA 上限一致：DATA ≤ 238 字节）
M.MAX_BLOCK_BYTES = 238

local ENABLED       = (cfg.MGMT_FOTA_ENABLE ~= false)
local LOCAL_ENABLED = (cfg.MGMT_FOTA_LOCAL_ENABLE ~= false)
local NET_WAIT_MS   = tonumber(cfg.MGMT_FOTA_NET_WAIT_MS) or 300000
local LOCAL_TIMEOUT_MS  = tonumber(cfg.MGMT_FOTA_LOCAL_TIMEOUT_MS) or 60000
local FINALIZE_MS       = tonumber(cfg.MGMT_FOTA_LOCAL_FINALIZE_MS) or 3000
local LOCAL_WAIT_MS     = tonumber(cfg.MGMT_FOTA_LOCAL_WAIT_MS) or 3000

local function d_info(...) if cfg.LOG_ENABLE then log.info("fota", ...) end end
local function d_warn(...) if cfg.LOG_ENABLE then log.warn("fota", ...) end end

--=============================================================================
-- 时钟（本地通道空闲超时用；离线自测可注入）
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

local now_ms = default_now_ms

--=============================================================================
-- 核心库 / 扩展库
--=============================================================================
-- 本地升级用的核心库 `fota`：LuatOS 固件里的**全局库对象**（不是 require 到的模块）。
-- ⚠ 它不是 Lua table —— LuatOS 的 C 库都用 `rotable2_newlib()` 注册成「userdata + 元表」的
--    **只读表**（Lua 侧 `type(fota) == "userdata"`，元表 `__name == "rotable"`，见 LuatOS 仓库
--    `lua/src/rotable2.c`），成员靠元表 `__index` 解析，`fota.init()` 照常可调。
--    所以这里**不能**用 `type(c) == "table"` 判可用性（会误报缺库 → `ENOLIB`），只能看
--    `c.init` 是不是可调用的函数。对「没有元表的 userdata / 其它值」索引会抛错，故用 pcall 兜住。
-- 取不到 / 被裁剪时本地通道优雅降级（失败码 -1），其它链路不受影响。
local function index_init(t)
    return t.init
end
local function get_core()
    local c = _G.fota
    if c == nil then
        return nil
    end
    local ok, init = pcall(index_init, c)
    if ok and type(init) == "function" then
        return c
    end
    return nil
end
local function core_available()
    return get_core() ~= nil
end

-- 取官方扩展库 libfota2（⚠ require 必须保持字面量写法，Luatools 依赖检查只认它）
local function load_lib()
    local ok, lib = pcall(function() return require("libfota2") end)
    if ok and type(lib) == "table" then
        return lib
    end
    return nil, ok and "require 到的不是模块表" or tostring(lib)
end

-- URL 通道的扩展库 libfota2 是否可用：只在第一次读「FOTA 能力」时探测一次
-- （给现场一个可读的判据：能力位 bit1 = libfota2 可用，见 README §7.5）
local lib_probe = nil
local function lib_available()
    if lib_probe == nil then
        local lib = load_lib()
        lib_probe = (type(lib) == "table" and type(lib.request) == "function") and true or false
    end
    return lib_probe
end

local function net_ready()
    if not (socket and socket.dft and socket.adapter) then
        return false
    end
    local ok, ad = pcall(socket.adapter, socket.dft())
    return ok and ad ~= nil
end

-- 等联网：先等 "net_ready" 事件，超时后退回 adapter 判据（最多再等 NET_WAIT_MS）
local function wait_net()
    if sys.waitUntil("net_ready", 60000) then
        return true
    end
    local waited = 0
    while waited < NET_WAIT_MS do
        if net_ready() then
            return true
        end
        sys.wait(5000)
        waited = waited + 5000
        d_warn("等待联网中（已等", waited, "ms），FOTA 尚未开始")
    end
    return false
end

--=============================================================================
-- CRC32（IEEE 802.3：多项式 0xEDB88320，初值/结果取反，与主站 zlib.crc32 一致）
--   ⚠ 位宽安全：PC 上的 Lua 是 64 位整数、固件是 32 位整数（LUA_32BITS），
--   0xFFFFFFFF 这类字面量在 32 位 Lua 下会变成浮点数（再做位运算直接抛
--   "number has no integer representation"）。所以掩码/多项式都用「能装进 32 位整数」
--   的写法拼出来，并且每一步都重新掩码（见 README §位宽差异）。
--=============================================================================
local MASK32 = (0xFFFF << 16) | 0xFFFF          -- 0xFFFFFFFF（32 位 Lua 下回绕成 -1，位型相同）
local CRC32_POLY = (0xEDB8 << 16) | 0x8320      -- 0xEDB88320

local function crc32_update(crc, data)
    crc = (tonumber(crc) or 0) & MASK32
    for i = 1, #data do
        crc = (crc ~ string.byte(data, i)) & MASK32
        for _ = 1, 8 do
            if (crc & 1) ~= 0 then
                crc = (crc >> 1) ~ CRC32_POLY
            else
                crc = crc >> 1
            end
            crc = crc & MASK32
        end
    end
    return crc
end

-- 标准 CRC32（初值 0xFFFFFFFF、结果取反）
local function crc32(data)
    return (crc32_update(MASK32, data or "") ~ MASK32) & MASK32
end
M.crc32 = crc32

--=============================================================================
-- URL 通道（原 fota.lua 行为）
--=============================================================================
local state    = M.STATE.IDLE
local result   = 0
local progress = 0
local url_now  = nil

--=============================================================================
-- 本地通道（核心库 fota 流式写入）
--=============================================================================
local lstage   = M.STAGE.IDLE     -- 本地阶段
local lresult  = 0                -- 本地失败码（负数，见下方注释）
local ltotal   = 0                -- 声明总字节数
local lrx      = 0                -- 已接收字节数
local lcrc_expect, lcrc_calc = 0, 0 -- 预期 / 累计 CRC32
local lseq, loff, llen = 0, 0, 0  -- 最近一次成功写入的块序号 / 偏移 / 长度
local llast_ms = 0                -- 最近一次有进展的时刻（空闲超时判据）
local lcache   = 0                -- 核心库当前未写 Flash 的缓存字节数（超 65535 上报时钳位）
local lfinalizing = false         -- 收尾任务是否已排期（防重复）

local function local_active()
    return lstage == M.STAGE.PREPARING or lstage == M.STAGE.READY
        or lstage == M.STAGE.RECEIVING or lstage == M.STAGE.VERIFYING
end

local function local_progress()
    if lstage == M.STAGE.SUCCESS then
        return 100
    end
    if ltotal <= 0 then
        return 0
    end
    local p = math.floor(lrx * 100 / ltotal)
    if p > 99 then p = 99 end         -- 100 留给「收齐并校验成功」
    if p < 0 then p = 0 end
    return p
end

local function reset_local()
    lstage, lresult = M.STAGE.IDLE, 0
    ltotal, lrx, lcrc_expect, lcrc_calc = 0, 0, 0, 0
    lseq, loff, llen = 0, 0, 0
    llast_ms, lcache, lfinalizing = 0, 0, false
end

-- 核心库当前未落盘缓存字节数（核心库没暴露这个接口时按 0 报）
local function query_cache()
    local core = get_core()
    if core and type(core.cache) == "function" then
        local ok, v = pcall(core.cache)
        if ok then
            local n = tonumber(v)
            if n and n >= 0 then
                lcache = math.floor(n)
            end
        end
    end
    return lcache
end

-- 本地升级失败/取消：阶段 = failed（主站可读 0x0031），并让核心库放弃这次写入
local function fail_local(code, why)
    lstage, lresult = M.STAGE.FAILED, code
    lfinalizing = false
    local core = get_core()
    if core and type(core.finish) == "function" then
        pcall(core.finish, false)
    end
    if why then
        d_warn("本地升级失败:", code, tostring(why))
    end
    sys.publish("fota_local_failed", code)
end

-- 收尾（后台任务）：等 isDone() → 校验 CRC32（若声明）→ finish(true) → success + 需重启
local function finalize_local()
    local core = get_core()
    local done = false
    if core and type(core.isDone) == "function" then
        local waited = 0
        while waited < FINALIZE_MS do
            -- 核心库 fota.isDone() 返回 (无异常, 写到最后一块)
            local ok, r1, r2 = pcall(core.isDone)
            if not ok then
                fail_local(-30, "fota.isDone 调用异常: " .. tostring(r1))
                return
            end
            if r1 == false then
                fail_local(-30, "fota.isDone 报错")
                return
            end
            if r2 == true or (r2 == nil and r1 == true) then
                done = true                 -- r2 = 本固件的完成标志；只回一个布尔的老固件按「true = 完成」
                break
            end
            sys.wait(100)              -- 让出协程，等核心库把缓存写进 Flash
            waited = waited + 100
        end
    else
        done = true                    -- 核心库没暴露 isDone：按写完处理
    end
    if not done then
        fail_local(-30, "fota.isDone 超时")
        return
    end
    -- 累计 CRC32 是「初值 0xFFFFFFFF、未取反」的中间值，比较前先做最终取反
    local crc_final = (lcrc_calc ~ MASK32) & MASK32
    if lcrc_expect ~= 0 and crc_final ~= lcrc_expect then
        fail_local(-31, "CRC32 不匹配")
        return
    end
    if core and type(core.finish) == "function" then
        local ok, err = pcall(core.finish, true)
        if not ok or err == false then
            fail_local(-32, "fota.finish 失败: " .. tostring(err))
            return
        end
    end
    lstage, lresult = M.STAGE.SUCCESS, 0
    lfinalizing = false
    d_info("本地升级成功（", lrx, "字节），等待主站下发重启命令")
    -- 升级包已写入、等重启生效：置「需重启」标记（与 URL 通道一致）
    local store_ok, store = pcall(require, "cfg_store")
    if store_ok and type(store) == "table" and store.set_needs_restart then
        store.set_needs_restart(true)
    end
    sys.publish("fota_done", 0)
end

local function start_finalize()
    if lfinalizing then
        return
    end
    lfinalizing = true
    lstage = M.STAGE.VERIFYING
    d_info("本地升级数据收齐：", lrx, "/", ltotal, "字节，开始收尾校验")
    sys.taskInit(function()
        finalize_local()
    end)
end

-- 开始本地升级：true / false + 原因（"busy"/"disabled"/"bad total"/"bad flags"/"lib missing"）
function M.local_begin(total, crc32_expect, flags)
    if not LOCAL_ENABLED then
        return false, "disabled"
    end
    if local_active() or state == M.STATE.ACTIVE then
        return false, "busy"
    end
    total = tonumber(total) or 0
    if total <= 0 then
        return false, "bad total"
    end
    if (tonumber(flags) or 0) ~= 0 then
        return false, "bad flags"
    end
    local core = get_core()
    if not core then
        lstage, lresult = M.STAGE.FAILED, -1
        d_warn("核心库 fota 不可用（固件裁剪掉了本地升级），无法开始本地升级")
        return false, "lib missing"
    end

    lstage, lresult = M.STAGE.PREPARING, 0
    ltotal = total
    lrx, lseq, loff, llen, lcache = 0, 0, 0, 0, 0
    lcrc_expect = (tonumber(crc32_expect) or 0) & MASK32
    lcrc_calc = MASK32                      -- CRC32 初值（收尾时做最终取反）
    lfinalizing = false
    llast_ms = now_ms()
    d_info("本地升级开始：总长", total, "字节，CRC32", lcrc_expect)

    -- 核心库 init/wait 可能阻塞：放后台任务，管理 RPC 侧立即返回「已受理」
    sys.taskInit(function()
        local ok, err = pcall(core.init)
        if not ok or err == false then
            fail_local(-10, "fota.init 失败: " .. tostring(err))
            return
        end
        -- fota.init() 之后按官方 demo（demo/fota 的 fota_file.lua / ble_5101_packet_fota.lua）
        -- 轮询 fota.wait() 等底层就绪：`while not fota.wait() do sys.wait(100) end`。
        -- 这里加**有上限**的轮询（MGMT_FOTA_LOCAL_WAIT_MS，默认 3s，100ms 一轮）：
        --   · wait() 返回 true  → 底层就绪，立刻进 ready；
        --   · 一直 false/报错   → 到时只告警、仍进 ready 继续（个别固件的 wait() 语义若不同，
        --                        不至于把本地升级卡死；真写不进去会在 fota.run 处回 EFLASH）。
        -- 自测里把该值设为 0 → 跳过轮询，保持"init 之后就绪"的确定性。
        if type(core.wait) == "function" and LOCAL_WAIT_MS > 0 and lstage == M.STAGE.PREPARING then
            local waited, ready = 0, false
            while waited < LOCAL_WAIT_MS and lstage == M.STAGE.PREPARING do
                local wok, wir = pcall(core.wait)
                if not wok then
                    d_warn("fota.wait 调用异常，按已就绪继续:", tostring(wir))
                    break
                end
                if wir == true then
                    ready = true
                    break
                end
                sys.wait(100)
                waited = waited + 100
            end
            if not ready and lstage == M.STAGE.PREPARING then
                d_warn("fota.wait 等底层就绪超时（", LOCAL_WAIT_MS, "ms），仍按就绪继续")
            end
        end
        if lstage == M.STAGE.PREPARING then
            lstage = M.STAGE.READY
            llast_ms = now_ms()
            d_info("本地升级就绪（ready），等待数据块")
            sys.publish("fota_local_ready")
        end
    end)

    -- 空闲看门狗：60s 没有新数据块 → finish(false) 清理会话
    sys.taskInit(function()
        while local_active() do
            sys.wait(1000)
            if local_active() and (now_ms() - llast_ms) > LOCAL_TIMEOUT_MS then
                d_warn("本地升级空闲超时（", LOCAL_TIMEOUT_MS, "ms 没有新数据块），取消")
                fail_local(-12, "idle timeout")
                return
            end
        end
    end)

    return true
end

-- 写一个数据块：true / false + 原因（"state"/"seq"/"offset"/"length"/"run"/"lib missing"）
--   *** 帧级校验（KEY/CMD/LEN 与帧内长度、是否越界）在 mgmt 里做 ***；
--   这里负责**会话级**校验：顺序（SEQ 递增）、偏移连续、块长上限、会话状态、幂等重试。
--   两边都做一遍是有意的——即使被别处调用（或将来换传输层）也不会乱序写 Flash。
function M.local_write(seq, offset, data)
    if not LOCAL_ENABLED then
        return false, "disabled"
    end
    seq, offset = tonumber(seq) or 0, tonumber(offset) or 0
    data = type(data) == "string" and data or ""
    local n = #data
    -- 幂等重试：同一块（SEQ/OFFSET/LEN 全同）重发 → 不重复写 Flash，直接算成功
    -- （收尾中/成功后也照样受理：主站可能是没收到上一块的回帧才重发的）
    if lseq > 0 and seq == lseq and offset == loff and n == llen
        and (lstage == M.STAGE.RECEIVING or lstage == M.STAGE.VERIFYING
             or lstage == M.STAGE.SUCCESS) then
        llast_ms = now_ms()
        return true
    end
    if lstage ~= M.STAGE.READY and lstage ~= M.STAGE.RECEIVING then
        return false, "state"
    end
    if n <= 0 or n > M.MAX_BLOCK_BYTES then
        return false, "length"
    end
    if seq ~= lseq + 1 then
        return false, "seq"
    end
    if offset ~= lrx then
        return false, "offset"
    end
    if ltotal > 0 and lrx + n > ltotal then
        return false, "length"
    end
    local core = get_core()
    if not core then
        fail_local(-20, "核心库 fota 不可用")
        return false, "lib missing"
    end
    -- 核心库 fota.run(buf) 返回 (无异常, 收到最后一块, 还未写入 Flash 的缓存字节数)
    --   —— 第三个返回值就是状态区 0x0037 要报的「未落盘缓存」（核心库提示超过 64K 需要等待）
    local ok, r1, r2, r3 = pcall(core.run, data)
    if not ok or r1 == false then
        fail_local(-21, "fota.run 失败: " .. tostring(r1))
        return false, "run"
    end
    if tonumber(r3) then
        lcache = math.max(0, math.floor(tonumber(r3)))
    else
        query_cache()                    -- 老固件不回第三值时退回 core.cache()（没有就保持原值）
    end
    lcrc_calc = crc32_update(lcrc_calc, data)
    lrx, lseq, loff, llen = lrx + n, seq, offset, n
    lstage = M.STAGE.RECEIVING
    llast_ms = now_ms()
    -- 完成信号：核心库说自己收到最后一块了，或按声明的总长已经收齐（双保险）
    if r2 == true or (ltotal > 0 and lrx >= ltotal) then
        start_finalize()
    end
    return true
end

-- 主动中止（方法 fota.abort）：true = 确实中止了一个会话
function M.local_abort(reason)
    if not local_active() then
        return false, "idle"
    end
    fail_local(-40, "abort " .. tostring(reason or ""))
    return true
end

--=============================================================================
-- URL 通道启动（原 start()）
--=============================================================================
function M.start(url)
    if not ENABLED then
        return false, "disabled"
    end
    url = (type(url) == "string") and url:gsub("^%s+", ""):gsub("%s+$", "") or ""
    if url == "" then
        return false, "empty url"
    end
    if state == M.STATE.ACTIVE or local_active() then
        return false, "busy"
    end
    local lib, lerr = load_lib()
    if not (lib and type(lib.request) == "function") then
        state, result = M.STATE.FAILED, -1
        d_warn("libfota2 不可用，FOTA 无法启动:", lerr,
            "（核对 Luatools 下载日志的文件列表里是否有 libfota2.luac、"
            .. "以及烧录时是否勾选「添加默认 lib」）")
        return false, "lib missing"
    end

    state, result, progress, url_now = M.STATE.ACTIVE, 0, 0, url
    d_info("FOTA 启动:", url)
    sys.publish("fota_started", url)

    sys.taskInit(function()
        if not wait_net() then
            state, result = M.STATE.FAILED, -2
            d_warn("等待联网超时，FOTA 未开始")
            sys.publish("fota_done", -2)
            return
        end

        local opts = { url = url }
        -- 进度回调不是所有 libfota2 版本都支持，给了也不影响（库里只认自己用的键）
        opts.progress = function(cur, total)
            cur, total = tonumber(cur) or 0, tonumber(total) or 0
            if total > 0 then
                progress = math.floor(cur * 100 / total)
                if progress > 99 then progress = 99 end   -- 100 留给「下载完成」
                if progress < 0 then progress = 0 end
            end
        end
        if type(lib.setTimeout) == "function" then
            pcall(lib.setTimeout, 300000)
        end

        local ok, err = pcall(lib.request, function(code)
            code = tonumber(code) or 0
            result = code
            if code == 0 then
                state, progress = M.STATE.SUCCESS, 100
                -- 升级包已落盘：置「需重启」标记，等主站下发重启命令进新版本
                local store_ok, store = pcall(require, "cfg_store")
                if store_ok and type(store) == "table" and store.set_needs_restart then
                    store.set_needs_restart(true)
                end
                d_info("FOTA 下载成功，等待重启升级（URL", url, "）")
            else
                state, progress = M.STATE.FAILED, progress
                d_warn("FOTA 下载失败，结果码", code)
            end
            sys.publish("fota_done", code)
        end, opts)

        if not ok then
            -- 库函数同步抛错（参数/网络库异常）：计入失败，不影响其它链路
            state, result = M.STATE.FAILED, -3
            d_warn("libfota2.request 异常:", err)
            sys.publish("fota_done", -3)
        end
    end)

    return true
end

function M.is_busy()
    return state == M.STATE.ACTIVE
end

function M.is_local_busy()
    return local_active()
end

--=============================================================================
-- 状态快照
--=============================================================================
function M.info()
    -- 来源判定：URL 在忙优先报 URL；否则本地会话（含已结束的成功/失败态）优先
    local mode = M.MODE.NONE
    --   优先级：URL 进行中 > 本地会话进行中 > URL 会话已结束（成功/失败）> 本地会话已结束。
    --   把「URL 已结束」放在「本地已结束」之前，是为了让 URL 通道的结果码不被上一次本地
    --   升级残留的 stage/result 盖掉（现场读 fota.status / evt.fota 时按 mode 就能分清来源）。
    if state == M.STATE.ACTIVE then
        mode = M.MODE.URL
    elseif local_active() then
        mode = M.MODE.LOCAL
    elseif state ~= M.STATE.IDLE then
        mode = M.MODE.URL
    elseif lstage ~= M.STAGE.IDLE then
        mode = M.MODE.LOCAL
    end

    local st, rc, pg
    if mode == M.MODE.LOCAL then
        if lstage == M.STAGE.SUCCESS then
            st = M.STATE.SUCCESS
        elseif lstage == M.STAGE.FAILED then
            st = M.STATE.FAILED
        else
            st = M.STATE.ACTIVE
        end
        rc, pg = lresult, local_progress()
    elseif mode == M.MODE.URL then
        st, rc, pg = state, result, progress
    else
        st, rc, pg = M.STATE.IDLE, 0, 0
    end

    return {
        -- 统一状态（fota.state / fota_busy）
        state = st, result = rc, progress = pg,
        -- 通道与本地细节（mode/stage/rx/total/last_seq/cache）
        mode = mode, stage = lstage,
        rx = lrx, total = ltotal, last_seq = lseq,
        cache = query_cache(),
        -- 兼容字段
        url = url_now, enabled = ENABLED and true or false,
        local_enabled = LOCAL_ENABLED and true or false,
        core = core_available(),
        -- 现场判据（type() 的字符串）："userdata" = LuatOS rotable（正常形态）、"table" = 别的实现、
        --                              "nil" = 固件真没这个全局（字段始终存在，便于上位机自动判定）
        core_type = type(_G.fota),
        lib = lib_available(),
    }
end

function M.state_name(s)
    return NAME[s] or "unknown"
end

function M.stage_name(s)
    return STAGE_NAME[s] or "unknown"
end

-- 离线自测用：注入时钟（返回毫秒）
function M.set_clock(fn)
    now_ms = fn or default_now_ms
end

-- 离线自测用：复位状态机（不碰已下载的升级包）
function M.reset()
    state, result, progress, url_now = M.STATE.IDLE, 0, 0, nil
    reset_local()
end

return M
