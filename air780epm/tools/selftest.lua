--[[
@module  selftest
@summary 离线自测（PC 端 Lua 5.4）
@version 2.0
@date    2026.10.14
@usage
在 air780epm 目录下执行：
    lua tools/selftest.lua

测试内容（对应 README §9）：
1. bms_codec（MessageV1）：pkg/bms §11 的五条黄金向量字节级比对
   （53B / 42B / 55B / 25B / 13B 空帧）+ 编码分片让出钩子；
2. sha256 / hmac：SHA-256("abc")、RFC4231 TC2、设备 KAT 与小写 hex；
3. bms_cache：环形覆盖、latest、history 闭区间（同秒多条/不插值）、since 游标与丢失计数；
4. bms_data.normalize：display 与 modbus 快照的字段对照（温度/均衡电流/循环容量/
   满电容量/上电时间/source）与 module_uptime_s；
5. jk_display：整轮 364 字节广播逐字段核对（含 25 串第 25 节帧、蜂鸣器帧、分片接收）；
6. jinjian_slave：缓存驱动 0-8/9-32/103-113/开关量/快充应答、写入与异常码、
   缓存为空时全 0 应答（不崩、不应答超时）；
7. 端到端：协程调度器真跑「JK_ROUND → bms_data → bms_cache → bms_uplink → MQTT」，
   用独立实现的 MessageV1 解码器逐字段核对（含消息级字段、鉴权参数、主题）；
   覆盖「未 NTP 对时不发布」「窗口非空发真实样本」「窗口为空发 13B 空帧（不补零）」；
8. jk_modbus：uart.write 打桩模拟极空从机，核对请求组帧/CRC、解析与单位换算、
   0x12BC RunTime → bmsUpTimeS、异常降级，以及同一份规范样本驱动金箭从机；
9. 串口回调只搬字节 + 独立解析任务（金箭/极空/突发帧预算）；
10. mgmt_uart 帧编解码与重同步：CRC16(0xA001)、组帧（SOF/VER/TYPE/ID/LEN 小端/CRC）、
    扫帧（前导垃圾重同步 / CRC 错 / VER 不符 / 超长分别计数）、分片到达、一包多帧、
    尾部不完整保留；
11. mgmt（统一管理服务）：方法分发（sys.ping/status.get/config.get/config.set/
    cmd.reboot/cmd.factory_reset/fota.url/fota.begin/fota.chunk/fota.abort/fota.status）、
    错误码（EBADJSON/EBADMETHOD/EBADPARAM/EBADKEY/EBADVALUE/EBUSY/ENOLIB/EFOTA/EBADSEQ/
    EFLASH）、config.set 白名单与原子性、ID 去重（重复请求只执行一次）、命令回帧先于动作、
    FOTA 编排（BLOB 与 base64 两条路径）、hello / fota 事件触发；
12. cfg_store：白名单/类型/范围校验、非法项跳过、写透往返（打桩 fskv）、
    覆盖在模块加载前生效、恢复出厂、fskv 缺失降级；
13. fota_mgr（原 fota，改名避免与核心库全局 fota 同名）：URL 通道在 libfota2 打桩下的
    状态机（idle→下载中→成功待重启/失败）、进度/结果码、未联网不启动、忙时拒绝、空
    URL、库缺失；本地通道在核心库 fota 打桩下的状态机（idle→preparing→ready→receiving
    →verifying→success）、init/wait/run 失败、isDone 超时、CRC32 不匹配、主动中止、
    空闲超时、核心库缺失、幂等重试、两条通道互斥、全程不联网也不碰 libfota2；
    CRC32 与「位宽守卫」（源码里不允许出现 32 位整数 Lua 装不下的整数字面量）；
14. mgmt_uart（UART1 裸帧 RPC 传输）：串口参数（115200 8N1、缓冲、dir=GPIO25）、回调只搬字节 +
    任务应答、重同步/CRC/超长计数、突发帧预算、hook 异常只丢该帧、evt.hello/evt.fota 事件；
15. mgmt_mqtt（管理 JSON-RPC，通道 = 主题后缀 rpc）：未启用不订阅、通道主题与后缀可配
    （空值回退 rpc）、订阅下行主题 + 响应发到 <suffix>/resp、事务号（id）原样回带、
    非 JSON / 坏载荷只丢该条、base64 fota.chunk、MQTT 未就绪丢弃并计数、**不发事件**；
16. main.lua 开机整链路 smoke（UART1 裸帧 RPC + 管理 MQTT JSON-RPC 订阅/回发）。
17. mgmt_uart 临时诊断模式（MGMT_ECHO_TEST = true）：回环 + 心跳；
18. 三条链路的串口分配：极空 = UART3（需要引脚复用）、管理 = UART1（默认复用）。
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
-- 事件投递目标：LuatOS 的 publish 会唤醒"当时所有"等待该事件的协程
-- （而不是被第一个消费者独占），这里用 per-task 记录来模拟该语义
local pending = {}
local scheduler = {}
local subscribers = {}       -- 事件名 → 订阅回调列表（见 _G.sys.subscribe）

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
    local v = (data == nil) and true or data
    events[name] = v
    for _, t in ipairs(tasks) do
        if t.event == name then
            pending[t] = v
        end
    end
    -- 同步回调订阅者（真机 LuatOS 语义；见 subscribers 定义）
    for _, fn in ipairs(subscribers[name] or {}) do
        pcall(fn, v)
    end
end

-- 事件订阅：LuatOS 的 sys.publish 会同步回调所有订阅者；这里实现成同样的语义，
-- 让 mgmt（net_ready / fota_* 事件）与 ntp_sync 的订阅在离线自测里也能被触发。
_G.sys.subscribe = function(name, fn)
    subscribers[name] = subscribers[name] or {}
    subscribers[name][#subscribers[name] + 1] = fn
end
_G.sys.unsubscribe = function(name, fn)
    local list = subscribers[name]
    if not list then return end
    for i = #list, 1, -1 do
        if list[i] == fn then table.remove(list, i) end
    end
end
_G.sys.timerStart = function() end
_G.sys.timerLoopStart = function() end

-- board.lua 在**加载期**就 require 工程自带的看门狗库 air153C_wtd，而该库顶部会
-- require "sys"/"sysplus"（固件自带模块，PC 上没有）。这里给两个空壳，让 board.lua
-- 能以真实代码在 PC 上加载；看门狗波形本身由 tools/wdt_selftest.lua 专门验证。
package.preload["sys"] = function() return _G.sys end
package.preload["sysplus"] = function() return {} end

local function pump(max_steps)
    max_steps = max_steps or 500
    local steps = 0
    while steps < max_steps do
        local ran = false
        for _, t in ipairs(tasks) do
            if not t.dead and not t.sleeping
               and (pending[t] ~= nil or not t.event or events[t.event] ~= nil) then
                local arg
                if t.event then
                    if pending[t] ~= nil then
                        arg = pending[t]
                        pending[t] = nil
                    else
                        arg = events[t.event]
                    end
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

-- 丢弃尚未被消费的事件（LuatOS 的 sys.publish 不排队投递给后来才等待的任务，
-- 测试里把前面章节遗留的 JK_ROUND 清掉，保证第 7 节从干净状态开始计数）
function scheduler.clear(name)
    events[name] = nil
    for _, t in ipairs(tasks) do
        if t.event == name then
            pending[t] = nil
        end
    end
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
-- socket 打桩：ntp_sync（main.lua 会 require）联网后调 socket.sntp；没有它会报 nil 索引
_G.socket = { dft = function() return nil end, adapter = function() return nil end,
              sntp = function() end }
_G.air153C_wtd = nil
_G.VERSION = "1.0.0"

-- 虚拟时钟：样本 ts 可复现（与仓库根目录 hwsim.lua 同一做法）。
-- 需要「未 NTP 对时」场景时把 VCLOCK 设回 0（os.time() 不可能是合理的 Unix 时间）。
local VCLOCK_BASE = 1700000000
local VCLOCK = VCLOCK_BASE
os.time = function() return VCLOCK end
local function vclock_set(v) VCLOCK = v end
local function vclock_advance(s) VCLOCK = VCLOCK + s end

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

-- 测试自带的 Modbus CRC16（不 require 被测模块，避免在用例还没准备好环境时就把它缓存住）
local function modbus_crc16(s)
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

local function build_frame(hex)
    local body = from_hex(hex)
    local crc = modbus_crc16(body)
    return body .. string.char(crc & 0xFF, (crc >> 8) & 0xFF)
end

--=============================================================================
-- 1. bms_codec：MessageV1 黄金向量（mqttagent/pkg/bms §11）
--=============================================================================
io.write("== bms_codec MessageV1 黄金向量 ==\n")
local codec = require "bms_codec"

-- 五条向量共用的消息级字段（11.5 空帧只有上电时间与两个计数有效）
local function golden_meta()
    return {
        socFullChargeCap = 54000,
        baseTimestamp = 1700000000,
        moduleUpTime = 3600,
        bmsUpTime = 3590,
        jinjianModbusRequestCount = 12,
        jikongModbusRequestCount = 13,
    }
end

-- 1.1 两样本、三电芯（53B）
do
    local hist = {
        count = 2, cellCount = 3,
        temp1 = { -125, -124 }, temp2 = { 231, 232 }, tempMos = { 450, 451 },
        balanCurrent = { 12, 13 },
        batVol = { 52340, 52341 }, batCurrent = { -1800, -1799 },
        socCycleCap = { 123456, 123457 }, socCapRemain = { 98765, 98766 },
        time = { 0, 1 },
        cellVols = { 3300, 3301, 3302, 3301, 3302, 3303 },
    }
    local want = "aa01010cf0a50380e2cfaa06901c861c0c0df90104ce03048407040c0"
        .. "4f49803048f1c04c0c40704cd8306040004e4190402010401"
    local got = codec.encode(hist, golden_meta())
    check("11.1 两样本×三电芯 = 53B", #got, 53)
    check("11.1 字节级一致", to_hex(got), want)
end

-- 1.2 三样本、两电芯、全静态（42B，modeTable 全常量）
do
    local hist = {
        count = 3, cellCount = 2,
        temp1 = { 250, 250, 250 }, temp2 = { 248, 248, 248 }, tempMos = { 300, 300, 300 },
        balanCurrent = { 10, 10, 10 },
        batVol = { 52000, 52000, 52000 }, batCurrent = { 0, 0, 0 },
        socCycleCap = { 100000, 100000, 100000 },
        socCapRemain = { 90000, 90000, 90000 },
        time = { 0, 0, 0 },
        cellVols = { 3300, 3301, 3300, 3301, 3300, 3301 },
    }
    local want = "aa010209aaaa2af0a50380e2cfaa06901c861c0c0df403f003d804"
        .. "0aa0960300a08d0690bf0500e41902"
    local got = codec.encode(hist, golden_meta())
    check("11.2 三样本×两电芯静态 = 42B", #got, 42)
    check("11.2 字节级一致", to_hex(got), want)
end

-- 1.3 四样本、四电芯、压差收窄（55B，spatial 布局 + Time 线性）
do
    local hist = {
        count = 4, cellCount = 4,
        temp1 = { 250, 250, 250, 250 }, temp2 = { 248, 248, 248, 248 },
        tempMos = { 300, 300, 300, 300 },
        balanCurrent = { 10, 10, 10, 10 },
        batVol = { 52000, 52000, 52000, 52000 }, batCurrent = { 0, 0, 0, 0 },
        socCycleCap = { 100000, 100000, 100000, 100000 },
        socCapRemain = { 90000, 90000, 90000, 90000 },
        time = { 0, 1, 2, 3 },
        cellVols = {
            3300, 3360, 3420, 3480,
            3300, 3340, 3380, 3420,
            3300, 3320, 3340, 3360,
            3300, 3310, 3320, 3330,
        },
    }
    local want = "aa010313aaaa0b00f0a50380e2cfaa06901c861c0c0df403f003d8040aa0960300a08d06"
        .. "90bf050004e419782929157829291578292915"
    local got = codec.encode(hist, golden_meta())
    check("11.3 四样本×四电芯 = 55B", #got, 55)
    check("11.3 字节级一致", to_hex(got), want)
end

-- 1.4 单样本、无电芯（25B）
do
    local hist = {
        count = 1, cellCount = 0,
        temp1 = { 1 }, temp2 = { 2 }, tempMos = { 3 },
        balanCurrent = { 4 }, batVol = { 5 }, batCurrent = { 6 },
        socCycleCap = { 7 }, socCapRemain = { 8 }, time = { 9 },
        cellVols = {},
    }
    local meta = {
        socFullChargeCap = 100000, baseTimestamp = 1700000000,
        moduleUpTime = 1, bmsUpTime = 2,
        jinjianModbusRequestCount = 3, jikongModbusRequestCount = 4,
    }
    local want = "aa010000a08d0680e2cfaa060102030402040604050c070809"
    local got = codec.encode(hist, meta)
    check("11.4 单样本无电芯 = 25B", #got, 25)
    check("11.4 字节级一致", to_hex(got), want)
end

-- 1.5 0 样本（13B 空帧）
do
    local got = codec.encode_empty(golden_meta())
    check("11.5 0 样本 = 13B", #got, 13)
    check("11.5 字节级一致", to_hex(got), "aa010080080000901c861c0c0d")
    -- 空帧与「样本数为 0」的 encode 语义区分：encode 不负责 0 样本
    check("encode(count=0) 返回 nil", codec.encode({ count = 0, cellCount = 0 }, golden_meta()), nil)
end

-- 1.6 编码分片让出钩子：每 16 列回调一次，且输出与同步编码字节一致
do
    local hist = {
        count = 30, cellCount = 32,
        temp1 = {}, temp2 = {}, tempMos = {}, balanCurrent = {},
        batVol = {}, batCurrent = {}, socCycleCap = {}, socCapRemain = {},
        time = {}, cellVols = {},
    }
    for i = 1, 30 do
        hist.temp1[i] = 320 + (i % 3)
        hist.temp2[i] = 325 + (i % 3)
        hist.tempMos[i] = 330
        hist.balanCurrent[i] = 10 + i
        hist.batVol[i] = 66070 + i
        hist.batCurrent[i] = 3400 - i
        hist.socCycleCap[i] = 54000
        hist.socCapRemain[i] = 25000 + i
        hist.time[i] = (i - 1) * 2
    end
    for s = 0, 29 do
        for j = 1, 32 do
            hist.cellVols[s * 32 + j] = 3300 + ((s + j) % 4)
        end
    end

    local slices = 0
    local with_hook = codec.encode(hist, golden_meta(), function() slices = slices + 1 end)
    check("编码分片钩子被调用", slices > 0, true)
    check("分片编码输出与同步编码一致", with_hook, codec.encode(hist, golden_meta()))
end

--=============================================================================
-- 2. SHA-256 / HMAC-SHA256（MQTT 鉴权）
--=============================================================================
io.write("== sha256 / hmac ==\n")
local sha256 = require "sha256"
local hmac = require "hmac"

check("sha256(\"abc\") = FIPS 已知答案",
    sha256.hex("abc"), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad")
check("sha256 空串 = FIPS 已知答案",
    sha256.hex(""), "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855")
check("sha256 两个分组 = FIPS 已知答案",
    sha256.hex("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"),
    "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1")

-- 填充边界（FIPS 180-4 §5.1.1）：55~57 跨 0x80/k 边界、63~65 跨 64B 分组边界、
-- 119/120/128 多分组。这些长度会走到 pad() 里 64 位比特长度的高/低半字写入
-- （原实现用 0x100000000 求高 32 位，在 32 位整数固件里该字面量回绕成 0 →
-- sha256.lua:50: attempt to divide by zero）。参考值取自 Python hashlib。
for _, case in ipairs({
    { 55,  "9f4390f8d30c2dd92ec9f095b65e2b9ae9b0a925a5258e241c9f1e910f734318" },
    { 56,  "b35439a4ac6f0948b6d6f9e3c6af0f5f590ce20f1bde7090ef7970686ec6738a" },
    { 57,  "f13b2d724659eb3bf47f2dd6af1accc87b81f09f59f2b75e5c0bed6589dfe8c6" },
    { 63,  "7d3e74a05d7db15bce4ad9ec0658ea98e3f06eeecf16b4c6fff2da457ddc2f34" },
    { 64,  "ffe054fe7ae0cb6dc65c3af9b61d5209f439851db43d0ba5997337df154668eb" },
    { 65,  "635361c48bb9eab14198e76ea8ab7f1a41685d6ad62aa9146d301d4f17eb0ae0" },
    { 119, "31eba51c313a5c08226adf18d4a359cfdfd8d2e816b13f4af952f7ea6584dcfb" },
    { 120, "2f3d335432c70b580af0e8e1b3674a7c020d683aa5f73aaaedfdc55af904c21c" },
    { 128, "6836cf13bac400e9105071cd6af47084dfacad4e5e302c94bfed24e013afb73e" },
}) do
    check(string.format("sha256(\"a\"×%d) 填充边界", case[1]),
        sha256.hex(string.rep("a", case[1])), case[2])
end
check("hmac_sha256 RFC4231 TC2",
    hmac.sha256_hex("Jefe", "what do ya want for nothing?"),
    "5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843")
check("hmac_sha256 RFC4231 TC6（key 131 字节，先哈希）",
    hmac.sha256_hex(string.rep("\170", 131), "Test Using Larger Than Block-Size Key - Hash Key First"),
    "60e431591ee0b67f0d8a26aacbf5b77f8e0bc6213728c5140546040f0ee37f54")
check("hmac_sha256 设备 KAT（key=soc, msg=IMEI）",
    hmac.sha256_hex("soc", "861234567890123"),
    "1b281b472b217c6e99ff6aba726dc5fae8331dbadfaead19ba7fa8c0c29e409a")
check("hmac 输出小写 hex（无大写字母）",
    hmac.sha256_hex("soc", "861234567890123"):match("%u") == nil, true)
check("hmac 长 key 输出仍是 64 字符 hex",
    #hmac.sha256_hex(string.rep("a", 131), "test"), 64)

--=============================================================================
-- 3. bms_cache：环形缓存（隔离实例，不影响业务共享实例）
--=============================================================================
io.write("== bms_cache 环形缓存 ==\n")

-- 用 dofile 拿一份独立实例（配置里的 BMS_CACHE_SAMPLES，默认 32）
local cache_u = dofile(here .. "/../bms_cache.lua")

-- 3.1 push 40 条：环绕、latest、count、统计
do
    check("缓存容量 = 32", cache_u.CAPACITY, 32)
    check("空缓存 latest = nil", cache_u.latest(), nil)
    check("空缓存 count = 0", cache_u.count(), 0)
    for i = 1, 40 do
        cache_u.push({ ts = 1000 + i, cellCount = 1, cells = { 3300 + i }, i = i })
    end
    check("push 40 条后 count = 32", cache_u.count(), 32)
    check("latest = 第 40 条", cache_u.latest() and cache_u.latest().i, 40)
    check("latest 带缓存分配的 seq", cache_u.latest() and cache_u.latest().seq, 40)
    check("最旧一条已被覆盖（history 起点 = 第 9 条）",
        cache_u.history(0, 100000)[1].i, 9)
    local st = cache_u.stats()
    check("stats.pushed = 40", st.pushed, 40)
    check("stats.overwritten = 8", st.overwritten, 8)
    check("stats.count = 32", st.count, 32)
end

-- 3.2 history 闭区间 / 同秒多条 / 不插值
do
    local h = dofile(here .. "/../bms_cache.lua")
    h.push({ ts = 100, cellCount = 0, cells = {}, tag = "a" })
    h.push({ ts = 100, cellCount = 0, cells = {}, tag = "b" }) -- 同秒多条
    h.push({ ts = 102, cellCount = 0, cells = {}, tag = "c" })
    h.push({ ts = 104, cellCount = 0, cells = {}, tag = "d" })
    local win = h.history(100, 102)
    check("闭区间开头含 from", win[1].tag, "a")
    check("闭区间含同秒多条", win[2].tag, "b")
    check("闭区间结尾含 to", win[3].tag, "c")
    check("闭区间之外不返回（无插值/无补零）", #win, 3)
    check("时间戳原样保存（不回退不修正）", h.latest().ts, 104)
end

-- 3.3 since 游标：窗口不重叠 + 被覆盖的样本计入丢失
do
    local h = dofile(here .. "/../bms_cache.lua")
    for i = 1, 5 do
        h.push({ ts = 200 + i, cellCount = 0, cells = {}, i = i })
    end
    local w1, latest1, lost1 = h.since(0)
    check("since(0) 返回全部 5 条", #w1, 5)
    check("since(0) 最新 seq = 5", latest1, 5)
    check("since(0) 无丢失", lost1, 0)
    for i = 6, 10 do
        h.push({ ts = 200 + i, cellCount = 0, cells = {}, i = i })
    end
    local w2, latest2 = h.since(5)
    check("since(5) 只返回新样本", #w2, 5)
    check("since(5) 第一条是 6", w2[1].i, 6)
    check("since(5) 最新 seq = 10", latest2, 10)
    local w3, latest3 = h.since(10)
    check("since(最新) 返回空表", #w3, 0)
    check("since(最新) 游标不倒退", latest3, 10)
    -- 游标落后于缓存起点（缓存被覆盖）→ 丢失条数 = 起点 - 1 - 游标
    local h2 = dofile(here .. "/../bms_cache.lua")
    for i = 1, 40 do
        h2.push({ ts = 300 + i, cellCount = 0, cells = {}, i = i })
    end
    local w4, latest4, lost4 = h2.since(0)
    check("被覆盖前未上报的样本计入 lost", lost4, 8)
    check("lost 之后只剩缓存内的 32 条", #w4, 32)
    check("最新 seq = 40", latest4, 40)
end

--=============================================================================
-- 4. bms_data：两套协议快照 → 规范样本
--=============================================================================
io.write("== bms_data 采集适配层 ==\n")
local bms_data = require "bms_data"

-- 4.1 display 快照（jk_display.get_state 的字段）归一
do
    local s = {
        ts = 1700000100, source = nil, cellCount = 3,
        cells = { 3301, 3302, 3303, 0 }, batVolMv = 9906, batCurrentMa = 3400,
        soc = 51, tempBatC = 32, tempMosC = 33,
        balanceSw = 1, chargeMos = 1, dischargeMos = 1,
        alarms = { 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        sysAlarm = 0, sysAlarmValid = true,
        capRemainMah = 27500, fullCapMah = 53922,
    }
    local n = bms_data.normalize(s)
    check("display: source = display", n.source, "display")
    check("display: tempBat1C", n.tempBat1C, 32)
    check("display: tempBat2C = tempBat1C", n.tempBat2C, 32)
    check("display: balanCurrentMa = 0", n.balanCurrentMa, 0)
    check("display: socCycleCapMah = 0", n.socCycleCapMah, 0)
    check("display: cycleCount = 0", n.cycleCount, 0)
    check("display: bmsUpTimeS = 0", n.bmsUpTimeS, 0)
    check("display: 满电容量沿用估算值", n.socFullChargeCapMah, 53922)
    check("display: 剩余容量", n.socCapRemainMah, 27500)
    check("display: 单体槽位补齐到 32", #n.cells, 32)
    check("display: 第 3 节电压", n.cells[3], 3303)
    check("display: 第 4 节空槽位 = 0", n.cells[4], 0)
    check("display: sysAlarmValid 透传", n.sysAlarmValid, true)
    check("display: 9 个图标位透传", n.alarms ~= nil, true)
end

-- 4.2 modbus 快照（jk_modbus.get_state 的字段）归一
do
    local s = {
        ts = 1700000200, source = "modbus", cellCount = 2,
        cells = { 3301, 3302 }, batVolMv = 6603, batCurrentMa = -1500, soc = 88,
        tempBatC = 25, tempBat2C = 27, tempMosC = 40,
        balanceSw = 0, chargeMos = 1, dischargeMos = 0,
        alarms = { 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        sysAlarm = 0x0100, sysAlarmValid = true,
        capRemainMah = 44000, fullCapMah = 50000,
        balanCurrentMa = 120, cycleCount = 77, cycleCapMah = 123456,
        soh = 98, bmsUpTimeS = 3600,
    }
    local n = bms_data.normalize(s)
    check("modbus: source = modbus", n.source, "modbus")
    check("modbus: tempBat1C", n.tempBat1C, 25)
    check("modbus: tempBat2C 两路独立", n.tempBat2C, 27)
    check("modbus: 均衡电流直通", n.balanCurrentMa, 120)
    check("modbus: 循环总容量直通", n.socCycleCapMah, 123456)
    check("modbus: 满电容量直通", n.socFullChargeCapMah, 50000)
    check("modbus: 循环次数直通", n.cycleCount, 77)
    check("modbus: SOH 直通", n.soh, 98)
    check("modbus: BMS 上电时间直通", n.bmsUpTimeS, 3600)
    check("modbus: 放电电流为负", n.batCurrentMa, -1500)
    check("modbus: 创建新表（不共享快照引用）", n.cells ~= s.cells, true)
end

-- 4.3 上电秒数 + 非法快照
do
    check("module_uptime_s 为整数秒", type(bms_data.module_uptime_s()), "number")
    check("normalize(nil) = nil", bms_data.normalize(nil), nil)
    check("normalize(\"x\") = nil", bms_data.normalize("x"), nil)
end

--=============================================================================
-- 5. 极空显示屏广播整轮解析（协议手册 §7）
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

-- 5.1 0x80 = 写寄存器：寄存器 0x02 的蜂鸣器帧不能当作轮起点
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

-- 5.2 ≥25 串：主块只有 24 槽，第 25 节由 0x2011 帧补发（每轮多 9 字节）
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
-- 6. 金箭 Modbus 从机（数据来源 = bms_cache 里的规范样本）
--=============================================================================
io.write("== jinjian_slave 应答（缓存驱动） ==\n")

local bms_cache = require "bms_cache"

-- 6.0 缓存为空：全部按 0 应答（不崩、不应答超时、PN/版本照常）
--     用一份独立空缓存实例加载从机，避免污染业务共享缓存
do
    local saved_cache_mod = package.loaded["bms_cache"]
    package.loaded["bms_cache"] = dofile(here .. "/../bms_cache.lua")
    local jj_empty = dofile(here .. "/../jinjian_slave.lua")
    local cfg = require "config"
    local resp = jj_empty.handle_frame(from_hex("01 03 00 00 00 09 85 CC"))
    check("空缓存：周期轮询有应答（不超时）", resp ~= nil, true)
    local function reg(a) return (string.byte(resp, 4 + a * 2) << 8) | string.byte(resp, 5 + a * 2) end
    check("空缓存：reg0 总电压 = 0", reg(0), 0)
    check("空缓存：reg1 串数 = 0", reg(1), 0)
    check("空缓存：reg2 SOC = 0", reg(2), 0)
    check("空缓存：reg3 容量 = 0", reg(3), 0)
    check("空缓存：reg8 板温 = 0", reg(8), 0)
    check("空缓存：reg5 电流 = 0", reg(5), 0)
    local r103 = jj_empty.handle_frame(build_frame("01 03 00 67 00 01"))
    check("空缓存：reg103 = 配置值", (string.byte(r103, 4) << 8) | string.byte(r103, 5), cfg.JJ_BATTERY_TYPE)
    local r104 = jj_empty.handle_frame(build_frame("01 03 00 68 00 01"))
    check("空缓存：reg104 循环次数 = 0", (string.byte(r104, 4) << 8) | string.byte(r104, 5), 0)
    local rc = jj_empty.handle_frame(build_frame("01 01 00 04 00 05"))
    check("空缓存：保护开关量全 0", string.byte(rc, 4), 0)
    local rfast = jj_empty.handle_frame(build_frame("01 01 00 3F 00 01"))
    check("空缓存：快充状态 = 0", string.byte(rfast, 4), 0)
    package.loaded["bms_cache"] = saved_cache_mod
end

-- 6.1 喂一轮广播 → bms_data 归一入缓存 → 从机按同一份样本应答
scheduler.clear("JK_ROUND")
bms_cache.reset()
jk.feed(round)
pump()
local cached = bms_cache.latest()
check("缓存收到一轮样本", cached ~= nil and cached.cellCount, 20)
check("缓存样本 source = display", cached and cached.source, "display")
check("缓存样本满电容量 = 剩余容量÷SOC 估计",
    cached and cached.socFullChargeCapMah, math.floor(27500 * 100 / 51 + 0.5))

-- 用 require（而不是 dofile）：bms_uplink 的消息级 JinjianModbusRequestCount 也是
-- 懒 require 同一个实例，两边读到的计数必须一致
local jj = require "jinjian_slave"

-- 6.2 CRC 与文档样例比对（文档 CRC 按 低字节 在前 排列）
check("CRC 01 03 0000 0009 = CC85", jj.crc16(from_hex("01 03 00 00 00 09")), 0xCC85)
check("CRC 01 01 0004 0005 = C8BD", jj.crc16(from_hex("01 01 00 04 00 05")), 0xC8BD)

-- 6.3 周期轮询 0-8
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

-- 6.4 详情 0-32（含单体电压）
do
    local resp = jj.handle_frame(from_hex("01 03 00 00 00 21 85 D2"))
    check("详情应答长度 = 3+66+2", #resp, 71)
    local function reg(a) return (string.byte(resp, 4 + a * 2) << 8) | string.byte(resp, 5 + a * 2) end
    check("单体1 mV", reg(9), 3304)
    check("单体7 mV", reg(15), 3302)
    check("单体20 mV", reg(28), 3304)
    check("单体21 mV", reg(29), 0)
end

-- 6.5 电池状态 103-113
do
    local resp = jj.handle_frame(from_hex("01 03 00 67 00 0B B5 D2"))
    check("状态应答长度 = 3+22+2", #resp, 27)
    local function reg(a) return (string.byte(resp, 4 + (a - 103) * 2) << 8) | string.byte(resp, 5 + (a - 103) * 2) end
    check("reg103 电池类型", reg(103), 0)
    check("reg104 循环次数(display 无数据 → 本地缓存 0)", reg(104), 0)
    check("reg110 均衡状态", reg(110), 1)
    check("reg113 标称容量", reg(113), 54)
end

-- 6.6 PN（1000-1007）为 16 ASCII，形如 SOC-XXXXXXXXXXXX
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

-- 6.7 版本 1016-1017（VERSION = 1.0.0 → 0x0100 / 0x0000）
do
    local resp = jj.handle_frame(from_hex("01 03 03 F8 00 02 45 BE"))
    local v1 = (string.byte(resp, 4) << 8) | string.byte(resp, 5)
    local v2 = (string.byte(resp, 6) << 8) | string.byte(resp, 7)
    check("版本寄存器1", v1, 0x0100)
    check("版本寄存器2", v2, 0x0000)
end

-- 6.8 保护开关量 4-8
do
    local resp = jj.handle_frame(from_hex("01 01 00 04 00 05 BD C8"))
    check("开关量应答长度 = 3+1+2", #resp, 6)
    check("开关量数据 = 0", string.byte(resp, 4), 0)
end

-- 6.9 快充状态（充电中 → 1）
do
    local resp = jj.handle_frame(from_hex("01 01 00 3F 00 01 CD C6"))
    check("快充状态 = 1", string.byte(resp, 4), 1)
end

-- 6.10 写充电时间 / 目标 SOC
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

-- 6.11 结束快充 → 快充状态转为 0
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

-- 6.12 非法地址 → 异常码 02；其它从站地址 → 不应答
do
    local resp = jj.handle_frame(build_frame("01 03 00 90 00 01"))
    check("非法地址异常码", resp and string.byte(resp, 3), 0x02)
    local resp2 = jj.handle_frame(from_hex("02 03 00 00 00 09 85 CC"))
    check("其它从站不应答", resp2, nil)
end

-- 6.13 计数统计：中控 BMS 查询次数 / 应答次数
do
    local st = jj.get_stats()
    check("查询计数已累加", st.queries > 0, true)
    check("查询次数 = 应答次数", st.queries, st.responses)
    check("忽略非本机地址帧计数", st.ignored, 1)
    check("收帧数 = 查询 + 忽略", st.rx_frames, st.queries + st.ignored)
    check("最近查询时间已记录", st.last_query_ms ~= nil, true)
end

--=============================================================================
-- 7. 端到端：JK_ROUND → bms_data → bms_cache → bms_uplink → MQTT
--=============================================================================
io.write("== 端到端上报链路（MessageV1） ==\n")

-- 上报主题前缀取自 config（部署里可能是 /bms，也可能是 /device 等），测试不要写死
local TOPIC_PREFIX = (require "config").MQTT_TOPIC_PREFIX or ""

-- 7.0 未配置 MQTT 地址 → 只关闭 4G 链路（采集/缓存/从机照常）
do
    local cfg0 = require "config"
    local saved_host = cfg0.MQTT_HOST
    cfg0.MQTT_HOST = ""
    local off = dofile(here .. "/../bms_uplink.lua")
    pump()
    check("未配置 MQTT 地址: 上报功能关闭", off.is_enabled(), false)
    check("未配置 MQTT 地址: 不产生任何报文", off.get_stats().batches, 0)
    check("未配置 MQTT 地址: 无上报主题", off.topic(), nil)
    check("未配置 MQTT 地址: MQTT 未就绪", off.is_mqtt_ready(), false)
    check("未配置 MQTT 地址: flush 拒绝", (select(2, off.flush())), "mqtt disabled")
    cfg0.MQTT_HOST = saved_host
end

-- 7.1 MQTT 客户端打桩：记录 auth 参数与 publish 内容
--     离线调度器不模拟毫秒睡眠（sys.wait 让任务一直"睡"到显式唤醒），
--     故自测里关掉编码分片让出；让出钩子本身由 1.6 单独覆盖。
do
    local cfg_y = require "config"
    cfg_y.ENCODE_YIELD_MS = 0
end

local published, auth_args, subscribed = {}, {}, {}
_G.mqtt = {
    create = function(_, host, port)
        local client = { _cb = nil, _ready = true, host = host, port = port }
        function client:auth(a, b, c)
            auth_args = { a, b, c }
            return true
        end
        function client:autoreconn() return true end
        function client:keepalive() return true end
        function client:on(cb) self._cb = cb end
        function client:connect()
            if self._cb then self._cb(self, "conack", nil, nil) end
            return true
        end
        function client:ready() return self._ready end
        function client:subscribe(topic, qos)
            subscribed[#subscribed + 1] = { topic = topic, qos = qos }
            return true
        end
        function client:publish(topic, payload, qos)
            published[#published + 1] = { topic = topic, payload = payload, qos = qos }
            return true
        end
        return client
    end,
}

-- 7.2 未 NTP 对时前不发布：把虚拟时钟拨回 0（os.time() 不可信）
vclock_set(0)
scheduler.clear("JK_ROUND")
bms_cache.reset()
local uplink = require "bms_uplink"
pump()
scheduler.wake()
check("未 NTP 对时前不发布", #published, 0)
check("未 NTP 对时前不产生报文", uplink.get_stats().batches, 0)

-- 7.3 联网 + 对时 → 连接 MQTT（clientId/用户名 = IMEI，密码 = HMAC-SHA256）
vclock_set(VCLOCK_BASE)
scheduler.publish("net_ready", "861234567890123")
check("MQTT 已连接", uplink.is_mqtt_ready(), true)
check("上报主题", uplink.topic(), TOPIC_PREFIX .. "/861234567890123/status")
check("MQTT 用户名/客户端 ID = IMEI", tostring(auth_args[1]) .. "|" .. tostring(auth_args[2]),
    "861234567890123|861234567890123")
check("MQTT 密码 = HMAC-SHA256(soc, IMEI) 小写 hex", auth_args[3],
    "1b281b472b217c6e99ff6aba726dc5fae8331dbadfaead19ba7fa8c0c29e409a")
check("MQTT 客户端连接参数 host", uplink.get_stats() ~= nil, true)

scheduler.publish("ntp_synced", VCLOCK_BASE)

-- 7.4 窗口内没有任何样本 → 13 字节 0 样本帧（不补零、不 keep-last）
scheduler.wake()
check("空窗上报 1 包", #published, 1)
check("空窗报文 = 0 样本帧头（aa 01 00 80 08）",
    published[1] and to_hex(published[1].payload:sub(1, 5)), "aa01008008")
-- 0 样本帧 = 5 字节帧头 + 6 个消息级 uvarint；标量全 0 时最小 11 字节，
-- 与 pkg/bms §11.5 的 13B 向量（上电时间/计数非 0）同为一种报文
check("空窗报文长度在 11..20 字节（标量 uvarint 决定）",
    published[1] and (#published[1].payload >= 11 and #published[1].payload <= 20), true)
check("空窗报文首字节 = 0xAA", published[1] and string.byte(published[1].payload, 1), 0xAA)

-- 7.5 喂 14 轮广播（SOC 每轮 +1、时间每轮 +2s）→ 上报一批 14 条真实样本
local function set_round_soc(hex_round, soc)
    -- 主数据块 SOC 位于数据块偏移 6，数据块起点为帧内第 12 字节 → 第 19 字节
    return hex_round:sub(1, 18) .. string.char(0, soc) .. hex_round:sub(21)
end

for i = 1, 14 do
    vclock_advance(2)
    jk.feed(set_round_soc(round, 40 + i))
    pump()
end
check("缓存收满 14 条", bms_cache.count(), 14)

scheduler.wake()
check("已发布 2 包（空帧 + 真实）", #published, 2)
local msg = published[2]
check("发布主题", msg and msg.topic, TOPIC_PREFIX .. "/861234567890123/status")
check("发布 QoS", msg and msg.qos, 0)
check("真实样本计数", uplink.get_stats().samples, 14)
check("空帧计数", uplink.get_stats().empty_frames, 1)
check("无丢失样本", uplink.get_stats().lost, 0)

--=============================================================================
-- 7.6 独立实现的 MessageV1 解码器（对照 mqttagent/pkg/bms 格式规范，
--     不与 bms_codec 共用任何代码；仓库根目录 hwsim.lua 也从这里抽取该段）
--=============================================================================
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
    if string.byte(data, pos) ~= 0xAA then
        error("bad header byte")
    end
    pos = pos + 1
    local version
    version, pos = get_uvarint(data, pos)
    if version ~= 1 then
        error("unsupported version " .. tostring(version))
    end
    local n1
    n1, pos = get_uvarint(data, pos)
    local header
    header, pos = get_uvarint(data, pos)
    local noSamples = (header & 0x400) ~= 0
    local count = noSamples and 0 or (n1 + 1)
    local cellCount = noSamples and 0 or (header >> 2)
    local hasModes = (not noSamples) and (header & 1) == 1
    local spatial = (header & 2) == 2
    local columnCount = noSamples and 0 or (9 + cellCount)

    local modes = {}
    if hasModes then
        for i = 1, columnCount do
            local bit = (i - 1) * 2
            local byte = string.byte(data, pos + (bit // 8))
            modes[i] = (byte >> (bit % 8)) & 3
        end
        pos = pos + (columnCount * 2 + 7) // 8
    end

    -- 消息级字段顺序：SOCFullChargeCap, BaseTimestamp, ModuleUPTime, BMSUPTime,
    --                 JinjianModbusRequestCount, JikongModbusRequestCount
    local scalars = {}
    for i = 1, 6 do
        scalars[i], pos = get_uvarint(data, pos)
    end

    local cols = {}
    if not noSamples then
        -- 列 1..3 有符号、4/5 无符号、6 有符号、7/8/9 无符号、电芯基准无符号、压差有符号
        local signed_cols = { true, true, true, false, false, true, false, false, false, false }
        for i = 1, columnCount do
            local signed = signed_cols[i] or (i > 10)
            local mode = hasModes and (modes[i] or 0) or 0
            cols[i], pos = decode_column(data, pos, count, signed, mode)
        end
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
    local function col(i) return cols[i] or {} end
    return {
        count = count, cellCount = cellCount, spatial = spatial, noSamples = noSamples,
        socFullChargeCap = scalars[1], baseTimestamp = scalars[2],
        moduleUpTime = scalars[3], bmsUpTime = scalars[4],
        jinjian = scalars[5], jikong = scalars[6],
        temp1 = col(1), temp2 = col(2), tempMos = col(3), balanCurrent = col(4),
        batVol = col(5), batCurrent = col(6), socCycleCap = col(7),
        socCapRemain = col(8), time = col(9), cells = cells,
    }
end

-- 7.7 空帧（7.4 那一包）字段核对
do
    local ok_empty, emp = pcall(decode_payload, published[1].payload)
    check("空帧可被独立解码", ok_empty, true)
    if ok_empty then
        check("空帧：0 样本标记", emp.noSamples, true)
        check("空帧：样本数 0", emp.count, 0)
        check("空帧：无电芯列", emp.cellCount, 0)
        check("空帧：SOCFullChargeCap 写 0", emp.socFullChargeCap, 0)
        check("空帧：BaseTimestamp 写 0", emp.baseTimestamp, 0)
        check("空帧：Jinjian 请求计数有效", emp.jinjian, jj.get_stats().queries)
        check("空帧：Jikong 请求计数（display=0）", emp.jikong, 0)
    else
        io.write("[error] ", tostring(emp), "\n")
    end
end

-- 7.8 真实样本批次逐字段核对
local ok_dec, dec = pcall(decode_payload, msg.payload)
check("payload 可被独立解码器解开", ok_dec, true)
if ok_dec then
    check("解码样本数", dec.count, 14)
    check("解码串数", dec.cellCount, 20)
    check("非空帧不带 0 样本标记", dec.noSamples, false)
    check("消息级 满电容量 = 最新样本(剩余容量÷SOC)",
        dec.socFullChargeCap, math.floor(27500 * 100 / 54 + 0.5))
    check("消息级 BaseTimestamp = 最新样本 ts", dec.baseTimestamp, VCLOCK)
    check("消息级 ModuleUPTime 有效", dec.moduleUpTime >= 0, true)
    check("消息级 BMSUPTime（display 无来源 = 0）", dec.bmsUpTime, 0)
    check("消息级 金箭请求计数 = jj.queries", dec.jinjian, jj.get_stats().queries)
    check("消息级 极空 modbus 请求计数（display=0）", dec.jikong, 0)
    check("Time 列 = Base - ts（最新一条 = 0）", dec.time[14], 0)
    check("Time 列（最旧一条 = 26s）", dec.time[1], 26)
    check("温度列（0.1℃, display 两路同值）", dec.temp1[1] .. "/" .. dec.temp2[1], "320/320")
    check("MOS 温度列（0.1℃）", dec.tempMos[14], 330)
    check("均衡电流列（display 无数据 = 0）", dec.balanCurrent[7], 0)
    check("总压列 mV", dec.batVol[7], 66070)
    check("电流列 mA", dec.batCurrent[14], 3400)
    check("循环总容量列（display 无数据 = 0）", dec.socCycleCap[1], 0)
    check("剩余容量列 mAh", dec.socCapRemain[1], 27500)
    check("单体1 mV", dec.cells[1][1], 3304)
    check("单体7 mV", dec.cells[14][7], 3302)
    check("单体20 mV", dec.cells[14][20], 3304)
    io.write(string.format("[info] 14×20 报文 %d 字节, 电芯布局 %s\n",
        #msg.payload, dec.spatial and "spatial" or "offset"))
else
    io.write("[error] ", tostring(dec), "\n")
end

-- 7.9 窗口内没有新样本 → 只能发空帧（不补零/不插值/不沿用旧值）
vclock_advance(2)
scheduler.wake()
check("窗口无新样本 → 第 3 包", #published, 3)
check("第 3 包 = 0 样本空帧",
    published[3] and to_hex(published[3].payload:sub(1, 5)), "aa01008008")
check("真实样本数不变（没有伪造样本）", uplink.get_stats().samples, 14)
check("空帧计数 = 2", uplink.get_stats().empty_frames, 2)
check("报文计数 = 3", uplink.get_stats().batches, 3)
check("发布计数 = 3", uplink.get_stats().published, 3)

-- 7.10 时钟微调（NTP 每小时对时）导致窗口内 ts 非单调时：BaseTimestamp 取窗口内
--      「最新采集时间」，偏移仍落在 0..255，不丢样本、不写出负偏移
do
    local base = VCLOCK
    local n_samples = uplink.get_stats().samples
    local function mk(ts)
        return bms_data.normalize({
            ts = ts, source = "modbus", cellCount = 1, cells = { 3300 },
            batVolMv = 3300, batCurrentMa = 0, soc = 50,
            tempBatC = 20, tempBat2C = 21, tempMosC = 21,
            balanceSw = 0, chargeMos = 1, dischargeMos = 1,
            alarms = { 0, 0, 0, 0, 0, 0, 0, 0, 0 },
            sysAlarm = 0, sysAlarmValid = true,
            capRemainMah = 1000, fullCapMah = 2000,
            balanCurrentMa = 0, cycleCount = 0, cycleCapMah = 0, soh = 100,
            bmsUpTimeS = 100,
        })
    end
    bms_cache.push(mk(base + 4))           -- 先到的一条时间更新
    bms_cache.push(mk(base))               -- 后到的一条时间更旧（NTP 回跳）
    local ok_flush = uplink.flush()
    check("非单调 ts 也能上报", ok_flush, true)
    check("两个样本都没有被丢", uplink.get_stats().samples - n_samples, 2)
    local dnm = decode_payload(published[#published].payload)
    check("BaseTimestamp = 窗口内最新采集时间", dnm.baseTimestamp, base + 4)
    check("Time 偏移非负（0/4）", dnm.time[1] .. "/" .. dnm.time[2], "0/4")
    check("消息级 BMS 上电时间取最新样本", dnm.bmsUpTime, 100)
end

--=============================================================================
-- 8. 极空 RS485 Modbus 主站（config.JK_PROTOCOL = "modbus" 的另一条数据源）
--    用 uart.write 打桩模拟一台极空从机，验证组帧 / CRC / 解析 / 单位换算 / 报警位，
--    并确认同一份规范样本（经 bms_data 归一 + bms_cache）能驱动金箭从机。
--=============================================================================
io.write("== 极空 RS485 Modbus 主站解析 ==\n")

local cfg_mb = require "config"
local JK_UART = cfg_mb.JK_UART_ID

-- 8.1 模拟从机寄存器表（地址 → 16bit 值），数值取手册样例量级
local sim = {}
for i = 0, 31 do sim[0x1200 + i] = 3304 end
sim[0x1206] = 3302                                    -- 第 7 节
sim[0x1207] = 3302                                    -- 第 8 节
sim[0x1240], sim[0x1241] = 0x000F, 0xFFFF             -- CellSta 低 20 位 = 1 → 20 节
sim[0x128A] = 330                                     -- TempMos 33.0℃
sim[0x1290], sim[0x1291] = 1, 534                     -- BatVol 66070mV
sim[0x1298], sim[0x1299] = 0, 3400                    -- BatCurrent +3400mA
sim[0x129C] = 320                                     -- TempBat1 32.0℃
sim[0x129E] = 325                                     -- TempBat2 32.5℃
sim[0x12A0], sim[0x12A1] = 0, 0                       -- Alarm = 0
sim[0x12A4] = 12                                      -- BalanCurrent 12mA
sim[0x12A6] = (2 << 8) | 51                           -- BalanSta=2(放电), SOC=51
sim[0x12A8], sim[0x12A9] = 0, 27500                   -- SOCCapRemain 27500mAh
sim[0x12AC], sim[0x12AD] = 0, 54000                   -- SOCFullChargeCap 54000mAh
sim[0x12B0], sim[0x12B1] = 0, 12                      -- SOCCycleCount 12
sim[0x12B4], sim[0x12B5] = 1, 57920                   -- SOCCycleCap 123456mAh (0x0001E240)
sim[0x12B8] = (100 << 8)                              -- SOCSOH 100%
sim[0x12BC], sim[0x12BD] = 1, 0x0E10                  -- RunTime 0x00010E10 = 69136s
sim[0x12C0] = (1 << 8) | 1                            -- 充电/放电 MOS 均闭合
sim[0x12C2] = 0

local sim_mode = "ok"   -- "ok" = 正常应答；"exception" = 返回异常帧
local requests = {}

local real_uart_setup, real_uart_write = _G.uart.setup, _G.uart.write
local jkmb
_G.uart.setup = function(id, ...)
    if id == JK_UART then
        return true                 -- 让 jk_modbus 的轮询任务能打开串口
    end
    return real_uart_setup(id, ...)
end
_G.uart.write = function(id, data)
    if not (id == JK_UART and #data == 8 and string.byte(data, 2) == 3) then
        return real_uart_write(id, data)   -- 金箭从机的应答等其它链路照旧
    end
    requests[#requests + 1] = data
    local addr  = string.byte(data, 1)
    local start = (string.byte(data, 3) << 8) | string.byte(data, 4)
    local num   = (string.byte(data, 5) << 8) | string.byte(data, 6)
    local body
    if sim_mode == "ok" then
        local out = { string.char(addr, 3, num * 2) }
        for i = 0, num - 1 do
            local v = sim[start + i] or 0
            out[#out + 1] = string.char((v >> 8) & 0xFF, v & 0xFF)
        end
        body = table.concat(out)
    else
        body = string.char(addr, 0x83, 0x02)   -- 异常响应：非法寄存器地址
    end
    local crc = jkmb.crc16(body)
    jkmb.feed(body .. string.char(crc & 0xFF, (crc >> 8) & 0xFF))
end

jkmb = dofile(here .. "/../jk_modbus.lua")

-- 8.2 CRC 与协议手册样例比对（CRC 低字节在前）
check("CRC 01 03 0005 0002 = 0x0AD4", jkmb.crc16(from_hex("01 03 00 05 00 02")), 0x0AD4)
check("CRC 01 03 04 11223344 = 0xC64B", jkmb.crc16(from_hex("01 03 04 11 22 33 44")), 0xC64B)

-- 8.3 轮询任务：开串口 → 一轮 3 帧请求 → 发布 JK_ROUND
pump()
local s_mb = jkmb.get_state()
check("Modbus 收到一轮数据", s_mb ~= nil, true)
check("请求帧1 = 读 0x1200 x32", to_hex(requests[1] and requests[1]:sub(1, 6)), "010312000020")
check("请求帧2 = 读 0x1240 x2", to_hex(requests[2] and requests[2]:sub(1, 6)), "010312400002")
check("请求帧3 = 读 0x128A x58", to_hex(requests[3] and requests[3]:sub(1, 6)), "0103128a003a")
do
    local req = requests[3] or ""
    local rc = jkmb.crc16(req:sub(1, 6))
    check("请求帧 CRC 正确", string.byte(req, 7) == (rc & 0xFF)
        and string.byte(req, 8) == ((rc >> 8) & 0xFF), true)
end
check("Modbus 串数(CellSta popcount)", s_mb and s_mb.cellCount, 20)
check("Modbus 总电压 mV", s_mb and s_mb.batVolMv, 66070)
check("Modbus 电流 mA(充电为正)", s_mb and s_mb.batCurrentMa, 3400)
check("Modbus SOC", s_mb and s_mb.soc, 51)
check("Modbus 单体1 mV", s_mb and s_mb.cells[1], 3304)
check("Modbus 单体7 mV", s_mb and s_mb.cells[7], 3302)
check("Modbus 单体20 mV", s_mb and s_mb.cells[20], 3304)
check("Modbus 单体21 mV(空槽位)", s_mb and s_mb.cells[21], 0)
check("Modbus 最大压差 mV", s_mb and s_mb.maxDiffMv, 2)
check("Modbus 电池温度1 ℃", s_mb and s_mb.tempBatC, 32)
check("Modbus 电池温度2 ℃", s_mb and s_mb.tempBat2C, 33)
check("Modbus MOS 温度 ℃", s_mb and s_mb.tempMosC, 33)
check("Modbus 均衡电流 mA", s_mb and s_mb.balanCurrentMa, 12)
check("Modbus 均衡开关", s_mb and s_mb.balanceSw, 1)
check("Modbus 充电 MOS", s_mb and s_mb.chargeMos, 1)
check("Modbus 放电 MOS", s_mb and s_mb.dischargeMos, 1)
check("Modbus 剩余容量 mAh", s_mb and s_mb.capRemainMah, 27500)
check("Modbus 实际容量 mAh", s_mb and s_mb.fullCapMah, 54000)
check("Modbus 循环次数", s_mb and s_mb.cycleCount, 12)
check("Modbus 循环容量 mAh", s_mb and s_mb.cycleCapMah, 123456)
check("Modbus SOH", s_mb and s_mb.soh, 100)
check("Modbus BMS 上电时间(0x12BC, u32 高低字)", s_mb and s_mb.bmsUpTimeS, 69136)
check("Modbus 报警位图", s_mb and s_mb.sysAlarm, 0)
check("Modbus 报警位图有效", s_mb and s_mb.sysAlarmValid, true)
check("Modbus 数据源标记", s_mb and s_mb.source, "modbus")
check("Modbus 统计轮数", jkmb.get_stats().rounds, 1)
check("Modbus 已发请求数(3 块)", jkmb.get_stats().requests, 3)
check("Modbus 无超时", jkmb.get_stats().timeouts, 0)
check("Modbus 无 CRC 错误", jkmb.get_stats().bad_crc, 0)
check("Modbus 数据新鲜", jkmb.is_fresh(), true)

-- 8.4 从机返回异常码 → 本轮失败、不产生新样本、保留上一轮快照
do
    local before_rounds = jkmb.get_stats().rounds
    sim_mode = "exception"
    sys.taskInit(function() jkmb.poll_once() end)
    pump()
    check("异常响应不产生新轮次", jkmb.get_stats().rounds, before_rounds)
    check("异常响应计入失败轮数", jkmb.get_stats().cycles_failed >= 1, true)
    check("异常后保留上一轮快照", jkmb.get_state().cellCount, 20)
    sim_mode = "ok"
end

-- 8.5 没有请求在途时收到的帧（迟到/噪声）直接丢弃，不会被配给下一个请求
do
    local before = jkmb.get_stats().late_frames
    local body = string.char(0x01, 0x03, 0x02, 0x12, 0x34)
    local crc = jkmb.crc16(body)
    jkmb.feed(body .. string.char(crc & 0xFF, (crc >> 8) & 0xFF))
    check("空闲时收到的帧计为迟到", jkmb.get_stats().late_frames, before + 1)
end

-- 8.6 报警位映射（bit8 充电过温）→ 归一入缓存 → 金箭从机寄存器/开关量
do
    sim[0x12A0], sim[0x12A1] = 0, 0x0100      -- bit8 充电过温保护
    sys.taskInit(function() jkmb.poll_once() end)
    pump()
    local s2 = jkmb.get_state()
    check("报警 bit8 → sysAlarm", s2 and s2.sysAlarm, 0x0100)
    check("报警 bit8 → alarms[5](电池过温)", s2 and s2.alarms[5], 1)
    check("报警位置位 → warn=1", s2 and s2.warn, 1)

    -- 同一份快照经适配层归一后进共享缓存；显式后写，保证 latest = modbus 样本
    -- （上面 pump() 时 display 侧采集任务也消费了同一个 JK_ROUND，那份样本更旧）
    bms_cache.push(bms_data.normalize(s2))
    local latest = bms_cache.latest()
    check("适配层：modbus source", latest and latest.source, "modbus")
    check("适配层：两路温度独立", latest and (latest.tempBat1C .. "/" .. latest.tempBat2C), "32/33")
    check("适配层：均衡电流", latest and latest.balanCurrentMa, 12)
    check("适配层：循环总容量", latest and latest.socCycleCapMah, 123456)
    check("适配层：满电容量", latest and latest.socFullChargeCapMah, 54000)
    check("适配层：BMS 上电时间", latest and latest.bmsUpTimeS, 69136)

    local jj2 = dofile(here .. "/../jinjian_slave.lua")
    local resp = jj2.handle_frame(build_frame("01 03 00 00 00 09"))
    local function reg2(a) return (string.byte(resp, 4 + a * 2) << 8) | string.byte(resp, 5 + a * 2) end
    check("金箭 reg0 总电压(0.01V)", reg2(0), 6607)
    check("金箭 reg1 串数", reg2(1), 20)
    check("金箭 reg2 SOC", reg2(2), 51)
    check("金箭 reg3 容量 Ah", reg2(3), 54)
    check("金箭 reg5 电流(0.01A)", reg2(5), 340)
    check("金箭 reg8 板温", reg2(8), 33)
    do
        local r = jj2.handle_frame(build_frame("01 03 00 68 00 01"))   -- reg104 循环次数
        check("金箭 reg104 循环次数(来自 Modbus)", (string.byte(r, 4) << 8) | string.byte(r, 5), 12)
    end
    do
        local r = jj2.handle_frame(build_frame("01 01 00 04 00 05"))   -- 保护开关量 4-8
        check("金箭开关量 4-8(仅过温充电置位)", string.byte(r, 4), 0x02)
    end

    -- 编码语义：列 6 = 循环总容量（0x12B4），消息级 SOCFullChargeCap = 满电容量（0x12AC）
    local function hist_from_sample(s_sample)
        local C = s_sample.cellCount
        local h = {
            count = 1, cellCount = C,
            temp1 = { s_sample.tempBat1C * 10 }, temp2 = { s_sample.tempBat2C * 10 },
            tempMos = { s_sample.tempMosC * 10 },
            balanCurrent = { s_sample.balanCurrentMa },
            batVol = { s_sample.batVolMv }, batCurrent = { s_sample.batCurrentMa },
            socCycleCap = { s_sample.socCycleCapMah },
            socCapRemain = { s_sample.socCapRemainMah },
            time = { 0 }, cellVols = {},
        }
        for j = 1, C do
            h.cellVols[j] = s_sample.cells[j] or 0
        end
        return h
    end
    local p1 = codec.encode(hist_from_sample(latest), {
        socFullChargeCap = latest.socFullChargeCapMah,
        baseTimestamp = latest.ts,
        moduleUpTime = 0, bmsUpTime = latest.bmsUpTimeS,
        jinjianModbusRequestCount = 0, jikongModbusRequestCount = 3,
    })
    local d1 = decode_payload(p1)
    check("modbus 单样本编码可解码", d1.count, 1)
    check("列 6 = 循环总容量（0x12B4）", d1.socCycleCap[1], 123456)
    check("消息级 = 满电容量（0x12AC）", d1.socFullChargeCap, 54000)
    check("消息级 BMS 上电时间 = 0x12BC", d1.bmsUpTime, 69136)
    check("消息级 极空 modbus 请求计数", d1.jikong, 3)
    check("均衡电流列（modbus 有值）", d1.balanCurrent[1], 12)
    check("温度列两路独立（0.1℃，快照为整数℃）", d1.temp1[1] .. "/" .. d1.temp2[1], "320/330")
end

-- 8.7 原始帧十六进制日志限频（LOG_RAW_FRAMES 打开时每 LOG_HEX_EVERY_N 帧打一次）
do
    local saved_info = _G.log.info
    local hex_lines = 0
    _G.log.info = function(tag, ...)
        if tag == "jkmb.raw" then
            hex_lines = hex_lines + 1
            return
        end
        return saved_info(tag, ...)
    end
    cfg_mb.LOG_RAW_FRAMES = true
    for _ = 1, 3 do                       -- 3 轮 = 9 帧请求 = 18 次 hex 打印机会
        sys.taskInit(function() jkmb.poll_once() end)
        pump()
    end
    check("原始帧 hex 每 N 帧只打一次", hex_lines, 1)
    cfg_mb.LOG_RAW_FRAMES = false
    _G.log.info = saved_info
end

-- 8.8 收尾：还原串口打桩、停止 Modbus 轮询任务
jkmb.stop()
_G.uart.setup = real_uart_setup
_G.uart.write = real_uart_write

--=============================================================================
-- 9. 串口回调只收字节 + 独立解析任务（回调里不做重活的实现验证）
--=============================================================================
io.write("== 接收回调与解析任务拆分 ==\n")

local saved_uart = { setup = _G.uart.setup, on = _G.uart.on, read = _G.uart.read,
                     write = _G.uart.write }
local handlers, rxq = {}, ""
local written = {}
_G.uart.on = function(id, _evt, cb) handlers[id] = cb end
_G.uart.read = function(_id, _len)
    local s = rxq
    rxq = ""
    return s
end
_G.uart.write = function(id, data) written[#written + 1] = { id = id, data = data } end
_G.uart.setup = function(id) return id == JK_UART or id == cfg_mb.JJ_UART_ID end

-- 9.1 金箭从机：回调只搬字节，帧扫描与应答由解析任务完成
do
    local jj3 = dofile(here .. "/../jinjian_slave.lua")
    pump()   -- 打开串口 → 注册接收回调 → 进入等待
    check("金箭接收回调已注册", handlers[cfg_mb.JJ_UART_ID] ~= nil, true)

    local req91 = build_frame("01 03 00 00 00 09")
    rxq = req91
    handlers[cfg_mb.JJ_UART_ID](cfg_mb.JJ_UART_ID)   -- 模拟串口中断：只搬运字节
    check("回调本身不应答", #written, 0)
    check("金箭：回调把原始字节计入 rx_bytes", jj3.get_stats().rx_bytes, #req91)
    pump()                                            -- 解析任务被 JJ_RX 事件唤醒
    check("解析任务发出应答", #written, 1)
    check("应答功能码 01 03", written[1] and to_hex(written[1].data:sub(1, 2)), "0103")
    check("应答字节数 18", written[1] and string.byte(written[1].data, 3), 18)
end

-- 9.2 突发帧流按预算分批处理（每 JJ_PARSE_MAX_FRAMES 帧主动让出一次）
do
    local req = build_frame("01 03 00 00 00 09")
    local before = #written
    rxq = string.rep(req, 12)
    handlers[cfg_mb.JJ_UART_ID](cfg_mb.JJ_UART_ID)
    pump()
    check("突发帧先处理一批(预算 8)", #written - before, 8)
    scheduler.wake()   -- 模拟 wait(1) 到期，继续处理剩余帧
    pump()
    check("让出后处理完剩余帧", #written - before, 12)
end

-- 9.3 极空显示屏：回调只搬字节，整轮解析由解析任务完成
do
    local jk5 = dofile(here .. "/../jk_display.lua")
    pump()   -- 打开串口 → 注册接收回调 → 进入等待
    check("极空接收回调已注册", handlers[cfg_mb.JK_UART_ID] ~= nil, true)

    rxq = round
    handlers[cfg_mb.JK_UART_ID](cfg_mb.JK_UART_ID)
    check("回调本身不解析", jk5.get_state(), nil)
    check("极空：回调把原始字节计入 rx_bytes", jk5.get_stats().rx_bytes, #round)
    pump()
    local s5 = jk5.get_state()
    check("解析任务产出快照", s5 ~= nil and s5.cellCount, 20)
    check("解析任务与直接喂帧结果一致", s5 and s5.batVolMv, 66070)
end

-- 9.4 收尾：还原串口打桩
_G.uart.setup, _G.uart.on, _G.uart.read, _G.uart.write =
    saved_uart.setup, saved_uart.on, saved_uart.read, saved_uart.write

--=============================================================================
-- 离线 JSON（真实现，够用即可）：mgmt / cfg_store / 两条传输都依赖全局 json。
--   · encode：递归；键排序便于字节级比对；字符串做标准转义；
--   · decode：递归下降，支持对象/数组/字符串转义/数字/true/false/null。
--=============================================================================
local function make_json()
    local ESC = { ['"'] = '\\"', ['\\'] = '\\\\', ['\n'] = '\\n', ['\r'] = '\\r',
                  ['\t'] = '\\t', ['\b'] = '\\b', ['\f'] = '\\f' }
    local function esc(s)
        return '"' .. s:gsub('[%c\\"]', function(c)
            return ESC[c] or string.format("\\u%04x", c:byte())
        end) .. '"'
    end
    local function enc(v)
        local t = type(v)
        if v == nil then return "null" end
        if t == "boolean" then return v and "true" or "false" end
        if t == "number" then
            if math.type(v) == "integer" then return string.format("%d", v) end
            return string.format("%.14g", v)
        end
        if t == "string" then return esc(v) end
        if t == "table" then
            if #v > 0 then
                local parts = {}
                for i = 1, #v do parts[i] = enc(v[i]) end
                return "[" .. table.concat(parts, ",") .. "]"
            end
            local keys = {}
            for k in pairs(v) do keys[#keys + 1] = tostring(k) end
            table.sort(keys)
            local parts = {}
            for _, k in ipairs(keys) do
                parts[#parts + 1] = esc(k) .. ":" .. enc(v[k])
            end
            return "{" .. table.concat(parts, ",") .. "}"
        end
        return "null"
    end

    local s, pos
    local function skip()
        while true do
            local c = s:sub(pos, pos)
            if c == " " or c == "\t" or c == "\n" or c == "\r" then pos = pos + 1 else break end
        end
    end
    local decode_value
    local function decode_string()
        pos = pos + 1
        local out = {}
        while true do
            local c = s:sub(pos, pos)
            if c == "" then error("unterminated string") end
            if c == '"' then pos = pos + 1 break end
            if c == "\\" then
                local e = s:sub(pos + 1, pos + 1); pos = pos + 2
                if e == "n" then out[#out + 1] = "\n"
                elseif e == "t" then out[#out + 1] = "\t"
                elseif e == "r" then out[#out + 1] = "\r"
                elseif e == "b" then out[#out + 1] = "\b"
                elseif e == "f" then out[#out + 1] = "\f"
                elseif e == "/" then out[#out + 1] = "/"
                elseif e == "\\" then out[#out + 1] = "\\"
                elseif e == '"' then out[#out + 1] = '"'
                elseif e == "u" then
                    local n = tonumber(s:sub(pos, pos + 3), 16); pos = pos + 4
                    out[#out + 1] = (n and n < 128) and string.char(n) or "?"
                else error("bad escape") end
            else
                out[#out + 1] = c; pos = pos + 1
            end
        end
        return table.concat(out)
    end
    decode_value = function()
        skip()
        local c = s:sub(pos, pos)
        if c == "{" then
            pos = pos + 1
            local obj = {}
            skip()
            if s:sub(pos, pos) == "}" then pos = pos + 1 return obj end
            while true do
                skip()
                local k = decode_string()
                skip()
                if s:sub(pos, pos) ~= ":" then error("expect :") end
                pos = pos + 1
                obj[k] = decode_value()
                skip()
                local d = s:sub(pos, pos); pos = pos + 1
                if d == "}" then return obj end
                if d ~= "," then error("expect , or }") end
            end
        elseif c == "[" then
            pos = pos + 1
            local arr = {}
            skip()
            if s:sub(pos, pos) == "]" then pos = pos + 1 return arr end
            while true do
                arr[#arr + 1] = decode_value()
                skip()
                local d = s:sub(pos, pos); pos = pos + 1
                if d == "]" then return arr end
                if d ~= "," then error("expect , or ]") end
            end
        elseif c == '"' then
            return decode_string()
        elseif s:sub(pos, pos + 3) == "true" then pos = pos + 4 return true
        elseif s:sub(pos, pos + 4) == "false" then pos = pos + 5 return false
        elseif s:sub(pos, pos + 3) == "null" then pos = pos + 4 return nil
        else
            local num = s:match("^-?%d+%.?%d*[eE]?[+-]?%d*", pos)
            if not num or num == "" then error("bad value at " .. pos) end
            pos = pos + #num
            return tonumber(num)
        end
    end
    return {
        encode = function(v) return enc(v) end,
        decode = function(str) s, pos = str, 1; local v = decode_value(); return v end,
    }
end

_G.json = make_json()

--=============================================================================
-- 10. mgmt_uart：帧编解码与重同步（裸帧：SOF/VER/TYPE/ID/LEN/CRC）
--     验证：CRC16(0xA001)、组帧、扫帧（前导垃圾重同步 / CRC 错 / VER 不符 / 超长计数）、
--           分片到达、一包多帧、尾部不完整保留。
--=============================================================================
io.write("== mgmt_uart 帧编解码与重同步 ==\n")

local uframe = require "mgmt_uart"

check("CRC16(01 03 00 00 00 09) = 0xCC85（0xA001 同 Modbus 多项式）",
    uframe.crc16(from_hex("01 03 00 00 00 09")), 0xCC85)

do
    local payload = '{"method":"sys.ping"}'
    local f = uframe.build(uframe.TYPE_REQ, 7, payload)
    check("帧 SOF = A5 5A", to_hex(f:sub(1, 2)), "a55a")
    check("帧 VER = 01", string.byte(f, 3), 0x01)
    check("帧 TYPE = 01（REQ）", string.byte(f, 4), uframe.TYPE_REQ)
    check("帧 ID = 07", string.byte(f, 5), 7)
    local flen = string.byte(f, 6) | (string.byte(f, 7) << 8)
    check("LEN 小端 = 载荷长度", flen, #payload)
    check("载荷原样", f:sub(8, 7 + #payload), payload)
    check("帧总长 = 9 + LEN", #f, 9 + #payload)
    local c = uframe.crc16(f:sub(3, 7 + #payload))
    check("CRC 覆盖 VER..PAYLOAD（低字节在前）",
        string.format("%02x%02x", string.byte(f, #f - 1), string.byte(f, #f)),
        string.format("%02x%02x", c & 0xFF, (c >> 8) & 0xFF))

    -- 扫帧：单帧
    local frames, rest = uframe.scan(f, {})
    check("单帧：切出 1 帧", #frames, 1)
    check("单帧：类型/ID 正确", frames[1] and (frames[1].type .. "/" .. frames[1].id), "1/7")
    check("单帧：载荷原样", frames[1] and frames[1].payload, payload)
    check("单帧：无残留", rest, "")

    -- 前导垃圾 → 重同步并计入 sync
    local st = {}
    frames = uframe.scan("abc" .. f, st)
    check("前导垃圾：仍切出 1 帧", #frames, 1)
    check("前导垃圾：计入 sync", st.sync, 3)

    -- CRC 错 → 丢弃并计入 crc
    st = {}
    local bad = f:sub(1, #f - 1) .. string.char((string.byte(f, #f) ~ 0xFF) & 0xFF)
    frames = uframe.scan(bad, st)
    check("CRC 错：不产出帧", #frames, 0)
    check("CRC 错：计入 crc", st.crc, 1)

    -- VER 不符 → 丢帧并计入 ver
    st = {}
    local badver = string.char(string.byte(f, 1), string.byte(f, 2), 0x02) .. f:sub(4)
    frames = uframe.scan(badver, st)
    check("VER 不符：不产出帧", #frames, 0)
    check("VER 不符：计入 ver", st.ver, 1)

    -- 超长（JSON 类型超过 MGMT_MAX_PAYLOAD=1024）→ 计入 overlong
    st = {}
    frames = uframe.scan(uframe.build(uframe.TYPE_REQ, 1, string.rep("x", 1100)), st, 1024)
    check("JSON 超长：不产出帧", #frames, 0)
    check("JSON 超长：计入 overlong", st.overlong, 1)

    -- BLOB 类型上限 242（DATA ≤ 238）
    st = {}
    frames = uframe.scan(uframe.build(uframe.TYPE_BLOB, 1, string.rep("x", 243)), st, 1024)
    check("BLOB 超长（243 > 242）：不产出帧", #frames, 0)
    check("BLOB 超长：计入 overlong", st.overlong, 1)

    -- 分片到达：先半帧、再补齐
    local half = #f // 2
    st = {}
    frames, rest = uframe.scan(f:sub(1, half), st)
    check("分片：半帧不产出", #frames, 0)
    check("分片：半帧被保留", rest, f:sub(1, half))
    frames = uframe.scan(rest .. f:sub(half + 1), st)
    check("分片：补齐后产出 1 帧", #frames, 1)

    -- 一包多帧
    frames = uframe.scan(f .. f .. f, {})
    check("一包多帧：切出 3 帧", #frames, 3)

    -- 尾部不完整保留
    frames, rest = uframe.scan(f .. f:sub(1, 5), {})
    check("尾部不完整：只产出完整帧", #frames, 1)
    check("尾部不完整：残留是半个头", rest, f:sub(1, 5))

    -- 纯垃圾：只保留最后 1 字节
    st = {}
    frames, rest = uframe.scan(string.rep("Z", 10), st)
    check("纯垃圾：不产出帧", #frames, 0)
    check("纯垃圾：只保留末字节", #rest, 1)
    check("纯垃圾：计入 sync", st.sync, 9)
end

--=============================================================================
-- 11. mgmt：统一管理服务（方法分发 / 错误码 / 配置 / 命令 / FOTA / 去重 / 事件）
--=============================================================================
io.write("== mgmt 统一管理服务 ==\n")

local cfg_mg = require "config"

-- fskv 打桩（存 JSON 文本）
local function make_fskv()
    local kv = {}
    return kv, {
        init = function() return true end,
        set = function(k, v) kv[k] = v; return true end,
        get = function(k) return kv[k] end,
        del = function(k) kv[k] = nil; return true end,
    }
end

local mg_kv, mg_fskv = make_fskv()
_G.fskv = mg_fskv

-- 被改动的配置键先备份，章节结束时还原
local mg_saved = {}
for _, k in ipairs({ "MGMT_BAUD", "LOG_ENABLE", "REPORT_PERIOD_MS", "STATUS_REPORT_MS",
        "JK_PROTOCOL", "JK_MODBUS_SLAVE_ADDR", "JK_MODBUS_POLL_MS", "JJ_LOW_TEMP_CHARGE_C",
        "JJ_LOW_TEMP_DISCHARGE_C", "JJ_FAST_CHARGE_CURRENT_MA", "MQTT_PORT", "MQTT_IS_SSL",
        "MQTT_QOS", "MQTT_KEEPALIVE_S", "MQTT_HOST", "MQTT_TOPIC_PREFIX" }) do
    mg_saved[k] = cfg_mg[k]
end

local store_m = require "cfg_store"
store_m.load()

-- fota_mgr 打桩（记账 + 可控失败，真实状态机见 §13）
local fake_fota = {
    state = 0, result = 0, progress = 0, mode = 0, stage = 0, rx = 0, total = 0,
    last_seq = 0, cache = 0, core = false, lib = false, enabled = true,
    busy = false, local_busy = false,
    started = {}, begin = {}, write = {}, abort = 0,
    fail_start = nil, fail_begin = nil, fail_write = nil,
}
function fake_fota.info()
    return { state = fake_fota.state, result = fake_fota.result, progress = fake_fota.progress,
             enabled = fake_fota.enabled, mode = fake_fota.mode, stage = fake_fota.stage,
             rx = fake_fota.rx, total = fake_fota.total, last_seq = fake_fota.last_seq,
             cache = fake_fota.cache, core = fake_fota.core, lib = fake_fota.lib }
end
function fake_fota.is_busy() return fake_fota.busy end
function fake_fota.is_local_busy() return fake_fota.local_busy end
function fake_fota.start(url)
    if fake_fota.fail_start then return false, fake_fota.fail_start end
    fake_fota.started[#fake_fota.started + 1] = url
    fake_fota.state = 1
    return true
end
function fake_fota.local_begin(total, crc, flags)
    if fake_fota.fail_begin then return false, fake_fota.fail_begin end
    fake_fota.begin[#fake_fota.begin + 1] = { total = total, crc = crc, flags = flags }
    return true
end
function fake_fota.local_write(seq, offset, data)
    if fake_fota.fail_write then return false, fake_fota.fail_write end
    fake_fota.write[#fake_fota.write + 1] = { seq = seq, offset = offset, data = data }
    fake_fota.rx = fake_fota.rx + #data
    return true
end
function fake_fota.local_abort(_reason) fake_fota.abort = fake_fota.abort + 1; return true end

local snap = {
    net_ready = true, mqtt = "up", jk_valid = true, power_on = true,
    needs_restart = false, has_override = false, factory_pending = false, fota_busy = false,
    reset_reason = 5, uptime_s = 70000, supply_mv = 50120,
    device = { project = "jjsoc", model = "D700Tm", version = "1.0.0-C",
               pn = "SOC-234567890123", proto = 1 },
    links = {
        jk = { rounds = 42, frames = 420, rx = 12345, sync_lost = 0, valid = true },
        jinjian = { queries = 9, responses = 9, ignored = 0, rx = 100 },
        mgmt_uart = { rx_bytes = 11, frames = 1, replies = 1, crc = 0, sync = 0,
                      drop = 0, tx_bytes = 22, evt = 1 },
        mgmt_mqtt = { rx = 0, reply = 0, ignored = 0, bad = 0 },
    },
    cache = { count = 14, capacity = 32 },
    uplink = { published = 77, samples = 1, empty = 2, lost = 3, batches = 4, dropped = 5 },
    fota = { mode = 0, state = 0, stage = 0, result = 0, progress = 0, rx = 0, total = 0,
             last_seq = 0,
             caps = { core = false, lib = false, local_enabled = true, url_enabled = true,
                      fskv = true, json = true } },
}

local posted = {}
local fake_now = 100000

local mgmt_m = require "mgmt"
local inst = mgmt_m.new{
    cfg = cfg_mg, store = store_m, fota = fake_fota, board = nil, no_watch = true,
    now_ms = function() return fake_now end,
    post = function(fn, ms) posted[#posted + 1] = { fn = fn, ms = ms } end,
    snapshot = function() return snap end,
}

local function rpc(method, params, id)
    local req = { method = method }
    if params ~= nil then req.params = params end
    return inst:handle(req, id)
end
local function rpc_json(s, id) return inst:handle(s, id) end

-- 11.0 hello
do
    local h = inst:hello()
    check("hello：project/model/version", h.project .. "/" .. h.model .. "/" .. h.version,
        "jjsoc/D700Tm/1.0.0-C")
    check("hello：pn 来自慢外设缓存", h.pn, "SOC-234567890123")
    check("hello：reset_reason / uptime_s", h.reset_reason .. "/" .. h.uptime_s, "5/70000")
    check("hello：proto = 1", h.proto, 1)
    check("hello：caps 为表", type(h.caps), "table")
end

-- 11.1 分发与基础错误码
do
    local r = rpc("sys.ping")
    check("sys.ping → ok", r.ok, true)
    check("sys.ping → 带 uptime_s", r.result and r.result.uptime_s, 70000)
    check("未知方法 → EBADMETHOD", rpc("no.such").error.code, "EBADMETHOD")
    check("非 JSON 载荷 → EBADJSON", rpc_json("{not json").error.code, "EBADJSON")
    check("JSON 合法但缺 method → EBADMETHOD", rpc_json('{"params":{}}').error.code, "EBADMETHOD")
    check("method 非字符串 → EBADMETHOD", rpc_json('{"method":123}').error.code, "EBADMETHOD")
    check("json 全局可用", type(_G.json.encode), "function")
end

-- 11.2 status.get（大对象透传）
do
    local r = rpc("status.get")
    check("status.get → ok", r.ok, true)
    check("status.get：net_ready/mqtt",
        tostring(r.result.net_ready) .. "/" .. r.result.mqtt, "true/up")
    check("status.get：device.model", r.result.device.model, "D700Tm")
    check("status.get：links.jk.rounds", r.result.links.jk.rounds, 42)
    check("status.get：links.mgmt_uart.frames", r.result.links.mgmt_uart.frames, 1)
    check("status.get：cache 高低", r.result.cache.count .. "/" .. r.result.cache.capacity, "14/32")
    check("status.get：uplink.batches", r.result.uplink.batches, 4)
    check("status.get：fota.caps.core", r.result.fota.caps.core, false)
    check("status.get：supply_mv", r.result.supply_mv, 50120)
end

-- 11.3 config.get
do
    local r = rpc("config.get")
    check("config.get → ok", r.ok, true)
    check("config.get：values 含白名单键（MGMT_BAUD）", r.result.values.MGMT_BAUD ~= nil, true)
    check("config.get：values 含 MQTT_HOST", r.result.values.MQTT_HOST ~= nil, true)
    check("config.get：restart_keys 是数组（含 MGMT_BAUD）",
        (function() for _, k in ipairs(r.result.restart_keys) do if k == "MGMT_BAUD" then return true end end return false end)(),
        true)
end

-- 11.4 config.set：全量校验 + 原子写入 + 需重启
do
    store_m.reset_memory()
    local r = rpc("config.set", { LOG_ENABLE = false, JK_MODBUS_POLL_MS = 1500 })
    check("config.set 批量成功", r.ok, true)
    check("config.set：applied 两个键", (r.result.applied.LOG_ENABLE == false)
        and (r.result.applied.JK_MODBUS_POLL_MS == 1500), true)
    check("config.set：实时键立即生效", cfg_mg.LOG_ENABLE, false)
    check("config.set：写入 config", cfg_mg.JK_MODBUS_POLL_MS, 1500)
    check("config.set：落盘（store.get）", store_m.get("LOG_ENABLE"), false)
    check("config.set：实时键不置需重启", r.result.restart_required, false)

    check("config.set：非法键 → EBADKEY", rpc("config.set", { NOT_A_KEY = 1 }).error.code, "EBADKEY")
    check("config.set：越界值 → EBADVALUE",
        rpc("config.set", { JK_MODBUS_POLL_MS = 50 }).error.code, "EBADVALUE")
    check("config.set：类型非法 → EBADVALUE",
        rpc("config.set", { MQTT_HOST = 123 }).error.code, "EBADVALUE")
    check("config.set：params 非对象 → EBADPARAM", rpc("config.set", "x").error.code, "EBADPARAM")
    check("config.set：空对象 → EBADPARAM", rpc("config.set", {}).error.code, "EBADPARAM")

    -- 原子性：一批里混了非法键 → 整批不落盘
    local before = cfg_mg.LOG_ENABLE
    local bad = rpc("config.set", { LOG_ENABLE = true, BOGUS = 1 })
    check("config.set 原子性：混合非法键 → EBADKEY", bad.error.code, "EBADKEY")
    check("config.set 原子性：合法项也没落盘", cfg_mg.LOG_ENABLE, before)

    local rr = rpc("config.set", { REPORT_PERIOD_MS = 60000 })
    check("config.set：[R] 键 → restart_required", rr.result.restart_required, true)
    check("config.set：store 记需重启", store_m.needs_restart(), true)
end

-- 11.5 cmd.reboot：回帧先于动作（动作排期 200ms）
do
    local reboots = 0
    local saved = _G.rtos.reboot
    _G.rtos.reboot = function() reboots = reboots + 1 end
    posted = {}
    local r = rpc("cmd.reboot")
    check("cmd.reboot → ok", r.ok, true)
    check("cmd.reboot：动作已排期", #posted, 1)
    check("cmd.reboot：延时 200ms", posted[1] and posted[1].ms, 200)
    check("cmd.reboot：调用返回时尚未重启（回帧在先）", reboots, 0)
    posted[1].fn()
    check("cmd.reboot：排期到点后执行重启", reboots, 1)
    _G.rtos.reboot = saved
end

-- 11.6 cmd.factory_reset：清覆盖 + 置待生效位
do
    store_m.reset_memory()
    store_m.stage("MQTT_KEEPALIVE_S", 33)
    store_m.save()
    check("恢复出厂前存在覆盖", store_m.has_override(), true)
    local r = rpc("cmd.factory_reset")
    check("cmd.factory_reset → ok", r.ok, true)
    check("恢复出厂清空覆盖", store_m.has_override(), false)
    check("恢复出厂清 fskv 键", mg_kv[store_m.key()], nil)
    check("恢复出厂置待生效", store_m.factory_pending(), true)
end

-- 11.7 FOTA：url / begin / chunk / abort / status + 错误码映射
do
    fake_fota.fail_start, fake_fota.fail_begin, fake_fota.fail_write = nil, nil, nil
    check("fota.url 空 URL → EBADPARAM", rpc("fota.url", {}).error.code, "EBADPARAM")
    check("fota.url 正常 → started", rpc("fota.url", { url = "http://x/y.bin" }).result.started, true)
    check("fota.url 已带 URL 交给 fota_mgr", fake_fota.started[#fake_fota.started], "http://x/y.bin")
    fake_fota.fail_start = "busy"
    check("fota.url 忙 → EBUSY", rpc("fota.url", { url = "http://x/y" }).error.code, "EBUSY")
    fake_fota.fail_start = "lib missing"
    check("fota.url 缺库 → ENOLIB", rpc("fota.url", { url = "http://x/y" }).error.code, "ENOLIB")
    fake_fota.fail_start = "disabled"
    check("fota.url 通道关闭 → EFOTA", rpc("fota.url", { url = "http://x/y" }).error.code, "EFOTA")
    fake_fota.fail_start = nil

    check("fota.begin 缺 total → EBADPARAM", rpc("fota.begin", {}).error.code, "EBADPARAM")
    check("fota.begin 正常 → started", rpc("fota.begin", { total = 241 }).result.started, true)
    check("fota.begin：total 透传", fake_fota.begin[#fake_fota.begin].total, 241)
    check("fota.begin：crc32 缺省为 0", fake_fota.begin[#fake_fota.begin].crc, 0)
    fake_fota.fail_begin = "busy"
    check("fota.begin 忙 → EBUSY", rpc("fota.begin", { total = 10 }).error.code, "EBUSY")
    fake_fota.fail_begin = "bad total"
    check("fota.begin 参数不符 → EBADPARAM", rpc("fota.begin", { total = 10 }).error.code, "EBADPARAM")
    fake_fota.fail_begin = "lib missing"
    check("fota.begin 缺库 → ENOLIB", rpc("fota.begin", { total = 10 }).error.code, "ENOLIB")
    fake_fota.fail_begin = nil

    -- fota.chunk（MQTT 路径，data = base64；偏移由设备按已收字节推得）
    fake_fota.rx = 10
    local b64 = require("mgmt_mqtt").b64_encode("abcd")
    check("base64(abcd) = YWJjZA==", b64, "YWJjZA==")
    local r = rpc("fota.chunk", { seq = 2, data = b64 })
    check("fota.chunk → ok", r.ok, true)
    check("fota.chunk：seq/偏移/数据", fake_fota.write[#fake_fota.write].seq
        .. "/" .. fake_fota.write[#fake_fota.write].offset
        .. "/" .. fake_fota.write[#fake_fota.write].data, "2/10/abcd")
    check("fota.chunk：返回已收字节", r.result.rx, 14)
    check("fota.chunk：坏 base64 → EBADPARAM",
        rpc("fota.chunk", { seq = 3, data = "!!!notbase64!!!" }).error.code, "EBADPARAM")
    check("fota.chunk：缺 data → EBADPARAM", rpc("fota.chunk", { seq = 3 }).error.code, "EBADPARAM")
    fake_fota.fail_write = "seq"
    check("fota.chunk 乱序 → EBADSEQ", rpc("fota.chunk", { seq = 9, data = b64 }).error.code, "EBADSEQ")
    fake_fota.fail_write = "run"
    check("fota.chunk 写 Flash 失败 → EFLASH", rpc("fota.chunk", { seq = 3, data = b64 }).error.code, "EFLASH")
    fake_fota.fail_write = "lib missing"
    check("fota.chunk 缺库 → ENOLIB", rpc("fota.chunk", { seq = 3, data = b64 }).error.code, "ENOLIB")
    fake_fota.fail_write = nil

    check("fota.abort → aborted", rpc("fota.abort").result.aborted, true)
    fake_fota.mode, fake_fota.stage, fake_fota.state = 2, 3, 1
    fake_fota.rx, fake_fota.total, fake_fota.last_seq = 100, 400, 5
    local fs = rpc("fota.status").result
    check("fota.status：mode/stage/state", fs.mode .. "/" .. fs.stage .. "/" .. fs.state, "2/3/1")
    check("fota.status：rx/total/last_seq", fs.rx .. "/" .. fs.total .. "/" .. fs.last_seq, "100/400/5")
    fake_fota.mode, fake_fota.stage, fake_fota.state = 0, 0, 0
    fake_fota.rx, fake_fota.total, fake_fota.last_seq = 0, 0, 0
end

-- 11.8 UART BLOB 路径：instance:fota_blob（偏移同样由设备推得）
do
    fake_fota.rx = 0
    local r = inst:fota_blob(1, "HELLO")
    check("fota_blob → ok", r.ok, true)
    check("fota_blob：seq/偏移/数据", fake_fota.write[#fake_fota.write].seq
        .. "/" .. fake_fota.write[#fake_fota.write].offset
        .. "/" .. fake_fota.write[#fake_fota.write].data, "1/0/HELLO")
    local bad = inst:fota_blob(2, string.rep("x", 239))
    check("fota_blob：DATA > 238 → EBADPARAM", bad.error.code, "EBADPARAM")
end

-- 11.9 ID 去重：同 ID + 同方法在窗口内只执行一次（重放缓存响应）
do
    local reboots = 0
    local saved = _G.rtos.reboot
    _G.rtos.reboot = function() reboots = reboots + 1 end
    posted = {}
    local r1 = rpc("cmd.reboot", nil, 42)
    check("去重：首次执行排期", #posted, 1)
    local r2 = rpc("cmd.reboot", nil, 42)
    check("去重：窗口内重复 → 不再排期", #posted, 1)
    check("去重：重放同一响应", r1.ok == r2.ok and r1.result.rebooting == r2.result.rebooting, true)
    check("去重：命中计数", inst.stats.dedup_hits >= 1, true)

    -- 同 ID 但方法不同 → 正常执行
    local r3 = rpc("sys.ping", nil, 42)
    check("去重：同 ID 不同方法仍执行", r3.ok, true)
    -- 去掉重不同 ID、同方法 → 正常执行
    rpc("cmd.reboot", nil, 43)
    check("去重：不同 ID 正常执行", #posted, 2)
    -- 超出窗口 → 重新执行
    fake_now = fake_now + (cfg_mg.MGMT_DEDUP_WINDOW_MS or 10000) + 1
    rpc("cmd.reboot", nil, 42)
    check("去重：超出窗口后重新执行", #posted, 3)
    posted[1].fn(); posted[2].fn(); posted[3].fn()
    _G.rtos.reboot = saved
end

-- 11.10 事件：hello / fota → MGMT_EVT（no_watch=false 时才订阅 fota 事件）
do
    local seen = {}
    local inst_e = mgmt_m.new{
        cfg = cfg_mg, store = store_m, fota = fake_fota, board = nil, no_watch = false,
        now_ms = function() return fake_now end, post = function() end,
        snapshot = function() return snap end }
    _G.sys.subscribe("MGMT_EVT", function(e) seen[#seen + 1] = e end)

    -- 直接触发本实例的 fota 事件（避开单例 mgmt 的同一事件订阅）
    fake_fota.mode, fake_fota.stage, fake_fota.state = 2, 5, 2
    inst_e:emit_fota_evt()
    check("FOTA 事件 → 推 MGMT_EVT(evt=fota)", seen[1] and seen[1].evt, "fota")
    check("FOTA 事件 data 含 mode/state/stage",
        seen[1] and (seen[1].data.mode .. "/" .. seen[1].data.state .. "/" .. seen[1].data.stage),
        "2/2/5")

    -- fota_mgr 的 fota_done 事件也会被订阅（no_watch=false）并推送 MGMT_EVT
    local n_before = #seen
    scheduler.publish("fota_done", 0)
    check("fota_done 事件 → 至少推 1 条 MGMT_EVT", #seen > n_before, true)
    fake_fota.mode, fake_fota.stage, fake_fota.state = 0, 0, 0
    check("hello 可构造（供 UART 传输主动发）", inst_e:hello().proto, 1)
end

-- 11.11 收尾：还原被改动的配置键与 store 内存状态
do
    for k, v in pairs(mg_saved) do
        cfg_mg[k] = v
    end
    store_m.reset_memory()
end

--=============================================================================
-- 12. cfg_store：白名单 / 类型 / 范围校验、写透往返、覆盖在模块加载前生效、
--     非法项跳过、恢复出厂、fskv 缺失降级
--=============================================================================
io.write("== cfg_store 配置持久化（fskv 打桩） ==\n")

do
    local kv12, fskv12 = make_fskv()
    _G.fskv = fskv12
    local store_a = dofile(here .. "/../cfg_store.lua")

    check("未知键被拒绝", store_a.set("NOT_A_KEY", 1), false)
    check("范围非法被拒绝（轮询 50ms）", store_a.set("JK_MODBUS_POLL_MS", 50), false)
    check("范围非法被拒绝（端口 70000）", store_a.set("MQTT_PORT", 70000), false)
    check("枚举非法被拒绝（JK_PROTOCOL）", store_a.set("JK_PROTOCOL", "xx"), false)
    check("类型非法被拒绝（MQTT_HOST 给数字）", store_a.set("MQTT_HOST", 123), false)
    check("超长字符串被拒绝（65 字符）", store_a.set("MQTT_HOST", string.rep("a", 65)), false)
    check("布尔 0/1 归一化（LOG_ENABLE=1 → true）",
        (store_a.set("LOG_ENABLE", 1) and cfg_mg.LOG_ENABLE), true)

    check("写 MGMT_BAUD = 9600 成功", store_a.set("MGMT_BAUD", 9600), true)
    check("写透：立即改 config", cfg_mg.MGMT_BAUD, 9600)
    check("写透：立即落盘（fskv 里是 JSON 文本）", type(kv12[store_a.key()]), "string")
    check("[R] 键置需重启", store_a.needs_restart(), true)
    check("从站地址概念已删除（MODBUS_SLAVE_ADDR 不在白名单）",
        store_a.set("MODBUS_SLAVE_ADDR", 240), false)

    -- 重启等价：新实例 load 后覆盖项在业务模块加载前生效
    cfg_mg.MGMT_BAUD, cfg_mg.LOG_ENABLE = 115200, false  -- 模拟未加载覆盖时的默认值
    local store_b = dofile(here .. "/../cfg_store.lua")
    check("load 应用 2 项覆盖", store_b.load(), 2)
    check("覆盖生效：波特率回到 9600", cfg_mg.MGMT_BAUD, 9600)
    check("覆盖生效：LOG_ENABLE 回到 true", cfg_mg.LOG_ENABLE, true)
    check("覆盖项如实记录", store_b.get("MGMT_BAUD"), 9600)

    -- 非法/未知项跳过（旧 MODBUS_* 覆盖项也按「不在白名单」跳过）
    kv12[store_a.key()] = _G.json.encode{
        MGMT_BAUD = 115200, MQTT_PORT = 99999, BOGUS_KEY = 1, MODBUS_SLAVE_ADDR = 240,
        JK_PROTOCOL = "modbus" }
    cfg_mg.MGMT_BAUD, cfg_mg.MQTT_PORT, cfg_mg.JK_PROTOCOL = 9600, 1883, "display"
    local store_c = dofile(here .. "/../cfg_store.lua")
    check("只应用合法项（越界/未知/旧 MODBUS_* 跳过）", store_c.load(), 2)
    check("合法项 1 生效", cfg_mg.MGMT_BAUD, 115200)
    check("合法项 2 生效", cfg_mg.JK_PROTOCOL, "modbus")
    check("越界项未生效（端口保持默认）", cfg_mg.MQTT_PORT, 1883)
    check("未知项不进覆盖", store_c.get("BOGUS_KEY"), nil)
    check("旧 MODBUS_SLAVE_ADDR 覆盖项被跳过", store_c.get("MODBUS_SLAVE_ADDR"), nil)

    check("恢复出厂", store_c.reset(), true)
    check("恢复出厂清空覆盖", store_c.has_override(), false)
    check("恢复出厂清掉 fskv 键", kv12[store_c.key()], nil)
    check("恢复出厂置待生效", store_c.factory_pending(), true)

    -- fskv 缺失：只告警 + 降级为「不持久化」
    _G.fskv = nil
    local store_d = dofile(here .. "/../cfg_store.lua")
    check("fskv 缺失：load 返回 0 不报错", store_d.load(), 0)
    check("fskv 缺失：写入仍改 config（本次运行有效）", store_d.set("LOG_ENABLE", true), true)
    check("fskv 缺失：覆盖记录仍在", store_d.get("LOG_ENABLE"), true)
    check("fskv 缺失：info 标明不可用", store_d.info().fskv_available, false)
    _G.fskv = fskv12

    for k, v in pairs(mg_saved) do
        cfg_mg[k] = v
    end
end

--=============================================================================
-- 13. fota_mgr：URL 通道（libfota2 打桩）+ 本地通道（核心库 fota 打桩）
--=============================================================================
io.write("== fota_mgr 状态机（libfota2 / 核心库 fota 打桩） ==\n")

local fota_cfg_saved = {
    FIN = cfg_mg.MGMT_FOTA_LOCAL_FINALIZE_MS,
    TO = cfg_mg.MGMT_FOTA_LOCAL_TIMEOUT_MS,
    ENABLE = cfg_mg.MGMT_FOTA_LOCAL_ENABLE,
    WAIT = cfg_mg.MGMT_FOTA_LOCAL_WAIT_MS,
}
cfg_mg.MGMT_FOTA_LOCAL_FINALIZE_MS = 300
cfg_mg.MGMT_FOTA_LOCAL_TIMEOUT_MS = 2000
cfg_mg.MGMT_FOTA_LOCAL_WAIT_MS = 0        -- 自测里跳过 fota.wait 轮询（保持"init 之后就绪"的确定性）

-- 核心库 fota 打桩：照真实 API 返回形状
--   fota.init() -> boolean；fota.run(buf) -> (ok, 收到最后一块, 未写 Flash 缓存字节)；
--   fota.isDone() -> (ok, 写到最后一块)；fota.finish(ok) -> boolean
local function make_core_fota(total)
    local core = { calls = {}, data = {}, end_ok = nil,
                   total = total or 0, got = 0, pending = 0 }
    function core.init() core.calls[#core.calls + 1] = "init"; return true end
    function core.wait() core.calls[#core.calls + 1] = "wait"; return false end
    function core.run(d)
        core.calls[#core.calls + 1] = "run"
        core.data[#core.data + 1] = d
        core.got = core.got + #d
        core.pending = core.pending + #d
        local isDone = (core.total > 0 and core.got >= core.total)
        if isDone then core.pending = 0 end
        return true, isDone, core.pending
    end
    function core.isDone() return true, true end
    function core.finish(ok) core.calls[#core.calls + 1] = "finish"; core.end_ok = ok; return true end
    return core
end

-- 13.0 标准 CRC32（IEEE，多项式 0xEDB88320，初值/结果取反）
do
    local fm = dofile(here .. "/../fota_mgr.lua")
    check("CRC32('') = 0", fm.crc32(""), 0)
    check("CRC32('123456789') = 0xCBF43926", fm.crc32("123456789"), (0xCBF4 << 16) | 0x3926)
    check("CRC32('a') = 0xE8B7BE43", fm.crc32("a"), (0xE8B7 << 16) | 0xBE43)
    check("CRC32('The quick brown fox jumps over the lazy dog') = 0x414FA339",
        fm.crc32("The quick brown fox jumps over the lazy dog"), (0x414F << 16) | 0xA339)
end

-- 13.1 URL 通道
do
    local fota_calls, fota_cb, fota_opts = 0, nil, nil
    package.preload["libfota2"] = function()
        return {
            request = function(cb, opts) fota_calls = fota_calls + 1; fota_cb, fota_opts = cb, opts; return true end,
            setTimeout = function() end,
        }
    end
    store_m.set_needs_restart(false)
    scheduler.clear("net_ready")
    local fota_m = dofile(here .. "/../fota_mgr.lua")
    check("空 URL 拒绝", select(2, fota_m.start("   ")), "empty url")
    check("启动成功", fota_m.start("http://ota/x.binpkg"), true)
    check("下载中再次启动 → busy", select(2, fota_m.start("http://ota/y.binpkg")), "busy")
    check("未联网不发起下载", fota_calls, 0)
    check("状态 = 1 下载中", fota_m.info().state, 1)
    check("is_busy 为真", fota_m.is_busy(), true)
    check("URL 通道来源 = 1", fota_m.info().mode, 1)

    scheduler.publish("net_ready", "dev")
    check("联网后才调用 libfota2.request", fota_calls, 1)
    check("URL 透传给库", fota_opts and fota_opts.url, "http://ota/x.binpkg")
    fota_opts.progress(30, 100)
    check("进度回调 → 30%", fota_m.info().progress, 30)
    fota_cb(0)
    check("成功后状态 = 2（待重启）", fota_m.info().state, 2)
    check("成功后进度 = 100", fota_m.info().progress, 100)
    check("成功后置「需重启」", store_m.needs_restart(), true)
    check("is_busy 回到 false", fota_m.is_busy(), false)

    local fota_m2 = dofile(here .. "/../fota_mgr.lua")
    fota_calls, fota_cb = 0, nil
    check("第二次启动成功", fota_m2.start("http://ota/z.binpkg"), true)
    scheduler.publish("net_ready", "dev")
    check("联网后发起下载", fota_calls, 1)
    fota_cb(7)
    check("失败状态 = 3", fota_m2.info().state, 3)
    check("失败结果码 = 7", fota_m2.info().result, 7)

    package.preload["libfota2"] = function() error("module 'libfota2' not found") end
    package.loaded["libfota2"] = nil
    local fota_m3 = dofile(here .. "/../fota_mgr.lua")
    check("库缺失：返回 lib missing", select(2, fota_m3.start("http://x/y")), "lib missing")
    check("库缺失：状态 = 3 失败", fota_m3.info().state, 3)
    check("库缺失：结果码 = -1", fota_m3.info().result, -1)
    package.preload["libfota2"] = nil
    package.loaded["libfota2"] = nil
end

-- 13.2 本地通道完整流程
do
    local payload = "HELLO-LUATOS-BIN-PAYLOAD"
    local core = make_core_fota(#payload)
    _G.fota = core
    store_m.set_needs_restart(false)
    local fm = dofile(here .. "/../fota_mgr.lua")
    check("本地升级受理（total/crc/flags 合法）", fm.local_begin(#payload, fm.crc32(payload), 0), true)
    check("受理后阶段 = preparing(1)", fm.info().stage, 1)
    check("来源 = 本地(2)", fm.info().mode, 2)
    check("统一状态 = 进行中(1)", fm.info().state, 1)
    check("本地通道 is_local_busy", fm.is_local_busy(), true)
    check("URL 通道空闲", fm.is_busy(), false)
    check("声明的总长写进 info", fm.info().total, #payload)

    pump()
    check("init 后就绪（自测 WAIT=0：跳过 wait 轮询）", table.concat(core.calls, ","), "init")
    check("阶段 = ready(2)", fm.info().stage, 2)

    check("第 1 块写入", fm.local_write(1, 0, payload:sub(1, 8)), true)
    check("阶段 = receiving(3)", fm.info().stage, 3)
    check("已收 8 字节", fm.info().rx, 8)
    check("进度 33%", fm.info().progress, 33)
    check("最后成功块序号 = 1", fm.info().last_seq, 1)
    check("未落盘缓存上报到 info().cache", fm.info().cache, 8)
    check("第 2 块（偏移 8）", fm.local_write(2, 8, payload:sub(9, 16)), true)
    check("收齐前核心库不说 isDone", fm.info().stage, 3)
    check("第 3 块（收齐 → 排期收尾）", fm.local_write(3, 16, payload:sub(17, 24)), true)
    check("阶段 = verifying(4)", fm.info().stage, 4)
    pump()
    check("成功：核心库收到 finish(true)", core.end_ok, true)
    check("阶段 = success(5)", fm.info().stage, 5)
    check("进度 = 100", fm.info().progress, 100)
    check("统一状态 = 成功待重启(2)", fm.info().state, 2)
    check("成功置「需重启」", store_m.needs_restart(), true)
    check("写入核心库的字节 = 整包", table.concat(core.data), payload)
    check("调用序列 = init,run×3,finish（无 wait）", table.concat(core.calls, ","), "init,run,run,run,finish")
    check("收齐后未落盘缓存归 0", fm.info().cache, 0)
    check("成功后不再忙", fm.is_local_busy(), false)
    check("成功后重发最后一块 → 幂等受理", fm.local_write(3, 16, payload:sub(17, 24)), true)
    check("幂等重发后仍是 success 且不重复写 Flash", table.concat(core.data), payload)
    check("成功后继续写块 → state 错", select(2, fm.local_write(4, 24, "x")), "state")
    check("本地升级全程不碰 libfota2", package.loaded["libfota2"], nil)
    check("成功后可以再开一轮（重新开始）", fm.local_begin(4, 0, 0), true)
end

-- 13.2b 核心库 fota 的「暴露形态」：LuatOS 的 C 库都是 rotable（userdata + 元表，
--       元表 __name="rotable"，见 lua/src/rotable2.c 的 rotable2_newlib），**不是 Lua table**。
--       探测若写成 `type(fota) == "table"`，真机上就会「有库却报缺库」：
--       现场现象 = status.get 的 caps.core=false + fota.begin 回 ENOLIB（本用例就是它的回归）。
do
    local fm = dofile(here .. "/../fota_mgr.lua")

    -- 形态 1：固件真没这个库（nil）→ 判不可用，local_begin 明确回 "lib missing"
    _G.fota = nil
    check("核心库 nil → core=false", fm.info().core, false)
    check("核心库 nil → core_type='nil'（type() 的字符串）", fm.info().core_type, "nil")
    check("核心库 nil → local_begin 回 lib missing", select(2, fm.local_begin(4, 0, 0)), "lib missing")

    -- 形态 2：真机形态 —— userdata + 元表（PC 端拿文件句柄这个 userdata 来模拟）。
    --   真机 rotable 的 __index 是**函数**（C 的 rotable_udata_index），这里同样用函数模拟，
    --   这样 type(t) == "userdata"、成员只能从 __index 取，与真机一致。
    local ud = io.tmpfile and io.tmpfile() or nil
    local mt = ud and getmetatable(ud)
    if type(mt) == "table" then
        local saved_index = rawget(mt, "__index")
        local calls = {}
        local methods = {
            init   = function() calls[#calls + 1] = "init"; return true end,
            wait   = function() return true end,
            run    = function(d) calls[#calls + 1] = "run:" .. tostring(#d); return true, false, 0 end,
            isDone = function() return true, true end,
            finish = function(_) calls[#calls + 1] = "finish"; return true end,
        }
        rawset(mt, "__index", function(_, k) return methods[k] end)

        _G.fota = ud
        check("rotable 形态：type(fota) 不是 table", type(ud) == "table", false)
        check("rotable 形态：core=true（修复点）", fm.info().core, true)
        check("rotable 形态：core_type=userdata", fm.info().core_type, "userdata")
        check("rotable 形态：local_begin 受理", fm.local_begin(4, 0, 0), true)
        pump()
        check("rotable 形态：调用链已进核心库（init）", table.concat(calls, ","), "init")

        -- 文件元表是所有文件句柄共享的：用完立刻还原
        rawset(mt, "__index", saved_index)
    else
        io.write("  note 跳过「rotable(userdata) 形态」用例：当前 Lua 取不到文件句柄元表\n")
    end

    -- 形态 3：打桩用的 table 形态照样认；非表非 userdata 的怪值只判不可用，不抛异常
    _G.fota = { init = function() return true end }
    check("table 形态仍识别为可用", fm.info().core, true)
    _G.fota = 0
    check("怪值（number）→ core=false 且不抛异常", fm.info().core, false)
    _G.fota = nil
end

-- 13.3 幂等重试与顺序/偏移/长度校验
do
    local core = make_core_fota(10)
    _G.fota = core
    local fm = dofile(here .. "/../fota_mgr.lua")
    check("开始（总长 10）", fm.local_begin(10, 0, 0), true)
    pump()
    check("第 1 块 5 字节", fm.local_write(1, 0, "12345"), true)
    local runs = #core.data
    check("同一块重发 → 幂等成功", fm.local_write(1, 0, "12345"), true)
    check("重发不重复写 Flash", #core.data, runs)
    check("重发不推进进度", fm.info().rx, 5)
    check("SEQ 跳号 → seq 错", select(2, fm.local_write(3, 5, "xy")), "seq")
    check("偏移不符 → offset 错", select(2, fm.local_write(2, 6, "xy")), "offset")
    check("单块超 238 字节 → length 错", select(2, fm.local_write(2, 5, string.rep("a", 239))), "length")
    check("超出声明总长 → length 错", select(2, fm.local_write(2, 5, string.rep("a", 6))), "length")
    check("未开始（stage=idle）时写块 → state 错", select(2, (function()
        local fm2 = dofile(here .. "/../fota_mgr.lua")
        return fm2.local_write(1, 0, "x")
    end)()), "state")
end

-- 13.4 核心库 init 失败 / 抛异常
do
    local core = make_core_fota(100)
    core.init = function() core.calls[#core.calls + 1] = "init"; return false end
    _G.fota = core
    local fm = dofile(here .. "/../fota_mgr.lua")
    check("开始受理", fm.local_begin(100, 0, 0), true)
    pump()
    check("init 返回 false → failed(6)", fm.info().stage, 6)
    check("失败码 -10", fm.info().result, -10)
    check("统一状态 = 失败(3)", fm.info().state, 3)
    check("失败时通知核心库 finish(false)", core.end_ok, false)
    check("失败后不忙", fm.is_local_busy(), false)

    local core2 = make_core_fota(100)
    core2.init = function() error("boom") end
    _G.fota = core2
    local fm2 = dofile(here .. "/../fota_mgr.lua")
    fm2.local_begin(100, 0, 0)
    pump()
    check("init 抛异常 → failed(6)", fm2.info().stage, 6)
    check("init 抛异常失败码 -10", fm2.info().result, -10)
end

-- 13.5 legacy fota.wait 的存在与返回值都不影响流程
do
    local core = make_core_fota(10)
    core.wait = function() core.calls[#core.calls + 1] = "wait"; return false end
    _G.fota = core
    local fm = dofile(here .. "/../fota_mgr.lua")
    fm.local_begin(100, 0, 0)
    pump()
    check("wait() 返回 false 不影响：阶段仍是 ready(2)", fm.info().stage, 2)
    check("（自测 WAIT=0）未调用 wait()", table.concat(core.calls, ","), "init")
end

-- 13.5b fota.init 后轮询 fota.wait() 等底层就绪（官方 demo 的做法）
do
    local saved_wait = cfg_mg.MGMT_FOTA_LOCAL_WAIT_MS
    cfg_mg.MGMT_FOTA_LOCAL_WAIT_MS = 1000
    local core = make_core_fota(10)
    local n = 0
    core.wait = function() n = n + 1; core.calls[#core.calls + 1] = "wait"; return true end
    _G.fota = core
    local fm = dofile(here .. "/../fota_mgr.lua")
    fm.local_begin(10, 0, 0)
    pump()
    check("fota.wait 返回 true → 立刻 ready(2)", fm.info().stage, 2)
    check("确实问了 fota.wait（init 之后）",
        (core.calls[1] or "") .. "/" .. (core.calls[2] or ""), "init/wait")
    cfg_mg.MGMT_FOTA_LOCAL_WAIT_MS = saved_wait
end

-- 13.5c fota.wait 一直 false → 到上限后仍进 ready（不把升级卡死）
do
    local saved_wait = cfg_mg.MGMT_FOTA_LOCAL_WAIT_MS
    cfg_mg.MGMT_FOTA_LOCAL_WAIT_MS = 300
    local core = make_core_fota(10)
    core.wait = function() core.calls[#core.calls + 1] = "wait"; return false end
    _G.fota = core
    local fm = dofile(here .. "/../fota_mgr.lua")
    fm.local_begin(10, 0, 0)
    pump()
    check("wait 未就绪时不急着 ready", fm.info().stage, 1)
    for _ = 1, 8 do scheduler.wake(); pump() end
    check("wait 超时后仍进 ready(2)（不卡死）", fm.info().stage, 2)
    cfg_mg.MGMT_FOTA_LOCAL_WAIT_MS = saved_wait
end

-- 13.6 核心库 run 失败（写 Flash 失败）
do
    local core = make_core_fota(10)
    core.run = function() core.calls[#core.calls + 1] = "run"; return false end
    _G.fota = core
    local fm = dofile(here .. "/../fota_mgr.lua")
    fm.local_begin(10, 0, 0)
    pump()
    check("run 失败 → 返回 run 原因", select(2, fm.local_write(1, 0, "12345")), "run")
    check("run 失败 → failed(6)", fm.info().stage, 6)
    check("run 失败码 -21", fm.info().result, -21)
end

-- 13.7 收尾时 isDone 超时 / 报错
do
    local core = make_core_fota(4)
    core.isDone = function() return true, false end
    _G.fota = core
    local fm = dofile(here .. "/../fota_mgr.lua")
    fm.local_begin(4, 0, 0)
    pump()
    fm.local_write(1, 0, "abcd")
    check("收齐后阶段 = verifying(4)", fm.info().stage, 4)
    for _ = 1, 6 do
        scheduler.wake(); pump()
    end
    check("isDone 超时 → failed(6)", fm.info().stage, 6)
    check("isDone 超时失败码 -30", fm.info().result, -30)
    check("超时后 finish(false)", core.end_ok, false)

    local core2 = make_core_fota(4)
    core2.isDone = function() return false, true end
    _G.fota = core2
    local fm2 = dofile(here .. "/../fota_mgr.lua")
    fm2.local_begin(4, 0, 0)
    pump()
    fm2.local_write(1, 0, "abcd")
    pump()
    check("isDone 报错 → failed(6)", fm2.info().stage, 6)
    check("isDone 报错失败码 -30", fm2.info().result, -30)
end

-- 13.8 收尾时 CRC32 不匹配
do
    local core = make_core_fota(4)
    _G.fota = core
    local fm = dofile(here .. "/../fota_mgr.lua")
    local data = "abcd"
    fm.local_begin(#data, fm.crc32(data) ~ 1, 0)
    pump()
    fm.local_write(1, 0, data)
    pump()
    check("CRC32 不匹配 → failed(6)", fm.info().stage, 6)
    check("CRC 失败码 -31", fm.info().result, -31)
    check("CRC 失败不调 finish(true)", core.end_ok, false)
end

-- 13.9 主动中止
do
    local core = make_core_fota(100)
    _G.fota = core
    local fm = dofile(here .. "/../fota_mgr.lua")
    check("空闲时中止返回 idle", select(2, fm.local_abort("none")), "idle")
    fm.local_begin(100, 0, 0)
    pump()
    check("中止本地升级", fm.local_abort("master"), true)
    check("中止 → failed(6)", fm.info().stage, 6)
    check("中止失败码 -40", fm.info().result, -40)
    check("中止通知核心库 finish(false)", core.end_ok, false)
    check("中止后不忙", fm.is_local_busy(), false)
end

-- 13.10 空闲超时（2s 阈值没有新块 → 自动 finish(false)）
do
    local core = make_core_fota(100)
    _G.fota = core
    local fm = dofile(here .. "/../fota_mgr.lua")
    local t = 1000
    fm.set_clock(function() return t end)
    fm.local_begin(100, 0, 0)
    pump()
    check("进入 ready", fm.info().stage, 2)
    t = t + 1000
    scheduler.wake(); pump()
    check("未超时（2s 阈值）仍是 ready", fm.info().stage, 2)
    t = t + 3000
    scheduler.wake(); pump()
    check("空闲超时 → failed(6)", fm.info().stage, 6)
    check("空闲超时失败码 -12", fm.info().result, -12)
    check("空闲超时 finish(false)", core.end_ok, false)
end

-- 13.11 核心库缺失：本地通道优雅降级
do
    _G.fota = nil
    local fm = dofile(here .. "/../fota_mgr.lua")
    check("核心库缺失：local_begin 返回 lib missing", select(2, fm.local_begin(10, 0, 0)), "lib missing")
    check("核心库缺失：阶段 failed(6)", fm.info().stage, 6)
    check("核心库缺失：失败码 -1", fm.info().result, -1)
    check("核心库缺失：info().core = false", fm.info().core, false)
    check("核心库缺失：不忙", fm.is_local_busy(), false)
end

-- 13.12 URL 通道与本地通道互斥
do
    local core = make_core_fota(10)
    _G.fota = core
    local fm = dofile(here .. "/../fota_mgr.lua")
    fm.local_begin(10, 0, 0)
    pump()
    check("本地在忙 → URL 启动 busy", select(2, fm.start("http://ota/x")), "busy")
    fm.local_abort("test")

    package.preload["libfota2"] = function()
        return { request = function() return true end, setTimeout = function() end }
    end
    package.loaded["libfota2"] = nil
    scheduler.clear("net_ready")
    local fm2 = dofile(here .. "/../fota_mgr.lua")
    check("URL 启动", fm2.start("http://ota/y"), true)
    check("URL 在忙 → 本地启动 busy", select(2, fm2.local_begin(10, 0, 0)), "busy")
    scheduler.publish("net_ready", "dev")
    pump()
    package.preload["libfota2"] = nil
    package.loaded["libfota2"] = nil
    _G.fota = nil
end

do
    cfg_mg.MGMT_FOTA_LOCAL_FINALIZE_MS = fota_cfg_saved.FIN
    cfg_mg.MGMT_FOTA_LOCAL_TIMEOUT_MS = fota_cfg_saved.TO
    cfg_mg.MGMT_FOTA_LOCAL_ENABLE = fota_cfg_saved.ENABLE
    cfg_mg.MGMT_FOTA_LOCAL_WAIT_MS = fota_cfg_saved.WAIT
end

-- 13.13 端到端（mgmt 方法 → fota_mgr → 核心库 fota）：最大块 + 奇数块走一遍
do
    local payload = string.rep("A", 238) .. "xyz"        -- 241 字节：最大块 + 奇数块
    local core = make_core_fota(#payload)
    _G.fota = core
    cfg_mg.MGMT_FOTA_LOCAL_WAIT_MS = 0    -- 同上：跳过 wait 轮询，保持端到端确定性
    local fm = dofile(here .. "/../fota_mgr.lua")
    store_m.set_needs_restart(false)
    local inst_e = mgmt_m.new{
        cfg = cfg_mg, store = store_m, fota = fm, board = nil, no_watch = true,
        now_ms = function() return 1 end, post = function() end }

    local r0 = inst_e:call("fota.begin", { total = #payload, crc32 = fm.crc32(payload) })
    check("端到端：fota.begin 受理", r0.ok, true)
    pump()
    check("端到端：阶段 ready(2)", inst_e:call("fota.status").result.stage, 2)
    check("端到端：来源 = 本地(2)", inst_e:call("fota.status").result.mode, 2)
    check("端到端：已收 0 字节", inst_e:call("fota.status").result.rx, 0)
    check("端到端：caps.core=true（核心库在场）", inst_e:call("fota.status").result.caps.core, true)
    check("端到端：caps.core_type 透传（本用例是 table 形态）",
        inst_e:call("fota.status").result.caps.core_type, "table")

    check("端到端：第 1 块（最大块 238 字节）受理",
        inst_e:fota_blob(1, payload:sub(1, 238)).ok, true)
    check("端到端：进度 = 98%（238/241）", inst_e:call("fota.status").result.progress, 98)
    check("端到端：最后块序号 = 1", inst_e:call("fota.status").result.last_seq, 1)
    check("端到端：第 2 块（奇数块 3 字节）受理",
        inst_e:fota_blob(2, payload:sub(239)).ok, true)
    pump()
    check("端到端：阶段 success(5)", inst_e:call("fota.status").result.stage, 5)
    check("端到端：统一 FOTA 状态 = 2（成功待重启）", inst_e:call("fota.status").result.state, 2)
    check("端到端：进度 100%", inst_e:call("fota.status").result.progress, 100)
    check("端到端：已收字节 = 241", inst_e:call("fota.status").result.rx, 241)
    check("端到端：置「需重启」", store_m.needs_restart(), true)
    check("端到端：写入核心库的字节 = 整包", table.concat(core.data), payload)
    check("端到端：CRC32 校验通过（finish(true)）", core.end_ok, true)
    cfg_mg.MGMT_FOTA_LOCAL_WAIT_MS = fota_cfg_saved.WAIT
end

-- 13.14 位宽守卫：新增模块里不允许出现 32 位整数 Lua 装不下的整数字面量
do
    for _, name in ipairs({ "fota_mgr.lua", "mgmt_mqtt.lua" }) do
        local bad = {}
        local fh = io.open(here .. "/../" .. name, "r")
        if fh then
            local src = fh:read("a")
            fh:close()
            src = src:gsub("%-%-%[%[.-%]%]", "")
            src = src:gsub("%-%-[^\r\n]*", "")
            for lit in src:gmatch("0[xX](%x+)") do
                local v = tonumber(lit, 16)
                if #lit > 8 or (v and v > 0x7FFFFFFF) then
                    bad[#bad + 1] = "0x" .. lit
                end
            end
        end
        check(name .. " 源码没有 32 位整数装不下的整数字面量", #bad, 0)
        if #bad > 0 then
            io.write("       ", name, ": ", table.concat(bad, ", "), "\n")
        end
    end
end

--=============================================================================
-- 14. mgmt_uart：UART1 裸帧 RPC 传输（串口参数 / 回调只搬字节 / 任务应答 /
--     重同步计数 / BLOB / 突发帧预算 / 事件 / 异常安全）
--=============================================================================
io.write("== mgmt_uart（UART1 裸帧 RPC 传输） ==\n")

do
    local UART_MGMT = 1
    local u_saved = { setup = _G.uart.setup, on = _G.uart.on,
                      read = _G.uart.read, write = _G.uart.write }
    local pins_saved = _G.pins
    local handlers, written, rxq, mux_seq, u_setup = {}, {}, "", {}, nil
    _G.pins = {
        setup = function(pin, fn)
            mux_seq[#mux_seq + 1] = string.format("%d:%s", pin, fn)
            return true
        end,
    }
    _G.uart.setup = function(id, baud, db, sb, par, endian, buf, dir, rxlevel, delay)
        if id ~= UART_MGMT then return false end
        u_setup = { baud = baud, db = db, sb = sb, par = par, buf = buf, dir = dir,
                    rxlevel = rxlevel, delay = delay }
        return true
    end
    _G.uart.on = function(id, _evt, cb) handlers[id] = cb end
    _G.uart.read = function() local s = rxq; rxq = ""; return s end
    _G.uart.write = function(id, d) written[#written + 1] = { id = id, data = d } end

    local cfg_14 = require "config"
    local saved_echo = cfg_14.MGMT_ECHO_TEST
    cfg_14.MGMT_ECHO_TEST = false
    local mu = dofile(here .. "/../mgmt_uart.lua")
    local t14 = mu.new{ mgmt = inst }        -- inst：第 11 节那个打了快照桩的 mgmt 实例
    t14:start()
    pump()
    check("管理链路 UART1 默认复用：不做 pins.setup", #mux_seq, 0)
    check("波特率 = 115200", u_setup and u_setup.baud, 115200)
    check("8 数据位 1 停止位 无校验",
        u_setup and (u_setup.db .. "/" .. u_setup.sb .. "/" .. u_setup.par), "8/1/0")
    check("接收缓冲 = 配置值", u_setup and u_setup.buf, cfg_14.MGMT_RX_BUFF_SIZE)
    check("方向脚 = GPIO25（RS485-1 EN）", u_setup and u_setup.dir, cfg_14.MGMT_RS485_DIR)
    check("接收回调已注册", handlers[UART_MGMT] ~= nil, true)

    -- evt.hello：串口打开后约 1s
    written = {}
    scheduler.wake(); pump()
    check("打开后发 1 帧事件", #written, 1)
    local evt = written[1] and written[1].data or ""
    check("事件是 EVT 帧且 ID=0",
        (string.byte(evt, 4) or 0) .. "/" .. (string.byte(evt, 5) or 0), mu.TYPE_EVT .. "/0")
    local eobj = json.decode(evt:sub(8, #evt - 2))
    check("evt.hello：evt=hello", eobj and eobj.evt, "hello")
    check("evt.hello：data.proto = 1", eobj and eobj.data.proto, 1)
    check("evt.hello 计入 evt 统计", t14:get_stats().evt, 1)

    -- REQ：回调只搬字节，应答在任务里
    written = {}
    local req = mu.build(mu.TYPE_REQ, 3, '{"method":"sys.ping"}')
    rxq = req
    handlers[UART_MGMT](UART_MGMT)
    check("回调本身不应答", #written, 0)
    check("回调把原始字节计入 rx_bytes", t14:get_stats().rx_bytes, #req)
    pump()
    check("解析任务写出应答", #written, 1)
    check("应答写回 UART1", written[1] and written[1].id, UART_MGMT)
    local resp = written[1] and written[1].data or ""
    check("应答是 RESP 帧且 ID 原样回带",
        (string.byte(resp, 4) or 0) .. "/" .. (string.byte(resp, 5) or 0), mu.TYPE_RESP .. "/3")
    local robj = json.decode(resp:sub(8, #resp - 2))
    check("应答 ok=true 且 uptime_s=70000", robj and robj.ok and robj.result.uptime_s, 70000)
    check("统计：frames/replies 各 1",
        t14:get_stats().frames .. "/" .. t14:get_stats().replies, "1/1")

    -- 重同步：前导垃圾 + 好帧 → 计入 sync 且仍应答
    written = {}
    local sync_before = t14:get_stats().sync
    rxq = "junk" .. mu.build(mu.TYPE_REQ, 4, '{"method":"status.get"}')
    handlers[UART_MGMT](UART_MGMT)
    pump()
    check("前导垃圾：仍应答", #written, 1)
    check("前导垃圾：计入 sync", t14:get_stats().sync, sync_before + 4)

    -- CRC 错：不应答并计入 crc
    written = {}
    local crc_before = t14:get_stats().crc
    local badframe = req:sub(1, #req - 1) .. string.char((string.byte(req, #req) ~ 0xFF) & 0xFF)
    rxq = badframe
    handlers[UART_MGMT](UART_MGMT)
    pump()
    check("CRC 错：不应答", #written, 0)
    check("CRC 错：计入 crc", t14:get_stats().crc, crc_before + 1)

    -- 未知方法：仍应答（错误信封）
    written = {}
    rxq = mu.build(mu.TYPE_REQ, 5, '{"method":"nope"}')
    handlers[UART_MGMT](UART_MGMT)
    pump()
    local eo = json.decode(written[1].data:sub(8, #written[1].data - 2))
    check("未知方法：应答 EBADMETHOD", eo and eo.error and eo.error.code, "EBADMETHOD")

    -- BLOB：FOTA 数据块（偏移由设备推得）
    written = {}
    fake_fota.rx = 0
    rxq = mu.build(mu.TYPE_BLOB, 6, string.char(1, 0, 4, 0) .. "WXYZ")
    handlers[UART_MGMT](UART_MGMT)
    pump()
    check("BLOB：应答 1 帧", #written, 1)
    check("BLOB：写入数据块 seq/data",
        fake_fota.write[#fake_fota.write].seq .. "/" .. fake_fota.write[#fake_fota.write].data,
        "1/WXYZ")

    -- BLOB：LEN 与实际数据不符 → EBADFRAME
    written = {}
    rxq = mu.build(mu.TYPE_BLOB, 7, string.char(2, 0, 9, 0) .. "AB")
    handlers[UART_MGMT](UART_MGMT)
    pump()
    local bo = json.decode(written[1].data:sub(8, #written[1].data - 2))
    check("BLOB：LEN 与实际数据不符 → EBADFRAME", bo and bo.error and bo.error.code, "EBADFRAME")

    -- 突发帧流按预算分批处理
    written = {}
    rxq = string.rep(req, 12)
    handlers[UART_MGMT](UART_MGMT)
    pump()
    check("突发帧先处理一批（预算 8）", #written, 8)
    scheduler.wake(); pump()
    check("让出后处理完剩余帧", #written, 12)

    -- 离线入口
    local off_resp = t14:handle_frame(mu.build(mu.TYPE_REQ, 9, '{"method":"sys.ping"}'))
    check("handle_frame 离线应答 ID 原样", off_resp and string.byte(off_resp, 5), 9)

    -- 事件队列：push_evt → 传输写回 EVT
    written = {}
    t14:push_evt("fota", { mode = 2, stage = 5 })
    scheduler.wake(); pump()
    local found = false
    for _, w in ipairs(written) do
        local o = json.decode(w.data:sub(8, #w.data - 2))
        if o and o.evt == "fota" then found = true end
    end
    check("MGMT_EVT → 写回 evt.fota", found, true)

    t14:stop()

    -- mgmt 抛异常：只丢这一帧，任务不崩，下一帧照常
    local bang = 0
    local boom = { handle = function(_, _payload, _id)
                       bang = bang + 1
                       if bang == 1 then error("模拟 mgmt 异常") end
                       return { ok = true, result = { pong = true } }
                   end,
                   fota_blob = function() error("boom") end,
                   hello = function() return {} end }
    local tboom = mu.new{ mgmt = boom }
    tboom:start()
    pump()
    written = {}
    rxq = mu.build(mu.TYPE_REQ, 1, '{"method":"sys.ping"}')
    handlers[UART_MGMT](UART_MGMT); pump()
    check("mgmt 抛异常时该帧不应答（任务不崩）", #written, 0)
    check("异常计入 errors", tboom:get_stats().errors, 1)
    written = {}
    rxq = mu.build(mu.TYPE_REQ, 2, '{"method":"sys.ping"}')
    handlers[UART_MGMT](UART_MGMT); pump()
    check("下一帧照常处理（任务没被打死）", #written, 1)
    tboom:stop()

    cfg_14.MGMT_ECHO_TEST = saved_echo
    _G.uart.setup, _G.uart.on, _G.uart.read, _G.uart.write =
        u_saved.setup, u_saved.on, u_saved.read, u_saved.write
    _G.pins = pins_saved
end

--=============================================================================
-- 15. mgmt_mqtt：管理 JSON-RPC（通道 = 主题后缀 rpc），与上报共用同一条 MQTT 连接
--=============================================================================
io.write("== mgmt_mqtt（管理 JSON-RPC，通道 rpc） ==\n")

do
    local cfg_mq = require "config"
    local saved_enable, saved_suffix = cfg_mq.MGMT_MQTT_ENABLE, cfg_mq.MGMT_MQTT_SUFFIX

    local function stub_link(ready)
        local L = { subs = {}, pubs = {}, ready = (ready ~= false) }
        L.subscribe = function(topic, fn) L.subs[#L.subs + 1] = { topic = topic, fn = fn }; return true end
        L.publish = function(topic, payload, qos)
            if not L.ready then return false end
            L.pubs[#L.pubs + 1] = { topic = topic, payload = payload, qos = qos }
            return true
        end
        L.device_id = function() return "dev1" end
        L.is_enabled = function() return true end
        L.is_mqtt_ready = function() return L.ready end
        return L
    end

    -- 15.1 未启用：不订阅（通道主题仍可算出来）
    cfg_mq.MGMT_MQTT_ENABLE = false
    local mq_off = dofile(here .. "/../mgmt_mqtt.lua")
    check("未启用（MGMT_MQTT_ENABLE = false）", mq_off.is_enabled(), false)
    local off_link = stub_link()
    local off_t = mq_off.new{ mgmt = inst, link = off_link }
    check("未启用时 open 返回 false/disabled", select(2, off_t:open()), "disabled")
    check("未启用时不订阅任何主题", #off_link.subs, 0)
    check("未启用时通道主题仍可计算", (select(1, mq_off.topics("dev1"))), TOPIC_PREFIX .. "/dev1/rpc")

    -- 15.2 通道 = 主题后缀（默认 rpc；可配、空值回退 rpc）
    cfg_mq.MGMT_MQTT_ENABLE = true
    cfg_mq.MGMT_MQTT_SUFFIX = "rpc"
    local mq = dofile(here .. "/../mgmt_mqtt.lua")
    check("启用后 is_enabled = true", mq.is_enabled(), true)
    check("通道 = rpc：<prefix>/<device>/rpc 与 .../rpc/resp",
        table.concat({ mq.topics("dev1") }, "|"),
        TOPIC_PREFIX .. "/dev1/rpc|" .. TOPIC_PREFIX .. "/dev1/rpc/resp")
    cfg_mq.MGMT_MQTT_SUFFIX = "/mb/"
    local mq_mb = dofile(here .. "/../mgmt_mqtt.lua")
    check("后缀可配（去掉多余斜杠）", (select(1, mq_mb.topics("dev1"))), TOPIC_PREFIX .. "/dev1/mb")
    cfg_mq.MGMT_MQTT_SUFFIX = ""
    local mq_empty = dofile(here .. "/../mgmt_mqtt.lua")
    check("后缀为空回退 rpc", mq_empty.suffix(), "rpc")
    cfg_mq.MGMT_MQTT_SUFFIX = "rpc"

    -- 15.3 open：登记下行主题订阅；重复 open 幂等
    local link = stub_link()
    local t = mq.new{ mgmt = inst, link = link }
    check("open 成功", t:open(), true)
    check("订阅下行主题（通道 rpc）", link.subs[1] and link.subs[1].topic, TOPIC_PREFIX .. "/dev1/rpc")
    check("opened 置位", t:get_stats().opened, true)
    check("上行主题 = 订阅主题 + /resp", (select(2, t:topics())), TOPIC_PREFIX .. "/dev1/rpc/resp")
    check("重复 open 幂等（不重复登记回调）", t:open() and #link.subs, 1)

    -- 15.4 一条 JSON-RPC：响应带 id、发到上行主题
    local sub_topic = TOPIC_PREFIX .. "/dev1/rpc"
    link.subs[1].fn(sub_topic, '{"id":7,"method":"sys.ping"}')
    check("回发 1 条响应", #link.pubs, 1)
    check("响应主题 = .../rpc/resp", link.pubs[1].topic, TOPIC_PREFIX .. "/dev1/rpc/resp")
    check("响应 QoS = MQTT_QOS", link.pubs[1].qos, cfg_mq.MQTT_QOS)
    local ro = json.decode(link.pubs[1].payload)
    check("事务号 id 原样回带", ro.id, 7)
    check("响应 ok + uptime_s", ro.ok and ro.result.uptime_s, 70000)
    check("统计：1 载荷 / 1 回发", t:get_stats().rx .. "/" .. t:get_stats().reply, "1/1")

    -- 15.5 无 id：响应不带 id（仍回发）
    t:on_message('{"method":"status.get"}')
    check("无 id 也回发", #link.pubs, 2)
    check("无 id 响应不带 id 字段", json.decode(link.pubs[2].payload).id, nil)

    -- 15.6 未知方法 → 错误信封（回发）
    t:on_message('{"id":8,"method":"no.such"}')
    local eo = json.decode(link.pubs[3].payload)
    check("未知方法回发 EBADMETHOD", eo.id .. "/" .. eo.error.code, "8/EBADMETHOD")

    -- 15.7 坏载荷只丢该条（非 JSON / 空），不影响后续
    local pubs_now, bad_now = #link.pubs, t:get_stats().bad
    check("非 JSON 不抛异常", pcall(t.on_message, t, "{not json"), true)
    check("非 JSON 不回发", #link.pubs, pubs_now)
    check("非 JSON 计入 bad", t:get_stats().bad, bad_now + 1)
    t:on_message("")
    check("空载荷计入 bad", t:get_stats().bad, bad_now + 2)
    t:on_message('{"id":9,"method":"sys.ping"}')
    check("坏载荷后合法请求照常", #link.pubs, pubs_now + 1)

    -- 15.8 base64 fota.chunk（MQTT 通道；偏移由设备按已收字节推得）
    fake_fota.rx = 100
    local before = #fake_fota.write
    t:on_message('{"id":10,"method":"fota.chunk","params":{"seq":5,"data":"YWJjZA=="}}')
    check("fota.chunk 回发响应", #link.pubs, pubs_now + 2)
    check("fota.chunk：偏移自动推得 + base64 解码",
        fake_fota.write[before + 1].offset .. "/" .. fake_fota.write[before + 1].data, "100/abcd")
    local co = json.decode(link.pubs[#link.pubs].payload)
    check("fota.chunk 响应含 id 与 rx", co.id .. "/" .. co.result.rx, "10/104")

    -- 15.9 MQTT 未就绪：响应丢弃并计数
    local down_link = stub_link(false)
    local tdown = mq.new{ mgmt = inst, link = down_link }
    tdown:open()
    tdown:on_message('{"id":1,"method":"sys.ping"}')
    check("MQTT 未就绪：响应丢弃并计数", tdown:get_stats().dropped, 1)
    check("MQTT 未就绪：不计入 reply", tdown:get_stats().reply, 0)

    -- 15.10 无链路 / 取不到设备 ID：open 明确返回原因
    local t_nolink = mq.new{ mgmt = inst, link = nil }
    check("无 MQTT 链路 → false/no link", select(2, t_nolink:open()), "no link")
    local blank = { subscribe = function() return true end, publish = function() return true end,
                    device_id = function() return nil end }
    local t_noid = mq.new{ mgmt = inst, link = blank, board = { device_id = function() return nil end } }
    check("取不到设备 ID → false/no device id", select(2, t_noid:open()), "no device id")

    -- 15.11 close / 未启用时 start 只登记单例
    local subs_now = #link.subs
    t:close()
    check("close 后 opened = false", t:get_stats().opened, false)
    check("close 后再 open 不重复登记", t:open() and #link.subs, subs_now)

    cfg_mq.MGMT_MQTT_ENABLE = false
    local mq_off2 = dofile(here .. "/../mgmt_mqtt.lua")
    local inst2 = mq_off2.start{ mgmt = inst, link = stub_link() }
    check("未启用时 start 不订阅", inst2:get_stats().opened, false)
    check("未启用时统计可读（全 0）", inst2:get_stats().reply, 0)

    cfg_mq.MGMT_MQTT_ENABLE = saved_enable
    cfg_mq.MGMT_MQTT_SUFFIX = saved_suffix
end

--=============================================================================
-- 16. main.lua 开机整链路 smoke：把固件侧接口（sys/sysplus/pm/errDump/pins/uart）打桩后
--     **真跑 main.lua**，再从「UART1 接收回调」喂一帧裸帧请求，验证响应写回 UART1；
--     并验证管理 MQTT RPC 随 main 一起接线（net_ready 后订阅 → 下行请求 → 上行响应）。
--=============================================================================
io.write("== main.lua 开机整链路 smoke ==\n")

do
    local saved = {
        pm = _G.pm, errDump = _G.errDump, pins = _G.pins,
        u_setup = _G.uart.setup, u_on = _G.uart.on,
        u_read = _G.uart.read, u_write = _G.uart.write,
        run = _G.sys.run, rtos_meminfo = _G.rtos.meminfo,
    }
    local ran_run = false
    local handlers, written, rxq, mux_seq, u1setup = {}, {}, "", {}, nil

    package.preload["sys"] = function() return _G.sys end
    package.preload["sysplus"] = function() return {} end
    _G.pm = { IOVOL_ALL_GPIO = 1, ioVol = function() return true end,
              lastReson = function() return 0, 0, 5 end }
    _G.errDump = { config = function() return true end }
    _G.rtos.meminfo = function() return 1024 * 1024, 44 * 1024, 45 * 1024 end
    _G.sys.run = function() ran_run = true end
    _G.pins = {
        loadjson = function() return true, 0 end,
        debug = function() end,
        setup = function(pin, fn)
            mux_seq[#mux_seq + 1] = string.format("%d:%s", pin, fn)
            return true
        end,
    }
    _G.uart.setup = function(id, baud, db, sb, par, endian, buf, dir)
        if id ~= 1 then return false end
        u1setup = { baud = baud, db = db, sb = sb, par = par, buf = buf, dir = dir }
        return true
    end
    _G.uart.on = function(id, _evt, cb) handlers[id] = cb end
    _G.uart.read = function() local s = rxq; rxq = ""; return s end
    _G.uart.write = function(id, d) written[#written + 1] = { id = id, data = d } end

    local saved_echo = cfg_mg.MGMT_ECHO_TEST
    local saved_mq = cfg_mg.MGMT_MQTT_ENABLE
    cfg_mg.MGMT_ECHO_TEST = false
    cfg_mg.MGMT_MQTT_ENABLE = true
    local ok, err = pcall(dofile, here .. "/../main.lua")
    check("main.lua 开机不抛异常（没有缺模块 / nil 索引）", ok, true)
    if not ok then
        io.write("       main.lua error: ", tostring(err), "\n")
    end
    check("main.lua 执行到 sys.run()", ran_run, true)

    pump()
    scheduler.wake()

    local mux_all = table.concat(mux_seq, ",")
    check("管理链路 UART1 默认复用：不做 pins.setup", mux_all:find("UART1", 1, true), nil)
    check("管理链路 UART1 按 115200 8N1 打开",
        u1setup and (u1setup.baud .. "/" .. u1setup.db .. "/" .. u1setup.sb), "115200/8/1")
    check("管理链路显式控制 485 方向（dir = GPIO25）", u1setup and u1setup.dir, cfg_mg.MGMT_RS485_DIR)
    check("管理链路接收回调已注册", handlers[1] ~= nil, true)

    local mgmt_uart_m = require "mgmt_uart"
    written = {}
    rxq = mgmt_uart_m.build(mgmt_uart_m.TYPE_REQ, 1, '{"method":"status.get"}')
    handlers[1](1)
    check("回调本身不应答（只搬字节）", #written, 0)
    pump()
    local resp
    for _, w in ipairs(written) do
        if string.byte(w.data, 4) == mgmt_uart_m.TYPE_RESP then resp = w end
    end
    check("整链路产生 RESP 应答", resp ~= nil, true)
    check("应答 ID 原样回带（1）", resp and string.byte(resp.data, 5), 1)
    local robj = resp and json.decode(resp.data:sub(8, #resp.data - 2))
    check("status.get 应答 ok", robj and robj.ok, true)
    check("status.get：device.model = D700Tm", robj and robj.result.device.model, "D700Tm")
    check("status.get：links.mgmt_uart 为表", type(robj and robj.result.links.mgmt_uart), "table")
    check("status.get：cache 为表", type(robj and robj.result.cache), "table")
    check("status.get：uplink 为表", type(robj and robj.result.uplink), "table")
    check("status.get：fota.caps 为表", type(robj and robj.result.fota.caps), "table")

    -- 管理 MQTT RPC：net_ready 一到就订阅下行主题
    local mgmt_mqtt_m = require "mgmt_mqtt"
    check("main 接线：管理 MQTT RPC 已启用", mgmt_mqtt_m.is_enabled(), true)
    check("net_ready 之前尚未订阅", mgmt_mqtt_m.is_open(), false)
    scheduler.publish("net_ready", "861234567890123")
    check("net_ready 之后已订阅", mgmt_mqtt_m.is_open(), true)
    local mq_subs = {}
    for _, s in ipairs(subscribed) do mq_subs[#mq_subs + 1] = s.topic end
    check("订阅 = <prefix>/<device>/rpc",
        table.concat(mq_subs, ","):find(TOPIC_PREFIX .. "/861234567890123/rpc", 1, true) ~= nil, true)

    local mc = uplink.client()
    local n_pub = #published
    mc._cb(mc, "recv", TOPIC_PREFIX .. "/861234567890123/rpc", '{"id":33,"method":"sys.ping"}')
    check("JSON-RPC 下行 → 1 条响应发布", #published, n_pub + 1)
    check("响应主题 = .../rpc/resp", published[n_pub + 1].topic,
        TOPIC_PREFIX .. "/861234567890123/rpc/resp")
    check("响应 id 原样回带", json.decode(published[n_pub + 1].payload).id, 33)
    check("统计 mgmq_rx/mgmq_reply = 1/1",
        mgmt_mqtt_m.get_stats().rx .. "/" .. mgmt_mqtt_m.get_stats().reply, "1/1")

    -- 收尾：还原被替换的固件侧接口
    require("mgmt_uart").stop()
    cfg_mg.MGMT_ECHO_TEST = saved_echo
    cfg_mg.MGMT_MQTT_ENABLE = saved_mq
    _G.pm, _G.errDump = saved.pm, saved.errDump
    _G.pins = saved.pins
    _G.uart.setup, _G.uart.on, _G.uart.read, _G.uart.write =
        saved.u_setup, saved.u_on, saved.u_read, saved.u_write
    _G.sys.run = saved.run
    -- 不还原成 nil：main.lua 的内存统计任务还在循环里，后续章节的 wake() 会再调它
    _G.rtos.meminfo = saved.rtos_meminfo or function() return 1024 * 1024, 0, 0 end
    package.preload["sys"], package.preload["sysplus"] = nil, nil
end

--=============================================================================
-- 17. mgmt_uart 临时诊断模式（MGMT_ECHO_TEST = true）：回环 + 心跳
--=============================================================================
io.write("== mgmt_uart 临时诊断模式（回环 + 心跳） ==\n")

do
    local UART_MGMT = 1
    local u_saved = { setup = _G.uart.setup, on = _G.uart.on,
                      read = _G.uart.read, write = _G.uart.write }
    local pins_saved = _G.pins
    local handlers, written, rxq, mux_seq, u1setup = {}, {}, "", {}, nil
    _G.pins = {
        setup = function(pin, fn)
            mux_seq[#mux_seq + 1] = string.format("%d:%s", pin, fn)
            return true
        end,
    }
    _G.uart.setup = function(id, baud, db, sb, par, endian, buf, dir)
        if id ~= UART_MGMT then return false end
        u1setup = { baud = baud, db = db, sb = sb, par = par, buf = buf, dir = dir }
        return true
    end
    _G.uart.on = function(id, _e, cb) handlers[id] = cb end
    _G.uart.read = function() local s = rxq; rxq = ""; return s end
    _G.uart.write = function(id, d) written[#written + 1] = { id = id, data = d } end

    local cfg_echo = require "config"
    local saved_echo = cfg_echo.MGMT_ECHO_TEST
    cfg_echo.MGMT_ECHO_TEST = true
    local mb = dofile(here .. "/../mgmt_uart.lua")
    local t = mb.new{}
    check("诊断模式已生效（echo_mode = true）", t:echo_mode(), true)
    t:start()
    pump()
    check("诊断模式在管理链路 UART1：不做 pins.setup", #mux_seq, 0)
    check("诊断模式同样 115200 8N1 打开",
        u1setup and (u1setup.baud .. "/" .. u1setup.db .. "/" .. u1setup.sb), "115200/8/1")
    check("诊断模式方向脚 = GPIO25（RS485-1 EN）", u1setup and u1setup.dir, cfg_echo.MGMT_RS485_DIR)
    check("诊断模式注册接收回调", handlers[UART_MGMT] ~= nil, true)

    -- 回环：回调只搬字节，回显在任务里做（二进制透明）
    local bin = string.char(0x00, 0x41, 0x0D, 0x0A, 0xFF)
    rxq = bin
    handlers[UART_MGMT](UART_MGMT)
    check("回调本身不回显（只搬字节）", #written, 0)
    pump()
    check("原样回显（含不可见字节）", written[1] and written[1].data, bin)
    check("回显写回 UART1", written[1] and written[1].id, UART_MGMT)

    -- 诊断模式下不跑 RPC：一帧合法请求被原样回显，而不是被应答
    written = {}
    local req = mb.build(mb.TYPE_REQ, 1, '{"method":"sys.ping"}')
    rxq = req
    handlers[UART_MGMT](UART_MGMT)
    pump()
    check("诊断模式不回 RPC（原样回显请求）", written[1] and written[1].data, req)
    check("诊断模式不做帧解析（frames = 0）", t:get_stats().frames, 0)

    -- 心跳
    written = {}
    scheduler.wake()
    pump()
    check("心跳按周期写一行", #written, 1)
    check("心跳行格式 [mg echo] 年月日 时分秒 #序号",
        written[1] ~= nil and string.match(written[1].data,
            "^%[mg echo%] %d%d%d%d%-%d%d%-%d%d %d%d:%d%d:%d%d #1\r\n$") ~= nil, true)
    check("心跳计入 heartbeats", t:get_stats().heartbeats, 1)

    t:stop()
    cfg_echo.MGMT_ECHO_TEST = saved_echo
    _G.uart.setup, _G.uart.on, _G.uart.read, _G.uart.write =
        u_saved.setup, u_saved.on, u_saved.read, u_saved.write
    _G.pins = pins_saved
end

--=============================================================================
-- 18. 三条链路的串口分配：极空 = UART3（需要引脚复用）、管理 = UART1（默认复用）
--=============================================================================
io.write("== 串口分配：极空 UART3 / 管理 UART1 ==\n")

do
    local u_saved = { setup = _G.uart.setup, on = _G.uart.on,
                      read = _G.uart.read, write = _G.uart.write }
    local pins_saved = _G.pins
    local mux_seq, opened = {}, {}
    _G.pins = {
        setup = function(pin, fn)
            mux_seq[#mux_seq + 1] = string.format("%d:%s", pin, fn)
            return true
        end,
    }
    _G.uart.setup = function(id, baud, db, sb, par, endian, buf, dir)
        opened[#opened + 1] = { id = id, baud = baud, db = db, sb = sb, par = par,
                                buf = buf, dir = dir }
        return true
    end
    _G.uart.on = function() end
    _G.uart.read = function() return "" end
    _G.uart.write = function() end

    -- display 协议（2400 广播，只收）
    dofile(here .. "/../jk_display.lua")
    pump()
    check("极空 display：开机复用 UART3 引脚 PIN53=RX、PIN52=TX",
        table.concat(mux_seq, ","), "53:UART3_RX,52:UART3_TX")
    local d = opened[#opened]
    check("极空 display：串口号 = 3（RS485-3）", d and d.id, 3)
    check("极空 display：2400 8N1", d and (d.baud .. "/" .. d.db .. "/" .. d.sb), "2400/8/1")
    check("极空 display：自动方向（无 EN，dir = 0xFFFFFFFF）", d and d.dir, 0xFFFFFFFF)

    -- modbus 协议（115200 主站轮询）
    mux_seq, opened = {}, {}
    local jkm = dofile(here .. "/../jk_modbus.lua")
    pump()
    check("极空 modbus：开机复用 UART3 引脚 PIN53=RX、PIN52=TX",
        table.concat(mux_seq, ","), "53:UART3_RX,52:UART3_TX")
    local m = opened[#opened]
    check("极空 modbus：串口号 = 3（RS485-3）", m and m.id, 3)
    check("极空 modbus：115200 8N1", m and (m.baud .. "/" .. m.db .. "/" .. m.sb), "115200/8/1")
    check("极空 modbus：自动方向（无 EN，dir = 0xFFFFFFFF）", m and m.dir, 0xFFFFFFFF)
    jkm.stop()

    _G.uart.setup, _G.uart.on, _G.uart.read, _G.uart.write =
        u_saved.setup, u_saved.on, u_saved.read, u_saved.write
    _G.pins = pins_saved
end

--=============================================================================
-- 汇总
--=============================================================================
do
    local ups = uplink.get_stats()
    local cs = bms_cache.stats()
    io.write(string.format(
        "\n[info] 共享缓存 %d/%d 条；上报 报文 %d / 真实样本 %d / 空帧 %d / 丢失 %d / 发布 %d\n",
        cs.count, cs.capacity, ups.batches, ups.samples, ups.empty_frames, ups.lost, ups.published))
end
io.write(string.format("\n通过 %d 项, 失败 %d 项\n", pass, fail))
if fail > 0 then
    os.exit(1)
end

