--[[
@module  wdt_selftest
@summary 板级资源初始化（board.lua）的离线自测（PC 端 Lua 5.4，不需要硬件）
@version 3.0
@date    2026.10.13
@usage
在 air780epm 目录下执行：
    lua tools/wdt_selftest.lua

背景：D700Tm 的板级资源固定，board.lua 里全部硬编码。本自测用打桩的
sys/gpio/log/pm/pins/adc/air153C_wtd 真跑 board.lua，覆盖：
1. 硬件看门狗（README §8）：air153C_wtd.init(28) 先建确定的初始电平
   → 加载期立刻喂一次 → 独立 sys 任务每 30s 喂一次，每次都是 400ms 正脉冲（拉高，
  400ms 后拉低）；GPIO28 上只有 board.lua 一个写入者，且不加载官方扩展库
  exair153x_wdt（它在本板上打"喂狗完成"但芯片照旧复位）；
2. 喂狗库 air153C_wtd 没随脚本打包 → board.lua 在**加载期**就报错退出（不是"跑起来但
   静默不喂狗"），错误信息里能直接看出是哪个模块取不到；
3. 引脚复用：开机按固定路径 pins.loadjson("/luadb/pins_Air780EPM.json")，表没下载进去 /
   固件没有 pins 库时只告警不崩，且都不影响喂狗；
4. 开机把 GPIO 高电平设成 3.3V（D700Tm 外围电路要求；1.8V 喂不动 Air153C），
   老固件没有 pm.ioVol 时只告警；
5. 供电电压采集 ADC0：mV = ADC mV × 273300 / 3300。
]]

local here = (arg and arg[0] or ""):match("^(.*)[/\\][^/\\]*$") or "."
package.path = here .. "/../?.lua;" .. here .. "/?.lua;" .. package.path

--=============================================================================
-- 运行环境打桩：让 board.lua 能在 PC 的 Lua 上真正跑起来
--=============================================================================
local now = 0                -- 虚拟毫秒时钟
local tasks, timers = {}, {}
local pin_level, pin_events, pin_setups, logs = {}, {}, {}, {}

local function schedule(fn, ms)
    timers[#timers + 1] = { at = now + (ms or 1), fn = fn }
    return #timers
end

_G.sys = {
    taskInit = function(fn) tasks[#tasks + 1] = { co = coroutine.create(fn), at = now } end,
    wait = function(ms) coroutine.yield(ms or 1) end,
    waitUntil = function(_, timeout) coroutine.yield(timeout or 100000) end,
    publish = function() end,
    subscribe = function() end,
    timerStart = function(fn, ms, ...)
        local args = { ... }
        return schedule(function() fn(table.unpack(args)) end, ms)
    end,
    timerLoopStart = function() return 0 end,
}

-- 引脚打桩：记录每次电平变化（t = 虚拟毫秒）、setup 参数，并保存当前电平
_G.gpio = {
    PULLUP = 1, PULLDOWN = 2,
    setup = function(pin, level, pull)
        pin_setups[#pin_setups + 1] = { pin = pin, level = level, pull = pull }
        if level ~= nil then pin_level[pin] = level end
    end,
    set = function(pin, level)
        if pin_level[pin] == level then
            return
        end
        pin_level[pin] = level
        pin_events[#pin_events + 1] = { t = now, pin = pin, level = level }
    end,
    get = function() return 1 end,          -- Reload 按键：常开（=1）
}

-- 日志参数可能是数字/布尔（LuatOS 的 log 都能打），统一 tostring 后再拼接
local function join(...)
    local parts = {}
    for i = 1, select("#", ...) do
        parts[i] = tostring((select(i, ...)))
    end
    return table.concat(parts, " ")
end

_G.log = {
    info = function(tag, ...) logs[#logs + 1] = { lvl = "info", tag = tostring(tag), msg = join(...) } end,
    warn = function(tag, ...) logs[#logs + 1] = { lvl = "warn", tag = tostring(tag), msg = join(...) } end,
    error = function(tag, ...) logs[#logs + 1] = { lvl = "error", tag = tostring(tag), msg = join(...) } end,
    debug = function() end,
}

local io_volt_calls = {}
_G.pm = {
    IOVOL_ALL_GPIO = 1,
    ioVol = function(id, mv) io_volt_calls[#io_volt_calls + 1] = { id = id, mv = mv }; return true end,
}
_G.mcu = {
    ticks = function() return now end, hz = function() return 1000 end,
    unique_id = function() return { toHex = function() return "9888E072BD78AABB" end } end,
}
_G.mobile = { imei = function() return "861234567890123" end, simid = function() end }
_G.rtos = { reboot = function() end, bsp = function() return "PC" end }

-- 供电电压采集打桩：默认"ADC 开不起来"（老固件没接/未初始化），个别场景再给值
local adc_opened, adc_mv = {}, nil
_G.adc = {
    open = function(ch) adc_opened[#adc_opened + 1] = ch; return adc_mv ~= nil end,
    get = function() return adc_mv end,
    close = function() end, setRange = function() end, ADC_RANGE_MAX = 1,
}

--=============================================================================
-- 引脚复用打桩（board.lua 开机加载 LuatIO 生成的 pins_Air780EPM.json）：
--   pins.loadjson(path) 成功返回 true, 0；失败返回 nil + 负的错误码（官方文档 4.3）。
--   pins_missing = true → 模拟"文件没随脚本下载到 /luadb/"
--=============================================================================
local pins_loads = {}
local pins_missing = false
local pins_stub = {
    loadjson = function(path)
        pins_loads[#pins_loads + 1] = path
        if pins_missing then
            return nil, -1                        -- $path not exist!!
        end
        return true, 0
    end,
    debug = function() end,
}
_G.pins = pins_stub

--=============================================================================
-- 极简调度器：sys.wait / sys.timerStart / 任务结束都能正确处理
--=============================================================================
local function run_until(t_end)
    while true do
        local next_at
        for _, t in ipairs(tasks) do
            if not t.dead and (next_at == nil or t.at < next_at) then next_at = t.at end
        end
        for _, tm in ipairs(timers) do
            if not tm.done and (next_at == nil or tm.at < next_at) then next_at = tm.at end
        end
        if next_at == nil or next_at > t_end then
            now = t_end
            return
        end
        now = next_at
        for _, tm in ipairs(timers) do
            if not tm.done and tm.at <= now then
                tm.done = true
                tm.fn()
            end
        end
        local ran = true
        while ran do
            ran = false
            for _, t in ipairs(tasks) do
                if not t.dead and t.at <= now then
                    ran = true
                    local ok, y = coroutine.resume(t.co)
                    if not ok then
                        t.dead = true
                        error("[sched] board 任务异常: " .. tostring(y))
                    elseif coroutine.status(t.co) == "dead" then
                        t.dead = true
                    else
                        t.at = now + (tonumber(y) or 1)
                    end
                end
            end
        end
    end
end

--=============================================================================
-- 工程自带的看门狗库 air153C_wtd 的打桩（接口与 air153C_wtd.lua 一致）：
--   init(pin)       gpio.setup(pin, 0, gpio.PULLDOWN) + gpio.set(pin, 0)（建确定的初始电平）
--   feed_dog(pin)   gpio.set(pin, 1)，400ms 后 gpio.set(pin, 0) → 400ms 正脉冲
-- 本板只有这种波形喂得动 Air153C（官方扩展库 exair153x_wdt 喂不动，每 30s 打
-- "喂狗完成"但芯片照旧按自身超时复位，见 README §8）。
--=============================================================================
local function make_air153c(opts)
    opts = opts or {}
    local m = { feeds = {}, init_pin = nil }
    function m.init(pin)
        if opts.init_throw then
            error("mock: air153C_wtd.init 内部异常")
        end
        m.init_pin = pin
        gpio.setup(pin, 0, gpio.PULLDOWN)
        gpio.set(pin, 0)
    end
    function m.feed_dog(pin)
        m.feeds[#m.feeds + 1] = now
        gpio.set(pin, 1)
        sys.timerStart(function() gpio.set(pin, 0) end, 400)
    end
    function m.close_watch_dog(pin)
        gpio.set(pin, 1)
        sys.timerStart(function() gpio.set(pin, 0) end, 700)
    end
    return m
end

--=============================================================================
-- 断言工具
--=============================================================================
local pass, fail = 0, 0

local function check(name, got, want)
    if got == want then
        pass = pass + 1
        io.write(string.format("  ok   %s\n", name))
    else
        fail = fail + 1
        io.write(string.format("  FAIL %s\n       got: %s\n       want: %s\n",
            name, tostring(got), tostring(want)))
    end
end

-- 日志文本可能在 tag（log.info("tag", ...)）或 msg（log.info("整句")）里，两处都找
local function find_log(pat)
    for _, l in ipairs(logs) do
        if (l.tag .. " " .. l.msg):find(pat, 1, true) then
            return l
        end
    end
    return nil
end

-- 某个引脚上的正脉冲列表（只算完整的一对 rise/fall）
local function pulses(pin)
    local out = {}
    local rise
    for _, e in ipairs(pin_events) do
        if e.pin == pin then
            if e.level == 1 then
                rise = e.t
            elseif rise then
                out[#out + 1] = { rise = rise, fall = e.t, width = e.t - rise }
                rise = nil
            end
        end
    end
    return out
end

local function count_of(list, v)
    local n = 0
    for _, x in ipairs(list) do
        if x == v then n = n + 1 end
    end
    return n
end

local function count_pin_setups(pin)
    local n = 0
    for _, s in ipairs(pin_setups) do
        if s.pin == pin then n = n + 1 end
    end
    return n
end

-- 某个引脚的 gpio.setup 参数（最后一次）
local function last_pin_setup(pin)
    for i = #pin_setups, 1, -1 do
        if pin_setups[i].pin == pin then
            return pin_setups[i]
        end
    end
    return nil
end

local function count_pin_events(pin)
    local n = 0
    for _, e in ipairs(pin_events) do
        if e.pin == pin then n = n + 1 end
    end
    return n
end

--=============================================================================
-- 场景脚手架：重置世界 → 摆好"设备上有什么" → 真跑 board.lua → 推进虚拟时钟
--=============================================================================
local function scenario(opts)
    opts = opts or {}
    tasks, timers, pin_events, pin_setups, logs, io_volt_calls = {}, {}, {}, {}, {}, {}
    pin_level = {}
    now = 0

    adc_opened, adc_mv = {}, opts.adc_mv

    -- 引脚复用：默认"pins 库在、pins json 也在"，个别场景再改（pins_missing / no_pins）
    pins_loads = {}
    pins_missing = opts.pins_missing and true or false
    -- 注意不能写 `opts.no_pins and nil or pins_stub`：no_pins=true 时 `true and nil` 是 nil，
    -- `nil or pins_stub` 又拣回 pins_stub，等于没生效（Lua 的 and/or 惯用法在 b 为 nil 时失效）
    if opts.no_pins then
        _G.pins = nil
    else
        _G.pins = pins_stub
    end

    package.loaded["air153C_wtd"], package.loaded["exair153x_wdt"], package.loaded["board"] = nil, nil, nil
    package.preload["air153C_wtd"] = nil
    package.preload["exair153x_wdt"] = nil

    local air153c
    if opts.no_air153c then
        -- 库文件没随脚本打包时，board.lua 加载期 require 就会失败
        package.preload["air153C_wtd"] = function() error("module 'air153C_wtd' not found") end
    else
        air153c = make_air153c(opts.air153c)
        package.preload["air153C_wdt"] = nil
        package.preload["air153C_wtd"] = function() return air153c end
    end
    -- 守卫"board.lua 不加载官方扩展库 exair153x_wdt"：真被 require 到就能发现
    _G.__exair_required = false
    package.preload["exair153x_wdt"] = function()
        _G.__exair_required = true
        return { init = function() return true end, feed = function() return true end }
    end

    local saved_ioVol = pm.ioVol
    if opts.no_ioVol then
        pm.ioVol = nil                    -- 模拟没有 pm.ioVol 的老固件
    end

    local ok, board_mod = pcall(require, "board")
    if ok then
        run_until(opts.until_ms or 70000)
    end

    pm.ioVol = saved_ioVol

    -- 打印这一轮的板级日志（与现场日志同样式，便于对照）
    io.write(string.format("\n== %s ==\n", opts.name))
    for _, l in ipairs(logs) do
        io.write(string.format("    [%s] %s %s\n", l.lvl, l.tag, l.msg))
    end
    if not ok then
        io.write(string.format("    [error] require(\"board\") 失败: %s\n", tostring(board_mod)))
    end
    return { ok = ok, err = (not ok) and board_mod or nil, mod = ok and board_mod or nil,
             air153c = air153c, exair_required = _G.__exair_required,
             pin_level = pin_level, io_volt = io_volt_calls }
end

--=============================================================================
-- 1. 工程自带的 air153C_wtd 喂狗（board.lua 默认路径）
--=============================================================================
do
    local w = scenario({ name = "1. air153C_wtd 喂狗" })
    check("1 board.lua 加载成功", w.ok, true)
    check("1 init 到 GPIO28", w.air153c.init_pin, 28)
    local s28 = last_pin_setup(28)
    check("1 用 gpio.setup 把 GPIO28 建成输出、初始电平为低、带下拉",
        s28 and (s28.level == 0) and (s28.pull == gpio.PULLDOWN), true)
    check("1 开机加载期就喂了第一次", w.air153c.feeds[1], 0)
    check("1 第二次间隔 30s", w.air153c.feeds[2] and (w.air153c.feeds[2] - w.air153c.feeds[1]), 30000)
    check("1 70s 内喂了 3 次（0 / 30 / 60s）", #w.air153c.feeds, 3)

    local ps = pulses(28)
    check("1 每次喂狗都是一对上升/下降沿", #ps, 3)
    check("1 喂狗脉冲宽度都是 400ms", ps[1] and ps[1].width == 400 and ps[2] and ps[2].width == 400
        and ps[3] and ps[3].width == 400, true)
    check("1 脉冲收尾回到低电平", w.pin_level[28], 0)

    check("1 喂狗任务打印 start", find_log("WTD feed task start!") ~= nil, true)
    check("1 每次喂完打印 #N done", find_log("WTD feed #2 done.") ~= nil, true)
    check("1 没有加载官方默认扩展库 exair153x_wdt", w.exair_required, false)
    check("1 开机把 GPIO 高电平设成 3.3V", w.io_volt[1] and w.io_volt[1].mv, 3300)
    check("1 只写一个 GPIO 电压寄存器（全 GPIO）", w.io_volt[1] and w.io_volt[1].id, 1)
    check("1 其它板级任务照常跑（NET LED / Reload 按键都已初始化）",
        count_pin_setups(27) >= 1 and count_pin_setups(30) >= 1, true)
end

--=============================================================================
-- 2. air153C_wtd 没随脚本打包 → 加载期就报错（不是静默不喂狗）
--=============================================================================
do
    local w = scenario({ name = "2. air153C_wtd 没打包 → board.lua 加载期报错", no_air153c = true })
    check("2 board.lua 加载失败", w.ok, false)
    check("2 错误信息指明是 air153C_wtd 取不到", tostring(w.err):find("air153C_wtd", 1, true) ~= nil, true)
    check("2 没有任何喂狗波形", #pin_events, 0)
    check("2 也没有碰 GPIO28", count_pin_setups(28), 0)
end

--=============================================================================
-- 3. 老固件没有 pm.ioVol → 只告警，不崩、不影响喂狗
--=============================================================================
do
    local w = scenario({ name = "3. 没有 pm.ioVol 的老固件", no_ioVol = true })
    check("3 告警而不是报错退出", find_log("固件没有 pm.ioVol") ~= nil, true)
    check("3 喂狗照常", #w.air153c.feeds, 3)
    check("3 没有设置 GPIO 电压", #w.io_volt, 0)
end

--=============================================================================
-- 4. 引脚复用：开机加载 LuatIO 生成的 pins_Air780EPM.json（D700Tm 的管脚表）
--=============================================================================
do
    local w = scenario({ name = "4. 引脚复用配置已加载" })
    check("4 按工程里的固定文件名请求 pins json", pins_loads[1], "/luadb/pins_Air780EPM.json")
    check("4 只请求一次（路径固定，不做候选回退）", #pins_loads, 1)
    check("4 打印“配置已加载”日志", find_log("引脚复用配置已加载") ~= nil, true)
    check("4 没有“未加载”告警", find_log("引脚复用配置未加载") == nil, true)
    check("4 对外可查加载路径", w.mod.pins_config_path(), "/luadb/pins_Air780EPM.json")
    check("4 不影响喂狗", #w.air153c.feeds, 3)
end

--=============================================================================
-- 5. 引脚复用：pins json 没随脚本下载 → 明确告警，不影响喂狗
--=============================================================================
do
    local w = scenario({ name = "5. pins json 缺失（没下载进去）", pins_missing = true })
    check("5 固定路径只请求一次", #pins_loads, 1)
    check("5 明确告警未加载", find_log("引脚复用配置未加载") ~= nil, true)
    check("5 未加载时对外返回 nil", w.mod.pins_config_path(), nil)
    check("5 引脚配置失败不影响喂狗", #w.air153c.feeds, 3)
end

--=============================================================================
-- 6. 引脚复用边界：老固件没有 pins 库 → 告警而不是报错
--=============================================================================
do
    local w = scenario({ name = "6. 没有 pins 库的老固件", no_pins = true })
    check("6 告警而不是报错", find_log("没有 pins.loadjson") ~= nil, true)
    check("6 没有尝试 loadjson", #pins_loads, 0)
    check("6 喂狗照常", #w.air153c.feeds, 3)
end

--=============================================================================
-- 7. 供电电压采集：ADC0，mV = ADC mV × 273300 / 3300
--=============================================================================
do
    -- 1087 mV × 273300 / 3300 ≈ 90023 mV ≈ 90.02V（0-90V 量程满偏附近）
    local w = scenario({ name = "7. 供电电压采集（ADC0）", adc_mv = 1087 })
    check("7 开了 ADC0", adc_opened[1], 0)
    check("7 换算并打印供电电压", find_log("supply voltage 90.02 V") ~= nil, true)
    check("7 不影响喂狗", #w.air153c.feeds, 3)
end

--=============================================================================
-- 汇总
--=============================================================================
io.write(string.format("\n通过 %d 项, 失败 %d 项\n", pass, fail))
if fail > 0 then
    os.exit(1)
end
