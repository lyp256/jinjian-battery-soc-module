--[[
@module  board
@summary 银尔达 D700Tm（Air780EPM）板级资源驱动
@version 1.0
@date    2026.09.28
@usage
本模块没有对外接口，直接在 main.lua 中 require "board" 加载运行，负责：
1、可控电压输出（GPIO22 使能 + GPIO21 输出）上电时序；
2、NET LED（GPIO27）状态指示：灭 / 常亮 / 慢闪（未联网）/ 快闪（连接中）；
3、Reload 按键（GPIO30）长按重启模组；
4、硬件看门狗（GPIO28 + air153C_wtd），周期喂狗；
5、供电电压采集（ADC0，0-90V）。
]]

local cfg = require "config"
local M = {}

local function d_info(...) if cfg.LOG_ENABLE then log.info("board", ...) end end
local function d_warn(...) if cfg.LOG_ENABLE then log.warn("board", ...) end end

--=============================================================================
-- 1. 可控电压输出
--    README: 控制电源输出时，先设置使能 GP10 高电平，然后控制输出 GPIO，
--            延迟 5ms 再设置使能脚低电平；模组重启后输出状态保持。
--=============================================================================
local power_on = false

function M.power_output(enable)
    enable = enable and true or false
    if power_on == enable then
        return true
    end
    gpio.setup(cfg.POWER_OUT_EN_GPIO, 1)
    gpio.setup(cfg.POWER_OUT_GPIO, enable and 1 or 0)
    sys.wait(cfg.POWER_OUT_SETTLE_MS)
    gpio.set(cfg.POWER_OUT_EN_GPIO, 0)
    power_on = enable
    d_info("power output", enable and "ON" or "OFF")
    return true
end

function M.power_output_state()
    return power_on
end

--=============================================================================
-- 2. NET LED（GPIO27，高电平亮）
--    M.set_net_led("off"|"on"|"blink"|"fast")
--=============================================================================
local led_mode = "off"

function M.set_net_led(mode)
    led_mode = mode or "off"
end

sys.taskInit(function()
    if cfg.NET_LED_GPIO == nil then
        return
    end
    gpio.setup(cfg.NET_LED_GPIO, 0)
    while true do
        local mode = led_mode
        if mode == "on" then
            gpio.set(cfg.NET_LED_GPIO, 1)
            sys.wait(500)
        elseif mode == "blink" then
            gpio.set(cfg.NET_LED_GPIO, 1)
            sys.wait(500)
            gpio.set(cfg.NET_LED_GPIO, 0)
            sys.wait(500)
        elseif mode == "fast" then
            gpio.set(cfg.NET_LED_GPIO, 1)
            sys.wait(120)
            gpio.set(cfg.NET_LED_GPIO, 0)
            sys.wait(120)
        else
            gpio.set(cfg.NET_LED_GPIO, 0)
            sys.wait(500)
        end
    end
end)

--=============================================================================
-- 3. Reload 按键（GPIO30，输入上拉，按下接 GND）
--    长按 cfg.RELOAD_KEY_REBOOT_MS 后重启模组。
--=============================================================================
sys.taskInit(function()
    if cfg.RELOAD_KEY_GPIO == nil then
        return
    end
    gpio.setup(cfg.RELOAD_KEY_GPIO, nil, gpio.PULLUP)
    local pressed_ticks = 0
    local reported = false
    local need = math.max(1, cfg.RELOAD_KEY_REBOOT_MS // 50)
    while true do
        local pressed = (gpio.get(cfg.RELOAD_KEY_GPIO) == 0)
        if pressed then
            pressed_ticks = pressed_ticks + 1
            if not reported then
                reported = true
                d_info("reload key pressed")
            end
            if pressed_ticks >= need then
                d_warn("reload key hold", cfg.RELOAD_KEY_REBOOT_MS, "ms, reboot")
                sys.wait(200)
                rtos.reboot()
            end
        else
            if reported then
                d_info("reload key released")
            end
            pressed_ticks = 0
            reported = false
        end
        sys.wait(50)
    end
end)

--=============================================================================
-- 4. 硬件看门狗（GPIO28 + air153C_wtd，README 建议 150 秒喂一次）
--=============================================================================
sys.taskInit(function()
    if cfg.WDT_GPIO == nil then
        return
    end
    if air153C_wtd == nil then
        d_warn("air153C_wtd library not found, skip hardware watchdog")
        return
    end
    air153C_wtd.init(cfg.WDT_GPIO)
    d_info("hardware watchdog init on GPIO", cfg.WDT_GPIO)
    while true do
        air153C_wtd.feed_dog(cfg.WDT_GPIO)
        sys.wait(cfg.WDT_FEED_INTERVAL_MS)
    end
end)

--=============================================================================
-- 5. 供电电压采集（ADC0，0-90V）
--    电压(mV) = ADC 电压(mV) * 273300 / 3300
--=============================================================================

-- 读取一次供电电压，返回 mV；ADC 不可用时返回 nil
function M.read_supply_voltage()
    if adc == nil or cfg.ADC_SUPPLY_CHANNEL == nil then
        return nil
    end
    adc.setRange(adc.ADC_RANGE_MAX)
    if not adc.open(cfg.ADC_SUPPLY_CHANNEL) then
        return nil
    end
    -- 单次 adc.get 是同步转换（部分固件里是忙等，可能占用数 ms），因此按
    -- cfg.ADC_SUPPLY_SAMPLES 逐次采样、每次之间让出一次，避免一次读满多次
    -- 把调度器占住（影响串口/网络任务）。
    local samples = cfg.ADC_SUPPLY_SAMPLES or 3
    local sum, n = 0, 0
    for i = 1, samples do
        local v = adc.get(cfg.ADC_SUPPLY_CHANNEL)
        if v then
            sum = sum + v
            n = n + 1
        end
        if i < samples then
            sys.wait(1)
        end
    end
    adc.close(cfg.ADC_SUPPLY_CHANNEL)
    if n == 0 then
        return nil
    end
    return (sum / n) * cfg.ADC_SUPPLY_SCALE
end

sys.taskInit(function()
    while true do
        sys.wait(cfg.SUPPLY_REPORT_MS)
        local mv = M.read_supply_voltage()
        if mv then
            d_info(string.format("supply voltage %.2f V", mv / 1000))
        else
            d_warn("read supply voltage failed")
        end
    end
end)

--=============================================================================
-- 6. 设备标识（供 MQTT 主题、金箭 PN 使用）
--=============================================================================

-- 优先 IMEI；取不到时退回模组唯一 ID 的十六进制
function M.device_id()
    local imei
    if mobile and mobile.imei then
        local ok, v = pcall(mobile.imei)
        if ok and type(v) == "string" and #v >= 8 then
            imei = v
        end
    end
    if imei then
        return imei
    end
    local uid = "0000000000000000"
    if mcu and mcu.unique_id then
        local ok, v = pcall(function() return mcu.unique_id():toHex() end)
        if ok and type(v) == "string" and #v >= 12 then
            uid = v
        end
    end
    return uid
end

-- 模块自身 PN：16 位 ASCII，形如 SOC-9888E072BD78（与 ESP32 侧一致）
function M.pn()
    local raw = M.device_id():upper():gsub("[^0-9A-F]", "")
    if #raw < 12 then
        raw = ("000000000000" .. raw)
    end
    return "SOC-" .. raw:sub(-12)
end

-- 开机初始化板级资源
function M.init()
    sys.taskInit(function()
        if cfg.POWER_OUT_ON_BOOT then
            M.power_output(true)
        end
    end)
end

return M
