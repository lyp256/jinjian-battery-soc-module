--[[
@module  board
@summary 银尔达 D700Tm（Air780EPM）板级资源初始化（资源固定，全部硬编码）
@version 2.0
@date    2026.10.12
@usage
本模块只服务银尔达 D700Tm 一款板子：引脚和外设都是固定的，所以不做 config 开关、不做
后端探测，硬件资源直接写死在下面的 HW 表里，开机按固定顺序初始化。main.lua 里
require "board" + board.init() 即可。

对外接口：
  board.init()                  开机初始化（可控电压输出）
  board.set_net_led(mode)       NET LED：off / on / blink（未联网）/ fast（联网中）
  board.power_output(bool)      可控电压输出开关，返回是否成功
  board.power_output_state()    当前输出状态
  board.read_supply_voltage()   读一次供电电压（mV），ADC 不可用时返回 nil
  board.device_id()             设备 ID（IMEI，取不到退回模组 unique_id）
  board.pn()                    模块 PN（SOC- + 12 位十六进制）
  board.pins_config_path()      开机加载的引脚复用表路径（nil = 没加载成功）

D700Tm 硬件资源（固定）：
  NET LED        GPIO27（输出，高电平亮）
  Reload 按键    GPIO30（输入上拉，按下接 GND，长按 3s 重启模组）
  硬件看门狗     GPIO28 → Air153C/Air153D 的 WTDOG（air153C_wtd：开机喂一次，
                 之后每 30s 一个 400ms 正脉冲；不要改用 exair153x_wdt，见文件内注释）
  可控电压输出   GPIO21 输出 + GPIO22 使能（时序：使能高 → 设输出 → 5ms → 使能低）
  供电电压采集   ADC0（0-90V，mV = ADC 电压(mV) × 273300 / 3300）
]]

local cfg              = require "config"

local M                = {}

--=============================================================================
-- D700Tm 硬件资源表（本板固定；只在换板子时才改这里）
--=============================================================================
local HW               = {
    NET_LED    = 27, -- NET LED：输出，高电平亮
    RELOAD_KEY = 30, -- Reload 按键：输入上拉，按下接 GND
    WTD        = 28, -- 外置看门狗 Air153C/Air153D 的 WTDOG
    POWER_EN   = 22, -- 可控电压输出使能脚
    POWER_OUT  = 21, -- 可控电压输出脚
    ADC_SUPPLY = 0,  -- 供电电压采集：ADC0
}

local PINS_JSON_PATH   = "/luadb/pins_Air780EPM.json" -- LuatIO 生成的 D700Tm 管脚表
local GPIO_VOLT_MV     = 3300                         -- GPIO 高电平（D700Tm 资源表要求 3.3V，见下）
local POWER_SETTLE_MS  = 5                            -- 可控电压输出时序延时（README：5ms）
local LED_BLINK_MS     = 1000                         -- LED 慢闪半周期（未联网）
local LED_FAST_MS      = 120                          -- LED 快闪半周期（联网中）
local RELOAD_REBOOT_MS = 3000                         -- Reload 长按多久重启模组
local RELOAD_SCAN_MS   = 50                           -- Reload 按键扫描周期
local WTD_FEED_MS      = 30000                        -- 喂狗周期（Air153C 超时最短约 209s，30s 有约 7 倍余量）
local ADC_SCALE        = 273300 / 3300                -- 供电电压(mV) = ADC 电压(mV) × scale
local ADC_SAMPLES      = 3                            -- 单次读取采样次数
local SUPPLY_REPORT_MS = 60000                        -- 供电电压打印周期

local function d_info(...) if cfg.LOG_ENABLE then log.info("board", ...) end end
local function d_warn(...) if cfg.LOG_ENABLE then log.warn("board", ...) end end



--=============================================================================
-- 0. GPIO 高电平 3.3V（D700Tm 资源表：pm.ioVol(pm.IOVOL_ALL_GPIO, 3300)）
--    26.04 NOCAM 固件开机默认只有 1.8V，1.8V 的"高"喂不动 3.3V 供电的 Air153C
--    （WTDOG 输入高电平门限不够）。所以喂狗脉冲的高电平必须是 3.3V：本文件在加载期就
--    喂第一次，这一条紧随其后执行，第一条 400ms 脉冲绝大部分已经是 3.3V。
--=============================================================================
if pm and pm.ioVol and pm.IOVOL_ALL_GPIO then
    local ok, ret = pcall(pm.ioVol, pm.IOVOL_ALL_GPIO, GPIO_VOLT_MV)
    if ok and ret then
        log.info("board", "GPIO 高电平输出 =", GPIO_VOLT_MV, "mV（D700Tm 外围电路要求 3.3V）")
    else
        log.error("board", "pm.ioVol 设置失败，喂狗脉冲可能低于芯片门限", ret)
    end
else
    log.warn("board", "固件没有 pm.ioVol，GPIO 高电平保持默认（喂狗脉冲可能低于芯片门限）")
end
--=============================================================================
-- 1. 硬件看门狗（GPIO28 → Air153C/Air153D 的 WTDOG）
--    本板只有这套波形能喂动看门狗芯片：**不要改用官方扩展库 exair153x_wdt**
--    （它按 GPIO→NPN→WTDOG 假设设计，在 D700Tm 上按时打"喂狗完成"，但芯片照旧按
--    自身超时（约 237s@3.8V）切供电复位，复位原因报 0）。
--    写法要点：
--      1) air153C_wtd.init(28)：把 GPIO28 配成输出并置低，建立确定的初始电平；
--      2) 开机立刻喂第一次：GPIO28 拉高，400ms 后拉低，形成一个 400ms 正脉冲。
--         此刻 GPIO 高电平还是固件默认值，紧接着上面的 pm.ioVol 把整片 GPIO 拉到
--         3.3V，所以这条脉冲绝大部分已经是 3.3V（Air153C 的高电平门限按 3.3V 设计）；
--      3) 独立 sys 任务每 WTD_FEED_MS（30s）调一次 feed_dog，波形同上；
--         Air153C 超时随供电 209s@4.3V~283s@3.3V，30s 有约 7 倍余量；
--      4) GPIO28 上只有本文件这一个写入者：不要再叠加 exair153x_wdt 或别的喂狗波形，
--         两种波形凑在一起反而可能凑成"配对复位"。
--    ⚠ air153C_wtd.lua 必须随脚本一起打包（它就在工程根目录，Luatools 会自动带上）：
--      本文件在**加载期**就 require 它，取不到会让整个脚本加载失败（不是"跑起来但没喂狗"），
--      所以这个文件不能丢；require 也保持字面量写法，别改成 require(变量)。
--    复位原因不在开机时打印，改由管理链路读 status.get 的 reset_reason（见 README §7.3）：
--      pm.lastReson() 第三个值 —— 0 上电/3 软重启/5 RESET 脚/8 内部看门狗/9 外部重启；
--      外置看门狗没吃到狗时是"始终 0 且间隔恒定"，可据此区分。
--=============================================================================
air153C_wtd = require 'air153C_wtd'
air153C_wtd.init(HW.WTD)

-- 模块开机第一步需要喂狗一次
air153C_wtd.feed_dog(HW.WTD)

-- 周期喂狗任务：sys.wait 挂起协程（不忙等），每 30s 一个 400ms 正脉冲
sys.taskInit(function()
    local count = 0

    log.info("WTD", "feed task start!")
    while true do
        count = count + 1
        sys.wait(WTD_FEED_MS)
        air153C_wtd.feed_dog(HW.WTD)
        log.info(string.format("WTD feed #%d done.", count))
    end
end)
--=============================================================================
-- 2. 引脚复用配置（LuatIO 生成的 pins_Air780EPM.json，随脚本下载到 /luadb/）
--    合宙模组一个管脚能复用成多种外设功能，"用哪一组"由开机加载的 pins_$model.json 决定：
--    本工程把 D700Tm 的管脚表随脚本一起打进 /luadb/，固件开机会自动加载；这里再显式
--    loadjson 一次，只为留一条"配置已加载"的日志（现场排查时不用靠 W/pins 报错反推）。
--    表里最要紧的两条：PIN52 = UART3_TXD、PIN53 = UART3_RXD —— 本板 RS485-3（极空那条链路）
--    接的就是这一组（合宙官方 demo 的 PIN57/58 是开发板 V1.3 的脚，D700Tm 没接出去）；
--    PIN106 = GPIO25（RS485-1 方向，管理链路用）、PIN78 = GPIO28（看门狗）、
--    PIN16 = GPIO27（NET LED）也都要靠这张表落到正确的物理脚上。
--    pins.loadjson(path) 成功返回 true, 0，失败返回 nil + 负错误码。
--=============================================================================
local pins_config_path

if pins and pins.loadjson then
    local ok, res, code = pcall(pins.loadjson, PINS_JSON_PATH)
    if ok and res == true then
        pins_config_path = PINS_JSON_PATH
        d_info("引脚复用配置已加载", PINS_JSON_PATH, "code", code)
    else
        d_warn("引脚复用配置未加载（确认 pins_Air780EPM.json 已随脚本下载到 /luadb/；"
            .. "UART3 等复用脚需要它才能落到正确的物理脚）", code)
    end
else
    d_warn("本固件没有 pins.loadjson，跳过引脚复用配置（UART3 等复用脚请在 LuatIO 里配置）")
end

-- 开机引脚复用配置加载到的路径（nil = 没加载成功），供日志/排查读取
function M.pins_config_path()
    return pins_config_path
end

--=============================================================================
-- 3. 可控电压输出（GPIO21 输出 + GPIO22 使能）
--    README：控制电源输出时，先设置使能脚高电平，然后控制输出 GPIO，延迟 5ms 再设置
--    使能脚低电平；模组重启后输出状态保持。
--=============================================================================
local power_on = false

function M.power_output(enable)
    enable = enable and true or false
    if power_on == enable then
        return true
    end
    gpio.setup(HW.POWER_EN, 1)
    gpio.setup(HW.POWER_OUT, enable and 1 or 0)
    sys.wait(POWER_SETTLE_MS)
    gpio.set(HW.POWER_EN, 0)
    power_on = enable
    d_info("power output", enable and "ON" or "OFF")
    return true
end

function M.power_output_state()
    return power_on
end

--=============================================================================
-- 4. NET LED（GPIO27，高电平亮）
--    M.set_net_led("off"|"on"|"blink"|"fast")
--=============================================================================
local led_mode = "off"

function M.set_net_led(mode)
    led_mode = mode or "off"
end

sys.taskInit(function()
    gpio.setup(HW.NET_LED, 0)
    while true do
        local mode = led_mode
        if mode == "on" then
            gpio.set(HW.NET_LED, 1)
            sys.wait(LED_BLINK_MS)
        elseif mode == "blink" then
            gpio.set(HW.NET_LED, 1)
            sys.wait(LED_BLINK_MS)
            gpio.set(HW.NET_LED, 0)
            sys.wait(LED_BLINK_MS)
        elseif mode == "fast" then
            gpio.set(HW.NET_LED, 1)
            sys.wait(LED_FAST_MS)
            gpio.set(HW.NET_LED, 0)
            sys.wait(LED_FAST_MS)
        else
            gpio.set(HW.NET_LED, 0)
            sys.wait(LED_BLINK_MS)
        end
    end
end)

--=============================================================================
-- 5. Reload 按键（GPIO30，输入上拉，按下接 GND）
--    长按 3s 后重启模组。
--=============================================================================
sys.taskInit(function()
    gpio.setup(HW.RELOAD_KEY, nil, gpio.PULLUP)
    local pressed_ticks = 0
    local reported = false
    local need = math.max(1, RELOAD_REBOOT_MS // RELOAD_SCAN_MS)
    while true do
        local pressed = (gpio.get(HW.RELOAD_KEY) == 0)
        if pressed then
            pressed_ticks = pressed_ticks + 1
            if not reported then
                reported = true
                d_info("reload key pressed")
            end
            if pressed_ticks >= need then
                d_warn("reload key hold", RELOAD_REBOOT_MS, "ms, reboot")
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
        sys.wait(RELOAD_SCAN_MS)
    end
end)





--=============================================================================
-- 6. 供电电压采集（ADC0，0-90V）
--   电压(mV) = ADC 电压(mV) × 273300 / 3300
--=============================================================================

-- 读取一次供电电压，返回 mV；ADC 不可用时返回 nil
function M.read_supply_voltage()
    if adc == nil then
        return nil
    end
    adc.setRange(adc.ADC_RANGE_MAX)
    if not adc.open(HW.ADC_SUPPLY) then
        return nil
    end
    -- 单次 adc.get 是同步转换（部分固件里是忙等，可能占用数 ms），因此逐次采样、每次
    -- 之间让出一次，避免一次读满多次把调度器占住（影响串口/网络任务）。
    local sum, n = 0, 0
    for i = 1, ADC_SAMPLES do
        local v = adc.get(HW.ADC_SUPPLY)
        if v then
            sum = sum + v
            n = n + 1
        end
        if i < ADC_SAMPLES then
            sys.wait(1)
        end
    end
    adc.close(HW.ADC_SUPPLY)
    if n == 0 then
        return nil
    end
    return (sum / n) * ADC_SCALE
end

sys.taskInit(function()
    while true do
        sys.wait(SUPPLY_REPORT_MS)
        local mv = M.read_supply_voltage()
        if mv then
            d_info(string.format("supply voltage %.2f V", mv / 1000))
        else
            d_warn("read supply voltage failed")
        end
    end
end)

--=============================================================================
-- 7. 设备标识（供 MQTT 主题、金箭 PN 使用）
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

-- 开机初始化板级资源（D700Tm 可控电压输出默认上电即打开）
function M.init()
    sys.taskInit(function()
        M.power_output(true)
    end)
end

return M
