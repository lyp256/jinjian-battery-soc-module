--[[
@module  uart_pins
@summary UART 引脚复用兜底：把配置的 RX/TX 脚设成某个 UART 的功能
@version 1.0
@date    2026.10.13
@usage
合宙模组的管脚是**复用**的：同一个脚既能当 GPIO、也能当 UART3_RXD…，"用哪一种"由开机
加载的 `pins_$model.json` 决定（见 README §2.2）。UART1/UART2 是默认复用功能，不用管；
UART3 **不是**默认复用功能，必须在 `uart.setup` 之前把 RX/TX 脚显式设成 UART3 功能，
否则串口能 open、日志照写，但 485 总线上一个字节都没有（判据见 README §7.1）。

    uart_pins.mux(3, 53, 52, "mb3")   -- PIN53=UART3_RXD、PIN52=UART3_TXD

约定：
  · 功能名按 uart_id 拼：`UART<n>_RX` / `UART<n>_RXD`（两种写法都试，不同固件叫法不一样）；
  · pin_rx / pin_tx 任一为 nil 就整段跳过 —— 默认复用功能的 UART 不配也不该配；
  · 失败只告警、不抛异常：配错引脚组时现象极具迷惑性，所以成功/失败都留日志；
  · tag 是日志前缀（默认 "pins"），跟调用方自己的日志前缀保持一致，便于现场按前缀过滤。

pins 是固件核心库，脚本里**不需要** require；PC 离线测试没有 pins 库时静默跳过。
]]

local cfg = require "config"

local M = {}

local function d_info(tag, ...) if cfg.LOG_ENABLE then log.info(tag, ...) end end
local function d_warn(tag, ...) if cfg.LOG_ENABLE then log.warn(tag, ...) end end

-- 按候选功能名逐个试；返回 true + 命中的功能名，或 false + 原因
local function set_pin(pin, names)
    local reason
    for _, fn in ipairs(names) do
        local ok, ret = pcall(pins.setup, pin, fn)
        if ok and ret == true then
            return true, fn
        end
        reason = ok and "不被支持" or ret
    end
    return false, reason
end

-- 把 UART 的 RX/TX 引脚复用到 uart_id 上；返回是否（该配的都）配好
function M.mux(uart_id, pin_rx, pin_tx, tag)
    tag = tag or "pins"
    uart_id = tonumber(uart_id)
    if not uart_id or not (pin_rx and pin_tx) then
        return true -- 默认复用功能 / 没配引脚：不需要做任何事
    end
    if not (pins and pins.setup) then
        d_warn(tag, "需要复用 UART" .. uart_id .. " 引脚，但本固件没有 pins 库"
            .. "（请用 LuatIO 生成 pins json）")
        return false
    end
    if pins.debug then
        pcall(pins.debug, true)
    end
    local plan = {
        { pin_rx, { "UART" .. uart_id .. "_RX", "UART" .. uart_id .. "_RXD" } },
        { pin_tx, { "UART" .. uart_id .. "_TX", "UART" .. uart_id .. "_TXD" } },
    }
    local all_ok = true
    for _, w in ipairs(plan) do
        local pin, names = w[1], w[2]
        local done, got = set_pin(pin, names)
        if done then
            d_info(tag, "引脚复用 PIN", pin, "->", got)
        else
            all_ok = false
            d_warn(tag, "pins.setup(PIN " .. pin .. ") 失败:", got, "（引脚组/方向不对？）")
        end
    end
    return all_ok
end

return M
