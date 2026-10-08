--[[
@module  jk
@summary 极空 BMS 数据源（按 config.JK_PROTOCOL 选择“显示屏广播”或“RS485 Modbus”）
@version 1.0
@date    2026.10.08
@usage
极空保护板有两套公开协议，本项目按配置二选一，产出同一份快照（表结构一致），
下游（金箭 Modbus 从机 jinjian_slave、4G 压缩上报 bms_uplink）完全不需要区分：

  config.JK_PROTOCOL = "display"（默认）
      极空“显示屏接口”2400 8N1 单向广播（只收不发），见 jk_display.lua；
  config.JK_PROTOCOL = "modbus"
      极空“RS485 接口”115200 8N1 主从轮询（本机为主站），见 jk_modbus.lua，
      协议见 docs/jikong/JK-BMS-RS485.md。

本模块只是转发：require "jk" 后拿到的是被选中实现本身，接口与 jk_display 相同：
    jk.get_state()   最近一轮快照（table 或 nil）
    jk.is_fresh()    是否在超时时间内
    jk.get_stats()   统计信息（rounds/frames/…，供周期状态日志汇总）
    jk.MAX_CELLS     单体槽位上限（display = 25，modbus = 32）

切换协议只需改 config.lua 的 JK_PROTOCOL 后重新烧录脚本；两种实现的串口/方向脚
（JK_UART_ID / JK_RS485_DIR_GPIO）共用，因为它们接的是同一个 485 通道。
]]

local cfg = require "config"

local M
if cfg.JK_PROTOCOL == "modbus" then
    M = require "jk_modbus"
    log.info("jk", "数据源 = 极空 RS485 Modbus（主站轮询）")
else
    M = require "jk_display"
    log.info("jk", "数据源 = 极空显示屏协议（2400 广播，只收）")
end

return M
