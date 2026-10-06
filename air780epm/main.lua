--[[
@module  main
@summary 电池 SOC 模块主入口（银尔达 D700Tm / Air780EPM）
@version 1.0
@date    2026.09.28
@usage
项目功能（见 README.md）：
1、从极空 BMS 保护板 RS485 接口采集电池信息（极空显示屏协议，2400 8N1 单向广播）；
2、作为 Modbus RTU 从设备响应金箭中控的 BMS 数据查询（9600 8N1，从站地址 1）；
3、通过 4G 连接 MQTT 服务器，每 30 秒把电池数据压缩聚合上报（无 RPC 服务）。

模块划分：
  config.lua          集中配置（串口/GPIO/波特率/MQTT/批次等）
  board.lua           D700Tm 板级资源（LED/按键/看门狗/可控电源/供电电压）
  jk_display.lua      极空显示屏协议接收解析，每轮广播发布 "JK_ROUND"
  jinjian_slave.lua   金箭 Modbus RTU 从机
  bms_uplink.lua      采样批次 + BMSStateHistory 压缩编码 + MQTT 上报
  bms_codec.lua       压缩编码器（列式 Delta+ZigZag+Varint，与上报服务端字节兼容）
  ntp_sync.lua        NTP 对时（上报 Time 列依赖）
]]

PROJECT = "jjsoc"
VERSION = "1.0.0"

_G.sys = require("sys")
_G.sysplus = require("sysplus")

log.info("main", PROJECT, VERSION, rtos.bsp())

-- 记录并上传脚本运行错误（固件支持时），便于现场排查
if errDump then
    errDump.config(true, 600)
end

-- 板级资源：可控电压输出、NET LED、Reload 按键、硬件看门狗、供电电压
local board = require "board"
board.init()

-- 时间同步：联网后 NTP 对时，成功发布 "ntp_synced"
require "ntp_sync"

-- 业务模块
local cfg = require "config"
local jk = require "jk_display"       -- 极空广播接收（RS4851）
local uplink = require "bms_uplink" -- 采样聚合 + MQTT 压缩上报（MQTT_HOST 为空时自动关闭）
local jj = require "jinjian_slave"    -- 金箭 Modbus 从机（RS4852）

--=============================================================================
-- 周期状态日志：每 STATUS_REPORT_MS 汇总两条链路的收发计数，便于现场判断
--   1) 极空保护板数据次数：jk.rounds（完整广播轮数）/ jk.frames（解析帧数）；
--   2) 中控 BMS 查询/应答：jj.queries（收到本机的查询次数）/ jj.responses（应答次数），
--      jj.ignored（非本机地址帧数）；
--   3) 4G 上报：采样/批次/发布/丢弃数与 MQTT 状态。
--=============================================================================
sys.taskInit(function()
    local period = cfg.STATUS_REPORT_MS or 60000
    while true do
        sys.wait(period)
        local jks = jk.get_stats()
        local jjs = jj.get_stats()
        local ups = uplink.get_stats()
        local mqtt_state = (not uplink.is_enabled()) and "disabled"
            or (uplink.is_mqtt_ready() and "up" or "down")
        if cfg.LOG_ENABLE then
            log.info("stat",
                "jk_rounds", jks.rounds, "jk_frames", jks.frames,
                "ctrl_query", jjs.queries, "ctrl_reply", jjs.responses,
                "ctrl_ignored", jjs.ignored,
                "samples", ups.samples, "fallback", ups.placeholders,
                "batches", ups.batches,
                "published", ups.published, "dropped", ups.dropped,
                "mqtt", mqtt_state)
        end
    end
end)

-- 网络任务：等待 4G 拨号成功（IP_READY），发布 "net_ready"（携带设备 ID）
-- 兼容不同固件的 socket API：adapter/dft 不存在时视为未就绪
local function net_ready()
    if not (socket and socket.dft and socket.adapter) then
        return false
    end
    return socket.adapter(socket.dft()) ~= nil
end

sys.taskInit(function()
    if mobile then
        -- 外置卡 SIM2 / 贴片卡 SIM1 可在此切换，默认保持模组当前配置
        -- mobile.simid(2)
        board.set_net_led("fast")
    end

    -- 等待默认网卡就绪
    while not net_ready() do
        sys.waitUntil("IP_READY", 1000)
    end

    local device_id = board.device_id()
    log.info("main", "net ready, device", device_id)
    -- 未配置 MQTT 地址时不会有 CONNACK：直接常亮表示"网络可用、不启用上报"
    board.set_net_led((uplink and uplink.is_enabled()) and "blink" or "on")
    sys.publish("net_ready", device_id)
end)

sys.run()
