--[[
@module  main
@summary 电池 SOC 模块主入口（银尔达 D700Tm / Air780EPM）
@version 1.0
@date    2026.10.14
@usage
项目功能（见 README.md）：
1、从极空 BMS 保护板 RS485 接口采集电池信息（RS485-3 / UART3：显示屏广播 2400 8N1，
   或极空 RS485 Modbus 主站 115200）；
2、作为 Modbus RTU 从设备响应金箭中控的 BMS 数据查询（RS485-2 / UART2，9600 8N1，从站地址 1）；
3、通过 4G 连接 MQTT 服务器，每 30 秒把电池数据压缩聚合上报（同一条连接也承载管理 JSON-RPC）；
4、RS485-1/UART1 作为**管理链路**（一个统一管理服务 + 两条传输）：UART1 走**自定义裸帧 RPC**
   （二进制帧头 + JSON 载荷 + 二进制 BLOB 分块），MQTT 走 **JSON-RPC**（主题后缀 rpc）：
   读状态、读写配置（写透持久化到 fskv）、命令（重启 / FOTA / 恢复出厂），见 README §7。

模块划分：
  config.lua          集中配置（串口/GPIO/波特率/MQTT/批次/管理链路等）
  cfg_store.lua       运行期配置持久化（fskv）：开机覆盖 config 表，写透 + 恢复出厂
  board.lua           D700Tm 板级资源（LED/按键/看门狗/可控电源/供电电压/设备 ID）
  jk.lua              极空数据源入口：按 config.JK_PROTOCOL 转发到下面的实现之一
  jk_display.lua      极空显示屏协议接收解析（2400 广播，只收），每轮发布 "JK_ROUND"
  jk_modbus.lua       极空 RS485 Modbus 主站轮询（115200），每轮发布 "JK_ROUND"
  jk_pb485.lua        极空 PB/BD 私有 300 字节实时帧（115200），每轮发布 "JK_ROUND"
  bms_data.lua        采集适配层：JK_ROUND 快照 → 规范样本 → bms_cache
  bms_cache.lua       共享采样缓存（环形，默认 32 条），采集与消费解耦
  jinjian_slave.lua   金箭 Modbus RTU 从机（只读 bms_cache.latest()）
  bms_uplink.lua      每 30s 从缓存取真实样本 + MessageV1 压缩编码 + MQTT 上报
  bms_codec.lua       MessageV1 编码器（列式 Delta+ZigZag+Varint，与上报服务端字节兼容）
  sha256.lua/hmac.lua 纯 Lua SHA-256 / HMAC-SHA256（MQTT 密码鉴权）
  ntp_sync.lua        NTP 对时（上报 Time 列依赖）
  mgmt.lua            统一管理服务（传输无关）：方法分发 / 状态快照 / 配置读写 / 命令 / FOTA / ID 去重
  mgmt_uart.lua       UART1（RS485-1）裸帧 RPC 传输：115200 8N1、EN=GPIO25、事件（hello/fota）
  uart_pins.lua       UART 引脚复用兜底（唤醒需要复用的串口，如极空的 UART3 PIN53/52）
  mgmt_mqtt.lua       管理 MQTT JSON-RPC 传输：通道 = 主题后缀 rpc，
                      与 4G 上报共用同一条 MQTT 连接（clientId=IMEI 只能有一条）
  fota_mgr.lua        FOTA 升级管理：① 主站下发 URL（libfota2 下载）② 主站经 Modbus
                      分段上传本地升级包（核心库 fota 直接写 Flash，不联网不落文件）；
                      两条通道互斥，成功都只置「成功待重启」，由主站下发现重启命令
]]

PROJECT = "jjsoc"
MODEL = "D700Tm"
VERSION = "1.0.2"
pm.ioVol(pm.IOVOL_ALL_GPIO, 3300)
_G.sys = require("sys")
_G.sysplus = require("sysplus")

log.info("main", PROJECT, MODEL, VERSION, rtos.bsp())

sys.taskInit(function()
    collectgarbage("setpause", 120)
    collectgarbage("setstepmul", 180)
    rtos.autoCollectMem(50, 70, 85)
    while true do
        local total, used, peak = rtos.meminfo("lua")
        log.info(string.format("内存使用率: %.2f%% 总计: %.2f KB  已用: %.2f KB  可用: %.2f KB  峰值: %.2f KB",
            used / total * 100, total / 1024, used / 1024, (total - used) / 1024, peak / 1024)
        )
        sys.wait(10000)
    end
end)

-- 记录并上传脚本运行错误（固件支持时），便于现场排查
if errDump then
    errDump.config(true, 600)
end

-- 集中配置 + 持久化覆盖：cfg_store 必须**在业务模块 require 之前**跑完
-- （jk/bms_uplink/bms_cache 等在 require 时就会把 config 的值缓存住）
local cfg = require "config"
require("cfg_store").load()

-- 板级资源：可控电压输出、NET LED、Reload 按键、硬件看门狗、供电电压
local board = require "board"
board.init()

-- 时间同步：联网后 NTP 对时，成功发布 "ntp_synced"
require "ntp_sync"

-- 业务模块
local jk = require "jk"               -- 极空数据源（RS485-3，显示屏广播或 Modbus 轮询）
local bms_cache = require "bms_cache" -- 共享采样缓存（采集 → 缓存 → 消费）
require "bms_data"                    -- 采集适配：JK_ROUND → 规范样本 → bms_cache
local uplink = require "bms_uplink"   -- 缓存 → MessageV1 → MQTT（MQTT_HOST 为空时自动关闭）
local jj = require "jinjian_slave"    -- 金箭 Modbus 从机（RS4852，只读 bms_cache）

-- 管理链路：统一管理服务（mgmt）+ 两条传输（UART1 裸帧 / MQTT JSON-RPC）
-- 两条传输的方法、字段、错误码完全一致（都由 mgmt 单例实现）。
local mgmt = require "mgmt"
local mgmt_uart = require "mgmt_uart"
local mgmt_mqtt = require "mgmt_mqtt"
local fota_mgr = require "fota_mgr"
mgmt_uart.start()
-- 第二条管理传输：管理 MQTT JSON-RPC（通道后缀 = MGMT_MQTT_SUFFIX，默认 rpc）
-- 与上报共用 bms_uplink 的那条 MQTT 连接（同一 clientId）；未启用/未配 MQTT 时只打日志。
mgmt_mqtt.start()

--=============================================================================
-- 周期状态日志：每 STATUS_REPORT_MS 汇总两条链路的收发计数，便于现场判断
--   1) 极空保护板数据次数：jk.rounds（完整广播轮数）/ jk.frames（解析帧数）；
--   2) 中控 BMS 查询/应答：jj.queries（收到本机的查询次数）/ jj.responses（应答次数），
--      jj.ignored（非本机地址帧数）；
--   3) 共享缓存深度：cache（当前条数/容量，覆盖率过低说明消费端跟不上或缓存太小）；
--   4) 4G 上报：samples（真实样本数）/ empty（0 样本空帧数）/ lost（未上报丢失样本数）/
--      batches（报文数）/ published / dropped 与 MQTT 状态；
--   5) 管理链路 UART1（RS485-1，裸帧 RPC）：mg_rx（收到字节）/ mg_frames（解析出的请求帧数）/
--      mg_reply（应答帧数）/ mg_crc（CRC 错的帧数）/ mg_sync（重同步丢弃字节）/ mg_drop
--      （缓冲溢出丢弃字节）/ mg_tx（写回字节）/ mg_evt（事件帧数，hello/fota）。
--      正常通信时 mg_crc / mg_sync 应为 0，mg_reply 随请求增长。
--   6) 管理 MQTT JSON-RPC（通道后缀 rpc）：mgmq_rx（收到载荷数）/ mgmq_reply（回发响应数）/
--      mgmq_ignored（未回发数）/ mgmq_bad（无法解析的载荷数）。
--=============================================================================
sys.taskInit(function()
    local period = cfg.STATUS_REPORT_MS or 60000
    while true do
        sys.wait(period)
        if cfg.LOG_ENABLE then
            local jks = jk.get_stats()
            local jjs = jj.get_stats()
            local ups = uplink.get_stats()
            local cs = bms_cache.stats()
            local mqtt_state = (not uplink.is_enabled()) and "disabled"
                or (uplink.is_mqtt_ready() and "up" or "down")
            local mbs = mgmt_uart.get_stats() or {}
            local mqs = mgmt_mqtt.get_stats() or {}
            local fi = fota_mgr.info()
            local jk_err = tostring(jks.last_error or "-"):gsub("%s+", "_")
            local fields = {
                "jk_rounds", jks.rounds, "jk_frames", jks.frames,
                "jk_rx", jks.rx_bytes or 0, "jk_sync", jks.sync_lost or 0,
                -- Modbus 主站的现场判据：有请求但 jk_rx=0 → 无回传；jk_to 增长 → 超时；
                -- jk_bad/jk_crc 增长 → 已有字节但帧损坏；jk_skip 增长 → 常见于 485 TX 回显。
                "jk_req", jks.requests or 0, "jk_tx", jks.tx_bytes or 0,
                "jk_to", jks.timeouts or 0, "jk_bad", jks.bad_response or jks.bad_len or 0,
                "jk_crc", jks.bad_crc or 0, "jk_skip", jks.bytecount_mismatch or 0,
                "jk_late", jks.late_frames or 0, "jk_resync", jks.resync or 0,
                "jk_deg", jks.degraded or 0,
                "jk_err", jk_err,
                "ctrl_query", jjs.queries, "ctrl_reply", jjs.responses,
                "ctrl_ignored", jjs.ignored, "ctrl_rx", jjs.rx_bytes or 0,
                "mg_rx", mbs.rx_bytes or 0, "mg_frames", mbs.frames or 0,
                "mg_reply", mbs.replies or 0,
                -- CRC 错 / 重同步丢弃字节：正常通信应为 0；一直涨 = 线路被破坏（A/B 接反/电平/接线）
                "mg_crc", mbs.crc or 0, "mg_sync", mbs.sync or 0,
                "mg_drop", mbs.drop or 0, "mg_tx", mbs.tx_bytes or 0,
                -- mg_evt = 主动事件帧数（evt.hello / evt.fota）
                "mg_evt", mbs.evt or 0,
                -- 管理 MQTT JSON-RPC（第二条管理传输）：收到载荷/回发响应/未回发/无法解析
                "mgmq_rx", mqs.rx or 0, "mgmq_reply", mqs.reply or 0,
                "mgmq_ignored", mqs.ignored or 0, "mgmq_bad", mqs.bad or 0,
                -- 升级状态：fota_src（0 无 / 1 URL / 2 本地）、fota_stage（本地阶段 0-6）、
                -- fota_rx/fota_total（本地已收/声明总字节）、fota_state（0 idle/1 进行中/2 成功/3 失败）
                "fota_src", fi.mode or 0, "fota_stage", fi.stage or 0,
                "fota_state", fi.state or 0,
                "fota_rx", fi.rx or 0, "fota_total", fi.total or 0,
                "cache", cs.count, "/", cs.capacity,
                "samples", ups.samples, "empty", ups.empty_frames,
                "lost", ups.lost,
                "batches", ups.batches,
                "published", ups.published, "dropped", ups.dropped,
                "mqtt", mqtt_state,
            }
            -- 用 table.concat 拼成一条日志：不依赖 table.unpack（不同 Lua/固件裁剪情况不一样，
            -- 万一没有 unpack，会在这里报错把整个 stat 任务打死）
            log.info("stat", table.concat(fields, " "))
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

-- 等 IP_READY 的上限：超时后退回上面的 adapter 判据（个别固件不发该事件）
local NET_READY_GRACE_MS = 30000

sys.taskInit(function()
    if mobile then
        -- 外置卡 SIM2 / 贴片卡 SIM1 可在此切换，默认保持模组当前配置
        -- mobile.simid(2)
        board.set_net_led("fast")
    end

    -- 先等 4G 链路真的就绪（mobile 库在 PDP 激活后 publish "IP_READY"）。
    -- 只用 socket.adapter(socket.dft()) 判据不够：它在 PDP 激活前就可能非 nil，
    -- 会让 NTP/MQTT 抢在没网时发查询/连接。以 IP_READY 为准。
    if not sys.waitUntil("IP_READY", NET_READY_GRACE_MS) then
        log.warn("main", "wait IP_READY", NET_READY_GRACE_MS, "ms timeout, fallback to adapter check")
        while not net_ready() do
            sys.waitUntil("IP_READY", 1000)
        end
    end

    local device_id = board.device_id()
    log.info("main", "net ready, device", device_id)
    -- 未配置 MQTT 地址时不会有 CONNACK：直接常亮表示"网络可用、不启用上报"
    board.set_net_led((uplink and uplink.is_enabled()) and "blink" or "on")
    sys.publish("net_ready", device_id)
end)

sys.run()
