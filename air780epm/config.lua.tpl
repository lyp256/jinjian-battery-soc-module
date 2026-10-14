--[[
@module  config
@summary Air780EPM 电池 SOC 模块集中配置（银尔达 D700Tm）
@version 1.0
@date    2026.09.28
@usage
本文件集中存放所有可调参数，其它模块通过 require "config" 读取。
修改参数后重新烧录脚本即可生效；运行期也可由管理链路 config.set 覆盖（写透 fskv，
见 README §7）。

硬件资源（银尔达 D700Tm / Air780EPM）：
  RS4851 = 串口1（UART1，默认复用），方向控制 EN = GPIO25 → 管理链路（裸帧 RPC）
  RS4852 = 串口2（UART2，默认复用，自动方向）  → 金箭中控 Modbus RTU 从机
  RS4853 = 串口3（UART3，需 pins.setup，自动方向）→ 极空 BMS 数据源（显示屏广播 / Modbus 主站）
  NET LED = GPIO27（输出，高电平亮）
  Reload 按键 = GPIO30（输入上拉，按下接 GND）
  硬件看门狗 = GPIO28 → Air153C/Air153D 的 WTDOG（air153C_wtd：开机喂一次 +
               每 30s 一个 400ms 正脉冲；不要改用官方扩展库 exair153x_wdt，见 README §8）
  可控电压输出 = GPIO21，输出使能 = GPIO22
  供电电压采集 = ADC0（0-90V，电压 = ADC 电压 * 273300 / 3300）
]]

local M                        = {}

--=============================================================================
-- 极空 BMS 数据源协议选择（RS485-3 / UART3）
--   极空保护板对外有两套协议，本项目二选一（同一根 485 线、同一份快照）：
--     "display" = 显示屏接口广播协议：2400 8N1 单向广播，只收不发（默认）；
--     "modbus"  = RS485 Modbus 通用协议：115200 8N1 主从应答，本机做主站轮询，
--                 协议见 docs/jikong/JK-BMS-RS485.md，可读出均衡电流/循环次数/实际
--                 容量等广播协议没有的字段。
--   改这里即可切换，下游（金箭 Modbus 从机 / 4G 上报）无需改动。
--=============================================================================
M.JK_PROTOCOL                  = "display" -- "display" 显示屏广播（只收） / "modbus" RS485 主站轮询

--=============================================================================
-- 极空显示屏广播参数（JK_PROTOCOL = "display" 时生效）
--   极空保护板"显示屏接口"以 2400 8N1 持续单向广播（一轮 ≈ 2.17s），
--   本模块只接收、不发送（本板 RS485-3 是自动方向收发器，不用方向脚）。
--   轮字节数随固件/串数变化（20 串 = 364 字节；≥25 串会多发一条 0x2011 第 25 节
--   电压帧，每轮多 9 字节；报警时轮尾还会出现 0x80/0x02 蜂鸣器帧），
--   解析一律按帧内 LEN 定界，不写死轮长。
--=============================================================================
M.JK_UART_ID                   = 3     -- 串口编号（RS485-3 = 串口3）
M.JK_PIN_RX                    = 53    -- UART3_RXD（UART3 不是默认复用功能，开机由 uart_pins 复用）
M.JK_PIN_TX                    = 52    -- UART3_TXD（同上；合宙官方 demo 的 57/58 是开发板 V1.3 的脚，本板没接）
M.JK_BAUD                      = 2400  -- 显示屏协议固定 2400 8N1
M.JK_RS485_DIR_GPIO            = nil   -- RS485-3 是自动方向收发器，无方向脚（驱动按 0xFFFFFFFF 处理）
M.JK_RS485_RX_LEVEL            = 0     -- 接收方向电平
M.JK_RS485_DELAY_US            = 20000 -- 发送->接收切换延时（纯接收链路用不到）
M.JK_RX_BUFF_SIZE              = 2048  -- 串口接收缓冲区
M.JK_CELL_MIN_MV               = 1000  -- 单体电压低于该值视为"未使用的槽位"
M.JK_DATA_TIMEOUT_MS           = 15000 -- 超过该时间没有完整一轮广播则标记数据失效
M.JK_CELL_CHANGE_DEBOUNCE      = 3     -- 串数变小需连续 N 轮确认（抗单节瞬时 0mV）

--=============================================================================
-- 极空 RS485 Modbus 参数（JK_PROTOCOL = "modbus" 时生效）
--   本机是主站：反复读 0x1200（32 路单体电压）、0x1240（在位位图）、
--   0x128A（温度/电流/容量/报警汇总），每读完整一轮 = 一条采样样本。
--   串口/方向脚与显示屏广播共用（JK_UART_ID / JK_RS485_DIR_GPIO）：
--   接的是同一个 485 通道，只是波特率和收发方式不同。
--=============================================================================
M.JK_MODBUS_BAUD               = 115200 -- 极空 RS485 口固定 115200 8N1
M.JK_MODBUS_SLAVE_ADDR         = 1      -- 极空保护板从机地址（1-247，可在保护板设置）
M.JK_MODBUS_POLL_MS            = 1000   -- 轮询周期（一轮 ≈ 3 帧请求，115200 下 <50ms）
M.JK_MODBUS_TIMEOUT_MS         = 120    -- 单个请求等待应答的上限（245 字节@115200 仅需 21ms）
M.JK_MODBUS_RETRY_MS           = 2000   -- 一轮失败后的退避间隔（避免持续占用总线/日志）
M.JK_MODBUS_RX_BUFF_SIZE       = 512    -- 串口接收缓冲区（单帧最大 245 字节）

--=============================================================================
-- 金箭中控 Modbus RTU 从机链路（RS4852 / 串口2）
--   金箭中控/仪表作为主站，以 9600 8N1 周期轮询本模块（从站地址 1）。
--=============================================================================
M.JJ_UART_ID                   = 2     -- 串口编号（RS4852 = 串口2）
M.JJ_BAUD                      = 9600  -- 金箭 BMS 485 总线固定 9600 8N1
M.JJ_SLAVE_ADDR                = 1     -- 从站地址
M.JJ_RS485_DIR_GPIO            = nil   -- 无方向控制脚（自动方向收发器）；如需控制填 GPIO 号
M.JJ_RS485_RX_LEVEL            = 0     -- 接收方向电平
M.JJ_RS485_DELAY_US            = 20000 -- 发送->接收切换延时
M.JJ_RX_BUFF_SIZE              = 512   -- 串口接收缓冲区
M.JJ_MAX_FRAME_LEN             = 256   -- 单帧最大长度（超出即丢弃重新同步）
M.JJ_PARSE_MAX_FRAMES          = 8     -- 解析任务单次最多处理多少帧后主动让出（防突发占满调度器）

-- 金箭协议里"本模块无对应数据源"的寄存器取值（本地缓存，可被主站写入覆盖）
M.JJ_DEFAULT_CHARGE_TIME_MIN   = 120 -- reg1089 充电时间，极空 RCVTime 默认 2.0H
M.JJ_DEFAULT_CHARGE_TARGET_SOC = 90  -- reg1090 目标 SOC（%）
-- 快充判定倍率（C）：判定电流 mA = 倍率 × 电池容量(Ah)（容量 54Ah 时 1C = 54A）
--   1 = 1C；支持小数（0.5 = 半倍率）；0 = 只要在充电即认为快充
M.JJ_FAST_CHARGE_C_RATE        = 1   -- 快充判定倍率（C，1 = 1C = 容量 Ah 对应的电流）
M.JJ_BATTERY_TYPE              = 0   -- reg103 电池类型，0 = 锂电

-- 低温保护推导阈值（极空显示屏协议没有专用的低温报警位）
M.JJ_LOW_TEMP_CHARGE_C         = 0   -- 电池温度 <= 该值 → 低温充电保护
M.JJ_LOW_TEMP_DISCHARGE_C      = -20 -- 电池温度 <= 该值 → 低温放电保护

--=============================================================================
-- 管理链路（RS485-1 / UART1）：裸帧 RPC（状态 / 配置 / 命令 / FOTA）
--   一个统一管理服务（mgmt.lua）+ 两条传输：UART1 走自定义裸帧（二进制帧头 + JSON 载荷 +
--   二进制 BLOB 分块），MQTT 走 JSON-RPC（主题后缀 rpc）。两条传输共用同一套方法、字段与
--   错误码，**没有寄存器表/从站地址**（RS485-1 是一对一专用线，无需寻址）。
--   帧格式（双向同一套）：
--     SOF(2)=A5 5A | VER(1)=01 | TYPE(1: 1 REQ / 2 RESP / 3 EVT / 4 BLOB) | ID(1) |
--     LEN(2, 小端) | PAYLOAD | CRC16(2, 0xA001 低字节在前，覆盖 VER..PAYLOAD)
--     REQ  = {"method","params"}；RESP = {"ok":true,"result":{…}} / {"ok":false,"error":{…}}
--     EVT  = {"evt":"hello|fota","data":{…}}；BLOB = SEQ(2 LE) | LEN(2 LE) | DATA
--   协议详情与 PC 验收见 README §7 与 docs/air780epm/MGMT_RPC.md。
--   引脚：本板 RS485-1 接的是 UART1（PIN18=UART1_TXD、PIN17=UART1_RXD，默认复用功能，
--     不用 pins.setup）；方向脚 EN = GPIO25，必须显式交给 uart.setup 控制（见 README §2.1）。
--   ⚠ RS485-1 少了方向控制会完全收发不了；GPIO 高电平还必须拉到 3.3V（board.lua 开机第一件事）。
--=============================================================================
M.MGMT_UART_ID                 = 1     -- 串口编号（RS485-1 = 串口1）
M.MGMT_BAUD                    = 115200 -- 管理链路波特率（[R]，改后需重启）
M.MGMT_PIN_RX                  = nil   -- UART1 是默认复用功能（PIN17=UART1_RXD），不用 pins.setup
M.MGMT_PIN_TX                  = nil   -- UART1 是默认复用功能（PIN18=UART1_TXD），不用 pins.setup
M.MGMT_RS485_DIR               = 25    -- 方向脚：RS485-1 EN = GPIO25（必须显式控制；自动方向收发器填 nil）
M.MGMT_RS485_RX_LEVEL          = 0     -- 接收方向电平（仅 DIR 有效时用）
M.MGMT_RS485_DELAY_US          = 20000 -- 发送→接收切换延时（仅 DIR 有效时用）
M.MGMT_RX_BUFF_SIZE            = 2048  -- 串口接收缓冲区（帧 ≤ 1200 字节，留余量）
M.MGMT_MAX_FRAME_LEN           = 1200  -- 单帧最大长度（8 头 + 1024 JSON + 2 CRC，超出即丢弃重同步）
M.MGMT_MAX_PAYLOAD             = 1024  -- JSON 载荷上限（帧头 LEN 字段的理论上限是 65535）
M.MGMT_PARSE_MAX_FRAMES        = 8     -- 解析任务单次最多处理多少帧后主动让出
M.MGMT_SNAP_TTL_MS             = 1000  -- 状态快照缓存时间（同一时刻只采样一次）
M.MGMT_SLOW_REFRESH_MS         = 5000  -- 慢外设（供电电压 ADC / PN 的 IMEI）后台刷新周期
M.MGMT_DEDUP_WINDOW_MS         = 10000 -- 同一 ID + 方法在此窗口内重复到达 → 重放缓存响应（不重复执行）
M.MGMT_FOTA_ENABLE             = true  -- 允许下发 URL 触发 FOTA（固件需带 libfota2）
M.MGMT_FOTA_NET_WAIT_MS        = 300000 -- 等联网的上限（超时置 FOTA 失败）
-- 本地 FOTA（主站经 BLOB 帧 / MQTT fota.chunk 分段上传升级包，核心库 fota 直接写 Flash）：
--   不联网、不落文件；收齐后只置「成功待重启」，由主站下发现有重启命令。见 README §7.5。
M.MGMT_FOTA_LOCAL_ENABLE       = true   -- 允许本地（分段上传）FOTA
M.MGMT_FOTA_LOCAL_TIMEOUT_MS   = 60000  -- 空闲超时：这么久没有新数据块 → fota.finish(false) 取消
M.MGMT_FOTA_LOCAL_FINALIZE_MS  = 3000   -- 收齐后等核心库写 Flash（fota.isDone）的上限
M.MGMT_FOTA_LOCAL_WAIT_MS      = 3000   -- fota.init 后轮询 fota.wait() 等底层就绪的上限（超时只告警继续）
-- 管理链路 MQTT JSON-RPC：与 UART1 并列的第二条管理传输，方法/字段完全一致，通道由**主题后缀**
-- 标识（与上报通道 status 并列）：
--   下行（主站→设备）<prefix>/<device_id>/<MGMT_MQTT_SUFFIX>       载荷 = {"id",...,"method","params"}
--   上行（设备→主站）<prefix>/<device_id>/<MGMT_MQTT_SUFFIX>/resp  载荷 = {"id",...,"ok",...}
-- 与 4G 上报共用同一条 MQTT 连接（clientId = username = IMEI，同 broker 只允许一条连接），
-- 所以 MQTT_HOST 未配置时本通道自动不启动（UART1 裸帧 RPC 不受影响）。MQTT 纯请求-应答、不发事件。
M.MGMT_MQTT_ENABLE             = true  -- 启用管理 MQTT JSON-RPC（需同时配置 MQTT_HOST）
M.MGMT_MQTT_SUFFIX             = "rpc" -- 通道后缀（主题最后一段）：.../<device_id>/rpc
M.MGMT_MQTT_MAX_FRAMES         = 8     -- 单条 MQTT 载荷最多解析多少个 JSON-RPC 请求（防畸形包）

--=============================================================================
-- 【临时诊断】管理总线（RS485-1 / UART1）回环 + 心跳（排查"这一路到底通不通"时用，
--   生产必须 false）。打开后 UART1 不跑管理 RPC，改成"收到什么原样回什么 +
--   每 N ms 一行心跳"（`[mg echo] 2026-10-14 16:20:00 #12`）。引脚复用与 uart.setup
--   和 RPC 完全一致，所以它只测电气层：收到心跳 = 模组→PC 通；发什么回什么 = PC→模组 通。
--   排查完请改回 false（回环与 RPC 不能同时开：同一时刻只有一个 UART1 写入者）。
--=============================================================================
M.MGMT_ECHO_TEST               = false -- 【临时诊断】true = 回环+心跳；生产 = false
M.MGMT_ECHO_HEARTBEAT_MS       = 5000  -- 诊断心跳周期（毫秒）

--=============================================================================
-- 运行期配置持久化（fskv）：管理链路（UART1 RPC / MQTT RPC）写下的配置项重启后仍生效
--   CFG_STORE_ENABLE = false 时完全不读写 fskv（等价"每次重启都回 config.lua"）。
--   存的是「覆盖项」JSON 文本，不是整份 config：未知键/非法值在开机加载时跳过并记日志。
--   恢复出厂命令（方法 cmd.factory_reset）会清掉这份覆盖，重启后回默认值。
--=============================================================================
M.CFG_STORE_ENABLE             = true    -- 是否启用配置持久化（fskv 不可用时自动降级为不持久化）
M.CFG_STORE_FSKV_KEY           = "jjcfg" -- fskv 键名

--=============================================================================
-- MQTT 上报（4G 通道）：MessageV1 压缩上报；管理 JSON-RPC 复用这条连接（见 MGMT_MQTT_*）
--   MQTT_HOST 留空（"" 或 nil）表示本机不接 4G 上报：不建 MQTT 连接、不上报，
--   其它功能（极空采集、共享缓存、金箭从机、UART1 管理 RPC）完全不受影响。
--   鉴权：clientId = 用户名 = IMEI，密码 = HMAC-SHA256(MQTT_HMAC_SECRET, IMEI) 小写 hex；
--   MQTT_HMAC_SECRET 换值即换密钥（服务端需同步改）。
--=============================================================================
M.MQTT_HOST                    = "" -- MQTT 服务器地址；留空 = 关闭 4G 上报
M.MQTT_PORT                    = 1883
M.MQTT_IS_SSL                  = false
M.MQTT_HMAC_SECRET             = "soc" -- 密码 = HMAC-SHA256(本值, IMEI) 小写 hex
M.MQTT_TOPIC_PREFIX            = "/device" -- 上报主题 = <prefix>/<device_id>/status
M.MQTT_QOS                     = 0      -- QoS0，偶发丢包可接受
M.MQTT_KEEPALIVE_S             = 120    -- 心跳周期（秒）
M.MQTT_RECONN_MS               = 3000   -- 断线自动重连间隔
M.MQTT_CONNACK_TIMEOUT_MS      = 60000  -- 单次等待 CONNACK 上限

--=============================================================================
-- NTP 对时（上报的 BaseTimestamp / Time 列依赖）
--   联网后立即对时一次，成功后每 NTP_SYNC_INTERVAL_MS 对时一次；失败按 NTP_RETRY_MS 重试。
--   未对时前 bms_uplink 不发布（时间列不可信）。
--=============================================================================
M.NTP_SERVER                   = "ntp.aliyun.com" -- 也可填 "ntp.ntsc.ac.cn" 等
M.NTP_SYNC_TIMEOUT_MS          = 5000             -- 单次同步等待上限（毫秒）
M.NTP_SYNC_INTERVAL_MS         = 3600 * 1000      -- 成功后对时周期（默认 1 小时）
M.NTP_RETRY_MS                 = 10 * 1000        -- 失败后重试间隔（毫秒）

--=============================================================================
-- 共享采样缓存（bms_cache）与上报周期
--   缓存条数：采集适配层（bms_data）写入、金箭从机与上报只读的环形缓冲。
--   display 一轮 ≈2.17s（32 条 ≈69s）、modbus 一轮 1s（32 条 ≈32s），都能覆盖 30s 上报窗口
--   并在消费端偶发卡顿时留余量；改大会多占内存（一条 ≈32 路单体 + 20 个标量）。
--   上报周期：每 REPORT_PERIOD_MS 把「本轮新到达的真实样本」编码上报；窗口内没有新样本时
--   发 13 字节 0 样本帧（MessageV1 flags bit10），不补零/不插值/不沿用旧值冒充采集数据。
--=============================================================================
M.BMS_CACHE_SAMPLES            = 32    -- 共享缓存条数（环形，覆盖不了就计入上报 lost）
M.REPORT_PERIOD_MS             = 30000 -- 上报周期（毫秒），默认 30s

-- 压缩编码分片让出：编码是纯 CPU 计算，每编码若干列主动让出一次，
-- 避免长 CPU 段挤占串口收发（0 / nil = 不让出，离线自测用）
M.ENCODE_YIELD_MS              = 1

-- 电池串数：0 = 按极空数据自动识别（推荐）；>0 = 固定串数（覆盖自动识别）
M.CELL_COUNT                   = 0

--=============================================================================
-- 板级资源（银尔达 D700Tm）
--=============================================================================
--   引脚/外设固定（NET LED GPIO27、Reload 按键 GPIO30、看门狗 GPIO28 → Air153C/Air153D
--   的 WTDOG、可控电压输出 GPIO21+GPIO22、供电电压 ADC0、引脚复用表 pins_Air780EPM.json），
--   全部硬编码在 board.lua 顶部的 HW 表里，这里不提供开关（引脚复用表路径也写死在
--   board.lua：/luadb/pins_Air780EPM.json）。
--   要调板级参数（喂狗周期 WTD_FEED_MS、长按时长 RELOAD_REBOOT_MS、采样次数 ADC_SAMPLES
--   等）改 board.lua；换板子请同步改 board.lua 的 HW 表。

--=============================================================================
-- 日志
--=============================================================================
M.LOG_ENABLE                   = true  -- 模块业务日志总开关
M.LOG_HEX_PAYLOAD              = false -- 是否打印 MQTT 报文整体 hex（调试用）
M.LOG_RAW_FRAMES               = false -- 是否打印极空/金箭链路原始帧（调试用）
M.LOG_HEX_EVERY_N              = 20    -- 原始帧 hex 限频：每 N 帧打一次（1 = 每帧都打）
M.STATUS_REPORT_MS             = 60000 -- 周期状态日志间隔（极空数据/中控查询/上报计数汇总）

return M
