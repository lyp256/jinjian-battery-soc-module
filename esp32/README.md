# jinjian-battery-soc-module

金箭电动车第三方锂电 SOC 上报模块（ESP32-S3 + 两路 THVD1406DR RS485 + ML307-NL 4G）。

模块作为 Modbus RTU 从机挂在金箭 2.0 系列车型的 RS485 总线上，用金箭原厂 BMS
协议应答中控/仪表对电池信息的周期查询；电池实时数据来自极空（Jikong）保护板，
可通过 **UART(RS485)** 或 **BLE 蓝牙** 获取，并缓存供车端查询；同时可选通过
**ML307-NL 4G 模块（AT 指令）**把电池数据上报到远端 TCP 服务器。

## 工作原理

```
金箭中控/仪表 ──RS485 9600 8N1── THVD1406DR ── UART1 ──┐
                      (金箭 BMS 协议)                    │
                                                          ├─ ESP32-S3 ─┬─ 4G ML307-NL ── MQTT
极空保护板 ────RS485 115200 8N1── THVD1406DR ── UART0 ──┘             │    (/bms/<id>/status)
                      (极空 Modbus)                                   └─ 极空 BLE（可选）
```

4G 侧的上报链路移植自 `mqttagent`（Air780E/LuatOS）：每秒采样一条电池数据，
攒满 30 条按 **BMSStateHistory** 压缩格式打包，经 MQTT 上报给 Go agent →
VictoriaMetrics；编码与服务端字节级兼容（见「MQTT 采集上报」）。

也可不接极空通讯线：ESP32 通过 BLE 扫描并连接极空保护板（0xFFE0/0xFFE1，
JK02 24S/32S 帧格式），解析后写入同一份电池快照。

- **车端（从机，第二路 485）**：`01 03/01/06/05`，地址 `0x01`，波特率 9600，8N1；
- **极空（主机，第一路 485）**：`01 03` 读寄存器，地址可配置（默认 `0x01`），波特率 115200，8N1；
- **极空（BLE）**：服务 `0xFFE0`、特征 `0xFFE1`，300 字节累加和帧，支持 JK02 24S/32S；
- **4G**：ML307-NL，AT 指令入网后用 `AT+MIPOPEN`/`AT+MIPSEND` 承载 MQTT 上报；
- 模块约每 3 秒轮询一次极空实时数据，车端查询直接读缓存，不阻塞在 485 总线上。

## 接线

| 信号 | ESP32-S3 GPIO | 说明 |
|---|---:|---|
| 第一路 485 TX（UART0 TXD） | 2 | 接第一颗 THVD1406 的 **D**（DI） |
| 第一路 485 RX（UART0 RXD） | 1 | 接第一颗 THVD1406 的 **R**（RO） |
| 第二路 485 TX（UART1 TXD） | 4 | 接第二颗 THVD1406 的 **D**（DI） |
| 第二路 485 RX（UART1 RXD） | 3 | 接第二颗 THVD1406 的 **R**（RO） |
| 4G EN / PWR_ON | 13 | ML307-NL 的 EN（低电平有效，脉冲开机） |
| 4G UART TX（UART2 TXD） | 12 | 接 ML307-NL 的 **RX** |
| 4G UART RX（UART2 RXD） | 11 | 接 ML307-NL 的 **TX** |

THVD1406DR 是**自动方向**收发器，没有 DE 控制脚，固件按普通 UART 使用：
芯片 **RE 必须接地**（否则接收器被内部上拉关断，收不到数据）、**SHDN 接高**
（或按需接到电源控制）。因此本工程不再需要 MAX3485 那种 RTS 方向控制。

> 引脚、串口号、波特率、极空从机地址都可在 `idf.py menuconfig` 的
> `Jinjian Battery SOC Module` 菜单中修改。

## 构建与烧录

```powershell
# 激活本机 ESP-IDF 6.1 环境
. C:\Espressif\tools\Microsoft.v6.1.PowerShell_profile.ps1

idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

若用 **JTAG（OpenOCD）** 烧录（本工程 VS Code 默认就是 JTAG），烧完后建议再执行一次
完整硬复位，避免烧录后卡死、需要手动按 RST：

```powershell
python -m esptool --chip esp32s3 -p COMx -b 115200 --after hard-reset read-mac
```

项目已带 `sdkconfig.defaults`，控制台走 USB-Serial/JTAG，`UART0` 专用于极空。
固件加入蓝牙 + WiFi 后体积较大，已默认使用 **无 factory 双 OTA** 分区
（`ota_0`/`ota_1` 各 1700K），Flash 按 4MB 配置。

## 代码结构

```
main/
├── main.c            # 初始化两路 RS485 / 4G / Web 并启动任务
├── app_config.c/.h   # Web/NVS 配置加载与保存
├── bms_interface.c/.h # 通用 BMS 驱动接口与驱动管理器
├── factory_reset.c/.h # 长按 BOOT 清空配置并重启
├── modbus_rtu.c/.h   # Modbus RTU CRC、组帧、从任意偏移切帧/解析
├── jk_bms.c/.h       # 极空 UART(RS485) 驱动 + 缓存 + 单位换算
├── jk_bms_ble.c/.h   # 极空 BLE 驱动（扫描/连接/GATT/帧解析）
├── jinjian_bms.c/.h  # 金箭 BMS 从机：响应车端查询，映射缓存到金箭寄存器
├── ml307_4g.c/.h     # ML307-NL 4G：EN 上电时序 + AT 状态机 + TCP 透传
├── mqtt_client.c/.h  # 极简 MQTT 3.1.1 组包/解包（CONNECT/PUBLISH/SUBSCRIBE/PING）
├── bms_hist.c/.h     # 采样环形批 + BMSStateHistory 压缩编码（与 Go 端字节兼容）
├── bms_uplink.c/.h   # 上报任务：采样 → 编码 → MQTT 发布 → 保活/重连
├── ota_update.c/.h   # OTA 写入/校验/切换启动分区
├── web_server.c/.h   # WiFi 热点 + HTTP API + 内嵌配置页
└── log_stream.c/.h   # HTTP chunked 实时日志流（独立端口，不缓存历史）
```

```text
tools/
├── golden_gen.lua    # 用 mqttagent 参考实现生成编码黄金向量
├── golden_vectors.h  # 生成的向量（golden1/golden2/active30/fake30）
└── bms_hist_test.c   # 编码器离线比对测试（WSL/Linux: gcc tools/bms_hist_test.c main/bms_hist.c）
```

## BMS 采集抽象与驱动切换

`bms_interface.h` 定义统一快照 `bms_snapshot_t` 和驱动接口
`init/deinit/get_snapshot/set_*`。当前注册两个驱动：

- `uart`：极空 RS485（原实现）；
- `ble`：极空 BLE（JK02 24S/32S）。

`bms_manager_set_driver()` 选择当前采集通道，金箭从机和 Web 页面只读统一快照，
后续接入其它保护板只需再实现一个 `bms_driver_t`。

> **本次同时修正了 `jk_bms.c` 汇总区寄存器下标**：原实现把「字节偏移」当成
> 「寄存器下标」使用，总电压/电流/温度/报警/SOC/容量等会整体读错寄存器。
> 现在按 JK-BMS-RS485 V1.1 §6.3 取值：TempMos `0x128A`、BatVol `0x1290`、
> BatCurrent `0x1298`、TempBat1/2 `0x129C/0x129E`、Alarm `0x12A0`、
> BalanCurrent `0x12A4`、BalanSta+SOC `0x12A6`、SOCCapRemain `0x12A8`、
> SOCFullChargeCap `0x12AC`、SOCCycleCount `0x12B0`、SOCCycleCap `0x12B4`、
> SOCSOH `0x12B8`、RunTime `0x12BC`、Charge/Discharge `0x12C0`、UserAlarm2 `0x12C2`。
> 快照相应新增 `total_voltage_mv` / `charge_current_ma` / `temp*_tenths` /
> `balan_current_ma` / `capacity_remain_mah` / `cycle_capacity_mah`（UART 与 BLE
> 驱动都会填充），供 MQTT 上报使用。

## Web 配置与调试

上电后模块开启热点，默认：

| 项 | 值 |
|---|---|
| SSID | `SOC-Module-<MAC 后 6 位十六进制>`（每台模块唯一） |
| 密码 | `12345678` |
| 地址 | `http://192.168.4.1` |

页面提供：

- **电池状态**：电压、SOC、电流、温度、单体电压、PN 等实时数据；
- **配置**：采集通道（UART/BLE）、热点 SSID/密码、BLE 名称过滤/MAC（支持页面扫描选择设备）、协议版本、轮询/重连间隔；
- **调试日志**：HTTP chunked 实时日志流（默认端口 8080，只转发当前输出，不缓存历史）。
- **动态日志级别**：NONE/ERROR/WARN/INFO/DEBUG/VERBOSE，下拉选择后立即生效并保存到 NVS。
- **在线升级**：选择 `.bin` 固件上传执行 OTA，成功后自动重启到新分区。

## 恢复出厂设置

模块运行中**随时**按住 **BOOT 键**（ESP32-S3 默认 GPIO0）即可触发：

- 按下瞬间 WS2812 变**紫色常亮**作为反馈；
- 按住超过 `JINJIAN_FACTORY_RESET_HOLD_MS`（默认 3000ms）后 WS2812 变**红色常亮**，
  随后擦除 NVS 配置分区并自动重启，恢复默认配置；
- 在到达阈值前松开则取消，恢复正常状态显示。

固件里有一个常驻按键监测任务，串口会打印
`BOOT pressed / released / factory reset triggered` 日志便于排查。
注意：BOOT 键同时也是 ESP32 的下载模式引脚，若在复位（EN）时按住会进入下载模式，
与恢复出厂是两回事。按键引脚、高低电平有效、持续时间都可在 menuconfig 的
`Jinjian Battery SOC Module` 菜单调整。

## 低功耗与抢占优化

当前硬件**无法把 485/Modbus 放到低功耗核运行**：ESP32-S3 的 ULP-RISC-V
协处理器没有 UART 外设，本设计的两路 RS485（GPIO1/2、GPIO3/4）与 4G
串口（GPIO11/12）都挂在主核，需要换芯片/改板才能考虑低功耗核方案。

> 4G 上报在低功耗空闲期间**继续运行**（它是远程上报通道），只有 WiFi/Web、
> 日志流和 LED 会被关掉；如果需要 4G 也休眠，可在 Web 页面或 menuconfig
> 里把 4G 模块关掉。

固件实现了**无客户端自动低功耗**（`JINJIAN_IDLE_LOW_POWER=y`，默认开启）：

- 无 WiFi 客户端持续 `JINJIAN_IDLE_TIMEOUT_MS`（默认 30s）后自动：
  - 关闭所有 LED；
  - 挂起日志流；
  - 关闭 Web/HTTP；
  - 关闭 WiFi 射频（`esp_wifi_stop`），同时移除 WiFi 高优先级任务对
    485/BMS 时序的抢占，改善响应稳定性；
- 只保留 BMS 采集（UART/BLE）、Modbus 从机和 BOOT 按键监测；
- **短按 BOOT** 立即唤醒，恢复 WiFi/Web/日志/LED；
- 长按 BOOT（默认 3s）仍是恢复出厂。

唤醒后若又没有客户端接入，会在超时后再次进入低功耗。该功能可在 menuconfig 的
`Jinjian Battery SOC Module` 菜单关闭或调整超时。

配置保存到 NVS，提交后自动重启生效。HTTP API：

| 接口 | 方法 | 说明 |
|---|---|---|
| `/api/status` | GET | 当前电池快照与驱动状态 |
| `/api/config` | GET/POST | 读取/保存配置 |
| `/api/reboot` | POST | 重启模块 |
| `/api/logs/level` | POST | 动态调整日志级别（0..5） |
| `/api/ota` | POST | 上传固件（raw body）并 OTA 升级 |
| `/api/ble/scan/start` | POST | 启动 5 秒蓝牙扫描 |
| `/api/ble/scan/results` | GET | 获取扫描到的蓝牙设备列表 |
| `/api/4g/status` | GET | 4G + MQTT 状态（状态机、信号、IMEI/ICCID、订阅、上报计数） |
| `/api/4g/action` | POST | `{"action":"publish"｜"mqttreconnect"｜"reconnect"｜"powercycle"}` |
| `/api/4g/at` | POST | `{"cmd":"AT+CSQ","timeoutMs":3000}` 手动下发 AT 指令 |

## MQTT 采集上报（移植自 mqttagent）

上报链路从原来的 Air780E/LuatOS 固件（`mqttagent/air780e`）移植到本模块，
**主题、BMSStateHistory 编码、QoS 与 Go 服务端保持一致**，现有 agent 与看板无需改动：

```
极空 BMS（第一路 485，0x1200 实时数据区）
  │  每秒采样 1 条（JINJIAN_MQTT_SAMPLE_INTERVAL_MS）
  ▼
bms_hist：攒满 30 条一批（不重叠）→ BMSStateHistory 压缩编码
  ▼
mqtt_client：PUBLISH QoS0 → <prefix>/<device_id>/status
  ▼
ml307_4g：AT+MIPOPEN 建 TCP → AT+MIPSEND 透传 MQTT 报文
  ▼
MQTT broker（默认 399.run:1883）→ Go agent → VictoriaMetrics
```

ML307-NL 接 UART2 + EN 控制脚（默认 `EN=GPIO13`、`ESP TX=GPIO12→模块 RX`、
`ESP RX=GPIO11←模块 TX`），上电入网流程：

```
EN 脉冲上电 → AT 应答（最多 45s）→ ATE0/AT+CGSN/AT+ICCID（可选 APN 拨号）
→ AT+CPIN? 等 READY → AT+CGATT=1 + AT+CEREG? 等注册（1/5）
→ AT+MIPCLOSE=0 + AT+MIPOPEN=0,"TCP",<broker>,<port> → MQTT CONNECT + SUBSCRIBE
→ 周期 MIPSEND 发布批次；掉线自动重连，AT 无响应则按 EN 断电重开机
```

- `device_id` 优先取 4G 模块 **IMEI**（等价 mqttagent 的 `mobile.imei()`），
  未读到前先用本机 PN（`SOC-xxxxxxxxxxxx`）占位，读到后自动切换；
- 主题：上报 `<prefix>/<device>/status`，下行订阅 `<prefix>/<device>/call`，
  回复 `<prefix>/<device>/reply`（JSON-RPC 方法分发尚未移植，收到下行只记录日志，
  并显示在 Web 状态页“下行数据”一行）；
- 时间戳：取 ML307 网络时间（`AT+CCLK?`），每小时重新对齐；未对时前用开机秒数占位；
- 编码列（对应 `mqttagent/docs/bms.md`）：temp1/temp2/tempMos（0.1℃）、
  balanCurrent（mA）、batVol（mV）、batCurrent（mA）、socCycleCap（mAh）、
  socCapRemain（mAh）、time（Unix 秒）与电芯电压列（offset/spatial 两种布局取较短者）；
- 默认 1 秒 × 30 条 ≈ 30 秒一包、QoS0，与 mqttagent 默认参数一致；
- 保活：每 keepalive/2 秒发 PINGREQ；socket 断开或连续 3 次发布失败即重连；
- 编码器已用 mqttagent 的黄金向量做字节级回归（见 `tools/`）：
  golden1 41B、golden2 30B、active30 421B、fake30 420B 全部一致。

Web 页面“4G / MQTT 上报”卡片提供：4G 状态机、信号强度、IMEI/ICCID、MQTT 连接与
订阅状态、采样/批次成功失败计数、最近报文大小与时间、最近错误，并可在线修改
broker/端口/账号/主题前缀/采样周期/批量/保活，或直接下发 AT 指令调试
（例如 `AT+CSQ`、`AT+CEREG?`、`AT+MIPSTATE=0`），也可以“立即上报一批”。

日志实时流使用独立端口的轻量 HTTP 服务器：

- `GET http://192.168.4.1:8080/api/logs/stream`，`Transfer-Encoding: chunked`；
- 每行日志即时组装为 chunk 推送给已连接页面；
- 日志流只保留最新一个客户端（`JINJIAN_MAX_LOG_STREAMS=1`），新客户端连接时会直接关闭旧连接；
- 只处理实时输出，不缓存历史，客户端断开即释放连接。

## LED 状态指示

本板（ESP32-S3）只有一颗 **WS2812 RGB LED（数据脚 GPIO48）**，没有单色状态 LED。
Web 页面“配置”里可勾选“启用 LED 指示灯”，保存后立即生效；关闭后 LED 熄灭。
固件仍保留单色 LED 支持，把 `JINJIAN_STATUS_LED_GPIO` 配成实际引脚即可启用
（本板默认 `-1`，表示未接）。

WS2812 负责事件、故障、OTA、BOOT 等全部显示：

| 事件 / 状态 | WS2812 RGB |
|---|---|
| 收到车端 485 查询 | 蓝色闪烁（成功双闪，失败单闪） |
| 读取一次 BMS 数据 | 绿色闪烁（成功双闪，失败单闪） |
| 4G 上报一次 | 青色闪烁（成功双闪，失败单闪） |
| 开机 / 等待 BMS 数据 | 灭 |
| 正常运行且数据新鲜 | 灭 |
| BMS 轮询失败（故障） | 橙色闪烁 |
| BMS 驱动不可用（故障） | 红色闪烁 |
| 多个故障同时存在 | 各故障色轮流切换 |
| OTA 升级 | 常亮黄绿交替 |
| 按住 BOOT | 紫色常亮 |
| 触发恢复出厂 | 红色常亮 |
| LED 总开关关闭 | 灭 |

事件说明：485 收到一次查询 → 蓝色单闪（150ms）；BMS 读取一次 → 绿色单闪；
4G 上报一次 → 青色单闪。**成功=同色双闪（150ms 亮 + 50ms 灭 + 100ms 亮），
失败=单闪**；接上单色 LED 后，成功改由单色 LED 点亮 150ms 提示，RGB 只闪一次。
事件较多时按到达顺序排队显示。
故障状态不是常亮，而是 250ms 亮 / 250ms 灭闪烁，多个故障时轮流切换各故障颜色。

电源 LED：如果它是接在某个 GPIO 上的，在 menuconfig 里把
`JINJIAN_PWR_LED_GPIO` 配成对应引脚后，固件会保持常亮（关闭 LED 总开关时熄灭）；
如果是硬接电源的，则无法用固件控制，出现闪烁通常是供电或 USB 接触问题。
引脚号和电平极性均可在 menuconfig 的 `Jinjian Battery SOC Module` 菜单调整。

## 在线 OTA

- 分区布局：无 `factory`，`ota_0`/`ota_1` 各 1700K + `otadata`，4MB Flash；
- Web 页面“在线升级”选择 `build/jinjian-battery-soc-module.bin` 上传；
- 服务器按 4KB 缓冲流式写入空闲 OTA 分区，`esp_ota_end` 校验通过后设置启动分区并重启；
- 首次烧录需要同时写入 `ota_data_initial.bin`，具体命令以 `idf.py flash` 输出为准。

## 金箭寄存器 ← 极空数据映射

| 金箭寄存器 | 内容 | 极空来源 | 换算 |
|---|---:|---|---|
| 0 | 总电压 | `0x1290` BatVol (mV) | `/10` → 0.01V |
| 1 | 电芯串数 | `0x1240` CellSta 位统计 | popcount |
| 2 | SOC | `0x12A6` SOC（低字节） | clamp 0–100 |
| 3 | 容量 | `0x12AC` FullChargeCap (mAh) | `/1000` |
| 4 | 短路保护 | `0x12A0` 报警位 7/14 | 充电/放电短路 |
| 5 | 充电电流 | `0x1298` BatCurrent (mA) | `/10` → 0.01A |
| 6/7 | 电池温度 2/1 | `0x129E`/`0x129C` (0.1℃) | `/10` |
| 8 | 板温 | `0x128A` TempMos (0.1℃) | `/10` |
| 9–32 | 单体电压 | `0x1200` 起 32 路 (mV) | 直传 |
| 103 | 电池类型 | — | 固定 0（锂电） |
| 104 | 循环次数 | `0x12B0` | clamp 0–65535 |
| 110 | 均衡状态 | `0x12A6` 高字节 | 非 0 → 1 |
| 113 | 标称容量 | `0x12AC` | `/1000` |
| 1000–1007 | PN | 模块自身 PN（由本机 MAC 派生，形如 `SOC-9888E072BD78`，16 ASCII） | 生成 |
| 1016/1017 | 版本 | `0x1318` SoftwareVersion (8 ASCII) | 解析 a.b.c.d |
| 1089 | 充电时间 | `0x1104` RCVTime（0.1H） | ×6 → 分钟 |
| 1090 | 目标 SOC | —（极空无此配置） | 本地缓存，默认 90% |

车端开关量：

| 金箭开关量 | 内容 | 极空来源 |
|---|---:|---|
| 4 | 短路保护 | 报警位 7/14 |
| 5 | 过温充电保护 | 报警位 8 |
| 6 | 过温放电保护 | 报警位 15 |
| 7 | 低温充电保护 | 报警位 9 或温度低于阈值 |
| 8 | 低温放电保护 | 温度低于阈值（极空无专用位） |
| 63 | 快充状态 | 充电中且电流 ≥ 阈值（默认 0） |

## 已支持的应答

| 请求 | 说明 |
|---|---|
| `01 03 0000 0009` | 周期轮询 0–8 |
| `01 03 0000 0021` | 详情 0–32（含单体电压） |
| `01 03 0067 000B` | 电池状态 103–113 |
| `01 03 03E8 0008` | PN |
| `01 03 03F8 0002` | 版本 |
| `01 03 0441 0002` | 充电设置 |
| `01 01 003F 0001` | 快充状态 |
| `01 01 0004 0005` | 保护状态 |
| `01 06 0441/0442` | 写充电时间/目标 SOC（更新本地缓存） |
| `01 05 0040 FF00` | 结束快充（更新本地状态） |
