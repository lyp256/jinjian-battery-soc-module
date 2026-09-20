# jinjian-battery-soc-module

金箭电动车第三方锂电 SOC 上报模块（ESP32-C6 + MAX3485）。

模块作为 Modbus RTU 从机挂在金箭 2.0 系列车型的 RS485 总线上，用金箭原厂 BMS
协议应答中控/仪表对电池信息的周期查询；电池实时数据来自极空（Jikong）保护板，
可通过 **UART(RS485)** 或 **BLE 蓝牙** 获取，并缓存供车端查询。

## 工作原理

```
金箭中控/仪表 ──RS485 9600 8N1── MAX3485 ── UART1 ── ESP32-C6 ── UART0 ── 极空保护板
                      (金箭 BMS 协议)                    (极空 115200 Modbus)
```

也可不接极空通讯线：ESP32 通过 BLE 扫描并连接极空保护板（0xFFE0/0xFFE1，
JK02 24S/32S 帧格式），解析后写入同一份电池快照。

- **车端（从机）**：`01 03/01/06/05`，地址 `0x01`，波特率 9600，8N1；
- **极空（主机）**：`01 03` 读寄存器，地址可配置（默认 `0x01`），波特率 115200，8N1；
- **极空（BLE）**：服务 `0xFFE0`、特征 `0xFFE1`，300 字节累加和帧，支持 JK02 24S/32S；
- 模块约每 3 秒轮询一次极空实时数据，车端查询直接读缓存，不阻塞在 485 总线上。

## 接线

| 信号 | ESP32-C6 GPIO | 说明 |
|---|---:|---|
| 极空 UART TX | 4 | 接极空保护板 485 模块 RX（若保护板是 485 需自备转 TTL） |
| 极空 UART RX | 5 | 接极空保护板 485 模块 TX |
| 车端 485 TX | 16 | 接 MAX3485 DI |
| 车端 485 RX | 17 | 接 MAX3485 RO |
| MAX3485 DE/RE | 6 | DE 与 RE 短接后接 GPIO6（RTS 自动控制方向） |

> 引脚、串口号、波特率、极空从机地址都可在 `idf.py menuconfig` 的
> `Jinjian Battery SOC Module` 菜单中修改。

## 构建与烧录

```powershell
# 激活本机 ESP-IDF 6.1 环境
. C:\Espressif\tools\Microsoft.v6.1.PowerShell_profile.ps1

idf.py set-target esp32c6
idf.py build
idf.py -p COMx flash monitor
```

项目已带 `sdkconfig.defaults`，控制台走 USB-Serial/JTAG，`UART0` 专用于极空。
固件加入蓝牙 + WiFi 后体积较大，已默认使用 **无 factory 双 OTA** 分区
（`ota_0`/`ota_1` 各 1700K），Flash 按 4MB 配置。

## 代码结构

```
main/
├── main.c            # 初始化两个 UART 并启动任务
├── app_config.c/.h   # Web/NVS 配置加载与保存
├── bms_interface.c/.h # 通用 BMS 驱动接口与驱动管理器
├── factory_reset.c/.h # 长按 BOOT 清空配置并重启
├── modbus_rtu.c/.h   # Modbus RTU CRC、组帧、从任意偏移切帧/解析
├── jk_bms.c/.h       # 极空 UART(RS485) 驱动 + 缓存 + 单位换算
├── jk_bms_ble.c/.h   # 极空 BLE 驱动（扫描/连接/GATT/帧解析）
├── jinjian_bms.c/.h  # 金箭 BMS 从机：响应车端查询，映射缓存到金箭寄存器
├── ota_update.c/.h   # OTA 写入/校验/切换启动分区
├── web_server.c/.h   # WiFi 热点 + HTTP API + 内嵌配置页
└── log_stream.c/.h   # HTTP chunked 实时日志流（独立端口，不缓存历史）
```

## BMS 采集抽象与驱动切换

`bms_interface.h` 定义统一快照 `bms_snapshot_t` 和驱动接口
`init/deinit/get_snapshot/set_*`。当前注册两个驱动：

- `uart`：极空 RS485（原实现）；
- `ble`：极空 BLE（JK02 24S/32S）。

`bms_manager_set_driver()` 选择当前采集通道，金箭从机和 Web 页面只读统一快照，
后续接入其它保护板只需再实现一个 `bms_driver_t`。

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

模块运行中**随时**按住 **BOOT 键**（默认 GPIO9）即可触发：

- 按下瞬间 WS2812 变**紫色常亮**作为反馈；
- 按住超过 `JINJIAN_FACTORY_RESET_HOLD_MS`（默认 3000ms）后 WS2812 变**红色常亮**，
  随后擦除 NVS 配置分区并自动重启，恢复默认配置；
- 在到达阈值前松开则取消，恢复正常状态显示。

固件里有一个常驻按键监测任务，串口会打印
`BOOT pressed / released / factory reset triggered` 日志便于排查。
注意：BOOT 键同时也是 ESP32 的下载模式引脚，若在复位（EN）时按住会进入下载模式，
与恢复出厂是两回事。按键引脚、高低电平有效、持续时间都可在 menuconfig 的
`Jinjian Battery SOC Module` 菜单调整。

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

日志实时流使用独立端口的轻量 HTTP 服务器：

- `GET http://192.168.4.1:8080/api/logs/stream`，`Transfer-Encoding: chunked`；
- 每行日志即时组装为 chunk 推送给已连接页面；
- 日志流只保留最新一个客户端（`JINJIAN_MAX_LOG_STREAMS=1`），新客户端连接时会直接关闭旧连接；
- 只处理实时输出，不缓存历史，客户端断开即释放连接。

## LED 状态指示

板上有三颗 LED：**电源 LED**（默认硬接电源）、**GPIO15 单色状态 LED**、
**GPIO8 WS2812 RGB LED**。Web 页面“配置”里可勾选“启用 LED 指示灯”，
保存后立即生效；关闭后所有 LED 熄灭。

两颗状态 LED 分工显示：GPIO15 单色 LED **只作为“任务成功”指示灯**（平时熄灭，
任务成功时点亮一下）；RGB 负责事件、故障、OTA、BOOT 等场景显示：

| 事件 / 状态 | GPIO15 单色 LED | WS2812 RGB |
|---|---|---|
| 收到车端 485 查询 | 成功回应后亮 150ms | 蓝色单闪 |
| 读取一次 BMS 数据 | 成功拿到数据后亮 150ms | 绿色单闪 |
| 开机 / 等待 BMS 数据 | 灭 | 灭 |
| 正常运行且数据新鲜 | 灭 | 灭 |
| BMS 轮询失败（故障） | 灭 | 橙色闪烁 |
| BMS 驱动不可用（故障） | 灭 | 红色闪烁 |
| 多个故障同时存在 | 灭 | 各故障色轮流切换 |
| OTA 升级 | 灭 | 常亮黄绿交替 |
| 按住 BOOT | 灭 | 紫色常亮 |
| 触发恢复出厂 | 灭 | 红色常亮 |
| LED 总开关关闭 | 灭 | 灭 |

事件说明：485 收到一次查询 → WS2812 蓝色单闪（150ms），若正常回应则 GPIO15
点亮 150ms，失败不亮；BMS 读取一次 → WS2812 绿色单闪，成功拿到数据则 GPIO15
点亮 150ms，失败不亮。事件较多时按到达顺序排队显示。
故障状态不再常亮，而是 250ms 亮 / 250ms 灭闪烁，多个故障时轮流切换各故障颜色，
故障、OTA、BOOT 状态只由 RGB 表达，GPIO15 保持熄灭。

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
| 1000–1007 | PN | `0x1300` ManufacturerDeviceID (16 ASCII) | 直传 |
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
