# 电池 SOC 模块（银尔达 D700Tm / Air780EPM）

基于银尔达 D700Tm（Air780EPM 方案）的 LuatOS Lua 工程：把极空（JK）保护板的电池数据
转换成金箭电摩中控可读的 Modbus 从机数据，并通过 4G 以 MQTT 压缩上报。

1. **采集**：从极空 BMS 保护板 RS485 接口取电池数据，两种协议**在配置里二选一**
   （`JK_PROTOCOL`）：显示屏协议广播（2400 8N1，只收不发）或极空 RS485 Modbus 通用协议
   （115200 8N1，本机做主站轮询），协议见 [极空显示屏协议.md](../docs/jikong/极空显示屏协议.md)
   与 [JK-BMS-RS485.md](../docs/jikong/JK-BMS-RS485.md)；
2. **协议转换**：作为 Modbus RTU 从设备（9600 8N1，从站地址 1）响应金箭中控的 BMS 查询，
   协议见 [金箭协议](../docs/jinjian)；
3. **上报**：通过 4G 连接 MQTT 服务器，每 30 秒把「这一窗口新到达的真实样本」压缩上报
   （消息格式 `MessageV1`：`0xAA` + 消息级字段 + 列式 Delta + ZigZag + Varint + 游程压缩；
   窗口内没有新样本时只发 0 样本帧，绝不补零/插值），采集时间粒度以极空一轮为单位；
4. **管理**：RS485-1（UART1，**115200** 8N1，点对点、无从站地址）跑**管理链路**：
   一个统一管理服务 + 两条传输 —— UART1 走**自定义裸帧 RPC**（二进制帧头 + JSON 载荷 + BLOB 分块），
   MQTT 走 **JSON-RPC**（主题后缀 `rpc`）；读状态（`status.get`）、读/写配置（`config.get`/`config.set`，
   写透持久化到 fskv）、命令（`cmd.reboot`/`cmd.factory_reset`）与两条 FOTA 通道（`fota.url` /
   `fota.begin`+`fota.chunk`），帧格式、方法表与验收步骤见 §7。

> 开发参考：`Air780EPM v2026.03.24.12` 的文档与 demo。

## 1. 数据流与文件结构

采集与消费彻底解耦：协议解析（`jk_display`/`jk_modbus`）只产出各自的快照，`bms_data` 把两套
协议**归一成同一份「规范样本」**写进共享缓存，金箭从机与 4G 上报都**只读缓存**、互不干扰：

```
极空保护板 ──RS485-3（UART3）──► jk.lua（按 config.JK_PROTOCOL 二选一）──"JK_ROUND"/一轮一条──┐
   │  display: 2400 8N1 广播（只收） ──► jk_display.lua  解析/自检/失效标记           │
   └─ modbus : 115200 8N1 主站轮询 ──► jk_modbus.lua   组帧/解析/失效标记             ▼
                                            bms_data.lua 归一（只做字段/单位映射，不插值）
                                                    │ push（缓存分配 seq + 原始采集 ts）
                                                    ▼
                                            bms_cache.lua 环形缓存（BMS_CACHE_SAMPLES = 32）
                                              │                             │
                        只读 latest()         │                             │  只读 since(上报游标)
金箭中控/仪表 ──RS485-2（UART2）──► jinjian_slave.lua ◄─┘                   ▼
   （Modbus RTU 主站 9600 8N1，01 03/01/05/06）              bms_uplink.lua ──每 30s──► bms_codec.lua
                                                             （窗口内真实样本 / MessageV1 压缩编码）
                                                                    └──4G──► <prefix>/<IMEI>/status
UART1/RS485-1 ──115200 8N1 裸帧 RPC──► mgmt_uart.lua ─┐
                                          （SOF/VER/TYPE/ID/LEN/CRC + JSON / BLOB） │
MQTT（通道 rpc，JSON-RPC）────────────► mgmt_mqtt.lua ─┴─► mgmt.lua（统一管理服务）
   └──4G──► <prefix>/<IMEI>/rpc(+/resp)  与上报共用同一条 MQTT 连接（clientId=IMEI）
                        方法分发 / 状态快照 / 配置白名单 / 命令 / FOTA / ID 去重 ──► cfg_store.lua（fskv）
```

缓存为空（保护板掉线/还没收到一轮）时，金箭从机按全 0 应答、4G 上报发 13 字节左右的 0 样本帧，
**绝不补零、插值或用旧值冒充采集数据**。

三条链路互不影响：极空链路（广播只收，或 Modbus 每秒一次、单轮 <50ms 的轮询）与金箭从机
（只应答主站请求）全部运行在 `sys` 协作式任务中，不用阻塞式等待；第三路 UART1 是**管理总线**
（115200 8N1，点对点，只应答管理主站的请求），与上面两条链路物理/逻辑独立，管理主站不来问就
一个字节都不发。管理侧有**两条传输**（UART1 的裸帧 RPC 与 MQTT 的 JSON-RPC，§7.7），它们共用
同一个 `mgmt` 服务、各答各的请求。管理写入的配置**写透**到 fskv（`cfg_store`），标 `[R]` 的项
重启后生效。

| 文件 | 作用 |
|---|---|
| `main.lua` | 入口：PROJECT/VERSION、加载模块、等 4G 拨号后发布 `net_ready`、周期汇总各链路的收发计数状态日志 |
| `config.lua` | 全部可调参数（串口/GPIO/波特率/MQTT/批次/阈值/日志开关） |
| `board.lua` | 板级资源：引脚复用（开机加载 `pins_Air780EPM.json`）、可控电压输出、NET LED、Reload 按键、硬件看门狗、供电电压、设备 ID/PN |
| `pins_Air780EPM.json` | LuatIO 管脚表（银尔达 D700Tm）：PIN52/53 = UART3_TXD/RXD、PIN106 = GPIO25（RS485-1 方向）、PIN78 = GPIO28（看门狗）、PIN16 = GPIO27（NET LED）…；随脚本打包进 `/luadb/`，固件开机自动加载（见 §2.2） |
| `air153C_wtd.lua` | 看门狗库（官方老库，**`board.lua` 实际用它喂狗**）：GPIO28 每 30s 一个 400ms 正脉冲，开机先喂一次。实测本板只有这种写法能喂动 Air153C，官方默认扩展库 `exair153x_wdt` 打"喂狗完成"但芯片照旧按自身超时复位（见 §8） |
| `jk.lua` | 极空数据源入口：按 `config.JK_PROTOCOL` 转发到下面两个实现之一（接口完全一致） |
| `jk_display.lua` | 极空显示屏协议接收解析（帧定界、三类数据块、自检、数据失效标记） |
| `jk_modbus.lua` | 极空 RS485 Modbus 主站（03H 读寄存器、CRC、轮询调度、单位换算、失效标记） |
| `bms_data.lua` | 采集适配层：`JK_ROUND` 快照 → 规范样本（字段/单位映射、固定 32 路单体槽位）→ `bms_cache`；`module_uptime_s()` 供上报 |
| `bms_cache.lua` | 共享采样缓存：32 条环形缓冲，`push/latest/count/history/since/stats`，采集与消费解耦的唯一交汇点 |
| `jinjian_slave.lua` | 金箭 Modbus RTU 从机（帧扫描、寄存器/开关量映射、写入、异常码）；数据只读 `bms_cache.latest()`，缓存为空按全 0 应答 |
| `bms_uplink.lua` | 每 30s 从缓存取「窗口内新到达的真实样本」→ MessageV1 压缩 → MQTT 发布；窗口为空发 0 样本帧；`MQTT_HOST` 为空时整条 4G 链路不启动 |
| `bms_codec.lua` | MessageV1 压缩编码器（0xAA + version + 消息级字段 + 列式 Delta+ZigZag+Varint+游程，自动选列模式与电芯布局，含 0 样本空帧） |
| `sha256.lua` / `hmac.lua` | 纯 Lua SHA-256 / HMAC-SHA256：MQTT 密码 = `HMAC-SHA256("soc", IMEI)` 小写 hex，不依赖固件 crypto 库 |
| `ntp_sync.lua` | NTP 对时（上报的 `BaseTimestamp`/`Time` 依赖；联网后立即同步，之后每小时一次，服务器/周期在 `config.lua`） |
| `mgmt.lua` | 统一管理服务（传输无关）：方法分发（`sys.ping`/`status.get`/`config.get`/`config.set`/`cmd.*`/`fota.*`）、状态快照（TTL + 慢外设后台刷新）、配置读写（白名单）、命令、FOTA 编排、ID 去重、`MGMT_EVT` 事件（见 §7） |
| `mgmt_uart.lua` | UART1（RS485-1）裸帧 RPC 传输：115200 8N1、EN=GPIO25；SOF/VER/TYPE/ID/LEN/CRC16 扫帧与重同步、JSON 载荷 + BLOB 分块、`evt.hello`/`evt.fota` 事件；回调只搬字节 + 独立解析任务 |
| `uart_pins.lua` | UART 引脚复用兜底：把配置的 RX/TX 脚设成 `UART<n>` 功能（极空链路的 UART3/PIN53/PIN52 需要；UART1/UART2 默认复用不用配） |
| `mgmt_mqtt.lua` | 管理 MQTT JSON-RPC 传输：与 UART1 并列的第二条管理通道，**通道 = 主题后缀**（默认 `rpc`：下行 `<prefix>/<device>/rpc`、上行 `.../rpc/resp`），与 4G 上报**共用同一条 MQTT 连接**（clientId=IMEI 只允许一条）；`MGMT_MQTT_ENABLE` 默认 true，`MQTT_HOST` 未配置时自动不启动（见 §7.7） |
| `cfg_store.lua` | 配置持久化：开机把 fskv 里的覆盖项按白名单写回 `config`（早于业务模块加载）、写透落盘、恢复出厂清覆盖；fskv 不可用时降级为不持久化 |
| `fota_mgr.lua` | FOTA 升级管理（**原名 `fota.lua`**，改名是为了避开 LuatOS 核心库里的全局 `fota`；同名会互相遮蔽）：① URL 通道封装 `libfota2.request`（联网下载）；② 本地通道用核心库 `fota.init/run/isDone/finish` 把主站经管理链路分段上传的升级包**流式写进 Flash**（不联网、不落文件、不依赖 `libfota2`）。两条通道互斥，成功都只置「成功待重启」+ 状态 2，由主站下发现重启命令（见 §7.4 / §7.5） |
| `tools/mgmt_probe.ps1` | PC 侧管理链路探针（裸帧 RPC 主站）：自动组帧/算 CRC16、`ping`/`status`/`config`/`set`/`reboot`/`factory`/`url` FOTA/`localfota`（BLOB 分块）/`abort`/`raw`/`echo`/`listen`（见 §7.6） |
| `tools/selftest.lua` | 离线自测（PC 端 Lua 5.4）：MessageV1 黄金向量、SHA/HMAC、缓存、适配层、广播解析、Modbus 主站、从机应答、端到端上报，以及管理帧编解码/方法分发/配置持久化/FOTA/两条传输/整链路 smoke |
| `../hwsim.lua` | 仓库根目录的虚拟时钟仿真：真跑 `jk_display → bms_data → bms_cache → bms_uplink`，观察「30s 窗口真实样本」与「掉线后 0 样本空帧」，并可 `DUMP=` 导出 payload 给服务端解码器核对 |

> `config.lua` 不入库（`air780epm/.gitignore` 已忽略）；仓库中的 `config.lua.tpl` 是脱敏模板。
> 首次使用（或模板更新后）请复制为 `config.lua`（Windows: `copy config.lua.tpl config.lua`）再填本地信息，
> 真实 MQTT 服务器/账号/密码只写在 `config.lua`，禁止提交到仓库。

## 2. 硬件资源与配置

[银尔达 D700Tm 官方文档](https://yinerda.yuque.com/yt1fh6/bihsy5/wlbazx2as7o46v0g#oIh6t)

| D700Tm 资源 | GPIO/接口 | 本项目用途与取值（板级资源硬编码在 `board.lua` 的 HW 表，其余配置在 `config.lua`） |
| :--- | :--- | :--- |
| 引脚复用 | `pins_Air780EPM.json` | 合宙模组的管脚是"复用"的：某个管脚用哪个功能由**开机加载的 `pins_$model.json`** 决定。本工程放的是 LuatIO 生成的 D700Tm 管脚表 `air780epm/pins_Air780EPM.json`，随脚本打包进 `/luadb/`、固件开机自动加载；`board.lua` 开机再按固定路径 `pins.loadjson` 一次并打印 `引脚复用配置已加载 …`（见 §2.2） |
| RS485-1 | 串口1 | **管理链路**（`MGMT_*`，**115200** 8N1、点对点无从站地址，见 §7）：裸帧 RPC（`mgmt_uart.lua`）+ 统一管理服务（`mgmt.lua`）——读状态（`status.get`）、读写配置（`config.get`/`config.set`，写透持久化）、命令（重启/FOTA/恢复出厂）。UART1 是默认复用功能（PIN18=UART1_TXD、PIN17=UART1_RXD），不用 `pins.setup` |
| RS485-1 EN | GPIO25 | 管理链路方向脚：`MGMT_RS485_DIR=25`、`MGMT_RS485_RX_LEVEL=0`，由串口驱动按收发自动切换（本板 RS485-1 是带方向控制的收发器，**必须显式控制**，见 §2.1） |
| RS485-2 | 串口2 | 金箭中控 Modbus RTU 从机（9600 8N1）：`JJ_UART_ID=2`、`JJ_BAUD=9600`、`JJ_SLAVE_ADDR=1`（自动方向收发器，无方向脚） |
| RS485-3 | 串口3 | 极空 BMS 数据源（`JK_PROTOCOL` 二选一）：广播只收 2400 8N1（`JK_UART_ID=3`、`JK_BAUD=2400`）或 Modbus 主站 115200 8N1（`JK_MODBUS_BAUD=115200`、`JK_MODBUS_SLAVE_ADDR=1`）。**本板 RS485-3 接的是 UART3 的 B 组：PIN53=UART3_RXD、PIN52=UART3_TXD**（收发器自动方向，无 EN）。UART3 不是默认复用功能，模块开机自动 `uart_pins` 复用；引脚组与排查见 §7.8 |
| NET LED | GPIO27 | 状态指示（输出，高电平亮）：注册网络快闪 → 拨号成功慢闪 → MQTT 连上常亮 |
| Reload 按键 | GPIO30 | 输入上拉，按下接 GND；长按 3s 重启模组 |
| 硬件看门狗 | GPIO28 | 外置 Air153C/Air153D 的 WTDOG。**GPIO 高电平必须是 3.3V**（`pm.ioVol(pm.IOVOL_ALL_GPIO, 3300)`，D700Tm 资源表要求；26.04 NOCAM 固件默认只有 1.8V，1.8V 喂不动芯片）。喂狗用工程自带的 `air153C_wtd`：开机 `init(28)` 并立刻喂一次，之后独立任务每 30s 一个 400ms 正脉冲（`board.lua` 是 GPIO28 的唯一写入者；改回 `exair153x_wdt` 会随机重启，见 §8）；复位原因用管理链路读（`status.get` 的 `reset_reason`，见 §7.3），`board.lua` 不再开机打印 |
| 可控电压输出 | GPIO21 | 输出供电电压（最大 1A），开机默认打开 |
| 输出使能脚 | GPIO22 | 输出时序：使能高 → 设置输出 → 延时 5ms → 使能低；模组重启后输出状态保持 |
| 供电电压采集 | ADC0 | 0–90V，`电压 = ADC 电压 × 273300/3300`，每 60s 打印 |
| 外置卡 / 贴片卡 | SIM2 / SIM1 | 4G 上网（保持模组当前配置，可用 `mobile.simid()` 切换） |
| USB 接口 / Boot 按键 | — | 下载程序；开机前按住 Boot 强制进入 USB 下载模式 |

> 三条链路各自独立：换线只需改 `JK_UART_ID` / `JJ_UART_ID` / `MGMT_UART_ID`，
> 需要 `pins.setup` 的 UART 在 `*_PIN_RX` / `*_PIN_TX` 填引脚（自动方向收发器方向脚填 `nil`）。
> 本工程当前分工：**UART1 = 管理链路、UART2 = 金箭 Modbus、UART3 = 极空 BMS**。
> 广播模式下极空链路只接收，不会向广播总线发数据；Modbus 模式下方向脚由驱动自动控制
> （本板 RS485-3 是自动方向收发器，填 `nil` 即可）。

### 2.1 三路 RS485/UART 速查与三条硬约束（本板实测结论）

| 板子端口 | 串口 | 模组引脚 | 方向控制 | 总线上挂什么 |
|---|---|---|---|---|
| **RS485-1** | UART1 | PIN18 = UART1_TXD、PIN17 = UART1_RXD（**默认复用，不用配**） | **EN = GPIO25（必须显式控制）** | 管理主站：读写本机状态/配置/命令（裸帧 RPC 115200，点对点，§7） |
| **RS485-2** | UART2 | PIN28 = UART2_RXD、PIN29 = UART2_TXD（**默认复用**） | 无（收发器自动方向） | 金箭中控 Modbus 从机 9600 |
| **RS485-3** | UART3 | **PIN53 = UART3_RXD、PIN52 = UART3_TXD（必须显式复用）** | 无（自动方向） | 极空 BMS：广播 2400（只收）或 Modbus 主站 115200（轮询） |

三条硬约束，少一条现象都很迷惑：

1. **GPIO 高电平拉到 3.3V**：`pm.ioVol(pm.IOVOL_ALL_GPIO, 3300)`（`board.lua` 开机第一件事，硬编码）。
   不设时 GPIO 高电平默认只有 1.8V，RS485-1 的 EN 与 UART 的逻辑"1"都可能不被 485 收发器认成高电平；
   开机日志可见 `D/pm io vol set : 0 3300`。
2. **`main.lua` 结尾必须 `sys.run()`**：少了它 LuatOS 认为脚本已跑完（`E/main Lua VM exit!! reboot in 15000ms`），
   现象是**每 15 秒重启一次**、日志停在开机 banner 那一行、串口一个字节都收不到。
3. **引脚复用表必须随脚本下载**：极空链路的 UART3 不是默认复用功能，本板接的是
   **PIN53 = UART3_RXD、PIN52 = UART3_TXD**（合宙官方 demo 的 `pins_Air780EPM.json` 用的是
   PIN57/58 —— 那是开发板 V1.3 的脚，D700Tm 没接出去）；管理链路的 RS485-1 还要交 GPIO25
   方向脚给串口驱动 —— 引脚组与判据见 §7.8，表怎么下发见 §2.2。

### 2.2 引脚复用：`pins_Air780EPM.json`（LuatIO 生成的管脚表）

合宙模组的管脚是**复用**的（同一个脚既能当 GPIO27、也能当 CAN_TXD…），"用哪一种"由**开机
初始化**决定，不是 `gpio.setup(27)` 说了算：固件启动时自动加载 `/luadb/pins_$model.json`，
按表把每个脚设成指定功能（`pins` 是核心库，脚本里**不需要** `require`）。

| 项 | 说明 |
|---|---|
| 文件 | `air780epm/pins_Air780EPM.json`：LuatIO 生成的 D700Tm 管脚表（`"model": "Air780EPM"`） |
| 下发 | 在 Luatools 的"脚本和资源列表"里带上它，跟 `.lua` 一起打包进 `/luadb/`（不是放在 PC 上就行） |
| 自动加载 | 固件开机自动加载 `pins_$model.json`；加载成功就不会再有 `W/pins … not exist!!` |
| 代码兜底 | `board.lua` 开机显式 `pins.loadjson("/luadb/pins_Air780EPM.json")`（路径硬编码）：成功打 `引脚复用配置已加载 … code 0`，失败明确告警（`pins.loadjson` 成功返回 `true, 0`，失败返回 `nil` + 负错误码）。不做候选名回退——表在工程里且有固定文件名，路径只认这一个最省事 |
| 开关 | 无（路径固定写在 `board.lua`）。要排查"配成功却没信号"，临时加一句 `pins.debug(true)` 打印每个脚落到哪组 PAD |
| 表里几条要紧的 | PIN52/53 = UART3_TXD/RXD（RS485-3，极空链路）、PIN106 = GPIO25（RS485-1 EN，管理链路方向）、PIN78 = GPIO28（看门狗）、PIN16 = GPIO27（NET LED）、PIN31 = GPIO30（Reload）、PIN19 = GPIO22 / PIN107 = GPIO21（可控电源） |

> 这张表决定了复用脚落在哪个**物理脚**上，例如 UART3 的两个信号必须配在 PIN52/53
> （`pins.setup(53, "UART3_RX")`），RS485-1 的方向脚 GPIO25 在表里是 `[106, "GPIO25"]`。
> `uart_pins.lua` 里那句 `pins.setup`（极空链路：PIN53=RX / PIN52=TX）只是兜底：
> `UART3_RX`/`UART3_RXD` 两种功能名都会试，主路径是这张表；两件事都做也不冲突。

## 3. 极空数据源协议：显示屏广播 / RS485 Modbus（配置切换）

极空保护板对外有两套协议，本项目按 `config.JK_PROTOCOL` 二选一，两者产出各自的快照
（`jk.get_state()`）；`bms_data` 再把它归一成**同一份规范样本**进共享缓存，金箭从机与
4G 上报都读缓存、不用区分数据来源：

| `JK_PROTOCOL` | 协议 | 物理/参数 | 特点 |
|---|---|---|---|
| `"display"`（默认） | 显示屏接口广播（原实现，解析要点见下一节） | 2400 8N1、单向广播、只收不发 | 无需轮询；但字段有缺失（无均衡电流/循环次数/实际容量），总容量靠 `剩余容量 ÷ SOC` 估算 |
| `"modbus"` | RS485 Modbus 通用协议（[JK-BMS-RS485.md](../docs/jikong/JK-BMS-RS485.md)） | 115200 8N1、主从应答、本机做主站轮询 | 字段齐全：均衡电流、循环次数、循环容量、SOH、实际容量、充/放电 MOS 状态，另可读配置区与信息区 |

切换只需改 `config.lua` 的 `JK_PROTOCOL` 后重新烧录脚本；两个实现共用同一个串口与方向脚
（`JK_UART_ID`、`JK_RS485_DIR_GPIO`），因为接的是同一根 485 线。`jk.lua` 只是转发入口，
对外接口（`get_state`/`is_fresh`/`get_stats`/`MAX_CELLS`）与 `jk_display.lua` 完全一致。

### 3.1 Modbus 主站轮询（`jk_modbus.lua`）

- 从机地址 `JK_MODBUS_SLAVE_ADDR`（默认 1），轮询周期 `JK_MODBUS_POLL_MS`（默认 1s），
  单帧应答超时 `JK_MODBUS_TIMEOUT_MS`（默认 **120ms**；245 字节@115200 仅需 21ms，留足余量即可，
  量程越短越不会在保护板掉线时长时间占着总线和日志），一轮 ≈ 3 帧请求（115200 下 <50ms）；
  整轮失败后退避 `JK_MODBUS_RETRY_MS`（默认 2s）再重试，连续失败只在第 1、10、20… 次打日志；
- 每轮读 3 块只读实时数据，**全部成功**才算一轮（任一块失败即整轮失败，不产出脏数据）：
  `0x1200`×32（单体电压 mV）、`0x1240`×2（`CellSta` 在位位图）、`0x128A`×58（汇总）；
- 汇总块字段（地址与 `esp32/main/jk_bms.c` 一致，实测与社区 esphome-jk-bms 相符）：

  | 寄存器 | 内容 | 换算 |
  |---|---|---|
  | `0x128A` | 功率板温度 TempMos | INT16，0.1℃ |
  | `0x1290` | 电池总电压 BatVol | UINT32，mV |
  | `0x1298` | 电池电流 BatCurrent | INT32，mA，充电为正 |
  | `0x129C` / `0x129E` | 电池温度 1 / 2 | INT16，0.1℃ |
  | `0x12A0` | 报警位域 Alarm | UINT32，手册 §6.3 位定义 |
  | `0x12A4` | 均衡电流 | INT16，mA |
  | `0x12A6` | [高]均衡状态 / [低]SOC | 非 0 → 均衡中；SOC 0–100% |
  | `0x12A8` | 剩余容量 | INT32，mAh |
  | `0x12AC` | 电池实际容量 | UINT32，mAh |
  | `0x12B0` | 循环次数 | UINT32 |
  | `0x12B4` | 循环总容量 | UINT32，mAh |
  | `0x12B8` | [高]SOH / [低]预充状态 | % |
  | `0x12BC` | RunTime BMS 运行时间 | UINT32，秒（→ 上报消息级 `BMSUPTime`） |
  | `0x12C0` | [高]充电开关 / [低]放电开关 | 0/1 |

- 串数取 `CellSta` 的置位个数；为 0 时退回“最后一个 ≥ `JK_CELL_MIN_MV` 的槽位”，
  变小需连续 `JK_CELL_CHANGE_DEBOUNCE` 轮确认（与广播实现同款抗抖）；
- 报警位图直接作为 `sysAlarm` 交给金箭从机——两套协议的 32 位报警位定义一致
  （bit7 充电短路、bit8 充电过温、bit9 充电低温、bit14 放电短路、bit15 放电过温、bit1 MOS 过温…），
  所以“短路/过温充电/过温放电/低温充电”等开关量在两种数据源下判断逻辑相同；
- 容错：单帧 CRC/地址/字节数非法 → 重新同步并计数；请求无响应 → 计超时并在下次请求前丢弃
  迟到帧；超过 `JK_DATA_TIMEOUT_MS` 无成功轮询 → 快照标记 `stale`（供 `is_fresh()`/日志诊断）；
  金箭从机与 4G 上报只看共享缓存里的最后一份规范样本，不因 `stale` 而停答（缓存为空时按全 0/空帧）；
- 本模块只读（功能码 03H），不写保护板配置（`0x1600` 写区未使用）。

## 4. 极空显示屏广播解析要点

- 帧格式 `A5 5A | LEN | CMD | ADDR_H | ADDR_L | DATA(N)`，`LEN = N + 3`，帧总长 `LEN + 3`；
- 命令码：`0x82` = 写变量区（2B 大端地址 + 数据）；`0x80` = **写寄存器**
  （第 4 字节寄存器号 + 第 5 字节值）——寄存器 `0x01`（背光/刷新）是**轮起点**，
  寄存器 `0x02` 是**蜂鸣器帧**（报警时出现在轮尾，不能当轮起点，否则会把一轮截断）；
- 一轮顺序：写寄存器 → 主数据块 `82/1000`(96B) → [≥25 串] 第 25 节电压 `82/2011`(9B)
  → 单体颜色帧×27(216B) → 扩展块 `82/1300`(46B) → [报警时] 蜂鸣器帧；
- 轮字节数/周期随固件与串数变化（20 串 = 364B，≥25 串 = 373B，周期 0.5–2.2s）：
  解析严格按 `LEN` 定界，并用「CMD + LEN 必须是固定组合」过滤数据里的假帧头，
  从任意位置接入都能在一个帧内重新同步，不写死轮长/周期；
- 主数据块：总压(10mV)、电流(0.1A，充电为正)、SOC、最大压差、MOS 温度、电池温度、
  均衡开关、充/放电 MOS 状态、24 路单体电压(mV，未用槽位 0)、9 个图标位（= `sysAlarm` 位组合）；
- 扩展块：SOC 图标格数、折算剩余容量(0.1Ah)、**剩余时间 时/分/秒**（偏移 4/6/8）、
  **功率 u32 @偏移 10**(0.1W)、最高/最低/平均单体、**完整 `sysAlarm` u32 @偏移 20**、
  动画计数、**最大压差 @偏移 28**、**MOS 温度 @偏移 30**、系统报警/均衡开关/内部状态 A/B；
- 单体颜色帧：每串一个 RGB565 颜色码，
  `0x07E0` 绿=普通、`0x001F` 蓝=**最高单体**、`0xF800` 红=**最低单体**、`0x0000` 黑=未用槽位；
  解析为最高/最低单体序号与有效槽位数（诊断用，不参与串数判定）；
- 第 25 节电压帧 `82/2011`（仅 ≥25 串）解析为第 25 路单体，故串数上限 25；
- 自检与容错：`Σ单体电压 ≈ 总电压`（超差告警不丢弃）；扩展块偏移 28/30 与主块偏移 8/10
  交叉校验并计数；串数取最后一个 ≥ `JK_CELL_MIN_MV` 的槽位，变小需连续 3 轮确认
  （抗单节瞬时 0mV）；超过 `JK_DATA_TIMEOUT_MS` 无完整一轮则标记 `stale`（供 `is_fresh()`/
  日志诊断），金箭从机与 4G 上报仍只用共享缓存里的最后一份规范样本。

## 5. 金箭 Modbus RTU 从机

寄存器映射与 ESP32 侧实现一致（“来源”都是共享缓存 `bms_cache` 里的规范样本，
两种数据源经 `bms_data` 归一后填的字段完全一样）：

| 金箭寄存器 | 内容 | 来源 | 换算 |
|---|---|---|---|
| 0 | 总电压 | 极空快照总压 | mV / 10 → 0.01V |
| 1 | 电芯串数 | 极空快照串数 | 直传（金箭侧上限 24） |
| 2 | SOC | 极空快照 SOC | clamp 0–100 |
| 3 | 容量 | 极空快照总容量（display 估算 / modbus 实际容量） | mAh / 1000 → Ah |
| 4 | 短路保护 | 极空快照短路报警位 | 0/1 |
| 5 | 充电电流 | 极空快照电流 | mA / 10 → 0.01A，充电为正 |
| 6/7 | 电池温度 2/1 | 极空快照电池温度（display 两路同值） | 整数 ℃ |
| 8 | 板温 | 极空快照 MOS 温度 | 整数 ℃ |
| 9–32 | 单体电压 | 极空快照单体电压 | mV 直传，超出串数补 0 |
| 103 | 电池类型 | 固定 0（锂电） | `JJ_BATTERY_TYPE` |
| 104 | 循环次数 | modbus 取极空 `SOCCycleCount`；display 无此数据用本地缓存 | 默认 0 |
| 110 | 均衡状态 | 极空快照均衡开关 | 0/1 |
| 113 | 标称容量 | 极空快照总容量 | Ah |
| 1000–1007 | PN | 模块自身 PN（IMEI 后 12 位，形如 `SOC-234567890123`） | 16 ASCII |
| 1016/1017 | 版本 | 脚本 `VERSION` → a.b.c.d，`a高a低` / `b高b低` | 默认 0100/0000 |
| 1089 | 充电时间 | 本地缓存（默认 120 分钟，可写 0–120） | 分钟 |
| 1090 | 目标 SOC | 本地缓存（默认 90%，可写 0–100） | % |

保护开关量优先取完整 `sysAlarm` 位图（display 来自扩展块、modbus 来自 `0x12A0`），
缺位图时回退到 9 个图标位（仅 display 可能缺；modbus 恒有）：

| 金箭开关量 | 含义 | sysAlarm 位 | 回退逻辑 |
|---|---|---|---|
| 4 | 短路保护 | bit7 `0x0080` 或 bit14 `0x4000` | 图标偏移 82 |
| 5 | 过温充电 | bit8 `0x0100` 或 bit1 `0x0002`(MOS 过温) | 图标偏移 78/80 |
| 6 | 过温放电 | bit15 `0x8000` 或 bit1 `0x0002` | 图标偏移 78/80 |
| 7 | 低温充电 | bit9 `0x0200` | 电池温度 ≤ `JJ_LOW_TEMP_CHARGE_C` |
| 8 | 低温放电 | 位表无专用位 | 电池温度 ≤ `JJ_LOW_TEMP_DISCHARGE_C` |
| 63 | 快充状态 | 充电中且电流 ≥ `JJ_FAST_CHARGE_CURRENT_MA`；主站写过"结束快充"后置 0，停充/放电后自动恢复判定 | — |

支持的应答：`01 03` 读寄存器、`01 01` 读开关量、`01 06` 写单寄存器（1089/1090）、
`01 05` 写单开关量（64 结束快充）；地址非法返回异常码 `02`，取值越界返回 `03`，
不支持的功能码返回 `01`，其它从站地址不应答。

> 金箭寄存器 `9–32` 只有 24 路单体：极空若为 ≥25 串，车端链路只暴露前 24 节，
> 第 25 节仍会进入 MQTT 压缩上报（`jk_display.MAX_CELLS = 25`）。

> 从机按标准 Modbus CRC 校验收帧（`docs/jinjian/02_BMS_PROTOCOL.md` §5 的 CRC 示例以标准计算为准）。

## 6. MQTT 压缩上报

- 开关：`config.lua` 的 `MQTT_HOST` 留空（默认值）即**关闭 4G 上报**——不建 MQTT 连接、不上报，
  极空采集、共享缓存与金箭从机照常工作；填入地址（如 `MQTT_HOST = "399.run"`）后重新烧录即启用；
- 鉴权：`clientId = 用户名 = device_id`（IMEI，取不到用模组唯一 ID），密码 =
  `HMAC-SHA256(MQTT_HMAC_SECRET, IMEI)` 的**小写 hex**（默认密钥 `"soc"`，纯 Lua 实现，
  见 `sha256.lua`/`hmac.lua`，连接时算一次）；服务端需按同一规则校验；
- 主题：`<MQTT_TOPIC_PREFIX>/<device_id>/status`（默认 `/bms/<IMEI>/status`），QoS0；
  `MQTT_HOST`/`MQTT_PORT`/`MQTT_IS_SSL`/`MQTT_HMAC_SECRET`/`MQTT_QOS` 均在 `config.lua` 修改；
- 周期：固定 `REPORT_PERIOD_MS`（默认 30s）上报一次。窗口 = 共享缓存里「上次上报游标之后
  新到达」的真实样本；游标是缓存分配的样本序号 `seq`（不是时间戳），所以同秒多条、NTP 跳变
  都不会让窗口重叠或丢样（被环形缓冲覆盖而没来得及上报的样本计入 `stat` 的 `lost`）；
- 窗口非空 → 按实际条数编码上报；窗口为空（保护板掉线/未接入/这一周期没有新数据）→ 只发
  **0 样本帧**（`flags bit10`，`aa 01 00 80 08` + 6 个消息级 uvarint，标量取 0/上电时间/计数，
  典型 11–20 字节，`pkg/bms` §11.5 的黄金向量为 13 字节），**绝不补零、插值或沿用旧值冒充
  采集数据**——旧实现的 keep-last 兜底会用同一时间戳反复覆盖，服务端按时间戳入库时表现为
  "这 30s 数据被覆盖/丢了"；
- 帧格式（MessageV1，与 `mqttagent/pkg/bms` 字节级一致；`bms.md` §11 五条黄金向量在
  `tools/selftest.lua` 里逐字节比对）：
  `u8 0xAA | uvarint version=1 | uvarint(sampleCount-1) | uvarint header | [modeTable] |
   6×uvarint 消息级字段 | 列数据`；header 的 bit2..bit9 = 串数、bit0 = 有 modeTable、
  bit1 = 电芯相邻压差布局、bit10 = 0 样本；
- 消息级字段：`SOCFullChargeCap` = 本批最新样本的**满电总容量**（display 用「剩余容量 ÷ SOC」
  估算，modbus 取 `0x12AC`）、`BaseTimestamp` = 本批最新样本的采集时间（样本时间 = `BaseTimestamp - Time`）、
  `ModuleUPTime` = 模组上电秒数（`mcu.ticks` 单调计时）、`BMSUPTime` = BMS 上电秒数
  （modbus `0x12BC` RunTime，display 无来源恒 0）、`JinjianModbusRequestCount` =
  金箭从机查询数（`jinjian_slave.get_stats().queries`）、`JikongModbusRequestCount` =
  极空 modbus 读请求数（display 模式为 0）；
- 列内容：`Temp1/Temp2/TempMos`(0.1℃；display 只有一路电池温度，`Temp1=Temp2`，modbus 取两路)、
  `BalanCurrent`(mA，display 无此数据填 0，modbus 取均衡电流)、`BatVol`(mV)、
  `BatCurrent`(mA，充电为正)、`SOCCycleCap`(**循环总容量** mAh，display 无此数据填 0、
  modbus 取 `0x12B4`)、`SOCCapRemain`(mAh)、`Time`(距 `BaseTimestamp` 的 uint8 秒偏移，≤255s)
  与单体电压列（编码器自动在 offset/spatial 两种布局取更短者）；一批内串数以最新样本为准，
  串数不同的样本本批跳过（重配串数时丢极少量样本，计入 `lost`）；
- 时间：`BaseTimestamp`/`Time` 都取 NTP 对时后的 `os.time()`；**未对时前不发布**（`ntp_synced`
  事件错过时用 `os.time() > 1.6e9` 兜底，最多等 60s 起步）；
- MQTT 未就绪时丢弃当前窗口并计数（QoS0、无本地堆积），断线由 `autoreconn` 自动重连；
  无 RPC/下行订阅；
- 调试：`LOG_HEX_PAYLOAD=true` 打印整包 hex；`bms_uplink.flush()` 立即按当前游标上报一次；
  仓库根目录 `lua hwsim.lua` 用虚拟时钟真跑整条链路，`DUMP=<file>` 导出 payload 给服务端解码器核对。

## 7. 管理链路：裸帧 RPC（RS485-1 / UART1）+ JSON-RPC over MQTT

> 本节的独立整理版（帧格式 / 方法表 / 错误码 / 两条 FOTA 通道 / MQTT 通道 / 验收 / 排障）见
> [`../docs/air780epm/MGMT_RPC.md`](../docs/air780epm/MGMT_RPC.md)。

RS485-1 是**运维/管理总线**：上位机（USB-RS485 + `tools/mgmt_probe.ps1`）当主站，本机当从机。
它是一条**一对一专用线**——点对点、**没有从站地址/广播/寄存器表**，主站一次只发一个请求；
请求是 JSON-RPC（`{"method","params"}`），只是外层套了一帧二进制头 + CRC 便于串口定界和容错。
三个用途：

1. **读状态**：`status.get` 一次拿回网络/MQTT/极空/缓存/上报/FOTA/三条链路计数等一大对象 ——
   不用接调试器就能判断"哪条链路没数据"；
2. **读/写配置**：`config.get` / `config.set`（白名单 + 范围校验，**写透**到 fskv，重启后仍生效）；
3. **命令**：`cmd.reboot` / `cmd.factory_reset` / `fota.url` / `fota.begin`·`fota.chunk`·`fota.abort`。

实现分两层：**一个统一管理服务** `mgmt.lua`（方法分发 / 状态快照 / 配置读写 / 命令 / FOTA / ID 去重，
与传输无关）+ **两条传输**：

| 传输 | 文件 | 说明 |
|---|---|---|
| UART1 裸帧 RPC | `mgmt_uart.lua` | RS485-1，115200 8N1（默认），EN=GPIO25；二进制帧头 + JSON 载荷 + 二进制 BLOB 分块；**只有这条有主动事件**（`evt.hello` / `evt.fota`） |
| MQTT JSON-RPC | `mgmt_mqtt.lua` | 通道 = 主题后缀 `rpc`，与 4G 上报**共用同一条 MQTT 连接**；纯请求-应答、不发事件 |

两条传输**方法、字段、错误码完全一致**（改一处要同时改两条）；`cfg_store.lua` 负责持久化，
`fota_mgr.lua` 负责升级编排。

升级有**两条互斥通道**（都"成功不自动重启"）：§7.4 `fota.url`（设备自己联网下载，需 `libfota2`）
和 §7.5 本地分段上传（主站把升级包按块写进设备，核心库 `fota` 直接写 Flash，**不需要联网**）。

### 7.1 帧格式（UART，双向同一套）

| 字段 | 长度 | 说明 |
|---|---|---|
| SOF | 2 | `0xA5 0x5A`（扫描器遇到非 SOF 丢弃并重新同步） |
| VER | 1 | `0x01`（不符丢帧并计数） |
| TYPE | 1 | `0x01` REQ / `0x02` RESP / `0x03` EVT / `0x04` BLOB |
| ID | 1 | `1–255` 请求号；EVT 固定 `0`；RESP 原样回带 |
| LEN | 2 | 载荷长度，**小端**；JSON ≤ **1024**，BLOB 载荷 ≤ **242** |
| PAYLOAD | LEN | 见下 |
| CRC16 | 2 | `0xA001`，**小端**，覆盖 `VER…PAYLOAD` |

载荷形态：

```
REQ  = {"method":"...","params":{...}}          （无 params 时可省略）
RESP = {"ok":true,"result":{...}} 或 {"ok":false,"error":{"code":"...","msg":"..."}}
EVT  = {"evt":"hello|fota","data":{...}}        （ID 固定 0，仅 UART 主动发）
BLOB = SEQ(2 LE) | LEN(2 LE) | DATA(LEN)        （DATA ≤ 238 字节，偏移由设备按已收字节自动推得）
```

收发规则：CRC 错 / VER 不符 / 超长分别计数丢弃；**同一时刻只允许一个请求在飞**；**回帧先于动作**
（重启等命令在响应发完 200 ms 后才执行）；**同一 ID 在 `MGMT_DEDUP_WINDOW_MS`（默认 10 s）内
重复到达且方法相同 → 直接重放缓存响应、不重复执行**（防止丢应答后主站重试造成二次重启/二次 FOTA）。

> ⚠ **主站每次请求都要用新的 ID**（`1–255`）。去重按「ID + 方法」生效，**读方法也一样**：
> 10 s 内用同一个 ID 再发一次 `status.get` / `config.get`，拿回的是缓存响应（不是新采样）。

| 项 | 说明 |
|---|---|
| 串口 | UART1，**115200** 8N1，无校验、无流控（`MGMT_UART_ID=1`、`MGMT_BAUD=115200`，`[R]`） |
| 引脚 | UART1 是**默认复用**功能（PIN18=UART1_TXD、PIN17=UART1_RXD），不用 `pins.setup` |
| 方向 | 本板 RS485-1 是带方向控制的收发器：`MGMT_RS485_DIR=25`（EN=GPIO25），由串口驱动按收发自动切换 —— 少了它完全收发不了 |
| 统计 | `stat` 日志追加 `mg_rx`（收到字节）/ `mg_frames`（解析出的请求帧）/ `mg_reply`（应答帧）/ `mg_crc`（CRC 错的帧）/ `mg_sync`（重同步丢弃字节）/ `mg_drop`（缓冲溢出丢弃）/ `mg_tx`（写回字节）/ `mg_evt`（事件帧）。**正常通信时 `mg_crc`/`mg_sync` 应为 0，`mg_reply` 随请求增长** |
| 容错 | 回调只搬字节、帧扫描/应答在任务里（见 §11）；`MGMT_RX_BUFF_SIZE=2048`、单帧最大 1200 字节；每帧处理包一层 pcall，坏帧/异常只丢这一帧 |

### 7.2 方法表（UART 与 MQTT 完全一致）

| 方法 | 参数 | 返回 |
|---|---|---|
| `sys.ping` | 无 | `{time, uptime_s}`（链路连通性自检） |
| `status.get` | 无 | 一个大对象（见 §7.3） |
| `config.get` | 无 | `{values{KEY:val}, restart_keys[], needs_restart, has_override}`（仅 `cfg_store.SPEC` 白名单） |
| `config.set` | 键值对象（可多个） | 先全量校验、再原子写入：`{applied{…}, restart_required}`；非法键 → `EBADKEY`，非法值 → `EBADVALUE`，**整批不落盘** |
| `cmd.reboot` | 无 | `{rebooting=true}`，回帧后 200 ms 重启（`[R]` 配置生效） |
| `cmd.factory_reset` | 无 | `{factory_reset=true}`，清 fskv 覆盖 + 置待生效位，回帧后由主机重启 |
| `fota.url` | `{url}` | `{started=true}`，启动 URL 升级（需 `libfota2`，否则 `ENOLIB`） |
| `fota.begin` | `{total, crc32?}` | `{started=true, total=…}`，开始本地分段升级（`crc32` 缺省/0 = 不校验） |
| `fota.chunk` | `{seq, data(base64)}` | `{seq, rx, total, progress}` —— **仅 MQTT**；UART 用 BLOB 帧 |
| `fota.abort` | 无 | `{aborted=bool}`，中止本地会话 |
| `fota.status` | 无 | `{mode, state, stage, result, progress, rx, total, last_seq, caps}` |

错误码（字符串）：`EBADJSON` `EBADFRAME` `EBADMETHOD` `EBADPARAM` `EBADKEY` `EBADVALUE` `EBUSY`
`ENOLIB` `EFLASH` `ECRC32` `EBADSEQ` `EFOTA` `EINTERNAL`。

**事件（仅 UART，设备主动推；MQTT 纯请求-应答）**：

| 事件 | 时机 | 内容 |
|---|---|---|
| `evt.hello` | mgmt 串口打开后约 1 s 发一次 | `project/model/version/pn/reset_reason/uptime_s/proto/caps` |
| `evt.fota` | FOTA 阶段/状态变化时发（失败也发一次） | `mode/state/stage/result/progress/rx/total` |

### 7.3 `status.get` / `config.get` 字段

`status.get` 返回一个大对象：

```
net_ready, mqtt("disabled|up|down"), jk_valid, power_on,
needs_restart, has_override, factory_pending, fota_busy,
reset_reason, uptime_s, supply_mv,
device{project, model, version, pn, proto},
links{jk{rounds,frames,rx,sync}, jinjian{queries,responses,ignored,rx},
      mgmt_uart{rx,frames,reply,crc,sync,drop,tx,evt}, mgmt_mqtt{rx,reply,ignored,bad}},
cache{count, capacity},
uplink{published, samples, empty, lost, batches, dropped},
fota{mode, state, stage, result, progress, rx, total, last_seq,
     caps{core, core_type, lib, local_enabled, url_enabled, fskv, json}}
```

`reset_reason` = `pm.lastReson()` 第 3 值（`0` 上电/看门狗切供电、`3` 软重启、`5` RESET 脚、
`8` 内部看门狗、`9` 外部重启，见 §8）。`supply_mv` / `device.pn` 由**后台任务**刷新（ADC 分次采样、
IMEI AT 查询），**不在请求路径里**——方法调用是纯函数、不挂起。

> `links.mgmt_uart` 是**本请求处理过程中的快照**：`frames` 已经把这次请求算进去，而本次应答尚未
> 计入 `reply`，所以每次读都是 `reply = frames - 1`（不是链路异常）。判断链路好坏看 `crc`/`sync`
> 是否一直为 0。

`caps` 是"现场一眼判据"：`core` 核心库 `fota` 可用（本地分段升级必需）/ `core_type` 它的 Lua
**类型名字符串**（`"userdata"` = 正常 LuatOS `rotable`、`"table"` = 别的实现、`"nil"` = 固件真
没这个库）/ `lib` `libfota2` 可用（URL 升级必需）/ `local_enabled`·`url_enabled` 两条通道开关 /
`fskv`·`json` 可用。

`config.get` 只列 `cfg_store.SPEC` 白名单键的**当前生效值**（有覆盖则是覆盖值）+ `restart_keys`
（需重启才生效的键）+ `needs_restart` / `has_override` 两个标志。`config.set` 的键名沿用现有大写名：

| 键 | 范围 / 取值 | 生效 |
|---|---|---|
| `MGMT_BAUD` | 1200/2400/4800/9600/19200/38400/57600/115200 | `[R]` |
| `LOG_ENABLE` | true/false | 实时 |
| `JK_MODBUS_POLL_MS` | 100–60000 | 实时 |
| `JJ_LOW_TEMP_CHARGE_C` / `JJ_LOW_TEMP_DISCHARGE_C` | -100…100（℃） | 实时 |
| `JJ_FAST_CHARGE_CURRENT_MA` | 0–65535（mA） | 实时 |
| `REPORT_PERIOD_MS` / `STATUS_REPORT_MS` | 1000–3600000 | `[R]` |
| `JK_PROTOCOL` | `display` / `modbus` | `[R]` |
| `JK_MODBUS_SLAVE_ADDR` | 1–247 | `[R]` |
| `MQTT_PORT` `MQTT_IS_SSL` `MQTT_QOS` `MQTT_KEEPALIVE_S` `MQTT_HOST` `MQTT_TOPIC_PREFIX` | 见 `config.lua` | `[R]` |

> 恢复出厂的语义：`cmd.factory_reset` 清掉 fskv 里的覆盖并置"恢复出厂待生效"，**重启后**回
> `config.lua` 默认值（`status.get` 里 `factory_pending=true`）。旧的 `MODBUS_*` 持久化覆盖项
> 已作废——开机加载时按"不在白名单"跳过并打日志，需要的主机侧配置用新的 `MGMT_*` 键重新下发。

### 7.4 FOTA 通道 A：URL 升级（`fota.url`，设备自己联网下载）

1. 主站发 `fota.url`，参数 `{"url":"http://ota.example/fw.binpkg"}`；
2. 设备侧：等联网（`net_ready`，最多 `MGMT_FOTA_NET_WAIT_MS`）→ 调 `libfota2.request` 下载 →
   `fota.status` 的 `state` 变 2（成功待重启）、`progress` 到 100；
3. 主站轮询 `fota.status` 确认成功后，再发 `cmd.reboot` 进新版本 —— **成功不自动重启**，
   现场可以挑合适的时间点重启（下完就重启会中断数据上报）；
4. 失败（网络/URL/校验）→ `state=3`、`result` 是结果码，可重新发命令重试；忙时（`fota_busy=true`，
   或本地通道在忙）新的启动请求返回 `EBUSY`。

`libfota2`（LuatOS 扩展库）要在 Luatools 烧录时勾「添加默认 lib」；取不到时 `fota.url` 返回
`ENOLIB` 并打明确日志，其它功能不受影响（**只影响 URL 通道**：§7.5 的本地通道不需要 `libfota2`、
也不需要联网）；`MGMT_FOTA_ENABLE=false` 可整体关闭 URL 通道（见 §8）。

### 7.5 FOTA 通道 B：本地分段上传（BLOB / `fota.chunk`，不联网）

主站（PC 工装/上位机）把 LuatOS 升级载荷**按块**写进设备，设备用核心库 `fota.init/run/isDone/finish`
**直接流式写 Flash** —— 不联网、不写文件系统、不依赖 `libfota2`，适合无网现场或私有包。

开关与参数（`config.lua`）：

| 配置项 | 默认 | 说明 |
|---|---|---|
| `MGMT_FOTA_LOCAL_ENABLE` | `true` | 关掉后 `fota.begin`/`fota.chunk`/BLOB 一律返回 `EFOTA` |
| `MGMT_FOTA_LOCAL_TIMEOUT_MS` | `60000` | 空闲超时：这么久没有新数据块 → 自动 `fota.finish(false)` 取消会话 |
| `MGMT_FOTA_LOCAL_FINALIZE_MS` | `3000` | 最后一块写完后等核心库落盘（`fota.isDone()`）的上限 |
| `MGMT_FOTA_LOCAL_WAIT_MS` | `3000` | `fota.init()` 后轮询 `fota.wait()` 等底层就绪的上限（超时只告警仍继续） |

**升级包格式限制**：这里收的是 **Luatools 产出的 LuatOS `.bin` 升级载荷**（脚本单包，或
core+script 差分包；例如 `jjsoc_2052.1.0_LuatOS-SoC_Air780EPM.bin`），**不是**约 2.6 MB 的
`.binpkg` 容器。包大小没有单独上限（`total` 是 u32 + 每块 ≤238 字节）；`crc32` 填 **0 = 不做最终
校验**，填值 = 主站对**整包**算的标准 CRC32（IEEE 802.3，多项式 `0xEDB88320`，初值/结果取反，
与 PC 的 `zlib.crc32`、`mgmt_probe.ps1` 里的 `Get-Crc32` 一致），设备边收边算、收齐后比对。

**两条通道的数据块形态**：

- **UART** 用 **BLOB 帧**：`TYPE=0x04`，载荷 = `SEQ(2 LE) | LEN(2 LE) | DATA(LEN)`，`DATA ≤ 238`；
  **偏移由设备按已收字节自动推得**（主站不再传 offset）。设备收到 BLOB 帧后与请求帧一样回一条
  RESP（同一 ID）：`{"ok":true,"result":{"seq":…,"rx":…,"total":…,"progress":…}}`；失败回错误码。
- **MQTT** 用 `fota.chunk`：参数 `{"seq":<块序号，从 1 递增>,"data":"<BASE64>"}`，服务端解码后
  与 BLOB 走同一条写入路径（偏移同样由设备推得）。

**升级流程**（主站视角）：

1. 先 `fota.status` 确认没有进行中的会话（`state != 1`）；必要时 `fota.abort` 清掉旧会话；
2. `fota.begin {"total":<包长>,"crc32":<可选>}`；设备后台执行 `fota.init()`，成功后 `stage` 变 `ready`(2)
   —— 主站**要等到 `ready`(2) 再发第一块**（轮询 `fota.status`；`init` 没跑完就写块会被判 `EBADSEQ`，
   `tools/mgmt_probe.ps1` 已按此实现）。另外**每次请求都要换新 ID**：同 ID + 同方法在 10s 内是重放
   缓存响应，固定 ID 轮询会一直读到第一份快照；
3. 按 SEQ 顺序发数据块（UART BLOB / MQTT `fota.chunk`，每块 ≤238 字节）；设备逐块 `fota.run(数据)`；
   **同 SEQ/偏移/长度的重发按幂等重试处理**，不重复写 Flash；返回的 `rx/total/progress` 可用于进度显示；
4. 最后一块写完：设备等 `fota.isDone()`（≤ `MGMT_FOTA_LOCAL_FINALIZE_MS`）→ 若声明了 CRC32 就校验整包
   → 成功则 `fota.finish(true)`，`stage=success`(5)、统一状态 `state=2`、进度 100%、置**「需重启」**，
   **不自动重启**；
5. 主站确认 `stage=5` 后，再发 `cmd.reboot` 进新版本；
6. 失败/中止 → `stage=failed`(6)，失败码进 `result`；中止方式：`fota.abort`，或空闲超时自动
   `fota.finish(false)`。失败/中止后**重启仍是旧版本**（半包被核心库丢弃），可以重新走一遍。

`result`（结果码）在本地通道下的取值：`-1` 核心库 `fota` 缺失（`ENOLIB`）、`-10` init 失败、
`-12` 空闲超时、`-21` run（写 Flash）失败（`EFLASH`）、`-30` `isDone` 超时/报错、
`-31` CRC32 不符（`ECRC32`）、`-32` finish 失败、`-40` 主站中止。

> **来源判定（`mode`）**：URL 进行中 > 本地会话进行中 > **URL 会话已结束（成功/失败）** > 本地会话
> 已结束。这样 URL 通道的结果码不会被上一次本地升级残留的 `stage/result` 盖掉；要看"这次是谁在升级"
> 就直接读 `mode`（`1`=URL、`2`=本地）。

> **核心库用法（照官方 demo `demo/fota/fota(使用fota核心库)/fota_uart.lua`、`fota_file.lua`、
> `demo/fota/ble_5101_fota/ble_5101_packet_fota.lua`）**：
> `fota.init()` →（可选）轮询 `fota.wait()` 等底层就绪 → 反复 `fota.run(buf)` → `fota.isDone()`
> → `fota.finish(ok)`。
> · `fota.run(buf)` 的 `buf` 可以是**字符串**（BLE 分段 demo 就传字符串；UART demo 传 zbuff），
>   返回 **`(无异常, 收到最后一块, 未写 Flash 的缓存字节数)`**；
> · `fota.isDone()` 返回 **`(无异常, 写到最后一块)`** —— 以**第二个**值为准；
> · `fota.wait()` 在官方 demo 里是**"等底层就绪"**的轮询（`while not fota.wait() do sys.wait(100) end`），
>   本模块按此实现，但加 **`MGMT_FOTA_LOCAL_WAIT_MS` 上限**（默认 3s）：就绪就进 `ready`，超时只告警
>   仍继续（个别固件 `wait()` 语义若不同，不至于把本地升级卡死）。
>
> **前置条件：底层固件必须带核心库 `fota`（`LUAT_USE_FOTA`）**。判据是 `status.get` 的
> `fota.caps.core` / `fota.status` 的 `caps.core`。缺失时一个字节的升级数据都不会写、设备仍跑
> 旧版本（"优雅降级 + 安全失败"）。
>
> ⚠ **踩过的坑（本机 1.0.0-D 实测）：明明固件带 `fota`，却报 `caps.core=false` + `fota.begin`
> 回 `ENOLIB`。** 原因不是固件裁剪，而是**判据写错了**：LuatOS 的 C 库不是 Lua table ——
> `rotable2_newlib()`（`lua/src/rotable2.c`）把它注册成 **userdata + 元表**的只读表
> （`type(fota) == "userdata"`、元表 `__name == "rotable"`，成员从元表 `__index` 取，
> `fota.init()` 点号调用照常可用）。旧探针写成 `type(fota) == "table"` 于是永远判「缺库」。
> 现在改成「`fota.init` 可调用即算有库」，并加了 `caps.core_type` 便于一眼分清；离线自测
> §13.2b 用 userdata 形态把这个坑固定住了（旧写法会让该用例 FAIL）。

**与 URL 通道的关系**：两条通道**互斥** —— URL 正在下载时 `fota.begin` 返回 `EBUSY`，
本地会话在忙时 `fota.url` 也返回 `EBUSY`。两条通道成功后都只置状态 2 + 「需重启」。

### 7.6 PC 侧验收：`tools/mgmt_probe.ps1`

脚本自动组帧 / 算 CRC16 / 解 JSON，覆盖本节的现场验收项，可脚本化（退出码 0 = 成功）：

```powershell
# 连通性自检 + 读状态（最常用）
powershell -ExecutionPolicy Bypass -File air780epm\tools\mgmt_probe.ps1 -Port COM11 -Action ping
powershell -ExecutionPolicy Bypass -File air780epm\tools\mgmt_probe.ps1 -Port COM11 -Action status

# 读配置；改配置（实时项 / [R] 项）
powershell -ExecutionPolicy Bypass -File air780epm\tools\mgmt_probe.ps1 -Port COM11 -Action config
powershell -ExecutionPolicy Bypass -File air780epm\tools\mgmt_probe.ps1 -Port COM11 `
    -Action set -Set "JK_MODBUS_POLL_MS=1500"
powershell -ExecutionPolicy Bypass -File air780epm\tools\mgmt_probe.ps1 -Port COM11 `
    -Action set -Set "MQTT_HOST=mqtt.399.run,MQTT_PORT=1883"

# 重启 / 恢复出厂
powershell -ExecutionPolicy Bypass -File air780epm\tools\mgmt_probe.ps1 -Port COM11 -Action reboot
powershell -ExecutionPolicy Bypass -File air780epm\tools\mgmt_probe.ps1 -Port COM11 -Action factory

# URL FOTA
powershell -ExecutionPolicy Bypass -File air780epm\tools\mgmt_probe.ps1 -Port COM11 `
    -Action url -Text "http://ota.example/fw.binpkg"

# 本地分段升级（BLOB 帧，成功后再 -Action reboot）；-Action abort 中止
powershell -ExecutionPolicy Bypass -File air780epm\tools\mgmt_probe.ps1 -Port COM11 `
    -Action localfota -File .\jjsoc_2052.1.0_LuatOS-SoC_Air780EPM.bin
powershell -ExecutionPolicy Bypass -File air780epm\tools\mgmt_probe.ps1 -Port COM11 -Action abort

# 监听设备主动事件（evt.hello / evt.fota）；诊断回环见 §7.9
powershell -ExecutionPolicy Bypass -File air780epm\tools\mgmt_probe.ps1 -Port COM11 -Action listen
```

适配器方向极性：USB-485 有自动方向的，也有把 RTS 当 DE（少数当 `/RE`）的；脚本默认每帧先在
`RTS=0`、再在 `RTS=1` 下各试一遍（`-Rts 0` / `-Rts 1` 可固定）。用前先关掉占用该 COM 口的串口助手。

**硬件验收清单**（USB-RS485 接 RS485-1，**115200** 8N1，A↔A/B↔B/共地）：

1. `-Action ping`：拿到设备时间与上电秒数（链路双向都通）；
2. `-Action status`：`net_ready`/`mqtt`/`jk_valid` 与三条链路计数；记下 `links.jk.rx`、
   `links.jinjian.queries`（过一会儿再读，应在涨）、`links.mgmt_uart.crc`/`sync` 应为 0；
3. `-Action set -Set "MQTT_HOST=<新 host>"` → `-Action config` 读回一致（`restart_required=true`）；
   `-Action reboot` 重启 → 再读，配置仍是新值（写透 + 持久化生效）；
4. `-Action set -Set "JK_MODBUS_POLL_MS=1500"`（实时项）→ 立即读回 1500，`restart_required=false`；
5. `-Action factory` → `-Action reboot` 后配置回 `config.lua` 默认值；
6. 非法项：`-Action set -Set "JK_MODBUS_POLL_MS=50"` 应回 `EBADVALUE`，
   `-Action set -Set "NOPE=1"` 应回 `EBADKEY`，`-Action raw -Text '{"method":"nope"}'` 应回 `EBADMETHOD`；
7. `-Action url -Text <真实 binpkg URL>` → `-Action status` 看 `fota.state/progress`
   （1 下载中 → 2 成功待重启）→ `-Action reboot` 进新版本；
8. **本地升级（§7.5）**：`-Action localfota -File <版本号已提升的 .bin>` → 打完整升级包 →
   脚本轮询到 `stage=success` 后，`-Action status` 应看到 `fota.mode=2`、`fota.stage=5`、进度 100%、
   `needs_restart=true`；再 `-Action reboot`，`-Action status` 应看到 `device.version` 已变；
   **反例**：重跑一遍、中途 `-Action abort`（或不发数据块等超时），重启后版本仍是旧值。

### 7.7 管理 MQTT JSON-RPC（通道 = 主题后缀 `rpc`）

与管理链路 UART1 并列的**第二条管理传输**：主站不接 485，而是把 JSON-RPC 请求经 MQTT 发下来，
本机把响应发回。方法表、字段、错误码与 UART 完全一致（同一个 `mgmt` 服务）。

| 项 | 说明 |
|---|---|
| 通道 | **主题后缀就是通道名**：与上报通道 `status` 并列，管理通道后缀 = `MGMT_MQTT_SUFFIX`（默认 `rpc`） |
| 下行 | `<MQTT_TOPIC_PREFIX>/<device_id>/rpc`，载荷 = `{"id":<事务号>,"method":"...","params":{...}}` |
| 上行 | `<MQTT_TOPIC_PREFIX>/<device_id>/rpc/resp`，载荷 = `{"id":<原样>,"ok":…,"result":…}` |
| 设备 ID | `IMEI`（取不到用模组唯一 ID），与上报同源 |
| 连接 | **与 4G 上报共用同一条 MQTT 连接**（clientId = username = IMEI，同 broker 只允许一条）；`mgmt_mqtt` 不自己 `create`/`connect`，只向 `bms_uplink` 登记订阅，断线重连后的订阅由 uplink 在每个 CONNACK 后重放 |
| 开关 | `MGMT_MQTT_ENABLE`（默认 true）**且** `MQTT_HOST` 已配置才启动；任一不满足只打日志，UART1 裸帧 RPC 与 4G 上报都不受影响 |
| 语义 | **每消息一条请求、回一条响应**，事务号用 JSON 里的 `id`（设备原样回带）；**纯请求-应答、不发事件**；非 JSON / 坏载荷只丢该条并计数（`stat` 的 `mgmq_bad`） |
| 统计 | `stat` 追加 `mgmq_rx`（收到载荷）/ `mgmq_reply`（回发响应）/ `mgmq_ignored`（未回发）/ `mgmq_bad`（无法解析） |

PC 侧验证（不接 485、不用真设备）—— 用 `mosquitto_pub`/`mosquitto_sub` 就能整条走一遍
（`<prefix>`/`<device>` 换成实际值，下面示例 `prefix=/device`、设备 `861234567890123`）：

```bash
# 1) 先订阅上行主题看设备回什么
mosquitto_sub -h 399.run -t '/device/861234567890123/rpc/resp' -v

# 2) 发一条 JSON-RPC（sys.ping）
mosquitto_pub -h 399.run -t '/device/861234567890123/rpc' -m '{"id":1,"method":"sys.ping"}'
# 上行主题收到：{"id":1,"ok":true,"result":{"time":…,"uptime_s":…}}

# 3) 读状态 / 读配置
mosquitto_pub -h 399.run -t '/device/861234567890123/rpc' -m '{"id":2,"method":"status.get"}'
mosquitto_pub -h 399.run -t '/device/861234567890123/rpc' -m '{"id":3,"method":"config.get"}'
```

> 认证与上报一致（同一个 clientId/用户名/密码）；broker 若开了 ACL，记得给设备放行
> `.../rpc`（订阅）与 `.../rpc/resp`（发布）两个主题。换通道名只改 `MGMT_MQTT_SUFFIX`，
> 响应主题固定是「订阅主题 + `/resp`」。旧 `.../modbus` 主题已失效。

### 7.8 UART3 引脚组与排查（极空链路，本板实测结论）

UART3 是**极空链路**（RS485-3）：它的收发依赖 §2.2 那套引脚复用，配错组就是"串口能开、总线上
没数据"。模组芯片上 UART3 有**两组**可用脚，`pins.setup` 对两组都返回成功 —— 所以
**"配成功"≠"接对了"**：

| 组 | RX | TX | 在本板 |
|---|---|---|---|
| A 组 | PIN58 | PIN57 | 合宙官方 `demo/pins/pins_Air780EPM.json`（开发板 V1.3）用的那组，**D700Tm 上没接出去** |
| **B 组** | **PIN53** | **PIN52** | **RS485-3 就是接这一组** —— 工程里的 `pins_Air780EPM.json` 就是这么配的 |

配错引脚组的现象极具迷惑性：串口能 `open`、日志一直在写、`uart.write` 也返回正常，但**总线上
一个字节都没有**；判据是量 485 芯片的 DI 脚（模组 TX → 收发器）：RS485-1/2 是 3.3V，配错组的
RS485-3 只有 ~2.8V。另外**两组都是 RX 号 > TX 号**（53>52、58>57），别按"小号是 RX"猜。

### 7.9 排障（管理链路）

排查顺序（`mg` 是管理链路 UART 传输的服务日志前缀）：

| 现象 | 说明 | 查哪里 |
|---|---|---|
| `ping`/`status` 有应答、`mg_crc`/`mg_sync` 为 0 | 管理链路双向都通 | — |
| 有应答但 `net_ready` 一直是 false | 网络没就绪（与方法无关） | 看 `stat` 的 `mqtt` 与拨号日志（§8） |
| `mg_frames` 不涨、`mg_sync` 一直涨 | 波特率/帧头不对（主站波特率不是 115200、或发的是别的协议） | 确认 `-Baud 115200`、帧头 `A5 5A`；用 `-Action raw` 发 §7.1 的帧对照 |
| `mg_crc` 一直涨 | 帧在路上被破坏（A/B 接反、电平、接线） | 查共地/A-B、GPIO 高电平是否 3.3V（§2.1） |
| 完全没应答 | 模组根本没收到/没上引脚 | ① 开机日志有没有 `mg uart 1 opened 115200 8N1 (mgmt rpc, raw frame)`；② GPIO25 方向脚（`MGMT_RS485_DIR=25`，§2.1）；③ A/B、共地、适配器方向极性 |
| 命令返回 `EBUSY`/`ENOLIB`/`EFOTA` | FOTA 忙 / 缺 `libfota2`（URL 通道）/ 本地核心库 `fota` 缺失、通道关闭 | 见 §7.4/§7.5；先读 `caps` |
| 配置写了重启后回默认 | 写的是非法键（`EBADKEY`）或 fskv 不可用 | 看 `config.set` 的返回与 `config.get.has_override`；fskv 不可用时只告警不报错 |
| 三路都没数据，日志停在开机 banner、每 ~15 秒重启一次 | `main.lua` 少了 `sys.run()` | 补上（§2.1） |
| RS485-1 完全收发不了；或各端口 EN／逻辑高电平"高不起来" | 缺 GPIO25 方向控制；或 GPIO 高电平只有 1.8V | 见 §2.1 三条硬约束 |

> **诊断模式**：`MGMT_ECHO_TEST=true` 时 UART1 不跑 RPC，改成"收到什么原样回什么 + 每
> `MGMT_ECHO_HEARTBEAT_MS` 一行 `[mg echo] … #N` 心跳"，只测电气层（收到心跳 = 模组→PC 通，
> 发什么回什么 = PC→模组 通）。排查完**必须改回 false**（回环与 RPC 不能同时开）。


## 8. 运行、烧录与运维

1. Luatools 新建项目，目录指向本 `air780epm` 文件夹（脚本在根目录，自动打包）；
2. **进 USB 下载模式只能硬件触发**：按住 **BOOT** → 短按 **RESET**（未上电则按开机键）→ 松开 BOOT，
   Luatools 会提示"发现 ec718pm 固件，请按住 BOOT 键复位设备"；没有 AT 口，也没有纯软件入口；
3. **脚本区地址必须和设备里的底层固件一致**：打开工程的"底层固件"要选成设备当年烧的同款 `.soc`。
   实测 `LuatOS-SoC_V2052_Air780EPM_2.soc` → 脚本区 **0x2A3000**（288KB），`..._1.soc` → **0x2B1000**（256KB）；
   混用会导致 `D/vfs mount error ret -1` / `E/main module 'main' not found`（现象：烧完还是老脚本、"未找到 main.lua"）。
   只烧脚本时尤其要选对；
4. **同一时刻只让一个工具占用下载口**：Luatools 与其它烧录工具同时跑会互相打断
   （`LPCSyncDetect failed` / `preamble no response`）；
5. 选择 Air780EPM 固件（需含 `mqtt`、`socket.sntp`、`adc`、`gpio`、`pins` 等库）。喂狗用的是
   工程自带的 `air153C_wtd.lua`（就在脚本根目录，随脚本一起打包，**不需要**勾"添加默认 lib"；
   但**不能把它从工程里删掉**：`board.lua` 在加载期就 require 它，取不到会导致整个脚本加载失败）；
   官方默认扩展库 `exair153x_wdt` 已弃用——实测它按时打"喂狗完成"，本板 Air153C 却照旧按自身
   超时复位（见本节末的两个现场条目）；
5.1 **引脚复用（`pins_Air780EPM.json`）**：`UART1`（PIN17/18）、`UART2`（PIN28/29）是芯片默认复用，
   不用配；极空链路的 `UART3` 不是，本板接 PIN53/PIN52 —— 工程已带 LuatIO 生成的 D700Tm 管脚表
   `air780epm/pins_Air780EPM.json`，**记得在"脚本和资源列表"里把它一起打包下载**（进 `/luadb/`，
   固件开机自动加载，之后不再有 `W/pins ... not exist!!`）；`board.lua` 开机还会显式
   `pins.loadjson` 一次兜底，成功会打 `引脚复用配置已加载 /luadb/pins_Air780EPM.json code 0`
   （没看到这行就说明表没随脚本下发）。`uart_pins.lua` 里那句 `pins.setup` 只是兜底，见 §2.2/§7.8；
5.2 **URL FOTA 需要扩展库 `libfota2`**：同样在 Luatools 烧录时勾「添加默认 lib」随工程下发。
   取不到时设备照常运行，只是管理链路的 `fota.url` 返回 `ENOLIB` 并在日志里写明原因；
   `fota_mgr.lua` 的 `require("libfota2")` 必须保持字面量写法（Luatools 打包依赖检查只认
   `require("字面量")`，写成 `require(变量)` 就不会被下发）；
   5.2.1 **本地 FOTA（§7.5）不需要 `libfota2`、也不需要联网**，它用的是 LuatOS **核心库**里的
   全局 `fota`（`fota.init/run/isDone/finish`）。固件裁剪掉该库时，`fota.begin` 返回 `ENOLIB`
   （`stage=6` / `result=-1`），日志写明原因，其它功能不受影响。
   这也是本模块从 `fota.lua` **改名 `fota_mgr.lua`** 的原因：`require "fota"` 会与核心库的
   全局 `fota` 互相遮蔽（LuatOS 的核心库是全局表，同名会让人分不清取到的是哪一个）。
6. 下载脚本后自动运行，日志可见 `main jjsoc 1.0.0`、数据源选择日志
   （`jk 数据源 = …`）、`jj modbus rtu slave ready, uart 2 ... PN SOC-xxxxxxxxxxxx`，
   极空链路（UART3）：`jk 引脚复用 PIN 53 -> UART3_RX`、`jk 引脚复用 PIN 52 -> UART3_TX`，
   `jk uart 3 opened 2400 8N1 (RX only)`（display 模式）或
   `jkmb uart 3 opened 115200 8N1 (master), slave addr 1`（modbus 模式），
   以及管理链路这一行：`mg uart 1 opened 115200 8N1 (mgmt rpc, raw frame), buf 2048`；
   如果配了持久化，还会有 `cfg 已应用持久化配置项 N 项`（见 §7）；
7. 现场验证：USB-RS485 接 RS485-2 模拟金箭主站（9600 8N1）轮询 `01 03 0000 0009`；
   display 模式下抓 RS485-3 波形确认 2400 8N1 广播，modbus 模式下改用 USB-RS485 抓
   `01 03 1200 0020`（115200 8N1）确认保护板按从机地址 1 应答；管理链路把 USB-RS485 接到
   RS485-1 的 A/B（A↔A、B↔B、共地），115200 8N1，跑 `tools/mgmt_probe.ps1` 按 §7.6 的
   验收清单逐项过一遍（ping → 读状态 → 读写配置 → 重启 → 恢复出厂 → FOTA）。

- NET LED：注册网络快闪 → 拨号成功慢闪 → MQTT 连上常亮；未配置 `MQTT_HOST` 时不会有 CONNACK，
  拨号成功后直接常亮（表示网络可用但不启用上报）；
- 周期状态日志：每 `STATUS_REPORT_MS`（默认 60s）打印一行 `stat`，汇总各链路的收发计数
  ——极空数据次数 `jk_rounds`（display = 完整广播轮数，modbus = 完整轮询轮数）/ `jk_frames`
  （解析帧数）/ `jk_rx`（RS485-3 收到的原始字节数）/ `jk_sync`（因帧头/长度非法丢弃的字节数）、
  中控 BMS 查询 `ctrl_query` / 应答 `ctrl_reply` / 非本机地址 `ctrl_ignored`，
  `ctrl_rx`（RS485-2 收到的原始字节数）、
  共享缓存深度 `cache`（当前条数/容量）、上报报文数 `batches` / 真实样本数 `samples` /
  0 样本空帧数 `empty` / 未上报丢失样本数 `lost` / 已发布 `published` / 丢弃 `dropped` 与
  MQTT 状态（`disabled` / `up` / `down`），以及管理链路（RS485-1/UART1 裸帧 RPC）的
  `mg_rx`（收到字节）/ `mg_frames`（解析出的请求帧）/ `mg_reply`（应答帧）/ `mg_crc`（CRC 错的帧）/
  `mg_sync`（重同步丢弃字节）/ `mg_drop`（缓冲溢出丢弃）/ `mg_tx`（写回字节）/ `mg_evt`（事件帧），
  以及管理 MQTT JSON-RPC 的 `mgmq_rx`（收到载荷）/ `mgmq_reply`（回发响应）/ `mgmq_ignored` /
  `mgmq_bad`（无法解析）；**正常通信时 `mg_crc`/`mg_sync` 应为 0**；供电电压（ADC0 换算值）单独打印；
- **"MQTT 连上了，但服务端收到的全是 0 样本空帧（`samples 0`）"**：脚本侧其实正常，是电池那路没数据。
  看同一行 `stat` 的三个原始字节计数即可分段定位：
  `jk_rx 0` = RS485-3 一个字节都没进来（接线 A/B、共地、保护板没开显示屏协议或波特率不对）；
  `jk_rx` 在涨但 `jk_rounds 0` = 收到字节但帧不合法（波特率/协议与保护板不一致）；
  `ctrl_rx 0` = 金箭中控（RS485-2）根本没发查询（缓存为空时从机按全 0 应答，上报也只发空帧）。
- Reload 按键长按 3s 重启模组；硬件看门狗（GPIO28 → Air153C/Air153D 的 WTDOG）用**工程自带的
  `air153C_wtd`**（`board.lua` 里是 GPIO28 的唯一写入者）：`init(28)` 把 GPIO28 配成输出并置低，
  开机立刻喂一次（"模块开机第一步需要喂狗一次"），之后独立 sys 任务每 30s 喂一次；每次喂狗是
  **400ms 正脉冲**（拉高 → `sys.timerStart` 400ms 后拉低）。Air153C 超时随电压
  209s@4.3V~283s@3.3V，30s 有约 7 倍余量。

  ⚠ **这套写法是现场试出来的唯一有效版本，别按"官方推荐用法"改回去**：官方默认扩展库
  `exair153x_wdt`（`init{ wdt_pin = 28, auto_feed_period_s = 30 }`，库自带周期喂狗任务、
  `init` 内先喂第一次）在 D700Tm 上**喂不动看门狗芯片**——日志里每 30s 都打"喂狗完成"，
  芯片却照旧按自身超时（Air153C 约 237s@3.8V）切供电复位整机，复位原因报 0（`0 0 0`，
  和正常上电/RESET 一样），现场表现就是"每隔几分钟丢一段数据"且 `stat` 计数每次归零。
  换成 `air153C_wtd` 的 400ms 正脉冲后不再随机重启（2026-10-12 实测）。
  `air153C_wtd.lua` 就在工程根目录、随脚本一起打包；`board.lua` 在**加载期**就 require 它，
  文件丢了会让整个脚本加载失败（不是"跑起来但没喂狗"），所以不要删、也不要改成 `require(变量)`。
- **重启排查**：`board.lua` 不再在开机时打印复位原因，改从管理链路读 `status.get` 的 `reset_reason`
  （`pm.lastReson()` 第 3 值，见 §7.3）：`0` = powerkey/上电开机（**外置看门狗经 PMOS 切供电时
  也报 0**）、`3` = 软件重启、`5` = RESET 脚被拉低（Luatools 下载时的硬复位、或外部电路拉低
  RESET）、`8` = 内部看门狗重启、`9` = 外部重启。
  **实测特征**：外置看门狗一直没吃到狗时，会按芯片自身超时（Air153C 约 237s@3.8V 供电）**稳稳地**
  周期性切供电复位，复位原因 `0 0 0` 且间隔几乎恒定（如 00:35:24→39:22→43:19→47:16→51:13，
  每次 237s）；而 `0 0 5` 基本都紧跟在一次 Luatools 下载之后。
  喂狗就位时开机日志应有这两行：
  `I/user.board GPIO 高电平输出 = 3300 mV（D700Tm 外围电路要求 3.3V）` 与
  `WTD feed task start!`，之后每 30s 一行 `WTD feed #N done.`。
  若现场出现"每隔几分钟丢一段数据"，先对比这段日志与 `stat` 计数是否归零。
- **"喂狗日志一直在打，设备还是每 ~4 分钟重启一次"**（2026-10-11/10-12 两次定位）：第一次是
  GPIO 高电平不够——开机日志里 `bsp_user_init_io ... io volt 1.8v` 说明高电平只有 1.8V，
  3.3V 供电的 Air153C 认不出这个"高"，必须看到开机那行 `I/user.board GPIO 高电平输出 = 3300 mV`
  （`board.lua` 硬编码 `pm.ioVol(pm.IOVOL_ALL_GPIO, 3300)`，且这条在任何喂狗动作之前/紧随其后，
  保证第一条 400ms 喂狗脉冲大部分是 3.3V）；第二次就是喂狗波形本身——3.3V 满足后
  `exair153x_wdt` 仍按 ~237s 周期复位，说明本板 WTDOG 前级接法不认库的波形（库按
  GPIO→NPN→WTDOG 假设设计），换成 `air153C_wtd` 的正脉冲才喂得动。观察期间**别点 Luatools
  下载**（下载本身会硬复位模组，会把结论搅乱），至少看 6 分钟（≈1.5 个超时周期）不重启才算通过。
- 调试开关：`LOG_ENABLE`、`LOG_HEX_PAYLOAD`（整包 hex）、`LOG_RAW_FRAMES`（485 原始帧）。

## 9. 离线验证

工程自带 `tools/selftest.lua`，在 PC 上用 Lua 5.4 运行（不需要硬件）：

```powershell
cd air780epm
lua tools/selftest.lua
```

当前 745 项全部通过（64 位与 32 位整数 Lua 各跑一遍都应是同样的「失败 0 项」，见下方"位宽差异"），覆盖：

> 自测里 MQTT 主题相关的几项按**模板默认** `MQTT_TOPIC_PREFIX = "/bms"` 断言；若本地
> `config.lua` 把它改成了别的值（例如 `/device`），这几项会 FAIL——是本地配置差异，
> 不是代码问题（`config.lua` 不入库）。其余用例与配置无关。

1. `bms_codec`（MessageV1）与 `mqttagent/pkg/bms` §11 的**五条黄金向量字节级比对**：
   11.1 两样本×三电芯 = **53B**、11.2 三样本×两电芯全静态 = **42B**（modeTable 全常量）、
   11.3 四样本×四电芯压差收窄 = **55B**（spatial 布局 + Time 线性）、11.4 单样本无电芯 = **25B**、
   11.5 **0 样本空帧 = 13B**；并核对编码分片让出钩子（每 16 列回调一次、输出与同步编码字节一致）；
2. `sha256`/`hmac`（MQTT 鉴权）：`sha256("abc")`/空串 FIPS 已知答案、RFC 4231 TC2、
   设备 KAT `hmac_sha256("soc","861234567890123") = 1b281b47…9e409a`、输出为小写 hex，
   以及 9 条**填充边界向量**（55/56/57/63/64/65/119/120/128 字节，锁住 64 位比特长度的高/低半字；
   见下方"位宽差异"）；
3. `bms_cache`：32 条环形缓冲 push 40 条验覆盖、`latest`、`history` 闭区间（同秒多条、
   不插值不补零）、`since` 游标不重叠、被覆盖样本计入丢失、空缓存查询；
4. `bms_data` 适配层：display 与 modbus 快照归一字段对照（`tempBat2C`、均衡电流、
   循环总容量、满电容量、BMS 上电时间、`source`、32 路槽位补齐、非法快照返回 nil）；
5. 极空整轮 364 字节样本（见 `docs/jikong/极空显示屏协议.md` §7）逐字段核对：总压/电流/SOC/温度/单体/串数、SOC 图标格数、
   剩余时间 7:39:37（时/分/秒）、u32 功率、`sysAlarm` 位图、颜色帧（蓝=最高第 1 串、
   红=最低第 7 串、绿=普通、黑=空槽）、扩展块与主块交叉校验；并覆盖 37 字节分片接收、
   0x80/0x02 蜂鸣器帧不截断轮、≥25 串的 `0x2011` 第 25 节电压；
6. 金箭从机（**缓存驱动**）：缓存为空时 `0-8`/开关量全 0 应答（不崩、不超时）且 PN/版本照常、
   缓存有一份 display 样本时的 `0-8`/`0-32`/`103-113`/PN/版本/开关量/快充应答、
   写充电设置与回显、越界异常 `03`、非法地址异常 `02`、非本机地址不应答、计数守恒；
7. 端到端：协程调度器真跑 `JK_ROUND → bms_data → bms_cache → bms_uplink → MQTT`，
   用**独立实现的 MessageV1 解码器**逐字段核对——消息级字段（满电容量/`BaseTimestamp`/
   上电时间/两个 modbus 计数）、`Time = BaseTimestamp - ts`、各列（温度/均衡电流/总压/电流/
   循环总容量/剩余容量/单体）与鉴权参数（clientId=用户名=IMEI、密码=HMAC 小写 hex）、主题、QoS；
   并覆盖「未 NTP 对时不发布」「窗口非空发真实样本」「窗口为空只发 0 样本帧（不补零/不 keep-last）」；
8. 极空 RS485 Modbus 主站（`jk_modbus`）：用 `uart.write` 打桩模拟一台从机，核对
   请求组帧（`01 03 1200 0020` / `1240 0002` / `128A 003A`）与 CRC（文档样例 `0x0AD4`/`0xC64B`）、
   三块寄存器的解析与单位换算（总压/电流/SOC/单体/温度/均衡电流/容量/循环/SOH/
   `0x12BC` RunTime）、`CellSta` popcount 串数、异常响应降级（不产出新样本、保留上一轮），
   以及同一份规范样本驱动金箭从机寄存器（`reg104` 来自 Modbus、保护开关量）与
   `SOCCycleCap`(列) / `SOCFullChargeCap`(消息级) 的语义核对；
9. 单线程/不阻塞约定：编码分片钩子、原始帧 hex 限频（每 `LOG_HEX_EVERY_N` 帧一条），
   以及"串口回调只搬字节、解析由独立任务完成"的实现验证——金箭回调本身不应答、
   解析任务产出应答、突发 12 帧按 `JJ_PARSE_MAX_FRAMES` 分批处理，极空整轮同理
   （回调不解析、任务产出快照）。
10. `mgmt_uart` 帧编解码与重同步（§7.1）：CRC16(`0xA001`，样例如 `0xCC85`)、组帧
    （SOF/VER/TYPE/ID/LEN 小端/CRC）、扫帧（前导垃圾重同步计入 `sync`、CRC 错、VER 不符、
    JSON 超长 >1024 / BLOB 超长 >242 分别计数丢弃）、**分片到达**、一包多帧、尾部不完整保留；
11. `mgmt` 统一管理服务（§7.2/§7.3）：方法分发（`sys.ping`/`status.get`/`config.get`/
    `config.set`/`cmd.reboot`/`cmd.factory_reset`/`fota.url`/`fota.begin`/`fota.chunk`/
    `fota.abort`/`fota.status`）与错误码（`EBADJSON`/`EBADMETHOD`/`EBADPARAM`/`EBADKEY`/
    `EBADVALUE`/`EBUSY`/`ENOLIB`/`EFOTA`/`EBADSEQ`/`EFLASH`）；`config.set` 白名单 +
    范围校验 + **整批原子**（混入非法键则一项都不落盘）+ `restart_required`；`cmd.reboot`
    **回帧先于动作**（动作排期 200ms）；`cmd.factory_reset` 清覆盖并置待生效；
    **ID 去重**（同 ID + 同方法在窗口内只执行一次、超出窗口/换方法/换 ID 正常执行）；
    FOTA 编排（`fota.url`/`fota.begin`/`fota.chunk` base64/`fota_blob` BLOB/`fota.abort`）；
    `hello` 字段与 `fota` 事件（`MGMT_EVT`）触发；
12. `cfg_store`：白名单/类型/范围校验（未知键、越界、枚举、类型、超长全部拒绝）、写透往返
    （打桩 fskv 存 JSON 文本）、重启等价（新实例 `load()` 把覆盖在业务模块加载前写回 config）、
    非法/未知项跳过、恢复出厂清键并置待生效、fskv 缺失时降级为「不持久化」但本次运行仍生效；
13. `fota_mgr`（原 `fota`，改名见 §8 5.2.1）：
    URL 通道——打桩 `libfota2` 下的状态机（idle→下载中→成功待重启/失败）、进度回调与结果码、
    未联网不发起下载（联网后才调 `request`）、忙时拒绝、空 URL 拒绝、库缺失时优雅降级；
    本地通道——打桩核心库 `fota`（**按真实返回形状**：`run → (ok, isDone, cache)`、
    `isDone → (ok, done)`）下的状态机（idle→preparing→ready→receiving→verifying→
    success）、init/run 失败、`isDone` 超时·报错、CRC32 不匹配、主动中止、空闲超时、
    核心库缺失（`-1`）、同块幂等重试（不重复写 Flash）、顺序/偏移/长度校验、两条通道互斥、
    不调用 legacy `fota.wait()`（返回 false 也不影响）、`run` 第三个返回值作为未落盘缓存上报、
    全程不联网也不碰 `libfota2`；另核对标准 CRC32 向量与一条**位宽守卫**用例
    （新模块源码里不允许出现 32 位整数 Lua 装不下的整数字面量，见下方"位宽差异"）；
14. `mgmt_uart` 传输（§7.1）：管理链路走 UART1 时不做 `pins.setup`（默认复用功能）、串口参数
    （115200 8N1、缓冲、`dir=GPIO25` 方向脚）、回调只搬字节 + 独立任务应答、前导垃圾重同步
    计入 `sync`、CRC 错不应答并计入 `crc`、未知方法回错误信封、BLOB 数据块写入（含长度不符
    回 `EBADFRAME`）、突发 12 帧按 `MGMT_PARSE_MAX_FRAMES` 分批处理、`evt.hello` 主动事件、
    `MGMT_EVT`/`push_evt` → `evt.fota` 写回、`mgmt` 抛异常只丢该帧不影响后续；
    `uart_pins`：极空 UART3 链路的引脚复用顺序（`UART3_RX`→PIN53、`UART3_TX`→PIN52，
    D700Tm 实测结论，顺序不能反）、两种功能名回退、`nil` 引脚整段跳过；
15. `mgmt_mqtt`（管理 JSON-RPC，§7.7）：未启用时 `open()` 返回 `disabled` 且**不订阅任何主题**；
    通道主题 = `<prefix>/<device>/<后缀>`（默认 `rpc`，后缀可配、空值/多余斜杠都回退到 `rpc`）、
    订阅下行主题 + 响应发到 `.../<后缀>/resp`；事务号（`id`）原样回带、无 `id` 也可回；
    非 JSON/空载荷只丢该条（`bad` 计数、不影响后续）、未知方法回错误信封、
    `fota.chunk` 的 base64 解码 + 偏移自动推得、MQTT 未就绪时响应丢弃并计数；
    无链路/取不到设备 ID 时 `open()` 明确返回原因；另在 §16 的 `main.lua` 整链路 smoke 里核对
    **开机接线**：`net_ready` 后订阅 `<prefix>/<IMEI>/rpc`，从 MQTT 下行喂一帧 JSON-RPC 请求能
    拿到 `.../rpc/resp` 响应（走的是与上报同一条连接）；
16. `main.lua` 开机整链路 smoke：真跑 `main.lua`，从 UART1 接收回调喂一帧裸帧 `status.get`
    能拿到 RESP（字段含 `device`/`links`/`cache`/`uplink`/`fota.caps`）；管理 MQTT RPC 随 main
    接线（`net_ready` 后订阅、下行请求得上行响应）；17. `mgmt_uart` 诊断模式（`MGMT_ECHO_TEST`）
    回环 + 心跳；18. 串口分配（极空 UART3 需引脚复用、管理 UART1 默认复用）。

另有仓库根目录的 `hwsim.lua`（`lua hwsim.lua`，虚拟时钟跑真实链路，不需要硬件）：
默认仿真 900s、前 300s 每 ≈2.17s 喂一轮广播、之后模拟保护板掉线，核对
「10 个真实批次（138 条样本）无重复时间戳、无时间戳倒退」「掉线后只发 0 样本空帧」
「每个上报周期恰好 1 包」，并可 `DUMP=<file> lua hwsim.lua` 导出 payload hex，
交给服务端解码器（`mqttagent/pkg/bms`、`cmd/mock`、`cmd/agent` 同源）再解一遍。

另有 `tools/wdt_selftest.lua`（`lua tools/wdt_selftest.lua`，38 项全通过）：用打桩的
`sys`/`gpio`/`log`/`pm`/`pins`/`adc`/`air153C_wdt` 真跑 `board.lua`（板级资源已硬编码），
覆盖看门狗这一环——`air153C_wtd.init(28)` 建初始电平、加载期先喂第一次、独立任务每 30s
喂一次且每次都是 **400ms 正脉冲**（核对脉冲宽度与收尾电平）、GPIO28 上只有 `board.lua`
一个写入者；喂狗库没随脚本打包时 `board.lua` **加载期就报错**（不是静默不喂狗）；
开机把 GPIO 高电平设成 3.3V（以及老固件没有 `pm.ioVol` 的边界）；
另覆盖引脚复用按固定路径 `/luadb/pins_Air780EPM.json` 加载并记录成功日志、表没随脚本下发或
老固件没有 `pins.loadjson` 时只告警不崩（都不影响喂狗）、供电电压 ADC0 的换算。

### 位宽差异：PC 的 64 位 Lua ≠ 固件的 32 位 Lua（2026-10-12 踩坑）

固件里的 Lua 是 **32 位整数**（`LUA_32BITS`，开机日志 `LuatOS@Air780EPM … 32bit`），而 PC 上的
Lua 5.4 是 64 位整数，于是有一类 bug **PC 自测全绿、烧到设备才炸**：

- `0x100000000` 在 32 位下回绕成 `0`，`sha256.lua` 里 `bits // 0x100000000` 就成了「除以 0」。
  现场表现：烧录后 `E/user.coroutine.resume sha256.lua:50: attempt to divide by zero`（MQTT 鉴权
  算密码时抛错）→ `Lua VM exit!! reboot in 15000ms`，设备每 15s 重启一次；
- `v > 0xFFFFFFFF`（高位常量比较）同理恒为真 → 上报的总压/剩余容量被钳成 0（无符号列写出 0）。
- **本次新增的 CRC32（`fota_mgr.crc32`，§7.5）正是按这条规则写的**：掩码与多项式都用能装进
  32 位整数的片段拼出来（`MASK32 = (0xFFFF << 16) | 0xFFFF`、`CRC32_POLY = (0xEDB8 << 16) | 0x8320`），
  **不能**直接写 `0xFFFFFFFF` / `0xEDB88320`，而且每一步移位/异或后都重新掩码；
  离线自测里有一条"位宽守卫"用例（§13 末）专门卡这类字面量。
  （PC 侧探针 `Get-Crc32` 有同一个坑的另一半：PowerShell 的 `0xEDB88320` 会解析成
  负的 Int32，`[uint32]0xEDB88320` 直接抛异常——所以那边用 `[int64]` + 掩码实现。）

结论：**位运算 / 高位常量 / 计时（`mcu.ticks`）/ `os.time` 相关的改动，必须用 32 位整数 Lua 再跑一遍
同一套自测**（两份 Lua 的结果应一致：失败 0 项；`tools/selftest.lua` 里的「位宽守卫」用例会把
`fota_mgr.lua` / `mgmt_mqtt.lua` 源码里装不进 32 位的整数字面量直接卡住）。管理 RPC 这层
到处是 16 位寄存器拼装与位移（`str_reg`/`str_write`/状态位域/uptime 高低字），所以两份 Lua
都必须跑绿。可以自己构建一份：

```bash
curl -fsSL -o lua.tar.gz https://www.lua.org/ftp/lua-5.4.6.tar.gz
tar xzf lua.tar.gz && cd lua-5.4.6
# 注意：不能靠 -DLUA_32BITS 编译宏——luaconf.h 里已经写了 #define LUA_32BITS 0，后者会覆盖它，
# 必须改头文件本身：
sed -i 's/^#define LUA_32BITS.*/#define LUA_32BITS 1/' src/luaconf.h
make posix
./src/lua -e 'print(math.maxinteger)'    # 输出 2147483647 才是 32 位整数
cd <工程>/air780epm && <刚构建的>/src/lua tools/selftest.lua
```

## 10. 已知取舍

| 项 | 说明 |
|---|---|
| 极空数据源 | **配置二选一**：`JK_PROTOCOL="display"` 用显示屏协议广播（2400 8N1 只收），`"modbus"` 用 RS485 Modbus 主站轮询（115200）；广播缺的字段（循环次数、名义容量、充电设置等）用本地缓存/推算，Modbus 可直接读到 |
| 采样粒度 | display 一条样本 = 一轮广播（本机 ≈2.17s，周期随固件版本 0.5–2.2s 变化）；modbus 一条样本 = 一次完整轮询（默认 1s，`JK_MODBUS_POLL_MS` 可调）。上报按 `REPORT_PERIOD_MS`（默认 30s）固定周期取「窗口内新到达的真实样本」，一包 1–30 条不等 |
| 共享缓存 | `BMS_CACHE_SAMPLES`（默认 32）条环形缓冲：display ≈69s、modbus ≈32s 的历史，既覆盖 30s 上报窗口，也给消费端偶发卡顿留余量。金箭从机与 4G 上报共用这一份缓存，各自只读；被覆盖而没来得及上报的样本计入 `stat` 的 `lost` |
| 0 样本帧 | 窗口内没有新样本（保护板掉线/未接入/这几秒没数据）时只发 MessageV1 的 0 样本帧（`flags bit10`，`aa 01 00 80 08` + 6 个消息级 uvarint，典型 11–20 字节；§11.5 黄金向量为 13 字节）。里面只有上电时间与两个 modbus 计数有意义，没有列数据，服务端不会把它当真实采样写库（`cmd/agent` 只为 `TimeSeries` 非空的报文写 `soc_full_charge` 等指标） |
| 温度列 | display 主块只有一路电池温度（固件按工况在 T1/T2 间取一个填入），`temp1=temp2`；modbus 取 `0x129C`/`0x129E` 两路传感器。`tempMos` 两路一致 |
| 均衡电流 | display 不提供，`balanCurrent` 固定 0；modbus 取 `0x12A4`（mA） |
| 满电总容量 | display 不提供，用 剩余容量 ÷ SOC 估算（SOC < 5% 时保持上次值）；modbus 直接取 `0x12AC`。上报时进**消息级** `SOCFullChargeCap`（不是任何一列），金箭 `reg3`/`reg113` 也用它 |
| 循环次数/循环容量 | display 广播里没有：`cycleCount`/`bmsUpTimeS` 归一为 0，金箭 `reg104` 回退本地缓存（默认 0）；modbus 取 `0x12B0`（循环次数 → 金箭 `reg104`）、`0x12B4`（循环总容量 → 上报列 `SOCCycleCap`）、`0x12BC`（RunTime → 消息级 `BMSUPTime`） |
| 报警位 | 两套协议的 `sysAlarm` 32 位定义一致（bit1 MOS 过温、bit7 充电短路、bit8 充电过温、bit9 充电低温、bit14 放电短路、bit15 放电过温），故金箭保护开关量的判断逻辑两种数据源共用 |
| 单体颜色帧 | RGB565 颜色码（非均衡线电阻表），本项目仅用于诊断 |
| 串数上限 | 主块 24 槽 + `0x2011` 第 25 节 = 25 路；车端金箭链路按协议只暴露 24 路 |
| 硬件看门狗 | 用工程自带的 `air153C_wtd`（`init(28)` + 开机喂一次 + 每 30s 一个 400ms 正脉冲），**不用**官方默认扩展库 `exair153x_wdt`：后者在 D700Tm 上按时打"喂狗完成"但 Air153C 照旧按自身超时（实测 237s@3.8V）复位整机。代价是喂狗波形写死在 `board.lua`，换板/换看门狗芯片要重新实测波形；且该库文件必须在脚本里（丢了会导致脚本加载失败，不是静默降级） |
| RS485-1 管理链路 | **一个统一管理服务 + 两条传输**：UART1 走自定义裸帧 RPC（115200 8N1，点对点无从站地址）、MQTT 走 JSON-RPC（通道后缀 `rpc`）。只应答管理主站的请求，主站不来问就一个字节都不发；写配置走白名单 + 范围校验并**写透** fskv，`[R]` 项重启生效；命令（重启/FOTA/恢复出厂）**回帧先于动作** + 10s/ID 去重。管理链路与极空（RS485-3）、金箭（RS485-2）物理/逻辑独立，互不影响 |
| 配置持久化 | fskv 明文存「覆盖项 JSON」，只存被主站改过的键（不是整份 config），开机在业务模块 `require` 之前写回 config；`CFG_STORE_ENABLE=false` 或 fskv 不可用时退化为"每次重启都回 config.lua"，只告警不报错。MQTT 口令随其它配置一起明文存储（沿用"凭据随脚本下发"的既有基线，不引入加密） |
| FOTA（URL 通道） | 管理主站下发 URL 触发 `libfota2` 下载；**下载成功不自动重启**（置状态 2 + 「需重启」，等主站发重启命令），现场可选合适时间点重启，避免下完就断服；FOTA 与上报共用 4G 链路，未联网不启动（限时等待后置失败） |
| FOTA（本地通道） | 主站用 UART BLOB 帧 / MQTT `fota.chunk` 把 LuatOS `.bin` 载荷**分段流式**写进设备（核心库 `fota.init/run/isDone/finish` 直接写 Flash）：不联网、不落文件、不依赖 `libfota2`。**偏移由设备按已收字节自动推得**（主站只给 SEQ）；收齐后（可选）校验整包标准 CRC32，成功同样只置「状态 2 + 需重启」，由主站下重启命令；同块重发幂等、60s 空闲自动取消、失败/中止重启仍是旧版本。代价：需要主站按顺序分块（§7.5），且**不做签名校验**（与 URL 通道同一安全基线） |
| 管理 MQTT JSON-RPC | 与 UART1 并列的第二条管理传输（§7.7），复用同一个 `mgmt` 服务的方法/字段/错误码；**通道 = 主题后缀**（默认 `rpc`，下行 `<prefix>/<device>/rpc`、上行 `.../rpc/resp`），`MGMT_MQTT_ENABLE` 默认开。代价：与上报**共用同一条 MQTT 连接**（clientId=IMEI 只允许一条），所以 `MQTT_HOST` 未配置、或 broker 掉线时这条通道也跟着不可用；载荷是明文 JSON（与 RTU 同一安全基线，不带签名/加密）；响应 QoS 取 `MQTT_QOS`（默认 0，偶发丢包由主站超时重试兜底） |
| 保留字段 | 扩展块偏移 36/38（内部状态 A/B）含义未定，只存不判 |

## 11. 单线程不阻塞约定

LuatOS 是**单线程协作式**调度：任何任务长时间不让出，都会拖慢串口收发与其它任务。
本工程按下面的约定实现，改动代码时请一并遵守（`tools/selftest.lua` 有对应回归用例）。

1. **每个循环都有让出点**：所有 `while`/`repeat` 要么调用 `sys.wait`/`sys.waitUntil`，
   要么可证明每次迭代都消费字节而收敛（帧解析的 `parse_buffer`/`scan_frame`/`extract_frames`）。
   没有 `while true do end` 之类的忙等。
2. **串口接收回调只搬字节**：`jk_display.lua`、`jinjian_slave.lua`、`mgmt_uart.lua` 的
   `on_receive` 只做 `uart.read` → 追加缓冲 → `sys.publish("…_RX")`，
   帧扫描/解析/应答全在各自的任务里；
   突发帧流按 `JJ_PARSE_MAX_FRAMES`（默认 8 帧）分批处理、批间 `sys.wait(1)` 让出。
   管理链路同理：帧扫描/JSON 编码/方法分发都是纯函数（只校验 + 组帧，不阻塞），
   命令类动作（重启/FOTA）也在响应帧之后才排期；本地 FOTA（§7.5）更进一步——请求路径里
   只做整帧校验 + `fota.run(块)`（几十字节的常数时间工作），`fota.init()`、
   收尾（等 `isDone` + CRC32 + `finish`）都在**后台任务**里跑，60s 空闲超时用独立任务计时，
   所以「主站发一个 238 字节的块」不会把从机任务卡住（§11.2 的 `sys.wait` 只在后台任务里）。
   `jk_modbus.lua` 是唯一例外：它只做帧定界 + CRC（常数时间、单帧 ≤245 字节），
   且处在请求/应答的时序路径上，多一次任务唤醒会直接影响超时判定，故保留在回调内。
3. **长 CPU 段分片让出**：`bms_codec.encode(hist, meta, on_slice)` 每编码 16 列回调一次，
   `bms_uplink` 借此 `sys.wait(cfg.ENCODE_YIELD_MS)`（默认 1ms，`0` = 不让出）。
   实测编码 14×20 ≈0.6ms、30×32 ≈1.3ms（PC Lua，模组更慢），分片后单段更短。
4. **同步外设逐次让出**：供电电压采样按 `board.lua` 的 `ADC_SAMPLES`（默认 3）逐次 `adc.get`，
   每次之间 `sys.wait(1)`（单次 ADC 转换是同步的，部分固件是忙等）。
5. **日志限频**：`LOG_RAW_FRAMES` 打开时，原始帧 hex 每 `LOG_HEX_EVERY_N`（默认 20）帧
   打一条（轮询每秒 3 帧、单帧最多 490 字符 hex，不限频会自己挤占调度）。
6. **失败退避**：Modbus 轮询整轮失败后按 `JK_MODBUS_RETRY_MS`（默认 2s）退避，
   连续失败只在第 1、10、20… 次打日志；`JK_MODBUS_TIMEOUT_MS` 默认 120ms
   （3 块最坏 360ms，不会长时间占住总线）。
7. **等待＝挂起协程**：`sys.waitUntil` 事件不来时任务休眠、不占 CPU；几处"无超时等待"
   完全是设计意图（如等 `JK_ROUND`）。为避免"卡住看不出"，等 `net_ready` 这类关键等待
   改成"限时等待 + 周期打印"，超时只是可观测性，不是阻塞。
8. **事件不排队**：`sys.publish` 只保留最后一次事件值，消费端不及时会丢轮。
   `JK_ROUND` 只有 `bms_data` 一个消费端，每条只做一次字段映射（微秒级）后立即 `push`
   进共享缓存；金箭从机与 4G 上报都不再监听事件，而是**只读缓存**（`latest` / `since`），
   所以上报任务偶发卡顿只影响「窗口大小」，不会丢轮、也不会复用到正在编码的缓冲。
   缓存被环形覆盖而没来得及上报的样本计入 `stat` 的 `lost`（`bms_uplink.get_stats().lost`）。
9. **喂狗也是普通 sys 任务**：`board.lua` 的看门狗任务 `sys.wait(WTD_FEED_MS)`（30s）挂起
   协程，脉冲收尾靠 `sys.timerStart` 的 400ms 定时器，全程不忙等（`tools/wdt_selftest.lua`
   用虚拟时钟核对脉冲宽度）；喂狗库在**加载期**就 `require`，取不到会直接报错退出，
   不会留下"脚本能跑但没喂狗"的隐患。

缓冲余量（衡量"卡住多久才会丢数据"）：

| 链路 | 缓冲 | 波特率 | 可容忍停顿 |
|---|---|---|---|
| 极空 display | 2048 B | 2400 | ≈8.5 s |
| 金箭从机 | 512 B | 9600 | ≈533 ms |
| 极空 modbus | 512 B | 115200 | ≈44 ms（最紧，故回调内只做常数时间工作） |
| RS485-1 管理 RPC | 2048 B | 115200 | ≈178 ms（回调只搬字节，帧扫描/应答在任务里） |
| 共享采样缓存（上报） | 32 条 | — | display ≈69 s / modbus ≈32 s（上报任务卡顿超过这个时长才开始丢样本，计入 `lost`） |
