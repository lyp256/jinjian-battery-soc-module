# Air780EPM 管理链路文档（Mgmt RPC）

> 对应固件侧：`air780epm/mgmt.lua`（统一管理服务）+ `air780epm/mgmt_uart.lua`（UART1 裸帧传输）
> + `air780epm/mgmt_mqtt.lua`（MQTT JSON-RPC 传输）+ `cfg_store.lua`（持久化）+ `fota_mgr.lua`（升级）。
> PC 侧验收工具：`air780epm/tools/mgmt_probe.ps1`。README 摘要见 `air780epm/README.md` §7。

## 1. 概述

RS485-1（UART1）是**运维/管理链路**：上位机当主站，本机当从机。它是一条**一对一专用线**——
点对点、**没有从站地址/广播/寄存器表**，主站一次只发一个请求。请求是 **JSON-RPC**
（`{"method","params"}`），只是外层套了一帧二进制头 + CRC 便于串口定界和容错。

管理侧有**两条传输**，共用同一套方法、字段与错误码：

| 传输 | 文件 | 帧 | 事件 |
|---|---|---|---|
| UART1 裸帧 RPC | `mgmt_uart.lua` | 二进制帧头 + JSON 载荷 + 二进制 BLOB 分块 | **有**（`evt.hello` / `evt.fota`） |
| MQTT JSON-RPC | `mgmt_mqtt.lua` | `{"id",…,"method","params"}` / `{"id",…,"ok",…}` | 无（纯请求-应答） |

改动任何方法/字段/错误码，都要**同时**照顾两条传输（都由 `mgmt.lua` 实现，传输层只搬字节）。

## 2. 链路参数（UART）

| 项 | 值 |
|---|---|
| 串口 | UART1（RS485-1），**115200** 8N1，无校验、无流控 |
| 引脚 | PIN18=UART1_TXD、PIN17=UART1_RXD（**默认复用**，不用 `pins.setup`） |
| 方向 | `MGMT_RS485_DIR=25`（EN=GPIO25），由串口驱动按收发自动切换；少了它完全收发不了 |
| 缓冲 | `MGMT_RX_BUFF_SIZE=2048`；单帧最大 `MGMT_MAX_FRAME_LEN=1200` |
| 配置键 | `MGMT_UART_ID` / `MGMT_BAUD`（`[R]`）/ `MGMT_RS485_DIR` / `MGMT_RS485_RX_LEVEL` / `MGMT_RS485_DELAY_US` / `MGMT_MAX_PAYLOAD` / `MGMT_DEDUP_WINDOW_MS` / … |

## 3. 帧格式与通用语义

| 字段 | 长度 | 说明 |
|---|---|---|
| SOF | 2 | `0xA5 0x5A` |
| VER | 1 | `0x01` |
| TYPE | 1 | `0x01` REQ / `0x02` RESP / `0x03` EVT / `0x04` BLOB |
| ID | 1 | `1–255` 请求号；EVT 固定 `0`；RESP 原样回带 |
| LEN | 2 | 载荷长度，**小端**；JSON ≤ 1024，BLOB 载荷 ≤ 242 |
| PAYLOAD | LEN | 见下 |
| CRC16 | 2 | 多项式 `0xA001`，**小端**，覆盖 `VER…PAYLOAD` |

```
REQ  = {"method":"...","params":{...}}          （无 params 时可省略）
RESP = {"ok":true,"result":{...}} 或 {"ok":false,"error":{"code":"...","msg":"..."}}
EVT  = {"evt":"hello|fota","data":{...}}
BLOB = SEQ(2 LE) | LEN(2 LE) | DATA(LEN)
```

**示例**（`sys.ping`，ID=1）：

```
REQ  A5 5A 01 01 01 1A 00 {"method":"sys.ping"}            CRC16(VER..PAYLOAD) 低字节在前
RESP A5 5A 01 02 01 LEN 00 {"ok":true,"result":{...}}      （LEN = JSON 字节数）
BLOB A5 5A 01 04 ID 06 00  01 00 EE 00 <238 bytes>          （SEQ=1, LEN=238）
```

通用语义：

* **重同步**：扫描器遇到非 SOF 丢弃并计入 `sync`；CRC 错 / VER 不符 / 超长分别计数丢弃；
* **同一时刻只允许一个请求在飞**（点对点，主站串行发）；
* **回帧先于动作**：命令在响应发完之后才排期（重启延时 200 ms）；
* **ID 去重**：同一 ID 在 `MGMT_DEDUP_WINDOW_MS`（默认 10 s）内重复到达且方法相同 → 直接重放
  缓存响应、**不重复执行**（防丢应答后主站重试造成二次重启/二次 FOTA）；窗口按"最后一次执行"
  起算，超出窗口、换方法、换 ID 都会重新执行。

> ⚠ **主站每次请求都要换新 ID**（`1–255`）。去重按「ID + 方法」生效，**读方法也一样**：10 s 内
> 用同一个 ID 再发一次 `status.get`/`config.get` 会拿到缓存响应，不是新采样。

## 4. 方法表

| 方法 | 参数 | 返回 | 说明 |
|---|---|---|---|
| `sys.ping` | — | `{time, uptime_s}` | 链路连通性自检 |
| `status.get` | — | 状态大对象（§5） | 一网打尽运行状态 |
| `config.get` | — | `{values, restart_keys[], needs_restart, has_override}` | 仅 `cfg_store.SPEC` 白名单 |
| `config.set` | `{KEY:val, ...}` | `{applied, restart_required}` | 先全量校验、再原子写入 |
| `cmd.reboot` | — | `{rebooting=true}` | 回帧后 200 ms `rtos.reboot()` |
| `cmd.factory_reset` | — | `{factory_reset=true}` | 清 fskv 覆盖 + 置待生效位 |
| `fota.url` | `{url}` | `{started=true}` | 启动 URL 升级（需 `libfota2`） |
| `fota.begin` | `{total, crc32?}` | `{started=true,total}` | 开始本地分段升级 |
| `fota.chunk` | `{seq, data(base64)}` | `{seq,rx,total,progress}` | **仅 MQTT**；UART 用 BLOB |
| `fota.abort` | — | `{aborted=bool}` | 中止本地会话 |
| `fota.status` | — | `{mode,state,stage,result,progress,rx,total,last_seq,caps}` | 升级状态 |

## 5. 字段

`status.get`：

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

* `reset_reason` = `pm.lastReson()` 第 3 值（0 上电/看门狗切供电、3 软重启、5 RESET 脚、
  8 内部看门狗、9 外部重启）；
* `supply_mv`（ADC 分次采样）与 `device.pn`（IMEI AT 查询）由**后台任务**刷新，**不在请求路径里** ——
  方法调用是纯函数、不挂起；
* `caps`：`core` 核心库 `fota`（本地分段升级必需）/ `core_type` 它的 Lua **类型名字符串**
  （`"userdata"` = 正常 LuatOS `rotable`、`"table"` = 别的实现、`"nil"` = 固件真没这个库）/
  `lib` `libfota2`（URL 升级必需）/ `local_enabled`·`url_enabled` 通道开关 / `fskv`·`json` 可用；
* `fota` 的 `mode` 0 无 / 1 URL / 2 本地；`state` 0 idle / 1 进行中 / 2 成功待重启 / 3 失败；
  `stage` 0 idle / 1 preparing / 2 ready / 3 receiving / 4 verifying / 5 success / 6 failed。
* `mode` 的来源判定优先级：URL 进行中 > 本地会话进行中 > **URL 会话已结束（成功/失败）** >
  本地会话已结束 —— 保证 URL 通道的结果码不被上一次本地升级残留的 `stage/result` 盖掉。
* `links.mgmt_uart` 是本请求处理中的快照：`frames` 已含本次请求、`reply` 尚未计入本次应答，
  所以每次读都是 `reply = frames - 1`（正常现象，不是丢应答）。

## 6. 错误码

| 码 | 含义 |
|---|---|
| `EBADJSON` | 载荷不是合法 JSON / 不是对象 |
| `EBADFRAME` | 帧结构非法（BLOB 长度不符等） |
| `EBADMETHOD` | 未知方法 |
| `EBADPARAM` | 参数缺失 / 类型不符 / URL 为空 / total ≤ 0 |
| `EBADKEY` | `config.set` 里有不在白名单的键 |
| `EBADVALUE` | `config.set` 里有取值非法的键 |
| `EBUSY` | FOTA 通道正忙 |
| `ENOLIB` | 需要的库缺失（`libfota2` / 核心库 `fota` / `json`） |
| `EFLASH` | 写 Flash 失败（`fota.run`/`finish`） |
| `ECRC32` | 整包 CRC32 不匹配 |
| `EBADSEQ` | 数据块序号/偏移/长度/会话状态不对（乱序、跳号、会话已结束等） |
| `EFOTA` | FOTA 其它失败 / 通道关闭 |
| `EINTERNAL` | 方法内部异常 |

## 7. 配置持久化（`config.get` / `config.set`）

`config.set` 的键名沿用现有大写名；**先全量校验，再原子写入**（任一项非法 → 整批不落盘）。写入走
`cfg_store`：改 `config` + 立即落盘（fskv）+ `[R]` 键置「需重启」。`config.get` 返回白名单键的当前
生效值（有覆盖则是覆盖值）+ `restart_keys` + `needs_restart` / `has_override`。

白名单（`cfg_store.SPEC`）：

| 键 | 范围 / 取值 | 生效 |
|---|---|---|
| `MGMT_BAUD` | 1200/2400/4800/9600/19200/38400/57600/115200 | `[R]` |
| `LOG_ENABLE` | true/false | 实时 |
| `JK_MODBUS_POLL_MS` | 100–60000 | 实时 |
| `JJ_LOW_TEMP_CHARGE_C` / `JJ_LOW_TEMP_DISCHARGE_C` | -100…100（℃） | 实时 |
| `JJ_FAST_CHARGE_C_RATE` | 0–10（C，充电倍率，可为小数；判定电流 = 倍率 × 容量 Ah，`1` = 1C，`0` = 只要在充电） | 实时 |
| `REPORT_PERIOD_MS` / `STATUS_REPORT_MS` | 1000–3600000 | `[R]` |
| `JK_PROTOCOL` | `display` / `modbus` | `[R]` |
| `JK_MODBUS_SLAVE_ADDR` | 1–247 | `[R]` |
| `MQTT_PORT` `MQTT_IS_SSL` `MQTT_QOS` `MQTT_KEEPALIVE_S` `MQTT_HOST` `MQTT_TOPIC_PREFIX` | 见 `config.lua` | `[R]` |

> 开机 `load()` 时不在白名单的覆盖项会被跳过并打日志。
> `cmd.factory_reset` 清掉 fskv 覆盖并置待生效位，**重启后**回 `config.lua` 默认值。
> 快充倍率是小数（`float` 类型），按 JSON number 传（`0.5`），字符串 `"0.5"` 也接受。

## 8. 事件（仅 UART）

| 事件 | 时机 | 内容 |
|---|---|---|
| `evt.hello` | mgmt 串口打开后约 1 s 发一次 | `project/model/version/pn/reset_reason/uptime_s/proto/caps` |
| `evt.fota` | FOTA 阶段/状态变化时发（失败也发一次） | `mode/state/stage/result/progress/rx/total` |

设备侧通过 `sys.publish("MGMT_EVT", {evt=..., data=...})` 推给 UART 传输，传输排队后写回。
MQTT 传输**不发事件**。`mgmt_probe.ps1 -Action listen` 可监听这两类事件。

## 9. FOTA 通道 A：URL 升级（`fota.url`）

主站发 `fota.url {"url":"http://ota.example/fw.binpkg"}` → 设备等联网（`MGMT_FOTA_NET_WAIT_MS`）
→ `libfota2.request` 下载 → `state` 1→2、`progress` 100；主站读 `fota.status` 确认后再
`cmd.reboot` 进新版本（**成功不自动重启**）。失败 → `state=3` + 结果码；忙 → `EBUSY`；
缺库 → `ENOLIB`。`MGMT_FOTA_ENABLE=false` 可整体关闭。

## 10. FOTA 通道 B：本地分段上传（BLOB / `fota.chunk`，不联网）

### 10.1 开关与参数（`config.lua`）

| 配置项 | 默认 | 说明 |
|---|---|---|
| `MGMT_FOTA_LOCAL_ENABLE` | `true` | 关掉后整段返回 `EFOTA` |
| `MGMT_FOTA_LOCAL_TIMEOUT_MS` | `60000` | 空闲超时：这么久没有新数据块 → 自动 `fota.finish(false)` |
| `MGMT_FOTA_LOCAL_FINALIZE_MS` | `3000` | 收齐后等核心库落盘（`fota.isDone()`）的上限 |
| `MGMT_FOTA_LOCAL_WAIT_MS` | `3000` | `fota.init()` 后轮询 `fota.wait()` 等底层就绪的上限（超时只告警仍继续） |

### 10.2 升级包格式限制

收的是 **Luatools 产出的 LuatOS `.bin` 升级载荷**（脚本单包或 core+script 差分包），**不是**
约 2.6 MB 的 `.binpkg` 容器。`total` 是 u32；`crc32` 填 **0 = 不校验**，填值 = 主站对**整包**算的
标准 CRC32（IEEE 802.3，多项式 `0xEDB88320`，初值/结果取反，与 `zlib.crc32` 一致）。

### 10.3 数据块形态

* **UART BLOB 帧**：`TYPE=0x04`，载荷 `SEQ(2 LE) | LEN(2 LE) | DATA(LEN)`，`DATA ≤ 238`；
  **偏移由设备按已收字节自动推得**（主站只给 SEQ）。设备回一条 RESP（同一 ID）。
* **MQTT `fota.chunk`**：`{"seq":<从 1 递增>,"data":"<BASE64>"}`，解码后走同一条写入路径。

### 10.4 升级流程（主站视角）

1. `fota.status` 确认没有进行中的会话；必要时 `fota.abort`；
2. `fota.begin {"total":<包长>,"crc32":<可选>}` → 轮询 `fota.status` 等 `stage=ready`(2)
   （`init` 在后台跑，没就绪就发块会被判 `EBADSEQ`）；轮询必须**每次换新 ID**，
   同 ID + 同方法在 10s 内会重放缓存响应、读不到新快照；
3. 按 SEQ 顺序发块（UART BLOB / MQTT `fota.chunk`）；设备逐块 `fota.run()`；
   同 SEQ/偏移/长度的重发按幂等重试处理；
4. 收齐 → 等 `fota.isDone()`（≤ `MGMT_FOTA_LOCAL_FINALIZE_MS`）→（可选）CRC32 校验 →
   `fota.finish(true)` → `stage=success`(5)、`state=2`、进度 100%、置「需重启」（**不自动重启**）；
5. 主站确认 `stage=5` 后发 `cmd.reboot`；
6. 失败/中止 → `stage=failed`(6)，失败码进 `result`。

### 10.5 异常码 / 结果码

* `fota.begin`：参数不符 → `EBADPARAM`；忙 → `EBUSY`；核心库缺失 → `ENOLIB`；通道关闭 → `EFOTA`。
* `fota.chunk` / BLOB：乱序/跳号/偏移/长度/会话状态 → `EBADSEQ`；写 Flash 失败 → `EFLASH`；
  核心库缺失 → `ENOLIB`。
* `result` 取值：`-1` 核心库 `fota` 缺失、`-10` init、`-12` 空闲超时、`-21` run、
  `-30` `isDone` 超时/报错、`-31` CRC32 不符、`-32` finish、`-40` 主站中止。

### 10.6 核心库用法与前置条件

> 照官方 demo `demo/fota/fota(使用fota核心库)/fota_uart.lua`、`fota_file.lua`、
> `demo/fota/ble_5101_fota/ble_5101_packet_fota.lua`：
> `fota.init()` →（可选）轮询 `fota.wait()` 等底层就绪 → 反复 `fota.run(buf)` → `fota.isDone()`
> → `fota.finish(ok)`。
> * `fota.run(buf)` 的 `buf` 可以是**字符串**（BLE 分段 demo 传字符串；UART demo 传 zbuff），
>   返回 `(无异常, 收到最后一块, 未写 Flash 缓存字节)`；
> * `fota.isDone()` 返回 `(无异常, 写到最后一块)`（以第二个值为准）；
> * `fota.wait()` 在官方 demo 里是**"等底层就绪"**的轮询（`while not fota.wait() do sys.wait(100) end`）；
>   本模块按此实现但加 `MGMT_FOTA_LOCAL_WAIT_MS` 上限（默认 3s），超时只告警仍继续。
>
> **底层固件必须带核心库 `fota`（`LUAT_USE_FOTA`）**。判据：`caps.core`（或 `fota.begin` 直接返回
> `ENOLIB`）。缺失时一个字节都不写、设备仍跑旧版本（优雅降级 + 安全失败）。
>
> ⚠ **判「有没有库」只能看「`fota.init` 是否可调用」**：LuatOS 的 C 库由 `rotable2_newlib()`
> （`lua/src/rotable2.c`）注册成 **userdata + 元表**的只读表 —— `type(fota) == "userdata"`、
> 元表 `__name == "rotable"`，成员从元表 `__index` 解析，`fota.init()` 点号调用照常可用；
> 用 `type(fota) == "table"` 判会误报「缺库」（`caps.core=false` + `ENOLIB`）。
> `caps.core_type` 是它的类型名字符串，便于现场区分。回归用例：自测 §13.2b。

## 11. 管理 MQTT JSON-RPC（通道 = 主题后缀 `rpc`）

| 项 | 说明 |
|---|---|
| 通道 | 主题后缀 = `MGMT_MQTT_SUFFIX`（默认 `rpc`） |
| 下行 | `<MQTT_TOPIC_PREFIX>/<device_id>/rpc`，载荷 = `{"id":<事务号>,"method":"...","params":{...}}` |
| 上行 | `<MQTT_TOPIC_PREFIX>/<device_id>/rpc/resp`，载荷 = `{"id":<原样>,"ok":…,"result":…}` |
| 设备 ID | `IMEI`（取不到用模组唯一 ID），与上报同源 |
| 连接 | 与 4G 上报**共用同一条 MQTT 连接**；`mgmt_mqtt` 不自己 `create`/`connect`，只向 `bms_uplink` 登记订阅，重连后的订阅由 uplink 在每个 CONNACK 后重放 |
| 开关 | `MGMT_MQTT_ENABLE`（默认 true）**且** `MQTT_HOST` 已配置才启动 |
| 语义 | 每消息一条请求、回一条响应（`id` 原样回带）；**不发事件**；非 JSON/坏载荷只丢该条 |
| 统计 | `stat` 的 `mgmq_rx` / `mgmq_reply` / `mgmq_ignored` / `mgmq_bad` |

```bash
mosquitto_sub -h 399.run -t '/device/861234567890123/rpc/resp' -v
mosquitto_pub -h 399.run -t '/device/861234567890123/rpc' -m '{"id":1,"method":"sys.ping"}'
mosquitto_pub -h 399.run -t '/device/861234567890123/rpc' -m '{"id":2,"method":"status.get"}'
```

> broker 若开 ACL，需放行 `.../rpc`（订阅）与 `.../rpc/resp`（发布）。

## 12. PC 侧验收：`tools/mgmt_probe.ps1`

```powershell
# 连通性自检 + 读状态
powershell -ExecutionPolicy Bypass -File air780epm\tools\mgmt_probe.ps1 -Port COM11 -Action ping
powershell -ExecutionPolicy Bypass -File air780epm\tools\mgmt_probe.ps1 -Port COM11 -Action status

# 读/写配置
powershell -ExecutionPolicy Bypass -File air780epm\tools\mgmt_probe.ps1 -Port COM11 -Action config
powershell -ExecutionPolicy Bypass -File air780epm\tools\mgmt_probe.ps1 -Port COM11 `
    -Action set -Set "MQTT_HOST=mqtt.399.run,MQTT_PORT=1883"

# 重启 / 恢复出厂
powershell -ExecutionPolicy Bypass -File air780epm\tools\mgmt_probe.ps1 -Port COM11 -Action reboot
powershell -ExecutionPolicy Bypass -File air780epm\tools\mgmt_probe.ps1 -Port COM11 -Action factory

# FOTA（URL / 本地 BLOB / 中止）
powershell -ExecutionPolicy Bypass -File air780epm\tools\mgmt_probe.ps1 -Port COM11 `
    -Action url -Text "http://ota.example/fw.binpkg"
powershell -ExecutionPolicy Bypass -File air780epm\tools\mgmt_probe.ps1 -Port COM11 `
    -Action localfota -File .\jjsoc_2052.1.0_LuatOS-SoC_Air780EPM.bin
powershell -ExecutionPolicy Bypass -File air780epm\tools\mgmt_probe.ps1 -Port COM11 -Action abort

# 原始 JSON / 诊断回环 / 监听事件
powershell -ExecutionPolicy Bypass -File air780epm\tools\mgmt_probe.ps1 -Port COM11 `
    -Action raw -Text '{"method":"sys.ping"}'
powershell -ExecutionPolicy Bypass -File air780epm\tools\mgmt_probe.ps1 -Port COM11 -Action echo
powershell -ExecutionPolicy Bypass -File air780epm\tools\mgmt_probe.ps1 -Port COM11 -Action listen
```

适配器方向极性：默认每帧在 `RTS=0`/`RTS=1` 下各试一遍（`-Rts 0`/`-Rts 1` 可固定）。波特率默认 115200。

## 13. 常用示例（裸帧）

### 13.1 组帧与校验

```
func crc16(body):                    # body = VER..PAYLOAD
    crc = 0xFFFF
    for b in body: crc ^= b; 8×{ crc = (crc&1) ? (crc>>1)^0xA001 : crc>>1 }
    return crc & 0xFFFF

frame = A5 5A | VER=01 | TYPE | ID | LEN(2 LE) | PAYLOAD | crc(2 LE)
```

### 13.2 读状态（`status.get`）与读配置（`config.get`）

```
REQ: {"method":"status.get"}     → RESP: {"ok":true,"result":{...}}
REQ: {"method":"config.get"}     → RESP: {"ok":true,"result":{"values":{...},...}}
```

### 13.3 写配置（`config.set`，整批原子）

```
REQ: {"method":"config.set","params":{"LOG_ENABLE":false,"JK_MODBUS_POLL_MS":1500}}
RESP: {"ok":true,"result":{"applied":{"LOG_ENABLE":false,"JK_MODBUS_POLL_MS":1500},
                           "restart_required":false}}
REQ: {"method":"config.set","params":{"NOPE":1}} → {"ok":false,"error":{"code":"EBADKEY",...}}
```

### 13.4 命令：重启 / 恢复出厂

```
REQ: {"method":"cmd.reboot"}        → {"ok":true,"result":{"rebooting":true}}（回帧后 200ms 重启）
REQ: {"method":"cmd.factory_reset"} → {"ok":true,"result":{"factory_reset":true}}（重启后回默认）
```

### 13.5 本地 FOTA（UART BLOB）

```
REQ : {"method":"fota.begin","params":{"total":115536,"crc32":3735928559}}
RESP: {"ok":true,"result":{"started":true,"total":115536}}
# 等 stage=ready（fota.status）
BLOB: A5 5A 01 04 <ID> <LEN=242> 01 00 EE 00 <238 字节>   # SEQ=1, LEN=238
RESP: A5 5A 01 02 <ID> ... {"ok":true,"result":{"seq":1,"rx":238,"total":115536,"progress":0}}
...
REQ : {"method":"fota.status"}    # 轮询到 stage=5（success）
REQ : {"method":"cmd.reboot"}     # 进新版本
```

### 13.6 用 `mgmt_probe.ps1 -Action raw -Hex …` 直接发 payload

`-Hex` 填的是**载荷**（不含帧头/CRC），脚本自动组帧：

```
powershell -ExecutionPolicy Bypass -File air780epm\tools\mgmt_probe.ps1 -Port COM11 `
    -Action raw -Text '{"method":"sys.ping"}'
```

## 14. 配置项一览（`config.lua`）

| 键 | 默认 | 说明 |
|---|---|---|
| `MGMT_UART_ID` | `1` | 管理链路串口（RS485-1） |
| `MGMT_BAUD` | `115200` | 波特率（`[R]`） |
| `MGMT_PIN_RX` / `MGMT_PIN_TX` | `nil` | UART1 默认复用，不用配 |
| `MGMT_RS485_DIR` | `25` | 方向脚 EN=GPIO25 |
| `MGMT_RS485_RX_LEVEL` / `MGMT_RS485_DELAY_US` | `0` / `20000` | 方向切换参数 |
| `MGMT_RX_BUFF_SIZE` | `2048` | 串口接收缓冲 |
| `MGMT_MAX_FRAME_LEN` | `1200` | 单帧上限 |
| `MGMT_MAX_PAYLOAD` | `1024` | JSON 载荷上限 |
| `MGMT_PARSE_MAX_FRAMES` | `8` | 突发帧预算 |
| `MGMT_SNAP_TTL_MS` / `MGMT_SLOW_REFRESH_MS` | `1000` / `5000` | 状态快照 TTL / 慢外设后台刷新 |
| `MGMT_DEDUP_WINDOW_MS` | `10000` | ID 去重窗口 |
| `MGMT_FOTA_ENABLE` / `MGMT_FOTA_NET_WAIT_MS` | `true` / `300000` | URL 通道开关 / 等联网上限 |
| `MGMT_FOTA_LOCAL_ENABLE` / `_TIMEOUT_MS` / `_FINALIZE_MS` | `true` / `60000` / `3000` | 本地通道 |
| `MGMT_MQTT_ENABLE` / `MGMT_MQTT_SUFFIX` / `MGMT_MQTT_MAX_FRAMES` | `true` / `"rpc"` / `8` | MQTT 通道 |
| `MGMT_ECHO_TEST` / `MGMT_ECHO_HEARTBEAT_MS` | `false` / `5000` | 【临时诊断】回环+心跳 |

## 15. 排障

（同 `air780epm/README.md` §7.9；服务日志前缀 `mg`。）要点：

* `mg_crc`/`mg_sync` 应为 0；一直涨 = 波特率/帧头不对（`-Baud 115200`、帧头 `A5 5A`）或线路被破坏；
* 开机日志应有 `mg uart 1 opened 115200 8N1 (mgmt rpc, raw frame), buf 2048`；
* RS485-1 完全收发不了：先看 GPIO25 方向脚与 GPIO 高电平是否 3.3V（README §2.1）；
* 误开 `MGMT_ECHO_TEST=true` 时链路只回环、不跑 RPC —— 排查完必须改回 `false`。
