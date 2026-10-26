# 极空 PB/BD 系列 RS485 私有实时帧解析

本文记录 Air780EPM 固件中 `jk_pb485.lua` 使用的极空 PB/BD 系列 RS485 私有协议。
它不同于 [JK-BMS-RS485.md](./JK-BMS-RS485.md) 描述的普通 Modbus 03H 读寄存器流程。

适用现象：

- 标准 Modbus 的 `0x1200` 单体块和 `0x1240` `CellSta` 可以正常读取；
- `0x128A` 起的汇总寄存器块无论是 `x58`、`x32` 还是单寄存器读取，设备都可能静默无响应；
- 极空 Console 能收到 `55 AA EB 90` 开头的 300 字节实时帧。

## 1. 物理层与总线角色

| 项目 | 值 |
|---|---|
| 接口 | RS485 |
| 波特率 | 115200 |
| 数据位 | 8 |
| 校验 | 无 |
| 停止位 | 1 |
| 流控 | 无 |
| 总线方式 | 半双工 |
| Air780EPM 角色 | 主站，负责选择 BMS 并接收实时帧 |
| 极空 BMS 角色 | 从站，返回 Modbus 写响应，并主动发送私有实时帧 |

Air780EPM 使用 RS485-3 / UART3：

```text
PIN53 = UART3_RXD
PIN52 = UART3_TXD
```

UART3 不是默认复用功能，串口打开前必须通过 `pins_Air780EPM.json` 或
`uart_pins.lua` 将 PIN52/53 复用为 UART3。

## 2. 交互流程

```text
Air780EPM/Console                    JK BMS
       |                                |
       | 01 10 16 20 00 01 02 00 00 D6 F1
       |------------------------------->|
       |                                | 选择 BMS，开始/继续发送实时帧
       |                                |
       |      55 AA EB 90 02 ... SUM8   |
       |<-------------------------------|
       |                                |
       | 01 10 16 20 00 01 02 00 00 D6 F1
       |------------------------------->|
       |      55 AA EB 90 02 ... SUM8   |
       |<-------------------------------|
```

### 2.1 选择 BMS

使用标准 Modbus 功能码 `0x10`，向 `0x1620` 写入 1 个寄存器：

```text
01 10 16 20 00 01 02 00 00 D6 F1
```

| 字段 | 值 | 说明 |
|---|---|---|
| 从机地址 | `01` | BMS Modbus 地址，默认配置为 1 |
| 功能码 | `10` | 写多个寄存器 |
| 起始地址 | `16 20` | 选择 BMS / 启动实时数据 |
| 寄存器数量 | `00 01` | 1 个寄存器 |
| 字节数 | `02` | 写 2 字节 |
| 数据 | `00 00` | BMS 索引 0 |
| CRC | `D6 F1` | Modbus CRC16，低字节在前 |

设备可能返回：

```text
01 10 16 20 00 01 04 4B
```

该写响应只说明选择命令被接收，实时数据通过另一个 `55 AA EB 90` 帧返回。
Air780EPM 以约 800ms 周期重发选择命令并等待 `0x02` 实时帧。

> `0x1620` 是私有实时流的选择/触发入口，不应按普通可读 Modbus 寄存器理解。

## 3. 300 字节实时帧格式

所有极空私有实时帧均为固定 300 字节：

```text
偏移  长度  内容
0x00   4   55 AA EB 90
0x04   1   帧类型 TYPE
0x05   1   00
0x06  293  数据区 DATA
0x012B 1   SUM8
```

帧总长：

```text
4 + 1 + 1 + 293 + 1 = 300
```

### 3.1 帧类型

| TYPE | 内容 | 本固件处理 |
|---|---|---|
| `0x01` | 设置/配置信息 | 忽略 |
| `0x02` | 实时电芯与 BMS 状态 | 解析并发布 `JK_ROUND` |
| `0x03` | 设备信息 | 忽略 |

极空 Console 抓包中，设备型号为 `JK-BD6A20S6P`，软件版本示例为 `20.25`。

### 3.2 SUM8 校验

```text
SUM8 = sum(frame[0 .. 298]) & 0xFF
SUM8 == frame[299]
```

即前 299 字节逐字节无符号相加，保留最低 8 位。

校验失败时丢弃从当前候选帧头开始的下一个字节，然后重新搜索 `55 AA EB 90`。

## 4. `0x02` 实时数据布局

下文“数据偏移”均相对于帧内 `DATA` 起点，即整帧偏移 `0x06`。
一个文档地址 `0x1200 + X` 对应实时帧数据偏移 `X`。

例如：

```text
总压地址 = 0x1200 + 0x0090
数据偏移 = 0x0090
整帧偏移 = 0x06 + 0x0090 = 0x0096
```

### 4.1 单体电压

| 数据偏移 | 类型 | 内容 |
|---|---|---|
| `0x0000 + 2*n` | UINT16 LE | 第 `n+1` 路单体电压，mV |

24 串布局使用 24 个槽位；32 串布局额外增加 16 个槽位，并在后续字段整体后移
32 字节。解析器会分别尝试：

```text
24 槽：字段偏移不变
32 槽：字段偏移整体 + 0x20
```

选择依据：

1. 串数必须大于 0；
2. 总压必须在合理范围；
3. 单体电压求和与总压的差值不得超过 `max(2000mV, 串数 × 150mV)`。

### 4.2 核心实时字段

| 数据偏移 | 类型 | 字段 | 单位/换算 |
|---|---|---|---|
| `0x008A` | INT16 LE | MOS 温度 | 0.1℃，显示时四舍五入为整数℃ |
| `0x008C` | UINT32 LE | 均衡线电阻状态 | 位图 |
| `0x0090` | UINT32 LE | 总电压 | mV |
| `0x0094` | INT32 LE | 电池功率 | mW |
| `0x0098` | INT32 LE | 电池电流 | mA，充电为正 |
| `0x009C` | INT16 LE | 电池温度 1 | 0.1℃ |
| `0x009E` | INT16 LE | 电池温度 2 | 0.1℃ |
| `0x00A0` | UINT32 LE | 报警位图 | 32 位 |
| `0x00A4` | INT16 LE | 均衡电流 | mA |
| `0x00A6` | UINT16 LE | 均衡状态 + SOC | 高字节 SOC，低字节均衡状态 |
| `0x00A8` | INT32 LE | 剩余容量 | mAh |
| `0x00AC` | UINT32 LE | 实际/满充容量 | mAh |
| `0x00B0` | UINT32 LE | 循环次数 | 次 |
| `0x00B4` | UINT32 LE | 循环总容量 | mAh |
| `0x00B8` | UINT16 LE | 预充状态 + SOH | 高字节预充，低字节 SOH% |
| `0x00BA` | UINT16 LE | 用户报警 1 | 位图 |
| `0x00BC` | UINT32 LE | BMS 运行时间 | 秒 |
| `0x00C0` | UINT16 LE | 充电 + 放电状态 | 高字节放电，低字节充电 |
| `0x00C2` | UINT16 LE | 用户报警 2 | 位图 |

UINT32/INT32 均按低字在前、高字在后：

```text
u32 = byte[o] | byte[o+1]<<8 | byte[o+2]<<16 | byte[o+3]<<24
```

### 4.3 打包字段

SOC 与均衡状态：

```text
raw       = UINT16LE(0x00A6)
SOC       = (raw >> 8) & 0xFF
balancing = raw & 0xFF
```

SOH 与预充状态：

```text
raw       = UINT16LE(0x00B8)
SOH       = raw & 0xFF
precharge = (raw >> 8) & 0xFF
```

充电与放电 MOS 状态：

```text
raw       = UINT16LE(0x00C0)
charge    = raw & 0xFF
discharge = (raw >> 8) & 0xFF
```

## 5. 报警位图

`0x00A0` 与 Modbus 汇总区的 `0x12A0` 报警位定义一致：

| 位 | 含义 |
|---|---|
| bit0 | 均衡线电阻过大 |
| bit1 | MOS 过温保护 |
| bit2 | 单体数量与设置不符 |
| bit3 | 电流传感器异常 |
| bit4 | 单体过压保护 |
| bit5 | 电池过压保护 |
| bit6 | 充电过流保护 |
| bit7 | 充电短路保护 |
| bit8 | 充电过温保护 |
| bit9 | 充电低温保护 |
| bit10 | 内部通信异常 |
| bit11 | 单体欠压保护 |
| bit12 | 电池欠压保护 |
| bit13 | 放电过流保护 |
| bit14 | 放电短路保护 |
| bit15 | 放电过温保护 |

更高位用于充电/放电 MOS 异常、GPS、授权密码、放电开启失败、电池超温等状态。

## 6. 接收与重同步

解析器的处理顺序：

1. UART 回调持续读取串口，并把数据追加到接收缓冲；
2. 在缓冲区中搜索 `55 AA EB 90`；
3. 丢弃帧头前的噪声并计入 `resync`；
4. 不足 300 字节时保留，等待下一批数据；
5. 足够 300 字节时取固定长度候选帧；
6. 校验 SUM8；
7. 校验失败计 `bad_checksum`，丢弃当前帧头 1 字节后重新搜索；
8. `TYPE=0x02` 时解析字段并发布 `JK_ROUND`；
9. `TYPE=0x01/0x03` 计入 `ignored_frames`，不参与样本生成。

关键统计：

| 字段 | 含义 |
|---|---|
| `frames` | 合法的 `0x02` 实时帧数 |
| `rounds` | 成功生成的样本数 |
| `requests` | 发送 `0x1620` 选择命令的次数 |
| `timeouts` | 等待 500ms 仍未收到完整 `0x02` 帧的次数 |
| `rx_bytes` / `tx_bytes` | 原始收发字节数 |
| `bad_checksum` | SUM8 不匹配次数 |
| `resync` | 因噪声或错帧丢弃的字节数 |
| `ignored_frames` | 忽略的 `0x01/0x03` 帧数 |

## 7. 真实抓包验证

验证素材：

```text
air780epm/tools/fixtures/jk_pb485_300.hex
```

该文件来自极空 BMS Console 实际交互，包含完整 300 字节 `0x02` 帧。

校验结果：

```text
帧长        = 300
帧头        = 55 AA EB 90 02 00
SUM8        = 0x49
```

解析结果：

| 字段 | 解析值 |
|---|---|
| 单体数 | 20 |
| 单体 1 | 3304mV |
| 总压 | 66081mV |
| 电流 | -62mA |
| SOC | 95% |
| MOS 温度 | 26.4℃ |
| 电池温度 1 | 23.6℃ |
| 电池温度 2 | 24.7℃ |
| 剩余容量 | 51090mAh |
| 满充容量 | 54000mAh |
| 循环次数 | 14 |
| 循环容量 | 799811mAh |
| SOH | 100% |
| 充电 MOS | 开 |
| 放电 MOS | 开 |

一致性检查：

```text
20 × 3304mV ≈ 66080mV，与总压 66081mV 一致
51090mAh / 54000mAh ≈ 94.6%，与 SOC 95% 一致
```

## 8. Air780EPM 下游映射

`jk_pb485.lua` 产出与 `jk_display.lua`、`jk_modbus.lua` 同结构的快照，字段包括：

```text
cellCount, cells, batVolMv, batCurrentMa, soc,
tempBatC, tempBat2C, tempMosC,
sysAlarm, sysAlarmValid, balanceSw, chargeMos, dischargeMos,
capRemainMah, fullCapMah, cycleCount, cycleCapMah,
soh, bmsUpTimeS, source = "pb485"
```

`bms_data.normalize` 将 `pb485` 视为完整字段源，不执行显示屏模式的归零或容量估算。

### 8.1 金箭 Modbus 从机

| 金箭寄存器 | 来源 |
|---|---|
| reg0 总电压 | `batVolMv / 10` |
| reg1 串数 | `cellCount` |
| reg2 SOC | `soc` |
| reg3 容量 Ah | `fullCapMah / 1000` |
| reg4 短路保护 | `sysAlarm` |
| reg5 电流 | `batCurrentMa / 10` |
| reg6 温度 2 | `tempBat2C` |
| reg7 温度 1 | `tempBat1C` |
| reg8 板温 | `tempMosC` |
| reg9–32 | 前 24 路单体 mV |
| reg104 | `cycleCount` |
| reg110 | `balanceSw` |
| reg113 | `fullCapMah / 1000` |

金箭协议仅暴露 24 路单体；25–32 串电池仍只向车端提供前 24 路。

### 8.2 MQTT MessageV1

MessageV1 可获得：

- 单体电压；
- 总压；
- 电流；
- 电池温度 1/2；
- MOS 温度；
- 均衡电流；
- 剩余容量；
- 循环总容量；
- 消息级满充容量；
- 消息级 BMS 运行时间。

现有 MessageV1 格式没有独立的 SOC、循环次数和 SOH 列。SOC 由服务端依据电压、
容量和单体数据计算；循环次数主要供金箭车端使用；SOH 在内部快照中保留。

## 9. 固件配置

```lua
M.JK_PROTOCOL        = "pb485"
M.JK_MODBUS_BAUD     = 115200
M.JK_MODBUS_SLAVE_ADDR = 1
M.JK_PB485_POLL_MS   = 800
M.JK_PB485_TIMEOUT_MS = 500
M.JK_PB485_RX_BUFF_SIZE = 1024
```

如果 fskv 中曾保存过 `JK_PROTOCOL=modbus`，持久化覆盖值会优先于 `config.lua`。
切换后需通过管理链路写入 `JK_PROTOCOL=pb485` 并重启，或恢复出厂配置后重启。

## 10. 与其他极空协议的区别

| 协议 | 物理参数 | 触发方式 | 数据格式 |
|---|---|---|---|
| 显示屏广播 | 2400 8N1 | 保护板持续主动广播 | `A5 5A` 帧 |
| 标准 Modbus | 115200 8N1 | 主站发送 03H 读寄存器 | 标准 Modbus RTU |
| PB485 私有流 | 115200 8N1 | 写 `0x1620` 后接收 | 固定 300B `55 AA EB 90` |

同一根 RS485 总线、同一时刻只能启用一种数据源实现，不能同时运行
`jk_display.lua`、`jk_modbus.lua` 和 `jk_pb485.lua`。

## 11. 相关文件

- `air780epm/jk_pb485.lua`：协议触发、帧定界、校验、字段解析；
- `air780epm/bms_data.lua`：快照归一；
- `air780epm/jinjian_slave.lua`：金箭寄存器映射；
- `air780epm/bms_uplink.lua`：MQTT 上报窗口；
- `air780epm/bms_codec.lua`：MessageV1 编码；
- `air780epm/tools/fixtures/jk_pb485_300.hex`：真实帧回归素材；
- `air780epm/tools/selftest.lua`：解析、金箭和 MQTT 端到端断言。
