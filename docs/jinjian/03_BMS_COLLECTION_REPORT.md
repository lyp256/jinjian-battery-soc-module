# BMS 上报消息与实现要点

> 范围：本模块需要产出/上报的 BMS 消息清单、JSON 示例与实现要点。
> 底层寄存器/指令/CRC 见 `02_BMS_PROTOCOL.md`；蓝牙链路/加解密见 `01_BLE_PROTOCOL.md`。

---

## 1. 上报消息清单

### 1.1 周期主动上报

| ID | 消息 | 数据源 | 关键字段 |
|---|---|---|---|
| BMS-01 | 基础电池信息 | reg0–8 | totalVoltage、cellCount、soc、capacity、chargeCurrent、temperature、temperatureBoard、charging、fullyCharged、dataValid |

### 1.2 按需查询/响应

| ID | 消息 | 数据源 | 关键字段 |
|---|---|---|---|
| BMS-02 | 电池详情 | reg0–32 | + cellVoltages[0..23]、maxCellVoltageDifference |
| BMS-03 | 电池状态/健康 | reg103–113 | batteryType、cycleIndex、balanceStatus、nominalCapacity |
| BMS-04 | 版本/PN | 1016–1017、1000–1007 | version、pn |
| BMS-05 | 快充状态 | 开关量 63 | fastCharging |
| BMS-06 | 充电设置 | 1089/1090 | chargeTimeLimitMin、chargeTargetSoc |
| BMS-07 | 充电设置写入 | 1089/1090 | 写 0–120 / 0–100 |
| BMS-08 | 结束快充 | 开关量 64 | 写 `FF00` |
| BMS-09 | 保护状态 | 开关量 4–8 | shortCircuit、overTempCharge/Discharge、lowTempCharge/Discharge |

### 1.3 事件/告警

| ID | 消息 | 触发 |
|---|---|---|
| BMS-10 | 充放电状态 | reg5 符号变化 |
| BMS-11 | 充满 | SOC≥100（3 次去抖） |
| BMS-12 | 低电量提醒 | SOC≤阈值（默认约 20%） |
| BMS-13 | 单体电压越限 | reg9–32 任一超限 |
| BMS-14 | 温度越限 | reg6–8 超限 |
| BMS-15 | 均衡状态 | reg110 变化 |
| BMS-16 | 故障/通讯异常 | 无响应/CRC 错/超时 |

---

## 2. 上报 JSON 示例

### BMS-01

```json
{
  "msgId": "BMS-01",
  "ts": "2026-09-19T00:18:18+08:00",
  "dataValid": true,
  "totalVoltageRaw": 4800,
  "totalVoltageV": 48,
  "cellCount": 4,
  "soc": 72,
  "capacity": 200,
  "chargeCurrentRaw": 500,
  "chargeCurrentA": 5.0,
  "charging": true,
  "discharging": false,
  "temperature": [25, 26],
  "maxTemperature": 26,
  "temperatureBoard": 24
}
```

### BMS-03

```json
{
  "msgId": "BMS-03",
  "ts": "2026-09-19T00:18:18+08:00",
  "batteryType": 0,
  "cycleIndex": 12,
  "balanceStatus": 0,
  "nominalCapacity": 20
}
```

### 事件

```json
{ "msgId": "BMS-10", "state": "charging", "chargeCurrentA": 5.0 }
{ "msgId": "BMS-11", "fullyCharged": true, "soc": 100 }
{ "msgId": "BMS-12", "lowBattery": true, "soc": 18, "threshold": 20 }
{ "msgId": "BMS-16", "errorCode": 1, "lastError": "BMS no response", "dataValid": false }
```

---

## 3. 模块实现要点

1. 串行化请求：同一时刻只发一个 BMS 请求，用 `address+num` 关联响应；
2. 周期调度：BMS-01 常驻（5–30s）；BMS-02/03/04/06 按需；事件 3 次去抖；
3. 单位处理：`totalVoltage=round(reg0/100)`（V）、`chargeCurrent=reg5/100`（A）、`soc` clamp 0–100；
4. 单体电压数量由 reg1 决定，不固定 24 路；
5. 缓存与降级：保留最近合法数据；超时 `dataValid=false`，不上报脏数据；
6. 通道差异：BLE 需 AES(key3)+CRC；485/4G 按各自链路格式；CRC 统一 LE；
7. 日志：记录明文帧、响应 hex、解析字段，便于排查。

---

## 4. 相关文档

- `01_BLE_PROTOCOL.md`
- `02_BMS_PROTOCOL.md`
