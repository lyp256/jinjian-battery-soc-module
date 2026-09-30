# 金箭协议文档

本目录收录本模块实现所需的金箭侧与极空侧协议。

| 文档 | 内容 |
|---|---|
| [01_BLE_PROTOCOL.md](01_BLE_PROTOCOL.md) | 金箭智行蓝牙串口协议：GATT、密钥派生（key2/key3）、AES-128-ECB、帧格式/CRC、ECU/MC/VEHAUTO/ABS/TFT 控制指令、响应解析、OTA/音频 |
| [02_BMS_PROTOCOL.md](02_BMS_PROTOCOL.md) | 金箭 BMS 通讯协议：485 物理层、Modbus 指令、寄存器地图、应答解析、单位、命令+CRC 示例 |
| [03_BMS_COLLECTION_REPORT.md](03_BMS_COLLECTION_REPORT.md) | BMS 上报消息清单与实现要点：上报消息、JSON 示例、单位处理、通道差异 |
| [JK-BMS-RS485.md](JK-BMS-RS485.md) | 极空 JK-BMS RS485 Modbus 通用协议（V1.1）：物理层、功能码、寄存器映射表 |

## 使用建议

- **实现 BMS 从机**（应答金箭中控查询）：以 `02_BMS_PROTOCOL.md` 为主，`03_BMS_COLLECTION_REPORT.md` 补充上报消息与实现要点。
- **实现蓝牙调试程序**：以 `01_BLE_PROTOCOL.md` 为主。
- **解析极空保护板数据**：Modbus 见 `JK-BMS-RS485.md`，显示屏广播见 [`../jikong/极空显示屏协议.md`](../jikong/极空显示屏协议.md)。
