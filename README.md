# 金箭电动车电量计量模块

金箭 2.0 车型中控支持使用 RS485 协议查询 BMS 状态信息并按照百分比显示电量。但是目前金箭中控支持自家原厂锂电的 BMS 协议，不能对接极空、蚂蚁等第三方保护板。根据分析，中控会持续通过 Modbus 协议查询 BMS 模块的电池状态，因此只要有个中间模块能读取第三方保护板的状态数据并翻译为金箭的 Modbus 协议就可以将第三方 BMS 的电池状态接入金箭电动车。

本项目主要是个人使用，我使用的是极空保护板和金箭快闪 PLUS 750 电动车。电动车电池仓预留有 RS485 通讯接口，实测只需要将支持金箭 Modbus 协议的模块接入，电动车中控和 APP 上可以看到百分比电量。我找到一款成熟的 DTU 模块银尔达 D700Tm 作为电量计量模块的硬件部分，D700Tm 可以支持 8-90V 直流电源接入，自带 3 路 RS485 还有 4G 功能简直完美契合我的需求。如果你的电池组没有通讯接口只能通过蓝牙接入的话，可以考虑度云的DTU模块带蓝牙 + RS485 可以对接保护板蓝牙协议但是电源输入只支持 8-36V 需要一个额外的 DCDC 降压模块。

## 参考实现

| 实现 | 硬件 / 框架 | 说明 |
|---|---|---|
| [`air780epm/`](air780epm/README.md) | 银尔达 D700Tm（Air780EPM）LuatOS | 极空采集（显示屏广播 / RS485 Modbus 可在配置里切换）|
| [`esp32/`](esp32/README.md) | ESP32-S3 + 两路 THVD1406DR RS485 + ML307-NL 4G | 半成品仅供参考，优先参考 `air780epm` 实现。esp32 系列有蓝牙模块支持多路串口，可以实现本地蓝牙、wifi 配置等高级功能但是奈何目前市面上没有成熟一体化模块，需要画PCB 定制。|


## 相关协议文档

| 文档 | 内容 |
|---|---|
| [`docs/jinjian/02_BMS_PROTOCOL.md`](docs/jinjian/02_BMS_PROTOCOL.md) | 金箭 BMS 通讯协议（本模块作为 Modbus 从机应答） |
| [`docs/jinjian/01_BLE_PROTOCOL.md`](docs/jinjian/01_BLE_PROTOCOL.md) | 金箭智行 BLE 控制协议 |
| [`docs/jinjian/03_BMS_COLLECTION_REPORT.md`](docs/jinjian/03_BMS_COLLECTION_REPORT.md) | BMS 上报消息清单与实现要点 |
| [`docs/jikong/JK-BMS-RS485.md`](docs/jikong/JK-BMS-RS485.md) | 极空 JK-BMS RS485 Modbus 通用协议（V1.1） |
| [`docs/jikong/极空显示屏协议.md`](docs/jikong/极空显示屏协议.md) | 极空显示屏广播协议 |
车。
