#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ML307_HOST_LEN     64
#define ML307_APN_LEN      33

/* ML307-NL 4G 模块参数（AT 指令控制）。
 * 模块只负责“上电入网 + TCP 连接 + 透传收发”，上层协议（MQTT 上报）由 mqtt_client
 * 与 bms_uplink 实现；host/port 指向 MQTT broker。 */
typedef struct {
    bool enable;                 /* 关闭时不上电、不建 socket */
    int uart_num;
    int tx_gpio;                 /* ESP TX → 模块 RX */
    int rx_gpio;                 /* ESP RX ← 模块 TX */
    int en_gpio;                 /* 模块 EN / PWR_ON，-1 = 硬件常开 */
    bool en_active_low;
    uint32_t en_pulse_ms;        /* >0 脉冲开机；0 = 电平使能 */
    uint32_t baud_rate;
    char apn[ML307_APN_LEN];     /* 留空 = 模块默认 APN */
    char host[ML307_HOST_LEN];   /* TCP 对端（MQTT broker），留空 = 只入网不建 socket */
    uint16_t port;
} ml307_4g_config_t;

typedef enum {
    ML307_STATE_DISABLED = 0,
    ML307_STATE_POWER_ON,
    ML307_STATE_WAIT_AT,
    ML307_STATE_INIT,
    ML307_STATE_SIM,
    ML307_STATE_NETWORK,
    ML307_STATE_SOCKET,
    ML307_STATE_ONLINE,
    ML307_STATE_RETRY,
} ml307_state_t;

typedef struct {
    bool enabled;
    ml307_state_t state;
    const char *state_name;
    bool at_ok;        /* AT 口已应答 */
    bool sim_ok;       /* SIM 就绪 */
    bool net_ok;       /* 已注册到 LTE 网络 */
    bool socket_ok;    /* TCP 已连接 */
    int csq;           /* 0..31，-1 = 未知 */
    int rssi_dbm;      /* 由 CSQ 换算，-1 = 未知 */
    int reg_stat;      /* +CEREG 状态，-1 = 未知（1/5 表示已注册） */
    uint32_t reports_ok;
    uint32_t reports_fail;
    uint32_t downlink_count;
    uint32_t rx_bytes;
    uint32_t tx_bytes;
    uint32_t at_cmd_count;
    char ip[24];
    char imei[24];
    char iccid[24];
    char cclk[24];
    bool time_valid;        /* CCLK 已同步到有效网络时间 */
    uint32_t unix_time;     /* 最近一次 AT+CCLK? 的 Unix 秒 */
    char last_error[48];
    char last_downlink[160];
    uint32_t last_ok_ms;    /* 最近一次 AT/网络就绪时间（esp_timer 毫秒），0 = 从未 */
} ml307_4g_status_t;

/* 初始化 UART/EN 并启动 AT 状态机任务；cfg 必须常驻有效（内部保存指针内容）。 */
void ml307_4g_start(const ml307_4g_config_t *cfg);

/* 读取模块状态快照（线程安全）。 */
void ml307_4g_get_status(ml307_4g_status_t *out);

/* 异步请求：立即上报一次 / 重连 socket / 整机重启（EN 断电重新开机）。 */
void ml307_4g_request_reconnect(void);
void ml307_4g_request_power_cycle(void);

/* TCP 通道：socket 就绪后上层可收发数据（MQTT 报文）。
 * 接收数据通过回调送给上层；回调在 AT 任务上下文里执行，需快速返回。 */
bool ml307_4g_socket_ready(void);
int ml307_4g_socket_send(const void *data, size_t len);
void ml307_4g_set_rx_callback(void (*cb)(const void *data, size_t len));

/* 网络时间（AT+CCLK?，运营商网络提供）。返回 true 表示时间有效。 */
bool ml307_4g_get_epoch(uint32_t *epoch);

/* 手动执行一条 AT 指令（Web 调试用）。
 * 返回 0 成功（resp 为模块回显的响应行），-1 失败/-2 忙。 */
int ml307_4g_at_command(const char *cmd, char *resp, size_t resp_cap,
                        uint32_t timeout_ms);
