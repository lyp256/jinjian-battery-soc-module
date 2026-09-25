#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* 极简 MQTT 3.1.1 客户端（移植自 mqttagent 的 MQTT 上报链路）：
 *   - 组包：CONNECT / SUBSCRIBE / PUBLISH(QoS0/1) / PUBACK / PINGREQ / DISCONNECT
 *   - 解包：CONNACK / SUBACK / PUBLISH / PUBACK / PINGRESP
 * 只做上报与下行分发，不含 QoS2、不含重传队列。
 * 字节流由 ml307_4g 的 TCP 透传通道提供（AT+MIPOPEN/MIPSEND）。 */

#define MQTT_RX_BUF_SIZE 2048
#define MQTT_TX_BUF_SIZE 4352  /* 需容纳 BMS_HIST_MAX_SIZE(4096) + topic + 报文头 */

enum {
    MQTT_TYPE_CONNECT = 1,
    MQTT_TYPE_CONNACK = 2,
    MQTT_TYPE_PUBLISH = 3,
    MQTT_TYPE_PUBACK = 4,
    MQTT_TYPE_SUBSCRIBE = 8,
    MQTT_TYPE_SUBACK = 9,
    MQTT_TYPE_PINGREQ = 12,
    MQTT_TYPE_PINGRESP = 13,
    MQTT_TYPE_DISCONNECT = 14,
};

typedef struct {
    uint8_t buf[MQTT_RX_BUF_SIZE];
    size_t len;
} mqtt_rx_t;

typedef struct {
    uint8_t type;
    uint8_t flags;
    uint16_t packet_id;
    char topic[160];
    size_t topic_len;
    const uint8_t *payload; /* 指向 mqtt_rx_t 缓冲区内部 */
    size_t payload_len;
} mqtt_msg_t;

void mqtt_rx_init(mqtt_rx_t *rx);

/* 追加收到的字节；缓冲不足（长时间无完整报文）时丢弃最旧数据并返回 false。 */
bool mqtt_rx_push(mqtt_rx_t *rx, const void *data, size_t len);

/* 取一个完整报文；返回 false 表示数据还不够。 */
bool mqtt_rx_next(mqtt_rx_t *rx, mqtt_msg_t *msg);

size_t mqtt_build_connect(uint8_t *out, size_t cap, const char *client_id,
                          const char *user, const char *password,
                          uint16_t keepalive_s, bool clean_session);
size_t mqtt_build_subscribe(uint8_t *out, size_t cap, uint16_t packet_id,
                            const char *topic, uint8_t qos);
size_t mqtt_build_publish(uint8_t *out, size_t cap, const char *topic,
                          const void *payload, size_t payload_len,
                          uint8_t qos, bool retain, uint16_t packet_id);
size_t mqtt_build_puback(uint8_t *out, size_t cap, uint16_t packet_id);
size_t mqtt_build_pingreq(uint8_t *out, size_t cap);
size_t mqtt_build_disconnect(uint8_t *out, size_t cap);
