#pragma once

#include <stdbool.h>
#include <stdint.h>

/* 采集上报任务（移植自 mqttagent/air780e）：
 *   1 秒采样一条 → 攒满 batch_samples（默认 30）条一批 →
 *   BMSStateHistory 压缩编码 → MQTT QoS0 发布到 <prefix>/<device>/status；
 *   同时订阅 <prefix>/<device>/call（下行 JSON-RPC，当前只记录日志）。
 * 时间戳取自 ML307 网络时间（AT+CCLK?），无对时前用开机秒数占位。 */

#define BMS_UPLINK_TOPIC_LEN 80
#define BMS_UPLINK_ID_LEN    24

typedef struct {
    bool enable;
    char host[64];         /* MQTT broker（与 4G TCP 目标一致） */
    uint16_t port;
    char user[33];
    char password[33];
    char topic_prefix[24]; /* 默认 /bms */
    uint32_t sample_interval_ms;
    uint8_t batch_samples;
    uint16_t keepalive_s;
} bms_uplink_config_t;

typedef struct {
    bool enabled;
    bool mqtt_connected;
    bool subscribed;
    uint32_t samples;
    uint32_t batches_ok;
    uint32_t batches_fail;
    uint32_t last_payload_size;
    uint32_t last_publish_ms; /* esp_timer 毫秒，0 = 从未 */
    uint32_t downlink_count;
    char device_id[BMS_UPLINK_ID_LEN];
    char status_topic[BMS_UPLINK_TOPIC_LEN];
    char last_error[48];
} bms_uplink_status_t;

void bms_uplink_start(const bms_uplink_config_t *cfg);
void bms_uplink_get_status(bms_uplink_status_t *out);

/* Web/调试用：立即发布当前批次（不足一批也发）、请求重建 MQTT 连接。 */
void bms_uplink_request_publish(void);
void bms_uplink_request_reconnect(void);
