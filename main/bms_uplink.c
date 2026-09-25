/* 采集上报任务：移植自 mqttagent/air780e/bms.lua + ntp_sync.lua
 *   采样 → 批编码（bms_hist）→ MQTT 发布（mqtt_client）→ 4G TCP 透传（ml307_4g）。
 * 下行 <prefix>/<device>/call 会订阅并把收到的 JSON-RPC 原文记入日志
 * （RPC 方法分发尚未移植，见 README「MQTT 上报」章节）。 */

#include "bms_uplink.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "bms_hist.h"
#include "bms_interface.h"
#include "led_ctrl.h"
#include "ml307_4g.h"
#include "mqtt_client.h"
#include "rpc_server.h"

static const char *TAG = "uplink";

#define UPLINK_TASK_STACK   5120
#define UPLINK_TASK_PRIO    4
#define UPLINK_RX_RING      4096
#define UPLINK_CONNECT_TO_MS 10000
#define UPLINK_PUBLISH_FAIL_MAX 3
#define UPLINK_TIME_SYNC_S  3600

static bms_uplink_config_t s_cfg;
static bms_uplink_status_t s_status;
static SemaphoreHandle_t s_mutex;
static bms_hist_t s_hist;
static mqtt_rx_t s_rx;
static bool s_started;

static uint8_t s_rx_ring[UPLINK_RX_RING];
static size_t s_rx_head;
static size_t s_rx_tail;
static SemaphoreHandle_t s_ring_mutex;

static uint8_t s_tx_pkt[MQTT_TX_BUF_SIZE];   /* 单线程（上报任务）使用 */
static uint8_t s_payload[BMS_HIST_MAX_SIZE];
static uint16_t s_packet_id;

static volatile bool s_req_publish;
static volatile bool s_req_reconnect;

static uint32_t s_time_offset;   /* epoch - 开机秒 */
static bool s_time_valid;

/* ---------------- 工具 ---------------- */

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void set_error(const char *msg)
{
    if (s_mutex != NULL) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
    }
    snprintf(s_status.last_error, sizeof(s_status.last_error), "%s", msg);
    if (s_mutex != NULL) {
        xSemaphoreGive(s_mutex);
    }
    ESP_LOGW(TAG, "%s", msg);
}

/* 采样时间戳：优先使用 4G 网络时间（每小时重新对齐一次） */
static uint32_t now_epoch(void)
{
    uint32_t mono = (uint32_t)(esp_timer_get_time() / 1000000);
    if (!s_time_valid) {
        uint32_t epoch = 0;
        if (ml307_4g_get_epoch(&epoch) && epoch > 1600000000u) {
            s_time_offset = epoch - mono;
            s_time_valid = true;
            ESP_LOGI(TAG, "对时成功：Unix %u", (unsigned)epoch);
        } else {
            return mono; /* 未对时：先用开机秒数，读数仅作占位 */
        }
    }
    return s_time_offset + mono;
}

/* ---------------- 接收环形缓冲（AT 任务 → 上报任务） ---------------- */

static void uplink_rx_push(const void *data, size_t len)
{
    if (s_ring_mutex == NULL) {
        return;
    }
    const uint8_t *p = (const uint8_t *)data;
    xSemaphoreTake(s_ring_mutex, portMAX_DELAY);
    for (size_t i = 0; i < len; i++) {
        size_t next = (s_rx_head + 1) % UPLINK_RX_RING;
        if (next == s_rx_tail) {
            s_rx_tail = (s_rx_tail + 1) % UPLINK_RX_RING; /* 丢弃最旧字节 */
        }
        s_rx_ring[s_rx_head] = p[i];
        s_rx_head = next;
    }
    xSemaphoreGive(s_ring_mutex);
}

/* 取走环形缓冲里的数据；返回拷贝字节数 */
static size_t ring_drain(uint8_t *out, size_t cap)
{
    size_t n = 0;
    if (s_ring_mutex == NULL) {
        return 0;
    }
    xSemaphoreTake(s_ring_mutex, portMAX_DELAY);
    while (s_rx_tail != s_rx_head && n < cap) {
        out[n++] = s_rx_ring[s_rx_tail];
        s_rx_tail = (s_rx_tail + 1) % UPLINK_RX_RING;
    }
    xSemaphoreGive(s_ring_mutex);
    return n;
}

/* ---------------- MQTT 收发 ---------------- */

static bool mqtt_send(const uint8_t *pkt, size_t len, const char *what)
{
    if (len == 0) {
        set_error(what);
        return false;
    }
    if (ml307_4g_socket_send(pkt, len) != 0) {
        set_error("4G socket 发送失败");
        return false;
    }
    return true;
}

static bool mqtt_publish_raw(const char *topic, const void *payload, size_t len,
                             uint8_t qos, bool retain)
{
    uint16_t pid = qos > 0 ? ++s_packet_id : 0;
    size_t n = mqtt_build_publish(s_tx_pkt, sizeof(s_tx_pkt), topic, payload,
                                  len, qos, retain, pid);
    return mqtt_send(s_tx_pkt, n, "发布报文过长");
}

static bool mqtt_publish_ping(void)
{
    size_t n = mqtt_build_pingreq(s_tx_pkt, sizeof(s_tx_pkt));
    if (!mqtt_send(s_tx_pkt, n, "PINGREQ 组包失败")) {
        return false;
    }
    ESP_LOGD(TAG, "keepalive PINGREQ");
    return true;
}

static uint32_t s_await; /* 位图：等待 CONNACK/SUBACK */

#define AWAIT_CONNACK 1u
#define AWAIT_SUBACK  2u

static void handle_downlink(const mqtt_msg_t *msg)
{
    /* 下行 JSON-RPC 请求（<prefix>/<device>/call）→ RPC 分发 → QoS1 回 <prefix>/<device>/reply */
    char text[160];
    size_t n = msg->payload_len < sizeof(text) - 1 ? msg->payload_len : sizeof(text) - 1;
    if (n > 0 && msg->payload != NULL) {
        memcpy(text, msg->payload, n);
    }
    text[n] = '\0';
    if (s_mutex != NULL) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_status.downlink_count++;
        xSemaphoreGive(s_mutex);
    }

    static char resp[RPC_RESPONSE_MAX];
    int resp_len = rpc_server_handle(text, msg->payload_len, resp, sizeof(resp));
    if (resp_len <= 0) {
        ESP_LOGI(TAG, "下行通知（无需回复）: %s", text);
        return;
    }

    char reply_topic[BMS_UPLINK_TOPIC_LEN];
    snprintf(reply_topic, sizeof(reply_topic), "%s/%s/reply", s_cfg.topic_prefix,
             s_status.device_id);
    bool ok = mqtt_publish_raw(reply_topic, resp, (size_t)resp_len, 1, false);
    ESP_LOGI(TAG, "%s → %s (%d 字节)", msg->topic, reply_topic, resp_len);
    if (!ok) {
        set_error("RPC 回包失败");
    }
}

static void handle_message(const mqtt_msg_t *msg)
{
    switch (msg->type) {
    case MQTT_TYPE_CONNACK:
        if (msg->payload_len >= 2) {
            if (msg->payload[1] == 0) {
                s_await &= ~AWAIT_CONNACK;
                if (s_mutex != NULL) {
                    xSemaphoreTake(s_mutex, portMAX_DELAY);
                    s_status.mqtt_connected = true;
                    xSemaphoreGive(s_mutex);
                }
                ESP_LOGI(TAG, "MQTT 已连接 %s:%u", s_cfg.host, (unsigned)s_cfg.port);
            } else {
                ESP_LOGW(TAG, "MQTT CONNACK 返回码 %u", (unsigned)msg->payload[1]);
                set_error("MQTT 被拒绝");
            }
        }
        break;
    case MQTT_TYPE_SUBACK:
        s_await &= ~AWAIT_SUBACK;
        if (s_mutex != NULL) {
            xSemaphoreTake(s_mutex, portMAX_DELAY);
            s_status.subscribed = true;
            xSemaphoreGive(s_mutex);
        }
        ESP_LOGI(TAG, "订阅成功: %s/%s/call", s_cfg.topic_prefix, s_status.device_id);
        break;
    case MQTT_TYPE_PUBLISH: {
        uint8_t qos = (uint8_t)((msg->flags >> 1) & 0x03);
        if (qos == 1) {
            size_t n = mqtt_build_puback(s_tx_pkt, sizeof(s_tx_pkt), msg->packet_id);
            mqtt_send(s_tx_pkt, n, "PUBACK 组包失败");
        }
        handle_downlink(msg);
        break;
    }
    case MQTT_TYPE_PINGRESP:
        ESP_LOGD(TAG, "PINGRESP");
        break;
    default:
        ESP_LOGD(TAG, "MQTT 报文 type=%u flags=0x%02x", msg->type, msg->flags);
        break;
    }
}

/* 把 4G 收到的新字节喂给解析器，分发所有完整报文 */
static void pump_rx(void)
{
    uint8_t chunk[512];
    size_t n;
    mqtt_msg_t msg;

    while ((n = ring_drain(chunk, sizeof(chunk))) > 0) {
        if (!mqtt_rx_push(&s_rx, chunk, n)) {
            ESP_LOGW(TAG, "MQTT 接收缓冲溢出，已丢弃最旧数据");
        }
    }
    while (mqtt_rx_next(&s_rx, &msg)) {
        handle_message(&msg);
    }
}

static void mqtt_mark_disconnected(const char *reason)
{
    if (s_mutex != NULL) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        bool was = s_status.mqtt_connected;
        s_status.mqtt_connected = false;
        s_status.subscribed = false;
        xSemaphoreGive(s_mutex);
        if (was) {
            ESP_LOGW(TAG, "MQTT 断开（%s）", reason);
        }
    }
    mqtt_rx_init(&s_rx);
}

/* 首次连接前刷新 device_id：优先 4G IMEI（与 mqttagent 的 mobile.imei() 一致），
 * 模块还没读到 IMEI 时先用本机 PN，读到后自动切换并更新主题。 */
static void refresh_device_id(void)
{
    if (strncmp(s_status.device_id, "SOC-", 4) != 0) {
        return;
    }
    ml307_4g_status_t lte;
    ml307_4g_get_status(&lte);
    if (strlen(lte.imei) < 14) {
        return;
    }
    if (s_mutex != NULL) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        snprintf(s_status.device_id, sizeof(s_status.device_id), "%s", lte.imei);
        snprintf(s_status.status_topic, sizeof(s_status.status_topic), "%s/%s/status",
                 s_cfg.topic_prefix, s_status.device_id);
        xSemaphoreGive(s_mutex);
    }
    ESP_LOGI(TAG, "device_id 切换为 IMEI: %s", lte.imei);
}

static bool mqtt_connect(void)
{
    size_t n = mqtt_build_connect(s_tx_pkt, sizeof(s_tx_pkt), s_status.device_id,
                                  s_cfg.user, s_cfg.password, s_cfg.keepalive_s, true);
    if (!mqtt_send(s_tx_pkt, n, "CONNECT 组包失败")) {
        return false;
    }
    s_await = AWAIT_CONNACK;
    uint32_t start = now_ms();
    while (now_ms() - start < UPLINK_CONNECT_TO_MS) {
        pump_rx();
        if ((s_await & AWAIT_CONNACK) == 0) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    set_error("MQTT CONNACK 超时");
    return false;
}

static void mqtt_subscribe(void)
{
    char topic[BMS_UPLINK_TOPIC_LEN];
    snprintf(topic, sizeof(topic), "%s/%s/call", s_cfg.topic_prefix, s_status.device_id);
    size_t n = mqtt_build_subscribe(s_tx_pkt, sizeof(s_tx_pkt), ++s_packet_id, topic, 1);
    if (!mqtt_send(s_tx_pkt, n, "SUBSCRIBE 组包失败")) {
        return;
    }
    s_await = AWAIT_SUBACK;
    uint32_t start = now_ms();
    while (now_ms() - start < 5000) {
        pump_rx();
        if ((s_await & AWAIT_SUBACK) == 0) {
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    ESP_LOGW(TAG, "SUBACK 超时（下行 RPC 暂不可用）");
}

/* ---------------- 批次发布 ---------------- */

static bool publish_batch(void)
{
    if (s_hist.count == 0) {
        return true;
    }
    size_t len = bms_hist_encode(&s_hist, s_payload, sizeof(s_payload));
    if (len == 0) {
        set_error("批次编码失败/超长");
        if (s_mutex != NULL) {
            xSemaphoreTake(s_mutex, portMAX_DELAY);
            s_status.batches_fail++;
            xSemaphoreGive(s_mutex);
        }
        bms_hist_reset(&s_hist);
        return false;
    }

    bool ok = mqtt_publish_raw(s_status.status_topic, s_payload, len, 0, false);
    if (s_mutex != NULL) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        if (ok) {
            s_status.batches_ok++;
            s_status.last_payload_size = (uint32_t)len;
            s_status.last_publish_ms = now_ms();
        } else {
            s_status.batches_fail++;
        }
        xSemaphoreGive(s_mutex);
    }
    if (ok) {
        ESP_LOGI(TAG, "已上报 %u 条样本（%u 字节）到 %s", (unsigned)s_hist.count,
                 (unsigned)len, s_status.status_topic);
    }
    led_ctrl_notify_event(LED_EVENT_LTE, ok);
    bms_hist_reset(&s_hist);
    return ok;
}

/* ---------------- 主任务 ---------------- */

static void uplink_task(void *arg)
{
    (void)arg;

    bms_hist_init(&s_hist, s_cfg.batch_samples);
    mqtt_rx_init(&s_rx);
    ml307_4g_set_rx_callback(uplink_rx_push);

    uint32_t last_sample = now_ms();
    uint32_t last_ping = now_ms();
    uint32_t last_time_sync = 0;
    uint32_t pub_fails = 0;

    for (;;) {
        if (!s_cfg.enable) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        if (!ml307_4g_socket_ready()) {
            mqtt_mark_disconnected("socket 未连接");
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        if (!s_status.mqtt_connected) {
            refresh_device_id(); /* IMEI 就绪后自动切换 device_id */
            if (!mqtt_connect()) {
                ml307_4g_request_reconnect();
                vTaskDelay(pdMS_TO_TICKS(3000));
                continue;
            }
            mqtt_subscribe();
            last_ping = now_ms();
        }

        pump_rx();
        uint32_t now = now_ms();

        if (s_req_reconnect) {
            s_req_reconnect = false;
            mqtt_mark_disconnected("手动重连");
            ml307_4g_request_reconnect();
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        /* 每小时重新对齐一次 4G 网络时间 */
        if (now - last_time_sync >= UPLINK_TIME_SYNC_S * 1000u) {
            uint32_t epoch = 0;
            if (ml307_4g_get_epoch(&epoch) && epoch > 1600000000u) {
                s_time_offset = epoch - (uint32_t)(esp_timer_get_time() / 1000000);
                s_time_valid = true;
            }
            last_time_sync = now;
        }

        /* 采样 */
        if (now - last_sample >= s_cfg.sample_interval_ms) {
            last_sample = now;
            bms_snapshot_t snap;
            if (bms_manager_get_snapshot(&snap) && snap.have_data) {
                bool full = bms_hist_add(&s_hist, &snap, now_epoch());
                if (s_mutex != NULL) {
                    xSemaphoreTake(s_mutex, portMAX_DELAY);
                    s_status.samples++;
                    xSemaphoreGive(s_mutex);
                }
                if (full || s_req_publish) {
                    s_req_publish = false;
                    if (publish_batch()) {
                        pub_fails = 0;
                    } else if (++pub_fails >= UPLINK_PUBLISH_FAIL_MAX) {
                        ESP_LOGW(TAG, "连续 %u 次发布失败，重建 socket",
                                 (unsigned)pub_fails);
                        pub_fails = 0;
                        mqtt_mark_disconnected("发布失败");
                        ml307_4g_request_reconnect();
                    }
                }
            } else if (s_req_publish) {
                s_req_publish = false;
                set_error("暂无 BMS 数据可上报");
            }
        }

        /* 保活：keepalive/2 发一次 PINGREQ */
        if (now - last_ping >= (uint32_t)s_cfg.keepalive_s * 500u) {
            last_ping = now;
            if (!mqtt_publish_ping()) {
                mqtt_mark_disconnected("PING 发送失败");
                ml307_4g_request_reconnect();
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ---------------- 对外接口 ---------------- */

void bms_uplink_start(const bms_uplink_config_t *cfg)
{
    s_cfg = *cfg;
    s_mutex = xSemaphoreCreateMutex();
    s_ring_mutex = xSemaphoreCreateMutex();
    memset(&s_status, 0, sizeof(s_status));
    s_status.enabled = cfg->enable;
    snprintf(s_status.last_error, sizeof(s_status.last_error), "%s", "未启动");

    /* device_id：优先用 4G 模块 IMEI，取不到时回退本机 PN */
    ml307_4g_status_t lte;
    ml307_4g_get_status(&lte);
    if (strlen(lte.imei) >= 14) {
        snprintf(s_status.device_id, sizeof(s_status.device_id), "%s", lte.imei);
    } else {
        bms_mac_pn(s_status.device_id, sizeof(s_status.device_id));
    }
    snprintf(s_status.status_topic, sizeof(s_status.status_topic), "%s/%s/status",
             cfg->topic_prefix, s_status.device_id);
    s_started = true;

    if (!cfg->enable) {
        ESP_LOGW(TAG, "MQTT 上报已在配置中关闭");
        return;
    }
    if (cfg->host[0] == '\0' || cfg->port == 0) {
        ESP_LOGW(TAG, "未配置 MQTT broker，采集上报不启动");
        return;
    }

    ESP_LOGI(TAG, "上报已启动: broker=%s:%u device=%s topic=%s 采样=%u ms × %u 条",
             cfg->host, (unsigned)cfg->port, s_status.device_id,
             s_status.status_topic, (unsigned)cfg->sample_interval_ms,
             (unsigned)cfg->batch_samples);
    xTaskCreate(uplink_task, "bms_uplink", UPLINK_TASK_STACK, NULL,
                UPLINK_TASK_PRIO, NULL);
}

void bms_uplink_get_status(bms_uplink_status_t *out)
{
    if (out == NULL) {
        return;
    }
    if (s_mutex == NULL) {
        memset(out, 0, sizeof(*out));
        return;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *out = s_status;
    xSemaphoreGive(s_mutex);
}

void bms_uplink_request_publish(void)
{
    s_req_publish = true;
}

void bms_uplink_request_reconnect(void)
{
    s_req_reconnect = true;
}
