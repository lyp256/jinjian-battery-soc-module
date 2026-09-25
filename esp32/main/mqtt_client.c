/* 极简 MQTT 3.1.1 编解码实现（详见 mqtt_client.h）。 */

#include "mqtt_client.h"

#include <string.h>

/* ---------------- 组包 ---------------- */

static size_t put_u16(uint8_t *out, uint16_t v)
{
    out[0] = (uint8_t)(v >> 8);
    out[1] = (uint8_t)(v & 0xFF);
    return 2;
}

static size_t put_string(uint8_t *out, const char *s)
{
    size_t len = strlen(s);
    size_t n = put_u16(out, (uint16_t)len);
    memcpy(out + n, s, len);
    return n + len;
}

/* 写固定头：类型/标志 + 剩余长度（最多 4 字节 varint） */
static size_t put_fixed_header(uint8_t *out, uint8_t type, uint8_t flags,
                               size_t remaining)
{
    size_t n = 0;
    out[n++] = (uint8_t)((type << 4) | flags);
    do {
        uint8_t b = (uint8_t)(remaining & 0x7F);
        remaining >>= 7;
        if (remaining > 0) {
            b |= 0x80;
        }
        out[n++] = b;
    } while (remaining > 0);
    return n;
}

static size_t remaining_len_size(size_t remaining)
{
    size_t n = 1;
    while (remaining >= 0x80) {
        remaining >>= 7;
        n++;
    }
    return n;
}

size_t mqtt_build_connect(uint8_t *out, size_t cap, const char *client_id,
                          const char *user, const char *password,
                          uint16_t keepalive_s, bool clean_session)
{
    bool has_user = user != NULL && user[0] != '\0';
    bool has_pass = password != NULL && password[0] != '\0';
    size_t id_len = strlen(client_id);
    size_t user_len = has_user ? strlen(user) : 0;
    size_t pass_len = has_pass ? strlen(password) : 0;

    /* 可变头 10 字节 + client id + user + password */
    size_t remaining = 10 + 2 + id_len;
    if (has_user) {
        remaining += 2 + user_len;
    }
    if (has_pass) {
        remaining += 2 + pass_len;
    }
    size_t total = 1 + remaining_len_size(remaining) + remaining;
    if (total > cap) {
        return 0;
    }

    size_t n = put_fixed_header(out, MQTT_TYPE_CONNECT, 0, remaining);
    /* 协议名 "MQTT"，级别 4 */
    n += put_string(out + n, "MQTT");
    out[n++] = 4;
    uint8_t flags = 0;
    if (clean_session) {
        flags |= 0x02;
    }
    if (has_user) {
        flags |= 0x80;
    }
    if (has_pass) {
        flags |= 0x40;
    }
    out[n++] = flags;
    n += put_u16(out + n, keepalive_s);
    n += put_string(out + n, client_id);
    if (has_user) {
        n += put_string(out + n, user);
    }
    if (has_pass) {
        n += put_string(out + n, password);
    }
    return n;
}

size_t mqtt_build_subscribe(uint8_t *out, size_t cap, uint16_t packet_id,
                            const char *topic, uint8_t qos)
{
    size_t topic_len = strlen(topic);
    size_t remaining = 2 + 2 + topic_len + 1;
    size_t total = 1 + remaining_len_size(remaining) + remaining;
    if (total > cap) {
        return 0;
    }
    size_t n = put_fixed_header(out, MQTT_TYPE_SUBSCRIBE, 0x02, remaining);
    n += put_u16(out + n, packet_id);
    n += put_string(out + n, topic);
    out[n++] = (uint8_t)(qos & 0x03);
    return n;
}

size_t mqtt_build_publish(uint8_t *out, size_t cap, const char *topic,
                          const void *payload, size_t payload_len,
                          uint8_t qos, bool retain, uint16_t packet_id)
{
    size_t topic_len = strlen(topic);
    size_t remaining = 2 + topic_len + payload_len;
    if (qos > 0) {
        remaining += 2;
    }
    size_t total = 1 + remaining_len_size(remaining) + remaining;
    if (total > cap) {
        return 0;
    }
    uint8_t flags = (uint8_t)((qos & 0x03) << 1);
    if (retain) {
        flags |= 0x01;
    }
    size_t n = put_fixed_header(out, MQTT_TYPE_PUBLISH, flags, remaining);
    n += put_string(out + n, topic);
    if (qos > 0) {
        n += put_u16(out + n, packet_id);
    }
    if (payload_len > 0 && payload != NULL) {
        memcpy(out + n, payload, payload_len);
        n += payload_len;
    }
    return n;
}

size_t mqtt_build_puback(uint8_t *out, size_t cap, uint16_t packet_id)
{
    if (cap < 4) {
        return 0;
    }
    size_t n = put_fixed_header(out, MQTT_TYPE_PUBACK, 0, 2);
    n += put_u16(out + n, packet_id);
    return n;
}

size_t mqtt_build_pingreq(uint8_t *out, size_t cap)
{
    if (cap < 2) {
        return 0;
    }
    out[0] = (uint8_t)(MQTT_TYPE_PINGREQ << 4);
    out[1] = 0;
    return 2;
}

size_t mqtt_build_disconnect(uint8_t *out, size_t cap)
{
    if (cap < 2) {
        return 0;
    }
    out[0] = (uint8_t)(MQTT_TYPE_DISCONNECT << 4);
    out[1] = 0;
    return 2;
}

/* ---------------- 解包 ---------------- */

void mqtt_rx_init(mqtt_rx_t *rx)
{
    rx->len = 0;
}

bool mqtt_rx_push(mqtt_rx_t *rx, const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    bool ok = true;
    while (len > 0) {
        size_t space = sizeof(rx->buf) - rx->len;
        if (space == 0) {
            /* 缓冲塞满：丢最旧的一半，保证能继续接收 */
            size_t keep = sizeof(rx->buf) / 2;
            memmove(rx->buf, rx->buf + (sizeof(rx->buf) - keep), keep);
            rx->len = keep;
            space = sizeof(rx->buf) - rx->len;
            ok = false;
        }
        size_t n = len < space ? len : space;
        memcpy(rx->buf + rx->len, p, n);
        rx->len += n;
        p += n;
        len -= n;
    }
    return ok;
}

bool mqtt_rx_next(mqtt_rx_t *rx, mqtt_msg_t *msg)
{
    if (rx->len < 2) {
        return false;
    }
    /* 解析剩余长度 varint（最多 4 字节） */
    size_t value = 0;
    size_t multiplier = 1;
    size_t hdr = 1;
    size_t remaining = 0;
    bool complete_len = false;
    for (size_t i = 1; i < rx->len && hdr < 5; i++) {
        uint8_t b = rx->buf[i];
        value += (size_t)(b & 0x7F) * multiplier;
        multiplier *= 128;
        hdr++;
        if ((b & 0x80) == 0) {
            remaining = value;
            complete_len = true;
            break;
        }
    }
    if (!complete_len) {
        return false; /* 长度字段还没收全 */
    }
    if (rx->len < hdr + remaining) {
        return false; /* 报文还没收全 */
    }

    memset(msg, 0, sizeof(*msg));
    msg->type = (uint8_t)(rx->buf[0] >> 4);
    msg->flags = (uint8_t)(rx->buf[0] & 0x0F);

    const uint8_t *body = rx->buf + hdr;
    size_t pos = 0;
    if (msg->type == MQTT_TYPE_PUBLISH) {
        if (remaining < 2) {
            goto drop;
        }
        size_t topic_len = ((size_t)body[0] << 8) | body[1];
        pos = 2;
        if (remaining < pos + topic_len || topic_len >= sizeof(msg->topic)) {
            goto drop;
        }
        memcpy(msg->topic, body + pos, topic_len);
        msg->topic[topic_len] = '\0';
        msg->topic_len = topic_len;
        pos += topic_len;
        uint8_t qos = (uint8_t)((msg->flags >> 1) & 0x03);
        if (qos > 0) {
            if (remaining < pos + 2) {
                goto drop;
            }
            msg->packet_id = (uint16_t)(((uint16_t)body[pos] << 8) | body[pos + 1]);
            pos += 2;
        }
        msg->payload = body + pos;
        msg->payload_len = remaining - pos;
    } else if (msg->type == MQTT_TYPE_CONNACK || msg->type == MQTT_TYPE_SUBACK ||
               msg->type == MQTT_TYPE_PUBACK) {
        msg->payload = body;
        msg->payload_len = remaining;
        if (remaining >= 2) {
            msg->packet_id = (uint16_t)(((uint16_t)body[0] << 8) | body[1]);
        }
        if (msg->type == MQTT_TYPE_CONNACK && remaining >= 2) {
            msg->packet_id = 0;
        }
    } else {
        msg->payload = body;
        msg->payload_len = remaining;
    }

    /* 消费该报文 */
    memmove(rx->buf, rx->buf + hdr + remaining, rx->len - hdr - remaining);
    rx->len -= hdr + remaining;
    return true;

drop:
    memmove(rx->buf, rx->buf + hdr + remaining, rx->len - hdr - remaining);
    rx->len -= hdr + remaining;
    return false;
}
