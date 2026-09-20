#include "log_stream.h"

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_log_write.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "sdkconfig.h"

#define LOG_STREAM_MAX         CONFIG_JINJIAN_MAX_LOG_STREAMS
#define LOG_STREAM_PORT        CONFIG_JINJIAN_LOG_STREAM_PORT
#define LOG_STREAM_LINE_MAX    160
#define LOG_STREAM_TX_CAP      2048
#define LOG_STREAM_REQ_CAP     512
#define LOG_STREAM_KEEPALIVE_MS 5000

static const char *TAG = "log_stream";

typedef struct {
    bool used;
    int fd;
    uint8_t req[LOG_STREAM_REQ_CAP];
    size_t req_len;
    bool headers_sent;
    uint8_t tx[LOG_STREAM_TX_CAP];
    size_t tx_len;
    uint64_t last_activity_ms;
} log_stream_conn_t;

static log_stream_conn_t s_conns[LOG_STREAM_MAX];
static SemaphoreHandle_t s_mutex;
static vprintf_like_t s_prev_vprintf;
static TaskHandle_t s_task;
static bool s_init_done;

static uint64_t now_ms(void)
{
    return (uint64_t)(esp_timer_get_time() / 1000);
}

static void conn_close_locked(log_stream_conn_t *c)
{
    if (!c->used) {
        return;
    }
    c->used = false;
    c->tx_len = 0;
    c->req_len = 0;
    c->headers_sent = false;
    int fd = c->fd;
    c->fd = -1;
    if (fd >= 0) {
        close(fd);
    }
}

static void log_stream_emit(const char *line, size_t len)
{
    if (len == 0) {
        return;
    }
    if (len > LOG_STREAM_LINE_MAX) {
        len = LOG_STREAM_LINE_MAX;
    }

    char frame[LOG_STREAM_LINE_MAX + 20];
    int head = snprintf(frame, sizeof(frame), "%x\r\n", (unsigned)len);
    if (head < 0 || head + (int)len + 2 > (int)sizeof(frame)) {
        return;
    }
    memcpy(frame + head, line, len);
    head += (int)len;
    frame[head++] = '\r';
    frame[head++] = '\n';

    if (s_mutex == NULL) {
        return;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (int i = 0; i < LOG_STREAM_MAX; i++) {
        log_stream_conn_t *c = &s_conns[i];
        if (!c->used || !c->headers_sent) {
            continue;
        }
        if (c->tx_len + (size_t)head <= sizeof(c->tx)) {
            memcpy(c->tx + c->tx_len, frame, (size_t)head);
            c->tx_len += (size_t)head;
        }
        /* 满则丢弃当前行：只保证实时性，不缓存历史 */
    }
    xSemaphoreGive(s_mutex);
}

static int log_vprintf(const char *fmt, va_list args)
{
    char buf[LOG_STREAM_LINE_MAX + 32];
    va_list copy;
    va_copy(copy, args);
    vsnprintf(buf, sizeof(buf), fmt, copy);
    va_end(copy);

    size_t len = strlen(buf);
    while (len > 0 && (buf[len - 1] == '\n' || buf[len - 1] == '\r')) {
        buf[--len] = '\0';
    }
    log_stream_emit(buf, len);

    if (s_prev_vprintf) {
        return s_prev_vprintf(fmt, args);
    }
    return vprintf(fmt, args);
}

static int set_nonblocking(int fd)
{
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0) {
        return -1;
    }
    return fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

static void send_http_headers(log_stream_conn_t *c)
{
    static const char resp[] =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/plain; charset=utf-8\r\n"
        "Transfer-Encoding: chunked\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: keep-alive\r\n\r\n";
    memcpy(c->tx, resp, sizeof(resp) - 1);
    c->tx_len = sizeof(resp) - 1;
    c->headers_sent = true;
    c->last_activity_ms = now_ms();
}

static void send_http_503(int fd)
{
    static const char resp[] =
        "HTTP/1.1 503 Service Unavailable\r\n"
        "Content-Length: 0\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Connection: close\r\n\r\n";
    send(fd, resp, sizeof(resp) - 1, 0);
}

static void send_http_options(int fd)
{
    static const char resp[] =
        "HTTP/1.1 204 No Content\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Access-Control-Allow-Methods: GET, OPTIONS\r\n"
        "Access-Control-Allow-Headers: *\r\n"
        "Content-Length: 0\r\n\r\n";
    send(fd, resp, sizeof(resp) - 1, 0);
}

static const uint8_t *find_crlf_crlf(const uint8_t *buf, size_t len)
{
    for (size_t i = 0; i + 4 <= len; i++) {
        if (buf[i] == '\r' && buf[i + 1] == '\n' &&
            buf[i + 2] == '\r' && buf[i + 3] == '\n') {
            return buf + i;
        }
    }
    return NULL;
}

static void handle_read(log_stream_conn_t *c)
{
    uint8_t tmp[128];
    ssize_t n = recv(c->fd, tmp, sizeof(tmp), 0);
    if (n <= 0) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        conn_close_locked(c);
        xSemaphoreGive(s_mutex);
        return;
    }
    if (c->headers_sent) {
        return;
    }

    for (ssize_t i = 0; i < n; i++) {
        if (c->req_len + 1 < sizeof(c->req)) {
            c->req[c->req_len++] = tmp[i];
        } else {
            xSemaphoreTake(s_mutex, portMAX_DELAY);
            conn_close_locked(c);
            xSemaphoreGive(s_mutex);
            return;
        }
    }

    if (c->req_len >= 4 && find_crlf_crlf(c->req, c->req_len) != NULL) {
        if (c->req_len >= 8 && memcmp(c->req, "OPTIONS ", 8) == 0) {
            send_http_options(c->fd);
            xSemaphoreTake(s_mutex, portMAX_DELAY);
            conn_close_locked(c);
            xSemaphoreGive(s_mutex);
            return;
        }
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        if (c->used) {
            send_http_headers(c);
        }
        xSemaphoreGive(s_mutex);
    }
}

static void append_keepalive(log_stream_conn_t *c)
{
    static const char chunk[] = "1\r\n\n\r\n";
    if (c->tx_len + sizeof(chunk) - 1 <= sizeof(c->tx)) {
        memcpy(c->tx + c->tx_len, chunk, sizeof(chunk) - 1);
        c->tx_len += sizeof(chunk) - 1;
    }
}

static void handle_write(log_stream_conn_t *c)
{
    if (c->tx_len == 0) {
        if (now_ms() - c->last_activity_ms >= LOG_STREAM_KEEPALIVE_MS) {
            append_keepalive(c);
            c->last_activity_ms = now_ms();
        }
        if (c->tx_len == 0) {
            return;
        }
    }

    ssize_t n = send(c->fd, c->tx, c->tx_len, 0);
    if (n > 0) {
        if ((size_t)n >= c->tx_len) {
            c->tx_len = 0;
        } else {
            memmove(c->tx, c->tx + n, c->tx_len - (size_t)n);
            c->tx_len -= (size_t)n;
        }
        c->last_activity_ms = now_ms();
        return;
    }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        return;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    conn_close_locked(c);
    xSemaphoreGive(s_mutex);
}

static void accept_connection(int listen_fd)
{
    struct sockaddr_in addr;
    socklen_t addr_len = sizeof(addr);
    int fd = accept(listen_fd, (struct sockaddr *)&addr, &addr_len);
    if (fd < 0) {
        return;
    }
    set_nonblocking(fd);

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    log_stream_conn_t *slot = NULL;
    for (int i = 0; i < LOG_STREAM_MAX; i++) {
        if (!s_conns[i].used) {
            slot = &s_conns[i];
            break;
        }
    }
    if (slot == NULL) {
        xSemaphoreGive(s_mutex);
        ESP_LOGW(TAG, "log stream rejected, max=%d", LOG_STREAM_MAX);
        send_http_503(fd);
        close(fd);
        return;
    }
    memset(slot, 0, sizeof(*slot));
    slot->used = true;
    slot->fd = fd;
    slot->last_activity_ms = now_ms();
    ESP_LOGI(TAG, "log stream opened (%d/%d)", (int)(slot - s_conns) + 1,
             LOG_STREAM_MAX);
    xSemaphoreGive(s_mutex);
}

static void log_stream_task(void *arg)
{
    (void)arg;
    int listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0) {
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(LOG_STREAM_PORT),
        .sin_addr.s_addr = INADDR_ANY,
    };
    if (bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0 ||
        listen(listen_fd, 8) != 0 || set_nonblocking(listen_fd) != 0) {
        close(listen_fd);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "log stream server on :%u, max=%d", LOG_STREAM_PORT, LOG_STREAM_MAX);

    while (1) {
        fd_set rfds;
        fd_set wfds;
        FD_ZERO(&rfds);
        FD_ZERO(&wfds);
        FD_SET(listen_fd, &rfds);
        int maxfd = listen_fd;

        xSemaphoreTake(s_mutex, portMAX_DELAY);
        for (int i = 0; i < LOG_STREAM_MAX; i++) {
            log_stream_conn_t *c = &s_conns[i];
            if (!c->used) {
                continue;
            }
            FD_SET(c->fd, &rfds);
            if (c->headers_sent) {
                FD_SET(c->fd, &wfds);
            }
            if (c->fd > maxfd) {
                maxfd = c->fd;
            }
        }
        xSemaphoreGive(s_mutex);

        struct timeval tv = {0, 100000};
        int active = select(maxfd + 1, &rfds, &wfds, NULL, &tv);
        if (active < 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (FD_ISSET(listen_fd, &rfds)) {
            accept_connection(listen_fd);
        }

        xSemaphoreTake(s_mutex, portMAX_DELAY);
        for (int i = 0; i < LOG_STREAM_MAX; i++) {
            log_stream_conn_t *c = &s_conns[i];
            if (!c->used) {
                continue;
            }
            bool readable = FD_ISSET(c->fd, &rfds);
            bool writable = FD_ISSET(c->fd, &wfds);
            if (readable && !c->headers_sent) {
                xSemaphoreGive(s_mutex);
                handle_read(c);
                xSemaphoreTake(s_mutex, portMAX_DELAY);
                if (!c->used) {
                    continue;
                }
            }
            if (c->used && c->headers_sent && writable) {
                xSemaphoreGive(s_mutex);
                handle_write(c);
                xSemaphoreTake(s_mutex, portMAX_DELAY);
            }
        }
        xSemaphoreGive(s_mutex);
    }
}

size_t log_stream_active_count(void)
{
    size_t count = 0;
    if (s_mutex == NULL) {
        return 0;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (int i = 0; i < LOG_STREAM_MAX; i++) {
        if (s_conns[i].used) {
            count++;
        }
    }
    xSemaphoreGive(s_mutex);
    return count;
}

void log_stream_init(void)
{
    if (s_init_done) {
        return;
    }
    s_init_done = true;
    s_mutex = xSemaphoreCreateMutex();
    memset(s_conns, 0, sizeof(s_conns));
    s_prev_vprintf = esp_log_set_vprintf(log_vprintf);
    ESP_LOGI(TAG, "log stream vprintf installed, max=%d", LOG_STREAM_MAX);
    xTaskCreate(log_stream_task, "log_stream", 4096, NULL, 4, &s_task);
}
