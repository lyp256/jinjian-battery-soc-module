/* ML307-NL 4G（Cat.1）模块驱动：EN 上电时序 + AT 指令状态机 + TCP 数据上报。
 *
 * 硬件接线（默认，可在 menuconfig / Web 页面修改）：
 *   GPIO13 → 模块 EN / PWR_ON（低电平有效，脉冲开机）
 *   GPIO12 → 模块 RX（ESP32 TX）
 *   GPIO11 → 模块 TX（ESP32 RX）
 *
 * 状态机：上电 → AT 应答 → 初始化/读 IMEI → SIM 就绪 → LTE 注册 →
 *         TCP socket（AT+MIPOPEN）→ 周期上报（AT+MIPSEND）→ 异常重试。
 */

#include "ml307_4g.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "ml307";

#define ML307_TASK_STACK   5120
#define ML307_TASK_PRIO    4
#define ML307_RX_BUF       2048
#define ML307_RX_CHUNK     512
#define ML307_LINE_MAX     256
#define ML307_RESP_MAX     640

#define ML307_POWER_OFF_MS 2000   /* 关机脉冲宽度 */
#define ML307_POWER_GAP_MS 1000   /* 断电到重新开机的间隔 */
#define ML307_BOOT_MS      3000   /* EN 之后等待模块启动 */
#define ML307_RETRY_MS     10000  /* 重试间隔 */
#define ML307_STAT_MS      30000  /* 在线期间刷新 CSQ/CEREG 的间隔 */

static const char *const STATE_NAMES[] = {
    "disabled", "power-on", "wait-at", "init",
    "sim", "network", "socket", "online", "retry",
};

typedef struct {
    const char *const *keywords;
    size_t keyword_count;
    bool stop_on_keyword;
    bool stop_on_prompt;
    uint32_t timeout_ms;
} at_opts_t;

typedef struct {
    char text[ML307_RESP_MAX];
    size_t len;
    bool ok;      /* 收到 OK */
    bool error;   /* ERROR / +CME ERROR */
    bool prompt;  /* 收到 '>' 提示符 */
    bool matched; /* 命中关键字 */
} at_resp_t;

static ml307_4g_config_t s_cfg;
static ml307_4g_status_t s_status;
static SemaphoreHandle_t s_at_mutex;     /* 串口 + AT 序列互斥 */
static SemaphoreHandle_t s_status_mutex; /* 状态快照互斥 */
static int s_uart = -1;
static bool s_started;

static volatile bool s_req_reconnect;
static volatile bool s_req_power_cycle;

static void (*s_rx_cb)(const void *data, size_t len);

/* ---------------- 基础工具 ---------------- */

static uint32_t ml307_now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void set_error(const char *msg)
{
    xSemaphoreTake(s_status_mutex, portMAX_DELAY);
    snprintf(s_status.last_error, sizeof(s_status.last_error), "%s", msg);
    xSemaphoreGive(s_status_mutex);
}

static void set_state(ml307_state_t st)
{
    xSemaphoreTake(s_status_mutex, portMAX_DELAY);
    s_status.state = st;
    s_status.state_name = STATE_NAMES[(int)st];
    xSemaphoreGive(s_status_mutex);
    ESP_LOGI(TAG, "state -> %s", STATE_NAMES[(int)st]);
}

/* ---------------- UART / 行读取 ---------------- */

static uint8_t s_rx[ML307_RX_CHUNK];
static size_t s_rx_len;

static void ml307_uart_init(void)
{
    /* 先把 TX 拉高：UART 初始化前避免模块 RX 悬空收到杂波 */
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << s_cfg.tx_gpio,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(s_cfg.tx_gpio, 1);

    uart_config_t uc = {
        .baud_rate = (int)s_cfg.baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(s_cfg.uart_num, &uc));
    ESP_ERROR_CHECK(uart_set_pin(s_cfg.uart_num, s_cfg.tx_gpio,
                                 s_cfg.rx_gpio, -1, -1));
    ESP_ERROR_CHECK(uart_driver_install(s_cfg.uart_num, ML307_RX_BUF, 0, 0,
                                        NULL, 0));
    s_uart = s_cfg.uart_num;
    ESP_LOGI(TAG, "uart%d ready: tx=%d rx=%d baud=%u",
             s_cfg.uart_num, s_cfg.tx_gpio, s_cfg.rx_gpio,
             (unsigned)s_cfg.baud_rate);
}

/* 读取一行（以 '\n' 结尾），返回 false 表示超时。调用者需持有 s_at_mutex。 */
static bool at_next_line(char *out, size_t cap, uint32_t timeout_ms)
{
    uint32_t start = ml307_now_ms();

    for (;;) {
        for (size_t i = 0; i < s_rx_len; i++) {
            if (s_rx[i] != '\n') {
                continue;
            }
            size_t len = i;
            while (len > 0 && (s_rx[len - 1] == '\r' || s_rx[len - 1] == '\n')) {
                len--;
            }
            size_t n = len < cap - 1 ? len : cap - 1;
            memcpy(out, s_rx, n);
            out[n] = '\0';
            memmove(s_rx, s_rx + i + 1, s_rx_len - i - 1);
            s_rx_len -= i + 1;
            return true;
        }
        if (s_rx_len >= sizeof(s_rx)) {
            s_rx_len = 0; /* 行过长：丢弃避免溢出 */
        }
        uint32_t elapsed = ml307_now_ms() - start;
        if (elapsed >= timeout_ms) {
            return false;
        }
        int n = uart_read_bytes(s_uart, s_rx + s_rx_len,
                                sizeof(s_rx) - s_rx_len,
                                pdMS_TO_TICKS(timeout_ms - elapsed));
        if (n > 0) {
            s_rx_len += (size_t)n;
        }
    }
}

/* ---------------- 响应解析 ---------------- */

static void resp_add(at_resp_t *resp, const char *line)
{
    size_t len = strlen(line);
    if (len == 0 || resp->len + len + 2 >= sizeof(resp->text)) {
        return;
    }
    if (resp->len > 0) {
        resp->text[resp->len++] = '\n';
    }
    memcpy(resp->text + resp->len, line, len);
    resp->len += len;
    resp->text[resp->len] = '\0';
}

/* 取 "KEY:" 之后的第一个整型参数 */
static bool parse_int_after(const char *text, const char *key, int *out)
{
    const char *p = strstr(text, key);
    if (p == NULL) {
        return false;
    }
    p = strchr(p, ':');
    if (p == NULL) {
        return false;
    }
    p++;
    while (*p == ' ') {
        p++;
    }
    char *end = NULL;
    long v = strtol(p, &end, 10);
    if (end == p) {
        return false;
    }
    *out = (int)v;
    return true;
}

/* 解析 +CEREG/+CGREG/+CREG 的注册状态（同时兼容主动上报与查询响应） */
static int parse_reg_stat(const char *line)
{
    const char *p = strchr(line, ':');
    if (p == NULL) {
        return -1;
    }
    p++;
    int fields[4] = {0};
    int count = 0;
    while (*p && count < 4) {
        while (*p == ' ') {
            p++;
        }
        if (*p == '"') {
            break;
        }
        char *end = NULL;
        long v = strtol(p, &end, 10);
        if (end == p) {
            break;
        }
        fields[count++] = (int)v;
        p = end;
        while (*p == ' ') {
            p++;
        }
        if (*p != ',') {
            break;
        }
        p++;
    }
    if (count == 0) {
        return -1;
    }
    /* 查询响应：<n>,<stat>；主动上报：<stat>[,...] */
    if (count >= 2 && fields[0] <= 2 && fields[1] <= 5) {
        return fields[1];
    }
    return fields[0];
}

static void note_reg(const char *line)
{
    int stat = parse_reg_stat(line);
    if (stat < 0) {
        return;
    }
    xSemaphoreTake(s_status_mutex, portMAX_DELAY);
    s_status.reg_stat = stat;
    s_status.net_ok = (stat == 1 || stat == 5);
    xSemaphoreGive(s_status_mutex);
}

static void note_csq(const char *line)
{
    int csq = -1;
    if (!parse_int_after(line, "+CSQ:", &csq)) {
        return;
    }
    xSemaphoreTake(s_status_mutex, portMAX_DELAY);
    s_status.csq = csq;
    s_status.rssi_dbm = (csq >= 0 && csq <= 31) ? (-113 + csq * 2) : -1;
    xSemaphoreGive(s_status_mutex);
}

/* Howard Hinnant days_from_civil：公历日期 → 1970-01-01 起的天数 */
static int64_t days_from_civil(int year, unsigned month, unsigned day)
{
    int y = year - (month <= 2);
    const int64_t era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned doy = (153u * (month + (month > 2 ? -3u : 9u)) + 2u) / 5u + day - 1u;
    const unsigned doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;
    return era * 146097 + (int64_t)doe - 719468;
}

/* "26/09/24,11:34:07+32" → Unix 秒（时区字段为 15 分钟数） */
static bool cclk_to_epoch(const char *s, uint32_t *out)
{
    int yy = 0, mm = 0, dd = 0, hh = 0, mi = 0, ss = 0;
    if (sscanf(s, "%d/%d/%d,%d:%d:%d", &yy, &mm, &dd, &hh, &mi, &ss) != 6) {
        return false;
    }
    if (mm < 1 || mm > 12 || dd < 1 || dd > 31 || hh > 23 || mi > 59 || ss > 60) {
        return false;
    }
    int tz_quarter = 0;
    const char *plus = strchr(s, '+');
    const char *minus = strchr(s, '-');
    if (plus != NULL) {
        tz_quarter = atoi(plus + 1);
    } else if (minus != NULL) {
        tz_quarter = -atoi(minus + 1);
    }
    int64_t days = days_from_civil(yy + 2000, (unsigned)mm, (unsigned)dd);
    int64_t secs = days * 86400 + hh * 3600 + mi * 60 + ss -
                   (int64_t)tz_quarter * 900;
    if (secs < 0) {
        return false;
    }
    *out = (uint32_t)secs;
    return true;
}

static void note_cclk(const char *line)
{
    const char *p = strstr(line, "+CCLK:");
    if (p == NULL) {
        return;
    }
    p += 6;
    while (*p == ' ') {
        p++;
    }
    if (*p == '"') {
        p++;
    }
    char tmp[24];
    size_t n = 0;
    while (*p && *p != '"' && *p != '\r' && *p != '\n' && n + 1 < sizeof(tmp)) {
        tmp[n++] = *p++;
    }
    tmp[n] = '\0';
    if (n == 0) {
        return;
    }
    xSemaphoreTake(s_status_mutex, portMAX_DELAY);
    memcpy(s_status.cclk, tmp, n + 1);
    uint32_t epoch = 0;
    if (cclk_to_epoch(tmp, &epoch)) {
        s_status.unix_time = epoch;
        s_status.time_valid = true;
    }
    xSemaphoreGive(s_status_mutex);
}

/* 解析 +MIPURC: "rtcp",<id>,<len>,<data> / "disconn" / "pdpdeact" */
static void note_mipurc(const char *line)
{
    const char *p = strchr(line, ':');
    if (p == NULL) {
        return;
    }
    p++;
    while (*p == ' ') {
        p++;
    }
    if (*p != '"') {
        return;
    }
    p++;
    char kind[16];
    size_t n = 0;
    while (*p && *p != '"' && n + 1 < sizeof(kind)) {
        kind[n++] = *p++;
    }
    kind[n] = '\0';
    if (*p == '"') {
        p++;
    }

    if (strcmp(kind, "disconn") == 0 || strcmp(kind, "pdpdeact") == 0) {
        ESP_LOGW(TAG, "socket/network event: %s", kind);
        xSemaphoreTake(s_status_mutex, portMAX_DELAY);
        if (strcmp(kind, "disconn") == 0) {
            s_status.socket_ok = false;
        } else {
            s_status.net_ok = false;
            s_status.socket_ok = false;
        }
        xSemaphoreGive(s_status_mutex);
        return;
    }
    if (strcmp(kind, "rtcp") != 0) {
        return;
    }

    /* 跳过 <id>,<len>, 取剩余部分作为数据 */
    for (int i = 0; i < 2; i++) {
        const char *c = strchr(p, ',');
        if (c == NULL) {
            return;
        }
        p = c + 1;
    }
    while (*p == ' ') {
        p++;
    }
    char data[160];
    size_t len = 0;
    if (*p == '"') {
        p++;
        while (*p && *p != '"' && len + 1 < sizeof(data)) {
            data[len++] = *p++;
        }
    } else {
        while (*p && *p != '\r' && *p != '\n' && len + 1 < sizeof(data)) {
            data[len++] = *p++;
        }
    }
    while (len > 0 && (data[len - 1] == ' ' || data[len - 1] == '\r')) {
        len--;
    }
    data[len] = '\0';
    if (len == 0) {
        return;
    }

    xSemaphoreTake(s_status_mutex, portMAX_DELAY);
    s_status.downlink_count++;
    s_status.rx_bytes += (uint32_t)len;
    snprintf(s_status.last_downlink, sizeof(s_status.last_downlink), "%s", data);
    xSemaphoreGive(s_status_mutex);
    ESP_LOGD(TAG, "socket rx %u bytes", (unsigned)len);

    /* 交给上层协议（MQTT）解析 */
    if (s_rx_cb != NULL) {
        s_rx_cb(data, len);
    }
}

/* 处理主动上报行；返回 true 表示该行已消费（不再计入命令响应） */
static bool urc_consume(const char *line)
{
    if (strncmp(line, "+MIPURC:", 8) == 0) {
        note_mipurc(line);
        return true;
    }
    if (strncmp(line, "+CEREG:", 7) == 0 ||
        strncmp(line, "+CGREG:", 7) == 0 ||
        strncmp(line, "+CREG:", 6) == 0) {
        note_reg(line);
        return false;
    }
    if (strncmp(line, "+MIPCLOSE:", 10) == 0) {
        xSemaphoreTake(s_status_mutex, portMAX_DELAY);
        s_status.socket_ok = false;
        xSemaphoreGive(s_status_mutex);
        return false;
    }
    return false;
}

/* ---------------- AT 收发 ---------------- */

/* 等待响应行；调用者需持有 s_at_mutex */
static bool at_wait_locked(const at_opts_t *opts, at_resp_t *resp)
{
    uint32_t timeout = opts ? opts->timeout_ms : 1000;
    uint32_t start = ml307_now_ms();
    char line[ML307_LINE_MAX];

    while (ml307_now_ms() - start < timeout) {
        uint32_t remain = timeout - (ml307_now_ms() - start);
        if (!at_next_line(line, sizeof(line), remain)) {
            break;
        }
        if (line[0] == '\0') {
            continue;
        }
        if (urc_consume(line)) {
            continue;
        }
        if (strcmp(line, "OK") == 0) {
            resp->ok = true;
            return true;
        }
        if (strcmp(line, "ERROR") == 0 ||
            strncmp(line, "+CME ERROR", 10) == 0 ||
            strncmp(line, "+CMS ERROR", 10) == 0) {
            resp->error = true;
            resp_add(resp, line);
            return false;
        }
        if (opts && opts->stop_on_prompt && strchr(line, '>') != NULL) {
            resp->prompt = true;
            return true;
        }
        resp_add(resp, line);
        if (strncmp(line, "+CSQ:", 5) == 0) {
            note_csq(line);
        } else if (strncmp(line, "+CCLK:", 6) == 0) {
            note_cclk(line);
        }
        if (opts && opts->keywords != NULL) {
            for (size_t i = 0; i < opts->keyword_count; i++) {
                if (strstr(line, opts->keywords[i]) != NULL) {
                    resp->matched = true;
                    if (opts->stop_on_keyword) {
                        return true;
                    }
                    break;
                }
            }
        }
    }
    return false;
}

/* 发送一条 AT 指令并等待响应；调用者需持有 s_at_mutex */
static bool at_run_locked(const char *cmd, const at_opts_t *opts, at_resp_t *resp)
{
    memset(resp, 0, sizeof(*resp));
    if (s_uart < 0) {
        return false;
    }
    uart_flush_input(s_uart);
    s_rx_len = 0;

    char cmdline[ML307_LINE_MAX];
    snprintf(cmdline, sizeof(cmdline), "%s", cmd);
    uart_write_bytes(s_uart, cmdline, strlen(cmdline));
    uart_write_bytes(s_uart, "\r", 1);

    /* 丢弃回显行 */
    uint32_t echo_start = ml307_now_ms();
    char line[ML307_LINE_MAX];
    while (ml307_now_ms() - echo_start < 200) {
        if (!at_next_line(line, sizeof(line), 200)) {
            break;
        }
        if (line[0] == '\0') {
            continue;
        }
        if (strcmp(line, cmdline) == 0) {
            break;
        }
        if (urc_consume(line)) {
            continue;
        }
        /* 非回显内容：按响应处理 */
        resp_add(resp, line);
        if (strcmp(line, "OK") == 0) {
            resp->ok = true;
            return true;
        }
        if (strcmp(line, "ERROR") == 0 || strncmp(line, "+CME ERROR", 10) == 0) {
            resp->error = true;
            return false;
        }
        if (opts && opts->stop_on_prompt && strchr(line, '>') != NULL) {
            resp->prompt = true;
            return true;
        }
        break;
    }
    return at_wait_locked(opts, resp);
}

static bool at_run(const char *cmd, const at_opts_t *opts, at_resp_t *resp)
{
    uint32_t timeout = (opts ? opts->timeout_ms : 1000) + 1500;
    if (xSemaphoreTake(s_at_mutex, pdMS_TO_TICKS(timeout)) != pdTRUE) {
        memset(resp, 0, sizeof(*resp));
        return false;
    }
    bool ok = at_run_locked(cmd, opts, resp);
    xSemaphoreGive(s_at_mutex);
    return ok;
}

/* 排空串口里缓存的主动上报（任务空闲时调用），避免 RX FIFO 溢出 */
static void at_drain(void)
{
    if (s_at_mutex == NULL ||
        xSemaphoreTake(s_at_mutex, 0) != pdTRUE) {
        return;
    }
    char line[ML307_LINE_MAX];
    int guard = 0;
    while (guard++ < 32 && at_next_line(line, sizeof(line), 0)) {
        if (line[0] != '\0') {
            urc_consume(line);
        }
    }
    xSemaphoreGive(s_at_mutex);
}

/* ---------------- EN / 电源控制 ---------------- */

static void en_drive(bool active)
{
    if (s_cfg.en_gpio < 0) {
        return;
    }
    int level;
    if (s_cfg.en_active_low) {
        level = active ? 0 : 1;
    } else {
        level = active ? 1 : 0;
    }
    gpio_set_level(s_cfg.en_gpio, level);
}

static void ml307_power_on(void)
{
    if (s_cfg.en_gpio < 0) {
        ESP_LOGW(TAG, "EN GPIO 未配置，假设模块由硬件常供电");
        return;
    }
    if (s_cfg.en_pulse_ms == 0) {
        en_drive(true); /* 电平使能：保持有效 */
        ESP_LOGI(TAG, "EN level enable (gpio=%d active_%s)", s_cfg.en_gpio,
                 s_cfg.en_active_low ? "low" : "high");
        return;
    }
    ESP_LOGI(TAG, "EN pulse %u ms (gpio=%d)", (unsigned)s_cfg.en_pulse_ms,
             s_cfg.en_gpio);
    en_drive(true);
    vTaskDelay(pdMS_TO_TICKS(s_cfg.en_pulse_ms));
    en_drive(false);
}

static void ml307_power_off(void)
{
    if (s_cfg.en_gpio < 0) {
        return;
    }
    if (s_cfg.en_pulse_ms == 0) {
        en_drive(false); /* 电平使能：断开供电 */
        return;
    }
    ESP_LOGI(TAG, "EN power-off pulse");
    en_drive(true);
    vTaskDelay(pdMS_TO_TICKS(ML307_POWER_OFF_MS));
    en_drive(false);
}

static void ml307_power_cycle(void)
{
    ml307_power_off();
    vTaskDelay(pdMS_TO_TICKS(ML307_POWER_GAP_MS));
    ml307_power_on();
    xSemaphoreTake(s_status_mutex, portMAX_DELAY);
    s_status.at_ok = false;
    s_status.sim_ok = false;
    s_status.net_ok = false;
    s_status.socket_ok = false;
    s_status.ip[0] = '\0';
    xSemaphoreGive(s_status_mutex);
}

/* ---------------- 入网流程 ---------------- */

static bool wait_at_ready(uint32_t timeout_ms)
{
    uint32_t start = ml307_now_ms();
    while (ml307_now_ms() - start < timeout_ms) {
        at_opts_t opts = {.timeout_ms = 1500};
        at_resp_t r;
        if (at_run("AT", &opts, &r) && r.ok) {
            xSemaphoreTake(s_status_mutex, portMAX_DELAY);
            s_status.at_ok = true;
            xSemaphoreGive(s_status_mutex);
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    set_error("AT 无响应");
    return false;
}

static void note_quoted_after(const char *text, const char *key, char *out, size_t cap)
{
    const char *p = strstr(text, key);
    if (p == NULL) {
        return;
    }
    p += strlen(key);
    while (*p == ' ') {
        p++;
    }
    if (*p == '"') {
        p++;
    }
    size_t n = 0;
    while (*p && *p != '"' && *p != '\r' && *p != '\n' && n + 1 < cap) {
        out[n++] = *p++;
    }
    out[n] = '\0';
}

static bool init_module(void)
{
    at_opts_t opts = {.timeout_ms = 3000};
    at_resp_t r;

    at_run("ATE0", &opts, &r); /* 关回显 */

    char imei[24] = {0};
    if (at_run("AT+CGSN", &opts, &r)) {
        for (size_t i = 0; i < r.len && i + 1 < sizeof(imei); i++) {
            char c = r.text[i];
            if (c >= '0' && c <= '9') {
                size_t n = strlen(imei);
                if (n + 1 < sizeof(imei)) {
                    imei[n] = c;
                    imei[n + 1] = '\0';
                }
            }
        }
    }
    if (strlen(imei) >= 14) {
        xSemaphoreTake(s_status_mutex, portMAX_DELAY);
        snprintf(s_status.imei, sizeof(s_status.imei), "%s", imei);
        xSemaphoreGive(s_status_mutex);
        ESP_LOGI(TAG, "IMEI: %s", imei);
    }

    char iccid[24] = {0};
    if (at_run("AT+ICCID", &opts, &r)) {
        note_quoted_after(r.text, "+ICCID:", iccid, sizeof(iccid));
    }
    if (strlen(iccid) < 10 && at_run("AT+CCID", &opts, &r)) {
        note_quoted_after(r.text, "+CCID:", iccid, sizeof(iccid));
    }
    if (strlen(iccid) >= 10) {
        xSemaphoreTake(s_status_mutex, portMAX_DELAY);
        snprintf(s_status.iccid, sizeof(s_status.iccid), "%s", iccid);
        xSemaphoreGive(s_status_mutex);
        ESP_LOGI(TAG, "ICCID: %s", iccid);
    }

    if (s_cfg.apn[0] != '\0') {
        char cmd[96];
        snprintf(cmd, sizeof(cmd), "AT+CGDCONT=1,\"IP\",\"%s\"", s_cfg.apn);
        at_run(cmd, &opts, &r);
        at_run("AT+MIPCALL=1", &opts, &r);
    }
    return true;
}

static bool wait_sim(uint32_t timeout_ms)
{
    uint32_t start = ml307_now_ms();
    while (ml307_now_ms() - start < timeout_ms) {
        at_opts_t opts = {.timeout_ms = 3000};
        at_resp_t r;
        if (at_run("AT+CPIN?", &opts, &r)) {
            if (strstr(r.text, "READY") != NULL) {
                xSemaphoreTake(s_status_mutex, portMAX_DELAY);
                s_status.sim_ok = true;
                xSemaphoreGive(s_status_mutex);
                return true;
            }
        }
        if (strstr(r.text, "SIM PIN") != NULL) {
            set_error("SIM 需要 PIN");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    set_error("SIM 未就绪");
    return false;
}

static bool wait_network(uint32_t timeout_ms)
{
    at_opts_t opts = {.timeout_ms = 3000};
    at_resp_t r;

    at_run("AT+CGATT=1", &opts, &r);

    uint32_t start = ml307_now_ms();
    while (ml307_now_ms() - start < timeout_ms) {
        if (at_run("AT+CEREG?", &opts, &r)) {
            int stat = parse_reg_stat(r.text);
            if (stat >= 0) {
                xSemaphoreTake(s_status_mutex, portMAX_DELAY);
                s_status.reg_stat = stat;
                s_status.net_ok = (stat == 1 || stat == 5);
                xSemaphoreGive(s_status_mutex);
            }
            if (stat == 1 || stat == 5) {
                break;
            }
            if (stat == 2) {
                ESP_LOGI(TAG, "正在搜网…");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
    if (!s_status.net_ok) {
        set_error("LTE 未注册");
        return false;
    }

    if (at_run("AT+CSQ", &opts, &r)) {
        note_csq(r.text);
    }
    if (at_run("AT+CCLK?", &opts, &r)) {
        note_cclk(r.text);
    }
    ESP_LOGI(TAG, "network registered (CSQ=%d, RSSI=%d dBm)",
             s_status.csq, s_status.rssi_dbm);
    return true;
}

static bool socket_open(void)
{
    if (s_cfg.host[0] == '\0' || s_cfg.port == 0) {
        ESP_LOGW(TAG, "未配置上报服务器，跳过 socket 建立");
        return false;
    }
    at_opts_t opts = {.timeout_ms = 8000};
    at_resp_t r;
    at_run("AT+MIPCLOSE=0", &opts, &r); /* 清理旧连接，失败忽略 */

    char cmd[160];
    snprintf(cmd, sizeof(cmd), "AT+MIPOPEN=0,\"TCP\",\"%s\",%u",
             s_cfg.host, (unsigned)s_cfg.port);
    if (!at_run(cmd, &opts, &r)) {
        ESP_LOGW(TAG, "MIPOPEN 失败: %s", r.text[0] ? r.text : "(超时)");
        set_error("socket 建立失败");
        return false;
    }
    const char *p = strstr(r.text, "+MIPOPEN:");
    if (p != NULL) {
        int vals[4] = {0};
        int count = 0;
        p = strchr(p, ':');
        if (p != NULL) {
            p++;
            while (*p && count < 4) {
                char *end = NULL;
                long v = strtol(p, &end, 10);
                if (end == p) {
                    break;
                }
                vals[count++] = (int)v;
                p = end;
                while (*p == ' ') {
                    p++;
                }
                if (*p != ',') {
                    break;
                }
                p++;
            }
        }
        if (count >= 2 && vals[1] != 0) {
            ESP_LOGW(TAG, "MIPOPEN 返回错误码 %d", vals[1]);
            set_error("socket 被拒绝");
            return false;
        }
    }
    xSemaphoreTake(s_status_mutex, portMAX_DELAY);
    s_status.socket_ok = true;
    s_status.ip[0] = '\0';
    xSemaphoreGive(s_status_mutex);
    ESP_LOGI(TAG, "TCP 已连接 %s:%u", s_cfg.host, (unsigned)s_cfg.port);
    return true;
}

static void socket_close(void)
{
    at_opts_t opts = {.timeout_ms = 3000};
    at_resp_t r;
    at_run("AT+MIPCLOSE=0", &opts, &r);
    xSemaphoreTake(s_status_mutex, portMAX_DELAY);
    s_status.socket_ok = false;
    xSemaphoreGive(s_status_mutex);
}

/* ---------------- TCP 透传（MQTT 报文走这里收发） ---------------- */

/* 把一段字节写进当前 TCP 连接（AT+MIPSEND + '>' + 数据） */
static bool socket_send_bytes(const void *data, size_t len)
{
    if (len == 0) {
        return true;
    }
    if (s_uart < 0) {
        return false;
    }
    xSemaphoreTake(s_status_mutex, portMAX_DELAY);
    bool socket_ok = s_status.socket_ok;
    xSemaphoreGive(s_status_mutex);
    if (!socket_ok) {
        set_error("socket 未连接");
        return false;
    }

    if (xSemaphoreTake(s_at_mutex, pdMS_TO_TICKS(10000)) != pdTRUE) {
        set_error("AT 忙");
        return false;
    }

    at_opts_t opts = {.timeout_ms = 4000, .stop_on_prompt = true};
    at_resp_t r;
    bool ok = false;
    char cmd[48];
    snprintf(cmd, sizeof(cmd), "AT+MIPSEND=0,%u", (unsigned)len);
    if (at_run_locked(cmd, &opts, &r) && r.prompt) {
        uart_write_bytes(s_uart, data, len);
        at_opts_t wait_opts = {.timeout_ms = 8000};
        at_resp_t r2;
        memset(&r2, 0, sizeof(r2));
        ok = at_wait_locked(&wait_opts, &r2);
        if (!ok) {
            ESP_LOGW(TAG, "socket send 无 OK: %s", r2.text[0] ? r2.text : "(超时)");
        }
    } else {
        ESP_LOGW(TAG, "MIPSEND 未获得 '>' 提示: %s", r.text[0] ? r.text : "(超时)");
        /* 模块可能已断开，交给状态机重连 */
    }
    xSemaphoreGive(s_at_mutex);

    if (ok) {
        xSemaphoreTake(s_status_mutex, portMAX_DELAY);
        s_status.tx_bytes += (uint32_t)len;
        xSemaphoreGive(s_status_mutex);
    }
    return ok;
}

static void refresh_status(void)
{
    at_opts_t opts = {.timeout_ms = 3000};
    at_resp_t r;
    if (at_run("AT+CSQ", &opts, &r)) {
        note_csq(r.text);
    }
    if (at_run("AT+CEREG?", &opts, &r)) {
        note_reg(r.text);
    }
    if (at_run("AT+CCLK?", &opts, &r)) {
        note_cclk(r.text);
    }
}

/* 在线循环：排空主动上报、定期刷新信号/注册/时间；返回后由状态机决定下一步 */
static void online_loop(void)
{
    uint32_t last_stat = ml307_now_ms();
    bool has_socket = s_status.socket_ok;

    for (;;) {
        if (s_req_power_cycle) {
            s_req_power_cycle = false;
            s_req_reconnect = false;
            set_state(ML307_STATE_RETRY);
            return;
        }
        if (s_req_reconnect) {
            s_req_reconnect = false;
            if (has_socket) {
                socket_close();
            }
            set_state(ML307_STATE_SOCKET);
            return;
        }

        at_drain();

        uint32_t now = ml307_now_ms();
        if (has_socket && !s_status.socket_ok) {
            ESP_LOGW(TAG, "socket 已断开，准备重连");
            set_state(ML307_STATE_SOCKET);
            return;
        }
        if (!s_status.net_ok) {
            ESP_LOGW(TAG, "网络掉线，重新入网");
            set_error("网络掉线");
            set_state(ML307_STATE_NETWORK);
            return;
        }

        if (now - last_stat >= ML307_STAT_MS) {
            refresh_status();
            last_stat = now;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/* ---------------- 主任务 ---------------- */

static void ml307_task(void *arg)
{
    (void)arg;

    ml307_uart_init();
    set_state(ML307_STATE_POWER_ON);

    for (;;) {
        switch (s_status.state) {
        case ML307_STATE_POWER_ON:
            ml307_power_on();
            vTaskDelay(pdMS_TO_TICKS(ML307_BOOT_MS));
            set_state(ML307_STATE_WAIT_AT);
            break;

        case ML307_STATE_WAIT_AT:
            if (wait_at_ready(45000)) {
                set_state(ML307_STATE_INIT);
            } else {
                set_state(ML307_STATE_RETRY);
            }
            break;

        case ML307_STATE_INIT:
            init_module();
            set_state(ML307_STATE_SIM);
            break;

        case ML307_STATE_SIM:
            if (wait_sim(30000)) {
                set_state(ML307_STATE_NETWORK);
            } else {
                set_state(ML307_STATE_RETRY);
            }
            break;

        case ML307_STATE_NETWORK:
            if (wait_network(120000)) {
                set_state(ML307_STATE_SOCKET);
            } else {
                set_state(ML307_STATE_RETRY);
            }
            break;

        case ML307_STATE_SOCKET:
            if (s_cfg.host[0] == '\0' || s_cfg.port == 0) {
                /* 未配置服务器：只保持入网，等待 Web 配置后重启生效 */
                set_state(ML307_STATE_ONLINE);
            } else if (socket_open()) {
                set_state(ML307_STATE_ONLINE);
            } else {
                set_state(ML307_STATE_RETRY);
            }
            break;

        case ML307_STATE_ONLINE:
            online_loop();
            break;

        case ML307_STATE_RETRY:
        default:
            ESP_LOGW(TAG, "重试：重启 4G 模块（%s）", s_status.last_error);
            vTaskDelay(pdMS_TO_TICKS(ML307_RETRY_MS));
            ml307_power_cycle();
            vTaskDelay(pdMS_TO_TICKS(ML307_BOOT_MS));
            set_state(ML307_STATE_WAIT_AT);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ---------------- 对外接口 ---------------- */

void ml307_4g_start(const ml307_4g_config_t *cfg)
{
    s_cfg = *cfg;
    s_at_mutex = xSemaphoreCreateMutex();
    s_status_mutex = xSemaphoreCreateMutex();
    memset(&s_status, 0, sizeof(s_status));
    s_status.enabled = cfg->enable;
    s_status.state = ML307_STATE_DISABLED;
    s_status.state_name = STATE_NAMES[0];
    s_status.csq = -1;
    s_status.rssi_dbm = -1;
    s_status.reg_stat = -1;
    snprintf(s_status.last_error, sizeof(s_status.last_error), "%s", "未启动");
    s_started = true;

    if (s_cfg.en_gpio >= 0) {
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << s_cfg.en_gpio,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io);
        en_drive(false); /* 默认释放：脉冲型保持高，电平型保持关断 */
    }

    if (!cfg->enable) {
        ESP_LOGW(TAG, "4G 模块已在配置中关闭");
        return;
    }

    ESP_LOGI(TAG, "4G 启动: en=%d tx=%d rx=%d uart=%d broker=%s:%u",
             s_cfg.en_gpio, s_cfg.tx_gpio, s_cfg.rx_gpio, s_cfg.uart_num,
             s_cfg.host[0] ? s_cfg.host : "(未配置)", (unsigned)s_cfg.port);
    xTaskCreate(ml307_task, "ml307_4g", ML307_TASK_STACK, NULL,
                ML307_TASK_PRIO, NULL);
}

void ml307_4g_get_status(ml307_4g_status_t *out)
{
    if (out == NULL) {
        return;
    }
    if (s_status_mutex == NULL) {
        memset(out, 0, sizeof(*out));
        out->state_name = STATE_NAMES[0];
        out->csq = -1;
        out->rssi_dbm = -1;
        out->reg_stat = -1;
        return;
    }
    xSemaphoreTake(s_status_mutex, portMAX_DELAY);
    *out = s_status;
    xSemaphoreGive(s_status_mutex);
}

void ml307_4g_request_reconnect(void)
{
    s_req_reconnect = true;
}

void ml307_4g_request_power_cycle(void)
{
    s_req_power_cycle = true;
}

bool ml307_4g_socket_ready(void)
{
    if (s_status_mutex == NULL) {
        return false;
    }
    xSemaphoreTake(s_status_mutex, portMAX_DELAY);
    bool ok = s_status.socket_ok;
    xSemaphoreGive(s_status_mutex);
    return ok;
}

int ml307_4g_socket_send(const void *data, size_t len)
{
    if (data == NULL || len == 0) {
        return -1;
    }
    return socket_send_bytes(data, len) ? 0 : -1;
}

void ml307_4g_set_rx_callback(void (*cb)(const void *data, size_t len))
{
    s_rx_cb = cb;
}

bool ml307_4g_get_epoch(uint32_t *epoch)
{
    if (s_status_mutex == NULL) {
        return false;
    }
    xSemaphoreTake(s_status_mutex, portMAX_DELAY);
    bool ok = s_status.time_valid;
    if (ok && epoch != NULL) {
        *epoch = s_status.unix_time;
    }
    xSemaphoreGive(s_status_mutex);
    return ok;
}

int ml307_4g_at_command(const char *cmd, char *resp, size_t resp_cap,
                        uint32_t timeout_ms)
{
    if (!s_started || s_uart < 0 || cmd == NULL || resp == NULL || resp_cap == 0) {
        return -1;
    }
    if (timeout_ms < 200) {
        timeout_ms = 200;
    } else if (timeout_ms > 15000) {
        timeout_ms = 15000;
    }
    if (xSemaphoreTake(s_at_mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        return -2;
    }
    at_opts_t opts = {.timeout_ms = timeout_ms};
    at_resp_t r;
    bool ok = at_run_locked(cmd, &opts, &r);
    if (r.text[0] != '\0') {
        snprintf(resp, resp_cap, "%s", r.text);
    } else if (r.error) {
        snprintf(resp, resp_cap, "ERROR");
    } else {
        snprintf(resp, resp_cap, "(no response)");
    }
    xSemaphoreGive(s_at_mutex);

    xSemaphoreTake(s_status_mutex, portMAX_DELAY);
    s_status.at_cmd_count++;
    xSemaphoreGive(s_status_mutex);
    return ok ? 0 : -1;
}
