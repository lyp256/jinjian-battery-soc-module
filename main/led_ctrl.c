#include "led_ctrl.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/cdefs.h>

#include "app_config.h"
#include "bms_interface.h"
#include "driver/gpio.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "ota_update.h"
#include "sdkconfig.h"

#define LED_TAG "led"

/* RMT 时钟 10MHz，1 tick = 0.1us，正好匹配 WS2812 的 0.3us/0.9us 时序 */
#define LED_RESOLUTION_HZ      (10 * 1000 * 1000)
#define LED_TASK_PERIOD_MS     50
#define LED_TASK_STACK         4096
#define LED_TASK_PRIO          4
#define LED_MAX_COUNT          8

#define LED_EVENT_QUEUE_MAX     16
#define LED_EVENT_FLASH_TICKS   3  /* WS2812 事件色闪烁 150ms */
#define LED_SUCCESS_BLINK_TICKS 3  /* 单色 LED 成功闪烁 150ms，与事件色一致 */
#define LED_FAULT_ON_TICKS      5  /* 故障色点亮 250ms */
#define LED_FAULT_OFF_TICKS     5  /* 故障色熄灭 250ms */

#define LED_FAULT_STALE         (1u << 0) /* BMS 轮询失败 */
#define LED_FAULT_ERROR         (1u << 1) /* BMS 驱动不可用 */

typedef enum {
    LED_STATE_OFF = 0, /* 用户在 Web 页面关闭了 LED */
    LED_STATE_BOOT,    /* 启动中 / 尚未收到第一包 BMS 数据 */
    LED_STATE_OK,      /* 正常运行且 BMS 数据新鲜 */
    LED_STATE_STALE,   /* 运行中，但最近一次 BMS 轮询失败 */
    LED_STATE_ERROR,   /* BMS 驱动不可用 */
} led_state_t;

typedef enum {
    LED_PHASE_IDLE = 0,
    LED_PHASE_EVENT_FLASH,  /* 正在显示 WS2812 事件色单闪 */
    LED_PHASE_SUCCESS_BLINK,/* 事件成功，单色 LED 闪烁一次 */
} led_phase_t;

typedef struct {
    led_event_t ev;
    bool success;
} led_event_rec_t;

/* ---------------- WS2812 RMT 编码器（GRB888，参考 ESP-IDF 示例时序） ---------------- */

typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *bytes_encoder;
    rmt_encoder_t *copy_encoder;
    int state;
    rmt_symbol_word_t reset_code;
} led_strip_encoder_t;

RMT_ENCODER_FUNC_ATTR
static size_t led_strip_encode(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
                               const void *primary_data, size_t data_size,
                               rmt_encode_state_t *ret_state)
{
    led_strip_encoder_t *led_encoder = __containerof(encoder, led_strip_encoder_t, base);
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;

    switch (led_encoder->state) {
    case 0:
        encoded_symbols += led_encoder->bytes_encoder->encode(
            led_encoder->bytes_encoder, channel, primary_data, data_size, &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            led_encoder->state = 1;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
        /* fall through */
    case 1:
        encoded_symbols += led_encoder->copy_encoder->encode(
            led_encoder->copy_encoder, channel, &led_encoder->reset_code,
            sizeof(led_encoder->reset_code), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            state |= RMT_ENCODING_COMPLETE;
            led_encoder->state = RMT_ENCODING_RESET;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
        break;
    }
out:
    *ret_state = state;
    return encoded_symbols;
}

static esp_err_t led_strip_del(rmt_encoder_t *encoder)
{
    led_strip_encoder_t *led_encoder = __containerof(encoder, led_strip_encoder_t, base);
    rmt_del_encoder(led_encoder->bytes_encoder);
    rmt_del_encoder(led_encoder->copy_encoder);
    free(led_encoder);
    return ESP_OK;
}

RMT_ENCODER_FUNC_ATTR
static esp_err_t led_strip_reset(rmt_encoder_t *encoder)
{
    led_strip_encoder_t *led_encoder = __containerof(encoder, led_strip_encoder_t, base);
    rmt_encoder_reset(led_encoder->bytes_encoder);
    rmt_encoder_reset(led_encoder->copy_encoder);
    led_encoder->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

static esp_err_t led_strip_new_encoder(rmt_encoder_handle_t *ret_encoder)
{
    led_strip_encoder_t *led_encoder = rmt_alloc_encoder_mem(sizeof(led_strip_encoder_t));
    if (led_encoder == NULL) {
        return ESP_ERR_NO_MEM;
    }
    led_encoder->base.encode = led_strip_encode;
    led_encoder->base.del = led_strip_del;
    led_encoder->base.reset = led_strip_reset;
    led_encoder->state = RMT_ENCODING_RESET;

    rmt_bytes_encoder_config_t bytes_cfg = {
        .bit0 = {
            .level0 = 1,
            .duration0 = 3, /* T0H = 0.3us */
            .level1 = 0,
            .duration1 = 9, /* T0L = 0.9us */
        },
        .bit1 = {
            .level0 = 1,
            .duration0 = 9, /* T1H = 0.9us */
            .level1 = 0,
            .duration1 = 3, /* T1L = 0.3us */
        },
        .flags.msb_first = 1, /* WS2812 字节序：G7..G0 R7..R0 B7..B0 */
    };
    esp_err_t err = rmt_new_bytes_encoder(&bytes_cfg, &led_encoder->bytes_encoder);
    if (err != ESP_OK) {
        free(led_encoder);
        return err;
    }
    rmt_copy_encoder_config_t copy_cfg = {};
    err = rmt_new_copy_encoder(&copy_cfg, &led_encoder->copy_encoder);
    if (err != ESP_OK) {
        rmt_del_encoder(led_encoder->bytes_encoder);
        free(led_encoder);
        return err;
    }
    /* 复位码 50us */
    led_encoder->reset_code = (rmt_symbol_word_t){
        .level0 = 0,
        .duration0 = 250,
        .level1 = 0,
        .duration1 = 250,
    };
    *ret_encoder = &led_encoder->base;
    return ESP_OK;
}

/* ---------------- 驱动状态 ---------------- */

static rmt_channel_handle_t s_ws2812_chan;
static rmt_encoder_handle_t s_ws2812_enc;
static uint8_t s_last_grb[3] = {0xFF, 0xFF, 0xFF}; /* 强制首次刷新 */
static int s_led_count = 1;
static bool s_initialized;

static SemaphoreHandle_t s_ev_mutex;
static led_event_rec_t s_events[LED_EVENT_QUEUE_MAX];
static int s_ev_head;
static int s_ev_tail;
static int s_ev_count;
static bool s_boot_hold;
static bool s_boot_triggered;
static bool s_power_save;

/* ---------------- WS2812 输出 ---------------- */

static void led_ws2812_write(uint8_t g, uint8_t r, uint8_t b)
{
    if (s_ws2812_chan == NULL) {
        return;
    }
    if (s_last_grb[0] == g && s_last_grb[1] == r && s_last_grb[2] == b) {
        return;
    }
    s_last_grb[0] = g;
    s_last_grb[1] = r;
    s_last_grb[2] = b;

    uint8_t buf[LED_MAX_COUNT * 3];
    int count = s_led_count > LED_MAX_COUNT ? LED_MAX_COUNT : s_led_count;
    for (int i = 0; i < count; i++) {
        buf[i * 3 + 0] = g;
        buf[i * 3 + 1] = r;
        buf[i * 3 + 2] = b;
    }
    rmt_transmit_config_t tx_cfg = {
        .loop_count = 0,
    };
    if (rmt_transmit(s_ws2812_chan, s_ws2812_enc, buf, count * 3, &tx_cfg) == ESP_OK) {
        rmt_tx_wait_all_done(s_ws2812_chan, pdMS_TO_TICKS(100));
    }
}

static esp_err_t led_ws2812_init(void)
{
    int gpio = CONFIG_JINJIAN_WS2812_GPIO;
    if (gpio < 0) {
        return ESP_OK;
    }
    s_led_count = CONFIG_JINJIAN_WS2812_LED_COUNT;
    int symbols = s_led_count * 48 + 4;
    symbols = (symbols + 1) & ~1; /* 必须是偶数且 >= 48 */

    rmt_tx_channel_config_t chan_cfg = {
        .gpio_num = gpio,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = LED_RESOLUTION_HZ,
        .mem_block_symbols = symbols,
        .trans_queue_depth = 2,
    };
    esp_err_t err = rmt_new_tx_channel(&chan_cfg, &s_ws2812_chan);
    if (err != ESP_OK) {
        ESP_LOGE(LED_TAG, "rmt_new_tx_channel failed: %s", esp_err_to_name(err));
        return err;
    }
    err = led_strip_new_encoder(&s_ws2812_enc);
    if (err != ESP_OK) {
        ESP_LOGE(LED_TAG, "led encoder failed: %s", esp_err_to_name(err));
        rmt_del_channel(s_ws2812_chan);
        s_ws2812_chan = NULL;
        return err;
    }
    err = rmt_enable(s_ws2812_chan);
    if (err != ESP_OK) {
        ESP_LOGE(LED_TAG, "rmt_enable failed: %s", esp_err_to_name(err));
        rmt_del_encoder(s_ws2812_enc);
        rmt_del_channel(s_ws2812_chan);
        s_ws2812_enc = NULL;
        s_ws2812_chan = NULL;
        return err;
    }
    return ESP_OK;
}

/* ---------------- GPIO LED ---------------- */

static void led_gpio_init_one(int gpio)
{
    if (gpio < 0) {
        return;
    }
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << gpio,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(gpio, 0);
}

static int status_led_level(bool on)
{
#ifdef CONFIG_JINJIAN_STATUS_LED_ACTIVE_LOW
    return on ? 0 : 1;
#else
    return on ? 1 : 0;
#endif
}

static int pwr_led_level(bool on)
{
#ifdef CONFIG_JINJIAN_PWR_LED_ACTIVE_LOW
    return on ? 0 : 1;
#else
    return on ? 1 : 0;
#endif
}

static void render_power_led(bool enabled)
{
    int gpio = CONFIG_JINJIAN_PWR_LED_GPIO;
    if (gpio < 0) {
        return;
    }
    gpio_set_level(gpio, pwr_led_level(enabled));
}

/* ---------------- 事件队列 ---------------- */

static void led_event_push(led_event_t ev, bool success)
{
    if (s_ev_mutex == NULL) {
        return;
    }
    xSemaphoreTake(s_ev_mutex, portMAX_DELAY);
    if (s_ev_count >= LED_EVENT_QUEUE_MAX) {
        /* 队列满：丢最旧、保留最新 */
        s_ev_head = (s_ev_head + 1) % LED_EVENT_QUEUE_MAX;
        s_ev_count--;
    }
    s_events[s_ev_tail].ev = ev;
    s_events[s_ev_tail].success = success;
    s_ev_tail = (s_ev_tail + 1) % LED_EVENT_QUEUE_MAX;
    s_ev_count++;
    xSemaphoreGive(s_ev_mutex);
}

static bool led_event_pop(led_event_rec_t *out)
{
    bool got = false;
    if (s_ev_mutex == NULL) {
        return false;
    }
    xSemaphoreTake(s_ev_mutex, portMAX_DELAY);
    if (s_ev_count > 0) {
        *out = s_events[s_ev_head];
        s_ev_head = (s_ev_head + 1) % LED_EVENT_QUEUE_MAX;
        s_ev_count--;
        got = true;
    }
    xSemaphoreGive(s_ev_mutex);
    return got;
}

static void led_event_flush(void)
{
    led_event_rec_t tmp;
    while (led_event_pop(&tmp)) {
    }
}

void led_ctrl_notify_event(led_event_t ev, bool success)
{
    led_event_push(ev, success);
}

void led_ctrl_boot_hold(bool holding, bool triggered)
{
    if (s_ev_mutex == NULL) {
        return;
    }
    xSemaphoreTake(s_ev_mutex, portMAX_DELAY);
    s_boot_hold = holding;
    s_boot_triggered = triggered;
    xSemaphoreGive(s_ev_mutex);
}

void led_ctrl_power_save(bool on)
{
    s_power_save = on;
}

/* ---------------- 状态渲染 ---------------- */

static led_state_t compute_state(const app_config_t *cfg)
{
    if (!cfg->led_enable) {
        return LED_STATE_OFF;
    }
    if (strcmp(bms_manager_name(), "none") == 0) {
        return LED_STATE_ERROR;
    }
    bms_snapshot_t snap;
    if (!bms_manager_get_snapshot(&snap) || !snap.have_data) {
        return LED_STATE_BOOT;
    }
    return snap.fresh ? LED_STATE_OK : LED_STATE_STALE;
}

/* 基线：RGB 完全熄灭，只有单色 LED 表达状态 */
static void render_ws2812_off(void)
{
    led_ws2812_write(0, 0, 0);
}

/* 事件单闪：全亮事件色 */
static void render_ws2812_event(led_event_t ev)
{
    uint8_t g = 0, r = 0, b = 0;
    if (ev == LED_EVENT_485) {
        b = 0xFF; /* 蓝色：收到 485 查询 */
    } else if (ev == LED_EVENT_BMS) {
        g = 0xFF; /* 绿色：读取 BMS */
    } else if (ev == LED_EVENT_LTE) {
        g = 0xFF; /* 青色：4G 上报 */
        b = 0xFF;
    }
    led_ws2812_write(g, r, b);
}

/* 故障：RGB 用故障色闪烁，多个故障时轮流切换颜色；单色 LED 节奏与 RGB 一致 */
static bool fault_phase_on(uint32_t faults, uint32_t tick, uint8_t (*color)[3])
{
    uint8_t colors[2][3] = {0};
    int n = 0;
    if (faults & LED_FAULT_ERROR) {
        colors[n][0] = 0x00;
        colors[n][1] = 0xFF;
        colors[n][2] = 0x00; /* 红：驱动不可用 */
        n++;
    }
    if (faults & LED_FAULT_STALE) {
        colors[n][0] = 0x80;
        colors[n][1] = 0xFF;
        colors[n][2] = 0x00; /* 橙：BMS 轮询失败 */
        n++;
    }
    if (n == 0) {
        return false;
    }
    uint32_t period = LED_FAULT_ON_TICKS + LED_FAULT_OFF_TICKS;
    uint32_t p = tick % (period * (uint32_t)n);
    bool on = (p % period) < LED_FAULT_ON_TICKS;
    uint32_t slot = p / period;
    if (slot >= (uint32_t)n) {
        slot = 0;
    }
    if (on && color != NULL) {
        memcpy(*color, colors[slot], 3);
    }
    return on;
}

static void render_fault_rgb(uint32_t faults, uint32_t tick)
{
    uint8_t color[3] = {0, 0, 0};
    if (fault_phase_on(faults, tick, &color)) {
        led_ws2812_write(color[0], color[1], color[2]);
    } else {
        led_ws2812_write(0, 0, 0);
    }
}

/* 单色 LED 只负责“任务成功”提示：平时熄灭，成功时点亮一下。
 * 板上没有单色 LED（GPIO=-1）时，改由事件色补闪一次表达成功。 */
static void render_status_on(void)
{
    int gpio = CONFIG_JINJIAN_STATUS_LED_GPIO;
    if (gpio < 0) {
        return;
    }
    gpio_set_level(gpio, status_led_level(true));
}

static void render_status_off(void)
{
    int gpio = CONFIG_JINJIAN_STATUS_LED_GPIO;
    if (gpio < 0) {
        return;
    }
    gpio_set_level(gpio, status_led_level(false));
}

/* OTA：WS2812 常亮黄绿交替（仅 RGB） */
static void render_ota(uint32_t tick)
{
    bool yellow = (tick % 20) < 10;
    if (yellow) {
        led_ws2812_write(0xFF, 0xFF, 0x00); /* 黄色 */
    } else {
        led_ws2812_write(0xFF, 0x00, 0x00); /* 绿色 */
    }
}

/* BOOT 按住：紫色常亮；触发恢复出厂：红色常亮 */
static void render_boot_mode(bool triggered)
{
    if (triggered) {
        led_ws2812_write(0x00, 0xFF, 0x00); /* 红色 */
    } else {
        led_ws2812_write(0x00, 0xFF, 0xFF); /* 紫色（R+B） */
    }
}

static void render_all_off(void)
{
    led_ws2812_write(0, 0, 0);
    render_status_off();
    render_power_led(false);
}

/* ---------------- 状态灯任务 ---------------- */

static void led_task(void *arg)
{
    (void)arg;
    uint32_t tick = 0;
    led_phase_t phase = LED_PHASE_IDLE;
    led_event_rec_t cur = {LED_EVENT_485, false};
    uint32_t phase_tick = 0;

    for (;;) {
        app_config_t cfg;
        app_config_get(&cfg);
        render_power_led(cfg.led_enable);

        bool boot = false, boot_trig = false, ota = false;
        xSemaphoreTake(s_ev_mutex, portMAX_DELAY);
        boot = s_boot_hold;
        boot_trig = s_boot_triggered;
        xSemaphoreGive(s_ev_mutex);
        ota = ota_update_active();

        if (!cfg.led_enable || s_power_save) {
            render_all_off();
            led_event_flush();
            phase = LED_PHASE_IDLE;
        } else if (boot) {
            render_boot_mode(boot_trig);
            led_event_flush();
            phase = LED_PHASE_IDLE;
        } else if (ota) {
            render_ota(tick);
            led_event_flush();
            phase = LED_PHASE_IDLE;
        } else {
            led_state_t st = compute_state(&cfg);
            uint32_t faults = 0;
            if (st == LED_STATE_STALE) {
                faults |= LED_FAULT_STALE;
            } else if (st == LED_STATE_ERROR) {
                faults |= LED_FAULT_ERROR;
            }
            switch (phase) {
            case LED_PHASE_IDLE:
                if (faults != 0) {
                    render_fault_rgb(faults, tick);
                } else {
                    render_ws2812_off();
                }
                render_status_off();
                if (led_event_pop(&cur)) {
                    phase_tick = 0;
                    phase = LED_PHASE_EVENT_FLASH;
                }
                break;
            case LED_PHASE_EVENT_FLASH:
                render_ws2812_event(cur.ev);
                render_status_off();
                phase_tick++;
                if (phase_tick >= LED_EVENT_FLASH_TICKS) {
                    if (cur.success) {
                        phase_tick = 0;
                        phase = LED_PHASE_SUCCESS_BLINK;
                    } else {
                        phase = LED_PHASE_IDLE;
                    }
                }
                break;
            case LED_PHASE_SUCCESS_BLINK:
                if (faults != 0) {
                    render_fault_rgb(faults, tick);
                } else if (CONFIG_JINJIAN_STATUS_LED_GPIO >= 0) {
                    /* 有单色 LED：成功提示由它负责，RGB 恢复正常显示 */
                    render_ws2812_off();
                } else if (phase_tick == 0) {
                    /* 板上没有单色 LED：先灭一拍再用事件色补闪一次，
                     * 于是“双闪=成功、单闪=失败”，反馈不丢失 */
                    render_ws2812_off();
                } else {
                    render_ws2812_event(cur.ev);
                }
                render_status_on();
                phase_tick++;
                if (phase_tick >= LED_SUCCESS_BLINK_TICKS) {
                    phase = LED_PHASE_IDLE;
                }
                break;
            default:
                phase = LED_PHASE_IDLE;
                break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(LED_TASK_PERIOD_MS));
        tick++;
    }
}

void led_ctrl_init(void)
{
    if (s_initialized) {
        return;
    }
    s_initialized = true;

    s_ev_mutex = xSemaphoreCreateMutex();
    led_gpio_init_one(CONFIG_JINJIAN_PWR_LED_GPIO);
    led_gpio_init_one(CONFIG_JINJIAN_STATUS_LED_GPIO);
    esp_err_t err = led_ws2812_init();
    if (err != ESP_OK) {
        ESP_LOGW(LED_TAG, "WS2812 init failed (%s), RGB LED disabled",
                 esp_err_to_name(err));
    }
    xTaskCreate(led_task, "led_ctrl", LED_TASK_STACK, NULL, LED_TASK_PRIO, NULL);
    ESP_LOGI(LED_TAG, "led ctrl started: pwr=%d status=%d ws2812=%d",
             CONFIG_JINJIAN_PWR_LED_GPIO, CONFIG_JINJIAN_STATUS_LED_GPIO,
             CONFIG_JINJIAN_WS2812_GPIO);
}
