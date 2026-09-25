#include "power_mgr.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_ctrl.h"
#include "log_stream.h"
#include "sdkconfig.h"
#include "web_server.h"

#define PM_TAG       "power"
#define PM_POLL_MS   500
#define PM_TASK_STACK 3072
#define PM_TASK_PRIO  2

static volatile bool s_wake_pending;
static volatile uint32_t s_last_activity_ms;

static uint32_t pm_now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void power_mgr_task(void *arg)
{
    (void)arg;
    s_last_activity_ms = pm_now_ms();

    for (;;) {
        bool has_client = web_server_has_client();
        uint32_t now = pm_now_ms();

        if (s_wake_pending) {
            s_wake_pending = false;
            s_last_activity_ms = now;
            ESP_LOGI(PM_TAG, "wake request");
            log_stream_resume();
            led_ctrl_power_save(false);
            web_server_start();
        }

        if (has_client) {
            s_last_activity_ms = now;
        } else if (now - s_last_activity_ms >= CONFIG_JINJIAN_IDLE_TIMEOUT_MS) {
            ESP_LOGW(PM_TAG, "no wifi client for %u ms, entering low-power idle",
                     (unsigned)CONFIG_JINJIAN_IDLE_TIMEOUT_MS);
            led_ctrl_power_save(true);   /* 关闭 LED */
            log_stream_suspend();        /* 关闭日志流 */
            web_server_stop();           /* 关闭 HTTP */
            web_server_wifi_stop();      /* 关闭 WiFi 射频与高优先级任务 */
            s_last_activity_ms = now;    /* 避免重复触发 */

            /* 空闲期间只保留 BMS 采集、Modbus 从机与 BOOT 按键监测 */
            while (!s_wake_pending && !web_server_has_client()) {
                vTaskDelay(pdMS_TO_TICKS(PM_POLL_MS));
            }
            /* 醒来后循环顶部会处理 wake / has_client */
        }

        vTaskDelay(pdMS_TO_TICKS(PM_POLL_MS));
    }
}

void power_mgr_init(void)
{
#if CONFIG_JINJIAN_IDLE_LOW_POWER
    xTaskCreate(power_mgr_task, "power_mgr", PM_TASK_STACK, NULL, PM_TASK_PRIO, NULL);
    ESP_LOGI(PM_TAG, "idle low-power enabled (timeout=%u ms, wake=BOOT short press)",
             (unsigned)CONFIG_JINJIAN_IDLE_TIMEOUT_MS);
#else
    ESP_LOGI(PM_TAG, "idle low-power disabled");
#endif
}

void power_mgr_notify_wifi_client(bool connected)
{
    if (connected) {
        s_last_activity_ms = pm_now_ms();
    }
}

void power_mgr_request_wake(void)
{
    s_wake_pending = true;
}
