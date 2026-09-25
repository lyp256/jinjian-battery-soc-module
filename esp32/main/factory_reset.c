#include "factory_reset.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_ctrl.h"
#include "nvs_flash.h"
#include "power_mgr.h"
#include "sdkconfig.h"

#define FR_TAG        "factory_reset"
#define FR_POLL_MS    50
#define FR_TASK_STACK 2048
#define FR_TASK_PRIO  3

static bool fr_button_pressed(int level)
{
#ifdef CONFIG_JINJIAN_FACTORY_RESET_ACTIVE_LOW
    return level == 0;
#else
    return level != 0;
#endif
}

static void factory_reset_task(void *arg)
{
    (void)arg;
    bool was_pressed = false;
    uint32_t held_ms = 0;

    for (;;) {
        int level = gpio_get_level(CONFIG_JINJIAN_FACTORY_RESET_GPIO);
        bool pressed = fr_button_pressed(level);

        if (pressed && !was_pressed) {
            ESP_LOGI(FR_TAG, "BOOT pressed (gpio=%d level=%d), hold %u ms to factory reset",
                     CONFIG_JINJIAN_FACTORY_RESET_GPIO, level,
                     (unsigned)CONFIG_JINJIAN_FACTORY_RESET_HOLD_MS);
            led_ctrl_boot_hold(true, false); /* 按住：WS2812 紫色常亮 */
        }

        if (pressed) {
            held_ms += FR_POLL_MS;
            if (held_ms >= CONFIG_JINJIAN_FACTORY_RESET_HOLD_MS) {
                led_ctrl_boot_hold(true, true); /* 触发：WS2812 红色常亮 */
                ESP_LOGW(FR_TAG, "factory reset triggered: erasing NVS and rebooting");
                nvs_flash_erase();
                vTaskDelay(pdMS_TO_TICKS(500));
                esp_restart();
            }
        } else {
            if (was_pressed) {
                led_ctrl_boot_hold(false, false);
                power_mgr_request_wake(); /* 短按 BOOT：从低功耗唤醒 */
                ESP_LOGI(FR_TAG, "BOOT released (held %u ms), factory reset cancelled",
                         (unsigned)held_ms);
            }
            held_ms = 0;
        }

        was_pressed = pressed;
        vTaskDelay(pdMS_TO_TICKS(FR_POLL_MS));
    }
}

void factory_reset_check(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << CONFIG_JINJIAN_FACTORY_RESET_GPIO,
        .mode = GPIO_MODE_INPUT,
#ifdef CONFIG_JINJIAN_FACTORY_RESET_ACTIVE_LOW
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
#else
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
#endif
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    xTaskCreate(factory_reset_task, "factory_reset", FR_TASK_STACK,
                NULL, FR_TASK_PRIO, NULL);
    ESP_LOGI(FR_TAG, "factory reset monitor started (gpio=%d hold=%u ms)",
             CONFIG_JINJIAN_FACTORY_RESET_GPIO,
             (unsigned)CONFIG_JINJIAN_FACTORY_RESET_HOLD_MS);
}
