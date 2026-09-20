#include "factory_reset.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

static const char *TAG = "factory_reset";

void factory_reset_check(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << CONFIG_JINJIAN_FACTORY_RESET_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    if (gpio_get_level(CONFIG_JINJIAN_FACTORY_RESET_GPIO) != 0) {
        return; /* 未按住 BOOT，正常启动 */
    }

    ESP_LOGI(TAG, "BOOT held, hold %u ms to factory reset",
             (unsigned)CONFIG_JINJIAN_FACTORY_RESET_HOLD_MS);
    uint32_t waited = 0;
    while (waited < CONFIG_JINJIAN_FACTORY_RESET_HOLD_MS) {
        vTaskDelay(pdMS_TO_TICKS(50));
        waited += 50;
        if (gpio_get_level(CONFIG_JINJIAN_FACTORY_RESET_GPIO) != 0) {
            ESP_LOGI(TAG, "BOOT released, factory reset cancelled");
            return;
        }
    }

    ESP_LOGW(TAG, "factory reset: erasing NVS and rebooting");
    nvs_flash_erase();
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
}
