#include "ota_update.h"

#include <stdbool.h>
#include <string.h>

#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"

static const char *TAG = "ota";

static esp_ota_handle_t s_handle;
static const esp_partition_t *s_partition;
static bool s_active;

esp_err_t ota_update_start(void)
{
    if (s_active) {
        return ESP_ERR_INVALID_STATE;
    }
    s_partition = esp_ota_get_next_update_partition(NULL);
    if (s_partition == NULL) {
        ESP_LOGE(TAG, "no OTA partition found");
        return ESP_FAIL;
    }
    esp_err_t err = esp_ota_begin(s_partition, OTA_WITH_SEQUENTIAL_WRITES, &s_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        return err;
    }
    s_active = true;
    ESP_LOGI(TAG, "OTA start -> %s @0x%lx", s_partition->label,
             (unsigned long)s_partition->address);
    return ESP_OK;
}

esp_err_t ota_update_write(const uint8_t *data, size_t len)
{
    if (!s_active) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = esp_ota_write(s_handle, data, len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t ota_update_finish(void)
{
    if (!s_active) {
        return ESP_ERR_INVALID_STATE;
    }
    s_active = false;
    esp_err_t err = esp_ota_end(s_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        return err;
    }
    err = esp_ota_set_boot_partition(s_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "OTA finish -> %s", s_partition->label);
    return ESP_OK;
}

void ota_update_abort(void)
{
    if (!s_active) {
        return;
    }
    s_active = false;
    esp_ota_abort(s_handle);
    ESP_LOGW(TAG, "OTA aborted");
}

bool ota_update_active(void)
{
    return s_active;
}
