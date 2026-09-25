#pragma once

#include "esp_err.h"
#include <stddef.h>

esp_err_t ota_update_start(void);
esp_err_t ota_update_write(const uint8_t *data, size_t len);
esp_err_t ota_update_finish(void);
void ota_update_abort(void);
bool ota_update_active(void);
