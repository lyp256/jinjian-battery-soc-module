/* 宿主机测试用 esp_log 桩（仅 tools/rpc_test.c 使用） */
#pragma once

#include <stdio.h>

#define ESP_LOGI(tag, fmt, ...) ((void)(tag))
#define ESP_LOGW(tag, fmt, ...) ((void)(tag))
#define ESP_LOGE(tag, fmt, ...) ((void)(tag))
#define ESP_LOGD(tag, fmt, ...) ((void)(tag))
