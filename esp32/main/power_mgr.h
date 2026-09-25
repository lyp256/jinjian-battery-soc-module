#pragma once

#include <stdbool.h>

/* 初始化低功耗管理任务（无 WiFi 客户端一段时间后自动休眠非必要功能）。 */
void power_mgr_init(void);

/* 由 web_server 在 AP 客户端接入/断开时通知（用于重置空闲计时）。 */
void power_mgr_notify_wifi_client(bool connected);

/* 请求唤醒（BOOT 短按），恢复 WiFi/Web/日志/LED。 */
void power_mgr_request_wake(void);
