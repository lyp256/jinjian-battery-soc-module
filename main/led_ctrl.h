#pragma once

#include <stdbool.h>

/* WS2812 事件单闪类型 */
typedef enum {
    LED_EVENT_485 = 0, /* 车端 485 总线收到查询：WS2812 蓝色闪烁 */
    LED_EVENT_BMS,     /* 读取 BMS 数据：WS2812 绿色闪烁 */
} led_event_t;

/* 初始化三个 LED 并启动状态灯任务。
 * 读取 app_config 的 led_enable 决定是否点亮，Web 页面修改后立即生效。 */
void led_ctrl_init(void);

/* 上报一次请求事件；success=true 时正常处理后单色 LED 会闪烁一次，失败不闪。 */
void led_ctrl_notify_event(led_event_t ev, bool success);

/* BOOT 按住期间 WS2812 紫色常亮；触发恢复出厂时切换为红色常亮。
 * holding=false 恢复正常状态显示。 */
void led_ctrl_boot_hold(bool holding, bool triggered);
