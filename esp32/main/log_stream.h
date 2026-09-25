#pragma once

#include <stddef.h>

void log_stream_init(void);
void log_stream_suspend(void);
void log_stream_resume(void);
size_t log_stream_active_count(void);
