#pragma once

#include <stdbool.h>

void web_server_start(void);
void web_server_stop(void);
void web_server_wifi_stop(void);
bool web_server_has_client(void);
