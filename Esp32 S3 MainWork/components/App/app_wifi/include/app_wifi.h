#ifndef APP_WIFI_H
#define APP_WIFI_H

#include <stdbool.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

esp_err_t app_wifi_init(void);
bool app_wifi_is_connected(void);
bool app_wifi_wait_connected(TickType_t timeout_ticks);

#endif
