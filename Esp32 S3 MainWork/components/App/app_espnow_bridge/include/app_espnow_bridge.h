#ifndef APP_ESPNOW_BRIDGE_H
#define APP_ESPNOW_BRIDGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

esp_err_t app_espnow_bridge_init(void);
esp_err_t app_espnow_bridge_send(const uint8_t *data, size_t length,
                                 TickType_t timeout_ticks);
bool app_espnow_bridge_peer_ready(void);

#endif
