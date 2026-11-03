#ifndef APP_UDP_LOG_H
#define APP_UDP_LOG_H

#include <stddef.h>

#include "esp_err.h"

esp_err_t app_udp_log_init(void);
esp_err_t app_udp_log_publish(const char *line, size_t length);

#endif
