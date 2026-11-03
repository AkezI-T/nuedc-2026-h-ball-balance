#ifndef APP_RECORDER_H
#define APP_RECORDER_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define APP_RECORDER_TEXT_MAX 16U

typedef struct {
    uint32_t run;
    uint32_t source_timestamp_ms;
    char task[APP_RECORDER_TEXT_MAX];
    int32_t target_mm;
    bool target_valid;
} app_recorder_start_t;

typedef struct {
    uint32_t run;
    uint32_t source_timestamp_ms;
    char task[APP_RECORDER_TEXT_MAX];
    char result[APP_RECORDER_TEXT_MAX];
    char fault[APP_RECORDER_TEXT_MAX];
} app_recorder_stop_t;

esp_err_t app_recorder_init(void);
esp_err_t app_recorder_notify_start(const app_recorder_start_t *event);
esp_err_t app_recorder_notify_stop(const app_recorder_stop_t *event);

#endif
