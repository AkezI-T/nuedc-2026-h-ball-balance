#ifndef APP_VISION_H
#define APP_VISION_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    bool detected;
    int16_t error_px;
    int16_t last_error_px;
    int32_t velocity_px_s;
    int32_t target_mm;
    int64_t received_at_us;
    int64_t last_detected_at_us;
    uint32_t frame_count;
    uint32_t parse_error_count;
} app_vision_state_t;

esp_err_t app_vision_init(void);
esp_err_t app_vision_set_target_mm(int32_t target_mm);
void app_vision_get_state(app_vision_state_t *state);

#endif
