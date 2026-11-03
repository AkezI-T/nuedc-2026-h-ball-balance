#ifndef APP_JY901S_H
#define APP_JY901S_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    bool initialized;
    bool valid;
    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;
    int32_t acceleration_x_mg;
    int32_t acceleration_y_mg;
    int32_t acceleration_z_mg;
    int32_t filtered_acceleration_x_mg;
    int32_t longitudinal_acceleration_mg;
    int64_t received_at_us;
    uint32_t sample_count;
    uint32_t read_error_count;
} app_jy901s_state_t;

esp_err_t app_jy901s_init(void);
void app_jy901s_get_state(app_jy901s_state_t *state);

#endif
