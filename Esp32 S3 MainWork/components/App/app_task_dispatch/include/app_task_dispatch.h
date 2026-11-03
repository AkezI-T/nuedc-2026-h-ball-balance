#ifndef APP_TASK_DISPATCH_H
#define APP_TASK_DISPATCH_H

#include <stdbool.h>
#include <stdint.h>

#include "app_tmx_ingest.h"
#include "app_vision.h"
#include "esp_err.h"

typedef enum {
    APP_TASK_DISPATCH_IDLE = 0,
    APP_TASK_DISPATCH_PREP,
    APP_TASK_DISPATCH_BYPASS,
    APP_TASK_DISPATCH_T3_POSITIVE,
    APP_TASK_DISPATCH_T3_NEGATIVE,
    APP_TASK_DISPATCH_TRACK,
    APP_TASK_DISPATCH_COMPLETE,
    APP_TASK_DISPATCH_FAULT,
} app_task_dispatch_phase_t;

typedef struct {
    app_tmx_task_id_t task_id;
    uint32_t run;
    app_task_dispatch_phase_t phase;
    app_tmx_task_params_t params;
    bool prep_active;
    bool task_running;
    bool motor_required;
    bool vision_control_active;
    bool reference_valid;
    bool task_complete;
    bool task_fault;
    int32_t target_mm;
    int32_t reference_px;
    uint32_t minimum_vision_frame;
    uint32_t elapsed_ms;
    uint32_t stable_frames;
} app_task_dispatch_state_t;

esp_err_t app_task_dispatch_init(void);
void app_task_dispatch_step(const app_tmx_state_t *tmx,
                            const app_vision_state_t *vision,
                            int64_t now_us,
                            bool motor_near_level,
                            bool t3_outbound_braking,
                            app_task_dispatch_state_t *state);
void app_task_dispatch_get_state(app_task_dispatch_state_t *state);
const char *app_task_dispatch_phase_name(app_task_dispatch_phase_t phase);

#endif
