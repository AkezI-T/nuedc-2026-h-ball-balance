#ifndef APP_TMX_INGEST_H
#define APP_TMX_INGEST_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum {
    APP_TMX_TASK_NONE = 0,
    APP_TMX_TASK_T1,
    APP_TMX_TASK_T2,
    APP_TMX_TASK_T3,
    APP_TMX_TASK_T4,
    APP_TMX_TASK_T5,
    APP_TMX_TASK_T6,
    APP_TMX_TASK_COUNT,
} app_tmx_task_id_t;

typedef struct {
    int32_t target_mm;
    int32_t secondary_target_mm;
    uint32_t timeout_ms;
    int32_t reach_error_px;
    uint32_t reach_frames;
    int32_t reach_speed_px_s;
    int32_t settle_error_px;
    int32_t settle_speed_px_s;
    uint32_t settle_frames;
    uint32_t capture_frames;
} app_tmx_task_params_t;

typedef struct {
    bool task_running;
    char task[16];
    app_tmx_task_id_t task_id;
    uint32_t run;
    bool mode_active;
    char mode[16];
    char mode_task[16];
    app_tmx_task_id_t mode_task_id;
    app_tmx_task_params_t active_params;
    uint32_t received_lines;
    uint32_t dropped_lines;
    uint32_t accepted_task_events;
    uint32_t duplicate_task_events;
    uint32_t last_event_sequence;
    uint32_t accepted_mode_events;
    uint32_t duplicate_mode_events;
    uint32_t last_mode_event_sequence;
    uint32_t parameter_updates;
} app_tmx_state_t;

esp_err_t app_tmx_ingest_init(void);
void app_tmx_ingest_get_state(app_tmx_state_t *state);
void app_tmx_ingest_get_task_params(app_tmx_task_id_t task_id,
                                    app_tmx_task_params_t *params);
bool app_tmx_ingest_complete_task(app_tmx_task_id_t task_id, uint32_t run);
app_tmx_task_id_t app_tmx_task_from_name(const char *name);
const char *app_tmx_task_name(app_tmx_task_id_t task_id);
bool app_tmx_task_uses_ball_control(app_tmx_task_id_t task_id);

#endif
