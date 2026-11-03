#include "app_task_dispatch.h"

#include <stdio.h>
#include <string.h>

#include "app_udp_log.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#define DISPATCH_VISION_FRESH_US 1000000LL
#define T3_HALF_CM_ERROR_PX 13
#define T3_SETTLE_CONFIRM_US 300000LL

typedef struct {
    app_task_dispatch_state_t state;
    int64_t task_started_at_us;
    uint32_t last_vision_frame;
    uint32_t capture_samples;
    int64_t capture_sum_px;
    bool capture_announced;
    bool t4_finish_hold_active;
    bool t3_outbound_brake_seen;
    int64_t t3_settle_started_at_us;
} task_dispatch_context_t;

static const char *TAG = "app_dispatch";
static portMUX_TYPE s_state_lock = portMUX_INITIALIZER_UNLOCKED;
static task_dispatch_context_t s_dispatch;
static app_task_dispatch_state_t s_public_state;

static int32_t absolute_i32(int32_t value)
{
    return value < 0 ? -value : value;
}

static bool vision_is_fresh(const app_vision_state_t *vision,
                            int64_t now_us)
{
    return vision->received_at_us > 0 && now_us >= vision->received_at_us &&
           now_us - vision->received_at_us <= DISPATCH_VISION_FRESH_US;
}

const char *app_task_dispatch_phase_name(app_task_dispatch_phase_t phase)
{
    switch (phase) {
    case APP_TASK_DISPATCH_PREP: return "PREP";
    case APP_TASK_DISPATCH_BYPASS: return "BYPASS";
    case APP_TASK_DISPATCH_T3_POSITIVE: return "T3_POS";
    case APP_TASK_DISPATCH_T3_NEGATIVE: return "T3_NEG";
    case APP_TASK_DISPATCH_TRACK: return "TRACK";
    case APP_TASK_DISPATCH_COMPLETE: return "COMPLETE";
    case APP_TASK_DISPATCH_FAULT: return "FAULT";
    case APP_TASK_DISPATCH_IDLE:
    default:
        return "IDLE";
    }
}

static void publish_target_state(const char *state)
{
    char line[192];
    int length = snprintf(
        line, sizeof(line),
        "@TARGET source=ESP32 task=%s run=%lu state=%s targetMm=%ld"
        " referencePx=%ld referenceValid=%u",
        app_tmx_task_name(s_dispatch.state.task_id),
        (unsigned long)s_dispatch.state.run, state,
        (long)s_dispatch.state.target_mm,
        (long)s_dispatch.state.reference_px,
        s_dispatch.state.reference_valid ? 1U : 0U);
    if (length > 0 && length < (int)sizeof(line)) {
        ESP_LOGI(TAG, "%s", line);
        (void)app_udp_log_publish(line, (size_t)length);
    }
}

static void set_target(const app_vision_state_t *vision,
                       int32_t target_mm, const char *state)
{
    s_dispatch.state.target_mm = target_mm;
    s_dispatch.state.minimum_vision_frame = vision->frame_count + 1U;
    s_dispatch.state.stable_frames = 0U;
    esp_err_t error = app_vision_set_target_mm(target_mm);
    if (error != ESP_OK && error != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "%s目标同步失败: %s", state,
                 esp_err_to_name(error));
    }
    publish_target_state(state);
}

static void load_task_params(app_tmx_task_id_t task_id)
{
    app_tmx_ingest_get_task_params(task_id, &s_dispatch.state.params);
    if (task_id == APP_TMX_TASK_T3) {
        if (s_dispatch.state.params.reach_error_px >
            T3_HALF_CM_ERROR_PX) {
            s_dispatch.state.params.reach_error_px =
                T3_HALF_CM_ERROR_PX;
        }
        if (s_dispatch.state.params.settle_error_px >
            T3_HALF_CM_ERROR_PX) {
            s_dispatch.state.params.settle_error_px =
                T3_HALF_CM_ERROR_PX;
        }
    }
}

static void reset_t3_trajectory(void)
{
    s_dispatch.t3_outbound_brake_seen = false;
    s_dispatch.t3_settle_started_at_us = 0;
}

static void enter_prepare(app_tmx_task_id_t task_id,
                          const app_vision_state_t *vision)
{
    s_dispatch.t4_finish_hold_active = false;
    s_dispatch.state = (app_task_dispatch_state_t){
        .task_id = task_id,
        .phase = APP_TASK_DISPATCH_PREP,
        .prep_active = true,
        .motor_required = app_tmx_task_uses_ball_control(task_id),
    };
    load_task_params(task_id);
    s_dispatch.capture_samples = 0U;
    s_dispatch.capture_sum_px = 0;
    s_dispatch.capture_announced = false;
    s_dispatch.last_vision_frame = vision->frame_count;
    reset_t3_trajectory();

    if (!s_dispatch.state.motor_required) {
        return;
    }
    int32_t prep_target = s_dispatch.state.params.target_mm;
    if (task_id == APP_TMX_TASK_T3) {
        prep_target = 0;
    }
    s_dispatch.state.vision_control_active = false;
    set_target(vision, prep_target, "PREP");
}

static void update_t6_capture(const app_vision_state_t *vision,
                              int64_t now_us)
{
    if (vision->frame_count == s_dispatch.last_vision_frame) {
        return;
    }
    s_dispatch.last_vision_frame = vision->frame_count;
    bool stable_sample = vision->detected && vision_is_fresh(vision, now_us) &&
        absolute_i32(vision->velocity_px_s) <=
            s_dispatch.state.params.settle_speed_px_s;
    if (!stable_sample) {
        s_dispatch.capture_samples = 0U;
        s_dispatch.capture_sum_px = 0;
        return;
    }

    s_dispatch.capture_sum_px += vision->error_px;
    s_dispatch.capture_samples++;
    if (s_dispatch.capture_samples <
        s_dispatch.state.params.capture_frames) {
        return;
    }
    s_dispatch.state.reference_px = (int32_t)(
        s_dispatch.capture_sum_px / (int64_t)s_dispatch.capture_samples);
    s_dispatch.state.reference_valid = true;
    s_dispatch.capture_samples = 0U;
    s_dispatch.capture_sum_px = 0;
    if (!s_dispatch.capture_announced) {
        s_dispatch.capture_announced = true;
        publish_target_state("CAPTURED");
    }
}

static void refresh_prepare_params(const app_vision_state_t *vision)
{
    app_tmx_task_params_t params;
    app_tmx_ingest_get_task_params(s_dispatch.state.task_id, &params);
    int32_t previous_target_mm = s_dispatch.state.target_mm;
    s_dispatch.state.params = params;

    if (!s_dispatch.state.motor_required) {
        return;
    }
    int32_t prep_target_mm = params.target_mm;
    if (s_dispatch.state.task_id == APP_TMX_TASK_T3) {
        prep_target_mm = 0;
    }
    if (prep_target_mm == previous_target_mm) {
        return;
    }

    if (s_dispatch.state.task_id == APP_TMX_TASK_T6) {
        s_dispatch.state.reference_valid = false;
        s_dispatch.capture_samples = 0U;
        s_dispatch.capture_sum_px = 0;
        s_dispatch.capture_announced = false;
    }
    set_target(vision, prep_target_mm, "PREP_PARAM");
}

static void begin_task(const app_tmx_state_t *tmx,
                       const app_vision_state_t *vision,
                       int64_t now_us)
{
    bool captured_reference_valid = s_dispatch.state.prep_active &&
        s_dispatch.state.reference_valid &&
        s_dispatch.state.task_id == APP_TMX_TASK_T6;
    int32_t captured_reference_px = s_dispatch.state.reference_px;

    s_dispatch.state = (app_task_dispatch_state_t){
        .task_id = tmx->task_id,
        .run = tmx->run,
        .task_running = true,
        .motor_required = app_tmx_task_uses_ball_control(tmx->task_id),
        .minimum_vision_frame = vision->frame_count + 1U,
    };
    s_dispatch.state.params = tmx->active_params;
    s_dispatch.t4_finish_hold_active = false;
    s_dispatch.task_started_at_us = now_us;
    s_dispatch.last_vision_frame = vision->frame_count;
    reset_t3_trajectory();

    switch (tmx->task_id) {
    case APP_TMX_TASK_T3:
        s_dispatch.state.phase = APP_TASK_DISPATCH_T3_POSITIVE;
        s_dispatch.state.vision_control_active = true;
        set_target(vision, s_dispatch.state.params.target_mm, "T3_POS");
        break;
    case APP_TMX_TASK_T4:
        s_dispatch.state.phase = APP_TASK_DISPATCH_TRACK;
        s_dispatch.state.vision_control_active = true;
        set_target(vision, s_dispatch.state.params.target_mm, "TRACK");
        break;
    case APP_TMX_TASK_T5:
        s_dispatch.state.phase = APP_TASK_DISPATCH_TRACK;
        s_dispatch.state.vision_control_active = true;
        set_target(vision, s_dispatch.state.params.target_mm, "TRACK");
        break;
    case APP_TMX_TASK_T6:
        s_dispatch.state.reference_valid = captured_reference_valid;
        s_dispatch.state.reference_px = captured_reference_px;
        s_dispatch.state.target_mm = s_dispatch.state.params.target_mm;
        if (captured_reference_valid) {
            s_dispatch.state.phase = APP_TASK_DISPATCH_TRACK;
            s_dispatch.state.vision_control_active = true;
            set_target(vision, s_dispatch.state.params.target_mm, "FROZEN");
        } else {
            s_dispatch.state.phase = APP_TASK_DISPATCH_FAULT;
            s_dispatch.state.task_fault = true;
            publish_target_state("NO_REFERENCE");
        }
        break;
    case APP_TMX_TASK_T1:
    case APP_TMX_TASK_T2:
        s_dispatch.state.phase = APP_TASK_DISPATCH_BYPASS;
        break;
    case APP_TMX_TASK_NONE:
    default:
        s_dispatch.state.phase = APP_TASK_DISPATCH_FAULT;
        s_dispatch.state.task_fault = true;
        break;
    }
}

static void end_task(void)
{
    s_dispatch.t4_finish_hold_active = false;
    s_dispatch.state.task_running = false;
    s_dispatch.state.prep_active = false;
    s_dispatch.state.motor_required = false;
    s_dispatch.state.vision_control_active = false;
    if (!s_dispatch.state.task_fault) {
        s_dispatch.state.task_complete = true;
        s_dispatch.state.phase = APP_TASK_DISPATCH_COMPLETE;
    }
}

static void enter_t4_finish_hold(const app_vision_state_t *vision)
{
    s_dispatch.t4_finish_hold_active = true;
    s_dispatch.state.task_complete = true;
    s_dispatch.state.phase = APP_TASK_DISPATCH_COMPLETE;
    s_dispatch.state.motor_required = true;
    s_dispatch.state.vision_control_active = true;
    if (s_dispatch.state.target_mm != 0) {
        set_target(vision, 0, "T4_HOLD_ZERO");
    } else {
        publish_target_state("T4_HOLD_ZERO");
    }
}

static bool new_vision_sample_ready(const app_vision_state_t *vision,
                                    int64_t now_us)
{
    if (vision->frame_count == s_dispatch.last_vision_frame ||
        vision->frame_count < s_dispatch.state.minimum_vision_frame) {
        return false;
    }
    s_dispatch.last_vision_frame = vision->frame_count;
    return vision->detected && vision_is_fresh(vision, now_us);
}

static void publish_t3_first_target_stable(int32_t error_px,
                                           int32_t velocity_px_s)
{
    char line[160];
    int length = snprintf(
        line, sizeof(line),
        "@T3 source=ESP32 run=%lu event=FIRST_TARGET_STABLE"
        " e=%ld v=%ld",
        (unsigned long)s_dispatch.state.run,
        (long)error_px, (long)velocity_px_s);
    if (length > 0 && length < (int)sizeof(line)) {
        ESP_LOGI(TAG, "%s", line);
        (void)app_udp_log_publish(line, (size_t)length);
    }
}

static void update_t3(const app_vision_state_t *vision, int64_t now_us,
                      bool motor_near_level,
                      bool t3_outbound_braking)
{
    if (s_dispatch.task_started_at_us > 0 &&
        now_us >= s_dispatch.task_started_at_us) {
        s_dispatch.state.elapsed_ms = (uint32_t)(
            (now_us - s_dispatch.task_started_at_us) / 1000LL);
    }
    if (!new_vision_sample_ready(vision, now_us)) {
        return;
    }

    if (s_dispatch.state.phase == APP_TASK_DISPATCH_T3_POSITIVE) {
        if (t3_outbound_braking) {
            s_dispatch.t3_outbound_brake_seen = true;
        }

        bool first_position_stable = absolute_i32(vision->error_px) <=
            s_dispatch.state.params.reach_error_px;
        bool first_speed_stable = absolute_i32(vision->velocity_px_s) <=
            s_dispatch.state.params.reach_speed_px_s;
        if (s_dispatch.t3_outbound_brake_seen &&
            first_position_stable && first_speed_stable &&
            motor_near_level) {
            if (s_dispatch.state.stable_frames < UINT32_MAX) {
                s_dispatch.state.stable_frames++;
            }
        } else {
            s_dispatch.state.stable_frames = 0U;
        }
        if (s_dispatch.state.stable_frames >=
            s_dispatch.state.params.reach_frames) {
            publish_t3_first_target_stable(
                vision->error_px, vision->velocity_px_s);
            s_dispatch.state.phase = APP_TASK_DISPATCH_T3_NEGATIVE;
            set_target(vision,
                       s_dispatch.state.params.secondary_target_mm,
                       "T3_NEG");
        }
        return;
    }

    if (s_dispatch.state.phase == APP_TASK_DISPATCH_T3_NEGATIVE) {
        bool position_stable = absolute_i32(vision->error_px) <=
            s_dispatch.state.params.settle_error_px;
        bool speed_stable = absolute_i32(vision->velocity_px_s) <=
            s_dispatch.state.params.settle_speed_px_s;
        if (position_stable && speed_stable && motor_near_level) {
            if (s_dispatch.t3_settle_started_at_us == 0) {
                s_dispatch.t3_settle_started_at_us = now_us;
            }
            s_dispatch.state.stable_frames++;
        } else {
            s_dispatch.state.stable_frames = 0U;
            s_dispatch.t3_settle_started_at_us = 0;
        }
        if (s_dispatch.state.stable_frames >=
                s_dispatch.state.params.settle_frames &&
            s_dispatch.t3_settle_started_at_us > 0 &&
            now_us - s_dispatch.t3_settle_started_at_us >=
                T3_SETTLE_CONFIRM_US) {
            s_dispatch.state.task_complete = true;
            s_dispatch.state.phase = APP_TASK_DISPATCH_COMPLETE;
            publish_target_state("STABLE");
            (void)app_tmx_ingest_complete_task(
                APP_TMX_TASK_T3, s_dispatch.state.run);
        }
    }
}

esp_err_t app_task_dispatch_init(void)
{
    s_dispatch = (task_dispatch_context_t){0};
    s_dispatch.state.phase = APP_TASK_DISPATCH_IDLE;
    taskENTER_CRITICAL(&s_state_lock);
    s_public_state = s_dispatch.state;
    taskEXIT_CRITICAL(&s_state_lock);
    return ESP_OK;
}

void app_task_dispatch_step(const app_tmx_state_t *tmx,
                            const app_vision_state_t *vision,
                            int64_t now_us,
                            bool motor_near_level,
                            bool t3_outbound_braking,
                            app_task_dispatch_state_t *state)
{
    if (tmx == NULL || vision == NULL) {
        return;
    }

    bool prep_or_test_now = tmx->mode_active &&
        (strcmp(tmx->mode, "PREP") == 0 ||
         strcmp(tmx->mode, "TEST") == 0);
    if (s_dispatch.t4_finish_hold_active && prep_or_test_now) {
        end_task();
    }

    bool same_running_task = s_dispatch.state.task_running &&
        tmx->task_running && s_dispatch.state.task_id == tmx->task_id &&
        s_dispatch.state.run == tmx->run;
    if (tmx->task_running && !same_running_task) {
        begin_task(tmx, vision, now_us);
    } else if (s_dispatch.state.task_running && !tmx->task_running) {
        if (s_dispatch.state.task_id == APP_TMX_TASK_T4) {
            if (!s_dispatch.t4_finish_hold_active) {
                enter_t4_finish_hold(vision);
            }
        } else {
            end_task();
        }
    }

    if (s_dispatch.state.task_running) {
        if (s_dispatch.state.task_id == APP_TMX_TASK_T3) {
            update_t3(vision, now_us, motor_near_level,
                      t3_outbound_braking);
        } else if (s_dispatch.task_started_at_us > 0 &&
                   now_us >= s_dispatch.task_started_at_us) {
            s_dispatch.state.elapsed_ms = (uint32_t)(
                (now_us - s_dispatch.task_started_at_us) / 1000LL);
        }
    } else {
        bool prep_now = tmx->mode_active &&
            strcmp(tmx->mode, "PREP") == 0 &&
            tmx->mode_task_id != APP_TMX_TASK_NONE;
        if (prep_now &&
            (!s_dispatch.state.prep_active ||
             s_dispatch.state.task_id != tmx->mode_task_id)) {
            enter_prepare(tmx->mode_task_id, vision);
        } else if (!prep_now && s_dispatch.state.prep_active) {
            s_dispatch.state.prep_active = false;
            s_dispatch.state.motor_required = false;
            s_dispatch.state.vision_control_active = false;
            s_dispatch.state.phase = APP_TASK_DISPATCH_IDLE;
        }
        if (prep_now) {
            refresh_prepare_params(vision);
            if (tmx->mode_task_id == APP_TMX_TASK_T6) {
                update_t6_capture(vision, now_us);
            }
        }
    }

    taskENTER_CRITICAL(&s_state_lock);
    s_public_state = s_dispatch.state;
    app_task_dispatch_state_t snapshot = s_public_state;
    taskEXIT_CRITICAL(&s_state_lock);
    if (state != NULL) {
        *state = snapshot;
    }
}

void app_task_dispatch_get_state(app_task_dispatch_state_t *state)
{
    if (state == NULL) {
        return;
    }
    taskENTER_CRITICAL(&s_state_lock);
    *state = s_public_state;
    taskEXIT_CRITICAL(&s_state_lock);
}
