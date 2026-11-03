#ifndef APP_MOTOR_CONTROL_H
#define APP_MOTOR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "app_task_dispatch.h"
#include "esp_err.h"

typedef enum {
    APP_MOTOR_CONTROL_PHASE_HOLD = 0,
    APP_MOTOR_CONTROL_PHASE_DRIVE,
    APP_MOTOR_CONTROL_PHASE_TURN,
    APP_MOTOR_CONTROL_PHASE_BRAKE,
    APP_MOTOR_CONTROL_PHASE_DAMP,
    APP_MOTOR_CONTROL_PHASE_SETTLE,
} app_motor_control_phase_t;

typedef struct {
    bool enabled;
    bool test_centering_active;
    bool test_balance_ready;
    bool vision_control_active;
    bool vision_fallback_active;
    bool vision_step_active;
    bool angle_limit_active;
    bool task_running;
    bool task_latched;
    bool neutral_valid;
    bool phase_angle_reached;
    bool drive_fault;
    bool task_complete;
    bool task_fault;
    bool reference_valid;
    bool t3_pulse_active;
    uint8_t drive_boost_level;
    uint8_t t3_pulse_attempts;
    app_tmx_task_id_t task_id;
    app_task_dispatch_phase_t dispatch_phase;
    app_motor_control_phase_t control_phase;
    int32_t control_error_px;
    int32_t reference_px;
    int32_t estimated_velocity_px_s;
    int32_t brake_compensation_px;
    int32_t predicted_stop_error_px;
    int32_t t4_acceleration_feedforward_count;
    int32_t control_target_mm;
    int32_t encoder_count;
    int32_t encoder_target_count;
    int32_t neutral_count;
    uint32_t balance_pwm_angle_mdeg;
    int32_t net_step_position;
    int8_t command_direction;
    uint32_t step_frequency_hz;
    uint32_t vision_pulse_count;
    uint32_t control_frame_count;
    uint32_t phase_elapsed_ms;
    uint32_t task_run;
    uint32_t task_elapsed_ms;
    uint32_t task_start_count;
} app_motor_control_state_t;

esp_err_t app_motor_control_init(void);
void app_motor_control_get_state(app_motor_control_state_t *state);

#endif
