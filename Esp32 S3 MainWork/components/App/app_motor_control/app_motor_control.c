#include "app_motor_control.h"

#include <string.h>

#include "app_core_affinity.h"
#include "app_jy901s.h"
#include "app_task_dispatch.h"
#include "app_tmx_ingest.h"
#include "app_vision.h"
#include "bsp_d36a.h"
#include "bsp_encoder.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MOTOR_CONTROL_PERIOD_MS 10U
#define MOTOR_CONTROL_STACK_SIZE 4096U
#define MOTOR_CONTROL_PRIORITY 8U

/* 视觉外环按固定有效角度驱动/制动，编码器位置内环跟踪摆杆角度目标。 */
#define MOTOR_BALANCE_MIN_SETTLE_US 1000000LL
#define MOTOR_BALANCE_STABLE_US 300000LL
#define MOTOR_PREP_HANDOFF_GRACE_US 1000000LL
#define MOTOR_VISION_CONTROL_FRESH_US 1000000LL
#define MOTOR_IMU_CONTROL_FRESH_US 200000LL
#define MOTOR_VISION_BRAKE_SPEED_LIMIT_PX_S 800
#define MOTOR_VISION_BRAKE_ACCEL_PX_S2 700
#define MOTOR_BRAKE_RESPONSE_MS 460
#define MOTOR_LEVEL_RESPONSE_MS 180
#define MOTOR_DRIVE_START_ERROR_PX 22
#define MOTOR_DRIVE_START_STABLE_FRAMES 3U
#define MOTOR_BRAKE_SWITCH_MARGIN_PX 6
#define MOTOR_T3_NEG_BRAKE_SWITCH_MARGIN_PX (-10)
#define MOTOR_SETTLE_POSITION_PX 8
#define MOTOR_SETTLE_SPEED_PX_S 30
#define MOTOR_BRAKE_DAMP_SWITCH_SPEED_PX_S 120
#define MOTOR_BRAKE_SPEED_DROP_PX_S 20
#define MOTOR_DAMP_POSITION_DIVISOR 6
#define MOTOR_DAMP_VELOCITY_DIVISOR 15
#define MOTOR_DAMP_MAX_OFFSET_COUNT 25
#define MOTOR_DAMP_SLEW_COUNT_PER_FRAME 3
#define MOTOR_DAMP_SETTLE_SPEED_PX_S 25
#define MOTOR_DAMP_STABLE_FRAMES 8U
#define MOTOR_DAMP_POSITION_STABLE_PX 4
#define MOTOR_DAMP_MIN_STABLE_MS 300U
#define MOTOR_DAMP_RECOVERY_ERROR_PX 48
#define MOTOR_DAMP_RECOVERY_SPEED_PX_S 60
#define MOTOR_T3_LARGE_OFFSET_COUNT 36
#define MOTOR_T3_POS_LEVEL_TRIM_COUNT 3
#define MOTOR_T3_NEG_LEVEL_TRIM_COUNT (-10)
#define MOTOR_T3_RETURN_POSITION_DIVISOR 6
#define MOTOR_T3_RETURN_VELOCITY_DIVISOR 15
#define MOTOR_T3_RETURN_MAX_OFFSET_COUNT MOTOR_T3_LARGE_OFFSET_COUNT
#define MOTOR_T3_DRIVE_PEAK_HOLD_US 250000LL
#define MOTOR_T3_TURN_LEVEL_HOLD_US 50000LL
#define MOTOR_T3_POS_BRAKE_MIN_OFFSET_COUNT 22
#define MOTOR_T3_POS_BRAKE_MAX_OFFSET_COUNT 30
#define MOTOR_T3_NEG_BRAKE_MIN_OFFSET_COUNT 30
#define MOTOR_T3_NEG_BRAKE_MAX_OFFSET_COUNT 42
#define MOTOR_T3_BRAKE_VELOCITY_DIVISOR 4
#define MOTOR_T3_BRAKE_DAMP_SWITCH_SPEED_PX_S 60
#define MOTOR_T3_RETURN_START_ERROR_PX 80
#define MOTOR_T3_DAMP_MAX_OFFSET_COUNT 10
#define MOTOR_T3_DAMP_SLEW_COUNT_PER_FRAME 2
#define MOTOR_T3_PULSE_ENTRY_ERROR_PX 80
#define MOTOR_T3_PULSE_STATIONARY_SPEED_PX_S 8
#define MOTOR_T3_PULSE_STATIONARY_US 300000LL
#define MOTOR_T3_PULSE_DRIVE_MIN_MS 180U
#define MOTOR_T3_PULSE_DRIVE_MAX_MS 450U
#define MOTOR_T3_PULSE_BRAKE_MIN_MS 120U
#define MOTOR_T3_PULSE_BRAKE_MAX_MS 300U
#define MOTOR_T3_PULSE_REFRACTORY_MS 600U
#define MOTOR_T3_PULSE_MIN_OFFSET_COUNT 16
#define MOTOR_T3_PULSE_STEP_OFFSET_COUNT 4
#define MOTOR_T3_PULSE_MAX_OFFSET_COUNT 24
#define MOTOR_T3_PULSE_OUTPUT_LIMIT_COUNT 50
#define MOTOR_T3_PULSE_RESPONSE_PX 2
#define MOTOR_T3_PULSE_RESPONSE_SPEED_PX_S 8
#define MOTOR_T3_PULSE_REVERSE_SPEED_PX_S 3
#define MOTOR_T3_PULSE_MAX_ATTEMPTS 3U
/* T4搬运采用连续PD阻尼，低速积分只学习管道与运输的稳态偏置。 */
#define MOTOR_T4_LAUNCH_DAMP_MS 600U
#define MOTOR_T4_LAUNCH_FEEDFORWARD_COUNT 50
#define MOTOR_T4_LAUNCH_MAX_OFFSET_COUNT 60
#define MOTOR_T4_LAUNCH_FEEDFORWARD_FADE_MS 300U
#define MOTOR_T4_POSITION_DIVISOR 14
#define MOTOR_T4_LAUNCH_VELOCITY_DIVISOR 10
#define MOTOR_T4_VELOCITY_DIVISOR 10
#define MOTOR_T4_STATIC_FRICTION_ERROR_PX 27
#define MOTOR_T4_STATIC_FRICTION_SPEED_PX_S 25
#define MOTOR_T4_STATIC_FRICTION_OFFSET_COUNT 22
#define MOTOR_T4_STATIC_FRICTION_CONFIRM_US 300000LL
#define MOTOR_T4_STATIC_FRICTION_PULSE_US 250000LL
#define MOTOR_T4_STATIC_FRICTION_REFRACTORY_US 700000LL
#define MOTOR_T4_BRAKE_PHASE_SPEED_PX_S 80
#define MOTOR_T4_INTEGRAL_SPEED_LIMIT_PX_S 60
#define MOTOR_T4_INTEGRAL_ERROR_LIMIT_PX 108
#define MOTOR_T4_INTEGRAL_INPUT_LIMIT_PX 45
#define MOTOR_T4_INTEGRAL_LIMIT_PX_FRAMES 900
#define MOTOR_T4_INTEGRAL_DIVISOR 150
#define MOTOR_T4_MAX_OFFSET_COUNT 26
#define MOTOR_T4_BRAKE_MAX_OFFSET_COUNT 30
#define MOTOR_T4_SLEW_COUNT_PER_FRAME 6
#define MOTOR_T4_ACCELERATION_DEADBAND_MG 12
#define MOTOR_T4_ACCELERATION_NUMERATOR 2
#define MOTOR_T4_ACCELERATION_DIVISOR 5
#define MOTOR_T4_ACCELERATION_MAX_OFFSET_COUNT 36
#define MOTOR_T4_ACCELERATION_SLEW_COUNT_PER_STEP 2
#define MOTOR_T4_TOTAL_MAX_OFFSET_COUNT 76
#define MOTOR_DRIVE_PROGRESS_WINDOW_MS 600U
#define MOTOR_DRIVE_MIN_PROGRESS_PX 8
#define MOTOR_DRIVE_HARD_TIMEOUT_MS 1800U
#define MOTOR_DRIVE_OFFSET_COUNT 24
#define MOTOR_DRIVE_BOOST_OFFSET_COUNT 37
/* 机械安全确认前不启用第二级恢复角。 */
#define MOTOR_DRIVE_MAX_RECOVERY_OFFSET_COUNT 45
#define MOTOR_BRAKE_MIN_OFFSET_COUNT 22
#define MOTOR_BRAKE_MAX_OFFSET_COUNT 30
#define MOTOR_BRAKE_VELOCITY_DIVISOR 7
#define MOTOR_PHASE_ANGLE_TOLERANCE_COUNT 2
#define MOTOR_SETTLE_STABLE_FRAMES 3U
#define MOTOR_MAX_OFFSET_COUNT 171
#define MOTOR_POSITION_TOLERANCE_COUNT 1
#define MOTOR_TASK_SETTLE_ANGLE_TOLERANCE_COUNT 2
#define MOTOR_STEP_MIN_FREQUENCY_HZ 100U
#define MOTOR_STEP_MAX_FREQUENCY_HZ 600U
#define MOTOR_DAMP_STEP_MAX_FREQUENCY_HZ 150U
#define MOTOR_DAMP_RETURN_STEP_MAX_FREQUENCY_HZ 300U
#define MOTOR_STEP_FREQUENCY_QUANTUM_HZ 100U
#define MOTOR_STEP_FREQUENCY_GAIN_HZ_PER_COUNT 10U
#define MOTOR_STEP_RAMP_INCREMENT_HZ 100U
#define MOTOR_T34_STEP_MAX_FREQUENCY_HZ 1200U
#define MOTOR_T34_DAMP_STEP_MAX_FREQUENCY_HZ 500U
#define MOTOR_T34_DAMP_RETURN_STEP_MAX_FREQUENCY_HZ 800U
#define MOTOR_T34_STEP_FREQUENCY_GAIN_HZ_PER_COUNT 20U
#define MOTOR_T34_STEP_RAMP_INCREMENT_HZ 300U
#define MOTOR_T4_LAUNCH_STEP_MAX_FREQUENCY_HZ 2000U
#define MOTOR_T4_LAUNCH_STEP_FREQUENCY_MULTIPLIER 2U
#define MOTOR_T4_LAUNCH_STEP_RAMP_INCREMENT_HZ 600U
#define MOTOR_T4_FINISH_ACCELERATION_HOLD_US 800000LL

static const char *TAG = "app_motor";
static portMUX_TYPE s_state_lock = portMUX_INITIALIZER_UNLOCKED;
static app_motor_control_state_t s_state;

typedef struct {
    app_motor_control_phase_t phase;
    int8_t cycle_error_sign;
    int32_t drive_offset_count;
    int32_t brake_offset_count;
    int32_t damp_offset_count;
    int32_t level_offset_count;
    bool local_level_hold_active;
    int64_t angle_reached_at_us;
    uint8_t stable_frames;
    uint8_t drive_start_frames;
    int32_t damp_stable_error_px;
    int64_t damp_stable_started_at_us;
    int64_t drive_progress_started_at_us;
    int32_t drive_start_error_px;
    int32_t drive_best_error_px;
    uint8_t drive_boost_level;
    bool drive_fault;
    int32_t brake_peak_speed_px_s;
    bool brake_speed_initialized;
    bool brake_speed_declined;
    int32_t t3_output_count;
    int64_t t3_stall_started_at_us;
    int64_t t3_pulse_started_at_us;
    int64_t t3_pulse_refractory_until_us;
    int32_t t3_pulse_start_error_px;
    int32_t t3_pulse_span_count;
    int8_t t3_pulse_error_sign;
    uint8_t t3_pulse_attempts;
    bool t3_pulse_active;
    bool t3_pulse_braking;
    int32_t t4_integral_px_frames;
    int32_t t4_acceleration_feedforward_count;
    int64_t t4_static_stall_started_at_us;
    int64_t t4_static_pulse_until_us;
    int64_t t4_static_refractory_until_us;
    int32_t t4_static_pulse_offset_count;
} motor_outer_control_t;

static int32_t clamp_i32(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static int32_t absolute_i32(int32_t value)
{
    return value < 0 ? -value : value;
}

static bool timestamp_is_fresh(int64_t now_us, int64_t timestamp_us,
                               int64_t maximum_age_us)
{
    return timestamp_us > 0 && now_us >= timestamp_us &&
           now_us - timestamp_us <= maximum_age_us;
}

static uint32_t calculate_pwm_angle_mdeg(const bsp_encoder_snapshot_t *encoder)
{
    if (encoder->pwm_period_ticks == 0U || encoder->pwm_high_ticks == 0U ||
        encoder->pwm_high_ticks >= encoder->pwm_period_ticks) {
        return 0U;
    }

    uint32_t position_code = (uint32_t)(
        ((uint64_t)encoder->pwm_high_ticks * 4115ULL) /
        encoder->pwm_period_ticks);
    if (position_code > 0U) {
        position_code--;
    }
    if (position_code > 4114U) {
        position_code = 4114U;
    }
    return (uint32_t)(((uint64_t)position_code * 360000ULL) / 4115ULL);
}

static int8_t direction_sign(bsp_d36a_direction_t direction)
{
    return direction == BSP_D36A_DIRECTION_TOWARD_FRONT ? 1 : -1;
}

static uint32_t calculate_step_frequency_hz(int32_t position_error,
                                            bool fast_angle_control)
{
    uint32_t frequency_gain = fast_angle_control ?
        MOTOR_T34_STEP_FREQUENCY_GAIN_HZ_PER_COUNT :
        MOTOR_STEP_FREQUENCY_GAIN_HZ_PER_COUNT;
    uint32_t maximum_frequency_hz = fast_angle_control ?
        MOTOR_T34_STEP_MAX_FREQUENCY_HZ :
        MOTOR_STEP_MAX_FREQUENCY_HZ;
    uint32_t requested_hz = (uint32_t)absolute_i32(position_error) *
                            frequency_gain;
    if (requested_hz < MOTOR_STEP_MIN_FREQUENCY_HZ) {
        return MOTOR_STEP_MIN_FREQUENCY_HZ;
    }
    if (requested_hz >= maximum_frequency_hz) {
        return maximum_frequency_hz;
    }
    return ((requested_hz + MOTOR_STEP_FREQUENCY_QUANTUM_HZ - 1U) /
            MOTOR_STEP_FREQUENCY_QUANTUM_HZ) *
           MOTOR_STEP_FREQUENCY_QUANTUM_HZ;
}

static uint32_t ramp_step_frequency_hz(uint32_t requested_hz,
                                       uint32_t current_hz,
                                       bool fast_angle_control,
                                       bool transport_launch_angle_control)
{
    if (current_hz == 0U) {
        return requested_hz < MOTOR_STEP_MIN_FREQUENCY_HZ ?
            requested_hz : MOTOR_STEP_MIN_FREQUENCY_HZ;
    }
    if (requested_hz <= current_hz) {
        return requested_hz;
    }
    uint32_t ramp_increment_hz = transport_launch_angle_control ?
        MOTOR_T4_LAUNCH_STEP_RAMP_INCREMENT_HZ :
        (fast_angle_control ? MOTOR_T34_STEP_RAMP_INCREMENT_HZ :
                              MOTOR_STEP_RAMP_INCREMENT_HZ);
    uint32_t next_hz = current_hz + ramp_increment_hz;
    return next_hz < requested_hz ? next_hz : requested_hz;
}

static int32_t calculate_brake_compensation_px(int32_t velocity_px_s)
{
    int32_t limited_velocity = clamp_i32(
        velocity_px_s, -MOTOR_VISION_BRAKE_SPEED_LIMIT_PX_S,
        MOTOR_VISION_BRAKE_SPEED_LIMIT_PX_S);
    return (int32_t)(((int64_t)limited_velocity *
                      absolute_i32(limited_velocity)) /
                     (2LL * MOTOR_VISION_BRAKE_ACCEL_PX_S2));
}

static void outer_control_enter(motor_outer_control_t *control,
                                app_motor_control_phase_t phase)
{
    control->phase = phase;
    control->angle_reached_at_us = 0;
    control->stable_frames = 0U;
    control->drive_start_frames = 0U;
    control->damp_stable_started_at_us = 0;
    control->brake_peak_speed_px_s = 0;
    control->brake_speed_initialized = false;
    control->brake_speed_declined = false;
    if (phase == APP_MOTOR_CONTROL_PHASE_DRIVE ||
        phase == APP_MOTOR_CONTROL_PHASE_BRAKE) {
        control->local_level_hold_active = false;
    } else if (phase == APP_MOTOR_CONTROL_PHASE_TURN ||
               phase == APP_MOTOR_CONTROL_PHASE_DAMP ||
               phase == APP_MOTOR_CONTROL_PHASE_SETTLE) {
        control->local_level_hold_active = true;
    }
    if (phase == APP_MOTOR_CONTROL_PHASE_DRIVE) {
        control->drive_progress_started_at_us = 0;
        control->drive_start_error_px = 0;
        control->drive_best_error_px = 0;
        control->drive_boost_level = 0U;
    }
}

static void outer_control_reset(motor_outer_control_t *control)
{
    *control = (motor_outer_control_t){
        .phase = APP_MOTOR_CONTROL_PHASE_HOLD,
    };
}

static int32_t outer_control_offset(const motor_outer_control_t *control)
{
    if (control->phase == APP_MOTOR_CONTROL_PHASE_DRIVE) {
        return control->drive_offset_count;
    }
    if (control->phase == APP_MOTOR_CONTROL_PHASE_TURN) {
        return control->level_offset_count;
    }
    if (control->phase == APP_MOTOR_CONTROL_PHASE_BRAKE) {
        return control->cycle_error_sign * control->brake_offset_count;
    }
    if (control->phase == APP_MOTOR_CONTROL_PHASE_DAMP) {
        return control->damp_offset_count;
    }
    if (control->phase == APP_MOTOR_CONTROL_PHASE_SETTLE ||
        (control->phase == APP_MOTOR_CONTROL_PHASE_HOLD &&
         control->local_level_hold_active)) {
        return control->level_offset_count;
    }
    return 0;
}

static int32_t task_level_trim_count(
    const app_task_dispatch_state_t *dispatch)
{
    if (dispatch == NULL || dispatch->task_id != APP_TMX_TASK_T3) {
        return 0;
    }
    if (dispatch->phase == APP_TASK_DISPATCH_T3_POSITIVE) {
        return MOTOR_T3_POS_LEVEL_TRIM_COUNT;
    }
    if (dispatch->phase == APP_TASK_DISPATCH_T3_NEGATIVE ||
        dispatch->phase == APP_TASK_DISPATCH_COMPLETE) {
        return MOTOR_T3_NEG_LEVEL_TRIM_COUNT;
    }
    return 0;
}

static bool drive_start_ready(motor_outer_control_t *control,
                              bool new_control_sample,
                              int32_t error_magnitude)
{
    if (!new_control_sample) {
        return false;
    }
    if (error_magnitude <= MOTOR_DRIVE_START_ERROR_PX) {
        control->drive_start_frames = 0U;
        return false;
    }
    if (control->drive_start_frames < MOTOR_DRIVE_START_STABLE_FRAMES) {
        control->drive_start_frames++;
    }
    return control->drive_start_frames >= MOTOR_DRIVE_START_STABLE_FRAMES;
}

static void start_drive(motor_outer_control_t *control,
                        int32_t control_error_px,
                        int32_t drive_offset_count)
{
    control->cycle_error_sign = control_error_px > 0 ? 1 : -1;
    control->drive_offset_count =
        -control->cycle_error_sign * drive_offset_count;
    outer_control_enter(control, APP_MOTOR_CONTROL_PHASE_DRIVE);
}

static int32_t t4_launch_feedforward_count(uint32_t elapsed_ms)
{
    if (elapsed_ms < MOTOR_T4_LAUNCH_DAMP_MS) {
        return MOTOR_T4_LAUNCH_FEEDFORWARD_COUNT;
    }
    if (elapsed_ms < MOTOR_T4_LAUNCH_DAMP_MS +
                         MOTOR_T4_LAUNCH_FEEDFORWARD_FADE_MS) {
        uint32_t fade_remaining_ms =
            MOTOR_T4_LAUNCH_DAMP_MS +
            MOTOR_T4_LAUNCH_FEEDFORWARD_FADE_MS - elapsed_ms;
        return (int32_t)(MOTOR_T4_LAUNCH_FEEDFORWARD_COUNT *
                         fade_remaining_ms /
                         MOTOR_T4_LAUNCH_FEEDFORWARD_FADE_MS);
    }
    return 0;
}

static int32_t t4_acceleration_feedforward_target(int32_t acceleration_mg)
{
    if (absolute_i32(acceleration_mg) <=
        MOTOR_T4_ACCELERATION_DEADBAND_MG) {
        return 0;
    }
    int32_t target_count =
        acceleration_mg * MOTOR_T4_ACCELERATION_NUMERATOR /
        MOTOR_T4_ACCELERATION_DIVISOR;
    return clamp_i32(target_count,
                     -MOTOR_T4_ACCELERATION_MAX_OFFSET_COUNT,
                     MOTOR_T4_ACCELERATION_MAX_OFFSET_COUNT);
}

static void update_t4_acceleration_feedforward(
    motor_outer_control_t *control, bool imu_fresh,
    int32_t longitudinal_acceleration_mg)
{
    int32_t target_count = imu_fresh ?
        t4_acceleration_feedforward_target(
            longitudinal_acceleration_mg) : 0;
    int32_t delta_count = clamp_i32(
        target_count - control->t4_acceleration_feedforward_count,
        -MOTOR_T4_ACCELERATION_SLEW_COUNT_PER_STEP,
        MOTOR_T4_ACCELERATION_SLEW_COUNT_PER_STEP);
    control->t4_acceleration_feedforward_count += delta_count;
}

/* 固定起步与X轴动态前馈独立更新，视觉有效时再叠加PD和受限积分。 */
static int32_t update_t4_transport_control(
    motor_outer_control_t *control, int64_t now_us, uint32_t elapsed_ms,
    bool new_control_sample, int32_t control_error_px,
    int32_t control_velocity_px_s)
{
    int32_t launch_feedforward_count =
        t4_launch_feedforward_count(elapsed_ms);
    int32_t acceleration_feedforward_count =
        control->t4_acceleration_feedforward_count;
    int32_t feedback_maximum_offset =
        control->phase == APP_MOTOR_CONTROL_PHASE_BRAKE ?
            MOTOR_T4_BRAKE_MAX_OFFSET_COUNT :
            MOTOR_T4_MAX_OFFSET_COUNT;

    if (new_control_sample) {
        app_motor_control_phase_t phase =
            absolute_i32(control_velocity_px_s) >=
                    MOTOR_T4_BRAKE_PHASE_SPEED_PX_S ?
                APP_MOTOR_CONTROL_PHASE_BRAKE :
                APP_MOTOR_CONTROL_PHASE_DAMP;
        if (control->phase != phase) {
            outer_control_enter(control, phase);
        }
        feedback_maximum_offset =
            phase == APP_MOTOR_CONTROL_PHASE_BRAKE ?
                MOTOR_T4_BRAKE_MAX_OFFSET_COUNT :
                MOTOR_T4_MAX_OFFSET_COUNT;

        if (elapsed_ms >= MOTOR_T4_LAUNCH_DAMP_MS &&
            absolute_i32(control_velocity_px_s) <=
                MOTOR_T4_INTEGRAL_SPEED_LIMIT_PX_S &&
            absolute_i32(control_error_px) <=
                MOTOR_T4_INTEGRAL_ERROR_LIMIT_PX) {
            int32_t integral_input = clamp_i32(
                control_error_px, -MOTOR_T4_INTEGRAL_INPUT_LIMIT_PX,
                MOTOR_T4_INTEGRAL_INPUT_LIMIT_PX);
            control->t4_integral_px_frames = clamp_i32(
                control->t4_integral_px_frames + integral_input,
                -MOTOR_T4_INTEGRAL_LIMIT_PX_FRAMES,
                MOTOR_T4_INTEGRAL_LIMIT_PX_FRAMES);
        }

        int32_t velocity_divisor =
            elapsed_ms < MOTOR_T4_LAUNCH_DAMP_MS ?
                MOTOR_T4_LAUNCH_VELOCITY_DIVISOR :
                MOTOR_T4_VELOCITY_DIVISOR;
        bool static_friction_candidate =
            absolute_i32(control_error_px) >=
                MOTOR_T4_STATIC_FRICTION_ERROR_PX &&
            absolute_i32(control_velocity_px_s) <=
                MOTOR_T4_STATIC_FRICTION_SPEED_PX_S;
        bool pulse_active = control->t4_static_pulse_until_us > now_us;
        if (pulse_active && !static_friction_candidate) {
            control->t4_static_pulse_until_us = 0;
            control->t4_static_pulse_offset_count = 0;
            control->t4_static_refractory_until_us = now_us +
                MOTOR_T4_STATIC_FRICTION_REFRACTORY_US;
            pulse_active = false;
        }
        if (!pulse_active && static_friction_candidate &&
            now_us >= control->t4_static_refractory_until_us) {
            if (control->t4_static_stall_started_at_us == 0) {
                control->t4_static_stall_started_at_us = now_us;
            } else if (now_us - control->t4_static_stall_started_at_us >=
                       MOTOR_T4_STATIC_FRICTION_CONFIRM_US) {
                control->t4_static_pulse_offset_count =
                    control_error_px > 0 ?
                        -MOTOR_T4_STATIC_FRICTION_OFFSET_COUNT :
                        MOTOR_T4_STATIC_FRICTION_OFFSET_COUNT;
                control->t4_static_pulse_until_us = now_us +
                    MOTOR_T4_STATIC_FRICTION_PULSE_US;
                control->t4_static_stall_started_at_us = 0;
                pulse_active = true;
                ESP_LOGI(TAG,
                         "T4静摩擦脉冲: e=%ld v=%ld offset=%ld",
                         (long)control_error_px,
                         (long)control_velocity_px_s,
                         (long)control->t4_static_pulse_offset_count);
            }
        } else if (!static_friction_candidate) {
            control->t4_static_stall_started_at_us = 0;
        }
        int32_t static_friction_offset = pulse_active ?
            control->t4_static_pulse_offset_count : 0;
        int32_t requested_feedback_offset =
            -control_error_px / MOTOR_T4_POSITION_DIVISOR -
            control_velocity_px_s / velocity_divisor -
            control->t4_integral_px_frames /
                MOTOR_T4_INTEGRAL_DIVISOR +
            static_friction_offset;
        requested_feedback_offset = clamp_i32(
            requested_feedback_offset, -feedback_maximum_offset,
            feedback_maximum_offset);
        int32_t offset_delta = clamp_i32(
            requested_feedback_offset - control->damp_offset_count,
            -MOTOR_T4_SLEW_COUNT_PER_FRAME,
            MOTOR_T4_SLEW_COUNT_PER_FRAME);
        control->damp_offset_count += offset_delta;
    }

    if (control->t4_static_pulse_until_us > 0 &&
        now_us >= control->t4_static_pulse_until_us) {
        control->t4_static_pulse_until_us = 0;
        control->t4_static_pulse_offset_count = 0;
        control->t4_static_stall_started_at_us = 0;
        control->t4_static_refractory_until_us = now_us +
            MOTOR_T4_STATIC_FRICTION_REFRACTORY_US;
    }

    int32_t base_maximum_offset = launch_feedforward_count > 0 ?
        MOTOR_T4_LAUNCH_MAX_OFFSET_COUNT : feedback_maximum_offset;
    int32_t maximum_offset = clamp_i32(
        base_maximum_offset +
            absolute_i32(acceleration_feedforward_count),
        base_maximum_offset, MOTOR_T4_TOTAL_MAX_OFFSET_COUNT);
    int32_t combined_offset =
        control->level_offset_count + launch_feedforward_count +
        acceleration_feedforward_count + control->damp_offset_count;
    return clamp_i32(combined_offset, -maximum_offset, maximum_offset);
}

static int32_t t3_return_pd_offset(const motor_outer_control_t *control,
                                   int32_t error_px,
                                   int32_t velocity_px_s)
{
    return clamp_i32(
        control->level_offset_count -
            error_px / MOTOR_T3_RETURN_POSITION_DIVISOR -
            velocity_px_s / MOTOR_T3_RETURN_VELOCITY_DIVISOR,
        -MOTOR_T3_RETURN_MAX_OFFSET_COUNT,
        MOTOR_T3_RETURN_MAX_OFFSET_COUNT);
}

static int32_t t3_brake_span_count(int32_t velocity_px_s,
                                   bool negative_segment)
{
    int32_t minimum_offset_count = negative_segment ?
        MOTOR_T3_NEG_BRAKE_MIN_OFFSET_COUNT :
        MOTOR_T3_POS_BRAKE_MIN_OFFSET_COUNT;
    int32_t maximum_offset_count = negative_segment ?
        MOTOR_T3_NEG_BRAKE_MAX_OFFSET_COUNT :
        MOTOR_T3_POS_BRAKE_MAX_OFFSET_COUNT;
    return clamp_i32(
        absolute_i32(velocity_px_s) /
            MOTOR_T3_BRAKE_VELOCITY_DIVISOR,
        minimum_offset_count, maximum_offset_count);
}

static void start_t3_terminal_pulse(motor_outer_control_t *control,
                                    int64_t now_us,
                                    int32_t error_px)
{
    int32_t span_count = MOTOR_T3_PULSE_MIN_OFFSET_COUNT +
        (int32_t)control->t3_pulse_attempts *
            MOTOR_T3_PULSE_STEP_OFFSET_COUNT;
    control->t3_pulse_span_count = clamp_i32(
        span_count, MOTOR_T3_PULSE_MIN_OFFSET_COUNT,
        MOTOR_T3_PULSE_MAX_OFFSET_COUNT);
    control->t3_pulse_attempts++;
    control->t3_pulse_active = true;
    control->t3_pulse_braking = false;
    control->t3_pulse_started_at_us = now_us;
    control->t3_pulse_start_error_px = error_px;
    control->t3_pulse_error_sign = error_px >= 0 ? 1 : -1;
    control->t3_stall_started_at_us = 0;
    outer_control_enter(control, APP_MOTOR_CONTROL_PHASE_DRIVE);
    ESP_LOGI(TAG,
             "T3终点停滞，启动第%u次小幅脉冲: e=%ld span=%ld",
             (unsigned)control->t3_pulse_attempts,
             (long)error_px, (long)control->t3_pulse_span_count);
}

static int32_t update_t3_return_control(
    motor_outer_control_t *control, int64_t now_us,
    bool new_control_sample, int32_t error_px,
    int32_t velocity_px_s, int32_t actual_offset_count,
    int32_t settle_error_px, int32_t settle_speed_px_s)
{
    /* T3以编码器到角为边界：大DRIVE、TURN回水平，再翻到对称BRAKE。 */
    if (!control->t3_pulse_active && control->cycle_error_sign != 0 &&
        control->angle_reached_at_us == 0 &&
        (control->phase == APP_MOTOR_CONTROL_PHASE_DRIVE ||
         control->phase == APP_MOTOR_CONTROL_PHASE_TURN ||
         control->phase == APP_MOTOR_CONTROL_PHASE_BRAKE) &&
        absolute_i32(actual_offset_count - control->t3_output_count) <=
            MOTOR_PHASE_ANGLE_TOLERANCE_COUNT) {
        control->angle_reached_at_us = now_us;
    }
    if (!control->t3_pulse_active &&
        control->phase == APP_MOTOR_CONTROL_PHASE_DRIVE &&
        control->angle_reached_at_us > 0 &&
        now_us - control->angle_reached_at_us >=
            MOTOR_T3_DRIVE_PEAK_HOLD_US) {
        outer_control_enter(control, APP_MOTOR_CONTROL_PHASE_TURN);
        control->t3_output_count = control->level_offset_count;
    }
    if (!control->t3_pulse_active &&
        control->phase == APP_MOTOR_CONTROL_PHASE_TURN &&
        control->angle_reached_at_us > 0 &&
        now_us - control->angle_reached_at_us >=
            MOTOR_T3_TURN_LEVEL_HOLD_US) {
        outer_control_enter(control, APP_MOTOR_CONTROL_PHASE_BRAKE);
        int32_t brake_span_count =
            t3_brake_span_count(velocity_px_s, true);
        control->t3_output_count = clamp_i32(
            control->level_offset_count +
                control->cycle_error_sign * brake_span_count,
            -MOTOR_MAX_OFFSET_COUNT, MOTOR_MAX_OFFSET_COUNT);
    }

    if (!new_control_sample) {
        if (control->t3_pulse_active) {
            uint32_t pulse_elapsed_ms = (uint32_t)(
                (now_us - control->t3_pulse_started_at_us) / 1000LL);
            uint32_t pulse_timeout_ms = control->t3_pulse_braking ?
                MOTOR_T3_PULSE_BRAKE_MAX_MS :
                MOTOR_T3_PULSE_DRIVE_MAX_MS;
            if (pulse_elapsed_ms >= pulse_timeout_ms) {
                control->t3_pulse_active = false;
                control->t3_pulse_braking = false;
                control->t3_pulse_refractory_until_us = now_us +
                    (int64_t)MOTOR_T3_PULSE_REFRACTORY_MS * 1000LL;
                outer_control_enter(
                    control, APP_MOTOR_CONTROL_PHASE_DAMP);
                control->t3_output_count =
                    control->level_offset_count;
                ESP_LOGW(TAG, "T3终点脉冲因视觉未更新而超时回平");
            }
        }
        return control->t3_output_count;
    }

    if (control->cycle_error_sign == 0 ||
        control->phase == APP_MOTOR_CONTROL_PHASE_HOLD) {
        if (absolute_i32(error_px) < MOTOR_T3_RETURN_START_ERROR_PX) {
            control->t3_output_count = control->level_offset_count;
            return control->t3_output_count;
        }
        control->cycle_error_sign = error_px >= 0 ? 1 : -1;
        outer_control_enter(control, APP_MOTOR_CONTROL_PHASE_DRIVE);
        control->t3_output_count = clamp_i32(
            control->level_offset_count -
                control->cycle_error_sign * MOTOR_T3_LARGE_OFFSET_COUNT,
            -MOTOR_MAX_OFFSET_COUNT, MOTOR_MAX_OFFSET_COUNT);
        return control->t3_output_count;
    } else if (control->phase == APP_MOTOR_CONTROL_PHASE_SETTLE) {
        outer_control_enter(control, APP_MOTOR_CONTROL_PHASE_DAMP);
    }

    int32_t requested_pd_offset =
        t3_return_pd_offset(control, error_px, velocity_px_s);
    int32_t requested_relative_offset = clamp_i32(
        requested_pd_offset - control->level_offset_count,
        -MOTOR_T3_DAMP_MAX_OFFSET_COUNT,
        MOTOR_T3_DAMP_MAX_OFFSET_COUNT);
    int32_t requested_damp_offset = clamp_i32(
        control->level_offset_count + requested_relative_offset,
        -MOTOR_MAX_OFFSET_COUNT, MOTOR_MAX_OFFSET_COUNT);

    if (control->t3_pulse_active) {
        int32_t target_directed_speed_px_s =
            -control->t3_pulse_error_sign * velocity_px_s;
        uint32_t pulse_elapsed_ms = (uint32_t)(
            (now_us - control->t3_pulse_started_at_us) / 1000LL);
        if (!control->t3_pulse_braking) {
            int32_t progress_px =
                absolute_i32(control->t3_pulse_start_error_px) -
                absolute_i32(error_px);
            bool response_visible =
                progress_px >= MOTOR_T3_PULSE_RESPONSE_PX ||
                target_directed_speed_px_s >=
                    MOTOR_T3_PULSE_RESPONSE_SPEED_PX_S;
            bool drive_complete =
                pulse_elapsed_ms >= MOTOR_T3_PULSE_DRIVE_MIN_MS &&
                response_visible;
            bool drive_timed_out =
                pulse_elapsed_ms >= MOTOR_T3_PULSE_DRIVE_MAX_MS;
            if (!drive_complete && !drive_timed_out) {
                control->t3_output_count = clamp_i32(
                    control->level_offset_count -
                        control->t3_pulse_error_sign *
                            control->t3_pulse_span_count,
                    -MOTOR_T3_PULSE_OUTPUT_LIMIT_COUNT,
                    MOTOR_T3_PULSE_OUTPUT_LIMIT_COUNT);
                return control->t3_output_count;
            }
            control->t3_pulse_braking = true;
            control->t3_pulse_started_at_us = now_us;
            outer_control_enter(control, APP_MOTOR_CONTROL_PHASE_BRAKE);
            pulse_elapsed_ms = 0U;
        }

        int32_t brake_span_count = control->t3_pulse_span_count;
        control->t3_output_count = clamp_i32(
            control->level_offset_count +
                control->t3_pulse_error_sign * brake_span_count,
            -MOTOR_T3_PULSE_OUTPUT_LIMIT_COUNT,
            MOTOR_T3_PULSE_OUTPUT_LIMIT_COUNT);
        bool brake_reversed =
            target_directed_speed_px_s <=
                -MOTOR_T3_PULSE_REVERSE_SPEED_PX_S;
        bool brake_slow =
            absolute_i32(velocity_px_s) <=
                MOTOR_T3_PULSE_STATIONARY_SPEED_PX_S;
        bool minimum_brake_complete =
            pulse_elapsed_ms >= MOTOR_T3_PULSE_BRAKE_MIN_MS &&
            (brake_reversed || brake_slow);
        bool brake_timed_out =
            pulse_elapsed_ms >= MOTOR_T3_PULSE_BRAKE_MAX_MS;
        if (minimum_brake_complete || brake_timed_out) {
            control->t3_pulse_active = false;
            control->t3_pulse_braking = false;
            control->t3_pulse_refractory_until_us = now_us +
                (int64_t)MOTOR_T3_PULSE_REFRACTORY_MS * 1000LL;
            outer_control_enter(control, APP_MOTOR_CONTROL_PHASE_DAMP);
            control->t3_output_count = requested_damp_offset;
            ESP_LOGI(TAG,
                     "T3终点脉冲制动完成: e=%ld v=%ld",
                     (long)error_px, (long)velocity_px_s);
        }
        return control->t3_output_count;
    }

    if (control->phase == APP_MOTOR_CONTROL_PHASE_DRIVE ||
        control->phase == APP_MOTOR_CONTROL_PHASE_TURN) {
        return control->t3_output_count;
    }

    if (control->phase == APP_MOTOR_CONTROL_PHASE_BRAKE) {
        bool brake_angle_reached =
            control->angle_reached_at_us > 0;
        bool speed_reduced = absolute_i32(velocity_px_s) <=
            MOTOR_T3_BRAKE_DAMP_SWITCH_SPEED_PX_S;
        if (!brake_angle_reached || !speed_reduced) {
            return control->t3_output_count;
        }
        outer_control_enter(control, APP_MOTOR_CONTROL_PHASE_DAMP);
        control->t3_output_count = requested_damp_offset;
    }

    bool settle_candidate =
        absolute_i32(error_px) <= settle_error_px &&
        absolute_i32(velocity_px_s) <= settle_speed_px_s;
    if (settle_candidate) {
        requested_damp_offset = control->level_offset_count;
    }
    int32_t capture_delta = clamp_i32(
        requested_damp_offset - control->t3_output_count,
        -MOTOR_T3_DAMP_SLEW_COUNT_PER_FRAME,
        MOTOR_T3_DAMP_SLEW_COUNT_PER_FRAME);
    control->t3_output_count += capture_delta;

    bool outside_settle = absolute_i32(error_px) > settle_error_px;
    bool pulse_entry = absolute_i32(error_px) <=
        MOTOR_T3_PULSE_ENTRY_ERROR_PX;
    bool stationary =
        absolute_i32(velocity_px_s) <=
            MOTOR_T3_PULSE_STATIONARY_SPEED_PX_S;
    bool pulse_allowed =
        now_us >= control->t3_pulse_refractory_until_us &&
        control->t3_pulse_attempts < MOTOR_T3_PULSE_MAX_ATTEMPTS;
    if (outside_settle && pulse_entry && stationary && pulse_allowed) {
        if (control->t3_stall_started_at_us == 0) {
            control->t3_stall_started_at_us = now_us;
        } else if (now_us - control->t3_stall_started_at_us >=
                   MOTOR_T3_PULSE_STATIONARY_US) {
            start_t3_terminal_pulse(control, now_us, error_px);
            control->t3_output_count = clamp_i32(
                control->level_offset_count -
                    control->t3_pulse_error_sign *
                        control->t3_pulse_span_count,
                -MOTOR_T3_PULSE_OUTPUT_LIMIT_COUNT,
                MOTOR_T3_PULSE_OUTPUT_LIMIT_COUNT);
        }
    } else {
        control->t3_stall_started_at_us = 0;
    }
    return control->t3_output_count;
}

static uint32_t phase_elapsed_ms(const motor_outer_control_t *control,
                                 int64_t now_us)
{
    if (control->angle_reached_at_us <= 0 ||
        now_us <= control->angle_reached_at_us) {
        return 0U;
    }
    return (uint32_t)((now_us - control->angle_reached_at_us) / 1000LL);
}

static void motor_control_task(void *argument)
{
    (void)argument;
    TickType_t last_wake = xTaskGetTickCount();
    app_task_dispatch_state_t previous_dispatch = {
        .phase = APP_TASK_DISPATCH_IDLE,
    };
    bool previous_test_active = false;
    bool prep_handoff_pending = false;
    int64_t prep_handoff_deadline_us = 0;
    int64_t t4_finish_hold_started_at_us = 0;
    uint32_t active_step_frequency_hz = 0U;
    bsp_d36a_direction_t commanded_direction =
        BSP_D36A_DIRECTION_TOWARD_REAR;

    bool balance_capture_pending = false;
    int64_t balance_capture_started_at_us = 0;
    int64_t balance_stable_since_us = 0;
    int32_t balance_observed_encoder_count = 0;
    bool neutral_valid = false;
    int32_t neutral_count = 0;
    uint32_t balance_pwm_angle_mdeg = 0U;
    int32_t net_step_position = 0;
    uint32_t vision_pulse_count = 0U;

    uint32_t previous_vision_frame = 0U;
    uint32_t control_minimum_frame = 0U;
    int32_t previous_control_target_mm = 0;
    bool control_target_initialized = false;
    motor_outer_control_t outer_control;
    outer_control_reset(&outer_control);

    for (;;) {
        app_tmx_state_t tmx;
        app_vision_state_t vision;
        app_jy901s_state_t imu;
        bsp_encoder_snapshot_t encoder = {0};
        app_tmx_ingest_get_state(&tmx);
        app_vision_get_state(&vision);
        app_jy901s_get_state(&imu);
        bool encoder_valid =
            bsp_encoder_get_snapshot(&encoder) == ESP_OK;
        int64_t now_us = esp_timer_get_time();
        bool imu_control_fresh = imu.valid &&
            timestamp_is_fresh(now_us, imu.received_at_us,
                               MOTOR_IMU_CONTROL_FRESH_US);

        int32_t settle_level_trim_count =
            task_level_trim_count(&previous_dispatch);
        bool motor_near_level = encoder_valid && neutral_valid &&
            absolute_i32(encoder.ab_count - neutral_count -
                         settle_level_trim_count) <=
                MOTOR_TASK_SETTLE_ANGLE_TOLERANCE_COUNT;
        bool t3_outbound_braking =
            previous_dispatch.task_running &&
            previous_dispatch.task_id == APP_TMX_TASK_T3 &&
            previous_dispatch.phase == APP_TASK_DISPATCH_T3_POSITIVE &&
            outer_control.phase == APP_MOTOR_CONTROL_PHASE_BRAKE;
        app_task_dispatch_state_t dispatch;
        app_task_dispatch_step(&tmx, &vision, now_us, motor_near_level,
                               t3_outbound_braking,
                               &dispatch);
        int32_t level_trim_count = task_level_trim_count(&dispatch);

        bool same_task_instance = previous_dispatch.task_running &&
            dispatch.task_running &&
            previous_dispatch.task_id == dispatch.task_id &&
            previous_dispatch.run == dispatch.run;
        bool task_started = dispatch.task_running && !same_task_instance;
        bool task_ended = previous_dispatch.task_running &&
            !same_task_instance;
        bool t4_finish_hold_started = dispatch.task_running &&
            dispatch.task_id == APP_TMX_TASK_T4 &&
            dispatch.phase == APP_TASK_DISPATCH_COMPLETE &&
            (!previous_dispatch.task_running ||
             previous_dispatch.task_id != APP_TMX_TASK_T4 ||
             previous_dispatch.phase != APP_TASK_DISPATCH_COMPLETE);
        bool prep_started = dispatch.prep_active &&
            (!previous_dispatch.prep_active ||
             previous_dispatch.task_id != dispatch.task_id);
        bool motor_session_active = dispatch.motor_required &&
            (dispatch.prep_active || dispatch.task_running);
        bool previous_motor_session_active =
            previous_dispatch.motor_required &&
            (previous_dispatch.prep_active ||
             previous_dispatch.task_running);
        bool task_running = dispatch.task_running;
        bool test_active = tmx.mode_active &&
            strcmp(tmx.mode, "TEST") == 0;

        if (prep_handoff_pending &&
            (motor_session_active || test_active || task_started ||
             prep_started)) {
            prep_handoff_pending = false;
            prep_handoff_deadline_us = 0;
        }

        if (!control_target_initialized) {
            previous_control_target_mm = vision.target_mm;
            control_target_initialized = true;
        } else if (vision.target_mm != previous_control_target_mm) {
            previous_control_target_mm = vision.target_mm;
            control_minimum_frame = vision.frame_count + 1U;
            if (outer_control.drive_fault) {
                outer_control_enter(
                    &outer_control, APP_MOTOR_CONTROL_PHASE_SETTLE);
            } else {
                outer_control_reset(&outer_control);
            }
        }

        bool vision_frame_updated =
            vision.frame_count != previous_vision_frame;
        if (vision_frame_updated) {
            previous_vision_frame = vision.frame_count;
        }

        if (test_active && !previous_test_active) {
            balance_capture_pending = false;
            vision_pulse_count = 0U;
            active_step_frequency_hz = 0U;
            (void)bsp_d36a_stop_pulse();
            outer_control_reset(&outer_control);
            esp_err_t error = bsp_d36a_release();
            if (error == ESP_OK) {
                ESP_LOGI(TAG,
                         "进入TEST，仅采集编码器诊断；视觉控制关闭，D36A取消使能");
            } else {
                ESP_LOGE(TAG, "进入TEST时D36A取消使能失败: %s",
                         esp_err_to_name(error));
            }
        }

        if (!test_active && previous_test_active) {
            balance_capture_pending = false;
            active_step_frequency_hz = 0U;
            outer_control_reset(&outer_control);
            esp_err_t error = bsp_d36a_release();
            if (error == ESP_OK) {
                ESP_LOGI(TAG, "退出TEST，D36A已取消使能");
            } else {
                ESP_LOGE(TAG, "退出TEST时D36A取消使能失败: %s",
                         esp_err_to_name(error));
            }
        }

        if (prep_started && dispatch.motor_required) {
            esp_err_t error = bsp_d36a_hold();
            if (error == ESP_OK) {
                vision_pulse_count = 0U;
                active_step_frequency_hz = 0U;
                (void)bsp_d36a_stop_pulse();
                outer_control_reset(&outer_control);
                /* 使能吸合可能造成轻微位移，等待使能后的最终手动水平位置。 */
                balance_capture_pending = true;
                balance_capture_started_at_us = now_us;
                balance_stable_since_us = now_us;
                balance_observed_encoder_count =
                    encoder_valid ? encoder.ab_count : 0;
                neutral_valid = false;
                ESP_LOGI(TAG,
                         "进入%s PREP，D36A已使能且STEP关闭；等待手动调平后记录零点",
                         app_tmx_task_name(dispatch.task_id));
            } else {
                ESP_LOGE(TAG, "进入%s PREP时D36A使能失败: %s",
                         app_tmx_task_name(dispatch.task_id),
                         esp_err_to_name(error));
            }
        } else if (prep_started && !dispatch.motor_required) {
            active_step_frequency_hz = 0U;
            (void)bsp_d36a_stop_pulse();
            (void)bsp_d36a_release();
            outer_control_reset(&outer_control);
            ESP_LOGI(TAG, "进入%s PREP，滚球电机保持关闭",
                     app_tmx_task_name(dispatch.task_id));
        }

        if (balance_capture_pending && dispatch.prep_active && !test_active &&
            encoder_valid) {
            if (encoder.ab_count != balance_observed_encoder_count) {
                balance_observed_encoder_count = encoder.ab_count;
                balance_stable_since_us = now_us;
            }
            bool minimum_settle_elapsed =
                now_us - balance_capture_started_at_us >=
                    MOTOR_BALANCE_MIN_SETTLE_US;
            bool encoder_is_stable =
                now_us - balance_stable_since_us >=
                    MOTOR_BALANCE_STABLE_US;
            if (minimum_settle_elapsed && encoder_is_stable &&
                (!neutral_valid || encoder.ab_count != neutral_count)) {
                neutral_count = encoder.ab_count;
                balance_pwm_angle_mdeg = calculate_pwm_angle_mdeg(&encoder);
                net_step_position = 0;
                neutral_valid = true;
                ESP_LOGI(TAG,
                         "%s PREP手动水平零点已更新: ab=%ld pwmAngleMd=%lu",
                         app_tmx_task_name(dispatch.task_id),
                         (long)neutral_count,
                         (unsigned long)balance_pwm_angle_mdeg);
            }
        }

        if (task_ended && previous_dispatch.motor_required) {
            esp_err_t error = bsp_d36a_hold();
            active_step_frequency_hz = 0U;
            outer_control_reset(&outer_control);
            if (error == ESP_OK) {
                ESP_LOGI(TAG, "%s结束，停止STEP并保持当前位置锁定",
                         app_tmx_task_name(previous_dispatch.task_id));
            } else {
                ESP_LOGE(TAG, "%s结束时D36A锁定失败: %s",
                         app_tmx_task_name(previous_dispatch.task_id),
                         esp_err_to_name(error));
            }
        }

        if (task_started) {
            t4_finish_hold_started_at_us = 0;
            taskENTER_CRITICAL(&s_state_lock);
            s_state.task_latched = true;
            s_state.task_start_count++;
            taskEXIT_CRITICAL(&s_state_lock);

            if (dispatch.motor_required) {
                esp_err_t error = bsp_d36a_hold();
                if (error != ESP_OK) {
                    ESP_LOGE(TAG, "%s开始时D36A使能失败: %s",
                             app_tmx_task_name(dispatch.task_id),
                             esp_err_to_name(error));
                } else {
                    active_step_frequency_hz = 0U;
                    control_minimum_frame = dispatch.minimum_vision_frame;
                    outer_control_reset(&outer_control);
                    if (encoder_valid &&
                        (balance_capture_pending || !neutral_valid)) {
                        neutral_count = encoder.ab_count;
                        balance_pwm_angle_mdeg =
                            calculate_pwm_angle_mdeg(&encoder);
                        net_step_position = 0;
                        neutral_valid = true;
                        ESP_LOGI(TAG,
                                 "%s开始，冻结当前手动水平零点: ab=%ld pwmAngleMd=%lu",
                                 app_tmx_task_name(dispatch.task_id),
                                 (long)neutral_count,
                                 (unsigned long)balance_pwm_angle_mdeg);
                    } else if (!encoder_valid &&
                               balance_capture_pending) {
                        if (neutral_valid) {
                            ESP_LOGW(TAG,
                                     "%s开始时编码器瞬时无效，沿用PREP最后稳定零点: ab=%ld",
                                     app_tmx_task_name(dispatch.task_id),
                                     (long)neutral_count);
                        } else {
                            ESP_LOGE(TAG,
                                     "%s开始时仍无有效水平零点，位置控制保持关闭",
                                     app_tmx_task_name(dispatch.task_id));
                        }
                    }
                    balance_capture_pending = false;
                    ESP_LOGI(TAG, "%s开始，调度阶段=%s target=%ldmm",
                             app_tmx_task_name(dispatch.task_id),
                             app_task_dispatch_phase_name(dispatch.phase),
                             (long)dispatch.target_mm);
                }
            } else {
                active_step_frequency_hz = 0U;
                (void)bsp_d36a_stop_pulse();
                (void)bsp_d36a_release();
                outer_control_reset(&outer_control);
                ESP_LOGI(TAG, "%s开始，滚球控制旁路",
                         app_tmx_task_name(dispatch.task_id));
            }
        }

        if (dispatch.task_running &&
            dispatch.task_id == APP_TMX_TASK_T4 &&
            dispatch.phase == APP_TASK_DISPATCH_COMPLETE) {
            /* 停车回零不继承运输阶段学习到的稳态倾角。 */
            outer_control.t4_integral_px_frames = 0;
            if (t4_finish_hold_started) {
                t4_finish_hold_started_at_us = now_us;
                ESP_LOGI(TAG, "T4进入停车回零，已清除运输积分");
            }
        }

        if (!motor_session_active && previous_motor_session_active &&
            !task_ended) {
            active_step_frequency_hz = 0U;
            esp_err_t error = bsp_d36a_hold();
            outer_control_reset(&outer_control);
            if (error == ESP_OK) {
                prep_handoff_pending = true;
                prep_handoff_deadline_us =
                    now_us + MOTOR_PREP_HANDOFF_GRACE_US;
                ESP_LOGI(TAG,
                         "%s PREP退出，停止STEP并保持使能等待任务交接",
                         app_tmx_task_name(previous_dispatch.task_id));
            } else {
                prep_handoff_pending = false;
                prep_handoff_deadline_us = 0;
                ESP_LOGE(TAG, "%s PREP退出时D36A保持使能失败: %s",
                         app_tmx_task_name(previous_dispatch.task_id),
                         esp_err_to_name(error));
            }
        }

        if (prep_handoff_pending &&
            now_us >= prep_handoff_deadline_us &&
            !motor_session_active && !test_active) {
            prep_handoff_pending = false;
            prep_handoff_deadline_us = 0;
            active_step_frequency_hz = 0U;
            esp_err_t error = bsp_d36a_release();
            outer_control_reset(&outer_control);
            if (error == ESP_OK) {
                ESP_LOGI(TAG, "PREP交接等待超时，D36A已取消使能");
            } else {
                ESP_LOGE(TAG, "PREP交接超时取消使能失败: %s",
                         esp_err_to_name(error));
            }
        }

        bool vision_control_active = false;
        bool vision_fallback_active = false;
        bool vision_step_active = false;
        bool angle_limit_active = false;
        int32_t control_error_px = 0;
        int32_t brake_compensation_px = 0;
        int32_t predicted_stop_error_px = 0;
        int32_t encoder_target_count =
            encoder_valid ? encoder.ab_count : neutral_count;
        int8_t command_direction_state = 0;

        outer_control.level_offset_count = level_trim_count;

        if (encoder_valid && neutral_valid && motor_session_active &&
            !dispatch.prep_active && !test_active) {
            bool current_detection_fresh = vision.detected &&
                timestamp_is_fresh(now_us, vision.received_at_us,
                                   MOTOR_VISION_CONTROL_FRESH_US);
            bool last_detection_fresh = !vision.detected &&
                timestamp_is_fresh(now_us, vision.received_at_us,
                                   MOTOR_VISION_CONTROL_FRESH_US) &&
                timestamp_is_fresh(now_us, vision.last_detected_at_us,
                                   MOTOR_VISION_CONTROL_FRESH_US);

            bool centering_control_active =
                dispatch.vision_control_active;
            bool transport_control_active = dispatch.task_running &&
                (dispatch.task_id == APP_TMX_TASK_T4 ||
                 dispatch.task_id == APP_TMX_TASK_T5 ||
                 dispatch.task_id == APP_TMX_TASK_T6) &&
                centering_control_active;
            bool t3_return_active = dispatch.task_running &&
                dispatch.task_id == APP_TMX_TASK_T3 &&
                dispatch.phase == APP_TASK_DISPATCH_T3_NEGATIVE &&
                centering_control_active;
            bool t3_outbound_active = dispatch.task_running &&
                dispatch.task_id == APP_TMX_TASK_T3 &&
                dispatch.phase == APP_TASK_DISPATCH_T3_POSITIVE &&
                centering_control_active;
            bool fast_angle_control =
                (centering_control_active &&
                 dispatch.task_id == APP_TMX_TASK_T3) ||
                transport_control_active;
            int32_t initial_drive_offset_count = t3_outbound_active ?
                MOTOR_T3_LARGE_OFFSET_COUNT :
                MOTOR_DRIVE_OFFSET_COUNT;
            if (transport_control_active) {
                bool acceleration_feedforward_active =
                    dispatch.phase != APP_TASK_DISPATCH_COMPLETE ||
                    t4_finish_hold_started_at_us == 0 ||
                    now_us - t4_finish_hold_started_at_us <=
                        MOTOR_T4_FINISH_ACCELERATION_HOLD_US;
                update_t4_acceleration_feedforward(
                    &outer_control,
                    imu_control_fresh && acceleration_feedforward_active,
                    imu.longitudinal_acceleration_mg);
            } else {
                outer_control.t4_acceleration_feedforward_count = 0;
            }
            int32_t desired_offset_count = 0;
            if (transport_control_active) {
                desired_offset_count = update_t4_transport_control(
                    &outer_control, now_us, dispatch.elapsed_ms,
                    false, 0, 0);
            } else if (t3_return_active) {
                desired_offset_count = outer_control.t3_output_count;
            }
            bool new_target_frame_available =
                vision.frame_count >= control_minimum_frame &&
                vision.frame_count >= dispatch.minimum_vision_frame;
            if (centering_control_active && new_target_frame_available &&
                (current_detection_fresh || last_detection_fresh)) {
                vision_control_active = true;
                vision_fallback_active = last_detection_fresh;
                control_error_px = current_detection_fresh ?
                    vision.error_px : vision.last_error_px;
                if (!test_active &&
                    dispatch.task_id == APP_TMX_TASK_T6 &&
                    dispatch.reference_valid) {
                    control_error_px -= dispatch.reference_px;
                }

                int32_t error_magnitude =
                    absolute_i32((int32_t)control_error_px);
                int32_t control_velocity_px_s = clamp_i32(
                    vision.velocity_px_s,
                    -MOTOR_VISION_BRAKE_SPEED_LIMIT_PX_S,
                    MOTOR_VISION_BRAKE_SPEED_LIMIT_PX_S);
                brake_compensation_px =
                    calculate_brake_compensation_px(
                        control_velocity_px_s);
                uint32_t response_time_ms = 0U;
                if (outer_control.phase == APP_MOTOR_CONTROL_PHASE_DRIVE) {
                    response_time_ms = MOTOR_BRAKE_RESPONSE_MS;
                } else if (outer_control.phase ==
                           APP_MOTOR_CONTROL_PHASE_BRAKE) {
                    response_time_ms = MOTOR_LEVEL_RESPONSE_MS;
                }
                predicted_stop_error_px =
                    (int32_t)control_error_px +
                    (control_velocity_px_s *
                     (int32_t)response_time_ms) / 1000 +
                    brake_compensation_px;
                int32_t brake_switch_margin_px =
                    MOTOR_BRAKE_SWITCH_MARGIN_PX;
                if (!test_active &&
                    dispatch.task_id == APP_TMX_TASK_T3 &&
                    dispatch.phase == APP_TASK_DISPATCH_T3_NEGATIVE) {
                    brake_switch_margin_px =
                        MOTOR_T3_NEG_BRAKE_SWITCH_MARGIN_PX;
                }

                bool new_control_sample =
                    current_detection_fresh && vision_frame_updated;
                if (transport_control_active) {
                    desired_offset_count = update_t4_transport_control(
                        &outer_control, now_us, dispatch.elapsed_ms,
                        new_control_sample, control_error_px,
                        control_velocity_px_s);
                } else if (t3_return_active) {
                    desired_offset_count = update_t3_return_control(
                        &outer_control, now_us, new_control_sample,
                        control_error_px, control_velocity_px_s,
                        encoder.ab_count - neutral_count,
                        dispatch.params.settle_error_px,
                        dispatch.params.settle_speed_px_s);
                } else {
                desired_offset_count = outer_control_offset(&outer_control);
                int32_t phase_target_count =
                    neutral_count + desired_offset_count;
                if (outer_control.angle_reached_at_us == 0 &&
                    absolute_i32(encoder.ab_count - phase_target_count) <=
                        MOTOR_PHASE_ANGLE_TOLERANCE_COUNT) {
                    outer_control.angle_reached_at_us = now_us;
                    if (outer_control.phase ==
                            APP_MOTOR_CONTROL_PHASE_DRIVE &&
                        outer_control.drive_progress_started_at_us == 0) {
                        outer_control.drive_progress_started_at_us = now_us;
                        outer_control.drive_start_error_px = error_magnitude;
                        outer_control.drive_best_error_px = error_magnitude;
                    }
                }

                switch (outer_control.phase) {
                case APP_MOTOR_CONTROL_PHASE_HOLD:
                    if (!outer_control.drive_fault &&
                        drive_start_ready(&outer_control,
                                          new_control_sample,
                                          error_magnitude)) {
                        start_drive(&outer_control, control_error_px,
                                    initial_drive_offset_count);
                    }
                    break;

                case APP_MOTOR_CONTROL_PHASE_DRIVE:
                    if (t3_outbound_active &&
                        outer_control.angle_reached_at_us > 0 &&
                        now_us - outer_control.angle_reached_at_us >=
                            MOTOR_T3_DRIVE_PEAK_HOLD_US) {
                        outer_control_enter(
                            &outer_control,
                            APP_MOTOR_CONTROL_PHASE_TURN);
                        break;
                    }
                    /* 用预测停车误差补偿角度响应和制动距离，避免系统性晚刹。 */
                    if (outer_control.drive_progress_started_at_us > 0) {
                        uint32_t total_drive_ms = (uint32_t)(
                            (now_us -
                             outer_control.drive_progress_started_at_us) /
                            1000LL);
                        if (total_drive_ms >=
                            MOTOR_DRIVE_HARD_TIMEOUT_MS) {
                            outer_control.drive_fault = true;
                            outer_control_enter(
                                &outer_control,
                                APP_MOTOR_CONTROL_PHASE_SETTLE);
                            ESP_LOGW(TAG, "DRIVE超时，强制回平并锁定故障");
                            break;
                        }
                    }
                    if (!t3_outbound_active &&
                        outer_control.angle_reached_at_us > 0 &&
                        new_control_sample &&
                        outer_control.cycle_error_sign *
                            predicted_stop_error_px <=
                            brake_switch_margin_px) {
                        outer_control_enter(
                            &outer_control,
                            APP_MOTOR_CONTROL_PHASE_BRAKE);
                        outer_control.brake_offset_count = clamp_i32(
                            absolute_i32(control_velocity_px_s) /
                                MOTOR_BRAKE_VELOCITY_DIVISOR,
                            MOTOR_BRAKE_MIN_OFFSET_COUNT,
                            MOTOR_BRAKE_MAX_OFFSET_COUNT);
                        break;
                    }
                    if (new_control_sample &&
                        outer_control.drive_progress_started_at_us > 0) {
                        if (error_magnitude <
                            outer_control.drive_best_error_px) {
                            outer_control.drive_best_error_px =
                                error_magnitude;
                        }
                        uint32_t drive_ms =
                            phase_elapsed_ms(&outer_control, now_us);
                        int32_t progress_px =
                            outer_control.drive_start_error_px -
                            outer_control.drive_best_error_px;
                        if (drive_ms >= MOTOR_DRIVE_PROGRESS_WINDOW_MS &&
                            progress_px < MOTOR_DRIVE_MIN_PROGRESS_PX) {
                            if (outer_control.drive_boost_level == 0U) {
                                outer_control.drive_boost_level = 1U;
                                outer_control.drive_offset_count =
                                    -outer_control.cycle_error_sign *
                                    MOTOR_DRIVE_BOOST_OFFSET_COUNT;
                                outer_control.angle_reached_at_us = 0;
                                ESP_LOGW(
                                    TAG,
                                    "DRIVE进展不足，提升启动角: progress=%ldpx",
                                    (long)progress_px);
                            } else {
                                outer_control.drive_fault = true;
                                outer_control_enter(
                                    &outer_control,
                                    APP_MOTOR_CONTROL_PHASE_SETTLE);
                                ESP_LOGW(TAG,
                                         "DRIVE无进展，强制回平并锁定故障");
                            }
                        }
                    }
                    break;

                case APP_MOTOR_CONTROL_PHASE_TURN:
                    if (t3_outbound_active &&
                        outer_control.angle_reached_at_us > 0 &&
                        now_us - outer_control.angle_reached_at_us >=
                            MOTOR_T3_TURN_LEVEL_HOLD_US) {
                        outer_control_enter(
                            &outer_control,
                            APP_MOTOR_CONTROL_PHASE_BRAKE);
                        outer_control.brake_offset_count =
                            t3_brake_span_count(
                                control_velocity_px_s, false);
                    }
                    break;

                case APP_MOTOR_CONTROL_PHASE_BRAKE:
                    if (new_control_sample) {
                        int32_t toward_target_speed_px_s =
                            -outer_control.cycle_error_sign *
                            control_velocity_px_s;
                        if (!outer_control.brake_speed_initialized) {
                            outer_control.brake_peak_speed_px_s =
                                toward_target_speed_px_s;
                            outer_control.brake_speed_initialized = true;
                        } else if (toward_target_speed_px_s >
                                   outer_control.brake_peak_speed_px_s) {
                            outer_control.brake_peak_speed_px_s =
                                toward_target_speed_px_s;
                        } else if (outer_control.brake_peak_speed_px_s -
                                       toward_target_speed_px_s >=
                                   MOTOR_BRAKE_SPEED_DROP_PX_S) {
                            outer_control.brake_speed_declined = true;
                        }
                        int32_t actual_offset_count =
                            encoder.ab_count - neutral_count;
                        bool brake_direction_established =
                            outer_control.cycle_error_sign *
                                actual_offset_count >=
                            MOTOR_PHASE_ANGLE_TOLERANCE_COUNT;
                        int32_t predicted_level_speed_px_s =
                            toward_target_speed_px_s -
                            (MOTOR_VISION_BRAKE_ACCEL_PX_S2 *
                             MOTOR_LEVEL_RESPONSE_MS) / 1000;
                        bool t3_outbound_brake =
                            dispatch.task_id == APP_TMX_TASK_T3 &&
                            dispatch.phase ==
                                APP_TASK_DISPATCH_T3_POSITIVE;
                        bool brake_ready_for_damp =
                            brake_direction_established &&
                            outer_control.brake_speed_declined &&
                            (t3_outbound_brake ?
                                absolute_i32(toward_target_speed_px_s) <=
                                    MOTOR_T3_BRAKE_DAMP_SWITCH_SPEED_PX_S :
                                predicted_level_speed_px_s <=
                                    MOTOR_BRAKE_DAMP_SWITCH_SPEED_PX_S);
                        if (brake_ready_for_damp) {
                            outer_control_enter(
                                &outer_control,
                                APP_MOTOR_CONTROL_PHASE_DAMP);
                            if (t3_outbound_brake) {
                                int32_t damp_minimum_offset =
                                    outer_control.level_offset_count -
                                        MOTOR_T3_DAMP_MAX_OFFSET_COUNT;
                                int32_t damp_maximum_offset =
                                    outer_control.level_offset_count +
                                        MOTOR_T3_DAMP_MAX_OFFSET_COUNT;
                                outer_control.damp_offset_count = clamp_i32(
                                    outer_control.level_offset_count -
                                        control_error_px /
                                            MOTOR_DAMP_POSITION_DIVISOR -
                                        control_velocity_px_s /
                                            MOTOR_DAMP_VELOCITY_DIVISOR,
                                    damp_minimum_offset,
                                    damp_maximum_offset);
                            } else {
                                outer_control.damp_offset_count = clamp_i32(
                                    outer_control.level_offset_count -
                                    control_error_px /
                                        MOTOR_DAMP_POSITION_DIVISOR -
                                    control_velocity_px_s /
                                        MOTOR_DAMP_VELOCITY_DIVISOR,
                                    -MOTOR_DAMP_MAX_OFFSET_COUNT,
                                    MOTOR_DAMP_MAX_OFFSET_COUNT);
                            }
                            outer_control.damp_stable_error_px =
                                control_error_px;
                        }
                    }
                    break;

                case APP_MOTOR_CONTROL_PHASE_DAMP:
                    if (new_control_sample) {
                        /* 位置项负责拉回目标，速度项负责抑制穿越目标时的惯性。 */
                        bool t3_outbound_damp =
                            dispatch.task_id == APP_TMX_TASK_T3 &&
                            dispatch.phase ==
                                APP_TASK_DISPATCH_T3_POSITIVE;
                        int32_t damp_minimum_offset =
                            t3_outbound_damp ?
                                outer_control.level_offset_count -
                                    MOTOR_T3_DAMP_MAX_OFFSET_COUNT :
                                -MOTOR_DAMP_MAX_OFFSET_COUNT;
                        int32_t damp_maximum_offset =
                            t3_outbound_damp ?
                                outer_control.level_offset_count +
                                    MOTOR_T3_DAMP_MAX_OFFSET_COUNT :
                                MOTOR_DAMP_MAX_OFFSET_COUNT;
                        int32_t requested_damp_offset = clamp_i32(
                            outer_control.level_offset_count -
                            control_error_px /
                                MOTOR_DAMP_POSITION_DIVISOR -
                            control_velocity_px_s /
                                MOTOR_DAMP_VELOCITY_DIVISOR,
                            damp_minimum_offset,
                            damp_maximum_offset);
                        bool recovery_ready =
                            error_magnitude >= MOTOR_DAMP_RECOVERY_ERROR_PX &&
                            absolute_i32(control_velocity_px_s) <=
                                MOTOR_DAMP_RECOVERY_SPEED_PX_S;
                        if (recovery_ready) {
                            requested_damp_offset = control_error_px > 0 ?
                                damp_minimum_offset :
                                damp_maximum_offset;
                        }

                        int32_t damp_slew_count = t3_outbound_damp ?
                            MOTOR_T3_DAMP_SLEW_COUNT_PER_FRAME :
                            MOTOR_DAMP_SLEW_COUNT_PER_FRAME;
                        int32_t damp_delta = clamp_i32(
                            requested_damp_offset -
                                outer_control.damp_offset_count,
                            -damp_slew_count,
                            damp_slew_count);
                        outer_control.damp_offset_count += damp_delta;

                        int32_t actual_offset_count =
                            encoder.ab_count - neutral_count;
                        bool velocity_stable =
                            absolute_i32(control_velocity_px_s) <=
                            MOTOR_DAMP_SETTLE_SPEED_PX_S;
                        bool beam_near_level =
                            absolute_i32(
                                actual_offset_count -
                                outer_control.level_offset_count) <=
                            MOTOR_PHASE_ANGLE_TOLERANCE_COUNT;
                        bool position_stable =
                            absolute_i32(
                                control_error_px -
                                outer_control.damp_stable_error_px) <=
                            MOTOR_DAMP_POSITION_STABLE_PX;

                        if (velocity_stable && beam_near_level &&
                            position_stable) {
                            if (outer_control.damp_stable_started_at_us == 0) {
                                outer_control.damp_stable_started_at_us = now_us;
                            }
                            if (outer_control.stable_frames <
                                MOTOR_DAMP_STABLE_FRAMES) {
                                outer_control.stable_frames++;
                            }
                        } else {
                            outer_control.stable_frames = 0U;
                            outer_control.damp_stable_started_at_us = 0;
                            outer_control.damp_stable_error_px =
                                control_error_px;
                        }
                        if (!t3_outbound_damp &&
                            outer_control.stable_frames >=
                                MOTOR_DAMP_STABLE_FRAMES &&
                            outer_control.damp_stable_started_at_us > 0 &&
                            now_us -
                                    outer_control.damp_stable_started_at_us >=
                                (int64_t)MOTOR_DAMP_MIN_STABLE_MS * 1000LL) {
                            outer_control_enter(
                                &outer_control,
                                APP_MOTOR_CONTROL_PHASE_SETTLE);
                        }
                    }
                    break;

                case APP_MOTOR_CONTROL_PHASE_SETTLE:
                    if (outer_control.angle_reached_at_us > 0 &&
                        new_control_sample) {
                        if (outer_control.drive_fault) {
                            break;
                        }
                        if (absolute_i32(control_velocity_px_s) <=
                            MOTOR_SETTLE_SPEED_PX_S) {
                            if (outer_control.stable_frames <
                                MOTOR_SETTLE_STABLE_FRAMES) {
                                outer_control.stable_frames++;
                            }
                        } else {
                            outer_control.stable_frames = 0U;
                            outer_control.drive_start_frames = 0U;
                        }

                        if (outer_control.stable_frames >=
                            MOTOR_SETTLE_STABLE_FRAMES) {
                            if (error_magnitude <=
                                MOTOR_SETTLE_POSITION_PX) {
                                outer_control_enter(
                                    &outer_control,
                                    APP_MOTOR_CONTROL_PHASE_HOLD);
                            } else if (drive_start_ready(
                                           &outer_control,
                                           true,
                                           error_magnitude)) {
                                start_drive(&outer_control,
                                            control_error_px,
                                            initial_drive_offset_count);
                            }
                        }
                    }
                    break;
                }

                desired_offset_count = outer_control_offset(&outer_control);
                }
                int32_t limited_offset_count = clamp_i32(
                    desired_offset_count, -MOTOR_MAX_OFFSET_COUNT,
                    MOTOR_MAX_OFFSET_COUNT);
                angle_limit_active =
                    limited_offset_count != desired_offset_count;
                desired_offset_count = limited_offset_count;
            } else {
                if (transport_control_active &&
                    (current_detection_fresh || last_detection_fresh)) {
                    control_error_px = current_detection_fresh ?
                        vision.error_px : vision.last_error_px;
                    vision_fallback_active = last_detection_fresh;
                }
                if (transport_control_active) {
                    /* 视觉无效时冻结反馈，继续使用独立更新的加速度前馈。 */
                    desired_offset_count = update_t4_transport_control(
                        &outer_control, now_us, dispatch.elapsed_ms,
                        false, 0, 0);
                } else if (t3_return_active) {
                    desired_offset_count = update_t3_return_control(
                        &outer_control, now_us, false, 0, 0,
                        encoder.ab_count - neutral_count,
                        dispatch.params.settle_error_px,
                        dispatch.params.settle_speed_px_s);
                } else if (outer_control.drive_fault) {
                    outer_control_enter(
                        &outer_control, APP_MOTOR_CONTROL_PHASE_SETTLE);
                } else {
                    outer_control_reset(&outer_control);
                }
            }

            encoder_target_count = neutral_count + desired_offset_count;
            int32_t position_error =
                encoder_target_count - encoder.ab_count;
            if (absolute_i32(position_error) >
                MOTOR_POSITION_TOLERANCE_COUNT) {
                bsp_d36a_direction_t desired_direction =
                    position_error > 0 ?
                        BSP_D36A_DIRECTION_TOWARD_FRONT :
                        BSP_D36A_DIRECTION_TOWARD_REAR;
                int8_t desired_sign = direction_sign(desired_direction);
                uint32_t requested_frequency_hz =
                    calculate_step_frequency_hz(
                        position_error, fast_angle_control);
                bool transport_launch_angle_control =
                    transport_control_active &&
                    dispatch.elapsed_ms < MOTOR_T4_LAUNCH_DAMP_MS;
                if (transport_launch_angle_control &&
                    requested_frequency_hz > 0U) {
                    requested_frequency_hz *=
                        MOTOR_T4_LAUNCH_STEP_FREQUENCY_MULTIPLIER;
                    if (requested_frequency_hz >
                        MOTOR_T4_LAUNCH_STEP_MAX_FREQUENCY_HZ) {
                        requested_frequency_hz =
                            MOTOR_T4_LAUNCH_STEP_MAX_FREQUENCY_HZ;
                    }
                }
                if ((outer_control.phase ==
                          APP_MOTOR_CONTROL_PHASE_DAMP ||
                      outer_control.phase ==
                          APP_MOTOR_CONTROL_PHASE_SETTLE) &&
                    requested_frequency_hz > 0U) {
                    int32_t actual_offset_count =
                        encoder.ab_count - neutral_count;
                    int32_t actual_level_error_count =
                        actual_offset_count -
                        outer_control.level_offset_count;
                    bool returning_toward_level =
                        (actual_level_error_count > 0 &&
                         position_error < 0) ||
                        (actual_level_error_count < 0 &&
                         position_error > 0);
                    uint32_t damp_frequency_limit_hz;
                    if (fast_angle_control) {
                        damp_frequency_limit_hz =
                            returning_toward_level ?
                                MOTOR_T34_DAMP_RETURN_STEP_MAX_FREQUENCY_HZ :
                                MOTOR_T34_DAMP_STEP_MAX_FREQUENCY_HZ;
                    } else {
                        damp_frequency_limit_hz =
                            returning_toward_level ?
                                MOTOR_DAMP_RETURN_STEP_MAX_FREQUENCY_HZ :
                                MOTOR_DAMP_STEP_MAX_FREQUENCY_HZ;
                    }
                    if (requested_frequency_hz > damp_frequency_limit_hz) {
                        requested_frequency_hz = damp_frequency_limit_hz;
                    }
                }
                if (commanded_direction != desired_direction) {
                    esp_err_t error = bsp_d36a_stop_pulse();
                    active_step_frequency_hz = 0U;
                    if (error == ESP_OK) {
                        error = bsp_d36a_set_direction(desired_direction);
                    }
                    if (error == ESP_OK) {
                        commanded_direction = desired_direction;
                    } else {
                        ESP_LOGE(TAG, "视觉闭环DIR设置失败: %s",
                                 esp_err_to_name(error));
                    }
                } else {
                    uint32_t output_frequency_hz =
                        ramp_step_frequency_hz(
                            requested_frequency_hz,
                            active_step_frequency_hz,
                            fast_angle_control,
                            transport_launch_angle_control);
                    esp_err_t error =
                        bsp_d36a_start_pulse(output_frequency_hz);
                    if (error == ESP_OK) {
                        uint32_t commanded_pulses =
                            output_frequency_hz /
                            (1000U / MOTOR_CONTROL_PERIOD_MS);
                        active_step_frequency_hz = output_frequency_hz;
                        vision_step_active = true;
                        command_direction_state = desired_sign;
                        net_step_position +=
                            desired_sign * (int32_t)commanded_pulses;
                        vision_pulse_count += commanded_pulses;
                    } else {
                        active_step_frequency_hz = 0U;
                        ESP_LOGE(TAG, "视觉闭环STEP输出失败: %s",
                                 esp_err_to_name(error));
                    }
                }
            } else {
                active_step_frequency_hz = 0U;
                (void)bsp_d36a_stop_pulse();
            }
        } else {
            active_step_frequency_hz = 0U;
            (void)bsp_d36a_stop_pulse();
        }

        taskENTER_CRITICAL(&s_state_lock);
        s_state.enabled = bsp_d36a_is_enabled();
        s_state.test_centering_active = false;
        s_state.test_balance_ready = false;
        s_state.vision_control_active = vision_control_active;
        s_state.vision_fallback_active = vision_fallback_active;
        s_state.vision_step_active = vision_step_active;
        s_state.angle_limit_active = angle_limit_active;
        s_state.task_running = task_running;
        s_state.neutral_valid = neutral_valid;
        s_state.phase_angle_reached =
            outer_control.angle_reached_at_us > 0;
        s_state.drive_fault = outer_control.drive_fault;
        s_state.task_complete = !test_active && dispatch.task_complete;
        s_state.task_fault = !test_active && dispatch.task_fault;
        s_state.reference_valid = !test_active && dispatch.reference_valid;
        s_state.t3_pulse_active = outer_control.t3_pulse_active;
        s_state.drive_boost_level = outer_control.drive_boost_level;
        s_state.t3_pulse_attempts = outer_control.t3_pulse_attempts;
        s_state.task_id = dispatch.task_id;
        s_state.dispatch_phase = dispatch.phase;
        s_state.control_phase = outer_control.phase;
        s_state.control_error_px = control_error_px;
        s_state.reference_px = test_active ? 0 : dispatch.reference_px;
        s_state.estimated_velocity_px_s = vision.velocity_px_s;
        s_state.brake_compensation_px = brake_compensation_px;
        s_state.predicted_stop_error_px = predicted_stop_error_px;
        s_state.t4_acceleration_feedforward_count =
            outer_control.t4_acceleration_feedforward_count;
        s_state.control_target_mm =
            !test_active && dispatch.task_id != APP_TMX_TASK_NONE ?
                dispatch.target_mm : vision.target_mm;
        s_state.encoder_count =
            encoder_valid ? encoder.ab_count : s_state.encoder_count;
        s_state.encoder_target_count = encoder_target_count;
        s_state.neutral_count = neutral_count;
        s_state.balance_pwm_angle_mdeg = balance_pwm_angle_mdeg;
        s_state.net_step_position = net_step_position;
        s_state.command_direction = command_direction_state;
        s_state.step_frequency_hz = active_step_frequency_hz;
        s_state.vision_pulse_count = vision_pulse_count;
        s_state.control_frame_count = vision.frame_count;
        s_state.phase_elapsed_ms =
            phase_elapsed_ms(&outer_control, now_us);
        s_state.task_run = test_active ? 0U : dispatch.run;
        s_state.task_elapsed_ms = test_active ? 0U : dispatch.elapsed_ms;
        taskEXIT_CRITICAL(&s_state_lock);

        previous_dispatch = dispatch;
        previous_test_active = test_active;
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MOTOR_CONTROL_PERIOD_MS));
    }
}

esp_err_t app_motor_control_init(void)
{
    s_state = (app_motor_control_state_t){0};
    esp_err_t error = bsp_d36a_init();
    if (error != ESP_OK) {
        return error;
    }

    if (xTaskCreatePinnedToCore(motor_control_task, "motor_control",
                                MOTOR_CONTROL_STACK_SIZE, NULL,
                                MOTOR_CONTROL_PRIORITY, NULL,
                                APP_CORE_CONTROL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG,
             "D36A控制任务已启动: Core=%d STEP=18(LEDC换向升频斜坡) DIR=8 EN=3；HOLD/DRIVE/TURN/BRAKE/DAMP/SETTLE闭环已启用",
             APP_CORE_CONTROL);
    return ESP_OK;
}

void app_motor_control_get_state(app_motor_control_state_t *state)
{
    if (state == NULL) {
        return;
    }
    taskENTER_CRITICAL(&s_state_lock);
    *state = s_state;
    taskEXIT_CRITICAL(&s_state_lock);
}
