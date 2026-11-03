#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "app_core_affinity.h"
#include "app_jy901s.h"
#include "app_motor_control.h"
#include "app_system.h"
#include "app_tmx_ingest.h"
#include "app_udp_log.h"
#include "app_vision.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MAIN_MONITOR_PERIOD_MS 20U
#define MAIN_STATUS_LOG_DIVIDER 100U
#define MAIN_VISION_CONTROL_STALE_MS 1000U
#define MAIN_MONITOR_STACK_SIZE 8192U
#define MAIN_MONITOR_PRIORITY 2U
#define MAIN_CTRL_SAMPLE_SIZE 512U
#define MAIN_TEST_SAMPLE_SIZE 256U
#define MAIN_IMU_STALE_MS 200U

static const char *TAG = "main";
static const char *MSP_TAG = "MSP";

static const char *motor_state_name(const app_motor_control_state_t *motor)
{
    if (motor->test_centering_active) {
        if (!motor->neutral_valid) {
            return "TEST_WAIT";
        }
        if (!motor->test_balance_ready) {
            return "TEST_BALANCE";
        }
        if (motor->vision_step_active) {
            return "TEST_CENTER";
        }
        if (motor->vision_control_active) {
            return motor->vision_fallback_active ?
                "TEST_LAST" : "TEST_HOLD";
        }
        return "TEST_SAFE";
    }
    if (motor->vision_step_active) {
        return motor->task_running ? "VISION_TRACK" : "BALANCE_RETURN";
    }
    if (motor->vision_control_active) {
        return motor->vision_fallback_active ?
            "VISION_LAST" : "VISION_HOLD";
    }
    return motor->enabled ? "HOLD" : "OFF";
}

static const char *control_phase_name(app_motor_control_phase_t phase)
{
    switch (phase) {
    case APP_MOTOR_CONTROL_PHASE_DRIVE:
        return "DRIVE";
    case APP_MOTOR_CONTROL_PHASE_TURN:
        return "TURN";
    case APP_MOTOR_CONTROL_PHASE_BRAKE:
        return "BRAKE";
    case APP_MOTOR_CONTROL_PHASE_DAMP:
        return "DAMP";
    case APP_MOTOR_CONTROL_PHASE_SETTLE:
        return "SETTLE";
    case APP_MOTOR_CONTROL_PHASE_HOLD:
    default:
        return "HOLD";
    }
}

static const char *telemetry_task_name(const app_motor_control_state_t *motor)
{
    return motor->test_centering_active ?
        "TEST" : app_tmx_task_name(motor->task_id);
}

static const char *telemetry_stage_name(const app_motor_control_state_t *motor)
{
    return motor->test_centering_active ?
        "TEST" : app_task_dispatch_phase_name(motor->dispatch_phase);
}

static const char *motor_direction_name(const app_motor_control_state_t *motor)
{
    if (motor->command_direction > 0) {
        return "FRONT";
    }
    if (motor->command_direction < 0) {
        return "REAR";
    }
    return "STOP";
}

static char control_source_code(const app_motor_control_state_t *motor)
{
    if (motor->vision_fallback_active) {
        return 'L';
    }
    return motor->vision_control_active ? 'C' : 'S';
}

static char motor_direction_code(const app_motor_control_state_t *motor)
{
    if (motor->command_direction > 0) {
        return 'F';
    }
    if (motor->command_direction < 0) {
        return 'R';
    }
    return 'S';
}

static uint32_t vision_age_ms(const app_vision_state_t *vision)
{
    if (vision->received_at_us <= 0) {
        return UINT32_MAX;
    }

    int64_t age_us = esp_timer_get_time() - vision->received_at_us;
    if (age_us <= 0) {
        return 0U;
    }
    uint64_t age_ms = (uint64_t)age_us / 1000U;
    return age_ms > UINT32_MAX ? UINT32_MAX : (uint32_t)age_ms;
}

static uint32_t imu_age_ms(const app_jy901s_state_t *imu)
{
    if (imu->received_at_us <= 0) {
        return UINT32_MAX;
    }
    int64_t age_us = esp_timer_get_time() - imu->received_at_us;
    if (age_us <= 0) {
        return 0U;
    }
    uint64_t age_ms = (uint64_t)age_us / 1000U;
    return age_ms > UINT32_MAX ? UINT32_MAX : (uint32_t)age_ms;
}

static void log_control_sample(const app_motor_control_state_t *motor,
                               const app_jy901s_state_t *imu,
                               uint32_t age_ms, bool new_vision_frame)
{
    int32_t angle_target_count =
        motor->encoder_target_count - motor->neutral_count;
    int32_t angle_actual_count =
        motor->encoder_count - motor->neutral_count;
    uint32_t timestamp_ms =
        (uint32_t)((uint64_t)esp_timer_get_time() / 1000ULL);
    uint32_t current_imu_age_ms = imu_age_ms(imu);
    bool imu_fresh = imu->valid &&
        current_imu_age_ms <= MAIN_IMU_STALE_MS;

    char line[MAIN_CTRL_SAMPLE_SIZE];
    int length = snprintf(
        line, sizeof(line),
        "@CTRL t=%" PRIu32 " task=%s run=%" PRIu32
        " stage=%s elapsed=%" PRIu32
        " done=%u tflt=%u ref=%" PRId32 " refOk=%u"
        " frame=%" PRIu32 " new=%u tgt=%" PRId32
        " src=%c age=%" PRIu32 " ph=%s ms=%" PRIu32
        " e=%d v=%" PRId32 " bd=%" PRId32 " sp=%" PRId32
        " cmd=%" PRId32 " act=%" PRId32 " hit=%u z=%u"
        " hz=%" PRIu32 " dir=%c lim=%u bst=%u flt=%u"
        " t3p=%u t3pa=%u"
        " imu=%u imuAge=%" PRIu32 " imuSeq=%" PRIu32
        " ax=%" PRId32 " ay=%" PRId32 " az=%" PRId32
        " aLong=%" PRId32 " aff=%" PRId32,
        timestamp_ms, telemetry_task_name(motor), motor->task_run,
        telemetry_stage_name(motor),
        motor->task_elapsed_ms,
        motor->task_complete ? 1U : 0U,
        motor->task_fault ? 1U : 0U,
        motor->reference_px,
        motor->reference_valid ? 1U : 0U,
        motor->control_frame_count,
        new_vision_frame ? 1U : 0U, motor->control_target_mm,
        control_source_code(motor), age_ms,
        control_phase_name(motor->control_phase),
        motor->phase_elapsed_ms,
        (int)motor->control_error_px,
        motor->estimated_velocity_px_s,
        motor->brake_compensation_px,
        motor->predicted_stop_error_px,
        angle_target_count, angle_actual_count,
        motor->phase_angle_reached ? 1U : 0U,
        motor->neutral_valid ? 1U : 0U,
        motor->step_frequency_hz,
        motor_direction_code(motor),
        motor->angle_limit_active ? 1U : 0U,
        (unsigned)motor->drive_boost_level,
        motor->drive_fault ? 1U : 0U,
        motor->t3_pulse_active ? 1U : 0U,
        (unsigned)motor->t3_pulse_attempts,
        imu_fresh ? 1U : 0U,
        current_imu_age_ms,
        imu->sample_count,
        imu->acceleration_x_mg,
        imu->acceleration_y_mg,
        imu->acceleration_z_mg,
        imu->longitudinal_acceleration_mg,
        motor->t4_acceleration_feedforward_count);
    if (length <= 0 || length >= (int)sizeof(line)) {
        ESP_LOGW(TAG, "CTRL调参样本编码失败");
        return;
    }
    ESP_LOGI(MSP_TAG, "%s", line);
    (void)app_udp_log_publish(line, (size_t)length);
}

static void log_test_sample(const app_vision_state_t *vision,
                            const app_jy901s_state_t *imu,
                            uint32_t age_ms, bool new_vision_frame)
{
    uint32_t timestamp_ms =
        (uint32_t)((uint64_t)esp_timer_get_time() / 1000ULL);
    uint32_t current_imu_age_ms = imu_age_ms(imu);
    bool vision_fresh = vision->detected &&
        age_ms <= MAIN_VISION_CONTROL_STALE_MS;
    bool imu_fresh = imu->valid &&
        current_imu_age_ms <= MAIN_IMU_STALE_MS;

    char line[MAIN_TEST_SAMPLE_SIZE];
    int length = snprintf(
        line, sizeof(line),
        "@TEST t=%" PRIu32 " task=TEST frame=%" PRIu32
        " new=%u vis=%u age=%" PRIu32
        " epx=%d v=%" PRId32
        " imu=%u imuAge=%" PRIu32 " imuSeq=%" PRIu32
        " imuErr=%" PRIu32 " ax=%" PRId32
        " ay=%" PRId32 " az=%" PRId32
        " aLong=%" PRId32,
        timestamp_ms, vision->frame_count,
        new_vision_frame ? 1U : 0U,
        vision_fresh ? 1U : 0U, age_ms,
        (int)vision->error_px, vision->velocity_px_s,
        imu_fresh ? 1U : 0U, current_imu_age_ms,
        imu->sample_count, imu->read_error_count,
        imu->acceleration_x_mg,
        imu->acceleration_y_mg,
        imu->acceleration_z_mg,
        imu->longitudinal_acceleration_mg);
    if (length <= 0 || length >= (int)sizeof(line)) {
        ESP_LOGW(TAG, "TEST采样编码失败");
        return;
    }
    ESP_LOGI(MSP_TAG, "%s", line);
    (void)app_udp_log_publish(line, (size_t)length);
}

static void main_monitor_task(void *argument)
{
    (void)argument;
    TickType_t last_wake = xTaskGetTickCount();
    bool vision_stale_reported = false;
    uint32_t status_log_divider = 0U;
    uint32_t previous_control_frame = 0U;
    uint32_t previous_test_vision_frame = 0U;

    for (;;) {
        app_vision_state_t vision;
        app_tmx_state_t tmx;
        app_motor_control_state_t motor;
        app_jy901s_state_t imu;
        app_vision_get_state(&vision);
        app_tmx_ingest_get_state(&tmx);
        app_motor_control_get_state(&motor);
        app_jy901s_get_state(&imu);

        uint32_t age_ms = vision_age_ms(&vision);
        bool test_active = tmx.mode_active &&
            strcmp(tmx.mode, "TEST") == 0;
        bool active_control = motor.test_centering_active ||
            (app_tmx_task_uses_ball_control(motor.task_id) &&
             (motor.task_running ||
              motor.dispatch_phase == APP_TASK_DISPATCH_PREP));
        bool vision_stale = (active_control || test_active) &&
                            age_ms > MAIN_VISION_CONTROL_STALE_MS;
        if (vision_stale && !vision_stale_reported) {
            ESP_LOGW(TAG,
                     "视觉数据超过1秒未更新: age=%" PRIu32 "ms",
                     age_ms);
        } else if (!vision_stale && vision_stale_reported) {
            ESP_LOGI(TAG, "视觉数据已恢复");
        }
        vision_stale_reported = vision_stale;

        status_log_divider++;
        bool periodic_status_due =
            status_log_divider >= MAIN_STATUS_LOG_DIVIDER;
        if (periodic_status_due) {
            status_log_divider = 0U;
            ESP_LOGI(
                TAG,
                "状态 task=%s taskState=%s run=%" PRIu32
                " stage=%s elapsed=%" PRIu32
                " target=%" PRId32 "mm ref=%" PRId32 " refOk=%u"
                " mode=%s modeTask=%s motor=%s ball=%s error=%dpx"
                " phase=%s phaseMs=%" PRIu32 " reached=%u"
                " v=%" PRId32 "px/s brake=%" PRId32
                " stopError=%" PRId32 "px"
                " age=%" PRIu32 "ms enc=%" PRId32
                " encTarget=%" PRId32 " balance=%" PRId32
                " balanceReady=%u dir=%s stepHz=%" PRIu32
                " netSteps=%" PRId32
                " pulses=%" PRIu32 " angleLimit=%u"
                " frames=%" PRIu32 " tmxLines=%" PRIu32
                " dropped=%" PRIu32 " taskEvents=%" PRIu32
                " duplicates=%" PRIu32 " lastEventSeq=%" PRIu32
                " modeEvents=%" PRIu32 " modeDuplicates=%" PRIu32
                " lastModeSeq=%" PRIu32 " paramUpdates=%" PRIu32,
                telemetry_task_name(&motor),
                tmx.task_running ? "RUN" : "IDLE", motor.task_run,
                telemetry_stage_name(&motor),
                motor.task_elapsed_ms,
                vision.target_mm, motor.reference_px,
                motor.reference_valid ? 1U : 0U,
                tmx.mode_active ? tmx.mode : "IDLE",
                tmx.mode_active && tmx.mode_task[0] != '\0' ?
                    tmx.mode_task : "NONE",
                motor_state_name(&motor),
                vision.detected ? "VALID" : "LOST",
                (int)vision.error_px,
                control_phase_name(motor.control_phase),
                motor.phase_elapsed_ms,
                motor.phase_angle_reached ? 1U : 0U,
                motor.estimated_velocity_px_s,
                motor.brake_compensation_px,
                motor.predicted_stop_error_px,
                age_ms, motor.encoder_count, motor.encoder_target_count,
                motor.neutral_count, motor.neutral_valid ? 1U : 0U,
                motor_direction_name(&motor), motor.step_frequency_hz,
                motor.net_step_position,
                motor.vision_pulse_count,
                motor.angle_limit_active ? 1U : 0U,
                vision.frame_count,
                tmx.received_lines, tmx.dropped_lines,
                tmx.accepted_task_events, tmx.duplicate_task_events,
                tmx.last_event_sequence,
                tmx.accepted_mode_events, tmx.duplicate_mode_events,
                tmx.last_mode_event_sequence, tmx.parameter_updates);
        }

        bool new_vision_frame =
            motor.control_frame_count != previous_control_frame;
        previous_control_frame = motor.control_frame_count;
        bool new_test_vision_frame =
            vision.frame_count != previous_test_vision_frame;
        previous_test_vision_frame = vision.frame_count;
        if (test_active) {
            log_test_sample(&vision, &imu, age_ms,
                            new_test_vision_frame);
        } else if (active_control) {
            log_control_sample(&motor, &imu, age_ms, new_vision_frame);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MAIN_MONITOR_PERIOD_MS));
    }
}

void app_main(void)
{
    esp_err_t error = app_system_start();
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "集中处理器启动失败: %s", esp_err_to_name(error));
        ESP_ERROR_CHECK(error);
        return;
    }

    BaseType_t task_created = xTaskCreatePinnedToCore(
        main_monitor_task, "MainMonitor", MAIN_MONITOR_STACK_SIZE,
        NULL, MAIN_MONITOR_PRIORITY, NULL, APP_CORE_COMMUNICATION);
    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "创建主状态监控任务失败");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }
}
