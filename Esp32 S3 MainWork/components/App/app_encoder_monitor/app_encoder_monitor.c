#include "app_encoder_monitor.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "app_core_affinity.h"
#include "app_tmx_ingest.h"
#include "app_udp_log.h"
#include "bsp_encoder.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ENCODER_MONITOR_PERIOD_MS 500U
#define ENCODER_PWM_STALE_MS 100U
#define ENCODER_MONITOR_STACK_SIZE 3072U
#define ENCODER_MONITOR_PRIORITY 7U
#define ENCODER_COUNTS_PER_REV 4096
#define ENCODER_LINE_SIZE 320U

static const char *TAG = "app_encoder";

static const char *encoder_motion_name(int32_t delta)
{
    if (delta > 0) {
        return "FRONT";
    }
    if (delta < 0) {
        return "REAR";
    }
    return "STOP";
}

static uint32_t ticks_to_us(uint32_t ticks, uint32_t resolution_hz)
{
    if (resolution_hz == 0U) {
        return 0U;
    }
    return (uint32_t)(((uint64_t)ticks * 1000000ULL) / resolution_hz);
}

static uint32_t calculate_pwm_angle_mdeg(uint32_t high, uint32_t period)
{
    if (period == 0U || high == 0U || high >= period) {
        return 0U;
    }
    uint32_t position_code =
        (uint32_t)(((uint64_t)high * 4115ULL) / period);
    if (position_code > 0U) {
        position_code--;
    }
    if (position_code > 4114U) {
        position_code = 4114U;
    }
    return (uint32_t)(((uint64_t)position_code * 360000ULL) / 4115ULL);
}

static void encoder_monitor_task(void *argument)
{
    (void)argument;
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t last_pwm_update = 0;
    uint32_t previous_capture_count = 0U;
    int32_t previous_ab_count = 0;
    bool previous_test_active = false;

    for (;;) {
        vTaskDelayUntil(&last_wake,
                        pdMS_TO_TICKS(ENCODER_MONITOR_PERIOD_MS));

        bsp_encoder_snapshot_t encoder;
        esp_err_t error = bsp_encoder_get_snapshot(&encoder);
        if (error != ESP_OK) {
            ESP_LOGE(TAG, "读取编码器失败: %s", esp_err_to_name(error));
            continue;
        }

        TickType_t now = xTaskGetTickCount();
        if (encoder.pwm_capture_count != previous_capture_count) {
            previous_capture_count = encoder.pwm_capture_count;
            last_pwm_update = now;
        }
        bool pwm_valid = encoder.pwm_capture_count > 0U &&
            (now - last_pwm_update) <= pdMS_TO_TICKS(ENCODER_PWM_STALE_MS);
        int32_t delta = encoder.ab_count - previous_ab_count;
        previous_ab_count = encoder.ab_count;

        app_tmx_state_t tmx;
        app_tmx_ingest_get_state(&tmx);
        bool test_active = tmx.mode_active &&
            strcmp(tmx.mode, "TEST") == 0;
        if (test_active && !previous_test_active) {
            ESP_LOGI(TAG, "进入TEST，开始输出编码器诊断");
        } else if (!test_active && previous_test_active) {
            ESP_LOGI(TAG, "退出TEST，停止输出编码器诊断");
        }
        previous_test_active = test_active;
        if (!test_active) {
            continue;
        }

        int32_t ab_angle_mdeg = (int32_t)(
            ((int64_t)encoder.ab_count * 360000LL) /
            ENCODER_COUNTS_PER_REV);
        uint32_t duty_permille = encoder.pwm_period_ticks == 0U ? 0U :
            (uint32_t)(((uint64_t)encoder.pwm_high_ticks * 1000ULL) /
                       encoder.pwm_period_ticks);

        char line[ENCODER_LINE_SIZE];
        int length = snprintf(
            line, sizeof(line),
            "@ENC ab=%" PRId32 " delta=%" PRId32
            " motion=%s abAngleMd=%" PRId32 " a=%d b=%d pwm=%s"
            " periodTicks=%" PRIu32 " highTicks=%" PRIu32
            " periodUs=%" PRIu32 " highUs=%" PRIu32
            " dutyPermille=%" PRIu32 " pwmAngleMd=%" PRIu32
            " z=%" PRIu32 " captures=%" PRIu32 "\n",
            encoder.ab_count, delta, encoder_motion_name(delta),
            ab_angle_mdeg,
            encoder.a_level, encoder.b_level,
            pwm_valid ? "VALID" : "LOST",
            encoder.pwm_period_ticks, encoder.pwm_high_ticks,
            ticks_to_us(encoder.pwm_period_ticks,
                        encoder.pwm_resolution_hz),
            ticks_to_us(encoder.pwm_high_ticks,
                        encoder.pwm_resolution_hz),
            duty_permille,
            calculate_pwm_angle_mdeg(encoder.pwm_high_ticks,
                                     encoder.pwm_period_ticks),
            encoder.z_count, encoder.pwm_capture_count);
        if (length <= 0 || length >= (int)sizeof(line)) {
            ESP_LOGE(TAG, "编码器诊断行编码失败");
            continue;
        }

        ESP_LOGI(TAG, "%.*s", length - 1, line);
        (void)app_udp_log_publish(line, (size_t)length);
    }
}

esp_err_t app_encoder_monitor_init(void)
{
    ESP_RETURN_ON_ERROR(bsp_encoder_init(), TAG, "编码器BSP初始化失败");
    if (xTaskCreatePinnedToCore(
            encoder_monitor_task, "encoder_monitor",
            ENCODER_MONITOR_STACK_SIZE, NULL,
            ENCODER_MONITOR_PRIORITY, NULL,
            APP_CORE_CONTROL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG,
             "编码器采集已启动: Core=%d A=4 B=5 PWM=6 Z=7 CPR=4096；TEST显示",
             APP_CORE_CONTROL);
    return ESP_OK;
}
