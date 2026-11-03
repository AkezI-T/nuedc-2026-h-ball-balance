#include "app_vision.h"

#include <stdio.h>
#include <string.h>

#include "app_core_affinity.h"
#include "bsp_uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#define VISION_LINE_MAX 96U
#define VISION_UART_RX_BUFFER_SIZE 2048
#define VISION_UART_READ_CHUNK 64U
#define VISION_TASK_STACK_SIZE 4096U
#define VISION_TARGET_REPEAT_US 1000000LL

static const char *TAG = "app_vision";
static portMUX_TYPE s_state_lock = portMUX_INITIALIZER_UNLOCKED;
static app_vision_state_t s_state;
static SemaphoreHandle_t s_uart_mutex;
static bool s_uart_ready;
static int64_t s_target_sent_at_us;

static int32_t get_current_target_mm(void)
{
    taskENTER_CRITICAL(&s_state_lock);
    int32_t target_mm = s_state.target_mm;
    taskEXIT_CRITICAL(&s_state_lock);
    return target_mm;
}

static void record_invalid_line(const char *reason)
{
    taskENTER_CRITICAL(&s_state_lock);
    s_state.parse_error_count++;
    uint32_t error_count = s_state.parse_error_count;
    taskEXIT_CRITICAL(&s_state_lock);

    if (error_count == 1U || error_count % 20U == 0U) {
        ESP_LOGW(TAG, "无效视觉串口帧: %s, count=%lu",
                 reason, (unsigned long)error_count);
    }
}

static bool parse_vision_line(const char *line, bool *detected,
                              int16_t *error_px, int16_t *last_error_px,
                              int32_t *velocity_px_s)
{
    int detected_value;
    int error_value;
    int last_error_value;
    int velocity_value;
    int consumed = 0;

    if (sscanf(line, "@VIS,%d,%d,%d,%d%n", &detected_value, &error_value,
               &last_error_value, &velocity_value, &consumed) != 4 ||
        line[consumed] != '\0') {
        return false;
    }
    if ((detected_value != 0 && detected_value != 1) ||
        error_value < -639 || error_value > 639 ||
        last_error_value < -639 || last_error_value > 639 ||
        velocity_value < -5000 || velocity_value > 5000) {
        return false;
    }

    *detected = detected_value == 1;
    *error_px = (int16_t)error_value;
    *last_error_px = (int16_t)last_error_value;
    *velocity_px_s = velocity_value;
    return true;
}

static bool process_vision_line(char *line)
{
    bool detected;
    int16_t error_px;
    int16_t last_error_px;
    int32_t velocity_px_s;

    if (!parse_vision_line(line, &detected, &error_px, &last_error_px,
                           &velocity_px_s)) {
        record_invalid_line(line);
        return false;
    }

    int64_t received_at_us = esp_timer_get_time();
    taskENTER_CRITICAL(&s_state_lock);
    s_state.detected = detected;
    s_state.error_px = error_px;
    s_state.last_error_px = last_error_px;
    s_state.velocity_px_s = velocity_px_s;
    s_state.received_at_us = received_at_us;
    if (detected) {
        s_state.last_detected_at_us = received_at_us;
    }
    s_state.frame_count++;
    taskEXIT_CRITICAL(&s_state_lock);
    return true;
}

static esp_err_t send_target_locked(int32_t target_mm)
{
    if (!s_uart_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    char frame[24];
    int length = snprintf(frame, sizeof(frame), "@TGT,%ld\r\n",
                          (long)target_mm);
    if (length <= 0 || length >= (int)sizeof(frame)) {
        return ESP_FAIL;
    }

    int written = bsp_uart_write(
        (uart_port_t)CONFIG_GRM_VISION_UART_PORT,
        frame, (size_t)length);
    s_target_sent_at_us = esp_timer_get_time();
    return written == length ? ESP_OK : ESP_FAIL;
}

static void repeat_target_if_due(void)
{
    int64_t now_us = esp_timer_get_time();
    int32_t target_mm = get_current_target_mm();
    xSemaphoreTake(s_uart_mutex, portMAX_DELAY);
    if (s_target_sent_at_us <= 0 ||
        now_us - s_target_sent_at_us >= VISION_TARGET_REPEAT_US) {
        (void)send_target_locked(target_mm);
    }
    xSemaphoreGive(s_uart_mutex);
}

static void vision_uart_task(void *argument)
{
    (void)argument;
    uint8_t rx_data[VISION_UART_READ_CHUNK];
    char line[VISION_LINE_MAX + 1U];
    size_t line_length = 0U;
    bool discard_until_newline = false;

    for (;;) {
        int received = bsp_uart_read(
            (uart_port_t)CONFIG_GRM_VISION_UART_PORT,
            rx_data, sizeof(rx_data), pdMS_TO_TICKS(20));
        for (int index = 0; index < received; index++) {
            char character = (char)rx_data[index];
            if (discard_until_newline) {
                if (character == '\n') {
                    discard_until_newline = false;
                    line_length = 0U;
                }
                continue;
            }

            if (character == '\n') {
                if (line_length == 0U || line[line_length - 1U] != '\r') {
                    record_invalid_line("缺少CRLF");
                } else {
                    line[line_length - 1U] = '\0';
                    (void)process_vision_line(line);
                }
                line_length = 0U;
                continue;
            }

            if (line_length >= VISION_LINE_MAX) {
                record_invalid_line("行长度溢出");
                line_length = 0U;
                discard_until_newline = true;
                continue;
            }
            line[line_length++] = character;
        }
        repeat_target_if_due();
    }
}

esp_err_t app_vision_init(void)
{
    s_state = (app_vision_state_t){0};
    s_uart_ready = false;
    s_target_sent_at_us = 0;
    s_uart_mutex = xSemaphoreCreateMutex();
    if (s_uart_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    const bsp_uart_config_t config = {
        .port = (uart_port_t)CONFIG_GRM_VISION_UART_PORT,
        .tx_pin = CONFIG_GRM_VISION_UART_TX_PIN,
        .rx_pin = CONFIG_GRM_VISION_UART_RX_PIN,
        .baud_rate = CONFIG_GRM_VISION_UART_BAUD,
        .rx_buffer_size = VISION_UART_RX_BUFFER_SIZE,
    };
    esp_err_t error = bsp_uart_init(&config);
    if (error != ESP_OK) {
        return error;
    }
    s_uart_ready = true;

    if (xTaskCreatePinnedToCore(
            vision_uart_task, "vision_uart",
            VISION_TASK_STACK_SIZE, NULL, 6, NULL,
            APP_CORE_COMMUNICATION) != pdPASS) {
        s_uart_ready = false;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG,
             "K230视觉UART已启动: UART%d RX=%d TX=%d baud=%d，VIS/TGT双向",
             CONFIG_GRM_VISION_UART_PORT,
             CONFIG_GRM_VISION_UART_RX_PIN,
             CONFIG_GRM_VISION_UART_TX_PIN,
             CONFIG_GRM_VISION_UART_BAUD);
    return ESP_OK;
}

esp_err_t app_vision_set_target_mm(int32_t target_mm)
{
    if (target_mm < -CONFIG_GRM_TARGET_LIMIT_MM ||
        target_mm > CONFIG_GRM_TARGET_LIMIT_MM) {
        return ESP_ERR_INVALID_ARG;
    }

    taskENTER_CRITICAL(&s_state_lock);
    s_state.target_mm = target_mm;
    taskEXIT_CRITICAL(&s_state_lock);

    xSemaphoreTake(s_uart_mutex, portMAX_DELAY);
    esp_err_t error = send_target_locked(target_mm);
    xSemaphoreGive(s_uart_mutex);
    return error;
}

void app_vision_get_state(app_vision_state_t *state)
{
    if (state == NULL) {
        return;
    }
    taskENTER_CRITICAL(&s_state_lock);
    *state = s_state;
    taskEXIT_CRITICAL(&s_state_lock);
}
