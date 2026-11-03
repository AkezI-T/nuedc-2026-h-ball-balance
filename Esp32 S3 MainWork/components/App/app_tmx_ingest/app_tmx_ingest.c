#include "app_tmx_ingest.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app_core_affinity.h"
#include "app_udp_log.h"
#include "bsp_uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#define TMX_LINE_MAX 384U
#define TMX_UART_RX_BUFFER_SIZE 2048
#define TMX_TASK_STACK_SIZE 4096U

#define TASK_DEFAULT_REACH_ERROR_PX 27
#define TASK_DEFAULT_REACH_FRAMES 2U
#define TASK_DEFAULT_REACH_SPEED_PX_S 50
#define TASK_DEFAULT_SETTLE_ERROR_PX 27
#define TASK_DEFAULT_SETTLE_SPEED_PX_S 25
#define TASK_DEFAULT_SETTLE_FRAMES 5U
#define TASK_DEFAULT_CAPTURE_FRAMES 3U

#if defined(CONFIG_GRM_TMX_UART_ENABLE) && CONFIG_GRM_TMX_UART_ENABLE
#define GRM_TMX_UART_ENABLED true
#else
#define GRM_TMX_UART_ENABLED false
#endif

static const char *TAG = "app_tmx";
static portMUX_TYPE s_state_lock = portMUX_INITIALIZER_UNLOCKED;
static app_tmx_state_t s_state;
static app_tmx_task_params_t s_task_params[APP_TMX_TASK_COUNT];

typedef enum {
    TMX_TASK_EVENT_INVALID = 0,
    TMX_TASK_EVENT_BEGIN,
    TMX_TASK_EVENT_END,
} tmx_task_event_type_t;

typedef struct {
    bool valid;
    char source[8];
    uint32_t event_sequence;
    uint32_t run;
    tmx_task_event_type_t type;
} tmx_task_event_key_t;

static tmx_task_event_key_t s_last_task_event;

typedef enum {
    TMX_MODE_EVENT_INVALID = 0,
    TMX_MODE_EVENT_ENTER,
    TMX_MODE_EVENT_EXIT,
} tmx_mode_event_type_t;

typedef struct {
    bool valid;
    char source[8];
    uint32_t event_sequence;
    tmx_mode_event_type_t type;
    char mode[16];
} tmx_mode_event_key_t;

static tmx_mode_event_key_t s_last_mode_event;

static bool get_field(const char *line, const char *key,
                      char *value, size_t value_size)
{
    size_t key_length = strlen(key);
    const char *cursor = line;

    while (*cursor != '\0') {
        while (*cursor == ' ') {
            cursor++;
        }
        const char *token_end = strchr(cursor, ' ');
        if (token_end == NULL) {
            token_end = cursor + strlen(cursor);
        }
        if ((size_t)(token_end - cursor) > key_length + 1U &&
            strncmp(cursor, key, key_length) == 0 &&
            cursor[key_length] == '=') {
            const char *field_value = cursor + key_length + 1U;
            size_t length = (size_t)(token_end - field_value);
            if (length == 0U || length >= value_size) {
                return false;
            }
            memcpy(value, field_value, length);
            value[length] = '\0';
            return true;
        }
        cursor = token_end;
    }
    return false;
}

static bool parse_int32_text(const char *text, int32_t *value)
{
    errno = 0;
    char *end = NULL;
    long parsed = strtol(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0' ||
        parsed < INT32_MIN || parsed > INT32_MAX) {
        return false;
    }
    *value = (int32_t)parsed;
    return true;
}

static bool parse_uint32_text(const char *text, uint32_t *value)
{
    if (text[0] < '0' || text[0] > '9') {
        return false;
    }
    errno = 0;
    char *end = NULL;
    unsigned long parsed = strtoul(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0' ||
        parsed > UINT32_MAX) {
        return false;
    }
    *value = (uint32_t)parsed;
    return true;
}

static bool parse_uint32_field(const char *line, const char *key,
                               uint32_t *value)
{
    char text[20];
    return get_field(line, key, text, sizeof(text)) &&
           parse_uint32_text(text, value);
}

static uint32_t parse_uint32_or_zero(const char *line, const char *key)
{
    uint32_t value;
    return parse_uint32_field(line, key, &value) ? value : 0U;
}

static void get_source(const char *line, char *source, size_t source_size)
{
    if (get_field(line, "source", source, source_size)) {
        return;
    }
    if (strstr(line, "source=") != NULL) {
        strlcpy(source, "INVALID", source_size);
        return;
    }
    strlcpy(source, "TMX", source_size);
}

app_tmx_task_id_t app_tmx_task_from_name(const char *name)
{
    if (name == NULL) {
        return APP_TMX_TASK_NONE;
    }
    if (strcmp(name, "T1") == 0) return APP_TMX_TASK_T1;
    if (strcmp(name, "T2") == 0) return APP_TMX_TASK_T2;
    if (strcmp(name, "T3") == 0) return APP_TMX_TASK_T3;
    if (strcmp(name, "T4") == 0) return APP_TMX_TASK_T4;
    if (strcmp(name, "T5") == 0) return APP_TMX_TASK_T5;
    if (strcmp(name, "T6") == 0) return APP_TMX_TASK_T6;
    return APP_TMX_TASK_NONE;
}

const char *app_tmx_task_name(app_tmx_task_id_t task_id)
{
    switch (task_id) {
    case APP_TMX_TASK_T1: return "T1";
    case APP_TMX_TASK_T2: return "T2";
    case APP_TMX_TASK_T3: return "T3";
    case APP_TMX_TASK_T4: return "T4";
    case APP_TMX_TASK_T5: return "T5";
    case APP_TMX_TASK_T6: return "T6";
    case APP_TMX_TASK_NONE:
    default:
        return "NONE";
    }
}

bool app_tmx_task_uses_ball_control(app_tmx_task_id_t task_id)
{
    return task_id >= APP_TMX_TASK_T3 && task_id <= APP_TMX_TASK_T6;
}

static void initialize_task_params(void)
{
    const app_tmx_task_params_t defaults = {
        .target_mm = 0,
        .secondary_target_mm = 0,
        .timeout_ms = 0U,
        .reach_error_px = TASK_DEFAULT_REACH_ERROR_PX,
        .reach_frames = TASK_DEFAULT_REACH_FRAMES,
        .reach_speed_px_s = TASK_DEFAULT_REACH_SPEED_PX_S,
        .settle_error_px = TASK_DEFAULT_SETTLE_ERROR_PX,
        .settle_speed_px_s = TASK_DEFAULT_SETTLE_SPEED_PX_S,
        .settle_frames = TASK_DEFAULT_SETTLE_FRAMES,
        .capture_frames = TASK_DEFAULT_CAPTURE_FRAMES,
    };
    for (size_t index = 0; index < APP_TMX_TASK_COUNT; index++) {
        s_task_params[index] = defaults;
    }
    s_task_params[APP_TMX_TASK_T2].timeout_ms = 20000U;
    s_task_params[APP_TMX_TASK_T3].target_mm = 50;
    s_task_params[APP_TMX_TASK_T3].secondary_target_mm = -50;
    s_task_params[APP_TMX_TASK_T3].timeout_ms = 0U;
    s_task_params[APP_TMX_TASK_T4].timeout_ms = 8000U;
    s_task_params[APP_TMX_TASK_T5].timeout_ms = 30000U;
    s_task_params[APP_TMX_TASK_T6].timeout_ms = 30000U;
}

static bool apply_optional_i32(const char *line, const char *key,
                               int32_t minimum, int32_t maximum,
                               int32_t *field, bool *changed)
{
    char text[20];
    if (!get_field(line, key, text, sizeof(text))) {
        return false;
    }
    int32_t value;
    if (!parse_int32_text(text, &value) || value < minimum ||
        value > maximum) {
        ESP_LOGW(TAG, "忽略无效任务参数: %s=%s", key, text);
        return true;
    }
    if (*field != value) {
        *field = value;
        *changed = true;
    }
    return true;
}

static bool apply_optional_u32(const char *line, const char *key,
                               uint32_t minimum, uint32_t maximum,
                               uint32_t *field, bool *changed)
{
    char text[20];
    if (!get_field(line, key, text, sizeof(text))) {
        return false;
    }
    uint32_t value;
    if (!parse_uint32_text(text, &value) || value < minimum ||
        value > maximum) {
        ESP_LOGW(TAG, "忽略无效任务参数: %s=%s", key, text);
        return true;
    }
    if (*field != value) {
        *field = value;
        *changed = true;
    }
    return true;
}

static void publish_task_params(app_tmx_task_id_t task_id,
                                const app_tmx_task_params_t *params,
                                const char *state)
{
    char line[256];
    int length = snprintf(
        line, sizeof(line),
        "@PARAM source=ESP32 task=%s state=%s targetMm=%ld"
        " target2Mm=%ld timeoutMs=%lu reachPx=%ld reachFrames=%lu"
        " reachSpeed=%ld settlePx=%ld settleSpeed=%ld settleFrames=%lu"
        " captureFrames=%lu",
        app_tmx_task_name(task_id), state,
        (long)params->target_mm, (long)params->secondary_target_mm,
        (unsigned long)params->timeout_ms,
        (long)params->reach_error_px,
        (unsigned long)params->reach_frames,
        (long)params->reach_speed_px_s,
        (long)params->settle_error_px,
        (long)params->settle_speed_px_s,
        (unsigned long)params->settle_frames,
        (unsigned long)params->capture_frames);
    if (length > 0 && length < (int)sizeof(line)) {
        ESP_LOGI(TAG, "%s", line);
        (void)app_udp_log_publish(line, (size_t)length);
    }
}

static app_tmx_task_params_t apply_task_params(app_tmx_task_id_t task_id,
                                                const char *line,
                                                bool announce)
{
    app_tmx_task_params_t params;
    taskENTER_CRITICAL(&s_state_lock);
    params = s_task_params[task_id];
    taskEXIT_CRITICAL(&s_state_lock);

    bool changed = false;
    bool target_found = apply_optional_i32(
        line, "targetMm", -CONFIG_GRM_TARGET_LIMIT_MM,
        CONFIG_GRM_TARGET_LIMIT_MM, &params.target_mm, &changed);
    if (!target_found) {
        target_found = apply_optional_i32(
            line, "target_mm", -CONFIG_GRM_TARGET_LIMIT_MM,
            CONFIG_GRM_TARGET_LIMIT_MM, &params.target_mm, &changed);
    }
    if (!target_found) {
        (void)apply_optional_i32(
            line, "target", -CONFIG_GRM_TARGET_LIMIT_MM,
            CONFIG_GRM_TARGET_LIMIT_MM, &params.target_mm, &changed);
    }

    bool secondary_found = apply_optional_i32(
        line, "target2Mm", -CONFIG_GRM_TARGET_LIMIT_MM,
        CONFIG_GRM_TARGET_LIMIT_MM, &params.secondary_target_mm, &changed);
    if (!secondary_found) {
        (void)apply_optional_i32(
            line, "secondaryTargetMm", -CONFIG_GRM_TARGET_LIMIT_MM,
            CONFIG_GRM_TARGET_LIMIT_MM, &params.secondary_target_mm,
            &changed);
    }
    (void)apply_optional_u32(line, "timeoutMs", 0U, 60000U,
                             &params.timeout_ms, &changed);
    (void)apply_optional_i32(line, "reachPx", 1, 639,
                             &params.reach_error_px, &changed);
    (void)apply_optional_u32(line, "reachFrames", 1U, 60U,
                             &params.reach_frames, &changed);
    (void)apply_optional_i32(line, "reachSpeed", 0, 5000,
                             &params.reach_speed_px_s, &changed);
    (void)apply_optional_i32(line, "settlePx", 1, 639,
                             &params.settle_error_px, &changed);
    (void)apply_optional_i32(line, "settleSpeed", 0, 5000,
                             &params.settle_speed_px_s, &changed);
    (void)apply_optional_u32(line, "settleFrames", 1U, 60U,
                             &params.settle_frames, &changed);
    (void)apply_optional_u32(line, "captureFrames", 1U, 30U,
                             &params.capture_frames, &changed);

    if (changed) {
        taskENTER_CRITICAL(&s_state_lock);
        s_task_params[task_id] = params;
        if (!s_state.task_running && s_state.mode_active &&
            s_state.mode_task_id == task_id) {
            s_state.active_params = params;
        }
        s_state.parameter_updates++;
        taskEXIT_CRITICAL(&s_state_lock);
    }
    if (announce || changed) {
        publish_task_params(task_id, &params,
                            changed ? "UPDATED" : "ACTIVE");
    }
    return params;
}

static bool accept_task_event(const char *source, uint32_t event_sequence,
                              uint32_t run, tmx_task_event_type_t type)
{
    if (s_last_task_event.valid &&
        strcmp(s_last_task_event.source, source) == 0 &&
        s_last_task_event.event_sequence == event_sequence &&
        s_last_task_event.run == run &&
        s_last_task_event.type == type) {
        taskENTER_CRITICAL(&s_state_lock);
        s_state.duplicate_task_events++;
        taskEXIT_CRITICAL(&s_state_lock);
        return false;
    }

    s_last_task_event = (tmx_task_event_key_t){
        .valid = true,
        .event_sequence = event_sequence,
        .run = run,
        .type = type,
    };
    strlcpy(s_last_task_event.source, source,
            sizeof(s_last_task_event.source));
    taskENTER_CRITICAL(&s_state_lock);
    s_state.accepted_task_events++;
    s_state.last_event_sequence = event_sequence;
    taskEXIT_CRITICAL(&s_state_lock);
    return true;
}

static void handle_task_line(const char *line)
{
    char source[8];
    char type[20];
    char task[16];
    uint32_t event_sequence;
    uint32_t run;
    get_source(line, source, sizeof(source));
    if (strcmp(source, "TMX") != 0 ||
        !get_field(line, "type", type, sizeof(type)) ||
        !parse_uint32_field(line, "eventSeq", &event_sequence) ||
        !parse_uint32_field(line, "run", &run) ||
        !get_field(line, "task", task, sizeof(task))) {
        ESP_LOGW(TAG, "忽略来源或字段无效的任务事件");
        return;
    }

    app_tmx_task_id_t task_id = app_tmx_task_from_name(task);
    if (task_id == APP_TMX_TASK_NONE) {
        ESP_LOGW(TAG, "忽略未知任务: %s", task);
        return;
    }
    tmx_task_event_type_t event_type;
    if (strcmp(type, "TASK_BEGIN") == 0) {
        event_type = TMX_TASK_EVENT_BEGIN;
    } else if (strcmp(type, "TASK_END") == 0) {
        event_type = TMX_TASK_EVENT_END;
    } else {
        return;
    }
    if (!accept_task_event(source, event_sequence, run, event_type)) {
        ESP_LOGD(TAG, "忽略重复任务事件: eventSeq=%lu run=%lu type=%s",
                 (unsigned long)event_sequence, (unsigned long)run, type);
        return;
    }

    if (event_type == TMX_TASK_EVENT_BEGIN) {
        app_tmx_task_params_t params =
            apply_task_params(task_id, line, true);
        taskENTER_CRITICAL(&s_state_lock);
        s_state.task_running = true;
        s_state.task_id = task_id;
        strlcpy(s_state.task, task, sizeof(s_state.task));
        s_state.run = run;
        s_state.active_params = params;
        taskEXIT_CRITICAL(&s_state_lock);
        ESP_LOGI(TAG,
                 "任务开始: eventSeq=%lu run=%lu task=%s",
                 (unsigned long)event_sequence, (unsigned long)run, task);
        return;
    }

    if (task_id == APP_TMX_TASK_T3) {
        ESP_LOGI(TAG,
                 "忽略TMX的T3任务结束事件，等待ESP32内部稳定判定: run=%lu",
                 (unsigned long)run);
        return;
    }

    char result[16];
    char fault[16];
    if (!get_field(line, "result", result, sizeof(result))) {
        strlcpy(result, "UNKNOWN", sizeof(result));
    }
    if (!get_field(line, "fault", fault, sizeof(fault))) {
        strlcpy(fault, "NONE", sizeof(fault));
    }
    taskENTER_CRITICAL(&s_state_lock);
    bool matches_active = !s_state.task_running ||
        (s_state.task_id == task_id && s_state.run == run);
    if (matches_active) {
        s_state.task_running = false;
        s_state.task_id = task_id;
        strlcpy(s_state.task, task, sizeof(s_state.task));
        s_state.run = run;
    }
    taskEXIT_CRITICAL(&s_state_lock);
    if (!matches_active) {
        ESP_LOGW(TAG, "任务结束与当前运行不匹配: task=%s run=%lu",
                 task, (unsigned long)run);
        return;
    }
    ESP_LOGI(TAG,
             "任务结束: eventSeq=%lu run=%lu task=%s result=%s fault=%s elapsed=%lu",
             (unsigned long)event_sequence, (unsigned long)run, task,
             result, fault,
             (unsigned long)parse_uint32_or_zero(line, "elapsed"));
}

bool app_tmx_ingest_complete_task(app_tmx_task_id_t task_id, uint32_t run)
{
    bool completed = false;
    taskENTER_CRITICAL(&s_state_lock);
    if (s_state.task_running && s_state.task_id == task_id &&
        s_state.run == run) {
        s_state.task_running = false;
        completed = true;
    }
    taskEXIT_CRITICAL(&s_state_lock);

    if (completed) {
        ESP_LOGI(TAG, "%s达到内部稳定条件，任务结束: run=%lu",
                 app_tmx_task_name(task_id), (unsigned long)run);
    }
    return completed;
}

static bool accept_mode_event(const char *source, uint32_t event_sequence,
                              tmx_mode_event_type_t type,
                              const char *mode)
{
    if (s_last_mode_event.valid &&
        strcmp(s_last_mode_event.source, source) == 0 &&
        s_last_mode_event.event_sequence == event_sequence &&
        s_last_mode_event.type == type &&
        strcmp(s_last_mode_event.mode, mode) == 0) {
        taskENTER_CRITICAL(&s_state_lock);
        s_state.duplicate_mode_events++;
        taskEXIT_CRITICAL(&s_state_lock);
        return false;
    }

    s_last_mode_event = (tmx_mode_event_key_t){
        .valid = true,
        .event_sequence = event_sequence,
        .type = type,
    };
    strlcpy(s_last_mode_event.source, source,
            sizeof(s_last_mode_event.source));
    strlcpy(s_last_mode_event.mode, mode,
            sizeof(s_last_mode_event.mode));
    taskENTER_CRITICAL(&s_state_lock);
    s_state.accepted_mode_events++;
    s_state.last_mode_event_sequence = event_sequence;
    taskEXIT_CRITICAL(&s_state_lock);
    return true;
}

static void handle_mode_line(const char *line)
{
    char source[8];
    char type[16];
    char mode[16];
    char task[16] = "NONE";
    uint32_t event_sequence;
    get_source(line, source, sizeof(source));
    if (strcmp(source, "TMX") != 0 ||
        !get_field(line, "type", type, sizeof(type)) ||
        !get_field(line, "mode", mode, sizeof(mode)) ||
        !parse_uint32_field(line, "eventSeq", &event_sequence)) {
        ESP_LOGW(TAG, "忽略来源或字段无效的模式事件");
        return;
    }

    tmx_mode_event_type_t event_type;
    if (strcmp(type, "ENTER") == 0) {
        event_type = TMX_MODE_EVENT_ENTER;
    } else if (strcmp(type, "EXIT") == 0) {
        event_type = TMX_MODE_EVENT_EXIT;
    } else {
        return;
    }
    if (!accept_mode_event(source, event_sequence, event_type, mode)) {
        ESP_LOGD(TAG, "忽略重复模式事件: eventSeq=%lu type=%s mode=%s",
                 (unsigned long)event_sequence, type, mode);
        return;
    }

    (void)get_field(line, "task", task, sizeof(task));
    app_tmx_task_id_t task_id = app_tmx_task_from_name(task);
    app_tmx_task_params_t params = {0};
    if (task_id != APP_TMX_TASK_NONE) {
        params = apply_task_params(task_id, line,
                                   event_type == TMX_MODE_EVENT_ENTER);
    }

    taskENTER_CRITICAL(&s_state_lock);
    s_state.mode_active = event_type == TMX_MODE_EVENT_ENTER;
    strlcpy(s_state.mode, mode, sizeof(s_state.mode));
    strlcpy(s_state.mode_task, task, sizeof(s_state.mode_task));
    s_state.mode_task_id = task_id;
    if (task_id != APP_TMX_TASK_NONE) {
        s_state.active_params = params;
    }
    taskEXIT_CRITICAL(&s_state_lock);
    ESP_LOGI(TAG, "模式事件: eventSeq=%lu type=%s mode=%s task=%s",
             (unsigned long)event_sequence, type, mode, task);
}

static void handle_parameter_line(const char *line)
{
    char source[8];
    char task[16];
    get_source(line, source, sizeof(source));
    if (strcmp(source, "TMX") != 0 ||
        !get_field(line, "task", task, sizeof(task))) {
        ESP_LOGW(TAG, "忽略来源或task无效的参数帧");
        return;
    }
    app_tmx_task_id_t task_id = app_tmx_task_from_name(task);
    if (task_id == APP_TMX_TASK_NONE) {
        ESP_LOGW(TAG, "忽略未知任务参数: %s", task);
        return;
    }
    (void)apply_task_params(task_id, line, true);
}

static void process_line(char *line, size_t length)
{
    taskENTER_CRITICAL(&s_state_lock);
    s_state.received_lines++;
    taskEXIT_CRITICAL(&s_state_lock);

    if (memchr(line, '@', length) != NULL) {
        (void)app_udp_log_publish(line, length);
    }
    if (strncmp(line, "@TASK ", 6U) == 0) {
        handle_task_line(line);
    } else if (strncmp(line, "@MODE ", 6U) == 0) {
        handle_mode_line(line);
    } else if (strncmp(line, "@PARAM ", 7U) == 0) {
        handle_parameter_line(line);
    }
}

static void tmx_uart_task(void *argument)
{
    (void)argument;
    uint8_t rx_data[64];
    char line[TMX_LINE_MAX];
    size_t line_length = 0U;
    bool discard_until_newline = false;

    for (;;) {
        int received = bsp_uart_read(CONFIG_GRM_TMX_UART_PORT, rx_data,
                                     sizeof(rx_data), pdMS_TO_TICKS(20));
        for (int index = 0; index < received; index++) {
            char character = (char)rx_data[index];
            if (character == '\r') {
                continue;
            }
            if (character == '\n') {
                if (!discard_until_newline && line_length > 0U) {
                    line[line_length] = '\0';
                    process_line(line, line_length);
                }
                line_length = 0U;
                discard_until_newline = false;
                continue;
            }
            if (discard_until_newline) {
                continue;
            }
            if (line_length >= sizeof(line) - 2U) {
                line_length = 0U;
                discard_until_newline = true;
                taskENTER_CRITICAL(&s_state_lock);
                s_state.dropped_lines++;
                taskEXIT_CRITICAL(&s_state_lock);
                continue;
            }
            line[line_length++] = character;
        }
    }
}

esp_err_t app_tmx_ingest_init(void)
{
    s_state = (app_tmx_state_t){0};
    strlcpy(s_state.task, "NONE", sizeof(s_state.task));
    strlcpy(s_state.mode, "IDLE", sizeof(s_state.mode));
    strlcpy(s_state.mode_task, "NONE", sizeof(s_state.mode_task));
    s_last_task_event = (tmx_task_event_key_t){0};
    s_last_mode_event = (tmx_mode_event_key_t){0};
    initialize_task_params();
    if (!GRM_TMX_UART_ENABLED) {
        ESP_LOGW(TAG, "TMX UART 未启用，等待menuconfig配置RX引脚");
        return ESP_OK;
    }

    const bsp_uart_config_t config = {
        .port = CONFIG_GRM_TMX_UART_PORT,
        .tx_pin = UART_PIN_NO_CHANGE,
        .rx_pin = CONFIG_GRM_TMX_UART_RX_PIN,
        .baud_rate = CONFIG_GRM_TMX_UART_BAUD,
        .rx_buffer_size = TMX_UART_RX_BUFFER_SIZE,
    };
    esp_err_t error = bsp_uart_init(&config);
    if (error != ESP_OK) {
        return error;
    }
    if (xTaskCreatePinnedToCore(tmx_uart_task, "tmx_uart_rx",
                                TMX_TASK_STACK_SIZE, NULL, 6, NULL,
                                APP_CORE_COMMUNICATION) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "TMX单向UART已启动: UART%d RX=%d baud=%d",
             CONFIG_GRM_TMX_UART_PORT, CONFIG_GRM_TMX_UART_RX_PIN,
             CONFIG_GRM_TMX_UART_BAUD);
    return ESP_OK;
}

void app_tmx_ingest_get_state(app_tmx_state_t *state)
{
    if (state == NULL) {
        return;
    }
    taskENTER_CRITICAL(&s_state_lock);
    *state = s_state;
    taskEXIT_CRITICAL(&s_state_lock);
}

void app_tmx_ingest_get_task_params(app_tmx_task_id_t task_id,
                                    app_tmx_task_params_t *params)
{
    if (params == NULL) {
        return;
    }
    if (task_id <= APP_TMX_TASK_NONE || task_id >= APP_TMX_TASK_COUNT) {
        *params = (app_tmx_task_params_t){0};
        return;
    }
    taskENTER_CRITICAL(&s_state_lock);
    *params = s_task_params[task_id];
    taskEXIT_CRITICAL(&s_state_lock);
}
