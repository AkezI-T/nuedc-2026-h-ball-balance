/**
 * @file app_tasks.c
 * @brief Sensor、Control、UI、Log四任务调度
 */

#include "app_tasks.h"
#include "app_config.h"
#include "app_menu.h"

#include "bsp_encoder.h"
#include "bsp_i2c_async.h"
#include "bsp_jy901s.h"
#include "bsp_key.h"
#include "bsp_line_sensor.h"
#include "bsp_log.h"
#include "bsp_motor.h"
#include "ti_msp_dl_config.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

static APP_SensorSnapshot g_sensors;

static StaticTask_t g_sensor_tcb;
static StaticTask_t g_control_tcb;
static StaticTask_t g_ui_tcb;
static StaticTask_t g_log_tcb;
static StackType_t g_sensor_stack[APP_TASK_STACK_SENSOR];
static StackType_t g_control_stack[APP_TASK_STACK_CONTROL];
static StackType_t g_ui_stack[APP_TASK_STACK_UI];
static StackType_t g_log_stack[APP_TASK_STACK_LOG];

static bool time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

void APP_Sensors_Get(APP_SensorSnapshot *snapshot)
{
    if (snapshot == NULL) return;
    taskENTER_CRITICAL();
    *snapshot = g_sensors;
    taskEXIT_CRITICAL();
}

static void sensor_publish_line(uint32_t now_ms, uint8_t raw, bool ok)
{
    taskENTER_CRITICAL();
    if (ok) {
        g_sensors.line_raw = raw;
        g_sensors.line_timestamp_ms = now_ms;
        g_sensors.line_valid = true;
    } else {
        g_sensors.line_error_count++;
    }
    taskEXIT_CRITICAL();
}

static void sensor_publish_gyro(uint32_t now_ms,
    const BSP_JY901S_Angles *angles, bool ok)
{
    taskENTER_CRITICAL();
    if (ok) {
        g_sensors.roll_x10 = angles->roll_x10;
        g_sensors.pitch_x10 = angles->pitch_x10;
        g_sensors.yaw_x10 = angles->yaw_x10;
        g_sensors.gyro_timestamp_ms = now_ms;
        g_sensors.gyro_valid = true;
    } else {
        g_sensors.gyro_error_count++;
    }
    taskEXIT_CRITICAL();
}

static void Task_Sensor(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t next_line_ms = 0U;
    uint32_t next_gyro_ms = 0U;
    uint8_t line_raw;
    bool ok;
    BSP_JY901S_Angles angles;

    (void)argument;
    BSP_LineSensor_Init();
    BSP_JY901S_Init();

    for (;;) {
        uint32_t now_ms = (uint32_t)xTaskGetTickCount();
        BSP_I2C_AsyncStep(now_ms);

        if (BSP_LineSensor_Take(&line_raw, &ok)) {
            sensor_publish_line(now_ms, line_raw, ok);
        }
        if (BSP_JY901S_Take(&angles, &ok)) {
            sensor_publish_gyro(now_ms, &angles, ok);
        }

        if (time_reached(now_ms, next_line_ms) &&
            BSP_LineSensor_Request(now_ms)) {
            next_line_ms = now_ms + 5U;
        }
        if (time_reached(now_ms, next_gyro_ms) &&
            BSP_JY901S_Request(now_ms)) {
            next_gyro_ms = now_ms + 20U;
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(APP_SENSOR_PERIOD_MS));
    }
}

static int32_t apply_encoder_invert(int32_t value, bool invert)
{
    return invert ? -value : value;
}

static void Task_Control(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();

    (void)argument;
    for (;;) {
        APP_SensorSnapshot sensors;
        APP_RunOutput output;
        BSP_EncoderSnapshot encoder;
        int32_t encoder_left;
        int32_t encoder_right;
        int32_t encoder_delta_left;
        int32_t encoder_delta_right;
        uint32_t now_ms;

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(APP_CONTROL_PERIOD_MS));
        now_ms = (uint32_t)xTaskGetTickCount();
        BSP_Encoder_Read(&encoder);
        encoder_left = APP_ENCODER_SWAP_LR ? encoder.count_b : encoder.count_a;
        encoder_right = APP_ENCODER_SWAP_LR ? encoder.count_a : encoder.count_b;
        encoder_delta_left = APP_ENCODER_SWAP_LR ?
            encoder.delta_b : encoder.delta_a;
        encoder_delta_right = APP_ENCODER_SWAP_LR ?
            encoder.delta_a : encoder.delta_b;

        taskENTER_CRITICAL();
        g_sensors.encoder_left = apply_encoder_invert(encoder_left,
            APP_ENCODER_LEFT_INVERT != 0);
        g_sensors.encoder_right = apply_encoder_invert(encoder_right,
            APP_ENCODER_RIGHT_INVERT != 0);
        g_sensors.encoder_delta_left = apply_encoder_invert(encoder_delta_left,
            APP_ENCODER_LEFT_INVERT != 0);
        g_sensors.encoder_delta_right = apply_encoder_invert(encoder_delta_right,
            APP_ENCODER_RIGHT_INVERT != 0);
        sensors = g_sensors;
        taskEXIT_CRITICAL();

        APP_Run_ControlStep(now_ms, &sensors, &output);
        if (output.motor_enable) {
            BSP_Motor_SetRaw(output.left_pwm, output.right_pwm);
            DL_GPIO_setPins(LED_PORT, LED_PIN_PIN);
        } else {
            BSP_Motor_Stop();
            DL_GPIO_clearPins(LED_PORT, LED_PIN_PIN);
        }
    }
}

static void Task_UI(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t next_lcd_ms = 0U;

    (void)argument;
    BSP_Key_Init((uint32_t)xTaskGetTickCount());
    APP_Menu_Init();

    for (;;) {
        uint32_t now_ms = (uint32_t)xTaskGetTickCount();
        APP_RunStatus status;
        BSP_KeyEvent key_event = BSP_Key_Scan(now_ms);

        APP_Run_GetStatus(&status);
        APP_Menu_Update(&status, now_ms);
        APP_Menu_HandleKey(key_event, now_ms);
        if (time_reached(now_ms, next_lcd_ms)) {
            APP_SensorSnapshot sensors;
            APP_Run_GetStatus(&status);
            APP_Sensors_Get(&sensors);
            APP_Menu_Render(&status, &sensors, now_ms);
            next_lcd_ms = now_ms + APP_LCD_PERIOD_MS;
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(APP_UI_PERIOD_MS));
    }
}

static void log_format_gray(uint8_t raw, char gray[9])
{
    uint8_t index;
    for (index = 0U; index < 8U; index++) {
        gray[index] = ((raw & (uint8_t)(0x80U >> index)) != 0U) ? '1' : '0';
    }
    gray[8] = '\0';
}

static int16_t log_calculate_raw_weight(uint8_t raw)
{
    static const int16_t weights[8] = {
        -350, -250, -150, -50, 50, 150, 250, 350
    };
    int32_t sum = 0;
    uint8_t count = 0U;
    uint8_t index;

    for (index = 0U; index < 8U; index++) {
        if ((raw & (uint8_t)(0x80U >> index)) == 0U) {
            sum += weights[index];
            count++;
        }
    }
    return (count == 0U) ? 0 : (int16_t)(sum / count);
}

static bool log_run_state_is_running(APP_RunState state)
{
    return state == APP_RUN_REMOTE || state == APP_RUN_LEAVE_START ||
           state == APP_RUN_FOLLOW || state == APP_RUN_FINISH_CONFIRM ||
           state == APP_RUN_ALIGN || state == APP_RUN_SOFT_STOP;
}

static const char *log_fault_name(APP_RunFault fault)
{
    switch (fault) {
    case APP_RUN_FAULT_NONE: return "NONE";
    case APP_RUN_FAULT_LINE_SENSOR: return "LINE_I2C";
    case APP_RUN_FAULT_LINE_LOST: return "LINE_LOST";
    case APP_RUN_FAULT_RUN_TIMEOUT: return "TIMEOUT";
    case APP_RUN_FAULT_STALL: return "STALL";
    default: return "UNKNOWN";
    }
}

typedef enum {
    LOG_TASK_EVENT_NONE = 0,
    LOG_TASK_EVENT_BEGIN,
    LOG_TASK_EVENT_END
} LogTaskEventType;

typedef struct {
    LogTaskEventType type;
    const char *task;
    uint32_t event_sequence;
    uint32_t run;
    uint32_t timestamp_ms;
    uint32_t elapsed_ms;
    uint32_t travel_counts;
    APP_RunState state;
    APP_RunFault fault;
    uint8_t repeat_index;
} LogTaskEvent;

static void log_task_event_schedule(LogTaskEvent *event,
    LogTaskEventType type, uint32_t event_sequence,
    uint32_t now_ms, const APP_RunStatus *status)
{
    *event = (LogTaskEvent){
        .type = type,
        .task = APP_Task_Name(status->task),
        .event_sequence = event_sequence,
        .run = status->run_number,
        .timestamp_ms = now_ms,
        .elapsed_ms = (status->finish_time_ms != 0U) ?
            status->finish_time_ms : status->elapsed_ms,
        .travel_counts = status->travel_counts,
        .state = status->state,
        .fault = status->fault,
        .repeat_index = 1U,
    };
}

static void log_task_event_send(LogTaskEvent *event)
{
    if (event->type == LOG_TASK_EVENT_BEGIN) {
        BSP_Log_Printf(
            "@TASK source=TMX eventSeq=%lu repeat=%u/%u"
            " type=TASK_BEGIN run=%lu t=%lu task=%s state=%s\n",
            (unsigned long)event->event_sequence,
            (unsigned)event->repeat_index,
            (unsigned)APP_TASK_EVENT_REPEAT_COUNT,
            (unsigned long)event->run,
            (unsigned long)event->timestamp_ms,
            event->task, APP_Run_StateName(event->state));
    } else if (event->type == LOG_TASK_EVENT_END) {
        BSP_Log_Printf(
            "@TASK source=TMX eventSeq=%lu repeat=%u/%u"
            " type=TASK_END run=%lu t=%lu task=%s"
            " result=%s fault=%s elapsed=%lu lapEnc=%lu\n",
            (unsigned long)event->event_sequence,
            (unsigned)event->repeat_index,
            (unsigned)APP_TASK_EVENT_REPEAT_COUNT,
            (unsigned long)event->run,
            (unsigned long)event->timestamp_ms,
            event->task, APP_Run_StateName(event->state),
            log_fault_name(event->fault),
            (unsigned long)event->elapsed_ms,
            (unsigned long)event->travel_counts);
    }

    event->repeat_index++;
    if (event->repeat_index > APP_TASK_EVENT_REPEAT_COUNT) {
        event->type = LOG_TASK_EVENT_NONE;
    }
}

typedef enum {
    LOG_MODE_EVENT_NONE = 0,
    LOG_MODE_EVENT_ENTER,
    LOG_MODE_EVENT_EXIT
} LogModeEventType;

typedef struct {
    LogModeEventType type;
    const char *mode;
    const char *task;
    uint32_t event_sequence;
    uint32_t timestamp_ms;
    uint8_t repeat_index;
} LogModeEvent;

static void log_mode_event_schedule(LogModeEvent *event,
    LogModeEventType type, const char *mode, const char *task,
    uint32_t event_sequence, uint32_t now_ms)
{
    *event = (LogModeEvent){
        .type = type,
        .mode = mode,
        .task = task,
        .event_sequence = event_sequence,
        .timestamp_ms = now_ms,
        .repeat_index = 1U,
    };
}

static void log_mode_event_send(LogModeEvent *event)
{
    const char *type_name = (event->type == LOG_MODE_EVENT_ENTER) ?
        "ENTER" : "EXIT";

    if (event->task != NULL) {
        BSP_Log_Printf(
            "@MODE source=TMX eventSeq=%lu repeat=%u/%u type=%s"
            " mode=%s task=%s t=%lu\n",
            (unsigned long)event->event_sequence,
            (unsigned)event->repeat_index,
            (unsigned)APP_MODE_EVENT_REPEAT_COUNT,
            type_name, event->mode, event->task,
            (unsigned long)event->timestamp_ms);
    } else {
        BSP_Log_Printf(
            "@MODE source=TMX eventSeq=%lu repeat=%u/%u"
            " type=%s mode=%s t=%lu\n",
            (unsigned long)event->event_sequence,
            (unsigned)event->repeat_index,
            (unsigned)APP_MODE_EVENT_REPEAT_COUNT,
            type_name, event->mode,
            (unsigned long)event->timestamp_ms);
    }

    event->repeat_index++;
    if (event->repeat_index > APP_MODE_EVENT_REPEAT_COUNT) {
        event->type = LOG_MODE_EVENT_NONE;
    }
}

static bool log_page_mode(APP_MenuPage page, APP_TaskId task,
    const char **mode, const char **task_name)
{
    if (page == APP_MENU_PAGE_TASK_PREP) {
        *mode = "PREP";
        *task_name = APP_Task_Name(task);
        return true;
    }
    if (page == APP_MENU_PAGE_TEST) {
        *mode = "TEST";
        *task_name = NULL;
        return true;
    }
    return false;
}

static void Task_Log(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t car_sequence = 0U;
    uint32_t protocol_event_sequence = 0U;
    uint32_t observed_run = 0U;
    uint32_t observed_checkpoint = 0U;
    APP_TaskId observed_task = APP_TASK_NONE;
    APP_MenuPage observed_page = APP_MENU_PAGE_MAIN;
    APP_TaskId observed_menu_task = APP_TASK_T1;
    bool run_open = false;
    static LogTaskEvent pending_task_event;
    static LogModeEvent pending_mode_event;
    /* 离开PREP/TEST的事件延迟到任务事件发完后补发，避免先于
       TASK_BEGIN 到达ESP32，导致其提前取消D36A使能。 */
    static LogModeEvent deferred_mode_event;

    (void)argument;

    for (;;) {
        APP_RunStatus status;
        APP_SensorSnapshot sensors;
        APP_MenuPage menu_page;
        APP_TaskId menu_task;
        char gray[9];
        char yaw_field[24];
        uint32_t now_ms;
        uint32_t line_age;
        uint32_t gyro_age;
        int16_t raw_weight;
        int16_t control_weight;
        bool sensor_ok;
        bool gyro_ok;
        bool protocol_event_sent = false;

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(APP_LOG_PERIOD_MS));
        now_ms = (uint32_t)xTaskGetTickCount();
        APP_Run_GetStatus(&status);
        APP_Sensors_Get(&sensors);
        APP_Menu_GetProtocolState(&menu_page, &menu_task);

        if (status.checkpoint_sequence != observed_checkpoint) {
            observed_checkpoint = status.checkpoint_sequence;
            BSP_Log_Printf(
                "@CHECKPOINT source=TMX eventSeq=%lu type=PASS"
                " run=%lu t=%lu task=%s point=%s elapsed=%lu lapEnc=%lu\n",
                (unsigned long)++protocol_event_sequence,
                (unsigned long)status.run_number,
                (unsigned long)now_ms,
                APP_Task_Name(status.task),
                APP_Checkpoint_Name(status.checkpoint),
                (unsigned long)status.checkpoint_elapsed_ms,
                (unsigned long)status.checkpoint_travel_counts);
            protocol_event_sent = true;
        }

        /* TASK事件先于离开PREP事件分配序号并发送。 */
        if (pending_task_event.type == LOG_TASK_EVENT_NONE) {
            if (status.run_number != observed_run) {
                observed_run = status.run_number;
                observed_task = status.task;
                run_open = true;
                log_task_event_schedule(&pending_task_event,
                    LOG_TASK_EVENT_BEGIN, ++protocol_event_sequence,
                    now_ms, &status);
            } else if (run_open &&
                !log_run_state_is_running(status.state)) {
                run_open = false;
                log_task_event_schedule(&pending_task_event,
                    LOG_TASK_EVENT_END, ++protocol_event_sequence,
                    now_ms, &status);
            }
        }

        if (pending_mode_event.type == LOG_MODE_EVENT_NONE &&
            (menu_page != observed_page ||
             (menu_page == APP_MENU_PAGE_TASK_PREP &&
              menu_task != observed_menu_task))) {
            const char *previous_mode = NULL;
            const char *previous_task = NULL;
            const char *current_mode = NULL;
            const char *current_task = NULL;
            bool exited_mode = log_page_mode(observed_page,
                observed_menu_task, &previous_mode, &previous_task);
            bool entered_mode = log_page_mode(menu_page,
                menu_task, &current_mode, &current_task);

            observed_page = menu_page;
            observed_menu_task = menu_task;
            if (exited_mode) {
                log_mode_event_schedule(&deferred_mode_event,
                    LOG_MODE_EVENT_EXIT, previous_mode, previous_task,
                    ++protocol_event_sequence, now_ms);
            } else if (entered_mode) {
                log_mode_event_schedule(&pending_mode_event,
                    LOG_MODE_EVENT_ENTER, current_mode, current_task,
                    ++protocol_event_sequence, now_ms);
            }
        }

        if (pending_task_event.type != LOG_TASK_EVENT_NONE) {
            log_task_event_send(&pending_task_event);
            protocol_event_sent = true;
        }
        if (pending_mode_event.type != LOG_MODE_EVENT_NONE) {
            log_mode_event_send(&pending_mode_event);
            protocol_event_sent = true;
        }
        /* 任务事件(BEGIN/END)发送期间挂起EXIT；队列空闲后补发，
           保证ESP32先收到任务开始再收到PREP退出。 */
        if (deferred_mode_event.type != LOG_MODE_EVENT_NONE &&
            pending_task_event.type == LOG_TASK_EVENT_NONE &&
            pending_mode_event.type == LOG_MODE_EVENT_NONE) {
            pending_mode_event = deferred_mode_event;
            deferred_mode_event = (LogModeEvent){0};
        }
        if (protocol_event_sent) continue;
        if (!run_open || !APP_Task_UsesChassis(observed_task)) continue;

        line_age = sensors.line_valid ?
            now_ms - sensors.line_timestamp_ms : 0xFFFFFFFFUL;
        gyro_age = sensors.gyro_valid ?
            now_ms - sensors.gyro_timestamp_ms : 0xFFFFFFFFUL;
        sensor_ok = sensors.line_valid && line_age <= LINE_STALE_MS;
        gyro_ok = sensors.gyro_valid && gyro_age <= APP_GYRO_STALE_MS;
        log_format_gray(status.line_raw, gray);
        raw_weight = log_calculate_raw_weight(status.line_raw);
        control_weight = (int16_t)(status.line_error * 50);
        yaw_field[0] = '\0';
        if (gyro_ok) {
            (void)snprintf(yaw_field, sizeof(yaw_field),
                " yaw10=%d", (int)sensors.yaw_x10);
        }
        car_sequence++;

        BSP_Log_Printf(
            "@CAR schema=2 source=GRAY seq=%lu t=%lu task=%s"
            " sensorOk=%u gray=%s grayValid=%u rawWeight=%d weight=%d"
            "%s angle=0 vValid=0 vL=0.0 vR=0.0 state=%s"
            " pwmL=%d pwmR=%d encL=%ld encR=%ld dEncL=%ld dEncR=%ld"
            " lapEnc=%lu\n",
            (unsigned long)car_sequence, (unsigned long)now_ms,
            APP_Task_Name(status.task), sensor_ok ? 1U : 0U, gray,
            (status.line_raw != 0xFFU) ? 1U : 0U,
            (int)raw_weight, (int)control_weight, yaw_field,
            APP_Run_StateName(status.state),
            (int)status.left_pwm, (int)status.right_pwm,
            (long)sensors.encoder_left, (long)sensors.encoder_right,
            (long)sensors.encoder_delta_left,
            (long)sensors.encoder_delta_right,
            (unsigned long)status.travel_counts);
    }
}

void APP_Tasks_Create(void)
{
    TaskHandle_t handle;

    g_sensors = (APP_SensorSnapshot){0};
    g_sensors.line_raw = 0xFFU;

    handle = xTaskCreateStatic(Task_Sensor, "Sensor",
        APP_TASK_STACK_SENSOR, NULL, APP_TASK_PRIO_SENSOR,
        g_sensor_stack, &g_sensor_tcb);
    configASSERT(handle != NULL);
    handle = xTaskCreateStatic(Task_Control, "Control",
        APP_TASK_STACK_CONTROL, NULL, APP_TASK_PRIO_CONTROL,
        g_control_stack, &g_control_tcb);
    configASSERT(handle != NULL);
    handle = xTaskCreateStatic(Task_UI, "UI",
        APP_TASK_STACK_UI, NULL, APP_TASK_PRIO_UI,
        g_ui_stack, &g_ui_tcb);
    configASSERT(handle != NULL);
    handle = xTaskCreateStatic(Task_Log, "Log",
        APP_TASK_STACK_LOG, NULL, APP_TASK_PRIO_LOG,
        g_log_stack, &g_log_tcb);
    configASSERT(handle != NULL);
}
