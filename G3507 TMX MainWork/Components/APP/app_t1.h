/**
 * @file app_t1.h
 * @brief 通用TMX任务运行器（文件名保留以兼容现有CCS工程）
 */

#ifndef APP_RUN_H
#define APP_RUN_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    APP_TASK_NONE = 0,
    APP_TASK_T1 = 1,
    APP_TASK_T2,
    APP_TASK_T3,
    APP_TASK_T4,
    APP_TASK_T5,
    APP_TASK_T6
} APP_TaskId;

typedef enum {
    APP_RUN_IDLE = 0,
    APP_RUN_REMOTE,
    APP_RUN_LEAVE_START,
    APP_RUN_FOLLOW,
    APP_RUN_FINISH_CONFIRM,
    APP_RUN_ALIGN,
    APP_RUN_SOFT_STOP,
    APP_RUN_DONE,
    APP_RUN_ABORTED,
    APP_RUN_FAULT
} APP_RunState;

typedef enum {
    APP_RUN_FAULT_NONE = 0,
    APP_RUN_FAULT_LINE_SENSOR,
    APP_RUN_FAULT_LINE_LOST,
    APP_RUN_FAULT_RUN_TIMEOUT,
    APP_RUN_FAULT_STALL
} APP_RunFault;

typedef enum {
    APP_CHECKPOINT_NONE = 0,
    APP_CHECKPOINT_A,
    APP_CHECKPOINT_B
} APP_Checkpoint;

typedef struct {
    bool line_valid;
    uint8_t line_raw;
    uint32_t line_timestamp_ms;
    uint32_t line_error_count;
    bool gyro_valid;
    int16_t roll_x10;
    int16_t pitch_x10;
    int16_t yaw_x10;
    uint32_t gyro_timestamp_ms;
    uint32_t gyro_error_count;
    int32_t encoder_left;
    int32_t encoder_right;
    int32_t encoder_delta_left;
    int32_t encoder_delta_right;
} APP_SensorSnapshot;

typedef struct {
    bool motor_enable;
    int16_t left_pwm;
    int16_t right_pwm;
} APP_RunOutput;

typedef struct {
    APP_TaskId task;
    APP_RunState state;
    APP_RunFault fault;
    uint32_t elapsed_ms;
    uint32_t finish_time_ms;
    uint8_t line_raw;
    uint8_t black_count;
    int16_t line_error;
    int16_t left_pwm;
    int16_t right_pwm;
    uint32_t travel_counts;
    uint32_t run_number;
    uint32_t checkpoint_sequence;
    APP_Checkpoint checkpoint;
    uint32_t checkpoint_elapsed_ms;
    uint32_t checkpoint_travel_counts;
} APP_RunStatus;

void APP_Run_Init(void);
void APP_Run_RequestStart(APP_TaskId task);
void APP_Run_RequestFinish(void);
void APP_Run_RequestAbort(void);
void APP_Run_ControlStep(uint32_t now_ms,
    const APP_SensorSnapshot *sensors, APP_RunOutput *output);
void APP_Run_GetStatus(APP_RunStatus *status);
bool APP_Run_IsRunning(void);
bool APP_Task_UsesChassis(APP_TaskId task);
const char *APP_Task_Name(APP_TaskId task);
const char *APP_Run_StateName(APP_RunState state);
const char *APP_Run_FaultName(APP_RunFault fault);
const char *APP_Checkpoint_Name(APP_Checkpoint checkpoint);

#endif
