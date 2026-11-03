/**
 * @file app_config.h
 * @brief TMX任务周期、巡线公共参数和分任务速度档
 */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* FreeRTOS周期与优先级。 */
#define APP_SENSOR_PERIOD_MS          1U
#define APP_CONTROL_PERIOD_MS         5U
#define APP_UI_PERIOD_MS             10U
#define APP_LCD_PERIOD_MS            250U
#define APP_LOG_PERIOD_MS            150U
#define APP_TASK_EVENT_REPEAT_COUNT   3U
#define APP_MODE_EVENT_REPEAT_COUNT   3U
#define APP_GYRO_STALE_MS            100U

#define APP_TASK_PRIO_LOG            1U
#define APP_TASK_PRIO_UI             2U
#define APP_TASK_PRIO_SENSOR         3U
#define APP_TASK_PRIO_CONTROL        4U

#define APP_TASK_STACK_SENSOR        256U
#define APP_TASK_STACK_CONTROL       384U
#define APP_TASK_STACK_UI            512U
#define APP_TASK_STACK_LOG           512U

/* 实车编码器左右插反：ENC2=左轮、ENC1=右轮；前进统一换算为正。 */
#define APP_ENCODER_SWAP_LR           1
#define APP_ENCODER_LEFT_INVERT       0
#define APP_ENCODER_RIGHT_INVERT      1

/* 八路探头公共参数：X1(bit7)到X8(bit0)，0表示黑线。 */
#define LINE_WEIGHT_X1               (-7)
#define LINE_WEIGHT_X2               (-5)
#define LINE_WEIGHT_X3               (-3)
#define LINE_WEIGHT_X4               (-1)
#define LINE_WEIGHT_X5                1
#define LINE_WEIGHT_X6                3
#define LINE_WEIGHT_X7                5
#define LINE_WEIGHT_X8                7
#define LINE_CROSS_BLACK_COUNT         3U
#define LINE_LEAVE_CONFIRM_CYCLES      3U
#define LINE_FINISH_CONFIRM_CYCLES     3U
#define LINE_STALE_MS                 50U
#define LINE_LOST_MS                 400U
#define LINE_STALL_ENABLE              1U
#define LINE_STALL_ARM_MS            800U
#define LINE_STALL_CONFIRM_MS        600U
#define LINE_STALL_PWM_THRESHOLD      160

/* T2：第一次快速循迹，完整保留v3实测参数。 */
#define T2_BASE_PWM                   320
#define T2_SEARCH_PWM                 170
#define T2_MAX_PWM                    600
#define T2_MAX_CORRECTION             340
#define T2_KP                          32
#define T2_KD                          24
#define T2_PWM_SLEW_PER_STEP           30
#define T2_ENABLE_LEAVE_START           0U
#define T2_MIN_FINISH_TIME_MS        3000U
#define T2_RUN_TIMEOUT_MS           30000U
#define T2_APPROACH_COUNTS          10050U
#define T2_APPROACH_BASE_PWM          220
#define T2_FINISH_ENCODER_COUNTS    10350U
#define T2_ALIGN_COUNTS                 0U  /* 软停本身约滑行半个车身，不再额外前行300 count。 */
#define T2_SOFT_STOP_MS              1500U

/* T3：TMX只负责固定5 s任务窗口和结束事件，底盘保持停止。 */
#define T3_RUN_DURATION_MS            5000U

/* T4：A到B降速循迹；B点2.7k锁存，继续前行后安全停车。 */
#define T4_BASE_PWM                   250
#define T4_LAUNCH_START_PWM           120
#define T4_LAUNCH_RAMP_MS            1200U
#define T4_LAUNCH_PWM_SLEW_PER_STEP     2
#define T4_SEARCH_PWM                 130
#define T4_MAX_PWM                    420
#define T4_MAX_CORRECTION             220
#define T4_KP                          26
#define T4_KD                          18
#define T4_PWM_SLEW_PER_STEP           18
#define T4_ENABLE_LEAVE_START           0U
#define T4_MIN_FINISH_TIME_MS        1000U
#define T4_RUN_TIMEOUT_MS           12000U
#define T4_B_PASS_ENCODER_COUNT      2700U
#define T4_APPROACH_COUNTS           2400U
#define T4_APPROACH_BASE_PWM          180
#define T4_APPROACH_PWM_SLEW_PER_STEP   4
#define T4_FINISH_ENCODER_COUNTS     2850U
#define T4_SOFT_STOP_MS              1500U

/* T5：降速整圈并稳球，参数与T2完全独立。 */
#define T5_BASE_PWM                   220
#define T5_LAUNCH_START_PWM           120
#define T5_LAUNCH_RAMP_MS            1200U
#define T5_LAUNCH_PWM_SLEW_PER_STEP     2
#define T5_SEARCH_PWM                 130
#define T5_MAX_PWM                    420
#define T5_MAX_CORRECTION             220
#define T5_KP                          26
#define T5_KD                          18
#define T5_PWM_SLEW_PER_STEP           18
#define T5_ENABLE_LEAVE_START           0U
#define T5_MIN_FINISH_TIME_MS        3000U
#define T5_RUN_TIMEOUT_MS           35000U
#define T5_APPROACH_COUNTS           9950U
#define T5_APPROACH_BASE_PWM          170
#define T5_APPROACH_PWM_SLEW_PER_STEP   4
#define T5_FINISH_ENCODER_COUNTS    10350U
#define T5_ALIGN_COUNTS               300U  /* 通过A点后继续循迹约16 cm。 */
#define T5_ALIGN_END_PWM              100
#define T5_SOFT_STOP_MS              1500U

/* T6与T5共用底盘参数，只在ESP32侧保留初始位置参考点差异。 */
#define T6_BASE_PWM                  T5_BASE_PWM
#define T6_LAUNCH_START_PWM          T5_LAUNCH_START_PWM
#define T6_LAUNCH_RAMP_MS            T5_LAUNCH_RAMP_MS
#define T6_LAUNCH_PWM_SLEW_PER_STEP  T5_LAUNCH_PWM_SLEW_PER_STEP
#define T6_SEARCH_PWM                T5_SEARCH_PWM
#define T6_MAX_PWM                   T5_MAX_PWM
#define T6_MAX_CORRECTION            T5_MAX_CORRECTION
#define T6_KP                        T5_KP
#define T6_KD                        T5_KD
#define T6_PWM_SLEW_PER_STEP         T5_PWM_SLEW_PER_STEP
#define T6_ENABLE_LEAVE_START        T5_ENABLE_LEAVE_START
#define T6_MIN_FINISH_TIME_MS        T5_MIN_FINISH_TIME_MS
#define T6_RUN_TIMEOUT_MS            T5_RUN_TIMEOUT_MS
#define T6_APPROACH_COUNTS           T5_APPROACH_COUNTS
#define T6_APPROACH_BASE_PWM         T5_APPROACH_BASE_PWM
#define T6_APPROACH_PWM_SLEW_PER_STEP T5_APPROACH_PWM_SLEW_PER_STEP
#define T6_FINISH_ENCODER_COUNTS     T5_FINISH_ENCODER_COUNTS
#define T6_ALIGN_COUNTS              T5_ALIGN_COUNTS
#define T6_ALIGN_END_PWM             T5_ALIGN_END_PWM
#define T6_SOFT_STOP_MS              T5_SOFT_STOP_MS

#endif
