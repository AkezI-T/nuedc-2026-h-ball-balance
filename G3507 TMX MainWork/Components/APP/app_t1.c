/**
 * @file app_t1.c
 * @brief T1-T6通用运行状态与T2/T4/T5/T6巡线控制
 */

#include "app_t1.h"
#include "app_config.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stddef.h>

typedef struct {
    APP_TaskId task;
    int16_t base_pwm;
    int16_t search_pwm;
    int16_t max_pwm;
    int16_t max_correction;
    int16_t kp;
    int16_t kd;
    int16_t slew_step;
    uint32_t enable_leave_start;
    uint32_t minimum_finish_ms;
    uint32_t timeout_ms;
    uint32_t approach_counts;
    int16_t approach_pwm;
    uint32_t finish_counts;
    uint32_t align_counts;
    uint32_t soft_stop_ms;
    bool finish_on_encoder;
    bool finish_on_cross;
    APP_Checkpoint checkpoint;
    uint32_t checkpoint_counts;
} APP_LineProfile;

typedef struct {
    APP_RunStatus status;
    const APP_LineProfile *profile;
    volatile APP_TaskId requested_task;
    volatile bool start_request;
    volatile bool finish_request;
    volatile bool abort_request;
    uint32_t start_ms;
    uint32_t align_start_counts;
    int16_t align_start_pwm;
    uint32_t soft_stop_started_ms;
    int16_t soft_stop_start_left_pwm;
    int16_t soft_stop_start_right_pwm;
    int32_t start_encoder_left;
    int32_t start_encoder_right;
    uint16_t lost_cycles;
    uint16_t stall_cycles;
    uint8_t leave_cycles;
    uint8_t finish_cycles;
    int16_t last_error;
    int16_t command_left;
    int16_t command_right;
} APP_RunContext;

static const APP_LineProfile g_line_profiles[] = {
    {
        APP_TASK_T2, T2_BASE_PWM, T2_SEARCH_PWM, T2_MAX_PWM,
        T2_MAX_CORRECTION, T2_KP, T2_KD, T2_PWM_SLEW_PER_STEP,
        T2_ENABLE_LEAVE_START, T2_MIN_FINISH_TIME_MS, T2_RUN_TIMEOUT_MS,
        T2_APPROACH_COUNTS, T2_APPROACH_BASE_PWM,
        T2_FINISH_ENCODER_COUNTS, T2_ALIGN_COUNTS, T2_SOFT_STOP_MS,
        true, true, APP_CHECKPOINT_NONE, 0U
    },
    {
        APP_TASK_T4, T4_BASE_PWM, T4_SEARCH_PWM, T4_MAX_PWM,
        T4_MAX_CORRECTION, T4_KP, T4_KD, T4_PWM_SLEW_PER_STEP,
        T4_ENABLE_LEAVE_START, T4_MIN_FINISH_TIME_MS, T4_RUN_TIMEOUT_MS,
        T4_APPROACH_COUNTS, T4_APPROACH_BASE_PWM,
        T4_FINISH_ENCODER_COUNTS, 0U, T4_SOFT_STOP_MS,
        true, false, APP_CHECKPOINT_B, T4_B_PASS_ENCODER_COUNT
    },
    {
        APP_TASK_T5, T5_BASE_PWM, T5_SEARCH_PWM, T5_MAX_PWM,
        T5_MAX_CORRECTION, T5_KP, T5_KD, T5_PWM_SLEW_PER_STEP,
        T5_ENABLE_LEAVE_START, T5_MIN_FINISH_TIME_MS, T5_RUN_TIMEOUT_MS,
        T5_APPROACH_COUNTS, T5_APPROACH_BASE_PWM,
        T5_FINISH_ENCODER_COUNTS, T5_ALIGN_COUNTS, T5_SOFT_STOP_MS,
        false, true, APP_CHECKPOINT_A, 0U
    },
    {
        APP_TASK_T6, T6_BASE_PWM, T6_SEARCH_PWM, T6_MAX_PWM,
        T6_MAX_CORRECTION, T6_KP, T6_KD, T6_PWM_SLEW_PER_STEP,
        T6_ENABLE_LEAVE_START, T6_MIN_FINISH_TIME_MS, T6_RUN_TIMEOUT_MS,
        T6_APPROACH_COUNTS, T6_APPROACH_BASE_PWM,
        T6_FINISH_ENCODER_COUNTS, T6_ALIGN_COUNTS, T6_SOFT_STOP_MS,
        false, true, APP_CHECKPOINT_A, 0U
    }
};

static const int8_t g_line_weights[8] = {
    LINE_WEIGHT_X1, LINE_WEIGHT_X2, LINE_WEIGHT_X3, LINE_WEIGHT_X4,
    LINE_WEIGHT_X5, LINE_WEIGHT_X6, LINE_WEIGHT_X7, LINE_WEIGHT_X8
};

static APP_RunContext g_run;

static const APP_LineProfile *profile_for_task(APP_TaskId task)
{
    uint32_t index;
    for (index = 0U;
         index < (sizeof(g_line_profiles) / sizeof(g_line_profiles[0]));
         index++) {
        if (g_line_profiles[index].task == task) return &g_line_profiles[index];
    }
    return NULL;
}

static int16_t clamp_i16(int16_t value, int16_t minimum, int16_t maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static int16_t slew(int16_t current, int16_t target, int16_t step)
{
    if (target > (int16_t)(current + step)) return (int16_t)(current + step);
    if (target < (int16_t)(current - step)) return (int16_t)(current - step);
    return target;
}

static int16_t absolute_i16(int16_t value)
{
    return (value < 0) ? (int16_t)-value : value;
}

static uint32_t encoder_delta_abs(int32_t current, int32_t start)
{
    return (current >= start) ?
        (uint32_t)(current - start) : (uint32_t)(start - current);
}

static uint32_t encoder_travel_counts(const APP_SensorSnapshot *sensors)
{
    uint32_t left = encoder_delta_abs(
        sensors->encoder_left, g_run.start_encoder_left);
    uint32_t right = encoder_delta_abs(
        sensors->encoder_right, g_run.start_encoder_right);
    return (left + right) / 2U;
}

static void decode_line(uint8_t raw, uint8_t *black_count, int16_t *error)
{
    int16_t sum = 0;
    uint8_t count = 0U;
    uint8_t index;

    for (index = 0U; index < 8U; index++) {
        uint8_t mask = (uint8_t)(0x80U >> index);
        if ((raw & mask) == 0U) {
            sum = (int16_t)(sum + g_line_weights[index]);
            count++;
        }
    }
    *black_count = count;
    *error = (count == 0U) ? 0 : (int16_t)(sum / count);
}

static bool state_is_running(APP_RunState state)
{
    return state == APP_RUN_REMOTE || state == APP_RUN_LEAVE_START ||
           state == APP_RUN_FOLLOW || state == APP_RUN_FINISH_CONFIRM ||
           state == APP_RUN_ALIGN || state == APP_RUN_SOFT_STOP;
}

static bool state_uses_motor(APP_RunState state)
{
    return state == APP_RUN_LEAVE_START || state == APP_RUN_FOLLOW ||
           state == APP_RUN_FINISH_CONFIRM || state == APP_RUN_ALIGN ||
           state == APP_RUN_SOFT_STOP;
}

static void stop_with_state(APP_RunState state, APP_RunFault fault)
{
    g_run.status.state = state;
    g_run.status.fault = fault;
    g_run.command_left = 0;
    g_run.command_right = 0;
    g_run.status.left_pwm = 0;
    g_run.status.right_pwm = 0;
}

static void latch_checkpoint(APP_Checkpoint checkpoint)
{
    if (checkpoint == APP_CHECKPOINT_NONE ||
        g_run.status.checkpoint != APP_CHECKPOINT_NONE) return;

    g_run.status.checkpoint = checkpoint;
    g_run.status.checkpoint_elapsed_ms = g_run.status.elapsed_ms;
    g_run.status.checkpoint_travel_counts = g_run.status.travel_counts;
    g_run.status.checkpoint_sequence++;
}

static void finish_success(void)
{
    if (g_run.status.task == APP_TASK_T5 ||
        g_run.status.task == APP_TASK_T6) {
        latch_checkpoint(APP_CHECKPOINT_A);
    }
    if (g_run.status.finish_time_ms == 0U) {
        g_run.status.finish_time_ms =
            (g_run.status.checkpoint != APP_CHECKPOINT_NONE) ?
            g_run.status.checkpoint_elapsed_ms : g_run.status.elapsed_ms;
    }
    stop_with_state(APP_RUN_DONE, APP_RUN_FAULT_NONE);
}

static void begin_soft_stop(uint32_t now_ms)
{
    if (g_run.profile == NULL || g_run.profile->soft_stop_ms == 0U) {
        finish_success();
        return;
    }
    g_run.soft_stop_started_ms = now_ms;
    g_run.soft_stop_start_left_pwm = g_run.command_left;
    g_run.soft_stop_start_right_pwm = g_run.command_right;
    g_run.stall_cycles = 0U;
    g_run.status.state = APP_RUN_SOFT_STOP;
}

static void begin_align_or_finish(uint32_t now_ms)
{
    if (g_run.profile == NULL) {
        finish_success();
        return;
    }

    if (g_run.status.task == APP_TASK_T5 ||
        g_run.status.task == APP_TASK_T6) {
        latch_checkpoint(APP_CHECKPOINT_A);
    }
    g_run.status.finish_time_ms =
        (g_run.status.checkpoint != APP_CHECKPOINT_NONE) ?
        g_run.status.checkpoint_elapsed_ms : g_run.status.elapsed_ms;
    if (g_run.profile->align_counts == 0U) {
        begin_soft_stop(now_ms);
        return;
    }

    g_run.align_start_counts = g_run.status.travel_counts;
    if (g_run.status.task == APP_TASK_T5 ||
        g_run.status.task == APP_TASK_T6) {
        int16_t average_pwm = (int16_t)(
            (absolute_i16(g_run.command_left) +
             absolute_i16(g_run.command_right)) / 2);
        g_run.align_start_pwm = average_pwm > T5_ALIGN_END_PWM ?
            average_pwm : T5_ALIGN_END_PWM;
    }
    g_run.finish_cycles = 0U;
    g_run.status.state = APP_RUN_ALIGN;
}

static void begin_run(APP_TaskId task, uint32_t now_ms,
    int16_t initial_error, int32_t encoder_left, int32_t encoder_right)
{
    g_run.profile = profile_for_task(task);
    g_run.status.task = task;
    g_run.status.state = (g_run.profile == NULL) ? APP_RUN_REMOTE :
        (g_run.profile->enable_leave_start ?
         APP_RUN_LEAVE_START : APP_RUN_FOLLOW);
    g_run.status.fault = APP_RUN_FAULT_NONE;
    g_run.status.elapsed_ms = 0U;
    g_run.status.finish_time_ms = 0U;
    g_run.status.travel_counts = 0U;
    g_run.status.checkpoint = APP_CHECKPOINT_NONE;
    g_run.status.checkpoint_elapsed_ms = 0U;
    g_run.status.checkpoint_travel_counts = 0U;
    g_run.status.run_number++;
    g_run.start_ms = now_ms;
    g_run.align_start_counts = 0U;
    g_run.align_start_pwm = 0;
    g_run.soft_stop_started_ms = 0U;
    g_run.soft_stop_start_left_pwm = 0;
    g_run.soft_stop_start_right_pwm = 0;
    g_run.start_encoder_left = encoder_left;
    g_run.start_encoder_right = encoder_right;
    g_run.lost_cycles = 0U;
    g_run.stall_cycles = 0U;
    g_run.leave_cycles = 0U;
    g_run.finish_cycles = 0U;
    g_run.last_error = initial_error;
    g_run.command_left = 0;
    g_run.command_right = 0;
}

void APP_Run_Init(void)
{
    g_run = (APP_RunContext){0};
    g_run.status.state = APP_RUN_IDLE;
}

void APP_Run_RequestStart(APP_TaskId task)
{
    g_run.abort_request = false;
    g_run.finish_request = false;
    g_run.requested_task = task;
    g_run.start_request = true;
}

void APP_Run_RequestFinish(void)
{
    g_run.finish_request = true;
}

void APP_Run_RequestAbort(void)
{
    g_run.start_request = false;
    g_run.finish_request = false;
    g_run.abort_request = true;
}

void APP_Run_ControlStep(uint32_t now_ms,
    const APP_SensorSnapshot *sensors, APP_RunOutput *output)
{
    uint8_t black_count = 0U;
    int16_t line_error = 0;
    int16_t target_left = 0;
    int16_t target_right = 0;
    bool line_fresh;

    if ((sensors == NULL) || (output == NULL)) return;

    decode_line(sensors->line_raw, &black_count, &line_error);
    g_run.status.line_raw = sensors->line_raw;
    g_run.status.black_count = black_count;
    g_run.status.line_error = line_error;
    line_fresh = sensors->line_valid &&
        ((uint32_t)(now_ms - sensors->line_timestamp_ms) <= LINE_STALE_MS);

    if (g_run.abort_request) {
        g_run.abort_request = false;
        if (state_is_running(g_run.status.state)) {
            stop_with_state(APP_RUN_ABORTED, APP_RUN_FAULT_NONE);
        }
    }

    if (g_run.finish_request) {
        g_run.finish_request = false;
        if (state_uses_motor(g_run.status.state) && g_run.profile != NULL) {
            begin_soft_stop(now_ms);
        } else if (state_is_running(g_run.status.state)) {
            finish_success();
        }
    }

    if (g_run.start_request) {
        APP_TaskId task = g_run.requested_task;
        g_run.start_request = false;
        begin_run(task, now_ms, line_error,
            sensors->encoder_left, sensors->encoder_right);
        if (g_run.profile != NULL && !line_fresh) {
            stop_with_state(APP_RUN_FAULT, APP_RUN_FAULT_LINE_SENSOR);
        }
    }

    if (state_is_running(g_run.status.state)) {
        g_run.status.elapsed_ms = now_ms - g_run.start_ms;
        g_run.status.travel_counts = encoder_travel_counts(sensors);
        if (g_run.status.task == APP_TASK_T3 &&
            g_run.status.elapsed_ms >= T3_RUN_DURATION_MS) {
            finish_success();
        }
    }

    if (state_uses_motor(g_run.status.state) && g_run.profile != NULL) {
        if (g_run.status.state == APP_RUN_SOFT_STOP) {
            if ((uint32_t)(now_ms - g_run.soft_stop_started_ms) >=
                g_run.profile->soft_stop_ms) {
                finish_success();
            }
        } else if (!line_fresh) {
            stop_with_state(APP_RUN_FAULT, APP_RUN_FAULT_LINE_SENSOR);
        } else if (g_run.status.elapsed_ms >= g_run.profile->timeout_ms) {
            stop_with_state(APP_RUN_FAULT, APP_RUN_FAULT_RUN_TIMEOUT);
        } else {
            if (g_run.profile->checkpoint == APP_CHECKPOINT_B &&
                g_run.status.travel_counts >=
                g_run.profile->checkpoint_counts) {
                latch_checkpoint(APP_CHECKPOINT_B);
            }
            if (g_run.profile->finish_on_encoder &&
                g_run.status.state != APP_RUN_ALIGN &&
                g_run.status.elapsed_ms >= g_run.profile->minimum_finish_ms &&
                g_run.status.travel_counts >= g_run.profile->finish_counts) {
                begin_align_or_finish(now_ms);
            } else if (g_run.status.state == APP_RUN_ALIGN &&
                g_run.status.travel_counts >= g_run.align_start_counts &&
                (g_run.status.travel_counts - g_run.align_start_counts) >=
                    g_run.profile->align_counts) {
                begin_soft_stop(now_ms);
            }
        }
    }

    if (state_uses_motor(g_run.status.state) && g_run.profile != NULL) {
        if (g_run.status.state == APP_RUN_SOFT_STOP) {
            uint32_t soft_stop_elapsed_ms =
                now_ms - g_run.soft_stop_started_ms;
            uint32_t remaining_ms = soft_stop_elapsed_ms <
                    g_run.profile->soft_stop_ms ?
                g_run.profile->soft_stop_ms - soft_stop_elapsed_ms : 0U;
            target_left = (int16_t)(
                (int32_t)g_run.soft_stop_start_left_pwm *
                (int32_t)remaining_ms /
                (int32_t)g_run.profile->soft_stop_ms);
            target_right = (int16_t)(
                (int32_t)g_run.soft_stop_start_right_pwm *
                (int32_t)remaining_ms /
                (int32_t)g_run.profile->soft_stop_ms);
            g_run.lost_cycles = 0U;
        } else if (black_count == 0U) {
            g_run.lost_cycles++;
            if ((uint32_t)g_run.lost_cycles * APP_CONTROL_PERIOD_MS >=
                LINE_LOST_MS) {
                stop_with_state(APP_RUN_FAULT, APP_RUN_FAULT_LINE_LOST);
            } else if (g_run.last_error < 0) {
                target_left = g_run.profile->search_pwm / 2;
                target_right = g_run.profile->search_pwm;
            } else {
                target_left = g_run.profile->search_pwm;
                target_right = g_run.profile->search_pwm / 2;
            }
        } else {
            int16_t base_pwm = g_run.profile->base_pwm;
            int16_t derivative = (int16_t)(line_error - g_run.last_error);
            int16_t correction;

            if (g_run.status.task == APP_TASK_T4 &&
                g_run.status.elapsed_ms < T4_LAUNCH_RAMP_MS) {
                uint32_t launch_range =
                    (uint32_t)(T4_BASE_PWM - T4_LAUNCH_START_PWM);
                base_pwm = (int16_t)(T4_LAUNCH_START_PWM +
                    (int32_t)(launch_range * g_run.status.elapsed_ms /
                              T4_LAUNCH_RAMP_MS));
            } else if ((g_run.status.task == APP_TASK_T5 ||
                        g_run.status.task == APP_TASK_T6) &&
                       g_run.status.elapsed_ms < T5_LAUNCH_RAMP_MS) {
                uint32_t launch_range =
                    (uint32_t)(T5_BASE_PWM - T5_LAUNCH_START_PWM);
                base_pwm = (int16_t)(T5_LAUNCH_START_PWM +
                    (int32_t)(launch_range * g_run.status.elapsed_ms /
                              T5_LAUNCH_RAMP_MS));
            }
            if (g_run.status.task == APP_TASK_T4 &&
                g_run.status.travel_counts >= T4_APPROACH_COUNTS &&
                g_run.status.travel_counts < T4_B_PASS_ENCODER_COUNT) {
                uint32_t approach_progress =
                    g_run.status.travel_counts - T4_APPROACH_COUNTS;
                uint32_t approach_range =
                    T4_B_PASS_ENCODER_COUNT - T4_APPROACH_COUNTS;
                uint32_t pwm_drop =
                    (uint32_t)(T4_BASE_PWM - T4_APPROACH_BASE_PWM);
                base_pwm = (int16_t)(T4_BASE_PWM -
                    (int32_t)(pwm_drop * approach_progress /
                              approach_range));
            } else if ((g_run.status.task == APP_TASK_T5 ||
                        g_run.status.task == APP_TASK_T6) &&
                       g_run.status.travel_counts >= T5_APPROACH_COUNTS &&
                       g_run.status.travel_counts <
                           T5_FINISH_ENCODER_COUNTS) {
                uint32_t approach_progress =
                    g_run.status.travel_counts - T5_APPROACH_COUNTS;
                uint32_t approach_range =
                    T5_FINISH_ENCODER_COUNTS - T5_APPROACH_COUNTS;
                uint32_t pwm_drop =
                    (uint32_t)(T5_BASE_PWM - T5_APPROACH_BASE_PWM);
                base_pwm = (int16_t)(T5_BASE_PWM -
                    (int32_t)(pwm_drop * approach_progress /
                              approach_range));
            } else if (g_run.status.travel_counts >=
                       g_run.profile->approach_counts) {
                base_pwm = g_run.profile->approach_pwm;
            }
            if ((g_run.status.task == APP_TASK_T5 ||
                 g_run.status.task == APP_TASK_T6) &&
                g_run.status.state == APP_RUN_ALIGN &&
                g_run.profile->align_counts > 0U) {
                uint32_t align_progress =
                    g_run.status.travel_counts - g_run.align_start_counts;
                if (align_progress > g_run.profile->align_counts) {
                    align_progress = g_run.profile->align_counts;
                }
                int16_t align_start_pwm = g_run.align_start_pwm >
                        T5_ALIGN_END_PWM ?
                    g_run.align_start_pwm : T5_ALIGN_END_PWM;
                uint32_t align_pwm_drop = (uint32_t)(
                    align_start_pwm - T5_ALIGN_END_PWM);
                base_pwm = (int16_t)(align_start_pwm -
                    (int32_t)(align_pwm_drop * align_progress /
                              g_run.profile->align_counts));
            }
            g_run.lost_cycles = 0U;
            correction = (int16_t)(g_run.profile->kp * line_error +
                g_run.profile->kd * derivative);
            correction = clamp_i16(correction,
                -g_run.profile->max_correction,
                g_run.profile->max_correction);
            target_left = clamp_i16((int16_t)(base_pwm + correction),
                0, g_run.profile->max_pwm);
            target_right = clamp_i16((int16_t)(base_pwm - correction),
                0, g_run.profile->max_pwm);
            g_run.last_error = line_error;

            if (g_run.status.state == APP_RUN_LEAVE_START) {
                if (black_count < LINE_CROSS_BLACK_COUNT) {
                    if (++g_run.leave_cycles >= LINE_LEAVE_CONFIRM_CYCLES) {
                        g_run.status.state = APP_RUN_FOLLOW;
                    }
                } else {
                    g_run.leave_cycles = 0U;
                }
            } else if (g_run.profile->finish_on_cross &&
                g_run.status.state == APP_RUN_FOLLOW &&
                g_run.status.elapsed_ms >= g_run.profile->minimum_finish_ms &&
                black_count >= LINE_CROSS_BLACK_COUNT) {
                g_run.status.state = APP_RUN_FINISH_CONFIRM;
                g_run.finish_cycles = 1U;
            } else if (g_run.status.state == APP_RUN_FINISH_CONFIRM) {
                if (black_count >= LINE_CROSS_BLACK_COUNT) {
                    if (++g_run.finish_cycles >= LINE_FINISH_CONFIRM_CYCLES) {
                        begin_align_or_finish(now_ms);
                    }
                } else {
                    g_run.finish_cycles = 0U;
                    g_run.status.state = APP_RUN_FOLLOW;
                }
            }
        }
    }

    if (state_uses_motor(g_run.status.state) && g_run.profile != NULL) {
        if (g_run.status.state == APP_RUN_SOFT_STOP) {
            g_run.command_left = target_left;
            g_run.command_right = target_right;
            g_run.stall_cycles = 0U;
        } else {
            int16_t slew_step = g_run.profile->slew_step;
            if (g_run.status.task == APP_TASK_T4 &&
                g_run.status.elapsed_ms < T4_LAUNCH_RAMP_MS) {
                slew_step = T4_LAUNCH_PWM_SLEW_PER_STEP;
            } else if ((g_run.status.task == APP_TASK_T5 ||
                        g_run.status.task == APP_TASK_T6) &&
                       g_run.status.elapsed_ms < T5_LAUNCH_RAMP_MS) {
                slew_step = T5_LAUNCH_PWM_SLEW_PER_STEP;
            } else if (g_run.status.task == APP_TASK_T4 &&
                       g_run.status.travel_counts >= T4_APPROACH_COUNTS) {
                slew_step = T4_APPROACH_PWM_SLEW_PER_STEP;
            } else if ((g_run.status.task == APP_TASK_T5 ||
                        g_run.status.task == APP_TASK_T6) &&
                       g_run.status.travel_counts >= T5_APPROACH_COUNTS) {
                slew_step = T5_APPROACH_PWM_SLEW_PER_STEP;
            }
            g_run.command_left = slew(g_run.command_left,
                target_left, slew_step);
            g_run.command_right = slew(g_run.command_right,
                target_right, slew_step);

            if ((LINE_STALL_ENABLE != 0U) &&
                g_run.status.elapsed_ms >= LINE_STALL_ARM_MS &&
                (((absolute_i16(g_run.command_left) +
                   absolute_i16(g_run.command_right)) / 2) >=
                 LINE_STALL_PWM_THRESHOLD)) {
                if (sensors->encoder_delta_left == 0 &&
                    sensors->encoder_delta_right == 0) {
                    g_run.stall_cycles++;
                } else {
                    g_run.stall_cycles = 0U;
                }
                if ((uint32_t)g_run.stall_cycles * APP_CONTROL_PERIOD_MS >=
                    LINE_STALL_CONFIRM_MS) {
                    stop_with_state(APP_RUN_FAULT, APP_RUN_FAULT_STALL);
                }
            } else {
                g_run.stall_cycles = 0U;
            }
        }
    }

    g_run.status.left_pwm = g_run.command_left;
    g_run.status.right_pwm = g_run.command_right;
    output->motor_enable = state_uses_motor(g_run.status.state);
    output->left_pwm = output->motor_enable ? g_run.command_left : 0;
    output->right_pwm = output->motor_enable ? g_run.command_right : 0;
}

void APP_Run_GetStatus(APP_RunStatus *status)
{
    if (status == NULL) return;
    taskENTER_CRITICAL();
    *status = g_run.status;
    taskEXIT_CRITICAL();
}

bool APP_Run_IsRunning(void)
{
    return state_is_running(g_run.status.state);
}

bool APP_Task_UsesChassis(APP_TaskId task)
{
    return profile_for_task(task) != NULL;
}

const char *APP_Task_Name(APP_TaskId task)
{
    switch (task) {
    case APP_TASK_T1: return "T1";
    case APP_TASK_T2: return "T2";
    case APP_TASK_T3: return "T3";
    case APP_TASK_T4: return "T4";
    case APP_TASK_T5: return "T5";
    case APP_TASK_T6: return "T6";
    default: return "NONE";
    }
}

const char *APP_Run_StateName(APP_RunState state)
{
    switch (state) {
    case APP_RUN_IDLE: return "READY";
    case APP_RUN_REMOTE: return "REMOTE";
    case APP_RUN_LEAVE_START: return "LEAVE";
    case APP_RUN_FOLLOW: return "FOLLOW";
    case APP_RUN_FINISH_CONFIRM: return "FINISH";
    case APP_RUN_ALIGN: return "ALIGN";
    case APP_RUN_SOFT_STOP: return "SOFT_STOP";
    case APP_RUN_DONE: return "DONE";
    case APP_RUN_ABORTED: return "ABORT";
    case APP_RUN_FAULT: return "FAULT";
    default: return "UNKNOWN";
    }
}

const char *APP_Run_FaultName(APP_RunFault fault)
{
    switch (fault) {
    case APP_RUN_FAULT_NONE: return "NONE";
    case APP_RUN_FAULT_LINE_SENSOR: return "LINE I2C";
    case APP_RUN_FAULT_LINE_LOST: return "LINE LOST";
    case APP_RUN_FAULT_RUN_TIMEOUT: return "TIMEOUT";
    case APP_RUN_FAULT_STALL: return "STALL";
    default: return "UNKNOWN";
    }
}

const char *APP_Checkpoint_Name(APP_Checkpoint checkpoint)
{
    switch (checkpoint) {
    case APP_CHECKPOINT_A: return "A";
    case APP_CHECKPOINT_B: return "B";
    default: return "NONE";
    }
}
