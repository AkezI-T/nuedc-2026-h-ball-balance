/**
 * @file bsp_motor.c
 * @brief 复用 VisionTrack 方案的 TB6612FNG 双路电机驱动
 */

#include "bsp_motor.h"
#include "ti_msp_dl_config.h"

#define MOTOR_PWM_MAX 1000

static uint16_t pwm_to_compare(int16_t pwm)
{
    uint16_t magnitude = (uint16_t)(pwm < 0 ? -pwm : pwm);

    if (magnitude > MOTOR_PWM_MAX) magnitude = MOTOR_PWM_MAX;
    if (magnitude == 0U) return 1U;
    if (magnitude >= MOTOR_PWM_MAX) return 1U;
    return (uint16_t)(MOTOR_PWM_MAX - magnitude);
}

static void set_motor_a(int16_t pwm)
{
    if (pwm == 0) {
        DL_GPIO_clearPins(TB6612_PORT,
            TB6612_AIN1_PIN | TB6612_AIN2_PIN);
    } else if (pwm > 0) {
        DL_GPIO_setPins(TB6612_PORT, TB6612_AIN1_PIN);
        DL_GPIO_clearPins(TB6612_PORT, TB6612_AIN2_PIN);
    } else {
        DL_GPIO_clearPins(TB6612_PORT, TB6612_AIN1_PIN);
        DL_GPIO_setPins(TB6612_PORT, TB6612_AIN2_PIN);
    }
    DL_TimerA_setCaptureCompareValue(PWM_TB6612_INST,
        pwm_to_compare(pwm), GPIO_PWM_TB6612_C1_IDX);
}

static void set_motor_b(int16_t pwm)
{
    if (pwm == 0) {
        DL_GPIO_clearPins(TB6612_PORT,
            TB6612_BIN1_PIN | TB6612_BIN2_PIN);
    } else if (pwm > 0) {
        DL_GPIO_setPins(TB6612_PORT, TB6612_BIN1_PIN);
        DL_GPIO_clearPins(TB6612_PORT, TB6612_BIN2_PIN);
    } else {
        DL_GPIO_clearPins(TB6612_PORT, TB6612_BIN1_PIN);
        DL_GPIO_setPins(TB6612_PORT, TB6612_BIN2_PIN);
    }
    DL_TimerA_setCaptureCompareValue(PWM_TB6612_INST,
        pwm_to_compare(pwm), GPIO_PWM_TB6612_C0_IDX);
}

void BSP_Motor_Init(void)
{
    DL_TimerA_startCounter(PWM_TB6612_INST);
    BSP_Motor_Stop();
}

void BSP_Motor_SetRaw(int16_t left_pwm, int16_t right_pwm)
{
    if ((left_pwm == 0) && (right_pwm == 0)) {
        BSP_Motor_Stop();
        return;
    }

    /* 实车 A/B 对应右/左轮，且两路安装方向均与 VisionTrack 样车相反。 */
    set_motor_a((int16_t)-right_pwm);
    set_motor_b((int16_t)-left_pwm);
    DL_GPIO_setPins(STBY_PORT, STBY_STBY_PIN_PIN);
}

void BSP_Motor_Stop(void)
{
    DL_GPIO_clearPins(STBY_PORT, STBY_STBY_PIN_PIN);
    set_motor_a(0);
    set_motor_b(0);
}
