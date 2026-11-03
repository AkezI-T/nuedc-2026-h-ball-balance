/**
 * @file bsp_motor.h
 * @brief TB6612FNG 双路电机驱动，正 PWM 表示整车前进
 */

#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <stdint.h>

void BSP_Motor_Init(void);
void BSP_Motor_SetRaw(int16_t left_pwm, int16_t right_pwm);
void BSP_Motor_Stop(void);

#endif
