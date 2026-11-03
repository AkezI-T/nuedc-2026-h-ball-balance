/**
 * @file bsp_encoder.h
 * @brief 双路正交编码器单边沿计数
 */

#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include <stdint.h>

typedef struct {
    int32_t count_a;
    int32_t count_b;
    int32_t delta_a;
    int32_t delta_b;
} BSP_EncoderSnapshot;

void BSP_Encoder_Init(void);
void BSP_Encoder_Read(BSP_EncoderSnapshot *snapshot);
void BSP_Encoder_IRQHandler(void);

#endif
