#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include <stdint.h>

#include "esp_err.h"

typedef struct {
    /* 已标定：计数增加=向车头，计数减小=向车尾。 */
    int32_t ab_count;
    int a_level;
    int b_level;
    uint32_t pwm_period_ticks;
    uint32_t pwm_high_ticks;
    uint32_t pwm_capture_count;
    uint32_t pwm_resolution_hz;
    uint32_t z_count;
} bsp_encoder_snapshot_t;

/**
 * @brief 初始化MS42CG编码器输入。
 *
 * 固定引脚：GPIO4=A、GPIO5=B、GPIO6=PWM、GPIO7=Z。
 */
esp_err_t bsp_encoder_init(void);

/** @brief 获取A/B累计计数、PWM捕获值及Z触发次数。 */
esp_err_t bsp_encoder_get_snapshot(bsp_encoder_snapshot_t *snapshot);

#endif
