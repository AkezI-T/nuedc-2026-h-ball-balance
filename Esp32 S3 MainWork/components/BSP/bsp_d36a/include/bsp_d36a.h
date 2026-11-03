#ifndef BSP_D36A_H
#define BSP_D36A_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum {
    BSP_D36A_DIRECTION_TOWARD_REAR = 0,
    BSP_D36A_DIRECTION_TOWARD_FRONT,
} bsp_d36a_direction_t;

/**
 * @brief 初始化 D36A 单轴接口。
 *
 * 固定引脚：GPIO18=STEP、GPIO8=DIR、GPIO3=EN，EN 高有效。
 * 初始化后 STEP、DIR、EN 均为低，电机处于释放状态。
 */
esp_err_t bsp_d36a_init(void);

/** @brief 设置 D36A 使能，true 表示励磁锁定。 */
esp_err_t bsp_d36a_set_enabled(bool enabled);

/**
 * @brief 设置已标定的物理转动方向。
 *
 * 当前接线固定为：向车尾=DIR低，向车头=DIR高。
 */
esp_err_t bsp_d36a_set_direction(bsp_d36a_direction_t direction);

/** @brief 使用LEDC开始输出指定频率、50%占空比的STEP脉冲。 */
esp_err_t bsp_d36a_start_pulse(uint32_t frequency_hz);

/** @brief 停止STEP脉冲并将输出保持为低电平。 */
esp_err_t bsp_d36a_stop_pulse(void);

/** @brief STEP 置低并保持使能。 */
esp_err_t bsp_d36a_hold(void);

/** @brief STEP 置低并取消使能，使电机可以手动转动。 */
esp_err_t bsp_d36a_release(void);

/** @brief 返回软件记录的使能状态。 */
bool bsp_d36a_is_enabled(void);

#endif
