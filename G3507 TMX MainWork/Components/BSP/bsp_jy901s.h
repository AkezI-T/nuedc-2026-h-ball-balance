/**
 * @file bsp_jy901s.h
 * @brief JY901S I2C 姿态读取
 */

#ifndef BSP_JY901S_H
#define BSP_JY901S_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int16_t roll_x10;
    int16_t pitch_x10;
    int16_t yaw_x10;
} BSP_JY901S_Angles;

void BSP_JY901S_Init(void);
bool BSP_JY901S_Request(uint32_t now_ms);
bool BSP_JY901S_Take(BSP_JY901S_Angles *angles, bool *ok);

#endif
