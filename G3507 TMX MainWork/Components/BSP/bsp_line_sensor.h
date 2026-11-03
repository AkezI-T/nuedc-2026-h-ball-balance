/**
 * @file bsp_line_sensor.h
 * @brief 亚博八路 I2C 巡线传感器协议
 */

#ifndef BSP_LINE_SENSOR_H
#define BSP_LINE_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

void BSP_LineSensor_Init(void);
bool BSP_LineSensor_Request(uint32_t now_ms);
bool BSP_LineSensor_Take(uint8_t *raw, bool *ok);

#endif
