/**
 * @file app_tasks.h
 * @brief T1-T6 FreeRTOS任务入口
 */

#ifndef APP_TASKS_H
#define APP_TASKS_H

#include "app_t1.h"

void APP_Tasks_Create(void);
void APP_Sensors_Get(APP_SensorSnapshot *snapshot);

#endif
