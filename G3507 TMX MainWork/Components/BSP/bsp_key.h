/**
 * @file bsp_key.h
 * @brief K1 消抖及短按、双击、长按事件
 */

#ifndef BSP_KEY_H
#define BSP_KEY_H

#include <stdint.h>

typedef enum {
    BSP_KEY_EVENT_NONE = 0,
    BSP_KEY_EVENT_SHORT,
    BSP_KEY_EVENT_DOUBLE,
    BSP_KEY_EVENT_LONG
} BSP_KeyEvent;

void BSP_Key_Init(uint32_t now_ms);
BSP_KeyEvent BSP_Key_Scan(uint32_t now_ms);

#endif
