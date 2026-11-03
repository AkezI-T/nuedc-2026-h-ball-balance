/**
 * @file app_menu.h
 * @brief T1-T6统一准备页与TianMengXinTest_v2单键菜单
 */

#ifndef APP_MENU_H
#define APP_MENU_H

#include "app_t1.h"
#include "bsp_key.h"

#include <stdint.h>

typedef enum {
    APP_MENU_PAGE_MAIN = 0,
    APP_MENU_PAGE_TASK_PREP,
    APP_MENU_PAGE_TASK_RUN,
    APP_MENU_PAGE_STOPPED,
    APP_MENU_PAGE_FAULT,
    APP_MENU_PAGE_TEST
} APP_MenuPage;

typedef enum {
    APP_MENU_ITEM_T1 = 0,
    APP_MENU_ITEM_T2,
    APP_MENU_ITEM_T3,
    APP_MENU_ITEM_T4,
    APP_MENU_ITEM_T5,
    APP_MENU_ITEM_T6,
    APP_MENU_ITEM_TEST,
    APP_MENU_ITEM_COUNT
} APP_MenuItem;

void APP_Menu_Init(void);
void APP_Menu_HandleKey(BSP_KeyEvent event, uint32_t now_ms);
void APP_Menu_Update(const APP_RunStatus *status, uint32_t now_ms);
void APP_Menu_Render(const APP_RunStatus *status,
    const APP_SensorSnapshot *sensors, uint32_t now_ms);
void APP_Menu_GetProtocolState(APP_MenuPage *page, APP_TaskId *task);

#endif
