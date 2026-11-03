/**
 * @file app_menu.c
 * @brief T1-T6主菜单、统一准备页、运行结果和传感器测试页面
 */

#include "app_menu.h"
#include "app_config.h"
#include "lcd.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

#define MENU_FONT_HEIGHT       12U
#define MENU_LINE_PITCH        14U
#define MENU_TEXT_COLUMNS      26U
#define MENU_CHAR_WIDTH        (MENU_FONT_HEIGHT / 2U)
#define MENU_TEXT_WIDTH        (MENU_TEXT_COLUMNS * MENU_CHAR_WIDTH)
#define MENU_ROW(index)        ((uint16_t)((index) * MENU_LINE_PITCH))

typedef struct {
    APP_MenuPage page;
    APP_MenuItem cursor;
    APP_TaskId selected_task;
    uint32_t short_count;
    uint32_t double_count;
    uint32_t long_count;
} APP_MenuContext;

static APP_MenuContext g_menu;

static APP_TaskId item_task(APP_MenuItem item)
{
    if (item <= APP_MENU_ITEM_T6) {
        return (APP_TaskId)((uint32_t)item + (uint32_t)APP_TASK_T1);
    }
    return APP_TASK_NONE;
}

static const char *item_name(APP_MenuItem item)
{
    switch (item) {
    case APP_MENU_ITEM_T1: return "T1 RECORD TEST";
    case APP_MENU_ITEM_T2: return "T2 FAST LINE LAP";
    case APP_MENU_ITEM_T3: return "T3 BALL +5 TO -5";
    case APP_MENU_ITEM_T4: return "T4 SLOW A TO B";
    case APP_MENU_ITEM_T5: return "T5 SLOW LINE LAP";
    case APP_MENU_ITEM_T6: return "T6 HOLD START POS";
    case APP_MENU_ITEM_TEST: return "SENSOR TEST";
    default: return "UNKNOWN";
    }
}

static const char *task_title(APP_TaskId task)
{
    switch (task) {
    case APP_TASK_T1: return "RECORD SWITCH TEST";
    case APP_TASK_T2: return "FAST LINE LAP";
    case APP_TASK_T3: return "BALL +5CM TO -5CM";
    case APP_TASK_T4: return "SLOW LINE A TO B";
    case APP_TASK_T5: return "SLOW LAP HOLD O";
    case APP_TASK_T6: return "SLOW LAP HOLD START";
    default: return "UNKNOWN TASK";
    }
}

static void go_page(APP_MenuPage page)
{
    g_menu.page = page;
}

void APP_Menu_Init(void)
{
    g_menu = (APP_MenuContext){0};
    g_menu.page = APP_MENU_PAGE_MAIN;
    g_menu.cursor = APP_MENU_ITEM_T1;
    g_menu.selected_task = APP_TASK_T1;
    LCD_Fill(0U, 0U, LCD_W, LCD_H, WHITE);
}

void APP_Menu_HandleKey(BSP_KeyEvent event, uint32_t now_ms)
{
    (void)now_ms;
    if (event == BSP_KEY_EVENT_NONE) return;
    if (event == BSP_KEY_EVENT_SHORT) g_menu.short_count++;
    if (event == BSP_KEY_EVENT_DOUBLE) g_menu.double_count++;
    if (event == BSP_KEY_EVENT_LONG) g_menu.long_count++;

    switch (g_menu.page) {
    case APP_MENU_PAGE_MAIN:
        if (event == BSP_KEY_EVENT_SHORT) {
            g_menu.cursor = (APP_MenuItem)(
                ((uint32_t)g_menu.cursor + 1U) % APP_MENU_ITEM_COUNT);
        } else if (event == BSP_KEY_EVENT_DOUBLE) {
            if (g_menu.cursor == APP_MENU_ITEM_TEST) {
                go_page(APP_MENU_PAGE_TEST);
            } else {
                g_menu.selected_task = item_task(g_menu.cursor);
                go_page(APP_MENU_PAGE_TASK_PREP);
            }
        }
        break;

    case APP_MENU_PAGE_TASK_PREP:
        if (event == BSP_KEY_EVENT_SHORT) {
            APP_Run_RequestStart(g_menu.selected_task);
            go_page(APP_MENU_PAGE_TASK_RUN);
        } else if (event == BSP_KEY_EVENT_LONG) {
            go_page(APP_MENU_PAGE_MAIN);
        }
        break;

    case APP_MENU_PAGE_TASK_RUN:
        if (g_menu.selected_task == APP_TASK_T1) {
            if (event == BSP_KEY_EVENT_SHORT ||
                event == BSP_KEY_EVENT_DOUBLE) {
                APP_Run_RequestFinish();
            } else if (event == BSP_KEY_EVENT_LONG) {
                APP_Run_RequestAbort();
                go_page(APP_MENU_PAGE_MAIN);
            }
        } else if (g_menu.selected_task == APP_TASK_T3) {
            if (event == BSP_KEY_EVENT_DOUBLE ||
                event == BSP_KEY_EVENT_LONG) {
                APP_Run_RequestAbort();
                go_page(APP_MENU_PAGE_MAIN);
            }
        } else if (event == BSP_KEY_EVENT_DOUBLE) {
            APP_Run_RequestAbort();
            go_page(APP_MENU_PAGE_STOPPED);
        } else if (event == BSP_KEY_EVENT_LONG) {
            APP_Run_RequestAbort();
            go_page(APP_MENU_PAGE_MAIN);
        }
        break;

    case APP_MENU_PAGE_STOPPED:
        if (event == BSP_KEY_EVENT_DOUBLE ||
            event == BSP_KEY_EVENT_LONG) {
            go_page(APP_MENU_PAGE_MAIN);
        }
        break;

    case APP_MENU_PAGE_FAULT:
    case APP_MENU_PAGE_TEST:
        if (event == BSP_KEY_EVENT_LONG) {
            go_page(APP_MENU_PAGE_MAIN);
        }
        break;

    default:
        go_page(APP_MENU_PAGE_MAIN);
        break;
    }
}

void APP_Menu_Update(const APP_RunStatus *status, uint32_t now_ms)
{
    (void)now_ms;
    if (status == NULL || g_menu.page != APP_MENU_PAGE_TASK_RUN ||
        status->task != g_menu.selected_task) return;

    if (status->state == APP_RUN_DONE ||
        status->state == APP_RUN_ABORTED) {
        go_page(APP_MENU_PAGE_STOPPED);
    } else if (status->state == APP_RUN_FAULT) {
        go_page(APP_MENU_PAGE_FAULT);
    }
}

void APP_Menu_GetProtocolState(APP_MenuPage *page, APP_TaskId *task)
{
    if (page == NULL || task == NULL) return;
    taskENTER_CRITICAL();
    *page = g_menu.page;
    *task = g_menu.selected_task;
    taskEXIT_CRITICAL();
}

static void show_line(uint16_t y, const char *text,
    uint16_t foreground, uint16_t background)
{
    char padded[MENU_TEXT_COLUMNS + 1U];

    (void)snprintf(padded, sizeof(padded), "%-*.*s",
        (int)MENU_TEXT_COLUMNS, (int)MENU_TEXT_COLUMNS, text);
    LCD_ShowString(0U, y, (const u8 *)padded,
        foreground, background, MENU_FONT_HEIGHT, 0U);
    if (MENU_TEXT_WIDTH < LCD_W) {
        LCD_Fill(MENU_TEXT_WIDTH, y, LCD_W,
            (uint16_t)(y + MENU_FONT_HEIGHT), background);
    }
}

static void render_main(void)
{
    char text[32];
    uint32_t index;

    show_line(MENU_ROW(0U), "TMX TASK MENU", BLACK, WHITE);
    for (index = 0U; index < APP_MENU_ITEM_COUNT; index++) {
        bool selected = index == (uint32_t)g_menu.cursor;
        (void)snprintf(text, sizeof(text), "%c %s",
            selected ? '>' : ' ', item_name((APP_MenuItem)index));
        show_line(MENU_ROW(index + 1U), text,
            selected ? WHITE : BLACK, selected ? BLUE : WHITE);
    }
    show_line(MENU_ROW(8U), "S:NEXT D:PREP", DARKBLUE, WHITE);
}

static void render_task_prep(const APP_SensorSnapshot *sensors,
    uint32_t now_ms)
{
    char text[32];
    uint32_t line_age = sensors->line_valid ?
        now_ms - sensors->line_timestamp_ms : 0xFFFFFFFFUL;
    bool line_ready = sensors->line_valid && line_age <= LINE_STALE_MS;

    (void)snprintf(text, sizeof(text), "%s PREP",
        APP_Task_Name(g_menu.selected_task));
    show_line(MENU_ROW(0U), text, WHITE, DARKBLUE);
    show_line(MENU_ROW(1U), task_title(g_menu.selected_task), BLACK, WHITE);
    show_line(MENU_ROW(2U),
        APP_Task_UsesChassis(g_menu.selected_task) ?
        (g_menu.selected_task == APP_TASK_T2 ?
         "CHASSIS FAST" : "CHASSIS SLOW") : "CHASSIS STOP",
        BLACK, WHITE);
    if (APP_Task_UsesChassis(g_menu.selected_task)) {
        (void)snprintf(text, sizeof(text), "LINE %02X AGE %lums",
            (unsigned)sensors->line_raw, (unsigned long)line_age);
        show_line(MENU_ROW(3U), text, line_ready ? BLACK : RED, WHITE);
        (void)snprintf(text, sizeof(text), "ENC %ld / %ld",
            (long)sensors->encoder_left, (long)sensors->encoder_right);
        show_line(MENU_ROW(4U), text, BLACK, WHITE);
    } else {
        show_line(MENU_ROW(3U), "WAIT ESP32/K230", GRAY, WHITE);
    }
    show_line(MENU_ROW(8U), "S:START L:BACK", DARKBLUE, WHITE);
}

static void render_task_run(const APP_RunStatus *status,
    const APP_SensorSnapshot *sensors)
{
    char text[32];
    uint32_t shown_ms = status->state == APP_RUN_DONE ?
        status->finish_time_ms : status->elapsed_ms;

    (void)snprintf(text, sizeof(text), "%s RUN",
        APP_Task_Name(g_menu.selected_task));
    show_line(MENU_ROW(0U), text, WHITE, BLUE);
    (void)snprintf(text, sizeof(text), "STATE %-8s",
        APP_Run_StateName(status->state));
    show_line(MENU_ROW(1U), text,
        status->state == APP_RUN_FAULT ? RED : BLACK, WHITE);
    (void)snprintf(text, sizeof(text), "TIME %02lu.%03lus",
        (unsigned long)(shown_ms / 1000U),
        (unsigned long)(shown_ms % 1000U));
    show_line(MENU_ROW(2U), text, BLACK, WHITE);

    if (APP_Task_UsesChassis(g_menu.selected_task)) {
        (void)snprintf(text, sizeof(text), "LINE %02X N%u E%+d",
            (unsigned)status->line_raw, (unsigned)status->black_count,
            (int)status->line_error);
        show_line(MENU_ROW(3U), text, BLACK, WHITE);
        (void)snprintf(text, sizeof(text), "PWM L%+d R%+d",
            (int)status->left_pwm, (int)status->right_pwm);
        show_line(MENU_ROW(4U), text, BLACK, WHITE);
        (void)snprintf(text, sizeof(text), "ENC %lu L%ld R%ld",
            (unsigned long)status->travel_counts,
            (long)sensors->encoder_left, (long)sensors->encoder_right);
        show_line(MENU_ROW(5U), text, BLACK, WHITE);
        show_line(MENU_ROW(8U), "D:STOP L:BACK", DARKBLUE, WHITE);
    } else if (g_menu.selected_task == APP_TASK_T1) {
        show_line(MENU_ROW(3U), "VIDEO RECORDING TEST", BLACK, WHITE);
        show_line(MENU_ROW(8U), "S:END L:ABORT", DARKBLUE, WHITE);
    } else {
        show_line(MENU_ROW(3U), "ESP32 BALL CONTROL", BLACK, WHITE);
        show_line(MENU_ROW(4U), "TMX CHASSIS STOP", BLACK, WHITE);
        show_line(MENU_ROW(8U), "D:EXIT L:EXIT", DARKBLUE, WHITE);
    }
}

static void render_stopped(const APP_RunStatus *status)
{
    char text[32];
    uint32_t shown_ms = status->finish_time_ms != 0U ?
        status->finish_time_ms : status->elapsed_ms;

    (void)snprintf(text, sizeof(text), "%s STOPPED",
        APP_Task_Name(status->task));
    show_line(MENU_ROW(0U), text, WHITE, DARKBLUE);
    (void)snprintf(text, sizeof(text), "RESULT %s",
        APP_Run_StateName(status->state));
    show_line(MENU_ROW(1U), text, BLACK, WHITE);
    (void)snprintf(text, sizeof(text), "TIME %02lu.%03lus",
        (unsigned long)(shown_ms / 1000U),
        (unsigned long)(shown_ms % 1000U));
    show_line(MENU_ROW(2U), text,
        status->state == APP_RUN_DONE ? GREEN : BLACK, WHITE);
    show_line(MENU_ROW(8U), "D:MENU L:MENU", DARKBLUE, WHITE);
}

static void render_fault(const APP_RunStatus *status)
{
    char text[32];
    (void)snprintf(text, sizeof(text), "%s FAULT",
        APP_Task_Name(status->task));
    show_line(MENU_ROW(0U), text, WHITE, RED);
    (void)snprintf(text, sizeof(text), "ERR %s",
        APP_Run_FaultName(status->fault));
    show_line(MENU_ROW(1U), text, RED, WHITE);
    (void)snprintf(text, sizeof(text), "LINE %02X N%u E%+d",
        (unsigned)status->line_raw, (unsigned)status->black_count,
        (int)status->line_error);
    show_line(MENU_ROW(2U), text, BLACK, WHITE);
    show_line(MENU_ROW(8U), "L:RESET TO MENU", DARKBLUE, WHITE);
}

static void render_test(const APP_SensorSnapshot *sensors, uint32_t now_ms)
{
    char text[32];
    uint32_t line_age = sensors->line_valid ?
        now_ms - sensors->line_timestamp_ms : 0xFFFFFFFFUL;

    show_line(MENU_ROW(0U), "SENSOR TEST", WHITE, DARKBLUE);
    (void)snprintf(text, sizeof(text), "LINE %02X AGE %lums",
        (unsigned)sensors->line_raw, (unsigned long)line_age);
    show_line(MENU_ROW(1U), text,
        sensors->line_valid ? BLACK : RED, WHITE);
    (void)snprintf(text, sizeof(text), "YAW %+d.%01d",
        (int)(sensors->yaw_x10 / 10),
        (int)((sensors->yaw_x10 < 0 ? -sensors->yaw_x10 :
            sensors->yaw_x10) % 10));
    show_line(MENU_ROW(2U), text,
        sensors->gyro_valid ? BLACK : RED, WHITE);
    (void)snprintf(text, sizeof(text), "ENC %ld / %ld",
        (long)sensors->encoder_left, (long)sensors->encoder_right);
    show_line(MENU_ROW(3U), text, BLACK, WHITE);
    (void)snprintf(text, sizeof(text), "I2C ERR %lu / %lu",
        (unsigned long)sensors->line_error_count,
        (unsigned long)sensors->gyro_error_count);
    show_line(MENU_ROW(4U), text, BLACK, WHITE);
    (void)snprintf(text, sizeof(text), "KEY S%lu D%lu L%lu",
        (unsigned long)g_menu.short_count,
        (unsigned long)g_menu.double_count,
        (unsigned long)g_menu.long_count);
    show_line(MENU_ROW(5U), text, BLACK, WHITE);
    show_line(MENU_ROW(8U), "L:BACK", DARKBLUE, WHITE);
}

void APP_Menu_Render(const APP_RunStatus *status,
    const APP_SensorSnapshot *sensors, uint32_t now_ms)
{
    static APP_MenuPage last_page = (APP_MenuPage)0xFF;

    if (status == NULL || sensors == NULL) return;
    if (g_menu.page != last_page) {
        LCD_Fill(0U, 0U, LCD_W, LCD_H, WHITE);
        last_page = g_menu.page;
    }

    switch (g_menu.page) {
    case APP_MENU_PAGE_MAIN:
        render_main();
        break;
    case APP_MENU_PAGE_TASK_PREP:
        render_task_prep(sensors, now_ms);
        break;
    case APP_MENU_PAGE_TASK_RUN:
        render_task_run(status, sensors);
        break;
    case APP_MENU_PAGE_STOPPED:
        render_stopped(status);
        break;
    case APP_MENU_PAGE_FAULT:
        render_fault(status);
        break;
    case APP_MENU_PAGE_TEST:
        render_test(sensors, now_ms);
        break;
    default:
        go_page(APP_MENU_PAGE_MAIN);
        break;
    }
}
