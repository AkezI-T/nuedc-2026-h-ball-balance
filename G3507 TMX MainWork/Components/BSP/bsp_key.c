/**
 * @file bsp_key.c
 * @brief 复用 TianMengXinTest_v2 交互时序的单键状态机
 */

#include "bsp_key.h"
#include "ti_msp_dl_config.h"

#include <stdbool.h>

#define KEY_DEBOUNCE_MS     20U
#define KEY_LONG_PRESS_MS  700U
#define KEY_DOUBLE_GAP_MS  500U

typedef enum {
    KEY_STATE_RELEASED = 0,
    KEY_STATE_DEBOUNCE,
    KEY_STATE_PRESSED,
    KEY_STATE_LONG,
    KEY_STATE_WAIT_DOUBLE,
    KEY_STATE_SECOND_PRESS
} KeyState;

static KeyState g_state;
static uint32_t g_press_ms;
static uint32_t g_release_ms;
static uint32_t g_debounce_ms;

static bool key_is_pressed(void)
{
    return DL_GPIO_readPins(KEY_PORT, KEY_K1_PIN) == 0U;
}

void BSP_Key_Init(uint32_t now_ms)
{
    g_state = KEY_STATE_RELEASED;
    g_press_ms = now_ms;
    g_release_ms = now_ms;
    g_debounce_ms = now_ms;
}

BSP_KeyEvent BSP_Key_Scan(uint32_t now_ms)
{
    bool pressed = key_is_pressed();

    switch (g_state) {
    case KEY_STATE_RELEASED:
        if (pressed) {
            g_state = KEY_STATE_DEBOUNCE;
            g_debounce_ms = now_ms;
        }
        break;

    case KEY_STATE_DEBOUNCE:
        if ((uint32_t)(now_ms - g_debounce_ms) >= KEY_DEBOUNCE_MS) {
            if (pressed) {
                g_state = KEY_STATE_PRESSED;
                g_press_ms = now_ms;
            } else {
                g_state = KEY_STATE_RELEASED;
            }
        }
        break;

    case KEY_STATE_PRESSED:
        if (!pressed) {
            g_state = KEY_STATE_WAIT_DOUBLE;
            g_release_ms = now_ms;
        } else if ((uint32_t)(now_ms - g_press_ms) >= KEY_LONG_PRESS_MS) {
            g_state = KEY_STATE_LONG;
            return BSP_KEY_EVENT_LONG;
        }
        break;

    case KEY_STATE_LONG:
        if (!pressed) g_state = KEY_STATE_RELEASED;
        break;

    case KEY_STATE_WAIT_DOUBLE:
        if (pressed) {
            g_state = KEY_STATE_SECOND_PRESS;
            g_press_ms = now_ms;
            g_debounce_ms = now_ms;
        } else if ((uint32_t)(now_ms - g_release_ms) >= KEY_DOUBLE_GAP_MS) {
            g_state = KEY_STATE_RELEASED;
            return BSP_KEY_EVENT_SHORT;
        }
        break;

    case KEY_STATE_SECOND_PRESS:
        if ((uint32_t)(now_ms - g_debounce_ms) >= KEY_DEBOUNCE_MS) {
            if (!pressed) {
                g_state = KEY_STATE_RELEASED;
                return BSP_KEY_EVENT_DOUBLE;
            }
            if ((uint32_t)(now_ms - g_press_ms) >= KEY_LONG_PRESS_MS) {
                g_state = KEY_STATE_LONG;
                return BSP_KEY_EVENT_LONG;
            }
        }
        break;

    default:
        g_state = KEY_STATE_RELEASED;
        break;
    }
    return BSP_KEY_EVENT_NONE;
}
