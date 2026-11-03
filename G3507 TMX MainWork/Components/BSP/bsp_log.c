/**
 * @file bsp_log.c
 * @brief UART0 轮询发送日志，仅供低优先级 LogTask 使用
 */

#include "bsp_log.h"
#include "ti_msp_dl_config.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>

#define BSP_LOG_BUFFER_SIZE 320U

/* 仅LogTask调用，使用静态缓冲避免长@CAR格式化压垮任务栈。 */
static char g_log_buffer[BSP_LOG_BUFFER_SIZE];

void BSP_Log_Printf(const char *format, ...)
{
    va_list args;
    int length;
    int index;

    va_start(args, format);
    length = vsnprintf(g_log_buffer, sizeof(g_log_buffer), format, args);
    va_end(args);

    if (length <= 0) return;
    if (length >= (int)sizeof(g_log_buffer)) {
        length = (int)sizeof(g_log_buffer) - 1;
    }
    for (index = 0; index < length; index++) {
        DL_UART_Main_transmitDataBlocking(UART_DEBUG_INST,
            (uint8_t)g_log_buffer[index]);
    }
}
