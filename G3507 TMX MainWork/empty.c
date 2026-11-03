/**
 * @file empty.c
 * @brief H题T1-T6任务系统FreeRTOS入口
 */

#include "ti_msp_dl_config.h"

#include "app_t1.h"
#include "app_tasks.h"
#include "bsp_encoder.h"
#include "bsp_i2c_async.h"
#include "bsp_motor.h"
#include "lcd.h"

#include "FreeRTOS.h"
#include "task.h"

volatile uint32_t g_app_fatal_reason;
const char * volatile g_app_stack_overflow_task;

static void fatal_stop(void)
{
    taskDISABLE_INTERRUPTS();
    BSP_Motor_Stop();
    DL_GPIO_setPins(LED_PORT, LED_PIN_PIN);
    for (;;) {
    }
}

void APP_AssertFailed(const char *file, unsigned long line)
{
    (void)file;
    (void)line;
    g_app_fatal_reason = 1U;
    fatal_stop();
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
    (void)task;
    g_app_fatal_reason = 2U;
    g_app_stack_overflow_task = task_name;
    fatal_stop();
}

void vApplicationGetIdleTaskMemory(StaticTask_t **tcb,
    StackType_t **stack, uint32_t *stack_size)
{
    static StaticTask_t idle_tcb;
    static StackType_t idle_stack[configMINIMAL_STACK_SIZE];

    *tcb = &idle_tcb;
    *stack = idle_stack;
    *stack_size = configMINIMAL_STACK_SIZE;
}

void GROUP1_IRQHandler(void)
{
    BSP_Encoder_IRQHandler();
}

int main(void)
{
    SYSCFG_DL_init();

    /* TIMG8 保留但不启动，FreeRTOS 使用 SysTick 产生 1 ms 时基。 */
    BSP_Motor_Init();
    BSP_Encoder_Init();
    BSP_I2C_AsyncInit();
    LCD_Init();
    APP_Run_Init();
    APP_Tasks_Create();

    vTaskStartScheduler();
    g_app_fatal_reason = 3U;
    fatal_stop();
    return 0;
}
