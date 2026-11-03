/**
 * @file FreeRTOSConfig.h
 * @brief MSPM0G3507 T1 巡线系统的轻量 FreeRTOS 配置
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stddef.h>
#include <stdint.h>

#define configCPU_CLOCK_HZ                       32000000UL
#define configTICK_RATE_HZ                       1000U
#define configTICK_TYPE_WIDTH_IN_BITS            TICK_TYPE_WIDTH_32_BITS
#define configMAX_PRIORITIES                     5U
#define configMINIMAL_STACK_SIZE                 128U
#define configMAX_TASK_NAME_LEN                  12U

#define configUSE_PREEMPTION                     1
#define configUSE_TIME_SLICING                   0
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  0
#define configUSE_TICKLESS_IDLE                  0
#define configIDLE_SHOULD_YIELD                  0
#define configENABLE_MPU                         0

/* 全部任务采用静态分配，避免运行时堆碎片和分配失败。 */
#define configSUPPORT_STATIC_ALLOCATION          1
#define configSUPPORT_DYNAMIC_ALLOCATION         0
#define configAPPLICATION_ALLOCATED_HEAP         0
#define configSTACK_ALLOCATION_FROM_SEPARATE_HEAP 0

/* T1 不使用队列、软件定时器、事件组或 POSIX。 */
#define configUSE_TIMERS                         0
#define configUSE_EVENT_GROUPS                   0
#define configUSE_STREAM_BUFFERS                 0
#define configUSE_TASK_NOTIFICATIONS             0
#define configUSE_MUTEXES                        0
#define configUSE_RECURSIVE_MUTEXES              0
#define configUSE_COUNTING_SEMAPHORES            0
#define configUSE_QUEUE_SETS                     0
#define configQUEUE_REGISTRY_SIZE                0
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS  0

#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0
#define configUSE_MALLOC_FAILED_HOOK             0
#define configCHECK_FOR_STACK_OVERFLOW           2
#define configUSE_TRACE_FACILITY                 0
#define configGENERATE_RUN_TIME_STATS            0
#define configUSE_STATS_FORMATTING_FUNCTIONS     0
#define configUSE_APPLICATION_TASK_TAG           0
#define configENABLE_BACKWARD_COMPATIBILITY      0
#define configUSE_MINI_LIST_ITEM                 1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES    1
#define configSTACK_DEPTH_TYPE                   size_t
#define configMESSAGE_BUFFER_LENGTH_TYPE         size_t
#define configENABLE_ISR_STACK_INIT              0

#define configPRIO_BITS                          2
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY  3
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 1
#define configKERNEL_INTERRUPT_PRIORITY          \
    (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY     \
    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

#define INCLUDE_vTaskDelay                       1
#define INCLUDE_vTaskDelayUntil                  1
#define INCLUDE_vTaskDelete                      0
#define INCLUDE_vTaskSuspend                     0
#define INCLUDE_vTaskPrioritySet                 0
#define INCLUDE_uxTaskPriorityGet                0
#define INCLUDE_xTaskGetCurrentTaskHandle        0
#define INCLUDE_xTaskGetSchedulerState           0
#define INCLUDE_uxTaskGetStackHighWaterMark      0
#define INCLUDE_eTaskGetState                    0
#define INCLUDE_xResumeFromISR                   0
#define INCLUDE_xEventGroupSetBitFromISR         0
#define INCLUDE_xTimerPendFunctionCall           0

void APP_AssertFailed(const char *file, unsigned long line);
#define configASSERT(x) do { \
    if ((x) == 0) { APP_AssertFailed(__FILE__, (unsigned long)__LINE__); } \
} while (0)

#endif
