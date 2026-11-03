/**
 * @file bsp_encoder.c
 * @brief 编码器 A 相上升沿计数，B 相判向
 */

#include "bsp_encoder.h"
#include "ti_msp_dl_config.h"

#include <stddef.h>

static volatile int32_t g_count_a;
static volatile int32_t g_count_b;
static int32_t g_last_a;
static int32_t g_last_b;

void BSP_Encoder_Init(void)
{
    g_count_a = 0;
    g_count_b = 0;
    g_last_a = 0;
    g_last_b = 0;

    /* 编码器输出空闲时保持确定电平，避免悬空产生虚假计数。 */
    DL_GPIO_initDigitalInputFeatures(ENC1_E1A_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(ENC1_E1B_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(ENC2_E2A_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(ENC2_E2B_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_setLowerPinsPolarity(ENC1_PORT, DL_GPIO_PIN_6_EDGE_RISE);
    DL_GPIO_setLowerPinsPolarity(ENC2_PORT, DL_GPIO_PIN_12_EDGE_RISE);
    DL_GPIO_clearInterruptStatus(ENC1_PORT, ENC1_E1A_PIN);
    DL_GPIO_clearInterruptStatus(ENC2_PORT, ENC2_E2A_PIN);
    DL_GPIO_enableInterrupt(ENC1_PORT, ENC1_E1A_PIN);
    DL_GPIO_enableInterrupt(ENC2_PORT, ENC2_E2A_PIN);

    NVIC_SetPriority(GPIOA_INT_IRQn, 3U);
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
}

void BSP_Encoder_IRQHandler(void)
{
    uint32_t pending;

    pending = DL_GPIO_getEnabledInterruptStatus(ENC1_PORT, ENC1_E1A_PIN);
    if ((pending & ENC1_E1A_PIN) != 0U) {
        DL_GPIO_clearInterruptStatus(ENC1_PORT, ENC1_E1A_PIN);
        g_count_a += (DL_GPIO_readPins(ENC1_PORT, ENC1_E1B_PIN) != 0U) ? 1 : -1;
    }

    pending = DL_GPIO_getEnabledInterruptStatus(ENC2_PORT, ENC2_E2A_PIN);
    if ((pending & ENC2_E2A_PIN) != 0U) {
        DL_GPIO_clearInterruptStatus(ENC2_PORT, ENC2_E2A_PIN);
        g_count_b += (DL_GPIO_readPins(ENC2_PORT, ENC2_E2B_PIN) != 0U) ? 1 : -1;
    }
}

void BSP_Encoder_Read(BSP_EncoderSnapshot *snapshot)
{
    uint32_t primask;
    int32_t count_a;
    int32_t count_b;

    if (snapshot == NULL) return;

    primask = __get_PRIMASK();
    __disable_irq();
    count_a = g_count_a;
    count_b = g_count_b;
    snapshot->count_a = count_a;
    snapshot->count_b = count_b;
    snapshot->delta_a = count_a - g_last_a;
    snapshot->delta_b = count_b - g_last_b;
    g_last_a = count_a;
    g_last_b = count_b;
    if (primask == 0U) __enable_irq();
}
