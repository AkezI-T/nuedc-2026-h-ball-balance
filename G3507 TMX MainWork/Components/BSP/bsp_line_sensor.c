/**
 * @file bsp_line_sensor.c
 * @brief 亚博八路巡线模块异步 I2C 封装
 */

#include "bsp_line_sensor.h"
#include "bsp_i2c_async.h"

#include <stddef.h>

#define LINE_I2C_ADDRESS 0x12U
#define LINE_DATA_REG    0x30U

static uint8_t g_raw;
static BSP_I2C_Transaction_t g_transaction;

void BSP_LineSensor_Init(void)
{
    g_raw = 0xFFU;
    g_transaction.status = BSP_I2C_STATUS_IDLE;
}

bool BSP_LineSensor_Request(uint32_t now_ms)
{
    (void)now_ms;
    if (!BSP_I2C_SetupReadReg8Split(&g_transaction, LINE_I2C_ADDRESS,
        LINE_DATA_REG, &g_raw, 1U, 10U)) {
        return false;
    }
    return BSP_I2C_Submit(&g_transaction);
}

bool BSP_LineSensor_Take(uint8_t *raw, bool *ok)
{
    BSP_I2C_Status_t result;
    bool success;

    if (!BSP_I2C_TakeResult(&g_transaction, &result)) return false;
    success = (result == BSP_I2C_STATUS_OK);
    if (raw != NULL) *raw = g_raw;
    if (ok != NULL) *ok = success;
    return true;
}
