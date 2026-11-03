/**
 * @file bsp_jy901s.c
 * @brief JY901S 角度寄存器异步 I2C 封装
 */

#include "bsp_jy901s.h"
#include "bsp_i2c_async.h"

#include <stddef.h>

#define JY901S_I2C_ADDRESS 0x50U
#define JY901S_ANGLE_REG   0x3DU

static uint8_t g_raw[6];
static BSP_I2C_Transaction_t g_transaction;

static int16_t raw_to_angle_x10(uint8_t low, uint8_t high)
{
    int16_t raw = (int16_t)(((uint16_t)high << 8) | low);
    return (int16_t)(((int32_t)raw * 1800) / 32768);
}

void BSP_JY901S_Init(void)
{
    g_transaction.status = BSP_I2C_STATUS_IDLE;
}

bool BSP_JY901S_Request(uint32_t now_ms)
{
    (void)now_ms;
    if (!BSP_I2C_SetupReadReg8(&g_transaction, JY901S_I2C_ADDRESS,
        JY901S_ANGLE_REG, g_raw, sizeof(g_raw), 15U)) {
        return false;
    }
    return BSP_I2C_Submit(&g_transaction);
}

bool BSP_JY901S_Take(BSP_JY901S_Angles *angles, bool *ok)
{
    BSP_I2C_Status_t result;
    bool success;

    if (!BSP_I2C_TakeResult(&g_transaction, &result)) return false;
    success = (result == BSP_I2C_STATUS_OK);
    if (ok != NULL) *ok = success;
    if (success && (angles != NULL)) {
        angles->roll_x10 = raw_to_angle_x10(g_raw[0], g_raw[1]);
        angles->pitch_x10 = raw_to_angle_x10(g_raw[2], g_raw[3]);
        angles->yaw_x10 = raw_to_angle_x10(g_raw[4], g_raw[5]);
    }
    return true;
}
