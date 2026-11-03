/**
 * @file    bsp_i2c_async.h
 * @brief   通用 I2C0 非阻塞事务队列
 *
 * 设备 BSP 持有事务对象并提交到 FIFO。I2C 层不认识具体传感器，只处理地址、
 * TX/RX 数据、超时和结果。事务完成前，事务对象及 RX 缓冲区必须保持有效。
 */

#ifndef BSP_I2C_ASYNC_H
#define BSP_I2C_ASYNC_H

#include <stdbool.h>
#include <stdint.h>

#define BSP_I2C_MAX_TX_BYTES  16U

typedef enum {
    BSP_I2C_STATUS_IDLE = 0,
    BSP_I2C_STATUS_QUEUED,
    BSP_I2C_STATUS_ACTIVE,
    BSP_I2C_STATUS_OK,
    BSP_I2C_STATUS_ERROR,
    BSP_I2C_STATUS_TIMEOUT,
} BSP_I2C_Status_t;

typedef struct {
    uint8_t address;
    uint8_t tx[BSP_I2C_MAX_TX_BYTES];
    uint8_t tx_len;
    uint8_t *rx;
    uint8_t rx_len;
    uint16_t timeout_ms;
    bool stop_between_phases;
    volatile BSP_I2C_Status_t status;
} BSP_I2C_Transaction_t;

void BSP_I2C_AsyncInit(void);
void BSP_I2C_AsyncStep(uint32_t now_ms);

bool BSP_I2C_Setup(BSP_I2C_Transaction_t *transaction, uint8_t address,
                   const uint8_t *tx, uint8_t tx_len,
                   uint8_t *rx, uint8_t rx_len, uint16_t timeout_ms);
bool BSP_I2C_SetupReadReg8(BSP_I2C_Transaction_t *transaction,
                           uint8_t address, uint8_t reg,
                           uint8_t *rx, uint8_t rx_len,
                           uint16_t timeout_ms);
bool BSP_I2C_SetupReadReg8Split(BSP_I2C_Transaction_t *transaction,
                                uint8_t address, uint8_t reg,
                                uint8_t *rx, uint8_t rx_len,
                                uint16_t timeout_ms);
bool BSP_I2C_SetupWriteReg8(BSP_I2C_Transaction_t *transaction,
                            uint8_t address, uint8_t reg,
                            const uint8_t *data, uint8_t data_len,
                            uint16_t timeout_ms);
bool BSP_I2C_Submit(BSP_I2C_Transaction_t *transaction);
bool BSP_I2C_TakeResult(BSP_I2C_Transaction_t *transaction,
                        BSP_I2C_Status_t *result);

bool BSP_I2C_IsIdle(void);
uint8_t BSP_I2C_GetQueueCount(void);
void BSP_I2C_IRQHandler(void);

#endif
