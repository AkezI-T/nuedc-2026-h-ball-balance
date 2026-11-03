/**
 * @file    bsp_i2c_async.c
 * @brief   基于 MSPM0 DriverLib FIFO 中断的通用 I2C0 事务队列
 */

#include "bsp_i2c_async.h"
#include "ti_msp_dl_config.h"

#define BSP_I2C_QUEUE_SIZE          8U
#define BSP_I2C_DEFAULT_TIMEOUT_MS  20U
#define BSP_I2C_CONTROLLER_INTERRUPTS \
    (DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST | \
     DL_I2C_INTERRUPT_CONTROLLER_NACK | \
     DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER | \
     DL_I2C_INTERRUPT_CONTROLLER_RX_DONE | \
     DL_I2C_INTERRUPT_CONTROLLER_TX_DONE | \
     DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER)

typedef enum {
    I2C_HW_IDLE = 0,
    I2C_HW_TX,
    I2C_HW_WAIT_SPLIT_RX,
    I2C_HW_RX,
} I2C_HwState_t;

static BSP_I2C_Transaction_t *g_queue[BSP_I2C_QUEUE_SIZE];
static volatile uint8_t g_head;
static volatile uint8_t g_tail;
static volatile uint8_t g_count;
static BSP_I2C_Transaction_t *volatile g_active;
static volatile I2C_HwState_t g_hwState;
static volatile uint8_t g_txCount;
static volatile uint8_t g_rxCount;
static uint32_t g_startMs;

static bool is_finished(BSP_I2C_Status_t status)
{
    return status == BSP_I2C_STATUS_OK ||
           status == BSP_I2C_STATUS_ERROR ||
           status == BSP_I2C_STATUS_TIMEOUT;
}

static void reset_transfer(void)
{
    DL_I2C_disableInterrupt(I2C_GRAY_INST,
        DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
    DL_I2C_resetControllerTransfer(I2C_GRAY_INST);
    DL_I2C_flushControllerTXFIFO(I2C_GRAY_INST);
    DL_I2C_flushControllerRXFIFO(I2C_GRAY_INST);
    DL_I2C_clearInterruptStatus(I2C_GRAY_INST,
        BSP_I2C_CONTROLLER_INTERRUPTS);
    g_hwState = I2C_HW_IDLE;
    g_txCount = 0;
    g_rxCount = 0;
}

static void finish_active(BSP_I2C_Status_t status)
{
    BSP_I2C_Transaction_t *transaction = g_active;
    if (transaction) transaction->status = status;
    g_active = NULL;
    g_hwState = I2C_HW_IDLE;
}

static void drain_rx_fifo(void)
{
    BSP_I2C_Transaction_t *transaction = g_active;
    while (!DL_I2C_isControllerRXFIFOEmpty(I2C_GRAY_INST)) {
        uint8_t data = DL_I2C_receiveControllerData(I2C_GRAY_INST);
        if (transaction && g_rxCount < transaction->rx_len) {
            transaction->rx[g_rxCount++] = data;
        }
    }
}

static void start_rx_phase(BSP_I2C_Transaction_t *transaction)
{
    DL_I2C_flushControllerRXFIFO(I2C_GRAY_INST);
    DL_I2C_clearInterruptStatus(I2C_GRAY_INST,
        DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER |
        DL_I2C_INTERRUPT_CONTROLLER_RX_DONE);
    g_rxCount = 0U;
    g_hwState = I2C_HW_RX;
    DL_I2C_startControllerTransferAdvanced(I2C_GRAY_INST,
        transaction->address, DL_I2C_CONTROLLER_DIRECTION_RX,
        transaction->rx_len, DL_I2C_CONTROLLER_START_ENABLE,
        DL_I2C_CONTROLLER_STOP_ENABLE,
        DL_I2C_CONTROLLER_ACK_DISABLE);
}

static void start_next(uint32_t now_ms)
{
    BSP_I2C_Transaction_t *transaction;
    if (g_active || g_count == 0U) return;
    if (!(DL_I2C_getControllerStatus(I2C_GRAY_INST) &
          DL_I2C_CONTROLLER_STATUS_IDLE)) return;

    __disable_irq();
    transaction = g_queue[g_head];
    g_head = (uint8_t)((g_head + 1U) % BSP_I2C_QUEUE_SIZE);
    g_count--;
    g_active = transaction;
    __enable_irq();

    transaction->status = BSP_I2C_STATUS_ACTIVE;
    /* 热插拔/NACK 后可能残留 FIFO 数据，每笔事务必须从干净边界开始。 */
    DL_I2C_flushControllerTXFIFO(I2C_GRAY_INST);
    DL_I2C_flushControllerRXFIFO(I2C_GRAY_INST);
    DL_I2C_clearInterruptStatus(I2C_GRAY_INST,
        BSP_I2C_CONTROLLER_INTERRUPTS);
    g_rxCount = 0;
    g_txCount = 0;
    g_startMs = now_ms;

    if (transaction->tx_len == 0U) {
        g_hwState = I2C_HW_RX;
        DL_I2C_startControllerTransferAdvanced(I2C_GRAY_INST,
            transaction->address, DL_I2C_CONTROLLER_DIRECTION_RX,
            transaction->rx_len, DL_I2C_CONTROLLER_START_ENABLE,
            DL_I2C_CONTROLLER_STOP_ENABLE,
            DL_I2C_CONTROLLER_ACK_DISABLE);
    } else {
        g_txCount = DL_I2C_fillControllerTXFIFO(
            I2C_GRAY_INST, transaction->tx, transaction->tx_len);
        if (g_txCount < transaction->tx_len) {
            DL_I2C_enableInterrupt(
                I2C_GRAY_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
        } else {
            DL_I2C_disableInterrupt(
                I2C_GRAY_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
        }
        g_hwState = I2C_HW_TX;
        DL_I2C_startControllerTransferAdvanced(I2C_GRAY_INST,
            transaction->address, DL_I2C_CONTROLLER_DIRECTION_TX,
            transaction->tx_len, DL_I2C_CONTROLLER_START_ENABLE,
            (transaction->rx_len && !transaction->stop_between_phases) ?
                DL_I2C_CONTROLLER_STOP_DISABLE :
                DL_I2C_CONTROLLER_STOP_ENABLE,
            DL_I2C_CONTROLLER_ACK_DISABLE);
    }
}

void BSP_I2C_AsyncInit(void)
{
    g_head = 0;
    g_tail = 0;
    g_count = 0;
    g_active = NULL;
    g_hwState = I2C_HW_IDLE;

    DL_I2C_setControllerTXFIFOThreshold(
        I2C_GRAY_INST, DL_I2C_TX_FIFO_LEVEL_BYTES_1);
    DL_I2C_setControllerRXFIFOThreshold(
        I2C_GRAY_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    reset_transfer();
    DL_I2C_enableInterrupt(I2C_GRAY_INST,
        BSP_I2C_CONTROLLER_INTERRUPTS &
        ~DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);

    NVIC_SetPriority(I2C_GRAY_INST_INT_IRQN, 3);
    NVIC_EnableIRQ(I2C_GRAY_INST_INT_IRQN);
}

bool BSP_I2C_Setup(BSP_I2C_Transaction_t *transaction, uint8_t address,
                   const uint8_t *tx, uint8_t tx_len,
                   uint8_t *rx, uint8_t rx_len, uint16_t timeout_ms)
{
    if (!transaction || (tx_len == 0U && rx_len == 0U) ||
        (tx_len && !tx) || tx_len > BSP_I2C_MAX_TX_BYTES ||
        (rx_len && !rx) ||
        transaction->status == BSP_I2C_STATUS_QUEUED ||
        transaction->status == BSP_I2C_STATUS_ACTIVE) {
        return false;
    }

    transaction->address = address;
    for (uint8_t i = 0; i < tx_len; i++) transaction->tx[i] = tx[i];
    transaction->tx_len = tx_len;
    transaction->rx = rx;
    transaction->rx_len = rx_len;
    transaction->timeout_ms = timeout_ms ? timeout_ms : BSP_I2C_DEFAULT_TIMEOUT_MS;
    transaction->stop_between_phases = false;
    transaction->status = BSP_I2C_STATUS_IDLE;
    return true;
}

bool BSP_I2C_SetupReadReg8Split(BSP_I2C_Transaction_t *transaction,
                                uint8_t address, uint8_t reg,
                                uint8_t *rx, uint8_t rx_len,
                                uint16_t timeout_ms)
{
    if (!BSP_I2C_Setup(transaction, address, &reg, 1U,
        rx, rx_len, timeout_ms)) {
        return false;
    }
    transaction->stop_between_phases = true;
    return true;
}

bool BSP_I2C_SetupReadReg8(BSP_I2C_Transaction_t *transaction,
                           uint8_t address, uint8_t reg,
                           uint8_t *rx, uint8_t rx_len,
                           uint16_t timeout_ms)
{
    return BSP_I2C_Setup(transaction, address, &reg, 1,
                         rx, rx_len, timeout_ms);
}

bool BSP_I2C_SetupWriteReg8(BSP_I2C_Transaction_t *transaction,
                            uint8_t address, uint8_t reg,
                            const uint8_t *data, uint8_t data_len,
                            uint16_t timeout_ms)
{
    uint8_t tx[BSP_I2C_MAX_TX_BYTES];
    if (!data || data_len == 0U ||
        (uint8_t)(data_len + 1U) > BSP_I2C_MAX_TX_BYTES) return false;

    tx[0] = reg;
    for (uint8_t i = 0; i < data_len; i++) tx[i + 1U] = data[i];
    return BSP_I2C_Setup(transaction, address, tx, data_len + 1U,
                         NULL, 0, timeout_ms);
}

bool BSP_I2C_Submit(BSP_I2C_Transaction_t *transaction)
{
    if (!transaction || transaction->status != BSP_I2C_STATUS_IDLE) return false;

    __disable_irq();
    if (g_count >= BSP_I2C_QUEUE_SIZE) {
        __enable_irq();
        return false;
    }
    transaction->status = BSP_I2C_STATUS_QUEUED;
    g_queue[g_tail] = transaction;
    g_tail = (uint8_t)((g_tail + 1U) % BSP_I2C_QUEUE_SIZE);
    g_count++;
    __enable_irq();
    return true;
}

bool BSP_I2C_TakeResult(BSP_I2C_Transaction_t *transaction,
                        BSP_I2C_Status_t *result)
{
    BSP_I2C_Status_t status;
    if (!transaction) return false;
    status = transaction->status;
    if (!is_finished(status)) return false;
    if (result) *result = status;
    transaction->status = BSP_I2C_STATUS_IDLE;
    return true;
}

void BSP_I2C_AsyncStep(uint32_t now_ms)
{
    BSP_I2C_Transaction_t *transaction = g_active;
    if (transaction &&
        (uint32_t)(now_ms - g_startMs) >= transaction->timeout_ms) {
        reset_transfer();
        finish_active(BSP_I2C_STATUS_TIMEOUT);
    } else if (transaction && (g_hwState == I2C_HW_WAIT_SPLIT_RX) &&
        ((DL_I2C_getControllerStatus(I2C_GRAY_INST) &
          DL_I2C_CONTROLLER_STATUS_IDLE) != 0U)) {
        /* 亚博模块要求上一笔写事务完成 STOP 后再发起独立读事务。 */
        start_rx_phase(transaction);
    }

    start_next(now_ms);
}

bool BSP_I2C_IsIdle(void)
{
    return g_active == NULL && g_count == 0U;
}

uint8_t BSP_I2C_GetQueueCount(void)
{
    return g_count;
}

void BSP_I2C_IRQHandler(void)
{
    BSP_I2C_Transaction_t *transaction = g_active;
    switch (DL_I2C_getPendingInterrupt(I2C_GRAY_INST)) {
    case DL_I2C_IIDX_CONTROLLER_TX_DONE:
        if (!transaction || g_hwState != I2C_HW_TX) break;
        DL_I2C_disableInterrupt(
            I2C_GRAY_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
        if (transaction->rx_len == 0U) {
            finish_active(BSP_I2C_STATUS_OK);
        } else if (transaction->stop_between_phases) {
            /* SensorTask 下一次 1 ms 调度时确认总线空闲后再读。 */
            g_hwState = I2C_HW_WAIT_SPLIT_RX;
        } else {
            start_rx_phase(transaction);
        }
        break;
    case DL_I2C_IIDX_CONTROLLER_RXFIFO_TRIGGER:
        drain_rx_fifo();
        break;
    case DL_I2C_IIDX_CONTROLLER_TXFIFO_TRIGGER:
        if (transaction && g_txCount < transaction->tx_len) {
            g_txCount += DL_I2C_fillControllerTXFIFO(I2C_GRAY_INST,
                &transaction->tx[g_txCount], transaction->tx_len - g_txCount);
        }
        break;
    case DL_I2C_IIDX_CONTROLLER_RX_DONE:
        drain_rx_fifo();
        finish_active(transaction && g_rxCount == transaction->rx_len ?
                      BSP_I2C_STATUS_OK : BSP_I2C_STATUS_ERROR);
        break;
    case DL_I2C_IIDX_CONTROLLER_ARBITRATION_LOST:
    case DL_I2C_IIDX_CONTROLLER_NACK:
        reset_transfer();
        finish_active(BSP_I2C_STATUS_ERROR);
        break;
    default:
        break;
    }
}

void I2C_GRAY_INST_IRQHandler(void)
{
    BSP_I2C_IRQHandler();
}
