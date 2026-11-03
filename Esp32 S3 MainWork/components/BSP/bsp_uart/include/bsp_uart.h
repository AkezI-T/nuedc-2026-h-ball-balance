#ifndef BSP_UART_H
#define BSP_UART_H

#include <stddef.h>
#include <stdint.h>

#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

typedef struct {
    uart_port_t port;
    int tx_pin;
    int rx_pin;
    int baud_rate;
    int rx_buffer_size;
} bsp_uart_config_t;

esp_err_t bsp_uart_init(const bsp_uart_config_t *config);
int bsp_uart_read(uart_port_t port, void *data, size_t length,
                  TickType_t timeout_ticks);
int bsp_uart_write(uart_port_t port, const void *data, size_t length);

#endif
