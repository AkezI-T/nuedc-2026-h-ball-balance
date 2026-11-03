#include "bsp_uart.h"

#include "esp_check.h"

static const char *TAG = "bsp_uart";

esp_err_t bsp_uart_init(const bsp_uart_config_t *config)
{
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "配置不能为空");
    ESP_RETURN_ON_FALSE(config->port >= UART_NUM_0 &&
                        config->port < UART_NUM_MAX,
                        ESP_ERR_INVALID_ARG, TAG, "UART 端口无效");
    ESP_RETURN_ON_FALSE(config->tx_pin != UART_PIN_NO_CHANGE ||
                        config->rx_pin != UART_PIN_NO_CHANGE,
                        ESP_ERR_INVALID_ARG, TAG, "至少配置一个引脚");
    ESP_RETURN_ON_FALSE(config->baud_rate > 0, ESP_ERR_INVALID_ARG, TAG,
                        "波特率无效");

    const uart_config_t uart_config = {
        .baud_rate = config->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    int rx_buffer_size = config->rx_buffer_size;
    if (rx_buffer_size < UART_HW_FIFO_LEN(config->port) + 1) {
        rx_buffer_size = UART_HW_FIFO_LEN(config->port) + 1;
    }
    ESP_RETURN_ON_ERROR(
        uart_driver_install(config->port, rx_buffer_size, 0, 0, NULL, 0),
        TAG, "安装 UART 驱动失败");

    esp_err_t error = uart_param_config(config->port, &uart_config);
    if (error == ESP_OK) {
        error = uart_set_pin(config->port, config->tx_pin, config->rx_pin,
                             UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (error != ESP_OK) {
        (void)uart_driver_delete(config->port);
        ESP_LOGE(TAG, "配置 UART 失败: %s", esp_err_to_name(error));
    }
    return error;
}

int bsp_uart_read(uart_port_t port, void *data, size_t length,
                  TickType_t timeout_ticks)
{
    if (data == NULL || length == 0U) {
        return 0;
    }
    return uart_read_bytes(port, data, (uint32_t)length, timeout_ticks);
}

int bsp_uart_write(uart_port_t port, const void *data, size_t length)
{
    if (data == NULL || length == 0U) {
        return 0;
    }
    return uart_write_bytes(port, data, length);
}
