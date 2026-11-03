#include "bsp_fill_light.h"

#include <stdbool.h>
#include <stdlib.h>

#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

#define BSP_FILL_LIGHT_RMT_RESOLUTION_HZ 10000000U
#define BSP_FILL_LIGHT_RMT_MEMORY_SYMBOLS 64U
#define BSP_FILL_LIGHT_TX_TIMEOUT_MS 100
#define BSP_FILL_LIGHT_RESET_US 80U

static const char *TAG = "bsp_fill_light";
static rmt_channel_handle_t s_tx_channel;
static rmt_encoder_handle_t s_bytes_encoder;
static uint8_t *s_grb_data;
static size_t s_led_count;
static bool s_initialized;

static void release_resources(void)
{
    if (s_tx_channel != NULL) {
        (void)rmt_disable(s_tx_channel);
    }
    if (s_bytes_encoder != NULL) {
        (void)rmt_del_encoder(s_bytes_encoder);
        s_bytes_encoder = NULL;
    }
    if (s_tx_channel != NULL) {
        (void)rmt_del_channel(s_tx_channel);
        s_tx_channel = NULL;
    }
    free(s_grb_data);
    s_grb_data = NULL;
    s_led_count = 0U;
    s_initialized = false;
}

esp_err_t bsp_fill_light_init(gpio_num_t gpio_num, size_t led_count)
{
    ESP_RETURN_ON_FALSE(!s_initialized, ESP_ERR_INVALID_STATE, TAG,
                        "补光灯已经初始化");
    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_OUTPUT_GPIO(gpio_num),
                        ESP_ERR_INVALID_ARG, TAG,
                        "GPIO%d不能作为输出", (int)gpio_num);
    ESP_RETURN_ON_FALSE(led_count > 0U && led_count <= 64U,
                        ESP_ERR_INVALID_ARG, TAG,
                        "灯珠数量必须为1至64");

    s_grb_data = calloc(led_count * 3U, sizeof(uint8_t));
    ESP_RETURN_ON_FALSE(s_grb_data != NULL, ESP_ERR_NO_MEM, TAG,
                        "补光灯数据缓冲区分配失败");
    s_led_count = led_count;

    const rmt_tx_channel_config_t channel_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = gpio_num,
        .mem_block_symbols = BSP_FILL_LIGHT_RMT_MEMORY_SYMBOLS,
        .resolution_hz = BSP_FILL_LIGHT_RMT_RESOLUTION_HZ,
        .trans_queue_depth = 1,
        .flags = {
            .invert_out = false,
            .with_dma = false,
        },
    };
    esp_err_t error = rmt_new_tx_channel(&channel_config, &s_tx_channel);
    if (error != ESP_OK) {
        release_resources();
        return error;
    }

    /* 10MHz下每个tick为0.1us，按WS2812的GRB、MSB优先时序发送。 */
    const rmt_bytes_encoder_config_t encoder_config = {
        .bit0 = {
            .level0 = 1,
            .duration0 = 3,
            .level1 = 0,
            .duration1 = 9,
        },
        .bit1 = {
            .level0 = 1,
            .duration0 = 9,
            .level1 = 0,
            .duration1 = 3,
        },
        .flags.msb_first = true,
    };
    error = rmt_new_bytes_encoder(&encoder_config, &s_bytes_encoder);
    if (error != ESP_OK) {
        release_resources();
        return error;
    }

    error = rmt_enable(s_tx_channel);
    if (error != ESP_OK) {
        release_resources();
        return error;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "WS2812补光灯就绪: GPIO%d, 灯珠=%u",
             (int)gpio_num, (unsigned)led_count);
    return ESP_OK;
}

esp_err_t bsp_fill_light_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, TAG,
                        "补光灯尚未初始化");

    for (size_t index = 0U; index < s_led_count; ++index) {
        size_t offset = index * 3U;
        s_grb_data[offset] = green;
        s_grb_data[offset + 1U] = red;
        s_grb_data[offset + 2U] = blue;
    }

    const rmt_transmit_config_t transmit_config = {
        .loop_count = 0,
        .flags = {
            .eot_level = 0,
            .queue_nonblocking = false,
        },
    };
    ESP_RETURN_ON_ERROR(
        rmt_transmit(s_tx_channel, s_bytes_encoder, s_grb_data,
                     s_led_count * 3U, &transmit_config),
        TAG, "补光灯数据发送失败");
    ESP_RETURN_ON_ERROR(
        rmt_tx_wait_all_done(s_tx_channel, BSP_FILL_LIGHT_TX_TIMEOUT_MS),
        TAG, "等待补光灯数据发送完成失败");

    esp_rom_delay_us(BSP_FILL_LIGHT_RESET_US);
    return ESP_OK;
}

esp_err_t bsp_fill_light_off(void)
{
    return bsp_fill_light_set_rgb(0U, 0U, 0U);
}
