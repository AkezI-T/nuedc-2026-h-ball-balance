#include "bsp_d36a.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_check.h"

#define BSP_D36A_STEP_GPIO GPIO_NUM_18
#define BSP_D36A_DIR_GPIO GPIO_NUM_8
#define BSP_D36A_ENABLE_GPIO GPIO_NUM_3

#define BSP_D36A_LEDC_MODE LEDC_LOW_SPEED_MODE
#define BSP_D36A_LEDC_TIMER LEDC_TIMER_0
#define BSP_D36A_LEDC_CHANNEL LEDC_CHANNEL_0
#define BSP_D36A_LEDC_DUTY_RESOLUTION LEDC_TIMER_10_BIT
#define BSP_D36A_LEDC_HALF_DUTY 512U
#define BSP_D36A_LEDC_INITIAL_FREQUENCY_HZ 100U

static const char *TAG = "bsp_d36a";
static bool s_initialized;
static bool s_enabled;
static bool s_pulse_running;
static uint32_t s_step_frequency_hz;

static esp_err_t check_initialized(void)
{
    return s_initialized ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t bsp_d36a_init(void)
{
    /* 先写输出锁存器，再打开输出，避免初始化期间误使能。 */
    ESP_RETURN_ON_ERROR(gpio_set_level(BSP_D36A_DIR_GPIO, 0), TAG,
                        "DIR 初始电平设置失败");
    ESP_RETURN_ON_ERROR(gpio_set_level(BSP_D36A_ENABLE_GPIO, 0), TAG,
                        "EN 初始电平设置失败");

    const gpio_config_t output_config = {
        .pin_bit_mask = (1ULL << BSP_D36A_DIR_GPIO) |
                        (1ULL << BSP_D36A_ENABLE_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&output_config), TAG,
                        "D36A GPIO 配置失败");

    const ledc_timer_config_t timer_config = {
        .speed_mode = BSP_D36A_LEDC_MODE,
        .duty_resolution = BSP_D36A_LEDC_DUTY_RESOLUTION,
        .timer_num = BSP_D36A_LEDC_TIMER,
        .freq_hz = BSP_D36A_LEDC_INITIAL_FREQUENCY_HZ,
        /* 固定XTAL时钟，确保10位分辨率下可以输出50 Hz低速STEP。 */
        .clk_cfg = LEDC_USE_XTAL_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_config), TAG,
                        "D36A STEP定时器配置失败");

    const ledc_channel_config_t channel_config = {
        .gpio_num = BSP_D36A_STEP_GPIO,
        .speed_mode = BSP_D36A_LEDC_MODE,
        .channel = BSP_D36A_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = BSP_D36A_LEDC_TIMER,
        .duty = 0U,
        .hpoint = 0U,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&channel_config), TAG,
                        "D36A STEP通道配置失败");

    s_enabled = false;
    s_pulse_running = false;
    s_step_frequency_hz = BSP_D36A_LEDC_INITIAL_FREQUENCY_HZ;
    s_initialized = true;
    return ESP_OK;
}

esp_err_t bsp_d36a_set_enabled(bool enabled)
{
    ESP_RETURN_ON_ERROR(check_initialized(), TAG, "D36A 尚未初始化");
    ESP_RETURN_ON_ERROR(
        gpio_set_level(BSP_D36A_ENABLE_GPIO, enabled ? 1U : 0U), TAG,
        "D36A EN 电平设置失败");
    s_enabled = enabled;
    return ESP_OK;
}

esp_err_t bsp_d36a_set_direction(bsp_d36a_direction_t direction)
{
    ESP_RETURN_ON_ERROR(check_initialized(), TAG, "D36A 尚未初始化");
    return gpio_set_level(
        BSP_D36A_DIR_GPIO,
        direction == BSP_D36A_DIRECTION_TOWARD_FRONT ? 1U : 0U);
}

esp_err_t bsp_d36a_start_pulse(uint32_t frequency_hz)
{
    ESP_RETURN_ON_ERROR(check_initialized(), TAG, "D36A 尚未初始化");
    ESP_RETURN_ON_FALSE(s_enabled, ESP_ERR_INVALID_STATE, TAG,
                        "D36A 尚未使能");
    ESP_RETURN_ON_FALSE(frequency_hz > 0U, ESP_ERR_INVALID_ARG, TAG,
                        "STEP频率必须大于0");

    if (frequency_hz != s_step_frequency_hz) {
        ESP_RETURN_ON_ERROR(
            ledc_set_freq(BSP_D36A_LEDC_MODE, BSP_D36A_LEDC_TIMER,
                          frequency_hz),
            TAG, "D36A STEP频率设置失败");
        s_step_frequency_hz = frequency_hz;
    }
    if (!s_pulse_running) {
        ESP_RETURN_ON_ERROR(
            ledc_set_duty(BSP_D36A_LEDC_MODE, BSP_D36A_LEDC_CHANNEL,
                          BSP_D36A_LEDC_HALF_DUTY),
            TAG, "D36A STEP占空比设置失败");
        ESP_RETURN_ON_ERROR(
            ledc_update_duty(BSP_D36A_LEDC_MODE,
                             BSP_D36A_LEDC_CHANNEL),
            TAG, "D36A STEP启动失败");
        s_pulse_running = true;
    }
    return ESP_OK;
}

esp_err_t bsp_d36a_stop_pulse(void)
{
    ESP_RETURN_ON_ERROR(check_initialized(), TAG, "D36A 尚未初始化");
    if (!s_pulse_running) {
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(
        ledc_stop(BSP_D36A_LEDC_MODE, BSP_D36A_LEDC_CHANNEL, 0U),
        TAG, "D36A STEP停止失败");
    s_pulse_running = false;
    return ESP_OK;
}

esp_err_t bsp_d36a_hold(void)
{
    ESP_RETURN_ON_ERROR(bsp_d36a_stop_pulse(), TAG,
                        "停止 STEP 失败");
    return bsp_d36a_set_enabled(true);
}

esp_err_t bsp_d36a_release(void)
{
    ESP_RETURN_ON_ERROR(bsp_d36a_stop_pulse(), TAG,
                        "停止 STEP 失败");
    return bsp_d36a_set_enabled(false);
}

bool bsp_d36a_is_enabled(void)
{
    return s_enabled;
}
