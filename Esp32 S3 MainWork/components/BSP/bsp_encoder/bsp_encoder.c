#include "bsp_encoder.h"

#include <stdbool.h>

#include "driver/gpio.h"
#include "driver/mcpwm_cap.h"
#include "driver/pulse_cnt.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"

#define BSP_ENCODER_A_GPIO GPIO_NUM_4
#define BSP_ENCODER_B_GPIO GPIO_NUM_5
#define BSP_ENCODER_PWM_GPIO GPIO_NUM_6
#define BSP_ENCODER_Z_GPIO GPIO_NUM_7

#define BSP_ENCODER_PCNT_LOW_LIMIT (-30000)
#define BSP_ENCODER_PCNT_HIGH_LIMIT 30000
#define BSP_ENCODER_PCNT_GLITCH_NS 1000U
#define BSP_ENCODER_PWM_RESOLUTION_HZ 10000000U

static const char *TAG = "bsp_encoder";
static pcnt_unit_handle_t s_pcnt_unit;
static mcpwm_cap_timer_handle_t s_capture_timer;
static mcpwm_cap_channel_handle_t s_capture_channel;
static portMUX_TYPE s_capture_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_initialized;

static bool s_pwm_have_rise;
static uint32_t s_pwm_last_rise;
static uint32_t s_pwm_period;
static uint32_t s_pwm_high;
static uint32_t s_pwm_capture_count;
static uint32_t s_pwm_resolution_hz;
static uint32_t s_z_count;

static bool IRAM_ATTR pwm_capture_callback(
    mcpwm_cap_channel_handle_t channel,
    const mcpwm_capture_event_data_t *event,
    void *user_context)
{
    (void)channel;
    (void)user_context;

    portENTER_CRITICAL_ISR(&s_capture_lock);
    if (event->cap_edge == MCPWM_CAP_EDGE_POS) {
        if (s_pwm_have_rise) {
            s_pwm_period = event->cap_value - s_pwm_last_rise;
        }
        s_pwm_last_rise = event->cap_value;
        s_pwm_have_rise = true;
    } else if (s_pwm_have_rise && s_pwm_period != 0U) {
        uint32_t high = event->cap_value - s_pwm_last_rise;
        if (high > 0U && high < s_pwm_period) {
            s_pwm_high = high;
            s_pwm_capture_count++;
        }
    }
    portEXIT_CRITICAL_ISR(&s_capture_lock);
    return false;
}

static void IRAM_ATTR z_rising_isr(void *argument)
{
    (void)argument;
    portENTER_CRITICAL_ISR(&s_capture_lock);
    s_z_count++;
    portEXIT_CRITICAL_ISR(&s_capture_lock);
}

static esp_err_t init_ab_counter(void)
{
    const pcnt_unit_config_t unit_config = {
        .low_limit = BSP_ENCODER_PCNT_LOW_LIMIT,
        .high_limit = BSP_ENCODER_PCNT_HIGH_LIMIT,
        .flags.accum_count = true,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&unit_config, &s_pcnt_unit), TAG,
                        "创建A/B计数单元失败");

    const pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = BSP_ENCODER_PCNT_GLITCH_NS,
    };
    ESP_RETURN_ON_ERROR(
        pcnt_unit_set_glitch_filter(s_pcnt_unit, &filter_config), TAG,
        "设置A/B毛刺滤波失败");

    pcnt_channel_handle_t channel_a;
    const pcnt_chan_config_t channel_a_config = {
        .edge_gpio_num = BSP_ENCODER_A_GPIO,
        .level_gpio_num = BSP_ENCODER_B_GPIO,
    };
    ESP_RETURN_ON_ERROR(
        pcnt_new_channel(s_pcnt_unit, &channel_a_config, &channel_a), TAG,
        "创建A相计数通道失败");
    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_edge_action(
            channel_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE),
        TAG, "设置A相边沿动作失败");
    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_level_action(
            channel_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
        TAG, "设置A相电平动作失败");

    pcnt_channel_handle_t channel_b;
    const pcnt_chan_config_t channel_b_config = {
        .edge_gpio_num = BSP_ENCODER_B_GPIO,
        .level_gpio_num = BSP_ENCODER_A_GPIO,
    };
    ESP_RETURN_ON_ERROR(
        pcnt_new_channel(s_pcnt_unit, &channel_b_config, &channel_b), TAG,
        "创建B相计数通道失败");
    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_edge_action(
            channel_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE),
        TAG, "设置B相边沿动作失败");
    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_level_action(
            channel_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
        TAG, "设置B相电平动作失败");

    ESP_RETURN_ON_ERROR(
        pcnt_unit_add_watch_point(s_pcnt_unit,
                                  BSP_ENCODER_PCNT_LOW_LIMIT),
        TAG, "设置A/B下限监视点失败");
    ESP_RETURN_ON_ERROR(
        pcnt_unit_add_watch_point(s_pcnt_unit,
                                  BSP_ENCODER_PCNT_HIGH_LIMIT),
        TAG, "设置A/B上限监视点失败");
    ESP_RETURN_ON_ERROR(pcnt_unit_enable(s_pcnt_unit), TAG,
                        "使能A/B计数单元失败");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(s_pcnt_unit), TAG,
                        "清零A/B计数失败");
    return pcnt_unit_start(s_pcnt_unit);
}

static esp_err_t init_pwm_capture(void)
{
    const mcpwm_capture_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
        .resolution_hz = BSP_ENCODER_PWM_RESOLUTION_HZ,
    };
    ESP_RETURN_ON_ERROR(
        mcpwm_new_capture_timer(&timer_config, &s_capture_timer), TAG,
        "创建PWM捕获定时器失败");

    const mcpwm_capture_channel_config_t channel_config = {
        .gpio_num = BSP_ENCODER_PWM_GPIO,
        .prescale = 1,
        .flags.pos_edge = true,
        .flags.neg_edge = true,
        .flags.pull_down = true,
    };
    ESP_RETURN_ON_ERROR(
        mcpwm_new_capture_channel(s_capture_timer, &channel_config,
                                  &s_capture_channel),
        TAG, "创建PWM捕获通道失败");

    const mcpwm_capture_event_callbacks_t callbacks = {
        .on_cap = pwm_capture_callback,
    };
    ESP_RETURN_ON_ERROR(
        mcpwm_capture_channel_register_event_callbacks(
            s_capture_channel, &callbacks, NULL),
        TAG, "注册PWM捕获回调失败");
    ESP_RETURN_ON_ERROR(
        mcpwm_capture_channel_enable(s_capture_channel), TAG,
        "使能PWM捕获通道失败");
    ESP_RETURN_ON_ERROR(mcpwm_capture_timer_enable(s_capture_timer), TAG,
                        "使能PWM捕获定时器失败");
    ESP_RETURN_ON_ERROR(
        mcpwm_capture_timer_get_resolution(s_capture_timer,
                                           &s_pwm_resolution_hz),
        TAG, "读取PWM捕获分辨率失败");
    return mcpwm_capture_timer_start(s_capture_timer);
}

static esp_err_t init_z_input(void)
{
    const gpio_config_t config = {
        .pin_bit_mask = 1ULL << BSP_ENCODER_Z_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&config), TAG, "配置Z相输入失败");

    esp_err_t error = gpio_install_isr_service(0);
    if (error != ESP_OK && error != ESP_ERR_INVALID_STATE) {
        return error;
    }
    return gpio_isr_handler_add(BSP_ENCODER_Z_GPIO, z_rising_isr, NULL);
}

esp_err_t bsp_encoder_init(void)
{
    s_initialized = false;
    s_pwm_have_rise = false;
    s_pwm_last_rise = 0U;
    s_pwm_period = 0U;
    s_pwm_high = 0U;
    s_pwm_capture_count = 0U;
    s_pwm_resolution_hz = 0U;
    s_z_count = 0U;

    ESP_RETURN_ON_ERROR(init_ab_counter(), TAG, "A/B输入初始化失败");
    ESP_RETURN_ON_ERROR(init_pwm_capture(), TAG, "PWM输入初始化失败");
    ESP_RETURN_ON_ERROR(init_z_input(), TAG, "Z相输入初始化失败");
    s_initialized = true;
    return ESP_OK;
}

esp_err_t bsp_encoder_get_snapshot(bsp_encoder_snapshot_t *snapshot)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (snapshot == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int count;
    ESP_RETURN_ON_ERROR(pcnt_unit_get_count(s_pcnt_unit, &count), TAG,
                        "读取A/B计数失败");
    snapshot->ab_count = count;
    snapshot->a_level = gpio_get_level(BSP_ENCODER_A_GPIO);
    snapshot->b_level = gpio_get_level(BSP_ENCODER_B_GPIO);

    portENTER_CRITICAL(&s_capture_lock);
    snapshot->pwm_period_ticks = s_pwm_period;
    snapshot->pwm_high_ticks = s_pwm_high;
    snapshot->pwm_capture_count = s_pwm_capture_count;
    snapshot->pwm_resolution_hz = s_pwm_resolution_hz;
    snapshot->z_count = s_z_count;
    portEXIT_CRITICAL(&s_capture_lock);
    return ESP_OK;
}
