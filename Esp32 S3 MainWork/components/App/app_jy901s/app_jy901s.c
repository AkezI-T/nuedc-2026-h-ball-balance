#include "app_jy901s.h"

#include <inttypes.h>
#include <stddef.h>

#include "app_core_affinity.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define JY901S_I2C_TASK_STACK_SIZE 3072U
#define JY901S_I2C_TASK_PRIORITY 6U
#define JY901S_SAMPLE_PERIOD_MS 20U
#define JY901S_I2C_TIMEOUT_MS 10
#define JY901S_I2C_ADDRESS 0x50U
#define JY901S_ACCELERATION_REGISTER 0x34U
#define JY901S_ACCELERATION_BYTES 6U
#define JY901S_ACCELERATION_RANGE_MG 16000

static const char *TAG = "app_jy901s";
static portMUX_TYPE s_state_lock = portMUX_INITIALIZER_UNLOCKED;
static app_jy901s_state_t s_state;
static i2c_master_bus_handle_t s_bus_handle;
static i2c_master_dev_handle_t s_device_handle;

static int16_t read_i16_le(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[1] << 8) | data[0]);
}

static int32_t raw_acceleration_to_mg(int16_t raw)
{
    return (int32_t)(((int64_t)raw * JY901S_ACCELERATION_RANGE_MG) /
                     32768LL);
}

static void publish_acceleration(const uint8_t *data)
{
    int16_t raw_x = read_i16_le(&data[0]);
    int16_t raw_y = read_i16_le(&data[2]);
    int16_t raw_z = read_i16_le(&data[4]);
    int32_t acceleration_x_mg = raw_acceleration_to_mg(raw_x);
    int32_t acceleration_y_mg = raw_acceleration_to_mg(raw_y);
    int32_t acceleration_z_mg = raw_acceleration_to_mg(raw_z);
    int64_t received_at_us = esp_timer_get_time();

    taskENTER_CRITICAL(&s_state_lock);
    int32_t filtered_x_mg = acceleration_x_mg;
    if (s_state.sample_count > 0U) {
        filtered_x_mg = s_state.filtered_acceleration_x_mg +
            (acceleration_x_mg - s_state.filtered_acceleration_x_mg) /
                CONFIG_GRM_JY901S_X_FILTER_DIVISOR;
    }
    s_state.valid = true;
    s_state.raw_x = raw_x;
    s_state.raw_y = raw_y;
    s_state.raw_z = raw_z;
    s_state.acceleration_x_mg = acceleration_x_mg;
    s_state.acceleration_y_mg = acceleration_y_mg;
    s_state.acceleration_z_mg = acceleration_z_mg;
    s_state.filtered_acceleration_x_mg = filtered_x_mg;
    s_state.longitudinal_acceleration_mg =
        filtered_x_mg - CONFIG_GRM_JY901S_X_BIAS_MG;
    s_state.received_at_us = received_at_us;
    s_state.sample_count++;
    taskEXIT_CRITICAL(&s_state_lock);
}

static void record_read_error(esp_err_t error)
{
    uint32_t error_count;
    taskENTER_CRITICAL(&s_state_lock);
    s_state.read_error_count++;
    error_count = s_state.read_error_count;
    taskEXIT_CRITICAL(&s_state_lock);

    if (error_count == 1U || error_count % 100U == 0U) {
        ESP_LOGW(TAG, "JY901S I2C读取失败: %s, count=%" PRIu32,
                 esp_err_to_name(error), error_count);
    }
}

static void jy901s_i2c_task(void *argument)
{
    (void)argument;
    uint8_t register_address = JY901S_ACCELERATION_REGISTER;
    uint8_t data[JY901S_ACCELERATION_BYTES];
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        esp_err_t error = i2c_master_transmit_receive(
            s_device_handle, &register_address, sizeof(register_address),
            data, sizeof(data), JY901S_I2C_TIMEOUT_MS);
        if (error == ESP_OK) {
            publish_acceleration(data);
        } else {
            record_read_error(error);
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(JY901S_SAMPLE_PERIOD_MS));
    }
}

esp_err_t app_jy901s_init(void)
{
#if defined(CONFIG_GRM_JY901S_I2C_ENABLE) && CONFIG_GRM_JY901S_I2C_ENABLE
    s_state = (app_jy901s_state_t){0};
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = CONFIG_GRM_JY901S_I2C_PORT,
        .sda_io_num = CONFIG_GRM_JY901S_I2C_SDA_PIN,
        .scl_io_num = CONFIG_GRM_JY901S_I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7U,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t error = i2c_new_master_bus(&bus_config, &s_bus_handle);
    if (error != ESP_OK) {
        return error;
    }

    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = JY901S_I2C_ADDRESS,
        .scl_speed_hz = CONFIG_GRM_JY901S_I2C_CLOCK_HZ,
    };
    error = i2c_master_bus_add_device(s_bus_handle, &device_config,
                                      &s_device_handle);
    if (error != ESP_OK) {
        (void)i2c_del_master_bus(s_bus_handle);
        s_bus_handle = NULL;
        return error;
    }

    if (xTaskCreatePinnedToCore(
            jy901s_i2c_task, "jy901s_i2c",
            JY901S_I2C_TASK_STACK_SIZE, NULL,
            JY901S_I2C_TASK_PRIORITY, NULL,
            APP_CORE_COMMUNICATION) != pdPASS) {
        (void)i2c_master_bus_rm_device(s_device_handle);
        (void)i2c_del_master_bus(s_bus_handle);
        s_device_handle = NULL;
        s_bus_handle = NULL;
        return ESP_ERR_NO_MEM;
    }

    taskENTER_CRITICAL(&s_state_lock);
    s_state.initialized = true;
    taskEXIT_CRITICAL(&s_state_lock);
    ESP_LOGI(TAG,
             "JY901S I2C采集已启动: I2C%d SDA=%d SCL=%d clock=%dHz"
             " addr=0x%02X Core=%d",
             CONFIG_GRM_JY901S_I2C_PORT,
             CONFIG_GRM_JY901S_I2C_SDA_PIN,
             CONFIG_GRM_JY901S_I2C_SCL_PIN,
             CONFIG_GRM_JY901S_I2C_CLOCK_HZ,
             JY901S_I2C_ADDRESS,
             APP_CORE_COMMUNICATION);
    return ESP_OK;
#else
    s_state = (app_jy901s_state_t){0};
    ESP_LOGW(TAG, "JY901S I2C采集未启用");
    return ESP_OK;
#endif
}

void app_jy901s_get_state(app_jy901s_state_t *state)
{
    if (state == NULL) {
        return;
    }
    taskENTER_CRITICAL(&s_state_lock);
    *state = s_state;
    taskEXIT_CRITICAL(&s_state_lock);
}
