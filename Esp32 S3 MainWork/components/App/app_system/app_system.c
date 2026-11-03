#include "app_system.h"

#include "app_encoder_monitor.h"
#include "app_jy901s.h"
#include "app_motor_control.h"
#include "app_task_dispatch.h"
#include "app_tmx_ingest.h"
#include "app_udp_log.h"
#include "app_vision.h"
#include "app_wifi.h"
#include "bsp_fill_light.h"
#include "esp_check.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "sdkconfig.h"

static const char *TAG = "app_system";

esp_err_t app_system_start(void)
{
    ESP_LOGI(TAG, "车载平衡滚球集中处理器启动，ESP-IDF %s",
             esp_get_idf_version());
    ESP_LOGI(TAG,
             "Core 0: 通信/无线/日志/监控，Core 1: D36A控制/编码器采集");

#if CONFIG_GRM_FILL_LIGHT_ENABLE
    esp_err_t fill_light_error = bsp_fill_light_init(
        (gpio_num_t)CONFIG_GRM_FILL_LIGHT_GPIO,
        (size_t)CONFIG_GRM_FILL_LIGHT_LED_COUNT);
    if (fill_light_error == ESP_OK) {
        fill_light_error = bsp_fill_light_set_rgb(
            (uint8_t)CONFIG_GRM_FILL_LIGHT_BRIGHTNESS,
            (uint8_t)CONFIG_GRM_FILL_LIGHT_BRIGHTNESS,
            (uint8_t)CONFIG_GRM_FILL_LIGHT_BRIGHTNESS);
    }
    if (fill_light_error != ESP_OK) {
        ESP_LOGE(TAG, "WS2812补光灯启动失败: %s",
                 esp_err_to_name(fill_light_error));
    } else {
        ESP_LOGI(TAG, "WS2812补光灯已点亮: GPIO%d, 灯珠=%d, 亮度=%d/255",
                 CONFIG_GRM_FILL_LIGHT_GPIO,
                 CONFIG_GRM_FILL_LIGHT_LED_COUNT,
                 CONFIG_GRM_FILL_LIGHT_BRIGHTNESS);
    }
#endif

    ESP_RETURN_ON_ERROR(app_encoder_monitor_init(), TAG,
                        "编码器诊断任务启动失败");
    ESP_RETURN_ON_ERROR(app_jy901s_init(), TAG,
                        "JY901S I2C采集任务启动失败");
    ESP_RETURN_ON_ERROR(app_task_dispatch_init(), TAG,
                        "任务调度器启动失败");
    ESP_RETURN_ON_ERROR(app_wifi_init(), TAG, "无线子系统启动失败");
    ESP_RETURN_ON_ERROR(app_udp_log_init(), TAG,
                        "上位机UDP日志启动失败");
    ESP_RETURN_ON_ERROR(app_vision_init(), TAG, "K230 通信启动失败");
    ESP_RETURN_ON_ERROR(app_motor_control_init(), TAG,
                        "D36A 控制任务启动失败");
    ESP_RETURN_ON_ERROR(app_tmx_ingest_init(), TAG, "TMX 通信启动失败");
    uint8_t primary_channel = 0U;
    wifi_second_chan_t secondary_channel = WIFI_SECOND_CHAN_NONE;
    ESP_RETURN_ON_ERROR(
        esp_wifi_get_channel(&primary_channel, &secondary_channel), TAG,
        "读取无线信道失败");
    ESP_LOGI(TAG, "系统就绪，当前 Wi-Fi 信道=%u",
             (unsigned)primary_channel);
    return ESP_OK;
}
