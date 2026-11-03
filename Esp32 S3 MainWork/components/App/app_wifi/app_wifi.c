#include "app_wifi.h"

#include <string.h>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#define WIFI_CONNECTED_BIT BIT0

#if defined(CONFIG_GRM_WIFI_CONNECT_ENABLE) && \
    CONFIG_GRM_WIFI_CONNECT_ENABLE
#define GRM_WIFI_CONNECT_ENABLED true
#else
#define GRM_WIFI_CONNECT_ENABLED false
#endif

static const char *TAG = "app_wifi";
static EventGroupHandle_t s_wifi_events;
static bool s_connect_enabled;

static void wifi_event_handler(void *argument, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)argument;
    (void)event_data;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_wifi_events, WIFI_CONNECTED_BIT);
        if (s_connect_enabled) {
            esp_err_t error = esp_wifi_connect();
            if (error != ESP_OK) {
                ESP_LOGW(TAG, "重连失败: %s", esp_err_to_name(error));
            }
        }
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *event = event_data;
        uint8_t primary_channel = 0U;
        wifi_second_chan_t secondary_channel = WIFI_SECOND_CHAN_NONE;
        esp_err_t error = esp_wifi_get_channel(&primary_channel,
                                               &secondary_channel);
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
        if (error == ESP_OK) {
            ESP_LOGI(TAG,
                      "已连接 K230 同一网络，IP=" IPSTR
                      "，实际 Wi-Fi 信道=%u",
                     IP2STR(&event->ip_info.ip),
                     (unsigned)primary_channel);
        } else {
            ESP_LOGW(TAG, "已连接 K230 同一网络，IP=" IPSTR
                          "，读取实际信道失败: %s",
                     IP2STR(&event->ip_info.ip), esp_err_to_name(error));
        }
    }
}

static esp_err_t init_nvs(void)
{
    esp_err_t error = nvs_flash_init();
    if (error == ESP_ERR_NVS_NO_FREE_PAGES ||
        error == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "擦除 NVS 失败");
        error = nvs_flash_init();
    }
    return error;
}

esp_err_t app_wifi_init(void)
{
    ESP_RETURN_ON_ERROR(init_nvs(), TAG, "初始化 NVS 失败");
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "初始化网络接口失败");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG,
                        "创建事件循环失败");

    s_wifi_events = xEventGroupCreate();
    ESP_RETURN_ON_FALSE(s_wifi_events != NULL, ESP_ERR_NO_MEM, TAG,
                        "创建 Wi-Fi 事件组失败");

    esp_netif_create_default_wifi_sta();
    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&wifi_init_config), TAG,
                        "初始化 Wi-Fi 失败");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG,
                        "配置 Wi-Fi 存储失败");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG,
                        "配置 STA 模式失败");

    ESP_RETURN_ON_ERROR(
        esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                   wifi_event_handler, NULL),
        TAG, "注册 Wi-Fi 事件失败");
    ESP_RETURN_ON_ERROR(
        esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                   wifi_event_handler, NULL),
        TAG, "注册 IP 事件失败");

    s_connect_enabled = GRM_WIFI_CONNECT_ENABLED &&
                        CONFIG_GRM_WIFI_SSID[0] != '\0';
    if (s_connect_enabled) {
        wifi_config_t station_config = {0};
        strlcpy((char *)station_config.sta.ssid, CONFIG_GRM_WIFI_SSID,
                sizeof(station_config.sta.ssid));
        strlcpy((char *)station_config.sta.password,
                CONFIG_GRM_WIFI_PASSWORD,
                sizeof(station_config.sta.password));
        station_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
        station_config.sta.pmf_cfg.capable = true;
        station_config.sta.pmf_cfg.required = false;
        station_config.sta.channel = 0U;
        ESP_RETURN_ON_ERROR(
            esp_wifi_set_config(WIFI_IF_STA, &station_config), TAG,
            "配置 Wi-Fi 凭据失败");
    }

    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "启动 Wi-Fi 失败");
    ESP_RETURN_ON_ERROR(esp_wifi_set_ps(WIFI_PS_NONE), TAG,
                        "关闭 Wi-Fi 省电失败");
    ESP_LOGI(TAG, "Wi-Fi省电已关闭，STA自动跟随热点信道");

    if (s_connect_enabled) {
        ESP_RETURN_ON_ERROR(esp_wifi_connect(), TAG, "启动 Wi-Fi 连接失败");
    } else {
        ESP_LOGW(TAG, "未配置STA凭据，UDP视觉与日志均不可用");
    }
    return ESP_OK;
}

bool app_wifi_is_connected(void)
{
    return s_wifi_events != NULL &&
           (xEventGroupGetBits(s_wifi_events) & WIFI_CONNECTED_BIT) != 0U;
}

bool app_wifi_wait_connected(TickType_t timeout_ticks)
{
    if (s_wifi_events == NULL) {
        return false;
    }
    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_events, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, timeout_ticks);
    return (bits & WIFI_CONNECTED_BIT) != 0U;
}
