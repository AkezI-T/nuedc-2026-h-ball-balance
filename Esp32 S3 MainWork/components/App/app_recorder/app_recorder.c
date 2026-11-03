#include "app_recorder.h"

#include <stdio.h>
#include <string.h>

#include "app_core_affinity.h"
#include "app_wifi.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "sdkconfig.h"

#define RECORDER_QUEUE_LENGTH 8U
#define RECORDER_TASK_STACK_SIZE 4096U

#if defined(CONFIG_GRM_RECORDER_ENABLE) && CONFIG_GRM_RECORDER_ENABLE
#define GRM_RECORDER_ENABLED true
#else
#define GRM_RECORDER_ENABLED false
#endif

typedef enum {
    RECORDER_ACTION_START,
    RECORDER_ACTION_STOP,
} recorder_action_t;

typedef struct {
    recorder_action_t action;
    union {
        app_recorder_start_t start;
        app_recorder_stop_t stop;
    } data;
} recorder_queue_item_t;

static const char *TAG = "app_recorder";
static QueueHandle_t s_event_queue;

static esp_err_t send_udp_json(const char *message)
{
    char port[8];
    (void)snprintf(port, sizeof(port), "%d", CONFIG_GRM_RECORDER_PORT);
    const struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_DGRAM,
    };
    struct addrinfo *address = NULL;
    int result = getaddrinfo(CONFIG_GRM_RECORDER_HOST, port, &hints,
                             &address);
    if (result != 0 || address == NULL) {
        ESP_LOGW(TAG, "解析上位机地址失败: %d", result);
        return ESP_FAIL;
    }

    int socket_fd = socket(address->ai_family, address->ai_socktype,
                           address->ai_protocol);
    if (socket_fd < 0) {
        freeaddrinfo(address);
        return ESP_FAIL;
    }
    int broadcast = 1;
    (void)setsockopt(socket_fd, SOL_SOCKET, SO_BROADCAST, &broadcast,
                     sizeof(broadcast));

    esp_err_t error = ESP_OK;
    size_t length = strlen(message);
    for (int repeat = 0; repeat < CONFIG_GRM_RECORDER_REPEAT_COUNT; repeat++) {
        int sent = sendto(socket_fd, message, length, 0,
                          address->ai_addr, address->ai_addrlen);
        if (sent != (int)length) {
            error = ESP_FAIL;
        }
        if (repeat + 1 < CONFIG_GRM_RECORDER_REPEAT_COUNT) {
            vTaskDelay(pdMS_TO_TICKS(CONFIG_GRM_RECORDER_REPEAT_INTERVAL_MS));
        }
    }
    close(socket_fd);
    freeaddrinfo(address);
    return error;
}

static void recorder_task(void *argument)
{
    (void)argument;
    recorder_queue_item_t item;
    char message[320];

    for (;;) {
        if (xQueueReceive(s_event_queue, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (!app_wifi_wait_connected(
                pdMS_TO_TICKS(CONFIG_GRM_WIFI_CONNECT_TIMEOUT_MS))) {
            ESP_LOGW(TAG, "Wi-Fi 未连接，录制事件未发送");
            continue;
        }

        int length;
        if (item.action == RECORDER_ACTION_START) {
            const app_recorder_start_t *event = &item.data.start;
            length = snprintf(
                message, sizeof(message),
                "{\"schema\":1,\"type\":\"recording\",\"action\":\"start\","
                "\"run\":%lu,\"task\":\"%s\",\"source_ts_ms\":%lu,"
                "\"target_mm\":%ld,\"target_valid\":%s}\n",
                (unsigned long)event->run, event->task,
                (unsigned long)event->source_timestamp_ms,
                (long)event->target_mm,
                event->target_valid ? "true" : "false");
        } else {
            const app_recorder_stop_t *event = &item.data.stop;
            length = snprintf(
                message, sizeof(message),
                "{\"schema\":1,\"type\":\"recording\",\"action\":\"stop\","
                "\"run\":%lu,\"task\":\"%s\",\"source_ts_ms\":%lu,"
                "\"result\":\"%s\",\"fault\":\"%s\"}\n",
                (unsigned long)event->run, event->task,
                (unsigned long)event->source_timestamp_ms,
                event->result, event->fault);
        }
        if (length <= 0 || length >= (int)sizeof(message)) {
            ESP_LOGW(TAG, "录制事件编码失败");
            continue;
        }

        esp_err_t error = send_udp_json(message);
        if (error == ESP_OK) {
            ESP_LOGI(TAG, "已通知上位机%s录制",
                     item.action == RECORDER_ACTION_START ? "开始" : "结束");
        } else {
            ESP_LOGW(TAG, "录制通知发送失败");
        }
    }
}

esp_err_t app_recorder_init(void)
{
    if (!GRM_RECORDER_ENABLED) {
        ESP_LOGW(TAG, "上位机录制通知已关闭");
        return ESP_OK;
    }
    s_event_queue = xQueueCreate(RECORDER_QUEUE_LENGTH,
                                 sizeof(recorder_queue_item_t));
    if (s_event_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreatePinnedToCore(recorder_task, "recorder_notify",
                                RECORDER_TASK_STACK_SIZE, NULL, 4, NULL,
                                APP_CORE_COMMUNICATION) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "UDP 录制通知已启动: %s:%d",
             CONFIG_GRM_RECORDER_HOST, CONFIG_GRM_RECORDER_PORT);
    return ESP_OK;
}

esp_err_t app_recorder_notify_start(const app_recorder_start_t *event)
{
    if (!GRM_RECORDER_ENABLED || s_event_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (event == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    recorder_queue_item_t item = {
        .action = RECORDER_ACTION_START,
        .data.start = *event,
    };
    return xQueueSend(s_event_queue, &item, 0) == pdTRUE
               ? ESP_OK
               : ESP_ERR_TIMEOUT;
}

esp_err_t app_recorder_notify_stop(const app_recorder_stop_t *event)
{
    if (!GRM_RECORDER_ENABLED || s_event_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (event == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    recorder_queue_item_t item = {
        .action = RECORDER_ACTION_STOP,
        .data.stop = *event,
    };
    return xQueueSend(s_event_queue, &item, 0) == pdTRUE
               ? ESP_OK
               : ESP_ERR_TIMEOUT;
}
