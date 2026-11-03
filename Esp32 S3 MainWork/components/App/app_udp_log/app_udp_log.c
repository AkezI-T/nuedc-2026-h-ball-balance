#include "app_udp_log.h"

#include <stdbool.h>
#include <stdint.h>
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

#define UDP_LOG_QUEUE_LENGTH 32U
#define UDP_LOG_LINE_MAX 384U
#define UDP_LOG_TASK_STACK_SIZE 4096U
#define UDP_LOG_TASK_PRIORITY 5U

#if defined(CONFIG_GRM_UDP_LOG_ENABLE) && CONFIG_GRM_UDP_LOG_ENABLE
#define GRM_UDP_LOG_ENABLED true
#else
#define GRM_UDP_LOG_ENABLED false
#endif

typedef struct {
    uint16_t length;
    char data[UDP_LOG_LINE_MAX];
} udp_log_item_t;

static const char *TAG = "app_udp_log";
static QueueHandle_t s_log_queue;

static void udp_log_task(void *argument)
{
    (void)argument;
    udp_log_item_t item;

    (void)app_wifi_wait_connected(portMAX_DELAY);

    char port[8];
    (void)snprintf(port, sizeof(port), "%d", CONFIG_GRM_UDP_LOG_PORT);
    const struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_DGRAM,
    };
    struct addrinfo *address = NULL;
    if (getaddrinfo(CONFIG_GRM_UDP_LOG_HOST, port, &hints, &address) != 0 ||
        address == NULL) {
        ESP_LOGE(TAG, "解析UDP日志目标失败");
        vTaskDelete(NULL);
        return;
    }

    int socket_fd = socket(address->ai_family, address->ai_socktype,
                           address->ai_protocol);
    if (socket_fd < 0) {
        freeaddrinfo(address);
        ESP_LOGE(TAG, "创建UDP日志socket失败");
        vTaskDelete(NULL);
        return;
    }
    int broadcast = 1;
    (void)setsockopt(socket_fd, SOL_SOCKET, SO_BROADCAST, &broadcast,
                     sizeof(broadcast));

    char link_line[96];
    int link_length = snprintf(
        link_line, sizeof(link_line),
        "@LINK type=UDP_LOG state=CONNECTED device=ESP32S3 port=%d\n",
        CONFIG_GRM_UDP_LOG_PORT);
    if (link_length > 0 && link_length < (int)sizeof(link_line)) {
        int sent_length = sendto(socket_fd, link_line, (size_t)link_length, 0,
                                 address->ai_addr, address->ai_addrlen);
        if (sent_length == link_length) {
            ESP_LOGI(TAG, "已向上位机发送UDP日志链路成功提示");
        } else {
            ESP_LOGW(TAG, "发送UDP日志链路成功提示失败");
        }
    }

    for (;;) {
        if (xQueueReceive(s_log_queue, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        (void)sendto(socket_fd, item.data, item.length, 0,
                     address->ai_addr, address->ai_addrlen);
    }
}

esp_err_t app_udp_log_init(void)
{
    if (!GRM_UDP_LOG_ENABLED) {
        ESP_LOGW(TAG, "上位机UDP日志已关闭");
        return ESP_OK;
    }

    s_log_queue = xQueueCreate(UDP_LOG_QUEUE_LENGTH, sizeof(udp_log_item_t));
    if (s_log_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreatePinnedToCore(udp_log_task, "udp_log_tx",
                                UDP_LOG_TASK_STACK_SIZE, NULL,
                                UDP_LOG_TASK_PRIORITY, NULL,
                                APP_CORE_COMMUNICATION) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "UDP日志广播已启动: %s:%d",
             CONFIG_GRM_UDP_LOG_HOST, CONFIG_GRM_UDP_LOG_PORT);
    return ESP_OK;
}

esp_err_t app_udp_log_publish(const char *line, size_t length)
{
    if (!GRM_UDP_LOG_ENABLED || s_log_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (line == NULL || length == 0U) {
        return ESP_ERR_INVALID_ARG;
    }
    while (length > 0U &&
           (line[length - 1U] == '\r' || line[length - 1U] == '\n')) {
        length--;
    }
    if (length == 0U || length >= UDP_LOG_LINE_MAX) {
        return ESP_ERR_INVALID_SIZE;
    }

    udp_log_item_t item = {
        .length = (uint16_t)(length + 1U),
    };
    memcpy(item.data, line, length);
    item.data[length] = '\n';
    return xQueueSend(s_log_queue, &item, 0) == pdTRUE ?
        ESP_OK : ESP_ERR_TIMEOUT;
}
