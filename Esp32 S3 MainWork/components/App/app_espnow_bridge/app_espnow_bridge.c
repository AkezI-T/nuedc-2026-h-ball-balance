#include "app_espnow_bridge.h"

#include <string.h>

#include "app_core_affinity.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#define PROTOCOL_MAGIC 0x53504E57U
#define PROTOCOL_VERSION 0x01U
#define PROTOCOL_DISCOVERY_LENGTH 15U
#define PROTOCOL_UART_HEADER_LENGTH 10U
#define PROTOCOL_UART_PAYLOAD_MAX 200U

#define MSG_DISCOVER_REQ 0x01U
#define MSG_SENDER_ONLINE 0x02U
#define MSG_RECEIVER_ONLINE 0x03U
#define MSG_DISCOVER_ACK 0x04U
#define MSG_UART_DATA 0x05U
#define ROLE_SENDER 0x01U

#define RX_QUEUE_LENGTH 8U
#define TX_QUEUE_LENGTH 12U
#define RELAY_DATA_MAX 384U
#define RX_TASK_STACK_SIZE 4096U
#define TX_TASK_STACK_SIZE 4096U
#define BEACON_TASK_STACK_SIZE 3072U

#if defined(CONFIG_GRM_ESPNOW_BRIDGE_ENABLE) && \
    CONFIG_GRM_ESPNOW_BRIDGE_ENABLE
#define GRM_ESPNOW_BRIDGE_ENABLED true
#else
#define GRM_ESPNOW_BRIDGE_ENABLED false
#endif

typedef struct {
    uint8_t source_mac[6];
    int length;
    uint8_t data[PROTOCOL_DISCOVERY_LENGTH];
} discovery_rx_item_t;

typedef struct {
    size_t length;
    uint8_t data[RELAY_DATA_MAX];
} relay_tx_item_t;

static const char *TAG = "app_espnow";
static const uint8_t s_broadcast_mac[6] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};
static uint8_t s_self_mac[6];
static uint8_t s_receiver_mac[6];
static bool s_receiver_found;
static SemaphoreHandle_t s_peer_mutex;
static QueueHandle_t s_rx_queue;
static QueueHandle_t s_tx_queue;
static uint16_t s_uart_sequence = 1U;

static void put_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xffU);
    data[1] = (uint8_t)(value >> 8);
}

static void put_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xffU);
    data[1] = (uint8_t)((value >> 8) & 0xffU);
    data[2] = (uint8_t)((value >> 16) & 0xffU);
    data[3] = (uint8_t)((value >> 24) & 0xffU);
}

static uint16_t get_u16_le(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static uint32_t get_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static size_t build_discovery(uint8_t *data, uint8_t type, uint16_t sequence)
{
    put_u32_le(data, PROTOCOL_MAGIC);
    data[4] = PROTOCOL_VERSION;
    data[5] = type;
    data[6] = ROLE_SENDER;
    put_u16_le(data + 7, sequence);
    memcpy(data + 9, s_self_mac, sizeof(s_self_mac));
    return PROTOCOL_DISCOVERY_LENGTH;
}

static bool parse_discovery(const uint8_t *data, int length,
                            uint8_t *type, uint16_t *sequence)
{
    if (length != (int)PROTOCOL_DISCOVERY_LENGTH ||
        get_u32_le(data) != PROTOCOL_MAGIC ||
        data[4] != PROTOCOL_VERSION) {
        return false;
    }
    if (data[5] != MSG_RECEIVER_ONLINE && data[5] != MSG_DISCOVER_ACK) {
        return false;
    }
    *type = data[5];
    *sequence = get_u16_le(data + 7);
    return true;
}

static esp_err_t ensure_peer(const uint8_t *mac)
{
    if (esp_now_is_peer_exist(mac)) {
        return ESP_OK;
    }
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, mac, ESP_NOW_ETH_ALEN);
    peer.channel = 0;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    esp_err_t error = esp_now_add_peer(&peer);
    return error == ESP_ERR_ESPNOW_EXIST ? ESP_OK : error;
}

static void send_discovery_to(const uint8_t *destination, uint8_t type,
                              uint16_t sequence)
{
    uint8_t data[PROTOCOL_DISCOVERY_LENGTH];
    build_discovery(data, type, sequence);
    esp_err_t error = esp_now_send(destination, data, sizeof(data));
    if (error != ESP_OK) {
        ESP_LOGW(TAG, "发送发现报文失败: %s", esp_err_to_name(error));
    }
}

static void receive_callback(const esp_now_recv_info_t *info,
                             const uint8_t *data, int data_length)
{
    if (info == NULL || info->src_addr == NULL || data == NULL ||
        data_length != (int)PROTOCOL_DISCOVERY_LENGTH) {
        return;
    }
    discovery_rx_item_t item = {.length = data_length};
    memcpy(item.source_mac, info->src_addr, ESP_NOW_ETH_ALEN);
    memcpy(item.data, data, (size_t)data_length);
    (void)xQueueSend(s_rx_queue, &item, 0);
}

static void send_callback(const uint8_t *mac,
                          esp_now_send_status_t status)
{
    (void)mac;
    if (status != ESP_NOW_SEND_SUCCESS) {
        ESP_EARLY_LOGW(TAG, "ESP-NOW 发送未确认");
    }
}

static void discovery_rx_task(void *argument)
{
    (void)argument;
    discovery_rx_item_t item;

    for (;;) {
        if (xQueueReceive(s_rx_queue, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        uint8_t type;
        uint16_t sequence;
        if (!parse_discovery(item.data, item.length, &type, &sequence)) {
            continue;
        }
        esp_err_t error = ensure_peer(item.source_mac);
        if (error != ESP_OK) {
            ESP_LOGW(TAG, "添加接收端 Peer 失败: %s", esp_err_to_name(error));
            continue;
        }

        bool just_found = false;
        xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
        if (!s_receiver_found ||
            memcmp(s_receiver_mac, item.source_mac, ESP_NOW_ETH_ALEN) != 0) {
            memcpy(s_receiver_mac, item.source_mac, ESP_NOW_ETH_ALEN);
            s_receiver_found = true;
            just_found = true;
        }
        xSemaphoreGive(s_peer_mutex);

        if (just_found) {
            ESP_LOGI(TAG,
                     "ESP-NOW 接收端已发现: %02x:%02x:%02x:%02x:%02x:%02x",
                     item.source_mac[0], item.source_mac[1],
                     item.source_mac[2], item.source_mac[3],
                     item.source_mac[4], item.source_mac[5]);
        }
        if (type == MSG_RECEIVER_ONLINE) {
            send_discovery_to(item.source_mac, MSG_DISCOVER_REQ, sequence);
        }
    }
}

static esp_err_t send_uart_chunk(const uint8_t *destination,
                                 const uint8_t *payload, size_t length)
{
    uint8_t packet[PROTOCOL_UART_HEADER_LENGTH + PROTOCOL_UART_PAYLOAD_MAX];
    put_u32_le(packet, PROTOCOL_MAGIC);
    packet[4] = PROTOCOL_VERSION;
    packet[5] = MSG_UART_DATA;
    put_u16_le(packet + 6, s_uart_sequence++);
    put_u16_le(packet + 8, (uint16_t)length);
    memcpy(packet + PROTOCOL_UART_HEADER_LENGTH, payload, length);

    esp_err_t error = ESP_FAIL;
    for (int attempt = 0; attempt < 4; attempt++) {
        error = esp_now_send(destination, packet,
                             PROTOCOL_UART_HEADER_LENGTH + length);
        if (error == ESP_OK) {
            return ESP_OK;
        }
        if (error != ESP_ERR_ESPNOW_NO_MEM) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return error;
}

static void relay_tx_task(void *argument)
{
    (void)argument;
    relay_tx_item_t item;

    for (;;) {
        if (xQueueReceive(s_tx_queue, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        uint8_t destination[ESP_NOW_ETH_ALEN];
        xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
        bool found = s_receiver_found;
        memcpy(destination, s_receiver_mac, sizeof(destination));
        xSemaphoreGive(s_peer_mutex);
        if (!found) {
            ESP_LOGW(TAG, "接收端未发现，丢弃一行 TMX 日志");
            continue;
        }

        size_t offset = 0U;
        while (offset < item.length) {
            size_t chunk_length = item.length - offset;
            if (chunk_length > PROTOCOL_UART_PAYLOAD_MAX) {
                chunk_length = PROTOCOL_UART_PAYLOAD_MAX;
            }
            esp_err_t error = send_uart_chunk(destination,
                                              item.data + offset,
                                              chunk_length);
            if (error != ESP_OK) {
                ESP_LOGW(TAG, "转发 TMX 日志失败: %s",
                         esp_err_to_name(error));
                break;
            }
            offset += chunk_length;
            if (offset < item.length) {
                vTaskDelay(pdMS_TO_TICKS(5));
            }
        }
    }
}

static void beacon_task(void *argument)
{
    (void)argument;
    uint16_t sequence = 1U;
    for (;;) {
        send_discovery_to(s_broadcast_mac, MSG_SENDER_ONLINE, sequence++);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

esp_err_t app_espnow_bridge_init(void)
{
    if (!GRM_ESPNOW_BRIDGE_ENABLED) {
        ESP_LOGW(TAG, "ESP-NOW 日志转发已关闭");
        return ESP_OK;
    }

    s_peer_mutex = xSemaphoreCreateMutex();
    s_rx_queue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(discovery_rx_item_t));
    s_tx_queue = xQueueCreate(TX_QUEUE_LENGTH, sizeof(relay_tx_item_t));
    if (s_peer_mutex == NULL || s_rx_queue == NULL || s_tx_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    ESP_RETURN_ON_ERROR(esp_wifi_get_mac(WIFI_IF_STA, s_self_mac), TAG,
                        "读取本机 MAC 失败");
    ESP_RETURN_ON_ERROR(esp_now_init(), TAG, "初始化 ESP-NOW 失败");
    ESP_RETURN_ON_ERROR(esp_now_register_recv_cb(receive_callback), TAG,
                        "注册 ESP-NOW 接收回调失败");
    ESP_RETURN_ON_ERROR(esp_now_register_send_cb(send_callback), TAG,
                        "注册 ESP-NOW 发送回调失败");
    ESP_RETURN_ON_ERROR(ensure_peer(s_broadcast_mac), TAG,
                        "添加广播 Peer 失败");

    if (xTaskCreatePinnedToCore(discovery_rx_task, "espnow_rx",
                                RX_TASK_STACK_SIZE, NULL, 6, NULL,
                                APP_CORE_COMMUNICATION) != pdPASS ||
        xTaskCreatePinnedToCore(relay_tx_task, "espnow_tx",
                                TX_TASK_STACK_SIZE, NULL, 5, NULL,
                                APP_CORE_COMMUNICATION) != pdPASS ||
        xTaskCreatePinnedToCore(beacon_task, "espnow_beacon",
                                BEACON_TASK_STACK_SIZE, NULL, 4, NULL,
                                APP_CORE_COMMUNICATION) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "ESP-NOW 日志桥已启动，兼容 Esp Tx/Rx 协议 v1");
    return ESP_OK;
}

esp_err_t app_espnow_bridge_send(const uint8_t *data, size_t length,
                                 TickType_t timeout_ticks)
{
    if (!GRM_ESPNOW_BRIDGE_ENABLED) {
        return ESP_ERR_INVALID_STATE;
    }
    if (data == NULL || length == 0U || length > RELAY_DATA_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    relay_tx_item_t item = {.length = length};
    memcpy(item.data, data, length);
    return xQueueSend(s_tx_queue, &item, timeout_ticks) == pdTRUE
               ? ESP_OK
               : ESP_ERR_TIMEOUT;
}

bool app_espnow_bridge_peer_ready(void)
{
    if (s_peer_mutex == NULL) {
        return false;
    }
    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    bool found = s_receiver_found;
    xSemaphoreGive(s_peer_mutex);
    return found;
}
