#include "udp.hpp"
#include "main.hpp"

static const char* TAG = "UDP";
struct sockaddr_in dest_addr;
int sock = -1;

#define UDP_TX_QUEUE_LEN 20
#define UDP_TX_MAX_PAYLOAD 64 

typedef struct {
    uint8_t data[UDP_TX_MAX_PAYLOAD];
    uint16_t len;
} udp_tx_item_t;

static QueueHandle_t udp_tx_queue = NULL;

int create_socket(void) {
    if (sock != -1) {
        close(sock);
    }

    dest_addr.sin_addr.s_addr = inet_addr(CONFIG_CRSF_SERVER_HOST);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(CONFIG_CRSF_SERVER_PORT);

    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return -1;
    }

    struct timeval timeout;
    timeout.tv_sec = 3;
    timeout.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(UDP_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(sock, (struct sockaddr*) &addr, sizeof(addr)) < 0) {
        ESP_LOGE("UDP", "bind failed");
        close(sock);
        vTaskDelete(NULL);
        return -1;
    }

    ESP_LOGI(TAG,
             "Socket created, connected to %s:%d",
             CONFIG_CRSF_SERVER_HOST,
             CONFIG_CRSF_SERVER_PORT);
    return 0;
}

void crsf_udp_send(const uint8_t* buf, const uint16_t len) {
    if (len > UDP_TX_MAX_PAYLOAD) {
        ESP_LOGW(TAG, "Packet too large for queue");
        return;
    }
    
    if (udp_tx_queue == NULL) return;

    udp_tx_item_t item;
    memcpy(item.data, buf, len);
    item.len = len;

    // Push to queue, do not wait if full (0 ticks) to prevent blocking main loop
    if (xQueueSend(udp_tx_queue, &item, 0) != pdTRUE) {
        ESP_LOGW(TAG, "UDP TX queue full, dropping packet");
    }
}

void crsf_udp_recv_task(void* pvParameters) {
    char rx_buffer[UDP_RX_BUF_SIZE];
    struct sockaddr_storage source_addr;
    socklen_t socklen = sizeof(source_addr);

    while (1) {
        if (sock < 0) {
            if (create_socket() != 0) {
                ESP_LOGE(TAG, "Failed to create socket, retrying in 2s...");
                ((CRSF_ParamInfo*) crsf.GetParam("Is Server Connected"))->valueStr = "Disconnected";
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                continue;
            }
        } else
            ((CRSF_ParamInfo*) crsf.GetParam("Is Server Connected"))->valueStr = "Connected";

        int len = recvfrom(sock,
                           rx_buffer,
                           sizeof(rx_buffer),
                           0,
                           (struct sockaddr*) &source_addr,
                           &socklen);

        if (len < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // 超时, 继续循环
                continue;
            } else {
                // 真正的错误
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                // 遇到严重错误，关闭 Socket，下一次循环会自动重建
                shutdown(sock, 0);
                close(sock);
                sock = -1;
            }
        } else if (len > 0) {
            crsf.txFunc((uint8_t*) rx_buffer, len);
            ESP_LOG_BUFFER_HEX(TAG, rx_buffer, len);
        }
    }
    vTaskDelete(NULL);
}

void crsf_udp_send_task(void* pvParameters) {
    udp_tx_item_t item;

    while (1) {
        // Block indefinitely until item received
        if (xQueueReceive(udp_tx_queue, &item, portMAX_DELAY) == pdTRUE) {
            
            // Only send if socket is valid
            if (sock >= 0) {
                int err = sendto(sock, item.data, item.len, 0, (struct sockaddr*) &dest_addr, sizeof(dest_addr));
                
                if (err < 0) {
                    ESP_LOGE(TAG, "sendto failed: errno %d", errno);
                    if (errno == ENOTCONN || errno == EBADF) {
                        close(sock);
                        sock = -1;
                    }
                }
            }
        }
    }
    vTaskDelete(NULL);
}

void crsf_udp_init(void) {
    udp_tx_queue = xQueueCreate(UDP_TX_QUEUE_LEN, sizeof(udp_tx_item_t));
    if (udp_tx_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create TX queue");
        return;
    }
    xTaskCreate(crsf_udp_recv_task, "udp_recv", 4096, NULL, 5, NULL);
    xTaskCreate(crsf_udp_send_task, "udp_send", 4096, NULL, 5, NULL);
}
