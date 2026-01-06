#include "udp.hpp"
#include "main.hpp"

static const char* TAG = "UDP";
struct sockaddr_in dest_addr;
int sock = -1;

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
    if (sock < 0) {
        ESP_LOGE(TAG, "Socket not ready");
        return;
    }

    int err = sendto(sock, buf, len, 0, (struct sockaddr*) &dest_addr, sizeof(dest_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        if (errno == ENOTCONN || errno == EBADF) {
            close(sock);
            sock = -1;
        }
    } else {
        // ESP_LOGI(TAG, "Message sent %d bytes", len);
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
            crsf.ProcessPacket((uint8_t*) rx_buffer, len);
            ESP_LOG_BUFFER_HEX(TAG, rx_buffer, len);
        }
    }
    vTaskDelete(NULL);
}

void crsf_udp_init(void) {
    xTaskCreate(crsf_udp_recv_task, "udp_recv", 4096, NULL, 5, NULL);
}
