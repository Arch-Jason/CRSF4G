#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#define RX_BUF_SIZE 1024 
#define TX_BUF_SIZE 0

#define LTE_RX_PIN GPIO_NUM_0
#define LTE_TX_PIN GPIO_NUM_1

#define CRSF_TX UART_PIN_NO_CHANGE
#define CRSF_RX UART_PIN_NO_CHANGE

#define HEARTBEAT_INTERVAL_MS 5000

// Custom protocol definition
#define TIME_SYNC_MAGIC 0xFA 
#define PACKET_TYPE_PING 0x01
#define PACKET_TYPE_PONG 0x02

static QueueHandle_t uart1_queue;

void (*txFunc)(uint8_t*, uint16_t);
typedef struct __attribute__((packed)) {
    uint8_t magic;      // 0xFA
    uint8_t type;       // 0x01 or 0x02
    int64_t timestamp;  // Microseconds
} time_packet_t;

// Get time in microseconds
int64_t get_time() {
    return esp_timer_get_time();
}

// Send Delay (Send Pong)
void send_delay(uint8_t* old_packet_data) {
    time_packet_t pkt;
    memcpy(&pkt, old_packet_data, sizeof(time_packet_t));
    
    pkt.type = PACKET_TYPE_PONG; // Type Pong

    // Send back to LTE
    uart_write_bytes(UART_NUM_1, (const char*)&pkt, sizeof(pkt));
}

// Returns 1 if it was a Time Packet and handled, 0 if it's CRSF data
void process_ping_and_reply(uint8_t* data, int len) {
    // 遍历 buffer，防止粘包导致漏掉 Ping
    for (int i = 0; i <= len - (int)sizeof(time_packet_t); i++) {
        // 查找 Magic 头 (0xFA)
        if (data[i] == TIME_SYNC_MAGIC) {
            time_packet_t* pkt = (time_packet_t*)&data[i];
            
            // 再次确认类型，防止误判
            if (pkt->type == PACKET_TYPE_PING) {
                // 找到 Ping 包，立即回复 Pong
                send_delay((uint8_t*)pkt);
                
                i += (sizeof(time_packet_t) - 1); 
            }
        }
    }
}

// LTE
void uart1_init(void) {
    uart_config_t uart_config = {};
    uart_config.baud_rate = 115200;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity    = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, LTE_TX_PIN, LTE_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, RX_BUF_SIZE * 2, 100, &uart1_queue, 0));
}

void uart0_init(void) {
    uart_config_t uart_config = {};
    uart_config.baud_rate = 420000;
    // uart_config.baud_rate = 115200;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity    = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, CRSF_TX, CRSF_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, RX_BUF_SIZE * 2, RX_BUF_SIZE * 2, 0, NULL, 0));
}

void LTE_init(void) {
    printf("Initializing LTE...\n");

    auto send_cmd = [](const char* cmd, int delay_ms) {
        uart_write_bytes(UART_NUM_1, cmd, strlen(cmd));
        vTaskDelay(delay_ms / portTICK_PERIOD_MS);
    };

    vTaskDelay(3000 / portTICK_PERIOD_MS);
    send_cmd("AT+QICSGP=1,1,\"CMIOT\",\"\",\"\",1\r\n", 3000);
    send_cmd("AT+QIACT=1\r\n", 3000);
    send_cmd("AT+QIOPEN=1,0,\"UDP\",\"36.150.231.230\",4000,0,2\r\n", 3000);
    
    printf("LTE Init Done.\n");
}

// LTE -> CRSF
void task_forward_lte_to_crsf(void* pvParameters) {
    uart_event_t event;
    uint8_t* buf = (uint8_t*) malloc(RX_BUF_SIZE);

    while (1) {
        if (xQueueReceive(uart1_queue, (void*) &event, (TickType_t) portMAX_DELAY)) {
            switch (event.type) {
            case UART_DATA:
                uart_read_bytes(UART_NUM_1, buf, event.size, portMAX_DELAY);
                if (event.size > 0) {
                    // 1. 先尝试在数据流中搜索 Ping 包并回复
                    process_ping_and_reply(buf, event.size);

                    // 2. 将数据无条件转发给飞控 (CRSF)
                    uart_write_bytes(UART_NUM_0, (const char *)buf, event.size);
                }
                break;

            // 缓冲区溢出
            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
                uart_flush_input(UART_NUM_1);
                xQueueReset(uart1_queue);
                break;

            case UART_BREAK:
            case UART_PARITY_ERR:
            case UART_FRAME_ERR:
            case UART_PATTERN_DET:
            default:
                break;
            }
        }
    }
    free(buf);
    vTaskDelete(NULL);
}

// CRSF -> LTE
void task_forward_crsf_to_lte(void *arg) {
    uint8_t *data = (uint8_t *) malloc(RX_BUF_SIZE);
    while (1) {
        int len = uart_read_bytes(UART_NUM_0, data, RX_BUF_SIZE, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            uart_write_bytes(UART_NUM_1, (const char *)data, len);
        }
    }
    free(data);
    vTaskDelete(NULL);
}

// Heartbeat
void task_heartbeat(void *arg) {
    const char* heartbeat_data = "HEARTBEAT_PACKET";
    
    while (1) {
        vTaskDelay(HEARTBEAT_INTERVAL_MS / portTICK_PERIOD_MS);

        uart_write_bytes(UART_NUM_1, heartbeat_data, strlen(heartbeat_data));

        // const char* log = "\r\n[ESP32] Heartbeat sent\r\n";
        // uart_write_bytes(UART_NUM_0, log, strlen(log));
    }
    vTaskDelete(NULL);
}

extern "C" void app_main(void) {
    uart0_init(); // CRSF
    uart1_init(); // LTE

    LTE_init();

    xTaskCreate(task_forward_lte_to_crsf, "lte_to_crsf", 4096, NULL, 10, NULL);
    xTaskCreate(task_forward_crsf_to_lte, "crsf_to_lte", 4096, NULL, 10, NULL);
    xTaskCreate(task_heartbeat,         "heartbeat", 2048, NULL, 5,  NULL);
    
    printf("Bridge & Heartbeat Started.\n");
}