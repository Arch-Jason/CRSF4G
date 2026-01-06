#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define RX_BUF_SIZE 1024 
#define TX_BUF_SIZE 0

#define LTE_RX_PIN GPIO_NUM_0
#define LTE_TX_PIN GPIO_NUM_1

#define CRSF_TX UART_PIN_NO_CHANGE
#define CRSF_RX UART_PIN_NO_CHANGE

#define HEARTBEAT_INTERVAL_MS 10000

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
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, RX_BUF_SIZE * 2, 0, NULL, 0));
}

void uart0_init(void) {
    uart_config_t uart_config = {};
    uart_config.baud_rate = 115200;
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
void task_forward_lte_to_crsf(void *arg) {
    uint8_t *data = (uint8_t *) malloc(RX_BUF_SIZE);
    while (1) {
        int len = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            uart_write_bytes(UART_NUM_0, (const char *)data, len);
        }
    }
    free(data);
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