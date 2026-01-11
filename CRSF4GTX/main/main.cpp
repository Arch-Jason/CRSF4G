#include "main.hpp"

extern "C" {
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "udp.hpp"
#include "wifi.hpp"
#include <cstring>
#include <string>
}

static QueueHandle_t uart0_queue;
static QueueHandle_t tx_queue;

#define RX_BUF_SIZE 1024
#define TASK_STACK_SIZE 4096
#define CRSF_UART_PIN GPIO_NUM_4

struct TxPacket {
    uint8_t buf[256];
    uint16_t len;
};

void sendData(uint8_t* buf, uint16_t len) {
    if (len > 256) return;
    TxPacket pkt;
    memcpy(pkt.buf, buf, len);
    pkt.len = len;
    // 使用 0 延迟，队列满则丢弃，防止阻塞 UDP 任务
    xQueueSend(tx_queue, &pkt, 0); 
}

CRSF_TxModule crsf(sendData);

void uart_init(void) {
    uart_config_t uart_config{};
    uart_config.baud_rate = 400000;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, RX_BUF_SIZE * 2, 20, &uart0_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1,
                 CRSF_UART_PIN, // TXD
                 CRSF_UART_PIN, // RXD (单线半双工)
                 UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_TXD_INV | UART_SIGNAL_RXD_INV));
    ESP_ERROR_CHECK(uart_set_mode(UART_NUM_1, UART_MODE_RS485_HALF_DUPLEX));
    ESP_ERROR_CHECK(uart_set_rx_timeout(UART_NUM_1, 10));
    gpio_set_pull_mode(CRSF_UART_PIN, GPIO_PULLDOWN_ONLY);
}

void uart_event_task(void* pvParameters) {
    uart_event_t event;
    uint8_t* rx_buf = (uint8_t*) malloc(RX_BUF_SIZE);
    TxPacket tx_pkt;

    while (1) {
        bool did_work = false;

        // 1. 处理 RX 事件
        if (xQueueReceive(uart0_queue, (void*) &event, 2)) {
            did_work = true;
            switch (event.type) {
            case UART_DATA:
                uart_read_bytes(UART_NUM_1, rx_buf, event.size, portMAX_DELAY);
                crsf.ProcessPacket(rx_buf, event.size);
                break;
            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
                uart_flush_input(UART_NUM_1);
                xQueueReset(uart0_queue);
                break;
            default:
                break;
            }
        }

        // 2. 处理 TX 队列
        int tx_count = 0;
        while (xQueueReceive(tx_queue, &tx_pkt, 0)) {
            uart_write_bytes(UART_NUM_1, tx_pkt.buf, tx_pkt.len);
            did_work = true;
            tx_count++;
            if(tx_count > 5) break; // 每次最多发5包，必须让出 CPU 检查 RX
        }

        // 3. 如果既没有 RX 也没有 TX，强制 Yield 一下，防止某些边界情况死循环
        if (!did_work) {
            taskYIELD(); 
        }
    }
    free(rx_buf);
    vTaskDelete(NULL);
}

static CRSF_ParamFolder crsf_add_basic_param() {
    static uint8_t rootChildren[] = {1};
    static CRSF_ParamFolder pRoot(0, 0, "ROOT", rootChildren, 1);
    crsf.AddParam(&pRoot);
    static CRSF_ParamInfo info(0, pRoot.id, "CRSF 4G v1.0", "by Arch-Jason");
    crsf.AddParam(&info);
    static CRSF_ParamInfo UDPAddr(0, pRoot.id, "UDP Address", CONFIG_CRSF_SERVER_HOST);
    crsf.AddParam(&UDPAddr);
    static CRSF_ParamFloat UDPPort(0,
                                   pRoot.id,
                                   "UDP Port",
                                   CONFIG_CRSF_SERVER_PORT,
                                   CONFIG_CRSF_SERVER_PORT,
                                   CONFIG_CRSF_SERVER_PORT,
                                   0,
                                   1,
                                   "");
    crsf.AddParam(&UDPPort);
    static CRSF_ParamInfo wifi_info(0, pRoot.id, "Is WiFi Connected", "Disconnected");
    crsf.AddParam(&wifi_info);
    static CRSF_ParamInfo server_info(0, pRoot.id, "Is Server Connected", "Disconnected");
    crsf.AddParam(&server_info);
    static CRSF_ParamInfo latency_ms(0, pRoot.id, "Latency (ms)", "NaN");
    crsf.AddParam(&latency_ms);
    return pRoot;
}

void ping_task(void* arg) {
    while (1) {
        crsf.SendPing();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void process_delay(uint64_t delay) {
    int8_t uplink_rssi = -110;
    uint8_t uplink_lq = 0;

    if (delay < 50) {
        uplink_rssi = -60;
        uplink_lq = 99;
    } else if (delay < 150) {
        uplink_rssi = -80;
        uplink_lq = 95;
    } else if (delay < 300) {
        uplink_rssi = -95;
        uplink_lq = 80;
    } else if (delay < 500) {
        uplink_rssi = -100;
        uplink_lq = 60;
    } else {
        uplink_rssi = -115;
        uplink_lq = 20;
    }

    uint8_t frame[14];
    frame[0] = CRSF_SYNC_BYTE;
    frame[1] = 12;
    frame[2] = CRSF_FRAMETYPE_LINK_STATISTICS;
    frame[3] = uplink_rssi;
    frame[4] = uplink_rssi;
    frame[5] = uplink_lq;
    frame[6] = 0;
    frame[7] = 1;
    frame[8] = 4;
    frame[9] = 3;
    frame[10] = uplink_rssi;
    frame[11] = uplink_lq;
    frame[12] = 0;
    frame[13] = crsf.CalcCRC8(&frame[2], 11);

    crsf.txFunc((uint8_t*) frame, sizeof(frame));
}

uint64_t delay;
void latency_update_task(void* arg) {
    uint16_t no_signal_cnt = 100;
    static char latency_buf[16];
    while (1) {
        if (current_latency_ms == 0) {
            if (no_signal_cnt < 100)
                no_signal_cnt++;
            else {
                delay = current_latency_ms;
                vTaskDelay(100 / portTICK_PERIOD_MS);
                continue;
            }
        } else {
            delay = current_latency_ms;
            no_signal_cnt = 0;
        }

        process_delay(delay);
        snprintf(latency_buf, sizeof(latency_buf), "%llu ms", delay);
        ((CRSF_ParamInfo*) crsf.GetParam("Latency (ms)"))->valueStr = latency_buf;
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

extern "C" void app_main(void) {
    tx_queue = xQueueCreate(20, sizeof(TxPacket));

    auto pRoot = crsf_add_basic_param();
    wifi_init_sta();
    
    // 初始化 UART
    uart_init();
    
    xTaskCreate(uart_event_task,
                "uart_rw_task",
                TASK_STACK_SIZE,
                NULL,
                configMAX_PRIORITIES - 1,
                NULL);
    while (!wifi_connect())
        vTaskDelay(100 / portTICK_PERIOD_MS);
    ((CRSF_ParamInfo*) crsf.GetParam("Is WiFi Connected"))->valueStr = "Connected";

    crsf_udp_init();

    xTaskCreate(ping_task, "ping_task", TASK_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(latency_update_task,
                "latency_update_task",
                TASK_STACK_SIZE,
                NULL,
                configMAX_PRIORITIES - 1,
                NULL);
}
