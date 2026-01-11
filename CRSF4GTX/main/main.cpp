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
#include <string>
}

static QueueHandle_t uart0_queue;
#define RX_BUF_SIZE 1024
#define TASK_STACK_SIZE 4096

void sendData(uint8_t* buf, uint16_t len) {
    uart_write_bytes(UART_NUM_1, buf, len);
}

CRSF_TxModule crsf(sendData);

#define CRSF_UART_PIN GPIO_NUM_4

void uart_init(void) {
    uart_config_t uart_config{};
    uart_config.baud_rate = 400000;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    uart_param_config(UART_NUM_1, &uart_config);

    uart_set_pin(UART_NUM_1,
                 CRSF_UART_PIN, // TXD
                 CRSF_UART_PIN, // RXD (same as TXD)
                 UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE);

    uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_TXD_INV | UART_SIGNAL_RXD_INV);
    uart_set_mode(UART_NUM_1, UART_MODE_RS485_HALF_DUPLEX);
    gpio_set_pull_mode(CRSF_UART_PIN, GPIO_PULLDOWN_ONLY);

    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, RX_BUF_SIZE * 2, 100, &uart0_queue, 0);
    uart_set_rx_timeout(UART_NUM_1, 10);
}

void uart_event_task(void* pvParameters) {
    uart_event_t event;
    uint8_t* buf = (uint8_t*) malloc(RX_BUF_SIZE);

    while (1) {
        if (xQueueReceive(uart0_queue, (void*) &event, (TickType_t) portMAX_DELAY)) {
            switch (event.type) {
            case UART_DATA:
                uart_read_bytes(UART_NUM_1, buf, event.size, portMAX_DELAY);
                crsf.ProcessPacket(buf, event.size);
                break;

            // 缓冲区溢出
            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
                uart_flush_input(UART_NUM_1);
                xQueueReset(uart0_queue);
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

// Process Delay & Generate CRSF 0x14 Packet
// Maps latency to fake RSSI/LQ and injects to RC
void process_delay(uint64_t delay) {
    // Map Delay to Signal Quality (Common Sense)
    // < 50ms : Perfect (-60dBm, LQ 100)
    // < 150ms: Good (-80dBm, LQ 95)
    // < 300ms: Fair (-95dBm, LQ 80)
    // > 500ms: Bad (-105dBm, LQ 50)

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

    // Construct CRSF Link Statistics Packet (Type 0x14)
    // Payload length is 10 bytes
    uint8_t frame[14];
    frame[0] = CRSF_SYNC_BYTE;                 // CRSF Sync Byte (Module -> Handset)
    frame[1] = 12;                             // Length (Type + Payload + CRC)
    frame[2] = CRSF_FRAMETYPE_LINK_STATISTICS; // Type

    // Payload
    frame[3] = uplink_rssi;  // Uplink RSSI 1 (Ant 1)
    frame[4] = uplink_rssi;  // Uplink RSSI 2 (Ant 2)
    frame[5] = uplink_lq;    // Uplink Link Quality
    frame[6] = 0;            // Uplink SNR (0 is generic)
    frame[7] = 1;            // Diversity active antenna
    frame[8] = 4;            // RF Mode (4 = 150Hz or similar enum)
    frame[9] = 3;            // TX Power (3 = 100mW enum)
    frame[10] = uplink_rssi; // Downlink RSSI (Simulated same as Up)
    frame[11] = uplink_lq;   // Downlink LQ
    frame[12] = 0;           // Downlink SNR

    // CRC
    frame[13] = crsf.CalcCRC8(&frame[2], 11); // CRC covers Type + Payload

    // Send to Remote Controller (RC)
    // Assuming UART_NUM_1 is connected to RC in your TX code
    crsf.txFunc((uint8_t*) frame, sizeof(frame));

    // printf("Delay: %lu ms -> RSSI: %d, LQ: %d\n", current_latency_ms, uplink_rssi, uplink_lq);
}

uint64_t delay;
void latency_update_task(void* arg) {
    uint16_t no_signal_cnt = 0;
    static char latency_buf[16];
    while (1) {
        if (current_latency_ms == 0) {
            if (no_signal_cnt < 100)
                no_signal_cnt++;
            else {
                delay = current_latency_ms;
                return;
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
    auto pRoot = crsf_add_basic_param();
    wifi_init_sta();
    uart_init();
    xTaskCreate(uart_event_task,
                "uart_rx_task",
                TASK_STACK_SIZE,
                NULL,
                configMAX_PRIORITIES - 1,
                NULL);
    while (!wifi_connect())
        ; // wait until wifi is connected
    ((CRSF_ParamInfo*) crsf.GetParam("Is WiFi Connected"))->valueStr = "Connected";

    crsf_udp_init();
    xTaskCreate(ping_task, "ping_task", 3072, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(latency_update_task,
                "latency_update_task",
                3072,
                NULL,
                configMAX_PRIORITIES - 1,
                NULL);
}