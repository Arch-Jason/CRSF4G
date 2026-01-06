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
    uart_config.parity    = UART_PARITY_DISABLE;
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

    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, RX_BUF_SIZE * 2, 10, &uart0_queue, 0);
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
    return pRoot;
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
}