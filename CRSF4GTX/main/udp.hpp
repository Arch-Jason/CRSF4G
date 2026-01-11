#pragma once

extern "C" {
#include "sdkconfig.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "esp_log.h"
#include "freertos/queue.h"
}

#define UDP_RX_BUF_SIZE 256
#define UDP_PORT 3000

#ifdef __cplusplus
extern "C" {
#endif

void crsf_udp_init(void);
void crsf_udp_send(const uint8_t* buf, const uint16_t len);
void crsf_udp_recv_task(void *pvParameters);

#ifdef __cplusplus
}
#endif