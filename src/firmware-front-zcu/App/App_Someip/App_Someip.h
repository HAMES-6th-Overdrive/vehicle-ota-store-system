#ifndef __APP_SOMEIP_H__
#define __APP_SOMEIP_H__


#include <stdint.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "someip/light_someip.h"


typedef enum AppSomeipTxType {
    APP_SOMEIP_TX_REQUEST,
    APP_SOMEIP_TX_RESPONSE,
    APP_SOMEIP_TX_EVENT
} AppSomeipTxType;

typedef struct AppSomeipRxMsg {
    LightSomeipPacket packet;
    char remote_ip[SOMEIP_IP_LEN];
    uint16_t remote_port;
} AppSomeipRxMsg;

typedef struct AppSomeipTxMsg {
    AppSomeipTxType msg_type;
    LightSomeipPacket packet;
    AppSomeipRxMsg req_msg;             /* RESPONSE용 */
    LightSomeipEndpoint dst_endpoint;   /* NOTIFICATION용 */
} AppSomeipTxMsg;

typedef QueueHandle_t (*AppSomeipRxQueueGetter)(void);

typedef struct AppSomeipRoute {
    uint16_t service_id;
    AppSomeipRxQueueGetter get_rx_queue;
} AppSomeipRoute;


/* 초기화 함수 */
BaseType_t AppSomeip_Start(void);

/* Task 함수 */

/* 송수신 함수 */
BaseType_t AppSomeip_Recv(QueueHandle_t rx_queue, AppSomeipRxMsg* rx_msg);
BaseType_t AppSomeip_SendRequest(LightSomeipPacket* request_packet);
BaseType_t AppSomeip_SendResponse(const AppSomeipRxMsg* request_msg, LightSomeipPacket* response_packet);
BaseType_t AppSomeip_SendEvent(LightSomeipPacket* event_packet, const LightSomeipEndpoint* dst_endpoint);


#endif
