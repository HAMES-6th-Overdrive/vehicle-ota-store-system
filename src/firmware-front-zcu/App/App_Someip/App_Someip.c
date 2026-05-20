#include "App_Someip.h"
#include "App_Eth/App_Eth.h"
#include "App_InfoService/App_InfoService.h"
#include "task.h"
#include <string.h>

#define APP_SOMEIP_TASK_STACK_SIZE  (1024u)
#define APP_SOMEIP_TASK_PRIORITY    (tskIDLE_PRIORITY + 3u)

#define APP_SOMEIP_TX_QUEUE_SIZE    (8u)
#define APP_SOMEIP_RX_DRAIN_LIMIT   (4u)

#define APP_SOMEIP_CLIENT_ID        (0x0002u)
#define APP_SOMEIP_PORT             (30500u)

#define APP_SOMEIP_SERVICE_COUNT    (4)

#define APP_SOMEIP_DRIVE_SERVICE    (0x0001u)
#define APP_SOMEIP_SENSOR_SERVICE   (0x0002u)
#define APP_SOMEIP_AEB_SERVICE      (0x0006u)
#define APP_SOMEIP_INFO_SERVICE     (0x0007u)

/* extern QueueHandle_t AppDriveService_GetSomeipRxQueue(void); */
/* extern QueueHandle_t AppAebService_GetSomeipRxQueue(void); */
/* extern QueueHandle_t AppSensorService_GetSomeipRxQueue(void); */

static QueueHandle_t g_tx_queue;
static AppSomeipRoute g_app_routes[APP_SOMEIP_SERVICE_COUNT];

static BaseType_t AppSomeip_Init(void);
static void AppSomeip_Task(void* arg);
static void AppSomeip_ProcessRx(void);
static void AppSomeip_ProcessTx(void);
static QueueHandle_t AppSomeip_FindAppQueue(uint16_t service_id);

BaseType_t AppSomeip_Start(void) {
    if(AppSomeip_Init() != pdPASS) return pdFAIL;

    return xTaskCreate(AppSomeip_Task, "APP SOMEIP", APP_SOMEIP_TASK_STACK_SIZE, NULL, APP_SOMEIP_TASK_PRIORITY, NULL);
}

static BaseType_t AppSomeip_Init(void) {
    LightSomeipConfig config = {
        .client_id = APP_SOMEIP_CLIENT_ID,
        .port = APP_SOMEIP_PORT,
        .routes = {
            {
                .service_id = APP_SOMEIP_DRIVE_SERVICE,
                .endpoint = {
                    .ip = "192.168.10.2",
                    .port = APP_SOMEIP_PORT
                }
            },
            {
                .service_id = APP_SOMEIP_SENSOR_SERVICE,
                .endpoint = {
                    .ip = "192.168.10.2",
                    .port = APP_SOMEIP_PORT
                }
            },
            {
                .service_id = APP_SOMEIP_AEB_SERVICE,
                .endpoint = {
                    .ip = "192.168.10.2",
                    .port = APP_SOMEIP_PORT
                }
            },
            {
                .service_id = APP_SOMEIP_INFO_SERVICE,
                .endpoint = {
                    .ip = "192.168.10.2",
                    .port = APP_SOMEIP_PORT
                }
            }
        },
    };

    if(AppEth_IsReady() != pdPASS) return pdFAIL;
    if(light_someip_init(&config) != SOMEIP_OK) return pdFAIL;

    g_tx_queue = xQueueCreate(APP_SOMEIP_TX_QUEUE_SIZE, sizeof(AppSomeipTxMsg));
    if(g_tx_queue == NULL) return pdFAIL;

    g_app_routes[0].service_id = APP_SOMEIP_DRIVE_SERVICE;
    g_app_routes[0].get_rx_queue = NULL; /* AppDriveService_GetSomeipRxQueue; */

    g_app_routes[1].service_id = APP_SOMEIP_SENSOR_SERVICE;
    g_app_routes[1].get_rx_queue = NULL; /* AppSensorService_GetSomeipRxQueue; */

    g_app_routes[2].service_id = APP_SOMEIP_AEB_SERVICE;
    g_app_routes[2].get_rx_queue = NULL; /* AppAebService_GetSomeipRxQueue; */

    g_app_routes[3].service_id = APP_SOMEIP_INFO_SERVICE;
    g_app_routes[3].get_rx_queue = AppInfoService_GetSomeipRxQueue;

    return pdPASS;
}

static void AppSomeip_Task(void* arg) {
    (void)arg;

    for(;;) {
        if(AppEth_IsReady() == pdPASS && g_tx_queue != NULL) {
            AppSomeip_ProcessRx();
            AppSomeip_ProcessTx();
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void AppSomeip_ProcessRx(void) {
    LightSomeipStatus status;
    LightSomeipPacket packet;
    char remote_ip[SOMEIP_IP_LEN];
    uint16_t remote_port;
    QueueHandle_t dst_queue;
    AppSomeipRxMsg rx_msg;

    for(uint32_t i = 0u; i < APP_SOMEIP_RX_DRAIN_LIMIT; i++) {
        status = light_someip_recv(&packet, remote_ip, &remote_port);
        if(status == SOMEIP_NO_MSG || status != SOMEIP_OK) break;

        dst_queue = AppSomeip_FindAppQueue(packet.service_id);
        if(dst_queue == NULL) {
            continue;
        }

        memset(&rx_msg, 0, sizeof(rx_msg));
        rx_msg.packet = packet;
        memcpy(rx_msg.remote_ip, remote_ip, SOMEIP_IP_LEN);
        rx_msg.remote_port = remote_port;

        (void)xQueueSend(dst_queue, &rx_msg, 0);
    }
}

static void AppSomeip_ProcessTx(void) {
    AppSomeipTxMsg tx_msg;

    if(g_tx_queue == NULL) return;

    while(xQueueReceive(g_tx_queue, &tx_msg, 0) == pdPASS) {
        switch(tx_msg.msg_type) {
            case APP_SOMEIP_TX_REQUEST:
                (void)light_someip_request(&tx_msg.packet);
                break;

            case APP_SOMEIP_TX_RESPONSE:
                (void)light_someip_respond(
                    tx_msg.req_msg.remote_ip,
                    tx_msg.req_msg.remote_port,
                    &tx_msg.req_msg.packet,
                    &tx_msg.packet);
                break;

            case APP_SOMEIP_TX_EVENT:
                (void)light_someip_event_notify(&tx_msg.dst_endpoint, &tx_msg.packet);
                break;

            default:
                break;
        }
    }
}

static QueueHandle_t AppSomeip_FindAppQueue(uint16_t service_id) {
    for(int i = 0; i < APP_SOMEIP_SERVICE_COUNT; i++) {
        if(g_app_routes[i].service_id == service_id) {
            if(g_app_routes[i].get_rx_queue == NULL) return NULL;
            return g_app_routes[i].get_rx_queue();
        }
    }

    return NULL;
}

BaseType_t AppSomeip_Recv(QueueHandle_t rx_queue, AppSomeipRxMsg* rx_msg) {
    if(rx_queue == NULL || rx_msg == NULL) return pdFAIL;

    return xQueueReceive(rx_queue, rx_msg, 0);
}

BaseType_t AppSomeip_SendRequest(LightSomeipPacket* request_packet) {
    AppSomeipTxMsg tx_msg;

    if(request_packet == NULL || g_tx_queue == NULL) return pdFAIL;

    memset(&tx_msg, 0, sizeof(tx_msg));
    tx_msg.msg_type = APP_SOMEIP_TX_REQUEST;
    tx_msg.packet = *request_packet;

    return xQueueSend(g_tx_queue, &tx_msg, 0);
}

BaseType_t AppSomeip_SendResponse(const AppSomeipRxMsg* request_msg, LightSomeipPacket* response_packet) {
    AppSomeipTxMsg tx_msg;

    if(request_msg == NULL || response_packet == NULL || g_tx_queue == NULL) return pdFAIL;
    if(request_msg->packet.message_type != SOMEIP_MSGTYPE_REQUEST) return pdFAIL;

    memset(&tx_msg, 0, sizeof(tx_msg));
    tx_msg.msg_type = APP_SOMEIP_TX_RESPONSE;
    tx_msg.packet = *response_packet;
    tx_msg.req_msg = *request_msg;

    return xQueueSend(g_tx_queue, &tx_msg, 0);
}

BaseType_t AppSomeip_SendEvent(LightSomeipPacket* event_packet, const LightSomeipEndpoint* dst_endpoint) {
    AppSomeipTxMsg tx_msg;

    if(event_packet == NULL || dst_endpoint == NULL || g_tx_queue == NULL) return pdFAIL;

    memset(&tx_msg, 0, sizeof(tx_msg));
    tx_msg.msg_type = APP_SOMEIP_TX_EVENT;
    tx_msg.packet = *event_packet;
    tx_msg.dst_endpoint = *dst_endpoint;

    return xQueueSend(g_tx_queue, &tx_msg, 0);
}
