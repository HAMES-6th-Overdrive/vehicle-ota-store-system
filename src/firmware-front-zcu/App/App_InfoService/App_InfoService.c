#include "App_InfoService.h"

#include "App_AebService/App_AebService.h"
#include "App_Someip/App_Someip.h"
#include "someip/light_someip.h"
#include "task.h"
#include <string.h>

#define APP_INFOSERVICE_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define APP_INFOSERVICE_TASK_PRIORITY   (tskIDLE_PRIORITY + 2u)
#define APP_INFOSERVICE_RX_QUEUE_SIZE   (8u)

#define APP_INFOSERVICE_SERVICE_ID      (0x0007u)
#define APP_INFOSERVICE_FRONTZCU_VERSION_METHOD (0x1003u)
#define APP_INFOSERVICE_AEB_VERSION_METHOD      (0x1004u)
#define APP_INFOSERVICE_VERSION_PAYLOAD_LENGTH  (10u)

static QueueHandle_t g_info_service_someip_rx_queue;

extern const char *App_GetFrontZcuVersion(void);

static BaseType_t AppInfoService_Init(void);
static void AppInfoService_Task(void *arg);
static void AppInfoService_SendVersionResponse(const AppSomeipRxMsg *request_msg);

BaseType_t AppInfoService_Start(void)
{
    if (AppInfoService_Init() != pdPASS)
    {
        return pdFAIL;
    }

    return xTaskCreate(AppInfoService_Task, "APP INFO SERVICE", APP_INFOSERVICE_TASK_STACK_SIZE, NULL, APP_INFOSERVICE_TASK_PRIORITY, NULL);
}

QueueHandle_t AppInfoService_GetSomeipRxQueue(void)
{
    return g_info_service_someip_rx_queue;
}

static BaseType_t AppInfoService_Init(void)
{
    if (g_info_service_someip_rx_queue == NULL)
    {
        g_info_service_someip_rx_queue = xQueueCreate(APP_INFOSERVICE_RX_QUEUE_SIZE, sizeof(AppSomeipRxMsg));
    }

    return (g_info_service_someip_rx_queue != NULL) ? pdPASS : pdFAIL;
}

static void AppInfoService_Task(void *arg)
{
    AppSomeipRxMsg rx_msg;

    (void)arg;

    for (;;)
    {
        if (AppSomeip_Recv(g_info_service_someip_rx_queue, &rx_msg) == pdPASS)
        {
            AppInfoService_SendVersionResponse(&rx_msg);
        }

        vTaskDelay(pdMS_TO_TICKS(1u));
    }
}

static void AppInfoService_SendVersionResponse(const AppSomeipRxMsg *request_msg)
{
    uint8_t version_payload[APP_INFOSERVICE_VERSION_PAYLOAD_LENGTH] = {0u};
    const char *version_string;
    size_t version_length;
    LightSomeipPacket response_packet;

    if (request_msg == NULL)
    {
        return;
    }

    if (request_msg->packet.service_id != APP_INFOSERVICE_SERVICE_ID)
    {
        return;
    }

    if (request_msg->packet.method_id == APP_INFOSERVICE_FRONTZCU_VERSION_METHOD)
    {
        version_string = App_GetFrontZcuVersion();
    }
    else if (request_msg->packet.method_id == APP_INFOSERVICE_AEB_VERSION_METHOD)
    {
        version_string = AppAebService_GetVersion();
    }
    else
    {
        return;
    }

    version_length = strlen(version_string);
    if (version_length > APP_INFOSERVICE_VERSION_PAYLOAD_LENGTH)
    {
        version_length = APP_INFOSERVICE_VERSION_PAYLOAD_LENGTH;
    }
    (void)memcpy(version_payload, version_string, version_length);

    if (light_someip_packet_init(
            &response_packet,
            request_msg->packet.service_id,
            request_msg->packet.method_id,
            version_payload,
            APP_INFOSERVICE_VERSION_PAYLOAD_LENGTH) != SOMEIP_OK)
    {
        return;
    }

    (void)AppSomeip_SendResponse(request_msg, &response_packet);
}
