#include "App_InfoService.h"

#include "App_Someip/App_Someip.h"
#include "someip/light_someip.h"
#include "task.h"
#include <string.h>

#define APP_INFOSERVICE_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define APP_INFOSERVICE_TASK_PRIORITY   (tskIDLE_PRIORITY + 2u)
#define APP_INFOSERVICE_RX_QUEUE_SIZE   (8u)

#define APP_INFOSERVICE_SERVICE_ID      (0x0007u)
#define APP_INFOSERVICE_VERSION_METHOD  (0x1001u)

static QueueHandle_t g_info_service_someip_rx_queue;

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
    static const uint8_t version_payload[] = "1.0.0";
    LightSomeipPacket response_packet;

    if (request_msg == NULL)
    {
        return;
    }

    if (request_msg->packet.service_id != APP_INFOSERVICE_SERVICE_ID ||
        request_msg->packet.method_id != APP_INFOSERVICE_VERSION_METHOD)
    {
        return;
    }

    if (light_someip_packet_init(
            &response_packet,
            request_msg->packet.service_id,
            APP_INFOSERVICE_VERSION_METHOD,
            version_payload,
            (uint32_t)(sizeof(version_payload) - 1u)) != SOMEIP_OK)
    {
        return;
    }

    (void)AppSomeip_SendResponse(request_msg, &response_packet);
}
