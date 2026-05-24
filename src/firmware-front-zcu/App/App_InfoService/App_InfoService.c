#include "App_InfoService.h"

#include "App_AebService/App_AebService.h"
#include "App_Can/App_Can.h"
#include "App_Someip/App_Someip.h"
#include "someip/light_someip.h"
#include "task.h"
#include <string.h>

#define APP_INFOSERVICE_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define APP_INFOSERVICE_TASK_PRIORITY   (tskIDLE_PRIORITY + 2u)
#define APP_INFOSERVICE_RX_QUEUE_SIZE   (8u)

#define APP_INFOSERVICE_SERVICE_ID      (0x0007u)
#define APP_INFOSERVICE_SENSOR_VERSION_METHOD   (0x1001u)
#define APP_INFOSERVICE_DRIVE_VERSION_METHOD    (0x1002u)
#define APP_INFOSERVICE_FRONTZCU_VERSION_METHOD (0x1003u)
#define APP_INFOSERVICE_AEB_VERSION_METHOD      (0x1004u)
#define APP_INFOSERVICE_VERSION_PAYLOAD_LENGTH  (8u)

#define APP_INFOSERVICE_CAN_ID_VERSION_REQUEST  (0x700u)
#define APP_INFOSERVICE_CAN_ID_FRONTZCU_VERSION (0x701u)
#define APP_INFOSERVICE_CAN_ID_DRIVE_VERSION    (0x702u)
#define APP_INFOSERVICE_CAN_ID_SENSOR_VERSION   (0x703u)

static QueueHandle_t g_info_service_someip_rx_queue;
static uint8_t g_drive_ecu_version[APP_INFOSERVICE_VERSION_PAYLOAD_LENGTH] = {0u};
static uint8_t g_sensor_ecu_version[APP_INFOSERVICE_VERSION_PAYLOAD_LENGTH] = {0u};

extern const char *App_GetFrontZcuVersion(void);

static BaseType_t AppInfoService_Init(void);
static void AppInfoService_Task(void *arg);
static void AppInfoService_ProcessCanVersionResponses(void);
static void AppInfoService_SendVersionResponse(const AppSomeipRxMsg *request_msg);
static void AppInfoService_CopyVersionString(uint8_t *payload, const char *version_string);
static void AppInfoService_CopyCanVersion(uint8_t *payload, const uint8_t *version);
static void AppInfoService_UpdateCanVersion(uint32_t can_id, uint8_t *version);
static void AppInfoService_RequestCanVersions(void);
static void AppInfoService_SendFrontZcuCanVersion(void);

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
    (void)memset(g_drive_ecu_version, 0, sizeof(g_drive_ecu_version));
    (void)memset(g_sensor_ecu_version, 0, sizeof(g_sensor_ecu_version));

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

    AppInfoService_RequestCanVersions();

    for (;;)
    {
        AppInfoService_ProcessCanVersionResponses();

        if (AppSomeip_Recv(g_info_service_someip_rx_queue, &rx_msg) == pdPASS)
        {
            AppInfoService_SendVersionResponse(&rx_msg);
        }

        vTaskDelay(pdMS_TO_TICKS(1u));
    }
}

static void AppInfoService_ProcessCanVersionResponses(void)
{
    AppInfoService_UpdateCanVersion(APP_INFOSERVICE_CAN_ID_DRIVE_VERSION,
                                    g_drive_ecu_version);
    AppInfoService_UpdateCanVersion(APP_INFOSERVICE_CAN_ID_SENSOR_VERSION,
                                    g_sensor_ecu_version);
}

static void AppInfoService_SendVersionResponse(const AppSomeipRxMsg *request_msg)
{
    uint8_t version_payload[APP_INFOSERVICE_VERSION_PAYLOAD_LENGTH] = {0u};
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
        AppInfoService_CopyVersionString(version_payload, App_GetFrontZcuVersion());
    }
    else if (request_msg->packet.method_id == APP_INFOSERVICE_AEB_VERSION_METHOD)
    {
        AppInfoService_CopyVersionString(version_payload, AppAebService_GetVersion());
    }
    else if (request_msg->packet.method_id == APP_INFOSERVICE_DRIVE_VERSION_METHOD)
    {
        AppInfoService_RequestCanVersions();
        AppInfoService_CopyCanVersion(version_payload, g_drive_ecu_version);
    }
    else if (request_msg->packet.method_id == APP_INFOSERVICE_SENSOR_VERSION_METHOD)
    {
        AppInfoService_RequestCanVersions();
        AppInfoService_CopyCanVersion(version_payload, g_sensor_ecu_version);
    }
    else
    {
        return;
    }

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

static void AppInfoService_CopyVersionString(uint8_t *payload, const char *version_string)
{
    size_t version_length;

    if ((payload == NULL) || (version_string == NULL))
    {
        return;
    }

    version_length = strlen(version_string);
    if (version_length > APP_INFOSERVICE_VERSION_PAYLOAD_LENGTH)
    {
        version_length = APP_INFOSERVICE_VERSION_PAYLOAD_LENGTH;
    }

    (void)memcpy(payload, version_string, version_length);
}

static void AppInfoService_CopyCanVersion(uint8_t *payload, const uint8_t *version)
{
    if ((payload == NULL) || (version == NULL))
    {
        return;
    }

    taskENTER_CRITICAL();
    (void)memcpy(payload, version, APP_INFOSERVICE_VERSION_PAYLOAD_LENGTH);
    taskEXIT_CRITICAL();
}

static void AppInfoService_UpdateCanVersion(uint32_t can_id, uint8_t *version)
{
    AppCanFrame frame;

    if (version == NULL)
    {
        return;
    }

    while (AppCan_RecvById(can_id, &frame, 0u) == pdPASS)
    {
        if ((frame.is_fd == 0u) && (frame.length >= APP_INFOSERVICE_VERSION_PAYLOAD_LENGTH))
        {
            taskENTER_CRITICAL();
            (void)memcpy(version, frame.data, APP_INFOSERVICE_VERSION_PAYLOAD_LENGTH);
            taskEXIT_CRITICAL();
        }
    }
}

static void AppInfoService_RequestCanVersions(void)
{
    (void)AppCan_SendClassic(APP_INFOSERVICE_CAN_ID_VERSION_REQUEST, NULL, 0u);
    AppInfoService_SendFrontZcuCanVersion();
}

static void AppInfoService_SendFrontZcuCanVersion(void)
{
    uint8_t version_payload[APP_INFOSERVICE_VERSION_PAYLOAD_LENGTH] = {0u};

    AppInfoService_CopyVersionString(version_payload, App_GetFrontZcuVersion());
    (void)AppCan_SendClassic(APP_INFOSERVICE_CAN_ID_FRONTZCU_VERSION,
                             version_payload,
                             APP_INFOSERVICE_VERSION_PAYLOAD_LENGTH);
}
