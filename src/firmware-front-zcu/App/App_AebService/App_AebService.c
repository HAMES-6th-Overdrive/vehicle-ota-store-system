#include "App_AebService.h"

#include "App_Can/App_Can.h"
#include "App_Someip/App_Someip.h"
#include "task.h"

#define APP_AEBSERVICE_TASK_STACK_SIZE       (configMINIMAL_STACK_SIZE)
#define APP_AEBSERVICE_TASK_PRIORITY         (tskIDLE_PRIORITY + 2u)
#define APP_AEBSERVICE_TASK_PERIOD_MS        (10u)
#define APP_AEBSERVICE_SOMEIP_RX_QUEUE_SIZE  (8u)
#define APP_AEBSERVICE_SOMEIP_DRAIN_LIMIT    (4u)

#define APP_AEBSERVICE_SERVICE_ID            (0x0006u)
#define APP_AEBSERVICE_EVENT_AEB_TRIGGERED   (0x2001u)
#define APP_AEBSERVICE_EVENT_PAYLOAD_LEN     (0u)
#define APP_AEBSERVICE_VEHICLE_COMPUTER_IP   "192.168.10.1"
#define APP_AEBSERVICE_VEHICLE_COMPUTER_PORT (30500u)

#define APP_AEBSERVICE_SENSOR_SERVICE_ID     (0x0002u)
#define APP_AEBSERVICE_SENSOR_EVENT_TOF      (0x2002u)
#define APP_AEBSERVICE_SENSOR_EVENT_SPEED    (0x2003u)

#define APP_AEBSERVICE_CAN_ID_TOF_DISTANCE   (0x201u)
#define APP_AEBSERVICE_CAN_ID_SPEED          (0x202u)

#define APP_AEB_TOF_DISTANCE_INVALID_MM      (0xFFFFu)

#define APP_AEB_CALIB_SPEED_KMH_X100         (630u)
#define APP_AEB_CALIB_BRAKE_DISTANCE_MM      (800u)
#define APP_AEB_CONTROL_DELAY_MS             (100u)
#define APP_AEB_SAFETY_MARGIN_MM             (150u)

#define APP_AEB_MIN_STOP_DISTANCE_MM         (200u)
#define APP_AEB_MAX_STOP_DISTANCE_MM         (2200u)

#define APP_AEB_RELEASE_MARGIN_MM            (200u)
#define APP_AEB_MIN_RELEASE_DISTANCE_MM      (200u)
#define APP_AEB_RELEASE_CONFIRM_COUNT        (3u)

#define APP_AEB_MAX_SPEED_KMH_X100           (700u)

static uint16_t g_aeb_distance_mm = APP_AEB_TOF_DISTANCE_INVALID_MM;
static uint16_t g_aeb_speed_kmh_x100 = 0u;
static AppAebServiceStopCmd g_aeb_stop_cmd = APP_AEBSERVICE_STOP_CMD_GO;
static BaseType_t g_aeb_active = pdFALSE;
static BaseType_t g_aeb_trigger_event_pending = pdFALSE;
static uint8_t g_aeb_release_count = 0u;
static QueueHandle_t g_aeb_service_someip_rx_queue = NULL;

static BaseType_t AppAebService_Init(void);
static void AppAebService_Task(void *arg);
static void AppAebService_SetSensorData(uint16_t distance_mm, uint16_t speed_kmh_x100);
static void AppAebService_SetDistance(uint16_t distance_mm);
static void AppAebService_SetSpeed(uint16_t speed_kmh_x100);
static void AppAebService_ReadLatestSensorFrames(void);
static void AppAebService_UpdateFromFrame(const AppCanFrame *frame);
static void AppAebService_ProcessSomeip(void);
static void AppAebService_HandleSomeipMessage(const AppSomeipRxMsg *rx_msg);
static void AppAebService_Evaluate(void);
static void AppAebService_SendPendingEvents(void);
static BaseType_t AppAebService_SendAebTriggeredEvent(void);
static uint16_t AppAebService_ReadU16Le(const uint8_t *data);
static uint16_t AppAebService_CalculateStopDistanceMm(uint16_t speed_kmh_x100);
static uint16_t AppAebService_CalculateReleaseDistanceMm(uint16_t stop_distance_mm);
static uint16_t AppAebService_CalculateBrakeDistanceMm(uint16_t speed_kmh_x100);
static uint16_t AppAebService_CalculateDelayDistanceMm(uint16_t speed_kmh_x100);

BaseType_t AppAebService_Start(void)
{
    if(AppAebService_Init() != pdPASS)
    {
        return pdFAIL;
    }

    return xTaskCreate(AppAebService_Task,
                       "APP AEB SERVICE",
                       APP_AEBSERVICE_TASK_STACK_SIZE,
                       NULL,
                       APP_AEBSERVICE_TASK_PRIORITY,
                       NULL);
}

QueueHandle_t AppAebService_GetSomeipRxQueue(void)
{
    return g_aeb_service_someip_rx_queue;
}

AppAebServiceStopCmd AppAebService_GetStopCmd(void)
{
    AppAebServiceStopCmd stop_cmd;

    taskENTER_CRITICAL();
    stop_cmd = g_aeb_stop_cmd;
    taskEXIT_CRITICAL();

    return stop_cmd;
}

static void AppAebService_SetSensorData(uint16_t distance_mm, uint16_t speed_kmh_x100)
{
    taskENTER_CRITICAL();
    g_aeb_distance_mm = distance_mm;
    g_aeb_speed_kmh_x100 = speed_kmh_x100;
    taskEXIT_CRITICAL();
}

static void AppAebService_SetDistance(uint16_t distance_mm)
{
    taskENTER_CRITICAL();
    g_aeb_distance_mm = distance_mm;
    taskEXIT_CRITICAL();
}

static void AppAebService_SetSpeed(uint16_t speed_kmh_x100)
{
    taskENTER_CRITICAL();
    g_aeb_speed_kmh_x100 = speed_kmh_x100;
    taskEXIT_CRITICAL();
}

static BaseType_t AppAebService_Init(void)
{
    g_aeb_distance_mm = APP_AEB_TOF_DISTANCE_INVALID_MM;
    g_aeb_speed_kmh_x100 = 0u;
    g_aeb_stop_cmd = APP_AEBSERVICE_STOP_CMD_GO;
    g_aeb_active = pdFALSE;
    g_aeb_trigger_event_pending = pdFALSE;
    g_aeb_release_count = 0u;

    if(g_aeb_service_someip_rx_queue == NULL)
    {
        g_aeb_service_someip_rx_queue =
            xQueueCreate(APP_AEBSERVICE_SOMEIP_RX_QUEUE_SIZE,
                         sizeof(AppSomeipRxMsg));
    }

    return (g_aeb_service_someip_rx_queue != NULL) ? pdPASS : pdFAIL;
}

static void AppAebService_Task(void *arg)
{
    (void)arg;

    for(;;)
    {
        AppAebService_ReadLatestSensorFrames();
        AppAebService_ProcessSomeip();
        AppAebService_Evaluate();
        AppAebService_SendPendingEvents();
        vTaskDelay(pdMS_TO_TICKS(APP_AEBSERVICE_TASK_PERIOD_MS));
    }
}

static void AppAebService_ReadLatestSensorFrames(void)
{
    AppCanFrame frame;

    if(AppCan_ReadLatestById(APP_AEBSERVICE_CAN_ID_TOF_DISTANCE, &frame) == pdPASS)
    {
        AppAebService_UpdateFromFrame(&frame);
    }

    if(AppCan_ReadLatestById(APP_AEBSERVICE_CAN_ID_SPEED, &frame) == pdPASS)
    {
        AppAebService_UpdateFromFrame(&frame);
    }
}

static void AppAebService_UpdateFromFrame(const AppCanFrame *frame)
{
    uint16_t value;

    if((frame == NULL) || (frame->is_fd != 0u) || (frame->length < 2u))
    {
        return;
    }

    value = AppAebService_ReadU16Le(frame->data);

    taskENTER_CRITICAL();
    if(frame->id == APP_AEBSERVICE_CAN_ID_TOF_DISTANCE)
    {
        g_aeb_distance_mm = value;
    }
    else if(frame->id == APP_AEBSERVICE_CAN_ID_SPEED)
    {
        g_aeb_speed_kmh_x100 = value;
    }
    else
    {
        /* No action required */
    }
    taskEXIT_CRITICAL();
}

static void AppAebService_ProcessSomeip(void)
{
    AppSomeipRxMsg rx_msg;
    uint8_t i;

    for(i = 0u; i < APP_AEBSERVICE_SOMEIP_DRAIN_LIMIT; i++)
    {
        if(AppSomeip_Recv(g_aeb_service_someip_rx_queue, &rx_msg) != pdPASS)
        {
            break;
        }

        AppAebService_HandleSomeipMessage(&rx_msg);
    }
}

static void AppAebService_HandleSomeipMessage(const AppSomeipRxMsg *rx_msg)
{
    uint16_t distance_mm;
    uint16_t speed_kmh_x100;

    if(rx_msg == NULL)
    {
        return;
    }

    if(rx_msg->packet.service_id != APP_AEBSERVICE_SENSOR_SERVICE_ID)
    {
        return;
    }

    if(rx_msg->packet.method_id == APP_AEBSERVICE_SENSOR_EVENT_TOF)
    {
        if(rx_msg->packet.payload_len < 2u)
        {
            return;
        }

        distance_mm = AppAebService_ReadU16Le(&rx_msg->packet.payload_arr[0]);
        AppAebService_SetDistance(distance_mm);
        return;
    }

    if(rx_msg->packet.method_id == APP_AEBSERVICE_SENSOR_EVENT_SPEED)
    {
        if(rx_msg->packet.payload_len < 2u)
        {
            return;
        }

        speed_kmh_x100 = AppAebService_ReadU16Le(&rx_msg->packet.payload_arr[0]);
        AppAebService_SetSpeed(speed_kmh_x100);
        return;
    }

    if(rx_msg->packet.payload_len < 4u)
    {
        return;
    }

    distance_mm = AppAebService_ReadU16Le(&rx_msg->packet.payload_arr[0]);
    speed_kmh_x100 = AppAebService_ReadU16Le(&rx_msg->packet.payload_arr[2]);

    AppAebService_SetSensorData(distance_mm, speed_kmh_x100);
}

static void AppAebService_Evaluate(void)
{
    uint16_t distance_mm;
    uint16_t speed_kmh_x100;
    uint16_t stop_distance_mm;
    uint16_t release_distance_mm;
    AppAebServiceStopCmd stop_cmd;

    taskENTER_CRITICAL();
    distance_mm = g_aeb_distance_mm;
    speed_kmh_x100 = g_aeb_speed_kmh_x100;
    taskEXIT_CRITICAL();

    if(speed_kmh_x100 > APP_AEB_MAX_SPEED_KMH_X100)
    {
        speed_kmh_x100 = APP_AEB_MAX_SPEED_KMH_X100;
    }

    stop_distance_mm = AppAebService_CalculateStopDistanceMm(speed_kmh_x100);
    release_distance_mm = AppAebService_CalculateReleaseDistanceMm(stop_distance_mm);

    if(distance_mm == APP_AEB_TOF_DISTANCE_INVALID_MM)
    {
        if(g_aeb_active == pdTRUE)
        {
            if(g_aeb_release_count < APP_AEB_RELEASE_CONFIRM_COUNT)
            {
                g_aeb_release_count++;
            }

            if(g_aeb_release_count >= APP_AEB_RELEASE_CONFIRM_COUNT)
            {
                g_aeb_active = pdFALSE;
                g_aeb_release_count = 0u;
            }
        }
        else
        {
            g_aeb_release_count = 0u;
        }
    }
    else if(g_aeb_active == pdTRUE)
    {
        if(distance_mm >= release_distance_mm)
        {
            if(g_aeb_release_count < APP_AEB_RELEASE_CONFIRM_COUNT)
            {
                g_aeb_release_count++;
            }

            if(g_aeb_release_count >= APP_AEB_RELEASE_CONFIRM_COUNT)
            {
                g_aeb_active = pdFALSE;
                g_aeb_release_count = 0u;
            }
        }
        else
        {
            g_aeb_release_count = 0u;
        }
    }
    else
    {
        g_aeb_release_count = 0u;

        if(distance_mm <= stop_distance_mm)
        {
            g_aeb_active = pdTRUE;
            g_aeb_trigger_event_pending = pdTRUE;
        }
    }

    stop_cmd = (g_aeb_active == pdTRUE) ?
        APP_AEBSERVICE_STOP_CMD_STOP :
        APP_AEBSERVICE_STOP_CMD_GO;

    taskENTER_CRITICAL();
    g_aeb_stop_cmd = stop_cmd;
    taskEXIT_CRITICAL();
}

static void AppAebService_SendPendingEvents(void)
{
    if(g_aeb_trigger_event_pending != pdTRUE)
    {
        return;
    }

    if(AppAebService_SendAebTriggeredEvent() == pdPASS)
    {
        g_aeb_trigger_event_pending = pdFALSE;
    }
}

static BaseType_t AppAebService_SendAebTriggeredEvent(void)
{
    static const LightSomeipEndpoint dst_endpoint = {
        .ip = APP_AEBSERVICE_VEHICLE_COMPUTER_IP,
        .port = APP_AEBSERVICE_VEHICLE_COMPUTER_PORT
    };
    LightSomeipPacket event_packet;

    if(light_someip_packet_init(&event_packet,
                                APP_AEBSERVICE_SERVICE_ID,
                                APP_AEBSERVICE_EVENT_AEB_TRIGGERED,
                                NULL,
                                APP_AEBSERVICE_EVENT_PAYLOAD_LEN) != SOMEIP_OK)
    {
        return pdFAIL;
    }

    return AppSomeip_SendEvent(&event_packet, &dst_endpoint);
}

static uint16_t AppAebService_ReadU16Le(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[1] << 8u) | (uint16_t)data[0]);
}

static uint16_t AppAebService_CalculateStopDistanceMm(uint16_t speed_kmh_x100)
{
    uint32_t stop_distance_mm;

    stop_distance_mm =
        (uint32_t)AppAebService_CalculateBrakeDistanceMm(speed_kmh_x100) +
        (uint32_t)AppAebService_CalculateDelayDistanceMm(speed_kmh_x100) +
        (uint32_t)APP_AEB_SAFETY_MARGIN_MM;

    if(stop_distance_mm < APP_AEB_MIN_STOP_DISTANCE_MM)
    {
        stop_distance_mm = APP_AEB_MIN_STOP_DISTANCE_MM;
    }
    else if(stop_distance_mm > APP_AEB_MAX_STOP_DISTANCE_MM)
    {
        stop_distance_mm = APP_AEB_MAX_STOP_DISTANCE_MM;
    }
    else
    {
        /* No action required */
    }

    return (uint16_t)stop_distance_mm;
}

static uint16_t AppAebService_CalculateReleaseDistanceMm(uint16_t stop_distance_mm)
{
    uint32_t release_distance_mm;

    release_distance_mm =
        (uint32_t)stop_distance_mm +
        (uint32_t)APP_AEB_RELEASE_MARGIN_MM;

    if(release_distance_mm < APP_AEB_MIN_RELEASE_DISTANCE_MM)
    {
        release_distance_mm = APP_AEB_MIN_RELEASE_DISTANCE_MM;
    }
    else if(release_distance_mm > 0xFFFFu)
    {
        release_distance_mm = 0xFFFFu;
    }
    else
    {
        /* No action required */
    }

    return (uint16_t)release_distance_mm;
}

static uint16_t AppAebService_CalculateBrakeDistanceMm(uint16_t speed_kmh_x100)
{
    uint32_t brake_distance_mm;
    uint32_t numerator;
    uint32_t denominator;

    numerator =
        (uint32_t)APP_AEB_CALIB_BRAKE_DISTANCE_MM *
        (uint32_t)speed_kmh_x100 *
        (uint32_t)speed_kmh_x100;

    denominator =
        (uint32_t)APP_AEB_CALIB_SPEED_KMH_X100 *
        (uint32_t)APP_AEB_CALIB_SPEED_KMH_X100;

    if(denominator == 0u)
    {
        return 0u;
    }

    brake_distance_mm = numerator / denominator;

    if(brake_distance_mm > 0xFFFFu)
    {
        brake_distance_mm = 0xFFFFu;
    }

    return (uint16_t)brake_distance_mm;
}

static uint16_t AppAebService_CalculateDelayDistanceMm(uint16_t speed_kmh_x100)
{
    uint32_t speed_mm_per_sec;
    uint32_t delay_distance_mm;

    speed_mm_per_sec = ((uint32_t)speed_kmh_x100 * 1000u) / 360u;
    delay_distance_mm =
        (speed_mm_per_sec * (uint32_t)APP_AEB_CONTROL_DELAY_MS) / 1000u;

    if(delay_distance_mm > 0xFFFFu)
    {
        delay_distance_mm = 0xFFFFu;
    }

    return (uint16_t)delay_distance_mm;
}
