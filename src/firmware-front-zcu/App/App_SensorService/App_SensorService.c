#include "App_SensorService.h"

#include "App_Can/App_Can.h"
#include "App_Someip/App_Someip.h"
#include "task.h"

#define APP_SENSORSERVICE_TASK_STACK_SIZE       (configMINIMAL_STACK_SIZE)
#define APP_SENSORSERVICE_TASK_PRIORITY         (tskIDLE_PRIORITY + 2u)
#define APP_SENSORSERVICE_TASK_PERIOD_MS        (1u)
#define APP_SENSORSERVICE_CAN_DRAIN_LIMIT       (4u)

#define APP_SENSORSERVICE_SERVICE_ID            (0x0002u)
#define APP_SENSORSERVICE_EVENT_TOF_UPDATED     (0x2002u)
#define APP_SENSORSERVICE_EVENT_SPEED_UPDATED   (0x2003u)
#define APP_SENSORSERVICE_PAYLOAD_U16_LEN       (2u)

#define APP_SENSORSERVICE_SPEED_PERIOD_MS       (100u)

#define APP_SENSORSERVICE_VEHICLE_COMPUTER_IP   "192.168.10.1"
#define APP_SENSORSERVICE_VEHICLE_COMPUTER_PORT (30500u)

#define APP_SENSORSERVICE_CAN_ID_TOF_DISTANCE   (0x201u)
#define APP_SENSORSERVICE_CAN_ID_SPEED          (0x202u)

static void AppSensorService_Task(void *arg);
static void AppSensorService_ProcessTofFrames(void);
static void AppSensorService_ProcessSpeedFrames(void);
static void AppSensorService_SendPendingSpeed(TickType_t now, TickType_t *last_speed_tick);
static BaseType_t AppSensorService_ReadU16Frame(const AppCanFrame *frame, uint16_t *value);
static BaseType_t AppSensorService_SendU16Event(uint16_t event_id, uint16_t value);
static void AppSensorService_WriteU16Le(uint8_t *data, uint16_t value);

static uint16_t g_sensor_service_speed_kmh_x100 = 0u;
static BaseType_t g_sensor_service_speed_pending = pdFALSE;

BaseType_t AppSensorService_Start(void)
{
    return xTaskCreate(AppSensorService_Task,
                       "APP SENSOR SERVICE",
                       APP_SENSORSERVICE_TASK_STACK_SIZE,
                       NULL,
                       APP_SENSORSERVICE_TASK_PRIORITY,
                       NULL);
}

static void AppSensorService_Task(void *arg)
{
    TickType_t last_speed_tick;
    TickType_t now;

    (void)arg;

    last_speed_tick = xTaskGetTickCount();

    for(;;)
    {
        now = xTaskGetTickCount();

        AppSensorService_ProcessTofFrames();
        AppSensorService_ProcessSpeedFrames();
        AppSensorService_SendPendingSpeed(now, &last_speed_tick);

        vTaskDelay(pdMS_TO_TICKS(APP_SENSORSERVICE_TASK_PERIOD_MS));
    }
}

static void AppSensorService_ProcessTofFrames(void)
{
    AppCanFrame frame;
    uint16_t tof_mm;
    uint8_t i;

    for(i = 0u; i < APP_SENSORSERVICE_CAN_DRAIN_LIMIT; i++)
    {
        if(AppCan_RecvById(APP_SENSORSERVICE_CAN_ID_TOF_DISTANCE, &frame, 0u) != pdPASS)
        {
            break;
        }

        if(AppSensorService_ReadU16Frame(&frame, &tof_mm) == pdPASS)
        {
            (void)AppSensorService_SendU16Event(APP_SENSORSERVICE_EVENT_TOF_UPDATED, tof_mm);
        }
    }
}

static void AppSensorService_ProcessSpeedFrames(void)
{
    AppCanFrame frame;
    uint16_t speed_kmh_x100;
    uint8_t i;

    for(i = 0u; i < APP_SENSORSERVICE_CAN_DRAIN_LIMIT; i++)
    {
        if(AppCan_RecvById(APP_SENSORSERVICE_CAN_ID_SPEED, &frame, 0u) != pdPASS)
        {
            break;
        }

        if(AppSensorService_ReadU16Frame(&frame, &speed_kmh_x100) == pdPASS)
        {
            g_sensor_service_speed_kmh_x100 = speed_kmh_x100;
            g_sensor_service_speed_pending = pdTRUE;
        }
    }
}

static void AppSensorService_SendPendingSpeed(TickType_t now, TickType_t *last_speed_tick)
{
    if(last_speed_tick == NULL)
    {
        return;
    }

    if((TickType_t)(now - *last_speed_tick) < pdMS_TO_TICKS(APP_SENSORSERVICE_SPEED_PERIOD_MS))
    {
        return;
    }

    *last_speed_tick = now;

    if(g_sensor_service_speed_pending != pdTRUE)
    {
        return;
    }

    if(AppSensorService_SendU16Event(APP_SENSORSERVICE_EVENT_SPEED_UPDATED,
                                     g_sensor_service_speed_kmh_x100) == pdPASS)
    {
        g_sensor_service_speed_pending = pdFALSE;
    }
}

static BaseType_t AppSensorService_ReadU16Frame(const AppCanFrame *frame, uint16_t *value)
{
    if((frame == NULL) || (value == NULL))
    {
        return pdFAIL;
    }

    if((frame->is_fd != 0u) || (frame->length < APP_SENSORSERVICE_PAYLOAD_U16_LEN))
    {
        return pdFAIL;
    }

    *value = (uint16_t)(((uint16_t)frame->data[1] << 8u) | (uint16_t)frame->data[0]);

    return pdPASS;
}

static BaseType_t AppSensorService_SendU16Event(uint16_t event_id, uint16_t value)
{
    static const LightSomeipEndpoint dst_endpoint = {
        .ip = APP_SENSORSERVICE_VEHICLE_COMPUTER_IP,
        .port = APP_SENSORSERVICE_VEHICLE_COMPUTER_PORT
    };
    LightSomeipPacket event_packet;
    uint8_t payload[APP_SENSORSERVICE_PAYLOAD_U16_LEN];

    AppSensorService_WriteU16Le(payload, value);

    if(light_someip_packet_init(&event_packet,
                                APP_SENSORSERVICE_SERVICE_ID,
                                event_id,
                                payload,
                                APP_SENSORSERVICE_PAYLOAD_U16_LEN) != SOMEIP_OK)
    {
        return pdFAIL;
    }

    return AppSomeip_SendEvent(&event_packet, &dst_endpoint);
}

static void AppSensorService_WriteU16Le(uint8_t *data, uint16_t value)
{
    if(data == NULL)
    {
        return;
    }

    data[0] = (uint8_t)(value & 0xFFu);
    data[1] = (uint8_t)((value >> 8u) & 0xFFu);
}
