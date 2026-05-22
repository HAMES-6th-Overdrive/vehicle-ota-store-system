#include "App_DriveService.h"

#include "App_AebService/App_AebService.h"
#include "App_Can/App_Can.h"
#include "App_Someip/App_Someip.h"
#include "task.h"
#include <stdint.h>

#define APP_DRIVESERVICE_TASK_STACK_SIZE     (configMINIMAL_STACK_SIZE)
#define APP_DRIVESERVICE_TASK_PRIORITY       (tskIDLE_PRIORITY + 2u)
#define APP_DRIVESERVICE_TASK_PERIOD_MS      (1u)
#define APP_DRIVESERVICE_TX_PERIOD_MS        (50u)
#define APP_DRIVESERVICE_SOMEIP_RX_QUEUE_SIZE (8u)
#define APP_DRIVESERVICE_SOMEIP_DRAIN_LIMIT  (8u)

#define APP_DRIVESERVICE_CAN_ID_VEHICLE_STATE       (0x080u)
#define APP_DRIVESERVICE_CAN_ID_CONTROL_CMD         (0x100u)
#define APP_DRIVESERVICE_CAN_DLC_VEHICLE_STATE      (1u)
#define APP_DRIVESERVICE_CAN_DLC_CONTROL_CMD        (3u)

#define APP_DRIVESERVICE_GEAR_STATE_D               (0x01u)
#define APP_DRIVESERVICE_DRIVE_CMD_STOP_VALUE       (127u)
#define APP_DRIVESERVICE_STEERING_CMD_CENTER_VALUE  (127u)

typedef uint8_t AppDriveServiceDriveCmd;
typedef uint8_t AppDriveServiceSteeringCmd;
typedef uint8_t AppDriveServiceGearState;

typedef struct AppDriveServiceCommand {
    AppDriveServiceDriveCmd drive_cmd;
    AppDriveServiceSteeringCmd steering_cmd;
} AppDriveServiceCommand;

static AppDriveServiceDriveCmd g_drive_cmd = APP_DRIVESERVICE_DRIVE_CMD_STOP_VALUE;
static AppDriveServiceSteeringCmd g_steering_cmd = APP_DRIVESERVICE_STEERING_CMD_CENTER_VALUE;
static QueueHandle_t g_drive_service_someip_rx_queue = NULL;

static BaseType_t AppDriveService_Init(void);
static void AppDriveService_Task(void *arg);
static void AppDriveService_SetCommand(AppDriveServiceDriveCmd drive_cmd,
                                       AppDriveServiceSteeringCmd steering_cmd);
static void AppDriveService_GetCommand(AppDriveServiceCommand *command);
static void AppDriveService_ProcessSomeip(void);
static void AppDriveService_HandleSomeipMessage(const AppSomeipRxMsg *rx_msg);
static BaseType_t AppDriveService_SendVehicleState(AppDriveServiceGearState gear_state);
static BaseType_t AppDriveService_SendControlCmd(AppDriveServiceDriveCmd drive_cmd,
                                                 AppDriveServiceSteeringCmd steering_cmd,
                                                 AppAebServiceStopCmd stop_cmd);

BaseType_t AppDriveService_Start(void)
{
    if(AppDriveService_Init() != pdPASS)
    {
        return pdFAIL;
    }

    return xTaskCreate(AppDriveService_Task,
                       "APP DRIVE SERVICE",
                       APP_DRIVESERVICE_TASK_STACK_SIZE,
                       NULL,
                       APP_DRIVESERVICE_TASK_PRIORITY,
                       NULL);
}

QueueHandle_t AppDriveService_GetSomeipRxQueue(void)
{
    return g_drive_service_someip_rx_queue;
}

static void AppDriveService_SetCommand(AppDriveServiceDriveCmd drive_cmd,
                                       AppDriveServiceSteeringCmd steering_cmd)
{
    taskENTER_CRITICAL();
    g_drive_cmd = drive_cmd;
    g_steering_cmd = steering_cmd;
    taskEXIT_CRITICAL();
}

static void AppDriveService_GetCommand(AppDriveServiceCommand *command)
{
    if(command == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    command->drive_cmd = g_drive_cmd;
    command->steering_cmd = g_steering_cmd;
    taskEXIT_CRITICAL();
}

static BaseType_t AppDriveService_SendVehicleState(AppDriveServiceGearState gear_state)
{
    uint8_t payload[APP_DRIVESERVICE_CAN_DLC_VEHICLE_STATE];

    payload[0] = gear_state;

    return AppCan_SendClassic(APP_DRIVESERVICE_CAN_ID_VEHICLE_STATE,
                              payload,
                              APP_DRIVESERVICE_CAN_DLC_VEHICLE_STATE);
}

static BaseType_t AppDriveService_Init(void)
{
    g_drive_cmd = APP_DRIVESERVICE_DRIVE_CMD_STOP_VALUE;
    g_steering_cmd = APP_DRIVESERVICE_STEERING_CMD_CENTER_VALUE;

    if(g_drive_service_someip_rx_queue == NULL)
    {
        g_drive_service_someip_rx_queue =
            xQueueCreate(APP_DRIVESERVICE_SOMEIP_RX_QUEUE_SIZE,
                         sizeof(AppSomeipRxMsg));
    }

    return (g_drive_service_someip_rx_queue != NULL) ? pdPASS : pdFAIL;
}

static void AppDriveService_Task(void *arg)
{
    AppDriveServiceCommand command;
    AppAebServiceStopCmd stop_cmd;
    TickType_t last_tx_tick;
    TickType_t now;

    (void)arg;

    (void)AppDriveService_SendVehicleState(APP_DRIVESERVICE_GEAR_STATE_D);
    last_tx_tick = xTaskGetTickCount();

    for(;;)
    {
        AppDriveService_ProcessSomeip();

        now = xTaskGetTickCount();
        if((TickType_t)(now - last_tx_tick) >= pdMS_TO_TICKS(APP_DRIVESERVICE_TX_PERIOD_MS))
        {
            last_tx_tick = now;

            AppDriveService_GetCommand(&command);
            stop_cmd = AppAebService_GetStopCmd();

            (void)AppDriveService_SendControlCmd(command.drive_cmd,
                                                 command.steering_cmd,
                                                 stop_cmd);
        }

        vTaskDelay(pdMS_TO_TICKS(APP_DRIVESERVICE_TASK_PERIOD_MS));
    }
}

static void AppDriveService_ProcessSomeip(void)
{
    AppSomeipRxMsg rx_msg;
    uint8_t i;

    for(i = 0u; i < APP_DRIVESERVICE_SOMEIP_DRAIN_LIMIT; i++)
    {
        if(AppSomeip_Recv(g_drive_service_someip_rx_queue, &rx_msg) != pdPASS)
        {
            break;
        }

        AppDriveService_HandleSomeipMessage(&rx_msg);
    }
}

static void AppDriveService_HandleSomeipMessage(const AppSomeipRxMsg *rx_msg)
{
    if(rx_msg == NULL)
    {
        return;
    }

    if(rx_msg->packet.payload_len < 2u)
    {
        return;
    }

    AppDriveService_SetCommand(rx_msg->packet.payload_arr[0],
                               rx_msg->packet.payload_arr[1]);
}

static BaseType_t AppDriveService_SendControlCmd(AppDriveServiceDriveCmd drive_cmd,
                                                 AppDriveServiceSteeringCmd steering_cmd,
                                                 AppAebServiceStopCmd stop_cmd)
{
    uint8_t payload[APP_DRIVESERVICE_CAN_DLC_CONTROL_CMD];

    payload[0] = drive_cmd;
    payload[1] = steering_cmd;
    payload[2] = (uint8_t)stop_cmd;

    return AppCan_SendClassic(APP_DRIVESERVICE_CAN_ID_CONTROL_CMD,
                              payload,
                              APP_DRIVESERVICE_CAN_DLC_CONTROL_CMD);
}
