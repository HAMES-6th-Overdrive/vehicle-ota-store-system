#include "App_VehicleService.h"
#include "App_Someip/App_Someip.h"
#include "someip/light_someip.h"
#include "App_Debug/App_Core1Debug.h"
#include "task.h"
#include "App_Can/App_Can.h"
#include <string.h>

#define APP_VEHICLESERVICE_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define APP_VEHICLESERVICE_TASK_PRIORITY   (tskIDLE_PRIORITY + 2u)
#define APP_VEHICLESERVICE_RX_QUEUE_SIZE   (8u)

#define APP_VEHICLESERVICE_SERVICE_ID      (0x0008u)

static QueueHandle_t g_vehicle_service_someip_rx_queue;

static BaseType_t AppVehicleService_Init(void);
static void AppVehicleService_Task(void *arg);

const uint8_t vehicleService_payload[APP_DRIVESERVICE_CAN_DLC] = {0x11, 0x01};

BaseType_t AppVehicleService_Start(void)
{
    if (AppVehicleService_Init() != pdPASS)
    {
        return pdFAIL;
    }

    return xTaskCreate(AppVehicleService_Task, "APP VEHICLE SERVICE", APP_VEHICLESERVICE_TASK_STACK_SIZE, NULL, APP_VEHICLESERVICE_TASK_PRIORITY, NULL);
}

QueueHandle_t AppVehicleService_GetSomeipRxQueue(void)
{
    return g_vehicle_service_someip_rx_queue;
}

static BaseType_t AppVehicleService_Init(void)
{
    if (g_vehicle_service_someip_rx_queue == NULL)
    {
        g_vehicle_service_someip_rx_queue = xQueueCreate(APP_VEHICLESERVICE_RX_QUEUE_SIZE, sizeof(AppSomeipRxMsg));
    }

    return (g_vehicle_service_someip_rx_queue != NULL) ? pdPASS : pdFAIL;
}

static void AppVehicleService_Task(void *arg)
{
    AppSomeipRxMsg rx_msg;

    (void)arg;

    for (;;)
    {
        if (AppSomeip_Recv(g_vehicle_service_someip_rx_queue, &rx_msg) == pdPASS)
        {
            AppCore1Debug_PushU32("method_id: ", rx_msg.packet.method_id);

            switch (rx_msg.packet.method_id) {
                case 0x2001: {
                    AppCore1Debug_Push("g_resetPending!");
                    g_resetPending = pdTRUE;            
                    __dsync();       
                    break;
                }

                case 0x2002: {
                    AppCan_SendClassic(APP_DRIVESERVICE_CAN_ID_RESET_DRIVE_ECU,
                                                vehicleService_payload,
                                                APP_DRIVESERVICE_CAN_DLC);                    
                    break;
                }
                
                case 0x2003: {
                    AppCan_SendClassic(APP_DRIVESERVICE_CAN_ID_RESET_SENSOR_ECU,
                                                vehicleService_payload,
                                                APP_DRIVESERVICE_CAN_DLC);                    
                    break;
                }
                
                default: {
                    break;
                }                
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1u));
    }
}