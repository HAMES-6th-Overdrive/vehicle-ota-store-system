#ifndef APP_VEHICLESERVICE_H
#define APP_VEHICLESERVICE_H

#include "FreeRTOS.h"
#include "queue.h"
#include "Ifx_Types.h"

#define APP_DRIVESERVICE_CAN_DLC 2
#define APP_DRIVESERVICE_CAN_ID_RESET_DRIVE_ECU 0x610
#define APP_DRIVESERVICE_CAN_ID_RESET_SENSOR_ECU 0x600

BaseType_t AppVehicleService_Start(void);
QueueHandle_t AppVehicleService_GetSomeipRxQueue(void);

extern volatile boolean g_resetPending;

#endif /* APP_VEHICLESERVICE_H */