#ifndef APP_VEHICLESERVICE_H
#define APP_VEHICLESERVICE_H

#include "FreeRTOS.h"
#include "queue.h"
#include "Ifx_Types.h"

BaseType_t AppVehicleService_Start(void);
QueueHandle_t AppVehicleService_GetSomeipRxQueue(void);

extern volatile boolean g_resetPending;

#endif /* APP_VEHICLESERVICE_H */