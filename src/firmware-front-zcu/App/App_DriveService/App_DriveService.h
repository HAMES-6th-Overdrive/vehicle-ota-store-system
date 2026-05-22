#ifndef APP_DRIVESERVICE_H
#define APP_DRIVESERVICE_H

#include "FreeRTOS.h"
#include "queue.h"

BaseType_t AppDriveService_Start(void);
QueueHandle_t AppDriveService_GetSomeipRxQueue(void);

#endif /* APP_DRIVESERVICE_H */
