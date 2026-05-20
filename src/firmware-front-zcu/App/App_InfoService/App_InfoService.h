#ifndef APP_INFOSERVICE_H
#define APP_INFOSERVICE_H

#include "FreeRTOS.h"
#include "queue.h"

BaseType_t AppInfoService_Start(void);
QueueHandle_t AppInfoService_GetSomeipRxQueue(void);

#endif /* APP_INFOSERVICE_H */
