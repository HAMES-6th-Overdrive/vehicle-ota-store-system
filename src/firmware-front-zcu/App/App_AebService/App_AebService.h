#ifndef APP_AEBSERVICE_H
#define APP_AEBSERVICE_H

#include "FreeRTOS.h"
#include "queue.h"
#include <stdint.h>

typedef enum AppAebServiceStopCmd {
    APP_AEBSERVICE_STOP_CMD_GO = 0x00u,
    APP_AEBSERVICE_STOP_CMD_STOP = 0x01u
} AppAebServiceStopCmd;

BaseType_t AppAebService_Start(void);
QueueHandle_t AppAebService_GetSomeipRxQueue(void);
AppAebServiceStopCmd AppAebService_GetStopCmd(void);

#endif /* APP_AEBSERVICE_H */
