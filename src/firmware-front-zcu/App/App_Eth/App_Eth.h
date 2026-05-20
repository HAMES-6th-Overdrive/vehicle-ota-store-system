#ifndef APP_ETH_H
#define APP_ETH_H

#include "FreeRTOS.h"
#include "task.h"

BaseType_t AppEth_Start(void);
BaseType_t AppEth_IsReady(void);

#endif /* APP_ETH_H */
