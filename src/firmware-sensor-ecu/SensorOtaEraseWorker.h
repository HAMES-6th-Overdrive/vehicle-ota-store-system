#ifndef SENSOR_OTA_ERASE_WORKER_H_
#define SENSOR_OTA_ERASE_WORKER_H_

#include "Ifx_Types.h"
#include "IfxFlash.h"

typedef enum
{
    SENSOR_OTA_ERASE_WORKER_IDLE = 0,
    SENSOR_OTA_ERASE_WORKER_REQUESTED,
    SENSOR_OTA_ERASE_WORKER_BUSY,
    SENSOR_OTA_ERASE_WORKER_DONE,
    SENSOR_OTA_ERASE_WORKER_ERROR
} SensorOtaEraseWorker_State_t;

void SensorOtaEraseWorker_Init(void);

boolean SensorOtaEraseWorker_Request(uint32 addr,
                                      uint32 size,
                                      IfxFlash_FlashType flashType);

void SensorOtaEraseWorker_Service(void);

SensorOtaEraseWorker_State_t SensorOtaEraseWorker_GetState(void);

boolean SensorOtaEraseWorker_IsBusy(void);
boolean SensorOtaEraseWorker_IsDone(void);
boolean SensorOtaEraseWorker_IsError(void);

void SensorOtaEraseWorker_ClearResult(void);

uint32 SensorOtaEraseWorker_GetLastAddr(void);
uint32 SensorOtaEraseWorker_GetLastSize(void);
uint32 SensorOtaEraseWorker_GetDoneCount(void);
uint32 SensorOtaEraseWorker_GetErrorCount(void);

#endif /* SENSOR_OTA_ERASE_WORKER_H_ */
