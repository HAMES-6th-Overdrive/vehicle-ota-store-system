#include "SensorOtaEraseWorker.h"
#include "SensorOtaFlash.h"

typedef struct
{
    volatile SensorOtaEraseWorker_State_t state;

    volatile uint32 addr;
    volatile uint32 size;
    volatile IfxFlash_FlashType flashType;

    volatile uint32 requestCount;
    volatile uint32 busyCount;
    volatile uint32 doneCount;
    volatile uint32 errorCount;

    volatile uint32 lastAddr;
    volatile uint32 lastSize;
    volatile uint32 lastFlashType;
    volatile uint32 lastResult;
} SensorOtaEraseWorker_Context_t;

static SensorOtaEraseWorker_Context_t g_eraseWorker;

/* Watch 확인용 */
volatile uint32 g_sensorOtaEraseWorkerState = 0U;
volatile uint32 g_sensorOtaEraseWorkerRequestCount = 0U;
volatile uint32 g_sensorOtaEraseWorkerBusyCount = 0U;
volatile uint32 g_sensorOtaEraseWorkerDoneCount = 0U;
volatile uint32 g_sensorOtaEraseWorkerErrorCount = 0U;
volatile uint32 g_sensorOtaEraseWorkerLastAddr = 0U;
volatile uint32 g_sensorOtaEraseWorkerLastSize = 0U;
volatile uint32 g_sensorOtaEraseWorkerLastFlashType = 0U;
volatile uint32 g_sensorOtaEraseWorkerLastResult = 0U;

static void updateDebugMirror(void)
{
    g_sensorOtaEraseWorkerState = (uint32)g_eraseWorker.state;
    g_sensorOtaEraseWorkerRequestCount = g_eraseWorker.requestCount;
    g_sensorOtaEraseWorkerBusyCount = g_eraseWorker.busyCount;
    g_sensorOtaEraseWorkerDoneCount = g_eraseWorker.doneCount;
    g_sensorOtaEraseWorkerErrorCount = g_eraseWorker.errorCount;
    g_sensorOtaEraseWorkerLastAddr = g_eraseWorker.lastAddr;
    g_sensorOtaEraseWorkerLastSize = g_eraseWorker.lastSize;
    g_sensorOtaEraseWorkerLastFlashType = g_eraseWorker.lastFlashType;
    g_sensorOtaEraseWorkerLastResult = g_eraseWorker.lastResult;
}

void SensorOtaEraseWorker_Init(void)
{
    g_eraseWorker.state = SENSOR_OTA_ERASE_WORKER_IDLE;

    g_eraseWorker.addr = 0U;
    g_eraseWorker.size = 0U;
    g_eraseWorker.flashType = IfxFlash_FlashType_P0;

    g_eraseWorker.requestCount = 0U;
    g_eraseWorker.busyCount = 0U;
    g_eraseWorker.doneCount = 0U;
    g_eraseWorker.errorCount = 0U;

    g_eraseWorker.lastAddr = 0U;
    g_eraseWorker.lastSize = 0U;
    g_eraseWorker.lastFlashType = 0U;
    g_eraseWorker.lastResult = 0U;

    updateDebugMirror();
}

boolean SensorOtaEraseWorker_Request(uint32 addr,
                                      uint32 size,
                                      IfxFlash_FlashType flashType)
{
    SensorOtaEraseWorker_State_t state;

    state = g_eraseWorker.state;

    if((state == SENSOR_OTA_ERASE_WORKER_REQUESTED) ||
       (state == SENSOR_OTA_ERASE_WORKER_BUSY))
    {
        return FALSE;
    }

    /*
     * 요청 파라미터를 먼저 채우고,
     * 마지막에 state를 REQUESTED로 바꾼다.
     */
    g_eraseWorker.addr = addr;
    g_eraseWorker.size = size;
    g_eraseWorker.flashType = flashType;

    g_eraseWorker.lastAddr = addr;
    g_eraseWorker.lastSize = size;
    g_eraseWorker.lastFlashType = (uint32)flashType;
    g_eraseWorker.lastResult = 0U;

    g_eraseWorker.requestCount++;

    g_eraseWorker.state = SENSOR_OTA_ERASE_WORKER_REQUESTED;

    updateDebugMirror();

    return TRUE;
}

void SensorOtaEraseWorker_Service(void)
{
    uint32 addr;
    uint32 size;
    IfxFlash_FlashType flashType;
    boolean result;

    if(g_eraseWorker.state != SENSOR_OTA_ERASE_WORKER_REQUESTED)
    {
        updateDebugMirror();
        return;
    }

    g_eraseWorker.state = SENSOR_OTA_ERASE_WORKER_BUSY;
    g_eraseWorker.busyCount++;

    updateDebugMirror();

    addr = g_eraseWorker.addr;
    size = g_eraseWorker.size;
    flashType = g_eraseWorker.flashType;

    result = SensorOtaFlash_EraseNoCoreHalt(addr, size, flashType);

    if(result == TRUE)
    {
        g_eraseWorker.lastResult = 1U;
        g_eraseWorker.doneCount++;
        g_eraseWorker.state = SENSOR_OTA_ERASE_WORKER_DONE;
    }
    else
    {
        g_eraseWorker.lastResult = 0U;
        g_eraseWorker.errorCount++;
        g_eraseWorker.state = SENSOR_OTA_ERASE_WORKER_ERROR;
    }

    updateDebugMirror();
}

SensorOtaEraseWorker_State_t SensorOtaEraseWorker_GetState(void)
{
    return g_eraseWorker.state;
}

boolean SensorOtaEraseWorker_IsBusy(void)
{
    boolean ret = FALSE;

    if((g_eraseWorker.state == SENSOR_OTA_ERASE_WORKER_REQUESTED) ||
       (g_eraseWorker.state == SENSOR_OTA_ERASE_WORKER_BUSY))
    {
        ret = TRUE;
    }

    return ret;
}

boolean SensorOtaEraseWorker_IsDone(void)
{
    return (g_eraseWorker.state == SENSOR_OTA_ERASE_WORKER_DONE) ? TRUE : FALSE;
}

boolean SensorOtaEraseWorker_IsError(void)
{
    return (g_eraseWorker.state == SENSOR_OTA_ERASE_WORKER_ERROR) ? TRUE : FALSE;
}

void SensorOtaEraseWorker_ClearResult(void)
{
    if((g_eraseWorker.state == SENSOR_OTA_ERASE_WORKER_DONE) ||
       (g_eraseWorker.state == SENSOR_OTA_ERASE_WORKER_ERROR))
    {
        g_eraseWorker.state = SENSOR_OTA_ERASE_WORKER_IDLE;
        g_eraseWorker.addr = 0U;
        g_eraseWorker.size = 0U;
        g_eraseWorker.flashType = IfxFlash_FlashType_P0;
    }

    updateDebugMirror();
}

uint32 SensorOtaEraseWorker_GetLastAddr(void)
{
    return g_eraseWorker.lastAddr;
}

uint32 SensorOtaEraseWorker_GetLastSize(void)
{
    return g_eraseWorker.lastSize;
}

uint32 SensorOtaEraseWorker_GetDoneCount(void)
{
    return g_eraseWorker.doneCount;
}

uint32 SensorOtaEraseWorker_GetErrorCount(void)
{
    return g_eraseWorker.errorCount;
}
