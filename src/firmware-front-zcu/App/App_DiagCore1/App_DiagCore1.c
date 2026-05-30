#include "App_DiagCore1.h"

#include "App_Uds/App_Uds.h"
#include "FreeRTOS.h"
#include "task.h"
#include "IfxCpu.h"
#include "IfxScuRcu.h"
#include "App_Debug/App_Core1Debug.h"

#include <string.h>

typedef struct
{
    volatile AppDiagCore1State state;
    volatile uint16 rxLen;
    volatile uint16 txLen;
    volatile uint32 error;

    uint8 rx[APP_DIAG_CORE1_RX_BUF_SIZE];
    uint8 tx[APP_DIAG_CORE1_TX_BUF_SIZE];
} AppDiagCore1Shared;

/*
 * For TC375 multi-core use, this object should be mapped to LMU/non-cached RAM.
 * Keep the symbol global/aligned so it is easy to place by linker section later.
 */
IFX_ALIGN(32) volatile AppDiagCore1Shared g_diagCore1;

static volatile uint32 g_req_count = 0u;
static volatile uint32 g_done_count = 0u;
static volatile uint32 g_busy_count = 0u;
static volatile uint32 g_timeout_count = 0u;
static volatile uint32 g_error_count = 0u;
volatile boolean g_resetPending = pdFALSE;

static boolean AppDiagCore1_WaitForIdle(uint32 timeoutMs);
static boolean AppDiagCore1_WaitForDone(uint32 timeoutMs);
static void AppDiagCore1_DelayBeforeReset(void);

void AppDiagCore1_Init(void)
{
    memset((void *)&g_diagCore1, 0, sizeof(g_diagCore1));
    g_diagCore1.state = APP_DIAG_CORE1_STATE_IDLE;
    __dsync();
}

boolean AppDiagCore1_TryStartRequest(const uint8 *rxData,
                                      uint16 rxLen)
{
    if ((rxData == NULL) ||
        (rxLen == 0u) || (rxLen > APP_DIAG_CORE1_RX_BUF_SIZE))
    {
        g_error_count++;
        return FALSE;
    }

    if (g_diagCore1.state != APP_DIAG_CORE1_STATE_IDLE)
    {
        g_busy_count++;
        return FALSE;
    }

    memcpy((void *)g_diagCore1.rx, rxData, rxLen);
    g_diagCore1.rxLen = rxLen;
    g_diagCore1.txLen = 0u;
    g_diagCore1.error = 0u;
    __dsync();

    g_diagCore1.state = APP_DIAG_CORE1_STATE_PENDING;
    g_req_count++;
    __dsync();

    return TRUE;
}

AppDiagCore1ResponseStatus AppDiagCore1_TryReadResponse(uint8 *txData,
                                                        uint16 *txLen)
{
    if ((txData == NULL) || (txLen == NULL))
    {
        g_error_count++;
        return APP_DIAG_CORE1_RESPONSE_ERROR;
    }

    __dsync();

    if (g_diagCore1.state == APP_DIAG_CORE1_STATE_ERROR)
    {
        *txLen = 0u;
        return APP_DIAG_CORE1_RESPONSE_ERROR;
    }

    if (g_diagCore1.state != APP_DIAG_CORE1_STATE_DONE)
    {
        return APP_DIAG_CORE1_RESPONSE_NOT_READY;
    }

    if (g_diagCore1.txLen > APP_DIAG_CORE1_TX_BUF_SIZE)
    {
        *txLen = 0u;
        g_diagCore1.error = 1u;
        g_diagCore1.state = APP_DIAG_CORE1_STATE_ERROR;
        g_error_count++;
        __dsync();
        return APP_DIAG_CORE1_RESPONSE_ERROR;
    }

    *txLen = g_diagCore1.txLen;
    if (*txLen > 0u)
    {
        memcpy(txData, (const void *)g_diagCore1.tx, *txLen);
    }

    return APP_DIAG_CORE1_RESPONSE_READY;
}

void AppDiagCore1_ReleaseResponse(void)
{
    AppDiagCore1State state;

    __dsync();
    state = g_diagCore1.state;

    if ((state != APP_DIAG_CORE1_STATE_DONE) &&
        (state != APP_DIAG_CORE1_STATE_ERROR))
    {
        return;
    }

    g_diagCore1.rxLen = 0u;
    g_diagCore1.txLen = 0u;
    g_diagCore1.error = 0u;
    g_diagCore1.state = APP_DIAG_CORE1_STATE_IDLE;

    if (state == APP_DIAG_CORE1_STATE_DONE)
    {
        g_done_count++;
    }

    __dsync();
}

boolean AppDiagCore1_RequestBlocking(const uint8 *rxData,
                                      uint16 rxLen,
                                      uint8 *txData,
                                      uint16 *txLen,
                                      uint32 timeoutMs)
{
    AppDiagCore1ResponseStatus responseStatus;

    if ((txData == NULL) || (txLen == NULL))
    {
        g_error_count++;
        return FALSE;
    }

    if (timeoutMs == 0u)
    {
        timeoutMs = APP_DIAG_CORE1_DEFAULT_TIMEOUT_MS;
    }

    if (AppDiagCore1_WaitForIdle(timeoutMs) != TRUE)
    {
        g_busy_count++;
        return FALSE;
    }

    if (AppDiagCore1_TryStartRequest(rxData, rxLen) != TRUE)
    {
        return FALSE;
    }

    if (AppDiagCore1_WaitForDone(timeoutMs) != TRUE)
    {
        g_timeout_count++;
        return FALSE;
    }

    responseStatus = AppDiagCore1_TryReadResponse(txData, txLen);
    if (responseStatus != APP_DIAG_CORE1_RESPONSE_READY)
    {
        AppDiagCore1_ReleaseResponse();
        return FALSE;
    }

    AppDiagCore1_ReleaseResponse();

    return TRUE;
}

void AppDiagCore1_MainFunction(void)
{
    uint16 localTxLen = 0u;
    uint32 waitLoop;

    if (g_diagCore1.state != APP_DIAG_CORE1_STATE_PENDING)
    {
        return;
    }

    g_diagCore1.state = APP_DIAG_CORE1_STATE_PROCESSING;
    __dsync();

    UDS_HandleService((uint8 *)g_diagCore1.rx,
                      g_diagCore1.rxLen,
                      (uint8 *)g_diagCore1.tx,
                      &localTxLen);

    if (localTxLen > APP_DIAG_CORE1_TX_BUF_SIZE)
    {
        g_diagCore1.txLen = 0u;
        g_diagCore1.error = 1u;
        g_diagCore1.state = APP_DIAG_CORE1_STATE_ERROR;
        g_error_count++;
        __dsync();
        return;
    }

    g_diagCore1.txLen = localTxLen;

    __dsync();
    g_diagCore1.state = APP_DIAG_CORE1_STATE_DONE;
    __dsync();

    if (g_resetPending)
    {
        AppCore1Debug_Push("Reset Pended!");
        /*
         * Core0 must copy and transmit the DoIP positive response first.
         * Wait until Core0 changes DONE -> IDLE, then reset.
         */
        for (waitLoop = 0u; waitLoop < 5000000u; waitLoop++)
        {
            if (g_diagCore1.state != APP_DIAG_CORE1_STATE_DONE)
            {
                break;
            }
            __nop();
        }

        AppCore1Debug_Push("OverDrive Front-ZCU Update completed!");

        AppDiagCore1_DelayBeforeReset();
        IfxScuRcu_performReset(IfxScuRcu_ResetType_system, 0);
    }
}

uint32 AppDiagCore1_GetRequestCount(void) { return g_req_count; }
uint32 AppDiagCore1_GetDoneCount(void) { return g_done_count; }
uint32 AppDiagCore1_GetBusyCount(void) { return g_busy_count; }
uint32 AppDiagCore1_GetTimeoutCount(void) { return g_timeout_count; }
uint32 AppDiagCore1_GetErrorCount(void) { return g_error_count; }

static boolean AppDiagCore1_WaitForIdle(uint32 timeoutMs)
{
    TickType_t start = xTaskGetTickCount();
    TickType_t timeoutTicks = pdMS_TO_TICKS(timeoutMs);

    while (g_diagCore1.state != APP_DIAG_CORE1_STATE_IDLE)
    {
        if ((TickType_t)(xTaskGetTickCount() - start) >= timeoutTicks)
        {
            return FALSE;
        }
        vTaskDelay(pdMS_TO_TICKS(1u));
    }

    return TRUE;
}

static boolean AppDiagCore1_WaitForDone(uint32 timeoutMs)
{
    TickType_t start = xTaskGetTickCount();
    TickType_t timeoutTicks = pdMS_TO_TICKS(timeoutMs);

    while ((g_diagCore1.state != APP_DIAG_CORE1_STATE_DONE) &&
           (g_diagCore1.state != APP_DIAG_CORE1_STATE_ERROR))
    {
        if ((TickType_t)(xTaskGetTickCount() - start) >= timeoutTicks)
        {
            return FALSE;
        }
        vTaskDelay(pdMS_TO_TICKS(1u));
    }

    return TRUE;
}

static void AppDiagCore1_DelayBeforeReset(void)
{
    volatile uint32 i;

    for (i = 0u; i < 3000000u; i++)
    {
        __nop();
    }
}
