#ifndef APP_DIAGCORE1_H
#define APP_DIAGCORE1_H

#include "Ifx_Types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Core0 <-> Core1 diagnostic bridge.
 *
 * Intended ownership:
 *  - Core0: Ethernet/lwIP/DoIP socket only
 *  - Core1: UDS service, flash erase/write, DFLASH flag, UCB/SOTA handling
 *
 * NOTE:
 *  g_diagCore1 is currently just aligned as a global symbol. For production,
 *  place this object in LMU/non-cached shared RAM through the linker script.
 */
#define APP_DIAG_CORE1_RX_BUF_SIZE       (4096u)
#define APP_DIAG_CORE1_TX_BUF_SIZE       (512u)
#define APP_DIAG_CORE1_DEFAULT_TIMEOUT_MS (10000u)

typedef enum
{
    APP_DIAG_CORE1_STATE_IDLE = 0,
    APP_DIAG_CORE1_STATE_PENDING,
    APP_DIAG_CORE1_STATE_PROCESSING,
    APP_DIAG_CORE1_STATE_DONE,
    APP_DIAG_CORE1_STATE_ERROR
} AppDiagCore1State;

void    AppDiagCore1_Init(void);
void    AppDiagCore1_MainFunction(void);
boolean AppDiagCore1_RequestBlocking(const uint8 *rxData,
                                      uint16 rxLen,
                                      uint8 *txData,
                                      uint16 *txLen,
                                      uint32 timeoutMs);

uint32  AppDiagCore1_GetRequestCount(void);
uint32  AppDiagCore1_GetDoneCount(void);
uint32  AppDiagCore1_GetBusyCount(void);
uint32  AppDiagCore1_GetTimeoutCount(void);
uint32  AppDiagCore1_GetErrorCount(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_DIAGCORE1_H */
