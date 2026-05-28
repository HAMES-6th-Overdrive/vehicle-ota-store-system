/**********************************************************************************************************************
 * \file UdsOtaClient.c
 * \brief ZCU UDS-style OTA Client over CAN FD - Streaming only version for FreeRTOS App_Can
 *
 * 역할:
 *  - ZCU가 Sensor ECU로 UDS OTA Request 송신
 *  - Sensor ECU의 0x601 UDS Response 수신 후 상태머신 진행
 *
 * 구조:
 *  - UdsOtaClient는 MCMCAN/CanIf를 직접 사용하지 않는다.
 *  - 팀원 RTOS 구조의 App_Can API를 사용한다.
 *
 * CAN:
 *  - TX: 0x600 UDS Request  ZCU -> Sensor ECU, CAN FD
 *  - RX: 0x601 UDS Response Sensor ECU -> ZCU, CAN FD
 *
 * 주의:
 *  - UdsOtaClient_OnResponse()는 response copy + flag set만 수행한다.
 *  - 실제 다음 request 송신과 timeout 처리는 UdsOtaClient_MainFunction()에서 수행한다.
 *
 * Streaming Gateway 구조:
 *  - ZCU는 전체 firmware binary를 저장하지 않는다.
 *  - ZCU는 firmwareSize / 현재 32-byte block만 보관한다.
 *  - CRC32는 두 가지 모드를 지원한다.
 *
 * CRC 모드:
 *  1. 기존 모드
 *     - UdsOtaClient_StartStream(firmwareSize, crc32)
 *     - OTA 시작 시점에 CRC32를 이미 알고 있다.
 *
 *  2. Late CRC 모드
 *     - UdsOtaClient_StartStreamWithoutCrc(firmwareSize)
 *     - Pi/HPC -> ZCU DoIP 흐름처럼 CRC32가 마지막 0x37에서 들어오는 경우 사용한다.
 *     - 모든 block 전송 완료 후 WAIT_FINAL_CRC 상태에서 대기한다.
 *     - 이후 UdsOtaClient_SetFinalCrc(crc32)가 호출되면
 *       Sensor ECU 쪽 RequestTransferExit + RoutineControl CRC를 진행한다.
 *
 * Download Phase:
 *  - 0x10 DiagnosticSessionControl
 *  - 0x34 RequestDownload
 *  - 0x36 TransferData
 *  - 0x37 RequestTransferExit
 *  - 0x31 RoutineControl CRC32
 *  - CRC 성공 후 DONE
 *********************************************************************************************************************/

#include "UdsOtaClient.h"

#include "App_Can/App_Can.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

/* ============================================================
   Fallback UDS define
   can_type_def.h에 이미 있으면 아래 define은 사용되지 않음
   ============================================================ */

#ifndef UDS_SID_DIAGNOSTIC_SESSION_CONTROL
#define UDS_SID_DIAGNOSTIC_SESSION_CONTROL  0x10U
#endif

#ifndef UDS_SID_ECU_RESET
#define UDS_SID_ECU_RESET                   0x11U
#endif

#ifndef UDS_SID_ROUTINE_CONTROL
#define UDS_SID_ROUTINE_CONTROL             0x31U
#endif

#ifndef UDS_SID_REQUEST_DOWNLOAD
#define UDS_SID_REQUEST_DOWNLOAD            0x34U
#endif

#ifndef UDS_SID_TRANSFER_DATA
#define UDS_SID_TRANSFER_DATA               0x36U
#endif

#ifndef UDS_SID_REQUEST_TRANSFER_EXIT
#define UDS_SID_REQUEST_TRANSFER_EXIT       0x37U
#endif

#ifndef UDS_SID_NEGATIVE_RESPONSE
#define UDS_SID_NEGATIVE_RESPONSE           0x7FU
#endif

#ifndef UDS_POSITIVE_RESPONSE_OFFSET
#define UDS_POSITIVE_RESPONSE_OFFSET        0x40U
#endif

#ifndef UDS_SESSION_PROGRAMMING
#define UDS_SESSION_PROGRAMMING             0x02U
#endif

#ifndef UDS_DOWNLOAD_DATA_FORMAT_ID
#define UDS_DOWNLOAD_DATA_FORMAT_ID         0x00U
#endif

#ifndef UDS_DOWNLOAD_ADDR_LEN_FORMAT
#define UDS_DOWNLOAD_ADDR_LEN_FORMAT        0x44U
#endif

#ifndef UDS_ROUTINE_START
#define UDS_ROUTINE_START                   0x01U
#endif

#ifndef UDS_ROUTINE_ID_CHECK_CRC32
#define UDS_ROUTINE_ID_CHECK_CRC32          0x0202U
#endif

#ifndef UDS_RESET_JUMP_TO_APP
#define UDS_RESET_JUMP_TO_APP               0x01U
#endif

#ifndef UDS_REQ_LEN_DIAGNOSTIC_SESSION_CONTROL
#define UDS_REQ_LEN_DIAGNOSTIC_SESSION_CONTROL   2U
#endif

#ifndef UDS_REQ_LEN_REQUEST_DOWNLOAD
#define UDS_REQ_LEN_REQUEST_DOWNLOAD             11U
#endif

#ifndef UDS_REQ_LEN_TRANSFER_DATA_MIN
#define UDS_REQ_LEN_TRANSFER_DATA_MIN            3U
#endif

#ifndef UDS_REQ_LEN_REQUEST_TRANSFER_EXIT
#define UDS_REQ_LEN_REQUEST_TRANSFER_EXIT        1U
#endif

#ifndef UDS_REQ_LEN_ROUTINE_CONTROL_CHECK_CRC32
#define UDS_REQ_LEN_ROUTINE_CONTROL_CHECK_CRC32  8U
#endif

#ifndef UDS_REQ_LEN_ECU_RESET
#define UDS_REQ_LEN_ECU_RESET                    2U
#endif


/* ============================================================
   OTA behavior option
   ============================================================ */

/*
 * Dual Slot / Bank B OTA 구조에서는
 * CRC 검증 성공 후 바로 ECUReset(0x11)을 보내지 않는다.
 *
 * 0U:
 *   Download Phase는 CRC 검증에서 DONE 처리.
 *   Activation은 사용자 승인 후 별도 Routine/Slot switch로 수행.
 *
 * 1U:
 *   기존 Single Slot 방식처럼 CRC 성공 후 ECUReset(0x11) 자동 송신.
 */
#define UDS_OTA_CLIENT_AUTO_RESET_AFTER_CRC    0U


/* ============================================================
   Internal state
   ============================================================ */

static UdsOtaClient_DebugInfo_t g_clientDebug;

static uint32_t g_firmwareSize = 0U;
static uint32_t g_firmwareCrc32 = 0U;

/*
 * CRC 제공 여부.
 *
 * 기존 모드:
 *  - StartStream(size, crc)에서 TRUE
 *
 * Late CRC 모드:
 *  - StartStreamWithoutCrc(size)에서 FALSE
 *  - 모든 block 전송 후 WAIT_FINAL_CRC에서 대기
 *  - SetFinalCrc(crc) 호출 시 TRUE
 */
static boolean g_finalCrcProvided = FALSE;

static volatile boolean g_responsePending = FALSE;
static uint8_t g_responseData[CANFD_MAX_DLC];
static uint8_t g_responseLength = 0U;

/*
 * Streaming block buffer
 *
 * ZCU는 전체 firmware를 저장하지 않는다.
 * 현재 요청된 block만 이 buffer에 복사한 뒤 0x36 TransferData로 보낸다.
 */
static uint8_t g_streamBlockBuffer[UDS_OTA_CLIENT_TRANSFER_DATA_SIZE];
static uint8_t g_streamBlockLength = 0U;
static boolean g_streamBlockReady = FALSE;

/* debug counter */
volatile uint32_t g_dbgTdRespEnterCount = 0U;
volatile uint32_t g_dbgTdRespFetchOkCount = 0U;
volatile uint32_t g_dbgTdRespTimeoutCount = 0U;
volatile uint32_t g_dbgTdRespSidMismatchCount = 0U;
volatile uint32_t g_dbgTdRespBscMismatchCount = 0U;
volatile uint32_t g_dbgTdRespLenFailCount = 0U;
volatile uint32_t g_dbgTdRespSuccessCount = 0U;

volatile uint8_t g_dbgTdRespLen = 0U;
volatile uint8_t g_dbgTdResp0 = 0U;
volatile uint8_t g_dbgTdResp1 = 0U;
volatile uint8_t g_dbgTdExpectedBsc = 0U;

/* ============================================================
   Private prototypes
   ============================================================ */

static void setState(UdsOtaClient_State_t state);
static void setError(UdsOtaClient_Result_t result);

static void clearResponse(void);
static boolean fetchResponse(uint8_t *buffer, uint8_t *length);

static uint8_t positiveResponseSid(uint8_t requestSid);

static uint16_t readU16Le(const uint8_t *p);
static uint32_t readU32Le(const uint8_t *p);
static void writeU16Le(uint8_t *p, uint16_t value);
static void writeU32Le(uint8_t *p, uint32_t value);

static void makePayload(uint8_t *p, uint8_t fill);
static boolean sendPayload(const uint8_t *payload, uint8_t length);

static void sendDiagnosticSessionControl(void);
static void handleDiagnosticSessionResponse(void);

static void sendRequestDownload(void);
static void handleRequestDownloadResponse(void);

static void sendTransferData(void);
static void handleTransferDataResponse(void);

static void sendRequestTransferExit(void);
static void handleRequestTransferExitResponse(void);

static void sendRoutineControlCrc(void);
static void handleRoutineControlCrcResponse(void);

static void sendEcuReset(void);
static void handleEcuResetResponse(void);

static boolean checkTimeout(uint32_t timeoutTicks);

static uint32_t calcTotalBlocks(uint32_t size);
static uint8_t calcCurrentBlockLength(void);

static void proceedAfterAllBlocksSent(void);
static void updateFinalCrcDebugFlag(void);


/* ============================================================
   Public API
   ============================================================ */

void UdsOtaClient_Init(void)
{
    UdsOtaClient_Reset();
}


void UdsOtaClient_Reset(void)
{
    taskENTER_CRITICAL();

    memset(&g_clientDebug, 0, sizeof(g_clientDebug));
    memset(g_responseData, 0, sizeof(g_responseData));

    g_firmwareSize = 0U;
    g_firmwareCrc32 = 0U;
    g_finalCrcProvided = FALSE;

    g_responsePending = FALSE;
    g_responseLength = 0U;

    memset(g_streamBlockBuffer, 0, sizeof(g_streamBlockBuffer));
    g_streamBlockLength = 0U;
    g_streamBlockReady = FALSE;

    g_clientDebug.state = UDS_OTA_CLIENT_STATE_IDLE;
    g_clientDebug.lastResult = UDS_OTA_CLIENT_RESULT_OK;
    g_clientDebug.targetAddress = UDS_OTA_CLIENT_TARGET_APP_ADDR;
    g_clientDebug.finalCrcProvided = g_finalCrcProvided;

    taskEXIT_CRITICAL();
}


/*
 * Streaming mode 시작 - CRC known.
 *
 * ZCU는 전체 firmware buffer를 넘기지 않는다.
 * firmwareSize와 crc32만 알고 시작한다.
 *
 * 이후 UdsOtaClient가 WAIT_STREAM_BLOCK 상태가 되면
 * 상위 계층이 UdsOtaClient_ProvideStreamBlock()으로 현재 block을 제공한다.
 */
UdsOtaClient_Result_t UdsOtaClient_StartStream(uint32_t firmwareSize,
                                               uint32_t crc32)
{
    if (firmwareSize == 0U)
    {
        return UDS_OTA_CLIENT_RESULT_INVALID_PARAM;
    }

    if (UdsOtaClient_IsBusy() == TRUE)
    {
        return UDS_OTA_CLIENT_RESULT_BUSY;
    }

    UdsOtaClient_Reset();

    g_firmwareSize = firmwareSize;
    g_firmwareCrc32 = crc32;
    g_finalCrcProvided = TRUE;

    g_clientDebug.firmwareSize = firmwareSize;
    g_clientDebug.firmwareCrc32 = crc32;
    g_clientDebug.targetAddress = UDS_OTA_CLIENT_TARGET_APP_ADDR;
    g_clientDebug.totalBlocks = calcTotalBlocks(firmwareSize);
    g_clientDebug.currentBlockIndex = 0U;
    g_clientDebug.currentOffset = 0U;
    g_clientDebug.sentBytes = 0U;
    g_clientDebug.currentBsc = 0x01U;
    g_clientDebug.lastProgressPercent = 0U;
    updateFinalCrcDebugFlag();

    setState(UDS_OTA_CLIENT_STATE_SEND_DIAGNOSTIC_SESSION);

    return UDS_OTA_CLIENT_RESULT_OK;
}


/*
 * Streaming mode 시작 - CRC later.
 *
 * Pi/HPC -> ZCU DoIP 흐름에서 CRC32가 마지막 0x37에서 들어오는 경우 사용한다.
 *
 * 모든 block 전송 완료 후 WAIT_FINAL_CRC 상태에서 대기한다.
 * 이후 UdsOtaClient_SetFinalCrc(crc32)가 호출되면
 * RequestTransferExit + RoutineControl CRC를 진행한다.
 */
UdsOtaClient_Result_t UdsOtaClient_StartStreamWithoutCrc(uint32_t firmwareSize)
{
    if (firmwareSize == 0U)
    {
        return UDS_OTA_CLIENT_RESULT_INVALID_PARAM;
    }

    if (UdsOtaClient_IsBusy() == TRUE)
    {
        return UDS_OTA_CLIENT_RESULT_BUSY;
    }

    UdsOtaClient_Reset();

    g_firmwareSize = firmwareSize;
    g_firmwareCrc32 = 0U;
    g_finalCrcProvided = FALSE;

    g_clientDebug.firmwareSize = firmwareSize;
    g_clientDebug.firmwareCrc32 = 0U;
    g_clientDebug.targetAddress = UDS_OTA_CLIENT_TARGET_APP_ADDR;
    g_clientDebug.totalBlocks = calcTotalBlocks(firmwareSize);
    g_clientDebug.currentBlockIndex = 0U;
    g_clientDebug.currentOffset = 0U;
    g_clientDebug.sentBytes = 0U;
    g_clientDebug.currentBsc = 0x01U;
    g_clientDebug.lastProgressPercent = 0U;
    updateFinalCrcDebugFlag();

    setState(UDS_OTA_CLIENT_STATE_SEND_DIAGNOSTIC_SESSION);

    return UDS_OTA_CLIENT_RESULT_OK;
}


/*
 * Late CRC mode에서 최종 CRC32 설정.
 *
 * 호출 조건:
 *  - 모든 block 전송 완료
 *  - UdsOtaClient_IsWaitingFinalCrc() == TRUE
 *
 * 주의:
 *  - crc32 == 0x00000000도 이론상 유효한 CRC일 수 있으므로 reject하지 않는다.
 */
UdsOtaClient_Result_t UdsOtaClient_SetFinalCrc(uint32_t crc32)
{
    if (g_clientDebug.state != UDS_OTA_CLIENT_STATE_WAIT_FINAL_CRC)
    {
        return UDS_OTA_CLIENT_RESULT_ERROR;
    }

    g_firmwareCrc32 = crc32;
    g_finalCrcProvided = TRUE;

    g_clientDebug.firmwareCrc32 = crc32;
    updateFinalCrcDebugFlag();

    /*
     * 이제 Sensor ECU 쪽으로 RequestTransferExit를 보내고,
     * 이후 RoutineControl CRC를 진행한다.
     */
    setState(UDS_OTA_CLIENT_STATE_SEND_REQUEST_TRANSFER_EXIT);

    return UDS_OTA_CLIENT_RESULT_OK;
}


/*
 * 현재 요청된 stream block을 제공한다.
 *
 * 호출 조건:
 *  - UdsOtaClient_IsWaitingStreamBlock() == TRUE
 *  - blockIndex == UdsOtaClient_GetRequestedBlockIndex()
 *  - length == UdsOtaClient_GetRequestedBlockLength()
 */
UdsOtaClient_Result_t UdsOtaClient_ProvideStreamBlock(uint32_t blockIndex,
                                                      const uint8_t *data,
                                                      uint8_t length)
{
    uint8_t expectedLength;

    if (data == NULL_PTR)
    {
        return UDS_OTA_CLIENT_RESULT_INVALID_PARAM;
    }

    if (g_clientDebug.state != UDS_OTA_CLIENT_STATE_WAIT_STREAM_BLOCK)
    {
        return UDS_OTA_CLIENT_RESULT_ERROR;
    }

    if (blockIndex != g_clientDebug.currentBlockIndex)
    {
        return UDS_OTA_CLIENT_RESULT_ERROR;
    }

    expectedLength = calcCurrentBlockLength();

    if ((length == 0U) ||
        (length > UDS_OTA_CLIENT_TRANSFER_DATA_SIZE) ||
        (length != expectedLength))
    {
        return UDS_OTA_CLIENT_RESULT_INVALID_PARAM;
    }

    memset(g_streamBlockBuffer, 0xFF, sizeof(g_streamBlockBuffer));
    memcpy(g_streamBlockBuffer, data, length);

    g_streamBlockLength = length;
    g_streamBlockReady = TRUE;

    setState(UDS_OTA_CLIENT_STATE_SEND_TRANSFER_DATA);

    return UDS_OTA_CLIENT_RESULT_OK;
}


void UdsOtaClient_MainFunction(void)
{
    g_clientDebug.tickCount++;

    switch (g_clientDebug.state)
    {
        case UDS_OTA_CLIENT_STATE_IDLE:
        case UDS_OTA_CLIENT_STATE_DONE:
        case UDS_OTA_CLIENT_STATE_ERROR:
        {
            break;
        }

        case UDS_OTA_CLIENT_STATE_SEND_DIAGNOSTIC_SESSION:
        {
            sendDiagnosticSessionControl();
            break;
        }

        case UDS_OTA_CLIENT_STATE_WAIT_DIAGNOSTIC_SESSION:
        {
            handleDiagnosticSessionResponse();
            break;
        }

        case UDS_OTA_CLIENT_STATE_SEND_REQUEST_DOWNLOAD:
        {
            sendRequestDownload();
            break;
        }

        case UDS_OTA_CLIENT_STATE_WAIT_REQUEST_DOWNLOAD:
        {
            handleRequestDownloadResponse();
            break;
        }

        case UDS_OTA_CLIENT_STATE_WAIT_STREAM_BLOCK:
        {
            /*
             * 상위 계층이 UdsOtaClient_ProvideStreamBlock()을 호출할 때까지 대기.
             * 이 상태에서는 CAN request를 보내지 않는다.
             */
            break;
        }

        case UDS_OTA_CLIENT_STATE_SEND_TRANSFER_DATA:
        {
            sendTransferData();
            break;
        }

        case UDS_OTA_CLIENT_STATE_WAIT_TRANSFER_DATA:
        {
            handleTransferDataResponse();
            break;
        }

        case UDS_OTA_CLIENT_STATE_WAIT_FINAL_CRC:
        {
            /*
             * Late CRC mode 전용.
             *
             * 모든 firmware block 전송이 끝난 상태.
             * Pi/HPC가 0x37 단계에서 CRC32를 주면
             * 상위 계층이 UdsOtaClient_SetFinalCrc()를 호출한다.
             *
             * 여기서는 CAN request를 보내지 않고 대기한다.
             */
            break;
        }

        case UDS_OTA_CLIENT_STATE_SEND_REQUEST_TRANSFER_EXIT:
        {
            sendRequestTransferExit();
            break;
        }

        case UDS_OTA_CLIENT_STATE_WAIT_REQUEST_TRANSFER_EXIT:
        {
            handleRequestTransferExitResponse();
            break;
        }

        case UDS_OTA_CLIENT_STATE_SEND_ROUTINE_CONTROL_CRC:
        {
            sendRoutineControlCrc();
            break;
        }

        case UDS_OTA_CLIENT_STATE_WAIT_ROUTINE_CONTROL_CRC:
        {
            handleRoutineControlCrcResponse();
            break;
        }

        case UDS_OTA_CLIENT_STATE_SEND_ECU_RESET:
        {
            sendEcuReset();
            break;
        }

        case UDS_OTA_CLIENT_STATE_WAIT_ECU_RESET:
        {
            handleEcuResetResponse();
            break;
        }

        default:
        {
            setError(UDS_OTA_CLIENT_RESULT_ERROR);
            break;
        }
    }
}


/*
 * Sensor ECU에서 온 0x601 response를 전달받는 함수.
 *
 * App_Can은 0x601의 의미를 해석하지 않는다.
 * App_OtaGateway 쪽에서 AppCan_RecvById(CAN_ID_OTA_RESPONSE, ...)로 받은 뒤
 * 이 함수에 넘기는 구조로 사용한다.
 */
void UdsOtaClient_OnResponse(const uint8_t *data, uint8_t length)
{
    if ((data == NULL_PTR) || (length == 0U))
    {
        return;
    }

    if (length > CANFD_MAX_DLC)
    {
        length = CANFD_MAX_DLC;
    }

    taskENTER_CRITICAL();

    memcpy(g_responseData, data, length);
    g_responseLength = length;
    g_responsePending = TRUE;

    g_clientDebug.responseCount++;
    g_clientDebug.lastRxSid = data[0];

    if ((data[0] == UDS_SID_NEGATIVE_RESPONSE) && (length >= 3U))
    {
        g_clientDebug.lastRxNrc = data[2];
        g_clientDebug.negativeResponseCount++;
    }

    taskEXIT_CRITICAL();
}


UdsOtaClient_State_t UdsOtaClient_GetState(void)
{
    return g_clientDebug.state;
}


UdsOtaClient_Result_t UdsOtaClient_GetLastResult(void)
{
    return g_clientDebug.lastResult;
}


boolean UdsOtaClient_IsBusy(void)
{
    boolean busy = FALSE;

    if ((g_clientDebug.state != UDS_OTA_CLIENT_STATE_IDLE) &&
        (g_clientDebug.state != UDS_OTA_CLIENT_STATE_DONE) &&
        (g_clientDebug.state != UDS_OTA_CLIENT_STATE_ERROR))
    {
        busy = TRUE;
    }

    return busy;
}


boolean UdsOtaClient_IsDone(void)
{
    return (g_clientDebug.state == UDS_OTA_CLIENT_STATE_DONE) ? TRUE : FALSE;
}


boolean UdsOtaClient_IsError(void)
{
    return (g_clientDebug.state == UDS_OTA_CLIENT_STATE_ERROR) ? TRUE : FALSE;
}


boolean UdsOtaClient_IsWaitingStreamBlock(void)
{
    return (g_clientDebug.state == UDS_OTA_CLIENT_STATE_WAIT_STREAM_BLOCK) ? TRUE : FALSE;
}


boolean UdsOtaClient_IsWaitingFinalCrc(void)
{
    return (g_clientDebug.state == UDS_OTA_CLIENT_STATE_WAIT_FINAL_CRC) ? TRUE : FALSE;
}


uint32_t UdsOtaClient_GetRequestedBlockIndex(void)
{
    return g_clientDebug.currentBlockIndex;
}


uint32_t UdsOtaClient_GetRequestedOffset(void)
{
    return g_clientDebug.currentBlockIndex * UDS_OTA_CLIENT_TRANSFER_DATA_SIZE;
}


uint8_t UdsOtaClient_GetRequestedBlockLength(void)
{
    return calcCurrentBlockLength();
}


uint8_t UdsOtaClient_GetProgress(void)
{
    return (uint8_t)g_clientDebug.lastProgressPercent;
}


void UdsOtaClient_GetDebugInfo(UdsOtaClient_DebugInfo_t *info)
{
    if (info != NULL_PTR)
    {
        g_clientDebug.finalCrcProvided = g_finalCrcProvided;
        memcpy(info, &g_clientDebug, sizeof(UdsOtaClient_DebugInfo_t));
    }
}


/* ============================================================
   State helpers
   ============================================================ */

static void setState(UdsOtaClient_State_t state)
{
    g_clientDebug.state = state;
    g_clientDebug.stateEnterTick = g_clientDebug.tickCount;
    updateFinalCrcDebugFlag();
}


static void setError(UdsOtaClient_Result_t result)
{
    g_clientDebug.lastResult = result;
    g_clientDebug.state = UDS_OTA_CLIENT_STATE_ERROR;
    updateFinalCrcDebugFlag();

    if (result == UDS_OTA_CLIENT_RESULT_TIMEOUT)
    {
        g_clientDebug.timeoutCount++;
    }
    else if (result == UDS_OTA_CLIENT_RESULT_CAN_TX_ERROR)
    {
        g_clientDebug.canTxErrorCount++;
    }
}


static boolean checkTimeout(uint32_t timeoutTicks)
{
    if ((g_clientDebug.tickCount - g_clientDebug.stateEnterTick) > timeoutTicks)
    {
        setError(UDS_OTA_CLIENT_RESULT_TIMEOUT);
        return TRUE;
    }

    return FALSE;
}


static void updateFinalCrcDebugFlag(void)
{
    g_clientDebug.finalCrcProvided = g_finalCrcProvided;
}


/* ============================================================
   Response buffer helpers
   ============================================================ */

static void clearResponse(void)
{
    taskENTER_CRITICAL();

    g_responsePending = FALSE;
    g_responseLength = 0U;
    memset(g_responseData, 0, sizeof(g_responseData));

    taskEXIT_CRITICAL();
}


static boolean fetchResponse(uint8_t *buffer, uint8_t *length)
{
    boolean result = FALSE;

    if ((buffer == NULL_PTR) || (length == NULL_PTR))
    {
        return FALSE;
    }

    taskENTER_CRITICAL();

    if (g_responsePending == TRUE)
    {
        memcpy(buffer, g_responseData, g_responseLength);
        *length = g_responseLength;

        g_responsePending = FALSE;
        g_responseLength = 0U;

        result = TRUE;
    }

    taskEXIT_CRITICAL();

    return result;
}


/* ============================================================
   Utility
   ============================================================ */

static uint8_t positiveResponseSid(uint8_t requestSid)
{
    return (uint8_t)(requestSid + UDS_POSITIVE_RESPONSE_OFFSET);
}


static uint16_t readU16Le(const uint8_t *p)
{
    return ((uint16_t)p[0]) |
           ((uint16_t)p[1] << 8);
}


static uint32_t readU32Le(const uint8_t *p)
{
    return ((uint32_t)p[0]) |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}


static void writeU16Le(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value & 0xFFU);
    p[1] = (uint8_t)((value >> 8) & 0xFFU);
}


static void writeU32Le(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)(value & 0xFFU);
    p[1] = (uint8_t)((value >> 8) & 0xFFU);
    p[2] = (uint8_t)((value >> 16) & 0xFFU);
    p[3] = (uint8_t)((value >> 24) & 0xFFU);
}


static void makePayload(uint8_t *p, uint8_t fill)
{
    uint8_t i;

    if (p == NULL_PTR)
    {
        return;
    }

    for (i = 0U; i < CANFD_MAX_DLC; i++)
    {
        p[i] = fill;
    }
}


static boolean sendPayload(const uint8_t *payload, uint8_t length)
{
    BaseType_t txResult;

    if ((payload == NULL_PTR) || (length > CANFD_MAX_DLC))
    {
        setError(UDS_OTA_CLIENT_RESULT_INVALID_PARAM);
        return FALSE;
    }

    /*
     * Team RTOS 구조:
     *  - UdsOtaClient는 MCMCAN/CanIf를 직접 사용하지 않는다.
     *  - App_Can raw CAN FD 송신 API를 통해 0x600 UDS Request를 보낸다.
     */
    txResult = AppCan_SendFd(CAN_ID_OTA_REQUEST, payload, length);

    if (txResult != pdPASS)
    {
        setError(UDS_OTA_CLIENT_RESULT_CAN_TX_ERROR);
        return FALSE;
    }

    g_clientDebug.requestCount++;

    return TRUE;
}


static uint32_t calcTotalBlocks(uint32_t size)
{
    uint32_t blocks;

    blocks = size / UDS_OTA_CLIENT_TRANSFER_DATA_SIZE;

    if ((size % UDS_OTA_CLIENT_TRANSFER_DATA_SIZE) != 0U)
    {
        blocks++;
    }

    return blocks;
}


static uint8_t calcCurrentBlockLength(void)
{
    uint32_t offset;
    uint32_t remain;

    if (g_firmwareSize == 0U)
    {
        return 0U;
    }

    offset = g_clientDebug.currentBlockIndex * UDS_OTA_CLIENT_TRANSFER_DATA_SIZE;

    if (offset >= g_firmwareSize)
    {
        return 0U;
    }

    remain = g_firmwareSize - offset;

    if (remain >= UDS_OTA_CLIENT_TRANSFER_DATA_SIZE)
    {
        return UDS_OTA_CLIENT_TRANSFER_DATA_SIZE;
    }

    return (uint8_t)(remain & 0xFFU);
}


static void proceedAfterAllBlocksSent(void)
{
    /*
     * 기존 CRC known 모드:
     *   StartStream(size, crc)로 시작했으면 CRC를 이미 알고 있으므로
     *   바로 RequestTransferExit로 진행한다.
     *
     * Late CRC 모드:
     *   StartStreamWithoutCrc(size)로 시작했으면
     *   Pi/HPC가 0x37에서 CRC32를 줄 때까지 WAIT_FINAL_CRC에서 대기한다.
     */
    if (g_finalCrcProvided == TRUE)
    {
        setState(UDS_OTA_CLIENT_STATE_SEND_REQUEST_TRANSFER_EXIT);
    }
    else
    {
        setState(UDS_OTA_CLIENT_STATE_WAIT_FINAL_CRC);
    }
}


/* ============================================================
   UDS send / response handlers
   ============================================================ */

static void sendDiagnosticSessionControl(void)
{
    uint8_t p[CANFD_MAX_DLC];

    makePayload(p, 0x00U);

    p[0] = UDS_SID_DIAGNOSTIC_SESSION_CONTROL;
    p[1] = UDS_SESSION_PROGRAMMING;

    clearResponse();

    if (sendPayload(p, UDS_REQ_LEN_DIAGNOSTIC_SESSION_CONTROL) == TRUE)
    {
        g_clientDebug.lastExpectedSid = positiveResponseSid(UDS_SID_DIAGNOSTIC_SESSION_CONTROL);
        setState(UDS_OTA_CLIENT_STATE_WAIT_DIAGNOSTIC_SESSION);
    }
}


static void handleDiagnosticSessionResponse(void)
{
    uint8_t resp[CANFD_MAX_DLC];
    uint8_t len = 0U;

    if (fetchResponse(resp, &len) == FALSE)
    {
        (void)checkTimeout(UDS_OTA_CLIENT_TIMEOUT_TICKS);
        return;
    }

    if ((len >= 3U) && (resp[0] == UDS_SID_NEGATIVE_RESPONSE))
    {
        setError(UDS_OTA_CLIENT_RESULT_NEGATIVE_RESPONSE);
        return;
    }

    if ((len >= 2U) &&
        (resp[0] == positiveResponseSid(UDS_SID_DIAGNOSTIC_SESSION_CONTROL)) &&
        (resp[1] == UDS_SESSION_PROGRAMMING))
    {
        setState(UDS_OTA_CLIENT_STATE_SEND_REQUEST_DOWNLOAD);
    }
    else
    {
        setError(UDS_OTA_CLIENT_RESULT_UNEXPECTED_RESPONSE);
    }
}


static void sendRequestDownload(void)
{
    uint8_t p[CANFD_MAX_DLC];

    makePayload(p, 0x00U);

    p[0] = UDS_SID_REQUEST_DOWNLOAD;
    p[1] = UDS_DOWNLOAD_DATA_FORMAT_ID;
    p[2] = UDS_DOWNLOAD_ADDR_LEN_FORMAT;

    writeU32Le(&p[3], UDS_OTA_CLIENT_TARGET_APP_ADDR);
    writeU32Le(&p[7], g_firmwareSize);

    clearResponse();

    if (sendPayload(p, UDS_REQ_LEN_REQUEST_DOWNLOAD) == TRUE)
    {
        g_clientDebug.lastExpectedSid = positiveResponseSid(UDS_SID_REQUEST_DOWNLOAD);
        setState(UDS_OTA_CLIENT_STATE_WAIT_REQUEST_DOWNLOAD);
    }
}


static void handleRequestDownloadResponse(void)
{
    uint8_t resp[CANFD_MAX_DLC];
    uint8_t len = 0U;
    uint16_t maxBlockLength;

    if (fetchResponse(resp, &len) == FALSE)
    {
        (void)checkTimeout(UDS_OTA_CLIENT_TIMEOUT_TICKS);
        return;
    }

    if ((len >= 3U) && (resp[0] == UDS_SID_NEGATIVE_RESPONSE))
    {
        setError(UDS_OTA_CLIENT_RESULT_NEGATIVE_RESPONSE);
        return;
    }

    if ((len >= 4U) && (resp[0] == positiveResponseSid(UDS_SID_REQUEST_DOWNLOAD)))
    {
        maxBlockLength = readU16Le(&resp[2]);

        if (maxBlockLength < UDS_OTA_CLIENT_TRANSFER_DATA_SIZE)
        {
            setError(UDS_OTA_CLIENT_RESULT_UNEXPECTED_RESPONSE);
            return;
        }

        setState(UDS_OTA_CLIENT_STATE_WAIT_STREAM_BLOCK);
    }
    else
    {
        setError(UDS_OTA_CLIENT_RESULT_UNEXPECTED_RESPONSE);
    }
}


static void sendTransferData(void)
{
    uint8_t p[CANFD_MAX_DLC];
    uint32_t offset;
    uint8_t dataLen;
    uint8_t i;

    if (g_clientDebug.currentBlockIndex >= g_clientDebug.totalBlocks)
    {
        proceedAfterAllBlocksSent();
        return;
    }

    offset = g_clientDebug.currentBlockIndex * UDS_OTA_CLIENT_TRANSFER_DATA_SIZE;
    dataLen = calcCurrentBlockLength();

    if (dataLen == 0U)
    {
        setError(UDS_OTA_CLIENT_RESULT_ERROR);
        return;
    }

    if (g_streamBlockReady == FALSE)
    {
        setState(UDS_OTA_CLIENT_STATE_WAIT_STREAM_BLOCK);
        return;
    }

    if (g_streamBlockLength != dataLen)
    {
        setError(UDS_OTA_CLIENT_RESULT_ERROR);
        return;
    }

    makePayload(p, 0xFFU);

    p[0] = UDS_SID_TRANSFER_DATA;
    p[1] = g_clientDebug.currentBsc;

    for (i = 0U; i < dataLen; i++)
    {
        p[2U + i] = g_streamBlockBuffer[i];
    }

    g_clientDebug.currentOffset = offset;

    clearResponse();

    if (sendPayload(p, (uint8_t)(2U + dataLen)) == TRUE)
    {
        g_clientDebug.lastExpectedSid = positiveResponseSid(UDS_SID_TRANSFER_DATA);
        setState(UDS_OTA_CLIENT_STATE_WAIT_TRANSFER_DATA);
    }
}
static void handleTransferDataResponse(void)
{
    uint8_t resp[CANFD_MAX_DLC];
    uint8_t len = 0U;

    g_dbgTdRespEnterCount++;

    /*
     * 1. 먼저 response가 있는지 확인
     *    response가 없을 때만 timeout 검사
     */
    if (fetchResponse(resp, &len) == FALSE)
    {
        if (checkTimeout(UDS_OTA_CLIENT_TRANSFER_TIMEOUT_TICKS) == TRUE)
        {
            g_dbgTdRespTimeoutCount++;
        }

        return;
    }

    /*
     * 여기까지 왔으면 0x601 response는 실제로 UdsOtaClient까지 들어온 것
     */
    g_dbgTdRespFetchOkCount++;

    g_dbgTdRespLen = len;
    g_dbgTdResp0 = resp[0];
    g_dbgTdResp1 = (len > 1U) ? resp[1] : 0U;
    g_dbgTdExpectedBsc = g_clientDebug.currentBsc;

    /*
     * Negative Response: 7F 36 NRC
     */
    if ((len >= 3U) && (resp[0] == UDS_SID_NEGATIVE_RESPONSE))
    {
        g_clientDebug.lastRxSid = resp[0];
        g_clientDebug.lastRxNrc = resp[2];

        setError(UDS_OTA_CLIENT_RESULT_NEGATIVE_RESPONSE);
        return;
    }

    /*
     * Positive response 최소 길이 확인
     * Sensor ECU는 CAN FD 64 byte로 응답하므로 len == 2가 아니라 len >= 2여야 함
     */
    if (len < 2U)
    {
        g_dbgTdRespLenFailCount++;
        setError(UDS_OTA_CLIENT_RESULT_UNEXPECTED_RESPONSE);
        return;
    }

    /*
     * SID 확인: 0x36 positive response = 0x76
     */
    if (resp[0] != positiveResponseSid(UDS_SID_TRANSFER_DATA))
    {
        g_dbgTdRespSidMismatchCount++;
        setError(UDS_OTA_CLIENT_RESULT_UNEXPECTED_RESPONSE);
        return;
    }

    /*
     * BSC 확인
     * 예: currentBsc = 0x02 이면 response는 76 02 여야 함
     */
    if (resp[1] != g_clientDebug.currentBsc)
    {
        g_dbgTdRespBscMismatchCount++;
        setError(UDS_OTA_CLIENT_RESULT_UNEXPECTED_RESPONSE);
        return;
    }

    /*
     * 여기까지 오면 76 xx 정상 처리 branch
     */
    g_dbgTdRespSuccessCount++;

    g_clientDebug.currentBlockIndex++;
    g_clientDebug.sentBytes =
        g_clientDebug.currentBlockIndex * UDS_OTA_CLIENT_TRANSFER_DATA_SIZE;

    if (g_clientDebug.sentBytes > g_firmwareSize)
    {
        g_clientDebug.sentBytes = g_firmwareSize;
    }

    if (g_firmwareSize > 0U)
    {
        g_clientDebug.lastProgressPercent =
            (g_clientDebug.sentBytes * 100U) / g_firmwareSize;
    }

    g_clientDebug.currentBsc++;

    /*
     * uint8 overflow로 0xFF 다음 0x00 자연 wrap.
     */
    g_streamBlockReady = FALSE;
    g_streamBlockLength = 0U;
    memset(g_streamBlockBuffer, 0, sizeof(g_streamBlockBuffer));

    if (g_clientDebug.currentBlockIndex >= g_clientDebug.totalBlocks)
    {
        proceedAfterAllBlocksSent();
    }
    else
    {
        setState(UDS_OTA_CLIENT_STATE_WAIT_STREAM_BLOCK);
    }
}
#if 0
static void handleTransferDataResponse(void)
{
    uint8_t resp[CANFD_MAX_DLC];
    uint8_t len = 0U;

    if (fetchResponse(resp, &len) == FALSE)
    {
        (void)checkTimeout(UDS_OTA_CLIENT_TRANSFER_TIMEOUT_TICKS);
        return;
    }

    if ((len >= 3U) && (resp[0] == UDS_SID_NEGATIVE_RESPONSE))
    {
        setError(UDS_OTA_CLIENT_RESULT_NEGATIVE_RESPONSE);
        return;
    }

    if ((len >= 2U) &&
        (resp[0] == positiveResponseSid(UDS_SID_TRANSFER_DATA)) &&
        (resp[1] == g_clientDebug.currentBsc))
    {
        g_clientDebug.currentBlockIndex++;
        g_clientDebug.sentBytes = g_clientDebug.currentBlockIndex * UDS_OTA_CLIENT_TRANSFER_DATA_SIZE;

        if (g_clientDebug.sentBytes > g_firmwareSize)
        {
            g_clientDebug.sentBytes = g_firmwareSize;
        }

        if (g_firmwareSize > 0U)
        {
            g_clientDebug.lastProgressPercent =
                (g_clientDebug.sentBytes * 100U) / g_firmwareSize;
        }

        g_clientDebug.currentBsc++;

        /*
         * uint8 overflow로 0xFF 다음 0x00 자연 wrap.
         */
        g_streamBlockReady = FALSE;
        g_streamBlockLength = 0U;
        memset(g_streamBlockBuffer, 0, sizeof(g_streamBlockBuffer));

        if (g_clientDebug.currentBlockIndex >= g_clientDebug.totalBlocks)
        {
            proceedAfterAllBlocksSent();
        }
        else
        {
            setState(UDS_OTA_CLIENT_STATE_WAIT_STREAM_BLOCK);
        }
    }
    else
    {
        setError(UDS_OTA_CLIENT_RESULT_UNEXPECTED_RESPONSE);
    }
}
#endif

static void sendRequestTransferExit(void)
{
    uint8_t p[CANFD_MAX_DLC];

    makePayload(p, 0x00U);

    p[0] = UDS_SID_REQUEST_TRANSFER_EXIT;

    clearResponse();

    if (sendPayload(p, UDS_REQ_LEN_REQUEST_TRANSFER_EXIT) == TRUE)
    {
        g_clientDebug.lastExpectedSid = positiveResponseSid(UDS_SID_REQUEST_TRANSFER_EXIT);
        setState(UDS_OTA_CLIENT_STATE_WAIT_REQUEST_TRANSFER_EXIT);
    }
}


static void handleRequestTransferExitResponse(void)
{
    uint8_t resp[CANFD_MAX_DLC];
    uint8_t len = 0U;

    if (fetchResponse(resp, &len) == FALSE)
    {
        (void)checkTimeout(UDS_OTA_CLIENT_TIMEOUT_TICKS);
        return;
    }

    if ((len >= 3U) && (resp[0] == UDS_SID_NEGATIVE_RESPONSE))
    {
        setError(UDS_OTA_CLIENT_RESULT_NEGATIVE_RESPONSE);
        return;
    }

    if ((len >= 1U) && (resp[0] == positiveResponseSid(UDS_SID_REQUEST_TRANSFER_EXIT)))
    {
        setState(UDS_OTA_CLIENT_STATE_SEND_ROUTINE_CONTROL_CRC);
    }
    else
    {
        setError(UDS_OTA_CLIENT_RESULT_UNEXPECTED_RESPONSE);
    }
}


static void sendRoutineControlCrc(void)
{
    uint8_t p[CANFD_MAX_DLC];

    if (g_finalCrcProvided == FALSE)
    {
        setError(UDS_OTA_CLIENT_RESULT_ERROR);
        return;
    }

    makePayload(p, 0x00U);

    p[0] = UDS_SID_ROUTINE_CONTROL;
    p[1] = UDS_ROUTINE_START;

    writeU16Le(&p[2], UDS_ROUTINE_ID_CHECK_CRC32);
    writeU32Le(&p[4], g_firmwareCrc32);

    clearResponse();

    if (sendPayload(p, UDS_REQ_LEN_ROUTINE_CONTROL_CHECK_CRC32) == TRUE)
    {
        g_clientDebug.lastExpectedSid = positiveResponseSid(UDS_SID_ROUTINE_CONTROL);
        setState(UDS_OTA_CLIENT_STATE_WAIT_ROUTINE_CONTROL_CRC);
    }
}


static void handleRoutineControlCrcResponse(void)
{
    uint8_t resp[CANFD_MAX_DLC];
    uint8_t len = 0U;
    uint16_t routineId;
    uint32_t calculatedCrc32;

    if (fetchResponse(resp, &len) == FALSE)
    {
        (void)checkTimeout(UDS_OTA_CLIENT_CRC_TIMEOUT_TICKS);
        return;
    }

    if ((len >= 3U) && (resp[0] == UDS_SID_NEGATIVE_RESPONSE))
    {
        setError(UDS_OTA_CLIENT_RESULT_NEGATIVE_RESPONSE);
        return;
    }

    if ((len >= 8U) && (resp[0] == positiveResponseSid(UDS_SID_ROUTINE_CONTROL)))
    {
        routineId = readU16Le(&resp[2]);
        calculatedCrc32 = readU32Le(&resp[4]);

        g_clientDebug.calculatedCrc32FromEcu = calculatedCrc32;

        if ((resp[1] != UDS_ROUTINE_START) ||
            (routineId != UDS_ROUTINE_ID_CHECK_CRC32))
        {
            setError(UDS_OTA_CLIENT_RESULT_UNEXPECTED_RESPONSE);
            return;
        }

        if (calculatedCrc32 != g_firmwareCrc32)
        {
            setError(UDS_OTA_CLIENT_RESULT_CRC_MISMATCH);
            return;
        }

        /*
         * CRC 검증 성공.
         *
         * 현재 Dual Slot / Bank B OTA:
         *   Download Phase는 여기서 종료한다.
         *   실제 Bank B 활성화/UCB_SWAP은 사용자 승인 후 별도 요청으로 수행한다.
         */
        g_clientDebug.lastResult = UDS_OTA_CLIENT_RESULT_OK;
        g_clientDebug.lastProgressPercent = 100U;

#if (UDS_OTA_CLIENT_AUTO_RESET_AFTER_CRC == 1U)
        setState(UDS_OTA_CLIENT_STATE_SEND_ECU_RESET);
#else
        setState(UDS_OTA_CLIENT_STATE_DONE);
#endif
    }
    else
    {
        setError(UDS_OTA_CLIENT_RESULT_UNEXPECTED_RESPONSE);
    }
}


static void sendEcuReset(void)
{
    uint8_t p[CANFD_MAX_DLC];

    makePayload(p, 0x00U);

    p[0] = UDS_SID_ECU_RESET;
    p[1] = UDS_RESET_JUMP_TO_APP;

    clearResponse();

    if (sendPayload(p, UDS_REQ_LEN_ECU_RESET) == TRUE)
    {
        g_clientDebug.lastExpectedSid = positiveResponseSid(UDS_SID_ECU_RESET);
        setState(UDS_OTA_CLIENT_STATE_WAIT_ECU_RESET);
    }
}


static void handleEcuResetResponse(void)
{
    uint8_t resp[CANFD_MAX_DLC];
    uint8_t len = 0U;

    if (fetchResponse(resp, &len) == FALSE)
    {
        (void)checkTimeout(UDS_OTA_CLIENT_TIMEOUT_TICKS);
        return;
    }

    if ((len >= 3U) && (resp[0] == UDS_SID_NEGATIVE_RESPONSE))
    {
        setError(UDS_OTA_CLIENT_RESULT_NEGATIVE_RESPONSE);
        return;
    }

    if ((len >= 2U) &&
        (resp[0] == positiveResponseSid(UDS_SID_ECU_RESET)) &&
        (resp[1] == UDS_RESET_JUMP_TO_APP))
    {
        g_clientDebug.lastResult = UDS_OTA_CLIENT_RESULT_OK;
        setState(UDS_OTA_CLIENT_STATE_DONE);
    }
    else
    {
        setError(UDS_OTA_CLIENT_RESULT_UNEXPECTED_RESPONSE);
    }
}
