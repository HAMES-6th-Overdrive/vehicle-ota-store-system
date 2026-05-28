/**********************************************************************************************************************
 * \file OtaGateway.c
 * \brief ZCU OTA Gateway Layer - Download/Verify phase
 *
 * 역할:
 *  - Pi/HPC 계층과 UdsOtaClient streaming API 사이의 중간 관리자
 *  - ZCU가 전체 firmware binary를 저장하지 않고, 현재 필요한 block만 Sensor ECU로 전달하도록 관리
 *
 * 현재 단계:
 *  - bin download + CRC 검증까지만 담당한다.
 *  - SOTA/UCB_SWAP activation은 별도 단계에서 수행한다.
 *
 * CRC 모드:
 *  1. CRC known mode
 *     - OtaGateway_Start(firmwareSize, firmwareCrc32)
 *
 *  2. Late CRC mode
 *     - OtaGateway_StartWithoutCrc(firmwareSize)
 *     - 모든 block 전송 완료 후 WAIT_FINAL_CRC에서 대기
 *     - OtaGateway_SetFinalCrc(firmwareCrc32) 호출 시
 *       Sensor ECU 쪽 RequestTransferExit + RoutineControl CRC 진행
 *********************************************************************************************************************/

#include "OtaGateway.h"
#include "UdsOtaClient.h"

#include <string.h>

/* ============================================================
   Internal state
   ============================================================ */

static OtaGateway_DebugInfo_t g_otaGatewayDebug;


/* ============================================================
   Private prototypes
   ============================================================ */

static void setState(OtaGateway_State_t state);
static void setResult(OtaGateway_Result_t result);
static void updateRequestedBlockInfo(void);
static void updateFinalCrcFlag(boolean provided);


/* ============================================================
   Public API
   ============================================================ */

void OtaGateway_Init(void)
{
    /*
     * Gateway가 UdsOtaClient를 소유한다.
     * 따라서 Gateway init 시 Client도 같이 초기화한다.
     */
    UdsOtaClient_Init();
    OtaGateway_Reset();
}


void OtaGateway_Reset(void)
{
    /*
     * Download/Verify phase 전체를 초기 상태로 되돌린다.
     * SOTA/UCB_SWAP activation 상태는 여기서 다루지 않는다.
     */
    UdsOtaClient_Reset();

    memset(&g_otaGatewayDebug, 0, sizeof(g_otaGatewayDebug));

    g_otaGatewayDebug.state = OTA_GATEWAY_STATE_IDLE;
    g_otaGatewayDebug.lastResult = OTA_GATEWAY_RESULT_OK;
    g_otaGatewayDebug.progressPercent = 0U;
    g_otaGatewayDebug.finalCrcProvided = FALSE;
}


OtaGateway_Result_t OtaGateway_Start(uint32_t firmwareSize,
                                     uint32_t firmwareCrc32)
{
    UdsOtaClient_Result_t clientResult;

    if(firmwareSize == 0U)
    {
        setResult(OTA_GATEWAY_RESULT_INVALID_PARAM);
        return OTA_GATEWAY_RESULT_INVALID_PARAM;
    }

    if(OtaGateway_IsBusy() == TRUE)
    {
        setResult(OTA_GATEWAY_RESULT_BUSY);
        return OTA_GATEWAY_RESULT_BUSY;
    }

    /*
     * Gateway 상태 초기화.
     *
     * UdsOtaClient_StartStream() 내부에서도 Client reset이 수행되지만,
     * Gateway debug/state를 정리하기 위해 Gateway Reset을 먼저 수행한다.
     */
    OtaGateway_Reset();

    g_otaGatewayDebug.firmwareSize = firmwareSize;
    g_otaGatewayDebug.firmwareCrc32 = firmwareCrc32;
    g_otaGatewayDebug.startRequestCount++;
    updateFinalCrcFlag(TRUE);

    /*
     * 핵심:
     *  - 전체 firmware pointer를 넘기지 않는다.
     *  - size / crc32만 넘기고 streaming OTA를 시작한다.
     *  - 이후 필요한 block은 OtaGateway_ProvideBlock()으로 하나씩 제공한다.
     */
    clientResult = UdsOtaClient_StartStream(firmwareSize, firmwareCrc32);

    if(clientResult != UDS_OTA_CLIENT_RESULT_OK)
    {
        setResult(OTA_GATEWAY_RESULT_CLIENT_ERROR);
        setState(OTA_GATEWAY_STATE_ERROR);
        return OTA_GATEWAY_RESULT_CLIENT_ERROR;
    }

    setResult(OTA_GATEWAY_RESULT_OK);
    setState(OTA_GATEWAY_STATE_IN_PROGRESS);

    return OTA_GATEWAY_RESULT_OK;
}


OtaGateway_Result_t OtaGateway_StartWithoutCrc(uint32_t firmwareSize)
{
    UdsOtaClient_Result_t clientResult;

    if(firmwareSize == 0U)
    {
        setResult(OTA_GATEWAY_RESULT_INVALID_PARAM);
        return OTA_GATEWAY_RESULT_INVALID_PARAM;
    }

    if(OtaGateway_IsBusy() == TRUE)
    {
        setResult(OTA_GATEWAY_RESULT_BUSY);
        return OTA_GATEWAY_RESULT_BUSY;
    }

    /*
     * Late CRC mode:
     *  - Pi/HPC -> ZCU DoIP 흐름에서는 CRC가 마지막 0x37에서 들어온다.
     *  - 따라서 여기서는 firmwareSize만 가지고 Sensor ECU OTA를 시작한다.
     */
    OtaGateway_Reset();

    g_otaGatewayDebug.firmwareSize = firmwareSize;
    g_otaGatewayDebug.firmwareCrc32 = 0U;
    g_otaGatewayDebug.startWithoutCrcRequestCount++;
    updateFinalCrcFlag(FALSE);

    clientResult = UdsOtaClient_StartStreamWithoutCrc(firmwareSize);

    if(clientResult != UDS_OTA_CLIENT_RESULT_OK)
    {
        setResult(OTA_GATEWAY_RESULT_CLIENT_ERROR);
        setState(OTA_GATEWAY_STATE_ERROR);
        return OTA_GATEWAY_RESULT_CLIENT_ERROR;
    }

    setResult(OTA_GATEWAY_RESULT_OK);
    setState(OTA_GATEWAY_STATE_IN_PROGRESS);

    return OTA_GATEWAY_RESULT_OK;
}


OtaGateway_Result_t OtaGateway_SetFinalCrc(uint32_t firmwareCrc32)
{
    UdsOtaClient_Result_t clientResult;

    /*
     * CRC32 값 0x00000000도 이론상 유효할 수 있으므로 값 자체는 reject하지 않는다.
     * 대신 상태만 확인한다.
     */
    if(UdsOtaClient_IsWaitingFinalCrc() == FALSE)
    {
        setResult(OTA_GATEWAY_RESULT_SEQUENCE_ERROR);
        return OTA_GATEWAY_RESULT_SEQUENCE_ERROR;
    }

    clientResult = UdsOtaClient_SetFinalCrc(firmwareCrc32);

    if(clientResult != UDS_OTA_CLIENT_RESULT_OK)
    {
        setResult(OTA_GATEWAY_RESULT_CLIENT_ERROR);
        setState(OTA_GATEWAY_STATE_ERROR);
        return OTA_GATEWAY_RESULT_CLIENT_ERROR;
    }

    g_otaGatewayDebug.firmwareCrc32 = firmwareCrc32;
    g_otaGatewayDebug.finalCrcSetRequestCount++;
    updateFinalCrcFlag(TRUE);

    setResult(OTA_GATEWAY_RESULT_OK);
    setState(OTA_GATEWAY_STATE_IN_PROGRESS);

    return OTA_GATEWAY_RESULT_OK;
}


OtaGateway_Result_t OtaGateway_ProvideBlock(uint32_t blockIndex,
                                            const uint8_t *data,
                                            uint8_t length)
{
    uint32_t expectedBlockIndex;
    uint8_t expectedLength;
    UdsOtaClient_Result_t clientResult;

    if(data == NULL_PTR)
    {
        setResult(OTA_GATEWAY_RESULT_INVALID_PARAM);
        g_otaGatewayDebug.blockProvideFailCount++;
        return OTA_GATEWAY_RESULT_INVALID_PARAM;
    }

    if(UdsOtaClient_IsWaitingStreamBlock() == FALSE)
    {
        setResult(OTA_GATEWAY_RESULT_SEQUENCE_ERROR);
        g_otaGatewayDebug.blockProvideFailCount++;
        return OTA_GATEWAY_RESULT_SEQUENCE_ERROR;
    }

    expectedBlockIndex = UdsOtaClient_GetRequestedBlockIndex();
    expectedLength = UdsOtaClient_GetRequestedBlockLength();

    if(blockIndex != expectedBlockIndex)
    {
        setResult(OTA_GATEWAY_RESULT_SEQUENCE_ERROR);
        g_otaGatewayDebug.blockProvideFailCount++;
        return OTA_GATEWAY_RESULT_SEQUENCE_ERROR;
    }

    if((length == 0U) || (length != expectedLength))
    {
        setResult(OTA_GATEWAY_RESULT_INVALID_PARAM);
        g_otaGatewayDebug.blockProvideFailCount++;
        return OTA_GATEWAY_RESULT_INVALID_PARAM;
    }

    clientResult = UdsOtaClient_ProvideStreamBlock(blockIndex, data, length);

    if(clientResult != UDS_OTA_CLIENT_RESULT_OK)
    {
        setResult(OTA_GATEWAY_RESULT_CLIENT_ERROR);
        setState(OTA_GATEWAY_STATE_ERROR);
        g_otaGatewayDebug.blockProvideFailCount++;
        return OTA_GATEWAY_RESULT_CLIENT_ERROR;
    }

    g_otaGatewayDebug.providedBlockCount++;
    g_otaGatewayDebug.lastProvidedBlockIndex = blockIndex;
    g_otaGatewayDebug.lastProvidedOffset = UdsOtaClient_GetRequestedOffset();
    g_otaGatewayDebug.lastProvidedLength = length;
    g_otaGatewayDebug.blockProvideOkCount++;

    setResult(OTA_GATEWAY_RESULT_OK);
    setState(OTA_GATEWAY_STATE_IN_PROGRESS);

    return OTA_GATEWAY_RESULT_OK;
}


OtaGateway_Result_t OtaGateway_Cancel(void)
{
    /*
     * Download phase 취소.
     * 현재 단계에서는 Sensor ECU 쪽 cancel UDS request는 보내지 않는다.
     * ZCU 내부 Gateway/Client 상태만 정리한다.
     */
    g_otaGatewayDebug.cancelRequestCount++;

    UdsOtaClient_Reset();

    setResult(OTA_GATEWAY_RESULT_CANCELLED);
    setState(OTA_GATEWAY_STATE_IDLE);

    return OTA_GATEWAY_RESULT_OK;
}


void OtaGateway_MainFunction(void)
{
    /*
     * Gateway가 UdsOtaClient를 소유한다.
     * 따라서 상위 task는 OtaGateway_MainFunction()만 1ms마다 호출하면 된다.
     */
    UdsOtaClient_MainFunction();

    g_otaGatewayDebug.progressPercent = UdsOtaClient_GetProgress();

    if(UdsOtaClient_IsError() == TRUE)
    {
        setResult(OTA_GATEWAY_RESULT_CLIENT_ERROR);
        setState(OTA_GATEWAY_STATE_ERROR);
        return;
    }

    if(UdsOtaClient_IsDone() == TRUE)
    {
        setResult(OTA_GATEWAY_RESULT_OK);
        setState(OTA_GATEWAY_STATE_DONE);
        g_otaGatewayDebug.progressPercent = 100U;
        return;
    }

    if(UdsOtaClient_IsWaitingStreamBlock() == TRUE)
    {
        updateRequestedBlockInfo();
        setState(OTA_GATEWAY_STATE_WAIT_BLOCK);
        return;
    }

    if(UdsOtaClient_IsWaitingFinalCrc() == TRUE)
    {
        /*
         * 모든 block은 Sensor ECU로 전달 완료.
         * 이제 Pi/HPC가 0x37 단계에서 CRC32를 줄 때까지 대기한다.
         */
        setState(OTA_GATEWAY_STATE_WAIT_FINAL_CRC);
        return;
    }

    if(UdsOtaClient_IsBusy() == TRUE)
    {
        setState(OTA_GATEWAY_STATE_IN_PROGRESS);
        return;
    }

    /*
     * UdsOtaClient가 IDLE이고 Gateway도 IDLE이 아닌 애매한 경우는
     * 현재 Gateway 상태를 유지한다.
     */
}


boolean OtaGateway_IsBusy(void)
{
    boolean busy = FALSE;

    if((g_otaGatewayDebug.state != OTA_GATEWAY_STATE_IDLE) &&
       (g_otaGatewayDebug.state != OTA_GATEWAY_STATE_DONE) &&
       (g_otaGatewayDebug.state != OTA_GATEWAY_STATE_ERROR))
    {
        busy = TRUE;
    }

    return busy;
}


boolean OtaGateway_IsWaitingBlock(void)
{
    return (g_otaGatewayDebug.state == OTA_GATEWAY_STATE_WAIT_BLOCK) ? TRUE : FALSE;
}


boolean OtaGateway_IsWaitingFinalCrc(void)
{
    return (g_otaGatewayDebug.state == OTA_GATEWAY_STATE_WAIT_FINAL_CRC) ? TRUE : FALSE;
}


boolean OtaGateway_IsDone(void)
{
    return (g_otaGatewayDebug.state == OTA_GATEWAY_STATE_DONE) ? TRUE : FALSE;
}


boolean OtaGateway_IsError(void)
{
    return (g_otaGatewayDebug.state == OTA_GATEWAY_STATE_ERROR) ? TRUE : FALSE;
}


uint32_t OtaGateway_GetRequestedBlockIndex(void)
{
    return g_otaGatewayDebug.requestedBlockIndex;
}


uint32_t OtaGateway_GetRequestedOffset(void)
{
    return g_otaGatewayDebug.requestedOffset;
}


uint8_t OtaGateway_GetRequestedLength(void)
{
    return g_otaGatewayDebug.requestedLength;
}


uint8_t OtaGateway_GetProgress(void)
{
    return g_otaGatewayDebug.progressPercent;
}


void OtaGateway_GetDebugInfo(OtaGateway_DebugInfo_t *info)
{
    if(info != NULL_PTR)
    {
        memcpy(info, &g_otaGatewayDebug, sizeof(OtaGateway_DebugInfo_t));
    }
}


/* ============================================================
   Private functions
   ============================================================ */

static void setState(OtaGateway_State_t state)
{
    g_otaGatewayDebug.state = state;
}


static void setResult(OtaGateway_Result_t result)
{
    g_otaGatewayDebug.lastResult = result;
}


static void updateFinalCrcFlag(boolean provided)
{
    g_otaGatewayDebug.finalCrcProvided = provided;
}


static void updateRequestedBlockInfo(void)
{
    uint32_t newIndex;
    uint32_t newOffset;
    uint8_t newLength;

    newIndex = UdsOtaClient_GetRequestedBlockIndex();
    newOffset = UdsOtaClient_GetRequestedOffset();
    newLength = UdsOtaClient_GetRequestedBlockLength();

    /*
     * 같은 block을 기다리는 동안 MainFunction이 1ms마다 호출되므로,
     * blockRequestCount가 계속 증가하지 않도록 새 block 요청일 때만 증가시킨다.
     */
    if((g_otaGatewayDebug.state != OTA_GATEWAY_STATE_WAIT_BLOCK) ||
       (g_otaGatewayDebug.requestedBlockIndex != newIndex) ||
       (g_otaGatewayDebug.requestedOffset != newOffset) ||
       (g_otaGatewayDebug.requestedLength != newLength))
    {
        g_otaGatewayDebug.blockRequestCount++;
    }

    g_otaGatewayDebug.requestedBlockIndex = newIndex;
    g_otaGatewayDebug.requestedOffset = newOffset;
    g_otaGatewayDebug.requestedLength = newLength;
}
