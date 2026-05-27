/**********************************************************************************************************************
 * \file UdsOta.c
 * \brief Sensor ECU UDS-style OTA Service Layer over CAN FD
 *
 * 담당 범위:
 *  - CAN FD 0x600 UDS Request payload 해석
 *  - UDS Positive / Negative Response 생성
 *  - Download / TransferData / TransferExit / RoutineControl / Reset 상태 관리
 *
 * 현재 수정 방향:
 *  - RequestDownload(0x34)에서 client가 보낸 memoryAddress는 직접 신뢰하지 않는다.
 *  - 현재 단독 검증 단계에서는 Slot A에서 실행 중이라고 가정하고 Slot B(0x80320000)에 write한다.
 *  - 최종 통합 시에는 SOTA_IsGroupBActive() 결과에 따라 targetAddress를 선택한다.
 *
 * Cleanup 개선 내용:
 *  - 새 Programming Session(0x10 02) 진입 시 이전 OTA 완료/오류 context 정리
 *  - 새 RequestDownload(0x34) 시작 직전 FlashOta 내부 상태 정리
 *  - 진행 중인 다운로드 중복 시작은 거절
 *  - 리셋 없이 연속 OTA Download/Verify 테스트 가능하도록 상태 복구 강화
 *********************************************************************************************************************/

#include "UdsOta.h"
#include "MCMCAN.h"
#include "FlashOta.h"
#include "SotaUcb.h"

#include <string.h>

/* ============================================================
   Internal state
   ============================================================ */

static UdsOta_DebugInfo_t g_udsOtaDebug;
static boolean g_programmingAllowed = TRUE;

/* ============================================================
   Private prototypes
   ============================================================ */

static void sendPositiveResponse(uint8_t requestSid, const uint8_t *payload, uint8_t payloadLength);
static void sendNegativeResponse(uint8_t requestSid, uint8_t nrc);

static uint16_t readU16Le(const uint8_t *p);
static uint32_t readU32Le(const uint8_t *p);
static void writeU16Le(uint8_t *p, uint16_t value);
static void writeU32Le(uint8_t *p, uint32_t value);

static boolean isDownloadActiveState(void);
static void resetDownloadContextOnly(void);

static void handleDiagnosticSessionControl(const uint8_t *payload, uint8_t length);
static void handleRequestDownload(const uint8_t *payload, uint8_t length);
static void handleTransferData(const uint8_t *payload, uint8_t length);
static void handleRequestTransferExit(const uint8_t *payload, uint8_t length);
static void handleRoutineControl(const uint8_t *payload, uint8_t length);
static void handleEcuReset(const uint8_t *payload, uint8_t length);

/*
 * Target hook
 * 나중에 팀원이 구현한 Flash/CRC/Jump 함수로 이 함수 내부만 교체하면 됨.
 */
static boolean Target_BeginDownload(uint32_t memoryAddress, uint32_t memorySize);
static boolean Target_WriteBlock(uint32_t blockIndex, const uint8_t *data, uint16_t length);
static boolean Target_RequestTransferExit(void);
static boolean Target_CheckCrc32(uint32_t expectedCrc32, uint32_t *calculatedCrc32);
static boolean Target_EcuReset(uint8_t resetType);
static uint32_t selectInactiveSlotAddress(void)
{
    if (Sota_IsGroupBActive() == TRUE)
    {
        return FLASH_OTA_SLOT_A_START_ADDR_C;
    }

    return FLASH_OTA_SLOT_B_START_ADDR_C;
}
/* ============================================================
   Public API
   ============================================================ */

void UdsOta_init(void)
{
    /*
     * FlashOta_Init()은 내부적으로 FlashOta_Reset()을 수행한다.
     * 이후 UdsOta_reset()에서 UDS 상태도 초기화한다.
     */
    FlashOta_Init();
    UdsOta_reset();

    g_programmingAllowed = TRUE;
}

void UdsOta_reset(void)
{
    memset(&g_udsOtaDebug, 0, sizeof(g_udsOtaDebug));

    g_udsOtaDebug.state = UDS_OTA_STATE_IDLE;
    g_udsOtaDebug.lastErrorDetail = 0xFFFFFFFFU;

    g_udsOtaDebug.expectedBlockIndex = 0U;
    g_udsOtaDebug.expectedBlockSequenceCounter = 0x01U;

    /*
     * UDS OTA 전체 리셋 시 FlashOta 내부 상태도 같이 정리한다.
     * 실제 Flash erase는 하지 않고, FlashOta debug/state만 초기화된다.
     */
    FlashOta_Reset();
}

void UdsOta_onRequest(const uint8_t *payload, uint8_t length)
{
    uint8_t sid;

    if ((payload == NULL_PTR) || (length == 0U))
    {
        return;
    }

    sid = payload[0];

    g_udsOtaDebug.requestCount++;
    g_udsOtaDebug.lastRequestSid = sid;
    g_udsOtaDebug.lastRxLength = length;

    switch (sid)
    {
        case UDS_SID_DIAGNOSTIC_SESSION_CONTROL:
        {
            handleDiagnosticSessionControl(payload, length);
            break;
        }

        case UDS_SID_REQUEST_DOWNLOAD:
        {
            handleRequestDownload(payload, length);
            break;
        }

        case UDS_SID_TRANSFER_DATA:
        {
            handleTransferData(payload, length);
            break;
        }

        case UDS_SID_REQUEST_TRANSFER_EXIT:
        {
            handleRequestTransferExit(payload, length);
            break;
        }

        case UDS_SID_ROUTINE_CONTROL:
        {
            handleRoutineControl(payload, length);
            break;
        }

        case UDS_SID_ECU_RESET:
        {
            handleEcuReset(payload, length);
            break;
        }

        default:
        {
            sendNegativeResponse(sid, UDS_NRC_SERVICE_NOT_SUPPORTED);
            break;
        }
    }
}

UdsOta_State_t UdsOta_getState(void)
{
    return g_udsOtaDebug.state;
}

void UdsOta_getDebugInfo(UdsOta_DebugInfo_t *info)
{
    if (info != NULL_PTR)
    {
        memcpy(info, &g_udsOtaDebug, sizeof(UdsOta_DebugInfo_t));
    }
}

void UdsOta_setProgrammingAllowed(boolean allowed)
{
    g_programmingAllowed = allowed;
}

boolean UdsOta_isProgrammingAllowed(void)
{
    return g_programmingAllowed;
}

boolean UdsOta_isDownloadInProgress(void)
{
    return isDownloadActiveState();
}

boolean UdsOta_isCrcVerified(void)
{
    return (g_udsOtaDebug.state == UDS_OTA_STATE_CRC_VERIFIED) ? TRUE : FALSE;
}

/* ============================================================
   UDS response helpers
   ============================================================ */

static void sendPositiveResponse(uint8_t requestSid, const uint8_t *payload, uint8_t payloadLength)
{
    uint8_t response[CANFD_MAX_DLC];

    memset(response, 0, sizeof(response));

    response[0] = (uint8_t)(requestSid + UDS_POSITIVE_RESPONSE_OFFSET);

    if ((payload != NULL_PTR) && (payloadLength > 0U))
    {
        if (payloadLength > (CANFD_MAX_DLC - 1U))
        {
            payloadLength = CANFD_MAX_DLC - 1U;
        }

        memcpy(&response[1], payload, payloadLength);
    }

    g_udsOtaDebug.positiveResponseCount++;
    g_udsOtaDebug.lastResponseSid = response[0];

    (void)CanIf_sendOtaResponse(response, CANFD_MAX_DLC);
}

static void sendNegativeResponse(uint8_t requestSid, uint8_t nrc)
{
    uint8_t response[CANFD_MAX_DLC];

    memset(response, 0, sizeof(response));

    response[0] = UDS_SID_NEGATIVE_RESPONSE;
    response[1] = requestSid;
    response[2] = nrc;

    g_udsOtaDebug.negativeResponseCount++;
    g_udsOtaDebug.lastResponseSid = response[0];
    g_udsOtaDebug.lastNrc = nrc;
    g_udsOtaDebug.state = UDS_OTA_STATE_ERROR;

    (void)CanIf_sendOtaResponse(response, CANFD_MAX_DLC);
}

/* ============================================================
   Utility
   ============================================================ */

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

static boolean isDownloadActiveState(void)
{
    boolean active = FALSE;

    if ((g_udsOtaDebug.state == UDS_OTA_STATE_DOWNLOAD_REQUESTED) ||
        (g_udsOtaDebug.state == UDS_OTA_STATE_TRANSFERRING))
    {
        active = TRUE;
    }

    return active;
}

static void resetDownloadContextOnly(void)
{
    /*
     * 새 다운로드를 시작하기 전에 이전 OTA 결과/오류/전송 상태를 정리한다.
     *
     * 주의:
     *  - requestCount, diagnosticSessionCount 같은 누적 카운터는 유지한다.
     *  - 실제 PFLASH erase는 여기서 하지 않는다.
     *  - FlashOta_Reset()은 FlashOta 내부 상태/debug만 초기화한다.
     */
    g_udsOtaDebug.firmwareSize = 0U;
    g_udsOtaDebug.receivedBytes = 0U;

    g_udsOtaDebug.memoryAddress = 0U;
    g_udsOtaDebug.expectedCrc32 = 0U;
    g_udsOtaDebug.calculatedCrc32 = 0U;

    g_udsOtaDebug.expectedBlockIndex = 0U;
    g_udsOtaDebug.lastBlockIndex = 0U;

    g_udsOtaDebug.expectedBlockSequenceCounter = 0x01U;
    g_udsOtaDebug.lastBlockSequenceCounter = 0U;

    g_udsOtaDebug.lastNrc = 0U;
    g_udsOtaDebug.lastErrorDetail = 0xFFFFFFFFU;

    FlashOta_Reset();
}

/* ============================================================
   UDS handlers
   ============================================================ */

static void handleDiagnosticSessionControl(const uint8_t *payload, uint8_t length)
{
    uint8_t responsePayload[1];

    g_udsOtaDebug.diagnosticSessionCount++;

    if (length < UDS_REQ_LEN_DIAGNOSTIC_SESSION_CONTROL)
    {
        sendNegativeResponse(UDS_SID_DIAGNOSTIC_SESSION_CONTROL,
                             UDS_NRC_INCORRECT_MESSAGE_LENGTH_OR_FORMAT);
        return;
    }

    if (payload[1] != UDS_SESSION_PROGRAMMING)
    {
        sendNegativeResponse(UDS_SID_DIAGNOSTIC_SESSION_CONTROL,
                             UDS_NRC_REQUEST_OUT_OF_RANGE);
        return;
    }

    if (g_programmingAllowed == FALSE)
    {
        sendNegativeResponse(UDS_SID_DIAGNOSTIC_SESSION_CONTROL,
                             UDS_NRC_CONDITIONS_NOT_CORRECT);
        return;
    }

    /*
     * 다운로드 도중 새 programming session 요청이 들어오면
     * 전송 중인 OTA context를 깨뜨릴 수 있으므로 거절한다.
     */
    if (isDownloadActiveState() == TRUE)
    {
        sendNegativeResponse(UDS_SID_DIAGNOSTIC_SESSION_CONTROL,
                             UDS_NRC_CONDITIONS_NOT_CORRECT);
        return;
    }

    /*
     * 이전 OTA가 CRC_VERIFIED / ERROR / RESET_REQUESTED 등에 남아 있어도
     * 새 programming session 진입 시 다음 다운로드가 가능하도록 정리한다.
     */
    resetDownloadContextOnly();

    g_udsOtaDebug.state = UDS_OTA_STATE_PROGRAMMING_SESSION;

    responsePayload[0] = UDS_SESSION_PROGRAMMING;

    sendPositiveResponse(UDS_SID_DIAGNOSTIC_SESSION_CONTROL,
                         responsePayload,
                         1U);
}

static void handleRequestDownload(const uint8_t *payload, uint8_t length)
{
    uint32_t requestedMemoryAddress;
    uint32_t targetMemoryAddress;
    uint32_t memorySize;
    uint8_t responsePayload[3];

    g_udsOtaDebug.requestDownloadCount++;

    /*
     * Simplified RequestDownload:
     * B0      0x34
     * B1      DataFormatIdentifier = 0x00
     * B2      AddressAndLengthFormatIdentifier = 0x44
     * B3~B6   MemoryAddress, little endian
     * B7~B10  MemorySize, little endian
     *
     * 현재 팀 구조:
     *  - client가 보내는 MemoryAddress는 직접 신뢰하지 않는다.
     *  - ECU 내부에서 현재 active group 기준으로 inactive slot을 선택한다.
     *
     * 현재 단독 검증 단계:
     *  - Slot A에서 실행 중이라고 가정
     *  - target = Slot B(0x80320000)
     *
     * 최종 통합 시:
     *  - if (SOTA_IsGroupBActive()) target = Slot A(0x80020000)
     *  - else                       target = Slot B(0x80320000)
     */

    /*
     * 진행 중인 다운로드가 있는데 새 RequestDownload가 들어오면 거절한다.
     * 완료/오류 상태는 0x10 programming session에서 cleanup 후 들어오는 것을 원칙으로 한다.
     */
    if (isDownloadActiveState() == TRUE)
    {
        sendNegativeResponse(UDS_SID_REQUEST_DOWNLOAD,
                             UDS_NRC_CONDITIONS_NOT_CORRECT);
        return;
    }

    if (g_udsOtaDebug.state != UDS_OTA_STATE_PROGRAMMING_SESSION)
    {
        sendNegativeResponse(UDS_SID_REQUEST_DOWNLOAD,
                             UDS_NRC_CONDITIONS_NOT_CORRECT);
        return;
    }

    if (length < UDS_REQ_LEN_REQUEST_DOWNLOAD)
    {
        sendNegativeResponse(UDS_SID_REQUEST_DOWNLOAD,
                             UDS_NRC_INCORRECT_MESSAGE_LENGTH_OR_FORMAT);
        return;
    }

    if ((payload[1] != UDS_DOWNLOAD_DATA_FORMAT_ID) ||
        (payload[2] != UDS_DOWNLOAD_ADDR_LEN_FORMAT))
    {
        sendNegativeResponse(UDS_SID_REQUEST_DOWNLOAD,
                             UDS_NRC_REQUEST_OUT_OF_RANGE);
        return;
    }

    requestedMemoryAddress = readU32Le(&payload[3]);
    memorySize = readU32Le(&payload[7]);

    /*
     * 현재는 client address를 사용하지 않는다.
     * 단, 디버깅 시 확인 가능하도록 변수는 읽어둔다.
     */
    (void)requestedMemoryAddress;

    /*
        * targetMemoryAddress는 현재 active slot 기준으로 inactive slot 주소를 선택한다.
     */
    targetMemoryAddress = selectInactiveSlotAddress();

    g_udsOtaDebug.memoryAddress = targetMemoryAddress;

    if (Target_BeginDownload(targetMemoryAddress, memorySize) == FALSE)
    {
        sendNegativeResponse(UDS_SID_REQUEST_DOWNLOAD,
                            UDS_NRC_GENERAL_PROGRAMMING_FAILURE);
        return;
    }
    if ((memorySize == 0U) ||
        (memorySize > UDS_OTA_MAX_IMAGE_SIZE))
    {
        sendNegativeResponse(UDS_SID_REQUEST_DOWNLOAD,
                             UDS_NRC_REQUEST_OUT_OF_RANGE);
        return;
    }

    /*
     * 새 다운로드 시작 직전 이전 전송 context를 한 번 더 정리한다.
     * 0x10에서 이미 정리했더라도 중복 호출되어도 문제 없다.
     */
    resetDownloadContextOnly();

    if (Target_BeginDownload(targetMemoryAddress, memorySize) == FALSE)
    {
        sendNegativeResponse(UDS_SID_REQUEST_DOWNLOAD,
                             UDS_NRC_GENERAL_PROGRAMMING_FAILURE);
        return;
    }

    g_udsOtaDebug.state = UDS_OTA_STATE_DOWNLOAD_REQUESTED;
    g_udsOtaDebug.memoryAddress = targetMemoryAddress;
    g_udsOtaDebug.firmwareSize = memorySize;
    g_udsOtaDebug.receivedBytes = 0U;
    g_udsOtaDebug.expectedBlockIndex = 0U;
    g_udsOtaDebug.lastBlockIndex = 0U;
    g_udsOtaDebug.expectedBlockSequenceCounter = 0x01U;
    g_udsOtaDebug.lastBlockSequenceCounter = 0U;
    g_udsOtaDebug.expectedCrc32 = 0U;
    g_udsOtaDebug.calculatedCrc32 = 0U;

    /*
     * Positive Response 0x74:
     * B1 = 0x20
     * B2~B3 = MaxNumberOfBlockLength = 32
     */
    responsePayload[0] = 0x20U;
    writeU16Le(&responsePayload[1], UDS_MAX_BLOCK_LENGTH);

    sendPositiveResponse(UDS_SID_REQUEST_DOWNLOAD,
                         responsePayload,
                         3U);
}

static void handleTransferData(const uint8_t *payload, uint8_t length)
{
    uint8_t blockSequenceCounter;
    uint32_t blockIndex;
    uint16_t dataLength;
    uint32_t remaining;
    const uint8_t *firmwareData;
    uint8_t responsePayload[1];

    g_udsOtaDebug.transferDataCount++;

    /*
     * Simplified TransferData:
     * B0    0x36
     * B1    BlockSequenceCounter, 1 byte
     * B2~   Firmware data
     */
    if ((g_udsOtaDebug.state != UDS_OTA_STATE_DOWNLOAD_REQUESTED) &&
        (g_udsOtaDebug.state != UDS_OTA_STATE_TRANSFERRING))
    {
        sendNegativeResponse(UDS_SID_TRANSFER_DATA,
                             UDS_NRC_CONDITIONS_NOT_CORRECT);
        return;
    }

    if (length < UDS_REQ_LEN_TRANSFER_DATA_MIN)
    {
        sendNegativeResponse(UDS_SID_TRANSFER_DATA,
                             UDS_NRC_INCORRECT_MESSAGE_LENGTH_OR_FORMAT);
        return;
    }

    blockSequenceCounter = payload[1];

    if (blockSequenceCounter != g_udsOtaDebug.expectedBlockSequenceCounter)
    {
        sendNegativeResponse(UDS_SID_TRANSFER_DATA,
                             UDS_NRC_WRONG_BLOCK_SEQUENCE_COUNTER);
        return;
    }

    if (g_udsOtaDebug.receivedBytes >= g_udsOtaDebug.firmwareSize)
    {
        sendNegativeResponse(UDS_SID_TRANSFER_DATA,
                             UDS_NRC_REQUEST_OUT_OF_RANGE);
        return;
    }

    remaining = g_udsOtaDebug.firmwareSize - g_udsOtaDebug.receivedBytes;

    if (remaining >= UDS_OTA_TRANSFER_DATA_SIZE)
    {
        dataLength = UDS_OTA_TRANSFER_DATA_SIZE;
    }
    else
    {
        dataLength = (uint16_t)remaining;
    }

    if (length < (uint8_t)(2U + dataLength))
    {
        sendNegativeResponse(UDS_SID_TRANSFER_DATA,
                             UDS_NRC_INCORRECT_MESSAGE_LENGTH_OR_FORMAT);
        return;
    }

    blockIndex = g_udsOtaDebug.expectedBlockIndex;
    firmwareData = &payload[2];

    if (Target_WriteBlock(blockIndex, firmwareData, dataLength) == FALSE)
    {
        sendNegativeResponse(UDS_SID_TRANSFER_DATA,
                             UDS_NRC_GENERAL_PROGRAMMING_FAILURE);
        return;
    }

    g_udsOtaDebug.state = UDS_OTA_STATE_TRANSFERRING;
    g_udsOtaDebug.lastBlockIndex = blockIndex;
    g_udsOtaDebug.lastBlockSequenceCounter = blockSequenceCounter;
    g_udsOtaDebug.receivedBytes += dataLength;

    g_udsOtaDebug.expectedBlockIndex++;
    g_udsOtaDebug.expectedBlockSequenceCounter++;

    /*
     * uint8_t라 0xFF 다음 0x00으로 자연 wrap.
     */

    responsePayload[0] = blockSequenceCounter;

    sendPositiveResponse(UDS_SID_TRANSFER_DATA,
                         responsePayload,
                         1U);
}

static void handleRequestTransferExit(const uint8_t *payload, uint8_t length)
{
    (void)payload;

    g_udsOtaDebug.requestTransferExitCount++;

    if (length < UDS_REQ_LEN_REQUEST_TRANSFER_EXIT)
    {
        sendNegativeResponse(UDS_SID_REQUEST_TRANSFER_EXIT,
                             UDS_NRC_INCORRECT_MESSAGE_LENGTH_OR_FORMAT);
        return;
    }

    if ((g_udsOtaDebug.state != UDS_OTA_STATE_TRANSFERRING) ||
        (g_udsOtaDebug.receivedBytes != g_udsOtaDebug.firmwareSize))
    {
        sendNegativeResponse(UDS_SID_REQUEST_TRANSFER_EXIT,
                             UDS_NRC_CONDITIONS_NOT_CORRECT);
        return;
    }

    if (Target_RequestTransferExit() == FALSE)
    {
        sendNegativeResponse(UDS_SID_REQUEST_TRANSFER_EXIT,
                             UDS_NRC_GENERAL_PROGRAMMING_FAILURE);
        return;
    }

    g_udsOtaDebug.state = UDS_OTA_STATE_TRANSFER_EXIT_DONE;

    sendPositiveResponse(UDS_SID_REQUEST_TRANSFER_EXIT,
                         NULL_PTR,
                         0U);
}

static void handleRoutineControl(const uint8_t *payload, uint8_t length)
{
    uint8_t routineControlType;
    uint16_t routineId;
    uint32_t expectedCrc32;
    uint32_t calculatedCrc32 = 0U;
    uint8_t responsePayload[7];

    g_udsOtaDebug.routineControlCount++;

    /*
     * B0      0x31
     * B1      RoutineControlType = 0x01
     * B2~B3   RoutineIdentifier = 0x0202
     * B4~B7   Expected CRC32
     */
    if (length < UDS_REQ_LEN_ROUTINE_CONTROL_CHECK_CRC32)
    {
        sendNegativeResponse(UDS_SID_ROUTINE_CONTROL,
                             UDS_NRC_INCORRECT_MESSAGE_LENGTH_OR_FORMAT);
        return;
    }

    if (g_udsOtaDebug.state != UDS_OTA_STATE_TRANSFER_EXIT_DONE)
    {
        sendNegativeResponse(UDS_SID_ROUTINE_CONTROL,
                             UDS_NRC_CONDITIONS_NOT_CORRECT);
        return;
    }

    routineControlType = payload[1];
    routineId = readU16Le(&payload[2]);
    expectedCrc32 = readU32Le(&payload[4]);

    if ((routineControlType != UDS_ROUTINE_START) ||
        (routineId != UDS_ROUTINE_ID_CHECK_CRC32))
    {
        sendNegativeResponse(UDS_SID_ROUTINE_CONTROL,
                             UDS_NRC_REQUEST_OUT_OF_RANGE);
        return;
    }

    if (Target_CheckCrc32(expectedCrc32, &calculatedCrc32) == FALSE)
    {
        g_udsOtaDebug.expectedCrc32 = expectedCrc32;
        g_udsOtaDebug.calculatedCrc32 = calculatedCrc32;

        sendNegativeResponse(UDS_SID_ROUTINE_CONTROL,
                             UDS_NRC_GENERAL_PROGRAMMING_FAILURE);
        return;
    }

    g_udsOtaDebug.state = UDS_OTA_STATE_CRC_VERIFIED;
    g_udsOtaDebug.expectedCrc32 = expectedCrc32;
    g_udsOtaDebug.calculatedCrc32 = calculatedCrc32;

    responsePayload[0] = routineControlType;
    writeU16Le(&responsePayload[1], routineId);
    writeU32Le(&responsePayload[3], calculatedCrc32);

    sendPositiveResponse(UDS_SID_ROUTINE_CONTROL,
                         responsePayload,
                         7U);

    /*
     * front-zcu OTA와 동일하게 CRC 값은 bootloader 검증용 flag에 넘긴다.
     * 0x71 응답을 먼저 queue에 올린 뒤 background FlashOta_Service()에서
     * flag 저장과 system reset을 수행한다.
     */
    g_udsOtaDebug.state = UDS_OTA_STATE_RESET_REQUESTED;
    (void)Target_EcuReset(UDS_RESET_JUMP_TO_APP);
}

static void handleEcuReset(const uint8_t *payload, uint8_t length)
{
    uint8_t resetType;
    uint8_t responsePayload[1];

    g_udsOtaDebug.ecuResetCount++;

    if (length < UDS_REQ_LEN_ECU_RESET)
    {
        sendNegativeResponse(UDS_SID_ECU_RESET,
                             UDS_NRC_INCORRECT_MESSAGE_LENGTH_OR_FORMAT);
        return;
    }

    if (g_udsOtaDebug.state != UDS_OTA_STATE_CRC_VERIFIED)
    {
        sendNegativeResponse(UDS_SID_ECU_RESET,
                             UDS_NRC_CONDITIONS_NOT_CORRECT);
        return;
    }

    resetType = payload[1];

    if (resetType != UDS_RESET_JUMP_TO_APP)
    {
        sendNegativeResponse(UDS_SID_ECU_RESET,
                             UDS_NRC_REQUEST_OUT_OF_RANGE);
        return;
    }

    responsePayload[0] = resetType;

    sendPositiveResponse(UDS_SID_ECU_RESET,
                         responsePayload,
                         1U);

    g_udsOtaDebug.state = UDS_OTA_STATE_RESET_REQUESTED;

    /*
     * 현재 단계에서는 실제 jump하지 않는다.
     * FlashOta_RequestJumpToApp()도 pending만 세우고 직접 jump하지 않는다.
     */
    (void)Target_EcuReset(resetType);
}

/* ============================================================
   Target hook functions
   ============================================================ */

/*
 * 아래 함수들은 현재 CAN/UDS 계층 테스트용 placeholder.
 * Flash erase/write/CRC/jump 함수를 완성하면,
 * 여기 내부를 실제 함수 호출로 교체하면 된다.
 */

static boolean Target_BeginDownload(uint32_t memoryAddress, uint32_t memorySize)
{
    return FlashOta_BeginDownload(memoryAddress, memorySize);
}

static boolean Target_WriteBlock(uint32_t blockIndex, const uint8_t *data, uint16_t length)
{
    return FlashOta_WriteBlock(blockIndex, data, length);
}

static boolean Target_RequestTransferExit(void)
{
    return FlashOta_EndTransfer();
}

static boolean Target_CheckCrc32(uint32_t expectedCrc32, uint32_t *calculatedCrc32)
{
    return FlashOta_CheckCrc32(expectedCrc32, calculatedCrc32);
}

static boolean Target_EcuReset(uint8_t resetType)
{
    return FlashOta_RequestJumpToApp(resetType);
}
