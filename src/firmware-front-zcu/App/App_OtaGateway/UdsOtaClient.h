#ifndef UDS_OTA_CLIENT_H_
#define UDS_OTA_CLIENT_H_

/**********************************************************************************************************************
 * \file UdsOtaClient.h
 * \brief ZCU UDS-style OTA Client over CAN FD for FreeRTOS App_Can
 *
 * 역할:
 *  - ZCU가 Sensor ECU로 UDS OTA Request를 송신한다.
 *  - Sensor ECU의 UDS Response를 수신하여 상태머신을 진행한다.
 *
 * CAN:
 *  - TX: 0x600 UDS Request   ZCU -> Sensor ECU
 *  - RX: 0x601 UDS Response  Sensor ECU -> ZCU
 *
 * 구조:
 *  - UdsOtaClient는 App_Can을 통해 CAN FD raw frame을 송신한다.
 *  - App_Can은 CAN ID 의미를 해석하지 않는다.
 *  - 0x601 수신 frame은 App_OtaGateway 쪽에서 AppCan_RecvById()로 꺼낸 뒤
 *    UdsOtaClient_OnResponse()에 전달한다.
 *
 * Streaming Gateway 구조:
 *  - ZCU는 전체 firmware binary를 저장하지 않는다.
 *  - firmwareSize / 현재 32-byte block만 보관한다.
 *  - CRC32는 두 가지 모드를 지원한다.
 *
 * CRC 모드:
 *  1. 기존 모드
 *     - UdsOtaClient_StartStream(firmwareSize, crc32)
 *     - OTA 시작 시점에 CRC를 이미 알고 있다.
 *
 *  2. Late CRC 모드
 *     - UdsOtaClient_StartStreamWithoutCrc(firmwareSize)
 *     - Pi/HPC -> ZCU DoIP 흐름처럼 CRC가 마지막 0x37에서 들어오는 경우 사용한다.
 *     - 모든 block 전송 완료 후 WAIT_FINAL_CRC 상태에서 대기한다.
 *     - 이후 UdsOtaClient_SetFinalCrc(crc32)가 호출되면
 *       Sensor ECU 쪽 RequestTransferExit + RoutineControl CRC를 진행한다.
 *********************************************************************************************************************/

#include "Ifx_Types.h"
#include <stdint.h>

/* ============================================================
   CAN / UDS fallback define
   ============================================================ */

#ifndef CAN_ID_OTA_REQUEST
#define CAN_ID_OTA_REQUEST                 0x600U
#endif

#ifndef CAN_ID_OTA_RESPONSE
#define CAN_ID_OTA_RESPONSE                0x601U
#endif

#ifndef CANFD_MAX_DLC
#define CANFD_MAX_DLC                      64U
#endif

#ifndef UDS_TRANSFER_DATA_SIZE
#define UDS_TRANSFER_DATA_SIZE             32U
#endif

#ifndef UDS_MAX_BLOCK_LENGTH
#define UDS_MAX_BLOCK_LENGTH               32U
#endif

/*
 * Sensor ECU SOTA slot 기준:
 *  - Slot A / App Start 후보: 0x80020000
 *  - Slot B / App Start 후보: 0x80320000
 *
 * 현재 ZCU -> Sensor ECU download/verify 테스트는
 * 현재 A active라고 가정하고 B slot인 0x80320000으로 전송한다.
 *
 * 최종 SOTA 구조에서는 현재 active group을 보고
 * inactive slot 주소를 선택해야 한다.
 */
#ifndef UDS_APP_START_ADDR
#define UDS_APP_START_ADDR                 0x80320000U
#endif

#ifndef UDS_SID_DIAGNOSTIC_SESSION_CONTROL
#define UDS_SID_DIAGNOSTIC_SESSION_CONTROL 0x10U
#endif

#ifndef UDS_SID_ECU_RESET
#define UDS_SID_ECU_RESET                  0x11U
#endif

#ifndef UDS_SID_ROUTINE_CONTROL
#define UDS_SID_ROUTINE_CONTROL            0x31U
#endif

#ifndef UDS_SID_REQUEST_DOWNLOAD
#define UDS_SID_REQUEST_DOWNLOAD           0x34U
#endif

#ifndef UDS_SID_TRANSFER_DATA
#define UDS_SID_TRANSFER_DATA              0x36U
#endif

#ifndef UDS_SID_REQUEST_TRANSFER_EXIT
#define UDS_SID_REQUEST_TRANSFER_EXIT      0x37U
#endif

#ifndef UDS_SID_NEGATIVE_RESPONSE
#define UDS_SID_NEGATIVE_RESPONSE          0x7FU
#endif

#ifndef UDS_POSITIVE_RESPONSE_OFFSET
#define UDS_POSITIVE_RESPONSE_OFFSET       0x40U
#endif

#ifndef UDS_SESSION_PROGRAMMING
#define UDS_SESSION_PROGRAMMING            0x02U
#endif

#ifndef UDS_DOWNLOAD_DATA_FORMAT_ID
#define UDS_DOWNLOAD_DATA_FORMAT_ID        0x00U
#endif

#ifndef UDS_DOWNLOAD_ADDR_LEN_FORMAT
#define UDS_DOWNLOAD_ADDR_LEN_FORMAT       0x44U
#endif

#ifndef UDS_ROUTINE_START
#define UDS_ROUTINE_START                  0x01U
#endif

#ifndef UDS_ROUTINE_ID_CHECK_CRC32
#define UDS_ROUTINE_ID_CHECK_CRC32         0x0202U
#endif

#ifndef UDS_RESET_JUMP_TO_APP
#define UDS_RESET_JUMP_TO_APP              0x01U
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
   UDS OTA Client 설정
   ============================================================ */

#define UDS_OTA_CLIENT_CANFD_PAYLOAD_SIZE      CANFD_MAX_DLC
#define UDS_OTA_CLIENT_TRANSFER_DATA_SIZE      UDS_TRANSFER_DATA_SIZE
#define UDS_OTA_CLIENT_TARGET_APP_ADDR         UDS_APP_START_ADDR

/*
 * Timeout은 UdsOtaClient_MainFunction() 호출 주기에 맞춰 tick 단위로 사용한다.
 * 예:
 *  - MainFunction이 1ms마다 호출되면 2000 = 2초
 */
#define UDS_OTA_CLIENT_TIMEOUT_TICKS           2000U
#define UDS_OTA_CLIENT_TRANSFER_TIMEOUT_TICKS  50000U
#define UDS_OTA_CLIENT_CRC_TIMEOUT_TICKS       10000U

/* ============================================================
   OTA Client 상태
   ============================================================ */

typedef enum
{
    UDS_OTA_CLIENT_STATE_IDLE = 0,

    UDS_OTA_CLIENT_STATE_SEND_DIAGNOSTIC_SESSION,
    UDS_OTA_CLIENT_STATE_WAIT_DIAGNOSTIC_SESSION,

    UDS_OTA_CLIENT_STATE_SEND_REQUEST_DOWNLOAD,
    UDS_OTA_CLIENT_STATE_WAIT_REQUEST_DOWNLOAD,

    UDS_OTA_CLIENT_STATE_WAIT_STREAM_BLOCK,

    UDS_OTA_CLIENT_STATE_SEND_TRANSFER_DATA,
    UDS_OTA_CLIENT_STATE_WAIT_TRANSFER_DATA,

    /*
     * Late CRC mode 전용 상태.
     *
     * 모든 firmware block을 Sensor ECU로 전송한 뒤,
     * Pi/HPC가 0x37 단계에서 CRC32를 줄 때까지 여기서 대기한다.
     *
     * UdsOtaClient_SetFinalCrc()가 호출되면
     * SEND_REQUEST_TRANSFER_EXIT 상태로 진행한다.
     */
    UDS_OTA_CLIENT_STATE_WAIT_FINAL_CRC,

    UDS_OTA_CLIENT_STATE_SEND_REQUEST_TRANSFER_EXIT,
    UDS_OTA_CLIENT_STATE_WAIT_REQUEST_TRANSFER_EXIT,

    UDS_OTA_CLIENT_STATE_SEND_ROUTINE_CONTROL_CRC,
    UDS_OTA_CLIENT_STATE_WAIT_ROUTINE_CONTROL_CRC,

    UDS_OTA_CLIENT_STATE_SEND_ECU_RESET,
    UDS_OTA_CLIENT_STATE_WAIT_ECU_RESET,

    UDS_OTA_CLIENT_STATE_DONE,
    UDS_OTA_CLIENT_STATE_ERROR
} UdsOtaClient_State_t;

/* ============================================================
   OTA Client 결과
   ============================================================ */

typedef enum
{
    UDS_OTA_CLIENT_RESULT_OK = 0,
    UDS_OTA_CLIENT_RESULT_BUSY,
    UDS_OTA_CLIENT_RESULT_INVALID_PARAM,
    UDS_OTA_CLIENT_RESULT_CAN_TX_ERROR,
    UDS_OTA_CLIENT_RESULT_TIMEOUT,
    UDS_OTA_CLIENT_RESULT_NEGATIVE_RESPONSE,
    UDS_OTA_CLIENT_RESULT_UNEXPECTED_RESPONSE,
    UDS_OTA_CLIENT_RESULT_CRC_MISMATCH,
    UDS_OTA_CLIENT_RESULT_ERROR
} UdsOtaClient_Result_t;

/* ============================================================
   Debug 정보
   ============================================================ */

typedef struct
{
    UdsOtaClient_State_t  state;
    UdsOtaClient_Result_t lastResult;

    uint32_t firmwareSize;
    uint32_t firmwareCrc32;
    uint32_t targetAddress;

    uint32_t totalBlocks;
    uint32_t currentBlockIndex;
    uint32_t currentOffset;
    uint32_t sentBytes;

    uint8_t  currentBsc;
    uint8_t  lastRxSid;
    uint8_t  lastRxNrc;
    uint8_t  lastExpectedSid;

    uint32_t requestCount;
    uint32_t responseCount;
    uint32_t negativeResponseCount;
    uint32_t timeoutCount;
    uint32_t canTxErrorCount;

    uint32_t tickCount;
    uint32_t stateEnterTick;
    uint32_t lastProgressPercent;

    uint32_t calculatedCrc32FromEcu;

    /*
     * Late CRC mode 확인용.
     *
     * finalCrcProvided:
     *  - TRUE  : CRC32를 이미 알고 있음
     *  - FALSE : WAIT_FINAL_CRC에서 상위 계층의 CRC32 입력을 기다릴 수 있음
     */
    boolean finalCrcProvided;

} UdsOtaClient_DebugInfo_t;

/* ============================================================
   Public API
   ============================================================ */

void UdsOtaClient_Init(void);

void UdsOtaClient_Reset(void);

/**
 * @brief OTA download 시작 - streaming mode, CRC known
 *
 * ZCU는 전체 firmware buffer를 저장하지 않는다.
 * firmwareSize와 crc32만 가지고 Sensor ECU에 RequestDownload를 시작한다.
 *
 * 이후 UdsOtaClient가 WAIT_STREAM_BLOCK 상태가 되면,
 * 상위 계층이 UdsOtaClient_ProvideStreamBlock()으로 현재 block을 제공해야 한다.
 *
 * 사용 예:
 *  - 기존 Gateway AutoTest
 *  - Receiver SelfTest
 *  - 시작 시점에 CRC32를 이미 알고 있는 입력 계층
 */
UdsOtaClient_Result_t UdsOtaClient_StartStream(uint32_t firmwareSize,
                                               uint32_t crc32);

/**
 * @brief OTA download 시작 - streaming mode, CRC later
 *
 * Pi/HPC -> ZCU DoIP 흐름에서는 CRC32가 마지막 0x37에서 들어올 수 있다.
 * 이 함수는 firmwareSize만으로 download를 시작한다.
 *
 * 모든 block 전송이 끝나면 UDS_OTA_CLIENT_STATE_WAIT_FINAL_CRC 상태에서 대기한다.
 * 이후 UdsOtaClient_SetFinalCrc()가 호출되면
 * Sensor ECU 쪽 RequestTransferExit + RoutineControl CRC를 진행한다.
 */
UdsOtaClient_Result_t UdsOtaClient_StartStreamWithoutCrc(uint32_t firmwareSize);

/**
 * @brief 마지막 0x37 단계에서 받은 CRC32 설정
 *
 * 호출 조건:
 *  - UdsOtaClient_IsWaitingFinalCrc() == TRUE
 *
 * 주의:
 *  - crc32 == 0x00000000도 이론상 유효한 CRC일 수 있으므로 reject하지 않는다.
 */
UdsOtaClient_Result_t UdsOtaClient_SetFinalCrc(uint32_t crc32);

/**
 * @brief OTA Client main function
 *
 * 주기적으로 호출해야 한다.
 * 권장:
 *  - 1ms 주기 App_OtaGateway task에서 호출
 */
void UdsOtaClient_MainFunction(void);

/**
 * @brief Sensor ECU에서 온 0x601 UDS Response 전달
 *
 * App_Can은 CAN ID 의미를 해석하지 않는다.
 * 따라서 App_OtaGateway 쪽에서 AppCan_RecvById(0x601, ...)로 frame을 꺼낸 뒤
 * 이 함수에 payload를 넘긴다.
 */
void UdsOtaClient_OnResponse(const uint8_t *data, uint8_t length);

UdsOtaClient_State_t UdsOtaClient_GetState(void);

UdsOtaClient_Result_t UdsOtaClient_GetLastResult(void);

boolean UdsOtaClient_IsBusy(void);

boolean UdsOtaClient_IsDone(void);

boolean UdsOtaClient_IsError(void);

boolean UdsOtaClient_IsWaitingStreamBlock(void);

/**
 * @brief 모든 block 전송 후 최종 CRC 입력을 기다리는지 확인
 *
 * Late CRC mode에서만 TRUE가 된다.
 */
boolean UdsOtaClient_IsWaitingFinalCrc(void);

uint32_t UdsOtaClient_GetRequestedBlockIndex(void);

uint32_t UdsOtaClient_GetRequestedOffset(void);

uint8_t UdsOtaClient_GetRequestedBlockLength(void);

/**
 * @brief 현재 요청된 firmware block 제공
 *
 * 호출 조건:
 *  - UdsOtaClient_IsWaitingStreamBlock() == TRUE
 *  - blockIndex == UdsOtaClient_GetRequestedBlockIndex()
 *  - length == UdsOtaClient_GetRequestedBlockLength()
 */
UdsOtaClient_Result_t UdsOtaClient_ProvideStreamBlock(uint32_t blockIndex,
                                                      const uint8_t *data,
                                                      uint8_t length);

/**
 * @brief 진행률 반환
 *
 * @return 0~100 [%]
 */
uint8_t UdsOtaClient_GetProgress(void);

void UdsOtaClient_GetDebugInfo(UdsOtaClient_DebugInfo_t *info);

#endif /* UDS_OTA_CLIENT_H_ */
