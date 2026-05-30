#ifndef UDS_OTA_H_
#define UDS_OTA_H_

/**********************************************************************************************************************
 * \file UdsOta.h
 * \brief Sensor ECU UDS-style OTA Service Layer over CAN FD
 *
 * 역할:
 *  - CAN FD 0x600 UDS Request payload 처리 진입점 제공
 *  - UDS 서비스 상태 관리
 *  - UDS Positive / Negative Response 생성을 위한 상위 API 제공
 *
 * 주의:
 *  - 이 파일은 CAN/UDS 계층 전용이다.
 *  - Flash erase/write/CRC/jump는 이 헤더에서 직접 다루지 않는다.
 *  - 팀원이 구현할 실제 OTA 처리 함수는 UdsOta.c 내부에서 연결한다.
 *
 * CAN ID:
 *  - Request  : CAN_ID_OTA_REQUEST  = 0x600
 *  - Response : CAN_ID_OTA_RESPONSE = 0x601
 *********************************************************************************************************************/

#include <stdint.h>

#include "Ifx_Types.h"
#include "can_type_def.h"

/* ============================================================
   Fallback define
   can_type_def.h에 이미 있으면 그 값을 사용한다.
   ============================================================ */

#ifndef UDS_CANFD_MAX_PAYLOAD_SIZE
#define UDS_CANFD_MAX_PAYLOAD_SIZE    CANFD_MAX_DLC
#endif

#ifndef UDS_TRANSFER_DATA_SIZE
#define UDS_TRANSFER_DATA_SIZE        32U
#endif

#ifndef UDS_MAX_BLOCK_LENGTH
#define UDS_MAX_BLOCK_LENGTH          32U
#endif

#ifndef UDS_APP_START_ADDR
#define UDS_APP_START_ADDR            0x80320000UL
#endif

/* ============================================================
   UDS OTA 설정
   ============================================================ */

/*
 * 현재 구현 기준:
 * - CAN FD Raw Frame 사용
 * - 1 frame payload 최대 64 byte
 * - TransferData 실제 firmware data는 32 byte 단위
 * - 추후 ISO-TP/CAN TP 확장 가능
 */
#define UDS_OTA_CANFD_PAYLOAD_SIZE    UDS_CANFD_MAX_PAYLOAD_SIZE
#define UDS_OTA_TRANSFER_DATA_SIZE    UDS_TRANSFER_DATA_SIZE
#define UDS_OTA_MAX_BLOCK_LENGTH      UDS_MAX_BLOCK_LENGTH
#define UDS_OTA_APP_START_ADDR        UDS_APP_START_ADDR

/*
 * A/B Slot 기준 최대 이미지 크기.
 *
 * Slot A: 0x80020000 ~ 0x802FFFFF
 * Slot B: 0x80320000 ~ 0x805FFFFF
 *
 * Slot size = 0x2E0000 bytes
 */
#define UDS_OTA_MAX_IMAGE_SIZE        0x002E0000U

/* ============================================================
   UDS OTA 상태
   ============================================================ */

typedef enum
{
    UDS_OTA_STATE_IDLE = 0,
    UDS_OTA_STATE_PROGRAMMING_SESSION,
    UDS_OTA_STATE_DOWNLOAD_ERASING,
    UDS_OTA_STATE_DOWNLOAD_REQUESTED,
    UDS_OTA_STATE_TRANSFERRING,
    UDS_OTA_STATE_TRANSFER_EXIT_DONE,
    UDS_OTA_STATE_CRC_VERIFIED,
    UDS_OTA_STATE_READY_TO_ACTIVATE,
    UDS_OTA_STATE_RESET_REQUESTED,
    UDS_OTA_STATE_ERROR,
   
} UdsOta_State_t;

/* ============================================================
   UDS OTA 처리 결과
   ============================================================ */

typedef enum
{
    UDS_OTA_RESULT_OK = 0,
    UDS_OTA_RESULT_REJECT,
    UDS_OTA_RESULT_INVALID_LENGTH,
    UDS_OTA_RESULT_INVALID_SEQUENCE,
    UDS_OTA_RESULT_INVALID_CONDITION,
    UDS_OTA_RESULT_TARGET_ERROR
} UdsOta_Result_t;

/* ============================================================
   UDS OTA 디버그 정보
   ============================================================ */

typedef struct
{
    UdsOta_State_t state;

    uint8_t  lastRequestSid;
    uint8_t  lastResponseSid;
    uint8_t  lastNrc;
    uint8_t  lastRxLength;

    uint32_t requestCount;
    uint32_t positiveResponseCount;
    uint32_t negativeResponseCount;

    uint32_t diagnosticSessionCount;
    uint32_t requestDownloadCount;
    uint32_t transferDataCount;
    uint32_t requestTransferExitCount;
    uint32_t routineControlCount;
    uint32_t ecuResetCount;

    uint32_t firmwareSize;
    uint32_t receivedBytes;

    uint32_t memoryAddress;
    uint32_t expectedCrc32;
    uint32_t calculatedCrc32;

    uint32_t expectedBlockIndex;
    uint32_t lastBlockIndex;

    uint8_t  expectedBlockSequenceCounter;
    uint8_t  lastBlockSequenceCounter;

    uint32_t lastErrorDetail;
} UdsOta_DebugInfo_t;

/* ============================================================
   Public API
   ============================================================ */

/**
 * @brief UDS OTA 계층 초기화
 *
 * Sensor ECU 초기화 시 1회 호출.
 */
void UdsOta_init(void);

/**
 * @brief UDS OTA 상태 초기화
 *
 * OTA 재시작, 오류 복구, 새 다운로드 요청 전 초기화에 사용.
 */
void UdsOta_reset(void);

/**
 * @brief CAN FD 0x600 UDS Request 처리 진입점
 *
 * MCMCAN.c의 CanIf_onReceive()에서 CAN_ID_OTA_REQUEST 수신 시 호출한다.
 *
 * 예:
 * case CAN_ID_OTA_REQUEST:
 * {
 *     UdsOta_onRequest(data, length);
 *     break;
 * }
 *
 * @param payload CAN FD payload pointer
 * @param length  CAN FD payload length
 */
void UdsOta_onRequest(const uint8_t *payload, uint8_t length);

/**
 * @brief UDS OTA background service
 *
 * 0x34 RequestDownload에서 erase를 비동기 시작한 뒤,
 * FlashOta erase 완료를 감시하고 0x74 지연 응답을 송신한다.
 * CPU0 main loop에서 주기적으로 호출해야 한다.
 */
void UdsOta_Service(void);

/**
 * @brief 현재 UDS OTA 상태 반환
 */
UdsOta_State_t UdsOta_getState(void);

/**
 * @brief UDS OTA 디버그 정보 복사
 *
 * @param info 디버그 정보를 받을 포인터
 */
void UdsOta_getDebugInfo(UdsOta_DebugInfo_t *info);

/**
 * @brief OTA 허용 조건 설정
 *
 * 예:
 * - Gear P일 때만 OTA 허용
 * - 차량 정지 상태일 때만 OTA 허용
 *
 * VehicleState 수신 결과나 ZCU 정책에 따라 MCMCAN.c 또는 상위 모듈에서 설정 가능.
 */
void UdsOta_setProgrammingAllowed(boolean allowed);

/**
 * @brief OTA 허용 조건 반환
 */
boolean UdsOta_isProgrammingAllowed(void);

/**
 * @brief 현재 다운로드가 진행 중인지 확인
 */
boolean UdsOta_isDownloadInProgress(void);

/**
 * @brief CRC 검증 완료 여부 확인
 */
boolean UdsOta_isCrcVerified(void);



#endif /* UDS_OTA_H_ */
