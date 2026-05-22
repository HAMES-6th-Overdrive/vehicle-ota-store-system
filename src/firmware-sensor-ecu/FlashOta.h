#ifndef FLASH_OTA_H_
#define FLASH_OTA_H_

/**********************************************************************************************************************
 * \file FlashOta.h
 * \brief Sensor ECU Flash OTA Core
 *
 * 역할:
 *  - Inactive Slot / Bank B 영역 erase
 *  - 32-byte PFLASH page write
 *  - read-back verify
 *  - CRC32 계산/검증
 *  - 추후 Bootloader / UCB_SWAP 기반 activation 연동
 *
 * UdsOta.c는 UDS 프로토콜을 처리하고,
 * 실제 Flash 동작은 이 FlashOta 모듈을 호출한다.
 *********************************************************************************************************************/

#include "Ifx_Types.h"
#include <stdint.h>

/* ============================================================
   Slot / Bank address map
   ============================================================ */

/*
 * Slot A / Active App 후보
 *
 * 현재 기존 Application이 실행되는 영역.
 * 기존 Single Slot OTA 코드에서는 이 영역에 직접 erase/write 했었다.
 */
#define FLASH_OTA_SLOT_A_START_ADDR_C        0x80020000U
#define FLASH_OTA_SLOT_A_START_ADDR_NC       0xA0020000U

/*
 * Slot B / Bank B / Inactive OTA Download Target
 *
 * 팀원 구조 기준 OTA로 새 firmware를 미리 저장할 영역.
 * 주행 중 다운로드/검증 대상은 이 영역으로 잡는다.
 */
#define FLASH_OTA_SLOT_B_START_ADDR_C        0x80320000U
#define FLASH_OTA_SLOT_B_START_ADDR_NC       0xA0320000U

/*
 * 현재 단계에서 OTA Download Target은 Slot B로 설정한다.
 *
 * 1차 목표:
 *  - 새 firmware를 Slot B에 저장
 *  - CRC 검증 성공
 *  - 아직 실행 전환은 하지 않음
 */
#define FLASH_OTA_DOWNLOAD_TARGET_ADDR_C     FLASH_OTA_SLOT_B_START_ADDR_C
#define FLASH_OTA_DOWNLOAD_TARGET_ADDR_NC    FLASH_OTA_SLOT_B_START_ADDR_NC

/*
 * 기존 코드 호환용 alias.
 *
 * FlashOta.c 수정 전까지 기존 이름을 깨지 않기 위해 유지한다.
 * 단, 앞으로 새 코드에서는 FLASH_OTA_DOWNLOAD_TARGET_ADDR_* 사용을 권장한다.
 */
#define FLASH_OTA_APP_START_ADDR_C           FLASH_OTA_SLOT_A_START_ADDR_C
#define FLASH_OTA_APP_START_ADDR_NC          FLASH_OTA_SLOT_A_START_ADDR_NC

/* ============================================================
   Flash size policy
   ============================================================ */

#define FLASH_OTA_PAGE_SIZE                  32U
#define FLASH_OTA_SECTOR_SIZE_BYTES          0x2000U
#define FLASH_OTA_MAX_IMAGE_SIZE             0x30000U

/* ============================================================
   Debug status
   ============================================================ */

typedef struct
{
    boolean started;
    boolean transferExitDone;
    boolean crcVerified;

    uint32_t targetAddress;
    uint32_t firmwareSize;
    uint32_t receivedBytes;

    uint32_t expectedCrc32;
    uint32_t calculatedCrc32;

    uint16_t lastBlockIndex;
    uint32_t lastWriteAddress;
    uint32_t verifyFailOffset;

    uint32_t eraseCount;
    uint32_t writeOkCount;
    uint32_t writeFailCount;
} FlashOta_DebugInfo_t;

void FlashOta_Init(void);
void FlashOta_Reset(void);

boolean FlashOta_BeginDownload(uint32_t targetAddress, uint32_t firmwareSize);

boolean FlashOta_WriteBlock(uint16_t blockIndex,
                            const uint8_t *data,
                            uint16_t length);

boolean FlashOta_EndTransfer(void);

boolean FlashOta_CheckCrc32(uint32_t expectedCrc32,
                            uint32_t *calculatedCrc32);

boolean FlashOta_RequestJumpToApp(uint8_t resetType);

void FlashOta_GetDebugInfo(FlashOta_DebugInfo_t *info);

boolean FlashOta_IsJumpPending(void);
void FlashOta_Service(void);

#endif /* FLASH_OTA_H_ */
