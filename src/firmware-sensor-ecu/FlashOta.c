/**********************************************************************************************************************
 * \file FlashOta.c
 * \brief Sensor ECU Flash OTA Core - Dual Slot targetAddress based version
 *
 * 목표:
 *  - 현재 active slot의 반대편 inactive slot에 새 firmware를 저장한다.
 *  - Slot A 또는 Slot B 영역에 erase / write / read-back verify / CRC32 검증을 수행한다.
 *  - UCB_SWAP, App jump, 실행 Slot 전환은 수행하지 않는다.
 *
 * 주소 정책:
 *  - Slot A / App 후보:
 *      Cached    : 0x80020000
 *      Noncached : 0xA0020000
 *
 *  - Slot B / App 후보:
 *      Cached    : 0x80320000
 *      Noncached : 0xA0320000
 *
 * 전제:
 *  - UDS RequestDownload 단계에서 현재 active group의 반대편 주소를 targetAddress로 넘긴다.
 *  - A active이면 targetAddress = 0x80320000
 *  - B active이면 targetAddress = 0x80020000
 *********************************************************************************************************************/

#include "FlashOta.h"

#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include "IfxScuRcu.h"
#include "IfxFlash.h"
#include "IfxStm.h"
#include "SotaUcb.h"
#include "SensorOtaFlash.h"

#include <string.h>
// debug
typedef enum
{
    FLASH_OTA_FAIL_NONE = 0,
    FLASH_OTA_FAIL_NOT_STARTED,
    FLASH_OTA_FAIL_BAD_ARG,
    FLASH_OTA_FAIL_NO_TARGET,
    FLASH_OTA_FAIL_OFFSET_RANGE,
    FLASH_OTA_FAIL_LENGTH_RANGE,
    FLASH_OTA_FAIL_ERASE,
    FLASH_OTA_FAIL_WRITE,
    FLASH_OTA_FAIL_VERIFY
} FlashOtaFailReason_t;

typedef struct
{
    uint32_t failReason;

    uint32_t blockIndex;
    uint32_t offset;
    uint32_t length;
    uint32_t remaining;
    uint32_t firmwareSize;

    uint32_t targetBaseNc;
    uint32_t writeAddressNc;
    uint32_t writeEndAddressNc;

    uint32_t eraseAddressNc;
    uint32_t erasedUntilNc;

    uint32_t flashType;
    uint32_t dmuErr;

    uint32_t verifyIndex;
    uint32_t verifyOffset;
    uint32_t verifyExpected;
    uint32_t verifyActual;

    uint32_t writeOkCount;
    uint32_t writeFailCount;
    uint32_t eraseCount;

} FlashOta_WriteDebug_t;
volatile FlashOta_WriteDebug_t g_flashOtaWriteDebug;

/* ============================================================
   Internal state
   ============================================================ */

static FlashOta_DebugInfo_t g_flashOtaDebug;
/*
 * 1: TransferData 중 32-byte page write 직후 byte-by-byte read-back verify 수행
 * 0: 즉시 verify 생략. 최종 CRC 검증에서 판단.
 *
 * SOTA alternative mapping 상태에서는 A/B alias 때문에 즉시 read-back이
 * 실제 write 결과와 다르게 보일 수 있으므로, 현재는 0 권장.
 */
#define FLASH_OTA_ENABLE_WRITE_VERIFY   0U
/*
 * 현재 단계에서는 이름은 유지하지만,
 * 실제 App jump 또는 UCB_SWAP은 수행하지 않는다.
 *
 * 최종 구조:
 *  - App/OTA 영역: write / read-back / CRC verify / flag 저장 / reset 요청
 *  - Bootloader 영역: flag 확인 / SOTA_UCB_SWAP / reset / rollback
 */
static volatile boolean g_jumpPending = FALSE;
static volatile uint8_t g_pendingResetType = 0U;

/*
 * 실제 erase / write / CRC 대상 주소.
 *
 * FlashOta_BeginDownload(targetAddress, firmwareSize)에서
 * Slot A 또는 Slot B 중 하나로 결정된다.
 */
static uint32_t g_downloadTargetAddrC  = 0U;
static uint32_t g_downloadTargetAddrNC = 0U;
static uint32_t g_erasedUntilAddrNC = 0U;

/* ============================================================
   Private prototypes
   ============================================================ */

static void delayMs(uint32 ms);

static IfxFlash_FlashType getPFlashTypeFromAddress(uint32 addr);

/* ============================================================
   Public API
   ============================================================ */

void FlashOta_Init(void)
{
    FlashOta_Reset();
}

void FlashOta_Reset(void)
{
    memset(&g_flashOtaDebug, 0, sizeof(g_flashOtaDebug));
    g_flashOtaDebug.verifyFailOffset = 0xFFFFFFFFU;

    /*
     * Download target은 FlashOta_BeginDownload()에서 결정한다.
     * A active이면 Slot B, B active이면 Slot A가 될 수 있다.
     */
    g_downloadTargetAddrC  = 0U;
    g_downloadTargetAddrNC = 0U;
    g_erasedUntilAddrNC = 0U;

    g_jumpPending = FALSE;
    g_pendingResetType = 0U;
}

boolean FlashOta_BeginDownload(uint32_t targetAddress, uint32_t firmwareSize)
{
    /*
     * 새 다운로드 시작 시 FlashOta 내부 상태 초기화.
     */
    FlashOta_Reset();

    /*
     * 최종 A/B OTA 구조:
     *
     *  - A active이면 Slot B(0x80320000)에 다운로드
     *  - B active이면 Slot A(0x80020000)에 다운로드
     *
     * 따라서 targetAddress는 Slot A 또는 Slot B 시작 주소만 허용한다.
     */
    if ((targetAddress != FLASH_OTA_SLOT_A_START_ADDR_C) &&
        (targetAddress != FLASH_OTA_SLOT_B_START_ADDR_C))
    {
        return FALSE;
    }

    if ((firmwareSize == 0U) || (firmwareSize > FLASH_OTA_MAX_IMAGE_SIZE))
    {
        return FALSE;
    }

    /*
     * 실제 download target 설정.
     * Flash command는 non-cached 주소를 사용한다.
     */
    g_downloadTargetAddrC = targetAddress;

    if (targetAddress == FLASH_OTA_SLOT_A_START_ADDR_C)
    {
        g_downloadTargetAddrNC = FLASH_OTA_SLOT_A_START_ADDR_NC;
    }
    else
    {
        g_downloadTargetAddrNC = FLASH_OTA_SLOT_B_START_ADDR_NC;
    }

    g_flashOtaDebug.targetAddress = g_downloadTargetAddrC;
    g_flashOtaDebug.firmwareSize = firmwareSize;
    g_flashOtaDebug.receivedBytes = 0U;
    g_flashOtaDebug.started = TRUE;
    g_erasedUntilAddrNC = g_downloadTargetAddrNC & ~(FLASH_OTA_SECTOR_SIZE_BYTES - 1U);

    return TRUE;
}

boolean FlashOta_WriteBlock(uint32_t blockIndex,
                            const uint8_t *data,
                            uint16_t length)
{
    uint8_t page[FLASH_OTA_PAGE_SIZE];
    uint32_t targetAddrNc;
    uint32_t offset;
    uint32_t remaining;
    uint32_t writeEndAddrNc;
    IfxFlash_FlashType flashType;

#if (FLASH_OTA_ENABLE_WRITE_VERIFY != 0U)
    volatile const uint8 *readPtr;
#endif

    /*
     * 공통 debug 초기 기록
     */
    g_flashOtaWriteDebug.failReason = FLASH_OTA_FAIL_NONE;
    g_flashOtaWriteDebug.blockIndex = blockIndex;
    g_flashOtaWriteDebug.length = length;
    g_flashOtaWriteDebug.targetBaseNc = g_downloadTargetAddrNC;
    g_flashOtaWriteDebug.firmwareSize = g_flashOtaDebug.firmwareSize;
    g_flashOtaWriteDebug.erasedUntilNc = g_erasedUntilAddrNC;
    g_flashOtaWriteDebug.dmuErr = MODULE_DMU.HF_ERRSR.U;

    /*
     * verify 관련 debug는 이전 실패값이 남으면 헷갈리므로 매 block 초기화
     */
    g_flashOtaWriteDebug.verifyIndex = 0U;
    g_flashOtaWriteDebug.verifyOffset = 0U;
    g_flashOtaWriteDebug.verifyExpected = 0U;
    g_flashOtaWriteDebug.verifyActual = 0U;

    if (g_flashOtaDebug.started == FALSE)
    {
        g_flashOtaWriteDebug.failReason = FLASH_OTA_FAIL_NOT_STARTED;
        g_flashOtaWriteDebug.writeFailCount++;
        g_flashOtaDebug.writeFailCount++;
        return FALSE;
    }

    if ((data == NULL_PTR) || (length == 0U) || (length > FLASH_OTA_PAGE_SIZE))
    {
        g_flashOtaWriteDebug.failReason = FLASH_OTA_FAIL_BAD_ARG;
        g_flashOtaWriteDebug.writeFailCount++;
        g_flashOtaDebug.writeFailCount++;
        return FALSE;
    }

    if (g_downloadTargetAddrNC == 0U)
    {
        g_flashOtaWriteDebug.failReason = FLASH_OTA_FAIL_NO_TARGET;
        g_flashOtaWriteDebug.writeFailCount++;
        g_flashOtaDebug.writeFailCount++;
        return FALSE;
    }

    offset = ((uint32_t)blockIndex * FLASH_OTA_PAGE_SIZE);
    g_flashOtaWriteDebug.offset = offset;

    if (offset >= g_flashOtaDebug.firmwareSize)
    {
        g_flashOtaWriteDebug.failReason = FLASH_OTA_FAIL_OFFSET_RANGE;
        g_flashOtaWriteDebug.writeFailCount++;
        g_flashOtaDebug.writeFailCount++;
        return FALSE;
    }

    remaining = g_flashOtaDebug.firmwareSize - offset;
    g_flashOtaWriteDebug.remaining = remaining;

    if (length > remaining)
    {
        g_flashOtaWriteDebug.failReason = FLASH_OTA_FAIL_LENGTH_RANGE;
        g_flashOtaWriteDebug.writeFailCount++;
        g_flashOtaDebug.writeFailCount++;
        return FALSE;
    }

    /*
     * 마지막 block이 32바이트보다 작을 수 있으므로 나머지는 0xFF padding.
     * CRC는 firmwareSize만큼만 계산하므로 padding은 CRC에 포함되지 않음.
     */
    memset(page, 0xFF, sizeof(page));
    memcpy(page, data, length);

    targetAddrNc = g_downloadTargetAddrNC + offset;
    writeEndAddrNc = targetAddrNc + FLASH_OTA_PAGE_SIZE;

    g_flashOtaWriteDebug.writeAddressNc = targetAddrNc;
    g_flashOtaWriteDebug.writeEndAddressNc = writeEndAddrNc;

    /*
     * 필요한 sector를 on-demand erase.
     */
    while (writeEndAddrNc > g_erasedUntilAddrNC)
    {
        flashType = getPFlashTypeFromAddress(g_erasedUntilAddrNC);

        g_flashOtaWriteDebug.eraseAddressNc = g_erasedUntilAddrNC;
        g_flashOtaWriteDebug.erasedUntilNc = g_erasedUntilAddrNC;
        g_flashOtaWriteDebug.flashType = (uint32_t)flashType;

        if (SensorOtaFlash_Erase(g_erasedUntilAddrNC,
                                 FLASH_OTA_SECTOR_SIZE_BYTES,
                                 flashType) == FALSE)
        {
            g_flashOtaWriteDebug.failReason = FLASH_OTA_FAIL_ERASE;
            g_flashOtaWriteDebug.dmuErr = MODULE_DMU.HF_ERRSR.U;
            g_flashOtaWriteDebug.writeFailCount++;

            g_flashOtaDebug.writeFailCount++;
            return FALSE;
        }

        g_erasedUntilAddrNC += FLASH_OTA_SECTOR_SIZE_BYTES;

        g_flashOtaWriteDebug.eraseCount++;
        g_flashOtaWriteDebug.erasedUntilNc = g_erasedUntilAddrNC;

        g_flashOtaDebug.eraseCount++;
    }

    /*
     * Write 직전 주소 기록.
     */
    g_flashOtaDebug.lastWriteAddress = targetAddrNc;

    flashType = getPFlashTypeFromAddress(targetAddrNc);
    g_flashOtaWriteDebug.flashType = (uint32_t)flashType;

    if (SensorOtaFlash_Write(targetAddrNc,
                             page,
                             FLASH_OTA_PAGE_SIZE,
                             flashType) == FALSE)
    {
        g_flashOtaWriteDebug.failReason = FLASH_OTA_FAIL_WRITE;
        g_flashOtaWriteDebug.dmuErr = MODULE_DMU.HF_ERRSR.U;
        g_flashOtaWriteDebug.writeFailCount++;

        g_flashOtaDebug.writeFailCount++;
        return FALSE;
    }

    /*
     * SOTA alternative mapping 상태에서는 즉시 read-back alias가
     * 실제 write 대상 physical bank와 다르게 보일 수 있으므로,
     * byte-by-byte verify는 전처리기로 선택한다.
     *
     * 현재는 bootloader/CRC 단계에서 전체 firmware 검증 예정.
     */
#if (FLASH_OTA_ENABLE_WRITE_VERIFY != 0U)
    readPtr = (volatile const uint8 *)targetAddrNc;

    for (uint32_t i = 0U; i < FLASH_OTA_PAGE_SIZE; i++)
    {
        if (readPtr[i] != page[i])
        {
            g_flashOtaWriteDebug.failReason = FLASH_OTA_FAIL_VERIFY;
            g_flashOtaWriteDebug.verifyIndex = i;
            g_flashOtaWriteDebug.verifyOffset = offset + i;
            g_flashOtaWriteDebug.verifyExpected = page[i];
            g_flashOtaWriteDebug.verifyActual = readPtr[i];
            g_flashOtaWriteDebug.writeFailCount++;

            g_flashOtaDebug.verifyFailOffset = offset + i;
            g_flashOtaDebug.writeFailCount++;
            return FALSE;
        }
    }
#else
    /*
     * verify off 상태임을 Watch에서 구분하기 위한 표시.
     * failReason은 NONE 유지.
     */
    g_flashOtaWriteDebug.verifyIndex = 0xFFFFFFFFU;
    g_flashOtaWriteDebug.verifyOffset = 0xFFFFFFFFU;
#endif

    g_flashOtaDebug.receivedBytes += length;
    g_flashOtaDebug.lastBlockIndex = blockIndex;
    g_flashOtaDebug.lastWriteAddress = targetAddrNc;
    g_flashOtaDebug.writeOkCount++;

    g_flashOtaWriteDebug.failReason = FLASH_OTA_FAIL_NONE;
    g_flashOtaWriteDebug.dmuErr = MODULE_DMU.HF_ERRSR.U;
    g_flashOtaWriteDebug.writeOkCount++;

    return TRUE;
}

boolean FlashOta_EndTransfer(void)
{
    if (g_flashOtaDebug.started == FALSE)
    {
        return FALSE;
    }

    if (g_flashOtaDebug.receivedBytes != g_flashOtaDebug.firmwareSize)
    {
        return FALSE;
    }

    g_flashOtaDebug.transferExitDone = TRUE;

    return TRUE;
}

boolean FlashOta_CheckCrc32(uint32_t expectedCrc32,
                            uint32_t *calculatedCrc32)
{
    if (g_flashOtaDebug.transferExitDone == FALSE)
    {
        return FALSE;
    }

    if (g_downloadTargetAddrNC == 0U)
    {
        return FALSE;
    }

    /*
     * front-zcu OTA와 동일하게 Application은 expected CRC만 flag로 넘긴다.
     * 실제 image CRC 검증과 SOTA swap 여부 판단은 bootloader가 수행한다.
     */
    g_flashOtaDebug.expectedCrc32 = expectedCrc32;
    g_flashOtaDebug.calculatedCrc32 = expectedCrc32;

    if (calculatedCrc32 != NULL_PTR)
    {
        *calculatedCrc32 = expectedCrc32;
    }

    g_flashOtaDebug.crcVerified = TRUE;
    return TRUE;
}

boolean FlashOta_RequestJumpToApp(uint8_t resetType)
{
    if (g_flashOtaDebug.crcVerified == FALSE)
    {
        return FALSE;
    }

    /*
     * 현재 구조에서는 실제 App jump / UCB_SWAP을 수행하지 않는다.
     *
     * 의미:
     *  - inactive slot image는 CRC 검증 완료
     *  - 실행 전환은 Bootloader / SOTA_UCB_SWAP 담당
     */
    g_pendingResetType = resetType;
    g_jumpPending = TRUE;

    return TRUE;
}

boolean FlashOta_IsJumpPending(void)
{
    return g_jumpPending;
}

void FlashOta_Service(void)
{
    if (g_jumpPending == FALSE)
    {
        return;
    }

    /*
     * 기존 Single Slot OTA에서는 여기서 App으로 직접 jump했다.
     *
     * 하지만 현재는 Dual Slot / SOTA 구조이므로
     * 절대 App에서 직접 jump하지 않는다.
     *
     * 최종 통합 시:
     *  - OTA flag 저장
     *  - system reset
     *  - Bootloader가 flag 확인 후 SOTA_SWAP 수행
     *
     * 지금은 pending 상태만 정리한다.
     */
    delayMs(200U);

    (void)g_pendingResetType;

    Sota_SetPendingUpdateFlag(g_flashOtaDebug.firmwareSize,
                              g_flashOtaDebug.expectedCrc32);

    delayMs(20U);

    IfxScuRcu_performReset(IfxScuRcu_ResetType_system, 0);

    g_jumpPending = FALSE;
    g_pendingResetType = 0U;
}

void FlashOta_GetDebugInfo(FlashOta_DebugInfo_t *info)
{
    if (info != NULL_PTR)
    {
        memcpy(info, &g_flashOtaDebug, sizeof(FlashOta_DebugInfo_t));
    }
}

/* ============================================================
   Private functions
   ============================================================ */

static void delayMs(uint32 ms)
{
    Ifx_STM *stm = &MODULE_STM0;
    uint32 ticks = IfxStm_getTicksFromMilliseconds(stm, ms);

    IfxStm_waitTicks(stm, ticks);
}

static IfxFlash_FlashType getPFlashTypeFromAddress(uint32 addr)
{
    uint32 offset;
    uint32 slotBOffset;

    /*
     * Cached / Non-cached 주소 모두 하위 offset 기준으로 판단한다.
     *
     * 예:
     *   0x80020000 & 0x0FFFFFFF = 0x00020000
     *   0xA0020000 & 0x0FFFFFFF = 0x00020000
     *   0x80320000 & 0x0FFFFFFF = 0x00320000
     *   0xA0320000 & 0x0FFFFFFF = 0x00320000
     */
    offset = addr & 0x0FFFFFFFU;

    /*
     * Slot B / Bank B 시작 offset.
     * FLASH_OTA_SLOT_B_START_ADDR_C = 0x80320000 기준이면
     * slotBOffset = 0x00320000
     */
    slotBOffset = FLASH_OTA_SLOT_B_START_ADDR_C & 0x0FFFFFFFU;

    if (offset >= slotBOffset)
    {
        return IfxFlash_FlashType_P1;
    }

    return IfxFlash_FlashType_P0;
}

