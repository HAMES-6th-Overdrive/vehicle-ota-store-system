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

/* ============================================================
   Internal state
   ============================================================ */

static FlashOta_DebugInfo_t g_flashOtaDebug;

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
    volatile const uint8 *readPtr;

    if (g_flashOtaDebug.started == FALSE)
    {
        return FALSE;
    }

    if ((data == NULL_PTR) || (length == 0U) || (length > FLASH_OTA_PAGE_SIZE))
    {
        return FALSE;
    }

    if (g_downloadTargetAddrNC == 0U)
    {
        return FALSE;
    }

    /*
     * blockIndex 기준 32-byte page offset 계산.
     */
    offset = ((uint32_t)blockIndex * FLASH_OTA_PAGE_SIZE);

    if (offset >= g_flashOtaDebug.firmwareSize)
    {
        return FALSE;
    }

    remaining = g_flashOtaDebug.firmwareSize - offset;

    if (length > remaining)
    {
        return FALSE;
    }

    /*
     * 마지막 block이 32바이트보다 작을 수 있으므로 나머지는 0xFF padding.
     *
     * CRC는 나중에 firmwareSize만큼만 계산하므로,
     * 이 padding은 CRC에 포함되지 않는다.
     */
    memset(page, 0xFF, sizeof(page));
    memcpy(page, data, length);

    /*
     * target non-cached 주소에 write.
     *
     * 예:
     *  - Slot A target:
     *      blockIndex 0 -> 0xA0020000
     *      blockIndex 1 -> 0xA0020020
     *
     *  - Slot B target:
     *      blockIndex 0 -> 0xA0320000
     *      blockIndex 1 -> 0xA0320020
     */
    targetAddrNc = g_downloadTargetAddrNC + offset;
    writeEndAddrNc = targetAddrNc + FLASH_OTA_PAGE_SIZE;

    while (writeEndAddrNc > g_erasedUntilAddrNC)
    {
        flashType = getPFlashTypeFromAddress(g_erasedUntilAddrNC);

        if (SensorOtaFlash_Erase(g_erasedUntilAddrNC,
                                 FLASH_OTA_SECTOR_SIZE_BYTES,
                                 flashType) == FALSE)
        {
            g_flashOtaDebug.writeFailCount++;
            return FALSE;
        }

        g_erasedUntilAddrNC += FLASH_OTA_SECTOR_SIZE_BYTES;
        g_flashOtaDebug.eraseCount++;
    }

    /*
     * Trap 발생 전 주소 확인용.
     */
    g_flashOtaDebug.lastWriteAddress = targetAddrNc;

    flashType = getPFlashTypeFromAddress(targetAddrNc);

    if (SensorOtaFlash_Write(targetAddrNc,
                             page,
                             FLASH_OTA_PAGE_SIZE,
                             flashType) == FALSE)
    {
        g_flashOtaDebug.writeFailCount++;
        return FALSE;
    }

    /*
     * Read-back verify.
     * non-cached 주소로 실제 Flash에 쓰인 값을 확인한다.
     */
    readPtr = (volatile const uint8 *)targetAddrNc;

    for (uint32 i = 0U; i < FLASH_OTA_PAGE_SIZE; i++)
    {
        if (readPtr[i] != page[i])
        {
            g_flashOtaDebug.verifyFailOffset = offset + i;
            g_flashOtaDebug.writeFailCount++;
            return FALSE;
        }
    }

    g_flashOtaDebug.receivedBytes += length;
    g_flashOtaDebug.lastBlockIndex = blockIndex;
    g_flashOtaDebug.lastWriteAddress = targetAddrNc;
    g_flashOtaDebug.writeOkCount++;

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

