/**********************************************************************************************************************
 * \file FlashOta.c
 * \brief Sensor ECU Flash OTA Core - Bank B download only version
 *
 * 1차 목표:
 *  - 새 firmware를 Active App 영역이 아니라 Bank B / Inactive Slot에 저장한다.
 *  - Bank B 영역에 erase / write / read-back verify / CRC32 검증을 수행한다.
 *  - 아직 UCB_SWAP, App jump, 실행 Slot 전환은 수행하지 않는다.
 *
 * 주소 정책:
 *  - Slot A / Active App 후보:
 *      Cached    : 0x80040000
 *      Noncached : 0xA0040000
 *
 *  - Slot B / Bank B / OTA Download Target:
 *      Cached    : 0x80300000
 *      Noncached : 0xA0300000
 *
 * 전제:
 *  - FlashOta.h에 FLASH_OTA_DOWNLOAD_TARGET_ADDR_C / NC가 정의되어 있어야 한다.
 *  - can_type_def.h의 UDS_APP_START_ADDR도 0x80300000U로 맞춰야 한다.
 *********************************************************************************************************************/

#include "FlashOta.h"

#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include "IfxFlash.h"
#include "IfxStm.h"

#include <string.h>

/* ============================================================
   Internal state
   ============================================================ */

static FlashOta_DebugInfo_t g_flashOtaDebug;

/*
 * 1차 단계에서는 "jump pending"이라는 이름은 유지하지만,
 * 실제로 App jump는 하지 않는다.
 *
 * 나중에 UCB_SWAP / Bank B Activation 단계에서
 * 이 부분을 SOTA_SwapToGroupB() 흐름으로 교체할 예정.
 */
static volatile boolean g_jumpPending = FALSE;
static volatile uint8_t g_pendingResetType = 0U;

/*
 * 실제 erase / write / CRC 대상 주소.
 *
 * 현재 1차 목표에서는 항상 Bank B / Inactive Slot을 대상으로 한다.
 */
static uint32_t g_downloadTargetAddrC  = FLASH_OTA_DOWNLOAD_TARGET_ADDR_C;
static uint32_t g_downloadTargetAddrNC = FLASH_OTA_DOWNLOAD_TARGET_ADDR_NC;


/* ============================================================
   Private prototypes
   ============================================================ */

static uint32_t readU32Le(const uint8_t *p);
static uint32_t calcSectorCount(uint32_t firmwareSize);
static uint32_t crc32Update(uint32_t crc, uint8_t data);
static uint32_t crc32FlashOriginalSize(void);
static void delayMs(uint32 ms);

static IfxFlash_FlashType getPFlashTypeFromAddress(uint32 addr);

#pragma section code "cpu0_psram"

static void pflashEraseSectorsPspr(uint32 sectorAddr, uint32 sectorCount);
static void pflashWritePagePspr(uint32 pageAddr, const uint32 *data);

#pragma section code restore


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
     * 1차 목표에서는 OTA download target을 항상 Bank B로 둔다.
     */
    g_downloadTargetAddrC  = FLASH_OTA_DOWNLOAD_TARGET_ADDR_C;
    g_downloadTargetAddrNC = FLASH_OTA_DOWNLOAD_TARGET_ADDR_NC;

    g_jumpPending = FALSE;
    g_pendingResetType = 0U;
}


boolean FlashOta_BeginDownload(uint32_t targetAddress, uint32_t firmwareSize)
{
    uint32_t sectorCount;

    /*
     * 새 다운로드 시작 시 FlashOta 내부 상태 초기화.
     */
    FlashOta_Reset();

    /*
     * Dual Slot / Bank B OTA 1차 목표:
     *   Active App 영역이 아니라 Bank B / Inactive Slot에만 다운로드를 허용한다.
     *
     * 따라서 ZCU의 RequestDownload targetAddress도 0x80300000이어야 한다.
     */
    if (targetAddress != FLASH_OTA_DOWNLOAD_TARGET_ADDR_C)
    {
        return FALSE;
    }

    if ((firmwareSize == 0U) || (firmwareSize > FLASH_OTA_MAX_IMAGE_SIZE))
    {
        return FALSE;
    }

    sectorCount = calcSectorCount(firmwareSize);

    g_downloadTargetAddrC  = FLASH_OTA_DOWNLOAD_TARGET_ADDR_C;
    g_downloadTargetAddrNC = FLASH_OTA_DOWNLOAD_TARGET_ADDR_NC;

    g_flashOtaDebug.targetAddress = g_downloadTargetAddrC;
    g_flashOtaDebug.firmwareSize = firmwareSize;
    g_flashOtaDebug.receivedBytes = 0U;
    g_flashOtaDebug.started = TRUE;

    /*
     * Bank B 영역 erase.
     * Flash command는 non-cached 주소를 사용한다.
     */
    pflashEraseSectorsPspr(g_downloadTargetAddrNC, sectorCount);

    g_flashOtaDebug.eraseCount = sectorCount;

    return TRUE;
}


boolean FlashOta_WriteBlock(uint16_t blockIndex,
                            const uint8_t *data,
                            uint16_t length)
{
    uint8_t page[FLASH_OTA_PAGE_SIZE];
    uint32 words[8];
    uint32_t targetAddrNc;
    uint32_t offset;
    uint32_t remaining;
    volatile const uint8 *readPtr;

    if (g_flashOtaDebug.started == FALSE)
    {
        return FALSE;
    }

    if ((data == NULL_PTR) || (length == 0U) || (length > FLASH_OTA_PAGE_SIZE))
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
     * Bank B non-cached 주소에 write.
     *
     * blockIndex 0 -> 0xA0300000
     * blockIndex 1 -> 0xA0300020
     * blockIndex 2 -> 0xA0300040
     */
    targetAddrNc = g_downloadTargetAddrNC + offset;

    words[0] = readU32Le(&page[0]);
    words[1] = readU32Le(&page[4]);
    words[2] = readU32Le(&page[8]);
    words[3] = readU32Le(&page[12]);
    words[4] = readU32Le(&page[16]);
    words[5] = readU32Le(&page[20]);
    words[6] = readU32Le(&page[24]);
    words[7] = readU32Le(&page[28]);

    /* Trap 발생 전 주소 확인용 */
    g_flashOtaDebug.lastWriteAddress = targetAddrNc;

    pflashWritePagePspr(targetAddrNc, words);

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
    uint32_t crc;

    if (g_flashOtaDebug.transferExitDone == FALSE)
    {
        return FALSE;
    }

    /*
     * Bank B에 저장된 firmware를 firmwareSize만큼만 CRC 계산한다.
     * 마지막 page의 0xFF padding은 CRC에 포함하지 않는다.
     */
    crc = crc32FlashOriginalSize();

    g_flashOtaDebug.expectedCrc32 = expectedCrc32;
    g_flashOtaDebug.calculatedCrc32 = crc;

    if (calculatedCrc32 != NULL_PTR)
    {
        *calculatedCrc32 = crc;
    }

    if (crc == expectedCrc32)
    {
        g_flashOtaDebug.crcVerified = TRUE;
        return TRUE;
    }

    g_flashOtaDebug.crcVerified = FALSE;
    return FALSE;
}


boolean FlashOta_RequestJumpToApp(uint8_t resetType)
{
    if (g_flashOtaDebug.crcVerified == FALSE)
    {
        return FALSE;
    }

    /*
     * 1차 단계에서는 실제 App jump / UCB_SWAP을 수행하지 않는다.
     *
     * 의미:
     *   - Bank B image는 CRC 검증 완료
     *   - 실행 전환은 아직 하지 않음
     *   - 나중에 P단 + 사용자 OK 시 SOTA_SwapToGroupB()를 붙일 예정
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
     * 기존 Single Slot OTA에서는 여기서 0x80040000으로 jump했다.
     *
     * 하지만 현재는 Dual Slot / Bank B OTA 1차 단계이므로
     * 절대 active app으로 jump하지 않는다.
     *
     * 지금 목표:
     *   - Bank B write
     *   - Bank B CRC 검증
     *   - 실행 전환은 아직 보류
     *
     * 나중에 이 위치 또는 별도 Activation 함수에서
     * SOTA_SwapToGroupB() / system reset 흐름을 연결한다.
     */
    delayMs(200U);

    (void)g_pendingResetType;

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

static uint32_t readU32Le(const uint8_t *p)
{
    return ((uint32_t)p[0]) |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}


static uint32_t calcSectorCount(uint32_t firmwareSize)
{
    uint32_t count;

    count = firmwareSize / FLASH_OTA_SECTOR_SIZE_BYTES;

    if ((firmwareSize % FLASH_OTA_SECTOR_SIZE_BYTES) != 0U)
    {
        count++;
    }

    if (count == 0U)
    {
        count = 1U;
    }

    return count;
}


static uint32_t crc32Update(uint32_t crc, uint8_t data)
{
    crc ^= data;

    for (uint8 i = 0U; i < 8U; i++)
    {
        if ((crc & 1U) != 0U)
        {
            crc = (crc >> 1) ^ 0xEDB88320U;
        }
        else
        {
            crc = crc >> 1;
        }
    }

    return crc;
}


static uint32_t crc32FlashOriginalSize(void)
{
    volatile const uint8 *flashPtr = (volatile const uint8 *)g_downloadTargetAddrNC;
    uint32_t crc = 0xFFFFFFFFU;

    for (uint32_t i = 0U; i < g_flashOtaDebug.firmwareSize; i++)
    {
        crc = crc32Update(crc, flashPtr[i]);
    }

    return crc ^ 0xFFFFFFFFU;
}


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
     *   0x80300000 & 0x0FFFFFFF = 0x00300000
     *   0xA0300000 & 0x0FFFFFFF = 0x00300000
     */
    offset = addr & 0x0FFFFFFFU;

    /*
     * Slot B / Bank B 시작 offset.
     * FLASH_OTA_SLOT_B_START_ADDR_C = 0x80300000 기준이면
     * slotBOffset = 0x00300000
     */
    slotBOffset = FLASH_OTA_SLOT_B_START_ADDR_C & 0x0FFFFFFFU;

    if (offset >= slotBOffset)
    {
        return IfxFlash_FlashType_P1;
    }

    return IfxFlash_FlashType_P0;
}

/* ============================================================
   PSRAM functions
   ============================================================ */

#pragma section code "cpu0_psram"

static void pflashEraseSectorsPspr(uint32 sectorAddr, uint32 sectorCount)
{
    uint16 safetyWdtPassword;
    IfxFlash_FlashType flashType;

    flashType = getPFlashTypeFromAddress(sectorAddr);

    safetyWdtPassword = IfxScuWdt_getSafetyWatchdogPasswordInline();

    IfxScuWdt_clearSafetyEndinitInline(safetyWdtPassword);
    IfxFlash_eraseMultipleSectors(sectorAddr, sectorCount);
    IfxScuWdt_setSafetyEndinitInline(safetyWdtPassword);

    IfxFlash_waitUnbusy(0, flashType);
}


static void pflashWritePagePspr(uint32 pageAddr, const uint32 *data)
{
    uint16 safetyWdtPassword;
    IfxFlash_FlashType flashType;

    flashType = getPFlashTypeFromAddress(pageAddr);

    IfxFlash_enterPageMode(pageAddr);
    IfxFlash_waitUnbusy(0, flashType);

    /*
     * PFLASH page = 32 bytes = 8 words
     */
    IfxFlash_loadPage2X32(pageAddr, data[0], data[1]);
    IfxFlash_loadPage2X32(pageAddr, data[2], data[3]);
    IfxFlash_loadPage2X32(pageAddr, data[4], data[5]);
    IfxFlash_loadPage2X32(pageAddr, data[6], data[7]);

    safetyWdtPassword = IfxScuWdt_getSafetyWatchdogPasswordInline();

    IfxScuWdt_clearSafetyEndinitInline(safetyWdtPassword);
    IfxFlash_writePage(pageAddr);
    IfxScuWdt_setSafetyEndinitInline(safetyWdtPassword);

    IfxFlash_waitUnbusy(0, flashType);
}

#pragma section code restore
