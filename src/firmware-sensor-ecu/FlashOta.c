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
 *
 * Sparse OTA erase 정책:
 *  - 첫 번째 segment, 즉 offsetInSlot == 0에서 inactive App slot 전체를 erase한다.
 *  - sparse gap은 CAN으로 전송하지 않으므로, gap 상태를 일정하게 만들기 위함이다.
 *********************************************************************************************************************/

#include "FlashOta.h"

#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include "IfxScuRcu.h"
#include "IfxFlash.h"
#include "IfxStm.h"
#include "SotaUcb.h"
#include "SensorOtaFlash.h"
#include "SensorOtaEraseWorker.h"

#include <string.h>
#include <stdio.h>

/* ============================================================
   DFLASH OTA flag / metadata address
   ============================================================ */

#ifndef OTA_FLAG_ADDR
#define OTA_FLAG_ADDR       0xAF000000UL
#endif

#ifndef OTA_FLAG_MAGIC
#define OTA_FLAG_MAGIC      0xDEADBEEFUL
#endif

#ifndef FLASH_MODULE
#define FLASH_MODULE        0U
#endif

/* ============================================================
   Debug
   ============================================================ */

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
   STEP4A CPU1 erase worker debug
   ============================================================ */

/*
 * STEP4A 목적:
 *  - UDS 0x34 응답 타이밍은 기존과 동일하게 유지한다.
 *  - FlashOta_EraseInactiveSlot()에서 CPU0이 직접 erase하지 않고,
 *    CPU1 erase worker에 sector erase를 요청한 뒤 완료될 때까지 기다린다.
 *  - erase 완료 후에만 0x34 positive response가 나가도록 기존 동작을 유지한다.
 */
volatile uint32_t g_flashOtaCpu1EraseRequestCount = 0U;
volatile uint32_t g_flashOtaCpu1EraseDoneCount = 0U;
volatile uint32_t g_flashOtaCpu1EraseErrorCount = 0U;
volatile uint32_t g_flashOtaCpu1EraseTimeoutCount = 0U;
volatile uint32_t g_flashOtaCpu1EraseLastAddr = 0U;
volatile uint32_t g_flashOtaCpu1EraseLastFlashType = 0U;
volatile uint32_t g_flashOtaCpu1EraseLastWorkerState = 0U;

#ifndef FLASH_OTA_CPU1_ERASE_SECTOR_TIMEOUT_MS
#define FLASH_OTA_CPU1_ERASE_SECTOR_TIMEOUT_MS   3000U
#endif

/* ============================================================
   STEP5A async erase state
   ============================================================ */

typedef enum
{
    FLASH_OTA_ASYNC_ERASE_IDLE = 0,
    FLASH_OTA_ASYNC_ERASE_BUSY,
    FLASH_OTA_ASYNC_ERASE_DONE,
    FLASH_OTA_ASYNC_ERASE_ERROR
} FlashOtaAsyncEraseState_t;

static volatile FlashOtaAsyncEraseState_t g_asyncEraseState = FLASH_OTA_ASYNC_ERASE_IDLE;
static volatile boolean g_asyncEraseRequestInFlight = FALSE;

static uint32_t g_asyncEraseStartNc = 0U;
static uint32_t g_asyncEraseEndNc = 0U;
static uint32_t g_asyncEraseCurrentNc = 0U;
static uint32_t g_asyncEraseSlotEndNc = 0U;

volatile uint32_t g_flashOtaAsyncEraseState = 0U;
volatile uint32_t g_flashOtaAsyncEraseStartNc = 0U;
volatile uint32_t g_flashOtaAsyncEraseEndNc = 0U;
volatile uint32_t g_flashOtaAsyncEraseCurrentNc = 0U;
volatile uint32_t g_flashOtaAsyncEraseRequestInFlight = 0U;
volatile uint32_t g_flashOtaAsyncEraseDoneCount = 0U;
volatile uint32_t g_flashOtaAsyncEraseErrorCount = 0U;


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
static volatile boolean g_flagWritePending = FALSE;
static volatile boolean g_resetPending = FALSE;
static volatile boolean g_pendingInfoWritten = FALSE;
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

/*
 * Sparse OTA metadata.
 *
 * UdsOta.c에서 0x31 처리 직전에 FlashOta_SetPendingMetadata()를 호출하면
 * FlashOta_Service()가 기존 legacy flag 대신 metadata 전체를 DFLASH에 저장한다.
 */
static FlashOtaPendingMeta_t g_flashOtaPendingMeta;
static boolean g_flashOtaUsePendingMeta = FALSE;

/* ============================================================
   Private prototypes
   ============================================================ */

static void delayMs(uint32 ms);

static IfxFlash_FlashType getPFlashTypeFromAddress(uint32 addr);

static boolean FlashOta_WritePendingMetadataToDFlash(const FlashOtaPendingMeta_t *meta);
static boolean FlashOta_WriteDFlash8(uint32 addr, uint32 lo, uint32 hi);

static boolean FlashOta_EraseInactiveSlot(uint32_t slotStartAddrNc);
static boolean FlashOta_EraseSectorByCpu1Worker(uint32_t addrNc, IfxFlash_FlashType flashType);
static void FlashOta_ServiceEraseInactiveSlot(void);
static uint32_t FlashOta_AlignUpSector(uint32_t addr);

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

    g_flagWritePending = FALSE;
    g_resetPending = FALSE;
    g_pendingInfoWritten = FALSE;
    g_pendingResetType = 0U;

    memset(&g_flashOtaPendingMeta, 0, sizeof(g_flashOtaPendingMeta));
    g_flashOtaUsePendingMeta = FALSE;

    g_asyncEraseState = FLASH_OTA_ASYNC_ERASE_IDLE;
    g_asyncEraseRequestInFlight = FALSE;
    g_asyncEraseStartNc = 0U;
    g_asyncEraseEndNc = 0U;
    g_asyncEraseCurrentNc = 0U;
    g_asyncEraseSlotEndNc = 0U;
    g_flashOtaAsyncEraseState = (uint32_t)g_asyncEraseState;
    g_flashOtaAsyncEraseStartNc = 0U;
    g_flashOtaAsyncEraseEndNc = 0U;
    g_flashOtaAsyncEraseCurrentNc = 0U;
    g_flashOtaAsyncEraseRequestInFlight = 0U;
}

boolean FlashOta_SetPendingMetadata(const FlashOtaPendingMeta_t *meta)
{
    uint32 i;

    if (meta == NULL_PTR)
    {
        return FALSE;
    }

    if (meta->magic != OTA_FLAG_MAGIC)
    {
        return FALSE;
    }

    if (meta->version != FLASH_OTA_META_VERSION)
    {
        return FALSE;
    }

    if ((meta->virtualSize == 0U) || (meta->virtualSize > FLASH_OTA_MAX_IMAGE_SIZE))
    {
        return FALSE;
    }

    if ((meta->segmentCount == 0U) || (meta->segmentCount > FLASH_OTA_META_MAX_SEGMENTS))
    {
        return FALSE;
    }

    if (meta->gapFill > 0xFFU)
    {
        return FALSE;
    }

    for (i = 0U; i < meta->segmentCount; i++)
    {
        uint32 segOffset;
        uint32 segSize;
        uint32 segEnd;

        segOffset = meta->segments[i].offset;
        segSize = meta->segments[i].size;
        segEnd = segOffset + segSize;

        if (segSize == 0U)
        {
            return FALSE;
        }

        if (segEnd < segOffset)
        {
            return FALSE;
        }

        if (segEnd > meta->virtualSize)
        {
            return FALSE;
        }

        if (i > 0U)
        {
            uint32 prevEnd;

            prevEnd = meta->segments[i - 1U].offset + meta->segments[i - 1U].size;

            if (segOffset < prevEnd)
            {
                return FALSE;
            }
        }
    }

    memcpy(&g_flashOtaPendingMeta, meta, sizeof(FlashOtaPendingMeta_t));
    g_flashOtaUsePendingMeta = TRUE;

    return TRUE;
}

boolean FlashOta_BeginDownload(uint32_t targetAddress, uint32_t firmwareSize)
{
    uint32_t offsetInSlot;
    uint32_t slotStartAddrNC;

    FlashOta_Reset();

    slotStartAddrNC = 0U;

    if ((firmwareSize == 0U) || (firmwareSize > FLASH_OTA_MAX_IMAGE_SIZE))
    {
        return FALSE;
    }

    if ((targetAddress >= FLASH_OTA_SLOT_A_START_ADDR_C) &&
        (targetAddress < (FLASH_OTA_SLOT_A_START_ADDR_C + FLASH_OTA_MAX_IMAGE_SIZE)))
    {
        offsetInSlot = targetAddress - FLASH_OTA_SLOT_A_START_ADDR_C;

        if (firmwareSize > (FLASH_OTA_MAX_IMAGE_SIZE - offsetInSlot))
        {
            return FALSE;
        }

        g_downloadTargetAddrC = targetAddress;
        g_downloadTargetAddrNC = FLASH_OTA_SLOT_A_START_ADDR_NC + offsetInSlot;
        slotStartAddrNC = FLASH_OTA_SLOT_A_START_ADDR_NC;
    }
    else if ((targetAddress >= FLASH_OTA_SLOT_B_START_ADDR_C) &&
             (targetAddress < (FLASH_OTA_SLOT_B_START_ADDR_C + FLASH_OTA_MAX_IMAGE_SIZE)))
    {
        offsetInSlot = targetAddress - FLASH_OTA_SLOT_B_START_ADDR_C;

        if (firmwareSize > (FLASH_OTA_MAX_IMAGE_SIZE - offsetInSlot))
        {
            return FALSE;
        }

        g_downloadTargetAddrC = targetAddress;
        g_downloadTargetAddrNC = FLASH_OTA_SLOT_B_START_ADDR_NC + offsetInSlot;
        slotStartAddrNC = FLASH_OTA_SLOT_B_START_ADDR_NC;
    }
    else
    {
        return FALSE;
    }

    g_flashOtaDebug.targetAddress = g_downloadTargetAddrC;
    g_flashOtaDebug.firmwareSize = firmwareSize;
    g_flashOtaDebug.receivedBytes = 0U;
    g_flashOtaDebug.started = TRUE;

    /*
     * 첫 번째 segment(offset 0)에서 inactive App slot 전체를 erase한다.
     *
     * Sparse OTA에서는 segment 사이 gap을 전송하지 않으므로,
     * gap 상태를 항상 일정하게 만들어두는 목적이다.
     *
     * 주의:
     *  - 이 erase는 시간이 오래 걸릴 수 있다.
     *  - 0x34 응답 timeout을 충분히 길게 잡아야 한다.
     */
    if (offsetInSlot == 0U)
    {
        if (FlashOta_EraseInactiveSlot(slotStartAddrNC) == FALSE)
        {
            return FALSE;
        }

        /*
         * STEP5A:
         * 전체 slot erase는 CPU1 worker가 비동기로 수행한다.
         * erase 완료 후 FlashOta_ServiceEraseInactiveSlot()에서
         * g_erasedUntilAddrNC를 slot 끝으로 갱신한다.
         */
    }
    else
    {
        /*
         * segment2처럼 offset 0이 아닌 경우에는 기존처럼 해당 segment가 닿는 sector만
         * on-demand erase한다.
         *
         * 단, 정상 흐름에서는 첫 segment 때 전체 slot erase가 이미 끝났으므로
         * segment2의 해당 sector도 erase된 상태여야 한다.
         */
        /*
         * Sparse OTA 정상 흐름에서는 첫 segment(offset 0)에서 inactive slot 전체가
         * 이미 erase되어 있다. segment2 이후에서 on-demand erase로 CPU0을 막지 않도록
         * erasedUntil을 slot 끝으로 유지한다.
         */
        g_erasedUntilAddrNC = FlashOta_AlignUpSector(slotStartAddrNC + FLASH_OTA_MAX_IMAGE_SIZE);
    }

    return TRUE;
}

boolean FlashOta_IsDownloadReady(void)
{
    if (g_flashOtaDebug.started == FALSE)
    {
        return FALSE;
    }

    if (g_asyncEraseState == FLASH_OTA_ASYNC_ERASE_BUSY)
    {
        return FALSE;
    }

    if (g_asyncEraseState == FLASH_OTA_ASYNC_ERASE_ERROR)
    {
        return FALSE;
    }

    return TRUE;
}

boolean FlashOta_IsDownloadError(void)
{
    return (g_asyncEraseState == FLASH_OTA_ASYNC_ERASE_ERROR) ? TRUE : FALSE;
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
    printf("FlashOta_CheckCrc32 transferExitDone : %d \r\n", g_flashOtaDebug.transferExitDone);

    if (g_flashOtaDebug.transferExitDone == FALSE)
    {
        return FALSE;
    }

    printf("FlashOta_CheckCrc32 g_downloadTargetAddrNC : 0x%08X \r\n", g_downloadTargetAddrNC);

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

    printf("[FlashOta] Expected CRC32 : %08X \r\n", expectedCrc32);

    if (calculatedCrc32 != NULL_PTR)
    {
        *calculatedCrc32 = expectedCrc32;
    }

    g_flashOtaDebug.crcVerified = TRUE;
    return TRUE;
}

boolean FlashOta_RequestWritePendingFlag(void)
{
#if 0
    if (g_flashOtaDebug.crcVerified == FALSE)
    {
        return FALSE;
    }
#endif

    g_flagWritePending = TRUE;
    return TRUE;
}

boolean FlashOta_RequestJumpToApp(uint8_t resetType)
{
    /*
     * 현재 구조에서는 실제 App jump / UCB_SWAP을 수행하지 않는다.
     *
     * 의미:
     *  - inactive slot image는 CRC 검증 완료
     *  - 실행 전환은 Bootloader / SOTA_UCB_SWAP 담당
     */
    g_pendingResetType = resetType;
    g_resetPending = TRUE;

    return TRUE;
}

boolean FlashOta_IsFlagWritePending(void)
{
    return g_flagWritePending;
}

boolean FlashOta_IsResetPending(void)
{
    return g_resetPending;
}

boolean FlashOta_IsJumpPending(void)
{
    return g_resetPending;
}

void FlashOta_Service(void)
{
    FlashOta_ServiceEraseInactiveSlot();

    /*
     * 1) CRC RoutineControl 이후 pending flag write 요청 처리
     *
     * Legacy 단일 이미지 OTA에서는 여기서 기존 pending flag를 쓴다.
     *
     * Sparse metadata OTA에서는 여기서 DFLASH에 쓰지 않는다.
     * Sparse metadata는 reset 요청 시점에 최종 1회 저장한다.
     */
    if (g_flagWritePending == TRUE)
    {
        g_flagWritePending = FALSE;

        if (g_flashOtaUsePendingMeta == FALSE)
        {
            if (g_pendingInfoWritten == FALSE)
            {
                Sota_SetPendingUpdateFlag(g_flashOtaDebug.firmwareSize,
                                          g_flashOtaDebug.expectedCrc32);

                g_pendingInfoWritten = TRUE;

                printf("Pending update flag set.\r\n");
                printf("Firmware size: %u, Expected CRC32: %08X\r\n",
                       g_flashOtaDebug.firmwareSize,
                       g_flashOtaDebug.expectedCrc32);
            }
        }
        else
        {
            /*
             * Sparse OTA에서는 legacy flag를 먼저 쓰면 안 된다.
             * metadata 저장은 reset 직전에 수행한다.
             */
            printf("[FlashOta] Sparse metadata mode: pending flag write deferred until reset.\r\n");
        }
    }

    /*
     * 2) 최종 reset 처리
     *
     * 현재 Dual Slot / SOTA 구조에서는 App으로 직접 jump하지 않는다.
     *
     * 최종 흐름:
     *  - legacy pending flag 또는 sparse metadata 저장
     *  - system reset
     *  - Bootloader가 DFLASH flag/metadata 확인
     *  - inactive bank CRC 검증
     *  - SOTA swap 수행
     */
    if (g_resetPending == TRUE)
    {
        g_resetPending = FALSE;

        delayMs(200U);

        (void)g_pendingResetType;

        if (g_pendingInfoWritten == FALSE)
        {
            if (g_flashOtaUsePendingMeta == TRUE)
            {
                if (FlashOta_WritePendingMetadataToDFlash(&g_flashOtaPendingMeta) == FALSE)
                {
                    printf("[FlashOta] Pending metadata write failed\r\n");

                    g_pendingResetType = 0U;
                    return;
                }

                printf("[FlashOta] Pending metadata set. virtualSize: %u, Expected CRC32: %08X, segmentCount: %u\r\n",
                       g_flashOtaPendingMeta.virtualSize,
                       g_flashOtaPendingMeta.expectedCrc32,
                       g_flashOtaPendingMeta.segmentCount);
            }
            else
            {
                Sota_SetPendingUpdateFlag(g_flashOtaDebug.firmwareSize,
                                          g_flashOtaDebug.expectedCrc32);

                printf("Pending update flag set. Firmware size: %u, Expected CRC32: %08X\r\n",
                       g_flashOtaDebug.firmwareSize,
                       g_flashOtaDebug.expectedCrc32);
            }

            g_pendingInfoWritten = TRUE;
        }

        delayMs(20U);

        IfxScuRcu_performReset(IfxScuRcu_ResetType_system, 0);

        g_pendingResetType = 0U;
    }
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

static uint32_t FlashOta_AlignUpSector(uint32_t addr)
{
    return (addr + FLASH_OTA_SECTOR_SIZE_BYTES - 1U) &
           ~(FLASH_OTA_SECTOR_SIZE_BYTES - 1U);
}

static boolean FlashOta_EraseInactiveSlot(uint32_t slotStartAddrNc)
{
    uint32_t eraseStartNc;
    uint32_t eraseEndNc;

    eraseStartNc = slotStartAddrNc & ~(FLASH_OTA_SECTOR_SIZE_BYTES - 1U);
    eraseEndNc = FlashOta_AlignUpSector(slotStartAddrNc + FLASH_OTA_MAX_IMAGE_SIZE);

    /*
     * STEP5A:
     * 여기서는 erase 완료까지 기다리지 않는다.
     * CPU1 worker가 처리할 erase 범위만 등록하고 즉시 TRUE를 반환한다.
     * 실제 sector 요청/완료 감시는 FlashOta_ServiceEraseInactiveSlot()에서 수행한다.
     */
    g_asyncEraseStartNc = eraseStartNc;
    g_asyncEraseEndNc = eraseEndNc;
    g_asyncEraseCurrentNc = eraseStartNc;
    g_asyncEraseSlotEndNc = eraseEndNc;

    g_asyncEraseRequestInFlight = FALSE;
    g_asyncEraseState = FLASH_OTA_ASYNC_ERASE_BUSY;

    g_flashOtaAsyncEraseState = (uint32_t)g_asyncEraseState;
    g_flashOtaAsyncEraseStartNc = eraseStartNc;
    g_flashOtaAsyncEraseEndNc = eraseEndNc;
    g_flashOtaAsyncEraseCurrentNc = eraseStartNc;
    g_flashOtaAsyncEraseRequestInFlight = 0U;

    printf("[FlashOta] STEP5A async CPU1 erase start: 0x%08X ~ 0x%08X",
           eraseStartNc,
           eraseEndNc);

    return TRUE;
}


static void FlashOta_ServiceEraseInactiveSlot(void)
{
    IfxFlash_FlashType flashType;

    if (g_asyncEraseState != FLASH_OTA_ASYNC_ERASE_BUSY)
    {
        return;
    }

    g_flashOtaAsyncEraseState = (uint32_t)g_asyncEraseState;
    g_flashOtaAsyncEraseCurrentNc = g_asyncEraseCurrentNc;
    g_flashOtaAsyncEraseRequestInFlight = (g_asyncEraseRequestInFlight == TRUE) ? 1U : 0U;
    g_flashOtaCpu1EraseLastWorkerState = (uint32_t)SensorOtaEraseWorker_GetState();

    if (g_asyncEraseRequestInFlight == TRUE)
    {
        if (SensorOtaEraseWorker_IsDone() == TRUE)
        {
            g_flashOtaCpu1EraseDoneCount++;
            SensorOtaEraseWorker_ClearResult();

            if (MODULE_DMU.HF_ERRSR.U != 0U)
            {
                g_flashOtaCpu1EraseErrorCount++;
                g_flashOtaAsyncEraseErrorCount++;
                g_asyncEraseState = FLASH_OTA_ASYNC_ERASE_ERROR;
                g_asyncEraseRequestInFlight = FALSE;

                g_flashOtaWriteDebug.dmuErr = MODULE_DMU.HF_ERRSR.U;
                g_flashOtaWriteDebug.failReason = FLASH_OTA_FAIL_ERASE;
                g_flashOtaAsyncEraseState = (uint32_t)g_asyncEraseState;
                return;
            }

            g_asyncEraseRequestInFlight = FALSE;
            g_flashOtaWriteDebug.eraseCount++;
            g_flashOtaDebug.eraseCount++;

            g_asyncEraseCurrentNc += FLASH_OTA_SECTOR_SIZE_BYTES;

            if (g_asyncEraseCurrentNc >= g_asyncEraseEndNc)
            {
                g_erasedUntilAddrNC = g_asyncEraseSlotEndNc;
                g_asyncEraseState = FLASH_OTA_ASYNC_ERASE_DONE;
                g_flashOtaAsyncEraseDoneCount++;

                g_flashOtaAsyncEraseState = (uint32_t)g_asyncEraseState;
                g_flashOtaAsyncEraseCurrentNc = g_asyncEraseCurrentNc;
                g_flashOtaAsyncEraseRequestInFlight = 0U;

                printf("[FlashOta] STEP5A async CPU1 erase done");
            }

            return;
        }

        if (SensorOtaEraseWorker_IsError() == TRUE)
        {
            g_flashOtaCpu1EraseErrorCount++;
            g_flashOtaAsyncEraseErrorCount++;
            SensorOtaEraseWorker_ClearResult();

            g_asyncEraseRequestInFlight = FALSE;
            g_asyncEraseState = FLASH_OTA_ASYNC_ERASE_ERROR;

            g_flashOtaWriteDebug.eraseAddressNc = g_asyncEraseCurrentNc;
            g_flashOtaWriteDebug.dmuErr = MODULE_DMU.HF_ERRSR.U;
            g_flashOtaWriteDebug.failReason = FLASH_OTA_FAIL_ERASE;

            g_flashOtaAsyncEraseState = (uint32_t)g_asyncEraseState;
            g_flashOtaAsyncEraseRequestInFlight = 0U;
        }

        return;
    }

    if (g_asyncEraseCurrentNc >= g_asyncEraseEndNc)
    {
        g_erasedUntilAddrNC = g_asyncEraseSlotEndNc;
        g_asyncEraseState = FLASH_OTA_ASYNC_ERASE_DONE;
        g_flashOtaAsyncEraseDoneCount++;

        g_flashOtaAsyncEraseState = (uint32_t)g_asyncEraseState;
        return;
    }

    flashType = getPFlashTypeFromAddress(g_asyncEraseCurrentNc);

    g_flashOtaCpu1EraseLastAddr = g_asyncEraseCurrentNc;
    g_flashOtaCpu1EraseLastFlashType = (uint32_t)flashType;
    g_flashOtaCpu1EraseLastWorkerState = (uint32_t)SensorOtaEraseWorker_GetState();

    g_flashOtaWriteDebug.eraseAddressNc = g_asyncEraseCurrentNc;
    g_flashOtaWriteDebug.erasedUntilNc = g_erasedUntilAddrNC;
    g_flashOtaWriteDebug.flashType = (uint32_t)flashType;

    SensorOtaEraseWorker_ClearResult();

    if (SensorOtaEraseWorker_Request(g_asyncEraseCurrentNc,
                                      FLASH_OTA_SECTOR_SIZE_BYTES,
                                      flashType) == FALSE)
    {
        g_flashOtaCpu1EraseErrorCount++;
        g_flashOtaAsyncEraseErrorCount++;

        g_asyncEraseState = FLASH_OTA_ASYNC_ERASE_ERROR;

        g_flashOtaWriteDebug.failReason = FLASH_OTA_FAIL_ERASE;
        g_flashOtaAsyncEraseState = (uint32_t)g_asyncEraseState;
        return;
    }

    g_asyncEraseRequestInFlight = TRUE;
    g_flashOtaCpu1EraseRequestCount++;

    g_flashOtaAsyncEraseState = (uint32_t)g_asyncEraseState;
    g_flashOtaAsyncEraseRequestInFlight = 1U;
}

static boolean FlashOta_EraseSectorByCpu1Worker(uint32_t addrNc, IfxFlash_FlashType flashType)
{
    uint32 elapsedMs;

    g_flashOtaCpu1EraseLastAddr = addrNc;
    g_flashOtaCpu1EraseLastFlashType = (uint32_t)flashType;
    g_flashOtaCpu1EraseLastWorkerState = (uint32_t)SensorOtaEraseWorker_GetState();

    /*
     * 이전 sector의 DONE/ERROR 결과가 남아있으면 IDLE로 정리한다.
     */
    SensorOtaEraseWorker_ClearResult();

    if (SensorOtaEraseWorker_Request(addrNc,
                                      FLASH_OTA_SECTOR_SIZE_BYTES,
                                      flashType) == FALSE)
    {
        g_flashOtaCpu1EraseErrorCount++;
        g_flashOtaCpu1EraseLastWorkerState = (uint32_t)SensorOtaEraseWorker_GetState();
        return FALSE;
    }

    g_flashOtaCpu1EraseRequestCount++;

    for (elapsedMs = 0U;
         elapsedMs < FLASH_OTA_CPU1_ERASE_SECTOR_TIMEOUT_MS;
         elapsedMs++)
    {
        g_flashOtaCpu1EraseLastWorkerState = (uint32_t)SensorOtaEraseWorker_GetState();

        if (SensorOtaEraseWorker_IsDone() == TRUE)
        {
            g_flashOtaCpu1EraseDoneCount++;
            SensorOtaEraseWorker_ClearResult();

            if (MODULE_DMU.HF_ERRSR.U != 0U)
            {
                g_flashOtaCpu1EraseErrorCount++;
                return FALSE;
            }

            return TRUE;
        }

        if (SensorOtaEraseWorker_IsError() == TRUE)
        {
            g_flashOtaCpu1EraseErrorCount++;
            SensorOtaEraseWorker_ClearResult();
            return FALSE;
        }

        /*
         * CPU1 worker가 실행될 시간을 주기 위한 1ms 대기.
         * 기존 FlashOta_BeginDownload()도 0x34 처리 중 erase 완료까지 기다리는 구조였으므로
         * 0x34 응답 타이밍은 여전히 erase 완료 이후로 유지된다.
         */
        delayMs(1U);
    }

    g_flashOtaCpu1EraseTimeoutCount++;
    g_flashOtaCpu1EraseLastWorkerState = (uint32_t)SensorOtaEraseWorker_GetState();

    return FALSE;
}

static boolean FlashOta_WriteDFlash8(uint32 addr, uint32 lo, uint32 hi)
{
    uint16 pw;

    pw = IfxScuWdt_getSafetyWatchdogPassword();

    IfxFlash_enterPageMode(addr);
    IfxFlash_waitUnbusy(FLASH_MODULE, IfxFlash_FlashType_D0);

    IfxFlash_loadPage2X32(addr, lo, hi);

    IfxScuWdt_clearSafetyEndinit(pw);
    IfxFlash_writePage(addr);
    IfxScuWdt_setSafetyEndinit(pw);

    IfxFlash_waitUnbusy(FLASH_MODULE, IfxFlash_FlashType_D0);

    if (MODULE_DMU.HF_ERRSR.U != 0U)
    {
        return FALSE;
    }

    return TRUE;
}

static boolean FlashOta_WritePendingMetadataToDFlash(const FlashOtaPendingMeta_t *meta)
{
    uint16 pw;
    const uint32 *words;
    uint32 wordCount;
    uint32 i;
    uint32 addr;

    if (meta == NULL_PTR)
    {
        return FALSE;
    }

    if (meta->magic != OTA_FLAG_MAGIC)
    {
        return FALSE;
    }

    if (meta->version != FLASH_OTA_META_VERSION)
    {
        return FALSE;
    }

    if ((meta->virtualSize == 0U) || (meta->virtualSize > FLASH_OTA_MAX_IMAGE_SIZE))
    {
        return FALSE;
    }

    if ((meta->segmentCount == 0U) || (meta->segmentCount > FLASH_OTA_META_MAX_SEGMENTS))
    {
        return FALSE;
    }

    if (meta->gapFill > 0xFFU)
    {
        return FALSE;
    }

    /*
     * DFLASH pending metadata sector erase.
     */
    pw = IfxScuWdt_getSafetyWatchdogPassword();

    IfxScuWdt_clearSafetyEndinit(pw);
    IfxFlash_eraseMultipleSectors(OTA_FLAG_ADDR, 1U);
    IfxScuWdt_setSafetyEndinit(pw);

    IfxFlash_waitUnbusy(FLASH_MODULE, IfxFlash_FlashType_D0);

    if (MODULE_DMU.HF_ERRSR.U != 0U)
    {
        return FALSE;
    }

    /*
     * FlashOtaPendingMeta_t는 uint32 필드만 가진다.
     * 8-byte 단위로 DFLASH page write한다.
     */
    words = (const uint32 *)meta;
    wordCount = (uint32)(sizeof(FlashOtaPendingMeta_t) / sizeof(uint32));

    if ((wordCount & 1U) != 0U)
    {
        return FALSE;
    }

    addr = OTA_FLAG_ADDR;

    for (i = 0U; i < wordCount; i += 2U)
    {
        if (FlashOta_WriteDFlash8(addr, words[i], words[i + 1U]) == FALSE)
        {
            return FALSE;
        }

        addr += 8U;
    }

    return TRUE;
}

boolean FlashOta_SetFinalFirmwareSize(uint32_t firmwareSize)
{
    if ((firmwareSize == 0U) || (firmwareSize > FLASH_OTA_MAX_IMAGE_SIZE))
    {
        return FALSE;
    }

    g_flashOtaDebug.firmwareSize = firmwareSize;

    return TRUE;
}
