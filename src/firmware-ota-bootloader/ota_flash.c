#include "ota_flash.h"
#include "IfxCpu.h"
#include <string.h>
#include <stdio.h>

/*************************************************************/
/* 전역 변수                                                  */
/*************************************************************/
static OTA_FlashFunc g_func;
static boolean       g_funcCopied = FALSE;

/*************************************************************/
/* Sparse OTA CRC 설정                                        */
/*************************************************************/
/*
 * 현재 sensor-ecu manifest 기준:
 *
 * segment1:
 *   offset = 0x00000000
 *   size   = 0x000097C0
 *
 * segment2:
 *   offset = 0x002DE020
 *   size   = 0x00000140
 *
 * virtualSize:
 *   0x002DE160
 *
 * sparse gap:
 *   실제 flash에서 읽지 않고 0x00으로 CRC에 반영
 *
 * 주의:
 *   sensor-ecu 빌드 결과가 바뀌어서 segment1 size / segment2 offset이 바뀌면
 *   이 값들도 manifest 기준으로 다시 맞춰야 한다.
 */
#define OTA_SPARSE_SEG1_OFFSET      0x00000000UL
#define OTA_SPARSE_SEG1_SIZE        0x000097C0UL

#define OTA_SPARSE_SEG2_OFFSET      0x002DE020UL
#define OTA_SPARSE_SEG2_SIZE        0x00000140UL

#define OTA_SPARSE_GAP_FILL         0x00U

/*************************************************************/
/* PSPR 래퍼 함수                                             */
/*************************************************************/

/*
 * Erase command만 직접 발생시키는 함수.
 * IfxFlash_eraseMultipleSectors()를 임의 길이만큼 memcpy해서 쓰면
 * 함수가 잘리거나 내부 분기/호출 때문에 SQER가 발생할 수 있으므로,
 * 공식 SOTA 예제처럼 raw command 방식으로 sector 단위 erase를 수행한다.
 */
static void OTA_EraseOneSectorCommand(uint32 sectorAddr)
{
    volatile uint32 *addr1 = (volatile uint32 *)(IFXFLASH_CMD_BASE_ADDRESS | 0xaa50);
    volatile uint32 *addr2 = (volatile uint32 *)(IFXFLASH_CMD_BASE_ADDRESS | 0xaa58);
    volatile uint32 *addr3 = (volatile uint32 *)(IFXFLASH_CMD_BASE_ADDRESS | 0xaaa8);
    volatile uint32 *addr4 = (volatile uint32 *)(IFXFLASH_CMD_BASE_ADDRESS | 0xaaa8);

    *addr1 = sectorAddr;
    *addr2 = 1U;
    *addr3 = 0x80U;
    *addr4 = 0x50U;

    __dsync();
}

/* Erase 래퍼 — PSPR에서 실행됨 */
static uint32 OTA_EraseWrapper(uint32 sectorAddr,
                               uint32 sectorCount,
                               IfxFlash_FlashType flashType)
{
    uint16 pw = IfxScuWdt_getSafetyWatchdogPasswordInline();
    uint32 err = 0U;
    uint32 i;

    for (i = 0U; i < sectorCount; i++)
    {
        uint32 addr = sectorAddr + (i * PFLASH_SECTOR_SIZE);

        IfxScuWdt_clearSafetyEndinitInline(pw);
        g_func.eraseSectorCmd(addr);
        IfxScuWdt_setSafetyEndinitInline(pw);

        g_func.waitUnbusy(FLASH_MODULE, flashType);

        err = MODULE_DMU.HF_ERRSR.U;
        if (err != 0U)
        {
            break;
        }
    }

    return err;
}

/* Write 래퍼 — PSPR에서 실행됨, 32바이트 한 페이지 */
static void OTA_WritePageWrapper(uint32 pageAddr,
                                 uint8 *buf,
                                 IfxFlash_FlashType flashType)
{
    uint16 pw = IfxScuWdt_getSafetyWatchdogPasswordInline();

    g_func.enterPageMode(pageAddr);
    g_func.waitUnbusy(FLASH_MODULE, flashType);

    /* 8바이트씩 4번 = 32바이트 로드 */
    g_func.load2X32bits(pageAddr, *((uint32 *)(buf + 0)),  *((uint32 *)(buf + 4)));
    g_func.load2X32bits(pageAddr, *((uint32 *)(buf + 8)),  *((uint32 *)(buf + 12)));
    g_func.load2X32bits(pageAddr, *((uint32 *)(buf + 16)), *((uint32 *)(buf + 20)));
    g_func.load2X32bits(pageAddr, *((uint32 *)(buf + 24)), *((uint32 *)(buf + 28)));

    IfxScuWdt_clearSafetyEndinitInline(pw);
    g_func.writePage(pageAddr);
    IfxScuWdt_setSafetyEndinitInline(pw);

    g_func.waitUnbusy(FLASH_MODULE, flashType);
}

/*************************************************************/
/* PSPR 복사 (static — 내부 사용)                            */
/*************************************************************/
static void OTA_CopyFuncsToPSPR(void)
{
    if (g_funcCopied)
    {
        return;
    }

    /* eraseMultipleSectors는 복사하지 않는다. raw sector erase command만 복사한다. */
    memcpy((void *)ERASESECTOR_ADDR,   (const void *)OTA_EraseOneSectorCommand, ERASESECTOR_LEN);
    memcpy((void *)WAITUNBUSY_ADDR,    (const void *)IfxFlash_waitUnbusy,       WAITUNBUSY_LEN);
    memcpy((void *)ENTERPAGEMODE_ADDR, (const void *)IfxFlash_enterPageMode,    ENTERPAGEMODE_LEN);
    memcpy((void *)LOAD2X32_ADDR,      (const void *)IfxFlash_loadPage2X32,     LOADPAGE2X32_LEN);
    memcpy((void *)WRITEPAGE_ADDR,     (const void *)IfxFlash_writePage,        WRITEPAGE_LEN);
    memcpy((void *)ERASEWRAPPER_ADDR,  (const void *)OTA_EraseWrapper,          ERASEWRAPPER_LEN);
    memcpy((void *)WRITEWRAPPER_ADDR,  (const void *)OTA_WritePageWrapper,      WRITEWRAPPER_LEN);

    g_func.eraseSectorCmd = (void *)ERASESECTOR_ADDR;
    g_func.waitUnbusy     = (void *)WAITUNBUSY_ADDR;
    g_func.enterPageMode  = (void *)ENTERPAGEMODE_ADDR;
    g_func.load2X32bits   = (void *)LOAD2X32_ADDR;
    g_func.writePage      = (void *)WRITEPAGE_ADDR;
    g_func.eraseWrapper   = (void *)ERASEWRAPPER_ADDR;
    g_func.writePageFull  = (void *)WRITEWRAPPER_ADDR;

    g_funcCopied = TRUE;
}

/*************************************************************/
/* CRC32 helper                                               */
/*************************************************************/

static uint32 OTA_Crc32_UpdateByte(uint32 crc, uint8 data)
{
    uint8 bit;

    crc ^= (uint32)data;

    for (bit = 0U; bit < 8U; bit++)
    {
        if ((crc & 1U) != 0U)
        {
            crc = (crc >> 1U) ^ 0xEDB88320UL;
        }
        else
        {
            crc = crc >> 1U;
        }
    }

    return crc;
}

static uint32 OTA_Crc32_UpdateFill(uint32 crc,
                                   uint8 fill,
                                   uint32 count)
{
    uint32 i;

    for (i = 0U; i < count; i++)
    {
        crc = OTA_Crc32_UpdateByte(crc, fill);
    }

    return crc;
}

static uint32 OTA_Crc32_UpdateFlashRange(uint32 crc,
                                         uint32 startAddr,
                                         uint32 size)
{
    uint32 i;
    uint32 readAddr;
    volatile const uint8 *p;

    readAddr = TO_FLASH_ADDR(startAddr);
    p = (volatile const uint8 *)readAddr;

    for (i = 0U; i < size; i++)
    {
        crc = OTA_Crc32_UpdateByte(crc, p[i]);
    }

    return crc;
}

/*************************************************************/
/* 외부 인터페이스 구현                                       */
/*************************************************************/

boolean OTA_Flash_Erase(uint32 addr,
                        uint32 size,
                        IfxFlash_FlashType flashType)
{
    uint32 flashAddr;
    uint32 alignedAddr;
    uint32 alignedEnd;
    uint32 sectorCount;
    uint32 err;
    boolean irq;

    /* PFlash 조작 중 다른 코어의 fetch/access 방지 */
    IfxCpu_setCoreMode(&MODULE_CPU1, IfxCpu_CoreMode_halt);
    IfxCpu_setCoreMode(&MODULE_CPU2, IfxCpu_CoreMode_halt);

    OTA_CopyFuncsToPSPR();

    flashAddr   = TO_FLASH_ADDR(addr);
    alignedAddr = flashAddr & ~(PFLASH_SECTOR_SIZE - 1U);
    alignedEnd  = ((flashAddr + size) + PFLASH_SECTOR_SIZE - 1U)
                  & ~(PFLASH_SECTOR_SIZE - 1U);
    sectorCount = (alignedEnd - alignedAddr) / PFLASH_SECTOR_SIZE;

    irq = IfxCpu_disableInterrupts();

    IfxFlash_clearStatus(FLASH_MODULE);

    err = g_func.eraseWrapper(alignedAddr, sectorCount, flashType);

    /* wrapper 내부에서 감지하지 못한 잔여 에러까지 최종 확인 */
    if (err == 0U)
    {
        err = MODULE_DMU.HF_ERRSR.U;
    }

    IfxCpu_restoreInterrupts(irq);

    return (err == 0U) ? TRUE : FALSE;
}

boolean OTA_Flash_Write(uint32 addr,
                        uint8 *data,
                        uint16 len,
                        IfxFlash_FlashType flashType)
{
    uint32 writeAddr;
    uint16 offset;
    uint32 pageBuf[PFLASH_PAGE_LEN / 4U] IFX_ALIGN(4);
    boolean irq;

    OTA_CopyFuncsToPSPR();

    writeAddr = TO_FLASH_ADDR(addr);
    offset    = 0U;

    irq = IfxCpu_disableInterrupts();

    IfxFlash_clearStatus(FLASH_MODULE);

    while (offset < len)
    {
        uint16 copyLen;

        memset(pageBuf, 0xFF, PFLASH_PAGE_LEN);

        copyLen = ((len - offset) > PFLASH_PAGE_LEN)
                  ? PFLASH_PAGE_LEN
                  : (uint16)(len - offset);

        memcpy(pageBuf, data + offset, copyLen);

        g_func.writePageFull(writeAddr, (uint8 *)pageBuf, flashType);

        if (MODULE_DMU.HF_ERRSR.U != 0U)
        {
            IfxCpu_restoreInterrupts(irq);
            return FALSE;
        }

        writeAddr += PFLASH_PAGE_LEN;
        offset    += PFLASH_PAGE_LEN;
    }

    IfxCpu_restoreInterrupts(irq);
    return TRUE;
}

boolean OTA_Flash_ReadPendingMeta(OtaPendingMeta_t *meta)
{
    volatile const OtaPendingMeta_t *src;

    if (meta == NULL_PTR)
    {
        return FALSE;
    }

    src = (volatile const OtaPendingMeta_t *)OTA_FLAG_ADDR;

    memcpy(meta, (const void *)src, sizeof(OtaPendingMeta_t));

    if (meta->magic != OTA_FLAG_MAGIC)
    {
        return FALSE;
    }

    if (meta->version != OTA_META_VERSION)
    {
        return FALSE;
    }

    if ((meta->virtualSize == 0U) || (meta->virtualSize > BANK_A_SIZE))
    {
        return FALSE;
    }

    if ((meta->segmentCount == 0U) || (meta->segmentCount > OTA_META_MAX_SEGMENTS))
    {
        return FALSE;
    }

    if (meta->gapFill > 0xFFU)
    {
        return FALSE;
    }

    return TRUE;
}

boolean OTA_Flash_VerifySparseCRC(uint32 targetBase, const OtaPendingMeta_t *meta)
{
    uint32 crc;
    uint32 cursor;
    uint32 i;

    if (meta == NULL_PTR)
    {
        return FALSE;
    }

    if (meta->magic != OTA_FLAG_MAGIC)
    {
        return FALSE;
    }

    if (meta->version != OTA_META_VERSION)
    {
        return FALSE;
    }

    if ((meta->virtualSize == 0U) || (meta->virtualSize > BANK_A_SIZE))
    {
        return FALSE;
    }

    if ((meta->segmentCount == 0U) || (meta->segmentCount > OTA_META_MAX_SEGMENTS))
    {
        return FALSE;
    }

    if (meta->gapFill > 0xFFU)
    {
        return FALSE;
    }

    printf("[BL][CRC] metadata sparse verify start\r\n");
    printf("[BL][CRC] targetBase  = 0x%08x\r\n", targetBase);
    printf("[BL][CRC] virtualSize = 0x%08x\r\n", meta->virtualSize);
    printf("[BL][CRC] expected    = 0x%08x\r\n", meta->expectedCrc32);
    printf("[BL][CRC] segCount    = %u\r\n", meta->segmentCount);
    printf("[BL][CRC] gapFill     = 0x%02x\r\n", meta->gapFill);

    crc = 0xFFFFFFFFUL;
    cursor = 0U;

    for (i = 0U; i < meta->segmentCount; i++)
    {
        uint32 segOffset;
        uint32 segSize;
        uint32 segEnd;
        uint32 gapSize;

        segOffset = meta->segments[i].offset;
        segSize   = meta->segments[i].size;
        segEnd    = segOffset + segSize;

        /*
         * Segment sanity check.
         * 현재 방식은 segment가 offset 오름차순으로 들어온다고 가정한다.
         */
        if (segSize == 0U)
        {
            printf("[BL][CRC] invalid segment size. index=%u\r\n", i);
            return FALSE;
        }

        if (segOffset < cursor)
        {
            printf("[BL][CRC] segment order error. index=%u offset=0x%08x cursor=0x%08x\r\n",
                   i,
                   segOffset,
                   cursor);
            return FALSE;
        }

        if (segEnd < segOffset)
        {
            printf("[BL][CRC] segment overflow. index=%u\r\n", i);
            return FALSE;
        }

        if (segEnd > meta->virtualSize)
        {
            printf("[BL][CRC] segment exceeds virtual size. index=%u end=0x%08x virtual=0x%08x\r\n",
                   i,
                   segEnd,
                   meta->virtualSize);
            return FALSE;
        }

        /*
         * Gap: Flash를 읽지 않고 gapFill 값으로 CRC에 반영.
         */
        if (cursor < segOffset)
        {
            gapSize = segOffset - cursor;

            printf("[BL][CRC] gap index=%u offset=0x%08x size=0x%08x fill=0x%02x\r\n",
                   i,
                   cursor,
                   gapSize,
                   meta->gapFill);

            crc = OTA_Crc32_UpdateFill(crc,
                                       (uint8)meta->gapFill,
                                       gapSize);

            cursor = segOffset;
        }

        /*
         * Segment: 실제 Flash에서 읽어서 CRC 반영.
         */
        printf("[BL][CRC] segment index=%u offset=0x%08x size=0x%08x target=0x%08x\r\n",
               i,
               segOffset,
               segSize,
               targetBase + segOffset);

        crc = OTA_Crc32_UpdateFlashRange(crc,
                                         targetBase + segOffset,
                                         segSize);

        cursor = segEnd;
    }

    /*
     * 마지막 segment 뒤 tail gap.
     */
    if (cursor < meta->virtualSize)
    {
        uint32 tailGapSize;

        tailGapSize = meta->virtualSize - cursor;

        printf("[BL][CRC] tail gap offset=0x%08x size=0x%08x fill=0x%02x\r\n",
               cursor,
               tailGapSize,
               meta->gapFill);

        crc = OTA_Crc32_UpdateFill(crc,
                                   (uint8)meta->gapFill,
                                   tailGapSize);
    }

    crc ^= 0xFFFFFFFFUL;

    printf("[BL][CRC] actual   = 0x%08x\r\n", crc);
    printf("[BL][CRC] expected = 0x%08x\r\n", meta->expectedCrc32);

    if (crc == meta->expectedCrc32)
    {
        printf("[BL][CRC] MATCH\r\n");
        return TRUE;
    }

    printf("[BL][CRC] MISMATCH\r\n");
    return FALSE;
}
/* ── CRC32 검증: sparse image aware version ─────────────────── */
/*
 * 기존 full bin 방식:
 *   addr부터 size만큼 Flash를 연속으로 읽으며 CRC 계산
 *
 * 현재 sparse OTA 방식:
 *   segment1 영역은 Flash에서 읽음
 *   segment1~segment2 사이 gap은 Flash를 읽지 않고 0x00으로 CRC에 반영
 *   segment2 영역은 Flash에서 읽음
 *
 * 이유:
 *   sparse gap은 실제로 write하지 않은 영역이다.
 *   해당 구간을 CPU가 PFLASH에서 직접 읽으면 ECC/trap/멈춤 문제가 발생할 수 있다.
 */
boolean OTA_Flash_VerifyCRC(uint32 addr,
                            uint32 size,
                            uint32 expectedCRC)
{
    uint32 crc;
    uint32 cursor;
    uint32 seg1Start;
    uint32 seg1End;
    uint32 seg2Start;
    uint32 seg2End;
    uint32 gapSize;

    printf("[BL][CRC] sparse verify start\r\n");
    printf("[BL][CRC] base=0x%08x size=0x%08x expected=0x%08x\r\n",
           addr,
           size,
           expectedCRC);

    seg1Start = OTA_SPARSE_SEG1_OFFSET;
    seg1End   = OTA_SPARSE_SEG1_OFFSET + OTA_SPARSE_SEG1_SIZE;

    seg2Start = OTA_SPARSE_SEG2_OFFSET;
    seg2End   = OTA_SPARSE_SEG2_OFFSET + OTA_SPARSE_SEG2_SIZE;

    if (size == 0U)
    {
        printf("[BL][CRC] size zero\r\n");
        return FALSE;
    }

    if (size != seg2End)
    {
        printf("[BL][CRC] unexpected size. got=0x%08x expected=0x%08x\r\n",
               size,
               seg2End);
        return FALSE;
    }

    crc = 0xFFFFFFFFUL;
    cursor = 0U;

    if (cursor < seg1Start)
    {
        gapSize = seg1Start - cursor;
        crc = OTA_Crc32_UpdateFill(crc, OTA_SPARSE_GAP_FILL, gapSize);
        cursor = seg1Start;
    }

    crc = OTA_Crc32_UpdateFlashRange(crc,
                                     addr + OTA_SPARSE_SEG1_OFFSET,
                                     OTA_SPARSE_SEG1_SIZE);
    cursor = seg1End;

    if (cursor < seg2Start)
    {
        gapSize = seg2Start - cursor;
        crc = OTA_Crc32_UpdateFill(crc, OTA_SPARSE_GAP_FILL, gapSize);
        cursor = seg2Start;
    }

    crc = OTA_Crc32_UpdateFlashRange(crc,
                                     addr + OTA_SPARSE_SEG2_OFFSET,
                                     OTA_SPARSE_SEG2_SIZE);
    cursor = seg2End;

    if (cursor < size)
    {
        gapSize = size - cursor;
        crc = OTA_Crc32_UpdateFill(crc, OTA_SPARSE_GAP_FILL, gapSize);
    }

    crc ^= 0xFFFFFFFFUL;

    printf("[BL][CRC] actual=0x%08x expected=0x%08x\r\n",
           crc,
           expectedCRC);

    if (crc == expectedCRC)
    {
        printf("[BL][CRC] MATCH\r\n");
        return TRUE;
    }

    printf("[BL][CRC] MISMATCH\r\n");
    return FALSE;
}

void OTA_Flash_ClearFlag(void)
{
    uint16 pw = IfxScuWdt_getSafetyWatchdogPassword();

    IfxScuWdt_clearSafetyEndinit(pw);
    IfxFlash_eraseMultipleSectors(OTA_FLAG_ADDR, 1);
    IfxScuWdt_setSafetyEndinit(pw);
    IfxFlash_waitUnbusy(FLASH_MODULE, IfxFlash_FlashType_D0);
}

void OTA_Flash_SetFlag(uint32 fwSize,
                       uint32 expectedCRC)
{
    uint16 pw = IfxScuWdt_getSafetyWatchdogPassword();

    /* 1. Erase */
    IfxScuWdt_clearSafetyEndinit(pw);
    IfxFlash_eraseMultipleSectors(OTA_FLAG_ADDR, 1);
    IfxScuWdt_setSafetyEndinit(pw);
    IfxFlash_waitUnbusy(FLASH_MODULE, IfxFlash_FlashType_D0);

    /* 2. MAGIC 저장 */
    IfxFlash_enterPageMode(OTA_FLAG_ADDR);
    IfxFlash_waitUnbusy(FLASH_MODULE, IfxFlash_FlashType_D0);
    IfxFlash_loadPage2X32(OTA_FLAG_ADDR, OTA_FLAG_MAGIC, 0x00000000UL);
    IfxScuWdt_clearSafetyEndinit(pw);
    IfxFlash_writePage(OTA_FLAG_ADDR);
    IfxScuWdt_setSafetyEndinit(pw);
    IfxFlash_waitUnbusy(FLASH_MODULE, IfxFlash_FlashType_D0);

    /* 3. fw_size 저장 */
    IfxFlash_enterPageMode(OTA_FLAG_ADDR + 8U);
    IfxFlash_waitUnbusy(FLASH_MODULE, IfxFlash_FlashType_D0);
    IfxFlash_loadPage2X32(OTA_FLAG_ADDR + 8U, fwSize, 0x00000000UL);
    IfxScuWdt_clearSafetyEndinit(pw);
    IfxFlash_writePage(OTA_FLAG_ADDR + 8U);
    IfxScuWdt_setSafetyEndinit(pw);
    IfxFlash_waitUnbusy(FLASH_MODULE, IfxFlash_FlashType_D0);

    /* 4. expectedCRC 저장 */
    IfxFlash_enterPageMode(OTA_FLAG_ADDR + 16U);
    IfxFlash_waitUnbusy(FLASH_MODULE, IfxFlash_FlashType_D0);
    IfxFlash_loadPage2X32(OTA_FLAG_ADDR + 16U, expectedCRC, 0x00000000UL);
    IfxScuWdt_clearSafetyEndinit(pw);
    IfxFlash_writePage(OTA_FLAG_ADDR + 16U);
    IfxScuWdt_setSafetyEndinit(pw);
    IfxFlash_waitUnbusy(FLASH_MODULE, IfxFlash_FlashType_D0);
}
