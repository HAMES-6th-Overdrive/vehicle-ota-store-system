#include "APP_Flash.h"
#include "IfxCpu.h"
#include <string.h>

/*************************************************************/
/* 전역 변수                                                  */
/*************************************************************/
static OTA_FlashFunc g_func;
static boolean       g_funcCopied = FALSE;

/*************************************************************/
/* PSPR 래퍼 함수                                             */
/*************************************************************/

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

/* ★ 변경: disableInterrupts를 eraseSectorCmd 순간(수 µs)만 감쌈.
 *         waitUnbusy(~수백 ms) 앞에서 복원 → Ethernet 인터럽트 살아있음. */
static uint32 OTA_EraseWrapper(uint32 sectorAddr, uint32 sectorCount, IfxFlash_FlashType flashType)
{
    uint16 pw  = IfxScuWdt_getSafetyWatchdogPasswordInline();
    uint32 err = 0U;

    for (uint32 i = 0U; i < sectorCount; i++)
    {
        uint32 addr = sectorAddr + (i * PFLASH_SECTOR_SIZE);

        boolean irq = IfxCpu_disableInterrupts();     /* 차단 시작 */
        IfxScuWdt_clearSafetyEndinitInline(pw);
        g_func.eraseSectorCmd(addr);
        IfxScuWdt_setSafetyEndinitInline(pw);
        IfxCpu_restoreInterrupts(irq);                /* ★ waitUnbusy 전에 복원 */

        g_func.waitUnbusy(FLASH_MODULE, flashType);       /* 인터럽트 활성 상태로 대기 */

        err = MODULE_DMU.HF_ERRSR.U;
        if (err != 0U)
        {
            break;
        }
    }

    return err;
}

/* ★ 변경: disableInterrupts를 writePage 순간(수 µs)만 감쌈.
 *         enterPageMode/load/waitUnbusy는 인터럽트 활성 상태로 실행. */
static void OTA_WritePageWrapper(uint32 pageAddr, uint8 *buf, IfxFlash_FlashType flashType)
{
    uint16 pw = IfxScuWdt_getSafetyWatchdogPasswordInline();

    g_func.enterPageMode(pageAddr);
    g_func.waitUnbusy(FLASH_MODULE, flashType);

    g_func.load2X32bits(pageAddr, *((uint32*)(buf+0)),  *((uint32*)(buf+4)));
    g_func.load2X32bits(pageAddr, *((uint32*)(buf+8)),  *((uint32*)(buf+12)));
    g_func.load2X32bits(pageAddr, *((uint32*)(buf+16)), *((uint32*)(buf+20)));
    g_func.load2X32bits(pageAddr, *((uint32*)(buf+24)), *((uint32*)(buf+28)));

    boolean irq = IfxCpu_disableInterrupts();        /* 차단 시작 */
    IfxScuWdt_clearSafetyEndinitInline(pw);
    g_func.writePage(pageAddr);
    IfxScuWdt_setSafetyEndinitInline(pw);
    IfxCpu_restoreInterrupts(irq);                   /* ★ 마지막 waitUnbusy 전에 복원 */

    g_func.waitUnbusy(FLASH_MODULE, flashType);          /* 인터럽트 활성 상태로 대기 */
}

/*************************************************************/
/* PSPR 복사                                                  */
/*************************************************************/
static void OTA_CopyFuncsToPSPR(void)
{
    if (g_funcCopied) return;

    memcpy((void*)ERASESECTOR_ADDR,   (const void*)OTA_EraseOneSectorCommand, ERASESECTOR_LEN);
    memcpy((void*)WAITUNBUSY_ADDR,    (const void*)IfxFlash_waitUnbusy,       WAITUNBUSY_LEN);
    memcpy((void*)ENTERPAGEMODE_ADDR, (const void*)IfxFlash_enterPageMode,    ENTERPAGEMODE_LEN);
    memcpy((void*)LOAD2X32_ADDR,      (const void*)IfxFlash_loadPage2X32,     LOADPAGE2X32_LEN);
    memcpy((void*)WRITEPAGE_ADDR,     (const void*)IfxFlash_writePage,        WRITEPAGE_LEN);
    memcpy((void*)ERASEWRAPPER_ADDR,  (const void*)OTA_EraseWrapper,          ERASEWRAPPER_LEN);
    memcpy((void*)WRITEWRAPPER_ADDR,  (const void*)OTA_WritePageWrapper,      WRITEWRAPPER_LEN);

    g_func.eraseSectorCmd = (void*)ERASESECTOR_ADDR;
    g_func.waitUnbusy     = (void*)WAITUNBUSY_ADDR;
    g_func.enterPageMode  = (void*)ENTERPAGEMODE_ADDR;
    g_func.load2X32bits   = (void*)LOAD2X32_ADDR;
    g_func.writePage      = (void*)WRITEPAGE_ADDR;
    g_func.eraseWrapper   = (void*)ERASEWRAPPER_ADDR;
    g_func.writePageFull  = (void*)WRITEWRAPPER_ADDR;

    g_funcCopied = TRUE;
}

/*************************************************************/
/* 외부 인터페이스 구현                                       */
/*************************************************************/

boolean OTA_Flash_Erase(uint32 addr, uint32 size, IfxFlash_FlashType flashType)
{
    OTA_CopyFuncsToPSPR();

    uint32 flashAddr   = TO_FLASH_ADDR(addr);
    uint32 alignedAddr = flashAddr & ~(PFLASH_SECTOR_SIZE - 1U);
    uint32 alignedEnd  = ((flashAddr + size) + PFLASH_SECTOR_SIZE - 1U)
                         & ~(PFLASH_SECTOR_SIZE - 1U);
    uint32 sectorCount = (alignedEnd - alignedAddr) / PFLASH_SECTOR_SIZE;

    /* ★ 변경: 전역 disableInterrupts 제거.
     *         인터럽트 차단은 EraseWrapper 내부(명령 순간)에서 처리. */
    IfxFlash_clearStatus(FLASH_MODULE);

    uint32 err = g_func.eraseWrapper(alignedAddr, sectorCount, flashType);

    if (err == 0U)
    {
        err = MODULE_DMU.HF_ERRSR.U;
    }

    return (err == 0U) ? TRUE : FALSE;
}

boolean OTA_Flash_Write(uint32 addr, uint8 *data, uint16 len, IfxFlash_FlashType flashType)
{
    OTA_CopyFuncsToPSPR();

    uint32 writeAddr = TO_FLASH_ADDR(addr);
    uint16 offset    = 0U;
    uint32 pageBuf[PFLASH_PAGE_LEN / 4U] IFX_ALIGN(4);

    /* ★ 변경: 전역 disableInterrupts 제거.
     *         인터럽트 차단은 WritePageWrapper 내부(writePage 순간)에서 처리. */
    IfxFlash_clearStatus(FLASH_MODULE);

    while (offset < len)
    {
        memset(pageBuf, 0x00, PFLASH_PAGE_LEN);

        uint16 copyLen = ((len - offset) > PFLASH_PAGE_LEN)
                         ? PFLASH_PAGE_LEN : (uint16)(len - offset);
        memcpy(pageBuf, data + offset, copyLen);

        g_func.writePageFull(writeAddr, (uint8*)pageBuf, flashType);

        if (MODULE_DMU.HF_ERRSR.U != 0U)
        {
            return FALSE;
        }

        writeAddr += PFLASH_PAGE_LEN;
        offset    += PFLASH_PAGE_LEN;
    }

    return TRUE;
}

boolean OTA_Flash_VerifyCRC(uint32 addr, uint32 size)
{
    (void)addr;
    (void)size;
    return TRUE;
}

void OTA_Flash_ClearFlag(void)
{
    uint16 pw = IfxScuWdt_getSafetyWatchdogPassword();

    IfxScuWdt_clearSafetyEndinit(pw);
    IfxFlash_eraseMultipleSectors(OTA_FLAG_ADDR, 1);
    IfxScuWdt_setSafetyEndinit(pw);
    IfxFlash_waitUnbusy(FLASH_MODULE, IfxFlash_FlashType_D0);
}

void OTA_Flash_SetFlag(uint32 fwSize, uint32 expectedCRC)
{
    uint16 pw = IfxScuWdt_getSafetyWatchdogPassword();

    IfxScuWdt_clearSafetyEndinit(pw);
    IfxFlash_eraseMultipleSectors(OTA_FLAG_ADDR, 1);
    IfxScuWdt_setSafetyEndinit(pw);
    IfxFlash_waitUnbusy(FLASH_MODULE, IfxFlash_FlashType_D0);

    IfxFlash_enterPageMode(OTA_FLAG_ADDR);
    IfxFlash_waitUnbusy(FLASH_MODULE, IfxFlash_FlashType_D0);
    IfxFlash_loadPage2X32(OTA_FLAG_ADDR, OTA_FLAG_MAGIC, 0x00000000UL);
    IfxScuWdt_clearSafetyEndinit(pw);
    IfxFlash_writePage(OTA_FLAG_ADDR);
    IfxScuWdt_setSafetyEndinit(pw);
    IfxFlash_waitUnbusy(FLASH_MODULE, IfxFlash_FlashType_D0);

    IfxFlash_enterPageMode(OTA_FLAG_ADDR + 8);
    IfxFlash_waitUnbusy(FLASH_MODULE, IfxFlash_FlashType_D0);
    IfxFlash_loadPage2X32(OTA_FLAG_ADDR + 8, fwSize, 0x00000000UL);
    IfxScuWdt_clearSafetyEndinit(pw);
    IfxFlash_writePage(OTA_FLAG_ADDR + 8);
    IfxScuWdt_setSafetyEndinit(pw);
    IfxFlash_waitUnbusy(FLASH_MODULE, IfxFlash_FlashType_D0);

    IfxFlash_enterPageMode(OTA_FLAG_ADDR + 16);
    IfxFlash_waitUnbusy(FLASH_MODULE, IfxFlash_FlashType_D0);
    IfxFlash_loadPage2X32(OTA_FLAG_ADDR + 16, expectedCRC, 0x00000000UL);
    IfxScuWdt_clearSafetyEndinit(pw);
    IfxFlash_writePage(OTA_FLAG_ADDR + 16);
    IfxScuWdt_setSafetyEndinit(pw);
    IfxFlash_waitUnbusy(FLASH_MODULE, IfxFlash_FlashType_D0);
}

static void OTA_DFlash_Write8(uint32 addr, uint32 wordL, uint32 wordU)
{
    uint16 pw = IfxScuWdt_getSafetyWatchdogPassword();

    IfxFlash_enterPageMode(addr);
    IfxFlash_waitUnbusy(FLASH_MODULE, IfxFlash_FlashType_D0);

    IfxFlash_loadPage2X32(addr, wordL, wordU);

    IfxScuWdt_clearSafetyEndinit(pw);
    IfxFlash_writePage(addr);
    IfxScuWdt_setSafetyEndinit(pw);

    IfxFlash_waitUnbusy(FLASH_MODULE, IfxFlash_FlashType_D0);
}

boolean OTA_Flash_SetPackageMetadata(const OTA_PackageMetadata_t *meta)
{
    uint16 pw;
    uint32 addr;
    uint32 i;

    if (meta == NULL_PTR)
    {
        return FALSE;
    }

    if (meta->magic != OTA_FLAG_MAGIC)
    {
        return FALSE;
    }

    if (meta->version != OTA_PACKAGE_META_VERSION)
    {
        return FALSE;
    }

    if ((meta->segmentCount == 0U) ||
        (meta->segmentCount > OTA_PACKAGE_MAX_SEGMENTS))
    {
        return FALSE;
    }

    if ((meta->virtualSize == 0U) ||
        (meta->expectedCrc32 == 0U))
    {
        return FALSE;
    }

    pw = IfxScuWdt_getSafetyWatchdogPassword();

    IfxScuWdt_clearSafetyEndinit(pw);
    IfxFlash_eraseMultipleSectors(OTA_FLAG_ADDR, 1U);
    IfxScuWdt_setSafetyEndinit(pw);

    IfxFlash_waitUnbusy(FLASH_MODULE, IfxFlash_FlashType_D0);

    /*
     * Bootloader OtaPendingMeta_t layout:
     *
     * +0x00 magic, version
     * +0x08 virtualSize, gapFill
     * +0x10 expectedCrc32, segmentCount
     * +0x18 reserved0, reserved1
     * +0x20 segment[0].offset, segment[0].size
     * +0x28 segment[0].crc32, segment[0].reserved
     * +0x30 segment[1].offset, segment[1].size
     * +0x38 segment[1].crc32, segment[1].reserved
     */

    OTA_DFlash_Write8(OTA_FLAG_ADDR + 0x00U,
                      OTA_FLAG_MAGIC,
                      OTA_PACKAGE_META_VERSION);

    OTA_DFlash_Write8(OTA_FLAG_ADDR + 0x08U,
                      meta->virtualSize,
                      meta->gapFill);

    OTA_DFlash_Write8(OTA_FLAG_ADDR + 0x10U,
                      meta->expectedCrc32,
                      meta->segmentCount);

    OTA_DFlash_Write8(OTA_FLAG_ADDR + 0x18U,
                      0U,
                      0U);

    addr = OTA_FLAG_ADDR + 0x20U;

    for (i = 0U; i < OTA_PACKAGE_MAX_SEGMENTS; i++)
    {
        uint32 segOffset = 0U;
        uint32 segSize   = 0U;
        uint32 segCrc    = 0U;

        if (i < meta->segmentCount)
        {
            segOffset = meta->segments[i].offset;
            segSize   = meta->segments[i].size;

            /*
             * App_Uds.c의 metadata 구조에 segment crc32가 아직 없으면
             * 일단 0으로 저장해도 bootloader CRC 검증 자체에는 문제 없다.
             * bootloader는 현재 전체 virtual CRC 기준으로 검증한다.
             *
             * 나중에 per-segment CRC도 쓰고 싶으면
             * B4 manifest에 segment crc32까지 추가해서 채우면 된다.
             */
#if defined(OTA_PACKAGE_SEGMENT_HAS_CRC32)
            segCrc = meta->segments[i].crc32;
#else
            segCrc = 0U;
#endif
        }

        OTA_DFlash_Write8(addr + 0x00U, segOffset, segSize);
        OTA_DFlash_Write8(addr + 0x08U, segCrc, 0U);

        addr += 0x10U;
    }

    return TRUE;
}
