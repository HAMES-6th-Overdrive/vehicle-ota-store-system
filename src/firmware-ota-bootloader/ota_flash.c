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
static uint32 OTA_EraseWrapper(uint32 sectorAddr, uint32 sectorCount, IfxFlash_FlashType flashType)
{
    uint16 pw = IfxScuWdt_getSafetyWatchdogPasswordInline();
    uint32 err = 0U;

    for (uint32 i = 0U; i < sectorCount; i++)
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
static void OTA_WritePageWrapper(uint32 pageAddr, uint8 *buf, IfxFlash_FlashType flashType)
{
    uint16 pw = IfxScuWdt_getSafetyWatchdogPasswordInline();

    g_func.enterPageMode(pageAddr);
    g_func.waitUnbusy(FLASH_MODULE, flashType);

    /* 8바이트씩 4번 = 32바이트 로드 */
    g_func.load2X32bits(pageAddr, *((uint32*)(buf+0)),  *((uint32*)(buf+4)));
    g_func.load2X32bits(pageAddr, *((uint32*)(buf+8)),  *((uint32*)(buf+12)));
    g_func.load2X32bits(pageAddr, *((uint32*)(buf+16)), *((uint32*)(buf+20)));
    g_func.load2X32bits(pageAddr, *((uint32*)(buf+24)), *((uint32*)(buf+28)));

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
    if (g_funcCopied) return;

    /* eraseMultipleSectors는 복사하지 않는다. raw sector erase command만 복사한다. */
    memcpy((void*)ERASESECTOR_ADDR,   (const void*)OTA_EraseOneSectorCommand,     ERASESECTOR_LEN);
    memcpy((void*)WAITUNBUSY_ADDR,    (const void*)IfxFlash_waitUnbusy,           WAITUNBUSY_LEN);
    memcpy((void*)ENTERPAGEMODE_ADDR, (const void*)IfxFlash_enterPageMode,        ENTERPAGEMODE_LEN);
    memcpy((void*)LOAD2X32_ADDR,      (const void*)IfxFlash_loadPage2X32,         LOADPAGE2X32_LEN);
    memcpy((void*)WRITEPAGE_ADDR,     (const void*)IfxFlash_writePage,            WRITEPAGE_LEN);
    memcpy((void*)ERASEWRAPPER_ADDR,  (const void*)OTA_EraseWrapper,              ERASEWRAPPER_LEN);
    memcpy((void*)WRITEWRAPPER_ADDR,  (const void*)OTA_WritePageWrapper,          WRITEWRAPPER_LEN);

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
    /* PFlash 조작 중 다른 코어의 fetch/access 방지 */
    IfxCpu_setCoreMode(&MODULE_CPU1, IfxCpu_CoreMode_halt);
    IfxCpu_setCoreMode(&MODULE_CPU2, IfxCpu_CoreMode_halt);

    OTA_CopyFuncsToPSPR();

    uint32 flashAddr   = TO_FLASH_ADDR(addr);
    uint32 alignedAddr = flashAddr & ~(PFLASH_SECTOR_SIZE - 1U);
    uint32 alignedEnd  = ((flashAddr + size) + PFLASH_SECTOR_SIZE - 1U)
                         & ~(PFLASH_SECTOR_SIZE - 1U);
    uint32 sectorCount = (alignedEnd - alignedAddr) / PFLASH_SECTOR_SIZE;

    boolean irq = IfxCpu_disableInterrupts();

    IfxFlash_clearStatus(FLASH_MODULE);

    uint32 err = g_func.eraseWrapper(alignedAddr, sectorCount, flashType);

    /* wrapper 내부에서 감지하지 못한 잔여 에러까지 최종 확인 */
    if (err == 0U)
    {
        err = MODULE_DMU.HF_ERRSR.U;
    }

    IfxCpu_restoreInterrupts(irq);

    return (err == 0U) ? TRUE : FALSE;
}

boolean OTA_Flash_Write(uint32 addr, uint8 *data, uint16 len, IfxFlash_FlashType flashType)
{
    OTA_CopyFuncsToPSPR();

    uint32 writeAddr = TO_FLASH_ADDR(addr);
    uint16 offset    = 0U;
    uint32 pageBuf[PFLASH_PAGE_LEN / 4U] IFX_ALIGN(4);

    boolean irq = IfxCpu_disableInterrupts();

    IfxFlash_clearStatus(FLASH_MODULE);

    while (offset < len)
    {
        memset(pageBuf, 0xFF, PFLASH_PAGE_LEN);

        uint16 copyLen = ((len - offset) > PFLASH_PAGE_LEN)
                         ? PFLASH_PAGE_LEN : (uint16)(len - offset);
        memcpy(pageBuf, data + offset, copyLen);

        g_func.writePageFull(writeAddr, (uint8*)pageBuf, flashType);

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

/* ── CRC32 검증 ─────────────────────────────────────────────── */
/* Flash에 기록된 데이터를 읽어서 CRC 계산                         */
boolean OTA_Flash_VerifyCRC(uint32 addr, uint32 size, uint32 expectedCRC)
{
    uint32  crc = 0xFFFFFFFF;
    uint8  *ptr = (uint8 *)addr;

    for (uint32 i = 0; i < size; i++)
    {
        crc ^= ptr[i];
        for (uint8 j = 0; j < 8; j++)
            crc = (crc >> 1) ^ (0xEDB88320UL & (uint32)(-(sint32)(crc & 1)));
    }
    crc ^= 0xFFFFFFFF;

    /*
     * TODO: RPi가 0x37 요청에 예상 CRC를 포함해서 보내도록 확장 시
     *       여기서 expected CRC와 비교.
     *       현재는 계산만 하고 항상 TRUE 반환 (개발 단계)
     */
    //return TRUE;
    printf("crc : 0x%08X, size : %x%08X, expectedCRC: 0x%08X\r\n", crc, size, expectedCRC);
    return (crc == expectedCRC);  // 실제 비교
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

    /* 1. Erase */
    IfxScuWdt_clearSafetyEndinit(pw);
    IfxFlash_eraseMultipleSectors(OTA_FLAG_ADDR, 1);
    IfxScuWdt_setSafetyEndinit(pw);
    IfxFlash_waitUnbusy(FLASH_MODULE, IfxFlash_FlashType_D0);

    /* 2. Write (DFLASH는 PSPR 불필요 — 예제와 동일) */
    /* 2. MAGIC 저장 */
    IfxFlash_enterPageMode(OTA_FLAG_ADDR);
    IfxFlash_waitUnbusy(FLASH_MODULE, IfxFlash_FlashType_D0);
    IfxFlash_loadPage2X32(OTA_FLAG_ADDR, OTA_FLAG_MAGIC, 0x00000000UL);
    IfxScuWdt_clearSafetyEndinit(pw);
    IfxFlash_writePage(OTA_FLAG_ADDR);
    IfxScuWdt_setSafetyEndinit(pw);
    IfxFlash_waitUnbusy(FLASH_MODULE, IfxFlash_FlashType_D0);

    /* 3. fw_size 저장 */
    IfxFlash_enterPageMode(OTA_FLAG_ADDR + 8);
    IfxFlash_waitUnbusy(FLASH_MODULE, IfxFlash_FlashType_D0);
    IfxFlash_loadPage2X32(OTA_FLAG_ADDR + 8, fwSize, 0x00000000UL);
    IfxScuWdt_clearSafetyEndinit(pw);
    IfxFlash_writePage(OTA_FLAG_ADDR + 8);
    IfxScuWdt_setSafetyEndinit(pw);
    IfxFlash_waitUnbusy(FLASH_MODULE, IfxFlash_FlashType_D0);

    /* 4. expectedCRC 저장 */
    IfxFlash_enterPageMode(OTA_FLAG_ADDR + 16);
    IfxFlash_waitUnbusy(FLASH_MODULE, IfxFlash_FlashType_D0);
    IfxFlash_loadPage2X32(OTA_FLAG_ADDR + 16, expectedCRC, 0x00000000UL);
    IfxScuWdt_clearSafetyEndinit(pw);
    IfxFlash_writePage(OTA_FLAG_ADDR + 16);
    IfxScuWdt_setSafetyEndinit(pw);
    IfxFlash_waitUnbusy(FLASH_MODULE, IfxFlash_FlashType_D0);
}