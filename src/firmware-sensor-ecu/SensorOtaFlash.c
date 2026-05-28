#include "SensorOtaFlash.h"

#include "IfxCpu.h"
#include "IfxScuWdt.h"

#include <string.h>

typedef struct
{
    void  (*eraseSectorCmd)(uint32 sectorAddr);
    uint8 (*waitUnbusy)(uint32 flash, IfxFlash_FlashType flashType);
    uint8 (*enterPageMode)(uint32 pageAddr);
    void  (*load2X32bits)(uint32 pageAddr, uint32 wordL, uint32 wordU);
    void  (*writePage)(uint32 pageAddr);
    uint32 (*eraseWrapper)(uint32 sectorAddr, uint32 sectorCount, IfxFlash_FlashType flashType);
    void  (*writePageFull)(uint32 pageAddr, uint8 *buf, IfxFlash_FlashType flashType);
} SensorOtaFlashFunc_t;

static SensorOtaFlashFunc_t g_func;
static boolean g_funcCopied = FALSE;

static void eraseOneSectorCommand(uint32 sectorAddr)
{
    volatile uint32 *addr1 = (volatile uint32 *)(IFXFLASH_CMD_BASE_ADDRESS | 0xaa50U);
    volatile uint32 *addr2 = (volatile uint32 *)(IFXFLASH_CMD_BASE_ADDRESS | 0xaa58U);
    volatile uint32 *addr3 = (volatile uint32 *)(IFXFLASH_CMD_BASE_ADDRESS | 0xaaa8U);
    volatile uint32 *addr4 = (volatile uint32 *)(IFXFLASH_CMD_BASE_ADDRESS | 0xaaa8U);

    *addr1 = sectorAddr;
    *addr2 = 1U;
    *addr3 = 0x80U;
    *addr4 = 0x50U;

    __dsync();
}

static uint32 eraseWrapper(uint32 sectorAddr, uint32 sectorCount, IfxFlash_FlashType flashType)
{
    uint16 pw = IfxScuWdt_getSafetyWatchdogPasswordInline();
    uint32 err = 0U;

    for (uint32 i = 0U; i < sectorCount; i++)
    {
        uint32 addr = sectorAddr + (i * SENSOR_OTA_PFLASH_SECTOR_SIZE);

        IfxScuWdt_clearSafetyEndinitInline(pw);
        g_func.eraseSectorCmd(addr);
        IfxScuWdt_setSafetyEndinitInline(pw);

        g_func.waitUnbusy(SENSOR_OTA_FLASH_MODULE, flashType);

        err = MODULE_DMU.HF_ERRSR.U;
        if (err != 0U)
        {
            break;
        }
    }

    return err;
}

static void writePageWrapper(uint32 pageAddr, uint8 *buf, IfxFlash_FlashType flashType)
{
    uint16 pw = IfxScuWdt_getSafetyWatchdogPasswordInline();

    g_func.enterPageMode(pageAddr);
    g_func.waitUnbusy(SENSOR_OTA_FLASH_MODULE, flashType);

    g_func.load2X32bits(pageAddr, *((uint32 *)(buf + 0U)),  *((uint32 *)(buf + 4U)));
    g_func.load2X32bits(pageAddr, *((uint32 *)(buf + 8U)),  *((uint32 *)(buf + 12U)));
    g_func.load2X32bits(pageAddr, *((uint32 *)(buf + 16U)), *((uint32 *)(buf + 20U)));
    g_func.load2X32bits(pageAddr, *((uint32 *)(buf + 24U)), *((uint32 *)(buf + 28U)));

    IfxScuWdt_clearSafetyEndinitInline(pw);
    g_func.writePage(pageAddr);
    IfxScuWdt_setSafetyEndinitInline(pw);

    g_func.waitUnbusy(SENSOR_OTA_FLASH_MODULE, flashType);
}

static void copyFuncsToPspr(void)
{
    if (g_funcCopied == TRUE)
    {
        return;
    }

    memcpy((void *)SENSOR_OTA_ERASESECTOR_ADDR,   (const void *)eraseOneSectorCommand, SENSOR_OTA_ERASESECTOR_LEN);
    memcpy((void *)SENSOR_OTA_WAITUNBUSY_ADDR,    (const void *)IfxFlash_waitUnbusy,   SENSOR_OTA_WAITUNBUSY_LEN);
    memcpy((void *)SENSOR_OTA_ENTERPAGEMODE_ADDR, (const void *)IfxFlash_enterPageMode, SENSOR_OTA_ENTERPAGEMODE_LEN);
    memcpy((void *)SENSOR_OTA_LOAD2X32_ADDR,      (const void *)IfxFlash_loadPage2X32, SENSOR_OTA_LOAD2X32_LEN);
    memcpy((void *)SENSOR_OTA_WRITEPAGE_ADDR,     (const void *)IfxFlash_writePage,    SENSOR_OTA_WRITEPAGE_LEN);
    memcpy((void *)SENSOR_OTA_ERASEWRAPPER_ADDR,  (const void *)eraseWrapper,          SENSOR_OTA_ERASEWRAPPER_LEN);
    memcpy((void *)SENSOR_OTA_WRITEWRAPPER_ADDR,  (const void *)writePageWrapper,      SENSOR_OTA_WRITEWRAPPER_LEN);

    g_func.eraseSectorCmd = (void *)SENSOR_OTA_ERASESECTOR_ADDR;
    g_func.waitUnbusy = (void *)SENSOR_OTA_WAITUNBUSY_ADDR;
    g_func.enterPageMode = (void *)SENSOR_OTA_ENTERPAGEMODE_ADDR;
    g_func.load2X32bits = (void *)SENSOR_OTA_LOAD2X32_ADDR;
    g_func.writePage = (void *)SENSOR_OTA_WRITEPAGE_ADDR;
    g_func.eraseWrapper = (void *)SENSOR_OTA_ERASEWRAPPER_ADDR;
    g_func.writePageFull = (void *)SENSOR_OTA_WRITEWRAPPER_ADDR;

    g_funcCopied = TRUE;
}

boolean SensorOtaFlash_Erase(uint32 addr, uint32 size, IfxFlash_FlashType flashType)
{
    uint32 flashAddr;
    uint32 alignedAddr;
    uint32 alignedEnd;
    uint32 sectorCount;
    uint32 err;
    boolean irq;

    IfxCpu_setCoreMode(&MODULE_CPU1, IfxCpu_CoreMode_halt);
    IfxCpu_setCoreMode(&MODULE_CPU2, IfxCpu_CoreMode_halt);

    copyFuncsToPspr();

    flashAddr = SENSOR_OTA_TO_FLASH_ADDR(addr);
    alignedAddr = flashAddr & ~(SENSOR_OTA_PFLASH_SECTOR_SIZE - 1U);
    alignedEnd = (flashAddr + size + SENSOR_OTA_PFLASH_SECTOR_SIZE - 1U) &
                 ~(SENSOR_OTA_PFLASH_SECTOR_SIZE - 1U);
    sectorCount = (alignedEnd - alignedAddr) / SENSOR_OTA_PFLASH_SECTOR_SIZE;

    irq = IfxCpu_disableInterrupts();

    IfxFlash_clearStatus(SENSOR_OTA_FLASH_MODULE);

    err = g_func.eraseWrapper(alignedAddr, sectorCount, flashType);
    if (err == 0U)
    {
        err = MODULE_DMU.HF_ERRSR.U;
    }

    IfxCpu_restoreInterrupts(irq);

    return (err == 0U) ? TRUE : FALSE;
}

boolean SensorOtaFlash_Write(uint32 addr, const uint8 *data, uint16 len, IfxFlash_FlashType flashType)
{
    uint32 writeAddr;
    uint16 offset = 0U;
    uint32 pageBuf[SENSOR_OTA_PFLASH_PAGE_LEN / 4U] IFX_ALIGN(4);
    boolean irq;

    if ((data == NULL_PTR) || (len == 0U))
    {
        return FALSE;
    }

    copyFuncsToPspr();

    writeAddr = SENSOR_OTA_TO_FLASH_ADDR(addr);
    irq = IfxCpu_disableInterrupts();

    IfxFlash_clearStatus(SENSOR_OTA_FLASH_MODULE);

    while (offset < len)
    {
        uint16 copyLen;

        memset(pageBuf, 0xFF, SENSOR_OTA_PFLASH_PAGE_LEN);

        copyLen = ((len - offset) > SENSOR_OTA_PFLASH_PAGE_LEN) ?
                  SENSOR_OTA_PFLASH_PAGE_LEN : (uint16)(len - offset);
        memcpy(pageBuf, data + offset, copyLen);

        g_func.writePageFull(writeAddr, (uint8 *)pageBuf, flashType);

        if (MODULE_DMU.HF_ERRSR.U != 0U)
        {
            IfxCpu_restoreInterrupts(irq);
            return FALSE;
        }

        writeAddr += SENSOR_OTA_PFLASH_PAGE_LEN;
        offset += SENSOR_OTA_PFLASH_PAGE_LEN;
    }

    IfxCpu_restoreInterrupts(irq);
    return TRUE;
}
