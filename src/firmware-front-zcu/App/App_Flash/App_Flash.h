#ifndef APP_FLASH_H
#define APP_FLASH_H

#include "Ifx_Types.h"
#include "IfxFlash.h"
#include "IfxScuWdt.h"
#include "IfxScuRcu.h"

/* ── 주소 정의 ──────────────────────────────────────────────── */
#define BANK_B_START        0x80300000UL   /* PF1 시작 (OTA 타겟) */

#define OTA_FLAG_ADDR       0xAF000000UL    /* DFLASH 플래그 저장 위치 */
#define OTA_FLAG_MAGIC      0xDEADBEEFUL   /* OTA 모드 진입 식별값    */

#define FLASH_MODULE        0
#define PFLASH_PAGE_LEN     32
#define PFLASH_SECTOR_SIZE  0x4000UL       /* 16KB */

/* 캐시(0x80xxxxxx) → 비캐시(0xA0xxxxxx) 변환 */
#define TO_FLASH_ADDR(addr) (((addr) & 0x0FFFFFFFU) | 0xA0000000U)

/* ── PSPR 배치 주소 ─────────────────────────────────────────── */
#define RELOCATION_START    0x70100000U

#define ERASESECTOR_LEN     256
#define WAITUNBUSY_LEN      256
#define ENTERPAGEMODE_LEN   256
#define LOADPAGE2X32_LEN    256
#define WRITEPAGE_LEN       256
#define ERASEWRAPPER_LEN    0x400
#define WRITEWRAPPER_LEN    0x400

#define ERASESECTOR_ADDR    (RELOCATION_START)
#define WAITUNBUSY_ADDR     (ERASESECTOR_ADDR   + ERASESECTOR_LEN)
#define ENTERPAGEMODE_ADDR  (WAITUNBUSY_ADDR    + WAITUNBUSY_LEN)
#define LOAD2X32_ADDR       (ENTERPAGEMODE_ADDR + ENTERPAGEMODE_LEN)
#define WRITEPAGE_ADDR      (LOAD2X32_ADDR      + LOADPAGE2X32_LEN)
#define ERASEWRAPPER_ADDR   (WRITEPAGE_ADDR     + WRITEPAGE_LEN)
#define WRITEWRAPPER_ADDR   (ERASEWRAPPER_ADDR  + ERASEWRAPPER_LEN)

#define OTA_PACKAGE_META_VERSION          1U
#define OTA_PACKAGE_MAX_SEGMENTS          4U

/*
 * DFLASH layout at OTA_FLAG_ADDR
 *
 * +0x00: OTA_FLAG_MAGIC, OTA_PACKAGE_META_VERSION
 * +0x08: virtualSize, 0
 * +0x10: expectedCRC, 0
 * +0x18: totalPayloadSize, segmentCount
 * +0x20: gapFill, 0
 * +0x28: seg0.offset, seg0.size
 * +0x30: seg1.offset, seg1.size
 * +0x38: seg2.offset, seg2.size
 * +0x40: seg3.offset, seg3.size
 */
typedef struct
{
    uint32 offset;
    uint32 size;
} OTA_PackageSegment_t;

typedef struct
{
    uint32 magic;
    uint32 version;
    uint32 virtualSize;
    uint32 expectedCrc32;
    uint32 totalPayloadSize;
    uint32 gapFill;
    uint32 segmentCount;
    OTA_PackageSegment_t segments[OTA_PACKAGE_MAX_SEGMENTS];
} OTA_PackageMetadata_t;

boolean OTA_Flash_SetPackageMetadata(const OTA_PackageMetadata_t *meta);

/* ── 함수 포인터 구조체 ──────────────────────────────────────── */
typedef struct
{
    void   (*eraseSectorCmd)(uint32 sectorAddr);
    uint8  (*waitUnbusy)    (uint32 flash, IfxFlash_FlashType flashType);
    uint8  (*enterPageMode) (uint32 pageAddr);
    void   (*load2X32bits)  (uint32 pageAddr, uint32 wordL, uint32 wordU);
    void   (*writePage)     (uint32 pageAddr);
    uint32 (*eraseWrapper)  (uint32 sectorAddr, uint32 sectorCount, IfxFlash_FlashType flashType);
    void   (*writePageFull) (uint32 pageAddr, uint8 *buf, IfxFlash_FlashType flashType);
} OTA_FlashFunc;

/* ── 외부 인터페이스 ─────────────────────────────────────────── */
boolean OTA_Flash_Erase(uint32 addr, uint32 size, IfxFlash_FlashType flashType);
boolean OTA_Flash_Write(uint32 addr, uint8 *data, uint16 len, IfxFlash_FlashType flashType);
boolean OTA_Flash_VerifyCRC(uint32 addr, uint32 size);
void    OTA_Flash_SetFlag(uint32 fwSize, uint32 expectedCRC);
void    OTA_Flash_ClearFlag(void);

extern volatile uint32 g_TickCount_1ms;

#endif /* APP_FLASH_H */
