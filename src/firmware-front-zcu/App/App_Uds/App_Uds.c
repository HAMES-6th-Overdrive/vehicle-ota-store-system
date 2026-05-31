/**********************************************************************************************************************
 * \file App_Uds.c
 * \brief ZCU Self OTA UDS Handler - Sparse Package Version
 *
 * 목적:
 *  - 기존 연속 bin OTA 방식에서 sparse package OTA 방식으로 변경한다.
 *  - PC/HPC는 B4 manifest + 34 totalPayloadSize + 36 segment stream + 37 virtual CRC를 보낸다.
 *  - ZCU는 manifest의 segment offset/size를 기준으로 inactive bank에 직접 write한다.
 *
 * 패키지 전송 형식:
 *  1) 10 03
 *  2) B4
 *      virtualSize      4B
 *      virtualCRC32     4B
 *      segmentCount     1B
 *      gapFill          1B
 *      repeated:
 *          segmentOffset 4B
 *          segmentSize   4B
 *  3) 34 totalPayloadSize
 *  4) 36 segment1.bin + segment2.bin + ... stream
 *  5) 37 virtualCRC32
 *
 * 주의:
 *  - 이 파일은 App 단계에서 sparse segment를 올바른 주소에 write하는 역할이다.
 *  - Bootloader가 sparse CRC를 검증하려면 DFLASH에 segment metadata 저장 기능이 추가로 필요하다.
 *********************************************************************************************************************/

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "App_Uds.h"
#include "App_Flash.h"
#include "App_Debug/App_Core1Debug.h"
#include "IfxScuRcu.h"
#include <string.h>
#include "lwip/opt.h"
#include "App_Sota.h"
#include "FreeRTOS.h"
#include "task.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

#ifndef UDS_SID_SPARSE_MANIFEST
#define UDS_SID_SPARSE_MANIFEST             0xB4U
#endif

#ifndef UDS_SID_SPARSE_MANIFEST_POS
#define UDS_SID_SPARSE_MANIFEST_POS         0xF4U
#endif

#define ZCU_OTA_MAX_SEGMENTS                4U
#define ZCU_OTA_META_VERSION                1U
#define ZCU_OTA_META_MAGIC                  0xDEADBEEFUL
#define ZCU_OTA_GAP_FILL_DEFAULT            0x00U

/*
 * ZCU App slot 크기.
 * 현재 프로젝트 기준:
 *  Bank A App start: 0x80020000
 *  Bank B App start: 0x80320000
 *  App range size  : 0x002E0000
 */
#define ZCU_OTA_SLOT_A_START_C              0x80020000UL
#define ZCU_OTA_SLOT_B_START_C              0x80320000UL
#define ZCU_OTA_MAX_IMAGE_SIZE              0x002E0000UL

#define ZCU_OTA_FLASH_RANGE_START_NC        0xA0000000UL
#define ZCU_OTA_FLASH_RANGE_END_NC          0xA0600000UL

#define ZCU_OTA_INVALID_SECTOR              0xFFFFFFFFUL

/*
 * 1로 바꾸려면 App_Flash.c / App_Flash.h에
 * OTA_Flash_SetPackageMetadata()를 추가해야 한다.
 *
 * 현재 0이면 기존 OTA_Flash_SetFlag(size, crc)를 사용한다.
 * 단, 이 경우 Bootloader가 sparse segment metadata를 알 수 없으므로
 * 완전한 package boot는 다음 단계에서 metadata 저장으로 교체해야 한다.
 */
#ifndef ZCU_OTA_USE_PACKAGE_METADATA
#define ZCU_OTA_USE_PACKAGE_METADATA        1U
#endif

/*********************************************************************************************************************/
/*------------------------------------------------Private Types------------------------------------------------------*/
/*********************************************************************************************************************/

typedef OTA_PackageSegment_t  ZcuOtaSegment_t;
typedef OTA_PackageMetadata_t ZcuOtaPackageMeta_t;

#if (ZCU_OTA_USE_PACKAGE_METADATA == 1U)
/*
 * App_Flash.c 쪽에 동일한 타입 또는 호환 함수가 필요하다.
 * 구현 전에는 ZCU_OTA_USE_PACKAGE_METADATA를 0으로 둔다.
 */
extern boolean OTA_Flash_SetPackageMetadata(const ZcuOtaPackageMeta_t *meta);
#endif

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

static uint8  g_session      = UDS_SESSION_DEFAULT;
static uint32 g_downloadAddr = 0U;      /* cached inactive bank base */
static uint32 g_downloadSize = 0U;      /* package mode에서는 totalPayloadSize */
static uint32 g_writtenBytes = 0U;      /* package stream 기준 누적 수신 byte */
static uint8  g_blockSeq     = 0x01U;

/*
 * 기존 연속 erase 방식 호환용.
 * package mode에서는 g_lastErasedSector 기준으로 필요한 sector만 erase한다.
 */
static uint32 g_erasedUntil       = 0U;
static uint32 g_lastErasedSector  = ZCU_OTA_INVALID_SECTOR;

static ZcuOtaPackageMeta_t g_pkgMeta;
static boolean g_pkgMetaValid = FALSE;
static boolean g_packageMode  = FALSE;

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

static uint16 UDS_NegResponse       (uint8 *tx, uint8 sid, uint8 nrc);
static uint16 UDS_SessionControl    (uint8 *rx, uint16 rxLen, uint8 *tx);
static uint16 UDS_SparseManifest    (uint8 *rx, uint16 rxLen, uint8 *tx);
static uint16 UDS_RequestDownload   (uint8 *rx, uint16 rxLen, uint8 *tx);
static uint16 UDS_TransferData      (uint8 *rx, uint16 rxLen, uint8 *tx);
static uint16 UDS_TransferExit      (uint8 *rx, uint16 rxLen, uint8 *tx);

static void   UDS_ResetDownloadState(boolean resetPackage);
static uint32 UDS_ReadU32BE(const uint8 *p);
static IfxFlash_FlashType UDS_GetFlashTypeByAddr(uint32 addr);

static boolean ZCUOTA_ValidatePackageMeta(const ZcuOtaPackageMeta_t *meta,
                                          uint32 inactiveBaseCached);

static boolean ZCUOTA_GetWriteAddressFromPayloadOffset(uint32 payloadOffset,
                                                       uint32 *writeAddrCached,
                                                       uint32 *segmentRemain);

static boolean ZCUOTA_EraseRangeIfNeeded(uint32 writeStartNc,
                                         uint32 writeEndNc);

static boolean ZCUOTA_WriteChunk(uint32 writeAddrCached,
                                 const uint8 *chunk,
                                 uint16 chunkLen);

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/

void UDS_HandleService(uint8  *rxData, uint16  rxLen,
                       uint8  *txData, uint16 *txLen)
{
    if (rxLen == 0U)
    {
        *txLen = 0U;
        return;
    }

    uint8 sid = rxData[0];

    switch (sid)
    {
        case UDS_SID_SESSION_CONTROL:
            *txLen = UDS_SessionControl(rxData, rxLen, txData);
            break;

        case UDS_SID_SPARSE_MANIFEST:
            *txLen = UDS_SparseManifest(rxData, rxLen, txData);
            break;

        case UDS_SID_REQUEST_DOWNLOAD:
            *txLen = UDS_RequestDownload(rxData, rxLen, txData);
            break;

        case UDS_SID_TRANSFER_DATA:
            *txLen = UDS_TransferData(rxData, rxLen, txData);
            break;

        case UDS_SID_TRANSFER_EXIT:
            *txLen = UDS_TransferExit(rxData, rxLen, txData);
            break;

        default:
            *txLen = UDS_NegResponse(txData, sid,
                                     UDS_NRC_SERVICE_NOT_SUPPORTED);
            break;
    }
}

/* ── 공통 상태 초기화 ───────────────────────────────────────── */
static void UDS_ResetDownloadState(boolean resetPackage)
{
    g_downloadAddr = 0U;
    g_downloadSize = 0U;
    g_writtenBytes = 0U;
    g_blockSeq = 0x01U;
    g_erasedUntil = 0U;
    g_lastErasedSector = ZCU_OTA_INVALID_SECTOR;

    if (resetPackage == TRUE)
    {
        memset(&g_pkgMeta, 0, sizeof(g_pkgMeta));
        g_pkgMetaValid = FALSE;
        g_packageMode = FALSE;
    }
}

/* ── BE 32-bit 읽기 ─────────────────────────────────────────── */
static uint32 UDS_ReadU32BE(const uint8 *p)
{
    return ((uint32)p[0] << 24) |
           ((uint32)p[1] << 16) |
           ((uint32)p[2] <<  8) |
           ((uint32)p[3]);
}

/* ── Flash type 판단 ───────────────────────────────────────── */
static IfxFlash_FlashType UDS_GetFlashTypeByAddr(uint32 addr)
{
    uint32 a = TO_FLASH_ADDR(addr);

    if ((a >= 0xA0000000UL) && (a < 0xA0300000UL))
    {
        return IfxFlash_FlashType_P0;
    }

    if ((a >= 0xA0300000UL) && (a < 0xA0600000UL))
    {
        return IfxFlash_FlashType_P1;
    }

    /*
     * TC375 6MB SOTA 영역 밖이면 호출부에서 먼저 차단해야 한다.
     */
    return IfxFlash_FlashType_P0;
}

/* ── 부정 응답 생성 ─────────────────────────────────────────── */
static uint16 UDS_NegResponse(uint8 *tx, uint8 sid, uint8 nrc)
{
    tx[0] = UDS_NEGATIVE_RSP;
    tx[1] = sid;
    tx[2] = nrc;
    return 3U;
}

/* ── 0x10 DiagnosticSessionControl ─────────────────────────── */
static uint16 UDS_SessionControl(uint8 *rx, uint16 rxLen, uint8 *tx)
{
    if (rxLen < 2U)
    {
        return UDS_NegResponse(tx, UDS_SID_SESSION_CONTROL,
                               UDS_NRC_GENERAL_REJECT);
    }

    uint8 reqSession = rx[1];

    if ((reqSession == UDS_SESSION_DEFAULT) ||
        (reqSession == UDS_SESSION_EXTENDED))
    {
        g_session = reqSession;

        /*
         * 새 OTA 시퀀스 시작 전 이전 context를 정리한다.
         */
        UDS_ResetDownloadState(TRUE);

        tx[0] = UDS_SID_SESSION_CONTROL + UDS_POS_RESPONSE_OFFSET;
        tx[1] = reqSession;
        tx[2] = 0x00U; tx[3] = 0x19U;    /* P2 = 25ms */
        tx[4] = 0x01U; tx[5] = 0xF4U;    /* P2* = 500ms */
        return 6U;
    }

    return UDS_NegResponse(tx, UDS_SID_SESSION_CONTROL,
                           UDS_NRC_SUBFUNC_NOT_SUPPORTED);
}

/* ── 0xB4 Sparse Manifest ─────────────────────────────────────
 *
 * 요청:
 *   [0]    B4
 *   [1:4]  virtualSize
 *   [5:8]  virtualCRC32
 *   [9]    segmentCount
 *   [10]   gapFill
 *   반복:
 *     segmentOffset 4B
 *     segmentSize   4B
 *
 * 응답:
 *   F4 segmentCount totalPayloadSize(4B)
 */
static uint16 UDS_SparseManifest(uint8 *rx, uint16 rxLen, uint8 *tx)
{
    ZcuOtaPackageMeta_t meta;
    uint32 totalPayload = 0U;
    uint32 requiredLen;
    uint32 i;

    if (g_session != UDS_SESSION_EXTENDED)
    {
        return UDS_NegResponse(tx, UDS_SID_SPARSE_MANIFEST,
                               UDS_NRC_CONDITIONS_NOT_CORRECT);
    }

    if (rxLen < 11U)
    {
        return UDS_NegResponse(tx, UDS_SID_SPARSE_MANIFEST,
                               UDS_NRC_GENERAL_REJECT);
    }

    memset(&meta, 0, sizeof(meta));

    meta.magic = ZCU_OTA_META_MAGIC;
    meta.version = ZCU_OTA_META_VERSION;
    meta.virtualSize = UDS_ReadU32BE(&rx[1]);
    meta.expectedCrc32 = UDS_ReadU32BE(&rx[5]);
    meta.segmentCount = (uint32)rx[9];
    meta.gapFill = (uint32)rx[10];

    if ((meta.segmentCount == 0U) ||
        (meta.segmentCount > ZCU_OTA_MAX_SEGMENTS))
    {
        return UDS_NegResponse(tx, UDS_SID_SPARSE_MANIFEST,
                               UDS_NRC_REQUEST_OUT_OF_RANGE);
    }

    requiredLen = 11U + (meta.segmentCount * 8U);
    if (rxLen < requiredLen)
    {
        return UDS_NegResponse(tx, UDS_SID_SPARSE_MANIFEST,
                               UDS_NRC_GENERAL_REJECT);
    }

    for (i = 0U; i < meta.segmentCount; i++)
    {
        uint32 base = 11U + (i * 8U);
        uint32 segOffset;
        uint32 segSize;
        uint32 segEnd;

        segOffset = UDS_ReadU32BE(&rx[base]);
        segSize   = UDS_ReadU32BE(&rx[base + 4U]);
        segEnd    = segOffset + segSize;

        if ((segSize == 0U) ||
            (segEnd < segOffset) ||
            (segEnd > meta.virtualSize))
        {
            return UDS_NegResponse(tx, UDS_SID_SPARSE_MANIFEST,
                                   UDS_NRC_REQUEST_OUT_OF_RANGE);
        }

        if (i > 0U)
        {
            uint32 prevEnd;

            prevEnd = meta.segments[i - 1U].offset +
                      meta.segments[i - 1U].size;

            if (segOffset < prevEnd)
            {
                return UDS_NegResponse(tx, UDS_SID_SPARSE_MANIFEST,
                                       UDS_NRC_REQUEST_OUT_OF_RANGE);
            }
        }

        if ((totalPayload + segSize) < totalPayload)
        {
            return UDS_NegResponse(tx, UDS_SID_SPARSE_MANIFEST,
                                   UDS_NRC_REQUEST_OUT_OF_RANGE);
        }

        meta.segments[i].offset = segOffset;
        meta.segments[i].size = segSize;
        totalPayload += segSize;
    }

    if ((meta.virtualSize == 0U) ||
        (meta.virtualSize > ZCU_OTA_MAX_IMAGE_SIZE) ||
        (totalPayload == 0U) ||
        (totalPayload > meta.virtualSize))
    {
        return UDS_NegResponse(tx, UDS_SID_SPARSE_MANIFEST,
                               UDS_NRC_REQUEST_OUT_OF_RANGE);
    }

    if (meta.gapFill > 0xFFU)
    {
        return UDS_NegResponse(tx, UDS_SID_SPARSE_MANIFEST,
                               UDS_NRC_REQUEST_OUT_OF_RANGE);
    }

    meta.totalPayloadSize = totalPayload;

    /*
     * 새 package manifest 수신 시 download 상태만 초기화하고,
     * session은 유지한다.
     */
    UDS_ResetDownloadState(FALSE);

    memcpy(&g_pkgMeta, &meta, sizeof(g_pkgMeta));
    g_pkgMetaValid = TRUE;
    g_packageMode = TRUE;

    AppCore1Debug_Push("UDS_SparseManifest() OK.");

    tx[0] = UDS_SID_SPARSE_MANIFEST_POS;
    tx[1] = (uint8)g_pkgMeta.segmentCount;
    tx[2] = (uint8)((g_pkgMeta.totalPayloadSize >> 24) & 0xFFU);
    tx[3] = (uint8)((g_pkgMeta.totalPayloadSize >> 16) & 0xFFU);
    tx[4] = (uint8)((g_pkgMeta.totalPayloadSize >>  8) & 0xFFU);
    tx[5] = (uint8)( g_pkgMeta.totalPayloadSize        & 0xFFU);

    return 6U;
}

/* ── package manifest 범위 검증 ─────────────────────────────── */
static boolean ZCUOTA_ValidatePackageMeta(const ZcuOtaPackageMeta_t *meta,
                                          uint32 inactiveBaseCached)
{
    uint32 i;

    if (meta == NULL_PTR)
    {
        return FALSE;
    }

    if ((meta->magic != ZCU_OTA_META_MAGIC) ||
        (meta->version != ZCU_OTA_META_VERSION))
    {
        return FALSE;
    }

    if ((meta->virtualSize == 0U) ||
        (meta->virtualSize > ZCU_OTA_MAX_IMAGE_SIZE))
    {
        return FALSE;
    }

    if ((meta->segmentCount == 0U) ||
        (meta->segmentCount > ZCU_OTA_MAX_SEGMENTS))
    {
        return FALSE;
    }

    for (i = 0U; i < meta->segmentCount; i++)
    {
        uint32 segOffset = meta->segments[i].offset;
        uint32 segSize = meta->segments[i].size;
        uint32 segEnd = segOffset + segSize;
        uint32 startNc;
        uint32 endNc;

        if ((segSize == 0U) ||
            (segEnd < segOffset) ||
            (segEnd > meta->virtualSize))
        {
            return FALSE;
        }

        startNc = TO_FLASH_ADDR(inactiveBaseCached + segOffset);
        endNc   = TO_FLASH_ADDR(inactiveBaseCached + segEnd);

        if ((startNc < ZCU_OTA_FLASH_RANGE_START_NC) ||
            (endNc > ZCU_OTA_FLASH_RANGE_END_NC) ||
            (endNc <= startNc))
        {
            return FALSE;
        }
    }

    return TRUE;
}

/* ── 0x34 RequestDownload ───────────────────────────────────── */
static uint16 UDS_RequestDownload(uint8 *rx, uint16 rxLen, uint8 *tx)
{
    uint8 addrLen;
    uint8 sizeLen;
    uint32 flashStart;
    uint32 flashEnd;

    if (g_session != UDS_SESSION_EXTENDED)
    {
        return UDS_NegResponse(tx, UDS_SID_REQUEST_DOWNLOAD,
                               UDS_NRC_CONDITIONS_NOT_CORRECT);
    }

    if (rxLen < 4U)
    {
        return UDS_NegResponse(tx, UDS_SID_REQUEST_DOWNLOAD,
                               UDS_NRC_GENERAL_REJECT);
    }

    AppCore1Debug_Push("UDS_RequestDownload() called.");

    addrLen = (rx[2] >> 0) & 0x0FU;
    sizeLen = (rx[2] >> 4) & 0x0FU;

    if (rxLen < (uint16)(3U + addrLen + sizeLen))
    {
        return UDS_NegResponse(tx, UDS_SID_REQUEST_DOWNLOAD,
                               UDS_NRC_GENERAL_REJECT);
    }

    /*
     * 현재 active bank의 반대편 inactive bank를 target으로 선택한다.
     */
    if (SOTA_IsGroupBActive())
    {
        g_downloadAddr = ZCU_OTA_SLOT_A_START_C;
    }
    else
    {
        g_downloadAddr = ZCU_OTA_SLOT_B_START_C;
    }

    /*
     * memorySize 파싱.
     * package mode에서는 이 값이 virtualSize가 아니라
     * 실제 전송 payload size, 즉 segment size 합이어야 한다.
     */
    g_downloadSize = 0U;
    for (uint8 i = 0U; i < sizeLen; i++)
    {
        g_downloadSize = (g_downloadSize << 8) |
                         (uint32)rx[3U + addrLen + i];
    }

    g_writtenBytes = 0U;
    g_blockSeq = 0x01U;
    g_lastErasedSector = ZCU_OTA_INVALID_SECTOR;

    if (g_pkgMetaValid == TRUE)
    {
        g_packageMode = TRUE;

        if (g_downloadSize != g_pkgMeta.totalPayloadSize)
        {
            return UDS_NegResponse(tx, UDS_SID_REQUEST_DOWNLOAD,
                                   UDS_NRC_REQUEST_OUT_OF_RANGE);
        }

        if (ZCUOTA_ValidatePackageMeta(&g_pkgMeta, g_downloadAddr) == FALSE)
        {
            return UDS_NegResponse(tx, UDS_SID_REQUEST_DOWNLOAD,
                                   UDS_NRC_REQUEST_OUT_OF_RANGE);
        }

        flashStart = TO_FLASH_ADDR(g_downloadAddr);
        flashEnd   = TO_FLASH_ADDR(g_downloadAddr + g_pkgMeta.virtualSize);
    }
    else
    {
        /*
         * B4 없이 들어오면 기존 legacy 연속 이미지 OTA로 동작시킨다.
         * 나중에 package only로 강제하고 싶으면 여기서 NRC를 반환하면 된다.
         */
        g_packageMode = FALSE;

        flashStart = TO_FLASH_ADDR(g_downloadAddr);
        flashEnd   = flashStart + g_downloadSize;
    }

    if ((g_downloadSize == 0U) ||
        (flashStart < ZCU_OTA_FLASH_RANGE_START_NC) ||
        (flashEnd > ZCU_OTA_FLASH_RANGE_END_NC) ||
        (flashEnd <= flashStart))
    {
        return UDS_NegResponse(tx, UDS_SID_REQUEST_DOWNLOAD,
                               UDS_NRC_REQUEST_OUT_OF_RANGE);
    }

    g_erasedUntil = flashStart & ~(PFLASH_SECTOR_SIZE - 1U);

    /*
     * 긍정 응답.
     * 0x0202 = SID 1 + BSC 1 + data 512
     */
    tx[0] = UDS_SID_REQUEST_DOWNLOAD + UDS_POS_RESPONSE_OFFSET;
    tx[1] = 0x20U;
    tx[2] = 0x02U;
    tx[3] = 0x02U;

    return 4U;
}

/* ── payload offset → segment 실제 write 주소 매핑 ─────────── */
static boolean ZCUOTA_GetWriteAddressFromPayloadOffset(uint32 payloadOffset,
                                                       uint32 *writeAddrCached,
                                                       uint32 *segmentRemain)
{
    uint32 acc = 0U;
    uint32 i;

    if ((writeAddrCached == NULL_PTR) ||
        (segmentRemain == NULL_PTR) ||
        (g_pkgMetaValid == FALSE))
    {
        return FALSE;
    }

    for (i = 0U; i < g_pkgMeta.segmentCount; i++)
    {
        uint32 segSize = g_pkgMeta.segments[i].size;

        if (payloadOffset < (acc + segSize))
        {
            uint32 localOffset;

            localOffset = payloadOffset - acc;

            *writeAddrCached = g_downloadAddr +
                               g_pkgMeta.segments[i].offset +
                               localOffset;

            *segmentRemain = segSize - localOffset;
            return TRUE;
        }

        acc += segSize;
    }

    return FALSE;
}

/* ── 필요한 sector만 erase ─────────────────────────────────── */
static boolean ZCUOTA_EraseRangeIfNeeded(uint32 writeStartNc,
                                         uint32 writeEndNc)
{
    uint32 sector;

    if ((writeStartNc < ZCU_OTA_FLASH_RANGE_START_NC) ||
        (writeEndNc > ZCU_OTA_FLASH_RANGE_END_NC) ||
        (writeEndNc <= writeStartNc))
    {
        return FALSE;
    }

    sector = writeStartNc & ~(PFLASH_SECTOR_SIZE - 1U);

    while (sector < writeEndNc)
    {
        if (sector != g_lastErasedSector)
        {
            IfxFlash_FlashType eraseType;

            eraseType = UDS_GetFlashTypeByAddr(sector);

            if (OTA_Flash_Erase(sector, PFLASH_SECTOR_SIZE, eraseType) == FALSE)
            {
                return FALSE;
            }

            g_lastErasedSector = sector;
            g_erasedUntil = sector + PFLASH_SECTOR_SIZE;
        }

        sector += PFLASH_SECTOR_SIZE;
    }

    return TRUE;
}

/* ── 지정 주소에 chunk write ───────────────────────────────── */
static boolean ZCUOTA_WriteChunk(uint32 writeAddrCached,
                                 const uint8 *chunk,
                                 uint16 chunkLen)
{
    uint32 writeStart;
    uint32 writeEnd;
    IfxFlash_FlashType flashType;

    if ((chunk == NULL_PTR) || (chunkLen == 0U))
    {
        return FALSE;
    }

    writeStart = TO_FLASH_ADDR(writeAddrCached);
    writeEnd = writeStart + chunkLen;

    if (ZCUOTA_EraseRangeIfNeeded(writeStart, writeEnd) == FALSE)
    {
        return FALSE;
    }

    flashType = UDS_GetFlashTypeByAddr(writeAddrCached);

    if (OTA_Flash_Write(writeAddrCached, chunk, chunkLen, flashType) == FALSE)
    {
        return FALSE;
    }

    return TRUE;
}

/* ── 0x36 TransferData ─────────────────────────────────────── */
static uint16 UDS_TransferData(uint8 *rx, uint16 rxLen, uint8 *tx)
{
    uint8 seqCounter;
    uint8 *chunk;
    uint16 chunkLen;

    if (g_downloadSize == 0U)
    {
        return UDS_NegResponse(tx, UDS_SID_TRANSFER_DATA,
                               UDS_NRC_CONDITIONS_NOT_CORRECT);
    }

    if (rxLen < 2U)
    {
        return UDS_NegResponse(tx, UDS_SID_TRANSFER_DATA,
                               UDS_NRC_GENERAL_REJECT);
    }

    seqCounter = rx[1];
    if (seqCounter != g_blockSeq)
    {
        return UDS_NegResponse(tx, UDS_SID_TRANSFER_DATA,
                               UDS_NRC_WRONG_BLOCK_SEQ_COUNTER);
    }

    chunk = rx + 2U;
    chunkLen = rxLen - 2U;

    if ((chunkLen == 0U) || (chunkLen > MAX_BLOCK_SIZE))
    {
        return UDS_NegResponse(tx, UDS_SID_TRANSFER_DATA,
                               UDS_NRC_REQUEST_OUT_OF_RANGE);
    }

    if ((g_writtenBytes + chunkLen) < g_writtenBytes)
    {
        return UDS_NegResponse(tx, UDS_SID_TRANSFER_DATA,
                               UDS_NRC_REQUEST_OUT_OF_RANGE);
    }

    if ((g_writtenBytes + chunkLen) > g_downloadSize)
    {
        return UDS_NegResponse(tx, UDS_SID_TRANSFER_DATA,
                               UDS_NRC_REQUEST_OUT_OF_RANGE);
    }

    if (g_packageMode == TRUE)
    {
        uint16 copied = 0U;

        while (copied < chunkLen)
        {
            uint32 writeAddrCached;
            uint32 segmentRemain;
            uint16 writeLen;

            if (ZCUOTA_GetWriteAddressFromPayloadOffset(g_writtenBytes,
                                                        &writeAddrCached,
                                                        &segmentRemain) == FALSE)
            {
                return UDS_NegResponse(tx, UDS_SID_TRANSFER_DATA,
                                       UDS_NRC_REQUEST_OUT_OF_RANGE);
            }

            writeLen = (uint16)(chunkLen - copied);
            if (writeLen > segmentRemain)
            {
                writeLen = (uint16)segmentRemain;
            }

            if (ZCUOTA_WriteChunk(writeAddrCached,
                                  &chunk[copied],
                                  writeLen) == FALSE)
            {
                return UDS_NegResponse(tx, UDS_SID_TRANSFER_DATA,
                                       UDS_NRC_GENERAL_PROG_FAILURE);
            }

            g_writtenBytes += writeLen;
            copied = (uint16)(copied + writeLen);
        }
    }
    else
    {
        /*
         * Legacy 연속 이미지 mode.
         */
        uint32 curAddr;

        curAddr = g_downloadAddr + g_writtenBytes;

        if (ZCUOTA_WriteChunk(curAddr, chunk, chunkLen) == FALSE)
        {
            return UDS_NegResponse(tx, UDS_SID_TRANSFER_DATA,
                                   UDS_NRC_GENERAL_PROG_FAILURE);
        }

        g_writtenBytes += chunkLen;
    }

    g_blockSeq = (g_blockSeq >= 0xFFU) ? 0x00U : (uint8)(g_blockSeq + 1U);

    tx[0] = UDS_SID_TRANSFER_DATA + UDS_POS_RESPONSE_OFFSET;
    tx[1] = seqCounter;

    return 2U;
}

/* ── 0x37 RequestTransferExit ───────────────────────────────── */
static uint16 UDS_TransferExit(uint8 *rx, uint16 rxLen, uint8 *tx)
{
    uint32 expectedCRC;

    if (g_writtenBytes == 0U)
    {
        return UDS_NegResponse(tx, UDS_SID_TRANSFER_EXIT,
                               UDS_NRC_CONDITIONS_NOT_CORRECT);
    }

    if (rxLen < 5U)
    {
        return UDS_NegResponse(tx, UDS_SID_TRANSFER_EXIT,
                               UDS_NRC_REQUEST_OUT_OF_RANGE);
    }

    if (g_writtenBytes != g_downloadSize)
    {
        return UDS_NegResponse(tx, UDS_SID_TRANSFER_EXIT,
                               UDS_NRC_CONDITIONS_NOT_CORRECT);
    }

    AppCore1Debug_Push("UDS_TransferExit() called.");

    expectedCRC = UDS_ReadU32BE(&rx[1]);

    if (g_packageMode == TRUE)
    {
        if (g_pkgMetaValid == FALSE)
        {
            return UDS_NegResponse(tx, UDS_SID_TRANSFER_EXIT,
                                   UDS_NRC_CONDITIONS_NOT_CORRECT);
        }

        if (expectedCRC != g_pkgMeta.expectedCrc32)
        {
            return UDS_NegResponse(tx, UDS_SID_TRANSFER_EXIT,
                                   UDS_NRC_REQUEST_OUT_OF_RANGE);
        }

#if (ZCU_OTA_USE_PACKAGE_METADATA == 1U)
        /*
         * 최종 package bootloader용.
         * App_Flash.c에 OTA_Flash_SetPackageMetadata() 구현 필요.
         */
        if (OTA_Flash_SetPackageMetadata(&g_pkgMeta) == FALSE)
        {
            return UDS_NegResponse(tx, UDS_SID_TRANSFER_EXIT,
                                   UDS_NRC_GENERAL_PROG_FAILURE);
        }
#else
        /*
         * 1차 적용용 fallback.
         * segment는 올바른 주소에 써지지만, bootloader가 sparse gap을 알 수 없으므로
         * 최종 SOTA 전환에는 package metadata 저장 함수로 교체해야 한다.
         */
        OTA_Flash_SetFlag(g_pkgMeta.virtualSize, expectedCRC);
#endif
    }
    else
    {
        /*
         * Legacy 연속 이미지 mode.
         */
        OTA_Flash_SetFlag(g_writtenBytes, expectedCRC);
    }

    tx[0] = UDS_SID_TRANSFER_EXIT + UDS_POS_RESPONSE_OFFSET;

    g_session = UDS_SESSION_DEFAULT;
    UDS_ResetDownloadState(TRUE);

    return 1U;
}

/*********************************************************************************************************************/
