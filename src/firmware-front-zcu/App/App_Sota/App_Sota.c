#include "App_Sota.h"
#include "IfxFlash.h"
#include "IfxScuWdt.h"
#include "IfxScuRcu.h"
#include "IfxCpu.h"

/*************************************************************/
/* DFLASH 8바이트 쓰기 (UCB / 일반 DFLASH 공용)             */
/*************************************************************/
void WriteDFlash8(uint32 addr, uint32 lo, uint32 hi)
{
    uint16 pw = IfxScuWdt_getSafetyWatchdogPassword();

    IfxFlash_enterPageMode(addr);
    IfxFlash_waitUnbusy(0, IfxFlash_FlashType_D0);

    IfxFlash_loadPage2X32(addr, lo, hi);

    IfxScuWdt_clearSafetyEndinit(pw);
    IfxFlash_writePage(addr);
    IfxScuWdt_setSafetyEndinit(pw);

    IfxFlash_waitUnbusy(0, IfxFlash_FlashType_D0);
}

/*************************************************************/
/* 내부 헬퍼                                                  */
/*************************************************************/

static void SOTA_UcbFailStop(void)
{
    /* UCB 조작 실패 후 reset을 걸면 더 위험하므로 여기서 정지한다.
     * 디버거로 원인을 확인하거나 HW reset/Bootstrap recovery를 수행한다.
     */
    while (1)
    {
        __nop();
    }
}

static boolean SOTA_IsValidTargetMarker(uint32 markerVal)
{
    return ((markerVal == SWAP_MARKER_A) || (markerVal == SWAP_MARKER_B)) ? TRUE : FALSE;
}

static boolean VerifyUcbBlockUnlocked(uint32 base)
{
    uint32 lo = *(volatile uint32 *)(base + UCB_BLOCK_CONFIRM_OFFSET);
    uint32 hi = *(volatile uint32 *)(base + UCB_BLOCK_CONFIRM_OFFSET + 4U);

    return ((lo == UCB_STATE_UNLOCKED_CODE) && (hi == 0x00000000UL)) ? TRUE : FALSE;
}

static boolean IsUcbBlockConfirmErased(uint32 base)
{
    uint32 lo = *(volatile uint32 *)(base + UCB_BLOCK_CONFIRM_OFFSET);
    uint32 hi = *(volatile uint32 *)(base + UCB_BLOCK_CONFIRM_OFFSET + 4U);

    return ((lo == 0x00000000UL) && (hi == 0x00000000UL)) ? TRUE : FALSE;
}

static void WriteUcbBlockUnlocked(uint32 base)
{
    WriteDFlash8(base + UCB_BLOCK_CONFIRM_OFFSET,
                 UCB_STATE_UNLOCKED_CODE,
                 0x00000000UL);
}

static void EnsureUcbBlockUnlocked(uint32 base)
{
    if (VerifyUcbBlockUnlocked(base))
    {
        return;
    }

    if (IsUcbBlockConfirmErased(base))
    {
        WriteUcbBlockUnlocked(base);
        if (VerifyUcbBlockUnlocked(base))
        {
            return;
        }
    }

    SOTA_UcbFailStop();
}

static boolean VerifyUcbSwapErased(uint32 base)
{
    for (uint32 off = 0U; off < UCB_SWAP_SIZE; off += 4U)
    {
        if (*(volatile uint32 *)(base + off) != 0x00000000UL)
        {
            return FALSE;
        }
    }

    return TRUE;
}

static void EraseUcbSwapSector(uint32 base)
{
    uint16 pw = IfxScuWdt_getSafetyWatchdogPassword();

    IfxScuWdt_clearSafetyEndinit(pw);
    IfxFlash_eraseMultipleSectors(base, 1);
    IfxScuWdt_setSafetyEndinit(pw);

    IfxFlash_waitUnbusy(0, IfxFlash_FlashType_D0);
}

static void WriteSwapEntryOneBase(uint32 base, uint32 entryIdx, uint32 markerVal)
{
    uint32 markerLAddr  = base + entryIdx * UCB_SWAP_ENTRY_SIZE;
    uint32 confirmLAddr = markerLAddr + UCB_SWAP_ENTRY_CONFIRM_OFFSET;

    WriteDFlash8(markerLAddr,  markerVal,        markerLAddr);
    WriteDFlash8(confirmLAddr, UCB_CONFIRM_CODE, confirmLAddr);
}

static boolean VerifySwapEntryOneBase(uint32 base, uint32 entryIdx, uint32 markerVal)
{
    uint32 markerLAddr  = base + entryIdx * UCB_SWAP_ENTRY_SIZE;
    uint32 confirmLAddr = markerLAddr + UCB_SWAP_ENTRY_CONFIRM_OFFSET;

    uint32 markerL  = *(volatile uint32 *)(markerLAddr + 0U);
    uint32 markerH  = *(volatile uint32 *)(markerLAddr + 4U);
    uint32 confirmL = *(volatile uint32 *)(confirmLAddr + 0U);
    uint32 confirmH = *(volatile uint32 *)(confirmLAddr + 4U);

    if ((markerL  != markerVal)        ||
        (markerH  != markerLAddr)      ||
        (confirmL != UCB_CONFIRM_CODE) ||
        (confirmH != confirmLAddr))
    {
        return FALSE;
    }

    return TRUE;
}

static sint8 FindCurrentSwapEntryInBase(uint32 base)
{
    if (!VerifyUcbBlockUnlocked(base))
    {
        return -1;
    }

    for (sint8 x = 15; x >= 0; x--)
    {
        uint32 markerLAddr  = base + (uint32)x * UCB_SWAP_ENTRY_SIZE;
        uint32 confirmLAddr = markerLAddr + UCB_SWAP_ENTRY_CONFIRM_OFFSET;

        uint32 markerL  = *(volatile uint32 *)(markerLAddr + 0U);
        uint32 markerH  = *(volatile uint32 *)(markerLAddr + 4U);
        uint32 confirmL = *(volatile uint32 *)(confirmLAddr + 0U);
        uint32 confirmH = *(volatile uint32 *)(confirmLAddr + 4U);

        if (((markerL == SWAP_MARKER_A) || (markerL == SWAP_MARKER_B)) &&
            (markerH == markerLAddr) &&
            (confirmL == UCB_CONFIRM_CODE) &&
            (confirmH == confirmLAddr))
        {
            return x;
        }
    }

    return -1;
}

/* ORIG/COPY 둘 다 확인. 둘 다 유효하면 index와 marker가 일치해야 한다. */
static boolean ReadCurrentSwapMarker(uint32 *marker)
{
    sint8  curOrig;
    sint8  curCopy;
    uint32 markerOrig = 0U;
    uint32 markerCopy = 0U;

    curOrig = FindCurrentSwapEntryInBase(UCB_SWAP_ORIG);
    curCopy = FindCurrentSwapEntryInBase(UCB_SWAP_COPY);

    if (curOrig >= 0)
    {
        markerOrig = *(volatile uint32 *)(UCB_SWAP_ORIG + (uint32)curOrig * UCB_SWAP_ENTRY_SIZE);
    }

    if (curCopy >= 0)
    {
        markerCopy = *(volatile uint32 *)(UCB_SWAP_COPY + (uint32)curCopy * UCB_SWAP_ENTRY_SIZE);
    }

    if ((curOrig >= 0) && (curCopy >= 0))
    {
        if ((curOrig != curCopy) || (markerOrig != markerCopy))
        {
            SOTA_UcbFailStop();
        }

        *marker = markerOrig;
        return TRUE;
    }

    if (curOrig >= 0)
    {
        *marker = markerOrig;
        return TRUE;
    }

    if (curCopy >= 0)
    {
        *marker = markerCopy;
        return TRUE;
    }

    return FALSE;
}

static sint8 FindCurrentSwapEntry(void)
{
    uint32 marker;
    sint8  curOrig = FindCurrentSwapEntryInBase(UCB_SWAP_ORIG);
    sint8  curCopy = FindCurrentSwapEntryInBase(UCB_SWAP_COPY);

    if ((curOrig >= 0) && (curCopy >= 0) && (curOrig != curCopy))
    {
        SOTA_UcbFailStop();
    }

    if (!ReadCurrentSwapMarker(&marker))
    {
        return -1;
    }

    return (curOrig >= 0) ? curOrig : curCopy;
}

/* Entry 쓰기 (ORIG + COPY 동시) — 일반 append 용도 */
static void WriteSwapEntry(uint32 entryIdx, uint32 markerVal)
{
    if (!SOTA_IsValidTargetMarker(markerVal))
    {
        SOTA_UcbFailStop();
    }

    EnsureUcbBlockUnlocked(UCB_SWAP_ORIG);    

    WriteSwapEntryOneBase(UCB_SWAP_ORIG, entryIdx, markerVal);
    if (!VerifySwapEntryOneBase(UCB_SWAP_ORIG, entryIdx, markerVal))
    {
        SOTA_UcbFailStop();
    }

    EnsureUcbBlockUnlocked(UCB_SWAP_COPY);

    WriteSwapEntryOneBase(UCB_SWAP_COPY, entryIdx, markerVal);
    if (!VerifySwapEntryOneBase(UCB_SWAP_COPY, entryIdx, markerVal))
    {
        SOTA_UcbFailStop();
    }

    /* 쓰기 직후 readback — 기존 진단 영역 유지 */
    WriteDFlash8(0xAF003000UL,
                 *(volatile uint32 *)(UCB_SWAP_ORIG + entryIdx * UCB_SWAP_ENTRY_SIZE),
                 *(volatile uint32 *)(UCB_SWAP_ORIG + entryIdx * UCB_SWAP_ENTRY_SIZE + UCB_SWAP_ENTRY_CONFIRM_OFFSET));

    WriteDFlash8(0xAF003010UL,
                 *(volatile uint32 *)(UCB_SWAP_COPY + entryIdx * UCB_SWAP_ENTRY_SIZE),
                 *(volatile uint32 *)(UCB_SWAP_COPY + entryIdx * UCB_SWAP_ENTRY_SIZE + UCB_SWAP_ENTRY_CONFIRM_OFFSET));
}

static void InvalidateSwapEntry(uint32 x)
{
    WriteDFlash8(UCB_SWAP_ORIG + x * UCB_SWAP_ENTRY_SIZE + UCB_SWAP_ENTRY_CONFIRM_OFFSET,
                 0xFFFFFFFFUL,
                 0xFFFFFFFFUL);

    WriteDFlash8(UCB_SWAP_COPY + x * UCB_SWAP_ENTRY_SIZE + UCB_SWAP_ENTRY_CONFIRM_OFFSET,
                 0xFFFFFFFFUL,
                 0xFFFFFFFFUL);
}

static uint32 SOTA_GetSwapCfg(void)
{
    uint32 stmem1 = *(volatile uint32 *)SCU_STMEM1_ADDR;
    return (stmem1 & SCU_STMEM1_SWAP_CFG_MASK) >> SCU_STMEM1_SWAP_CFG_POS;
}

static uint32 SOTA_GetSwapen(void)
{
    uint32 procontp = *(volatile uint32 *)SOTA_PROCONTP_ADDR;
    return (procontp & SOTA_PROCONTP_SWAPEN_MASK) >> SOTA_PROCONTP_SWAPEN_POS;
}

static void SOTA_EraseAndResetSwap(uint32 targetMarker)  /* SWAP_MARKER_A or SWAP_MARKER_B */
{
    boolean irq;

    if (!SOTA_IsValidTargetMarker(targetMarker))
    {
        SOTA_UcbFailStop();
    }

    irq = IfxCpu_disableInterrupts();

    /* 1) COPY 먼저 재작성한다.
     *    이 단계 중 문제가 생겨도 ORIG의 기존 entry가 아직 fallback으로 남는다.
     */
    EraseUcbSwapSector(UCB_SWAP_COPY);
    if (!VerifyUcbSwapErased(UCB_SWAP_COPY))
    {
        SOTA_UcbFailStop();
    }

    WriteSwapEntryOneBase(UCB_SWAP_COPY, 0U, targetMarker);
    WriteUcbBlockUnlocked(UCB_SWAP_COPY);       /* UCB_SWAP_COPY + 0x1F0 = 0x43211234 */

    if (!VerifySwapEntryOneBase(UCB_SWAP_COPY, 0U, targetMarker) ||
        !VerifyUcbBlockUnlocked(UCB_SWAP_COPY))
    {
        SOTA_UcbFailStop();
    }

    /* 2) COPY가 유효해진 뒤 ORIG를 재작성한다. */
    EraseUcbSwapSector(UCB_SWAP_ORIG);
    if (!VerifyUcbSwapErased(UCB_SWAP_ORIG))
    {
        SOTA_UcbFailStop();
    }

    WriteSwapEntryOneBase(UCB_SWAP_ORIG, 0U, targetMarker);
    WriteUcbBlockUnlocked(UCB_SWAP_ORIG);       /* UCB_SWAP_ORIG + 0x1F0 = 0x43211234 */

    if (!VerifySwapEntryOneBase(UCB_SWAP_ORIG, 0U, targetMarker) ||
        !VerifyUcbBlockUnlocked(UCB_SWAP_ORIG))
    {
        SOTA_UcbFailStop();
    }

    /* 3) 최종 이중 검증 */
    if (!VerifySwapEntryOneBase(UCB_SWAP_COPY, 0U, targetMarker) ||
        !VerifySwapEntryOneBase(UCB_SWAP_ORIG, 0U, targetMarker) ||
        !VerifyUcbBlockUnlocked(UCB_SWAP_COPY) ||
        !VerifyUcbBlockUnlocked(UCB_SWAP_ORIG))
    {
        SOTA_UcbFailStop();
    }

    IfxCpu_restoreInterrupts(irq);

    // IfxScuRcu_performReset(IfxScuRcu_ResetType_system, 0); // jdTest 빼야됨
}

/*************************************************************/
/* 공개 인터페이스                                            */
/*************************************************************/

boolean SOTA_IsInitialized(void)
{
    return (SOTA_GetSwapen() == SOTA_PROCONTP_SWAPEN_ENABLED) ? TRUE : FALSE;
}

void SOTA_EnableSwapen(void)
{
    uint32 procontp = *(volatile uint32 *)SOTA_PROCONTP_ADDR;

    /* SWAPEN = bits 17:16, Enabled = 0b11 */
    if ((procontp & SOTA_PROCONTP_SWAPEN_MASK) != SOTA_PROCONTP_SWAPEN_MASK)
    {
        WriteDFlash8(SOTA_PROCONTP_ADDR,
                     procontp | SOTA_PROCONTP_SWAPEN_MASK,
                     0x00000000UL);
    }
}

void SOTA_InitialSetup(void)
{
    sint8  cur;
    uint32 next;

    /* 실제 SWAPEN 값 기준으로 disabled 상태일 때만 초기화한다.
     * 기존 유효 entry가 있으면 덮어쓰지 않고 next entry에 Bank A marker를 append한다.
     */
    if (SOTA_IsInitialized())
    {
        return;
    }

    cur  = FindCurrentSwapEntry();
    next = (cur < 0) ? 0U : (uint32)(cur + 1);

    if (next >= 16U)
    {
        SOTA_EraseAndResetSwap(SWAP_MARKER_A);
    }
    else
    {
        WriteSwapEntry(next, SWAP_MARKER_A);  /* 초기 표준 맵: Bank A */

        if (cur >= 0)
        {
            InvalidateSwapEntry((uint32)cur);
        }
    }

    SOTA_EnableSwapen();

    IfxScuRcu_performReset(IfxScuRcu_ResetType_system, 0);
}

void SOTA_SwapToGroupB(void)
{
    sint8  cur  = FindCurrentSwapEntry();
    uint32 next = (cur < 0) ? 0U : (uint32)(cur + 1);

    if (next >= 16U)
    {
        SOTA_EraseAndResetSwap(SWAP_MARKER_B);
    }
    else
    {
        WriteSwapEntry(next, SWAP_MARKER_B);

        if (cur >= 0)
        {
            InvalidateSwapEntry((uint32)cur);
        }

        IfxScuRcu_performReset(IfxScuRcu_ResetType_system, 0);
    }
}

void SOTA_SwapToGroupA(void)
{
    sint8  cur  = FindCurrentSwapEntry();
    uint32 next = (cur < 0) ? 0U : (uint32)(cur + 1);

    if (next >= 16U)
    {
        SOTA_EraseAndResetSwap(SWAP_MARKER_A);
    }
    else
    {
        WriteSwapEntry(next, SWAP_MARKER_A);

        if (cur >= 0)
        {
            InvalidateSwapEntry((uint32)cur);
        }

        IfxScuRcu_performReset(IfxScuRcu_ResetType_system, 0);
    }
}

boolean SOTA_IsGroupBActive(void)
{
    uint32 swapCfg = SOTA_GetSwapCfg();

    if (swapCfg == SOTA_SWAP_CFG_B)
    {
        return TRUE;
    }

    if (swapCfg == SOTA_SWAP_CFG_A)
    {
        return FALSE;
    }

    /* 00b 또는 11b는 SOTA map 상태가 명확하지 않다.
     * 초기 개발 중에는 UCB marker fallback도 가능하지만,
     * OTA 방향 결정용으로는 잘못된 bank에 쓰는 것이 더 위험하므로 fail-stop 처리한다.
     */
    SOTA_UcbFailStop();
    return FALSE;
}
