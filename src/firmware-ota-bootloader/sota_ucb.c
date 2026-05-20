#include "sota_ucb.h"
#include "IfxFlash.h"
#include "IfxScuWdt.h"
#include "IfxScuRcu.h"

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

/* UCB_SWAP_ORIG에서 마지막 유효 Entry 인덱스 반환 (-1: 없음) */
static sint8 FindCurrentSwapEntry(void)
{
    for (sint8 x = 15; x >= 0; x--)
    {
        uint32 confirmL = *(volatile uint32*)(UCB_SWAP_ORIG + (uint32)x * 0x10 + 0x08);
        if (confirmL == UCB_CONFIRM_CODE)
            return x;
    }
    return -1;
}

/* Entry 쓰기 (ORIG + COPY 동시) — readback 진단 포함 */
static void WriteSwapEntry(uint32 entryIdx, uint32 markerVal)
{
    uint32 bases[2] = { UCB_SWAP_ORIG, UCB_SWAP_COPY };

    for (uint8 i = 0; i < 2; i++)
    {
        uint32 markerLAddr  = bases[i] + entryIdx * 0x10;
        uint32 confirmLAddr = markerLAddr + 0x08;

        WriteDFlash8(markerLAddr,  markerVal,        markerLAddr);
        WriteDFlash8(confirmLAddr, UCB_CONFIRM_CODE, confirmLAddr);

        /* 쓰기 직후 readback — 0xAF003000+에 저장 */
        uint32 rbMarker  = *(volatile uint32*)markerLAddr;
        uint32 rbConfirm = *(volatile uint32*)confirmLAddr;
        WriteDFlash8(0xAF003000UL + (uint32)i * 0x10, rbMarker, rbConfirm);
    }
}

/* Entry 무효화 (CONFIRMATION → 0xFFFFFFFF) */
static void InvalidateSwapEntry(uint32 x)
{
    (void)x;
}

static void SOTA_EraseAndResetSwap(uint32 targetMarker)  /* SWAP_MARKER_A or SWAP_MARKER_B */
{
    return;

    uint16 pw = IfxScuWdt_getSafetyWatchdogPassword();
    IfxScuWdt_clearSafetyEndinit(pw);

    IfxFlash_eraseMultipleSectors(UCB_SWAP_ORIG, 1);
    IfxFlash_waitUnbusy(0, IfxFlash_FlashType_D0);
    IfxFlash_eraseMultipleSectors(UCB_SWAP_COPY, 1);
    IfxFlash_waitUnbusy(0, IfxFlash_FlashType_D0);

    IfxScuWdt_setSafetyEndinit(pw);

    WriteSwapEntry(0, targetMarker);
    IfxScuRcu_performReset(IfxScuRcu_ResetType_system, 0);
}

/*************************************************************/
/* 공개 인터페이스                                            */
/*************************************************************/

boolean SOTA_IsInitialized(void)
{
    return (FindCurrentSwapEntry() >= 0) ? TRUE : FALSE;
}

void SOTA_EnableSwapen(void)
{
    uint32 procontp = *(volatile uint32*)0xAF4041E8UL;

    /* SWAPEN = bits 17:16, Enabled = 0b11 */
    if ((procontp & 0x00030000UL) != 0x00030000UL)
        WriteDFlash8(0xAF4041E8UL,
                     procontp | 0x00030000UL,
                     0x00000000UL);
}

void SOTA_InitialSetup(void)
{
    SOTA_EnableSwapen();
    WriteSwapEntry(0, SWAP_MARKER_A);  /* Entry 0: Group A (초기 표준 맵) */
    IfxScuRcu_performReset(IfxScuRcu_ResetType_system, 0);
}

void SOTA_SwapToGroupB(void)
{
    sint8  cur  = FindCurrentSwapEntry();
    uint32 next = (cur < 0) ? 0 : (uint32)(cur + 1);

    if (next >= 16) {
        SOTA_EraseAndResetSwap(SWAP_MARKER_B);
        return;
    }

    WriteSwapEntry(next, SWAP_MARKER_B);

    if (cur >= 0)
        InvalidateSwapEntry((uint32)cur);  /* 이전 Entry 무효화 */

    //IfxScuRcu_performReset(IfxScuRcu_ResetType_system, 0);
}

void SOTA_SwapToGroupA(void)
{
    sint8  cur  = FindCurrentSwapEntry();
    uint32 next = (cur < 0) ? 0 : (uint32)(cur + 1);

    if (next >= 16) {
        SOTA_EraseAndResetSwap(SWAP_MARKER_A);
        return;
    }

    WriteSwapEntry(next, SWAP_MARKER_A);

    if (cur >= 0)
        InvalidateSwapEntry((uint32)cur);

    //IfxScuRcu_performReset(IfxScuRcu_ResetType_system, 0);
}

boolean SOTA_IsGroupBActive(void)
{
    sint8 cur = FindCurrentSwapEntry();
    if (cur < 0) return FALSE;

    uint32 markerL = *(volatile uint32*)(UCB_SWAP_ORIG + (uint32)cur * 0x10);
    return (markerL == SWAP_MARKER_B) ? TRUE : FALSE;
}
