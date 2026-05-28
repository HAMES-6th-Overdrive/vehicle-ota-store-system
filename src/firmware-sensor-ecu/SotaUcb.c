#include "SotaUcb.h"

#include "IfxCpu.h"
#include "IfxFlash.h"
#include "IfxScuRcu.h"
#include "IfxScuWdt.h"

void Sota_WriteDFlash8(uint32 addr, uint32 lo, uint32 hi)
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

static void failStop(void)
{
    while (1)
    {
        __nop();
    }
}

static boolean isValidMarker(uint32 marker)
{
    return ((marker == SOTA_SWAP_MARKER_A) || (marker == SOTA_SWAP_MARKER_B)) ? TRUE : FALSE;
}

static boolean isBlockUnlocked(uint32 base)
{
    uint32 lo = *(volatile uint32 *)(base + SOTA_UCB_BLOCK_CONFIRM_OFFSET);
    uint32 hi = *(volatile uint32 *)(base + SOTA_UCB_BLOCK_CONFIRM_OFFSET + 4U);

    return ((lo == SOTA_UCB_UNLOCKED_CODE) && (hi == 0U)) ? TRUE : FALSE;
}

static boolean isBlockConfirmErased(uint32 base)
{
    uint32 lo = *(volatile uint32 *)(base + SOTA_UCB_BLOCK_CONFIRM_OFFSET);
    uint32 hi = *(volatile uint32 *)(base + SOTA_UCB_BLOCK_CONFIRM_OFFSET + 4U);

    return ((lo == 0U) && (hi == 0U)) ? TRUE : FALSE;
}

static void ensureBlockUnlocked(uint32 base)
{
    if (isBlockUnlocked(base) == TRUE)
    {
        return;
    }

    if (isBlockConfirmErased(base) == TRUE)
    {
        Sota_WriteDFlash8(base + SOTA_UCB_BLOCK_CONFIRM_OFFSET, SOTA_UCB_UNLOCKED_CODE, 0U);
        if (isBlockUnlocked(base) == TRUE)
        {
            return;
        }
    }

    failStop();
}

static void writeSwapEntryOneBase(uint32 base, uint32 entry, uint32 marker)
{
    uint32 markerAddr = base + (entry * SOTA_UCB_SWAP_ENTRY_SIZE);
    uint32 confirmAddr = markerAddr + SOTA_UCB_SWAP_CONFIRM_OFFSET;

    Sota_WriteDFlash8(markerAddr, marker, markerAddr);
    Sota_WriteDFlash8(confirmAddr, SOTA_UCB_CONFIRM_CODE, confirmAddr);
}

static boolean verifySwapEntryOneBase(uint32 base, uint32 entry, uint32 marker)
{
    uint32 markerAddr = base + (entry * SOTA_UCB_SWAP_ENTRY_SIZE);
    uint32 confirmAddr = markerAddr + SOTA_UCB_SWAP_CONFIRM_OFFSET;

    return ((*(volatile uint32 *)(markerAddr + 0U) == marker) &&
            (*(volatile uint32 *)(markerAddr + 4U) == markerAddr) &&
            (*(volatile uint32 *)(confirmAddr + 0U) == SOTA_UCB_CONFIRM_CODE) &&
            (*(volatile uint32 *)(confirmAddr + 4U) == confirmAddr)) ? TRUE : FALSE;
}

static sint8 findCurrentSwapEntryInBase(uint32 base)
{
    if (isBlockUnlocked(base) == FALSE)
    {
        return -1;
    }

    for (sint8 i = 15; i >= 0; i--)
    {
        uint32 markerAddr = base + ((uint32)i * SOTA_UCB_SWAP_ENTRY_SIZE);
        uint32 confirmAddr = markerAddr + SOTA_UCB_SWAP_CONFIRM_OFFSET;
        uint32 marker = *(volatile uint32 *)(markerAddr + 0U);

        if ((isValidMarker(marker) == TRUE) &&
            (*(volatile uint32 *)(markerAddr + 4U) == markerAddr) &&
            (*(volatile uint32 *)(confirmAddr + 0U) == SOTA_UCB_CONFIRM_CODE) &&
            (*(volatile uint32 *)(confirmAddr + 4U) == confirmAddr))
        {
            return i;
        }
    }

    return -1;
}

static sint8 findCurrentSwapEntry(void)
{
    sint8 orig = findCurrentSwapEntryInBase(SOTA_UCB_SWAP_ORIG);
    sint8 copy = findCurrentSwapEntryInBase(SOTA_UCB_SWAP_COPY);

    if ((orig >= 0) && (copy >= 0) && (orig != copy))
    {
        failStop();
    }

    return (orig >= 0) ? orig : copy;
}

static void invalidateSwapEntry(uint32 entry)
{
    Sota_WriteDFlash8(SOTA_UCB_SWAP_ORIG +
                      (entry * SOTA_UCB_SWAP_ENTRY_SIZE) +
                      SOTA_UCB_SWAP_CONFIRM_OFFSET,
                      0xFFFFFFFFUL,
                      0xFFFFFFFFUL);

    Sota_WriteDFlash8(SOTA_UCB_SWAP_COPY +
                      (entry * SOTA_UCB_SWAP_ENTRY_SIZE) +
                      SOTA_UCB_SWAP_CONFIRM_OFFSET,
                      0xFFFFFFFFUL,
                      0xFFFFFFFFUL);
}

static void swapToMarker(uint32 marker)
{
    sint8 current = findCurrentSwapEntry();
    uint32 next = (current < 0) ? 0U : ((uint32)current + 1U);

    if ((isValidMarker(marker) == FALSE) || (next >= 16U))
    {
        failStop();
    }

    ensureBlockUnlocked(SOTA_UCB_SWAP_ORIG);
    ensureBlockUnlocked(SOTA_UCB_SWAP_COPY);
    writeSwapEntryOneBase(SOTA_UCB_SWAP_ORIG, next, marker);
    writeSwapEntryOneBase(SOTA_UCB_SWAP_COPY, next, marker);

    if ((verifySwapEntryOneBase(SOTA_UCB_SWAP_ORIG, next, marker) == FALSE) ||
        (verifySwapEntryOneBase(SOTA_UCB_SWAP_COPY, next, marker) == FALSE))
    {
        failStop();
    }

    if (current >= 0)
    {
        invalidateSwapEntry((uint32)current);
    }

    IfxScuRcu_performReset(IfxScuRcu_ResetType_system, 0);
}

void Sota_SetPendingUpdateFlag(uint32 firmwareSize, uint32 expectedCrc32)
{
    uint16 pw = IfxScuWdt_getSafetyWatchdogPassword();

    IfxScuWdt_clearSafetyEndinit(pw);
    IfxFlash_eraseMultipleSectors(SOTA_OTA_FLAG_ADDR, 1U);
    IfxScuWdt_setSafetyEndinit(pw);
    IfxFlash_waitUnbusy(0, IfxFlash_FlashType_D0);

    Sota_WriteDFlash8(SOTA_OTA_FLAG_ADDR, SOTA_OTA_FLAG_MAGIC, 0U);
    Sota_WriteDFlash8(SOTA_OTA_FLAG_ADDR + 8U, firmwareSize, 0U);
    Sota_WriteDFlash8(SOTA_OTA_FLAG_ADDR + 16U, expectedCrc32, 0U);
}

void Sota_ClearPendingUpdateFlag(void)
{
    uint16 pw = IfxScuWdt_getSafetyWatchdogPassword();

    IfxScuWdt_clearSafetyEndinit(pw);
    IfxFlash_eraseMultipleSectors(SOTA_OTA_FLAG_ADDR, 1U);
    IfxScuWdt_setSafetyEndinit(pw);
    IfxFlash_waitUnbusy(0, IfxFlash_FlashType_D0);
}

boolean Sota_IsInitialized(void)
{
    return (findCurrentSwapEntry() >= 0) ? TRUE : FALSE;
}

boolean Sota_IsGroupBActive(void)
{
    uint32 swapCfg = (*(volatile uint32 *)SOTA_SCU_STMEM1_ADDR &
                      SOTA_SCU_STMEM1_SWAP_CFG_MASK) >>
                     SOTA_SCU_STMEM1_SWAP_CFG_POS;

    return (swapCfg == SOTA_SWAP_CFG_B) ? TRUE : FALSE;
}

void Sota_SwapToGroupA(void)
{
    swapToMarker(SOTA_SWAP_MARKER_A);
}

void Sota_SwapToGroupB(void)
{
    swapToMarker(SOTA_SWAP_MARKER_B);
}
