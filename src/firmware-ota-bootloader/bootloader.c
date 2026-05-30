/**********************************************************************************************************************
 * \file bootloader.c
 * \copyright Copyright (C) Infineon Technologies AG 2019
 *********************************************************************************************************************/

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "Ifx_Ssw.h"

#include "bootloader.h"
#include "ota_flash.h"
#include "sota_ucb.h"

#include <stdio.h>

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/
typedef void (*AppFunc)(void);

extern volatile uint32 g_bootJumpToAppRequest;

static void Bootloader_JumpToApp(uint32 appAddr);

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/

static void Bootloader_JumpToApp(uint32 appAddr)
{
    AppFunc app = (AppFunc)TO_FLASH_ADDR(appAddr);
    volatile uint32 wait;

    IfxCpu_disableInterrupts();

    /*
     * CPU1에게 App START1로 넘어가라고 알린다.
     * Bootloader CPU1은 이 flag를 보고 App CPU1 startup entry로 jump한다.
     *
     * CPU2는 이번 단계에서 건드리지 않는다.
     */
    g_bootJumpToAppRequest = 1U;

    Ifx_Ssw_DSYNC();
    Ifx_Ssw_ISYNC();

    /*
     * CPU1이 flag를 보고 jump할 시간을 조금 준다.
     */
    for (wait = 0U; wait < 100000U; wait++)
    {
        __nop();
    }

    Ifx_Ssw_DSYNC();
    Ifx_Ssw_ISYNC();

    /*
     * CPU0도 App START0로 jump.
     */
    Ifx_Ssw_jumpToFunction(app);
}

void Bootloader_Main(void)
{
    uint32 flag;

    printf("[BL] Bootloader_Main enter\r\n");

    flag = *(volatile uint32 *)OTA_FLAG_ADDR;

    printf("[BL] flag = 0x%08X\r\n", (unsigned int)flag);

    if (flag == OTA_FLAG_MAGIC)
    {
        uint32 fwSize;
        uint32 expectedCRC;
        boolean isGroupBActive;
        uint32 targetStart;
        uint32 targetSize;
        boolean verifyOk;
        OtaPendingMeta_t meta;

        fwSize = *(volatile uint32 *)(OTA_FLAG_ADDR + 8U);
        expectedCRC = *(volatile uint32 *)(OTA_FLAG_ADDR + 16U);
        isGroupBActive = SOTA_IsGroupBActive();

        if (isGroupBActive == TRUE)
        {
            targetStart = BANK_A_START;
            targetSize = BANK_A_SIZE;
            printf("[BL] target = Bank A\r\n");
        }
        else
        {
            targetStart = BANK_B_START;
            targetSize = BANK_B_SIZE;
            printf("[BL] target = Bank B\r\n");
        }

        printf("[BL] fwSize      = 0x%08X\r\n", (unsigned int)fwSize);
        printf("[BL] expectedCRC = 0x%08X\r\n", (unsigned int)expectedCRC);
        printf("[BL] targetStart = 0x%08X\r\n", (unsigned int)targetStart);
        printf("[BL] targetSize  = 0x%08X\r\n", (unsigned int)targetSize);

        /*
         * 새 metadata layout이 있으면 metadata 기반 sparse CRC 사용.
         * 아직 Sensor ECU가 metadata를 저장하지 않는 구버전 layout이면
         * 기존 fwSize/expectedCRC 기반 legacy CRC path로 fallback.
         */
        if (OTA_Flash_ReadPendingMeta(&meta) == TRUE)
        {
            printf("[BL] metadata found\r\n");
            printf("[BL] meta.virtualSize  = 0x%08X\r\n", (unsigned int)meta.virtualSize);
            printf("[BL] meta.expectedCRC  = 0x%08X\r\n", (unsigned int)meta.expectedCrc32);
            printf("[BL] meta.segmentCount = %u\r\n", (unsigned int)meta.segmentCount);
            printf("[BL] meta.gapFill      = 0x%08X\r\n", (unsigned int)meta.gapFill);

            verifyOk = OTA_Flash_VerifySparseCRC(targetStart, &meta);
        }
        else
        {
            printf("[BL] metadata not found, use legacy CRC path\r\n");

            verifyOk = FALSE;

            if ((fwSize > 0U) && (fwSize <= targetSize))
            {
                verifyOk = OTA_Flash_VerifyCRC(targetStart, fwSize, expectedCRC);
            }
        }

        if (verifyOk == TRUE)
        {
            printf("[BL] CRC OK\r\n");

            OTA_Flash_ClearFlag();
            printf("[BL] flag clear OK\r\n");

            if (isGroupBActive == TRUE)
            {
                printf("[BL] swap to A\r\n");
                SOTA_SwapToGroupA();
            }
            else
            {
                printf("[BL] swap to B\r\n");
                SOTA_SwapToGroupB();
            }

            /*
             * SOTA_SwapToGroupA/B 내부에서 reset이 걸리는 구조지만,
             * 기존 안전장치 유지.
             */
            IfxScuRcu_performReset(IfxScuRcu_ResetType_system, 0);

            while (1)
            {
            }
        }
        else
        {
            printf("[BL] CRC FAILED\r\n");

            OTA_Flash_ClearFlag();
            printf("[BL] flag clear after fail\r\n");

            Bootloader_JumpToApp(APP_START_ADDR);
        }
    }
    else
    {
        printf("[BL] no pending flag, jump app\r\n");

        Bootloader_JumpToApp(APP_START_ADDR);
    }
}
