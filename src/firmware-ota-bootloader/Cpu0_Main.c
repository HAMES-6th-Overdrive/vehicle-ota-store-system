/**********************************************************************************************************************
 * \file Cpu0_Main.c
 * \copyright Copyright (C) Infineon Technologies AG 2019
 *
 * Bootloader Main
 *********************************************************************************************************************/

#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include "Ifx_Cfg_Ssw.h"
#include "Ifx_Ssw.h"

#include "sota_ucb.h"
#include "ota_flash.h"
#include "UART_VCOM.h"

#include <stdio.h>

#include "IfxAsclin_Asc.h"
#include "IfxAsclin_reg.h"

boolean g_isGroupBActive = FALSE;
extern IfxAsclin_Asc g_ascPrint;

typedef void (*AppFunc)(void);

/*
 * App LSL 기준:
 * APP_START_ADDR = 0x80020000 계열
 * TO_FLASH_ADDR(APP_START_ADDR) = non-cached alias, 즉 0xA0020000 계열
 *
 * CPU1 App startup entry:
 *   __START1 = APP_START_NC + 0x500
 */
#define BOOT_APP_START0_NC   TO_FLASH_ADDR(APP_START_ADDR)
#define BOOT_APP_START1_NC   (BOOT_APP_START0_NC + 0x500U)

/*
 * Bootloader CPU0이 App으로 넘어가기 직전에 1로 만든다.
 * Bootloader CPU1은 이 flag를 보고 App START1로 jump한다.
 */
volatile uint32 g_bootJumpToAppRequest = 0U;

/* Watch 확인용 */
volatile uint32 g_bootCpu1EntryCount = 0U;
volatile uint32 g_bootCpu1WaitCount  = 0U;
volatile uint32 g_bootCpu1JumpCount  = 0U;

int _write(int fd, char *buf, int len)
{
    (void)fd;

    if ((buf == NULL_PTR) || (len <= 0))
    {
        return 0;
    }

    for (int i = 0; i < len; i++)
    {
        uint32 timeout = 1000000u;

        while ((IfxAsclin_getTxFifoFillLevel(&MODULE_ASCLIN0) >= 16u) &&
               (timeout-- > 0u))
        {
            __nop();
        }

        if (timeout == 0u)
        {
            break;
        }

        IfxAsclin_writeTxData(&MODULE_ASCLIN0, (uint16)(uint8)buf[i]);
    }

    return len;
}

IFX_ALIGN(4) IfxCpu_syncEvent cpuSyncEvent = 0;

extern void Bootloader_Main(void);

/*
 * Bootloader CPU1.
 *
 * 기존에는 while(1)에 영원히 남아 있어서,
 * CPU0만 App으로 jump한 뒤에도 CPU1 PC가 bootloader 주소에 남았다.
 *
 * 이제는 CPU0이 g_bootJumpToAppRequest를 1로 만들면
 * CPU1도 App의 CPU1 startup entry(START1)로 직접 jump한다.
 */
void core1_main(void)
{
    g_bootCpu1EntryCount++;

    IfxCpu_enableInterrupts();

    /*
     * WATCHDOG1 disable.
     */
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());

    while (g_bootJumpToAppRequest == 0U)
    {
        g_bootCpu1WaitCount++;
        __nop();
    }

    g_bootCpu1JumpCount++;

    IfxCpu_disableInterrupts();

    Ifx_Ssw_DSYNC();
    Ifx_Ssw_ISYNC();

    /*
     * App CPU1 startup entry로 jump.
     * core1_main() 직접 주소가 아니라 START1로 보내야
     * App 쪽 CPU1 startup/CSA/stack 흐름을 탄다.
     */
    Ifx_Ssw_jumpToFunction((AppFunc)BOOT_APP_START1_NC);

    while (1)
    {
    }
}

/*
 * CPU2는 이번 단계에서 건드리지 않는다.
 */
void core2_main(void)
{
    while (1)
    {
    }
}

void core0_main(void)
{
    /*
     * 기존 bootloader 구조 유지.
     * 필요 시 interrupt enable은 기존 주석 상태 그대로 둔다.
     */
    /* IfxCpu_enableInterrupts(); */

    /*
     * WATCHDOG0 AND SAFETY WATCHDOG disable.
     */
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

    /*
     * Wait for CPU sync event.
     */
    IfxCpu_emitEvent(&cpuSyncEvent);
    IfxCpu_waitEvent(&cpuSyncEvent, 1);

    init_UART();

    if (!SOTA_IsInitialized())
    {
        SOTA_InitialSetup();
    }

    g_isGroupBActive = SOTA_IsGroupBActive();

    if (g_isGroupBActive)
    {
        printf("Bootloader Bank B!\r\n");
    }
    else
    {
        printf("Bootloader Bank A!\r\n");
    }

    for (volatile int i = 0; i < 10000000; ++i)
    {
        __nop();
    }

    Bootloader_Main();

    while (1)
    {
    }
}
