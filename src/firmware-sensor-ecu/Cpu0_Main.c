/**********************************************************************************************************************
 * \file Cpu0_Main.c
 * \brief Sensor ECU Main
 *********************************************************************************************************************/

#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"

#include "MCMCAN.h"
#include "Scheduler.h"
#include "HallSensor.h"
#include "FlashOta.h"
#include "UART_VCOM.h"
#include "IfxAsclin_Asc.h"
#include <stdio.h>

#define APP_SENSOR_VERSION "1.0.1"

//#define SLOW
IFX_ALIGN(4) IfxCpu_syncEvent g_cpuSyncEvent = 0;

/* Watch 확인용 */
volatile uint32_t mainLoopCount = 0U;

boolean g_isGroupBActive = FALSE;
extern IfxAsclin_Asc g_ascPrint;
int _write(int fd, char *buf, int len)
{
    (void)fd;

    if ((buf == NULL_PTR) || (len <= 0))
        return 0;

    for (int i = 0; i < len; i++)
    {
        uint32 timeout = 1000000u;

        while ((IfxAsclin_getTxFifoFillLevel(&MODULE_ASCLIN0) >= 16u) && (timeout-- > 0u))
            __nop();

        if (timeout == 0u)
            break;

        IfxAsclin_writeTxData(&MODULE_ASCLIN0, (uint16)(uint8)buf[i]);
    }

    return len;
}

void core0_main(void)
{
    IfxCpu_enableInterrupts();

    /*
     * Watchdog disable
     */
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

    /*
     * CPU sync
     */
    IfxCpu_emitEvent(&g_cpuSyncEvent);
    IfxCpu_waitEvent(&g_cpuSyncEvent, 1);

    /*
     * Init
     */
    initMcmcan();
    HallSensor_init();
    initScheduler();
    //debug용
    init_UART();
    IfxPort_setPinModeOutput(&MODULE_P00, 5, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general); 
    g_isGroupBActive = Sota_IsGroupBActive();
    g_isGroupBActive ? printf("Bank B!\r\n") : printf("Bank A!\r\n");       
    volatile boolean b = TRUE;
    volatile uint32 temp_uint = 3;
    printf("%d %x\r\n", b, temp_uint);
    #ifdef SLOW
        printf("Sensor ECU Main - SLOW\r\n");
    #else
        printf("Sensor ECU Main - FAST\r\n");
    #endif
    while(1)
    {
        mainLoopCount++;

        /*
         * Scheduler 내부에서
         * - 10ms task: 0x200, 0x201
         * - 50ms task: 0x202
         * 를 실행한다.
         */

        Scheduler_run();
        CanIf_ProcessPendingOtaRequest();
        FlashOta_Service();
        /*
         * 나중에 background task 추가 가능:
         * - OTA state machine
         * - fault monitoring
         * - debug service
         */
        static volatile uint32 i;
        #ifdef SLOW
           if (++i >= 1000000) {
        #else
            if (++i >= 100000) {
        #endif
                    i = 0;
                    IfxPort_togglePin(&MODULE_P00, 5);
             }

    }
}
