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

IFX_ALIGN(4) IfxCpu_syncEvent g_cpuSyncEvent = 0;

/* Watch 확인용 */
volatile uint32_t mainLoopCount = 0U;

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
    initScheduler();
    HallSensor_init();

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

        /*
         * 나중에 background task 추가 가능:
         * - OTA state machine
         * - fault monitoring
         * - debug service
         */
    }
}
