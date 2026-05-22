/**********************************************************************************************************************
 * \file Scheduler.c
 * \brief Sensor ECU Scheduler
 *
 * 구조:
 *  - STM ISR에서는 1ms tick 기준으로 주기 flag만 set
 *  - 실제 CAN 송신 및 센서 처리는 main context의 Scheduler_run()에서 수행
 *
 * 주기:
 *  - 1ms   : HallSensor D0 pulse count update
 *            + Pending OTA Request 처리
 *            + CAN TX service
 *  - 10ms  : 0x201 TofDistanceData, FEATURE_TOF_SENSOR == 1U일 때만
 *  - 100ms : 0x202 SpeedData
 *
 * 정책:
 *  - Sensor ECU의 센서값 송신은 Gear P/D와 무관하게 계속 수행한다.
 *  - OTA 허용 조건도 Gear P/D와 무관하게 항상 허용한다.
 *  - ZCU는 Gear 정보를 전달하지 않는다.
 *  - IMU 센서는 현재 프로젝트에서 사용하지 않는다.
 *********************************************************************************************************************/

#include "Scheduler.h"

#include "IfxStm.h"
#include "IfxCpu.h"
#include "IfxCpu_Irq.h"
#include "IfxSrc.h"

#include "MCMCAN.h"
#include "can_type_def.h"
#include "FeatureConfig.h"
#include "HallSensor.h"

#if (FEATURE_TOF_SENSOR == 1U)
#include "TofSensor.h"
#endif

#include <stdint.h>

/*********************************************************************************************************************/
/*-------------------------------------------------Static variables--------------------------------------------------*/
/*********************************************************************************************************************/

#define USE_FIXED_SPEED_FOR_AEB_TEST    0U
#define TEST_FIXED_SPEED_KMH_X100       630U

static uint32               s_tick = 0U;
static IfxStm_CompareConfig s_stmConfig;

/*
 * flag 방식 사용.
 * 이미 flag가 TRUE인데 또 주기가 오면 overrun으로 카운트.
 * 센서값은 밀린 만큼 몰아서 보내는 것보다 최신값을 주기적으로 보내는 게 맞음.
 */
static volatile boolean s_task1msFlag   = FALSE;
static volatile boolean s_task10msFlag  = FALSE;
static volatile boolean s_task100msFlag = FALSE;

/*********************************************************************************************************************/
/*------------------------------------------------Debug variables----------------------------------------------------*/
/*********************************************************************************************************************/

/* Scheduler Watch 확인용 */
volatile uint32 schedulerTickCount = 0U;

volatile uint32 scheduler1msCount = 0U;
volatile uint32 scheduler10msCount = 0U;
volatile uint32 scheduler100msCount = 0U;

volatile uint32 scheduler1msOverrunCount = 0U;
volatile uint32 scheduler10msOverrunCount = 0U;
volatile uint32 scheduler100msOverrunCount = 0U;

/*********************************************************************************************************************/
/*------------------------------------------------Private prototypes-------------------------------------------------*/
/*********************************************************************************************************************/

static void    Scheduler_setTaskFlag(volatile boolean *flag, volatile uint32 *overrunCounter);
static boolean Scheduler_consumeTaskFlag(volatile boolean *flag);

static void task_1ms(void);
static void task_10ms(void);
static void task_100ms(void);

#if (FEATURE_TOF_SENSOR == 1U)
static void sendTofDistanceData10ms(void);
#endif

static void sendSpeedData100ms(uint16_t vehicleSpeed);

/*********************************************************************************************************************/
/*------------------------------------------------ISR Definition-----------------------------------------------------*/
/*********************************************************************************************************************/

IFX_INTERRUPT(stm0IsrHandler, 0, ISR_PRIORITY_STM);

void stm0IsrHandler(void)
{
    /*
     * 다음 compare 시점 갱신
     */
    IfxStm_increaseCompare(&MODULE_STM0,
                           s_stmConfig.comparator,
                           IfxStm_getTicksFromMilliseconds(&MODULE_STM0,
                                                           SCHEDULER_BASE_MS));

    s_tick++;
    schedulerTickCount = s_tick;

    /*
     * 1ms task flag
     */
    Scheduler_setTaskFlag(&s_task1msFlag, &scheduler1msOverrunCount);
    scheduler1msCount++;

    /*
     * 10ms task flag
     */
    if((s_tick % TASK_10MS_DIV) == 0U)
    {
        Scheduler_setTaskFlag(&s_task10msFlag, &scheduler10msOverrunCount);
        scheduler10msCount++;
    }

    /*
     * 100ms task flag
     */
    if((s_tick % TASK_100MS_DIV) == 0U)
    {
        Scheduler_setTaskFlag(&s_task100msFlag, &scheduler100msOverrunCount);
        scheduler100msCount++;
    }
}

/*********************************************************************************************************************/
/*------------------------------------------------Initialization-----------------------------------------------------*/
/*********************************************************************************************************************/

void initScheduler(void)
{
    s_tick = 0U;

    s_task1msFlag   = FALSE;
    s_task10msFlag  = FALSE;
    s_task100msFlag = FALSE;

    schedulerTickCount = 0U;

    scheduler1msCount = 0U;
    scheduler10msCount = 0U;
    scheduler100msCount = 0U;

    scheduler1msOverrunCount = 0U;
    scheduler10msOverrunCount = 0U;
    scheduler100msOverrunCount = 0U;

    IfxStm_initCompareConfig(&s_stmConfig);

    s_stmConfig.comparator      = IfxStm_Comparator_0;
    s_stmConfig.compareOffset   = IfxStm_ComparatorOffset_0;
    s_stmConfig.compareSize     = IfxStm_ComparatorSize_32Bits;

    s_stmConfig.ticks           = IfxStm_getTicksFromMilliseconds(&MODULE_STM0,
                                                                  SCHEDULER_BASE_MS);

    s_stmConfig.triggerPriority = ISR_PRIORITY_STM;
    s_stmConfig.typeOfService   = IfxSrc_Tos_cpu0;

    IfxStm_initCompare(&MODULE_STM0, &s_stmConfig);
}

/*********************************************************************************************************************/
/*------------------------------------------------Public functions---------------------------------------------------*/
/*********************************************************************************************************************/

void Scheduler_run(void)
{
    if(Scheduler_consumeTaskFlag(&s_task1msFlag) == TRUE)
    {
        task_1ms();
    }

    if(Scheduler_consumeTaskFlag(&s_task10msFlag) == TRUE)
    {
        task_10ms();
    }

    if(Scheduler_consumeTaskFlag(&s_task100msFlag) == TRUE)
    {
        task_100ms();
    }
}

uint32 Scheduler_getTick(void)
{
    return s_tick;
}

/*********************************************************************************************************************/
/*------------------------------------------------Private functions--------------------------------------------------*/
/*********************************************************************************************************************/

static void Scheduler_setTaskFlag(volatile boolean *flag, volatile uint32 *overrunCounter)
{
    if(*flag == FALSE)
    {
        *flag = TRUE;
    }
    else
    {
        (*overrunCounter)++;
    }
}

static boolean Scheduler_consumeTaskFlag(volatile boolean *flag)
{
    boolean ret = FALSE;
    boolean interruptState;

    interruptState = IfxCpu_disableInterrupts();

    if(*flag == TRUE)
    {
        *flag = FALSE;
        ret = TRUE;
    }

    IfxCpu_restoreInterrupts(interruptState);

    return ret;
}

/*********************************************************************************************************************/
/*------------------------------------------------Task functions-----------------------------------------------------*/
/*********************************************************************************************************************/

/**
 * @brief 1ms task
 *
 * 처리:
 *  - HallSensor D0 pulse count update
 *  - RX ISR에서 pending 처리한 0x600 OTA Request 처리
 *  - CAN TX Queue 보조 service
 *
 * 주의:
 *  - 1ms task에서는 주기 CAN 데이터 송신은 하지 않음.
 *  - 홀센서 펄스를 놓치지 않기 위해 1ms마다 GPIO를 읽음.
 *  - OTA Request는 RX ISR 안에서 직접 처리하지 않고 여기서 처리한다.
 *  - CanIf_TxService1ms()는 TX complete ISR 누락/stuck 진단 및 복구 보조용.
 */
static void task_1ms(void)
{
    HallSensor_update1ms();

    /*
     * RX ISR에서 복사해둔 0x600 UDS OTA Request를
     * ISR 밖 main/scheduler context에서 처리한다.
     *
     * UdsOta_onRequest() 내부에서 Flash erase/write/CRC 같은 작업이 수행될 수 있으므로
     * CAN RX ISR 안에서 직접 호출하지 않는다.
     */
    CanIf_ProcessPendingOtaRequest();

    CanIf_TxService1ms();
}

/**
 * @brief 10ms task
 *
 * 송신:
 *  - FEATURE_TOF_SENSOR == 1U일 때만 0x201 TofDistanceData
 *
 * 정책:
 *  - TOF 기능이 있는 Application에서는 거리값을 10ms마다 송신한다.
 *  - TOF 기능이 없는 Application에서는 TOF를 읽지도 않고 0x201도 송신하지 않는다.
 */
static void task_10ms(void)
{
    HallSensor_calcSpeed10ms();

#if (FEATURE_TOF_SENSOR == 1U)
    sendTofDistanceData10ms();
#else
    /*
     * TOF OFF 버전:
     *  - TOF 센서 읽기 없음
     *  - 0x201 TofDistanceData 송신 없음
     */
#endif
}

/**
 * @brief 100ms task
 *
 * 처리:
 *  - HallSensor pulse interval 기반 속도 계산
 *
 * 송신:
 *  - 0x202 SpeedData
 *
 * 정책:
 *  - SpeedData는 Gear P/D와 무관하게 계속 송신한다.
 *  - ZCU의 AEB 판단을 위해 최신 차량 속도값이 필요하다.
 */
static void task_100ms(void)
{
    uint16_t vehicleSpeed;

#if (USE_FIXED_SPEED_FOR_AEB_TEST == 1U)

    /*
     * AEB 테스트용:
     * 홀센서 위치가 정확하지 않으므로 속도를 고정값으로 송신한다.
     *
     * 단위:
     * 630 = 6.30 km/h
     */
    vehicleSpeed = TEST_FIXED_SPEED_KMH_X100;

#else

    /*
     * 실제 홀센서 기반 속도 계산
     */
    vehicleSpeed = HallSensor_getVehicleSpeed();

#endif

    sendSpeedData100ms(vehicleSpeed);
}

/*********************************************************************************************************************/
/*------------------------------------------------Send helper functions----------------------------------------------*/
/*********************************************************************************************************************/

#if (FEATURE_TOF_SENSOR == 1U)

/**
 * @brief 0x201 TofDistanceData 송신
 *
 * 새 TOF 값이 들어왔을 때만 보내는 게 아니라,
 * 최신 거리값을 10ms마다 반복 송신한다.
 */
static void sendTofDistanceData10ms(void)
{
    TofDistanceData_t tofData;
    boolean tofValid;

    if(TofSensor_hasNewData() == TRUE)
    {
        TofSensor_clearNewDataFlag();
    }

    tofValid = TofSensor_isValid();

    if(tofValid == TRUE)
    {
        tofData.distanceMm = TofSensor_getDistanceMm();
    }
    else
    {
        tofData.distanceMm = TOF_DISTANCE_INVALID_MM;
    }

    (void)CanIf_sendTofDistanceData(&tofData);
}

#endif /* FEATURE_TOF_SENSOR */

/**
 * @brief 0x202 SpeedData 송신
 */
static void sendSpeedData100ms(uint16_t vehicleSpeed)
{
    SpeedData_t speedData;

    speedData.vehicleSpeed = vehicleSpeed;

    (void)CanIf_sendSpeedData(&speedData);
}
