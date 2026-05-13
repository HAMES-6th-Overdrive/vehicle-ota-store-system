/**********************************************************************************************************************
 * \file Scheduler.c
 * \brief Sensor ECU Scheduler
 *
 * 구조:
 *  - STM ISR에서는 1ms tick 기준으로 주기 flag만 set
 *  - 실제 CAN 송신 및 센서 처리는 main context의 Scheduler_run()에서 수행
 *
 * 주기:
 *  - 1ms : HallSensor pulse count update
 *  - 10ms: 0x200 ImuData, 0x201 TofDistanceData
 *  - 50ms: 0x202 SpeedData
 *********************************************************************************************************************/

#include "Scheduler.h"

#include "IfxStm.h"
#include "IfxCpu.h"
#include "IfxCpu_Irq.h"

#include "MCMCAN.h"
#include "can_type_def.h"
#include "TofSensor.h"
#include "HallSensor.h"

#include <stdint.h>

/*********************************************************************************************************************/
/*-------------------------------------------------Static variables--------------------------------------------------*/
/*********************************************************************************************************************/

static uint32               s_tick = 0U;
static IfxStm_CompareConfig s_stmConfig;

/*
 * flag 방식 사용.
 * 이미 flag가 TRUE인데 또 주기가 오면 overrun으로 카운트.
 * 센서값은 밀린 만큼 몰아서 보내는 것보다 최신값을 주기적으로 보내는 게 맞음.
 */
static volatile boolean s_task1msFlag  = FALSE;
static volatile boolean s_task10msFlag = FALSE;
static volatile boolean s_task50msFlag = FALSE;

/*********************************************************************************************************************/
/*------------------------------------------------Debug variables----------------------------------------------------*/
/*********************************************************************************************************************/

/* Scheduler Watch 확인용 */
volatile uint32 schedulerTickCount = 0U;

volatile uint32 scheduler1msCount = 0U;
volatile uint32 scheduler10msCount = 0U;
volatile uint32 scheduler50msCount = 0U;

volatile uint32 scheduler1msOverrunCount = 0U;
volatile uint32 scheduler10msOverrunCount = 0U;
volatile uint32 scheduler50msOverrunCount = 0U;

/* TOF Watch 확인용 */
volatile uint16_t mainTofDistanceMm = 0U;
volatile uint32_t mainTofRaw24 = 0U;
volatile uint32_t mainTofLastCanId = 0U;
volatile boolean  mainTofValid = FALSE;
volatile uint32_t mainTofUpdateCount = 0U;

/* HallSensor Watch 확인용 */
volatile uint32_t mainHallPulseCount = 0U;
volatile uint16_t mainHallVehicleSpeed = 0U;
volatile uint32_t mainHallUpdateCount = 0U;

volatile uint16_t mainHallAdcRaw = 0U;
volatile uint8_t  mainHallDetected = 0U;

/* 주기 송신 Watch 확인용 */
volatile uint32_t mainTxImuCount = 0U;
volatile uint32_t mainTxTofCount = 0U;
volatile uint32_t mainTxSpeedCount = 0U;

volatile uint32_t mainTxImuFailCount = 0U;
volatile uint32_t mainTxTofFailCount = 0U;
volatile uint32_t mainTxSpeedFailCount = 0U;

/*********************************************************************************************************************/
/*---------------------------------------------External variables----------------------------------------------------*/
/*********************************************************************************************************************/

/*
 * MCMCAN.c에서 0x080 VehicleState 수신 시 갱신됨.
 * Gear D일 때 TRUE.
 */
extern volatile boolean testSensorReadEnabled;

/*********************************************************************************************************************/
/*------------------------------------------------Private prototypes-------------------------------------------------*/
/*********************************************************************************************************************/

static boolean consume1msFlag(void);
static boolean consume10msFlag(void);
static boolean consume50msFlag(void);

static void task_1ms(void);
static void task_10ms(void);
static void task_50ms(void);

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
     * HallSensor pulse를 놓치지 않기 위해 1ms마다 처리
     */
    if(s_task1msFlag == FALSE)
    {
        s_task1msFlag = TRUE;
    }
    else
    {
        scheduler1msOverrunCount++;
    }

    scheduler1msCount++;

    /*
     * 10ms task flag
     */
    if((s_tick % TASK_10MS_DIV) == 0U)
    {
        if(s_task10msFlag == FALSE)
        {
            s_task10msFlag = TRUE;
        }
        else
        {
            scheduler10msOverrunCount++;
        }

        scheduler10msCount++;
    }

    /*
     * 50ms task flag
     */
    if((s_tick % TASK_50MS_DIV) == 0U)
    {
        if(s_task50msFlag == FALSE)
        {
            s_task50msFlag = TRUE;
        }
        else
        {
            scheduler50msOverrunCount++;
        }

        scheduler50msCount++;
    }
}

/*********************************************************************************************************************/
/*------------------------------------------------Initialization-----------------------------------------------------*/
/*********************************************************************************************************************/

void initScheduler(void)
{
    s_tick = 0U;

    s_task1msFlag  = FALSE;
    s_task10msFlag = FALSE;
    s_task50msFlag = FALSE;

    schedulerTickCount = 0U;

    scheduler1msCount = 0U;
    scheduler10msCount = 0U;
    scheduler50msCount = 0U;

    scheduler1msOverrunCount = 0U;
    scheduler10msOverrunCount = 0U;
    scheduler50msOverrunCount = 0U;

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
    if(consume1msFlag() == TRUE)
    {
        task_1ms();
    }

    if(consume10msFlag() == TRUE)
    {
        task_10ms();
    }

    if(consume50msFlag() == TRUE)
    {
        task_50ms();
    }
}

uint32 Scheduler_getTick(void)
{
    return s_tick;
}

/*********************************************************************************************************************/
/*------------------------------------------------Private functions--------------------------------------------------*/
/*********************************************************************************************************************/

static boolean consume1msFlag(void)
{
    boolean ret = FALSE;
    boolean interruptState;

    interruptState = IfxCpu_disableInterrupts();

    if(s_task1msFlag == TRUE)
    {
        s_task1msFlag = FALSE;
        ret = TRUE;
    }

    IfxCpu_restoreInterrupts(interruptState);

    return ret;
}

static boolean consume10msFlag(void)
{
    boolean ret = FALSE;
    boolean interruptState;

    interruptState = IfxCpu_disableInterrupts();

    if(s_task10msFlag == TRUE)
    {
        s_task10msFlag = FALSE;
        ret = TRUE;
    }

    IfxCpu_restoreInterrupts(interruptState);

    return ret;
}

static boolean consume50msFlag(void)
{
    boolean ret = FALSE;
    boolean interruptState;

    interruptState = IfxCpu_disableInterrupts();

    if(s_task50msFlag == TRUE)
    {
        s_task50msFlag = FALSE;
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
 *
 * 주의:
 *  - CAN 송신은 여기서 하지 않음.
 *  - 홀센서 펄스를 놓치지 않기 위해 1ms마다 GPIO를 읽음.
 */
static void task_1ms(void)
{
    HallSensor_update1ms();

    mainHallUpdateCount++;
    mainHallPulseCount = HallSensor_getPulseCount();
}

/**
 * @brief 10ms task
 *
 * 송신:
 *  - 0x200 ImuData
 *  - 0x201 TofDistanceData
 */
static void task_10ms(void)
{
    static uint16_t yawAngle = 0U;
    static int16_t  yawRate = 10;

    ImuData_t imuData;
    TofDistanceData_t tofData;

    /*
     * 인터페이스 정의:
     * Gear P이면 Sensor ECU는 센서값 송신 X
     */
    if(testSensorReadEnabled != TRUE)
    {
        return;
    }

    /******************************************************************************************************************
     * 0x200 ImuData
     * 주기: 10ms
     ******************************************************************************************************************/
    imuData.yawAngle = yawAngle;
    imuData.yawRate  = yawRate;

    if(CanIf_sendImuData(&imuData) == CAN_TX_OK)
    {
        mainTxImuCount++;
    }
    else
    {
        mainTxImuFailCount++;
    }

    /*
     * 현재는 테스트용 dummy yaw.
     * 나중에 실제 IMU 값으로 교체.
     */
    yawAngle += 10U;

    if(yawAngle >= 36000U)
    {
        yawAngle = 0U;
    }

    /******************************************************************************************************************
     * 0x201 TofDistanceData
     * 주기: 10ms
     *
     * 새 TOF 값이 들어왔을 때만 보내는 게 아니라,
     * 최신 거리값을 10ms마다 반복 송신한다.
     ******************************************************************************************************************/
    if(TofSensor_hasNewData() == TRUE)
    {
        mainTofUpdateCount++;
        TofSensor_clearNewDataFlag();
    }

    mainTofDistanceMm = TofSensor_getDistanceMm();
    mainTofRaw24      = TofSensor_getRaw24();
    mainTofLastCanId  = TofSensor_getLastCanId();
    mainTofValid      = TofSensor_isValid();

    if(mainTofValid == TRUE)
    {
        tofData.distanceMm = mainTofDistanceMm;
    }
    else
    {
        tofData.distanceMm = TOF_DISTANCE_INVALID_MM;
    }

    if(CanIf_sendTofDistanceData(&tofData) == CAN_TX_OK)
    {
        mainTxTofCount++;
    }
    else
    {
        mainTxTofFailCount++;
    }
}

/**
 * @brief 50ms task
 *
 * 송신:
 *  - 0x202 SpeedData
 */
static void task_50ms(void)
{
    SpeedData_t speedData;

    /*
     * 50ms마다 Hall pulse 기반 속도 계산.
     * Gear P라서 송신하지 않더라도 계산은 계속 해둔다.
     * 그래야 P 상태에서 쌓인 pulse가 D 전환 순간에 한 번에 반영되는 문제를 줄일 수 있다.
     */
    HallSensor_calcSpeed50ms();

    mainHallVehicleSpeed = HallSensor_getVehicleSpeed();

    /*
     * 인터페이스 정의:
     * Gear P이면 Sensor ECU는 센서값 송신 X
     */
    if(testSensorReadEnabled != TRUE)
    {
        return;
    }

    /******************************************************************************************************************
     * 0x202 SpeedData
     * 주기: 50ms
     ******************************************************************************************************************/
    speedData.vehicleSpeed = mainHallVehicleSpeed;

    if(CanIf_sendSpeedData(&speedData) == CAN_TX_OK)
    {
        mainTxSpeedCount++;
    }
    else
    {
        mainTxSpeedFailCount++;
    }
}
