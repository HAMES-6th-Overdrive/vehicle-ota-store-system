#include "HallSensor.h"
#include "IfxPort.h"

/*********************************************************************************************************************/
/*------------------------------------------------Configuration------------------------------------------------------*/
/*********************************************************************************************************************/

/*
 * DM2246 D0 -> TC375 P02.7
 */
#define HALL_PORT                       (&MODULE_P02)
#define HALL_PIN_INDEX                  (7U)

/*
 * 바퀴에 자석 3개 부착 기준
 * 바퀴 1회전 = pulse 3개
 */
#define HALL_MAGNETS_PER_REV            (3U)

/*
 * 바퀴 둘레 [mm]
 * 실제 바퀴 지름 재서 수정해야 함.
 *
 * 예:
 * 바퀴 지름 70mm라면
 * 둘레 = 70 * 3.14 = 약 220mm
 */
#define HALL_WHEEL_CIRCUMFERENCE_MM     (200U)

/*
 * 네가 확인한 D0 상태:
 *
 * 자석 가까움 -> P02.7 LOW  = 0
 * 자석 멀어짐 -> P02.7 HIGH = 1
 *
 * 따라서 Active Low가 맞음.
 */
#define HALL_ACTIVE_LOW                 (1U)

/*********************************************************************************************************************/
/*------------------------------------------------Static variables---------------------------------------------------*/
/*********************************************************************************************************************/

static volatile uint8_t  s_detected = 0U;
static volatile uint32_t s_pulseCount = 0U;
static volatile uint16_t s_vehicleSpeed = 0U;

static uint8_t  s_prevDetected = 0U;
static uint32_t s_prevPulseCountForSpeed = 0U;

/*********************************************************************************************************************/
/*------------------------------------------------Debug variables----------------------------------------------------*/
/*********************************************************************************************************************/

/*
 * Watch 확인용
 *
 * debugHallRawLevel:
 *   자석 가까움 -> 0
 *   자석 멀어짐 -> 1
 *
 * debugHallDetected:
 *   자석 감지   -> 1
 *   자석 미감지 -> 0
 */
volatile uint8_t debugHallRawLevel = 0U;
volatile uint8_t debugHallDetected = 0U;

/*********************************************************************************************************************/
/*------------------------------------------------Public functions---------------------------------------------------*/
/*********************************************************************************************************************/

void HallSensor_init(void)
{
    /*
     * DM2246 D0를 P02.7로 입력.
     *
     * D0가 전압분배/레벨시프터를 거쳐 3.3V 레벨로 들어온다는 기준.
     * 외부 회로가 신호 레벨을 만들고 있으므로 내부 pull-up은 끔.
     */
    IfxPort_setPinModeInput(HALL_PORT,
                            HALL_PIN_INDEX,
                            IfxPort_InputMode_noPullDevice);

    s_detected = 0U;
    s_pulseCount = 0U;
    s_vehicleSpeed = 0U;

    s_prevDetected = 0U;
    s_prevPulseCountForSpeed = 0U;

    debugHallRawLevel = 0U;
    debugHallDetected = 0U;
}

uint8_t HallSensor_isDetected(void)
{
    IfxPort_State state;

    state = IfxPort_getPinState(HALL_PORT, HALL_PIN_INDEX);

    /*
     * raw level 저장
     */
    if(state == IfxPort_State_high)
    {
        debugHallRawLevel = 1U;
    }
    else
    {
        debugHallRawLevel = 0U;
    }

#if HALL_ACTIVE_LOW
    /*
     * 자석 가까움: LOW -> detected 1
     * 자석 멀어짐: HIGH -> detected 0
     */
    if(state == IfxPort_State_low)
    {
        return 1U;
    }
    else
    {
        return 0U;
    }
#else
    if(state == IfxPort_State_high)
    {
        return 1U;
    }
    else
    {
        return 0U;
    }
#endif
}

void HallSensor_update1ms(void)
{
    IfxPort_State state;
    uint8_t nowDetected;

    state = IfxPort_getPinState(HALL_PORT, HALL_PIN_INDEX);

    /*
     * 실제 P02.7 입력 레벨
     * 자석 가까움: 0
     * 자석 멀음  : 1
     */
    if(state == IfxPort_State_high)
    {
        debugHallRawLevel = 1U;
    }
    else
    {
        debugHallRawLevel = 0U;
    }

    /*
     * 네 센서는 active-low로 확인됨.
     * 자석 가까움: raw 0 -> detected 1
     * 자석 멀음  : raw 1 -> detected 0
     */
    if(debugHallRawLevel == 0U)
    {
        nowDetected = 1U;
    }
    else
    {
        nowDetected = 0U;
    }

    debugHallDetected = nowDetected;
    s_detected = nowDetected;

    /*
     * 자석이 처음 가까워지는 순간만 pulse 1개 증가
     */
    if((s_prevDetected == 0U) && (nowDetected == 1U))
    {
        s_pulseCount++;
    }

    s_prevDetected = nowDetected;
}

void HallSensor_calcSpeed50ms(void)
{
    uint32_t nowPulseCount;
    uint32_t diffPulse;
    uint32_t numerator;
    uint32_t denominator;
    uint32_t speedX10;

    nowPulseCount = s_pulseCount;
    diffPulse = nowPulseCount - s_prevPulseCountForSpeed;
    s_prevPulseCountForSpeed = nowPulseCount;

    /*
     * vehicleSpeed 단위: km/h x10
     *
     * 50ms 동안 diffPulse개 들어왔다고 할 때:
     *
     * 바퀴 회전수 = diffPulse / 자석 개수
     * 이동거리[mm] = 바퀴 회전수 * 바퀴둘레[mm]
     *
     * speed[km/h] x10
     * = diffPulse * wheel_circumference_mm * 36 / (period_ms * magnets)
     *
     * period_ms = 50ms
     */
    numerator = diffPulse * HALL_WHEEL_CIRCUMFERENCE_MM * 36U;
    denominator = 50U * HALL_MAGNETS_PER_REV;

    if(denominator == 0U)
    {
        s_vehicleSpeed = 0U;
        return;
    }

    speedX10 = numerator / denominator;

    if(speedX10 > 0xFFFFU)
    {
        speedX10 = 0xFFFFU;
    }

    s_vehicleSpeed = (uint16_t)speedX10;
}

uint32_t HallSensor_getPulseCount(void)
{
    return s_pulseCount;
}

uint16_t HallSensor_getVehicleSpeed(void)
{
    return s_vehicleSpeed;
}

void HallSensor_reset(void)
{
    s_detected = 0U;
    s_pulseCount = 0U;
    s_vehicleSpeed = 0U;

    s_prevDetected = 0U;
    s_prevPulseCountForSpeed = 0U;

    debugHallRawLevel = 0U;
    debugHallDetected = 0U;
}
