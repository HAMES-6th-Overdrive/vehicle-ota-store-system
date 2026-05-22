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
 * 바퀴에 자석 2개 부착 기준
 * 바퀴 1회전 = pulse 2개
 */
#define HALL_MAGNETS_PER_REV            (2U)

/*
 * 바퀴 둘레 [mm]
 * 73cm = 730mm
 */
#define HALL_WHEEL_CIRCUMFERENCE_MM     (730U)

/*
 * Scheduler.c의 10ms task에서 HallSensor_calcSpeed10ms() 호출
 * 이 값은 timeout/debug 기준으로만 사용하고,
 * 실제 속도는 pulse interval 기반으로 계산한다.
 */
#define HALL_SPEED_CALC_PERIOD_MS       (10U)

/*
 * IIR filter alpha = 1 / 4
 * filtered = filtered + (raw - filtered) / 4
 */
#define HALL_SPEED_FILTER_DIV           (4)

/*
 * 너무 짧은 펄스 간격은 노이즈로 무시
 * 10ms면 이론상 매우 높은 속도까지 허용하므로 일반 주행에서는 충분히 안전.
 */
#define HALL_MIN_PULSE_INTERVAL_MS      (10U)

/*
 * 마지막 펄스 이후 이 시간 이상 새 펄스가 없으면 정지로 판단
 * 1500ms 기준:
 * speedX100 = 730 * 360 / (2 * 1500) = 87.6
 * 즉 약 0.88km/h 이하에서는 0으로 떨어질 수 있음.
 */
#define HALL_NO_PULSE_TIMEOUT_MS        (1500U)

/*********************************************************************************************************************/
/*------------------------------------------------Static variables---------------------------------------------------*/
/*********************************************************************************************************************/

static volatile uint8_t  s_detected = 0U;
static volatile uint32_t s_pulseCount = 0U;
static volatile uint16_t s_vehicleSpeed = 0U;

static uint8_t  s_prevDetected = 0U;
static uint16_t s_filteredVehicleSpeed = 0U;

/*
 * HallSensor_update1ms()가 1ms마다 호출된다는 전제의 소프트웨어 시간
 */
static uint32_t s_timeMs = 0U;

/*
 * 펄스 간 시간 기반 속도 계산용
 */
static uint32_t s_lastPulseTimeMs = 0U;
static uint32_t s_lastPulseIntervalMs = 0U;
static uint8_t  s_hasValidInterval = 0U;

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
 *
 * debugHallRawVehicleSpeed:
 *   필터 전 속도, 단위 km/h x100
 *
 * debugHallFilteredVehicleSpeed:
 *   최종 필터 후 속도, 단위 km/h x100
 */
volatile uint8_t  debugHallRawLevel = 0U;
volatile uint8_t  debugHallDetected = 0U;
volatile uint16_t debugHallRawVehicleSpeed = 0U;
volatile uint16_t debugHallFilteredVehicleSpeed = 0U;

/*
 * Period 방식 확인용 debug
 */
volatile uint32_t debugHallTimeMs = 0U;
volatile uint32_t debugHallLastPulseTimeMs = 0U;
volatile uint32_t debugHallLastPulseIntervalMs = 0U;
volatile uint32_t debugHallPulseAgeMs = 0U;
volatile uint8_t  debugHallHasValidInterval = 0U;
volatile uint32_t debugHallIgnoredPulseCount = 0U;

/*********************************************************************************************************************/
/*------------------------------------------------Private prototypes-------------------------------------------------*/
/*********************************************************************************************************************/

static uint8_t HallSensor_readRawLevel(void);
static uint8_t HallSensor_convertRawToDetected(uint8_t rawLevel);
static void    HallSensor_updateDebug(uint8_t rawLevel, uint8_t detected);

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
                            IfxPort_InputMode_pullUp);

    s_detected = 0U;
    s_pulseCount = 0U;
    s_vehicleSpeed = 0U;

    s_prevDetected = 0U;
    s_filteredVehicleSpeed = 0U;

    s_timeMs = 0U;
    s_lastPulseTimeMs = 0U;
    s_lastPulseIntervalMs = 0U;
    s_hasValidInterval = 0U;

    debugHallRawLevel = 0U;
    debugHallDetected = 0U;
    debugHallRawVehicleSpeed = 0U;
    debugHallFilteredVehicleSpeed = 0U;

    debugHallTimeMs = 0U;
    debugHallLastPulseTimeMs = 0U;
    debugHallLastPulseIntervalMs = 0U;
    debugHallPulseAgeMs = 0U;
    debugHallHasValidInterval = 0U;
    debugHallIgnoredPulseCount = 0U;
}

uint8_t HallSensor_isDetected(void)
{
    uint8_t rawLevel;
    uint8_t detected;

    rawLevel = HallSensor_readRawLevel();
    detected = HallSensor_convertRawToDetected(rawLevel);

    HallSensor_updateDebug(rawLevel, detected);

    return detected;
}

void HallSensor_update1ms(void)
{
    uint8_t rawLevel;
    uint8_t nowDetected;

    /*
     * 이 함수가 1ms마다 호출된다는 전제.
     */
    s_timeMs++;
    debugHallTimeMs = s_timeMs;

    rawLevel = HallSensor_readRawLevel();
    nowDetected = HallSensor_convertRawToDetected(rawLevel);

    HallSensor_updateDebug(rawLevel, nowDetected);

    s_detected = nowDetected;

    /*
     * 자석이 처음 가까워지는 순간만 pulse 1개 증가.
     *
     * 즉,
     * 자석 멀어짐 -> 자석 가까움
     * 0 -> 1 전이에서만 count 증가.
     */
    if((s_prevDetected == 0U) && (nowDetected == 1U))
    {
        uint32_t nowMs;
        uint32_t intervalMs;

        nowMs = s_timeMs;

        /*
         * 첫 번째 펄스는 기준 시간이 없으므로 interval 계산 불가.
         * pulseCount만 올리고 기준 시각 저장.
         */
        if(s_lastPulseTimeMs == 0U)
        {
            s_pulseCount++;
            s_lastPulseTimeMs = nowMs;

            debugHallLastPulseTimeMs = s_lastPulseTimeMs;
        }
        else
        {
            intervalMs = nowMs - s_lastPulseTimeMs;

            /*
             * 너무 짧은 interval은 노이즈/글리치로 보고 무시.
             */
            if(intervalMs >= HALL_MIN_PULSE_INTERVAL_MS)
            {
                s_pulseCount++;

                s_lastPulseIntervalMs = intervalMs;
                s_lastPulseTimeMs = nowMs;
                s_hasValidInterval = 1U;

                debugHallLastPulseIntervalMs = s_lastPulseIntervalMs;
                debugHallLastPulseTimeMs = s_lastPulseTimeMs;
                debugHallHasValidInterval = s_hasValidInterval;
            }
            else
            {
                debugHallIgnoredPulseCount++;
            }
        }
    }

    s_prevDetected = nowDetected;
}

void HallSensor_calcSpeed10ms(void)
{
    uint32_t rawSpeedX100;
    uint32_t denominator;
    uint32_t pulseAgeMs;
    int32_t  speedDelta;
    int32_t  filterStep;
    int32_t  filteredSpeed;

    rawSpeedX100 = 0U;
    pulseAgeMs = 0U;

    /*
     * 아직 유효한 펄스 간격이 없으면 속도 계산 불가.
     * 첫 펄스만 들어온 상태에서는 0으로 둔다.
     */
    if(s_hasValidInterval == 0U)
    {
        rawSpeedX100 = 0U;
        pulseAgeMs = 0U;
    }
    else
    {
        pulseAgeMs = s_timeMs - s_lastPulseTimeMs;

        /*
         * 마지막 펄스 이후 일정 시간 이상 새 펄스가 없으면 정지로 판단.
         */
        if(pulseAgeMs > HALL_NO_PULSE_TIMEOUT_MS)
        {
            rawSpeedX100 = 0U;
        }
        else
        {
            /*
             * vehicleSpeed 단위: km/h x100
             *
             * pulseIntervalMs는 자석 하나와 다음 자석 사이 시간.
             *
             * 한 펄스당 이동거리 = wheel_circumference_mm / magnets
             *
             * speed[km/h] x100
             * = wheel_circumference_mm * 360 / (magnets * pulseIntervalMs)
             */
            denominator = HALL_MAGNETS_PER_REV * s_lastPulseIntervalMs;

            if(denominator == 0U)
            {
                rawSpeedX100 = 0U;
            }
            else
            {
                rawSpeedX100 =
                    (HALL_WHEEL_CIRCUMFERENCE_MM * 360U) / denominator;
            }
        }
    }

    if(rawSpeedX100 > 0xFFFFU)
    {
        rawSpeedX100 = 0xFFFFU;
    }

    debugHallRawVehicleSpeed = (uint16_t)rawSpeedX100;
    debugHallPulseAgeMs = pulseAgeMs;

    /*
     * IIR filter
     * filtered = filtered + (raw - filtered) / HALL_SPEED_FILTER_DIV
     */
    speedDelta = (int32_t)rawSpeedX100 - (int32_t)s_filteredVehicleSpeed;
    filterStep = speedDelta / HALL_SPEED_FILTER_DIV;

    /*
     * 차이가 있는데 정수 나눗셈 때문에 0이 되어버리면
     * 최소 1씩은 따라가게 만든다.
     */
    if((filterStep == 0) && (speedDelta != 0))
    {
        filterStep = (speedDelta > 0) ? 1 : -1;
    }

    filteredSpeed = (int32_t)s_filteredVehicleSpeed + filterStep;

    if(filteredSpeed < 0)
    {
        filteredSpeed = 0;
    }
    else if(filteredSpeed > 0xFFFF)
    {
        filteredSpeed = 0xFFFF;
    }

    s_filteredVehicleSpeed = (uint16_t)filteredSpeed;
    s_vehicleSpeed = s_filteredVehicleSpeed;

    debugHallFilteredVehicleSpeed = s_vehicleSpeed;
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
    s_filteredVehicleSpeed = 0U;

    s_timeMs = 0U;
    s_lastPulseTimeMs = 0U;
    s_lastPulseIntervalMs = 0U;
    s_hasValidInterval = 0U;

    debugHallRawLevel = 0U;
    debugHallDetected = 0U;
    debugHallRawVehicleSpeed = 0U;
    debugHallFilteredVehicleSpeed = 0U;

    debugHallTimeMs = 0U;
    debugHallLastPulseTimeMs = 0U;
    debugHallLastPulseIntervalMs = 0U;
    debugHallPulseAgeMs = 0U;
    debugHallHasValidInterval = 0U;
    debugHallIgnoredPulseCount = 0U;
}

/*********************************************************************************************************************/
/*------------------------------------------------Private functions--------------------------------------------------*/
/*********************************************************************************************************************/

static uint8_t HallSensor_readRawLevel(void)
{
    IfxPort_State state;

    state = IfxPort_getPinState(HALL_PORT, HALL_PIN_INDEX);

    if(state == IfxPort_State_high)
    {
        return 1U;
    }
    else
    {
        return 0U;
    }
}

static uint8_t HallSensor_convertRawToDetected(uint8_t rawLevel)
{
    /*
     * Active Low:
     * raw 0 -> 자석 감지
     * raw 1 -> 자석 미감지
     */
    return (rawLevel == 0U) ? 1U : 0U;
}

static void HallSensor_updateDebug(uint8_t rawLevel, uint8_t detected)
{
    debugHallRawLevel = rawLevel;
    debugHallDetected = detected;
}
