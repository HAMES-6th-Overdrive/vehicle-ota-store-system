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
 * Wheel has 2 magnets.
 * 1 wheel revolution = 2 pulses.
 */
#define HALL_MAGNETS_PER_REV            (2U)

/*
 * Wheel circumference [mm]
 * 73cm = 730mm
 */
#define HALL_WHEEL_CIRCUMFERENCE_MM     (730U)

/*
 * Scheduler.c calls HallSensor_updateMs(1U) from the 1ms task.
 * Speed is calculated immediately on a valid pulse edge.
 */
#define HALL_SPEED_CALC_PERIOD_MS       (1U)

/*
 * Ignore too-short pulse intervals as noise/glitches.
 */
#define HALL_MIN_PULSE_INTERVAL_MS      (10U)

/*
 * If no new pulse arrives for this time, treat the vehicle as stopped.
 * With 2 magnets and 3000ms timeout:
 * speedX100 = 730 * 360 / (2 * 3000) = 43.8
 * This keeps speed down to about 0.44km/h before dropping to zero.
 */
#define HALL_NO_PULSE_TIMEOUT_MS        (3000U)

/*********************************************************************************************************************/
/*------------------------------------------------Static variables---------------------------------------------------*/
/*********************************************************************************************************************/

static volatile uint8_t  s_detected = 0U;
static volatile uint32_t s_pulseCount = 0U;
static volatile uint16_t s_vehicleSpeed = 0U;

static uint8_t  s_prevDetected = 0U;
static uint8_t  s_hasPulseBase = 0U;
static uint8_t  s_hasValidInterval = 0U;

/*
 * Software time advanced by HallSensor_updateMs().
 */
static uint32_t s_timeMs = 0U;

/*
 * Pulse interval based speed calculation state.
 */
static uint32_t s_lastPulseTimeMs = 0U;
static uint32_t s_lastPulseIntervalMs = 0U;

/*********************************************************************************************************************/
/*------------------------------------------------Debug variables----------------------------------------------------*/
/*********************************************************************************************************************/

volatile uint8_t  debugHallRawLevel = 0U;
volatile uint8_t  debugHallDetected = 0U;
volatile uint32_t debugHallTimeMs = 0U;
volatile uint32_t debugHallPulseCount = 0U;
volatile uint32_t debugHallLastPulseTimeMs = 0U;
volatile uint32_t debugHallLastPulseIntervalMs = 0U;
volatile uint32_t debugHallPulseAgeMs = 0U;
volatile uint16_t debugHallVehicleSpeedX100 = 0U;
volatile uint32_t debugHallIgnoredPulseCount = 0U;
volatile uint8_t  debugHallHasSpeed = 0U;

/*
 * Legacy watch variables. Kept for existing debugger watch setups.
 */
volatile uint16_t debugHallRawVehicleSpeed = 0U;
volatile uint16_t debugHallFilteredVehicleSpeed = 0U;
volatile uint8_t  debugHallHasValidInterval = 0U;

/*********************************************************************************************************************/
/*------------------------------------------------Private prototypes-------------------------------------------------*/
/*********************************************************************************************************************/

static uint8_t  HallSensor_readRawLevel(void);
static uint8_t  HallSensor_convertRawToDetected(uint8_t rawLevel);
static void     HallSensor_updateDebug(uint8_t rawLevel, uint8_t detected);
static uint16_t HallSensor_calcSpeedX100(uint32_t intervalMs);
static void     HallSensor_updatePulseAgeAndTimeout(void);
static void     HallSensor_storeSpeed(uint16_t speedX100);

/*********************************************************************************************************************/
/*------------------------------------------------Public functions---------------------------------------------------*/
/*********************************************************************************************************************/

void HallSensor_init(void)
{
    /*
     * DM2246 D0 input.
     * Existing hardware uses active-low output for magnet detection.
     */
    IfxPort_setPinModeInput(HALL_PORT,
                            HALL_PIN_INDEX,
                            IfxPort_InputMode_pullUp);

    HallSensor_reset();
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

void HallSensor_updateMs(uint32_t periodMs)
{
    uint8_t rawLevel;
    uint8_t nowDetected;

    if(periodMs == 0U)
    {
        periodMs = 1U;
    }

    s_timeMs += periodMs;
    debugHallTimeMs = s_timeMs;

    rawLevel = HallSensor_readRawLevel();
    nowDetected = HallSensor_convertRawToDetected(rawLevel);

    HallSensor_updateDebug(rawLevel, nowDetected);

    s_detected = nowDetected;

    /*
     * Count only the transition from not-detected to detected.
     */
    if((s_prevDetected == 0U) && (nowDetected == 1U))
    {
        uint32_t nowMs;
        uint32_t intervalMs;

        nowMs = s_timeMs;

        if(s_hasPulseBase == 0U)
        {
            s_pulseCount++;
            s_hasPulseBase = 1U;
            s_lastPulseTimeMs = nowMs;

            debugHallPulseCount = s_pulseCount;
            debugHallLastPulseTimeMs = s_lastPulseTimeMs;
        }
        else
        {
            intervalMs = nowMs - s_lastPulseTimeMs;

            if(intervalMs >= HALL_MIN_PULSE_INTERVAL_MS)
            {
                uint16_t speedX100;

                s_pulseCount++;
                s_lastPulseIntervalMs = intervalMs;
                s_lastPulseTimeMs = nowMs;
                s_hasValidInterval = 1U;

                speedX100 = HallSensor_calcSpeedX100(intervalMs);
                HallSensor_storeSpeed(speedX100);

                debugHallPulseCount = s_pulseCount;
                debugHallLastPulseIntervalMs = s_lastPulseIntervalMs;
                debugHallLastPulseTimeMs = s_lastPulseTimeMs;
                debugHallPulseAgeMs = 0U;
                debugHallHasSpeed = 1U;
                debugHallHasValidInterval = s_hasValidInterval;
            }
            else
            {
                debugHallIgnoredPulseCount++;
            }
        }
    }

    s_prevDetected = nowDetected;

    HallSensor_updatePulseAgeAndTimeout();
}

void HallSensor_update1ms(void)
{
    HallSensor_updateMs(1U);
}

void HallSensor_calcSpeed10ms(void)
{
    /*
     * Deprecated compatibility wrapper.
     * Speed is updated immediately in HallSensor_updateMs().
     */
    HallSensor_updatePulseAgeAndTimeout();
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
    s_hasPulseBase = 0U;
    s_hasValidInterval = 0U;

    s_timeMs = 0U;
    s_lastPulseTimeMs = 0U;
    s_lastPulseIntervalMs = 0U;

    debugHallRawLevel = 0U;
    debugHallDetected = 0U;
    debugHallTimeMs = 0U;
    debugHallPulseCount = 0U;
    debugHallLastPulseTimeMs = 0U;
    debugHallLastPulseIntervalMs = 0U;
    debugHallPulseAgeMs = 0U;
    debugHallVehicleSpeedX100 = 0U;
    debugHallIgnoredPulseCount = 0U;
    debugHallHasSpeed = 0U;

    debugHallRawVehicleSpeed = 0U;
    debugHallFilteredVehicleSpeed = 0U;
    debugHallHasValidInterval = 0U;
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
     * Active low:
     * raw 0 -> magnet detected
     * raw 1 -> magnet not detected
     */
    return (rawLevel == 0U) ? 1U : 0U;
}

static void HallSensor_updateDebug(uint8_t rawLevel, uint8_t detected)
{
    debugHallRawLevel = rawLevel;
    debugHallDetected = detected;
}

static uint16_t HallSensor_calcSpeedX100(uint32_t intervalMs)
{
    uint32_t denominator;
    uint32_t speedX100;

    denominator = HALL_MAGNETS_PER_REV * intervalMs;

    if(denominator == 0U)
    {
        return 0U;
    }

    speedX100 = (HALL_WHEEL_CIRCUMFERENCE_MM * 360U) / denominator;

    if(speedX100 > 0xFFFFU)
    {
        speedX100 = 0xFFFFU;
    }

    return (uint16_t)speedX100;
}

static void HallSensor_updatePulseAgeAndTimeout(void)
{
    uint32_t pulseAgeMs;

    if(s_hasPulseBase == 0U)
    {
        debugHallPulseAgeMs = 0U;
        return;
    }

    pulseAgeMs = s_timeMs - s_lastPulseTimeMs;
    debugHallPulseAgeMs = pulseAgeMs;

    if((s_hasValidInterval != 0U) &&
       (pulseAgeMs > HALL_NO_PULSE_TIMEOUT_MS))
    {
        HallSensor_storeSpeed(0U);

        /*
         * After timeout, the next pulse becomes a new baseline.
         * This prevents a restart from using the long stopped interval.
         */
        s_hasPulseBase = 0U;
        s_hasValidInterval = 0U;
        debugHallHasSpeed = 0U;
        debugHallHasValidInterval = 0U;
    }
}

static void HallSensor_storeSpeed(uint16_t speedX100)
{
    s_vehicleSpeed = speedX100;

    debugHallVehicleSpeedX100 = speedX100;

    /*
     * Legacy aliases.
     */
    debugHallRawVehicleSpeed = speedX100;
    debugHallFilteredVehicleSpeed = speedX100;
}
