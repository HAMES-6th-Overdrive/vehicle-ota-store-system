#include "TofSensor.h"
#include <string.h>

static volatile uint32_t s_lastCanId = 0U;
static volatile uint32_t s_tofRaw24 = 0U;
static volatile uint16_t s_distanceMm = TOF_DISTANCE_INVALID_MM;
static volatile uint16_t s_signalStrength = 0U;
static volatile boolean  s_valid = FALSE;
static volatile boolean  s_newData = FALSE;

void TofSensor_init(void)
{
    s_lastCanId = 0U;
    s_tofRaw24 = 0U;
    s_distanceMm = TOF_DISTANCE_INVALID_MM;
    s_signalStrength = 0U;
    s_valid = FALSE;
    s_newData = FALSE;
}

void TofSensor_onCanFrame(uint32 id, const uint8_t *data, uint8 length)
{
    uint32 tofRaw = 0U;
    uint16 signalStrength = 0U;
    boolean valid = FALSE;

    s_lastCanId = id;

    /*
     * 포맷 1:
     * data[0]~data[2] = TOF 24bit little-endian
     * data[4]~data[5] = signal strength
     */
    if(length >= 6U)
    {
        tofRaw = ((uint32)data[2] << 16) |
                 ((uint32)data[1] << 8)  |
                 ((uint32)data[0]);

        signalStrength = ((uint16)data[5] << 8) |
                          (uint16)data[4];

        if(signalStrength != 0U)
        {
            valid = TRUE;
        }
    }

    /*
     * 포맷 2:
     * 기존 진단형 프레임 흔적
     * id >= 0x700
     * data[1] = SID = 0x2A
     * data[2]~data[3] = DID = 1
     * data[4]~data[6] = TOF 24bit big-endian
     */
    if((id >= 0x700U) &&
       (length >= 7U) &&
       (data[1] == 0x2AU) &&
       ((((uint16)data[2] << 8) | data[3]) == 0x0001U))
    {
        tofRaw = ((uint32)data[4] << 16) |
                 ((uint32)data[5] << 8)  |
                 ((uint32)data[6]);

        valid = TRUE;
    }

    s_tofRaw24 = tofRaw;
    s_signalStrength = signalStrength;

    if((valid == TRUE) && (tofRaw <= 0xFFFFU))
    {
        s_distanceMm = (uint16_t)tofRaw;
        s_valid = TRUE;
    }
    else
    {
        s_distanceMm = TOF_DISTANCE_INVALID_MM;
        s_valid = FALSE;
    }

    s_newData = TRUE;
}

boolean TofSensor_isValid(void)
{
    return s_valid;
}

boolean TofSensor_hasNewData(void)
{
    return s_newData;
}

uint16_t TofSensor_getDistanceMm(void)
{
    return s_distanceMm;
}

uint32_t TofSensor_getRaw24(void)
{
    return s_tofRaw24;
}

uint32_t TofSensor_getLastCanId(void)
{
    return s_lastCanId;
}

void TofSensor_clearNewDataFlag(void)
{
    s_newData = FALSE;
}
