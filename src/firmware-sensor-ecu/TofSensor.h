#ifndef TOF_SENSOR_H_
#define TOF_SENSOR_H_

#include <stdint.h>
#include "Ifx_Types.h"

#define TOF_DISTANCE_INVALID_MM    0xFFFFU

void TofSensor_init(void);

void TofSensor_onCanFrame(uint32 id, const uint8_t *data, uint8 length);

boolean  TofSensor_isValid(void);
boolean  TofSensor_hasNewData(void);
uint16_t TofSensor_getDistanceMm(void);
uint32_t TofSensor_getRaw24(void);
uint32_t TofSensor_getLastCanId(void);

void TofSensor_clearNewDataFlag(void);

#endif /* TOF_SENSOR_H_ */
