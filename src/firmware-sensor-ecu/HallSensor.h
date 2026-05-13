#ifndef HALLSENSOR_H_
#define HALLSENSOR_H_

#include "Ifx_Types.h"
#include <stdint.h>

void HallSensor_init(void);
void HallSensor_update1ms(void);
void HallSensor_calcSpeed50ms(void);

uint8_t  HallSensor_isDetected(void);
uint32_t HallSensor_getPulseCount(void);
uint16_t HallSensor_getVehicleSpeed(void);

void HallSensor_reset(void);

#endif /* HALLSENSOR_H_ */
