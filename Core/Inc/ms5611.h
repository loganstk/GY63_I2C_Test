/*
 * ms5611.h
 *
 *  Created on: 10 січ. 2021 р.
 *      Author: logan
 */
#include "stm32f4xx_hal.h"

#ifndef INC_MS5611_H_
#define INC_MS5611_H_

extern I2C_HandleTypeDef hi2c1;

extern void MS5611_Init(void);
extern int32_t MS5611_ReadCompensatedTemperature(void);
extern int32_t MS5611_ReadCompensatedPressure(void);

#endif /* INC_MS5611_H_ */

