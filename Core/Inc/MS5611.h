/*
 * ms5611.h
 *
 *  Created on: 10 січ. 2021 р.
 *      Author: logan
 */
#include "stm32f4xx_hal.h"

#ifndef INC_MS5611_H_
#define INC_MS5611_H_

#define MS5611_ADDR_1  (0x77 << 1)
#define MS5611_ADDR_2  (0x76 << 1)

class MS5611 {
	public:
		MS5611(I2C_HandleTypeDef hi2c, uint16_t address);
		float GetTemperature(void);
		float GetPressure(void);

	private:
		I2C_HandleTypeDef _hi2c;
		uint16_t _i2cAddress;
		uint16_t _promData[8];

		void Init(void);
		void WriteCommand(uint8_t command);
		uint16_t ReadProm(uint8_t address);
		uint32_t ReadADC(void);
		uint32_t ReadRawTemperature(void);
		uint32_t ReadRawPressure(void);
};

#endif /* INC_MS5611_H_ */

