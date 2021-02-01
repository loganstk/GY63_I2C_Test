/*
 * ms5611.cpp
 *
 *  Created on: 10 січ. 2021 р.
 *      Author: logan
 */
#include "stm32f4xx_hal.h"
#include "MS5611.h"

#define MS5611_PROM_SIZE        8
#define MS5611_CMD_RESET        0x1E
#define MS5611_CMD_PROM_RD      0xA0
#define MS5611_CMD_CONV_PRES    0x48
#define MS5611_CMD_CONV_TEMP    0x58
#define MS5611_CMD_ADC_RD       0x00


MS5611::MS5611(I2C_HandleTypeDef hi2c, uint16_t i2cAddress) {
	_hi2c = hi2c;
	_i2cAddress = i2cAddress;
	Init();
}

void MS5611::Init(void) {
	WriteCommand(MS5611_CMD_RESET);
	for (uint8_t i = 0; i < MS5611_PROM_SIZE; i++) {
		_promData[i] = ReadProm(i * 2);
	}
}

void MS5611::WriteCommand(uint8_t command) {
	HAL_I2C_Master_Transmit(&_hi2c, _i2cAddress, &command, 1, HAL_MAX_DELAY);
}

uint16_t MS5611::ReadProm(uint8_t address) {
	uint8_t prom_rd_addr = MS5611_CMD_PROM_RD | address;
	uint8_t bytes[2];

	WriteCommand(prom_rd_addr);
	HAL_I2C_Master_Receive(&_hi2c, _i2cAddress, bytes, 2, HAL_MAX_DELAY);

	return ((uint16_t) bytes[0] << 8) | bytes[1];
}

uint32_t MS5611::ReadADC(void) {
	uint8_t addr = MS5611_CMD_ADC_RD;
	uint8_t adc_bytes[3];

	HAL_I2C_Master_Transmit(&_hi2c, _i2cAddress, &addr, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&_hi2c, _i2cAddress, adc_bytes, 3, HAL_MAX_DELAY);

	return ((uint32_t) adc_bytes[0] << 16) | ((uint32_t) adc_bytes[1] << 8)
			| ((uint32_t) adc_bytes[2]);
}

uint32_t MS5611::ReadRawTemperature(void) {
	WriteCommand(MS5611_CMD_CONV_TEMP);
	HAL_Delay(100); // TODO: poll the bus instead of delay
	uint32_t d2 = ReadADC();
	return d2;
}

uint32_t MS5611::ReadRawPressure(void) {
	WriteCommand(MS5611_CMD_CONV_PRES);
	HAL_Delay(100); // TODO: poll the bus instead of delay
	uint32_t d1 = ReadADC();
	return d1;
}

float MS5611::GetTemperature(void) {
	uint32_t d2 = ReadRawTemperature();
	int32_t delta_t = d2 - ((uint32_t) _promData[5] << 8);
	int32_t temp = 2000 + ((int64_t) delta_t * _promData[6] >> 23);
	return temp / 100.0f;
}

float MS5611::GetPressure(void) {
	uint32_t d1 = ReadRawPressure();
	uint32_t d2 = ReadRawTemperature();

	int32_t delta_t = d2 - ((uint32_t) _promData[5] << 8);

	int64_t offset = ((int64_t) _promData[2] << 16)
			+ (((int64_t) _promData[4] * delta_t) >> 7);
	int64_t sens = ((int64_t) _promData[1] << 15)
			+ (((int64_t) _promData[3] * delta_t) >> 8);
	int32_t pres = ((((int64_t) d1 * sens) >> 21) - offset) >> 15;

	return pres / 100.0f;
}

