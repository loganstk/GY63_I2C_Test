/*
 * ms5611.c
 *
 *  Created on: 10 січ. 2021 р.
 *      Author: logan
 */
#include "stm32f4xx_hal.h"

#define MS5611_PROM_SIZE        8
#define MS5611_ADDR             (0x77 << 1)
#define MS5611_CMD_RESET        0x1E
#define MS5611_CMD_PROM_RD      0xA0
#define MS5611_CMD_CONV_PRES    0x48
#define MS5611_CMD_CONV_TEMP    0x58
#define MS5611_CMD_ADC_RD       0x00


extern I2C_HandleTypeDef hi2c1;
static uint16_t prom_data[8];

void MS5611_Init(void);
static void MS5611_WriteCommand(uint8_t command);
static uint16_t MS5611_ReadProm(uint8_t address);
static uint32_t MS5611_ReadADC(void);
static uint32_t MS5611_ReadRawTemperature(void);
static uint32_t MS5611_ReadRawPressure(void);
int32_t MS5611_ReadCompensatedTemperature(void);
int32_t MS5611_ReadCompensatedPressure(void);

void MS5611_Init(void) {
	MS5611_WriteCommand(MS5611_CMD_RESET);
	for (uint8_t i = 0; i < MS5611_PROM_SIZE; i++) {
		prom_data[i] = MS5611_ReadProm(i * 2);
	}
}

void MS5611_WriteCommand(uint8_t command) {
	HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDR, &command, 1, HAL_MAX_DELAY);
}

static uint16_t MS5611_ReadProm(uint8_t address) {
	uint8_t prom_rd_addr = MS5611_CMD_PROM_RD | address;
	uint8_t bytes[2];

	MS5611_WriteCommand(prom_rd_addr);
	HAL_I2C_Master_Receive(&hi2c1, MS5611_ADDR, bytes, 2, HAL_MAX_DELAY);

	return ((uint16_t) bytes[0] << 8) | bytes[1];
}

static uint32_t MS5611_ReadADC(void) {
	uint8_t addr = MS5611_CMD_ADC_RD;
	uint8_t adc_bytes[3];

	HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDR, &addr, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c1, MS5611_ADDR, adc_bytes, 3, HAL_MAX_DELAY);

	return ((uint32_t) adc_bytes[0] << 16) | ((uint32_t) adc_bytes[1] << 8)
			| ((uint32_t) adc_bytes[2]);
}

static uint32_t MS5611_ReadRawTemperature(void) {
	MS5611_WriteCommand(MS5611_CMD_CONV_TEMP);
	HAL_Delay(100); // TODO: poll the bus instead of delay
	uint32_t d2 = MS5611_ReadADC();
	return d2;
}

static uint32_t MS5611_ReadRawPressure(void) {
	MS5611_WriteCommand(MS5611_CMD_CONV_PRES);
	HAL_Delay(100); // TODO: poll the bus instead of delay
	uint32_t d1 = MS5611_ReadADC();
	return d1;
}

int32_t MS5611_ReadCompensatedTemperature(void) {
	uint32_t d2 = MS5611_ReadRawTemperature();
	int32_t delta_t = d2 - ((uint32_t) prom_data[5] << 8);
	int32_t temp = 2000 + ((int64_t) delta_t * prom_data[6] >> 23);
	return temp;
}

int32_t MS5611_ReadCompensatedPressure(void) {
	uint32_t d1 = MS5611_ReadRawPressure();
	uint32_t d2 = MS5611_ReadRawTemperature();

	int32_t delta_t = d2 - ((uint32_t) prom_data[5] << 8);

	int64_t offset = ((int64_t) prom_data[2] << 16)
			+ (((int64_t) prom_data[4] * delta_t) >> 7);
	int64_t sens = ((int64_t) prom_data[1] << 15)
			+ (((int64_t) prom_data[3] * delta_t) >> 8);
	int32_t pres = ((((int64_t) d1 * sens) >> 21) - offset) >> 15;

	return pres;
}

