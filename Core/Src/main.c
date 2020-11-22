/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MS5611_PROM_SIZE 				8
#define MS5611_ADDR 						(0x77 << 1)
#define MS5611_CMD_RESET 				0x1E
#define MS5611_CMD_PROM_RD 			0xA0
#define MS5611_CMD_CONV_PRES 		0x48
#define MS5611_CMD_CONV_TEMP		0x58
#define MS5611_CMD_ADC_RD 			0x00
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint16_t prom_data[8];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static void MS5611_Init(void);
/* private */ static void MS5611_WriteCommand(uint8_t command);
/* private */ static uint16_t MS5611_ReadProm(uint8_t address);
/* private */ static uint32_t MS5611_ReadADC(void);
/* private */ static uint32_t MS5611_ReadRawTemperature(void);
/* private */ static uint32_t MS5611_ReadRawPressure(void);
static int32_t MS5611_ReadCompensatedTemperature(void);
static int32_t MS5611_ReadCompensatedPressure(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void MS5611_Init(void)
{
	MS5611_WriteCommand(MS5611_CMD_RESET);
	for (uint8_t i = 0; i < MS5611_PROM_SIZE; i++)
	{
		prom_data[i] = MS5611_ReadProm(i*2);
	}
}

static void MS5611_WriteCommand(uint8_t command)
{
	HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDR, &command, 1, HAL_MAX_DELAY);
}

static uint16_t MS5611_ReadProm(uint8_t address)
{
	uint8_t prom_rd_addr = MS5611_CMD_PROM_RD | address;
	uint8_t bytes[2];

//	HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDR, &addr, 1, HAL_MAX_DELAY);
	MS5611_WriteCommand(prom_rd_addr);
	HAL_I2C_Master_Receive(&hi2c1, MS5611_ADDR, bytes, 2, HAL_MAX_DELAY);

	return ((uint16_t) bytes[0] << 8) | bytes[1];
}

static uint32_t MS5611_ReadADC(void)
{
	uint8_t addr = MS5611_CMD_ADC_RD;
	uint8_t adc_bytes[3];
	
	HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDR, &addr, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c1, MS5611_ADDR, adc_bytes, 3, HAL_MAX_DELAY);

	return ((uint32_t) adc_bytes[0] << 16) | ((uint32_t) adc_bytes[1] << 8) | ((uint32_t) adc_bytes[2]);
}

static uint32_t MS5611_ReadRawTemperature(void)
{
	MS5611_WriteCommand(MS5611_CMD_CONV_TEMP);
	HAL_Delay(100); // TODO: poll the bus instead of delay
	uint32_t d2 = MS5611_ReadADC();
	return d2;
}

static uint32_t MS5611_ReadRawPressure(void)
{
	MS5611_WriteCommand(MS5611_CMD_CONV_PRES);
	HAL_Delay(100); // TODO: poll the bus instead of delay
	uint32_t d1 = MS5611_ReadADC();
	return d1;
}

static int32_t MS5611_ReadCompensatedTemperature(void)
{
	uint32_t d2 = MS5611_ReadRawTemperature();
	int32_t delta_t = d2 - ((uint32_t) prom_data[5] << 8);
	int32_t temp = 2000 + ((int64_t) delta_t * prom_data[6] >> 23);
	return temp;
}

static int32_t MS5611_ReadCompensatedPressure(void)
{
	uint32_t d1 = MS5611_ReadRawPressure();
	uint32_t d2 = MS5611_ReadRawTemperature();

	int32_t delta_t = d2 - ((uint32_t) prom_data[5] << 8);

	int64_t offset = ((int64_t) prom_data[2] << 16) + (((int64_t) prom_data[4] * delta_t) >> 7);
	int64_t sens = ((int64_t) prom_data[1] << 15) + (((int64_t) prom_data[3] * delta_t) >> 8);
	int32_t pres = ((((int64_t) d1 * sens) >> 21) - offset) >> 15;

	return pres;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MS5611_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  	uint8_t buf[32];
  	int32_t temp = MS5611_ReadCompensatedTemperature();
  	int32_t pres = MS5611_ReadCompensatedPressure();

  	sprintf((char*) buf, "Temp: %.2f C, P: %.2f mbar\r\n", temp / 100.0f, pres / 100.0f);

  	HAL_UART_Transmit(&huart2, buf, strlen((char*) buf), HAL_MAX_DELAY);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 20000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
