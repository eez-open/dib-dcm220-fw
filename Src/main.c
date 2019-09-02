/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SDADC_HandleTypeDef hsdadc1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SDADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  MX_SDADC1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

  // DAC configuration SW RESET
  HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_RESET); // DAC CS active low
  uint8_t inputReset[3] = { 0b01010001, 0, 0 };
  if (HAL_SPI_Transmit(&hspi1, inputReset, 3, 100) != HAL_OK) {
	Error_Handler();
  }
  HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_SET); // DAC CS active high

  // DAC configuration POWER
  HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_RESET); // DAC CS active low
  uint8_t inputPower[3] = { 0b01000000, 0b00001111, 0 };
  if (HAL_SPI_Transmit(&hspi1, inputPower, 3, 100) != HAL_OK) {
	Error_Handler();
  }
  HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_SET); // DAC CS active high

  HAL_Delay(10);

  // DAC configuration SW CLEAR
  HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_RESET); // DAC CS active low
  uint8_t inputClear[3] = { 0b01010000, 0, 0 };
  if (HAL_SPI_Transmit(&hspi1, inputClear, 3, 100) != HAL_OK) {
	Error_Handler();
  }
  HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_SET); // DAC CS active high

  HAL_Delay(10);

  // DAC set internal ref. voltage to 2.5V
  HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_RESET); // DAC CS active low
  uint8_t input1[3] = { 0b01110101, 0, 0 };
  if (HAL_SPI_Transmit(&hspi1, input1, 3, 100) != HAL_OK) {
	Error_Handler();
  }
  HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_SET); // DAC CS active high

  // OUTPUT ENABLE
  HAL_GPIO_WritePin(OE_1_GPIO_Port, OE_1_Pin, GPIO_PIN_SET);

  HAL_Delay(10);

//  // DAC set voltage to "pola skale" (0b10000000 0000)
//  HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_RESET); // DAC CS active low
//  uint8_t input21[3] = { 0b00000000, 0b10000000, 0 };
//  if (HAL_SPI_Transmit(&hspi1, input21, 3, 100) != HAL_OK) {
//	Error_Handler();
//  }
//  HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_SET); // DAC CS active high
//
//  HAL_Delay(10);
//
//  HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_RESET); // DAC CS active low
//  uint8_t input22[3] = { 0b00010000, 0, 0 };
//  if (HAL_SPI_Transmit(&hspi1, input22, 3, 100) != HAL_OK) {
//	Error_Handler();
//  }
//  HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_SET); // DAC CS active high
//
//  // DAC set current to "pola skale"
//  HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_RESET); // DAC CS active low
//  uint8_t input31[3] = { 0b00000001, 0b10000000, 0 };
//  if (HAL_SPI_Transmit(&hspi1, input31, 3, 100) != HAL_OK) {
//	Error_Handler();
//  }
//  HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_SET); // DAC CS active high
//
//  HAL_Delay(10);
//
//  HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_RESET); // DAC CS active low
//  uint8_t input32[3] = { 0b00010001, 0, 0 };
//  if (HAL_SPI_Transmit(&hspi1, input32, 3, 100) != HAL_OK) {
//	Error_Handler();
//  }
//  HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_SET); // DAC CS active high

    HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_RESET); // DAC CS active low
    uint8_t input42[3] = { 0b10000010, 0b10000000, 0 };
    if (HAL_SPI_Transmit(&hspi1, input42, 3, 100) != HAL_OK) {
    	Error_Handler();
    }
    HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_SET); // DAC CS active high

    HAL_Delay(10);

//  // DAC set current to "pola skale"
//  HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_RESET); // DAC CS active low
//  uint8_t inputCurrentPola[3] = { 0b00110001, 0b10000000, 0 };
//  if (HAL_SPI_Transmit(&hspi1, inputCurrentPola, 3, 100) != HAL_OK) {
//	Error_Handler();
//  }
//  HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_SET); // DAC CS active high


  // 6 napon
  // 5 struja
  if (HAL_SDADC_AssociateChannelConfig(&hsdadc1, SDADC_CHANNEL_5, SDADC_CONF_INDEX_0) != HAL_OK) {
	  Error_Handler();
  }
  if (HAL_SDADC_AssociateChannelConfig(&hsdadc1, SDADC_CHANNEL_6, SDADC_CONF_INDEX_0) != HAL_OK) {
	  Error_Handler();
  }
  if (HAL_SDADC_SelectRegularTrigger(&hsdadc1, SDADC_SOFTWARE_TRIGGER) != HAL_OK) {
	  Error_Handler();
  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	  HAL_Delay(100);
	  // voltage
	  if (HAL_SDADC_ConfigChannel(&hsdadc1, SDADC_CHANNEL_6, SDADC_CONTINUOUS_CONV_OFF) != HAL_OK) {
		  Error_Handler();
	  }

	  if (HAL_SDADC_Start(&hsdadc1) != HAL_OK) {
		  Error_Handler();
	  }

	  if (HAL_SDADC_PollForConversion(&hsdadc1, HAL_MAX_DELAY) != HAL_OK) {
		  Error_Handler();
	  }

	  uint32_t value = HAL_SDADC_GetValue(&hsdadc1);

	  if (HAL_SDADC_Stop(&hsdadc1) != HAL_OK) {
		  Error_Handler();
	  }

	  if (value > 0) {
		  HAL_Delay(1);
	  } else {
		  HAL_Delay(2);
	  }


	  HAL_Delay(100);
	  // current
	  if (HAL_SDADC_ConfigChannel(&hsdadc1, SDADC_CHANNEL_5, SDADC_CONTINUOUS_CONV_OFF) != HAL_OK) {
		  Error_Handler();
	  }

	  if (HAL_SDADC_Start(&hsdadc1) != HAL_OK) {
		  Error_Handler();
	  }

	  if (HAL_SDADC_PollForConversion(&hsdadc1, HAL_MAX_DELAY) != HAL_OK) {
		  Error_Handler();
	  }

	   value = HAL_SDADC_GetValue(&hsdadc1);

	  if (HAL_SDADC_Stop(&hsdadc1) != HAL_OK) {
		  Error_Handler();
	  }

	  if (value > 0) {
		  HAL_Delay(1);
	  } else {
		  HAL_Delay(2);
	  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC1|RCC_PERIPHCLK_SDADC;
  PeriphClkInit.SdadcClockSelection = RCC_SDADCSYSCLK_DIV16;
  PeriphClkInit.Adc1ClockSelection = RCC_ADC1PCLK2_DIV2;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_PWREx_EnableSDADC(PWR_SDADC_ANALOG1);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SDADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDADC1_Init(void)
{

  /* USER CODE BEGIN SDADC1_Init 0 */

  /* USER CODE END SDADC1_Init 0 */

  SDADC_ConfParamTypeDef ConfParamStruct = {0};

  /* USER CODE BEGIN SDADC1_Init 1 */

  /* USER CODE END SDADC1_Init 1 */
  /** Configure the SDADC low power mode, fast conversion mode,
  slow clock mode and SDADC1 reference voltage 
  */
  hsdadc1.Instance = SDADC1;
  hsdadc1.Init.IdleLowPowerMode = SDADC_LOWPOWER_NONE;
  hsdadc1.Init.FastConversionMode = SDADC_FAST_CONV_DISABLE;
  hsdadc1.Init.SlowClockMode = SDADC_SLOW_CLOCK_DISABLE;
  hsdadc1.Init.ReferenceVoltage = SDADC_VREF_EXT;
  if (HAL_SDADC_Init(&hsdadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Set parameters for SDADC configuration 0 Register 
  */
  ConfParamStruct.InputMode = SDADC_INPUT_MODE_SE_OFFSET;
  ConfParamStruct.Gain = SDADC_GAIN_1;
  ConfParamStruct.CommonMode = SDADC_COMMON_MODE_VSSA;
  ConfParamStruct.Offset = 0;
  if (HAL_SDADC_PrepareChannelConfig(&hsdadc1, SDADC_CONF_INDEX_0, &ConfParamStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDADC1_Init 2 */

  /* USER CODE END SDADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIB_IRQ_GPIO_Port, DIB_IRQ_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OE_1_Pin|OE_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CC_1_Pin CC_2_Pin */
  GPIO_InitStruct.Pin = CC_1_Pin|CC_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DAC_CS_Pin DIB_IRQ_Pin */
  GPIO_InitStruct.Pin = DAC_CS_Pin|DIB_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DIB_SYNC_Pin */
  GPIO_InitStruct.Pin = DIB_SYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIB_SYNC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PWRGOOD_Pin Reserved_Pin ReservedB9_Pin */
  GPIO_InitStruct.Pin = PWRGOOD_Pin|Reserved_Pin|ReservedB9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : OE_1_Pin OE_2_Pin */
  GPIO_InitStruct.Pin = OE_1_Pin|OE_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
