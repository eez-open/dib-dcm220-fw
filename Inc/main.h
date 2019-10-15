/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CC_LED_1_Pin GPIO_PIN_13
#define CC_LED_1_GPIO_Port GPIOC
#define CC_1_Pin GPIO_PIN_14
#define CC_1_GPIO_Port GPIOC
#define CC_1_EXTI_IRQn EXTI15_10_IRQn
#define CC_2_Pin GPIO_PIN_15
#define CC_2_GPIO_Port GPIOC
#define CC_2_EXTI_IRQn EXTI15_10_IRQn
#define TEMP_1_Pin GPIO_PIN_0
#define TEMP_1_GPIO_Port GPIOA
#define CC_LED_2_Pin GPIO_PIN_1
#define CC_LED_2_GPIO_Port GPIOA
#define TEMP_2_Pin GPIO_PIN_4
#define TEMP_2_GPIO_Port GPIOA
#define U_SET_2_Pin GPIO_PIN_5
#define U_SET_2_GPIO_Port GPIOA
#define U_SET_1_Pin GPIO_PIN_6
#define U_SET_1_GPIO_Port GPIOA
#define U_MON_1_Pin GPIO_PIN_0
#define U_MON_1_GPIO_Port GPIOB
#define I_MON_1_Pin GPIO_PIN_1
#define I_MON_1_GPIO_Port GPIOB
#define U_MON_2_Pin GPIO_PIN_2
#define U_MON_2_GPIO_Port GPIOB
#define I_MON_2_Pin GPIO_PIN_8
#define I_MON_2_GPIO_Port GPIOE
#define DIB_SCLK_Pin GPIO_PIN_8
#define DIB_SCLK_GPIO_Port GPIOA
#define DIB_MISO_Pin GPIO_PIN_9
#define DIB_MISO_GPIO_Port GPIOA
#define DIB_MOSI_Pin GPIO_PIN_10
#define DIB_MOSI_GPIO_Port GPIOA
#define DIB_NSS_Pin GPIO_PIN_11
#define DIB_NSS_GPIO_Port GPIOA
#define DIB_IRQ_Pin GPIO_PIN_12
#define DIB_IRQ_GPIO_Port GPIOA
#define DIB_SYNC_Pin GPIO_PIN_7
#define DIB_SYNC_GPIO_Port GPIOF
#define DIB_SYNC_EXTI_IRQn EXTI9_5_IRQn
#define PWRGOOD_Pin GPIO_PIN_4
#define PWRGOOD_GPIO_Port GPIOB
#define OE_1_Pin GPIO_PIN_5
#define OE_1_GPIO_Port GPIOB
#define OE_2_Pin GPIO_PIN_6
#define OE_2_GPIO_Port GPIOB
#define I_SET_1_Pin GPIO_PIN_8
#define I_SET_1_GPIO_Port GPIOB
#define I_SET_2_Pin GPIO_PIN_9
#define I_SET_2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
