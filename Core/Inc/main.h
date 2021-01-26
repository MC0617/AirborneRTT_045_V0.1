/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EL_LIM_Pin GPIO_PIN_2
#define EL_LIM_GPIO_Port GPIOE
#define RX_LIM_Pin GPIO_PIN_4
#define RX_LIM_GPIO_Port GPIOE
#define TX_LIM_Pin GPIO_PIN_5
#define TX_LIM_GPIO_Port GPIOE
#define RX_DIR_Pin GPIO_PIN_1
#define RX_DIR_GPIO_Port GPIOF
#define RX_EN_Pin GPIO_PIN_2
#define RX_EN_GPIO_Port GPIOF
#define EL_EN_Pin GPIO_PIN_11
#define EL_EN_GPIO_Port GPIOF
#define TX_EN_Pin GPIO_PIN_7
#define TX_EN_GPIO_Port GPIOH
#define LED_Pin GPIO_PIN_9
#define LED_GPIO_Port GPIOH
#define EL_DIR_Pin GPIO_PIN_11
#define EL_DIR_GPIO_Port GPIOD
#define TX_DIR_Pin GPIO_PIN_8
#define TX_DIR_GPIO_Port GPIOC
#define AZ_DIR_Pin GPIO_PIN_3
#define AZ_DIR_GPIO_Port GPIOD
#define AZ_EN_Pin GPIO_PIN_7
#define AZ_EN_GPIO_Port GPIOD
#define AZ_LIM_Pin GPIO_PIN_0
#define AZ_LIM_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

#define ETH_RST_Pin GPIO_PIN_2
#define ETH_RST_GPIO_Port GPIOB

#define AZ_EC_TIM TIM1
#define EL_EC_TIM TIM2
#define RL_EC_TIM TIM3
#define POL_EC_TIM TIM4

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
