/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32g4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Fs 20000
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define ADC_BEMF1_Pin GPIO_PIN_0
#define ADC_BEMF1_GPIO_Port GPIOC
#define ADC_BEMF3_Pin GPIO_PIN_1
#define ADC_BEMF3_GPIO_Port GPIOC
#define ADC_BEMF2_Pin GPIO_PIN_3
#define ADC_BEMF2_GPIO_Port GPIOC
#define ADC_VBUS_Pin GPIO_PIN_0
#define ADC_VBUS_GPIO_Port GPIOA
#define ADC_CURR1_Pin GPIO_PIN_1
#define ADC_CURR1_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define ADC2_NTC_Pin GPIO_PIN_4
#define ADC2_NTC_GPIO_Port GPIOC
#define ADC_CURR3_Pin GPIO_PIN_0
#define ADC_CURR3_GPIO_Port GPIOB
#define ADC_CURR2_Pin GPIO_PIN_1
#define ADC_CURR2_GPIO_Port GPIOB
#define ADC2_POS_Pin GPIO_PIN_2
#define ADC2_POS_GPIO_Port GPIOB
#define EN_FAULT_Pin GPIO_PIN_12
#define EN_FAULT_GPIO_Port GPIOB
#define ENU_Pin GPIO_PIN_13
#define ENU_GPIO_Port GPIOB
#define ENV_Pin GPIO_PIN_14
#define ENV_GPIO_Port GPIOB
#define ENW_Pin GPIO_PIN_15
#define ENW_GPIO_Port GPIOB
#define SYNC_SIG_Pin GPIO_PIN_6
#define SYNC_SIG_GPIO_Port GPIOC
#define INU_Pin GPIO_PIN_8
#define INU_GPIO_Port GPIOA
#define INV_Pin GPIO_PIN_9
#define INV_GPIO_Port GPIOA
#define INW_Pin GPIO_PIN_10
#define INW_GPIO_Port GPIOA
#define H1_Pin GPIO_PIN_15
#define H1_GPIO_Port GPIOA
#define H2_Pin GPIO_PIN_3
#define H2_GPIO_Port GPIOB
#define STBY_Pin GPIO_PIN_5
#define STBY_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
