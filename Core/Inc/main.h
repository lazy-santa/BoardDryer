/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define LAMPS_Pin GPIO_PIN_0
#define LAMPS_GPIO_Port GPIOA
#define LED_Red_Pin GPIO_PIN_1
#define LED_Red_GPIO_Port GPIOA
#define LED_Green_Pin GPIO_PIN_2
#define LED_Green_GPIO_Port GPIOA
#define LED_Blue_Pin GPIO_PIN_3
#define LED_Blue_GPIO_Port GPIOA
#define MOSFET_Pin GPIO_PIN_6
#define MOSFET_GPIO_Port GPIOA
#define BUTTON_ENCODER_Pin GPIO_PIN_10
#define BUTTON_ENCODER_GPIO_Port GPIOA
#define BUTTON_ENCODER_EXTI_IRQn EXTI15_10_IRQn
#define BUZZER_Pin GPIO_PIN_6
#define BUZZER_GPIO_Port GPIOB
#define BUTTON_DOOR_Pin GPIO_PIN_7
#define BUTTON_DOOR_GPIO_Port GPIOB
#define BUTTON_DOOR_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
