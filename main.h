/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define HCSR04_TRIG_Pin GPIO_PIN_2
#define HCSR04_TRIG_GPIO_Port GPIOA
#define GREEN_RGB_Pin GPIO_PIN_0
#define GREEN_RGB_GPIO_Port GPIOB
#define BLUE_RGB_Pin GPIO_PIN_1
#define BLUE_RGB_GPIO_Port GPIOB
#define HCSR04_ECHO_Pin GPIO_PIN_12
#define HCSR04_ECHO_GPIO_Port GPIOB
#define HCSR04_ECHO_EXTI_IRQn EXTI15_10_IRQn
#define BEEP_Pin GPIO_PIN_8
#define BEEP_GPIO_Port GPIOA
#define RED_RGB_Pin GPIO_PIN_5
#define RED_RGB_GPIO_Port GPIOB
#define ESP_EN_Pin GPIO_PIN_8
#define ESP_EN_GPIO_Port GPIOB
#define ESP_RST_Pin GPIO_PIN_9
#define ESP_RST_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
