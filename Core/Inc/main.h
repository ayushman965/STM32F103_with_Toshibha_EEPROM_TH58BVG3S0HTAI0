/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define CE_Pin GPIO_PIN_0
#define CE_GPIO_Port GPIOA
#define WE_Pin GPIO_PIN_1
#define WE_GPIO_Port GPIOA
#define RE_Pin GPIO_PIN_2
#define RE_GPIO_Port GPIOA
#define CLE_Pin GPIO_PIN_3
#define CLE_GPIO_Port GPIOA
#define ALE_Pin GPIO_PIN_4
#define ALE_GPIO_Port GPIOA
#define WP_Pin GPIO_PIN_5
#define WP_GPIO_Port GPIOA
#define BY_RY_Pin GPIO_PIN_6
#define BY_RY_GPIO_Port GPIOA
#define DATA_1_Pin GPIO_PIN_7
#define DATA_1_GPIO_Port GPIOA
#define DATA_2_Pin GPIO_PIN_8
#define DATA_2_GPIO_Port GPIOA
#define DATA_3_Pin GPIO_PIN_9
#define DATA_3_GPIO_Port GPIOA
#define DATA_4_Pin GPIO_PIN_10
#define DATA_4_GPIO_Port GPIOA
#define DATA_5_Pin GPIO_PIN_11
#define DATA_5_GPIO_Port GPIOA
#define DATA_6_Pin GPIO_PIN_12
#define DATA_6_GPIO_Port GPIOA
#define DATA_7_Pin GPIO_PIN_13
#define DATA_7_GPIO_Port GPIOA
#define DATA_8_Pin GPIO_PIN_14
#define DATA_8_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
