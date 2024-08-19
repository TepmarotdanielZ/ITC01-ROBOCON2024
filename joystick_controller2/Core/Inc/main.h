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
#define Back_rice_Pin GPIO_PIN_13
#define Back_rice_GPIO_Port GPIOC
#define CSN_Pin GPIO_PIN_4
#define CSN_GPIO_Port GPIOA
#define CE_Pin GPIO_PIN_0
#define CE_GPIO_Port GPIOB
#define SMR_Pin GPIO_PIN_1
#define SMR_GPIO_Port GPIOB
#define Front_rice_Pin GPIO_PIN_10
#define Front_rice_GPIO_Port GPIOB
#define SML_Pin GPIO_PIN_11
#define SML_GPIO_Port GPIOB
#define Button3_L_Pin GPIO_PIN_14
#define Button3_L_GPIO_Port GPIOB
#define Mode_Pin GPIO_PIN_3
#define Mode_GPIO_Port GPIOB
#define Drop_rice_Pin GPIO_PIN_4
#define Drop_rice_GPIO_Port GPIOB
#define Take_rice_Pin GPIO_PIN_5
#define Take_rice_GPIO_Port GPIOB
#define Speed_Pin GPIO_PIN_6
#define Speed_GPIO_Port GPIOB
#define Shooter_Pin GPIO_PIN_8
#define Shooter_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
