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
#include "stm32wbxx_hal.h"

#include "app_conf.h"
#include "app_entry.h"
#include "app_common.h"

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
#define SW1_Pin GPIO_PIN_4
#define SW1_GPIO_Port GPIOC
#define SW1_EXTI_IRQn EXTI4_IRQn
#define SW2_Pin GPIO_PIN_0
#define SW2_GPIO_Port GPIOD
#define SW2_EXTI_IRQn EXTI0_IRQn
#define SW3_Pin GPIO_PIN_1
#define SW3_GPIO_Port GPIOD
#define SW3_EXTI_IRQn EXTI1_IRQn
/* USER CODE BEGIN Private defines */

#define LED2_PORT GPIOB        /* Green LED2 on NUCLEO-WB55RG */
#define LED2_PIN  GPIO_PIN_5   /* Green LED2 on NUCLEO-WB55RG */

#define EXT_PA_TX_PORT  GPIOB         /* PB0 in AF6 mode (EXT_PA_TX) to control CTX on Ext PA */
#define EXT_PA_TX_PIN   GPIO_PIN_0
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
