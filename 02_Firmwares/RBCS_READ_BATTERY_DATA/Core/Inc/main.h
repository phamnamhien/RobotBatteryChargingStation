/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define RS4851_DI_Pin GPIO_PIN_2
#define RS4851_DI_GPIO_Port GPIOA
#define RS4851_RO_Pin GPIO_PIN_3
#define RS4851_RO_GPIO_Port GPIOA
#define RS4851_TXEN_Pin GPIO_PIN_4
#define RS4851_TXEN_GPIO_Port GPIOA
#define RD_VOLTAGE_Pin GPIO_PIN_6
#define RD_VOLTAGE_GPIO_Port GPIOA
#define ADDR4_Pin GPIO_PIN_0
#define ADDR4_GPIO_Port GPIOB
#define ADDR3_Pin GPIO_PIN_1
#define ADDR3_GPIO_Port GPIOB
#define ADDR2_Pin GPIO_PIN_2
#define ADDR2_GPIO_Port GPIOB
#define RS4852_DI_Pin GPIO_PIN_10
#define RS4852_DI_GPIO_Port GPIOB
#define RS4852_RO_Pin GPIO_PIN_11
#define RS4852_RO_GPIO_Port GPIOB
#define RS4852_TXEN_Pin GPIO_PIN_12
#define RS4852_TXEN_GPIO_Port GPIOB
#define CHARGE_CTRL_Pin GPIO_PIN_8
#define CHARGE_CTRL_GPIO_Port GPIOA
#define EMERGENCY_Pin GPIO_PIN_10
#define EMERGENCY_GPIO_Port GPIOA
#define ADDR1_Pin GPIO_PIN_3
#define ADDR1_GPIO_Port GPIOB
#define ADDR0_Pin GPIO_PIN_4
#define ADDR0_GPIO_Port GPIOB
#define LED_STT_Pin GPIO_PIN_5
#define LED_STT_GPIO_Port GPIOB
#define LED_RUN_Pin GPIO_PIN_6
#define LED_RUN_GPIO_Port GPIOB
#define LED_FAULT_Pin GPIO_PIN_7
#define LED_FAULT_GPIO_Port GPIOB
#define LIMIT_SWITCH0_Pin GPIO_PIN_8
#define LIMIT_SWITCH0_GPIO_Port GPIOB
#define LIMIT_SWITCH1_Pin GPIO_PIN_9
#define LIMIT_SWITCH1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
