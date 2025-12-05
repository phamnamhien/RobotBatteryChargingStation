/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RS4851_TXEN_Pin|CHARGE_CTRL_Pin|EMERGENCY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RS4852_TXEN_Pin|LED_STT_Pin|LED_RUN_Pin|LED_FAULT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RS4851_TXEN_Pin CHARGE_CTRL_Pin EMERGENCY_Pin */
  GPIO_InitStruct.Pin = RS4851_TXEN_Pin|CHARGE_CTRL_Pin|EMERGENCY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RD_VOLTAGE_Pin */
  GPIO_InitStruct.Pin = RD_VOLTAGE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(RD_VOLTAGE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ADDR4_Pin ADDR3_Pin ADDR2_Pin ADDR1_Pin
                           ADDR0_Pin LIMIT_SWITCH0_Pin LIMIT_SWITCH1_Pin */
  GPIO_InitStruct.Pin = ADDR4_Pin|ADDR3_Pin|ADDR2_Pin|ADDR1_Pin
                          |ADDR0_Pin|LIMIT_SWITCH0_Pin|LIMIT_SWITCH1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RS4852_TXEN_Pin LED_STT_Pin LED_RUN_Pin LED_FAULT_Pin */
  GPIO_InitStruct.Pin = RS4852_TXEN_Pin|LED_STT_Pin|LED_RUN_Pin|LED_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
