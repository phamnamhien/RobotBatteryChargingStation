/*
 * app_params.c
 *
 *  Created on: Dec 4, 2025
 *      Author: admin
 */
#include "app_states.h"

#define EEPROM_MAGIC_NUMBER	0xAAAA

#define MIN_STORAGE_BAUDRATE	1
#define DEF_STORAGE_BAUDRATE	10
#define MAX_STORAGE_BAUDRATE	10
void default_parameter_init(DeviceHSM_t* hsm) {
	EE_Init(&hsm->storage, sizeof(Storage_t));
	if (hsm == NULL) return;
	EE_Read();
	if(hsm->storage.magic != EEPROM_MAGIC_NUMBER) {
		hsm->storage.magic = EEPROM_MAGIC_NUMBER;
		hsm->storage.baudrate = DEF_STORAGE_BAUDRATE;
		EE_Write();
	}
}


/**
 * @brief Convert baudrate code to actual baudrate value
 * @param code: Baudrate code (1-10)
 * @return Baudrate value, or 0 if invalid code
 */
uint32_t BaudrateCodeToValue(uint8_t code)
{
    switch(code)
    {
        case 1:  return 1200;
        case 2:  return 2400;
        case 3:  return 4800;
        case 4:  return 9600;
        case 5:  return 14400;
        case 6:  return 19200;
        case 7:  return 38400;
        case 8:  return 56000;
        case 9:  return 57600;
        case 10: return 115200;
        default: return 0; // Invalid code
    }
}

/**
 * @brief Convert baudrate value to baudrate code
 * @param baudrate: Baudrate value
 * @return Baudrate code (1-10), or 0 if invalid baudrate
 */
uint8_t BaudrateValueToCode(uint32_t baudrate)
{
    switch(baudrate)
    {
        case 1200:   return 1;
        case 2400:   return 2;
        case 4800:   return 3;
        case 9600:   return 4;
        case 14400:  return 5;
        case 19200:  return 6;
        case 38400:  return 7;
        case 56000:  return 8;
        case 57600:  return 9;
        case 115200: return 10;
        default:     return 0; // Invalid baudrate
    }
}
/**
 * @brief Reconfigure UART with new baudrate (8-N-1 format)
 * @param huart: Pointer to UART handle
 * @param baudrate: New baudrate value
 * @return HAL_StatusTypeDef: HAL_OK if successful
 */
HAL_StatusTypeDef UART_Reconfigure(UART_HandleTypeDef *huart, uint32_t baudrate)
{
    HAL_StatusTypeDef status;

    // Deinit UART
    if(HAL_UART_DeInit(huart) != HAL_OK)
    {
        return HAL_ERROR;
    }

    // Reconfigure with new baudrate
    huart->Init.BaudRate = baudrate;
    huart->Init.WordLength = UART_WORDLENGTH_8B;
    huart->Init.StopBits = UART_STOPBITS_1;
    huart->Init.Parity = UART_PARITY_NONE;
    huart->Init.Mode = UART_MODE_TX_RX;
    huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart->Init.OverSampling = UART_OVERSAMPLING_16;

    // Reinit UART
    status = HAL_UART_Init(huart);

    return status;
}

/**
 * @brief Reconfigure UART with baudrate code (1-10)
 * @param huart: Pointer to UART handle
 * @param code: Baudrate code (1-10)
 * @return HAL_StatusTypeDef: HAL_OK if successful
 */
HAL_StatusTypeDef UART_ReconfigureByCode(UART_HandleTypeDef *huart, uint8_t code)
{
    uint32_t baudrate = BaudrateCodeToValue(code);

    if(baudrate == 0)
    {
        return HAL_ERROR; // Invalid code
    }

    return UART_Reconfigure(huart, baudrate);
}
