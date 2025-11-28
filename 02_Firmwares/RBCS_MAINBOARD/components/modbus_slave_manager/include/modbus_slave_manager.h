#ifndef MODBUS_SLAVE_MANAGER_H
#define MODBUS_SLAVE_MANAGER_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Modbus Slave configuration structure
 */
typedef struct {
    uint8_t slave_addr;     // Slave address (1-247)
    int uart_port;          // UART port (UART_NUM_1, UART_NUM_2)
    int tx_pin;             // TX GPIO pin
    int rx_pin;             // RX GPIO pin
    int rts_pin;            // RTS GPIO pin (DE/RE for RS485)
    uint32_t baudrate;      // Baudrate (9600, 19200, 115200...)
} modbus_slave_config_t;

/**
 * @brief Callback when Master writes to slave
 * 
 * @param reg_type Register type (MB_PARAM_HOLDING, MB_PARAM_COIL)
 * @param address Register/coil address
 * @param value New value
 */
typedef void (*modbus_slave_change_callback_t)(uint8_t reg_type, uint16_t address, uint16_t value);

/**
 * @brief Initialize Modbus Slave
 * 
 * @param config Slave configuration
 * @return ESP_OK if successful
 */
esp_err_t modbus_slave_init(const modbus_slave_config_t *config);

/**
 * @brief Deinitialize Modbus Slave
 * 
 * @return ESP_OK if successful
 */
esp_err_t modbus_slave_deinit(void);

/**
 * @brief Register callback for write events
 * 
 * @param callback Callback function
 */
void modbus_slave_register_callback(modbus_slave_change_callback_t callback);

/**
 * @brief Update Holding Register value
 * 
 * @param address Register address (0-based)
 * @param value New value
 * @return ESP_OK if successful
 */
esp_err_t modbus_slave_set_holding_register(uint16_t address, uint16_t value);

/**
 * @brief Read Holding Register value
 * 
 * @param address Register address (0-based)
 * @param value Pointer to store value
 * @return ESP_OK if successful
 */
esp_err_t modbus_slave_get_holding_register(uint16_t address, uint16_t *value);

/**
 * @brief Update Input Register value
 * 
 * @param address Register address (0-based)
 * @param value New value
 * @return ESP_OK if successful
 */
esp_err_t modbus_slave_set_input_register(uint16_t address, uint16_t value);

/**
 * @brief Update Coil value
 * 
 * @param address Coil address (0-based)
 * @param value true=ON, false=OFF
 * @return ESP_OK if successful
 */
esp_err_t modbus_slave_set_coil(uint16_t address, bool value);

/**
 * @brief Read Coil value
 * 
 * @param address Coil address (0-based)
 * @param value Pointer to store value
 * @return ESP_OK if successful
 */
esp_err_t modbus_slave_get_coil(uint16_t address, bool *value);

/**
 * @brief Update Discrete Input value
 * 
 * @param address Input address (0-based)
 * @param value true=ON, false=OFF
 * @return ESP_OK if successful
 */
esp_err_t modbus_slave_set_discrete_input(uint16_t address, bool value);

/**
 * @brief Check if Slave is running
 * 
 * @return true if running
 */
bool modbus_slave_is_running(void);

#ifdef __cplusplus
}
#endif

#endif // MODBUS_SLAVE_MANAGER_H

