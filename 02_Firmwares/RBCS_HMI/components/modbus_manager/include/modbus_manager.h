#ifndef MODBUS_MANAGER_H
#define MODBUS_MANAGER_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h> 


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Cấu hình Modbus Manager
 */
typedef struct {
    int uart_port;          // UART port (UART_NUM_1, UART_NUM_2)
    int tx_pin;             // TX GPIO pin
    int rx_pin;             // RX GPIO pin
    int rts_pin;            // RTS GPIO pin (DE/RE cho RS485)
    uint32_t baudrate;      // Baudrate (9600, 19200, 115200...)
} modbus_config_t;

/**
 * @brief Callback khi có dữ liệu Modbus mới
 * 
 * @param slave_addr Địa chỉ slave
 * @param reg_type Loại register (0x03=Holding, 0x04=Input)
 * @param reg_addr Địa chỉ register bắt đầu
 * @param data Con trỏ tới dữ liệu
 * @param length Số lượng register
 */
typedef void (*modbus_data_callback_t)(uint8_t slave_addr, uint8_t reg_type, 
                                       uint16_t reg_addr, uint16_t *data, 
                                       uint16_t length);

/**
 * @brief Khởi tạo Modbus Manager
 * 
 * @param config Con trỏ tới cấu hình
 * @return ESP_OK nếu thành công
 */
esp_err_t modbus_manager_init(const modbus_config_t *config);

/**
 * @brief Dừng Modbus Manager
 * 
 * @return ESP_OK nếu thành công
 */
esp_err_t modbus_manager_deinit(void);

/**
 * @brief Đăng ký callback nhận dữ liệu
 * 
 * @param callback Hàm callback
 */
void modbus_manager_register_callback(modbus_data_callback_t callback);

/**
 * @brief Đọc Holding Registers (FC 0x03)
 * 
 * @param slave_addr Địa chỉ slave
 * @param reg_addr Địa chỉ register bắt đầu
 * @param reg_count Số lượng register
 * @param data Buffer để lưu dữ liệu đọc được
 * @return ESP_OK nếu thành công
 */
esp_err_t modbus_read_holding_registers(uint8_t slave_addr, uint16_t reg_addr, 
                                        uint16_t reg_count, uint16_t *data);

/**
 * @brief Đọc Input Registers (FC 0x04)
 * 
 * @param slave_addr Địa chỉ slave
 * @param reg_addr Địa chỉ register bắt đầu
 * @param reg_count Số lượng register
 * @param data Buffer để lưu dữ liệu đọc được
 * @return ESP_OK nếu thành công
 */
esp_err_t modbus_read_input_registers(uint8_t slave_addr, uint16_t reg_addr, 
                                      uint16_t reg_count, uint16_t *data);

/**
 * @brief Ghi Single Register (FC 0x06)
 * 
 * @param slave_addr Địa chỉ slave
 * @param reg_addr Địa chỉ register
 * @param value Giá trị cần ghi
 * @return ESP_OK nếu thành công
 */
esp_err_t modbus_write_single_register(uint8_t slave_addr, uint16_t reg_addr, 
                                       uint16_t value);

/**
 * @brief Ghi Multiple Registers (FC 0x10)
 * 
 * @param slave_addr Địa chỉ slave
 * @param reg_addr Địa chỉ register bắt đầu
 * @param reg_count Số lượng register
 * @param data Dữ liệu cần ghi
 * @return ESP_OK nếu thành công
 */
esp_err_t modbus_write_multiple_registers(uint8_t slave_addr, uint16_t reg_addr,
                                          uint16_t reg_count, uint16_t *data);

/**
 * @brief Đọc Coils (FC 0x01)
 * 
 * @param slave_addr Địa chỉ slave
 * @param coil_addr Địa chỉ coil bắt đầu
 * @param coil_count Số lượng coil
 * @param data Buffer để lưu trạng thái (bit packed)
 * @return ESP_OK nếu thành công
 */
esp_err_t modbus_read_coils(uint8_t slave_addr, uint16_t coil_addr,
                            uint16_t coil_count, uint8_t *data);

/**
 * @brief Ghi Single Coil (FC 0x05)
 * 
 * @param slave_addr Địa chỉ slave
 * @param coil_addr Địa chỉ coil
 * @param value true=ON, false=OFF
 * @return ESP_OK nếu thành công
 */
esp_err_t modbus_write_single_coil(uint8_t slave_addr, uint16_t coil_addr,
                                   bool value);

/**
 * @brief Kiểm tra Modbus Manager có đang chạy không
 * 
 * @return true nếu đang chạy
 */
bool modbus_manager_is_running(void);

#ifdef __cplusplus
}
#endif

#endif // MODBUS_MANAGER_H

