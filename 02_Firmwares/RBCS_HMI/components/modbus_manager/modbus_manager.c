#include "modbus_manager.h"
#include "esp_modbus_master.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "MODBUS_MGR";

// Internal state
static struct {
    void *master_handle;
    modbus_config_t config;
    modbus_data_callback_t callback;
    SemaphoreHandle_t mutex;
    bool initialized;
    bool running;
} modbus_ctx = {0};

// Private functions
static esp_err_t modbus_lock(void)
{
    if (!modbus_ctx.mutex) return ESP_ERR_INVALID_STATE;
    return (xSemaphoreTake(modbus_ctx.mutex, pdMS_TO_TICKS(5000)) == pdTRUE) 
           ? ESP_OK : ESP_ERR_TIMEOUT;
}

static void modbus_unlock(void)
{
    if (modbus_ctx.mutex) {
        xSemaphoreGive(modbus_ctx.mutex);
    }
}

// Public API implementation
esp_err_t modbus_manager_init(const modbus_config_t *config)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }

    if (modbus_ctx.initialized) {
        ESP_LOGW(TAG, "Modbus Manager đã được khởi tạo");
        return ESP_OK;
    }

    // Tạo mutex
    modbus_ctx.mutex = xSemaphoreCreateMutex();
    if (!modbus_ctx.mutex) {
        ESP_LOGE(TAG, "Không thể tạo mutex");
        return ESP_ERR_NO_MEM;
    }

    // Lưu config
    memcpy(&modbus_ctx.config, config, sizeof(modbus_config_t));

    // Cấu hình Modbus
    mb_communication_info_t comm_info = {
        .ser_opts = {
            .mode = MB_RTU,
            .port = config->uart_port,
            .baudrate = config->baudrate,
            .data_bits = UART_DATA_8_BITS,
            .parity = MB_PARITY_NONE,
            .stop_bits = UART_STOP_BITS_1,
            .uid = 1
        }
    };

    // Tạo Modbus Master
    esp_err_t err = mbc_master_create_serial(&comm_info, &modbus_ctx.master_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Lỗi tạo Modbus Master: %s", esp_err_to_name(err));
        vSemaphoreDelete(modbus_ctx.mutex);
        return err;
    }

    // Cấu hình GPIO
    err = uart_set_pin(config->uart_port, config->tx_pin, config->rx_pin, 
                       config->rts_pin, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Lỗi cấu hình GPIO: %s", esp_err_to_name(err));
        mbc_master_delete(modbus_ctx.master_handle);
        vSemaphoreDelete(modbus_ctx.mutex);
        return err;
    }

    // Khởi động Modbus stack
    err = mbc_master_start(modbus_ctx.master_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Lỗi start Modbus: %s", esp_err_to_name(err));
        mbc_master_delete(modbus_ctx.master_handle);
        vSemaphoreDelete(modbus_ctx.mutex);
        return err;
    }

    modbus_ctx.initialized = true;
    modbus_ctx.running = true;

    ESP_LOGI(TAG, "Modbus Manager khởi tạo thành công");
    ESP_LOGI(TAG, "  Port: UART%d, Baudrate: %lu", config->uart_port, config->baudrate);
    ESP_LOGI(TAG, "  TX: GPIO%d, RX: GPIO%d, RTS: GPIO%d", 
             config->tx_pin, config->rx_pin, config->rts_pin);

    return ESP_OK;
}

esp_err_t modbus_manager_deinit(void)
{
    if (!modbus_ctx.initialized) {
        return ESP_OK;
    }

    modbus_lock();
    
    modbus_ctx.running = false;
    
    if (modbus_ctx.master_handle) {
        mbc_master_stop(modbus_ctx.master_handle);
        mbc_master_delete(modbus_ctx.master_handle);
        modbus_ctx.master_handle = NULL;
    }

    modbus_ctx.initialized = false;
    modbus_ctx.callback = NULL;

    modbus_unlock();
    
    if (modbus_ctx.mutex) {
        vSemaphoreDelete(modbus_ctx.mutex);
        modbus_ctx.mutex = NULL;
    }

    ESP_LOGI(TAG, "Modbus Manager đã dừng");
    return ESP_OK;
}

void modbus_manager_register_callback(modbus_data_callback_t callback)
{
    modbus_ctx.callback = callback;
}

esp_err_t modbus_read_holding_registers(uint8_t slave_addr, uint16_t reg_addr, 
                                        uint16_t reg_count, uint16_t *data)
{
    if (!modbus_ctx.initialized || !data) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = modbus_lock();
    if (err != ESP_OK) return err;

    mb_param_request_t request = {
        .slave_addr = slave_addr,
        .command = 0x03,
        .reg_start = reg_addr,
        .reg_size = reg_count
    };

    err = mbc_master_send_request(modbus_ctx.master_handle, &request, data);
    
    if (err == ESP_OK && modbus_ctx.callback) {
        modbus_ctx.callback(slave_addr, 0x03, reg_addr, data, reg_count);
    }

    modbus_unlock();
    return err;
}

esp_err_t modbus_read_input_registers(uint8_t slave_addr, uint16_t reg_addr, 
                                      uint16_t reg_count, uint16_t *data)
{
    if (!modbus_ctx.initialized || !data) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = modbus_lock();
    if (err != ESP_OK) return err;

    mb_param_request_t request = {
        .slave_addr = slave_addr,
        .command = 0x04,
        .reg_start = reg_addr,
        .reg_size = reg_count
    };

    err = mbc_master_send_request(modbus_ctx.master_handle, &request, data);
    
    if (err == ESP_OK && modbus_ctx.callback) {
        modbus_ctx.callback(slave_addr, 0x04, reg_addr, data, reg_count);
    }

    modbus_unlock();
    return err;
}

esp_err_t modbus_write_single_register(uint8_t slave_addr, uint16_t reg_addr, 
                                       uint16_t value)
{
    if (!modbus_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = modbus_lock();
    if (err != ESP_OK) return err;

    mb_param_request_t request = {
        .slave_addr = slave_addr,
        .command = 0x06,
        .reg_start = reg_addr,
        .reg_size = 1
    };

    err = mbc_master_send_request(modbus_ctx.master_handle, &request, &value);
    
    modbus_unlock();
    return err;
}

esp_err_t modbus_write_multiple_registers(uint8_t slave_addr, uint16_t reg_addr,
                                          uint16_t reg_count, uint16_t *data)
{
    if (!modbus_ctx.initialized || !data) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = modbus_lock();
    if (err != ESP_OK) return err;

    mb_param_request_t request = {
        .slave_addr = slave_addr,
        .command = 0x10,
        .reg_start = reg_addr,
        .reg_size = reg_count
    };

    err = mbc_master_send_request(modbus_ctx.master_handle, &request, data);
    
    modbus_unlock();
    return err;
}

esp_err_t modbus_read_coils(uint8_t slave_addr, uint16_t coil_addr,
                            uint16_t coil_count, uint8_t *data)
{
    if (!modbus_ctx.initialized || !data) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = modbus_lock();
    if (err != ESP_OK) return err;

    mb_param_request_t request = {
        .slave_addr = slave_addr,
        .command = 0x01,
        .reg_start = coil_addr,
        .reg_size = coil_count
    };

    err = mbc_master_send_request(modbus_ctx.master_handle, &request, data);
    
    modbus_unlock();
    return err;
}

esp_err_t modbus_write_single_coil(uint8_t slave_addr, uint16_t coil_addr,
                                   bool value)
{
    if (!modbus_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = modbus_lock();
    if (err != ESP_OK) return err;

    uint16_t coil_value = value ? 0xFF00 : 0x0000;
    
    mb_param_request_t request = {
        .slave_addr = slave_addr,
        .command = 0x05,
        .reg_start = coil_addr,
        .reg_size = 1
    };

    err = mbc_master_send_request(modbus_ctx.master_handle, &request, &coil_value);
    
    modbus_unlock();
    return err;
}

bool modbus_manager_is_running(void)
{
    return modbus_ctx.running;
}

