#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

#include "app_states.h"

DeviceHSM_t chrg_main;


static const char *TAG = "MODBUS_SLAVE";



// Địa chỉ Modbus Slave
#define SLAVE_ADDRESS       1

// Callback khi Master ghi dữ liệu
void on_modbus_change(uint8_t reg_type, uint16_t address, uint16_t value)
{
    ESP_LOGI(TAG, "Master ghi: type=%d, addr=%d, value=%d", reg_type, address, value);
}

// Task cập nhật dữ liệu định kỳ
void data_update_task(void *arg)
{
    uint16_t counter = 0;
    
    while (1) {
        // Cập nhật Input Registers (giả lập sensor data)
        modbus_slave_set_input_register(0, counter);
        modbus_slave_set_input_register(1, counter * 2);
        modbus_slave_set_input_register(2, 25 + (counter % 10));  // Temperature 25-35°C
        
        counter++;
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "  Modbus Slave Test");
    ESP_LOGI(TAG, "===========================================");

    // Cấu hình Modbus Slave
    modbus_slave_config_t config = {
        .slave_addr = SLAVE_ADDRESS,
        .uart_port = SLAVE_UART_PORT,
        .tx_pin = SLAVE_TX_PIN,
        .rx_pin = SLAVE_RX_PIN,
        .rts_pin = SLAVE_RTS_PIN,
        .baudrate = 9600,
    };

    // Khởi tạo Modbus Slave
    esp_err_t ret = modbus_slave_init(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Lỗi khởi tạo Modbus Slave!");
        return;
    }

    // Đăng ký callback
    modbus_slave_register_callback(on_modbus_change);

    // Khởi tạo dữ liệu mặc định cho Holding Registers
    for (int i = 0; i < 10; i++) {
        modbus_slave_set_holding_register(i, i * 100);
    }

    // Khởi tạo dữ liệu cho Input Registers
    for (int i = 0; i < 5; i++) {
        modbus_slave_set_input_register(i, i * 10);
    }

    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "Modbus Slave đang chạy tại địa chỉ %d", SLAVE_ADDRESS);
    ESP_LOGI(TAG, "Baudrate: 9600, UART%d", SLAVE_UART_PORT);
    ESP_LOGI(TAG, "TX: GPIO%d, RX: GPIO%d, RTS: GPIO%d", 
             SLAVE_TX_PIN, SLAVE_RX_PIN, SLAVE_RTS_PIN);
    ESP_LOGI(TAG, "===========================================");

    // Tạo task cập nhật dữ liệu
    xTaskCreate(
        data_update_task,
        "data_update",
        4096,
        NULL,
        5,
        NULL
    );

    ESP_LOGI(TAG, "Sẵn sàng nhận lệnh từ Master...");

    // Initialize ticks system
    ESP_ERROR_CHECK(ticks_init());

    app_state_hsm_init(&chrg_main);
    
}

