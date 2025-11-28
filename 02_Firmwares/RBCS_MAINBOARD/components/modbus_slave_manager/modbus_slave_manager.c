#include "modbus_slave_manager.h"
#include "esp_modbus_slave.h"
#include "mbcontroller.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "MODBUS_SLAVE";

// Register/coil count definitions
#define MB_SLAVE_HOLDING_REG_COUNT    100
#define MB_SLAVE_INPUT_REG_COUNT      50
#define MB_SLAVE_COIL_COUNT           50
#define MB_SLAVE_DISCRETE_INPUT_COUNT 50

// Internal state
static struct {
    void *slave_handle;
    modbus_slave_config_t config;
    modbus_slave_change_callback_t callback;
    TaskHandle_t task_handle;
    bool initialized;
    bool running;
    
    // Data areas
    uint16_t holding_registers[MB_SLAVE_HOLDING_REG_COUNT];
    uint16_t input_registers[MB_SLAVE_INPUT_REG_COUNT];
    uint8_t coils[MB_SLAVE_COIL_COUNT];
    uint8_t discrete_inputs[MB_SLAVE_DISCRETE_INPUT_COUNT];
} slave_ctx = {0};

// Task to handle Modbus events
static void modbus_slave_task(void *arg)
{
    ESP_LOGI(TAG, "Modbus Slave task started");
    
    // Event mask - wait for all events
    const mb_event_group_t event_mask = MB_EVENT_HOLDING_REG_WR | 
                                         MB_EVENT_COILS_WR;
    
    while (slave_ctx.running) {
        // API v2.1.0: mb_event_group_t mbc_slave_check_event(void *ctx, mb_event_group_t group)
        // Function blocks until event occurs, returns occurred event
        mb_event_group_t event = mbc_slave_check_event(slave_ctx.slave_handle, event_mask);
        
        if (event != 0) {
            if (event & MB_EVENT_HOLDING_REG_WR) {
                ESP_LOGI(TAG, "Holding Register written by Master");
                
                if (slave_ctx.callback) {
                    slave_ctx.callback(MB_PARAM_HOLDING, 0, 0);
                }
            }
            
            if (event & MB_EVENT_COILS_WR) {
                ESP_LOGI(TAG, "Coil written by Master");
                
                if (slave_ctx.callback) {
                    slave_ctx.callback(MB_PARAM_COIL, 0, 0);
                }
            }
        }
    }
    
    ESP_LOGI(TAG, "Modbus Slave task stopped");
    vTaskDelete(NULL);
}

// Public API implementation
esp_err_t modbus_slave_init(const modbus_slave_config_t *config)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }

    if (slave_ctx.initialized) {
        ESP_LOGW(TAG, "Modbus Slave already initialized");
        return ESP_OK;
    }

    // Save configuration
    memcpy(&slave_ctx.config, config, sizeof(modbus_slave_config_t));

    // Initialize default data
    memset(slave_ctx.holding_registers, 0, sizeof(slave_ctx.holding_registers));
    memset(slave_ctx.input_registers, 0, sizeof(slave_ctx.input_registers));
    memset(slave_ctx.coils, 0, sizeof(slave_ctx.coils));
    memset(slave_ctx.discrete_inputs, 0, sizeof(slave_ctx.discrete_inputs));

    // Configure Modbus communication
    mb_communication_info_t comm_info = {0};
    comm_info.ser_opts.mode = MB_RTU;
    comm_info.ser_opts.port = config->uart_port;
    comm_info.ser_opts.baudrate = config->baudrate;
    comm_info.ser_opts.data_bits = UART_DATA_8_BITS;
    comm_info.ser_opts.parity = MB_PARITY_NONE;
    comm_info.ser_opts.stop_bits = UART_STOP_BITS_1;
    comm_info.ser_opts.uid = config->slave_addr;

    // Create Modbus Slave
    esp_err_t err = mbc_slave_create_serial(&comm_info, &slave_ctx.slave_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create Modbus Slave: %s", esp_err_to_name(err));
        return err;
    }

    // Configure GPIO pins
    err = uart_set_pin(config->uart_port, config->tx_pin, config->rx_pin, 
                       config->rts_pin, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO: %s", esp_err_to_name(err));
        mbc_slave_delete(slave_ctx.slave_handle);
        return err;
    }

    // Setup Holding Registers
    mb_register_area_descriptor_t reg_area;
    memset(&reg_area, 0, sizeof(mb_register_area_descriptor_t));
    reg_area.type = MB_PARAM_HOLDING;
    reg_area.start_offset = 0;
    reg_area.address = (void*)slave_ctx.holding_registers;
    reg_area.size = sizeof(slave_ctx.holding_registers);
    
    err = mbc_slave_set_descriptor(slave_ctx.slave_handle, reg_area);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set holding registers: %s", esp_err_to_name(err));
        mbc_slave_delete(slave_ctx.slave_handle);
        return err;
    }

    // Setup Input Registers
    memset(&reg_area, 0, sizeof(mb_register_area_descriptor_t));
    reg_area.type = MB_PARAM_INPUT;
    reg_area.start_offset = 0;
    reg_area.address = (void*)slave_ctx.input_registers;
    reg_area.size = sizeof(slave_ctx.input_registers);
    
    err = mbc_slave_set_descriptor(slave_ctx.slave_handle, reg_area);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set input registers: %s", esp_err_to_name(err));
        mbc_slave_delete(slave_ctx.slave_handle);
        return err;
    }

    // Setup Coils
    memset(&reg_area, 0, sizeof(mb_register_area_descriptor_t));
    reg_area.type = MB_PARAM_COIL;
    reg_area.start_offset = 0;
    reg_area.address = (void*)slave_ctx.coils;
    reg_area.size = sizeof(slave_ctx.coils);
    
    err = mbc_slave_set_descriptor(slave_ctx.slave_handle, reg_area);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set coils: %s", esp_err_to_name(err));
        mbc_slave_delete(slave_ctx.slave_handle);
        return err;
    }

    // Setup Discrete Inputs
    memset(&reg_area, 0, sizeof(mb_register_area_descriptor_t));
    reg_area.type = MB_PARAM_DISCRETE;
    reg_area.start_offset = 0;
    reg_area.address = (void*)slave_ctx.discrete_inputs;
    reg_area.size = sizeof(slave_ctx.discrete_inputs);
    
    err = mbc_slave_set_descriptor(slave_ctx.slave_handle, reg_area);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set discrete inputs: %s", esp_err_to_name(err));
        mbc_slave_delete(slave_ctx.slave_handle);
        return err;
    }

    // Start Modbus stack
    err = mbc_slave_start(slave_ctx.slave_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start Modbus Slave: %s", esp_err_to_name(err));
        mbc_slave_delete(slave_ctx.slave_handle);
        return err;
    }

    slave_ctx.initialized = true;
    slave_ctx.running = true;

    // Create task to handle events
    BaseType_t ret = xTaskCreate(
        modbus_slave_task,
        "modbus_slave",
        4096,
        NULL,
        5,
        &slave_ctx.task_handle
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create task");
        mbc_slave_stop(slave_ctx.slave_handle);
        mbc_slave_delete(slave_ctx.slave_handle);
        slave_ctx.initialized = false;
        slave_ctx.running = false;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Modbus Slave initialized successfully");
    ESP_LOGI(TAG, "  Slave Addr: %d", config->slave_addr);
    ESP_LOGI(TAG, "  Port: UART%d, Baudrate: %lu", config->uart_port, config->baudrate);
    ESP_LOGI(TAG, "  TX: GPIO%d, RX: GPIO%d, RTS: GPIO%d", 
             config->tx_pin, config->rx_pin, config->rts_pin);
    ESP_LOGI(TAG, "  Holding Regs: %d, Input Regs: %d", 
             MB_SLAVE_HOLDING_REG_COUNT, MB_SLAVE_INPUT_REG_COUNT);
    ESP_LOGI(TAG, "  Coils: %d, Discrete Inputs: %d", 
             MB_SLAVE_COIL_COUNT, MB_SLAVE_DISCRETE_INPUT_COUNT);

    return ESP_OK;
}

esp_err_t modbus_slave_deinit(void)
{
    if (!slave_ctx.initialized) {
        return ESP_OK;
    }

    slave_ctx.running = false;
    
    // Wait for task to finish
    if (slave_ctx.task_handle) {
        vTaskDelay(pdMS_TO_TICKS(200));
        slave_ctx.task_handle = NULL;
    }
    
    if (slave_ctx.slave_handle) {
        mbc_slave_stop(slave_ctx.slave_handle);
        mbc_slave_delete(slave_ctx.slave_handle);
        slave_ctx.slave_handle = NULL;
    }

    slave_ctx.initialized = false;
    slave_ctx.callback = NULL;

    ESP_LOGI(TAG, "Modbus Slave stopped");
    return ESP_OK;
}

void modbus_slave_register_callback(modbus_slave_change_callback_t callback)
{
    slave_ctx.callback = callback;
}

esp_err_t modbus_slave_set_holding_register(uint16_t address, uint16_t value)
{
    if (!slave_ctx.initialized || address >= MB_SLAVE_HOLDING_REG_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    slave_ctx.holding_registers[address] = value;
    return ESP_OK;
}

esp_err_t modbus_slave_get_holding_register(uint16_t address, uint16_t *value)
{
    if (!slave_ctx.initialized || !value || address >= MB_SLAVE_HOLDING_REG_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    *value = slave_ctx.holding_registers[address];
    return ESP_OK;
}

esp_err_t modbus_slave_set_input_register(uint16_t address, uint16_t value)
{
    if (!slave_ctx.initialized || address >= MB_SLAVE_INPUT_REG_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    slave_ctx.input_registers[address] = value;
    return ESP_OK;
}

esp_err_t modbus_slave_set_coil(uint16_t address, bool value)
{
    if (!slave_ctx.initialized || address >= MB_SLAVE_COIL_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    slave_ctx.coils[address] = value ? 1 : 0;
    return ESP_OK;
}

esp_err_t modbus_slave_get_coil(uint16_t address, bool *value)
{
    if (!slave_ctx.initialized || !value || address >= MB_SLAVE_COIL_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    *value = slave_ctx.coils[address] ? true : false;
    return ESP_OK;
}

esp_err_t modbus_slave_set_discrete_input(uint16_t address, bool value)
{
    if (!slave_ctx.initialized || address >= MB_SLAVE_DISCRETE_INPUT_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    slave_ctx.discrete_inputs[address] = value ? 1 : 0;
    return ESP_OK;
}

bool modbus_slave_is_running(void)
{
    return slave_ctx.running;
}

