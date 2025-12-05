#include "app_states.h"

DeviceHSM_t mainboard;

static const char *TAG = "MAINBOARD";


#define MAINBOARD_ADDRESS           1   // Địa chỉ của MAINBOARD khi là Slave


// ============================================
// Utility Functions
// ============================================
static uint32_t get_time_ms(void)
{
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}
static bool take_data_mutex(uint32_t timeout_ms)
{
    if (mainboard.g_system.data_mutex == NULL) return false;
    return xSemaphoreTake(mainboard.g_system.data_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}
static void give_data_mutex(void)
{
    if (mainboard.g_system.data_mutex) {
        xSemaphoreGive(mainboard.g_system.data_mutex);
    }
}
// ============================================
// Update Station Info từ dữ liệu Pin
// ============================================
static void update_station_info(void)
{
    if (!take_data_mutex(100)) return;

    uint8_t active_count = 0;
    uint32_t total_volt = 0;
    uint32_t total_temp = 0;
    uint16_t min_soc = 100;
    uint16_t max_soc = 0;
    uint16_t alarm_bits = 0;
    uint16_t fault_bits = 0;

    for (int i = 0; i < TOTAL_SLOTS; i++) {
        if (mainboard.g_system.slot[i].is_online) {
            active_count++;
            
            total_volt += mainboard.g_system.slot[i].registers[REG_STA_PACK_VOLT];
            
            // Temperature (giả sử TEMP1_HIGH chứa nhiệt độ chính)
            total_temp += mainboard.g_system.slot[i].registers[REG_STA_TEMP1_HIGH];
            
            // SOC
            uint16_t soc = mainboard.g_system.slot[i].registers[REG_STA_SOC_PERCENT];
            if (soc < min_soc) min_soc = soc;
            if (soc > max_soc) max_soc = soc;
            
            // Alarms & Faults
            alarm_bits |= (mainboard.g_system.slot[i].registers[REG_STA_ALARM_BITS] != 0) ? (1 << i) : 0;
            fault_bits |= (mainboard.g_system.slot[i].registers[REG_STA_FAULTS] != 0) ? (1 << i) : 0;
        }
    }

    mainboard.g_station_info.active_batteries = active_count;
    mainboard.g_station_info.total_voltage = (active_count > 0) ? (total_volt / active_count) : 0;
    mainboard.g_station_info.avg_temperature = (active_count > 0) ? (total_temp / active_count) : 0;
    mainboard.g_station_info.min_soc = (active_count > 0) ? min_soc : 0;
    mainboard.g_station_info.max_soc = (active_count > 0) ? max_soc : 0;
    mainboard.g_station_info.alarm_status = alarm_bits;
    mainboard.g_station_info.fault_status = fault_bits;
    mainboard.g_station_info.system_state = (active_count > 0) ? 1 : 0; // 1=Running, 0=Stopped

    give_data_mutex();
}
// ============================================
// Sync dữ liệu Pin vào Slave Registers
// ============================================
static void sync_battery_to_slave_registers(uint8_t battery_index)
{
    if (battery_index >= TOTAL_SLOTS) return;
    
    if (!take_data_mutex(100)) return;

    uint16_t base_addr = battery_index * REGS_PER_BATTERY;
    Slot_Data_t *bat = &mainboard.g_system.slot[battery_index];

    // Copy tất cả thanh ghi từ Pin vào vùng Slave register tương ứng
    for (int i = 0; i < TOTAL_STA_REGISTERS && i < REGS_PER_BATTERY; i++) {
        modbus_slave_set_holding_register(base_addr + i, bat->registers[i]);
    }

    give_data_mutex();
}

// ============================================
// Sync Station Info vào Slave Registers
// ============================================
static void sync_station_info_to_slave(void)
{
    modbus_slave_set_holding_register(1000, mainboard.g_station_info.system_state);
    modbus_slave_set_holding_register(1001, mainboard.g_station_info.active_batteries);
    modbus_slave_set_holding_register(1002, mainboard.g_station_info.total_voltage);
    modbus_slave_set_holding_register(1003, mainboard.g_station_info.total_current_high);
    modbus_slave_set_holding_register(1004, mainboard.g_station_info.total_current_low);
    modbus_slave_set_holding_register(1005, mainboard.g_station_info.avg_temperature);
    modbus_slave_set_holding_register(1006, mainboard.g_station_info.min_soc);
    modbus_slave_set_holding_register(1007, mainboard.g_station_info.max_soc);
    modbus_slave_set_holding_register(1008, mainboard.g_station_info.alarm_status);
    modbus_slave_set_holding_register(1009, mainboard.g_station_info.fault_status);
}

// ============================================
// Modbus Master: Polling Task
// ============================================
static void modbus_master_poll_task(void *arg)
{
    ESP_LOGI(TAG, "📡 Modbus Master polling task started");
    
    uint16_t read_buffer[TOTAL_STA_REGISTERS];
    const uint32_t POLL_INTERVAL_MS = 500;      // Poll mỗi Pin 500ms
    const uint32_t OFFLINE_TIMEOUT_MS = 3000;   // Timeout 3s
    const uint8_t MAX_ERRORS_BEFORE_OFFLINE = 5;

    while (1) {
        mainboard.g_system.total_poll_count++;

        for (uint8_t bat_id = 1; bat_id <= TOTAL_SLOTS; bat_id++) {
            uint8_t bat_index = bat_id - 1;
            
            // Đọc 50 thanh ghi từ Pin (FC03, địa chỉ 0, 50 registers)
            esp_err_t err = modbus_master_read_holding_registers(
                bat_id,                     // Slave address (1-5)
                0,                          // Starting register
                TOTAL_STA_REGISTERS,        // Number of registers
                read_buffer
            );

            if (err == ESP_OK) {
                // ✅ Đọc thành công
                if (take_data_mutex(100)) {
                    memcpy(mainboard.g_system.slot[bat_index].registers, 
                           read_buffer, 
                           sizeof(read_buffer));
                    
                    mainboard.g_system.slot[bat_index].is_online = true;
                    mainboard.g_system.slot[bat_index].last_update_ms = get_time_ms();
                    mainboard.g_system.slot[bat_index].error_count = 0;
                    
                    give_data_mutex();
                }

                // Đồng bộ vào Slave registers
                sync_battery_to_slave_registers(bat_index);

                ESP_LOGI(TAG, "✅ Battery %d: Volt=%d, SOC=%d%%, Temp=%d", 
                         bat_id,
                         read_buffer[REG_STA_PACK_VOLT],
                         read_buffer[REG_STA_SOC_PERCENT],
                         read_buffer[REG_STA_TEMP1_HIGH]);

            } else {
                // ❌ Lỗi đọc
                ESP_LOGW(TAG, "❌ Battery %d read failed: %s", 
                         bat_id, esp_err_to_name(err));
                
                if (take_data_mutex(100)) {
                    mainboard.g_system.slot[bat_index].error_count++;
                    
                    // Nếu quá nhiều lỗi → đánh dấu offline
                    if (mainboard.g_system.slot[bat_index].error_count >= MAX_ERRORS_BEFORE_OFFLINE) {
                        mainboard.g_system.slot[bat_index].is_online = false;
                    }
                    
                    mainboard.g_system.error_count++;
                    give_data_mutex();
                }

                // Flush UART để xóa dữ liệu lỗi
                uart_flush(MASTER_UART_PORT);
            }

            vTaskDelay(pdMS_TO_TICKS(50)); // Delay giữa các lần đọc
        }

        // Check timeout cho các Pin không phản hồi lâu
        if (take_data_mutex(100)) {
            uint32_t now = get_time_ms();
            for (int i = 0; i < TOTAL_SLOTS; i++) {
                if (mainboard.g_system.slot[i].is_online) {
                    if ((now - mainboard.g_system.slot[i].last_update_ms) > OFFLINE_TIMEOUT_MS) {
                        mainboard.g_system.slot[i].is_online = false;
                        ESP_LOGW(TAG, "⏱️  Battery %d timeout (offline)", i + 1);
                    }
                }
            }
            give_data_mutex();
        }

        // Cập nhật Station Info
        update_station_info();
        sync_station_info_to_slave();

        // Log trạng thái tổng quan
        ESP_LOGI(TAG, "📊 Poll #%lu | Online: %d/%d | Errors: %lu",
                 mainboard.g_system.total_poll_count,
                 mainboard.g_station_info.active_batteries,
                 TOTAL_SLOTS,
                 mainboard.g_system.error_count);

        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }
}
// ============================================
// Modbus Slave: Callback khi HMI ghi dữ liệu
// ============================================
static void modbus_slave_write_callback(uint8_t reg_type, uint16_t address, uint16_t value)
{
    ESP_LOGI(TAG, "📝 HMI wrote: type=%d, addr=%d, value=%d", reg_type, address, value);
    
    // Xử lý lệnh từ HMI (nếu cần)
    // Ví dụ: HMI ghi vào register 2000 để điều khiển
    if (address == 2000) {
        ESP_LOGI(TAG, "🎛️  Received command from HMI: %d", value);
        // TODO: Xử lý lệnh
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "==================================================================");
    ESP_LOGI(TAG, "Starting Robot Battery Charging Station Mainboard firmware...");
    ESP_LOGI(TAG, "==================================================================");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Create mutex
    mainboard.g_system.data_mutex = xSemaphoreCreateMutex();
    if (mainboard.g_system.data_mutex == NULL) {
        ESP_LOGE(TAG, "❌ Failed to create mutex!");
        return;
    }

    // Initialize ticks system
    ESP_ERROR_CHECK(ticks_init());

    app_state_hsm_init(&mainboard);
    // ============================================
    // STEP 1: Initialize Modbus MASTER
    // ============================================
    ESP_LOGI(TAG, "[1/3] Initializing Modbus MASTER...");
    modbus_master_config_t master_cfg = {
        .uart_port = MASTER_UART_PORT,
        .tx_pin = MASTER_TX_PIN,
        .rx_pin = MASTER_RX_PIN,
        .rts_pin = MASTER_RTS_PIN,
        .baudrate = 115200,
    };

    ret = modbus_master_init(&master_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Modbus Master init failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "      ✅ Master: UART%d @ %lu baud (TX=%d, RX=%d, RTS=%d)",
             master_cfg.uart_port, master_cfg.baudrate,
             master_cfg.tx_pin, master_cfg.rx_pin, master_cfg.rts_pin);    

    // ============================================
    // STEP 2: Initialize Modbus SLAVE
    // ============================================
    ESP_LOGI(TAG, "[2/3] Initializing Modbus SLAVE...");
    modbus_slave_config_t slave_cfg = {
        .slave_addr = MAINBOARD_ADDRESS,
        .uart_port = SLAVE_UART_PORT,
        .tx_pin = SLAVE_TX_PIN,
        .rx_pin = SLAVE_RX_PIN,
        .rts_pin = SLAVE_RTS_PIN,
        .baudrate = 115200,
    };

    ret = modbus_slave_init(&slave_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Modbus Slave init failed: %s", esp_err_to_name(ret));
        modbus_master_deinit();
        return;
    }        
    // Register callback
    modbus_slave_register_callback(modbus_slave_write_callback);

    ESP_LOGI(TAG, "      ✅ Slave: Addr=%d, UART%d @ %lu baud (TX=%d, RX=%d, RTS=%d)",
             slave_cfg.slave_addr, slave_cfg.uart_port, slave_cfg.baudrate,
             slave_cfg.tx_pin, slave_cfg.rx_pin, slave_cfg.rts_pin);

    // ============================================
    // STEP 3: Khởi tạo dữ liệu mặc định
    // ============================================
    ESP_LOGI(TAG, "[3/3] Initializing default data...");
    for (int i = 0; i < TOTAL_SLOTS; i++) {
        memset(&mainboard.g_system.slot[i], 0, sizeof(Slot_Data_t));
        mainboard.g_system.slot[i].is_online = false;
    }

    // Khởi tạo Station Info registers với giá trị mặc định
    for (uint16_t addr = 1000; addr < 1010; addr++) {
        modbus_slave_set_holding_register(addr, 0);
    }

    ESP_LOGI(TAG, "      ✅ Data structures initialized");

    // ============================================
    // STEP 4: Create Polling Task
    // ============================================
    ESP_LOGI(TAG, "Creating Modbus Master polling task...");
    BaseType_t task_ret = xTaskCreate(
        modbus_master_poll_task,
        "modbus_master_poll",
        4096,
        NULL,
        5,
        NULL
    );

    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "❌ Failed to create polling task!");
        modbus_slave_deinit();
        modbus_master_deinit();
        return;
    }

    ESP_LOGI(TAG, "      ✅ Polling task created");

    // ============================================
    // System Ready
    // ============================================
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "  ✅ RBCS MAINBOARD Ready!");
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "  Master: Reading %d Batteries (Addr 1-5)", TOTAL_SLOTS);
    ESP_LOGI(TAG, "  Slave:  Serving HMI (Addr %d)", MAINBOARD_ADDRESS);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "  📋 Register Map for HMI:");
    ESP_LOGI(TAG, "     0-49:    Battery 1");
    ESP_LOGI(TAG, "     50-99:   Battery 2");
    ESP_LOGI(TAG, "     100-149: Battery 3");
    ESP_LOGI(TAG, "     150-199: Battery 4");
    ESP_LOGI(TAG, "     200-249: Battery 5");
    ESP_LOGI(TAG, "     1000+:   Station Info");
    ESP_LOGI(TAG, "===========================================");
}

