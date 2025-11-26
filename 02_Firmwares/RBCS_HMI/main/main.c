#include "app_states.h"

static const char *TAG = "MAIN";

DeviceHSM_t device;

// Cấu hình Modbus
modbus_config_t modbus_cfg = {
    .uart_port = APP_IO_UART_NUM,
    .tx_pin = APP_IO_UART_TX_PIN,
    .rx_pin = APP_IO_UART_RX_PIN,
    .rts_pin = APP_IO_UART_RTS_PIN,
    .baudrate = 9600,
};

// ============================================
// LVGL Display Driver
// ============================================

// Dummy flush callback (thay thế bằng driver thật khi có màn hình)
static void dummy_flush_cb(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    // Không làm gì, chỉ báo LVGL là đã flush xong
    lv_disp_flush_ready(disp_drv);
}

// ============================================
// Modbus Callbacks & Tasks
// ============================================

// Callback nhận dữ liệu Modbus
void modbus_data_received(uint8_t slave_addr, uint8_t reg_type, uint16_t reg_addr, uint16_t *data, uint16_t length)
{
    ESP_LOGI(TAG, "Modbus: Nhận dữ liệu từ slave %d, type 0x%02X, addr %d:", slave_addr, reg_type, reg_addr);
    for (int i = 0; i < length; i++) {
        ESP_LOGI(TAG, "  [%d] = %d", i, data[i]);
    }
}

// Task Modbus polling
void modbus_poll_task(void *arg)
{
    uint16_t holding_regs[10];
    uint16_t input_regs[5];
    
    ESP_LOGI(TAG, "Modbus polling task started");

    while (1) {
        // Đọc Holding Registers
        if (modbus_read_holding_registers(1, 0, 10, holding_regs) == ESP_OK) {
            ESP_LOGI(TAG, "Modbus: Đọc Holding Registers OK");
        }
        vTaskDelay(pdMS_TO_TICKS(500));

        // Đọc Input Registers
        if (modbus_read_input_registers(1, 0, 5, input_regs) == ESP_OK) {
            ESP_LOGI(TAG, "Modbus: Đọc Input Registers OK");
        }
        vTaskDelay(pdMS_TO_TICKS(500));

        // Ghi register (ví dụ)
        // modbus_write_single_register(1, 0, 1234);
        // vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ============================================
// LVGL Task
// ============================================

void lvgl_task(void *arg)
{
    ESP_LOGI(TAG, "LVGL task started");
    
    while (1) {
        // LVGL timer handler - xử lý animations, events, etc.
        lv_timer_handler();
        
        // Delay 5ms - LVGL khuyến nghị
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ============================================
// Main Application Entry Point
// ============================================

void app_main(void)
{
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "  RBCS HMI - Robot Battery Charging Station");
    ESP_LOGI(TAG, "===========================================");

    // ========================================
    // BƯỚC 1: Khởi tạo LVGL
    // ========================================
    ESP_LOGI(TAG, "[1/5] Initializing LVGL...");
    lv_init();
    ESP_LOGI(TAG, "      LVGL initialized");

    // ========================================
    // BƯỚC 2: Cấp phát Display Buffer
    // ========================================
    ESP_LOGI(TAG, "[2/5] Allocating display buffer...");
    
    static lv_disp_draw_buf_t draw_buf;
    static lv_color_t *buf1;
    
    // Cấp phát buffer từ PSRAM (800x48 = 1/10 màn hình 800x480)
    size_t buf_size = 800 * 48 * sizeof(lv_color_t);
    buf1 = heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM);
    
    if (buf1 == NULL) {
        ESP_LOGE(TAG, "      FAILED to allocate LVGL buffer (%d bytes)", buf_size);
        ESP_LOGE(TAG, "      System halted!");
        return;
    }
    
    lv_disp_draw_buf_init(&draw_buf, buf1, NULL, 800 * 48);
    ESP_LOGI(TAG, "      Display buffer allocated: %d bytes from PSRAM", buf_size);

    // ========================================
    // BƯỚC 3: Đăng ký Display Driver
    // ========================================
    ESP_LOGI(TAG, "[3/5] Registering display driver...");
    
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.draw_buf = &draw_buf;
    disp_drv.flush_cb = dummy_flush_cb;  // TODO: Thay bằng driver thật (ILI9341, etc.)
    disp_drv.hor_res = 800;
    disp_drv.ver_res = 480;
    lv_disp_drv_register(&disp_drv);
    
    ESP_LOGI(TAG, "      Display driver registered (800x480)");
    ESP_LOGI(TAG, "      Using dummy flush callback (no physical display)");

    // ========================================
    // BƯỚC 4: Khởi tạo UI từ SquareLine Studio
    // ========================================
    ESP_LOGI(TAG, "[4/5] Initializing UI components...");
    
    ui_init();
    
    ESP_LOGI(TAG, "      UI initialized successfully");

    // ========================================
    // BƯỚC 5: Khởi tạo Modbus
    // ========================================
    ESP_LOGI(TAG, "[5/5] Initializing Modbus RTU Master...");
    
    esp_err_t ret = modbus_manager_init(&modbus_cfg);
    if (ret == ESP_OK) {
        modbus_manager_register_callback(modbus_data_received);
        ESP_LOGI(TAG, "      Modbus initialized successfully");
        ESP_LOGI(TAG, "      Port: UART%d, Baudrate: %d", modbus_cfg.uart_port, modbus_cfg.baudrate);
        ESP_LOGI(TAG, "      TX: GPIO%d, RX: GPIO%d, RTS: GPIO%d", 
                 modbus_cfg.tx_pin, modbus_cfg.rx_pin, modbus_cfg.rts_pin);
    } else {
        ESP_LOGE(TAG, "      Modbus initialization FAILED: %s", esp_err_to_name(ret));
    }

    // ========================================
    // Tạo Tasks
    // ========================================
    ESP_LOGI(TAG, "Creating tasks...");
    
    // Task LVGL handler (priority 10 - cao để UI mượt)
    BaseType_t lvgl_task_ret = xTaskCreate(
        lvgl_task,
        "lvgl_task",
        8192,           // Stack size: 8KB
        NULL,
        10,             // Priority: High
        NULL
    );
    
    if (lvgl_task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create LVGL task!");
    } else {
        ESP_LOGI(TAG, "  - LVGL task created (priority 10)");
    }
    
    // Task Modbus polling (priority 5 - trung bình)
    if (ret == ESP_OK) {
        BaseType_t modbus_task_ret = xTaskCreate(
            modbus_poll_task,
            "modbus_poll",
            4096,       // Stack size: 4KB
            NULL,
            5,          // Priority: Medium
            NULL
        );
        
        if (modbus_task_ret != pdPASS) {
            ESP_LOGE(TAG, "Failed to create Modbus task!");
        } else {
            ESP_LOGI(TAG, "  - Modbus polling task created (priority 5)");
        }
    }

    // ========================================
    // Khởi tạo HSM (State Machine)
    // ========================================
    ESP_LOGI(TAG, "Initializing HSM...");
    app_state_hsm_init(&device);
    ESP_LOGI(TAG, "  - HSM initialized");

    // ========================================
    // Hoàn tất khởi động
    // ========================================
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "  System startup completed successfully!");
    ESP_LOGI(TAG, "  LVGL: Running");
    ESP_LOGI(TAG, "  Modbus: %s", ret == ESP_OK ? "Running" : "Disabled");
    ESP_LOGI(TAG, "  Free heap: %d bytes (internal)", esp_get_free_heap_size());
    ESP_LOGI(TAG, "  Free PSRAM: %d bytes", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    ESP_LOGI(TAG, "===========================================");
    
    // app_main() kết thúc, nhường CPU cho các task khác
    // LVGL và Modbus tiếp tục chạy trong background
}