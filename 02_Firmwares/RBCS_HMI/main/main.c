#include "app_states.h"
#include "modbus_master_manager.h"
#include "ui.h"
#include "ui_support.h"

static const char *TAG = "RBCS_HMI";

// ============================================
// Modbus & HSM Variables
// ============================================
DeviceHSM_t device;

modbus_master_config_t modbus_cfg = {
    .uart_port = APP_IO_UART_NUM,
    .tx_pin = APP_IO_UART_TX_PIN,
    .rx_pin = APP_IO_UART_RX_PIN,
    .rts_pin = APP_IO_UART_RTS_PIN,
    .baudrate = 9600,
};

// LCD Configuration
#define LCD_NUM_FB             2  // Double buffer for maximum speed
#define LCD_H_RES              800
#define LCD_V_RES              480

// LVGL Task Configuration
#define LCD_LVGL_TICK_PERIOD_MS    2
#define LCD_LVGL_TASK_MAX_DELAY_MS 500
#define LCD_LVGL_TASK_MIN_DELAY_MS 1
#define LCD_LVGL_TASK_STACK_SIZE   (4 * 1024)
#define LCD_LVGL_TASK_PRIORITY     2

// ============================================
// Global Variables
// ============================================
static SemaphoreHandle_t lvgl_mux = NULL;
static SemaphoreHandle_t sem_vsync_end;
static SemaphoreHandle_t sem_gui_ready;

// ============================================
// SquareLine UI Event Handler
// ============================================
static uint8_t pic_num = 1;

static void screen_img_num4event_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    
    if (code == LV_EVENT_CLICKED) {
        pic_num++;
        if (pic_num == 6) pic_num = 1;
        
        switch (pic_num) {
            case 1:
                ui_image_set_src(ui_Image4, &ui_img_scrmain_batteryempty_png);
                break;
            case 2:
                ui_image_set_src(ui_Image4, &ui_img_scrsettingicon_png);
                break;
            case 3:
                ui_image_set_src(ui_Image4, &ui_img_scrsplash_background_png);
                break;
            default:
                break;
        }
    }
}

// ============================================
// Modbus Callbacks & Task
// ============================================
void modbus_data_received(uint8_t slave_addr, uint8_t reg_type, 
                          uint16_t reg_addr, uint16_t *data, uint16_t length)
{
    ESP_LOGI(TAG, "Modbus: Slave %d, type 0x%02X, addr %d", 
             slave_addr, reg_type, reg_addr);
    for (int i = 0; i < length; i++) {
        ESP_LOGI(TAG, "  [%d] = %d", i, data[i]);
    }
    
    // Example: Update UI with Modbus data
    // ui_label_set_text_fmt(ui_LabelStatus, "Value: %d", data[0]);
}

void modbus_poll_task(void *arg)
{
    uint16_t holding_regs[10];
    uint16_t input_regs[5];
    
    ESP_LOGI(TAG, "Modbus polling task started");

    while (1) {
        if (modbus_master_read_holding_registers(1, 0, 10, holding_regs) == ESP_OK) {
            ESP_LOGI(TAG, "Modbus: Read Holding OK");
        }
        vTaskDelay(pdMS_TO_TICKS(500));

        if (modbus_master_read_input_registers(1, 0, 5, input_regs) == ESP_OK) {
            ESP_LOGI(TAG, "Modbus: Read Input OK");
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ============================================
// LCD & LVGL Callbacks
// ============================================
static bool lcd_on_vsync_event(esp_lcd_panel_handle_t panel, 
                                   const esp_lcd_rgb_panel_event_data_t *event_data, 
                                   void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
    if (xSemaphoreTakeFromISR(sem_gui_ready, &high_task_awoken) == pdTRUE) {
        xSemaphoreGiveFromISR(sem_vsync_end, &high_task_awoken);
    }
    return high_task_awoken == pdTRUE;
}

static void lcd_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    
    xSemaphoreGive(sem_gui_ready);
    xSemaphoreTake(sem_vsync_end, portMAX_DELAY);
    
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
    lv_disp_flush_ready(drv);
}

static void lcd_increase_lvgl_tick(void *arg)
{
    lv_tick_inc(LCD_LVGL_TICK_PERIOD_MS);
}

// ============================================
// LVGL Port Task
// ============================================
static void lcd_lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t task_delay_ms = LCD_LVGL_TASK_MAX_DELAY_MS;
    
    while (1) {
        if (ui_lock(-1)) {
            task_delay_ms = lv_timer_handler();
            ui_unlock();
        }
        
        if (task_delay_ms > LCD_LVGL_TASK_MAX_DELAY_MS) {
            task_delay_ms = LCD_LVGL_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < LCD_LVGL_TASK_MIN_DELAY_MS) {
            task_delay_ms = LCD_LVGL_TASK_MIN_DELAY_MS;
        }
        
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

// ============================================
// I2C Initialization
// ============================================
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        return err;
    }
    
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 
                             I2C_MASTER_RX_BUF_DISABLE, 
                             I2C_MASTER_TX_BUF_DISABLE, 0);
}

// ============================================
// Touch Controller Initialization
// ============================================
static esp_err_t touch_controller_init(void)
{
    ESP_LOGI(TAG, "Initializing GT911 touch controller...");
    
    // Reset I2C bus
    i2c_driver_delete(I2C_MASTER_NUM);
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_ERROR_CHECK(i2c_master_init());
    
    // Delay for touch IC boot
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Retry logic for stable initialization
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_touch_handle_t tp = NULL;
    int retry = 0;
    const int max_retry = 3;
    
    while (retry < max_retry && tp == NULL) {
        if (retry > 0) {
            ESP_LOGW(TAG, "  Touch retry #%d...", retry);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        
        esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
        esp_err_t tp_io_ret = esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_MASTER_NUM, 
                                                        &tp_io_config, &tp_io_handle);
        
        if (tp_io_ret == ESP_OK) {
            esp_lcd_touch_config_t tp_cfg = {
                .x_max = LCD_H_RES,
                .y_max = LCD_V_RES,
                .rst_gpio_num = GPIO_NUM_NC,
                .int_gpio_num = GPIO_INPUT_IO_4,
                .levels = {
                    .reset = 0,
                    .interrupt = 0,
                },
                .flags = {
                    .swap_xy = 0,
                    .mirror_x = 0,
                    .mirror_y = 0,
                },
            };
            
            esp_err_t touch_ret = esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, &tp);
            
            if (touch_ret == ESP_OK) {
                ESP_LOGI(TAG, "  GT911 OK after %d retries!", retry);
                return ESP_OK;
            }
        }
        retry++;
    }
    
    ESP_LOGW(TAG, "  GT911 failed after %d retries - Touch disabled", max_retry);
    return ESP_FAIL;
}

// ============================================
// Main Application
// ============================================
void app_main(void)
{
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "  RBCS HMI - Battery Charging Station");
    ESP_LOGI(TAG, "===========================================");

    // ========================================
    // STEP 1: Initialize I2C
    // ========================================
    ESP_LOGI(TAG, "[1/7] Initializing I2C...");
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "      I2C initialized");

    // ========================================
    // STEP 2: Initialize LVGL
    // ========================================
    ESP_LOGI(TAG, "[2/7] Initializing LVGL...");
    lv_init();
    ESP_LOGI(TAG, "      LVGL initialized");

    // ========================================
    // STEP 3: Allocate Display Buffers
    // ========================================
    ESP_LOGI(TAG, "[3/7] Allocating LVGL draw buffers...");
    static lv_disp_draw_buf_t disp_buf;
    void *buf1 = NULL;
    void *buf2 = NULL;
    size_t buffer_size = LCD_H_RES * LCD_V_RES * sizeof(lv_color_t);
    
    buf1 = heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
    buf2 = heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
    assert(buf1 && buf2);
    
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, LCD_H_RES * LCD_V_RES);
    ESP_LOGI(TAG, "      Double buffering: %d bytes x2", buffer_size);

    // ========================================
    // STEP 4: Initialize RGB LCD Panel
    // ========================================
    ESP_LOGI(TAG, "[4/7] Installing RGB LCD panel...");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_rgb_panel_config_t panel_config = {
        .data_width = 16,
        .psram_trans_align = 64,
        .num_fbs = LCD_NUM_FB,
        .bounce_buffer_size_px = 10 * LCD_H_RES,
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .disp_gpio_num = LCD_PIN_NUM_DISP_EN,
        .pclk_gpio_num = LCD_PIN_NUM_PCLK,
        .vsync_gpio_num = LCD_PIN_NUM_VSYNC,
        .hsync_gpio_num = LCD_PIN_NUM_HSYNC,
        .de_gpio_num = LCD_PIN_NUM_DE,
        .data_gpio_nums = {
            LCD_PIN_NUM_DATA0,  LCD_PIN_NUM_DATA1,  LCD_PIN_NUM_DATA2,
            LCD_PIN_NUM_DATA3,  LCD_PIN_NUM_DATA4,  LCD_PIN_NUM_DATA5,
            LCD_PIN_NUM_DATA6,  LCD_PIN_NUM_DATA7,  LCD_PIN_NUM_DATA8,
            LCD_PIN_NUM_DATA9,  LCD_PIN_NUM_DATA10, LCD_PIN_NUM_DATA11,
            LCD_PIN_NUM_DATA12, LCD_PIN_NUM_DATA13, LCD_PIN_NUM_DATA14,
            LCD_PIN_NUM_DATA15,
        },
        .timings = {
            .pclk_hz = LCD_PIXEL_CLOCK_HZ,
            .h_res = LCD_H_RES,
            .v_res = LCD_V_RES,
            .hsync_back_porch = 8,
            .hsync_front_porch = 8,
            .hsync_pulse_width = 4,
            .vsync_back_porch = 8,
            .vsync_front_porch = 8,
            .vsync_pulse_width = 4,
            .flags.pclk_active_neg = true,
        },
        .flags.fb_in_psram = true,
        .flags.double_fb = true,
    };
    
    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));

    // Create semaphores for tear effect avoidance
    sem_vsync_end = xSemaphoreCreateBinary();
    assert(sem_vsync_end);
    sem_gui_ready = xSemaphoreCreateBinary();
    assert(sem_gui_ready);

    // Register VSYNC callback
    esp_lcd_rgb_panel_event_callbacks_t cbs = {
        .on_vsync = lcd_on_vsync_event,
    };
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(panel_handle, &cbs, NULL));

    // Initialize the LCD panel
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_LOGI(TAG, "      RGB LCD panel initialized");

    // ========================================
    // STEP 5: Register LVGL Display Driver
    // ========================================
    ESP_LOGI(TAG, "[5/7] Registering LVGL display driver...");
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_H_RES;
    disp_drv.ver_res = LCD_V_RES;
    disp_drv.flush_cb = lcd_lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    disp_drv.full_refresh = true;
    lv_disp_drv_register(&disp_drv);
    ESP_LOGI(TAG, "      Display driver registered (%dx%d)", LCD_H_RES, LCD_V_RES);

    // ========================================
    // STEP 6: Initialize Touch Controller
    // ========================================
    ESP_LOGI(TAG, "[6/7] Initializing touch controller...");
    esp_err_t touch_ret = touch_controller_init();
    if (touch_ret == ESP_OK) {
        ESP_LOGI(TAG, "      Touch controller initialized");
    } else {
        ESP_LOGW(TAG, "      Touch controller disabled");
    }

    // ========================================
    // STEP 7: Create LVGL Infrastructure
    // ========================================
    ESP_LOGI(TAG, "[7/7] Creating LVGL infrastructure...");
    
    // Create LVGL tick timer
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &lcd_increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LCD_LVGL_TICK_PERIOD_MS * 1000));

    // Create LVGL mutex
    lvgl_mux = xSemaphoreCreateRecursiveMutex();
    assert(lvgl_mux);

    // Initialize UI Support Library
    ESP_LOGI(TAG, "      Initializing UI Support...");
    ui_support_init(lvgl_mux);

    // Create LVGL task
    xTaskCreate(lcd_lvgl_port_task, "LVGL", 
                LCD_LVGL_TASK_STACK_SIZE, NULL, 
                LCD_LVGL_TASK_PRIORITY, NULL);
    ESP_LOGI(TAG, "      LVGL task created");

    // ========================================
    // Initialize SquareLine UI
    // ========================================
    ESP_LOGI(TAG, "Initializing SquareLine UI...");
    if (ui_lock(-1)) {
        ui_init();
        lv_obj_add_event_cb(ui_Image4, screen_img_num4event_handler, LV_EVENT_CLICKED, NULL);
        ui_unlock();
    }
    ESP_LOGI(TAG, "      SquareLine UI initialized");

    // ========================================
    // Initialize Modbus RTU Master
    // ========================================
    ESP_LOGI(TAG, "Initializing Modbus RTU Master...");
    esp_err_t modbus_ret = modbus_master_init(&modbus_cfg);
    if (modbus_ret == ESP_OK) {
        modbus_master_register_callback(modbus_data_received);
        ESP_LOGI(TAG, "      Modbus: UART%d @ %d baud", 
                 modbus_cfg.uart_port, modbus_cfg.baudrate);
        ESP_LOGI(TAG, "      Pins: TX=%d RX=%d RTS=%d", 
                 modbus_cfg.tx_pin, modbus_cfg.rx_pin, modbus_cfg.rts_pin);
        
        // Create Modbus polling task
        xTaskCreate(modbus_poll_task, "modbus_poll", 4096, NULL, 5, NULL);
        ESP_LOGI(TAG, "      Modbus task created");
    } else {
        ESP_LOGE(TAG, "      Modbus FAILED: %s", esp_err_to_name(modbus_ret));
    }

    // ========================================
    // Initialize HSM (State Machine)
    // ========================================
    ESP_LOGI(TAG, "Initializing HSM...");
    ESP_ERROR_CHECK(ticks_init());
    app_state_hsm_init(&device);
    ESP_LOGI(TAG, "      HSM initialized");

    // ========================================
    // System Startup Complete
    // ========================================
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "  ✓ System Startup Completed!");
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "  LCD:    800x480 RGB (Double Buffer)");
    ESP_LOGI(TAG, "  Touch:  %s", touch_ret == ESP_OK ? "GT911 Active" : "Disabled");
    ESP_LOGI(TAG, "  Modbus: %s", modbus_ret == ESP_OK ? "Active" : "Disabled");
    ESP_LOGI(TAG, "  HSM:    Running");
    ESP_LOGI(TAG, "-------------------------------------------");
    ESP_LOGI(TAG, "  Free heap:  %lu bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "  Free PSRAM: %lu bytes", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    ESP_LOGI(TAG, "===========================================");
}
