#include "app_states.h"


// SquareLine Studio UI
#define USE_SQUARELINE 1
#if USE_SQUARELINE
#include "ui.h"


static uint8_t pic_num = 1;

static void screen_img_num4event_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    
    if (code == LV_EVENT_CLICKED) {
        pic_num++;
        if (pic_num == 6) pic_num = 1;
        
        switch (pic_num) {
            case 1:
                lv_img_set_src(ui_Image4, &ui_img_scrmain_batteryempty_png);
                break;
            case 2:
                lv_img_set_src(ui_Image4, &ui_img_scrsettingicon_png);
                break;
            case 3:
                lv_img_set_src(ui_Image4, &ui_img_scrsplash_background_png);
                break;
            default:
                break;
        }
    }
}
#endif

static const char *TAG = "RBCS_HMI";


#if CONFIG_LCD_DOUBLE_FB
#define LCD_NUM_FB             2
#else
#define LCD_NUM_FB             1
#endif

// LVGL Task Configuration
#define LCD_LVGL_TICK_PERIOD_MS    2
#define LCD_LVGL_TASK_MAX_DELAY_MS 500
#define LCD_LVGL_TASK_MIN_DELAY_MS 1
#define LCD_LVGL_TASK_STACK_SIZE   (4 * 1024)
#define LCD_LVGL_TASK_PRIORITY     2

// LCD Resolution
#define LCD_H_RES              800
#define LCD_V_RES              480
// ============================================
// Global Variables
// ============================================

static SemaphoreHandle_t lvgl_mux = NULL;

#if CONFIG_LCD_AVOID_TEAR_EFFECT_WITH_SEM
static SemaphoreHandle_t sem_vsync_end;
static SemaphoreHandle_t sem_gui_ready;
#endif

extern void lcd_lvgl_demo_ui(lv_disp_t *disp);

// ============================================
// LCD & LVGL Callbacks
// ============================================

static bool LCD_on_vsync_event(esp_lcd_panel_handle_t panel, 
                                   const esp_lcd_rgb_panel_event_data_t *event_data, 
                                   void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
#if CONFIG_LCD_AVOID_TEAR_EFFECT_WITH_SEM
    if (xSemaphoreTakeFromISR(sem_gui_ready, &high_task_awoken) == pdTRUE) {
        xSemaphoreGiveFromISR(sem_vsync_end, &high_task_awoken);
    }
#endif
    return high_task_awoken == pdTRUE;
}

static void lcd_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    
#if CONFIG_LCD_AVOID_TEAR_EFFECT_WITH_SEM
    xSemaphoreGive(sem_gui_ready);
    xSemaphoreTake(sem_vsync_end, portMAX_DELAY);
#endif
    
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
    lv_disp_flush_ready(drv);
}

static void LCD_increase_lvgl_tick(void *arg)
{
    lv_tick_inc(LCD_LVGL_TICK_PERIOD_MS);
}

// ============================================
// LVGL Lock/Unlock
// ============================================

bool lcd_lvgl_lock(int timeout_ms)
{
    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(lvgl_mux, timeout_ticks) == pdTRUE;
}

void lcd_lvgl_unlock(void)
{
    xSemaphoreGiveRecursive(lvgl_mux);
}

static void lcd_lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t task_delay_ms = LCD_LVGL_TASK_MAX_DELAY_MS;
    
    while (1) {
        if (lcd_lvgl_lock(-1)) {
            task_delay_ms = lv_timer_handler();
            lcd_lvgl_unlock();
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
// Main Application
// ============================================

void app_main(void)
{
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "  RBCS HMI - Battery Charging Station");
    ESP_LOGI(TAG, "===========================================");

    // Initialize I2C for touch controller
    ESP_LOGI(TAG, "Initializing I2C...");
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized");

    // Initialize LVGL
    ESP_LOGI(TAG, "Initializing LVGL...");
    lv_init();

    // Allocate draw buffers from PSRAM
    static lv_disp_draw_buf_t disp_buf;
    void *buf1 = NULL;
    void *buf2 = NULL;
    
    ESP_LOGI(TAG, "Allocating LVGL draw buffers...");
    size_t buffer_size = LCD_H_RES * LCD_V_RES * sizeof(lv_color_t);
    
#if CONFIG_LCD_DOUBLE_FB
    buf1 = heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
    buf2 = heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
    ESP_LOGI(TAG, "Double buffering enabled, allocated %d bytes x2", buffer_size);
#else
    buf1 = heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
    ESP_LOGI(TAG, "Single buffer allocated: %d bytes", buffer_size);
#endif
    
    assert(buf1);
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, LCD_H_RES * LCD_V_RES);

    // Initialize RGB LCD panel
    ESP_LOGI(TAG, "Installing RGB LCD panel driver...");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_rgb_panel_config_t panel_config = {
        .data_width = 16,
        .psram_trans_align = 64,
        .num_fbs = LCD_NUM_FB,
#if CONFIG_LCD_USE_BOUNCE_BUFFER
        .bounce_buffer_size_px = 10 * LCD_H_RES,
#endif
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
#if CONFIG_LCD_DOUBLE_FB
        .flags.double_fb = true,
#endif
    };
    
    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));

    // Create semaphores BEFORE registering VSYNC callback
#if CONFIG_LCD_AVOID_TEAR_EFFECT_WITH_SEM
    ESP_LOGI(TAG, "Creating semaphores for tear effect avoidance...");
    sem_vsync_end = xSemaphoreCreateBinary();
    assert(sem_vsync_end);
    sem_gui_ready = xSemaphoreCreateBinary();
    assert(sem_gui_ready);
#endif

    // Register VSYNC callback
    ESP_LOGI(TAG, "Registering event callbacks...");
    esp_lcd_rgb_panel_event_callbacks_t cbs = {
        .on_vsync = LCD_on_vsync_event,
    };
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(panel_handle, &cbs, NULL));

    // Initialize the LCD panel
    ESP_LOGI(TAG, "Initializing RGB LCD panel...");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    // Initialize LVGL display driver
    ESP_LOGI(TAG, "Registering LVGL display driver...");
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_H_RES;
    disp_drv.ver_res = LCD_V_RES;
    disp_drv.flush_cb = lcd_lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
#if CONFIG_LCD_DOUBLE_FB
    disp_drv.full_refresh = true;
#endif
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    // Initialize touch controller (GT911)
    ESP_LOGI(TAG, "Initializing GT911 touch controller...");
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    // Note: scl_speed_hz already set in I2C master init (400kHz)
    
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_MASTER_NUM, 
                                             &tp_io_config, &tp_io_handle));

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

    esp_lcd_touch_handle_t tp = NULL;
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, &tp));

    // Create LVGL tick timer
    ESP_LOGI(TAG, "Creating LVGL tick timer...");
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &LCD_increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LCD_LVGL_TICK_PERIOD_MS * 1000));

    // Create mutex for LVGL
    lvgl_mux = xSemaphoreCreateRecursiveMutex();
    assert(lvgl_mux);

    // Create LVGL task
    ESP_LOGI(TAG, "Creating LVGL task...");
    xTaskCreate(lcd_lvgl_port_task, "LVGL", 
                LCD_LVGL_TASK_STACK_SIZE, NULL, 
                LCD_LVGL_TASK_PRIORITY, NULL);

    // Initialize UI
#if USE_SQUARELINE
    ESP_LOGI(TAG, "Initializing SquareLine UI...");
    if (lcd_lvgl_lock(-1)) {
        ui_init();
        lv_obj_add_event_cb(ui_Image4, screen_img_num4event_handler, LV_EVENT_CLICKED, NULL);
        lcd_lvgl_unlock();
    }
#else
    ESP_LOGI(TAG, "Initializing demo UI...");
    if (lcd_lvgl_lock(-1)) {
        lcd_lvgl_demo_ui(disp);
        lcd_lvgl_unlock();
    }
#endif

    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "  System startup completed!");
    ESP_LOGI(TAG, "  Free heap: %lu bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "  Free PSRAM: %lu bytes", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    ESP_LOGI(TAG, "===========================================");
}

