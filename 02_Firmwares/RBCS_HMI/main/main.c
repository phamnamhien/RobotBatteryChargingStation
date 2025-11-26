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

// Callback nhận dữ liệu Modbus
void modbus_data_received(uint8_t slave_addr, uint8_t reg_type, 
                         uint16_t reg_addr, uint16_t *data, uint16_t length)
{
    ESP_LOGI(TAG, "Nhận dữ liệu từ slave %d, type 0x%02X, addr %d:", 
             slave_addr, reg_type, reg_addr);
    for (int i = 0; i < length; i++) {
        ESP_LOGI(TAG, "  [%d] = %d", i, data[i]);
    }
}

// Task Modbus polling
void modbus_poll_task(void *arg)
{
    uint16_t holding_regs[10];
    uint16_t input_regs[5];
    
    while (1) {
        // Đọc Holding Registers
        if (modbus_read_holding_registers(1, 0, 10, holding_regs) == ESP_OK) {
            ESP_LOGI(TAG, "Đọc Holding OK");
        }
        
        vTaskDelay(pdMS_TO_TICKS(200));
        
        // Đọc Input Registers
        if (modbus_read_input_registers(1, 0, 5, input_regs) == ESP_OK) {
            ESP_LOGI(TAG, "Đọc Input OK");
        }
        
        vTaskDelay(pdMS_TO_TICKS(200));
        
        // // Ghi register
        // modbus_write_single_register(1, 0, 1234);
        
        // vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Khởi động ứng dụng...");


    
    // Khởi tạo Modbus Manager
    ESP_ERROR_CHECK(modbus_manager_init(&modbus_cfg));
    
    // Đăng ký callback (tùy chọn)
    modbus_manager_register_callback(modbus_data_received);

    // Tạo task Modbus polling
    xTaskCreate(modbus_poll_task, "modbus_poll", 4096, NULL, 5, NULL);

    // Khởi tạo HSM
    app_state_hsm_init(&device);

    // TODO: Khởi tạo UI, WiFi, các module khác...
    ESP_LOGI(TAG, "Các module khác có thể khởi tạo ở đây");
}








// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_timer.h"
// #include "esp_lcd_panel_io.h"
// #include "esp_lcd_panel_vendor.h"
// #include "esp_lcd_panel_ops.h"
// #include "driver/gpio.h"
// #include "driver/spi_master.h"
// #include "esp_err.h"
// #include "esp_log.h"
// #include "lvgl.h"
// #include "esp_lvgl_port.h"
// #include "esp_lcd_ili9341.h" 

// static const char *TAG = "main";


// void app_main(void)
// {
    
// }


// // Cấu hình pins (thay đổi theo phần cứng của bạn)
// #define LCD_HOST       SPI2_HOST
// #define LCD_PIXEL_CLK  20000000
// #define LCD_H_RES      240
// #define LCD_V_RES      320

// #define PIN_NUM_MISO   -1
// #define PIN_NUM_MOSI   23
// #define PIN_NUM_CLK    18
// #define PIN_NUM_CS     5
// #define PIN_NUM_DC     2
// #define PIN_NUM_RST    4
// #define PIN_NUM_BCKL   15

// void app_main(void)
// {
//     ESP_LOGI(TAG, "Initialize SPI bus");
    
//     // SPI bus init
//     spi_bus_config_t buscfg = {
//         .mosi_io_num = PIN_NUM_MOSI,
//         .miso_io_num = PIN_NUM_MISO,
//         .sclk_io_num = PIN_NUM_CLK,
//         .quadwp_io_num = -1,
//         .quadhd_io_num = -1,
//         .max_transfer_sz = LCD_H_RES * LCD_V_RES * sizeof(uint16_t)
//     };
//     ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

//     ESP_LOGI(TAG, "Install panel IO");
//     esp_lcd_panel_io_handle_t io_handle = NULL;
//     esp_lcd_panel_io_spi_config_t io_config = {
//         .dc_gpio_num = PIN_NUM_DC,
//         .cs_gpio_num = PIN_NUM_CS,
//         .pclk_hz = LCD_PIXEL_CLK,
//         .lcd_cmd_bits = 8,
//         .lcd_param_bits = 8,
//         .spi_mode = 0,
//         .trans_queue_depth = 10,
//     };
//     ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

//     ESP_LOGI(TAG, "Install LCD driver");
//     esp_lcd_panel_handle_t panel_handle = NULL;
//     esp_lcd_panel_dev_config_t panel_config = {
//         .reset_gpio_num = PIN_NUM_RST,
//         .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
//         .bits_per_pixel = 16,
//     };
//     ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(io_handle, &panel_config, &panel_handle));
//     ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
//     ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
//     ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));
//     ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

//     // Backlight
//     gpio_config_t bk_gpio_config = {
//         .pin_bit_mask = 1ULL << PIN_NUM_BCKL,
//         .mode = GPIO_MODE_OUTPUT,
//     };
//     ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
//     gpio_set_level(PIN_NUM_BCKL, 1);

//     ESP_LOGI(TAG, "Initialize LVGL");
//     const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
//     ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));

//     const lvgl_port_display_cfg_t disp_cfg = {
//         .io_handle = io_handle,
//         .panel_handle = panel_handle,
//         .buffer_size = LCD_H_RES * 40,
//         .double_buffer = false,
//         .hres = LCD_H_RES,
//         .vres = LCD_V_RES,
//         .monochrome = false,
//         .rotation = {
//             .swap_xy = false,
//             .mirror_x = false,
//             .mirror_y = false,
//         },
//         .flags = {
//             .buff_dma = true,
//         }
//     };
//     lv_disp_t *disp = lvgl_port_add_disp(&disp_cfg);

//     ESP_LOGI(TAG, "Display LVGL UI");
//     // Lock LVGL để tạo UI
//     lvgl_port_lock(0);
    
//     lv_obj_t *scr = lv_disp_get_scr_act(disp);
//     lv_obj_t *label = lv_label_create(scr);
//     lv_label_set_text(label, "Hello LVGL!");
//     lv_obj_center(label);
    
//     lvgl_port_unlock();

//     ESP_LOGI(TAG, "Display initialized");
    
//     // Main loop
//     while (1) {
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
// }