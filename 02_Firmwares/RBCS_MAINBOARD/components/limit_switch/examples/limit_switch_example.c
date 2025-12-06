/**
 * @file limit_switch_example.c
 * @brief Example: Control 3 limit switches with callbacks
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "limit_switch.h"

static const char *TAG = "EXAMPLE";

// ========================================
// GPIO Pin Definitions
// ========================================
#define LIMIT_SWITCH_1_PIN    GPIO_NUM_34
#define LIMIT_SWITCH_2_PIN    GPIO_NUM_35
#define LIMIT_SWITCH_3_PIN    GPIO_NUM_32

// Limit switch handles
static limit_switch_handle_t limit1 = NULL;
static limit_switch_handle_t limit2 = NULL;
static limit_switch_handle_t limit3 = NULL;

// ========================================
// Callback Functions
// ========================================

void limit_switch_1_callback(limit_switch_handle_t handle, 
                             limit_switch_event_t event, 
                             void *user_data)
{
    if (event == LIMIT_SWITCH_PRESSED) {
        ESP_LOGI(TAG, "🔴 Limit Switch 1 PRESSED");
    } else {
        ESP_LOGI(TAG, "🟢 Limit Switch 1 RELEASED");
    }
}

void limit_switch_2_callback(limit_switch_handle_t handle, 
                             limit_switch_event_t event, 
                             void *user_data)
{
    if (event == LIMIT_SWITCH_PRESSED) {
        ESP_LOGI(TAG, "🔴 Limit Switch 2 PRESSED");
    } else {
        ESP_LOGI(TAG, "🟢 Limit Switch 2 RELEASED");
    }
}

void limit_switch_3_callback(limit_switch_handle_t handle, 
                             limit_switch_event_t event, 
                             void *user_data)
{
    if (event == LIMIT_SWITCH_PRESSED) {
        ESP_LOGI(TAG, "🔴 Limit Switch 3 PRESSED");
        // Example: Stop motor when limit switch triggered
        // motor_stop();
    } else {
        ESP_LOGI(TAG, "🟢 Limit Switch 3 RELEASED");
    }
}

// ========================================
// Main Application
// ========================================

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " Limit Switch Example");
    ESP_LOGI(TAG, " 3 Switches with Callbacks");
    ESP_LOGI(TAG, "========================================\n");
    
    // Install GPIO ISR service
    gpio_install_isr_service(0);
    
    // ========================================
    // Initialize Limit Switch 1
    // ========================================
    limit_switch_config_t config1 = {
        .gpio_pin = LIMIT_SWITCH_1_PIN,
        .active_level = 0,              // Active LOW (pressed = 0)
        .debounce_ms = 50,
        .callback = limit_switch_1_callback,
        .user_data = NULL
    };
    
    esp_err_t ret = limit_switch_init(&config1, &limit1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init limit switch 1");
        return;
    }
    
    // ========================================
    // Initialize Limit Switch 2
    // ========================================
    limit_switch_config_t config2 = {
        .gpio_pin = LIMIT_SWITCH_2_PIN,
        .active_level = 0,
        .debounce_ms = 50,
        .callback = limit_switch_2_callback,
        .user_data = NULL
    };
    
    ret = limit_switch_init(&config2, &limit2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init limit switch 2");
        return;
    }
    
    // ========================================
    // Initialize Limit Switch 3
    // ========================================
    limit_switch_config_t config3 = {
        .gpio_pin = LIMIT_SWITCH_3_PIN,
        .active_level = 0,
        .debounce_ms = 50,
        .callback = limit_switch_3_callback,
        .user_data = NULL
    };
    
    ret = limit_switch_init(&config3, &limit3);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init limit switch 3");
        return;
    }
    
    ESP_LOGI(TAG, "✅ All limit switches initialized!\n");
    
    // ========================================
    // Main Loop - Monitor status
    // ========================================
    while (1) {
        ESP_LOGI(TAG, "Status: SW1=%s, SW2=%s, SW3=%s",
                 limit_switch_is_pressed(limit1) ? "PRESSED" : "released",
                 limit_switch_is_pressed(limit2) ? "PRESSED" : "released",
                 limit_switch_is_pressed(limit3) ? "PRESSED" : "released");
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}