/**
 * @file limit_switch.c
 * @brief Limit Switch Library Implementation
 */

#include "limit_switch.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>

static const char *TAG = "LIMIT_SWITCH";

/**
 * @brief Internal limit switch structure
 */
struct limit_switch_t {
    gpio_num_t gpio_pin;
    bool active_level;
    uint32_t debounce_ms;
    limit_switch_callback_t callback;
    void *user_data;
    
    esp_timer_handle_t debounce_timer;
    bool last_state;
    volatile bool debouncing;
};

/**
 * @brief Debounce timer callback
 */
static void debounce_timer_callback(void *arg)
{
    limit_switch_handle_t handle = (limit_switch_handle_t)arg;
    
    // Read current GPIO state
    int level = gpio_get_level(handle->gpio_pin);
    bool pressed = (level == handle->active_level);
    
    // Only trigger callback if state changed
    if (pressed != handle->last_state) {
        handle->last_state = pressed;
        
        if (handle->callback) {
            limit_switch_event_t event = pressed ? LIMIT_SWITCH_PRESSED : LIMIT_SWITCH_RELEASED;
            handle->callback(handle, event, handle->user_data);
        }
    }
    
    handle->debouncing = false;
}

/**
 * @brief GPIO ISR handler
 */
static void IRAM_ATTR limit_switch_isr_handler(void *arg)
{
    limit_switch_handle_t handle = (limit_switch_handle_t)arg;
    
    if (!handle->debouncing) {
        handle->debouncing = true;
        esp_timer_start_once(handle->debounce_timer, handle->debounce_ms * 1000);
    }
}

/**
 * API Implementation
 */

esp_err_t limit_switch_init(const limit_switch_config_t *config, 
                            limit_switch_handle_t *out_handle)
{
    if (!config || !out_handle) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Allocate handle
    limit_switch_handle_t handle = calloc(1, sizeof(struct limit_switch_t));
    if (!handle) {
        ESP_LOGE(TAG, "Out of memory");
        return ESP_ERR_NO_MEM;
    }
    
    // Copy configuration
    handle->gpio_pin = config->gpio_pin;
    handle->active_level = config->active_level;
    handle->debounce_ms = (config->debounce_ms > 0) ? config->debounce_ms : 50;
    handle->callback = config->callback;
    handle->user_data = config->user_data;
    handle->debouncing = false;
    
    // Configure GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << handle->gpio_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = (handle->active_level == 0) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = (handle->active_level == 1) ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    
    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO config failed: %s", esp_err_to_name(err));
        free(handle);
        return err;
    }
    
    // Read initial state
    int level = gpio_get_level(handle->gpio_pin);
    handle->last_state = (level == handle->active_level);
    
    // Create debounce timer
    esp_timer_create_args_t timer_args = {
        .callback = debounce_timer_callback,
        .arg = handle,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "limit_debounce"
    };
    
    err = esp_timer_create(&timer_args, &handle->debounce_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Timer create failed: %s", esp_err_to_name(err));
        free(handle);
        return err;
    }
    
    // Install ISR
    err = gpio_isr_handler_add(handle->gpio_pin, limit_switch_isr_handler, handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ISR add failed: %s", esp_err_to_name(err));
        esp_timer_delete(handle->debounce_timer);
        free(handle);
        return err;
    }
    
    *out_handle = handle;
    
    ESP_LOGI(TAG, "Limit switch initialized: GPIO%d, active=%d, debounce=%lums",
             handle->gpio_pin, handle->active_level, handle->debounce_ms);
    
    return ESP_OK;
}

esp_err_t limit_switch_deinit(limit_switch_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    gpio_isr_handler_remove(handle->gpio_pin);
    
    if (handle->debounce_timer) {
        esp_timer_stop(handle->debounce_timer);
        esp_timer_delete(handle->debounce_timer);
    }
    
    free(handle);
    ESP_LOGI(TAG, "Limit switch deinitialized");
    
    return ESP_OK;
}

bool limit_switch_is_pressed(limit_switch_handle_t handle)
{
    if (!handle) {
        return false;
    }
    
    return handle->last_state;
}