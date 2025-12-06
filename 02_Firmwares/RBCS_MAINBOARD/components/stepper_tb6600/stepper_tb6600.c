/**
 * @file stepper_tb6600.c
 * @brief TB6600 Stepper Motor Driver - PWM with Interrupt-based step counting
 */

#include "stepper_tb6600.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "STEPPER_TB6600";

/**
 * @brief PWM configuration - 10-bit for best frequency range
 */
#define STEPPER_PWM_TIMER       LEDC_TIMER_0
#define STEPPER_PWM_MODE        LEDC_LOW_SPEED_MODE
#define STEPPER_PWM_DUTY_RES    LEDC_TIMER_10_BIT
#define STEPPER_PWM_DUTY        512  // 50% duty

// Frequency limits
#define MIN_FREQ_HZ            100
#define MAX_FREQ_HZ            40000

/**
 * @brief Motor operating mode
 */
typedef enum {
    STEPPER_MODE_IDLE,
    STEPPER_MODE_POSITION,
    STEPPER_MODE_VELOCITY
} stepper_mode_t;

/**
 * @brief Internal motor structure
 */
struct stepper_motor_t {
    // GPIO pins
    gpio_num_t pulse_pin;
    gpio_num_t dir_pin;
    gpio_num_t enable_pin;
    
    // Motor parameters
    uint16_t steps_per_rev;
    stepper_microstep_t microstep;
    uint16_t max_speed_rpm;
    uint32_t accel_steps;
    
    // Callback
    stepper_complete_cb_t complete_cb;
    void *user_data;
    
    // PWM channel
    ledc_channel_t pwm_channel;
    
    // Current state
    stepper_direction_t current_dir;
    volatile int32_t current_position;
    int32_t target_position;
    stepper_mode_t mode;
    bool is_enabled;
    
    // Acceleration state
    uint32_t current_speed_rpm;
    uint32_t target_speed_rpm;
    volatile uint32_t steps_moved;
    uint32_t total_steps;
    
    // Step accumulator for precise counting
    volatile uint32_t step_accumulator;
    
    // High-resolution timer for acceleration updates
    esp_timer_handle_t accel_timer;
    volatile bool timer_running;  // Atomic flag to prevent use-after-free
    
    // Synchronization
    SemaphoreHandle_t mutex;
    volatile bool stop_requested;
};

// Global channel allocator
static uint8_t g_next_pwm_channel = 0;
static SemaphoreHandle_t g_channel_mutex = NULL;

/**
 * @brief Calculate frequency from RPM
 */
static uint32_t rpm_to_hz(stepper_handle_t handle, uint32_t rpm)
{
    uint32_t hz = ((uint32_t)rpm * handle->steps_per_rev * handle->microstep) / 60;
    if (hz < MIN_FREQ_HZ) hz = MIN_FREQ_HZ;
    if (hz > MAX_FREQ_HZ) hz = MAX_FREQ_HZ;
    return hz;
}

/**
 * @brief Allocate PWM channel
 */
static ledc_channel_t allocate_pwm_channel(void)
{
    if (g_channel_mutex == NULL) {
        g_channel_mutex = xSemaphoreCreateMutex();
    }
    
    xSemaphoreTake(g_channel_mutex, portMAX_DELAY);
    ledc_channel_t channel = (ledc_channel_t)g_next_pwm_channel++;
    xSemaphoreGive(g_channel_mutex);
    
    return (channel >= LEDC_CHANNEL_MAX) ? LEDC_CHANNEL_0 : channel;
}

/**
 * @brief Start PWM at specified frequency
 */
static esp_err_t start_pwm(stepper_handle_t handle, uint32_t freq_hz)
{
    if (freq_hz == 0) return ESP_ERR_INVALID_ARG;
    
    if (freq_hz < MIN_FREQ_HZ) freq_hz = MIN_FREQ_HZ;
    if (freq_hz > MAX_FREQ_HZ) freq_hz = MAX_FREQ_HZ;
    
    esp_err_t err = ledc_set_freq(STEPPER_PWM_MODE, STEPPER_PWM_TIMER, freq_hz);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PWM freq %lu Hz: %s", freq_hz, esp_err_to_name(err));
        return err;
    }
    
    err = ledc_set_duty(STEPPER_PWM_MODE, handle->pwm_channel, STEPPER_PWM_DUTY);
    if (err == ESP_OK) {
        err = ledc_update_duty(STEPPER_PWM_MODE, handle->pwm_channel);
    }
    
    return err;
}

/**
 * @brief Stop PWM
 */
static esp_err_t stop_pwm(stepper_handle_t handle)
{
    esp_err_t err = ledc_set_duty(STEPPER_PWM_MODE, handle->pwm_channel, 0);
    if (err == ESP_OK) {
        err = ledc_update_duty(STEPPER_PWM_MODE, handle->pwm_channel);
    }
    return err;
}

/**
 * @brief High-resolution timer callback for acceleration control
 * Called every 10ms to update speed smoothly
 */
static void IRAM_ATTR accel_timer_callback(void *arg)
{
    stepper_handle_t handle = (stepper_handle_t)arg;
    
    // Safety check - prevent crash if handle is invalid or timer stopped
    if (handle == NULL || !handle->timer_running) {
        return;
    }
    
    // Support both POSITION and VELOCITY modes
    if (handle->mode == STEPPER_MODE_IDLE) {
        return;
    }
    
    // For POSITION mode: check if reached target
    if (handle->mode == STEPPER_MODE_POSITION) {
        int32_t remaining_steps = abs(handle->target_position - handle->current_position);
        if (remaining_steps == 0) {
            handle->mode = STEPPER_MODE_IDLE;
            return;
        }
    }
    
    // ===== CALCULATE STEPS MOVED =====
    uint32_t current_freq_hz = rpm_to_hz(handle, handle->current_speed_rpm);
    
    // Estimate steps moved in last 10ms
    uint32_t steps_in_period = (current_freq_hz * 10) / 1000;
    
    handle->steps_moved += steps_in_period;
    
    // ===== ACCELERATION LOGIC =====
    if (handle->accel_steps == 0) {
        return;  // No acceleration configured
    }
    
    uint32_t new_speed_rpm = handle->target_speed_rpm;
    
    // For VELOCITY mode: only accelerate, no deceleration
    if (handle->mode == STEPPER_MODE_VELOCITY) {
        if (handle->steps_moved < handle->accel_steps) {
            // Accelerating to target speed
            new_speed_rpm = (handle->target_speed_rpm * handle->steps_moved) / handle->accel_steps;
            if (new_speed_rpm < 10) new_speed_rpm = 10;
            
            ESP_LOGD(TAG, "VELOCITY ACCEL: step %lu/%lu, speed %lu RPM", 
                     handle->steps_moved, handle->accel_steps, new_speed_rpm);
        } else {
            // Reached target speed, stop timer
            new_speed_rpm = handle->target_speed_rpm;
            esp_timer_stop(handle->accel_timer);
            
            ESP_LOGI(TAG, "VELOCITY: Reached target %lu RPM", new_speed_rpm);
        }
    }
    // For POSITION mode: full acceleration/deceleration
    else if (handle->mode == STEPPER_MODE_POSITION) {
        if (handle->total_steps <= handle->accel_steps * 2) {
            return;  // Not enough steps for accel/decel
        }
        
        int32_t remaining_steps = abs(handle->target_position - handle->current_position);
        
        // Acceleration phase
        if (handle->steps_moved < handle->accel_steps) {
            new_speed_rpm = (handle->target_speed_rpm * handle->steps_moved) / handle->accel_steps;
            if (new_speed_rpm < 10) new_speed_rpm = 10;
            
            ESP_LOGD(TAG, "POS ACCEL: step %lu/%lu, speed %lu RPM", 
                     handle->steps_moved, handle->accel_steps, new_speed_rpm);
        }
        // Deceleration phase
        else if (remaining_steps <= (int32_t)handle->accel_steps) {
            new_speed_rpm = (handle->target_speed_rpm * remaining_steps) / handle->accel_steps;
            if (new_speed_rpm < 10) new_speed_rpm = 10;
            
            ESP_LOGD(TAG, "POS DECEL: remaining %ld/%lu, speed %lu RPM", 
                     remaining_steps, handle->accel_steps, new_speed_rpm);
        }
    }
    
    // Update speed if changed significantly (> 5 RPM)
    if (abs((int32_t)new_speed_rpm - (int32_t)handle->current_speed_rpm) > 5) {
        handle->current_speed_rpm = new_speed_rpm;
        uint32_t new_freq = rpm_to_hz(handle, new_speed_rpm);
        ledc_set_freq(STEPPER_PWM_MODE, STEPPER_PWM_TIMER, new_freq);
        
        if (handle->mode == STEPPER_MODE_POSITION) {
            int32_t remaining = abs(handle->target_position - handle->current_position);
            ESP_LOGI(TAG, "Speed: %lu RPM (%lu Hz), moved: %lu/%lu, remaining: %ld", 
                     new_speed_rpm, new_freq, handle->steps_moved, handle->total_steps, remaining);
        } else {
            ESP_LOGI(TAG, "Speed: %lu RPM (%lu Hz)", new_speed_rpm, new_freq);
        }
    }
}

/**
 * @brief Monitor task - tracks actual position via PWM counting
 */
static void monitor_task(void *arg)
{
    stepper_handle_t handle = (stepper_handle_t)arg;
    
    // Track position by counting PWM pulses
    TickType_t last_wake = xTaskGetTickCount();
    
    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));  // 100 Hz update rate
        
        if (handle->mode == STEPPER_MODE_POSITION) {
            // Calculate steps moved in last 10ms based on current PWM frequency
            uint32_t freq_hz = rpm_to_hz(handle, handle->current_speed_rpm);
            
            // steps = (frequency × time) / 1000
            // For 10ms: steps = (freq_hz × 10) / 1000
            handle->step_accumulator += freq_hz;
            
            if (handle->step_accumulator >= 100) {  // Every 100Hz accumulation
                uint32_t steps = handle->step_accumulator / 100;
                handle->step_accumulator %= 100;
                
                // Update position
                if (handle->current_dir == STEPPER_DIR_CW) {
                    handle->current_position += steps;
                } else {
                    handle->current_position -= steps;
                }
                
                // Check if reached or passed target
                int32_t diff = handle->target_position - handle->current_position;
                if (handle->current_dir == STEPPER_DIR_CW) {
                    // Moving forward: stop if reached or passed
                    if (diff <= 0) {
                        handle->current_position = handle->target_position;
                        handle->mode = STEPPER_MODE_IDLE;
                    }
                } else {
                    // Moving backward: stop if reached or passed
                    if (diff >= 0) {
                        handle->current_position = handle->target_position;
                        handle->mode = STEPPER_MODE_IDLE;
                    }
                }
            }
        }
        
        // Check if completed
        if (handle->mode == STEPPER_MODE_IDLE || handle->stop_requested) {
            // ✅ CRITICAL: Set flag FIRST to prevent timer callback
            handle->timer_running = false;
            
            // Stop acceleration timer
            if (handle->accel_timer) {
                esp_timer_stop(handle->accel_timer);
            }
            
            // Stop PWM
            stop_pwm(handle);
            
            // Wait to ensure timer callback has exited
            vTaskDelay(pdMS_TO_TICKS(100));
            
            ESP_LOGI(TAG, "Movement complete!");
            ESP_LOGI(TAG, "  Final position: %ld", handle->current_position);
            ESP_LOGI(TAG, "  Steps moved: %lu", handle->steps_moved);
            
            // Call completion callback safely
            stepper_complete_cb_t cb = handle->complete_cb;
            void *user_data = handle->user_data;
            
            if (cb != NULL) {
                cb(handle, user_data);
            }
            
            break;
        }
    }
    
    vTaskDelete(NULL);
}

esp_err_t stepper_init(const stepper_config_t *config, stepper_handle_t *out_handle)
{
    if (!config || !out_handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    stepper_handle_t handle = (stepper_handle_t)calloc(1, sizeof(struct stepper_motor_t));
    if (!handle) {
        ESP_LOGE(TAG, "Failed to allocate memory");
        return ESP_ERR_NO_MEM;
    }
    
    // Copy configuration
    handle->pulse_pin = config->pulse_pin;
    handle->dir_pin = config->dir_pin;
    handle->enable_pin = config->enable_pin;
    handle->steps_per_rev = config->steps_per_revolution;
    handle->microstep = config->microstep;
    handle->max_speed_rpm = config->max_speed_rpm;
    handle->accel_steps = config->accel_steps;
    handle->complete_cb = config->complete_cb;
    handle->user_data = config->user_data;
    
    // Initialize state
    handle->current_dir = STEPPER_DIR_CW;
    handle->current_position = 0;
    handle->target_position = 0;
    handle->mode = STEPPER_MODE_IDLE;
    handle->is_enabled = false;
    handle->stop_requested = false;
    handle->step_accumulator = 0;
    handle->timer_running = false;
    
    // Create mutex
    handle->mutex = xSemaphoreCreateMutex();
    if (!handle->mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        free(handle);
        return ESP_ERR_NO_MEM;
    }
    
    // Allocate PWM channel
    handle->pwm_channel = allocate_pwm_channel();
    
    // Configure PWM timer
    ledc_timer_config_t timer_conf = {
        .speed_mode = STEPPER_PWM_MODE,
        .duty_resolution = STEPPER_PWM_DUTY_RES,
        .timer_num = STEPPER_PWM_TIMER,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    
    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "PWM timer config failed: %s", esp_err_to_name(err));
        vSemaphoreDelete(handle->mutex);
        free(handle);
        return err;
    }
    
    // Configure PWM channel
    ledc_channel_config_t channel_conf = {
        .gpio_num = handle->pulse_pin,
        .speed_mode = STEPPER_PWM_MODE,
        .channel = handle->pwm_channel,
        .timer_sel = STEPPER_PWM_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    
    err = ledc_channel_config(&channel_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PWM channel config failed: %s", esp_err_to_name(err));
        vSemaphoreDelete(handle->mutex);
        free(handle);
        return err;
    }
    
    // Configure direction pin
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << handle->dir_pin),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(handle->dir_pin, 0);
    
    // Configure enable pin
    if (handle->enable_pin != -1) {
        io_conf.pin_bit_mask = (1ULL << handle->enable_pin);
        gpio_config(&io_conf);
        gpio_set_level(handle->enable_pin, 1);  // Disabled
    }
    
    // Create high-resolution timer for step counting (1 kHz)
    const esp_timer_create_args_t timer_args = {
        .callback = accel_timer_callback,
        .arg = handle,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "stepper_accel"
    };
    
    err = esp_timer_create(&timer_args, &handle->accel_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create accel timer: %s", esp_err_to_name(err));
        vSemaphoreDelete(handle->mutex);
        free(handle);
        return err;
    }
    
    uint32_t max_freq_hz = rpm_to_hz(handle, handle->max_speed_rpm);
    
    ESP_LOGI(TAG, "Motor initialized:");
    ESP_LOGI(TAG, "  Steps/rev: %d, Microstep: %d", handle->steps_per_rev, handle->microstep);
    ESP_LOGI(TAG, "  Max speed: %d RPM (%lu Hz)", handle->max_speed_rpm, max_freq_hz);
    ESP_LOGI(TAG, "  Accel steps: %lu", handle->accel_steps);
    
    *out_handle = handle;
    return ESP_OK;
}

esp_err_t stepper_deinit(stepper_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    
    stepper_stop(handle);
    
    if (handle->accel_timer) {
        esp_timer_stop(handle->accel_timer);
        esp_timer_delete(handle->accel_timer);
    }
    
    if (handle->mutex) {
        vSemaphoreDelete(handle->mutex);
    }
    
    free(handle);
    return ESP_OK;
}

esp_err_t stepper_enable(stepper_handle_t handle, bool enable)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    
    if (handle->enable_pin != -1) {
        gpio_set_level(handle->enable_pin, enable ? 0 : 1);
    }
    
    handle->is_enabled = enable;
    ESP_LOGI(TAG, "Motor %s", enable ? "ENABLED" : "DISABLED");
    
    return ESP_OK;
}

esp_err_t stepper_set_direction(stepper_handle_t handle, stepper_direction_t dir)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    
    xSemaphoreTake(handle->mutex, portMAX_DELAY);
    handle->current_dir = dir;
    gpio_set_level(handle->dir_pin, dir);
    xSemaphoreGive(handle->mutex);
    
    return ESP_OK;
}

esp_err_t stepper_move_steps(stepper_handle_t handle, int32_t steps, uint32_t speed_rpm)
{
    if (!handle || steps == 0) return ESP_ERR_INVALID_ARG;
    
    if (speed_rpm > handle->max_speed_rpm) {
        speed_rpm = handle->max_speed_rpm;
    }
    
    // Set direction
    stepper_direction_t dir = (steps > 0) ? STEPPER_DIR_CW : STEPPER_DIR_CCW;
    stepper_set_direction(handle, dir);
    
    // Setup movement parameters
    xSemaphoreTake(handle->mutex, portMAX_DELAY);
    handle->target_position = handle->current_position + steps;
    handle->target_speed_rpm = speed_rpm;
    handle->steps_moved = 0;
    handle->step_accumulator = 0;  // Reset accumulator
    handle->total_steps = abs(steps);
    handle->mode = STEPPER_MODE_POSITION;
    handle->stop_requested = false;
    
    // Initial speed
    if (handle->accel_steps > 0 && handle->total_steps > handle->accel_steps * 2) {
        handle->current_speed_rpm = 10;  // Start slow
    } else {
        handle->current_speed_rpm = speed_rpm;
    }
    xSemaphoreGive(handle->mutex);
    
    uint32_t freq_hz = rpm_to_hz(handle, handle->current_speed_rpm);
    
    ESP_LOGI(TAG, "Move %ld steps at %lu RPM (accel: %lu steps)", 
             steps, speed_rpm, handle->accel_steps);
    ESP_LOGI(TAG, "  Initial speed: %lu RPM (%lu Hz)", handle->current_speed_rpm, freq_hz);
    
    // Start PWM
    esp_err_t err = start_pwm(handle, freq_hz);
    if (err != ESP_OK) {
        return err;
    }
    
    // Start acceleration timer (100 Hz = 10ms period)
    handle->timer_running = true;  // ✅ Set flag before starting
    esp_timer_start_periodic(handle->accel_timer, 10000);  // 10,000 μs = 10 ms
    
    // Create monitor task
    xTaskCreate(monitor_task, "stepper_mon", 2048, handle, 5, NULL);
    
    return ESP_OK;
}

esp_err_t stepper_run_continuous(stepper_handle_t handle, int32_t speed_rpm)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    
    if (speed_rpm == 0) {
        return stepper_stop(handle);
    }
    
    uint32_t abs_speed = (speed_rpm < 0) ? -speed_rpm : speed_rpm;
    if (abs_speed > handle->max_speed_rpm) {
        abs_speed = handle->max_speed_rpm;
    }
    
    stepper_direction_t dir = (speed_rpm > 0) ? STEPPER_DIR_CW : STEPPER_DIR_CCW;
    stepper_set_direction(handle, dir);
    
    xSemaphoreTake(handle->mutex, portMAX_DELAY);
    handle->mode = STEPPER_MODE_VELOCITY;
    handle->stop_requested = false;
    handle->target_speed_rpm = abs_speed;
    handle->steps_moved = 0;
    handle->step_accumulator = 0;
    handle->total_steps = handle->accel_steps * 2;  // Fake total for accel calculation
    
    // Apply acceleration if configured
    if (handle->accel_steps > 0) {
        handle->current_speed_rpm = 10;  // Start from 10 RPM
        ESP_LOGI(TAG, "Continuous mode: target %ld RPM with %lu steps acceleration", 
                 speed_rpm, handle->accel_steps);
    } else {
        handle->current_speed_rpm = abs_speed;  // Jump to full speed
        ESP_LOGI(TAG, "Continuous mode: %ld RPM - NO ACCELERATION", speed_rpm);
    }
    xSemaphoreGive(handle->mutex);
    
    uint32_t freq_hz = rpm_to_hz(handle, handle->current_speed_rpm);
    
    // Start PWM
    esp_err_t err = start_pwm(handle, freq_hz);
    if (err != ESP_OK) {
        return err;
    }
    
    // Start acceleration timer if configured
    if (handle->accel_steps > 0) {
        handle->timer_running = true;  // ✅ Set flag before starting
        esp_timer_start_periodic(handle->accel_timer, 10000);  // 10ms
    }
    
    return ESP_OK;
}

esp_err_t stepper_stop(stepper_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    
    xSemaphoreTake(handle->mutex, portMAX_DELAY);
    handle->stop_requested = true;
    handle->mode = STEPPER_MODE_IDLE;
    handle->timer_running = false;  // ✅ Clear flag
    xSemaphoreGive(handle->mutex);
    
    // Stop timers
    if (handle->accel_timer) {
        esp_timer_stop(handle->accel_timer);
    }
    
    // Wait for timer to fully stop
    vTaskDelay(pdMS_TO_TICKS(50));
    
    esp_err_t err = stop_pwm(handle);
    
    ESP_LOGI(TAG, "Motor stopped at position %ld", handle->current_position);
    
    return err;
}

bool stepper_is_running(stepper_handle_t handle)
{
    if (!handle) return false;
    
    xSemaphoreTake(handle->mutex, portMAX_DELAY);
    bool running = (handle->mode != STEPPER_MODE_IDLE);
    xSemaphoreGive(handle->mutex);
    
    return running;
}

int32_t stepper_get_position(stepper_handle_t handle)
{
    if (!handle) return 0;
    return handle->current_position;  // Atomic read on 32-bit systems
}

esp_err_t stepper_set_position(stepper_handle_t handle, int32_t position)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    
    xSemaphoreTake(handle->mutex, portMAX_DELAY);
    handle->current_position = position;
    xSemaphoreGive(handle->mutex);
    
    ESP_LOGI(TAG, "Position set to %ld", position);
    return ESP_OK;
}

