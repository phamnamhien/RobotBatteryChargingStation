/**
 * @file stepper_tb6600.c
 * @brief TB6600 Stepper Motor Driver Implementation
 */

#include "stepper_tb6600.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

static const char *TAG = "STEPPER_TB6600";

/**
 * @brief PWM configuration constants
 */
#define STEPPER_PWM_TIMER       LEDC_TIMER_0
#define STEPPER_PWM_MODE        LEDC_LOW_SPEED_MODE
#define STEPPER_PWM_DUTY_RES    LEDC_TIMER_1_BIT  // 50% duty cycle
#define STEPPER_PWM_DUTY        1                 // 50% duty (1 out of 2)

/**
 * @brief Internal motor state structure
 */
struct stepper_motor_t {
    // GPIO pins
    gpio_num_t pulse_pin;
    gpio_num_t dir_pin;
    gpio_num_t enable_pin;
    
    // Motor parameters
    uint16_t steps_per_rev;
    stepper_microstep_t microstep;
    uint32_t max_speed_hz;
    uint32_t accel_steps;
    
    // Callback
    stepper_complete_cb_t complete_cb;
    void *user_data;
    
    // PWM channel
    ledc_channel_t pwm_channel;
    
    // Current state
    stepper_direction_t current_dir;
    int32_t current_position;
    int32_t target_position;
    stepper_mode_t mode;
    bool is_enabled;
    
    // Synchronization
    SemaphoreHandle_t mutex;
    TaskHandle_t monitor_task;
    volatile bool stop_requested;
};

/**
 * @brief Global channel allocator
 */
static uint8_t g_next_pwm_channel = 0;
static SemaphoreHandle_t g_channel_mutex = NULL;

/**
 * @brief Allocate next available PWM channel
 */
static ledc_channel_t allocate_pwm_channel(void)
{
    if (g_channel_mutex == NULL) {
        g_channel_mutex = xSemaphoreCreateMutex();
    }
    
    xSemaphoreTake(g_channel_mutex, portMAX_DELAY);
    ledc_channel_t channel = (ledc_channel_t)g_next_pwm_channel;
    g_next_pwm_channel = (g_next_pwm_channel + 1) % LEDC_CHANNEL_MAX;
    xSemaphoreGive(g_channel_mutex);
    
    return channel;
}

/**
 * @brief Convert RPM to frequency (Hz)
 */
static uint32_t rpm_to_freq(stepper_handle_t handle, uint32_t speed_rpm)
{
    uint32_t steps_per_min = speed_rpm * handle->steps_per_rev * handle->microstep;
    uint32_t freq_hz = steps_per_min / 60;
    
    // Clamp to max speed
    if (freq_hz > handle->max_speed_hz) {
        freq_hz = handle->max_speed_hz;
    }
    
    return freq_hz;
}

/**
 * @brief Start PWM generation at specified frequency
 */
static esp_err_t start_pwm(stepper_handle_t handle, uint32_t freq_hz)
{
    if (freq_hz == 0 || freq_hz > handle->max_speed_hz) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t err;
    
    // Update frequency
    err = ledc_set_freq(STEPPER_PWM_MODE, STEPPER_PWM_TIMER, freq_hz);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PWM freq: %s", esp_err_to_name(err));
        return err;
    }
    
    // Start PWM (50% duty cycle)
    err = ledc_set_duty(STEPPER_PWM_MODE, handle->pwm_channel, STEPPER_PWM_DUTY);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PWM duty: %s", esp_err_to_name(err));
        return err;
    }
    
    err = ledc_update_duty(STEPPER_PWM_MODE, handle->pwm_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update PWM: %s", esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGD(TAG, "PWM started: %lu Hz", freq_hz);
    return ESP_OK;
}

/**
 * @brief Stop PWM generation
 */
static esp_err_t stop_pwm(stepper_handle_t handle)
{
    esp_err_t err = ledc_set_duty(STEPPER_PWM_MODE, handle->pwm_channel, 0);
    if (err == ESP_OK) {
        err = ledc_update_duty(STEPPER_PWM_MODE, handle->pwm_channel);
    }
    
    ESP_LOGD(TAG, "PWM stopped");
    return err;
}

/**
 * @brief Monitor task for position mode
 * 
 * Counts steps and stops motor when target is reached
 */
static void position_monitor_task(void *arg)
{
    stepper_handle_t handle = (stepper_handle_t)arg;
    
    int32_t start_pos = handle->current_position;
    int32_t target = handle->target_position;
    int32_t steps_to_go = abs(target - start_pos);
    
    ESP_LOGI(TAG, "Position monitor started: %ld -> %ld (%ld steps)",
             start_pos, target, steps_to_go);
    
    // Get PWM frequency to calculate time per step
    uint32_t freq_hz = ledc_get_freq(STEPPER_PWM_MODE, STEPPER_PWM_TIMER);
    uint32_t delay_us = (freq_hz > 0) ? (1000000 / freq_hz) : 1000;
    
    int32_t steps_done = 0;
    
    while (steps_done < steps_to_go && !handle->stop_requested) {
        // Update position
        if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(10))) {
            if (handle->current_dir == STEPPER_DIR_CW) {
                handle->current_position++;
            } else {
                handle->current_position--;
            }
            xSemaphoreGive(handle->mutex);
        }
        
        steps_done++;
        
        // Wait for one step duration
        esp_rom_delay_us(delay_us);
    }
    
    // Stop PWM
    stop_pwm(handle);
    
    if (xSemaphoreTake(handle->mutex, portMAX_DELAY)) {
        handle->mode = STEPPER_MODE_IDLE;
        xSemaphoreGive(handle->mutex);
    }
    
    ESP_LOGI(TAG, "Position reached: %ld", handle->current_position);
    
    // Call completion callback
    if (handle->complete_cb) {
        handle->complete_cb(handle, handle->user_data);
    }
    
    handle->monitor_task = NULL;
    vTaskDelete(NULL);
}

/**
 * API Implementation
 */

esp_err_t stepper_init(const stepper_config_t *config, stepper_handle_t *out_handle)
{
    if (!config || !out_handle) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Allocate handle
    stepper_handle_t handle = calloc(1, sizeof(struct stepper_motor_t));
    if (!handle) {
        ESP_LOGE(TAG, "Out of memory");
        return ESP_ERR_NO_MEM;
    }
    
    // Copy configuration
    handle->pulse_pin = config->pulse_pin;
    handle->dir_pin = config->dir_pin;
    handle->enable_pin = config->enable_pin;
    handle->steps_per_rev = config->steps_per_revolution;
    handle->microstep = config->microstep;
    handle->max_speed_hz = config->max_speed_hz;
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
    
    // Create mutex
    handle->mutex = xSemaphoreCreateMutex();
    if (!handle->mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        free(handle);
        return ESP_ERR_NO_MEM;
    }
    
    // Allocate PWM channel
    handle->pwm_channel = allocate_pwm_channel();
    
    // Configure PWM timer (shared by all motors)
    ledc_timer_config_t timer_conf = {
        .speed_mode = STEPPER_PWM_MODE,
        .duty_resolution = STEPPER_PWM_DUTY_RES,
        .timer_num = STEPPER_PWM_TIMER,
        .freq_hz = config->max_speed_hz,
        .clk_cfg = LEDC_AUTO_CLK
    };
    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        // ESP_ERR_INVALID_STATE means timer already configured (OK)
        ESP_LOGE(TAG, "PWM timer config failed: %s", esp_err_to_name(err));
        vSemaphoreDelete(handle->mutex);
        free(handle);
        return err;
    }
    
    // Configure PWM channel for pulse pin
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
    
    // Configure enable pin (if used)
    if (handle->enable_pin != -1) {
        io_conf.pin_bit_mask = (1ULL << handle->enable_pin);
        gpio_config(&io_conf);
        gpio_set_level(handle->enable_pin, 1); // Disabled by default
    } else {
        handle->is_enabled = true;
    }
    
    *out_handle = handle;
    
    ESP_LOGI(TAG, "Motor initialized: CH%d, PULSE=%d, DIR=%d, EN=%d",
             handle->pwm_channel, handle->pulse_pin, handle->dir_pin, handle->enable_pin);
    ESP_LOGI(TAG, "  Steps/rev=%d, Microstep=%d, Max=%lu Hz",
             handle->steps_per_rev, handle->microstep, handle->max_speed_hz);
    
    return ESP_OK;
}

esp_err_t stepper_deinit(stepper_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    
    stepper_stop(handle);
    stepper_enable(handle, false);
    
    if (handle->mutex) {
        vSemaphoreDelete(handle->mutex);
    }
    
    free(handle);
    ESP_LOGI(TAG, "Motor deinitialized");
    
    return ESP_OK;
}

esp_err_t stepper_enable(stepper_handle_t handle, bool enable)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    
    if (handle->enable_pin == -1) {
        return ESP_OK;
    }
    
    gpio_set_level(handle->enable_pin, enable ? 0 : 1);
    handle->is_enabled = enable;
    
    ESP_LOGI(TAG, "Motor %s", enable ? "ENABLED" : "DISABLED");
    return ESP_OK;
}

esp_err_t stepper_set_direction(stepper_handle_t handle, stepper_direction_t dir)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    
    if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(100))) {
        gpio_set_level(handle->dir_pin, dir);
        handle->current_dir = dir;
        xSemaphoreGive(handle->mutex);
        
        esp_rom_delay_us(5); // Direction setup time
        return ESP_OK;
    }
    
    return ESP_ERR_TIMEOUT;
}

esp_err_t stepper_move_steps(stepper_handle_t handle, int32_t steps, uint32_t speed_rpm)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    if (steps == 0) return ESP_OK;
    
    if (handle->mode != STEPPER_MODE_IDLE) {
        ESP_LOGW(TAG, "Motor already running");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Set direction
    stepper_direction_t dir = (steps > 0) ? STEPPER_DIR_CW : STEPPER_DIR_CCW;
    stepper_set_direction(handle, dir);
    
    // Calculate target position
    if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(100))) {
        handle->target_position = handle->current_position + steps;
        handle->mode = STEPPER_MODE_POSITION;
        handle->stop_requested = false;
        xSemaphoreGive(handle->mutex);
    } else {
        return ESP_ERR_TIMEOUT;
    }
    
    // Start PWM
    uint32_t freq_hz = rpm_to_freq(handle, speed_rpm);
    esp_err_t err = start_pwm(handle, freq_hz);
    if (err != ESP_OK) {
        handle->mode = STEPPER_MODE_IDLE;
        return err;
    }
    
    // Create monitor task
    BaseType_t ret = xTaskCreate(
        position_monitor_task,
        "stepper_mon",
        2048,
        handle,
        5,
        &handle->monitor_task
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create monitor task");
        stop_pwm(handle);
        handle->mode = STEPPER_MODE_IDLE;
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Move started: %ld steps @ %lu RPM (%lu Hz)",
             steps, speed_rpm, freq_hz);
    
    return ESP_OK;
}

esp_err_t stepper_move_degrees(stepper_handle_t handle, float degrees, uint32_t speed_rpm)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    
    float steps_per_degree = (handle->steps_per_rev * handle->microstep) / 360.0f;
    int32_t steps = (int32_t)(degrees * steps_per_degree);
    
    return stepper_move_steps(handle, steps, speed_rpm);
}

esp_err_t stepper_move_to(stepper_handle_t handle, int32_t target_position, uint32_t speed_rpm)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    
    int32_t current = stepper_get_position(handle);
    int32_t steps = target_position - current;
    
    return stepper_move_steps(handle, steps, speed_rpm);
}

esp_err_t stepper_run_continuous(stepper_handle_t handle, int32_t speed_rpm)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    
    if (handle->mode != STEPPER_MODE_IDLE) {
        stepper_stop(handle);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Set direction
    stepper_direction_t dir = (speed_rpm > 0) ? STEPPER_DIR_CW : STEPPER_DIR_CCW;
    stepper_set_direction(handle, dir);
    
    // Set mode
    if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(100))) {
        handle->mode = STEPPER_MODE_VELOCITY;
        xSemaphoreGive(handle->mutex);
    }
    
    // Start PWM
    uint32_t freq_hz = rpm_to_freq(handle, abs(speed_rpm));
    esp_err_t err = start_pwm(handle, freq_hz);
    
    ESP_LOGI(TAG, "Continuous mode: %ld RPM (%lu Hz)", speed_rpm, freq_hz);
    
    return err;
}

esp_err_t stepper_stop(stepper_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    
    handle->stop_requested = true;
    
    stop_pwm(handle);
    
    if (handle->monitor_task) {
        uint32_t timeout = 0;
        while (handle->monitor_task && timeout < 1000) {
            vTaskDelay(pdMS_TO_TICKS(10));
            timeout += 10;
        }
    }
    
    if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(100))) {
        handle->mode = STEPPER_MODE_IDLE;
        xSemaphoreGive(handle->mutex);
    }
    
    ESP_LOGI(TAG, "Motor stopped");
    return ESP_OK;
}

bool stepper_is_running(stepper_handle_t handle)
{
    if (!handle) return false;
    return (handle->mode != STEPPER_MODE_IDLE);
}

int32_t stepper_get_position(stepper_handle_t handle)
{
    if (!handle) return 0;
    
    int32_t pos = 0;
    if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(100))) {
        pos = handle->current_position;
        xSemaphoreGive(handle->mutex);
    }
    
    return pos;
}

esp_err_t stepper_set_position(stepper_handle_t handle, int32_t position)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    
    if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(100))) {
        handle->current_position = position;
        xSemaphoreGive(handle->mutex);
        ESP_LOGI(TAG, "Position set to %ld", position);
        return ESP_OK;
    }
    
    return ESP_ERR_TIMEOUT;
}

int32_t stepper_get_target_position(stepper_handle_t handle)
{
    if (!handle) return 0;
    
    int32_t target = 0;
    if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(100))) {
        target = handle->target_position;
        xSemaphoreGive(handle->mutex);
    }
    
    return target;
}

stepper_mode_t stepper_get_mode(stepper_handle_t handle)
{
    if (!handle) return STEPPER_MODE_IDLE;
    return handle->mode;
}

esp_err_t stepper_wait_complete(stepper_handle_t handle, uint32_t timeout_ms)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    
    uint32_t elapsed = 0;
    uint32_t wait_forever = (timeout_ms == 0);
    
    while (stepper_is_running(handle)) {
        vTaskDelay(pdMS_TO_TICKS(10));
        
        if (!wait_forever) {
            elapsed += 10;
            if (elapsed >= timeout_ms) {
                ESP_LOGW(TAG, "Wait timeout");
                return ESP_ERR_TIMEOUT;
            }
        }
    }
    
    return ESP_OK;
}

