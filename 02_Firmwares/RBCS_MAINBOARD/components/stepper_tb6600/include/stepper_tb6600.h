/**
 * @file stepper_tb6600.h
 * @brief TB6600 Stepper Motor Driver Library for ESP-IDF
 * 
 * This library provides high-performance control of stepper motors using
 * the TB6600 driver with PWM-based pulse generation for high speeds.
 * 
 * Features:
 * - PWM-based pulse generation (supports high speed)
 * - Non-blocking operation (async only)
 * - Multiple motor support
 * - Position tracking
 * - Completion callbacks
 * - Direction control
 * - Enable/disable control
 * 
 * @author Your Name
 * @date 2025
 * @license MIT
 */

#ifndef STEPPER_TB6600_H
#define STEPPER_TB6600_H

#include "esp_err.h"
#include "driver/gpio.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Microstep configuration for TB6600 driver
 * 
 * Set these values using DIP switches SW1-SW3 on TB6600:
 * - MICROSTEP_1:  SW1=ON,  SW2=ON,  SW3=OFF (200 steps/rev for 200-step motor)
 * - MICROSTEP_2:  SW1=ON,  SW2=OFF, SW3=ON  (400 steps/rev)
 * - MICROSTEP_4:  SW1=ON,  SW2=OFF, SW3=OFF (800 steps/rev)
 * - MICROSTEP_8:  SW1=OFF, SW2=ON,  SW3=OFF (1600 steps/rev)
 * - MICROSTEP_16: SW1=OFF, SW2=OFF, SW3=ON  (3200 steps/rev)
 * - MICROSTEP_32: SW1=OFF, SW2=OFF, SW3=OFF (6400 steps/rev)
 */
typedef enum {
    STEPPER_MICROSTEP_1 = 1,    /**< Full step mode */
    STEPPER_MICROSTEP_2 = 2,    /**< Half step mode */
    STEPPER_MICROSTEP_4 = 4,    /**< Quarter step mode */
    STEPPER_MICROSTEP_8 = 8,    /**< 1/8 step mode */
    STEPPER_MICROSTEP_16 = 16,  /**< 1/16 step mode */
    STEPPER_MICROSTEP_32 = 32   /**< 1/32 step mode */
} stepper_microstep_t;

/**
 * @brief Motor rotation direction
 */
typedef enum {
    STEPPER_DIR_CW = 0,   /**< Clockwise rotation */
    STEPPER_DIR_CCW = 1   /**< Counter-clockwise rotation */
} stepper_direction_t;

/**
 * @brief Motor operation mode
 */
typedef enum {
    STEPPER_MODE_IDLE = 0,      /**< Motor is idle */
    STEPPER_MODE_POSITION,      /**< Position control mode (move to target) */
    STEPPER_MODE_VELOCITY       /**< Velocity control mode (continuous run) */
} stepper_mode_t;

/**
 * @brief Stepper motor handle (opaque pointer)
 */
typedef struct stepper_motor_t* stepper_handle_t;

/**
 * @brief Callback function type called when motor completes movement
 * 
 * @param handle Motor handle
 * @param user_data User-defined data passed during initialization
 */
typedef void (*stepper_complete_cb_t)(stepper_handle_t handle, void *user_data);

/**
 * @brief Stepper motor configuration structure
 */
typedef struct {
    gpio_num_t pulse_pin;           /**< GPIO pin for PULSE signal (required) */
    gpio_num_t dir_pin;             /**< GPIO pin for DIRECTION signal (required) */
    gpio_num_t enable_pin;          /**< GPIO pin for ENABLE signal (-1 to disable) */
    
    uint16_t steps_per_revolution;  /**< Motor steps per revolution (typically 200) */
    stepper_microstep_t microstep;  /**< Microstepping mode (must match DIP switches) */
    
    uint32_t max_speed_hz;          /**< Maximum pulse frequency in Hz (typical: 20000-50000) */
    uint32_t accel_steps;           /**< Acceleration/deceleration steps (0 = no accel) */
    
    stepper_complete_cb_t complete_cb;  /**< Callback on movement completion (NULL = none) */
    void *user_data;                    /**< User data passed to callback */
} stepper_config_t;

/**
 * @brief Initialize stepper motor driver
 * 
 * This function allocates resources, configures GPIO pins and PWM channels.
 * 
 * @param config Pointer to motor configuration structure
 * @param[out] handle Pointer to receive motor handle
 * @return 
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid parameters
 *     - ESP_ERR_NO_MEM: Out of memory
 *     - ESP_FAIL: PWM or GPIO configuration failed
 */
esp_err_t stepper_init(const stepper_config_t *config, stepper_handle_t *handle);

/**
 * @brief Deinitialize and free motor resources
 * 
 * Stops motor if running, releases PWM channels and frees memory.
 * 
 * @param handle Motor handle
 * @return ESP_OK on success
 */
esp_err_t stepper_deinit(stepper_handle_t handle);

/**
 * @brief Enable or disable motor driver
 * 
 * When disabled (enable=false), motor enters free-running state.
 * Only works if enable_pin was configured (!= -1).
 * 
 * @param handle Motor handle
 * @param enable true to enable, false to disable
 * @return ESP_OK on success
 */
esp_err_t stepper_enable(stepper_handle_t handle, bool enable);

/**
 * @brief Set motor rotation direction
 * 
 * @param handle Motor handle
 * @param dir Direction (CW or CCW)
 * @return ESP_OK on success
 */
esp_err_t stepper_set_direction(stepper_handle_t handle, stepper_direction_t dir);

/**
 * @brief Move motor by specified number of steps (non-blocking)
 * 
 * Starts motor movement and returns immediately. Use callback or
 * stepper_is_running() to detect completion.
 * 
 * @param handle Motor handle
 * @param steps Number of steps to move (negative for reverse direction)
 * @param speed_rpm Target speed in RPM (will be clamped to max_speed_hz)
 * @return 
 *     - ESP_OK: Movement started
 *     - ESP_ERR_INVALID_STATE: Motor already running
 *     - ESP_FAIL: Failed to start PWM
 */
esp_err_t stepper_move_steps(stepper_handle_t handle, int32_t steps, uint32_t speed_rpm);

/**
 * @brief Move motor by specified angle in degrees (non-blocking)
 * 
 * @param handle Motor handle
 * @param degrees Angle in degrees (negative for reverse)
 * @param speed_rpm Target speed in RPM
 * @return Same as stepper_move_steps()
 */
esp_err_t stepper_move_degrees(stepper_handle_t handle, float degrees, uint32_t speed_rpm);

/**
 * @brief Move motor to absolute position (non-blocking)
 * 
 * @param handle Motor handle
 * @param target_position Target position in steps
 * @param speed_rpm Speed in RPM
 * @return Same as stepper_move_steps()
 */
esp_err_t stepper_move_to(stepper_handle_t handle, int32_t target_position, uint32_t speed_rpm);

/**
 * @brief Run motor continuously at constant speed (non-blocking)
 * 
 * Motor will run indefinitely until stepper_stop() is called.
 * 
 * @param handle Motor handle
 * @param speed_rpm Speed in RPM (negative for reverse)
 * @return ESP_OK on success
 */
esp_err_t stepper_run_continuous(stepper_handle_t handle, int32_t speed_rpm);

/**
 * @brief Stop motor immediately
 * 
 * Stops PWM generation and motor movement.
 * 
 * @param handle Motor handle
 * @return ESP_OK on success
 */
esp_err_t stepper_stop(stepper_handle_t handle);

/**
 * @brief Check if motor is currently running
 * 
 * @param handle Motor handle
 * @return true if motor is running, false otherwise
 */
bool stepper_is_running(stepper_handle_t handle);

/**
 * @brief Get current motor position in steps
 * 
 * @param handle Motor handle
 * @return Current position in steps
 */
int32_t stepper_get_position(stepper_handle_t handle);

/**
 * @brief Set current position (without moving motor)
 * 
 * Useful for homing: after reaching home switch, call this with position=0.
 * 
 * @param handle Motor handle
 * @param position New position value
 * @return ESP_OK on success
 */
esp_err_t stepper_set_position(stepper_handle_t handle, int32_t position);

/**
 * @brief Get target position for current movement
 * 
 * @param handle Motor handle
 * @return Target position in steps (equals current position if idle)
 */
int32_t stepper_get_target_position(stepper_handle_t handle);

/**
 * @brief Get current motor operation mode
 * 
 * @param handle Motor handle
 * @return Current mode (IDLE, POSITION, or VELOCITY)
 */
stepper_mode_t stepper_get_mode(stepper_handle_t handle);

/**
 * @brief Wait for motor to complete current movement
 * 
 * Blocks until motor stops or timeout expires.
 * 
 * @param handle Motor handle
 * @param timeout_ms Maximum time to wait in milliseconds (0 = wait forever)
 * @return 
 *     - ESP_OK: Motor stopped
 *     - ESP_ERR_TIMEOUT: Timeout expired
 */
esp_err_t stepper_wait_complete(stepper_handle_t handle, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif // STEPPER_TB6600_H

