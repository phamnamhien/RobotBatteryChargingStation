/**
 * @file limit_switch.h
 * @brief Limit Switch Library with Callback Support
 */

#ifndef LIMIT_SWITCH_H
#define LIMIT_SWITCH_H

#include "esp_err.h"
#include "driver/gpio.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Limit switch handle
 */
typedef struct limit_switch_t* limit_switch_handle_t;

/**
 * @brief Limit switch event type
 */
typedef enum {
    LIMIT_SWITCH_PRESSED = 0,   /**< Switch pressed (triggered) */
    LIMIT_SWITCH_RELEASED = 1   /**< Switch released */
} limit_switch_event_t;

/**
 * @brief Callback function type
 * 
 * @param handle Limit switch handle
 * @param event Event type (pressed/released)
 * @param user_data User data
 */
typedef void (*limit_switch_callback_t)(limit_switch_handle_t handle, 
                                        limit_switch_event_t event, 
                                        void *user_data);

/**
 * @brief Limit switch configuration
 */
typedef struct {
    gpio_num_t gpio_pin;                    /**< GPIO pin number */
    bool active_level;                      /**< Active level (0=LOW, 1=HIGH) */
    uint32_t debounce_ms;                   /**< Debounce time in ms (default: 50) */
    limit_switch_callback_t callback;       /**< Callback function */
    void *user_data;                        /**< User data for callback */
} limit_switch_config_t;

/**
 * @brief Initialize limit switch
 * 
 * @param config Configuration
 * @param[out] handle Output handle
 * @return ESP_OK on success
 */
esp_err_t limit_switch_init(const limit_switch_config_t *config, 
                            limit_switch_handle_t *handle);

/**
 * @brief Deinitialize limit switch
 * 
 * @param handle Limit switch handle
 * @return ESP_OK on success
 */
esp_err_t limit_switch_deinit(limit_switch_handle_t handle);

/**
 * @brief Get current switch state
 * 
 * @param handle Limit switch handle
 * @return true if pressed, false if released
 */
bool limit_switch_is_pressed(limit_switch_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif // LIMIT_SWITCH_H

