/**
 * @file example_3_motors.c
 * @brief Example: Control 3 NEMA17 stepper motors with TB6600 drivers
 * 
 * This example demonstrates:
 * - Initializing 3 motors with different configurations
 * - Moving motors independently
 * - Moving multiple motors simultaneously
 * - Using callbacks to detect completion
 * - Continuous rotation mode
 * - Position tracking and homing
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "stepper_tb6600.h"

static const char *TAG = "EXAMPLE";

// ========================================
// GPIO Pin Definitions
// ========================================
#define MOTOR1_PULSE    GPIO_NUM_25
#define MOTOR1_DIR      GPIO_NUM_26
#define MOTOR1_EN       GPIO_NUM_27

#define MOTOR2_PULSE    GPIO_NUM_32
#define MOTOR2_DIR      GPIO_NUM_33
#define MOTOR2_EN       GPIO_NUM_14  // Using enable pin

#define MOTOR3_PULSE    GPIO_NUM_12
#define MOTOR3_DIR      GPIO_NUM_13
#define MOTOR3_EN       -1           // No enable pin

// Motor handles
static stepper_handle_t motor1 = NULL;
static stepper_handle_t motor2 = NULL;
static stepper_handle_t motor3 = NULL;

// Completion flags for synchronization
static volatile bool motor1_done = false;
static volatile bool motor2_done = false;
static volatile bool motor3_done = false;

// ========================================
// Callback Functions
// ========================================

/**
 * @brief Callback for motor 1 completion
 */
void motor1_complete_cb(stepper_handle_t handle, void *user_data)
{
    int32_t pos = stepper_get_position(handle);
    ESP_LOGI(TAG, "✅ Motor 1 COMPLETE! Position: %ld", pos);
    motor1_done = true;
}

/**
 * @brief Callback for motor 2 completion
 */
void motor2_complete_cb(stepper_handle_t handle, void *user_data)
{
    int32_t pos = stepper_get_position(handle);
    ESP_LOGI(TAG, "✅ Motor 2 COMPLETE! Position: %ld", pos);
    motor2_done = true;
}

/**
 * @brief Callback for motor 3 completion
 */
void motor3_complete_cb(stepper_handle_t handle, void *user_data)
{
    int32_t pos = stepper_get_position(handle);
    ESP_LOGI(TAG, "✅ Motor 3 COMPLETE! Position: %ld", pos);
    motor3_done = true;
}

// ========================================
// Test Functions
// ========================================

/**
 * @brief Test 1: Single motor movement
 */
void test_single_motor(void)
{
    ESP_LOGI(TAG, "\n--- TEST 1: Single Motor Movement ---");
    
    stepper_enable(motor1, true);
    
    // Move 800 steps forward at 60 RPM
    ESP_LOGI(TAG, "Moving motor 1: 800 steps @ 60 RPM");
    stepper_move_steps(motor1, 800, 60);
    
    // Wait for completion
    stepper_wait_complete(motor1, 0);
    ESP_LOGI(TAG, "Motor 1 position: %ld", stepper_get_position(motor1));
    
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Move back to origin
    ESP_LOGI(TAG, "Moving back to origin");
    stepper_move_to(motor1, 0, 60);
    stepper_wait_complete(motor1, 0);
    
    stepper_enable(motor1, false);
}

/**
 * @brief Test 2: Move by angle
 */
void test_angle_movement(void)
{
    ESP_LOGI(TAG, "\n--- TEST 2: Angle Movement ---");
    
    stepper_enable(motor2, true);
    
    // Rotate 90 degrees
    ESP_LOGI(TAG, "Rotating 90 degrees");
    stepper_move_degrees(motor2, 90.0f, 30);
    stepper_wait_complete(motor2, 0);
    
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Rotate another 180 degrees
    ESP_LOGI(TAG, "Rotating 180 degrees");
    stepper_move_degrees(motor2, 180.0f, 30);
    stepper_wait_complete(motor2, 0);
    
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Return to start
    ESP_LOGI(TAG, "Returning to start");
    stepper_move_degrees(motor2, -270.0f, 30);
    stepper_wait_complete(motor2, 0);
    
    stepper_enable(motor2, false);
}

/**
 * @brief Test 3: Three motors simultaneously
 */
void test_simultaneous_motors(void)
{
    ESP_LOGI(TAG, "\n--- TEST 3: Three Motors Simultaneously ---");
    
    // Reset completion flags
    motor1_done = false;
    motor2_done = false;
    motor3_done = false;
    
    // Enable all motors
    stepper_enable(motor1, true);
    stepper_enable(motor2, true);
    stepper_enable(motor3, true);
    
    // Start all motors (non-blocking)
    ESP_LOGI(TAG, "Starting motor 1: 1600 steps @ 60 RPM");
    stepper_move_steps(motor1, 1600, 60);
    
    ESP_LOGI(TAG, "Starting motor 2: 800 steps @ 30 RPM");
    stepper_move_steps(motor2, 800, 30);
    
    ESP_LOGI(TAG, "Starting motor 3: 1200 steps @ 45 RPM");
    stepper_move_steps(motor3, 1200, 45);
    
    ESP_LOGI(TAG, "✅ All motors running!");
    
    // Monitor progress
    while (!motor1_done || !motor2_done || !motor3_done) {
        ESP_LOGI(TAG, "Status: M1=%s M2=%s M3=%s",
                 stepper_is_running(motor1) ? "RUN" : "STOP",
                 stepper_is_running(motor2) ? "RUN" : "STOP",
                 stepper_is_running(motor3) ? "RUN" : "STOP");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    ESP_LOGI(TAG, "✅ All motors completed!");
    ESP_LOGI(TAG, "Final positions: M1=%ld, M2=%ld, M3=%ld",
             stepper_get_position(motor1),
             stepper_get_position(motor2),
             stepper_get_position(motor3));
    
    // Disable all
    stepper_enable(motor1, false);
    stepper_enable(motor2, false);
    stepper_enable(motor3, false);
}

/**
 * @brief Test 4: Continuous rotation
 */
void test_continuous_rotation(void)
{
    ESP_LOGI(TAG, "\n--- TEST 4: Continuous Rotation ---");
    
    stepper_enable(motor1, true);
    
    // Run continuously at 30 RPM
    ESP_LOGI(TAG, "Running continuous @ 30 RPM");
    stepper_run_continuous(motor1, 30);
    
    // Let it run for 3 seconds
    for (int i = 1; i <= 3; i++) {
        ESP_LOGI(TAG, "Running... %d seconds", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    // Stop
    ESP_LOGI(TAG, "Stopping...");
    stepper_stop(motor1);
    
    ESP_LOGI(TAG, "Final position: %ld", stepper_get_position(motor1));
    
    stepper_enable(motor1, false);
}

/**
 * @brief Test 5: Homing simulation
 */
void test_homing(void)
{
    ESP_LOGI(TAG, "\n--- TEST 5: Homing Simulation ---");
    
    stepper_enable(motor1, true);
    
    // Simulate: Move to random position
    ESP_LOGI(TAG, "Moving to random position");
    stepper_move_steps(motor1, 500, 60);
    stepper_wait_complete(motor1, 0);
    
    ESP_LOGI(TAG, "Current position: %ld", stepper_get_position(motor1));
    
    // Simulate homing: slowly move until home switch
    ESP_LOGI(TAG, "Simulating homing...");
    stepper_move_steps(motor1, -1000, 20); // Slow speed for homing
    
    // Simulate home switch detected after 2 seconds
    vTaskDelay(pdMS_TO_TICKS(2000));
    stepper_stop(motor1);
    
    // Set current position as home (0)
    stepper_set_position(motor1, 0);
    ESP_LOGI(TAG, "✅ Home position set. Position: %ld", stepper_get_position(motor1));
    
    // Move to absolute position
    ESP_LOGI(TAG, "Moving to position 400");
    stepper_move_to(motor1, 400, 60);
    stepper_wait_complete(motor1, 0);
    
    ESP_LOGI(TAG, "Current position: %ld", stepper_get_position(motor1));
    
    stepper_enable(motor1, false);
}

/**
 * @brief Test 6: High speed test
 */
void test_high_speed(void)
{
    ESP_LOGI(TAG, "\n--- TEST 6: High Speed Test ---");
    
    stepper_enable(motor1, true);
    
    // Test at different speeds
    uint32_t speeds[] = {60, 120, 180, 240};
    
    for (int i = 0; i < 4; i++) {
        ESP_LOGI(TAG, "Speed test: %lu RPM", speeds[i]);
        stepper_move_steps(motor1, 400, speeds[i]);
        stepper_wait_complete(motor1, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
        
        stepper_move_steps(motor1, -400, speeds[i]);
        stepper_wait_complete(motor1, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    stepper_enable(motor1, false);
}

// ========================================
// Main Application
// ========================================

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " TB6600 Stepper Motor Driver Example");
    ESP_LOGI(TAG, " 3 Motors - PWM Based - Non-blocking");
    ESP_LOGI(TAG, "========================================\n");
    
    // ========================================
    // Initialize Motor 1
    // ========================================
    stepper_config_t config1 = {
        .pulse_pin = MOTOR1_PULSE,
        .dir_pin = MOTOR1_DIR,
        .enable_pin = MOTOR1_EN,
        .steps_per_revolution = 200,
        .microstep = STEPPER_MICROSTEP_16,
        .max_speed_hz = 50000,      // 50 kHz max
        .accel_steps = 0,           // No acceleration
        .complete_cb = motor1_complete_cb,
        .user_data = NULL
    };
    
    esp_err_t ret = stepper_init(&config1, &motor1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init motor 1");
        return;
    }
    
    // ========================================
    // Initialize Motor 2
    // ========================================
    stepper_config_t config2 = {
        .pulse_pin = MOTOR2_PULSE,
        .dir_pin = MOTOR2_DIR,
        .enable_pin = MOTOR2_EN,
        .steps_per_revolution = 200,
        .microstep = STEPPER_MICROSTEP_8,
        .max_speed_hz = 40000,
        .accel_steps = 0,
        .complete_cb = motor2_complete_cb,
        .user_data = NULL
    };
    
    ret = stepper_init(&config2, &motor2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init motor 2");
        return;
    }
    
    // ========================================
    // Initialize Motor 3
    // ========================================
    stepper_config_t config3 = {
        .pulse_pin = MOTOR3_PULSE,
        .dir_pin = MOTOR3_DIR,
        .enable_pin = MOTOR3_EN,
        .steps_per_revolution = 200,
        .microstep = STEPPER_MICROSTEP_4,
        .max_speed_hz = 30000,
        .accel_steps = 0,
        .complete_cb = motor3_complete_cb,
        .user_data = NULL
    };
    
    ret = stepper_init(&config3, &motor3);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init motor 3");
        return;
    }
    
    ESP_LOGI(TAG, "✅ All motors initialized!\n");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // ========================================
    // Run Tests
    // ========================================
    
    test_single_motor();
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    test_angle_movement();
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    test_simultaneous_motors();
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    test_continuous_rotation();
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    test_homing();
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    test_high_speed();
    
    // ========================================
    // Cleanup
    // ========================================
    ESP_LOGI(TAG, "\n========================================");
    ESP_LOGI(TAG, " ✅ All tests completed!");
    ESP_LOGI(TAG, "========================================");
    
    stepper_deinit(motor1);
    stepper_deinit(motor2);
    stepper_deinit(motor3);
}

