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
 * @brief Cấu hình microstep cho TB6600
 */
typedef enum {
    MICROSTEP_1 = 1,
    MICROSTEP_2A = 2,
    MICROSTEP_2B = 2,
    MICROSTEP_4 = 4,
    MICROSTEP_8 = 8,
    MICROSTEP_16 = 16,
    MICROSTEP_32 = 32
} stepper_microstep_t;

/**
 * @brief Chiều quay
 */
typedef enum {
    STEPPER_DIR_CW = 0,   // Clockwise
    STEPPER_DIR_CCW = 1   // Counter-clockwise
} stepper_direction_t;

/**
 * @brief Handle cho stepper motor (forward declaration)
 */
typedef struct stepper_motor_t* stepper_handle_t;

/**
 * @brief Callback khi motor hoàn thành di chuyển
 * @param handle Handle motor
 * @param user_data Dữ liệu người dùng
 */
typedef void (*stepper_complete_cb_t)(stepper_handle_t handle, void *user_data);

/**
 * @brief Cấu hình stepper motor
 */
typedef struct {
    gpio_num_t pulse_pin;           // Chân PUL (bắt buộc)
    gpio_num_t dir_pin;             // Chân DIR (bắt buộc)
    gpio_num_t enable_pin;          // Chân EN (-1 nếu không dùng)
    uint16_t steps_per_revolution;  // Bước/vòng của motor (thường 200)
    stepper_microstep_t microstep;  // Chế độ microstep
    uint32_t max_speed_rpm;         // Tốc độ tối đa (RPM)
    uint32_t accel_steps;           // Số bước để tăng/giảm tốc (0 = không gia tốc)
    stepper_complete_cb_t complete_cb;  // Callback khi hoàn thành (NULL nếu không dùng)
    void *user_data;                // Dữ liệu người dùng cho callback
} stepper_config_t;

/**
 * @brief Khởi tạo một động cơ stepper
 */
esp_err_t stepper_init(const stepper_config_t *config, stepper_handle_t *handle);

/**
 * @brief Xóa motor
 */
esp_err_t stepper_deinit(stepper_handle_t handle);

/**
 * @brief Bật/tắt motor (EN pin)
 */
esp_err_t stepper_enable(stepper_handle_t handle, bool enable);

/**
 * @brief Đặt hướng quay
 */
esp_err_t stepper_set_direction(stepper_handle_t handle, stepper_direction_t dir);

/**
 * @brief Quay một số bước (blocking)
 */
esp_err_t stepper_move_steps(stepper_handle_t handle, int32_t steps, uint32_t speed_rpm);

/**
 * @brief Quay một số bước (non-blocking)
 * Return ngay, motor quay trong task riêng. Callback được gọi khi hoàn thành.
 */
esp_err_t stepper_move_steps_async(stepper_handle_t handle, int32_t steps, uint32_t speed_rpm);

/**
 * @brief Quay một góc (blocking)
 */
esp_err_t stepper_move_degrees(stepper_handle_t handle, float degrees, uint32_t speed_rpm);

/**
 * @brief Quay một góc (non-blocking)
 */
esp_err_t stepper_move_degrees_async(stepper_handle_t handle, float degrees, uint32_t speed_rpm);

/**
 * @brief Chờ motor hoàn thành (dùng sau async)
 */
esp_err_t stepper_wait_complete(stepper_handle_t handle, uint32_t timeout_ms);

/**
 * @brief Quay liên tục với tốc độ cho trước
 */
esp_err_t stepper_run_continuous(stepper_handle_t handle, int32_t speed_rpm);

/**
 * @brief Dừng quay liên tục
 */
esp_err_t stepper_stop(stepper_handle_t handle);

/**
 * @brief Kiểm tra motor có đang chạy không
 */
bool stepper_is_running(stepper_handle_t handle);

/**
 * @brief Lấy vị trí hiện tại (số bước)
 */
int32_t stepper_get_position(stepper_handle_t handle);

/**
 * @brief Reset vị trí về 0
 */
esp_err_t stepper_reset_position(stepper_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif // STEPPER_TB6600_H

