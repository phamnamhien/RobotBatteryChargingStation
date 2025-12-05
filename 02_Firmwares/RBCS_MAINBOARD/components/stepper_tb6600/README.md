# TB6600 Stepper Motor Driver Library for ESP-IDF

High-performance stepper motor control library using PWM for TB6600 driver modules.

## Features

✅ **PWM-Based Pulse Generation**
- Uses ESP32 LEDC (PWM) hardware for accurate, high-speed pulses
- Supports up to **200 kHz** pulse frequency (hardware dependent)
- Typical operation: **20-50 kHz** (very smooth, no missed steps)

✅ **Non-Blocking Operation**
- All movements are asynchronous (non-blocking)
- Use callbacks to detect completion
- Control multiple motors simultaneously

✅ **Position Tracking**
- Real-time position monitoring
- Move to absolute positions
- Homing support

✅ **Multiple Motors**
- Control up to 8 motors independently
- Each motor uses one LEDC channel
- Thread-safe operation

## Maximum Speed

### Frequency Limits
- **Hardware limit**: ~200 kHz (LEDC maximum)
- **Recommended**: 20-50 kHz
- **Example speeds** (200-step motor, 16 microsteps):
  - 20 kHz → 375 RPM
  - 50 kHz → 937 RPM
  - 100 kHz → 1875 RPM

### Why PWM is Better
Previous implementation used `esp_rom_delay_us()`:
- ❌ CPU blocking
- ❌ Limited to ~20-30 kHz due to timing jitter
- ❌ Cannot run multiple motors smoothly

PWM implementation:
- ✅ Hardware-generated pulses (no CPU overhead)
- ✅ Perfect timing accuracy
- ✅ High frequency support (50+ kHz)
- ✅ Run multiple motors simultaneously

## Installation

1. Copy to your project's `components` directory:
```bash
cd your_project/components
cp -r stepper_tb6600_pwm .
```

2. The component will be automatically included in build.

## Hardware Connection

### ESP32 to TB6600

```
ESP32          TB6600
------         ------
GPIO_X   --→   PUL-
GPIO_Y   --→   DIR-
GPIO_Z   --→   EN-   (optional)
5V       --→   PUL+, DIR+, EN+
GND      --→   GND
```

### TB6600 to NEMA17 Motor

```
TB6600         NEMA17
------         ------
A+, A-   --→   Coil A
B+, B-   --→   Coil B
VCC/GND  --→   12-36V Power Supply
```

### TB6600 DIP Switch Settings

**Microstep (SW1-3):**
| Microstep | SW1 | SW2 | SW3 | Steps/Rev (200-step motor) |
|-----------|-----|-----|-----|----------------------------|
| 1         | ON  | ON  | OFF | 200                        |
| 2         | ON  | OFF | ON  | 400                        |
| 4         | ON  | OFF | OFF | 800                        |
| 8         | OFF | ON  | OFF | 1600                       |
| 16        | OFF | OFF | ON  | 3200                       |
| 32        | OFF | OFF | OFF | 6400                       |

**Current (SW4-6):** Match to your motor's rated current

## Quick Start

### Basic Example

```c
#include "stepper_tb6600.h"

stepper_handle_t motor;

// Callback when movement completes
void motor_done(stepper_handle_t handle, void *user_data) {
    ESP_LOGI("MOTOR", "Movement complete!");
}

void app_main(void) {
    // Configure motor
    stepper_config_t config = {
        .pulse_pin = GPIO_NUM_25,
        .dir_pin = GPIO_NUM_26,
        .enable_pin = GPIO_NUM_27,
        .steps_per_revolution = 200,
        .microstep = STEPPER_MICROSTEP_16,
        .max_speed_hz = 50000,  // 50 kHz
        .accel_steps = 0,
        .complete_cb = motor_done,
        .user_data = NULL
    };
    
    // Initialize
    stepper_init(&config, &motor);
    
    // Enable motor
    stepper_enable(motor, true);
    
    // Move 800 steps at 60 RPM
    stepper_move_steps(motor, 800, 60);
    
    // Do other work...
    
    // Wait for completion
    stepper_wait_complete(motor, 0);
    
    // Disable motor
    stepper_enable(motor, false);
}
```

### Multiple Motors

```c
stepper_handle_t motor1, motor2, motor3;

// Initialize all motors...

// Start all motors (non-blocking)
stepper_enable(motor1, true);
stepper_enable(motor2, true);
stepper_enable(motor3, true);

stepper_move_steps(motor1, 1600, 60);
stepper_move_steps(motor2, 800, 30);
stepper_move_steps(motor3, 1200, 45);

// All motors running simultaneously!

// Wait for all to complete
while (stepper_is_running(motor1) || 
       stepper_is_running(motor2) || 
       stepper_is_running(motor3)) {
    vTaskDelay(pdMS_TO_TICKS(100));
}
```

## API Reference

### Initialization

```c
esp_err_t stepper_init(const stepper_config_t *config, stepper_handle_t *handle);
esp_err_t stepper_deinit(stepper_handle_t handle);
esp_err_t stepper_enable(stepper_handle_t handle, bool enable);
```

### Movement (All Non-Blocking)

```c
// Move by steps (negative = reverse)
esp_err_t stepper_move_steps(stepper_handle_t handle, int32_t steps, uint32_t speed_rpm);

// Move by angle
esp_err_t stepper_move_degrees(stepper_handle_t handle, float degrees, uint32_t speed_rpm);

// Move to absolute position
esp_err_t stepper_move_to(stepper_handle_t handle, int32_t target_pos, uint32_t speed_rpm);

// Run continuously
esp_err_t stepper_run_continuous(stepper_handle_t handle, int32_t speed_rpm);

// Stop
esp_err_t stepper_stop(stepper_handle_t handle);
```

### Status & Position

```c
bool stepper_is_running(stepper_handle_t handle);
int32_t stepper_get_position(stepper_handle_t handle);
esp_err_t stepper_set_position(stepper_handle_t handle, int32_t position);
int32_t stepper_get_target_position(stepper_handle_t handle);
stepper_mode_t stepper_get_mode(stepper_handle_t handle);
esp_err_t stepper_wait_complete(stepper_handle_t handle, uint32_t timeout_ms);
```

## Speed Calculation

### Formula
```
Frequency (Hz) = (RPM × steps_per_rev × microstep) / 60
```

### Examples (200-step motor)

| RPM | Microstep | Frequency | Achievable? |
|-----|-----------|-----------|-------------|
| 60  | 16        | 3.2 kHz   | ✅ Easy     |
| 120 | 16        | 6.4 kHz   | ✅ Easy     |
| 240 | 16        | 12.8 kHz  | ✅ Good     |
| 480 | 16        | 25.6 kHz  | ✅ Good     |
| 960 | 16        | 51.2 kHz  | ⚠️ Test     |
| 1875| 16        | 100 kHz   | ⚠️ Limit    |

## Troubleshooting

### Motor doesn't move
✅ Check power supply (12-36V)
✅ Check DIP switches (current setting)
✅ Verify GPIO connections
✅ Check if motor is enabled

### Motor vibrates/skips steps
✅ Reduce speed (lower RPM)
✅ Increase current (DIP switches)
✅ Reduce microstep
✅ Check max_speed_hz setting

### Motor runs in wrong direction
✅ Use negative steps: `stepper_move_steps(motor, -800, 60)`
✅ Or swap motor A+/A- wires

### Motor overheats
✅ Reduce current if possible
✅ Add heatsink to motor
✅ Disable motor when not in use

## Performance Tips

1. **Use appropriate microstep**: Higher = smoother but slower max speed
2. **Set realistic max_speed_hz**: Start with 50000 Hz, adjust based on testing
3. **Disable when idle**: `stepper_enable(motor, false)` to reduce heat
4. **Use callbacks**: Don't poll `stepper_is_running()` in tight loops

## Examples

See `examples/` directory:
- `example_3_motors.c` - Complete example with 3 motors

## License

MIT License

## Support

For issues or questions, please refer to the example code or check ESP-IDF documentation for LEDC driver.
