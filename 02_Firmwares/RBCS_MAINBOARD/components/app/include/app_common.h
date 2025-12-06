#ifndef APP_COMMON_H
#define APP_COMMON_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h> 


#ifdef __cplusplus
extern "C" {
#endif

// GPIO pins for Modbus Master
#define MASTER_UART_PORT        UART_NUM_1
#define MASTER_TX_PIN           GPIO_NUM_19
#define MASTER_RX_PIN           GPIO_NUM_18
#define MASTER_RTS_PIN          -1

// GPIO pins for Modbus Slave
#define SLAVE_UART_PORT             UART_NUM_2
#define SLAVE_TX_PIN                GPIO_NUM_17
#define SLAVE_RX_PIN                GPIO_NUM_16
#define SLAVE_RTS_PIN               -1

#define LIMIT_SWITCH_1_PIN          GPIO_NUM_1          // For Motor 1
#define LIMIT_SWITCH_2_PIN          GPIO_NUM_35
#define LIMIT_SWITCH_3_PIN          GPIO_NUM_32         // For Motor 2
#define LIMIT_SWITCH_4_PIN          -1
#define LIMIT_SWITCH_5_PIN          -1                  // For Motor 3
#define LIMIT_SWITCH_6_PIN          -1
#define LIMIT_SWITCH_7_PIN          -1                  // For Robot Detected

#define MOTOR_DIR_1_PIN                 GPIO_NUM_5
#define MOTOR_PWM_1_PIN                 GPIO_NUM_6
#define MOTOR_DIR_2_PIN                 GPIO_NUM_7
#define MOTOR_PWM_2_PIN                 GPIO_NUM_8
#define MOTOR_DIR_3_PIN                 GPIO_NUM_9
#define MOTOR_PWM_3_PIN                 GPIO_NUM_10
#define MOTOR_1_STEPS_PER_REVOLUTION    200    
#define MOTOR_2_STEPS_PER_REVOLUTION    200    
#define MOTOR_3_STEPS_PER_REVOLUTION    200  
#define MOTOR_1_MAX_SPEED_RMP           9000
#define MOTOR_2_MAX_SPEED_RMP           9000
#define MOTOR_3_MAX_SPEED_RMP           9000

#define MOTOR_SET_SPEED_ZP_RPM          200


#define SOLENOID_CTRL_PIN               GPIO_NUM_26



#ifdef __cplusplus
}
#endif

#endif // APP_COMMON_H

