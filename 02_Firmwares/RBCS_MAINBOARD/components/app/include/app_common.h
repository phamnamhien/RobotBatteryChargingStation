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

#define LIMIT_SWITCH_1_PIN          GPIO_NUM_34
#define LIMIT_SWITCH_2_PIN          GPIO_NUM_35
#define LIMIT_SWITCH_3_PIN          GPIO_NUM_32
#define LIMIT_SWITCH_4_PIN          -1
#define LIMIT_SWITCH_5_PIN          -1
#define LIMIT_SWITCH_6_PIN          -1


#define MOTOR_DIR_1_PIN             GPIO_NUM_25
#define MOTOR_PWM_1_PIN             GPIO_NUM_26
#define MOTOR_DIR_2_PIN             GPIO_NUM_25
#define MOTOR_PWM_2_PIN             GPIO_NUM_26
#define MOTOR_DIR_3_PIN             GPIO_NUM_25
#define MOTOR_PWM_3_PIN             GPIO_NUM_26

#define SOLENOID_CTRL_PIN           GPIO_NUM_26






#ifdef __cplusplus
}
#endif

#endif // APP_COMMON_H

