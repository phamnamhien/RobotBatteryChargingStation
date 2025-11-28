#ifndef APP_IO_H
#define APP_IO_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h> 


#ifdef __cplusplus
extern "C" {
#endif

// GPIO pins for Modbus Master
#define MASTER_UART_PORT        UART_NUM_3
#define MASTER_TX_PIN           GPIO_NUM_19
#define MASTER_RX_PIN           GPIO_NUM_18
#define MASTER_RTS_PIN          GPIO_NUM_5

// GPIO pins for Modbus Slave
#define SLAVE_UART_PORT         UART_NUM_2
#define SLAVE_TX_PIN            GPIO_NUM_17
#define SLAVE_RX_PIN            GPIO_NUM_16
#define SLAVE_RTS_PIN           GPIO_NUM_4

#define LIMIT_SWITCH_1_PIN      GPIO_NUM_34
#define MOTOR_DIR_PIN           GPIO_NUM_25
#define MOTOR_PWM_PIN           GPIO_NUM_26
#define MOTOR_ENABLE_PIN        GPIO_NUM_27


#ifdef __cplusplus
}
#endif

#endif // APP_IO_H

