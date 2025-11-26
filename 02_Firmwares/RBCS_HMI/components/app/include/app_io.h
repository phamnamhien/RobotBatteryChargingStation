#ifndef APP_IO_H
#define APP_IO_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h> 


#ifdef __cplusplus
extern "C" {
#endif

#define APP_IO_UART_NUM       UART_NUM_2
#define APP_IO_UART_TX_PIN    17   
#define APP_IO_UART_RX_PIN    16
#define APP_IO_UART_RTS_PIN   4



#ifdef __cplusplus
}
#endif

#endif // APP_IO_H

