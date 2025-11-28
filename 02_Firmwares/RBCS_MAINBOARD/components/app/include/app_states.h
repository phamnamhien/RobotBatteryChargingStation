#ifndef APP_STATES_H
#define APP_STATES_H


#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "modbus_master_manager.h"
#include "modbus_slave_manager.h"
#include "esp_timer.h"
#include "esp_ticks.h"
#include "hsm.h"
#include "app_io.h"


#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	HSME_LOOP = HSME_START,

	HSME_LOADING_DONE,
	HSME_SET_TO_ZERO_POSTION,

	HSME_BLINK_1S_TIMER,
} HSMEvent_t;

typedef struct {
	HSM parent;
	
} DeviceHSM_t;


void app_state_hsm_init(DeviceHSM_t *me);


#ifdef __cplusplus
}
#endif

#endif // APP_STATES_H

