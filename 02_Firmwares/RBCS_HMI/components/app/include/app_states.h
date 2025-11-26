#ifndef APP_STATES_H
#define APP_STATES_H


#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "modbus_manager.h"


#include "hsm.h"
#include "app_io.h"


#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	HSME_LOOP = HSME_START,

	HSME_LOADING_DONE,

} HSMEvent_t;

typedef struct {
	HSM parent;
	
} DeviceHSM_t;


void app_state_hsm_init(DeviceHSM_t *me);


#ifdef __cplusplus
}
#endif

#endif // APP_STATES_H

