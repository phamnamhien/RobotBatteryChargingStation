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
#include "modbus_manager.h"
#include "esp_timer.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

#include "lvgl.h"
#include "esp_lvgl_port.h"
#include "ui.h"

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

