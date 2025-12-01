#ifndef APP_STATES_H
#define APP_STATES_H


#include <stdio.h>
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "modbus_master_manager.h"
#include "esp_timer.h"

#include "hsm.h"
#include "esp_timer.h"
#include "esp_ticks.h"
#include "app_io.h"
#include "ui_support.h"

#include "lvgl.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_touch_gt911.h"
#include "ui.h"

#ifdef __cplusplus
extern "C" {
#endif


#define BMS_BATTERY_NUM        5

typedef struct {
    // BMS State Machine
    uint8_t bms_state;          // BMS state: 2=standby, 3=load, 4=charge, 5=error
    uint8_t ctrl_request;       // Control request from host
    uint8_t ctrl_response;      // Control response from BMS
    uint8_t fet_ctrl_pin;       // FET control pin status
    uint8_t fet_status;         // FET status bits: PDSG|DSG|PCHG|CHG
    uint16_t alarm_bits;        // Alarm status bits
    uint8_t faults;             // Fault flags: OCC|UV|OV|SCD|OCD
    
    // Voltage Measurements
    uint16_t pack_volt;         // Pack voltage in mV
    uint16_t stack_volt;        // Total stack voltage in mV
    uint16_t cell_volt[13];     // Individual cell voltages in mV (13 cells)
    uint16_t ld_volt;           // Load voltage in mV
    
    // Current Measurement
    int16_t pack_current;       // Pack current in mA (signed)
    
    // Temperature Sensors
    int16_t temp1;              // Temperature sensor 1 in 0.1°C
    int16_t temp2;              // Temperature sensor 2 in 0.1°C
    int16_t temp3;              // Temperature sensor 3 in 0.1°C
    
    // Capacity & State of Charge
    uint16_t capacity;          // Battery capacity in mAh
    uint8_t soc_percent;        // State of Charge in %
    uint16_t soh_value;         // State of Health value in mAh
    uint8_t pin_percent;        // Current percentage indicator
    uint8_t percent_target;     // Target percentage
    
    // Safety Status
    uint16_t safety_a;          // Safety status register A
    uint16_t safety_b;          // Safety status register B
    uint16_t safety_c;          // Safety status register C
    
    // Accumulator Values
    uint32_t accu_int;          // Accumulator integer part
    uint32_t accu_frac;         // Accumulator fractional part
    uint32_t accu_time;         // Accumulator time counter
    
    // Additional Parameters
    uint8_t cell_resistance;    // Cell internal resistance in mΩ
    uint8_t single_parallel;    // Battery configuration: single/parallel
    
} BMS_Data_t;

typedef enum {
	HSME_LOOP = HSME_START,

	HSME_LOADING_DONE,

	HSME_LOADING_COUNT_TIMER,
	HSME_BLINK_1S_TIMER,
} HSMEvent_t;

typedef struct {
	HSM parent;
	
	BMS_Data_t bms_data[BMS_BATTERY_NUM];
} DeviceHSM_t;



void app_state_hsm_init(DeviceHSM_t *me);


#ifdef __cplusplus
}
#endif

#endif // APP_STATES_H

