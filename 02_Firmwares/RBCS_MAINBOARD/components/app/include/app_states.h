#ifndef APP_STATES_H
#define APP_STATES_H


#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/spi_master.h"
#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_log.h"

#include "app_common.h"
#include "modbus_master_manager.h"
#include "modbus_slave_manager.h"
#include "esp_timer.h"
#include "esp_ticks.h"
#include "hsm.h"
#include "stepper_tb6600.h"


#define TOTAL_SLOTS		     	5
#define REGS_PER_BATTERY		55  


#ifdef __cplusplus
extern "C" {
#endif

#ifndef LOW
#define LOW  0
#endif

#ifndef HIGH
#define HIGH 1
#endif


// ============================================
// Battery Register Definitions
// ============================================
typedef enum {
    REG_STA_BMS_STATE = 0,
    REG_STA_CTRL_REQUEST,
    REG_STA_CTRL_RESPONSE,
    REG_STA_FET_CTRL_PIN,
    REG_STA_FET_STATUS,
    REG_STA_ALARM_BITS,
    REG_STA_FAULTS,
    REG_STA_PACK_VOLT,
    REG_STA_STACK_VOLT,
    REG_STA_PACK_CURRENT_HIGH,
    REG_STA_PACK_CURRENT_LOW,
    REG_STA_ID_VOLT,
    REG_STA_TEMP1_HIGH,
    REG_STA_TEMP1_LOW,
    REG_STA_TEMP2_HIGH,
    REG_STA_TEMP2_LOW,
    REG_STA_TEMP3_HIGH,
    REG_STA_TEMP3_LOW,
    REG_STA_CELL1,
    REG_STA_CELL2,
    REG_STA_CELL3,
    REG_STA_CELL4,
    REG_STA_CELL5,
    REG_STA_CELL6,
    REG_STA_CELL7,
    REG_STA_CELL8,
    REG_STA_CELL9,
    REG_STA_CELL10,
    REG_STA_CELL11,
    REG_STA_CELL12,
    REG_STA_CELL_NONE_1,
    REG_STA_CELL_NONE_2,
    REG_STA_CELL_NONE_3,
    REG_STA_CELL13,
    REG_STA_SAFETY_A,
    REG_STA_SAFETY_B,
    REG_STA_SAFETY_C,
    REG_STA_ACCU_INT_HIGH,
    REG_STA_ACCU_INT_LOW,
    REG_STA_ACCU_FRAC_HIGH,
    REG_STA_ACCU_FRAC_LOW,
    REG_STA_ACCU_TIME_HIGH,
    REG_STA_ACCU_TIME_LOW,
    REG_STA_PIN_PERCENT,
    REG_STA_PERCENT_TARGET,
    REG_STA_CELL_RESISTANCE,
    REG_STA_SOC_PERCENT,
    REG_STA_SOH_VALUE,
    REG_STA_CAPACITY,
    REG_STA_SINGLE_PARALLEL,
    TOTAL_STA_REGISTERS,
} SlotRegister_t;

// Cấu trúc lưu dữ liệu mỗi Pin
typedef struct {
    uint16_t registers[TOTAL_STA_REGISTERS];
    bool is_online;
    uint32_t last_update_ms;
    uint8_t error_count;
} Slot_Data_t;

typedef struct {
    Slot_Data_t slot[TOTAL_SLOTS];
    SemaphoreHandle_t data_mutex;
    uint32_t total_poll_count;
    uint32_t error_count;
} G_System_t;

typedef struct {
    uint16_t system_state;          // 1000
    uint16_t active_batteries;      // 1001
    uint16_t total_voltage;         // 1002
    uint16_t total_current_high;    // 1003
    uint16_t total_current_low;     // 1004
    uint16_t avg_temperature;       // 1005
    uint16_t min_soc;               // 1006
    uint16_t max_soc;               // 1007
    uint16_t alarm_status;          // 1008
    uint16_t fault_status;          // 1009
} G_Station_Info_t;


typedef enum {
    IDX_MOTOR1,
    IDX_MOTOR2,
    IDX_MOTOR3,
    MOTOR_TOTAL,
} MotorName_t;

typedef enum {
	HSME_LOOP = HSME_START,

    HSME_MOTOR_1_COMPLETE,
    HSME_MOTOR_2_COMPLETE,
    HSME_MOTOR_3_COMPLETE,
    HSME_LIMIT_SWITCH_1_TRIGGERED,
    HSME_LIMIT_SWITCH_1_RELEASED,
    HSME_LIMIT_SWITCH_2_TRIGGERED,
    HSME_LIMIT_SWITCH_2_RELEASED,
    HSME_LIMIT_SWITCH_3_TRIGGERED,
    HSME_LIMIT_SWITCH_3_RELEASED,

	HSME_LOADING_DONE,
	HSME_SET_TO_ZERO_POSTION,

    
	HSME_BLINK_1S_TIMER,
} HSMEvent_t;

typedef enum {
    MOTOR1_DIR_PUSH = STEPPER_DIR_CW,
    MOTOR1_DIR_PULL = STEPPER_DIR_CCW,
} Motor1Direction_t;
typedef enum {
    MOTOR2_DIR_PUSH = STEPPER_DIR_CW,
    MOTOR2_DIR_PULL = STEPPER_DIR_CCW,
} Motor2Direction_t;
typedef enum {
    MOTOR3_DIR_PUSH = STEPPER_DIR_CW,
    MOTOR3_DIR_PULL = STEPPER_DIR_CCW,
} Motor3Direction_t;
typedef enum {
    LIMIT_SWITCH_1_TRIGGERED = LOW,
    LIMIT_SWITCH_1_RELEASED = HIGH,
} LimitSwitch1State_t;
typedef enum {
    LIMIT_SWITCH_2_TRIGGERED = LOW,
    LIMIT_SWITCH_2_RELEASED = HIGH,
} LimitSwitch2State_t;
typedef enum {
    LIMIT_SWITCH_3_TRIGGERED = LOW,
    LIMIT_SWITCH_3_RELEASED = HIGH,
} LimitSwitch3State_t;

typedef struct {
	HSM parent;
	
	G_System_t g_system;
	G_Station_Info_t g_station_info;

    stepper_handle_t motor[MOTOR_TOTAL];
} DeviceHSM_t;


void app_state_hsm_init(DeviceHSM_t *me);


#ifdef __cplusplus
}
#endif

#endif // APP_STATES_H

