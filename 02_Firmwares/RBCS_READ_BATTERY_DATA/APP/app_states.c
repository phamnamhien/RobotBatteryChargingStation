/*
 * app_states.c
 *
 *  Created on: Dec 4, 2025
 *      Author: admin
 */
#include "app_states.h"






static HSM_EVENT app_state_setting_baudrate_handler(HSM *This, HSM_EVENT event, void *param);
static HSM_EVENT app_state_run_handler(HSM *This, HSM_EVENT event, void *param);
static HSM_EVENT app_state_fault_handler(HSM *This, HSM_EVENT event, void *param);
static HSM_EVENT app_state_bat_not_connected_handler(HSM *This, HSM_EVENT event, void *param);

static void hsmt_custom_tick_callback(void* arg);
static void hsmt_setting_done_tick_callback(void* arg);
static void hsmt_turn_off_led_stt_tick_callback(void* arg);

static HSM_STATE app_state_setting_baudrate;
static HSM_STATE app_state_run;
static HSM_STATE app_state_fault;
static HSM_STATE app_state_bat_not_connected;

static hsmTimerId hsmt_custom_tick_handle;
static hsmTimerId hsmt_turn_off_led_stt_tick_handle;
static hsmTimerId hsmt_setting_done_tick_handle;


static hsmTimerDef_t hsmt_custom_tick_att = {
		"tCustom",
		hsmt_custom_tick_callback,
};
static hsmTimerDef_t hsmt_turn_off_led_stt_tick_att = {
		"tTurnOffLEDStt",
		hsmt_turn_off_led_stt_tick_callback,
};
static hsmTimerDef_t hsmt_setting_done_tick_att = {
		"tSettingDone",
		hsmt_setting_done_tick_callback,
};


void app_states_hsm_init(DeviceHSM_t *me) {

	hsmt_custom_tick_handle =  hsmTimerCreate(&hsmt_custom_tick_att, hsmTimerPeriodic, me);
	hsmt_turn_off_led_stt_tick_handle =  hsmTimerCreate(&hsmt_turn_off_led_stt_tick_att, hsmTimerOnce, me);
	hsmt_setting_done_tick_handle =  hsmTimerCreate(&hsmt_setting_done_tick_att, hsmTimerOnce, me);


	HSM_STATE_Create(&app_state_setting_baudrate, "s_setbaud", app_state_setting_baudrate_handler, NULL);
	HSM_STATE_Create(&app_state_run, "s_run", app_state_run_handler, NULL);
	HSM_STATE_Create(&app_state_fault, "s_fault", app_state_fault_handler, NULL);
	HSM_STATE_Create(&app_state_bat_not_connected, "s_not_connected", app_state_bat_not_connected_handler, NULL);

	if(!me->modbus_address) {
		HSM_Create((HSM *)me, "app", &app_state_setting_baudrate);
	} else {
		HSM_Create((HSM *)me, "app", &app_state_run);
	}
}


static HSM_EVENT app_state_setting_baudrate_handler(HSM *This, HSM_EVENT event, void *param) {
	DeviceHSM_t* me = (DeviceHSM_t*)This;
    switch (event) {
        case HSME_ENTRY:
        	HAL_GPIO_WritePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin, LED_OFF);
        	HAL_GPIO_WritePin(LED_RUN_GPIO_Port, LED_RUN_Pin, LED_OFF);
        	HAL_GPIO_WritePin(LED_STT_GPIO_Port, LED_STT_Pin, LED_OFF);
            break;
        case HSME_INIT:
        	hsmTimerStart(hsmt_custom_tick_handle, 500);
        	hsmTimerStart(hsmt_setting_done_tick_handle, SETTING_WAIT_DONE_MS);
            break;
        case HSME_EXIT:
        	hsmTimerStop(hsmt_custom_tick_handle);
        	hsmTimerStop(hsmt_setting_done_tick_handle);
            break;
        case HSME_CUSTOM_TICK_UPDATE:
        	HAL_GPIO_TogglePin(LED_RUN_GPIO_Port, LED_RUN_Pin);

        	break;
        case HSME_SET_INVALID_BAUD:
        	hsmTimerStart(hsmt_setting_done_tick_handle, SETTING_WAIT_DONE_MS);
        	break;
        case HSME_SET_BAUD_CHANGE_VALUE:
        	hsmTimerStart(hsmt_setting_done_tick_handle, SETTING_WAIT_DONE_MS);
        	break;
        case HSME_SETTING_DONE_TICK_UPDATE:
        	me->storage.baudrate = me->modbus_address;
        	EE_Write();
        	HAL_GPIO_WritePin(LED_RUN_GPIO_Port, LED_RUN_Pin, LED_ON);
        	hsmTimerStop(hsmt_setting_done_tick_handle);
        	break;
        default:
            return event;
    }
    return 0;
}

static HSM_EVENT app_state_run_handler(HSM *This, HSM_EVENT event, void *param) {
	DeviceHSM_t* me = (DeviceHSM_t*)This;
    switch (event) {
        case HSME_ENTRY:
        	HAL_GPIO_WritePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin, LED_OFF);
        	HAL_GPIO_WritePin(LED_RUN_GPIO_Port, LED_RUN_Pin, LED_ON);
        	HAL_GPIO_WritePin(LED_STT_GPIO_Port, LED_STT_Pin, LED_OFF);

            break;
        case HSME_INIT:
    		HAL_GPIO_WritePin(EMERGENCY_GPIO_Port, EMERGENCY_Pin, EM_PASSTIVE);
    		HAL_GPIO_WritePin(CHARGE_CTRL_GPIO_Port, CHARGE_CTRL_Pin, CHRG_OFF);
            break;
        case HSME_EXIT:
        	hsmTimerStop(hsmt_turn_off_led_stt_tick_handle);
            break;
        case HSME_SWITCH_LIMIT_ACTIVE:
        	me->dataModbusSlave[REG_STA_IS_PIN_IN_SLOT] = SLOT_FULL;
        	break;
        case HSME_SWITCH_LIMIT_PASSTIVE:
        	for(uint16_t i = 0; i < TOTAL_BAT_REGISTERS; i++) {
        		me->dataModbusSlave[i] = 0;
        	}
        	me->dataModbusSlave[REG_STA_IS_PIN_IN_SLOT] = SLOT_EMPTY;
        	break;
        case HSME_COMM_RECEIVED_OK:
        	for(uint16_t i = 0; i < TOTAL_BAT_REGISTERS; i++) {
        		me->dataModbusSlave[i] = me->dataModbusMaster[i];
        	}
        	// Check Emergency
        	if(me->dataModbusSlave[REG_STA_IS_EMERGENCY_STOP]) {
        		HAL_GPIO_WritePin(EMERGENCY_GPIO_Port, EMERGENCY_Pin, EM_ACTIVE);
        	} else {
        		HAL_GPIO_WritePin(EMERGENCY_GPIO_Port, EMERGENCY_Pin, EM_PASSTIVE);

            	// Check Chrg
            	if(me->dataModbusSlave[REG_STA_CHRG_CTRL]) {
            		HAL_GPIO_WritePin(CHARGE_CTRL_GPIO_Port, CHARGE_CTRL_Pin, CHRG_ON);
            	} else {
            		HAL_GPIO_WritePin(CHARGE_CTRL_GPIO_Port, CHARGE_CTRL_Pin, CHRG_OFF);
            	}
        	}
        	// Turn ON LED
        	HAL_GPIO_WritePin(LED_STT_GPIO_Port, LED_STT_Pin, LED_ON);
        	hsmTimerStart(hsmt_turn_off_led_stt_tick_handle, LED_STATUS_ON_MS);
        	break;

        case HSME_BAT_RECEIVED_TIMEOUT:
        	for(uint16_t i = 0; i < TOTAL_BAT_REGISTERS; i++) {
        		me->dataModbusSlave[i] = 0;
        	}
        	me->dataModbusSlave[REG_STA_IS_PIN_TIMEOUT] = 1;
        	HSM_Tran(This, &app_state_bat_not_connected, 0, NULL);
        	break;
        case HSME_BAT_RECEIVED_OK:
        	me->dataModbusSlave[REG_STA_IS_PIN_TIMEOUT] = 0;
        	// Check Fault:
        	if(me->dataModbusSlave[REG_STA_FAULTS]) {
        		HSM_Tran(This, &app_state_fault, 0, NULL);
        	}

        	break;
        case HSME_TURN_OFF_LED_STT_TICK_UPDATE:
        	HAL_GPIO_WritePin(LED_STT_GPIO_Port, LED_STT_Pin, LED_OFF);
        	break;
        default:
            return event;
    }
    return 0;
}
static HSM_EVENT app_state_fault_handler(HSM *This, HSM_EVENT event, void *param) {
	DeviceHSM_t* me = (DeviceHSM_t*)This;
    switch (event) {
        case HSME_ENTRY:
        	HAL_GPIO_WritePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin, LED_ON);
        	HAL_GPIO_WritePin(LED_RUN_GPIO_Port, LED_RUN_Pin, LED_OFF);
        	HAL_GPIO_WritePin(LED_STT_GPIO_Port, LED_STT_Pin, LED_OFF);
            break;
        case HSME_INIT:
    		HAL_GPIO_WritePin(EMERGENCY_GPIO_Port, EMERGENCY_Pin, EM_PASSTIVE);
    		HAL_GPIO_WritePin(CHARGE_CTRL_GPIO_Port, CHARGE_CTRL_Pin, CHRG_OFF);
            break;
        case HSME_EXIT:

            break;
        case HSME_SWITCH_LIMIT_PASSTIVE:
        	HSM_Tran(This, &app_state_run, 0, NULL);
        	break;
        case HSME_BAT_RECEIVED_OK:
        	if(!me->dataModbusSlave[REG_STA_FAULTS]) {
        		HSM_Tran(This, &app_state_run, 0, NULL);
        	}
        	break;
        default:
            return event;
    }
    return 0;
}

static HSM_EVENT app_state_bat_not_connected_handler(HSM *This, HSM_EVENT event, void *param) {
	DeviceHSM_t* me = (DeviceHSM_t*)This;
    switch (event) {
        case HSME_ENTRY:
        	HAL_GPIO_WritePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin, LED_ON);
        	HAL_GPIO_WritePin(LED_RUN_GPIO_Port, LED_RUN_Pin, LED_OFF);
        	HAL_GPIO_WritePin(LED_STT_GPIO_Port, LED_STT_Pin, LED_OFF);
            break;
        case HSME_INIT:
    		HAL_GPIO_WritePin(EMERGENCY_GPIO_Port, EMERGENCY_Pin, EM_PASSTIVE);
    		HAL_GPIO_WritePin(CHARGE_CTRL_GPIO_Port, CHARGE_CTRL_Pin, CHRG_OFF);

    		hsmTimerStart(hsmt_custom_tick_handle, 500);
            break;
        case HSME_EXIT:
        	hsmTimerStop(hsmt_custom_tick_handle);
            break;
        case HSME_SWITCH_LIMIT_PASSTIVE:
        	HSM_Tran(This, &app_state_run, 0, NULL);
        	break;
        case HSME_BAT_RECEIVED_OK:
        	if(!me->dataModbusSlave[REG_STA_FAULTS]) {
        		HSM_Tran(This, &app_state_run, 0, NULL);
        	} else {
        		HSM_Tran(This, &app_state_fault, 0, NULL);
        	}
        	break;
        case HSME_CUSTOM_TICK_UPDATE:
        	HAL_GPIO_TogglePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin);
			break;
        default:
            return event;
    }
    return 0;
}

static void
hsmt_custom_tick_callback(void* arg) {
	HSM *This = (HSM *)arg;
	HSM_Run(This, HSME_CUSTOM_TICK_UPDATE, 0);
}
static void
hsmt_turn_off_led_stt_tick_callback(void* arg) {
	HSM *This = (HSM *)arg;
	HSM_Run(This, HSME_TURN_OFF_LED_STT_TICK_UPDATE, 0);
}
static void
hsmt_setting_done_tick_callback(void* arg) {
	HSM *This = (HSM *)arg;
	HSM_Run(This, HSME_SETTING_DONE_TICK_UPDATE, 0);
}


