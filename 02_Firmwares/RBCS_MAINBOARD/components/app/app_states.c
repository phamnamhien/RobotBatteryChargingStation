#include "app_states.h"


static const char *TAG = "HSM";


static HSM_EVENT app_state_loading_handler(HSM *This, HSM_EVENT event, void *param);

static HSM_EVENT app_state_get_zero_postion_motor_1_handler(HSM *This, HSM_EVENT event, void *param);
static HSM_EVENT app_state_get_zero_postion_motor_2_handler(HSM *This, HSM_EVENT event, void *param);
static HSM_EVENT app_state_get_zero_postion_motor_3_handler(HSM *This, HSM_EVENT event, void *param);

static HSM_EVENT app_state_go_to_zero_postion_motor_1_handler(HSM *This, HSM_EVENT event, void *param);
static HSM_EVENT app_state_go_to_zero_postion_motor_2_handler(HSM *This, HSM_EVENT event, void *param);
static HSM_EVENT app_state_go_to_zero_postion_motor_3_handler(HSM *This, HSM_EVENT event, void *param);

static HSM_EVENT app_state_idle_handler(HSM *This, HSM_EVENT event, void *param);

static HSM_EVENT app_state_run_handler(HSM *This, HSM_EVENT event, void *param);

static HSM_EVENT app_state_stop_handler(HSM *This, HSM_EVENT event, void *param);
static HSM_EVENT app_state_test_handler(HSM *This, HSM_EVENT event, void *param);

void motor_axis_z_complete_cb(stepper_handle_t handle, void *user_data);
void motor_axis_x_complete_cb(stepper_handle_t handle, void *user_data);
void motor_get_battery_complete_cb(stepper_handle_t handle, void *user_data);

static void blink_1s_timer_callback(void *arg);


static HSM_STATE app_state_loading;

static HSM_STATE app_state_get_zero_postion_motor_1;
static HSM_STATE app_state_get_zero_postion_motor_2;
static HSM_STATE app_state_get_zero_postion_motor_3;

static HSM_STATE app_state_go_to_zero_postion_motor_1;
static HSM_STATE app_state_go_to_zero_postion_motor_2;
static HSM_STATE app_state_go_to_zero_postion_motor_3;

static HSM_STATE app_state_idle;
static HSM_STATE app_state_run;
static HSM_STATE app_state_stop;
static HSM_STATE app_state_test;

tick_handle_t blink_1s_timer;


void
app_state_hsm_init(DeviceHSM_t *me) {

    ESP_ERROR_CHECK(ticks_create(&blink_1s_timer, 
                                 blink_1s_timer_callback,
                                 TICK_PERIODIC, 
                                 me));

    HSM_STATE_Create(&app_state_loading, "s_loading", app_state_loading_handler, NULL);
    HSM_STATE_Create(&app_state_get_zero_postion_motor_1, "s_szpmotor1", app_state_get_zero_postion_motor_1_handler, NULL);
    HSM_STATE_Create(&app_state_get_zero_postion_motor_2, "s_szpmotor2", app_state_get_zero_postion_motor_2_handler, NULL);
    HSM_STATE_Create(&app_state_get_zero_postion_motor_3, "s_szpmotor3", app_state_get_zero_postion_motor_3_handler, NULL);
    HSM_STATE_Create(&app_state_go_to_zero_postion_motor_1, "s_gzpmotor1", app_state_go_to_zero_postion_motor_1_handler, NULL);
    HSM_STATE_Create(&app_state_go_to_zero_postion_motor_2, "s_gzpmotor2", app_state_go_to_zero_postion_motor_2_handler, NULL);
    HSM_STATE_Create(&app_state_go_to_zero_postion_motor_3, "s_gzpmotor3", app_state_go_to_zero_postion_motor_3_handler, NULL);

    HSM_STATE_Create(&app_state_idle, "s_idle", app_state_idle_handler, NULL);


    HSM_STATE_Create(&app_state_run, "s_run", app_state_run_handler, NULL);
    HSM_STATE_Create(&app_state_stop, "s_stop", app_state_stop_handler, NULL);
    
    HSM_STATE_Create(&app_state_test, "s_test", app_state_test_handler, NULL);

    HSM_Create((HSM *)me, "app", &app_state_loading);
    ESP_LOGI(TAG, "App HSM initialized.");
}    

static HSM_EVENT 
app_state_loading_handler(HSM *This, HSM_EVENT event, void *param) {
    DeviceHSM_t *me = (DeviceHSM_t *)This;
    switch (event) {
        case HSME_ENTRY:
            // ESP_ERROR_CHECK(ticks_start(blink_1s_timer, 500));
            break;
        case HSME_INIT:
            // HSM_Trans
            ESP_LOGI(TAG, "Go to Set Zero Postion.");
            break;
        case HSME_EXIT:
            break;
            
        default:
            return event;
    }
    return 0;
}


static HSM_EVENT 
app_state_get_zero_postion_motor_1_handler(HSM *This, HSM_EVENT event, void *param) {
    DeviceHSM_t *me = (DeviceHSM_t *)This;
    switch (event) {
        case HSME_ENTRY:

            break;
        case HSME_INIT:
            stepper_set_direction(me->motor[IDX_MOTOR1], MOTOR1_DIR_PUSH);
            stepper_run_continuous(me->motor[IDX_MOTOR1], MOTOR_SET_SPEED_ZP_RPM);
            ESP_LOGI(TAG, "Motor 1 to zero Postion.");
            break;
        case HSME_EXIT:
            break;
        case HSME_MOTOR_1_COMPLETE:

            break;
        case HSME_LIMIT_SWITCH_1_TRIGGERED:

            break;
        case HSME_LIMIT_SWITCH_2_TRIGGERED:

            break;
        default:
            return event;
    }
    return 0;
}
static HSM_EVENT 
app_state_get_zero_postion_motor_2_handler(HSM *This, HSM_EVENT event, void *param) {
    DeviceHSM_t *me = (DeviceHSM_t *)This;
    switch (event) {
        case HSME_ENTRY:

            break;
        case HSME_INIT:
            stepper_set_direction(me->motor[IDX_MOTOR2], MOTOR2_DIR_PUSH);
            stepper_run_continuous(me->motor[IDX_MOTOR2], MOTOR_SET_SPEED_ZP_RPM);
            ESP_LOGI(TAG, "Motor 2 to zero Postion.");
            break;
        case HSME_EXIT:
            break;
        case HSME_MOTOR_2_COMPLETE:

            break;
        case HSME_LIMIT_SWITCH_3_TRIGGERED:

            break;
        case HSME_LIMIT_SWITCH_4_TRIGGERED:

            break;
        default:
            return event;
    }
    return 0;
}
static HSM_EVENT 
app_state_get_zero_postion_motor_3_handler(HSM *This, HSM_EVENT event, void *param) {
    DeviceHSM_t *me = (DeviceHSM_t *)This;
    switch (event) {
        case HSME_ENTRY:

            break;
        case HSME_INIT:
            stepper_set_direction(me->motor[IDX_MOTOR3], MOTOR3_DIR_PUSH);
            stepper_run_continuous(me->motor[IDX_MOTOR3], MOTOR_SET_SPEED_ZP_RPM);
            ESP_LOGI(TAG, "Motor 3 to zero Postion.");
            break;
        case HSME_EXIT:
            break;
        case HSME_MOTOR_3_COMPLETE:

            break;
        case HSME_LIMIT_SWITCH_5_TRIGGERED:

            break;
        case HSME_LIMIT_SWITCH_6_TRIGGERED:

            break;
        default:
            return event;
    }
    return 0;
}

static HSM_EVENT 
app_state_go_to_zero_postion_motor_1_handler(HSM *This, HSM_EVENT event, void *param) {
    DeviceHSM_t *me = (DeviceHSM_t *)This;
    switch (event) {
        case HSME_ENTRY:
            ESP_LOGI(TAG, "Motor 3 go_to zero Posttion.");
            break;
        case HSME_INIT:
            break;
        case HSME_EXIT:
            break;
        case HSME_MOTOR_1_COMPLETE:

            break;
        case HSME_LIMIT_SWITCH_1_TRIGGERED:

            break;
        case HSME_LIMIT_SWITCH_2_TRIGGERED:

            break;
        default:
            return event;
    }
    return 0;
}
static HSM_EVENT 
app_state_go_to_zero_postion_motor_2_handler(HSM *This, HSM_EVENT event, void *param) {
    DeviceHSM_t *me = (DeviceHSM_t *)This;
    switch (event) {
        case HSME_ENTRY:
            ESP_LOGI(TAG, "Motor 2 go_to zero Posttion.");
            break;
        case HSME_INIT:
            break;
        case HSME_EXIT:
            break;
        case HSME_MOTOR_2_COMPLETE:

            break;
        case HSME_LIMIT_SWITCH_3_TRIGGERED:

            break;
        case HSME_LIMIT_SWITCH_4_TRIGGERED:

            break;
        default:
            return event;
    }
    return 0;
}
static HSM_EVENT 
app_state_go_to_zero_postion_motor_3_handler(HSM *This, HSM_EVENT event, void *param) {
    DeviceHSM_t *me = (DeviceHSM_t *)This;
    switch (event) {
        case HSME_ENTRY:
            ESP_LOGI(TAG, "Motor 3 go_to zero Posttion.");
            break;
        case HSME_INIT:
            break;
        case HSME_EXIT:
            break;
        case HSME_MOTOR_3_COMPLETE:

            break;
        case HSME_LIMIT_SWITCH_5_TRIGGERED:

            break;
        case HSME_LIMIT_SWITCH_6_TRIGGERED:

            break;
        default:
            return event;
    }
    return 0;
}

static HSM_EVENT 
app_state_idle_handler(HSM *This, HSM_EVENT event, void *param) {
    switch (event) {
        case HSME_ENTRY:
        
            break;
        case HSME_INIT:

            break;
        case HSME_EXIT:

            break;
        case HSME_LIMIT_SWITCH_7_TRIGGERED:   

            break;
        default:
            return event;
    }
    return 0;
}
static HSM_EVENT 
app_state_run_handler(HSM *This, HSM_EVENT event, void *param) {
    switch (event) {
        case HSME_ENTRY:
            
            break;
        case HSME_INIT:

            break;
        case HSME_EXIT:

            break;
        default:
            return event;
    }
    return 0;
}
static HSM_EVENT 
app_state_stop_handler(HSM *This, HSM_EVENT event, void *param) {
       switch (event) {
        case HSME_ENTRY:

            break;
        case HSME_INIT:
            break;
        case HSME_EXIT:
            break;
        default:
            return event;
    }
    return 0; 
}
static HSM_EVENT 
app_state_test_handler(HSM *This, HSM_EVENT event, void *param) {
    switch (event) {
        case HSME_ENTRY:

            break;
        case HSME_INIT:
            break;
        case HSME_EXIT:
            break;
        default:
            return event;
    }
    return 0; 
}

static void blink_1s_timer_callback(void *arg)
{
    HSM *This = (HSM *)arg;
    HSM_Run(This, HSME_BLINK_1S_TIMER, NULL);
}

