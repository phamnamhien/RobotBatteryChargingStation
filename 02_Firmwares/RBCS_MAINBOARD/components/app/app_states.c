#include "app_states.h"


static const char *TAG = "HSM";

static HSM_EVENT app_state_loading_handler(HSM *This, HSM_EVENT event, void *param);

static void blink_1s_timer_callback(void *arg);


static HSM_STATE app_state_loading;


tick_handle_t blink_1s_timer;


void
app_state_hsm_init(DeviceHSM_t *me) {

    ESP_ERROR_CHECK(ticks_create(&blink_1s_timer, 
                                 blink_1s_timer_callback,
                                 TICK_PERIODIC, 
                                 me));


    HSM_STATE_Create(&app_state_loading, "s_loading", app_state_loading_handler, NULL);

    HSM_Create((HSM *)me, "app", &app_state_loading);
}    

static HSM_EVENT 
app_state_loading_handler(HSM *This, HSM_EVENT event, void *param) {
    switch (event) {
        case HSME_ENTRY:
            // ESP_LOGI(TAG, "Writing to Modbus register...");
            // modbus_master_write_single_register(1, 0, 1111);
            ESP_ERROR_CHECK(ticks_start(blink_1s_timer, 500));
            break;
        case HSME_INIT:
            break;
        case HSME_EXIT:
            break;
        case HSME_BLINK_1S_TIMER:
            ESP_LOGI(TAG, "1s Blink Timer Event");
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