#include "app_states.h"


static const char *TAG = "MAIN";

static HSM_EVENT app_state_loading_handler(HSM *This, HSM_EVENT event, void *param);



static HSM_STATE app_state_loading;

void
app_state_hsm_init(DeviceHSM_t *me) {

    HSM_STATE_Create(&app_state_loading, "s_loading", app_state_loading_handler, NULL);

    HSM_Create((HSM *)me, "app", &app_state_loading);
}    





static HSM_EVENT 
app_state_loading_handler(HSM *This, HSM_EVENT event, void *param) {
    switch (event) {
        case HSME_ENTRY:
            ESP_LOGI(TAG, "Writing to Modbus register...");
            modbus_master_write_single_register(1, 0, 1111);
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