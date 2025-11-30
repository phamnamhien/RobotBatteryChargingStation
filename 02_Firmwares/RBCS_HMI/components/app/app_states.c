#include "app_states.h"


static const char *TAG = "HSM";

static HSM_EVENT app_state_loading_handler(HSM *This, HSM_EVENT event, void *param);
static HSM_EVENT app_state_idle_handler(HSM *This, HSM_EVENT event, void *param);

static void blink_1s_timer_callback(void *arg);


static HSM_STATE app_state_loading;
static HSM_STATE app_state_idle;


tick_handle_t blink_1s_timer;


void
app_state_hsm_init(DeviceHSM_t *me) {

    ESP_ERROR_CHECK(ticks_create(&blink_1s_timer, 
                                 blink_1s_timer_callback,
                                 TICK_PERIODIC, 
                                 me));


    HSM_STATE_Create(&app_state_loading, "s_loading", app_state_loading_handler, NULL);
    HSM_STATE_Create(&app_state_idle, "s_idle", app_state_idle_handler, NULL);

    HSM_Create((HSM *)me, "app", &app_state_loading);
}    



uint8_t count = 0;
static HSM_EVENT 
app_state_loading_handler(HSM *This, HSM_EVENT event, void *param) {
    switch (event) {
        case HSME_ENTRY:
            // ESP_LOGI(TAG, "Writing to Modbus register...");
            // modbus_master_write_single_register(1, 0, 1111);
            ESP_ERROR_CHECK(ticks_start(blink_1s_timer, 2000));
            break;
        case HSME_INIT:
            break;
        case HSME_EXIT:
            break;
        case HSME_BLINK_1S_TIMER:
            if(++count % 2 == 0) {
                // Slide từ trái sang
                if (ui_lock(-1)) {
                    ui_load_screen_slide(ui_Screen1, LV_SCR_LOAD_ANIM_OVER_LEFT, 300, 0);
                    ui_unlock();
                }
            } else {
                // Slide từ phải sang
                if (ui_lock(-1)) {
                    ui_load_screen_slide(ui_Screen2, LV_SCR_LOAD_ANIM_OVER_RIGHT, 300, 0);
                    ui_unlock();
                }
            }
            // if(++count > 100) {
            //     count = 0;
            // }
            // if (ui_lock(-1)) {
            //     lv_bar_set_value(ui_barSplashLoading, count, LV_ANIM_OFF);
            //     ui_unlock();
            // }
            ESP_LOGI(TAG, "1s Blink Timer Event");
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
            ESP_LOGI(TAG, "Entered Idle State");
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

