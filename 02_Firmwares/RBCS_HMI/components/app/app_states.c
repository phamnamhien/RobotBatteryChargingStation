#include "app_states.h"


static const char *TAG = "HSM";

static HSM_EVENT app_state_loading_handler(HSM *This, HSM_EVENT event, void *param);
static HSM_EVENT app_state_idle_handler(HSM *This, HSM_EVENT event, void *param);


static void tm_loading_callback(void *arg);

static void blink_1s_timer_callback(void *arg);


static HSM_STATE app_state_loading;
static HSM_STATE app_state_idle;


tick_handle_t tm_loading;

tick_handle_t blink_1s_timer;


void
app_state_hsm_init(DeviceHSM_t *me) {

    ESP_ERROR_CHECK(ticks_create(&blink_1s_timer, 
                                 blink_1s_timer_callback,
                                 TICK_PERIODIC, 
                                 me));
    ESP_ERROR_CHECK(ticks_create(&tm_loading, 
                                 tm_loading_callback,
                                 TICK_PERIODIC, 
                                 me));

    HSM_STATE_Create(&app_state_loading, "s_loading", app_state_loading_handler, NULL);
    HSM_STATE_Create(&app_state_idle, "s_idle", app_state_idle_handler, NULL);

    HSM_Create((HSM *)me, "app", &app_state_loading);
}    




static HSM_EVENT 
app_state_loading_handler(HSM *This, HSM_EVENT event, void *param) {
    static uint8_t loading_count = 0;
    switch (event) {
        case HSME_ENTRY:
            // ESP_LOGI(TAG, "Writing to Modbus register...");
            // modbus_master_write_single_register(1, 0, 1111);
            ticks_start(tm_loading, 100);
            break;
        case HSME_INIT:
            break;
        case HSME_EXIT:
            loading_count = 0;
            ticks_stop(tm_loading);
            ticks_stop(blink_1s_timer);
            break;
        case HSME_LOADING_COUNT_TIMER:
            loading_count++;
            if (loading_count >= 100) {
                HSM_Tran((HSM *)This, &app_state_idle, NULL, NULL);
                ESP_LOGI(TAG, "Loading Done, transitioning to Idle State");
            } else {
                if (ui_lock(-1)) {
                    lv_bar_set_value(ui_barSplashLoading, loading_count, LV_ANIM_OFF);
                    ui_unlock();
                }
            }
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
            ui_load_screen_slide(ui_Screen2, LV_SCR_LOAD_ANIM_OVER_RIGHT, 20, 0);
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

static void tm_loading_callback(void *arg)
{
    HSM *This = (HSM *)arg;
    HSM_Run(This, HSME_LOADING_COUNT_TIMER, NULL);
}

static void blink_1s_timer_callback(void *arg)
{
    HSM *This = (HSM *)arg;
    HSM_Run(This, HSME_BLINK_1S_TIMER, NULL);
}

