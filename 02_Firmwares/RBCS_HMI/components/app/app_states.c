#include "app_states.h"


static const char *TAG = "HSM";

static HSM_EVENT app_state_loading_handler(HSM *This, HSM_EVENT event, void *param);
static HSM_EVENT app_state_main_handler(HSM *This, HSM_EVENT event, void *param);
static HSM_EVENT app_state_main_slot_1_handler(HSM *This, HSM_EVENT event, void *param);
static HSM_EVENT app_state_main_slot_2_handler(HSM *This, HSM_EVENT event, void *param);
static HSM_EVENT app_state_main_slot_3_handler(HSM *This, HSM_EVENT event, void *param);
static HSM_EVENT app_state_main_slot_4_handler(HSM *This, HSM_EVENT event, void *param);
static HSM_EVENT app_state_main_slot_5_handler(HSM *This, HSM_EVENT event, void *param);
static HSM_EVENT app_state_setting_handler(HSM *This, HSM_EVENT event, void *param);

static void tm_loading_callback(void *arg);

static void blink_1s_timer_callback(void *arg);


static HSM_STATE app_state_loading;
static HSM_STATE app_state_main;
static HSM_STATE app_state_main_slot_1;
static HSM_STATE app_state_main_slot_2;
static HSM_STATE app_state_main_slot_3;
static HSM_STATE app_state_main_slot_4;
static HSM_STATE app_state_main_slot_5;
static HSM_STATE app_state_setting;

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
    HSM_STATE_Create(&app_state_main, "s_main", app_state_main_handler, NULL);
    HSM_STATE_Create(&app_state_main_slot_1, "s_main_sl1", app_state_main_slot_1_handler, NULL);
    HSM_STATE_Create(&app_state_main_slot_2, "s_main_sl2", app_state_main_slot_2_handler, NULL);
    HSM_STATE_Create(&app_state_main_slot_3, "s_main_sl3", app_state_main_slot_3_handler, NULL);
    HSM_STATE_Create(&app_state_main_slot_4, "s_main_sl4", app_state_main_slot_4_handler, NULL);
    HSM_STATE_Create(&app_state_main_slot_5, "s_main_sl5", app_state_main_slot_5_handler, NULL);
    HSM_STATE_Create(&app_state_setting, "s_setting", app_state_setting_handler, NULL);

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
                HSM_Tran((HSM *)This, &app_state_main, NULL, NULL);
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
app_state_main_handler(HSM *This, HSM_EVENT event, void *param) {
    switch (event) {
        case HSME_ENTRY:
            ui_load_screen(ui_scrMain);
            ESP_LOGI(TAG, "Entered Main State");
            break;
        case HSME_INIT:
            break;
        case HSME_EXIT:
            break;
        case HSME_MAIN_SLOT_1_CLICKED:
            HSM_Tran((HSM *)This, &app_state_main_slot_1, NULL, NULL);
            break;
        case HSME_MAIN_SLOT_2_CLICKED:
            HSM_Tran((HSM *)This, &app_state_main_slot_2, NULL, NULL);
            break;
        case HSME_MAIN_SLOT_3_CLICKED:
            HSM_Tran((HSM *)This, &app_state_main_slot_3, NULL, NULL);
            break;
        case HSME_MAIN_SLOT_4_CLICKED:
            HSM_Tran((HSM *)This, &app_state_main_slot_4, NULL, NULL);
            break;
        case HSME_MAIN_SLOT_5_CLICKED:
            HSM_Tran((HSM *)This, &app_state_main_slot_5, NULL, NULL);
            break;
        case HSME_CHANGE_SCR_MAIN_TO_SETTING:
            HSM_Tran((HSM *)This, &app_state_setting, NULL, NULL);
            break;
        default:
            return event;
    }
    return 0;
}

static HSM_EVENT 
app_state_main_slot_1_handler(HSM *This, HSM_EVENT event, void *param) {
    switch (event) {
        case HSME_ENTRY:
            ESP_LOGI(TAG, "Main Slot 1 Clicked");
            break;
        case HSME_INIT:
            break;
        case HSME_EXIT:
            break;
        case HSME_MAIN_SLOT_1_CLICKED:
            HSM_Tran((HSM *)This, &app_state_main, NULL, NULL);
            break;
        case HSME_MAIN_SLOT_2_CLICKED:
            HSM_Tran((HSM *)This, &app_state_main_slot_2, NULL, NULL);
            break;
        case HSME_MAIN_SLOT_3_CLICKED:
            HSM_Tran((HSM *)This, &app_state_main_slot_3, NULL, NULL);
            break;
        case HSME_MAIN_SLOT_4_CLICKED:
            HSM_Tran((HSM *)This, &app_state_main_slot_4, NULL, NULL);
            break;
        case HSME_MAIN_SLOT_5_CLICKED:
            HSM_Tran((HSM *)This, &app_state_main_slot_5, NULL, NULL);
            break;
        default:
            return event;
    }
    return 0;
}
static HSM_EVENT 
app_state_main_slot_2_handler(HSM *This, HSM_EVENT event, void *param) {
    switch (event) {
        case HSME_ENTRY:
            ESP_LOGI(TAG, "Main Slot 2 Clicked");
            break;
        case HSME_INIT:
            break;
        case HSME_EXIT:
            break;
        case HSME_MAIN_SLOT_1_CLICKED:
            HSM_Tran((HSM *)This, &app_state_main_slot_1, NULL, NULL);
            break;
        case HSME_MAIN_SLOT_2_CLICKED:
            HSM_Tran((HSM *)This, &app_state_main, NULL, NULL);
            break;
        case HSME_MAIN_SLOT_3_CLICKED:
            HSM_Tran((HSM *)This, &app_state_main_slot_3, NULL, NULL);
            break;
        case HSME_MAIN_SLOT_4_CLICKED:
            HSM_Tran((HSM *)This, &app_state_main_slot_4, NULL, NULL);
            break;
        case HSME_MAIN_SLOT_5_CLICKED:
            HSM_Tran((HSM *)This, &app_state_main_slot_5, NULL, NULL);
            break;
        default:
            return event;
    }
    return 0;
}
static HSM_EVENT 
app_state_main_slot_3_handler(HSM *This, HSM_EVENT event, void *param) {
    switch (event) {
        case HSME_ENTRY:
            ESP_LOGI(TAG, "Main Slot 3 Clicked");
            break;
        case HSME_INIT:
            break;
        case HSME_EXIT:
            break;
        case HSME_MAIN_SLOT_1_CLICKED:
            HSM_Tran((HSM *)This, &app_state_main_slot_1, NULL, NULL);
            break;
        case HSME_MAIN_SLOT_2_CLICKED:
            HSM_Tran((HSM *)This, &app_state_main_slot_2, NULL, NULL);
            break;
        case HSME_MAIN_SLOT_3_CLICKED:
            HSM_Tran((HSM *)This, &app_state_main, NULL, NULL);
            break;
        case HSME_MAIN_SLOT_4_CLICKED:
            HSM_Tran((HSM *)This, &app_state_main_slot_4, NULL, NULL);
            break;
        case HSME_MAIN_SLOT_5_CLICKED:
            HSM_Tran((HSM *)This, &app_state_main_slot_5, NULL, NULL);
            break;
        default:
            return event;
    }
    return 0;
}
static HSM_EVENT 
app_state_main_slot_4_handler(HSM *This, HSM_EVENT event, void *param) {
    switch (event) {
        case HSME_ENTRY:
            ESP_LOGI(TAG, "Main Slot 4 Clicked");
            break;
        case HSME_INIT:
            break;
        case HSME_EXIT:
            break;
        case HSME_MAIN_SLOT_1_CLICKED:
            HSM_Tran((HSM *)This, &app_state_main_slot_1, NULL, NULL);
            break;
        case HSME_MAIN_SLOT_2_CLICKED:
            HSM_Tran((HSM *)This, &app_state_main_slot_2, NULL, NULL);
            break;
        case HSME_MAIN_SLOT_3_CLICKED:
            HSM_Tran((HSM *)This, &app_state_main_slot_3, NULL, NULL);
            break;
        case HSME_MAIN_SLOT_4_CLICKED:
            HSM_Tran((HSM *)This, &app_state_main, NULL, NULL);
            break;
        case HSME_MAIN_SLOT_5_CLICKED:
            HSM_Tran((HSM *)This, &app_state_main_slot_5, NULL, NULL);
            break;
        default:
            return event;
    }
    return 0;
}
static HSM_EVENT 
app_state_main_slot_5_handler(HSM *This, HSM_EVENT event, void *param) {
    switch (event) {
        case HSME_ENTRY:
            ESP_LOGI(TAG, "Main Slot 5 Clicked");
            break;
        case HSME_INIT:
            break;
        case HSME_EXIT:
            break;
        case HSME_MAIN_SLOT_1_CLICKED:
            HSM_Tran((HSM *)This, &app_state_main_slot_1, NULL, NULL);
            break;
        case HSME_MAIN_SLOT_2_CLICKED:
            HSM_Tran((HSM *)This, &app_state_main_slot_2, NULL, NULL);
            break;
        case HSME_MAIN_SLOT_3_CLICKED:
            HSM_Tran((HSM *)This, &app_state_main_slot_3, NULL, NULL);
            break;
        case HSME_MAIN_SLOT_4_CLICKED:
            HSM_Tran((HSM *)This, &app_state_main_slot_4, NULL, NULL);
            break;
        case HSME_MAIN_SLOT_5_CLICKED:
            HSM_Tran((HSM *)This, &app_state_main, NULL, NULL);
            break;
        default:
            return event;
    }
    return 0;
}

static HSM_EVENT 
app_state_setting_handler(HSM *This, HSM_EVENT event, void *param) {
    switch (event) {
        case HSME_ENTRY:
            ui_load_screen(ui_scrSetting);
            ESP_LOGI(TAG, "Entered Setting State");
            break;
        case HSME_INIT:
            break;
        case HSME_EXIT:
            break;
        case HSME_CHANGE_SCR_SETTING_TO_MAIN:
        HSM_Tran((HSM *)This, &app_state_main, NULL, NULL);
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

