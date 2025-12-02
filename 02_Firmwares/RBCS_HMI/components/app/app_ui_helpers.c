#include "app_states.h"

static const char *TAG = "UI_HELPERS";

/*--------------------------------------------------------------------*/
/* BATTERY FUNCTION HELPERS */
/*--------------------------------------------------------------------*/
void ui_update_main_slot_voltage(DeviceHSM_t *me, int8_t slot_index)
{
    if (slot_index >= BMS_BATTERY_NUM) {
        ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
        return;
    }
    
    // Array chứa các label voltage
    lv_obj_t *voltage_values[] = {
        ui_lbMainVoltageSlot1,
        ui_lbMainVoltageSlot2,
        ui_lbMainVoltageSlot3,
        ui_lbMainVoltageSlot4,
        ui_lbMainVoltageSlot5
    };
    
    // Lấy giá trị voltage (mV)
    uint16_t stack_volt_mv = me->bms_data[slot_index].stack_volt;
    
    // Chuyển sang V và format
    char voltage_str[16];
    snprintf(voltage_str, sizeof(voltage_str), "%.3fV", stack_volt_mv / 1000.0f);
    
    if (ui_lock(-1)) {
        lv_label_set_text(voltage_values[slot_index], voltage_str);
        ui_unlock();
    }
}

void ui_update_main_battery_percent(DeviceHSM_t *me, uint8_t slot_index)
{
    if (slot_index >= BMS_BATTERY_NUM) {
        ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
        return;
    }
    
    // Array chứa các bar và label
    lv_obj_t *battery_bars[] = {
        ui_barMainBatPercent1,
        ui_barMainBatPercent2,
        ui_barMainBatPercent3,
        ui_barMainBatPercent4,
        ui_barMainBatPercent5
    };
    
    lv_obj_t *soc_values[] = {
        ui_lbMainSlot1SOC,
        ui_lbMainSlot2SOC,
        ui_lbMainSlot3SOC,
        ui_lbMainSlot4SOC,
        ui_lbMainSlot5SOC
    };
    
    // Lấy SOC percent
    uint8_t soc = me->bms_data[slot_index].soc_percent;
    
    // Giới hạn 0-100%
    if (soc > 100) {
        soc = 100;
    }
    
    // Format label text
    char soc_str[8];
    snprintf(soc_str, sizeof(soc_str), "%d%%", soc);
    
    if (ui_lock(-1)) {
        // Cập nhật bar value
        lv_bar_set_value(battery_bars[slot_index], soc, LV_ANIM_OFF);
        
        // Cập nhật label text
        lv_label_set_text(soc_values[slot_index], soc_str);
        
        ui_unlock();
    }
}

void ui_update_main_slot_capacity(DeviceHSM_t *me, int8_t slot_index)
{
    if (slot_index >= BMS_BATTERY_NUM) {
        ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
        return;
    }
    
    // Array chứa các label capacity
    lv_obj_t *capacity_values[] = {
        ui_lbMainCapSlot1,
        ui_lbMainCapSlot2,
        ui_lbMainCapSlot3,
        ui_lbMainCapSlot4,
        ui_lbMainCapSlot5
    };
    
    // Lấy giá trị cap (mAh)
    uint16_t stack_capacity_mah = me->bms_data[slot_index].capacity;
    
    // Format
    char capacity_str[16];
    snprintf(capacity_str, sizeof(capacity_str), "%dmAh", stack_capacity_mah);
    
    if (ui_lock(-1)) {
        lv_label_set_text(capacity_values[slot_index], capacity_str);
        ui_unlock();
    }
}

/*--------------------------------------------------------------------*/
/* BATTERY DETAILS PANEL */
/*--------------------------------------------------------------------*/
void ui_show_slot_serial_detail(uint8_t slot_num)
{
    if (ui_lock(-1)) {
        // Tắt tất cả trước
        lv_obj_add_flag(ui_imgMainSlotSerialDetail1, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_imgMainSlotSerialDetail2, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_imgMainSlotSerialDetail3, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_imgMainSlotSerialDetail4, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_imgMainSlotSerialDetail5, LV_OBJ_FLAG_HIDDEN);
        
        // Bật slot được chọn (nếu slot_num != 0)
        switch (slot_num) {
            case 1:
                lv_obj_clear_flag(ui_imgMainSlotSerialDetail1, LV_OBJ_FLAG_HIDDEN);
                break;
            case 2:
                lv_obj_clear_flag(ui_imgMainSlotSerialDetail2, LV_OBJ_FLAG_HIDDEN);
                break;
            case 3:
                lv_obj_clear_flag(ui_imgMainSlotSerialDetail3, LV_OBJ_FLAG_HIDDEN);
                break;
            case 4:
                lv_obj_clear_flag(ui_imgMainSlotSerialDetail4, LV_OBJ_FLAG_HIDDEN);
                break;
            case 5:
                lv_obj_clear_flag(ui_imgMainSlotSerialDetail5, LV_OBJ_FLAG_HIDDEN);
                break;
            case 0:
            default:
                // Tất cả đã bị tắt ở trên rồi
                break;
        }
        
        ui_unlock();
    }
}

void ui_show_slot_detail_panel(bool show)
{
    if (ui_lock(-1)) {
        if (show) {
            lv_obj_clear_flag(ui_imgMainSlotDetalBg, LV_OBJ_FLAG_HIDDEN);
            
            // Đảm bảo background opacity = 100%
            lv_obj_set_style_bg_opa(ui_imgMainSlotDetalBg, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
            
            // Nếu có background image
            lv_obj_set_style_bg_img_opa(ui_imgMainSlotDetalBg, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
        } else {
            lv_obj_add_flag(ui_imgMainSlotDetalBg, LV_OBJ_FLAG_HIDDEN);
        }
        ui_unlock();
    }
}

void ui_update_bms_state_value(DeviceHSM_t *me, uint8_t slot_index)
{
    if (slot_index >= BMS_BATTERY_NUM) {
        ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
        return;
    }
    uint8_t bms_state = me->bms_data[slot_index].bms_state;
    const char *state_text = "UNKNOWN";
    
    switch (bms_state) {
        case 2:
            state_text = "STANDBY";
            break;
        case 3:
            state_text = "LOAD";
            break;
        case 4:
            state_text = "CHARGE";
            break;
        case 5:
            state_text = "ERROR";
            break;
        default:
            state_text = "UNKNOWN";
            break;
    }
    
    if (ui_lock(-1)) {
        lv_label_set_text(ui_lbMainSlotBMSStateValue, state_text);
        ui_unlock();
    }
}

void ui_update_ctrl_request_value(DeviceHSM_t *me, uint8_t slot_index)
{
    if (slot_index >= BMS_BATTERY_NUM) {
        ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
        return;
    }

    uint8_t ctrl_request = me->bms_data[slot_index].ctrl_request;

    char ctrl_request_str[6];
    snprintf(ctrl_request_str, sizeof(ctrl_request_str), "%d", ctrl_request);

    if (ui_lock(-1)) {
        lv_label_set_text(ui_lbMainSlotCtrlRqValue, ctrl_request_str);
        ui_unlock();
    }    
}

void ui_update_ctrl_response_value(DeviceHSM_t *me, uint8_t slot_index)
{
    if (slot_index >= BMS_BATTERY_NUM) {
        ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
        return;
    }

    uint8_t ctrl_response = me->bms_data[slot_index].ctrl_response;
    
    char ctrl_response_str[6];
    snprintf(ctrl_response_str, sizeof(ctrl_response_str), "%d", ctrl_response);

    if (ui_lock(-1)) {
        lv_label_set_text(ui_lbMainSlotCtrlRpValue, ctrl_response_str);
        ui_unlock();
    }    
}

void ui_update_fet_ctrl_pin_value(DeviceHSM_t *me, uint8_t slot_index)
{
    if (slot_index >= BMS_BATTERY_NUM) {
        ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
        return;
    }

    uint8_t fet_ctrl_pin = me->bms_data[slot_index].fet_ctrl_pin;
    
    char fet_ctrl_pin_str[6];
    snprintf(fet_ctrl_pin_str, sizeof(fet_ctrl_pin_str), "%d", fet_ctrl_pin);

    if (ui_lock(-1)) {
        lv_label_set_text(ui_lbMainSlotFETCtrlPValue, fet_ctrl_pin_str);
        ui_unlock();
    }    
}

void ui_update_fet_status_value(DeviceHSM_t *me, uint8_t slot_index)
{
    if (slot_index >= BMS_BATTERY_NUM) {
        ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
        return;
    }
    uint8_t fet_status = me->bms_data[slot_index].fet_status;
    const char *state_text = "UNKNOWN";
    
    switch (fet_status) {
        case 2:
            state_text = "STANDBY";
            break;
        case 3:
            state_text = "LOAD";
            break;
        case 4:
            state_text = "CHARGE";
            break;
        case 5:
            state_text = "ERROR";
            break;
        default:
            state_text = "UNKNOWN";
            break;
    }
    
    if (ui_lock(-1)) {
        lv_label_set_text(ui_lbMainSlotFETSttValue, state_text);
        ui_unlock();
    }
}
