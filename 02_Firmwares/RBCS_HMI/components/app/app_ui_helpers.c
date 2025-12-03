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
    
    bool chg_on  = (fet_status & 0x01) != 0;
    bool pchg_on = (fet_status & 0x02) != 0;
    bool dsg_on  = (fet_status & 0x04) != 0;
    bool pdsg_on = (fet_status & 0x08) != 0;
    
    const char *status_text = "UNKNOWN";
    
    if (chg_on && !dsg_on) {
        status_text = pchg_on ? "PCHG" : "CHG";
    } 
    else if (!chg_on && dsg_on) {
        status_text = pdsg_on ? "PDSG" : "DSG";
    }
    else if (!chg_on && !dsg_on) {
        status_text = "IDLE";
    }
    else if (chg_on && dsg_on) {
        status_text = "ERROR";
    }
    else {
        status_text = "UNKNOWN";
    }
    
    if (ui_lock(-1)) {
        lv_label_set_text(ui_lbMainSlotFETSttValue, status_text);
        ui_unlock();
    }
}
void ui_update_alarm_bits_value(DeviceHSM_t *me, uint8_t slot_index)
{
    if (slot_index >= BMS_BATTERY_NUM) {
        ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
        return;
    }

    uint8_t alarm_bits = me->bms_data[slot_index].alarm_bits;
    
    char alarm_bits_str[6];
    snprintf(alarm_bits_str, sizeof(alarm_bits_str), "%d", alarm_bits);

    if (ui_lock(-1)) {
        lv_label_set_text(ui_lbMainSlotAlarmBitsValue, alarm_bits_str);
        ui_unlock();
    }    
}

void ui_update_faults_value(DeviceHSM_t *me, uint8_t slot_index)
{
    if (slot_index >= BMS_BATTERY_NUM) {
        ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
        return;
    }
    
    uint8_t faults = me->bms_data[slot_index].faults;
    
    bool ocd = (faults & 0x01) != 0;
    bool scd = (faults & 0x02) != 0;
    bool ov  = (faults & 0x04) != 0;
    bool uv  = (faults & 0x08) != 0;
    bool occ = (faults & 0x10) != 0;
    
    const char *status_text = "NONE";
    
    if (faults == 0) {
        status_text = "NONE";
    }
    else if (scd) {
        status_text = "SCD";
    }
    else if (ocd) {
        status_text = "OCD";
    }
    else if (occ) {
        status_text = "OCC";
    }
    else if (ov) {
        status_text = "OV";
    }
    else if (uv) {
        status_text = "UV";
    }
    else {
        status_text = "FAULT";
    }
    
    if (ui_lock(-1)) {
        lv_label_set_text(ui_lbMainSlotFaultValue, status_text);
        ui_unlock();
    }
}

void ui_update_pack_volt_value(DeviceHSM_t *me, uint8_t slot_index)
{
    if (slot_index >= BMS_BATTERY_NUM) {
        ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
        return;
    }
    
    char value_str[16];
    float volt = me->bms_data[slot_index].pack_volt / 1000.0f;
    snprintf(value_str, sizeof(value_str), "%.3fV", volt);
    
    if (ui_lock(-1)) {
        lv_label_set_text(ui_lbMainSlotPackVoltValue, value_str);
        ui_unlock();
    }
}

void ui_update_stack_volt_value(DeviceHSM_t *me, uint8_t slot_index)
{
    if (slot_index >= BMS_BATTERY_NUM) {
        ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
        return;
    }
    
    char value_str[16];
    float volt = me->bms_data[slot_index].stack_volt / 1000.0f;
    snprintf(value_str, sizeof(value_str), "%.3fV", volt);
    
    if (ui_lock(-1)) {
        lv_label_set_text(ui_lbMainSlotStackVoltValue, value_str);
        ui_unlock();
    }
}

void ui_update_pack_cur_value(DeviceHSM_t *me, uint8_t slot_index)
{
    if (slot_index >= BMS_BATTERY_NUM) {
        ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
        return;
    }
    
    char value_str[16];
    float current = me->bms_data[slot_index].pack_current / 1000.0f;
    snprintf(value_str, sizeof(value_str), "%.3fA", current);
    
    if (ui_lock(-1)) {
        lv_label_set_text(ui_lbMainSlotPackCurValue, value_str);
        ui_unlock();
    }
}

void ui_update_ld_volt_value(DeviceHSM_t *me, uint8_t slot_index)
{
    if (slot_index >= BMS_BATTERY_NUM) {
        ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
        return;
    }
    
    char value_str[16];
    float volt = me->bms_data[slot_index].ld_volt / 1000.0f;
    snprintf(value_str, sizeof(value_str), "%.3fV", volt);
    
    if (ui_lock(-1)) {
        lv_label_set_text(ui_lbMainSlotIdVoltValue, value_str);
        ui_unlock();
    }
}

void ui_update_pin_percent_value(DeviceHSM_t *me, uint8_t slot_index)
{
    if (slot_index >= BMS_BATTERY_NUM) {
        ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
        return;
    }
    
    char value_str[8];
    snprintf(value_str, sizeof(value_str), "%d%%", me->bms_data[slot_index].pin_percent);
    
    if (ui_lock(-1)) {
        lv_label_set_text(ui_lbMainSlotPinPercentValue, value_str);
        ui_unlock();
    }
}

void ui_update_tg_percent_value(DeviceHSM_t *me, uint8_t slot_index)
{
    if (slot_index >= BMS_BATTERY_NUM) {
        ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
        return;
    }
    
    char value_str[8];
    snprintf(value_str, sizeof(value_str), "%d%%", me->bms_data[slot_index].percent_target);
    
    if (ui_lock(-1)) {
        lv_label_set_text(ui_lbMainSlotTgPercentValue, value_str);
        ui_unlock();
    }
}

void ui_update_cel_res_value(DeviceHSM_t *me, uint8_t slot_index)
{
    if (slot_index >= BMS_BATTERY_NUM) {
        ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
        return;
    }
    
    char value_str[16];
    snprintf(value_str, sizeof(value_str), "%dmR", me->bms_data[slot_index].cell_resistance);
    
    if (ui_lock(-1)) {
        lv_label_set_text(ui_lbMainSlotCelResValue, value_str);
        ui_unlock();
    }
}

void ui_update_soc_per_value(DeviceHSM_t *me, uint8_t slot_index)
{
    if (slot_index >= BMS_BATTERY_NUM) {
        ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
        return;
    }
    
    char value_str[8];
    snprintf(value_str, sizeof(value_str), "%d%%", me->bms_data[slot_index].soc_percent);
    
    if (ui_lock(-1)) {
        lv_label_set_text(ui_lbMainSlotSocPerValue, value_str);
        ui_unlock();
    }
}

void ui_update_soh_value(DeviceHSM_t *me, uint8_t slot_index)
{
    if (slot_index >= BMS_BATTERY_NUM) {
        ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
        return;
    }
    
    char value_str[16];
    snprintf(value_str, sizeof(value_str), "%d", me->bms_data[slot_index].soh_value);
    
    if (ui_lock(-1)) {
        lv_label_set_text(ui_lbMainSlotSOHValue, value_str);
        ui_unlock();
    }
}

void ui_update_sin_par_value(DeviceHSM_t *me, uint8_t slot_index)
{
    if (slot_index >= BMS_BATTERY_NUM) {
        ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
        return;
    }
    
    const char *status_text = me->bms_data[slot_index].single_parallel == 0 ? "SINGLE" : "PARALLEL";
    
    if (ui_lock(-1)) {
        lv_label_set_text(ui_lbMainSlotSinParValue, status_text);
        ui_unlock();
    }
}

void ui_update_temp1_value(DeviceHSM_t *me, uint8_t slot_index)
{
    if (slot_index >= BMS_BATTERY_NUM) {
        ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
        return;
    }
    
    char value_str[16];
    float temp = me->bms_data[slot_index].temp1 / 10.0f;
    snprintf(value_str, sizeof(value_str), "%.1f°C", temp);
    
    if (ui_lock(-1)) {
        lv_label_set_text(ui_lbMainSlotTemp1Value, value_str);
        ui_unlock();
    }
}

void ui_update_temp2_value(DeviceHSM_t *me, uint8_t slot_index)
{
    if (slot_index >= BMS_BATTERY_NUM) {
        ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
        return;
    }
    
    char value_str[16];
    float temp = me->bms_data[slot_index].temp2 / 10.0f;
    snprintf(value_str, sizeof(value_str), "%.1f°C", temp);
    
    if (ui_lock(-1)) {
        lv_label_set_text(ui_lbMainSlotTemp2Value, value_str);
        ui_unlock();
    }
}

void ui_update_temp3_value(DeviceHSM_t *me, uint8_t slot_index)
{
    if (slot_index >= BMS_BATTERY_NUM) {
        ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
        return;
    }
    
    char value_str[16];
    float temp = me->bms_data[slot_index].temp3 / 10.0f;
    snprintf(value_str, sizeof(value_str), "%.1f°C", temp);
    
    if (ui_lock(-1)) {
        lv_label_set_text(ui_lbMainSlotTemp3Value, value_str);
        ui_unlock();
    }
}

void ui_update_cell_voltages(DeviceHSM_t *me, uint8_t slot_index)
{
    if (slot_index >= BMS_BATTERY_NUM) {
        ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
        return;
    }
    
    lv_obj_t *cell_labels[] = {
        ui_lbMainSlotCell1Value,
        ui_lbMainSlotCell2Value,
        ui_lbMainSlotCell3Value,
        ui_lbMainSlotCell4Value,
        ui_lbMainSlotCell5Value,
        ui_lbMainSlotCell6Value,
        ui_lbMainSlotCell7Value,
        ui_lbMainSlotCell8Value,
        ui_lbMainSlotCell9Value,
        ui_lbMainSlotCell10Value,
        ui_lbMainSlotCell11Value,
        ui_lbMainSlotCell12Value,
        ui_lbMainSlotCell13Value
    };
    
    char value_str[16];
    
    if (ui_lock(-1)) {
        for (uint8_t i = 0; i < 13; i++) {
            snprintf(value_str, sizeof(value_str), "%dmV", me->bms_data[slot_index].cell_volt[i]);
            lv_label_set_text(cell_labels[i], value_str);
        }
        ui_unlock();
    }
}

void ui_update_accu_int_value(DeviceHSM_t *me, uint8_t slot_index)
{
    if (slot_index >= BMS_BATTERY_NUM) {
        ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
        return;
    }
    
    char value_str[16];
    snprintf(value_str, sizeof(value_str), "%lu", me->bms_data[slot_index].accu_int);
    
    if (ui_lock(-1)) {
        lv_label_set_text(ui_lbMainSlotACCUIntValue, value_str);
        ui_unlock();
    }
}

void ui_update_accu_frac_value(DeviceHSM_t *me, uint8_t slot_index)
{
    if (slot_index >= BMS_BATTERY_NUM) {
        ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
        return;
    }
    
    char value_str[16];
    snprintf(value_str, sizeof(value_str), "%lu", me->bms_data[slot_index].accu_frac);
    
    if (ui_lock(-1)) {
        lv_label_set_text(ui_lbMainSlotACCUFracValue, value_str);
        ui_unlock();
    }
}

void ui_update_accu_time_value(DeviceHSM_t *me, uint8_t slot_index)
{
    if (slot_index >= BMS_BATTERY_NUM) {
        ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
        return;
    }
    
    char value_str[16];
    snprintf(value_str, sizeof(value_str), "%lu", me->bms_data[slot_index].accu_time);
    
    if (ui_lock(-1)) {
        lv_label_set_text(ui_lbMainSlotACCUTimeValue, value_str);
        ui_unlock();
    }
}

void ui_update_safety_a_value(DeviceHSM_t *me, uint8_t slot_index)
{
    if (slot_index >= BMS_BATTERY_NUM) {
        ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
        return;
    }
    
    char value_str[16];
    snprintf(value_str, sizeof(value_str), "0x%04X", me->bms_data[slot_index].safety_a);
    
    if (ui_lock(-1)) {
        lv_label_set_text(ui_lbMainSlotSafetyAValue, value_str);
        ui_unlock();
    }
}

void ui_update_safety_b_value(DeviceHSM_t *me, uint8_t slot_index)
{
    if (slot_index >= BMS_BATTERY_NUM) {
        ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
        return;
    }
    
    char value_str[16];
    snprintf(value_str, sizeof(value_str), "0x%04X", me->bms_data[slot_index].safety_b);
    
    if (ui_lock(-1)) {
        lv_label_set_text(ui_lbMainSlotSafetyBValue, value_str);
        ui_unlock();
    }
}

void ui_update_safety_c_value(DeviceHSM_t *me, uint8_t slot_index)
{
    if (slot_index >= BMS_BATTERY_NUM) {
        ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
        return;
    }
    
    char value_str[16];
    snprintf(value_str, sizeof(value_str), "0x%04X", me->bms_data[slot_index].safety_c);
    
    if (ui_lock(-1)) {
        lv_label_set_text(ui_lbMainSlotSafetyCValue, value_str);
        ui_unlock();
    }
}

void ui_update_all_slot_details(DeviceHSM_t *me, uint8_t slot_index)
{
    if (slot_index >= BMS_BATTERY_NUM) {
        ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
        return;
    }
    
    ui_update_bms_state_value(me, slot_index);
    ui_update_ctrl_request_value(me, slot_index);
    ui_update_ctrl_response_value(me, slot_index);
    ui_update_fet_ctrl_pin_value(me, slot_index);
    ui_update_fet_status_value(me, slot_index);
    ui_update_alarm_bits_value(me, slot_index);
    ui_update_faults_value(me, slot_index);
    ui_update_pack_volt_value(me, slot_index);
    ui_update_stack_volt_value(me, slot_index);
    ui_update_pack_cur_value(me, slot_index);
    ui_update_ld_volt_value(me, slot_index);
    ui_update_pin_percent_value(me, slot_index);
    ui_update_tg_percent_value(me, slot_index);
    ui_update_cel_res_value(me, slot_index);
    ui_update_soc_per_value(me, slot_index);
    ui_update_soh_value(me, slot_index);
    ui_update_sin_par_value(me, slot_index);
    ui_update_temp1_value(me, slot_index);
    ui_update_temp2_value(me, slot_index);
    ui_update_temp3_value(me, slot_index);
    ui_update_cell_voltages(me, slot_index);
    ui_update_accu_int_value(me, slot_index);
    ui_update_accu_frac_value(me, slot_index);
    ui_update_accu_time_value(me, slot_index);
    ui_update_safety_a_value(me, slot_index);
    ui_update_safety_b_value(me, slot_index);
    ui_update_safety_c_value(me, slot_index);
}
