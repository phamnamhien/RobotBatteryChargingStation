#include "app_states.h"

static const char *TAG = "UI_HELPERS";

/*--------------------------------------------------------------------*/
/* OPTIMIZED BATCH UPDATE - SINGLE LOCK */
/*--------------------------------------------------------------------*/

void ui_update_all_slot_details(DeviceHSM_t *me, uint8_t slot_index)
{
    if (slot_index >= TOTAL_SLOT) {
        ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
        return;
    }
    
    if (!ui_lock(-1)) {
        ESP_LOGE(TAG, "Failed to lock UI");
        return;
    }
    
    // ✅ CHỈ LOCK 1 LẦN CHO TẤT CẢ UPDATE
    BMS_Data_t *bms = &me->bms_data[slot_index];
    char buf[32];
    
    // --- BMS State ---
    const char *state_text = "UNKNOWN";
    switch (bms->bms_state) {
        case 2: state_text = "STANDBY"; break;
        case 3: state_text = "LOAD"; break;
        case 4: state_text = "CHARGE"; break;
        case 5: state_text = "ERROR"; break;
    }
    lv_label_set_text(ui_lbMainSlotBMSStateValue, state_text);
    
    // --- Control Values ---
    snprintf(buf, sizeof(buf), "%d", bms->ctrl_request);
    lv_label_set_text(ui_lbMainSlotCtrlRqValue, buf);
    
    snprintf(buf, sizeof(buf), "%d", bms->ctrl_response);
    lv_label_set_text(ui_lbMainSlotCtrlRpValue, buf);
    
    snprintf(buf, sizeof(buf), "%d", bms->fet_ctrl_pin);
    lv_label_set_text(ui_lbMainSlotFETCtrlPValue, buf);
    
    // --- FET Status ---
    uint8_t fet = bms->fet_status;
    bool chg_on = (fet & 0x01), pchg_on = (fet & 0x02);
    bool dsg_on = (fet & 0x04), pdsg_on = (fet & 0x08);
    
    const char *fet_text = (!chg_on && !dsg_on) ? "IDLE" :
                          (chg_on && !dsg_on) ? (pchg_on ? "PCHG" : "CHG") :
                          (!chg_on && dsg_on) ? (pdsg_on ? "PDSG" : "DSG") : "ERROR";
    lv_label_set_text(ui_lbMainSlotFETSttValue, fet_text);
    
    // --- Alarm & Faults ---
    snprintf(buf, sizeof(buf), "%d", bms->alarm_bits);
    lv_label_set_text(ui_lbMainSlotAlarmBitsValue, buf);
    
    uint8_t faults = bms->faults;
    const char *fault_text = (faults == 0) ? "NONE" :
                            (faults & 0x02) ? "SCD" :
                            (faults & 0x01) ? "OCD" :
                            (faults & 0x10) ? "OCC" :
                            (faults & 0x04) ? "OV" :
                            (faults & 0x08) ? "UV" : "FAULT";
    lv_label_set_text(ui_lbMainSlotFaultValue, fault_text);
    
    // --- Voltages ---
    snprintf(buf, sizeof(buf), "%.3fV", bms->pack_volt / 1000.0f);
    lv_label_set_text(ui_lbMainSlotPackVoltValue, buf);
    
    snprintf(buf, sizeof(buf), "%.3fV", bms->stack_volt / 1000.0f);
    lv_label_set_text(ui_lbMainSlotStackVoltValue, buf);
    
    snprintf(buf, sizeof(buf), "%.3fV", bms->ld_volt / 1000.0f);
    lv_label_set_text(ui_lbMainSlotIdVoltValue, buf);
    
    // --- Current ---
    snprintf(buf, sizeof(buf), "%.3fA", bms->pack_current / 1000.0f);
    lv_label_set_text(ui_lbMainSlotPackCurValue, buf);
    
    // --- Percentages ---
    snprintf(buf, sizeof(buf), "%d%%", bms->pin_percent);
    lv_label_set_text(ui_lbMainSlotPinPercentValue, buf);
    
    snprintf(buf, sizeof(buf), "%d%%", bms->percent_target);
    lv_label_set_text(ui_lbMainSlotTgPercentValue, buf);
    
    snprintf(buf, sizeof(buf), "%d%%", bms->soc_percent);
    lv_label_set_text(ui_lbMainSlotSocPerValue, buf);
    
    // --- Battery Health ---
    snprintf(buf, sizeof(buf), "%dmR", bms->cell_resistance);
    lv_label_set_text(ui_lbMainSlotCelResValue, buf);
    
    snprintf(buf, sizeof(buf), "%d", bms->soh_value);
    lv_label_set_text(ui_lbMainSlotSOHValue, buf);
    
    lv_label_set_text(ui_lbMainSlotSinParValue, 
                     bms->single_parallel == 0 ? "SINGLE" : "PARALLEL");
    
    // --- Temperatures ---
    snprintf(buf, sizeof(buf), "%.1f°C", bms->temp1 / 10.0f);
    lv_label_set_text(ui_lbMainSlotTemp1Value, buf);
    
    snprintf(buf, sizeof(buf), "%.1f°C", bms->temp2 / 10.0f);
    lv_label_set_text(ui_lbMainSlotTemp2Value, buf);
    
    snprintf(buf, sizeof(buf), "%.1f°C", bms->temp3 / 10.0f);
    lv_label_set_text(ui_lbMainSlotTemp3Value, buf);
    
    // --- Cell Voltages (13 cells) ---
    lv_obj_t *cells[] = {
        ui_lbMainSlotCell1Value,  ui_lbMainSlotCell2Value,  ui_lbMainSlotCell3Value,
        ui_lbMainSlotCell4Value,  ui_lbMainSlotCell5Value,  ui_lbMainSlotCell6Value,
        ui_lbMainSlotCell7Value,  ui_lbMainSlotCell8Value,  ui_lbMainSlotCell9Value,
        ui_lbMainSlotCell10Value, ui_lbMainSlotCell11Value, ui_lbMainSlotCell12Value,
        ui_lbMainSlotCell13Value
    };
    for (uint8_t i = 0; i < 13; i++) {
        snprintf(buf, sizeof(buf), "%dmV", bms->cell_volt[i]);
        lv_label_set_text(cells[i], buf);
    }
    
    // --- Accumulator Values ---
    snprintf(buf, sizeof(buf), "%lu", bms->accu_int);
    lv_label_set_text(ui_lbMainSlotACCUIntValue, buf);
    
    snprintf(buf, sizeof(buf), "%lu", bms->accu_frac);
    lv_label_set_text(ui_lbMainSlotACCUFracValue, buf);
    
    snprintf(buf, sizeof(buf), "%lu", bms->accu_time);
    lv_label_set_text(ui_lbMainSlotACCUTimeValue, buf);
    
    // --- Safety Status ---
    snprintf(buf, sizeof(buf), "0x%04X", bms->safety_a);
    lv_label_set_text(ui_lbMainSlotSafetyAValue, buf);
    
    snprintf(buf, sizeof(buf), "0x%04X", bms->safety_b);
    lv_label_set_text(ui_lbMainSlotSafetyBValue, buf);
    
    snprintf(buf, sizeof(buf), "0x%04X", bms->safety_c);
    lv_label_set_text(ui_lbMainSlotSafetyCValue, buf);
    
    ui_unlock();  // ✅ CHỈ UNLOCK 1 LẦN
}

void ui_clear_all_slot_details(void)
{
    if (!ui_lock(-1)) {
        ESP_LOGE(TAG, "Failed to lock UI");
        return;
    }
    
    // ✅ CHỈ LOCK 1 LẦN CHO TẤT CẢ CLEAR
    const char *no_data = "-";
    
    // --- BMS State & Control ---
    lv_label_set_text(ui_lbMainSlotBMSStateValue, no_data);
    lv_label_set_text(ui_lbMainSlotCtrlRqValue, no_data);
    lv_label_set_text(ui_lbMainSlotCtrlRpValue, no_data);
    lv_label_set_text(ui_lbMainSlotFETCtrlPValue, no_data);
    lv_label_set_text(ui_lbMainSlotFETSttValue, no_data);
    lv_label_set_text(ui_lbMainSlotAlarmBitsValue, no_data);
    lv_label_set_text(ui_lbMainSlotFaultValue, no_data);
    
    // --- Voltages ---
    lv_label_set_text(ui_lbMainSlotPackVoltValue, no_data);
    lv_label_set_text(ui_lbMainSlotStackVoltValue, no_data);
    lv_label_set_text(ui_lbMainSlotIdVoltValue, no_data);
    
    // --- Current ---
    lv_label_set_text(ui_lbMainSlotPackCurValue, no_data);
    
    // --- Percentages ---
    lv_label_set_text(ui_lbMainSlotPinPercentValue, no_data);
    lv_label_set_text(ui_lbMainSlotTgPercentValue, no_data);
    lv_label_set_text(ui_lbMainSlotSocPerValue, no_data);
    
    // --- Battery Health ---
    lv_label_set_text(ui_lbMainSlotCelResValue, no_data);
    lv_label_set_text(ui_lbMainSlotSOHValue, no_data);
    lv_label_set_text(ui_lbMainSlotSinParValue, no_data);
    
    // --- Temperatures ---
    lv_label_set_text(ui_lbMainSlotTemp1Value, no_data);
    lv_label_set_text(ui_lbMainSlotTemp2Value, no_data);
    lv_label_set_text(ui_lbMainSlotTemp3Value, no_data);
    
    // --- Cell Voltages (13 cells) ---
    lv_obj_t *cells[] = {
        ui_lbMainSlotCell1Value,  ui_lbMainSlotCell2Value,  ui_lbMainSlotCell3Value,
        ui_lbMainSlotCell4Value,  ui_lbMainSlotCell5Value,  ui_lbMainSlotCell6Value,
        ui_lbMainSlotCell7Value,  ui_lbMainSlotCell8Value,  ui_lbMainSlotCell9Value,
        ui_lbMainSlotCell10Value, ui_lbMainSlotCell11Value, ui_lbMainSlotCell12Value,
        ui_lbMainSlotCell13Value
    };
    for (uint8_t i = 0; i < 13; i++) {
        lv_label_set_text(cells[i], no_data);
    }
    
    // --- Accumulator Values ---
    lv_label_set_text(ui_lbMainSlotACCUIntValue, no_data);
    lv_label_set_text(ui_lbMainSlotACCUFracValue, no_data);
    lv_label_set_text(ui_lbMainSlotACCUTimeValue, no_data);
    
    // --- Safety Status ---
    lv_label_set_text(ui_lbMainSlotSafetyAValue, no_data);
    lv_label_set_text(ui_lbMainSlotSafetyBValue, no_data);
    lv_label_set_text(ui_lbMainSlotSafetyCValue, no_data);
    
    ui_unlock();  // ✅ CHỈ UNLOCK 1 LẦN
}

/*--------------------------------------------------------------------*/
/* MAIN SCREEN BATTERY UPDATES */
/*--------------------------------------------------------------------*/

void ui_update_main_slot_voltage(DeviceHSM_t *me, int8_t slot_index)
{
    if (slot_index >= TOTAL_SLOT) return;
    
    // ✅ THÊM LOG
    ESP_LOGI("UI_HELPERS", "📺 Updating UI for slot_index=%d", slot_index);
    ESP_LOGI("UI_HELPERS", "   Reading from bms_data[%d].stack_volt = %d", 
             slot_index, me->bms_data[slot_index].stack_volt);
    
    lv_obj_t *labels[] = {
        ui_lbMainVoltageSlot1, ui_lbMainVoltageSlot2, ui_lbMainVoltageSlot3,
        ui_lbMainVoltageSlot4, ui_lbMainVoltageSlot5
    };
    
    // ✅ IN RA ĐỊA CHỈ LABEL
    ESP_LOGI("UI_HELPERS", "   Writing to UI label[%d] = %p", slot_index, labels[slot_index]);
    
    char buf[16];
    snprintf(buf, sizeof(buf), "%.3fV", me->bms_data[slot_index].stack_volt / 1000.0f);
    
    ESP_LOGI("UI_HELPERS", "   Display text: %s", buf);
    
    if (ui_lock(-1)) {
        lv_label_set_text(labels[slot_index], buf);
        ui_unlock();
    }
}

void ui_update_main_battery_percent(DeviceHSM_t *me, uint8_t slot_index)
{
    if (slot_index >= TOTAL_SLOT) return;
    
    lv_obj_t *bars[] = {
        ui_barMainBatPercent1, ui_barMainBatPercent2, ui_barMainBatPercent3,
        ui_barMainBatPercent4, ui_barMainBatPercent5
    };
    
    lv_obj_t *soc_labels[] = {
        ui_lbMainSlot1SOC, ui_lbMainSlot2SOC, ui_lbMainSlot3SOC,
        ui_lbMainSlot4SOC, ui_lbMainSlot5SOC
    };
    
    uint8_t soc = me->bms_data[slot_index].soc_percent;
    if (soc > 100) soc = 100;
    
    char buf[8];
    snprintf(buf, sizeof(buf), "%d%%", soc);
    
    if (ui_lock(-1)) {
        lv_bar_set_value(bars[slot_index], soc, LV_ANIM_OFF);
        lv_label_set_text(soc_labels[slot_index], buf);
        ui_unlock();
    }
}

void ui_update_main_slot_capacity(DeviceHSM_t *me, int8_t slot_index)
{
    if (slot_index >= TOTAL_SLOT) return;
    
    lv_obj_t *labels[] = {
        ui_lbMainCapSlot1, ui_lbMainCapSlot2, ui_lbMainCapSlot3,
        ui_lbMainCapSlot4, ui_lbMainCapSlot5
    };
    
    char buf[16];
    snprintf(buf, sizeof(buf), "%dmAh", me->bms_data[slot_index].capacity);
    
    if (ui_lock(-1)) {
        lv_label_set_text(labels[slot_index], buf);
        ui_unlock();
    }
}

/*--------------------------------------------------------------------*/
/* SLOT DETAIL PANEL VISIBILITY */
/*--------------------------------------------------------------------*/

void ui_show_slot_serial_detail(uint8_t slot_num)
{
    static uint8_t last_slot = 0;
    if (slot_num == last_slot) return;
    
    if (!ui_lock(-1)) return;
    
    lv_obj_t *slots[] = {
        ui_imgMainSlotSerialDetail1, ui_imgMainSlotSerialDetail2,
        ui_imgMainSlotSerialDetail3, ui_imgMainSlotSerialDetail4,
        ui_imgMainSlotSerialDetail5
    };
    
    // Hide old slot
    if (last_slot > 0 && last_slot <= 5) {
        lv_obj_add_flag(slots[last_slot - 1], LV_OBJ_FLAG_HIDDEN);
    }
    
    // Show new slot
    if (slot_num > 0 && slot_num <= 5) {
        lv_obj_clear_flag(slots[slot_num - 1], LV_OBJ_FLAG_HIDDEN);
    }
    
    last_slot = slot_num;
    ui_unlock();
}

void ui_show_slot_detail_panel(bool show)
{
    static bool last_state = false;
    if (show == last_state) return;
    
    if (!ui_lock(-1)) return;
    
    if (show) {
        lv_obj_clear_flag(ui_imgMainSlotDetalBg, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(ui_imgMainSlotDetalBg, LV_OBJ_FLAG_HIDDEN);
    }
    
    last_state = show;
    ui_unlock();
}





void ui_show_main_not_connect(bool show)
{
    static bool last_state = false;
    if (show == last_state) return;  // Skip nếu không đổi
    
    if (ui_lock(-1)) {
        if (show) {
            lv_obj_clear_flag(ui_imgMainNotConnect, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(ui_imgMainNotConnect, LV_OBJ_FLAG_HIDDEN);
        }
        last_state = show;
        ui_unlock();
    }
}






















// #include "app_states.h"

// static const char *TAG = "UI_HELPERS";

// /*--------------------------------------------------------------------*/
// /* BATTERY FUNCTION HELPERS */
// /*--------------------------------------------------------------------*/
// void ui_update_main_slot_voltage(DeviceHSM_t *me, int8_t slot_index)
// {
//     if (slot_index >= BMS_BATTERY_NUM) {
//         ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
//         return;
//     }
    
//     // Array chứa các label voltage
//     lv_obj_t *voltage_values[] = {
//         ui_lbMainVoltageSlot1,
//         ui_lbMainVoltageSlot2,
//         ui_lbMainVoltageSlot3,
//         ui_lbMainVoltageSlot4,
//         ui_lbMainVoltageSlot5
//     };
    
//     // Lấy giá trị voltage (mV)
//     uint16_t stack_volt_mv = me->bms_data[slot_index].stack_volt;
    
//     // Chuyển sang V và format
//     char voltage_str[16];
//     snprintf(voltage_str, sizeof(voltage_str), "%.3fV", stack_volt_mv / 1000.0f);
    
//     if (ui_lock(-1)) {
//         lv_label_set_text(voltage_values[slot_index], voltage_str);
//         ui_unlock();
//     }
// }

// void ui_update_main_battery_percent(DeviceHSM_t *me, uint8_t slot_index)
// {
//     if (slot_index >= BMS_BATTERY_NUM) {
//         ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
//         return;
//     }
    
//     // Array chứa các bar và label
//     lv_obj_t *battery_bars[] = {
//         ui_barMainBatPercent1,
//         ui_barMainBatPercent2,
//         ui_barMainBatPercent3,
//         ui_barMainBatPercent4,
//         ui_barMainBatPercent5
//     };
    
//     lv_obj_t *soc_values[] = {
//         ui_lbMainSlot1SOC,
//         ui_lbMainSlot2SOC,
//         ui_lbMainSlot3SOC,
//         ui_lbMainSlot4SOC,
//         ui_lbMainSlot5SOC
//     };
    
//     // Lấy SOC percent
//     uint8_t soc = me->bms_data[slot_index].soc_percent;
    
//     // Giới hạn 0-100%
//     if (soc > 100) {
//         soc = 100;
//     }
    
//     // Format label text
//     char soc_str[8];
//     snprintf(soc_str, sizeof(soc_str), "%d%%", soc);
    
//     if (ui_lock(-1)) {
//         // Cập nhật bar value
//         lv_bar_set_value(battery_bars[slot_index], soc, LV_ANIM_OFF);
        
//         // Cập nhật label text
//         lv_label_set_text(soc_values[slot_index], soc_str);
        
//         ui_unlock();
//     }
// }

// void ui_update_main_slot_capacity(DeviceHSM_t *me, int8_t slot_index)
// {
//     if (slot_index >= BMS_BATTERY_NUM) {
//         ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
//         return;
//     }
    
//     // Array chứa các label capacity
//     lv_obj_t *capacity_values[] = {
//         ui_lbMainCapSlot1,
//         ui_lbMainCapSlot2,
//         ui_lbMainCapSlot3,
//         ui_lbMainCapSlot4,
//         ui_lbMainCapSlot5
//     };
    
//     // Lấy giá trị cap (mAh)
//     uint16_t stack_capacity_mah = me->bms_data[slot_index].capacity;
    
//     // Format
//     char capacity_str[16];
//     snprintf(capacity_str, sizeof(capacity_str), "%dmAh", stack_capacity_mah);
    
//     if (ui_lock(-1)) {
//         lv_label_set_text(capacity_values[slot_index], capacity_str);
//         ui_unlock();
//     }
// }

// /*--------------------------------------------------------------------*/
// /* BATTERY DETAILS PANEL */
// /*--------------------------------------------------------------------*/
// // void ui_show_slot_serial_detail(uint8_t slot_num)
// // {
// //     if (ui_lock(-1)) {
// //         // Tắt tất cả trước
// //         lv_obj_add_flag(ui_imgMainSlotSerialDetail1, LV_OBJ_FLAG_HIDDEN);
// //         lv_obj_add_flag(ui_imgMainSlotSerialDetail2, LV_OBJ_FLAG_HIDDEN);
// //         lv_obj_add_flag(ui_imgMainSlotSerialDetail3, LV_OBJ_FLAG_HIDDEN);
// //         lv_obj_add_flag(ui_imgMainSlotSerialDetail4, LV_OBJ_FLAG_HIDDEN);
// //         lv_obj_add_flag(ui_imgMainSlotSerialDetail5, LV_OBJ_FLAG_HIDDEN);
        
// //         // Bật slot được chọn (nếu slot_num != 0)
// //         switch (slot_num) {
// //             case 1:
// //                 lv_obj_clear_flag(ui_imgMainSlotSerialDetail1, LV_OBJ_FLAG_HIDDEN);
// //                 break;
// //             case 2:
// //                 lv_obj_clear_flag(ui_imgMainSlotSerialDetail2, LV_OBJ_FLAG_HIDDEN);
// //                 break;
// //             case 3:
// //                 lv_obj_clear_flag(ui_imgMainSlotSerialDetail3, LV_OBJ_FLAG_HIDDEN);
// //                 break;
// //             case 4:
// //                 lv_obj_clear_flag(ui_imgMainSlotSerialDetail4, LV_OBJ_FLAG_HIDDEN);
// //                 break;
// //             case 5:
// //                 lv_obj_clear_flag(ui_imgMainSlotSerialDetail5, LV_OBJ_FLAG_HIDDEN);
// //                 break;
// //             case 0:
// //             default:
// //                 // Tất cả đã bị tắt ở trên rồi
// //                 break;
// //         }
        
// //         ui_unlock();
// //     }
// // }
// void ui_show_slot_serial_detail(uint8_t slot_num) 
// { 
//     static uint8_t last_slot = 0;  // Track slot trước đó
    
//     if (slot_num == last_slot) return;  // Không làm gì nếu không đổi
    
//     if (ui_lock(-1)) { 
//         lv_obj_t *slot_details[] = {
//             ui_imgMainSlotSerialDetail1,
//             ui_imgMainSlotSerialDetail2,
//             ui_imgMainSlotSerialDetail3,
//             ui_imgMainSlotSerialDetail4,
//             ui_imgMainSlotSerialDetail5
//         };
        
//         // Ẩn slot cũ
//         if (last_slot > 0 && last_slot <= 5) {
//             lv_obj_add_flag(slot_details[last_slot - 1], LV_OBJ_FLAG_HIDDEN);
//         }
        
//         // Hiện slot mới
//         if (slot_num > 0 && slot_num <= 5) {
//             lv_obj_clear_flag(slot_details[slot_num - 1], LV_OBJ_FLAG_HIDDEN);
//         }
        
//         last_slot = slot_num;
//         ui_unlock(); 
//     } 
// }

// void ui_show_slot_detail_panel(bool show)
// {
//     static bool last_state = false;
//     if (show == last_state) return;  // Skip nếu không đổi
    
//     if (ui_lock(-1)) {
//         if (show) {
//             lv_obj_clear_flag(ui_imgMainSlotDetalBg, LV_OBJ_FLAG_HIDDEN);
//         } else {
//             lv_obj_add_flag(ui_imgMainSlotDetalBg, LV_OBJ_FLAG_HIDDEN);
//         }
//         last_state = show;
//         ui_unlock();
//     }
// }

// void ui_update_bms_state_value(DeviceHSM_t *me, uint8_t slot_index)
// {
//     if (slot_index >= BMS_BATTERY_NUM) {
//         ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
//         return;
//     }
//     uint8_t bms_state = me->bms_data[slot_index].bms_state;
//     const char *state_text = "UNKNOWN";
    
//     switch (bms_state) {
//         case 2:
//             state_text = "STANDBY";
//             break;
//         case 3:
//             state_text = "LOAD";
//             break;
//         case 4:
//             state_text = "CHARGE";
//             break;
//         case 5:
//             state_text = "ERROR";
//             break;
//         default:
//             state_text = "UNKNOWN";
//             break;
//     }
    
//     if (ui_lock(-1)) {
//         lv_label_set_text(ui_lbMainSlotBMSStateValue, state_text);
//         ui_unlock();
//     }
// }

// void ui_update_ctrl_request_value(DeviceHSM_t *me, uint8_t slot_index)
// {
//     if (slot_index >= BMS_BATTERY_NUM) {
//         ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
//         return;
//     }

//     uint8_t ctrl_request = me->bms_data[slot_index].ctrl_request;

//     char ctrl_request_str[6];
//     snprintf(ctrl_request_str, sizeof(ctrl_request_str), "%d", ctrl_request);

//     if (ui_lock(-1)) {
//         lv_label_set_text(ui_lbMainSlotCtrlRqValue, ctrl_request_str);
//         ui_unlock();
//     }    
// }

// void ui_update_ctrl_response_value(DeviceHSM_t *me, uint8_t slot_index)
// {
//     if (slot_index >= BMS_BATTERY_NUM) {
//         ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
//         return;
//     }

//     uint8_t ctrl_response = me->bms_data[slot_index].ctrl_response;
    
//     char ctrl_response_str[6];
//     snprintf(ctrl_response_str, sizeof(ctrl_response_str), "%d", ctrl_response);

//     if (ui_lock(-1)) {
//         lv_label_set_text(ui_lbMainSlotCtrlRpValue, ctrl_response_str);
//         ui_unlock();
//     }    
// }

// void ui_update_fet_ctrl_pin_value(DeviceHSM_t *me, uint8_t slot_index)
// {
//     if (slot_index >= BMS_BATTERY_NUM) {
//         ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
//         return;
//     }

//     uint8_t fet_ctrl_pin = me->bms_data[slot_index].fet_ctrl_pin;
    
//     char fet_ctrl_pin_str[6];
//     snprintf(fet_ctrl_pin_str, sizeof(fet_ctrl_pin_str), "%d", fet_ctrl_pin);

//     if (ui_lock(-1)) {
//         lv_label_set_text(ui_lbMainSlotFETCtrlPValue, fet_ctrl_pin_str);
//         ui_unlock();
//     }    
// }

// void ui_update_fet_status_value(DeviceHSM_t *me, uint8_t slot_index)
// {
//     if (slot_index >= BMS_BATTERY_NUM) {
//         ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
//         return;
//     }
    
//     uint8_t fet_status = me->bms_data[slot_index].fet_status;
    
//     bool chg_on  = (fet_status & 0x01) != 0;
//     bool pchg_on = (fet_status & 0x02) != 0;
//     bool dsg_on  = (fet_status & 0x04) != 0;
//     bool pdsg_on = (fet_status & 0x08) != 0;
    
//     const char *status_text = "UNKNOWN";
    
//     if (chg_on && !dsg_on) {
//         status_text = pchg_on ? "PCHG" : "CHG";
//     } 
//     else if (!chg_on && dsg_on) {
//         status_text = pdsg_on ? "PDSG" : "DSG";
//     }
//     else if (!chg_on && !dsg_on) {
//         status_text = "IDLE";
//     }
//     else if (chg_on && dsg_on) {
//         status_text = "ERROR";
//     }
//     else {
//         status_text = "UNKNOWN";
//     }
    
//     if (ui_lock(-1)) {
//         lv_label_set_text(ui_lbMainSlotFETSttValue, status_text);
//         ui_unlock();
//     }
// }
// void ui_update_alarm_bits_value(DeviceHSM_t *me, uint8_t slot_index)
// {
//     if (slot_index >= BMS_BATTERY_NUM) {
//         ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
//         return;
//     }

//     uint8_t alarm_bits = me->bms_data[slot_index].alarm_bits;
    
//     char alarm_bits_str[6];
//     snprintf(alarm_bits_str, sizeof(alarm_bits_str), "%d", alarm_bits);

//     if (ui_lock(-1)) {
//         lv_label_set_text(ui_lbMainSlotAlarmBitsValue, alarm_bits_str);
//         ui_unlock();
//     }    
// }

// void ui_update_faults_value(DeviceHSM_t *me, uint8_t slot_index)
// {
//     if (slot_index >= BMS_BATTERY_NUM) {
//         ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
//         return;
//     }
    
//     uint8_t faults = me->bms_data[slot_index].faults;
    
//     bool ocd = (faults & 0x01) != 0;
//     bool scd = (faults & 0x02) != 0;
//     bool ov  = (faults & 0x04) != 0;
//     bool uv  = (faults & 0x08) != 0;
//     bool occ = (faults & 0x10) != 0;
    
//     const char *status_text = "NONE";
    
//     if (faults == 0) {
//         status_text = "NONE";
//     }
//     else if (scd) {
//         status_text = "SCD";
//     }
//     else if (ocd) {
//         status_text = "OCD";
//     }
//     else if (occ) {
//         status_text = "OCC";
//     }
//     else if (ov) {
//         status_text = "OV";
//     }
//     else if (uv) {
//         status_text = "UV";
//     }
//     else {
//         status_text = "FAULT";
//     }
    
//     if (ui_lock(-1)) {
//         lv_label_set_text(ui_lbMainSlotFaultValue, status_text);
//         ui_unlock();
//     }
// }

// void ui_update_pack_volt_value(DeviceHSM_t *me, uint8_t slot_index)
// {
//     if (slot_index >= BMS_BATTERY_NUM) {
//         ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
//         return;
//     }
    
//     char value_str[16];
//     float volt = me->bms_data[slot_index].pack_volt / 1000.0f;
//     snprintf(value_str, sizeof(value_str), "%.3fV", volt);
    
//     if (ui_lock(-1)) {
//         lv_label_set_text(ui_lbMainSlotPackVoltValue, value_str);
//         ui_unlock();
//     }
// }

// void ui_update_stack_volt_value(DeviceHSM_t *me, uint8_t slot_index)
// {
//     if (slot_index >= BMS_BATTERY_NUM) {
//         ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
//         return;
//     }
    
//     char value_str[16];
//     float volt = me->bms_data[slot_index].stack_volt / 1000.0f;
//     snprintf(value_str, sizeof(value_str), "%.3fV", volt);
    
//     if (ui_lock(-1)) {
//         lv_label_set_text(ui_lbMainSlotStackVoltValue, value_str);
//         ui_unlock();
//     }
// }

// void ui_update_pack_cur_value(DeviceHSM_t *me, uint8_t slot_index)
// {
//     if (slot_index >= BMS_BATTERY_NUM) {
//         ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
//         return;
//     }
    
//     char value_str[16];
//     float current = me->bms_data[slot_index].pack_current / 1000.0f;
//     snprintf(value_str, sizeof(value_str), "%.3fA", current);
    
//     if (ui_lock(-1)) {
//         lv_label_set_text(ui_lbMainSlotPackCurValue, value_str);
//         ui_unlock();
//     }
// }

// void ui_update_ld_volt_value(DeviceHSM_t *me, uint8_t slot_index)
// {
//     if (slot_index >= BMS_BATTERY_NUM) {
//         ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
//         return;
//     }
    
//     char value_str[16];
//     float volt = me->bms_data[slot_index].ld_volt / 1000.0f;
//     snprintf(value_str, sizeof(value_str), "%.3fV", volt);
    
//     if (ui_lock(-1)) {
//         lv_label_set_text(ui_lbMainSlotIdVoltValue, value_str);
//         ui_unlock();
//     }
// }

// void ui_update_pin_percent_value(DeviceHSM_t *me, uint8_t slot_index)
// {
//     if (slot_index >= BMS_BATTERY_NUM) {
//         ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
//         return;
//     }
    
//     char value_str[8];
//     snprintf(value_str, sizeof(value_str), "%d%%", me->bms_data[slot_index].pin_percent);
    
//     if (ui_lock(-1)) {
//         lv_label_set_text(ui_lbMainSlotPinPercentValue, value_str);
//         ui_unlock();
//     }
// }

// void ui_update_tg_percent_value(DeviceHSM_t *me, uint8_t slot_index)
// {
//     if (slot_index >= BMS_BATTERY_NUM) {
//         ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
//         return;
//     }
    
//     char value_str[8];
//     snprintf(value_str, sizeof(value_str), "%d%%", me->bms_data[slot_index].percent_target);
    
//     if (ui_lock(-1)) {
//         lv_label_set_text(ui_lbMainSlotTgPercentValue, value_str);
//         ui_unlock();
//     }
// }

// void ui_update_cel_res_value(DeviceHSM_t *me, uint8_t slot_index)
// {
//     if (slot_index >= BMS_BATTERY_NUM) {
//         ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
//         return;
//     }
    
//     char value_str[16];
//     snprintf(value_str, sizeof(value_str), "%dmR", me->bms_data[slot_index].cell_resistance);
    
//     if (ui_lock(-1)) {
//         lv_label_set_text(ui_lbMainSlotCelResValue, value_str);
//         ui_unlock();
//     }
// }

// void ui_update_soc_per_value(DeviceHSM_t *me, uint8_t slot_index)
// {
//     if (slot_index >= BMS_BATTERY_NUM) {
//         ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
//         return;
//     }
    
//     char value_str[8];
//     snprintf(value_str, sizeof(value_str), "%d%%", me->bms_data[slot_index].soc_percent);
    
//     if (ui_lock(-1)) {
//         lv_label_set_text(ui_lbMainSlotSocPerValue, value_str);
//         ui_unlock();
//     }
// }

// void ui_update_soh_value(DeviceHSM_t *me, uint8_t slot_index)
// {
//     if (slot_index >= BMS_BATTERY_NUM) {
//         ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
//         return;
//     }
    
//     char value_str[16];
//     snprintf(value_str, sizeof(value_str), "%d", me->bms_data[slot_index].soh_value);
    
//     if (ui_lock(-1)) {
//         lv_label_set_text(ui_lbMainSlotSOHValue, value_str);
//         ui_unlock();
//     }
// }

// void ui_update_sin_par_value(DeviceHSM_t *me, uint8_t slot_index)
// {
//     if (slot_index >= BMS_BATTERY_NUM) {
//         ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
//         return;
//     }
    
//     const char *status_text = me->bms_data[slot_index].single_parallel == 0 ? "SINGLE" : "PARALLEL";
    
//     if (ui_lock(-1)) {
//         lv_label_set_text(ui_lbMainSlotSinParValue, status_text);
//         ui_unlock();
//     }
// }

// void ui_update_temp1_value(DeviceHSM_t *me, uint8_t slot_index)
// {
//     if (slot_index >= BMS_BATTERY_NUM) {
//         ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
//         return;
//     }
    
//     char value_str[16];
//     float temp = me->bms_data[slot_index].temp1 / 10.0f;
//     snprintf(value_str, sizeof(value_str), "%.1f°C", temp);
    
//     if (ui_lock(-1)) {
//         lv_label_set_text(ui_lbMainSlotTemp1Value, value_str);
//         ui_unlock();
//     }
// }

// void ui_update_temp2_value(DeviceHSM_t *me, uint8_t slot_index)
// {
//     if (slot_index >= BMS_BATTERY_NUM) {
//         ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
//         return;
//     }
    
//     char value_str[16];
//     float temp = me->bms_data[slot_index].temp2 / 10.0f;
//     snprintf(value_str, sizeof(value_str), "%.1f°C", temp);
    
//     if (ui_lock(-1)) {
//         lv_label_set_text(ui_lbMainSlotTemp2Value, value_str);
//         ui_unlock();
//     }
// }

// void ui_update_temp3_value(DeviceHSM_t *me, uint8_t slot_index)
// {
//     if (slot_index >= BMS_BATTERY_NUM) {
//         ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
//         return;
//     }
    
//     char value_str[16];
//     float temp = me->bms_data[slot_index].temp3 / 10.0f;
//     snprintf(value_str, sizeof(value_str), "%.1f°C", temp);
    
//     if (ui_lock(-1)) {
//         lv_label_set_text(ui_lbMainSlotTemp3Value, value_str);
//         ui_unlock();
//     }
// }

// void ui_update_cell_voltages(DeviceHSM_t *me, uint8_t slot_index)
// {
//     if (slot_index >= BMS_BATTERY_NUM) {
//         ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
//         return;
//     }
    
//     lv_obj_t *cell_labels[] = {
//         ui_lbMainSlotCell1Value,
//         ui_lbMainSlotCell2Value,
//         ui_lbMainSlotCell3Value,
//         ui_lbMainSlotCell4Value,
//         ui_lbMainSlotCell5Value,
//         ui_lbMainSlotCell6Value,
//         ui_lbMainSlotCell7Value,
//         ui_lbMainSlotCell8Value,
//         ui_lbMainSlotCell9Value,
//         ui_lbMainSlotCell10Value,
//         ui_lbMainSlotCell11Value,
//         ui_lbMainSlotCell12Value,
//         ui_lbMainSlotCell13Value
//     };
    
//     char value_str[16];
    
//     if (ui_lock(-1)) {
//         for (uint8_t i = 0; i < 13; i++) {
//             snprintf(value_str, sizeof(value_str), "%dmV", me->bms_data[slot_index].cell_volt[i]);
//             lv_label_set_text(cell_labels[i], value_str);
//         }
//         ui_unlock();
//     }
// }

// void ui_update_accu_int_value(DeviceHSM_t *me, uint8_t slot_index)
// {
//     if (slot_index >= BMS_BATTERY_NUM) {
//         ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
//         return;
//     }
    
//     char value_str[16];
//     snprintf(value_str, sizeof(value_str), "%lu", me->bms_data[slot_index].accu_int);
    
//     if (ui_lock(-1)) {
//         lv_label_set_text(ui_lbMainSlotACCUIntValue, value_str);
//         ui_unlock();
//     }
// }

// void ui_update_accu_frac_value(DeviceHSM_t *me, uint8_t slot_index)
// {
//     if (slot_index >= BMS_BATTERY_NUM) {
//         ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
//         return;
//     }
    
//     char value_str[16];
//     snprintf(value_str, sizeof(value_str), "%lu", me->bms_data[slot_index].accu_frac);
    
//     if (ui_lock(-1)) {
//         lv_label_set_text(ui_lbMainSlotACCUFracValue, value_str);
//         ui_unlock();
//     }
// }

// void ui_update_accu_time_value(DeviceHSM_t *me, uint8_t slot_index)
// {
//     if (slot_index >= BMS_BATTERY_NUM) {
//         ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
//         return;
//     }
    
//     char value_str[16];
//     snprintf(value_str, sizeof(value_str), "%lu", me->bms_data[slot_index].accu_time);
    
//     if (ui_lock(-1)) {
//         lv_label_set_text(ui_lbMainSlotACCUTimeValue, value_str);
//         ui_unlock();
//     }
// }

// void ui_update_safety_a_value(DeviceHSM_t *me, uint8_t slot_index)
// {
//     if (slot_index >= BMS_BATTERY_NUM) {
//         ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
//         return;
//     }
    
//     char value_str[16];
//     snprintf(value_str, sizeof(value_str), "0x%04X", me->bms_data[slot_index].safety_a);
    
//     if (ui_lock(-1)) {
//         lv_label_set_text(ui_lbMainSlotSafetyAValue, value_str);
//         ui_unlock();
//     }
// }

// void ui_update_safety_b_value(DeviceHSM_t *me, uint8_t slot_index)
// {
//     if (slot_index >= BMS_BATTERY_NUM) {
//         ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
//         return;
//     }
    
//     char value_str[16];
//     snprintf(value_str, sizeof(value_str), "0x%04X", me->bms_data[slot_index].safety_b);
    
//     if (ui_lock(-1)) {
//         lv_label_set_text(ui_lbMainSlotSafetyBValue, value_str);
//         ui_unlock();
//     }
// }

// void ui_update_safety_c_value(DeviceHSM_t *me, uint8_t slot_index)
// {
//     if (slot_index >= BMS_BATTERY_NUM) {
//         ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
//         return;
//     }
    
//     char value_str[16];
//     snprintf(value_str, sizeof(value_str), "0x%04X", me->bms_data[slot_index].safety_c);
    
//     if (ui_lock(-1)) {
//         lv_label_set_text(ui_lbMainSlotSafetyCValue, value_str);
//         ui_unlock();
//     }
// }

// void ui_update_all_slot_details(DeviceHSM_t *me, uint8_t slot_index)
// {
//     if (slot_index >= BMS_BATTERY_NUM) {
//         ESP_LOGE(TAG, "Invalid slot index: %d", slot_index);
//         return;
//     }
    
//     ui_update_bms_state_value(me, slot_index);
//     ui_update_ctrl_request_value(me, slot_index);
//     ui_update_ctrl_response_value(me, slot_index);
//     ui_update_fet_ctrl_pin_value(me, slot_index);
//     ui_update_fet_status_value(me, slot_index);
//     ui_update_alarm_bits_value(me, slot_index);
//     ui_update_faults_value(me, slot_index);
//     ui_update_pack_volt_value(me, slot_index);
//     ui_update_stack_volt_value(me, slot_index);
//     ui_update_pack_cur_value(me, slot_index);
//     ui_update_ld_volt_value(me, slot_index);
//     ui_update_pin_percent_value(me, slot_index);
//     ui_update_tg_percent_value(me, slot_index);
//     ui_update_cel_res_value(me, slot_index);
//     ui_update_soc_per_value(me, slot_index);
//     ui_update_soh_value(me, slot_index);
//     ui_update_sin_par_value(me, slot_index);
//     ui_update_temp1_value(me, slot_index);
//     ui_update_temp2_value(me, slot_index);
//     ui_update_temp3_value(me, slot_index);
//     ui_update_cell_voltages(me, slot_index);
//     ui_update_accu_int_value(me, slot_index);
//     ui_update_accu_frac_value(me, slot_index);
//     ui_update_accu_time_value(me, slot_index);
//     ui_update_safety_a_value(me, slot_index);
//     ui_update_safety_b_value(me, slot_index);
//     ui_update_safety_c_value(me, slot_index);
// }
