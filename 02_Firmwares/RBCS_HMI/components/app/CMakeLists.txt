idf_component_register(
                    SRCS            "app_params.c"
                                    "app_states.c"
                    INCLUDE_DIRS    "include"
                    REQUIRES        driver 
                                    esp_timer
                                    hsm 

                                    modbus_manager

                                    esp_lcd_touch_gt911
                                    esp_lcd_touch
                                    lvgl
                                    ui                               
)