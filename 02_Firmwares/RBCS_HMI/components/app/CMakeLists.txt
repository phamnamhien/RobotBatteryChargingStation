idf_component_register(
                    SRCS            "app_params.c"
                                    "app_states.c"
                    INCLUDE_DIRS    "include"
                    REQUIRES        driver 
                                    esp_timer
                                    modbus_manager
                                    lvgl__lvgl
                                    espressif__esp_lvgl_port
                                    ui  
                                    hsm
)