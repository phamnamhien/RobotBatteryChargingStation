idf_component_register(
                    SRCS            "app_params.c"
                                    "app_states.c"
                    INCLUDE_DIRS    "include"
                    REQUIRES        driver 
                                    esp_timer
                                    nvs_flash
                                    hsm 
                                    esp_ticks

                                    modbus_master_manager  
                                    modbus_slave_manager  

                                    stepper_tb6600
                                    limit_switch
)