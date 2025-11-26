# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.

idf_component_register(
    SRCS main.c                     
    INCLUDE_DIRS        "."   
                        "app"      
    PRIV_INCLUDE_DIRS                          
    REQUIRES            modbus_manager
                        lvgl__lvgl 
                        espressif__esp_lvgl_port
                        driver 
                        esp_timer     
                        ui         
    PRIV_REQUIRES                   
)