# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.

idf_component_register(
    SRCS main.c                     
    INCLUDE_DIRS        "."         
    PRIV_INCLUDE_DIRS                          
    REQUIRES            espressif__esp-modbus 
                        lvgl__lvgl 
                        espressif__esp_lvgl_port
                        driver 
                        esp_timer             
    PRIV_REQUIRES                   
)