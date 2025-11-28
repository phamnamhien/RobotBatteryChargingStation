idf_component_register(
    SRCS "modbus_slave_manager.c"
    INCLUDE_DIRS "include"
    REQUIRES espressif__esp-modbus driver
)