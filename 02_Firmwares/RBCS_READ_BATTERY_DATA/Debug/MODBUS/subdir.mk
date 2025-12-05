################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MODBUS/modbus_rtu.c 

OBJS += \
./MODBUS/modbus_rtu.o 

C_DEPS += \
./MODBUS/modbus_rtu.d 


# Each subdirectory must supply rules for building sources it contributes
MODBUS/%.o MODBUS/%.su MODBUS/%.cyclo: ../MODBUS/%.c MODBUS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"D:/Documents/GitHub/vinmotion/RobotBatteryChargingStation/02_Firmwares/RBCS_READ_BATTERY_DATA/EE" -I"D:/Documents/GitHub/vinmotion/RobotBatteryChargingStation/02_Firmwares/RBCS_READ_BATTERY_DATA/HSM" -I"D:/Documents/GitHub/vinmotion/RobotBatteryChargingStation/02_Firmwares/RBCS_READ_BATTERY_DATA/APP" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I"D:/Documents/GitHub/vinmotion/RobotBatteryChargingStation/02_Firmwares/RBCS_READ_BATTERY_DATA/MODBUS" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-MODBUS

clean-MODBUS:
	-$(RM) ./MODBUS/modbus_rtu.cyclo ./MODBUS/modbus_rtu.d ./MODBUS/modbus_rtu.o ./MODBUS/modbus_rtu.su

.PHONY: clean-MODBUS

