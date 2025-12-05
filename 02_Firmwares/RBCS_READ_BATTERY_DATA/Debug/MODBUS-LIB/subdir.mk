################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MODBUS-LIB/Modbus.c \
../MODBUS-LIB/UARTCallback.c 

OBJS += \
./MODBUS-LIB/Modbus.o \
./MODBUS-LIB/UARTCallback.o 

C_DEPS += \
./MODBUS-LIB/Modbus.d \
./MODBUS-LIB/UARTCallback.d 


# Each subdirectory must supply rules for building sources it contributes
MODBUS-LIB/%.o MODBUS-LIB/%.su MODBUS-LIB/%.cyclo: ../MODBUS-LIB/%.c MODBUS-LIB/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"D:/Documents/GitHub/vinmotion/RobotBatteryChargingStation/02_Firmwares/RBCS_READ_BATTERY_DATA/EE" -I"D:/Documents/GitHub/vinmotion/RobotBatteryChargingStation/02_Firmwares/RBCS_READ_BATTERY_DATA/HSM" -I"D:/Documents/GitHub/vinmotion/RobotBatteryChargingStation/02_Firmwares/RBCS_READ_BATTERY_DATA/APP" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I"D:/Documents/GitHub/vinmotion/RobotBatteryChargingStation/02_Firmwares/RBCS_READ_BATTERY_DATA/MODBUS-LIB" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-MODBUS-2d-LIB

clean-MODBUS-2d-LIB:
	-$(RM) ./MODBUS-LIB/Modbus.cyclo ./MODBUS-LIB/Modbus.d ./MODBUS-LIB/Modbus.o ./MODBUS-LIB/Modbus.su ./MODBUS-LIB/UARTCallback.cyclo ./MODBUS-LIB/UARTCallback.d ./MODBUS-LIB/UARTCallback.o ./MODBUS-LIB/UARTCallback.su

.PHONY: clean-MODBUS-2d-LIB

