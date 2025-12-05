################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../HSM/hsm.c \
../HSM/hsm_timer.c 

OBJS += \
./HSM/hsm.o \
./HSM/hsm_timer.o 

C_DEPS += \
./HSM/hsm.d \
./HSM/hsm_timer.d 


# Each subdirectory must supply rules for building sources it contributes
HSM/%.o HSM/%.su HSM/%.cyclo: ../HSM/%.c HSM/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"D:/Documents/GitHub/vinmotion/RobotBatteryChargingStation/02_Firmwares/RBCS_READ_BATTERY_DATA/EE" -I"D:/Documents/GitHub/vinmotion/RobotBatteryChargingStation/02_Firmwares/RBCS_READ_BATTERY_DATA/HSM" -I"D:/Documents/GitHub/vinmotion/RobotBatteryChargingStation/02_Firmwares/RBCS_READ_BATTERY_DATA/APP" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I"D:/Documents/GitHub/vinmotion/RobotBatteryChargingStation/02_Firmwares/RBCS_READ_BATTERY_DATA/MODBUS-LIB" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-HSM

clean-HSM:
	-$(RM) ./HSM/hsm.cyclo ./HSM/hsm.d ./HSM/hsm.o ./HSM/hsm.su ./HSM/hsm_timer.cyclo ./HSM/hsm_timer.d ./HSM/hsm_timer.o ./HSM/hsm_timer.su

.PHONY: clean-HSM

