################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bmi160/bmi160_port/Src/bmi160_port.c 

OBJS += \
./bmi160/bmi160_port/Src/bmi160_port.o 

C_DEPS += \
./bmi160/bmi160_port/Src/bmi160_port.d 


# Each subdirectory must supply rules for building sources it contributes
bmi160/bmi160_port/Src/%.o bmi160/bmi160_port/Src/%.su bmi160/bmi160_port/Src/%.cyclo: ../bmi160/bmi160_port/Src/%.c bmi160/bmi160_port/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"C:/Users/Kubilay/STM32CubeIDE/workspace_1.18.1/Inertial_Measurement_Unit_BMI160/bmi160/bmi160_port/Inc" -I"C:/Users/Kubilay/STM32CubeIDE/workspace_1.18.1/Inertial_Measurement_Unit_BMI160/bmi160/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-bmi160-2f-bmi160_port-2f-Src

clean-bmi160-2f-bmi160_port-2f-Src:
	-$(RM) ./bmi160/bmi160_port/Src/bmi160_port.cyclo ./bmi160/bmi160_port/Src/bmi160_port.d ./bmi160/bmi160_port/Src/bmi160_port.o ./bmi160/bmi160_port/Src/bmi160_port.su

.PHONY: clean-bmi160-2f-bmi160_port-2f-Src

