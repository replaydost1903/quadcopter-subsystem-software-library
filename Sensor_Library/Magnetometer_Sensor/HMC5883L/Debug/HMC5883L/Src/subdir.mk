################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../HMC5883L/Src/hmc5883l.c 

OBJS += \
./HMC5883L/Src/hmc5883l.o 

C_DEPS += \
./HMC5883L/Src/hmc5883l.d 


# Each subdirectory must supply rules for building sources it contributes
HMC5883L/Src/%.o HMC5883L/Src/%.su HMC5883L/Src/%.cyclo: ../HMC5883L/Src/%.c HMC5883L/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"C:/Users/Kubilay/STM32CubeIDE/workspace_1.18.1/Magnetometer_Sensor_HMC5883L/HMC5883L/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-HMC5883L-2f-Src

clean-HMC5883L-2f-Src:
	-$(RM) ./HMC5883L/Src/hmc5883l.cyclo ./HMC5883L/Src/hmc5883l.d ./HMC5883L/Src/hmc5883l.o ./HMC5883L/Src/hmc5883l.su

.PHONY: clean-HMC5883L-2f-Src

