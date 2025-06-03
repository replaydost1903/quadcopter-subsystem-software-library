################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bmp388/bmp388_selftest/Src/bmp3_selftest.c 

OBJS += \
./bmp388/bmp388_selftest/Src/bmp3_selftest.o 

C_DEPS += \
./bmp388/bmp388_selftest/Src/bmp3_selftest.d 


# Each subdirectory must supply rules for building sources it contributes
bmp388/bmp388_selftest/Src/%.o bmp388/bmp388_selftest/Src/%.su bmp388/bmp388_selftest/Src/%.cyclo: ../bmp388/bmp388_selftest/Src/%.c bmp388/bmp388_selftest/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"C:/Users/Kubilay/STM32CubeIDE/workspace_1.18.1/BMP388/bmp388/bmp388_port/Inc" -I"C:/Users/Kubilay/STM32CubeIDE/workspace_1.18.1/BMP388/bmp388/bmp388_selftest/Inc" -I"C:/Users/Kubilay/STM32CubeIDE/workspace_1.18.1/BMP388/bmp388/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-bmp388-2f-bmp388_selftest-2f-Src

clean-bmp388-2f-bmp388_selftest-2f-Src:
	-$(RM) ./bmp388/bmp388_selftest/Src/bmp3_selftest.cyclo ./bmp388/bmp388_selftest/Src/bmp3_selftest.d ./bmp388/bmp388_selftest/Src/bmp3_selftest.o ./bmp388/bmp388_selftest/Src/bmp3_selftest.su

.PHONY: clean-bmp388-2f-bmp388_selftest-2f-Src

