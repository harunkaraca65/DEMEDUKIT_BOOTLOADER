################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ssd1306_LL/ssd1306_ll.c 

OBJS += \
./ssd1306_LL/ssd1306_ll.o 

C_DEPS += \
./ssd1306_LL/ssd1306_ll.d 


# Each subdirectory must supply rules for building sources it contributes
ssd1306_LL/%.o ssd1306_LL/%.su ssd1306_LL/%.cyclo: ../ssd1306_LL/%.c ssd1306_LL/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DSTM32G030xx -DUSE_FULL_LL_DRIVER -DHSE_VALUE=8000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DEXTERNAL_CLOCK_VALUE=12288000 -DHSI_VALUE=16000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=1 -DINSTRUCTION_CACHE_ENABLE=1 -DDATA_CACHE_ENABLE=1 -c -I../Core/Inc -I"C:/ST/STProjects/DEMSAY/DEM_BOOTLOADER_STM/ssd1306_LL" -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-ssd1306_LL

clean-ssd1306_LL:
	-$(RM) ./ssd1306_LL/ssd1306_ll.cyclo ./ssd1306_LL/ssd1306_ll.d ./ssd1306_LL/ssd1306_ll.o ./ssd1306_LL/ssd1306_ll.su

.PHONY: clean-ssd1306_LL

