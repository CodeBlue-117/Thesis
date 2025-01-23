################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/L6470.c \
../Core/Src/example.c \
../Core/Src/main.c \
../Core/Src/params.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/stm32f4xx_nucleo.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/xnucleoihm02a1.c \
../Core/Src/xnucleoihm02a1_interface.c 

OBJS += \
./Core/Src/L6470.o \
./Core/Src/example.o \
./Core/Src/main.o \
./Core/Src/params.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/stm32f4xx_nucleo.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/xnucleoihm02a1.o \
./Core/Src/xnucleoihm02a1_interface.o 

C_DEPS += \
./Core/Src/L6470.d \
./Core/Src/example.d \
./Core/Src/main.d \
./Core/Src/params.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/stm32f4xx_nucleo.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/xnucleoihm02a1.d \
./Core/Src/xnucleoihm02a1_interface.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F410Rx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/L6470.cyclo ./Core/Src/L6470.d ./Core/Src/L6470.o ./Core/Src/L6470.su ./Core/Src/example.cyclo ./Core/Src/example.d ./Core/Src/example.o ./Core/Src/example.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/params.cyclo ./Core/Src/params.d ./Core/Src/params.o ./Core/Src/params.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/stm32f4xx_nucleo.cyclo ./Core/Src/stm32f4xx_nucleo.d ./Core/Src/stm32f4xx_nucleo.o ./Core/Src/stm32f4xx_nucleo.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/xnucleoihm02a1.cyclo ./Core/Src/xnucleoihm02a1.d ./Core/Src/xnucleoihm02a1.o ./Core/Src/xnucleoihm02a1.su ./Core/Src/xnucleoihm02a1_interface.cyclo ./Core/Src/xnucleoihm02a1_interface.d ./Core/Src/xnucleoihm02a1_interface.o ./Core/Src/xnucleoihm02a1_interface.su

.PHONY: clean-Core-2f-Src

