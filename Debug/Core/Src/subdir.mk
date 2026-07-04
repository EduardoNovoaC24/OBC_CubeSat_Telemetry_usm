################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/FusionAhrs.c \
../Core/Src/FusionBias.c \
../Core/Src/FusionCompass.c \
../Core/Src/freertos.c \
../Core/Src/hmc5883l.c \
../Core/Src/lorawan_crypto.c \
../Core/Src/main.c \
../Core/Src/mpu6050.c \
../Core/Src/radio_test.c \
../Core/Src/stm32l4xx_hal_msp.c \
../Core/Src/stm32l4xx_hal_timebase_tim.c \
../Core/Src/stm32l4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32l4xx.c 

OBJS += \
./Core/Src/FusionAhrs.o \
./Core/Src/FusionBias.o \
./Core/Src/FusionCompass.o \
./Core/Src/freertos.o \
./Core/Src/hmc5883l.o \
./Core/Src/lorawan_crypto.o \
./Core/Src/main.o \
./Core/Src/mpu6050.o \
./Core/Src/radio_test.o \
./Core/Src/stm32l4xx_hal_msp.o \
./Core/Src/stm32l4xx_hal_timebase_tim.o \
./Core/Src/stm32l4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32l4xx.o 

C_DEPS += \
./Core/Src/FusionAhrs.d \
./Core/Src/FusionBias.d \
./Core/Src/FusionCompass.d \
./Core/Src/freertos.d \
./Core/Src/hmc5883l.d \
./Core/Src/lorawan_crypto.d \
./Core/Src/main.d \
./Core/Src/mpu6050.d \
./Core/Src/radio_test.d \
./Core/Src/stm32l4xx_hal_msp.d \
./Core/Src/stm32l4xx_hal_timebase_tim.d \
./Core/Src/stm32l4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32l4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/SX126x -I/Users/fablabutfsm/Desktop/Dudu_folder/OBC_MPU6050_FreeRTOS_BT/Drivers/Radiolib -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/FusionAhrs.cyclo ./Core/Src/FusionAhrs.d ./Core/Src/FusionAhrs.o ./Core/Src/FusionAhrs.su ./Core/Src/FusionBias.cyclo ./Core/Src/FusionBias.d ./Core/Src/FusionBias.o ./Core/Src/FusionBias.su ./Core/Src/FusionCompass.cyclo ./Core/Src/FusionCompass.d ./Core/Src/FusionCompass.o ./Core/Src/FusionCompass.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/hmc5883l.cyclo ./Core/Src/hmc5883l.d ./Core/Src/hmc5883l.o ./Core/Src/hmc5883l.su ./Core/Src/lorawan_crypto.cyclo ./Core/Src/lorawan_crypto.d ./Core/Src/lorawan_crypto.o ./Core/Src/lorawan_crypto.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/mpu6050.cyclo ./Core/Src/mpu6050.d ./Core/Src/mpu6050.o ./Core/Src/mpu6050.su ./Core/Src/radio_test.cyclo ./Core/Src/radio_test.d ./Core/Src/radio_test.o ./Core/Src/radio_test.su ./Core/Src/stm32l4xx_hal_msp.cyclo ./Core/Src/stm32l4xx_hal_msp.d ./Core/Src/stm32l4xx_hal_msp.o ./Core/Src/stm32l4xx_hal_msp.su ./Core/Src/stm32l4xx_hal_timebase_tim.cyclo ./Core/Src/stm32l4xx_hal_timebase_tim.d ./Core/Src/stm32l4xx_hal_timebase_tim.o ./Core/Src/stm32l4xx_hal_timebase_tim.su ./Core/Src/stm32l4xx_it.cyclo ./Core/Src/stm32l4xx_it.d ./Core/Src/stm32l4xx_it.o ./Core/Src/stm32l4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32l4xx.cyclo ./Core/Src/system_stm32l4xx.d ./Core/Src/system_stm32l4xx.o ./Core/Src/system_stm32l4xx.su

.PHONY: clean-Core-2f-Src

