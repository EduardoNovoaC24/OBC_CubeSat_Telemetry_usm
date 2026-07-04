################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/SX126x/sx126x.c \
../Drivers/SX126x/sx126x_hal_stm32.c 

OBJS += \
./Drivers/SX126x/sx126x.o \
./Drivers/SX126x/sx126x_hal_stm32.o 

C_DEPS += \
./Drivers/SX126x/sx126x.d \
./Drivers/SX126x/sx126x_hal_stm32.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/SX126x/%.o Drivers/SX126x/%.su Drivers/SX126x/%.cyclo: ../Drivers/SX126x/%.c Drivers/SX126x/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/SX126x -I/Users/fablabutfsm/Desktop/Dudu_folder/OBC_MPU6050_FreeRTOS_BT/Drivers/Radiolib -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-SX126x

clean-Drivers-2f-SX126x:
	-$(RM) ./Drivers/SX126x/sx126x.cyclo ./Drivers/SX126x/sx126x.d ./Drivers/SX126x/sx126x.o ./Drivers/SX126x/sx126x.su ./Drivers/SX126x/sx126x_hal_stm32.cyclo ./Drivers/SX126x/sx126x_hal_stm32.d ./Drivers/SX126x/sx126x_hal_stm32.o ./Drivers/SX126x/sx126x_hal_stm32.su

.PHONY: clean-Drivers-2f-SX126x

