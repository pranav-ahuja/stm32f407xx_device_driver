################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/007i2c_masterTx.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/007i2c_masterTx.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/007i2c_masterTx.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"C:/Users/Rakesh Ahuja/Desktop/folders/Embedded/Device drivers/stm32f407x/STM32F407_Device_Drivers/drivers/src" -I"C:/Users/Rakesh Ahuja/Desktop/folders/Embedded/Device drivers/stm32f407x/STM32F407_Device_Drivers/drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/007i2c_masterTx.cyclo ./Src/007i2c_masterTx.d ./Src/007i2c_masterTx.o ./Src/007i2c_masterTx.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

