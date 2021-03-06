################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/MotionControl.c \
../src/MotorCommunication.c \
../src/_write.c \
../src/delay.c \
../src/main.c \
../src/motor.c 

OBJS += \
./src/MotionControl.o \
./src/MotorCommunication.o \
./src/_write.o \
./src/delay.o \
./src/main.o \
./src/motor.o 

C_DEPS += \
./src/MotionControl.d \
./src/MotorCommunication.d \
./src/_write.d \
./src/delay.d \
./src/main.d \
./src/motor.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DSTM32F051 -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f0-stdperiph" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


