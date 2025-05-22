################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/command_parser.c \
../Src/main.c \
../Src/serial.c \
../Src/servo.c \
../Src/stepper_timer.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/command_parser.o \
./Src/main.o \
./Src/serial.o \
./Src/servo.o \
./Src/stepper_timer.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/command_parser.d \
./Src/main.d \
./Src/serial.d \
./Src/servo.d \
./Src/stepper_timer.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F303VCTx -DSTM32 -DSTM32F3 -DSTM32F3DISCOVERY -c -I../Inc -I"C:/Users/AngusMclean/Downloads/MTRX2700-2025-main/MTRX2700-2025-main/stm32f303-definitions/Core/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/command_parser.cyclo ./Src/command_parser.d ./Src/command_parser.o ./Src/command_parser.su ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/serial.cyclo ./Src/serial.d ./Src/serial.o ./Src/serial.su ./Src/servo.cyclo ./Src/servo.d ./Src/servo.o ./Src/servo.su ./Src/stepper_timer.cyclo ./Src/stepper_timer.d ./Src/stepper_timer.o ./Src/stepper_timer.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

