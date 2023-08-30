################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../CPP/Source/main.cpp 

OBJS += \
./CPP/Source/main.o 

CPP_DEPS += \
./CPP/Source/main.d 


# Each subdirectory must supply rules for building sources it contributes
CPP/Source/%.o CPP/Source/%.su CPP/Source/%.cyclo: ../CPP/Source/%.cpp CPP/Source/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../CPP -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-CPP-2f-Source

clean-CPP-2f-Source:
	-$(RM) ./CPP/Source/main.cyclo ./CPP/Source/main.d ./CPP/Source/main.o ./CPP/Source/main.su

.PHONY: clean-CPP-2f-Source

