################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../CPP/Src/mainCPP.cpp 

OBJS += \
./CPP/Src/mainCPP.o 

CPP_DEPS += \
./CPP/Src/mainCPP.d 


# Each subdirectory must supply rules for building sources it contributes
CPP/Src/%.o CPP/Src/%.su CPP/Src/%.cyclo: ../CPP/Src/%.cpp CPP/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-CPP-2f-Src

clean-CPP-2f-Src:
	-$(RM) ./CPP/Src/mainCPP.cyclo ./CPP/Src/mainCPP.d ./CPP/Src/mainCPP.o ./CPP/Src/mainCPP.su

.PHONY: clean-CPP-2f-Src

