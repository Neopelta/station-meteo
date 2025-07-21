################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/user/hts221_reg.c \
../Drivers/user/lps22hh_reg.c 

OBJS += \
./Drivers/user/hts221_reg.o \
./Drivers/user/lps22hh_reg.o 

C_DEPS += \
./Drivers/user/hts221_reg.d \
./Drivers/user/lps22hh_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/user/%.o Drivers/user/%.su Drivers/user/%.cyclo: ../Drivers/user/%.c Drivers/user/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../../Inc -I../../Drivers/BSP/STM32L4xx_Nucleo -I../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../Drivers/CMSIS/Include -I../../Middlewares/ST/BlueNRG-2/hci/hci_tl_patterns/Basic -I../../Middlewares/ST/BlueNRG-2/utils -I../../Middlewares/ST/BlueNRG-2/includes -I"C:/Users/victo/Desktop/Bureau/Cours/M1/LE/PROJETS/SensorDemo_BLE_l476rg/STM32CubeIDE/Drivers/user" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-user

clean-Drivers-2f-user:
	-$(RM) ./Drivers/user/hts221_reg.cyclo ./Drivers/user/hts221_reg.d ./Drivers/user/hts221_reg.o ./Drivers/user/hts221_reg.su ./Drivers/user/lps22hh_reg.cyclo ./Drivers/user/lps22hh_reg.d ./Drivers/user/lps22hh_reg.o ./Drivers/user/lps22hh_reg.su

.PHONY: clean-Drivers-2f-user

