################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/lsm303dlhc/lsm303dlhc.c 

OBJS += \
./Drivers/BSP/Components/lsm303dlhc/lsm303dlhc.o 

C_DEPS += \
./Drivers/BSP/Components/lsm303dlhc/lsm303dlhc.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/lsm303dlhc/lsm303dlhc.o: ../Drivers/BSP/Components/lsm303dlhc/lsm303dlhc.c
	arm-none-eabi-gcc -c "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/steve/Desktop/EleDigi/Test/Middlewares/Third_Party/FatFs/src" -I"C:/Users/steve/Desktop/EleDigi/Test/Middlewares/ST/STM32_USB_Host_Library/Class/MSC" -I"C:/Users/steve/Desktop/EleDigi/Test/Middlewares/ST/STM32_USB_Host_Library/Class/MSC/Inc" -I"C:/Users/steve/Desktop/EleDigi/Test/Middlewares/ST/STM32_USB_Host_Library/Class/MSC/Src" -I"C:/Users/steve/Desktop/EleDigi/Test/Drivers/BSP/STM32F4-DIscovery" -I"C:/Users/steve/Desktop/EleDigi/Test/Middlewares/Third_Party/SpiritDSP_MP3_Dec/inc" -I"C:/Users/steve/Desktop/EleDigi/Test/Middlewares/Third_Party/SpiritDSP_MP3_Dec/lib" -I"C:/Users/steve/Desktop/EleDigi/Test/Drivers/STM32F4xx_HAL_Driver/Src" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/Components/lsm303dlhc/lsm303dlhc.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"

