################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/Duet/Lwip/contrib/apps/netbios/netbios.c 

OBJS += \
./src/Duet/Lwip/contrib/apps/netbios/netbios.o 

C_DEPS += \
./src/Duet/Lwip/contrib/apps/netbios/netbios.d 


# Each subdirectory must supply rules for building sources it contributes
src/Duet/Lwip/contrib/apps/netbios/%.o: ../src/Duet/Lwip/contrib/apps/netbios/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	arm-none-eabi-gcc -D__SAM3X8E__ -Dprintf=iprintf -I"C:\Eclipse\CoreNG\cores\arduino" -I"C:\Eclipse\CoreNG\libraries\Storage" -I"C:\Eclipse\CoreNG\asf" -I"C:\Eclipse\CoreNG\asf\common\utils" -I"C:\Eclipse\CoreNG\asf\sam\drivers\emac" -I"C:\Eclipse\CoreNG\asf\sam\drivers\hsmci" -I"C:\Eclipse\CoreNG\asf\sam\drivers\rstc" -I"C:\Eclipse\CoreNG\asf\sam\drivers\rtc" -I"C:\Eclipse\CoreNG\asf\sam\utils" -I"C:\Eclipse\CoreNG\asf\sam\utils\cmsis\sam3x\include" -I"C:\Eclipse\CoreNG\asf\sam\utils\header_files" -I"C:\Eclipse\CoreNG\asf\sam\utils\preprocessor" -I"C:\Eclipse\CoreNG\asf\thirdparty\CMSIS\Include" -I"C:\Eclipse\CoreNG\variants\duet" -I"C:\Eclipse\Firmware\src\Duet\Lwip" -I"C:\Eclipse\Firmware\src\Duet\Lwip\lwip\src\include" -I"C:\Eclipse\Firmware\src\Duet\EMAC" -O2 -Wall -c -std=gnu99 -mcpu=cortex-m3 -mthumb -ffunction-sections -fdata-sections -nostdlib "-Wa,-ahl=$*.s" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


