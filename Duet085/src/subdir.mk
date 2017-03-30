################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Fan.cpp \
../src/OutputMemory.cpp \
../src/Platform.cpp \
../src/PrintMonitor.cpp \
../src/RepRapFirmware.cpp \
../src/Reprap.cpp \
../src/Roland.cpp \
../src/Tool.cpp 

OBJS += \
./src/Fan.o \
./src/OutputMemory.o \
./src/Platform.o \
./src/PrintMonitor.o \
./src/RepRapFirmware.o \
./src/Reprap.o \
./src/Roland.o \
./src/Tool.o 

CPP_DEPS += \
./src/Fan.d \
./src/OutputMemory.d \
./src/Platform.d \
./src/PrintMonitor.d \
./src/RepRapFirmware.d \
./src/Reprap.d \
./src/Roland.d \
./src/Tool.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-none-eabi-g++ -D__SAM3X8E__ -Dprintf=iprintf -I"C:\Eclipse\CoreNG\cores\arduino" -I"C:\Eclipse\CoreNG\libraries\Flash" -I"C:\Eclipse\CoreNG\libraries\SharedSpi" -I"C:\Eclipse\CoreNG\libraries\Storage" -I"C:\Eclipse\CoreNG\libraries\Wire" -I"C:\Eclipse\CoreNG\asf" -I"C:\Eclipse\CoreNG\asf\common\utils" -I"C:\Eclipse\CoreNG\asf\common\services\clock" -I"C:\Eclipse\CoreNG\asf\sam\drivers\efc" -I"C:\Eclipse\CoreNG\asf\sam\drivers\emac" -I"C:\Eclipse\CoreNG\asf\sam\drivers\pmc" -I"C:\Eclipse\CoreNG\asf\sam\drivers\spi" -I"C:\Eclipse\CoreNG\asf\sam\drivers\twi" -I"C:\Eclipse\CoreNG\asf\sam\services\flash_efc" -I"C:\Eclipse\CoreNG\asf\sam\utils" -I"C:\Eclipse\CoreNG\asf\sam\utils\cmsis\sam3x\include" -I"C:\Eclipse\CoreNG\asf\sam\utils\header_files" -I"C:\Eclipse\CoreNG\asf\sam\utils\preprocessor" -I"C:\Eclipse\CoreNG\asf\thirdparty\CMSIS\Include" -I"C:\Eclipse\CoreNG\variants\duet" -I"C:\Eclipse\Firmware\src" -I"C:\Eclipse\Firmware\src\Duet" -I"C:\Eclipse\Firmware\src\Duet\Lwip" -I"C:\Eclipse\Firmware\src\Duet\EMAC" -O2 -Wall -c -std=gnu++11 -mcpu=cortex-m3 -mthumb -ffunction-sections -fdata-sections -fno-threadsafe-statics -fno-rtti -fno-exceptions -nostdlib "-Wa,-ahl=$*.s" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


