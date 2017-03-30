################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Duet/ConnectionState.cpp \
../src/Duet/Network.cpp \
../src/Duet/NetworkTransaction.cpp \
../src/Duet/Webserver.cpp 

OBJS += \
./src/Duet/ConnectionState.o \
./src/Duet/Network.o \
./src/Duet/NetworkTransaction.o \
./src/Duet/Webserver.o 

CPP_DEPS += \
./src/Duet/ConnectionState.d \
./src/Duet/Network.d \
./src/Duet/NetworkTransaction.d \
./src/Duet/Webserver.d 


# Each subdirectory must supply rules for building sources it contributes
src/Duet/%.o: ../src/Duet/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-none-eabi-g++ -D__SAM3X8E__ -Dprintf=iprintf -I"C:\Eclipse\CoreNG\cores\arduino" -I"C:\Eclipse\CoreNG\libraries\Flash" -I"C:\Eclipse\CoreNG\libraries\SharedSpi" -I"C:\Eclipse\CoreNG\libraries\Storage" -I"C:\Eclipse\CoreNG\libraries\Wire" -I"C:\Eclipse\CoreNG\asf" -I"C:\Eclipse\CoreNG\asf\common\utils" -I"C:\Eclipse\CoreNG\asf\common\services\clock" -I"C:\Eclipse\CoreNG\asf\sam\drivers\efc" -I"C:\Eclipse\CoreNG\asf\sam\drivers\emac" -I"C:\Eclipse\CoreNG\asf\sam\drivers\pmc" -I"C:\Eclipse\CoreNG\asf\sam\drivers\spi" -I"C:\Eclipse\CoreNG\asf\sam\drivers\twi" -I"C:\Eclipse\CoreNG\asf\sam\services\flash_efc" -I"C:\Eclipse\CoreNG\asf\sam\utils" -I"C:\Eclipse\CoreNG\asf\sam\utils\cmsis\sam3x\include" -I"C:\Eclipse\CoreNG\asf\sam\utils\header_files" -I"C:\Eclipse\CoreNG\asf\sam\utils\preprocessor" -I"C:\Eclipse\CoreNG\asf\thirdparty\CMSIS\Include" -I"C:\Eclipse\CoreNG\variants\duet" -I"C:\Eclipse\Firmware\src" -I"C:\Eclipse\Firmware\src\Duet" -I"C:\Eclipse\Firmware\src\Duet\Lwip" -I"C:\Eclipse\Firmware\src\Duet\EMAC" -O2 -Wall -c -std=gnu++11 -mcpu=cortex-m3 -mthumb -ffunction-sections -fdata-sections -fno-threadsafe-statics -fno-rtti -fno-exceptions -nostdlib "-Wa,-ahl=$*.s" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


