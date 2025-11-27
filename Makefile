###############################################################################
#  SPDX-License-Identifier: GPL-3.0-or-later                                  #
#  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
#  -------------------------------------------------------------------------  #
#  Copyright (c) 2025 Drona Aviation                                          #
#  All rights reserved.                                                       #
#  -------------------------------------------------------------------------  #
#  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
#  Project: MagisV2-MechAsh-Dev                                               #
#  File: \Makefile                                                            #
#  Created Date: Mon, 28th Apr 2025                                           #
#  Brief:                                                                     #
#  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
#  Last Modified: Tue, 29th Apr 2025                                          #
#  Modified By: AJ                                                            #
#  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
#  HISTORY:                                                                   #
#  Date      	By	Comments                                                    #
#  ----------	---	---------------------------------------------------------   #
###############################################################################
#
# Makefile for building the MasigV2 firmware.
#
# Invoke this with 'make help' to see the list of supported targets.
#
###############################################################################
# User-configurable options
FORKNAME	=	MAGISV2
TARGET	?=	
BUILD_TYPE	?= BIN
PROJECT ?= DEFAULT
LIB_MAJOR_VERSION	=	1
LIB_MINOR_VERSION	=	1
FW_Version	=	2.8.0-beta
API_Version	=	0.28.0
# Flash size (KB).  Some low-end chips actually have more flash than advertised, use this to override.
FLASH_SIZE	?=
# Debugger optons, must be empty or GDB
DEBUG	?=
# Serial port/Device for flashing
SERIAL_DEVICE	?=	$(firstword $(wildcard /dev/ttyUSB*) no-port-found)


# Compile-time options
OPTIONS	?= 	'__FORKNAME__="$(FORKNAME)"' \
		   			'__TARGET__="$(TARGET)"' \
			 			'__FW_VER__="$(FW_Version)"' \
		   			'__API_VER__="$(API_Version)"' \
		   			'__PROJECT__="$(PROJECT)"' \
        		'__BUILD_DATE__="$(shell date +%d-%m-%Y)"' \
        		'__BUILD_TIME__="$(shell date +%H:%M:%S)"' \

# Configure default flash sizes for the targets
ifeq ($(FLASH_SIZE),)
	ifeq ($(TARGET),$(filter $(TARGET),PRIMUSX2 PRIMUS_V5 PRIMUS_X2_v1))
		FLASH_SIZE = 256
	else
		$(error FLASH_SIZE not configured for target)
	endif
endif

VALID_TARGETS	=	PRIMUSX2 PRIMUS_V5 PRIMUS_X2_v1

###############################################################################
# Directorie

ROOT	:=	$(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))
SRC_DIR	=	$(ROOT)/src/main
BUILD_DIR	=	$(ROOT)/Build
CMSIS_DIR	=	$(ROOT)/lib/main/CMSIS
INCLUDE_DIRS	=	$(SRC_DIR)
LINKER_DIR	=	$(ROOT)/src/main/target

# Search path for sources
VPATH	:=	$(SRC_DIR) \
					$(SRC_DIR)/startup


USBFS_DIR	=	$(ROOT)/lib/main/STM32_USB-FS-Device_Driver
USBPERIPH_SRC	=	$(notdir $(wildcard $(USBFS_DIR)/src/*.c))

# Ranging sensor VL53L0X libraries
RANGING_DIR	=	$(ROOT)/lib/main/VL53L0X_API
RANGING_SRC	=	$(notdir $(wildcard $(RANGING_DIR)/core/src/*.c \
																	$(RANGING_DIR)/platform/src/*.c\
																	$(RANGING_DIR)/core/src/*.cpp \
																	$(RANGING_DIR)/platform/src/*.cpp))

RANGING_DIR2	=	$(ROOT)/lib/main/VL53L1X_API
RANGING_SRC2	=	$(notdir $(wildcard $(RANGING_DIR2)/core/src/*.c \
																	$(RANGING_DIR2)/platform/src/*.c\
																	$(RANGING_DIR2)/core/src/*.cpp \
																	$(RANGING_DIR2)/platform/src/*.cpp))

INCLUDE_DIRS	:=	$(INCLUDE_DIRS) \
              		$(RANGING_DIR)/core/inc \
              		$(RANGING_DIR)/platform/inc \
									$(RANGING_DIR2)/core/inc \
              		$(RANGING_DIR2)/platform/inc

VPATH := 	$(VPATH) \
					$(RANGING_DIR)/core/src \
					$(RANGING_DIR)/platform/src \
					$(RANGING_DIR2)/core/src \
					$(RANGING_DIR2)/platform/src


CSOURCES	:=	$(shell find $(SRC_DIR) -name '*.c')

# MCU and Peripheral settings for PRIMUSX2
ifeq ($(TARGET),$(filter $(TARGET),PRIMUSX2 PRIMUS_V5 PRIMUS_X2_v1))

STDPERIPH_DIR = $(ROOT)/lib/main/STM32F30x_StdPeriph_Driver

STDPERIPH_SRC = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))

EXCLUDES	=	stm32f30x_crc.c \
    				stm32f30x_can.c

STDPERIPH_SRC := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))

DEVICE_STDPERIPH_SRC = $(STDPERIPH_SRC)

VPATH := 	$(VPATH) \
					$(CMSIS_DIR)/CM1/CoreSupport \
					$(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x

CMSIS_SRC = $(notdir $(wildcard $(CMSIS_DIR)/CM1/CoreSupport/*.c \
               									$(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x/*.c))

INCLUDE_DIRS := $(INCLUDE_DIRS) \
								$(ROOT) \
           			$(STDPERIPH_DIR)/inc \
           			$(CMSIS_DIR)/CM1/CoreSupport \
           			$(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x \
           			$(USBFS_DIR)/inc \
           			$(ROOT)/src/main/vcp

VPATH :=	$(VPATH) \
					$(USBFS_DIR)/src

DEVICE_STDPERIPH_SRC	:=	$(DEVICE_STDPERIPH_SRC) \
          								$(USBPERIPH_SRC)

LD_SCRIPT = $(LINKER_DIR)/stm32_flash_f303_$(FLASH_SIZE)k.ld

ARCH_FLAGS = -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -Wdouble-promotion
DEVICE_FLAGS = -DSTM32F303xC -DSTM32F303
TARGET_FLAGS = -D$(TARGET)

endif

ifneq ($(FLASH_SIZE),)
DEVICE_FLAGS := $(DEVICE_FLAGS) -DFLASH_SIZE=$(FLASH_SIZE)
endif

TARGET_DIR = $(ROOT)/src/main/target/$(TARGET)
TARGET_SRC = $(notdir $(wildcard $(TARGET_DIR)/*.c))



INCLUDE_DIRS	:=	$(INCLUDE_DIRS) \
		    					$(TARGET_DIR)

VPATH	:=	$(VPATH) \
					$(TARGET_DIR)

MAIN_COMMON = common/maths.cpp \
		   				common/printf.cpp \
		   				common/typeconversion.cpp \
		   				common/encoding.cpp \

MAIN_FLIGHT = flight/altitudehold.cpp \
		   				flight/failsafe.cpp \
		   				flight/pid.cpp \
		   				flight/imu.cpp \
		   				flight/mixer.cpp \
		   				flight/lowpass.cpp \
		   				flight/filter.cpp \
		   				flight/navigation.cpp \
		   				flight/gps_conversion.c \
		   				flight/motor.cpp \

MAIN_CONFIG = config/config.cpp \
		   				config/runtime_config.cpp 

MAIN_DRIVERS =	drivers/bus_i2c_soft.cpp \
		   					drivers/serial.cpp\
		   					drivers/sound_beeper.c \
		   					drivers/system.c \

MAIN_IO = io/beeper.cpp \
       		io/oled_display.c \
		   		io/rc_controls.cpp \
		   		io/rc_curves.cpp \
		   		io/serial.cpp \
		   		io/serial_1wire.cpp \
		   		io/serial_cli.cpp \
		   		io/serial_msp.cpp \
		   		io/statusindicator.cpp \
		   		io/flashfs.cpp \
		   		io/gps.cpp \

MAIN_RX = rx/rx.cpp \
		   		rx/pwm.c \
		   		rx/msp.c \
		   		rx/sbus.c \
		   		rx/sumd.c \
		   		rx/sumh.c \
		   		rx/spektrum.c \
		   		rx/xbus.cpp \

MAIN_SENSOR = sensors/acceleration.cpp \
		   				sensors/battery.cpp \
		   				sensors/boardalignment.cpp \
		   				sensors/compass.cpp \
		   				sensors/gyro.cpp \
		   				sensors/initialisation.cpp \

MAIN_BLACKBOX = blackbox/blackbox.cpp \
		   					blackbox/blackbox_io.cpp \

COMMON_SRC = 	build_config.cpp \
		   				debug.cpp \
		   				version.cpp \
		   				main.cpp \
		   				mw.cpp \
							$(TARGET_SRC) \
		   				$(MAIN_CONFIG) \
		   				$(MAIN_COMMON) \
		   				$(MAIN_FLIGHT) \
		   				$(MAIN_DRIVERS) \
		   				$(MAIN_IO) \
		   				$(MAIN_RX) \
		   				$(MAIN_SENSOR) \
		   				$(MAIN_BLACKBOX) \
		   				$(CMSIS_SRC) \
		   				$(DEVICE_STDPERIPH_SRC) \

HIGHEND_SRC = \
		   flight/gtune.c \
		   flight/navigation.c \
		   flight/gps_conversion.c \
		   common/colorconversion.c \
		   io/gps.c \
		   io/ledstrip.c \
		   io/display.c \
		   telemetry/telemetry.c \
		   telemetry/frsky.c \
		   telemetry/hott.c \
		   telemetry/msp.c \
		   telemetry/smartport.c \
		   sensors/sonar.c \
		   sensors/barometer.c \
		   blackbox/blackbox.c \
		   blackbox/blackbox_io.c

VCP_SRC = \
		   vcp/hw_config.c \
		   vcp/stm32_it.c \
		   vcp/usb_desc.c \
		   vcp/usb_endp.c \
		   vcp/usb_istr.c \
		   vcp/usb_prop.c \
		   vcp/usb_pwr.c \
		   drivers/serial_usb_vcp.c

DRONA_FLIGHT = 	flight/acrobats.cpp \
								flight/posControl.cpp \
            		flight/posEstimate.cpp \
            		flight/opticflow.cpp \

DRONA_DRIVERS = drivers/opticflow_paw3903.cpp \
								drivers/paw3903_opticflow.cpp \
								drivers/display_ug2864hsweg01 \
            		drivers/ranging_vl53l0x.cpp \
            		drivers/ranging_vl53l1x.cpp \
            		drivers/sc18is602b.cpp \
            		drivers/bridge_sc18is602b.cpp \

DRONA_COMMAND = command/command.cpp \
            		command/localisationCommand.cpp \

DRONA_API =	API/Specifiers.cpp \
						API-Src/Status-LED.cpp \
		    		API-Src/Peripheral-ADC.cpp \
		    		API-Src/Peripheral-PWM.cpp \
		    		API-Src/Peripheral-GPIO.cpp \
		    		API-Src/Serial-IO-Uart.cpp \
		    		API-Src/Serial-IO-I2C.cpp \
		    		API-Src/Serial-IO-SPI.cpp \
						API-Src/BMS.cpp \
						API-Src/FC-Config-PID.cpp \
						API-Src/FC-Config-Setpoints.cpp \
						API-Src/FC-Control-Status.cpp \
						API-Src/FC-Control-Command.cpp \
						API-Src/FC-Data-Sensors.cpp \
						API-Src/FC-Data-Estimate.cpp \
						API-Src/RC-Interface.cpp \
						API-Src/Scheduler-Timer.cpp \
						API-Src/Debugging.cpp \
		    		API-Src/XRanging.cpp \
						API-Src/Motor.cpp \
						API-Src/API-Utils.cpp \
						API-Src/RxConfig.cpp \
						API-Src/Localisation.cpp \

DRONA_SRC = $(DRONA_FLIGHT) \
						$(DRONA_DRIVERS) \
						$(DRONA_COMMAND) \
						$(DRONA_API) \

PRIMUSX2_DRIVERS = 	drivers/adc.cpp \
		   							drivers/adc_stm32f30x.c \
		   							drivers/accgyro_mpu.cpp \
		   							drivers/accgyro_icm20948.cpp \
		   							drivers/bus_i2c_stm32f30x.c \
		   							drivers/bus_spi.c \
		   							drivers/compass_ak09916.cpp \
		   							drivers/gpio_stm32f30x.c \
		   							drivers/light_led_stm32f30x.c \
		   							drivers/flash_m25p16.cpp \
		   							drivers/pwm_mapping.cpp \
			 							drivers/ina219.cpp \
		   							drivers/pwm_output.cpp \
		   							drivers/pwm_rx.cpp \
		   							drivers/serial_uart.c \
		   							drivers/serial_uart_stm32f30x.c \
		   							drivers/sound_beeper_stm32f30x.c \
		   							drivers/system_stm32f30x.c \
		   							drivers/timer.cpp \
		   							drivers/timer_stm32f30x.c \
		   							drivers/barometer_icp10111.cpp 

PRIMUSX2_SENSORS = 	sensors/barometer.cpp \

PRIMUSX2_SRC = 	startup_stm32f30x_md_gcc.S \
		  					$(COMMON_SRC) \
      					$(DRONA_SRC) \
		  					$(RANGING_SRC) \
		  					$(RANGING_SRC2) \
		  					$(PRIMUSX2_DRIVERS) \
		  					$(PRIMUSX2_SENSORS) \

PRIMUS_X2_v1_SRC = 	startup_stm32f30x_md_gcc.S \
		  					$(COMMON_SRC) \
      					$(DRONA_SRC) \
		  					$(RANGING_SRC) \
		  					$(RANGING_SRC2) \
		  					$(PRIMUSX2_DRIVERS) \
		  					$(PRIMUSX2_SENSORS) \

PRIMUS_V5_SRC = 	startup_stm32f30x_md_gcc.S \
		  					$(COMMON_SRC) \
      					$(DRONA_SRC) \
		  					$(RANGING_SRC) \
		  					$(RANGING_SRC2) \
		  					$(PRIMUSX2_DRIVERS) \
		  					$(PRIMUSX2_SENSORS) \

ifeq ($(BUILD_TYPE),BIN)
$(TARGET)_SRC:=$($(TARGET)_SRC)\
			PlutoPilot.cpp
endif               

# Search path and source files for the ST stdperiph library
VPATH	:=	$(VPATH) \
					$(STDPERIPH_DIR)/src

###############################################################################
# Compiler and Tools
CC = arm-none-eabi-g++
C = arm-none-eabi-gcc
AR = arm-none-eabi-ar
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size


ifeq ($(DEBUG),GDB)
OPTIMIZE	=	-O0
LTO_FLAGS	=	$(OPTIMIZE)
else
OPTIMIZE	=	-Os
LTO_FLAGS	=	-flto --use-linker-plugin $(OPTIMIZE)
endif

DEBUG_FLAGS	 = -ggdb3 -DDEBUG

CFLAGS	=	$(ARCH_FLAGS) \
		   		$(LTO_FLAGS) \
		   		$(addprefix -D,$(OPTIONS)) \
		   		$(addprefix -I,$(INCLUDE_DIRS)) \
		   		$(DEBUG_FLAGS) \
		   		-std=gnu17 \
		   		-Wall -Wextra -Wunsafe-loop-optimizations -Wdouble-promotion \
					-Wshadow -Wundef -Wconversion -Wsign-conversion \
		   		-ffunction-sections \
		   		-fdata-sections \
		   		-ffat-lto-objects\
		   		$(DEVICE_FLAGS) \
		   		-DUSE_STDPERIPH_DRIVER \
		   		$(TARGET_FLAGS) \
		   		-save-temps=obj \
		   		-MMD -MP


CCFLAGS	=	$(ARCH_FLAGS) \
		   		$(LTO_FLAGS) \
		   		$(addprefix -D,$(OPTIONS)) \
		   		$(addprefix -I,$(INCLUDE_DIRS)) \
		   		$(DEBUG_FLAGS) \
		   		-std=gnu++17 \
		   		-Wall -Wextra -Wunsafe-loop-optimizations -Wdouble-promotion \
					-Wshadow -Wundef -Wconversion -Wsign-conversion \
		   		-ffunction-sections \
		   		-fdata-sections \
		   		-ffat-lto-objects\
		   		$(DEVICE_FLAGS) \
		   		-DUSE_STDPERIPH_DRIVER \
		   		$(TARGET_FLAGS) \
		   		-save-temps=obj \
		   		-MMD -MP

ASFLAGS	= $(ARCH_FLAGS) \
		   		-x assembler-with-cpp \
		   		$(addprefix -I,$(INCLUDE_DIRS)) \
		  		-MMD -MP

LDFLAGS	= -lm \
		   		-nostartfiles \
		   		--specs=nosys.specs \
		   		-lc \
		   		-lnosys \
		   		$(ARCH_FLAGS) \
		   		$(LTO_FLAGS) \
		   		$(DEBUG_FLAGS) \
		   		-static \
		   		-Wl,-gc-sections,-Map,$(TARGET_MAP) \
		   		-Wl,-L$(LINKER_DIR) \
		   		-T$(LD_SCRIPT)

###############################################################################
# No user-serviceable parts below
###############################################################################

CPPCHECK = 	cppcheck $(CSOURCES) --enable=all --platform=unix64 \
		   			--std=c99 --inline-suppr --quiet --force \
		   			$(addprefix -I,$(INCLUDE_DIRS)) \
		   			-I/usr/include -I/usr/include/linux


## all         : default task; compile C code, build firmware

ifeq ($(BUILD_TYPE),BIN)
all: binary
else 
all: libcreate
endif
#
# Things we will build
#
ifeq ($(filter $(TARGET),$(VALID_TARGETS)),)
$(error Target '$(TARGET)' is not valid, must be one of $(VALID_TARGETS))
endif

TARGET_BIN	 = $(BUILD_DIR)/$(TARGET)/$(FORKNAME)_$(TARGET).bin
TARGET_HEX	 = $(BUILD_DIR)/$(TARGET)/$(PROJECT)_$(TARGET)_$(FW_Version).hex
TARGET_ELF	 = $(BUILD_DIR)/$(TARGET)/$(FORKNAME)_$(TARGET).elf
TARGET_MAP	 = $(BUILD_DIR)/$(TARGET)/$(FORKNAME)_$(TARGET).map
TARGET_OBJS	 = $(addsuffix .o,$(addprefix $(BUILD_DIR)/$(TARGET)/bin/,$(basename $($(TARGET)_SRC))))
TARGET_DEPS	 = $(addsuffix .d,$(addprefix $(BUILD_DIR)/$(TARGET)/bin/,$(basename $($(TARGET)_SRC))))

# List of buildable ELF files and their object dependencies.
# It would be nice to compute these lists, but that seems to be just beyond make.

$(TARGET_HEX): $(TARGET_ELF)
	$(OBJCOPY) -O ihex --set-start 0x8000000 $< $@

$(TARGET_BIN): $(TARGET_ELF)
	$(OBJCOPY) -O binary $< $@

$(TARGET_ELF):  $(TARGET_OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)
	$(SIZE) $(TARGET_ELF) 

# Compile


libs/libpluto_$(LIB_MAJOR_VERSION).$(LIB_MINOR_VERSION).a: $(TARGET_OBJS)
	mkdir -p $(dir $@)
	$(AR) rcs $@ $^


$(BUILD_DIR)/$(TARGET)/bin/%.o: %.cpp
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(CCFLAGS) $<


$(BUILD_DIR)/$(TARGET)/bin/%.o: %.c
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(C) -c -o $@ $(CFLAGS) $<

# Assemble
$(BUILD_DIR)/$(TARGET)/bin/%.o: %.s
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(ASFLAGS) $<

$(BUILD_DIR)/$(TARGET)/bin/%.o: %.S
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(ASFLAGS) $<


libcreate: libs/libpluto_$(LIB_MAJOR_VERSION).$(LIB_MINOR_VERSION).a

## clean       : clean up all temporary / machine-generated files
clean:
	rm -f $(TARGET_BIN) $(TARGET_HEX) $(TARGET_ELF) $(TARGET_OBJS) $(TARGET_MAP)
	rm -rf $(BUILD_DIR)/$(TARGET)
	cd src/test && $(MAKE) clean || true

flash_$(TARGET): $(TARGET_HEX)
	stty -F $(SERIAL_DEVICE) raw speed 115200 -crtscts cs8 -parenb -cstopb -ixon
	echo -n 'R' >$(SERIAL_DEVICE)
	stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 115200 $(SERIAL_DEVICE)

## flash       : flash firmware (.hex) onto flight controller
flash: flash_$(TARGET)

st-flash_$(TARGET): $(TARGET_BIN)
	st-flash --reset write $< 0x08000000

## st-flash    : flash firmware (.bin) onto flight controller
st-flash: st-flash_$(TARGET)

binary: $(TARGET_HEX)

unbrick_$(TARGET): $(TARGET_HEX)
	stty -F $(SERIAL_DEVICE) raw speed 115200 -crtscts cs8 -parenb -cstopb -ixon
	stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 115200 $(SERIAL_DEVICE)

## unbrick     : unbrick flight controller
unbrick: unbrick_$(TARGET)

## cppcheck    : run static analysis on C source code
cppcheck: $(CSOURCES)
	$(CPPCHECK)

cppcheck-result.xml: $(CSOURCES)
	$(CPPCHECK) --xml-version=2 2> cppcheck-result.xml

## help        : print this help message and exit
help: Makefile
	@echo ""
	@echo "Makefile for the $(FORKNAME) firmware"
	@echo ""
	@echo "Usage:"
	@echo "        make [TARGET=<target>] [OPTIONS=\"<options>\"]"
	@echo ""
	@echo "Valid TARGET values are: $(VALID_TARGETS)"
	@echo ""
	@sed -n 's/^## //p' $<

## test        : run the cleanflight test suite
test:
	cd src/test && $(MAKE) test || true

# rebuild everything when makefile changes
$(TARGET_OBJS) : Makefile

# include auto-generated dependencies
-include $(TARGET_DEPS)
