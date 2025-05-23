###############################################################################
# "THE BEER-WARE LICENSE" (Revision 42):
# <msmith@FreeBSD.ORG> wrote this file. As long as you retain this notice you
# can do whatever you want with this stuff. If we meet some day, and you think
# this stuff is worth it, you can buy me a beer in return
###############################################################################
#
# Makefile for building the cleanflight firmware.
#
# Invoke this with 'make help' to see the list of supported targets.
#

###############################################################################
# Things that the user might override on the commandline
#
FORKNAME			 = MAGISV2
# The target to build, see VALID_TARGETS below
TARGET		?= PRIMUSX2

BUILD_TYPE  ?= BIN

LIB_MAJOR_VERSION?= 1

LIB_MINOR_VERSION?= 1

FW_Version = 1.0.0

API_Version = 1.0.1

# Compile-time options
OPTIONS		?= '__FORKNAME__="$(FORKNAME)"' \
		   '__TARGET__="$(TARGET)"' \
		   '__FW_VER__="$(FW_Version)"' \
		   '__API_VER__="$(API_Version)"' \
       '__BUILD_DATE__="$(shell date +%Y-%m-%d)"' \
       '__BUILD_TIME__="$(shell date +%H:%M:%S)"' \

# Debugger optons, must be empty or GDB
DEBUG ?=

# Serial port/Device for flashing
SERIAL_DEVICE	?= $(firstword $(wildcard /dev/ttyUSB*) no-port-found)

# Flash size (KB).  Some low-end chips actually have more flash than advertised, use this to override.
FLASH_SIZE ?=

###############################################################################
# Things that need to be maintained as the source changes
#



VALID_TARGETS	 = ALIENWIIF1 ALIENWIIF3 PRIMUSX PRIMUSX2 CC3D CHEBUZZF3 CJMCU PRIMUSV3R COLIBRI_RACE EUSTM32F103RC MOTOLAB NAZE NAZE32PRO OLIMEXINO PORT103R RMDO SPARKY SPRACINGF3 STM32F3DISCOVERY 

# Configure default flash sizes for the targets
ifeq ($(FLASH_SIZE),)
ifeq ($(TARGET),$(filter $(TARGET),CJMCU))
FLASH_SIZE = 64
else ifeq ($(TARGET),$(filter $(TARGET),ALIENWIIF1  PRIMUSV3R  CC3D NAZE OLIMEXINO RMDO))
FLASH_SIZE = 128
else ifeq ($(TARGET),$(filter $(TARGET),ALIENWIIF3 PRIMUSX PRIMUSX2 CHEBUZZF3 COLIBRI_RACE EUSTM32F103RC MOTOLAB NAZE32PRO PORT103R SPARKY SPRACINGF3 STM32F3DISCOVERY))
FLASH_SIZE = 256
else
$(error FLASH_SIZE not configured for target)
endif
endif

#REVISION = $(shell git log -1 --format="%h")


# Working directories
ROOT		 := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))
SRC_DIR		 = $(ROOT)/src/main
OBJECT_DIR	 = $(ROOT)/obj/main
BIN_DIR		 = $(ROOT)/obj
CMSIS_DIR	 = $(ROOT)/lib/main/CMSIS
INCLUDE_DIRS	 = $(SRC_DIR)
LINKER_DIR	 = $(ROOT)/src/main/target



# Search path for sources
VPATH		:= $(SRC_DIR):$(SRC_DIR)/startup
USBFS_DIR	= $(ROOT)/lib/main/STM32_USB-FS-Device_Driver
USBPERIPH_SRC = $(notdir $(wildcard $(USBFS_DIR)/src/*.c))


# Ranging sensor VL53L0X libraries
RANGING_DIR  = $(ROOT)/lib/main/VL53L0X_API
RANGING_SRC = $(notdir $(wildcard $(RANGING_DIR)/core/src/*.c \
								$(RANGING_DIR)/platform/src/*.c\
								$(RANGING_DIR)/core/src/*.cpp \
								$(RANGING_DIR)/platform/src/*.cpp))
INCLUDE_DIRS:=$(INCLUDE_DIRS) \
              $(RANGING_DIR)/core/inc \
              $(RANGING_DIR)/platform/inc   
VPATH		:= $(VPATH):$(RANGING_DIR)/core/src:$(RANGING_DIR)/platform/src


CSOURCES        := $(shell find $(SRC_DIR) -name '*.c')

ifeq ($(TARGET),$(filter $(TARGET),ALIENWIIF3 PRIMUSX PRIMUSX2 CHEBUZZF3 COLIBRI_RACE MOTOLAB NAZE32PRO RMDO SPARKY SPRACINGF3 STM32F3DISCOVERY))

STDPERIPH_DIR	= $(ROOT)/lib/main/STM32F30x_StdPeriph_Driver

STDPERIPH_SRC = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))

EXCLUDES	= stm32f30x_crc.c \
		stm32f30x_can.c

STDPERIPH_SRC := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))

DEVICE_STDPERIPH_SRC = \
		$(STDPERIPH_SRC)


VPATH		:= $(VPATH):$(CMSIS_DIR)/CM1/CoreSupport:$(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x
CMSIS_SRC	 = $(notdir $(wildcard $(CMSIS_DIR)/CM1/CoreSupport/*.c \
			   $(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x/*.c))

INCLUDE_DIRS := $(INCLUDE_DIRS) \
		   $(STDPERIPH_DIR)/inc \
		   $(CMSIS_DIR)/CM1/CoreSupport \
		   $(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x

ifneq ($(TARGET),SPRACINGF3)
INCLUDE_DIRS := $(INCLUDE_DIRS) \
		   $(USBFS_DIR)/inc \
		   $(ROOT)/src/main/vcp

VPATH := $(VPATH):$(USBFS_DIR)/src

DEVICE_STDPERIPH_SRC := $(DEVICE_STDPERIPH_SRC)\
		   $(USBPERIPH_SRC) 

endif

LD_SCRIPT	 = $(LINKER_DIR)/stm32_flash_f303_$(FLASH_SIZE)k.ld

ARCH_FLAGS	 = -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -Wdouble-promotion
DEVICE_FLAGS = -DSTM32F303xC -DSTM32F303
TARGET_FLAGS = -D$(TARGET)
ifeq ($(TARGET),CHEBUZZF3)
# CHEBUZZ is a VARIANT of STM32F3DISCOVERY
TARGET_FLAGS := $(TARGET_FLAGS) -DSTM32F3DISCOVERY
endif

ifeq ($(TARGET),RMDO)
# RMDO is a VARIANT of SPRACINGF3
TARGET_FLAGS := $(TARGET_FLAGS) -DSPRACINGF3
endif

else ifeq ($(TARGET),$(filter $(TARGET),EUSTM32F103RC PORT103R))


STDPERIPH_DIR	 = $(ROOT)/lib/main/STM32F10x_StdPeriph_Driver

STDPERIPH_SRC = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))

EXCLUDES	= stm32f10x_crc.c \
		stm32f10x_cec.c \
		stm32f10x_can.c

STDPERIPH_SRC := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))

# Search path and source files for the CMSIS sources
VPATH		:= $(VPATH):$(CMSIS_DIR)/CM3/CoreSupport:$(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x
CMSIS_SRC	 = $(notdir $(wildcard $(CMSIS_DIR)/CM3/CoreSupport/*.c \
			   $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x/*.c))

INCLUDE_DIRS := $(INCLUDE_DIRS) \
		   $(STDPERIPH_DIR)/inc \
		   $(CMSIS_DIR)/CM3/CoreSupport \
		   $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x \

LD_SCRIPT	 = $(LINKER_DIR)/stm32_flash_f103_$(FLASH_SIZE)k.ld

ARCH_FLAGS	 = -mthumb -mcpu=cortex-m3
TARGET_FLAGS = -D$(TARGET) -pedantic
DEVICE_FLAGS = -DSTM32F10X_HD -DSTM32F10X

DEVICE_STDPERIPH_SRC = $(STDPERIPH_SRC)

else

STDPERIPH_DIR	 = $(ROOT)/lib/main/STM32F10x_StdPeriph_Driver

STDPERIPH_SRC = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))

EXCLUDES	= stm32f10x_crc.c \
		stm32f10x_cec.c \
		stm32f10x_can.c

STDPERIPH_SRC := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))

# Search path and source files for the CMSIS sources
VPATH		:= $(VPATH):$(CMSIS_DIR)/CM3/CoreSupport:$(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x
CMSIS_SRC	 = $(notdir $(wildcard $(CMSIS_DIR)/CM3/CoreSupport/*.c \
			   $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x/*.c))

INCLUDE_DIRS := $(INCLUDE_DIRS) \
		   $(STDPERIPH_DIR)/inc \
		   $(CMSIS_DIR)/CM3/CoreSupport \
		   $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x \

DEVICE_STDPERIPH_SRC = $(STDPERIPH_SRC)

ifeq ($(TARGET),CC3D)
INCLUDE_DIRS := $(INCLUDE_DIRS) \
		   $(USBFS_DIR)/inc \
		   $(ROOT)/src/main/vcp

VPATH := $(VPATH):$(USBFS_DIR)/src

DEVICE_STDPERIPH_SRC := $(DEVICE_STDPERIPH_SRC) \
		   $(USBPERIPH_SRC) 

endif

LD_SCRIPT	 = $(LINKER_DIR)/stm32_flash_f103_$(FLASH_SIZE)k.ld

ARCH_FLAGS	 = -mthumb -mcpu=cortex-m3
TARGET_FLAGS = -D$(TARGET) -pedantic
DEVICE_FLAGS = -DSTM32F10X_MD -DSTM32F10X

endif

ifneq ($(FLASH_SIZE),)
DEVICE_FLAGS := $(DEVICE_FLAGS) -DFLASH_SIZE=$(FLASH_SIZE)
endif

TARGET_DIR = $(ROOT)/src/main/target/$(TARGET)
TARGET_SRC = $(notdir $(wildcard $(TARGET_DIR)/*.c))

ifeq ($(TARGET),ALIENWIIF1)
# ALIENWIIF1 is a VARIANT of NAZE
TARGET_FLAGS := $(TARGET_FLAGS) -DNAZE -DALIENWII32
TARGET_DIR = $(ROOT)/src/main/target/NAZE
endif

INCLUDE_DIRS := $(INCLUDE_DIRS) \
		    $(TARGET_DIR)

VPATH		:= $(VPATH):$(TARGET_DIR)

COMMON_SRC = build_config.cpp \
		   debug.cpp \
		   version.cpp \
		   $(TARGET_SRC) \
		   config/config.cpp \
		   config/runtime_config.cpp \
		   common/maths.cpp \
		   common/printf.cpp \
		   common/typeconversion.cpp \
		   common/encoding.cpp \
		   main.cpp \
		   mw.cpp \
		   flight/altitudehold.cpp \
		   flight/failsafe.cpp \
		   flight/pid.cpp \
		   flight/imu.cpp \
		   flight/mixer.cpp \
		   flight/lowpass.cpp \
		   flight/filter.cpp \
		   flight/navigation.cpp\
		   flight/gps_conversion.c\
		   flight/motor.cpp\
		   drivers/bus_i2c_soft.cpp \
		   drivers/serial.cpp\
		   drivers/sound_beeper.c \
		   drivers/system.c \
		   io/beeper.cpp \
       io/oled_display.c \
		   io/rc_controls.cpp \
		   io/rc_curves.cpp \
		   io/serial.cpp \
		   io/serial_1wire.cpp \
		   io/serial_cli.cpp \
		   io/serial_msp.cpp \
		   io/statusindicator.cpp \
		   io/flashfs.cpp \
		   io/gps.cpp\
		   rx/rx.cpp\
		   rx/pwm.c \
		   rx/msp.c \
		   rx/sbus.c \
		   rx/sumd.c \
		   rx/sumh.c \
		   rx/spektrum.c \
		   rx/xbus.cpp \
		   sensors/acceleration.cpp \
		   sensors/battery.cpp \
		   sensors/power.cp \
		   sensors/boardalignment.cpp \
		   sensors/compass.cpp \
		   sensors/gyro.cpp \
		   sensors/initialisation.cpp \
		   blackbox/blackbox.cpp \
		   blackbox/blackbox_io.cpp \
		   $(CMSIS_SRC) \
		   $(DEVICE_STDPERIPH_SRC)

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

NAZE_SRC = startup_stm32f10x_md_gcc.S \
		   drivers/accgyro_adxl345.c \
		   drivers/accgyro_bma280.c \
		   drivers/accgyro_l3g4200d.c \
		   drivers/accgyro_mma845x.c \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_mpu3050.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/accgyro_mpu6500.c \
		   drivers/accgyro_spi_mpu6500.c \
		   drivers/adc.c \
		   drivers/adc_stm32f10x.c \
		   drivers/barometer_bmp085.c \
		   drivers/barometer_ms5611.c \
		   drivers/barometer_bmp280.c \
		   drivers/bus_spi.c \
		   drivers/bus_i2c_stm32f10x.c \
		   drivers/compass_hmc5883l.c \
		   drivers/display_ug2864hsweg01.h \
		   drivers/flash_m25p16.c \
		   drivers/gpio_stm32f10x.c \
		   drivers/inverter.c \
		   drivers/light_led_stm32f10x.c \
		   drivers/light_ws2811strip.c \
		   drivers/light_ws2811strip_stm32f10x.c \
		   drivers/sonar_hcsr04.c \
		   drivers/pwm_mapping.c \
		   drivers/pwm_output.c \
		   drivers/pwm_rx.c \
		   drivers/serial_softserial.c \
		   drivers/serial_uart.c \
		   drivers/serial_uart_stm32f10x.c \
		   drivers/sound_beeper_stm32f10x.c \
		   drivers/system_stm32f10x.c \
		   drivers/timer.c \
		   drivers/timer_stm32f10x.c \
		   io/flashfs.c \
		   hardware_revision.c \
		   $(HIGHEND_SRC) \
		   $(COMMON_SRC)

ALIENWIIF1_SRC = $(NAZE_SRC)

EUSTM32F103RC_SRC = startup_stm32f10x_hd_gcc.S \
		   drivers/accgyro_adxl345.c \
		   drivers/accgyro_bma280.c \
		   drivers/accgyro_l3g4200d.c \
		   drivers/accgyro_mma845x.c \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_mpu3050.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/accgyro_spi_mpu6000.c \
		   drivers/accgyro_spi_mpu6500.c \
		   drivers/adc.c \
		   drivers/adc_stm32f10x.c \
		   drivers/barometer_bmp085.c \
		   drivers/barometer_ms5611.c \
		   drivers/bus_i2c_stm32f10x.c \
		   drivers/bus_spi.c \
		   drivers/compass_ak8975.c \
		   drivers/compass_hmc5883l.c \
		   drivers/display_ug2864hsweg01.c \
		   drivers/flash_m25p16.c \
		   drivers/gpio_stm32f10x.c \
		   drivers/inverter.c \
		   drivers/light_led_stm32f10x.c \
		   drivers/light_ws2811strip.c \
		   drivers/light_ws2811strip_stm32f10x.c \
		   drivers/pwm_mapping.c \
		   drivers/pwm_output.c \
		   drivers/pwm_rx.c \
		   drivers/serial_softserial.c \
		   drivers/serial_uart.c \
		   drivers/serial_uart_stm32f10x.c \
		   drivers/sonar_hcsr04.c \
		   drivers/sound_beeper_stm32f10x.c \
		   drivers/system_stm32f10x.c \
		   drivers/timer.c \
		   drivers/timer_stm32f10x.c \
		   io/flashfs.c \
		   $(HIGHEND_SRC) \
		   $(COMMON_SRC)

PORT103R_SRC = $(EUSTM32F103RC_SRC)

OLIMEXINO_SRC = startup_stm32f10x_md_gcc.S \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/adc.c \
		   drivers/adc_stm32f10x.c \
		   drivers/barometer_bmp085.c \
		   drivers/bus_i2c_stm32f10x.c \
		   drivers/bus_spi.c \
		   drivers/compass_hmc5883l.c \
		   drivers/gpio_stm32f10x.c \
		   drivers/light_led_stm32f10x.c \
		   drivers/light_ws2811strip.c \
		   drivers/light_ws2811strip_stm32f10x.c \
		   drivers/pwm_mapping.c \
		   drivers/pwm_output.c \
		   drivers/pwm_rx.c \
		   drivers/serial_softserial.c \
		   drivers/serial_uart.c \
		   drivers/serial_uart_stm32f10x.c \
		   drivers/sonar_hcsr04.c \
		   drivers/sound_beeper_stm32f10x.c \
		   drivers/system_stm32f10x.c \
		   drivers/timer.c \
		   drivers/timer_stm32f10x.c \
		   $(HIGHEND_SRC) \
		   $(COMMON_SRC)


DRONA_SRC = flight/acrobats.cpp \
            drivers/opticflow_paw3903.cpp \
						drivers/display_ug2864hsweg01 \
            drivers/ranging_vl53l0x.cpp \
            drivers/sc18is602b.cpp\
            flight/posControl.cpp\
            flight/posEstimate.cpp\
            flight/opticflow.cpp\
            command/command.cpp\
            command/localisationCommand.cpp\
            API/Specifiers.cpp \
		    API/Peripheral.cpp \
		    API/XRanging.cpp \
			API/Sensor.cpp \
			API/Control.cpp \
			API/Estimate.cpp \
			API/Utils.cpp\
			API/User.cpp\
			API/Motor.cpp\
			API/API-Utils.cpp\
			API/RxConfig.cpp\
			API/Localisation.cpp\


		    
PRIMUSV3R_SRC = \
		   startup_stm32f10x_md_gcc.S \
		   $(RANGING_SRC) \
		   drivers/adc.cpp \
		   drivers/adc_stm32f10x.cpp \
		   drivers/accgyro_mpu.cpp \
		   drivers/accgyro_mpu6500.cpp \
		   drivers/bus_i2c_stm32f10x.c \
		   drivers/bus_spi.c \
		   drivers/compass_ak8963.cpp \
		   drivers/gpio_stm32f10x.cpp \
		   drivers/light_led_stm32f10x.cpp \
		   drivers/flash_m25p16.cpp \
		   drivers/pwm_mapping.cpp \
		   drivers/pwm_output.cpp \
		   drivers/pwm_rx.cpp \
		   drivers/serial_uart.c \
		   drivers/serial_uart_stm32f10x.c \
		   drivers/sound_beeper_stm32f10x.cpp \
		   drivers/system_stm32f10x.cpp \
		   drivers/timer.cpp \
		   drivers/timer_stm32f10x.cpp \
		   hardware_revision.cpp \
		   drivers/barometer_ms5611.cpp \
		   sensors/barometer.cpp \
		   $(COMMON_SRC) \
		   $(DRONA_SRC)		    

PRIMUSX_SRC = \
		   startup_stm32f30x_md_gcc.S \
		   $(RANGING_SRC) \
		   drivers/adc.cpp \
		   drivers/adc_stm32f30x.c \
		   drivers/accgyro_mpu.cpp \
		   drivers/accgyro_mpu6500.cpp \
		   drivers/accgyro_icm20948.cpp \
		   drivers/bus_i2c_stm32f30x.c \
		   drivers/bus_spi.c \
		   drivers/compass_ak8963.cpp \
		   drivers/compass_ak09916.cpp \
		   drivers/gpio_stm32f30x.c \
		   drivers/light_led_stm32f30x.c \
		   drivers/flash_m25p16.cpp \
		   drivers/pwm_mapping.cpp \
		   drivers/pwm_output.cpp \
		   drivers/pwm_rx.cpp \
		   drivers/serial_uart.c \
		   drivers/serial_uart_stm32f30x.c \
		   drivers/sound_beeper_stm32f30x.c \
		   drivers/system_stm32f30x.c \
		   drivers/timer.cpp \
		   drivers/timer_stm32f30x.c \
		   drivers/barometer_ms5611.cpp \
		   drivers/barometer_icp10111.cpp \
		   sensors/barometer.cpp \
		   $(COMMON_SRC) \
           $(DRONA_SRC)


PRIMUSX2_SRC = \
		   startup_stm32f30x_md_gcc.S \
		   $(RANGING_SRC) \
		   drivers/adc.cpp \
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
		   drivers/barometer_icp10111.cpp \
		   sensors/barometer.cpp \
		   $(COMMON_SRC) \
           $(DRONA_SRC)


CJMCU_SRC = \
		   startup_stm32f10x_md_gcc.S \
		   $(RANGING_SRC) \
		   drivers/adc.cpp \
		   drivers/adc_stm32f10x.cpp \
		   drivers/accgyro_mpu.cpp \
		   drivers/accgyro_mpu6500.cpp \
		   drivers/bus_i2c_stm32f10x.c \
		   drivers/bus_spi.c \
		   drivers/compass_ak8963.cpp \
		   drivers/gpio_stm32f10x.cpp \
		   drivers/light_led_stm32f10x.cpp \
			drivers/flash_m25p16.cpp \
		   drivers/pwm_mapping.cpp \
		   drivers/pwm_output.cpp \
		   drivers/pwm_rx.cpp \
		   drivers/serial_uart.c \
		   drivers/serial_uart_stm32f10x.c \
		   drivers/sound_beeper_stm32f10x.cpp \
		   drivers/system_stm32f10x.cpp \
		   drivers/timer.cpp \
		   drivers/timer_stm32f10x.cpp \
		   hardware_revision.cpp \
		   drivers/barometer_ms5611.cpp \
		   sensors/barometer.cpp \
		   $(COMMON_SRC) \
		   $(DRONA_SRC)



ALIENWIIF3_SRC = \
		   startup_stm32f30x_md_gcc.S \
		   $(RANGING_SRC) \
		   drivers/adc.cpp \
		   drivers/adc_stm32f30x.c \
		   drivers/accgyro_mpu.cpp \
		   drivers/accgyro_mpu6500.cpp \
		   drivers/bus_i2c_stm32f30x.c \
		   drivers/bus_spi.c \
		   drivers/compass_ak8963.cpp \
		   drivers/gpio_stm32f30x.c \
		   drivers/light_led_stm32f30x.c \
		   drivers/flash_m25p16.cpp \
		   drivers/pwm_mapping.cpp \
		   drivers/pwm_output.cpp \
		   drivers/pwm_rx.cpp \
		   drivers/serial_uart.c \
		   drivers/serial_uart_stm32f30x.c \
		   drivers/sound_beeper_stm32f30x.c \
		   drivers/system_stm32f30x.c \
		   drivers/timer.cpp \
		   drivers/timer_stm32f30x.c \
		   drivers/barometer_ms5611.cpp \
		   sensors/barometer.cpp \
		   $(COMMON_SRC) \
           $(DRONA_SRC)




CC3D_SRC = \
		   startup_stm32f10x_md_gcc.S \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_spi_mpu6000.c \
		   drivers/adc.c \
		   drivers/adc_stm32f10x.c \
		   drivers/barometer_bmp085.c \
		   drivers/barometer_ms5611.c \
		   drivers/bus_spi.c \
		   drivers/bus_i2c_stm32f10x.c \
		   drivers/compass_hmc5883l.c \
		   drivers/display_ug2864hsweg01.c \
		   drivers/flash_m25p16.c \
		   drivers/gpio_stm32f10x.c \
		   drivers/inverter.c \
		   drivers/light_led_stm32f10x.c \
		   drivers/light_ws2811strip.c \
		   drivers/light_ws2811strip_stm32f10x.c \
		   drivers/pwm_mapping.c \
		   drivers/pwm_output.c \
		   drivers/pwm_rx.c \
		   drivers/serial_softserial.c \
		   drivers/serial_uart.c \
		   drivers/serial_uart_stm32f10x.c \
		   drivers/sonar_hcsr04.c \
		   drivers/sound_beeper_stm32f10x.c \
		   drivers/system_stm32f10x.c \
		   drivers/timer.c \
		   drivers/timer_stm32f10x.c \
		   io/flashfs.c \
		   $(HIGHEND_SRC) \
		   $(COMMON_SRC) \
		   $(VCP_SRC)

STM32F30x_COMMON_SRC = \
		   startup_stm32f30x_md_gcc.S \
		   drivers/adc.c \
		   drivers/adc_stm32f30x.c \
		   drivers/bus_i2c_stm32f30x.c \
		   drivers/bus_spi.c \
		   drivers/gpio_stm32f30x.c \
		   drivers/light_led_stm32f30x.c \
		   drivers/light_ws2811strip.c \
		   drivers/light_ws2811strip_stm32f30x.c \
		   drivers/pwm_mapping.c \
		   drivers/pwm_output.c \
		   drivers/pwm_rx.c \
		   drivers/serial_uart.c \
		   drivers/serial_uart_stm32f30x.c \
		   drivers/sound_beeper_stm32f30x.c \
		   drivers/system_stm32f30x.c \
		   drivers/timer.c \
		   drivers/timer_stm32f30x.c

NAZE32PRO_SRC = \
		   $(STM32F30x_COMMON_SRC) \
		   $(HIGHEND_SRC) \
		   $(COMMON_SRC) \
		   $(VCP_SRC)

STM32F3DISCOVERY_COMMON_SRC = \
		   $(STM32F30x_COMMON_SRC) \
		   drivers/accgyro_l3gd20.c \
		   drivers/accgyro_l3gd20.c \
		   drivers/accgyro_lsm303dlhc.c \
		   drivers/compass_hmc5883l.c \
		   $(VCP_SRC)

STM32F3DISCOVERY_SRC = \
		   $(STM32F3DISCOVERY_COMMON_SRC) \
		   drivers/accgyro_adxl345.c \
		   drivers/accgyro_bma280.c \
		   drivers/accgyro_mma845x.c \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_mpu3050.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/accgyro_l3g4200d.c \
		   drivers/barometer_ms5611.c \
		   drivers/compass_ak8975.c \
		   $(HIGHEND_SRC) \
		   $(COMMON_SRC)

CHEBUZZF3_SRC = \
		   $(STM32F3DISCOVERY_SRC) \
		   $(HIGHEND_SRC) \
		   $(COMMON_SRC)

COLIBRI_RACE_SRC = \
		   $(STM32F30x_COMMON_SRC) \
		   drivers/display_ug2864hsweg01.c \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_mpu6500.c \
		   drivers/accgyro_spi_mpu6500.c \
		   drivers/accgyro_mpu6500.c \
		   drivers/barometer_ms5611.c \
		   drivers/compass_ak8975.c \
		   drivers/compass_hmc5883l.c \
		   drivers/serial_usb_vcp.c \
		   $(HIGHEND_SRC) \
		   $(COMMON_SRC) \
		   $(VCP_SRC)

SPARKY_SRC = \
		   $(STM32F30x_COMMON_SRC) \
		   drivers/display_ug2864hsweg01.c \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/barometer_ms5611.c \
		   drivers/compass_ak8975.c \
		   drivers/serial_usb_vcp.c \
		   $(HIGHEND_SRC) \
		   $(COMMON_SRC) \
		   $(VCP_SRC)

ALIENWIIF4_SRC = \
		   $(SPARKY_SRC)

RMDO_SRC = \
		   $(STM32F30x_COMMON_SRC) \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/barometer_bmp280.c \
		   drivers/display_ug2864hsweg01.h \
		   drivers/flash_m25p16.c \
		   drivers/serial_softserial.c \
		   drivers/sonar_hcsr04.c \
		   io/flashfs.c \
		   $(HIGHEND_SRC) \
		   $(COMMON_SRC)

SPRACINGF3_SRC = \
		   $(STM32F30x_COMMON_SRC) \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/barometer_ms5611.c \
		   drivers/compass_ak8975.c \
		   drivers/compass_hmc5883l.c \
		   drivers/display_ug2864hsweg01.h \
		   drivers/flash_m25p16.c \
		   drivers/serial_softserial.c \
		   drivers/sonar_hcsr04.c \
		   io/flashfs.c \
		   $(HIGHEND_SRC) \
		   $(COMMON_SRC)

MOTOLAB_SRC = \
		   $(STM32F30x_COMMON_SRC) \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/barometer_ms5611.c \
		   drivers/compass_hmc5883l.c \
		   drivers/display_ug2864hsweg01.c \
		   drivers/serial_usb_vcp.c \
		   drivers/flash_m25p16.c \
		   io/flashfs.c \
		   $(HIGHEND_SRC) \
		   $(COMMON_SRC) \
		   $(VCP_SRC)


ifeq ($(BUILD_TYPE),BIN)
$(TARGET)_SRC:=$($(TARGET)_SRC)\
			API/PlutoPilot.cpp
endif               



# Search path and source files for the ST stdperiph library
VPATH		:= $(VPATH):$(STDPERIPH_DIR)/src

###############################################################################
# Things that might need changing to use different tools
#

# Tool names
CC		 =arm-none-eabi-g++
C		 = arm-none-eabi-gcc
AR               =arm-none-eabi-ar
OBJCOPY		 =arm-none-eabi-objcopy
SIZE		 =arm-none-eabi-size

#
# Tool options.
#

ifeq ($(DEBUG),GDB)
OPTIMIZE	 = -O0
LTO_FLAGS	 = $(OPTIMIZE)
else
OPTIMIZE	 = -Os
LTO_FLAGS	 = -flto --use-linker-plugin $(OPTIMIZE)
endif

DEBUG_FLAGS	 = -ggdb3 -DDEBUG

CFLAGS		 = $(ARCH_FLAGS) \
		   $(LTO_FLAGS) \
		   $(addprefix -D,$(OPTIONS)) \
		   $(addprefix -I,$(INCLUDE_DIRS)) \
		   $(DEBUG_FLAGS) \
		   -std=gnu99 \
		   -Wall -Wextra -Wunsafe-loop-optimizations -Wdouble-promotion \
		   -ffunction-sections \
		   -fdata-sections \
		   -ffat-lto-objects\
		   $(DEVICE_FLAGS) \
		   -DUSE_STDPERIPH_DRIVER \
		   $(TARGET_FLAGS) \
		   
		   -save-temps=obj \
		   -MMD -MP


CCFLAGS		 = $(ARCH_FLAGS) \
		   $(LTO_FLAGS) \
		   $(addprefix -D,$(OPTIONS)) \
		   $(addprefix -I,$(INCLUDE_DIRS)) \
		   $(DEBUG_FLAGS) \
		   -std=gnu++98 \
		   -Wall -Wextra -Wunsafe-loop-optimizations -Wdouble-promotion \
		   -ffunction-sections \
		   -fdata-sections \
		   -ffat-lto-objects\
		   $(DEVICE_FLAGS) \
		   -DUSE_STDPERIPH_DRIVER \
		   $(TARGET_FLAGS) \
		  
		   -save-temps=obj \
		   -MMD -MP

ASFLAGS		 = $(ARCH_FLAGS) \
		   -x assembler-with-cpp \
		   $(addprefix -I,$(INCLUDE_DIRS)) \
		  -MMD -MP

LDFLAGS		 = -lm \
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

CPPCHECK         = cppcheck $(CSOURCES) --enable=all --platform=unix64 \
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

TARGET_BIN	 = $(BIN_DIR)/$(FORKNAME)_$(TARGET).bin
TARGET_HEX	 = $(BIN_DIR)/Experience_PLUTOX.hex
TARGET_ELF	 = $(OBJECT_DIR)/$(FORKNAME)_$(TARGET).elf
TARGET_OBJS	 = $(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $($(TARGET)_SRC))))
TARGET_DEPS	 = $(addsuffix .d,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $($(TARGET)_SRC))))
TARGET_MAP	 = $(OBJECT_DIR)/$(FORKNAME)_$(TARGET).map

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


$(OBJECT_DIR)/$(TARGET)/%.o: %.cpp
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(CCFLAGS) $<


$(OBJECT_DIR)/$(TARGET)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(C) -c -o $@ $(CFLAGS) $<

# Assemble
$(OBJECT_DIR)/$(TARGET)/%.o: %.s
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(ASFLAGS) $<

$(OBJECT_DIR)/$(TARGET)/%.o: %.S
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(ASFLAGS) $<


libcreate: libs/libpluto_$(LIB_MAJOR_VERSION).$(LIB_MINOR_VERSION).a

## clean       : clean up all temporary / machine-generated files
clean:
	rm -f $(TARGET_BIN) $(TARGET_HEX) $(TARGET_ELF) $(TARGET_OBJS) $(TARGET_MAP)
	rm -rf $(OBJECT_DIR)/$(TARGET)
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
