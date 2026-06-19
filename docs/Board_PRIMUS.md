# PRIMUS Target Hardware

The **PRIMUS** target (encompassing `PRIMUS_X2_v1`, `PRIMUSX2`, and `PRIMUS_V5`) is the exclusive, modern hardware architecture supported by the MagisV2 firmware. 

## Hardware Specifications
- **MCU**: STM32F303xC ARM Cortex-M4 @ 72MHz
- **IMU**: MPU6000 or ICM20689 connected via high-speed SPI
- **Barometer**: BMP280 or MS5611 connected via I2C (Fast Mode 400kHz)
- **Time-of-Flight**: VL53L0X / VL53L1X supported on I2C
- **Optic Flow**: PAW3903 connected via SPI

## Core Philosophy
The PRIMUS board architecture explicitly rejects legacy hardware bloat. It abandons features like integrated SD card readers (for Blackbox), bulky GPS connectors, and 8-channel PWM receiver headers. Instead, it relies on modern, serialized digital communication (CRSF/SBUS) and tightly integrated onboard sensors.

## Pinout and Layout
Due to the closed-loop nature of the PRIMUS architecture, users do not need to manually assign resource pins in a CLI. The firmware ships with hardcoded, optimized DMA streams and Timer channels mapping directly to the physical STM32 pins on the PRIMUS boards.

### Serial Ports
- **UART1**: Reserved for MultiWii Serial Protocol (MSP) / PlutoPilot API / Ground Control.
- **UART2**: Dedicated Serial RX input (SBUS/CRSF).
- **UART3**: General purpose / Telemetry output.

### I2C / SPI Buses
- **SPI1**: Dedicated solely to the IMU to guarantee 8kHz+ gyro polling without collision.
- **SPI2/3**: Available for Optic Flow or external flash.
- **I2C1**: Master bus for Barometer and Magnetometer (if connected).

## Supported Mixers
The PRIMUS target natively assumes a **Quadcopter X** layout. Support for Tricopters and Hexacopters has been deprecated at the firmware level to save Flash space and optimize matrix math.
