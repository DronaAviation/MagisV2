# Hardware Bus Pipeline (I2C & SPI)

## Overview
The Hardware Bus abstraction layer acts as the physical communication bridge between the STM32F3 microcontroller and the onboard peripherals (IMUs, Barometers, Magnetometers, Time-of-Flight sensors). I2C is **polled**: every transaction busy-waits on the peripheral's status flags ( `I2C_LONG_TIMEOUT`, then a bus recovery on timeout ) and blocks the loop until it finishes. There is no I2C DMA or interrupt. Peripherals on the shared I2C1 bus must therefore budget their bytes per loop.

## Source Files
- **I2C Drivers**: `src/main/drivers/bus_i2c.h`, `src/main/drivers/bus_i2c_stm32f30x.c`, `src/main/drivers/bus_i2c_soft.cpp`
- **SPI Drivers**: `src/main/drivers/bus_spi.h`, `src/main/drivers/bus_spi.c`
- **Peripheral Setup**: `src/main/drivers/system_stm32f30x.c`
- **Laser bus use**: `src/main/drivers/tof_manager.cpp` ( VL53L0X polling ), `src/main/drivers/tof_manager_l1x.cpp`
  ( VL53L1X polling ), `src/main/API-Src/XRanging.cpp` ( `xRangingInit ( )` ) and `src/main/API-Src/X1Ranging.cpp`
  ( `x1RangingInit ( )` ): addresses, SPI2 release; `src/main/drivers/tof_dev_area.cpp` ( device structs )

## Key Data Structures
- `I2CDevice`: Enumerator representing the I2C buses ( `I2CDEV_1`, `I2CDEV_2` ). Every target uses `I2CDEV_1` ( `I2C_DEVICE` in `target.h` ).
- SPI buses are passed as `SPI_TypeDef *` ( `SPI1`, `SPI2` ). The three targets enable `SPI2` only ( `USE_SPI_DEVICE_2` ).
- `i2c1ErrorCount` / `i2c2ErrorCount`: one count of failed transfers per bus ( `i2cGetErrorCounter ( )` ). There is no error type: a NACK, a missing device and a stuck bus all end as a timeout.
- `tofDevArea` ( `drivers/tof_dev_area.cpp` ): `union tofDevArea_u { VL53L0X_Dev_t l0x [ 6 ]; VL53L1_Dev_t l1x [ 6 ]; }`,
  5592 B of `.bss`, the ST device structs of both laser managers. Only one family runs per boot
  ( `rangingFamily ( )` ), so the two tables share the RAM:
  - Each manager uses only its own member.
  - `tofManagerBringUp ( )` / `tofL1xBringUp ( )` return false unless the latched family is theirs.
  - Every later access ( polls, start ) needs the entry's status OK, which only that bring-up sets.
  - The status tables are separate, outside the union. With NONE or BOTH no member is touched.

## Primary Functions
- `i2cInit()` / `spiInit()`: configure the GPIO alternate functions and clock speeds for the bus. I2C uses no DMA. `spiInit ( SPI2 )` returns false once SPI2 is released.
- `i2cRead()` / `i2cWrite()` / `i2cWriteBuffer()`: blocking, polled I2C transactions at 400 kHz
  ( `I2C_Timing` 0x00E0257A ). A byte costs ~25 µs on the wire plus per-transaction overhead.
- `spiTransfer()` / `spiTransferByte()`: synchronous, polled SPI exchange ( TXE / RXNE, 1000-count timeout ). On these targets SPI2 carries the M25P16 flash and the user SPI API; the IMU is on I2C1. On a released SPI2 they return at once ( `false` / 0 ).
- `spiRelease ( SPI2 )` / `spiIsReleased ( )`: switch SPI2 off for the rest of the run ( see below ).

## Data Flow & Boundaries
- **I2C errors ( limitation )**: the driver never reads the NACK flag ( NACKF ). A device that does not acknowledge
  ( absent, held in reset, wrong address ) only leaves the flag the driver waits for unset. The transfer therefore
  costs the full `I2C_LONG_TIMEOUT` ( 10 × 0x1000 polling counts ). Then `i2cTimeoutUserCallback ( )` counts the
  error and re-initialises the I2C1 peripheral ( disable, clear flags, enable, `i2cInitPort ( )` ). The caller gets
  false and cannot tell a NACK from a stuck bus. Each probe of an absent device stalls the loop for one timeout. The
  laser bring-up probes for absent devices at most 7 times ( 6 at 0x29, 1 at 0x30 for a down-laser without an XSHUT wire ), at boot only. A VL53L1X that is enabled but does not answer also costs
  up to 10 timeouts in its boot wait ( `tofL1xWaitBooted ( )` ), at boot only.
- **SPI Constraints**: SPI is polled like I2C. The clock is bounded by the device ( `spiSetDivisor ( )` ).
- **Shared I2C1 budget**: the IMU ( ICM-20948 ), barometer, magnetometer, INA219, the OLED ( 0x3C ) and the lasers
  ( VL53L0X or VL53L1X, one family per boot ) share I2C1 inside the gated loop.
  The OLED never writes the panel from the loop directly. Drawing goes to a 1 KB RAM framebuffer, and
  `oledFlushStep ( )` ( called from `OledDisplayData ( )`, `mw.cpp:1168` ) sends at most **one run of up to 16 changed
  bytes per loop** ( ~0.6 ms ), and only while the frame differs from what the panel shows. The old per-byte
  writes ( ~630 transactions, 45-60 ms every 200 ms ) made the drone twitch at 5 Hz
  ( [oled-flight-wobble](../../active-development/oled-flight-wobble/README.md) ).
- **Laser addresses on I2C1**: every VL53L0X and every VL53L1X boots at 0x29. The bring-up of the latched family
  ( `xRangingInit ( )`, main.cpp:585, or `x1RangingInit ( )`, main.cpp:586 ) moves each laser while it is alone on
  the bus:
  - the down-laser goes to **0x30**;
  - each side laser goes to **0x2A + n**, where n counts the enabled side lasers before it in table order ( LEFT, RIGHT,
    FRONT, BACK, EXTERNAL ).

  With either family, 0x29 is in use only during the bring-up, with one exception. A fitted laser that is not enabled
  stays awake at 0x29, and every release after it is refused ( `BUS_BUSY`: `vl53l0xProbe ( 0x29 )` or
  `tofL1xProbe ( 0x29 )` answers ). The two families cannot share the bus. Both boot at 0x29, the VL53L0X probe cannot
  see a VL53L1X, and the VL53L1X probe's 16-bit index would write a register of a VL53L0X. User code that enables
  lasers through both APIs gets no laser at all ( `RANGING_BOTH` ). A user I2C device must not use 0x29-0x2E or 0x30
  while lasers are fitted.
- **Laser I2C load**: `UPDATE_LASER_TOF_TASK` calls both managers on every pass ( `mw.cpp:850-858` ). The manager of
  the family that is not latched has no laser up and returns without I2C.
  - **VL53L0X:** the down-laser every 33 ms ( start + read ), and one side laser per 10 ms: a data-ready read, plus the
    result read and the interrupt clear only when a sample is ready. The worst laser-task call measured 1.9-2.2 ms
    ( one down read plus one side read ), and the mean was 13 µs
    ( [tof-single-pipeline TESTING.md](../../active-development/tof-single-pipeline/TESTING.md), test 3 ).
  - **VL53L1X:** `tofL1xPollDown ( )` every 10 ms makes a 1-byte data-ready read. Only when a result is ready ( one
    per 50 ms ) does it add the 17-byte result read and a 1-byte interrupt clear. Its first poll also starts the
    measurement. `tofL1xPollSides ( )` serves one side laser per 10 ms the same way, and it skips any pass in which
    the down poll used the bus. So one pass carries at most one VL53L1X's I2C: one data-ready read plus at most one
    result read. A result read costs ~0.79 ms at 400 kHz, as measured on the former single-laser driver, whose read
    path the down poll keeps line for line.

  Readers ( `XRanging.getRange ( )`, `X1Ranging.getRange ( )`, object avoidance, altitude hold ) use the cached
  sample and make no I2C call.
- **Laser errors**: each laser has its own status in its manager. A failed read marks only that laser I2C_ERROR, and it
  is not polled again until the next restart.
- **SPI2 is off whenever XRanging or X1Ranging starts lasers**: SPI2's pins are GPIO_5-8 ( PB15 MOSI, PB14 MISO, PB13
  SCK, PB12 NSS / flash CS ), and both laser APIs use them as XSHUT and LED pins. If any laser of the latched family
  is requested, `xRangingInit ( )` or `x1RangingInit ( )` calls `spiRelease ( SPI2 )` and `m25p16_release ( )` before
  it touches a laser pin:
  - SPI2 is disabled and its clock stopped.
  - Pins still in SPI2's alternate function ( AF5 ) become pulled-up inputs. Pins user code already took in
    `plutoInit ( )` are left alone, and PB12 is not touched.
  - From then on `spiInit ( SPI2 )` fails, SPI2 transfers return at once, the flash reports size 0 and the user SPI
    API does nothing.

  Without lasers, and with both APIs used ( no laser starts ), SPI2 and the flash are unchanged.
- **OLED failure**: if the panel stops answering ( 3 failed runs in a row, or more than ~1 in 5 failing ), the
  OLED is switched off, so a loose connector cannot keep stalling the loop. The 2 s start-up splash still writes
  the panel directly, at boot only.

```mermaid
flowchart TD
    Start([Bus transfer request]) --> BusCheck{Which bus?}

    BusCheck -- SPI2 --> Rel{SPI2 released<br/>by XRanging or X1Ranging?}
    Rel -- Yes --> SPIOff[Return at once<br/>spiTransfer false, spiTransferByte 0]
    Rel -- No --> SPISetup[Device driver drives CS low]
    SPISetup --> SPIXfer[spiTransfer: polled TXE / RXNE]
    SPIXfer --> SPIRelease[Device driver drives CS high]

    BusCheck -- I2C1 --> I2CBusy[Wait for BUSY clear]
    I2CBusy --> I2CStart[I2C_TransferHandling<br/>start, address, read or write]
    I2CStart --> I2CXfer[Wait TXIS / TCR / TC / RXNE<br/>for each byte]
    I2CXfer --> I2CStop[Wait STOPF, clear it]

    I2CBusy -- I2C_LONG_TIMEOUT --> I2CErr[i2cTimeoutUserCallback<br/>error count +1, re-init I2C1]
    I2CXfer -- I2C_LONG_TIMEOUT<br/>also a NACK: NACKF is never read --> I2CErr
    I2CStop -- I2C_LONG_TIMEOUT --> I2CErr

    SPIRelease --> Merge[Data Ready in Buffer]
    I2CStop --> Merge
    I2CErr --> MergeFail[Data Invalid]
    SPIOff --> MergeFail

    Merge --> End([Return Success])
    MergeFail --> EndFail([Return Failure])
```
