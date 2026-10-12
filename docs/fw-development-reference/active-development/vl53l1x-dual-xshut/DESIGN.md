# VL53L1X Dual XSHUT - Design

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [DESIGN](DESIGN.md)

## Problem

Two VL53L1X ( TOF400C modules ) share I2C1. Both answer at the fixed address 0x29. The bottom sensor's XSHUT
is wired to `GPIO_5` ( PB15 ), the second sensor's to `GPIO_6` ( PB14 ). The firmware has no XSHUT handling:
the STM32 pins are floating inputs at reset, so both sensors boot. Every write reaches both sensors and every
read is a wired-AND of both replies. The result is a wrong NVM calibration in `DataInit` and wrong ranges, with
no I2C error, so `Global_Status_L1` stays 0 and the estimator trusts the bad range ( silent failure ).

Wanted: only the bottom sensor active and used; the second one held in shutdown for now.

## Options

| | (a) XSHUT in the driver, before the probe | (b) Full dual support, one used | (c) User code in `plutoInit` |
|---|---|---|---|
| **How** | `ranging_init_L1` drives the OFF pin LOW and enables the ON pin, waits 2 ms, then `WaitDeviceBooted` as today. Pins from `target.h` defines. | Hold B low, init A, move A to 0x2A, optionally bring B up at 0x29. | `PlutoPilot.cpp` sets the GPIOs. |
| **Files** | `ranging_vl53l1x.cpp`, `PRIMUS_X2_v1/target.h`, `PRIMUS_V5/target.h` | driver + `MyDevice_L1` struct, second device struct | `PlutoPilot.cpp` |
| **Flash / RAM / loop** | ~100 B / 0 / boot only | 0.5-1 KB / ~200 B if B is initialised / boot only | small / 0 / 0 |
| **Safety** | none on ALT_HOLD; missing laser still falls back to baro | same, more boot I2C | **unsafe**: `plutoInit` ( `main.cpp:540` ) runs after `ranging_init_L1` ( `main.cpp:523` ), so the laser is already initialised with mixed calibration |
| **API impact** | none | none | none |
| **Risk** | a later `Peripheral_Init` / XRanging call on PB14/PB15 can undo it ( documented, not guarded ) | more code to test for a sensor that is not used | does not work |

## Choice

**(a)**, 5 Oct 2026. The user wants the second sensor held off only, with the pins given by `target.h` defines.
(b) is a later topic if the second sensor gets a job. (c) runs too late.

Details fixed during planning:

- Defines in `target.h` of **PRIMUS_X2_v1 and PRIMUS_V5** ( names settled in task 2, e.g. `L1X_XSHUT_ON_GPIO`
  / `L1X_XSHUT_OFF_GPIO` ). The driver code is compiled only when they are defined, so a target without them
  is unchanged. On PRIMUS_V5 only the defines are added; `LASER_TOF_L1x` / `LASER_ALT` stay as they are.
- **How the ON pin enables the bottom sensor:** released as a floating input to the module's own pull-up,
  never driven HIGH. Task 1 measured the TOF400C SHUT at 2.7 V with the wire off the FC ( pull-up to its ~2.8 V
  rail ).
- **Boot wait:** `delay ( 2 )` after releasing the bottom sensor ( tBOOT 1.2 ms ). Planning assumed `WaitDeviceBooted`
  polls for up to 500 ms; the task 3 review found the platform wait functions are stubs, so it does not wait at all.
- **Pin conflicts:** XRanging FRONT / LEFT, ADC_2 / ADC_3, PWM_4 / PWM_5, RGB_3, `OPTIC_FLOW`, the user SPI API
  and the M25P16 SPI2 flash all use PB14 / PB15. The flash ( not needed on these boards ) is compiled out on
  PRIMUS_X2_v1 and guarded by an `#error` ( task 10 ); the others are documented only.
- A missing or dead bottom sensor behaves as today: laser not detected, baro-only hold.
