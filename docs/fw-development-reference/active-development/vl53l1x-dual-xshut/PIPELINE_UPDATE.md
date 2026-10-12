# PIPELINE_UPDATE: vl53l1x-dual-xshut

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [DESIGN](DESIGN.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

Staged for commit. Applied by `pluto-commit`; do not edit `fw-architecture-pipeline/` before then.

## 1. What this replaces

Target: [`fw-architecture-pipeline/subsystems/Altitude_Hold_Estimator.md`](../../fw-architecture-pipeline/subsystems/Altitude_Hold_Estimator.md).
Each part in section 2 is the full replacement text for one place in that doc; paste it verbatim over the
text named in its heading. Everything else in the doc stays as it is.

| # | Place in `Altitude_Hold_Estimator.md` | Why |
|---|---|---|
| 2.1 | **Source Files**, the `- **Laser**` bullet ( line 9 ) | A second VL53L1X may now be fitted, held in shutdown |
| 2.2 | **Laser fusion**, the first paragraph ( "With a downward laser, altitude hold means ..." ) | "Only one can be fitted" becomes "Only one is used", a sentence on the second VL53L1X is added, "two sensors" becomes "two sensor types" |
| 2.3 | **VL53L1X driver**, the `- **Set-up**` bullet | New XSHUT step and boot wait before the ST init |

No flowchart changes: no Mermaid chart in the doc shows `ranging_init_L1 ( )` or the boot path, and the fusion
flow is unchanged.

No other pipeline doc changes:

- `Main_Initialization.md`: describes boot at the level of clocks, buses and `sensorsAutodetect ( )`; it does
  not list the laser init or `plutoInit ( )`, so there is nothing to correct at its level of detail.
- `Hardware_Bus_Pipeline.md`, `Firmware_Pipeline.md`, `User_Space_API.md`: no bus, task or API change. No
  public API change, so no `docs/API/` update.

Edited directly with this topic ( these files are not in the pipeline folder ):

- `PIN_MAP.md`: PB14 / PB15 rows marked `XSHUT¹`, footnote ¹ under section 1, new conflict 8.
- `TIMER_MAP.md`: `PWM_4/PWM_5` unavailable with the `L1X_XSHUT_*` defines.
- `DMA_MAP.md`: `ADC_2` / `ADC_3` unavailable; DMA2_Channel2 then only via `ADC_5`.
- `dev-guide/HARDWARE_RESOURCES.md`: "Two VL53L1X on one bus" paragraph under *I2C1 and the down-laser*.

## 2. Replacement text

### 2.1 Source Files, `- **Laser**` bullet

- **Laser** ( one down-laser, only used by the estimator with `LASER_ALT` ): `src/main/drivers/ranging_vl53l0x.cpp` ( VL53L0X, `LASER_TOF`: 33 ms single ranging, `LASER_LPS` 0.1 IIR ) or `src/main/drivers/ranging_vl53l1x.cpp` / `.h` ( VL53L1X, `LASER_TOF_L1x`: Medium mode, 45 ms budget, 50 ms period, no filter ). Both sit on I2C1 at 0x29, so only one can be defined: `altitudehold.cpp` stops the build with an `#error` if both are, and if `LASER_ALT` is defined without either. With the `L1X_XSHUT_*` defines in `target.h` ( PRIMUS_X2_v1, PRIMUS_V5 ) a second VL53L1X may be fitted on the same bus: it is held in shutdown through its XSHUT pin and never read.

### 2.2 Laser fusion, first paragraph

With a downward laser, altitude hold means height above the surface under the craft: gentle
terrain is followed. `checkReading()` decides each estimator step what corrects the complementary
filter. One fusion body serves both lasers, the VL53L0X ( `LASER_TOF` ) and the VL53L1X
( `LASER_TOF_L1x` ). Only one is used: both sit on I2C1 at 0x29, and both would write the same
estimator state. Defining both stops the build with an `#error`, and so does `LASER_ALT` without a
laser. A second VL53L1X may be fitted only when the target holds it in shutdown ( `L1X_XSHUT_*`, see
**VL53L1X driver** ); two sensors awake at 0x29 answer together and corrupt every reply without an I2C
error. The two sensor types differ only in the accessors and constants below, plus a return guard that
only the VL53L1X has.

### 2.3 VL53L1X driver, `- **Set-up**` bullet

- **Set-up** ( `ranging_init_L1()`, from `main.cpp` ): with the `L1X_XSHUT_*` target defines it first takes
  the two XSHUT pins back from SPI2 ( `spiInit ( SPI2 )` ran earlier ): the second sensor's pin ( PB14 ) is
  driven LOW for the whole session and the bottom sensor's ( PB15 ) is released as a floating input, so the
  module's own pull-up ( ~2.8 V ) enables it; it is never driven HIGH. A 2 ms `delay ( )` then covers tBOOT
  ( 1.2 ms ): the VL53L1 platform's `WaitUs` / `WaitValueMaskEx` are stubs, so `VL53L1_WaitDeviceBooted`
  does not wait. Then Medium distance mode, 45 ms timing budget,
  50 ms inter-measurement period ( TIMED ranging ), each step only if the previous one succeeded; an
  error latches in `Global_Status_L1`. The mode, budget and period are target-overridable
  ( `L1X_DISTANCE_MODE`, `L1X_TIMING_BUDGET_US`, `L1X_SAMPLE_PERIOD_MS` ), and a `static_assert` requires
  period ≥ budget + 5 ms ( ST refuses to start below budget + 4 ms, and the error would latch for the
  flight ).

## 3. CHANGELOG entry ( applied at commit )

Into the open release section of `CHANGELOG.md` ( the topmost `## [vX.Y.Z]` heading; `pluto-commit` sets
the version ).

### Added

- **Laser driver ( VL53L1X ), two sensors on one bus**: Boards with two VL53L1X ( TOF400C ) at 0x29 can
  now fly on the bottom one. With the new `L1X_XSHUT_*` target defines ( PRIMUS_X2_v1 and PRIMUS_V5:
  bottom XSHUT on PB15 / GPIO_5, second on PB14 / GPIO_6 ), `ranging_init_L1 ( )` holds the second sensor
  in shutdown and releases the bottom one ( floating input to the module's own 2.8 V pull-up, never driven
  HIGH ) with a 2 ms boot wait before the first I2C access. Without it both sensors answered together:
  wrong calibration and ranges with no error. Flight on PRIMUS_X2_v1 with both fitted: no driver error, 20
  samples/s, valid status in 99.9 % of flight samples, handovers at 160 / 140 cm unchanged. Costs 68 B of
  flash. PB14 / PB15 are then unavailable to the M25P16 flash ( the driver stops the build with an
  `#error` if `USE_FLASHFS` / `USE_FLASH_M25P16` are still defined ), `GPIO_5/6`,
  `ADC_2/3`, `PWM_4/5`, `RGB_3`, XRanging FRONT / LEFT, `OPTIC_FLOW` and the SPI API.

### Changed

- **Target ( PRIMUS_X2_v1 )**: `LASER_TOF_L1x` and `LASER_ALT` are on: the VL53L1X down-laser is in the
  altitude-hold loop by default. With the `L1X_XSHUT_*` defines this also takes PB14 / PB15 by default
  ( see Added ), so the M25P16 flash is compiled out ( `USE_FLASHFS`, `USE_FLASH_M25P16` commented out ):
  no blackbox flash, CLI flash commands or MSP dataflash on this target.

### Documentation

- `fw-architecture-pipeline/subsystems/Altitude_Hold_Estimator.md`: a second VL53L1X held in shutdown;
  XSHUT step and boot wait in the driver set-up.
- `PIN_MAP.md` ( conflict 8 ), `TIMER_MAP.md`, `DMA_MAP.md`, `dev-guide/HARDWARE_RESOURCES.md`: PB14 / PB15
  as laser XSHUT and what that takes away.
