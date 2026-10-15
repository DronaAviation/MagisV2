# ToF Single Pipeline - Changes

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

## Task 2: ToF manager core ( 14 Oct 2026 )

Nothing calls the new code yet, so the firmware behaves exactly as before. `ranging_init ( )`, `getRange ( )`,
`LaserSensor` and the shared globals are untouched until task 5.

### Files

- [drivers/ranging_vl53l0x.h](../../../../src/main/drivers/ranging_vl53l0x.h) and
  [ranging_vl53l0x.cpp](../../../../src/main/drivers/ranging_vl53l0x.cpp) gain per-device helpers. Each takes a
  `VL53L0X_Dev_t *` and returns that device's status. None of them touches `Global_Status` or the result globals.
  - `vl53l0xConfigure ( dev, addr7, cfg, &failedStep )`: the full init sequence, stopping at the first error. It
    reports which step failed ( `vl53l0xStep_e` ), so a BOOT_FAILED says where it failed ( `pluto-driver` §5 ).
  - `vl53l0xSetAddress ( dev, addr7 )`: the host side follows only if the address write succeeded ( fixes F2 ).
  - `vl53l0xProbe ( addr7 )`: reads model ID register 0xC0 and expects 0xEE. A missing chip costs one I2C timeout
    ( F1 ). Used by the task-5 bring-up for the warm-reset probe.
  - `vl53l0xStartAndRead ( )`: start, then read straight away, which is today's behaviour for both pipelines.
  - `vl53l0xReadIfReady ( )`: data-ready check, then result read, then interrupt clear. For the task-7 paced poll.
- New [drivers/tof_manager.h](../../../../src/main/drivers/tof_manager.h) and
  [tof_manager.cpp](../../../../src/main/drivers/tof_manager.cpp):
  - A 6-entry table indexed by `tofId_e`, with the same values as `laser_e` plus `TOF_DOWN = 5`. Each entry holds:
    requested, XSHUT pin, address, status ( NOT_ENABLED / OK / BOOT_FAILED / I2C_ERROR ), failed step, last error,
    range status, out-of-range, new-sample, raw mm, and the cached side range.
  - A device struct per entry ( `tofDev [ 6 ]` ).
  - `tofManagerBringUp ( id, from, to )`: moves the chip to its address, then configures it for its role.
  - `tofManagerPollDown ( )`: today's `getRange ( )`, but per entry.
  - `tofManagerReadNow ( id )`: today's `LaserSensor::startRanging ( )`, but per entry.
  - The DOWN accessors `tofDownNew / ClearNew / OutOfRange / Reseed / FiltMm / RawMm`, which task 4 points altitude
    hold's macros at.
- [Makefile](../../../../Makefile): `drivers/tof_manager.cpp` added to `DRONA_DRIVERS`.

### DOWN configuration against today's `ranging_init ( )` ( R6 )

| # | `ranging_init ( )` ( ranging_vl53l0x.cpp:85-159 ) | `vl53l0xConfigure` with `tofDownConfig` | Same |
|---|---|---|---|
| 1 | `I2cDevAddr` 0x29, `comms_type` 1, 400 kHz | `addr7` ( the address it is moved to, 0x30 ), 1, 400 kHz | yes, apart from the address |
| 2 | `VL53L0X_DataInit` | `VL53L0X_DataInit` | yes |
| 3 | `VL53L0X_StaticInit` | same | yes |
| 4 | `PerformRefSpadManagement` | same | yes |
| 5 | `PerformRefCalibration` | same | yes |
| 6 | `SetDeviceMode ( SINGLE_RANGING )` | `cfg->deviceMode` = `VL53L0X_DEVICEMODE_SINGLE_RANGING` | yes |
| 7 | `SetLimitCheckEnable ( SIGMA_FINAL_RANGE, 1 )` | same | yes |
| 8 | `SetLimitCheckEnable ( SIGNAL_RATE_FINAL_RANGE, 1 )` | same | yes |
| 9 | `SetLimitCheckValue ( SIGNAL_RATE, 0.1 × 65536 )` | `0.1f × 65536` ( the same float: the tree builds with `-fsingle-precision-constant` ) | yes |
| 10 | `SetLimitCheckValue ( SIGMA, 60 × 65536 )`, **status not stored** | same value, status checked | value yes; see note |
| 11 | `SetMeasurementTimingBudgetMicroSeconds ( 33000 )` | `cfg->timingBudgetUs` = 33000 | yes |
| 12 | `SetVcselPulsePeriod ( PRE_RANGE, 18 )` | `cfg->vcselPrePclk` = 18 | yes |
| 13 | `SetVcselPulsePeriod ( FINAL_RANGE, 14 )` | `cfg->vcselFinalPclk` = 14 | yes |

The side configuration ( `tofSideConfig` ) is `LaserSensor::init ( )`'s: continuous mode, 20000 µs, 18 / 14, with the
same limit checks. It adds **one final step**: `VL53L0X_SetGpioConfig ( NEW_MEASURE_READY )` ( `sampleReadyInterrupt` ).
The DOWN configuration leaves this flag false, so it stays exactly as above.

**Why the side lasers need it ( review finding ).** The vendored `StaticInit` was changed locally to
`VL53L0X_GPIOFUNCTIONALITY_OFF` ( `lib/main/VL53L0X_API/core/src/vl53l0x_api.c:521`, marked `DD` ). With that
setting, `VL53L0X_GetMeasurementDataReady ( )` reads bit 0 of `RESULT_RANGE_STATUS` ( api.c:1953-1959 ), which
`ClearInterruptMask ( )` never resets. The task-7 paced poll could then report one sample as new over and over. With
the interrupt on, data-ready comes from the interrupt status, which the clear does reset.

`vl53l0xReadIfReady ( )` also treats `VL53L0X_ERROR_INTERRUPT_NOT_CLEARED` as success. ST returns that code whenever
the clear needed a third try, even if the try worked. The sample is already read, so the worst case is the same
sample seen twice, and task 7 must not latch I2C_ERROR on it.

### Behaviour differences ( deliberate, all on error paths )

- **Row 10.** Today a failed sigma-limit write is ignored and the init carries on. Now it stops the init as
  BOOT_FAILED, at step `VL53L0X_STEP_SIGMA_VALUE`.
- **A failed DOWN read.** Today `getRange ( )` still sets `isTofDataNewflag` and `Range_Status` from the stale
  result struct on the read that fails, and then stops for good. Now a failed read sets I2C_ERROR without flagging a
  new sample, and `tofDownOutOfRange ( )` is true from then on ( R3, D6 ).
- **DOWN never brought up or BOOT_FAILED.** `tofDownOutOfRange ( )` is true. Today `isOutofRange ( )` stayed false
  in that case, because `getRange ( )` does nothing on a latched error. Altitude hold behaves the same either way,
  since no new sample ever arrives and `ToF_Height` stays 0 ( altitudehold.cpp:1083, :1100 ).
- **Address before init.** `tofManagerBringUp ( )` moves the chip to its address and then configures it there.
  Today `LaserSensor` runs `init ( )` at 0x29 and then `setAddress ( )` ( XRanging.cpp:109-110 ). This is safe:
  nothing in `DataInit`, `StaticInit` or the tuning table writes register 0x8A or resets the chip ( reviewer
  checked ). The task-6 bench check confirms it on hardware.
- **Side out-of-range.** Today it was set only for `RangeStatus == 2` and was never cleared on other errors. Now it
  is `RangeStatus != 0`. Side out-of-range was only ever read by altitude hold, through the shared global ( H2 ),
  which is what this topic removes.

### Size

`tof_manager.o`: 840 B text, 2214 B bss ( `tofDev` 2112 B = 6 × 352 B, `tofTable` 84 B ). Nothing calls it yet, so
the linker drops it. The build is unchanged at 102.1 KB flash and 16.8 KB RAM, and the gate reports no new
warnings.

Once task 5 links it, `tofDev` replaces the global `MyDevice` and the 5 `LaserSensor::MyDevice` members, which are
also 6 × 352 B. Expected net RAM is about +100 B.

### Review ( pluto-reviewer, 14 Oct 2026 )

No blocking findings. The reviewer confirmed:
- DOWN's configuration is equal at every step;
- `tofManagerPollDown ( )` matches `getRange ( )` value for value on the success path ( same `Interval`, start then
  read, IIR, `+0.5f`, `< 2000`, reseed, `rawMm` );
- the 7-bit / 8-bit address handling, the probe register and value, and that the statics are safe without
  constructors.

Its three findings:
- the data-ready flag ( SHOULD-FIX ): **fixed** as above;
- two undocumented differences ( NITs ): **documented** above.

The gate passed again after the fix: 102.1 KB / 16.8 KB, no new warnings. `PRIMUS_V5` and `PRIMUSX2` are not built
yet; that happens in task 9 and at commit.


## Task 3: XRanging API ( 14 Oct 2026 )

The public API gains the DOWN laser. No existing signature changed ( R7 ). The bring-up is still the old one until
task 5: `xRangingAltLatch ( )` has no caller yet, so `getStatus ( )` reports `LASER_NOT_ENABLED` for every laser and
`getRange ( DOWN )` returns −100.

### Public API ( [API/XRanging.h](../../../../src/main/API/XRanging.h) )

- **`laser_e`** gains `DOWN` ( = 5, appended, so `EXTERNAL` keeps 4 ). `triggerThreshold` grows to 6 entries.
- **New `laser_status_e`:** `LASER_NOT_ENABLED`, `LASER_OK`, `LASER_BOOT_FAILED`, `LASER_I2C_ERROR`,
  `LASER_OUT_OF_RANGE`. No name clashes in the `PlutoPilot.h` include closure.
- **`bool initAlt ( peripheral_gpio_pin_e xshutGpio )` and `bool initAlt ( void )`** only record the request. They
  return `false` when the request is refused:
  - the bring-up has already run ( D4: only `plutoInit ( )` can request it );
  - `LASER_TOF_L1x` is compiled in ( D8 );
  - the pin is invalid;
  - the pin is **any** side laser's XSHUT ( GPIO_7 / 9 / 16 / 15 / 14 ), enabled or not, because user code may
    enable that laser later in `plutoInit ( )`;
  - the pin's ADC or PWM alias is already enabled.
- **`laser_status_e getStatus ( laser_e )`** maps the manager's status. OK with an out-of-range last sample is
  reported as `LASER_OUT_OF_RANGE`.
- **`getRange ( DOWN )`** returns DOWN's cached range in mm, −100 while there is no valid sample, or −1 when DOWN was
  not requested.
- The header now includes `API/Peripherals.h` for `peripheral_gpio_pin_e`.

### Implementation ( [API-Src/XRanging.cpp](../../../../src/main/API-Src/XRanging.cpp) )

- **Latch.** `xRangingAltLatch ( )`, declared in `API-Utils.h`, is called once by the task-5 bring-up. It checks the
  pin again, because ADC / PWM may be enabled after `initAlt` inside `plutoInit ( )`, and it fixes `altEnabled`
  for the rest of the run. A request refused at the latch clears `isXLaserInit [ DOWN ]`, and the firmware stays
  baro-only ( D5 ). `xRangingAltEnabled ( )` and `xRangingAltXshut ( )` feed task 4's flag and task 5's bring-up.
- **ADC / PWM aliases.** `gpioAlias [ GPIO_COUNT ]` holds each GPIO's ADC / PWM aliases, from `PIN_MAP.md`
  ( GPIO_5 = ADC_2 / PWM_4, GPIO_2 = ADC_1, … ). The ADC check uses the new `adcPinEnabled ( )` in
  [Peripheral-ADC.cpp](../../../../src/main/API-Src/Peripheral-ADC.cpp), and the PWM check uses `isPwmInit [ ]`.
- **An XSHUT claim beats an LED.** Every LED write now goes through `ledInit ( )` / `ledWrite ( )`, which skip the
  pin given to `initAlt`. That covers the bring-up, the blink animation, `isTriggered ( )`, `disableOA ( )` and the
  four OA writes. A grep shows no other `Peripheral_Write` on an `ledGpio`. With `initAlt ( GPIO_5 )`, FRONT has no
  LED. The skip stays in place even if the request is refused at the latch: a pin the user pointed `initAlt` at is
  never driven as an LED.
- **Bounds.** `init ( laser, t )`, `initObjectAvoidance ( )`, `isTriggered ( )` accept side lasers only
  ( `< LASER_COUNT` ). `getRange ( )` and `getStatus ( )` accept `< 6`. Out-of-range ids return `-1` / `false` /
  `LASER_NOT_ENABLED`. A `static_assert` ties `laser_e` to `tofId_e`.
- **Arrays.** `isXLaserInit` ( [API-Utils.h](../../../../src/main/API/API-Utils.h) / `.cpp` ) and OA `dFilt` grow to
  6 entries.
- **Duplicate removed.** The commented-out copy of the whole file ( old lines 384-733 ) is gone. It held 350 lines
  of comments and blank lines, with no live code ( checked ).
- **Other.** `triggerThreshold [ laser ] = threshold * 10` now has an explicit `int16_t` cast.

### Build

102.1 KB flash, 16.9 KB RAM ( +88 B: the manager's status table is now linked through `getStatus ( )` ). The gate
reports no new warnings.

The first attempt added 6 warnings, which were then fixed:
- `( peripheral_gpio_pin_e ) -1` is an unspecified conversion, because the enum has no negative values. The
  down-laser pin is now stored as `int8_t`.
- Including `API-Utils.h` in `Peripheral-ADC.cpp` repeated `utils.h`'s existing `cmp16` / `cmp32` warnings in
  one more translation unit. `adcPinEnabled` now gets C linkage at its definition, and the include is gone.

### Review ( pluto-reviewer, 14 Oct 2026 ) and fixes

There were no blocking findings. The reviewer confirmed that every LED path in `XRanging.cpp` is guarded, that the
`gpioAlias` table matches `Peripheral-GPIO.cpp` / `adcPinMap` / `pwmConfig`, the latch semantics, the bounds on
every entry point, and that there are no header / ABI / name problems.

- **SHOULD-FIX ( fixed ): writes from outside XRanging could still reach the XSHUT pin.** A user
  `Peripheral_Write ( GPIO_5, … )` ( an old "FRONT LED" habit ), or `Peripheral_Init ( PWM_4 / ADC_2 )` from
  `plutoLoop ( )` after the latch, would reset the altitude laser in flight. The fix:
  - **Query functions.** After the latch, `xRangingGpioClaimed ( )`, `xRangingAdcClaimed ( )` and
    `xRangingPwmClaimed ( )` ( in XRanging.cpp, declared in API-Utils.h ) report the claimed pin and its aliases.
  - **Refusal.** The public `Peripheral_Init` / `Peripheral_Write` for GPIO
    ( [Peripheral-GPIO.cpp](../../../../src/main/API-Src/Peripheral-GPIO.cpp) ), `Peripheral_Init` for ADC
    ( [Peripheral-ADC.cpp](../../../../src/main/API-Src/Peripheral-ADC.cpp) ) and `Peripheral_Init` for PWM
    ( [Peripheral-PWM.cpp](../../../../src/main/API-Src/Peripheral-PWM.cpp) ) ignore the claimed pin. A PWM write
    is already gated on `isPwmInit`.
  - **Bring-up path.** The task-5 bring-up drives the pin through the new guard-free `xshutPinInit ( )` /
    `xshutPinWrite ( )` ( Peripheral-GPIO.cpp ).
  - **Before the latch.** Inside `plutoInit ( )` the claim is not active yet. The latch re-check catches ADC / PWM
    use there.
  - **SPI2.** User `SPI_Init` can still take PB12-15. This is documented for the wiki ( task 11 ), not guarded.
- **NIT ( fixed ):** the `initObjectAvoidance` doc comment had ended up above `initAlt`. It is back above its own
  function.
- **NIT ( fixed ):** `lasers [ ]` is now `constexpr`, with `static_assert`s that entry i is `laser_e` i and that
  there is one entry per side laser.
- **NIT ( fixed ):** the header now says that a refused `initAlt` call keeps an earlier accepted request, and that
  the pin is reserved after start-up.

`Peripheral-ADC.cpp` declares `xRangingAdcClaimed` locally with `extern "C"` instead of including `API-Utils.h`,
for the same reason as `adcPinEnabled` ( including it would repeat `utils.h`'s warnings ).

The gate passed again: 102.2 KB / 16.9 KB, no new warnings.


## Task 4: boot-latched altitude-laser flag ( 14 Oct 2026 )

One flag replaces the `LASER_TOF` / `LASER_ALT` build switches for the VL53L0X. `altLaserFusion ( )` reads it and
`altLaserFusionLatch ( on )` sets it, both in
[altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp) and declared in `altitudehold.h`.
- It starts **off**. A VL53L1X build with `LASER_TOF_L1x` + `LASER_ALT` has it on at compile time
  ( `ALT_LASER_FUSION_BUILD` ), so L1x builds are unchanged ( D8 ).
- `altLaserFusionLatch ( )` runs once. With `on`, it sets τ = `ALT_EST_TAU_S` ( 1.5 s ) and calls `updateGains ( )`,
  because `main.cpp:467` computed the gains at boot from the initial 2 s.
- **Nothing calls the latch until task 5**, so for now every build behaves as today's baro build.
- `#error` now stops a target that still defines `LASER_TOF`, or `LASER_ALT` without `LASER_TOF_L1x`, and points to
  `initAlt`.

### Every gate: flag-off branch against today's baro build ( D5 )

| Old gate | Today's baro build ( no `LASER_*` ) | New, flag off | Same |
|---|---|---|---|
| altitudehold.cpp:108 `#error` ( both lasers ) | - | replaced by the two `#error`s above | n/a |
| altitudehold.cpp:111-235 constants and accessors ( `LASER_ALT` ) | not compiled; `_time_constant_z = 2.0f` | compiled ( L1x block, else the L0X block reading the DOWN entry ); `_time_constant_z = 2.0f` | yes: same τ, and the macros are unused while off |
| altitudehold.cpp:918 `checkBaro ( )` / `checkReading ( )` | `checkBaro ( )` | `if ( altLaserFusion ( ) ) checkReading ( ); else checkBaro ( );` → `checkBaro ( )` | yes |
| altitudehold.cpp:969-1313 the fusion ( `altHoldSource`, ring, `altShiftFrame`, `checkReading` ) | not compiled | compiled; only `checkReading ( )` reaches it, and only with the flag on | yes ( not called ) |
| altitudehold.cpp:1048, :1059 `LASER_TOF \|\| L1x` | - | removed ( always one laser body ) | n/a |
| altitudehold.cpp:1339 τ in `correctedWithBaro` | tilt > 30° → 5 s, else 2 s, `updateGains ( )` each call | `if ( altLaserFusion ( ) ) { 1.5 s; return; }`, then the same tilt code | yes |
| altitudehold.cpp:1360 `correctedWithTof` | not compiled | compiled; called only from `checkReading ( )` | yes ( not called ) |
| altitudehold.h:59 `altHoldSource` declaration | not declared | declared, plus `altLaserFusion` / `altLaserFusionLatch` | n/a |
| imu.cpp:51 `ALT_EST_ACC_Z_DEADBAND` | not defined | defined ( 0 ) | n/a |
| imu.cpp:261 accSum Z deadband | `accDeadband->z` | `altLaserFusion ( ) ? 0 : accDeadband->z` → `accDeadband->z` | yes |
| command.cpp:116 take-off height | `DesiredPosition_setRelative ( Z, … )` | `if ( altLaserFusion ( ) ) setDesiredPosition … else DesiredPosition_setRelative …` → relative | yes |
| mw.cpp:814 `UPDATE_LASER_TOF_TASK` | empty case | `tofManagerPollDown ( )`, which returns at its first check ( DOWN not OK ) | yes, apart from one call and compare |
| opticflow.cpp:355 `NewSensorRange` | `OPTIC_FLOW` is off on all targets, so not compiled | `tofDownFiltMm ( )` | n/a ( not compiled ) |
| target.h × 3 | `LASER_TOF` / `LASER_ALT` comments | `LASER_TOF` line replaced by "call `XRanging.initAlt ( )`"; `LASER_ALT` comment now says "with `LASER_TOF_L1x`" ( PRIMUSX2 has no L1x line, so both lines are replaced ) | n/a |

Still in place, and deleted in task 5: `ranging_vl53l0x.cpp:74` ( `ranging_init`, `getRange` … under `LASER_TOF` )
and `main.cpp:516`. Both are dead now, because `LASER_TOF` is an `#error`.

### Flag on, against today's `LASER_ALT` build

- **τ before the latch.** It is 2 s from boot until the latch ( main.cpp:600, before the loop and before arming is
  possible ), then 1.5 s with the gains recomputed. Today it was 1.5 s from boot. Nothing runs the estimator in
  between.
- **Laser accessors.** `tofOutOfRange ( )` is also true while DOWN's status is not OK ( R3 / D6 ). The other
  accessors read the DOWN entry, value for value ( task 2 ).

### Build

`PRIMUS_X2_v1`: 105.1 KB flash, 19.3 KB RAM ( text +3060 B, bss +2488 B ), no new warnings. The growth breaks down
as follows:
- the always-compiled fusion: about 2.9 KB flash and 0.4 KB RAM, matching the measured cost of `LASER_ALT`
  ( task 1 );
- the manager's `tofDev` ( 2112 B ), now linked through `tofManagerPollDown ( )`. Task 5 removes the global
  `MyDevice` and the 5 `LaserSensor` copies ( 2112 B ).

Line endings: the Python edit scripts wrote LF in the working copy. The index is LF ( `autocrlf=true` ), so
neither diffs nor commits change.

### Review ( pluto-reviewer, 14 Oct 2026 )

There were no blocking findings. The reviewer confirmed:
- every flag-off gate takes today's baro branch;
- the always-compiled fusion is reached only through `checkReading ( )`, and `altHoldSource ( )` has no callers;
- `_k1_z` … `_k3_z` are read only in the loop, so the 2 s gains between main.cpp:467 and :600 never fly;
- the accessors match task 2;
- no L1x constant left its block;
- the `#error`s are right;
- the opticflow expression is right;
- the dead `LASER_TOF` blocks have no other users.

Its findings:
- **SHOULD-FIX ( kept on purpose ): the latch turns the fusion on even if DOWN fails to start.** This is the user's
  decision **D6** ( "laser fails to start: the fusion runs with a dropout, today's `LASER_ALT` behaviour" ). The
  craft starts on the laser source, drops to the baro after `ALT_TOF_DROPOUT_MS`, and keeps the laser tuning
  ( τ 1.5 s, Z deadband 0, absolute take-off height ). **D5** ( baro-only build ) applies only when `initAlt` was not
  called or was refused. Task 5 therefore latches `altLaserFusionLatch ( xRangingAltLatch ( ) )`, **not**
  gated on DOWN's status.
- **NIT ( done ):** L1x alone had never been built. It is now: `LASER_TOF_L1x` without `LASER_ALT` compiles
  ( 113.9 KB / 20.2 KB ). `PRIMUS_V5` / `PRIMUSX2` are built in task 9.
- **NIT ( left ):** altitudehold.cpp:950 `#else checkReading ( )` ( no `BARO` ) ignores the flag. No target can
  reach it: all three define `BARO`, and the estimator compiles only under `BARO || LASER_ALT`.


## Task 5: single bring-up at main.cpp:600 ( 14 Oct 2026 )

There is now exactly one VL53L0X bring-up, `xRangingInit ( )`, at [main.cpp](../../../../src/main/main.cpp):600. It
runs after `plutoInit ( )`, so every `XRanging` request is known by then. The old down-laser path is gone.

### Bring-up order ( [XRanging.cpp](../../../../src/main/API-Src/XRanging.cpp) `xRangingInit ( )` )

| Step | Code | INVESTIGATION.md step |
|---|---|---|
| Latch | `altOn = xRangingAltLatch ( )`, then `altLaserFusionLatch ( altOn )`: the request and the fusion are fixed for the run. The fusion is **not** gated on DOWN's start-up result ( D6 ) | - |
| Reset | `tofManagerReset ( )`. Each enabled side laser is requested in the manager, its XSHUT driven low and its LED initialised. DOWN's XSHUT ( if wired ) goes low through `xshutPinInit` / `xshutPinWrite`, because the public calls refuse the claimed pin | 1-2 |
| DOWN alone, with a wire | release XSHUT, wait 30 ms, `tofManagerBringUp ( DOWN, 0x29, 0x30 )`. If that fails, XSHUT goes low again so the chip stays off the bus | 3a |
| DOWN alone, no wire | `vl53l0xProbe ( 0x30 )` → bring up at 0x30 ( warm reset ); else move 0x29 → 0x30 | 3b |
| Side lasers | in table order: release, wait 30 ms, `tofManagerBringUp ( id, 0x29, 0x2A + n )`. A failed laser is held in reset so the next one is alone at 0x29. `n` counts every enabled laser, as before | 4 |
| Status | each entry's own status ( OK / BOOT_FAILED + step + error ). Nothing is retried | 5 |
| LEDs | the animation is unchanged ( through `ledWrite`, so a claimed XSHUT is skipped ) | - |

### Removed

- [main.cpp](../../../../src/main/main.cpp): the `#ifdef LASER_TOF ranging_init ( )` block at :516, and the
  commented-out `useRangingSensor` lines. [mw.cpp](../../../../src/main/mw.cpp): the commented-out `getRange ( )`.
- [ranging_vl53l0x.cpp](../../../../src/main/drivers/ranging_vl53l0x.cpp) / `.h`: `ranging_init`, `getRange`,
  `isTofDataNew`, `isOutofRange`, `tofRequestReseed`, `update_status`, the `LaserSensor` class, and the globals
  `MyDevice`, `Global_Status`, `RangingMeasurementData`, `Range_Status`, `NewSensorRange`, `debug_range`,
  `isTofDataNewflag`, `out_of_range`, `startRanging`, `useRangingSensor` and `rangePoll`. Only the per-device
  helpers remain. A grep finds no remaining references outside comments.
- [XRanging.cpp](../../../../src/main/API-Src/XRanging.cpp): the 5 `LaserSensor` objects and the table's sensor
  pointer.

### Reads

- **DOWN.** `UPDATE_LASER_TOF_TASK` → `tofManagerPollDown ( )` ( task 4 ). Today's 33 ms start + read, into the DOWN
  entry only.
- **Side lasers.** `XRanging.getRange ( id )` → `tofManagerReadNow ( id )`: a blocking start + read of that laser,
  as before, but written only to its own entry ( H2 closed ). This phase-1 behaviour lasts until task 7.

`XRanging.cpp` declares `altLaserFusionLatch` locally with `extern "C"`, because `flight/altitudehold.h` cannot be
included on its own ( it needs the mixer / PID headers first ).

### Build

`PRIMUS_X2_v1`: 106.2 KB flash, 17.5 KB RAM ( bss −1796 B against task 4: the 6 old device structs are gone ). The
gate reports no new warnings.

| Build | Flash | RAM |
|---|---|---|
| Today's stock ( baro ) | 101.3 KB | 16.8 KB |
| Today's `LASER_ALT` ( rollback hex ) | 104.2 KB | 17.5 KB |
| This build ( either mode, chosen at boot ) | 106.2 KB | 17.5 KB |

### Review ( pluto-reviewer, 14 Oct 2026 ) and fixes

There were no blocking findings. The reviewer confirmed:
- for enabled lasers, the order follows INVESTIGATION.md step by step;
- the latch runs before anything reads the flag ( the loop starts at main.cpp:629 ), and it runs on every boot on
  all three targets;
- DOWN's claimed XSHUT is driven only through `xshutPinInit` / `xshutPinWrite`;
- the addresses match HEAD;
- no references remain to the removed code;
- stack and delays are bounded ( about 0.7 s worst case at boot, 300 ms of it the existing LED animation );
- the L1x case ( `ranging_init_L1` at 0x29 before the side lasers ) is unchanged from HEAD and out of scope.

Its findings:
- **SHOULD-FIX ( fixed ): a fitted laser whose XSHUT nothing drives can sit at 0x29.** A side laser that is not
  enabled is never driven: FRONT's PA15 keeps its JTAG pull-up, and LEFT's PB13 idles high as SPI2 SCK. Releasing
  DOWN would then re-address both chips together.
  - **The fix:** before each release, `vl53l0xProbe ( 0x29 )` must find nothing; this covers DOWN with a wire and
    every side laser.
  - **When 0x29 is busy:** that laser stays in reset and is marked BOOT_FAILED at the new step
    `VL53L0X_STEP_BUS_BUSY`, through the new `tofManagerMarkFailed ( )`.
  - **Cost:** each probe of an empty 0x29 is one I2C timeout plus a bus recovery ( F1 ), at most 6 times, once at
    boot.
- **NIT ( covered by the same check ):** a no-wire DOWN that fails its address write stays at 0x29. Every side
  release is now refused instead of colliding with it.
- **NIT ( bench, task 6 ):** the warm-reset path runs the full init again on a chip that is already running. The ST
  API allows this; the bench check confirms it.

**Limits that only the documentation can cover ( task 11 wiki ):**
- A no-wire DOWN cannot be checked for company at 0x29, because it sits there itself.
- If the down-laser is wired to GPIO_5 but `initAlt ( GPIO_5 )` is not called, FRONT's LED writes release DOWN
  together with FRONT, and both end up at FRONT's address.
- **Rule for the wiki:** every fitted laser must be enabled, and a wired down-laser must always get
  `initAlt ( <its pin> )`.

The gate passed again: 106.3 KB / 17.5 KB, no new warnings.


## Task 7: paced side-laser poll with cached getters ( 14 Oct 2026 )

The side lasers are no longer read on demand. The periodic laser task polls them, and every reader gets the cached
sample.

- **Continuous ranging starts once.** [tof_manager.cpp](../../../../src/main/drivers/tof_manager.cpp)
  `tofManagerBringUp ( )` calls `VL53L0X_StartMeasurement ( )` once, after a side laser's configuration. A failure
  is BOOT_FAILED at the new step `VL53L0X_STEP_START_RANGING`. DOWN stays in single mode, unchanged ( R6 ).
- **New `tofManagerPollSides ( )`.**
  - **Pacing:** it is paced to one call per 10 ms ( `TOF_SIDE_POLL_MS` ) and serves the **next running side laser in
    round-robin**. With 4 lasers, each one is read every ~40 ms, against their 20 ms timing budget.
  - **Per call:** one `vl53l0xReadIfReady ( )`. That is a data-ready read, plus, only when a sample is ready, the
    result read and the interrupt clear. Since a sample is never read before it exists, the start-up 8191 seen in
    log-2 cannot happen ( F5 ).
  - **Errors:** an error marks that laser I2C_ERROR, and only that laser ( R4 ). It is not retried in flight.
- **New sample counter.** Each `tofEntry_t` has a `samples` counter ( it wraps ), used for the task-8 update-rate
  check.
- **Getters read the cache.** [XRanging.cpp](../../../../src/main/API-Src/XRanging.cpp) `getRange ( )` returns the
  cached `rangeMm` for every laser, DOWN included, and makes **no I2C call**. So `isTriggered ( )` and
  `applyObjectAvoidance ( )`, which use it, make none either. A grep confirms that every I2C call left in
  XRanging.cpp is inside `xRangingInit ( )`. The blocking `tofManagerReadNow ( )` is deleted.
- **The periodic task.** [mw.cpp](../../../../src/main/mw.cpp) `UPDATE_LASER_TOF_TASK` calls
  `tofManagerPollDown ( )`, then `tofManagerPollSides ( )`. A call can do at most DOWN's 33 ms start + read plus
  one side laser's read.
- **TEMPORARY timing, removed in task 8.** `dbgTofTaskUsMax`, `dbgOaUsMax` and `dbgTofTaskCalls` in mw.cpp
  ( C linkage ) give the worst case of the laser task and of OA in µs, plus the task's call count. The bench
  `PlutoPilot.cpp` reads them.

**OA response.** OA now sees a side range at most ~40 ms plus one 20 ms ranging period old. Before, it started a
fresh read that returned the *previous* measurement ( F5 ), so the age is similar, without the 4-6 ms of blocking
I2C every 100 ms ( H5 ). `filterDistance ( )` can be fed the same cached sample twice when OA ( every 100 ms ) runs
faster than that laser's update. That is harmless for its 3:1 average.

**Build:** 107.4 KB / 17.6 KB ( this includes the task-6 bench `PlutoPilot.cpp` ). The gate reports no new warnings.

### Review ( pluto-reviewer, 14 Oct 2026 ) and fixes

There were no blocking findings. The reviewer confirmed:
- the continuous sequence on this lib ( `SetGpioConfig` after the mode, interrupt-status data-ready reset by the
  clear );
- the round-robin is fair and correct for 0-5 lasers;
- no synchronous side reads remain;
- the bus load is about 2 ms worst case per periodic call against a 3.5 ms `looptime` ( measured in task 8 ).

Its findings, all fixed:
- **SHOULD-FIX: a sensor flag was treated as a bus fault.** `GetInterruptMaskStatus ( )` returns `RANGE_ERROR`
  whenever `RESULT_INTERRUPT_STATUS` bits 3-4 are set ( lib api.c:2441 ). That would have marked a working laser
  I2C_ERROR for good. `vl53l0xReadIfReady ( )` now treats it as no error: data-ready still comes from bits 0-2, and
  the sample's `RangeStatus` marks it invalid.
- **SHOULD-FIX: a dead side laser kept OA pushing.** `filterDistance ( )` returns its last value when `d <= 0`. If
  a laser died with an obstacle inside the avoidance distance, OA kept pushing and braking forever, with the pilot
  held to 30 % / 20 % on that axis. The new `oaDistance ( )` in XRanging.cpp gives "no obstacle" and clears that
  laser's filter whenever its status is neither OK nor OUT_OF_RANGE. `dFilt` moved to file scope, and its update
  got an explicit `int16_t` cast. The old hold on a merely out-of-range sample is unchanged ( OA behaviour is out
  of scope ).
- **NIT: no mean timing.** `dbgTofTaskUsSum` ( temporary ) was added, so the bench can compute the mean.
- **NIT: lasers firing during calibration.** Side lasers started ranging during the next lasers' calibration. The
  new `tofManagerStartSides ( )` now starts continuous ranging on every side laser that came up, once, after the
  last bring-up. `tofManagerBringUp ( )` no longer starts them.

**Open for the user:** one NACK still marks a side laser I2C_ERROR for the rest of the run ( the "mark failed, no
retry" decision ). The reviewer suggested counting 3 consecutive errors before marking it. That is not a re-init,
so it still satisfies R4.

The gate passed again: 107.4 KB / 17.6 KB, no new warnings.


## Task 8: temporary timing removed ( 14 Oct 2026 )

[mw.cpp](../../../../src/main/mw.cpp) is back to two plain calls in `UPDATE_LASER_TOF_TASK`
( `tofManagerPollDown ( )`, `tofManagerPollSides ( )` ) and a bare `applyObjectAvoidance ( )`. The `dbg*` timing
globals are gone. The measurements are in TESTING.md test 3. The root `PlutoPilot.cpp` is back to the task-1
baseline log ( temporary, for task 10 ). Build: 106.6 KB / 17.6 KB, gate clean.

## Task 14: SPI2 off whenever XRanging is used ( 14 Oct 2026 )

GPIO_5-8 are SPI2's four pins ( PB15 MOSI, PB14 MISO, PB13 SCK, PB12 NSS / M25P16 CS ). XRanging uses them as XSHUT and
LED pins: LEFT 6 / 7, FRONT's LED 5, BACK's LED 8, and the `initAlt ( GPIO_5 )` XSHUT. User decisions: SPI2 goes off
**whenever XRanging is used at all**, and **XRanging wins** over the user SPI API.

- **[bus_spi.c](../../../../src/main/drivers/bus_spi.c) / `.h`: new `spiRelease ( SPI2 )`.** It runs once and does
  the following:
  - SPI2 disabled, `SPI_I2S_DeInit`, clock off;
  - SCK / MISO / MOSI set as floating inputs;
  - NSS kept as an output driven **high**, so the on-board flash stays deselected unless the pin's new user
    ( BACK's LED ) drives it;
  - after that, `spiInit ( SPI2 )` returns false and `spiTransfer` / `spiTransferByte` on SPI2 return at once.

  `spiIsReleased ( )` reports the state.
- **[flash_m25p16.cpp](../../../../src/main/drivers/flash_m25p16.cpp) / `.h`: new `m25p16_release ( )`.** It zeroes
  the geometry, so flashfs sees size 0. After it, every entry point returns at once: init, isReady ( true ), erase,
  program, readBytes ( 0 ), status. No call touches the bus or the CS pin. `BLACKBOX` is off in target.h.
- **[Serial-IO-SPI.cpp](../../../../src/main/API-Src/Serial-IO-SPI.cpp):** `SPI_init`, `SPI_Init`, `SPI_Enable`,
  `SPI_Disable`, both `SPI_Read` overloads and `SPI_Write` return at once once SPI2 is released ( XRanging wins ).
- **[XRanging.cpp](../../../../src/main/API-Src/XRanging.cpp) `xRangingInit ( )`:** right after the latch, if any
  `isXLaserInit [ ]` entry is set ( DOWN included, after the latch check ), it calls `spiRelease ( SPI2 )` and
  `m25p16_release ( )`. Both are gated on `USE_SPI_DEVICE_2` / `USE_FLASH_M25P16`. This happens before any laser pin
  is configured. With no XRanging calls, SPI2 and the flash stay exactly as before ( spiInit at main.cpp:313,
  `m25p16_init` / `flashfsInit` at :450-453 ).

**Order at boot.** `spiInit ( SPI2 )` ( :313 ) → flash detection ( :450 ) → `plutoInit ( )` ( :540, user SPI calls
here still work ) → `xRangingInit ( )` ( :600 ): SPI2 is released here, and from then on the user SPI API is inert.

**Not covered.**
- The PAW3903 optic-flow drivers set up SPI2 themselves. They compile only with `OPTIC_FLOW`, which is off on every
  target, and their transfers would be refused by the bus guard anyway.
- The `RGB_3` LED data pin is PB14 ( GPIO_6, LEFT's LED ). That is an existing pin overlap and out of scope.
- **The flash shares the pins.** With PB12 as BACK's LED, writing it LOW selects the flash chip, which can then drive
  PB14 ( MISO ) against LEFT's LED output. This is a board-level overlap, known and out of scope; it goes in the wiki.

**Build:** 107.0 KB / 17.6 KB ( `PRIMUS_X2_v1` ), 107.0 / 17.6 ( `PRIMUS_V5` ), 106.8 / 17.7 ( `PRIMUSX2` ). All three
gate clean.

### Review ( pluto-reviewer, 14 Oct 2026 ) and fixes

- **BLOCKING ( fixed ): the release overrode pins user code had already taken.** It ran after `plutoInit ( )` and
  `APIAdcInit ( )`, and turned PB13-15 into inputs and forced PB12 high, whoever owned them. That broke `RGB_3`,
  `PWM_4` / `PWM_5`, `ADC_2-4` and user GPIO_5-8 even when XRanging used none of those pins. Now `spiRelease ( )`
  reconfigures a pin only while **SPI2 still owns it** ( MODER = alternate function and AFR = AF5 ), and leaves PB12
  alone. The reviewer re-checked the bit math, the types, and each user case: RGB / PWM use AF1, ADC is analog mode,
  a GPIO is mode 0 / 1, so all are left alone.
- **SHOULD-FIX ( fixed ): floating pins.** Released pins are now **pulled-up inputs**, which matches an XSHUT's idle
  state, so a fitted-but-disabled LEFT stays reliably awake at 0x29 and the `BUS_BUSY` check still sees it. The
  XRanging header comment was corrected.
- **NIT ( wiki ):** after the release, the MSP dataflash summary reports size 0, but the boot-time offset and read
  replies carry no data bytes. Harmless unless an MSP client uses dataflash.
- **NIT ( wiki ):** `SPI_Enable` / `SPI_Disable` ( which drive PB5, GPIO_13 ) are inert too. Separately, the header
  declares `SPI_Init ( )` while the source defines `SPI_init ( )` ( pre-existing ).

**The wiki must say:**
- SPI calls in `plutoInit ( )` work;
- from the bring-up on, every SPI call silently does nothing ( `SPI_Read` returns 0, the buffer overload leaves the
  buffer untouched );
- the flash reports size 0.

The gate passed again on all three targets: 107.0 / 17.6, 106.9 / 17.7, 107.0 / 17.6 KB.

**Hardware check.** The A3 run's user code prints `SPIoff:1 Flash:0` on its first tick: XRanging is used, so SPI2
must be released and the flash absent.

## Task 12: topic review fixes ( 15 Oct 2026 )

A fresh `pluto-reviewer` reviewed the whole topic diff ( 25 files + `tof_manager.*` ). **No BLOCKING findings.** All
six required items were clean: flag-off equivalence ( re-walked gate by gate ), the LED / XSHUT rule, `laser_e`
bounds, no I2C in getters, no re-init in flight, and the SPI2 release.

- **SHOULD-FIX ( fixed ): RGB and UART2 could still drive a claimed XSHUT pin.** The claim covered the GPIO / ADC /
  PWM APIs, but `RGB_Init ( )` ( RGB slots ) and `Uart_Init ( UART2 )` ( including the UWB set-up at main.cpp:597,
  which runs *after* the bring-up ) could take the pin later. A WS2812 line idling low, or UART2 TX, would reset the
  down-laser in flight. [XRanging.cpp](../../../../src/main/API-Src/XRanging.cpp) `altPinConflict ( )` now refuses
  the RGB slot and UART2 pins outright ( `altReservedGpio`: GPIO_1, 3, 6, 10, 11, 12, 13, 16, 17, 18 ). That leaves
  GPIO_2, 4, 5 and 8 as XSHUT candidates, still subject to the ADC / PWM checks. The wiki and PIPELINE_UPDATE list
  the rule.
- **NIT ( fixed ): leftover includes.** The unused `drivers/ranging_vl53l0x.h` was removed from main.cpp, mw.cpp and
  altitudehold.cpp, and six unused includes from ranging_vl53l0x.cpp.
- **NIT ( fixed ): dead code.** `tofManagerStatus ( )` and `xRangingAltEnabled ( )` had no callers and are removed.
  `altHoldSource ( )` stays, because diagnostics read it.
- **NIT ( fixed ): stale comments.** The `getRange` brief in XRanging.h now describes the cached read, the
  `initObjectAvoidance` doc describes the side-laser bound, and the tof_manager.h comment refers to the *former*
  `getRange ( )`.
- **NIT ( fixed ): literal array sizes.** `static_assert`s now tie `isXLaserInit [ 6 ]` and
  `triggerThreshold [ 6 ]` to `XLASER_ID_COUNT`.
- **SHOULD-FIX ( planned ): version bump.** `FW_Version` / `API_Version` ( 1.4.1 → 1.5.0, the public XRanging API
  grew ) is bumped in task 13 by `/pluto-commit`. The warnings baseline lists 15 *fixed* warnings and can be
  refreshed at commit; it must not be regenerated to hide anything new.

The gate passed again on all three targets: 107.0 / 17.6, 107.0 / 17.6, 106.9 / 17.7 KB.
