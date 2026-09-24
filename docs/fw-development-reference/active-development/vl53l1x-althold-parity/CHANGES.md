# VL53L1X Altitude Hold Parity - Changes

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

## Task 3: VL53L1X driver ( 23 Sep 2026 )

Why: the bench log-1 ( [TESTING.md](TESTING.md) Test 1 ) showed that the driver never clears the
sensor's data-ready interrupt. Every 10 ms poll therefore re-reads the same result and flags it as new.
The ST default timing ( 41 ms budget / 100 ms period ) was also slower than the 50 ms decided in
planning.

| File | Change | Why |
|---|---|---|
| [ranging_vl53l1x.cpp](../../../../src/main/drivers/ranging_vl53l1x.cpp) `ranging_init_L1 ( )` | After `SetDistanceMode ( MEDIUM )`: `SetMeasurementTimingBudgetMicroSeconds ( 45000 )`, then `SetInterMeasurementPeriodMilliSeconds ( L1X_SAMPLE_PERIOD_MS = 50 )`, each only if the status is OK | 20 Hz samples, close to the L0X's 33 ms tuning. ST requires period ≥ budget + 4 ms ( `vl53l1_api.c:1779-1781` ), and 50 ≥ 49. The budget was changed from the planned 33000 after review: in the AUTONOMOUS preset a fixed ~26.6 ms guard comes off the budget and the rest is split over two phases ( `vl53l1_api.c:1172-1213` ), so 33000 gave only 3.2 ms per phase. 45000 gives 9.2 ms, more than the 41000 default's 7.2 |
| same, `getRange_L1 ( )` | On data-ready: clear the local `dataFlag`, read the result, then `VL53L1_ClearInterruptAndStartMeasurement ( )` ( **superseded in task 14**: 17-byte result read + `VL53L1_clear_interrupt ( )` ). `isTofDataNewflag_L1`, `sampleCount_L1++` and `lastSampleMs_L1` are set only on that genuine new result | One "new" sample per measurement instead of about 10 re-reads |
| same | A negative valid range is clamped to 0 before it goes into the `uint16_t NewSensorRange_L1` ( **superseded in task 6**: a result below `L1X_MIN_VALID_MM` 15 mm is invalid ) | Fixes the `-Wsign-conversion`; a negative value would have wrapped to about 65 m |
| same, `isOutofRange_L1 ( )` | Also true when `Global_Status_L1` is non-zero, or when no new result has arrived for `L1X_STALE_MS` = 3 × 50 + 10 = 160 ms | A latched ST error stops ranging for good, and a stall can happen without an error; either way the consumer must fall back to the baro |
| same | Removed the unused locals ( 6 ) and the unused double `LASER_LPS 0.75` | Warnings; no double constants |
| [ranging_vl53l1x.h](../../../../src/main/drivers/ranging_vl53l1x.h) | `#define L1X_SAMPLE_PERIOD_MS 50`, outside the laser `#ifdef`s; `extern` for `sampleCount_L1` / `lastSampleMs_L1` | Task 6 derives the L1x estimator constants from the period; the counter gives the log-2 interval and the estimator's staleness |
| same, `LaserSensor_L1` | Mem-initializer list reordered to declaration order | `-Wreorder` ( 12 baseline entries, refresh the baseline at commit ). The class is otherwise untouched ( XVision is out of scope ) |

Numbers: about +0.5 KB flash, +12 B RAM ( L1x build: 110.3 KB / 15.8 KB ), including the temporary poll timer. Gate passes on PRIMUS_X2_v1 with
`LASER_TOF_L1x`. The L0X `altitudehold.o` dump is identical to the baseline ( `35812d92…` ).

### Tooling

- [baseline/snapshot.sh](baseline/snapshot.sh) now sets all three laser defines explicitly for each
  configuration. The first version only turned defines on, so with the temporary `LASER_TOF_L1x` in
  the working tree, "L0X" built both sensors ( found by `cpp-pro` ).

## Task 14: cut the VL53L1X sample-poll cost ( 23 Sep 2026 )

Why: log-2 / log-3 measured the sample poll at 5437-5469 µs ( `T` ), once per ~53 ms, so one 3.5 ms
loop in ~15 stretched to ~9 ms. The poll read 133 B of system + core + debug results
( `VL53L1_GetRangingMeasurementData`, `vl53l1_api_core.c:2225-2252` ) and re-wrote the 68-byte
GENERAL_ONWARDS config block 0x0044..0x0087 ( `VL53L1_ClearInterruptAndStartMeasurement` →
`vl53l1_api_core.c:2421-2425` ).

Why the short path is consistent with the ST state machine in this preset:
- RangeStatus comes only from `RESULT__RANGE_STATUS` and `RESULT__STREAM_COUNT`
  ( `vl53l1_api_core.c:2457-2464, 2583-2597`; `vl53l1_api.c:2034-2160` ). The sigma / signal limits
  are thresholds in the timing config ( `vl53l1_api.c:1409-1428` → `vl53l1_api_core.c:1687-1760` ),
  checked on the device and reported as SIGMATHRESHOLDCHECK / MSRCNOTARGET; the API only maps them.
- Offset and crosstalk are corrected on the device ( `final_crosstalk_corrected_range_mm_sd0` ); the
  only software step is the gain factor 2011/2048 ( `vl53l1_api_core.c:2499-2507` ).
- The TIMED preset disables GPH ( `vl53l1_api_preset_modes.c:959-960` ) and is not low-power-auto,
  so the per-range rewrite re-sends unchanged config ( the "Dynamic Management" step is empty,
  `vl53l1_api_core.c:2409-2417` ). The stream-count / GPH-id checks run only in back-to-back mode
  ( `vl53l1_core.c:292-294` ), and `cfg_gph_id` is used only there ( `vl53l1_api_core.c:2106-2112` ).

| File | Change | Why |
|---|---|---|
| [ranging_vl53l1x.cpp](../../../../src/main/drivers/ranging_vl53l1x.cpp) `getRange_L1 ( )` ( ~line 255 ) | On data-ready: `VL53L1_ReadMulti` of the 17 bytes 0x0089..0x0099, then `VL53L1_clear_interrupt ( )` ( 1-byte `SYSTEM__INTERRUPT_CLEAR` ), then decode | Replaces the 133 B read + 68 B write. Same one-sample-per-measurement flow and bookkeeping |
| same, `l1xRangeStatus ( )` ( line 118 ) / `l1xDecodeResult ( )` ( line 152 ) | RangeStatus, range and the other struct fields with the API's own mapping and arithmetic | Same RangeStatus values and range as the full API. Not updated: `TimeStamp`, `RangeQualityLevel`, the API's `LimitChecksCurrent/Status` ( nothing reads them ) |
| same, `ranging_init_L1 ( )` ( line 218 ) | `VL53L1_GetTuningParameter ( VL53L1_TUNING_PROXY_MIN )` into `l1xProxyMinMm` ( -30 mm ) | The negative-range floor that SetSimpleData uses |

Limits: a `VL53L1_SetDistanceMode ( )` after start would no longer be applied ( the API applied it
through `ChangePresetMode` in the clear call ); nothing calls it at runtime. Numbers: flash -2.1 KB
( 111728 → 109604 B text; the full-API read and clear paths are no longer linked ), RAM +8 B data.
Gate passes, L0X `altitudehold.o` identical.

**Measured ( log-4 ):** worst sample poll 784-789 µs, down from 5437-5469 µs; interval 51.2 ms. Correction after review: the device-side limit values are the Medium preset defaults, sigma 90 mm / signal 1.5 MCPS, not DataInit's 18 mm / 0.25 MCPS.

## Task 16: distance-mode switch ( 24 Sep 2026 )

| File | Change | Why |
|---|---|---|
| [ranging_vl53l1x.cpp](../../../../src/main/drivers/ranging_vl53l1x.cpp) | `L1X_DISTANCE_MODE`, default `VL53L1_DISTANCEMODE_MEDIUM` ( `#ifndef` ), passed to `VL53L1_SetDistanceMode ( )` in `ranging_init_L1 ( )` | Long / Medium A/B with one line. `SetDistanceMode` re-applies the budget and period ( `vl53l1_api.c:1068-1094` ); Long uses VCSEL periods 15 / 13 with the same sigma 90 mm / signal 1.5 MCPS limits ( `vl53l1_tuning_parm_defaults.h:104-113` ) |
| [target.h](../../../../src/main/target/PRIMUS_X2_v1/target.h) | TEMPORARY `#define L1X_DISTANCE_MODE VL53L1_DISTANCEMODE_LONG` | Tasks 17-18 fly Long; task 18 sets the final mode |

Build: gate PASS, 108.4 KB / 15.8 KB, `ranging_init_L1` disassembly passes mode 3 ( LONG ), L0X
`altitudehold.o` identical.

## Task 19: overridable budget and period ( 24 Sep 2026 )

| File | Change | Why |
|---|---|---|
| [ranging_vl53l1x.h](../../../../src/main/drivers/ranging_vl53l1x.h) | `L1X_SAMPLE_PERIOD_MS` default 50 inside `#ifndef` | The target can set the period ( Long at 10 Hz test ) |
| [ranging_vl53l1x.cpp](../../../../src/main/drivers/ranging_vl53l1x.cpp) | `L1X_TIMING_BUDGET_US` default 45000 inside `#ifndef` | Same, for the budget. `L1X_STALE_MS` follows the period |
| [target.h](../../../../src/main/target/PRIMUS_X2_v1/target.h) | TEMPORARY `L1X_TIMING_BUDGET_US 95000`, `L1X_SAMPLE_PERIOD_MS 100` | Long at about 34 ms per phase ( log-7 ) |

Build: gate PASS, 108.5 KB; `ranging_init_L1` passes mode 3, period 100 and budget 0x17318 ( 95000 );
the L0X and no-laser `altitudehold.o` are identical.

## Task 5: shared fusion helpers and sensor accessors ( 24 Sep 2026 )

Why: task 6 runs the flight-validated VL53L0X fusion body for the VL53L1X too. In this task that body
reads the sensor only through accessors and per-sensor constants, so the L1x can plug in without
touching the logic. The L0X object code is unchanged.

| File | Change | Why |
|---|---|---|
| [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp) ~:107-109 | `#error` if `LASER_TOF` and `LASER_TOF_L1x` are both defined | Both sensors are at I2C 0x29, and both blocks would write the estimator |
| same ~:161-205 | Accessor macros `tofNew ( )`, `tofClearNew ( )`, `tofOutOfRange ( )`, `tofReseed ( )`, `tofFiltCm ( )`, `tofRawCm ( )`. Per-sensor constants `ALT_TOF_HANDOVER_UP/DOWN_CM`, `_DROPOUT_MS`, `_RETURN_SAMPLES`, `_IIR_LAG_S`, `_STEADY_SAMPLES`, `_RING_LEN`, and the new `_OFFSET_DT_MAX_S` ( was a bare `0.05f` ) | Under `LASER_TOF` each expands to exactly the old expression and literal. The L1x side is provisional: derived from `L1X_SAMPLE_PERIOD_MS`; the handover edges are left undefined until task 6 |
| same ~:202-203 | `#error` if `LASER_ALT` has no laser | The shared ring needs a sensor's constants; no target uses that combination |
| same ~:951-1022 | Ring buffer, `tofWindowReset / Push / Mismatch`, `altShiftFrame` moved from `#ifdef LASER_TOF` to `LASER_ALT`; `[[maybe_unused]]` on three | Available to the L1x in task 6 without new warnings now |
| same ~:1034-1250 | The fusion body reads the sensor only through the accessors; still under `#ifdef LASER_TOF` | Task 6 widens **both** guards ( `tofTiltOk` ~:1023 and the body ~:1034 ) and deletes the old L1x branch ( ~:1254-1281, untouched here ) |

**Proof:** debug-stripped `altitudehold.o` dumps are identical to the baselines for **L0X, no-laser and
L1x + `LASER_ALT`** ( checked by `cpp-pro` and again independently ). The L1x + `LASER_ALT` warning set is
unchanged ( 33 in `altitudehold.cpp` ), and so is the L0X + `LASER_ALT` set ( 32, compared against HEAD's `altitudehold.cpp` ). Gate PASS. After review, the L1x side was changed to a dropout of 3P + 25 ( 175 ms; **superseded in task 6**: 3P + 35 = 185 ms ) and an offset clamp of 1.5 P, with `static_assert`s on the steady count, the ring span and the dropout; the objects are still identical. Flash / RAM unchanged in every configuration. Both
`#error`s were proven with a temporary `target.h` edit, and `target.h` was restored byte-for-byte.

## Task 6: the VL53L1X runs the shared fusion path ( 24 Sep 2026 )

| File | Change | Why |
|---|---|---|
| [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp) ~:1039, ~:1050 | `tofTiltOk` and the fusion body guarded by `#if defined( LASER_TOF ) \|\| defined( LASER_TOF_L1x )` | The L1x runs the flight-validated L0X body: tilt rejection above 25°, the 160 / 140 handover with frozen offset and frame shift, dropout, object hold-off / window / re-base, landing re-base |
| same, old `#ifdef LASER_TOF_L1x` branch | **Deleted**: the 350 cm hard switch, the radian tilt test ( never rejected ), `baro_offset = filtered − EstAlt` every tick, and the stale-reading correction past the reach | The causes in INVESTIGATION.md, findings 1, 2, 4 and 5 |
| same ~:191-192 | L1x `ALT_TOF_HANDOVER_UP_CM 160.0f` / `DOWN_CM 140.0f` | log-5: 100 % valid up to 180 cm ( decision 2026-09-23 ) |
| same ~:200-209, ~:1241-1248 | **Return guard, L1x only**: on the baro, armed and not at ground idle, a sample below the return edge that disagrees with `_position_z` by more than `ALT_TOF_RETURN_AGREE_CM` = `ALT_TOF_STEP_CM` ( 30 ) + `ALT_TOF_RETURN_BARO_CM` ( 20 ) = 50 cm resets the return count | log-5: ceiling-fan blades read a valid 56 cm for 4 samples at 250 cm and would have shifted the frame by −194 cm. `_position_z` already carries the frozen offset and is smoothed by the accelerometer; the difference equals the frame shift the return would take. Off while disarmed or in the pre-take-off ground idle. **Exit ( after review, user ):** if the disagreement ( laser − estimate ) stays within `ALT_TOF_STEADY_CM` ( 10 cm; **superseded in task 9**: `ALT_TOF_RETURN_STEADY_CM` 25 cm ) for `ALT_TOF_STEP_HOLD_MS` ( 2.5 s ), it is a new floor ( take-off from a table, a baro drift ) and the return is taken with the normal frame shift. Testing the disagreement, not the raw reading, lets the craft climb, descend or land meanwhile; fan blades are intermittent and never last 2.5 s |
| same ~:969 | `[[maybe_unused]]` removed from the three helpers | Every `LASER_ALT` build now uses them |
| [ranging_vl53l1x.cpp](../../../../src/main/drivers/ranging_vl53l1x.cpp) ~:60-63, ~:296 | `L1X_MIN_VALID_MM 15`: a status-0 result is valid only if `15 ≤ mm < 4500` | A covered window reads 0-10 mm with status 0; landed reads 25-28 mm and stays valid, so the landing logic is unchanged ( user ). In the driver, so `newGoodSample`, `tofUsable`, the dropout and the return all see it |

**Final L1x constants** ( P = `L1X_SAMPLE_PERIOD_MS` = 50 ms ):

| Constant | Value | Derivation |
|---|---|---|
| `ALT_TOF_HANDOVER_UP / DOWN_CM` | 160 / 140 cm | log-5 reach: 100 % valid to 180 cm |
| `ALT_TOF_DROPOUT_MS` | 3P + 35 = 185 ms | 2 missed samples at the measured 51-53 ms period + ~14 ms poll quantisation + the 10 ms estimator tick; ≥ `L1X_STALE_MS` ( 3P + 10, now in `ranging_vl53l1x.h` ) |
| `ALT_TOF_RETURN_SAMPLES` | 3 ( 150 ms ) | as L0X |
| `ALT_TOF_IIR_LAG_S` | 0 | no driver filter |
| `ALT_TOF_STEADY_SAMPLES` | 500 / P = 10 | 0.5 s |
| `ALT_TOF_RING_LEN` | ceil ( 660 / P ) = 14 | ≥ 0.66 s, spans the 500 ms window |
| `ALT_TOF_OFFSET_DT_MAX_S` | 1.5 P = 0.075 s | L0X 0.05 s ≈ 1.5 × 33 ms |
| `ALT_TOF_RETURN_AGREE_CM` | 30 + 20 = 50 cm | object step + baro allowance |
| `L1X_MIN_VALID_MM` | 15 mm | covered 0-10 mm, landed 25-28 mm |

Shared with the L0X: 25° tilt limit, 30 cm step, 2.5 s hold-off, window 500 / 400 ms, suspect 15 cm,
escalate 20 cm, steady 10 cm, offset tau 2 s, airborne grace 1 s. The laser correction runs on every
10 ms estimator tick with the last held `ToF_Height`, so its strength does not depend on the 20 Hz
sample rate ( this closes the 2026-09-23 risk ).

**Proof:** L0X and no-laser `altitudehold.o` identical ( checked twice ). L1x + `LASER_ALT`: 32
`altitudehold.cpp` warnings against 33 before; none are new, and the one removed was in the deleted branch.
Gate PASS. Flash / RAM: shipped configuration 108.4 KB / 15.8 KB; L1x + `LASER_ALT` 111.0 KB / 16.1 KB
( about +2.2 KB flash and +230 B RAM over the old L1x branch ). Review: no BLOCKING; the return-guard exit, the
comments, the 185 ms dropout and `L1X_STALE_MS` in the header came from its findings, and the exit was re-reviewed.

## Task 9 change: guard exit band ( 24 Sep 2026 )

| File | Change | Why |
|---|---|---|
| [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp) ~:215-218, ~:1237 | New L1x constant `ALT_TOF_RETURN_STEADY_CM` = 25 cm for the guard's steady exit ( was `ALT_TOF_STEADY_CM` 10 cm ) | log-12: the baro-path estimate swung 20-40 cm while descending, so the 10 cm band restarted the 2.5 s timer and the exit took ~10 s. The object hold-off keeps 10 cm |

L0X and no-laser `altitudehold.o` identical; gate PASS; 111.0 KB.

## Temporary diagnostics ( **removed in task 10, 24 Sep 2026** )

`PlutoPilot.cpp` and PRIMUS_X2_v1 `target.h` were restored to HEAD ( laser defines off: user ), and `maxPollUs_L1`
with its two `micros ( )` lines was removed from the driver. Shipped build 98.9 KB / 14.8 KB, the same as HEAD; gate
PASS; L0X and no-laser `altitudehold.o` identical; L1x + `LASER_ALT` 110.1 KB / 16.1 KB. What was there:

- [PlutoPilot.cpp](../../../../PlutoPilot.cpp): bench log in `plutoLoop ( )`, under `#ifdef LASER_TOF_L1x`.
  Task 2 logged `mm St R SC d A G O B`. Task 3 logs `mm St N A G O T B`, about 87 B per tick.
- [ranging_vl53l1x.cpp/.h](../../../../src/main/drivers/ranging_vl53l1x.cpp): `maxPollUs_L1`, the worst `getRange_L1 ( )` poll in µs ( `micros ( )` around the poll body ). The reviewer estimated about 4.8 ms per sample poll ( data-ready + 133 B results + 66 B clear at 400 kHz ), longer than one 3.5 ms loop.
- [target.h](../../../../src/main/target/PRIMUS_X2_v1/target.h): `#define LASER_TOF_L1x` enabled; the tasks 16-19 Long overrides were removed in task 18 ( Medium 45 / 50 chosen ).
