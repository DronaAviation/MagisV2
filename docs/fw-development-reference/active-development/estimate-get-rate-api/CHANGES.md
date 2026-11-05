# Estimate_Get Rate API - Changes

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

What changed and how. Why is in [INVESTIGATION.md](INVESTIGATION.md).

## Task 1 ( 4 Nov 2026 ): measured rate from `Estimate_Get ( Rate, axis )`

### [FC-Data-Estimate.cpp](../../../../src/main/API-Src/FC-Data-Estimate.cpp)

- New file-local constant `GYRO_COUNTS_PER_DPS_X10 = 1.64f`, with a comment naming its basis: the flown DCM attitude
  ( `imu.cpp` `matrix_update` ) integrates `gyroADC` at 16.4 counts per deg/s, the +/-2000 dps range of
  `GYRO_CONFIG_1`.
- New file-local helper `roundToInt16 ( float )`: round to nearest, half away from zero, clamped to
  `INT16_MIN..INT16_MAX`, explicit cast. No `<math.h>`, because this file includes `flight/pid.h`, which redefines
  `M_PI` ( the same reason as `roundToInt32` in `FC-Data-Sensors.cpp` ).
- `case Rate`: for X, Y and Z returns `roundToInt16 ( ( float ) gyroADC [ _axis ] / GYRO_COUNTS_PER_DPS_X10 )`, the
  measured rate in deg/s x10. Was `currentControlRateProfile->rates [ FD_ROLL | FD_PITCH | FD_YAW ]`, the stick-rate
  setting. Any other axis ( `Net_Acc` ) still falls to `break` and the final `return 0`.
- Banner `Last Modified` set to 4 Nov 2026.

### [FC-Data.h](../../../../src/main/API/FC-Data.h)

- `Estimate_Get ( FC_Estimate_e, axis_e )` doc: `@return int16_t` ( was `uint16_t` ); Rate in deg/s x10 from the gyro,
  rounded; Position X / Y and altitude in cm; Velocity X / Y / Z in cm/s; X and Y stay 0 without a positioning sensor
  ( UWB ).
- `@warning` narrowed: Position / Velocity X and Y need UWB, Z is always valid ( review nit ).
- `Estimate_Get ( FC_Estimate_e, angle_e )` doc: `@return int16_t` ( was `uint16_t` ).
- Banner `Last Modified` set to 4 Nov 2026.

### Not changed

Position, Velocity and Angle code; `rates []` ( owned by the flip code and the rate profile ); the gyro driver.

## Build ( task 1, gate on `PRIMUS_X2_v1` )

| | |
|---|---|
| Flash | 121.1 KB / 256 KB |
| RAM | 20.5 KB / 40 KB |
| Warning gate | no new `src/` warning; the four warnings left in this file are the baselined Position / Velocity conversions |

## Build gate ( task 2, 4 Nov 2026, `PRIMUS_X2_v1` )

Re-run after the task 1 review fix. `driver.sh --gate PRIMUS_X2_v1`: **no new `src/` warnings**, all builds passed.

| | HEAD ( `7a41a37` ) | This change |
|---|---|---|
| text / data / bss ( B ) | 122752 / 1316 / 19728 | 122752 / 1316 / 19728 |
| Flash | 121.1 KB / 256 KB | 121.1 KB / 256 KB |
| RAM | 20.5 KB / 40 KB | 20.5 KB / 40 KB |

No firmware code calls `Estimate_Get`, and the link uses `-Wl,-gc-sections`, so the function is not in the ELF in either
build. In the object file the fixed `Estimate_Get ( FC_Estimate_e, axis_e )` is 0xd4 = 212 B ( `roundToInt16` inlined );
that is the flash cost when user code calls it. RAM cost is 0.

## Task 4 ( 5 Nov 2026 ): docs

### [FC-Data.h](../../../../src/main/API/FC-Data.h) ( comments only )

- `Estimate_Get ( axis_e )` Rate doc: the sign of each axis as measured in task 3 ( X > 0 rolling right, Y > 0 nose
  down, Z < 0 turning right, opposite to the heading ).
- `Estimate_Get ( angle_e )` doc: roll > 0 right side down, pitch > 0 nose down, heading 0 to 359 rising clockwise.

### [FC_DATA_API_WIKI.md](../../../API/FC_DATA_API_WIKI.md)

- Title and intro cover `Estimate_Get`; version 1.8.2.
- `Sensor_Get ( Gyroscope )` unit: deg/s x10 ( was "not verified" ); the known-gaps note now says the gyro scale is
  confirmed and only the magnetometer unit is open.
- New sections `Estimate_Get ( estimate, axis )` ( units, sign table, yaw note, range, when 0 ) and
  `Estimate_Get ( Angle, angle )` ( units, ranges, signs ).
- Three examples ( rates and heading, altitude and climb rate, counting clockwise turns ); "Changes in 1.8.2"; four
  file-table rows.

### [PIPELINE_UPDATE.md](PIPELINE_UPDATE.md)

- New `Estimate_Get` bullet for `User_Space_API.md` with every case's unit and sign; gyro unit and API 1.8.2 in the
  `Sensor_Get` bullet. No flowchart change.

### Spec-to-code check ( inline: the `spec-to-code-compliance` skill is not installed in this session )

| Doc claim | Source | Holds |
|---|---|---|
| Rate = `gyroADC / 1.64` rounded, deg/s x10, int16 | `FC-Data-Estimate.cpp:76-91` | yes |
| 16.4 counts per deg/s | `imu.cpp` `matrix_update` ( ~590 ), `GYRO_CONFIG_1 = 0x1F`; bench 354-387 deg per turn | yes |
| `Sensor_Get ( Gyroscope )` = same counts / 1.64, truncated | `FC-Data-Sensors.cpp:52,65` | yes |
| Position Z = `EstAlt` cm; Velocity Z = `VelocityZ` cm/s, climbing positive | `altitudehold.cpp:389,994-1012,1510-1517` ( `_velocity_z` integrates into `_position_z` ) | yes |
| Position / Velocity X, Y = `PositionX/Y`, `VelocityX/Y`, whole cm and cm/s, UWB only | `posEstimate.cpp:299-302`, `posEstimate.h:48-50`, UWB case `posEstimate.cpp:188` | yes |
| Net_Acc or any other axis returns 0 | `FC-Data-Estimate.cpp` `break` then `return 0` | yes |
| Roll -1800..1800, pitch -900..900 deci-degrees | `imu.cpp:479-488` `bodyDcmToEuler` ( roll `atan2f`, pitch `-safe_asin` ), `imu.cpp:1185-1186`; the `imu.cpp:423-426` path is compiled out ( `DCM` defined on every target ) | yes |
| Heading whole degrees 0-359, from `-anglerad [ AI_YAW ]` | `imu.cpp:1191-1197` | yes, and it explains why Rate Z counts against the heading |
| Signs of X, Y, Z | bench log-1 ( TESTING.md ) | yes ( measured ) |
| Gyro 0 during calibration | `gyro.cpp:94` | yes |

## Task 5 ( 5 Nov 2026 ): topic review fixes ( docs only )

- [FC_DATA_API_WIKI.md](../../../API/FC_DATA_API_WIKI.md): the "Count clockwise turns" example zeroes its count in
  `plutoInit` and only resets `lastMs` in `onLoopStart` ( over Wi-Fi a late RC frame re-runs `onLoopStart`, 26 times in
  the 122 s bench log, so a reset there lost most turns ); note reworded. The `gyro.scale` ( 1 / 131 ) note now says the
  default `pidRewrite` and the DCM attitude do not use it, while `pidLuxFloat` and the blackbox header still read it.
- This file: the roll / pitch range row cites the flown `bodyDcmToEuler` ( `imu.cpp:479-488` ).
- `active-development/README.md:62`: a U+2212 minus replaced by an ASCII hyphen ( pre-existing ).

## File index

| File | Change |
|---|---|
| `src/main/API-Src/FC-Data-Estimate.cpp` | `GYRO_COUNTS_PER_DPS_X10`, `roundToInt16`; Rate case from `gyroADC` |
| `src/main/API/FC-Data.h` | `@return int16_t`; units for every case ; signs of Rate and Angle ( task 4 ) |
| `docs/API/FC_DATA_API_WIKI.md` | `Estimate_Get` sections, gyro unit, 1.8.2 ( task 4 ) |
| `PIPELINE_UPDATE.md` | staged `User_Space_API.md` bullets ( task 4 ) |
