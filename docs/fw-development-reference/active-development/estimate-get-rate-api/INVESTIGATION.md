# Estimate_Get Rate API - Investigation

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md)

## Problem

API test T03 ( item B8, the testing team's external sheet ): `Estimate_Get ( Rate, X | Y | Z )` returns the
stick-rate setting, always `75`, not the measured rotation rate. In the same test `Estimate_Get ( Position, Z )` read
0 at rest, which is correct.

## Evidence

- [FC-Data-Estimate.cpp](../../../../src/main/API-Src/FC-Data-Estimate.cpp) lines 77-86: the `Rate` case returns
  `currentControlRateProfile->rates [ FD_ROLL | FD_PITCH | FD_YAW ]`. That is the profile's stick-to-rate setting
  ( uint8 ), a configuration value, not a measurement. It never changes while the drone rotates.
- Why 75: `acrobats.cpp` writes `rates [ ROLL / PITCH ] = 75` ( 110 while a flip runs ). The config reset default is 0
  ( `config.cpp:357` ). Yaw shows its own stored value.
- The measured rate is `gyroADC [ axis ]` ( int16, bias removed, board-aligned; `sensors/gyro.h:34` ). Index 0/1/2 is
  roll/pitch/yaw, the same order as `axis_e` X/Y/Z.
- Gyro scale: both targets define `DCM`, so the attitude flown comes from `matrix_update ( )`, which integrates
  `gyroADC` at **16.4 counts per deg/s** ( `imu.cpp:590` ). `GYRO_CONFIG_1 = 0x1F` is the +/-2000 dps range, which is
  16.4 counts per deg/s. The driver's `gyro->scale = 1 / 131` ( `accgyro_icm20948.cpp:68` ) is only read by code that
  is not active on these targets ( non-DCM attitude, `pidLuxFloat` ), so it is stale but harmless. Correct angles in
  flight confirm 16.4.
- So `gyroADC / 1.64` is deg/s x10, the same scale `Sensor_Get ( Gyroscope )` already uses.
- The other cases are correct and stay as they are: Position Z in cm ( `EstAlt` ), Velocity Z in cm/s, Angle in
  deci-degrees ( roll, pitch ) and whole degrees ( yaw ). Position / Velocity X and Y are 0 unless UWB ( or the
  disabled `OPTIC_FLOW` ) feeds the position estimator: documented behaviour.
- No firmware code and no `PlutoPilot.cpp` example calls `Estimate_Get ( Rate, ... )`. Known users only read Angle and
  Position / Velocity Z, which do not change.

## Causal chain

`Estimate_Get ( Rate, axis )` -> `currentControlRateProfile->rates [ axis ]` ( a stored setting ) -> the tester's
profile holds 75 -> 75 on every read, whatever the drone does.

## Fix chosen

Option A: the `Rate` case returns the measured gyro rate in **deg/s x10**,
`roundToInt16 ( gyroADC [ axis ] / 1.64f )`, raw ( no extra filter: the same `gyroADC` the rate loop uses ). A
file-local float rounding helper is used because `<math.h>` in a file that includes `flight/pid.h` adds a baselined
`M_PI` redefinition warning ( lesson from baro-temperature-api ). Header doc corrected: `int16_t` return, units for
every case. API 1.8.2, FW 3.15.4.

Rejected: whole deg/s ( loses resolution below 1 deg/s ), a filtered copy in `imu.cpp` ( adds RAM, a loop hook and
delay, for a getter nobody uses yet ).

## Range

+/-32767 gyro counts gives +/-19980 deg/s x10: fits `int16_t`.

## Out of scope ( README open items )

- The stale `gyro->scale = 1 / 131` in the ICM-20948 driver and its contradicting comments: its own topic.
- Position / Velocity X and Y returning 0 without UWB: documented only, unchanged.

## Q&A that shaped the plan

The session ran unattended, so the defaults below were taken and logged as decisions; correct any before
`/pluto-task next`.

| Question | Answer taken |
|---|---|
| Mode | `fix` ( "check this api and help me fix the issue" ); cause proven from the code by the scout |
| Target | `PRIMUS_X2_v1` ( `selected_target` in `plutoide.ini` ); `PRIMUS_V5` built at commit |
| Unit for Rate | deg/s x10, same as `Sensor_Get ( Gyroscope )` |
| Raw or filtered | raw |
| Gyro note in the wiki | corrected in this topic ( 16.4 counts per deg/s proven ); the driver's stale scale is a separate topic |
| Validation | bench: at rest about 0, a hand rotation of one turn per axis integrates to about 360 deg |
| X / Y without UWB | documented only |
| Done when | `Estimate_Get ( Rate, axis )` follows rotation on all three axes, about 0 at rest, one turn integrates to 360 +/- 30 deg |
