# Baro Temperature API

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

| | |
|---|---|
| **Status** | **Closed** in `7a41a37` ( 3 Nov 2026, FW 3.15.3, API 1.8.1 ) |
| **Mode** | `fix` |
| **Branch** | `BugFix-June26` |
| **Target** | `PRIMUS_X2_v1` ( `PRIMUS_V5` must stay correct; both use the ICP-10111 ) |
| **Last updated** | 3 Nov 2026 |
| **Pipeline docs** | Promoted at commit: one `Sensor_Get` bullet in [User_Space_API.md](../../fw-architecture-pipeline/subsystems/User_Space_API.md); new wiki [FC_DATA_API_WIKI.md](../../../API/FC_DATA_API_WIKI.md) |

**Problem.** API test T03 ( B7 ): `Sensor_Get ( Barometer, Temperature )` returns `32` where the header
[FC-Data.h](../../../../src/main/API/FC-Data.h) promises deg C x100 ( `3270` ). Pressure is fine ( `100470` Pa ).
Cause ( proven by code ): [FC-Data-Sensors.cpp](../../../../src/main/API-Src/FC-Data-Sensors.cpp) line 90 returns
the float `getBaroTemperature ( )` ( deg C ) through an implicit float -> int32 truncation and never multiplies
by 100; the x100 comment is a leftover from the Cleanflight integer barometer. Both implicit conversions are
sitting in the warnings baseline. Details: [INVESTIGATION.md](INVESTIGATION.md).

**Fix.** Return `roundToInt32 ( getBaroTemperature ( ) * 100.0f )` ( a file-local float helper, half away from zero;
`lrintf` would need `<math.h>`, which adds a baselined `M_PI` redefinition warning ), round the pressure case the same way, correct the
header comment ( "Pa ( = mbar x100 )", "deg C x100" ). Internal units ( deg C in `barometer.cpp`, used by the
altitude-hold temperature compensation ) do not change. API 1.8.1, FW 3.15.3.

## Open items

- All tasks done ( 3 Nov 2026 ): all three targets built, FW 3.15.3 / API 1.8.1, docs promoted, CHANGELOG updated. Task 1-4 summary: fix built and gated on `PRIMUS_X2_v1` ( flash / RAM identical to HEAD, the getter is uncalled and stripped ); bench log-1 shows `Tx100` 3500-3574 in steps of 1, matching the internal deg C, pressure 101242 Pa steady; temporary instrumentation removed ( `PlutoPilot.cpp` back to HEAD ); `PIPELINE_UPDATE.md` and `docs/API/FC_DATA_API_WIKI.md` written and checked against the code. Defaults taken unattended: x100 rounded half away from zero, API 1.8.1, new wiki.
- Out of scope, worth a later topic: the `Sensor_Get` axis overload falls off the end of the function for Gyro / Mag
  with `Net_Acc` ( undefined behaviour, baselined `-Wreturn-type` ); `FC-Data.h:95,115` say `uint16_t` for
  `int16_t` returns; gyro / mag axis units undocumented.
- The user's test sheet ( T03 / B7 ) is not in the repo; other failures on it are unknown.
- Side findings from the task 4 spec check ( out of scope, flight-relevant, worth their own `check` topic ):
  the ICM-20948 driver sets `gyro->scale = 1 / 131` ( `accgyro_icm20948.cpp:68` ) while its own comments say 16.4 counts
  per deg/s and `GYRO_CONFIG_1 = 0x1F` ( 2000 dps ): an 8x conflict that feeds attitude integration and the rate loop,
  not only this API. `Sensor_Get ( Magnetometer, axis )` returns 0 until the compass is calibrated ( `magScale`
  defaults to 0 ). `main.cpp:464-467` calls `baroInit ( )` / `baroCalibrate ( )` without checking the baro was detected.

## Next action

None: closed, pending the user's commit. Follow-up candidates are the side findings above ( a `check` topic ).
