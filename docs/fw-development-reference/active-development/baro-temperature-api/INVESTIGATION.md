# Baro Temperature API - Investigation

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

## Problem

API test T03 ( item B7, the user's external test sheet ): `Sensor_Get ( Barometer, Temperature )` returns
`32` for a room at about 32.7 C. The public header [FC-Data.h](../../../../src/main/API/FC-Data.h) ( lines 79-80 )
promises "100*degreeCelsius", so the expected value was `3270`. The decimals are lost. In the same test
`Sensor_Get ( Barometer, Pressure )` returned `100470`, which is correct ( Pa ).

## Evidence

- Both targets ( `PRIMUS_X2_v1`, `PRIMUS_V5` ) use the ICP-10111 barometer. The driver computes
  `_temperature_C = -45 + 175 / 65536 * raw_t` ( float, plain deg C, about 0.003 C resolution ) and
  `_pressure_Pa` ( float ) ( [barometer_icp10111.cpp](../../../../src/main/drivers/barometer_icp10111.cpp) lines 213, 236 ).
- `baroTemperature` is a `float` in deg C ( [barometer.cpp](../../../../src/main/sensors/barometer.cpp) line 43 );
  `getBaroTemperature ( )` ( line 289 ) returns it unchanged. The altitude-hold temperature compensation
  `baroCompensationPa ( )` ( line 400, `BARO_COMP_TEMP_PA_PER_DEGC 2.10f` ) and the calibration code
  ( lines 529, 549, 573 ) consume it **in deg C**: the internal unit must not change.
- [FC-Data-Sensors.cpp](../../../../src/main/API-Src/FC-Data-Sensors.cpp) lines 87 and 90 `return getBaroPressure ( );`
  and `return getBaroTemperature ( );` from an `int32_t` function: an implicit float -> int32 truncation toward zero
  with no x100 scaling. The compiler flagged both as `-Wfloat-conversion`; both are recorded in
  `tools/warnings-baseline.json` ( line 117 ), so the bug was baselined rather than fixed.
- The "x100" wording is inherited from the Cleanflight `int32_t baroTemperature` ( deg C x100 ), still visible
  in `frsky.c:215` ( `( baroTemperature + 50 ) / 100` ) and in the APM path at `barometer.cpp:304`. It went stale
  when the float ICP-10111 driver replaced it.
- No firmware code calls `Sensor_Get`; MSP and blackbox do not read the baro temperature; FrSky telemetry is
  not compiled for either target; `PlutoPilot.cpp` has no example using it. The change is therefore confined
  to user code.

## Causal chain

ICP-10111 float deg C -> `baroTemperature` ( deg C ) -> `getBaroTemperature ( )` -> implicit float -> int32 at
`FC-Data-Sensors.cpp:90` -> whole degrees returned, header promises hundredths.

## Fix chosen

Option ( a ): make the code match the header. `Sensor_Get ( Barometer, Temperature )` returns
`roundToInt32 ( getBaroTemperature ( ) * 100.0f )` ( deg C x100, rounded to nearest, half away from zero; a file-local
float helper, because `lrintf` needs `<math.h>` and that adds a baselined `M_PI` redefinition warning from `flight/pid.h` );
the pressure case is rounded the same way and its comment says "Pa ( = mbar x100 )". The driver and `getBaroTemperature ( )` are
untouched. API 1.8.1 ( behaviour-only change, same precedent as 1.4.1 ), FW 3.15.3.

Option ( b ) ( keep whole degrees, fix the comment ) was rejected: the user asked for the API to deliver the
documented value, and the driver already has the precision.

## Side findings ( out of scope, see README open items )

- `Sensor_Get ( FC_Sensors_e, FC_Axis_e )` axis overload: Gyro / Mag with `Net_Acc` `break`s and falls off the
  end of the function ( `-Wreturn-type`, baseline line 118 ): undefined behaviour.
- `FC-Data.h:95,115` document `@return uint16_t` for `int16_t` functions.
- Units of the gyro ( `/1.64` ) and mag ( `/6.82` ) axis values are undocumented.

## Q&A that shaped the plan

The session ran unattended, so the defaults below were taken and logged as decisions; correct any before
`/pluto-task next`.

| Question | Answer taken |
|---|---|
| Mode | `fix` ( "resolve this issue in the api" ) |
| Target | `PRIMUS_X2_v1` ( `selected_target` in `plutoide.ini` ); `PRIMUS_V5` built at commit |
| x100 or whole degrees | x100, as the header says |
| Rounding | nearest, half away from zero ( `roundToInt32` helper; `lrintf` rejected, see D2 ) |
| Pressure doc | "Pa ( = mbar x100 )", value unchanged |
| API_Version | 1.8.1 |
| Wiki | create a short `docs/API/FC_DATA_API_WIKI.md` covering `Sensor_Get` |
| Done when | bench print shows about 3270 for a 32.7 C room and about 100470 Pa |
