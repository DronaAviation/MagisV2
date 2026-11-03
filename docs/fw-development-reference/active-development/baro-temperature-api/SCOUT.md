# Scout brief: Sensor_Get ( Barometer, Temperature ) returns whole deg C, but the header says deg C x100

( `pluto-scout` brief, 3 Nov 2026, verbatim. Graph was stale ( built a81c180d, HEAD a7f398d1 ); every point was confirmed by grep and git. )

Subsystems: User API ( FC-Data ), barometer sensor ( ICP-10111 ) -> pipeline docs: `fw-architecture-pipeline/subsystems/User_Space_API.md`, `subsystems/Altitude_Hold_Estimator.md` ( read only )

Key code:
- `src/main/API-Src/FC-Data-Sensors.cpp:79` `Sensor_Get ( FC_Sensors_e, BARO_Data_e )`: lines 87 and 90 return the `float` getters through an implicit float -> int32 conversion.
- `src/main/sensors/barometer.cpp:43`: `float baroTemperature`, in deg C ( float ).
- `src/main/sensors/barometer.cpp:289` `getBaroTemperature ( )`: returns that float.
- `src/main/drivers/barometer_icp10111.cpp:213`: `_temperature_C = -45 + 175/65536*raw_t`, so plain deg C with about 0.003 C resolution. Line 236 gives `_pressure_Pa`.
- `src/main/sensors/barometer.cpp:400` `baroCompensationPa ( )`: an internal consumer that expects deg C ( `BARO_COMP_TEMP_PA_PER_DEGC 2.10f`, line 385 ).
- `src/main/API/FC-Data.h:79-80`: the doc comment says "100*millibar" and "100*degreeCelsius".

Mode: fix

Current behaviour: Both targets define `BARO` + `USE_BARO_ICP10111` ( `target/PRIMUS_X2_v1/target.h:75-76`, `PRIMUS_V5/target.h:76-77` ). The driver stores Pa and deg C as floats. `Sensor_Get` truncates both toward zero, so 32.70 becomes 32.

Analysis ( fix ):
- Proven causal chain: ICP driver gives float deg C ( `barometer_icp10111.cpp:213/239` ) -> `baroTemperature` ( `barometer.cpp:262` ) -> `getBaroTemperature ( )` ( `:291` ) -> the implicit float -> int32 return at `FC-Data-Sensors.cpp:90`. Nothing multiplies by 100, so the decimals are lost.
- The x100 wording comes from the old Cleanflight int `baroTemperature`, which was x100 ( still visible in `frsky.c:215` `( baroTemperature+50 )/100` and in the APM path `barometer.cpp:304`: `+27315 )/100` ). It went stale when the ICP float driver came in.
- Pressure: the return is truncated Pa. 100*mbar = hPa*100 = Pa, so the header is correct but ambiguous. It should say "Pa ( = mbar x100 )".
- `tools/warnings-baseline.json:117` already records these two lines: two `-Wfloat-conversion` "float to int32_t" warnings, i.e. the bug was baselined. Line 118 records a `-Wreturn-type` in the axis overload: Gyro/Mag with `Net_Acc` breaks and falls off the end of the function, which is undefined behaviour.
- Minimal fix ( option a ) touches only `FC-Data-Sensors.cpp:85-90`: an explicit rounded `( int32_t )` of `getBaroTemperature ( )*100.0f`, plus an explicit cast for pressure. Cost is a few dozen bytes of flash and no RAM. Do NOT change `getBaroTemperature ( )` or the driver, because the compensation and calibration code ( `barometer.cpp:400, 529, 549, 573` ) need deg C.

Options:
- ( a ) Return deg C x100 as the header says. The driver already has the full-precision float, so no data is invented. This changes the API value for any user code written against whole degrees, so bump `API_Version` ( Makefile:32, currently 1.8.0 ) and `FW_Version` ( Makefile:31, currently 3.15.2 ). It also removes two baselined warnings, so the gate count goes down.
- ( b ) Keep whole deg C and fix only the header and comment. Decimals stay lost. This is a documentation fix: bump at most the FW patch version and leave the API unchanged.

Constraints / invariants that apply:
- pluto-rules: a public API behaviour change means updating the `docs/API/` wiki and bumping FW/API_Version ( `pluto-rules/SKILL.md:164`, CLAUDE.md Two-layer API ).
- `-Wconversion` is strict: write the cast explicitly and never regenerate the baseline.
- Single-precision math only: use `100.0f`, not `double`.
- If temperature is printed: the `Monitor_Print` double overload loses digits after the first decimal, so print the int x100 value.

Resource conflicts: none

Already answered by the code:
- Both targets use the ICP-10111.
- Pressure is already Pa ( 100470 is correct ).
- No firmware code calls `Sensor_Get`. MSP and blackbox do not read baro temperature. FrSky ( `frsky.c:215` ) is inside `#ifdef TELEMETRY`, which neither target defines.
- `PlutoPilot.cpp` has no example that uses it.

Questions for the user:
1. Option ( a ) x100 or option ( b ) whole deg C?
2. Should rounding be to nearest, or is truncation to 0.01 C fine?
3. Should the pressure doc read "Pa"?
4. API_Version bump level for ( a ): 1.8.1 or 1.9.0?
5. There is no FC-Data wiki. Should a `docs/API/FC_DATA_API_WIKI.md` be created?
6. Where does the T03 ( B7 ) checklist live? Does it list other failures?

Doc drift:
- `FC-Data.h:79-80` and `FC-Data-Sensors.cpp:86,89`: the "100*degreeCelsius" wording vs the code at `:90`.
- `FC-Data.h:95,115` say "@return uint16_t", but the signature is `int16_t`.
- Units of the `Sensor_Get` axis overload ( gyro `/1.64`, mag `/6.82` ) are undocumented. Listed only, not expanded.

Prior work: `altitude-hold` ( closed, `4657cbf` ) added the temperature compensation. Other commits on these files: `f3f9a95` ( uint32 -> int32 return ), `8d827ed` ( temperature LPF ). No open topic covers this.
"T03 ( B7 )" is not anywhere in the repo, so it comes from the user's external test sheet.

Suggested skills: pluto-rules, pluto-build ( `--gate PRIMUS_X2_v1`, then PRIMUS_V5 ), pluto-flighttest ( bench check: Monitor_Print shows about 3270 ), pluto-commit

Pipeline docs to update at the end:
- `User_Space_API.md` ( after line 27 ): add a `Sensor_Get ( Barometer, Pressure|Temperature )` line with units ( Pa, deg C x100 or whole deg C ).
- `Altitude_Hold_Estimator.md`: no change, as long as `getBaroTemperature ( )` stays in deg C.
