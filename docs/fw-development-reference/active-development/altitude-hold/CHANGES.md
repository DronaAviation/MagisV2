# Altitude Hold: Changes

[README](README.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

What changed in the code and how it works. Why each change was needed is in
[INVESTIGATION.md](INVESTIGATION.md); the measurements behind the numbers are in
[TESTING.md](TESTING.md). Committed on `BugFix-June26` as FW 3.7.0 ( API 1.3.1 unchanged ).

### Sensor chain

**1. Ground re-zero can no longer fire in flight** - [mw.cpp:174](../../../../src/main/mw.cpp#L174), [mw.cpp:396](../../../../src/main/mw.cpp#L396), [mw.cpp:555](../../../../src/main/mw.cpp#L555), [mw.cpp:592](../../../../src/main/mw.cpp#L592)

`mwArm ( )` is reached from the throttle-low stick handler on every loop where
the arm switch is on and the throttle is at the bottom, in flight as well as on
the ground. It called `baroResetGroundLevel ( )` unconditionally, so one throttle
chop at 2 m re-zeroed the datum to 2 m. Logged: ground reference jumped +255 cm in
flight. The static flag `throttleRaisedSinceArm` now gates the reset.

**2. Estimator tilt threshold, deci-degree units bug** - [altitudehold.cpp:698](../../../../src/main/flight/altitudehold.cpp#L698)

`> 30` meant 3.0 degrees, so the slow branch ( `_time_constant_z = 5`, ~15 s
settling ) was the normal in-flight state. Changed to `> 300` ( 30 degrees ).

**3. Pressure to altitude at a fixed ISA temperature** - [barometer.cpp:318](../../../../src/main/sensors/barometer.cpp#L318)

The old conversion scaled by the ICP-10111 die temperature, which self-heats, so
the altitude scale changed flight to flight. Now the ISA form pinned at 288.15 K,
as INAV does. Ground altitude is cached in `getBaroGroundAltitude ( )`
( [barometer.cpp:329](../../../../src/main/sensors/barometer.cpp#L329) ).

**4. Zero tracks while disarmed, freezes on arm** - [barometer.cpp:351-369](../../../../src/main/sensors/barometer.cpp#L351-L369)

`baroUpdateZero ( )` follows the reading with an IIR ( `BARO_ZERO_TRACK_ALPHA
0.05f` ) while disarmed and stops the instant the craft arms. Applied as an
offset, never by rewriting `baroGroundPressure`. Exposed as `getBaroZeroOffset ( )`.

**5. Throttle and temperature compensation** - [barometer.cpp:371-414](../../../../src/main/sensors/barometer.cpp#L371-L414)

```c
#define BARO_COMP_THROTTLE_PA_PER_COUNT 0.0086f   // 8.6 Pa per 1000 counts
#define BARO_COMP_TEMP_PA_PER_DEGC      2.10f     // was 1.90 until 17 Sep
#define BARO_COMP_LIMIT_PA              25.0f     // was 15 until 17 Sep
```

Both terms are referenced to the arm instant, so the correction is zero on the
first armed sample and cannot introduce an absolute offset. The total is clamped.
The temperature input is the unrounded, filtered ICP-10111 die temperature
( `getBaroTemperature ( )` ). How the numbers were chosen is in [TESTING.md](TESTING.md).

**6. ICP-10111 OTP constants read as signed** - [barometer_icp10111.cpp:155](../../../../src/main/drivers/barometer_icp10111.cpp#L155)

Correctness fix. The tested units' constants are positive, so no behaviour change
on them.

**7. ICP-10111 temperature no longer frozen at power-on** - [barometer_icp10111.cpp:276-320](../../../../src/main/drivers/barometer_icp10111.cpp#L276-L320)

Seeded from a 50-sample average, then tracked with `TEMP_LPF_ALPHA 0.02f`.
Without this the sensor's own compensation ran on a stale temperature.

### Controller side

**8. Position-error deadband removed** - [altitudehold.cpp:375](../../../../src/main/flight/altitudehold.cpp#L375)

`applyDeadband ( error, 5 )` subtracts, so every error lost 5 cm and a 10 cm sag
entered the loop as 5.

**9. `P8[PIDALT]` 100 -> 128, EEPROM version 106 -> 107** - [config.cpp:187](../../../../src/main/config/config.cpp#L187), [config.cpp:153](../../../../src/main/config/config.cpp#L153)

128 is unity in `setVel = P8 * error / 128`. The EEPROM bump is required or stored
profiles keep the old gain.

**10. `errorVelocityI` not cleared on setpoint change** - [altitudehold.cpp:280](../../../../src/main/flight/altitudehold.cpp#L280), [altitudehold.cpp:301](../../../../src/main/flight/altitudehold.cpp#L301)

It carries the whole hover trim, so clearing it dropped the craft. Reset only on
real transitions ( BARO entry, disarm ).

**11. Hover-trim offload into the baseline** - [altitudehold.cpp:232](../../../../src/main/flight/altitudehold.cpp#L232)

`offloadHoverTrim ( )` moves the trim from the clamped integrator into
`initialThrottleHold` one count per `ALT_TRIM_OFFLOAD_MS` ( 20 ms ) while settled.
Confirmed in flight: `Ivel` stays ~0 while `Base` climbs ~47 counts/min as the
battery sags.

**12. Ceiling margin 50 -> 25 cm** - [altitudehold.cpp:73](../../../../src/main/flight/altitudehold.cpp#L73)

Named `ALT_CEILING_MARGIN_CM`.

### Diagnostics ( temporary )

**13. `PlutoPilot.cpp` telemetry** - [PlutoPilot.cpp](../../../../PlutoPilot.cpp)

One line every 100 ms, trimmed to the fields in [TESTING.md](TESTING.md). **Not
committed** - it stays in the local working copy for further test flights; the
committed `PlutoPilot.cpp` is the stock template. Its long header comment still
describes the old 19-field line and is stale.

The BMP280 reference barometer and STM32 internal temperature sensor used during
the September tests have been **removed completely** ( drivers deleted, Makefile,
target and `main.cpp` restored ). `LASER_TOF` is off on both targets again.

## File and line index

| What | Where |
|---|---|
| ISA pressure to altitude | [barometer.cpp:318](../../../../src/main/sensors/barometer.cpp#L318) |
| Ground altitude cache | [barometer.cpp:329](../../../../src/main/sensors/barometer.cpp#L329) |
| Zero tracking / freeze on arm | [barometer.cpp:351-369](../../../../src/main/sensors/barometer.cpp#L351-L369) |
| Throttle + temperature compensation | [barometer.cpp:371-414](../../../../src/main/sensors/barometer.cpp#L371-L414) |
| Altitude entry point | [barometer.cpp:416](../../../../src/main/sensors/barometer.cpp#L416) |
| ICP measurement mode calls | [barometer.cpp:100](../../../../src/main/sensors/barometer.cpp#L100), [258](../../../../src/main/sensors/barometer.cpp#L258), [265](../../../../src/main/sensors/barometer.cpp#L265) |
| `getBaroZeroOffset ( )` decl | [barometer.h:63](../../../../src/main/sensors/barometer.h#L63) |
| Signed OTP constants | [barometer_icp10111.cpp:155](../../../../src/main/drivers/barometer_icp10111.cpp#L155) |
| Temperature IIR | [barometer_icp10111.cpp:276-320](../../../../src/main/drivers/barometer_icp10111.cpp#L276-L320) |
| Ceiling margin | [altitudehold.cpp:73](../../../../src/main/flight/altitudehold.cpp#L73) |
| Trim offload | [altitudehold.cpp:232](../../../../src/main/flight/altitudehold.cpp#L232) |
| Integrator reset sites ( disabled ) | [altitudehold.cpp:280](../../../../src/main/flight/altitudehold.cpp#L280), [altitudehold.cpp:301](../../../../src/main/flight/altitudehold.cpp#L301) |
| Deadband removal | [altitudehold.cpp:375](../../../../src/main/flight/altitudehold.cpp#L375) |
| ToF vs baro source switch | [altitudehold.cpp:610](../../../../src/main/flight/altitudehold.cpp#L610) |
| Baro correction into estimator | [altitudehold.cpp:681](../../../../src/main/flight/altitudehold.cpp#L681) |
| Tilt threshold | [altitudehold.cpp:698](../../../../src/main/flight/altitudehold.cpp#L698) |
| `limitAltitude ( )` | [altitudehold.cpp:805](../../../../src/main/flight/altitudehold.cpp#L805) |
| `errorVelocityI` export | [altitudehold.h:77](../../../../src/main/flight/altitudehold.h#L77) |
| In-flight re-zero guard | [mw.cpp:174](../../../../src/main/mw.cpp#L174), [396](../../../../src/main/mw.cpp#L396), [555](../../../../src/main/mw.cpp#L555), [592](../../../../src/main/mw.cpp#L592) |
| EEPROM version 107 | [config.cpp:153](../../../../src/main/config/config.cpp#L153) |
| `P8[PIDALT] = 128` | [config.cpp:187](../../../../src/main/config/config.cpp#L187) |
| UART TX overwrite | [serial_uart.c:301](../../../../src/main/drivers/serial_uart.c#L301) |
| Diagnostic telemetry | [PlutoPilot.cpp](../../../../PlutoPilot.cpp) |
