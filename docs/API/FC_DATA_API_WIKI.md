# MagisV2 FC-Data API Wiki

Sensor readings and flight estimates for user code, all from `API/FC-Data.h`:

- `Sensor_Get` ( two overloads, `int32_t` ): barometric pressure and temperature, and the accelerometer, gyroscope and
  magnetometer axes.
- `Estimate_Get` ( two overloads, `int16_t` ): the rotation rate, position and velocity per axis, and the roll, pitch
  and heading angles.

API version **1.8.2**. See "Changes in 1.8.2" and "Changes in 1.8.1" for what differs from earlier versions.

## Quick Start

```cpp
#include "PlutoPilot.h"

void plutoLoop ( void ) {
    // Barometer: temperature in deg C x100, pressure in Pa
    Monitor_Print ( "T x100:", ( int ) Sensor_Get ( Barometer, Temperature ) );
    Monitor_Println ( " Pa:", ( int ) Sensor_Get ( Barometer, Pressure ) );
}
```

---

## Barometer: `Sensor_Get ( Barometer, data )`

```cpp
int32_t Sensor_Get ( FC_Sensors_e _sensor, BARO_Data_e _data );
```

| `data` | Unit | What it returns | Example |
|--------|------|-----------------|---------|
| `Pressure` | Pa ( = mbar x100 = hPa x100 ) | Pressure at the board, rounded to the nearest Pa. | `101242` = 1012.42 mbar |
| `Temperature` | deg C x100 | Temperature of the barometer chip, rounded to the nearest 0.01 C. | `3270` = 32.70 C |

- **Same data altitude hold uses.** Both values come from the ICP-10111 driver through `getBaroPressure ( )` and
  `getBaroTemperature ( )` ( `sensors/barometer.cpp` ). The temperature is the value the altitude-hold temperature
  compensation reads, multiplied by 100.
- **It is the chip temperature, not the air.** The barometer sits on a board that warms up after power-on, so it reads
  a few degrees above the room and rises for the first minutes. In a bench log it rose 0.7 C between 40 s and 93 s after boot.
- **Resolution.** Temperature changes in steps of 0.01 C ( the sensor resolves about 0.003 C ); pressure in steps of
  1 Pa. On a still bench the pressure holds within a few Pa.
- **Fallback.** Any `_sensor` other than `Barometer`, or an unknown `_data`, returns 0. Before the first barometer
  reading after power-up both values are 0.
- **To get degrees with decimals,** divide as float with an explicit cast: `( float ) Sensor_Get ( Barometer, Temperature ) / 100.0f` ( without the cast
  `-Wconversion` warns about int32 to float ). To print it, print
  the integer x100 value: the `Monitor_Print` double overload prints only the first decimal correctly.

## Accelerometer, gyroscope, magnetometer: `Sensor_Get ( sensor, axis )`

```cpp
int32_t Sensor_Get ( FC_Sensors_e _sensor, axis_e _axis );
```

`_axis` is `X`, `Y` or `Z`, or `Net_Acc` for the accelerometer only.

| `sensor` | `axis` | Unit | What it returns |
|----------|--------|------|-----------------|
| `Accelerometer` | `X` / `Y` / `Z` | cm/s^2 | Low-pass filtered acceleration on that axis, gravity included ( `accSmooth x 9.80665 / acc_1G x 100`, truncated ). 980 on the axis pointing up at rest ( exactly 1 g truncates to 980 ). |
| `Accelerometer` | `Net_Acc` | ( g^2 ) x100 | The **squared** magnitude of the filtered acceleration over all three axes, in hundredths of g^2. 100 at rest ( 1 g ), 400 at 2 g; integer truncation can read 1 low ( 99, 399 ). |
| `Gyroscope` | `X` / `Y` / `Z` | deg/s x10 | Bias-corrected, board-aligned gyro counts divided by 1.64, truncated ( `900` = 90 deg/s ). The same rate as `Estimate_Get ( Rate, axis )`, which rounds instead, with the same signs. 0 while the gyro calibrates after power-up. |
| `Magnetometer` | `X` / `Y` / `Z` | not verified | Calibration-scaled magnetometer value divided by 6.82, truncated. **0 on a board whose compass has never been calibrated in the app** ( the stored scale defaults to 0 ). |
| any other | - | - | 0 |

**Known gaps** ( not yet fixed, do not rely on them ):

- `Gyroscope` or `Magnetometer` with `Net_Acc` is undefined behaviour: the function ends without a `return` on that
  path. Pass only `X`, `Y` or `Z` for these two sensors.
- The magnetometer unit is not confirmed against the AK09916 driver scale: nothing records where 6.82 comes from. Use
  it as a relative value ( sign, change ). The gyro unit **is** confirmed: the gyro runs at +/-2000 deg/s
  ( 16.4 counts per deg/s ), the scale the flown attitude estimate integrates, and a bench turn of 360 deg integrated
  to 354-387 deg from the same gyro rate ( through `Estimate_Get ( Rate, axis )` ). The 131 counts per deg/s declared in the gyro driver
  ( `gyro.scale` ) is stale; the default `pidRewrite` controller and the DCM attitude do not use it ( only the optional
  `pidLuxFloat` controller and the blackbox header read it ).
- All axis values are 0 until the first IMU update after power-up ( so also inside `plutoInit` ), and 0 when the
  sensor is not detected.

## Rate, position, velocity: `Estimate_Get ( estimate, axis )`

```cpp
int16_t Estimate_Get ( FC_Estimate_e _estimateOf, axis_e _axis );
```

| `estimate` | `axis` | Unit | What it returns |
|------------|--------|------|-----------------|
| `Rate` | `X` / `Y` / `Z` | deg/s x10 | The measured rotation rate from the gyro: roll rate on `X`, pitch rate on `Y`, yaw rate on `Z`. Bias-corrected gyro counts divided by 1.64, rounded to nearest ( `900` = 90 deg/s ). Unfiltered: the value the rate loop uses. |
| `Position` | `Z` | cm | Estimated altitude ( `EstAlt` ), the value altitude hold flies. About 0 on the ground: the baro reference tracks the ground while disarmed and is frozen at arming. |
| `Velocity` | `Z` | cm/s | Estimated vertical velocity ( `VelocityZ` ), climbing positive. |
| `Position` | `X` / `Y` | cm | Horizontal position estimate, whole cm. **0 unless a UWB positioning sensor feeds the estimator.** |
| `Velocity` | `X` / `Y` | cm/s | Horizontal velocity estimate. **0 unless UWB.** |
| any other | - | - | 0 ( also `Net_Acc` on any estimate ) |

**Sign of each rate axis** ( measured on the bench, `PRIMUS_X2_v1` ):

| Rate | Positive when | Compared with the angle |
|------|---------------|-------------------------|
| `X` | rolling right ( right side going down ) | same sign as `Angle AG_ROLL` |
| `Y` | pitching nose down | same sign as `Angle AG_PITCH` |
| `Z` | turning **left** ( anticlockwise seen from above ) | **opposite** to the heading: a right turn reads negative while `Angle AG_YAW` counts up |

- **Yaw sign.** `Rate Z` is the gyro's own sign, so it is opposite to the heading and to
  `DesiredAngle_setRelative ( AG_YAW, + )`, where clockwise is positive. Negate it when you compare it with a heading
  change: `int32_t yawRateCw = - Estimate_Get ( Rate, Z );` ( `int32_t`: the negation is an `int`, and storing it back in an
  `int16_t` makes `-Wconversion` warn ).
- **Range.** About +/-19980 ( +/-1998 deg/s ), well inside `int16_t`. At rest it reads within a few counts of 0.
- **Rate is not the stick-rate setting.** Up to 1.8.1 `Rate` returned the stored rate profile ( `75` on every axis );
  see "Changes in 1.8.2".
- **When it is 0.** Before the first IMU update after power-up ( so also inside `plutoInit` ), and while the gyro
  calibrates.

## Angles: `Estimate_Get ( Angle, angle )`

```cpp
int16_t Estimate_Get ( FC_Estimate_e _estimateOf, angle_e _angle );
```

| `angle` | Unit | Range | Positive when |
|---------|------|-------|---------------|
| `AG_ROLL` | deci-degrees | -1800 .. 1800 | right side down ( `100` = 10 deg ) |
| `AG_PITCH` | deci-degrees | -900 .. 900 | nose down |
| `AG_YAW` | whole degrees | 0 .. 359 | heading, rising clockwise seen from above |

`_estimateOf` must be `Angle`; anything else returns 0. Unchanged in 1.8.2.

## Examples

### Print the barometer once when Developer Mode starts ( `onLoopStart` )

```cpp
void onLoopStart ( void ) {
    Monitor_Print ( "Baro T x100:", ( int ) Sensor_Get ( Barometer, Temperature ) );
    Monitor_Println ( " Pa:", ( int ) Sensor_Get ( Barometer, Pressure ) );
}
```

### Watch the board warm up ( `plutoLoop` )

```cpp
static int32_t startTx100 = 0;

void onLoopStart ( void ) {
    startTx100 = Sensor_Get ( Barometer, Temperature );
}

void plutoLoop ( void ) {
    int32_t rise = Sensor_Get ( Barometer, Temperature ) - startTx100;    // deg C x100 since Dev Mode on
    Monitor_Println ( "Rise x100:", ( int ) rise );
}
```

### Vertical acceleration ( `plutoLoop` )

```cpp
void plutoLoop ( void ) {
    Monitor_Println ( "AccZ cm/s^2:", ( int ) Sensor_Get ( Accelerometer, Z ) );
}
```

### Rotation rate and attitude ( `plutoLoop` )

```cpp
void plutoLoop ( void ) {
    // Rates in deg/s x10: roll right +, pitch nose down +, yaw right - ( opposite to the heading )
    Monitor_Print ( "Rate R:", ( int ) Estimate_Get ( Rate, X ) );
    Monitor_Print ( " P:", ( int ) Estimate_Get ( Rate, Y ) );
    Monitor_Print ( " Y:", ( int ) Estimate_Get ( Rate, Z ) );
    Monitor_Println ( " Hdg:", ( int ) Estimate_Get ( Angle, AG_YAW ) );
}
```

### Altitude and climb rate ( `plutoLoop` )

```cpp
void plutoLoop ( void ) {
    Monitor_Print ( "Alt cm:", ( int ) Estimate_Get ( Position, Z ) );
    Monitor_Println ( " Vz cm/s:", ( int ) Estimate_Get ( Velocity, Z ) );
}
```

### Count clockwise turns from the yaw rate ( `plutoInit`, `onLoopStart`, `plutoLoop` )

```cpp
static int32_t turnedDeciDeg = 0;    // deci-degrees turned clockwise since power-up
static uint32_t lastMs = 0;

void plutoInit ( void ) {
    turnedDeciDeg = 0;    // zeroed once at power-up, not on every Developer Mode start
}

void onLoopStart ( void ) {
    lastMs = millis ( );    // do not count the time spent outside the loop
}

void plutoLoop ( void ) {
    uint32_t nowMs = millis ( );
    int32_t yawRateCw = - Estimate_Get ( Rate, Z );    // deg/s x10, clockwise positive like the heading
    turnedDeciDeg += yawRateCw * ( int32_t ) ( nowMs - lastMs ) / 1000;
    lastMs = nowMs;
    Monitor_Println ( "Turned deg:", ( int ) ( turnedDeciDeg / 10 ) );
}
```

The count is zeroed in `plutoInit`, not in `onLoopStart`: over Wi-Fi a late RC frame re-runs `onLoopStart` ( in a
bench log about every 5 s ), which would reset it in the middle of a turn. Turning while Developer Mode is off is not
counted. The integer division drops up to 0.1 deg per call, so use it for whole turns, not for precise heading.

## Changes in 1.8.2

- `Estimate_Get ( Rate, X | Y | Z )` returns the **measured rotation rate in deg/s x10** from the gyro, rounded to
  nearest. Up to 1.8.1 it returned the stored stick-rate setting of the rate profile ( `75` on every axis, whatever
  the drone did ). User code that used it as a setting must read it some other way; code that wanted the rate now gets
  it. The signs are in the table above: note that yaw right is negative.
- The header documents both `Estimate_Get` overloads as `int16_t` ( the declarations always were ) and gives the unit
  of every case.
- `Sensor_Get ( Gyroscope, axis )` is documented as deg/s x10 ( the scale was confirmed, the code is unchanged ).

## Changes in 1.8.1

- `Sensor_Get ( Barometer, Temperature )` returns **deg C x100**, as the header always documented. Up to 1.8.0 it
  returned whole degrees ( `32` for 32.70 C ): the float temperature was truncated to an integer and never scaled.
  **User code written against 1.8.0 that expects whole degrees must now divide by 100.**
- `Sensor_Get ( Barometer, Pressure )` is rounded to the nearest Pa instead of truncated ( at most 1 Pa different ).
  The unit is unchanged: Pa, which the header used to call "100*millibar".

## File Architecture

| File | Contents |
|------|----------|
| `src/main/API/FC-Data.h` | `FC_Sensors_e`, `BARO_Data_e`, `FC_Estimate_e`, both `Sensor_Get` and both `Estimate_Get` declarations |
| `src/main/API-Src/FC-Data-Sensors.cpp` | Both `Sensor_Get` implementations |
| `src/main/API-Src/FC-Data-Estimate.cpp` | Both `Estimate_Get` implementations |
| `src/main/sensors/gyro.cpp` | `gyroADC [ ]`: bias-corrected, board-aligned gyro counts ( 16.4 per deg/s ) |
| `src/main/flight/altitudehold.cpp` | `getEstAltitude ( )` ( cm ), `getEstVelocity ( )` ( cm/s ) |
| `src/main/flight/posEstimate.cpp` | `PositionX / Y` ( cm ), `VelocityX / Y` ( cm/s ), UWB only |
| `src/main/sensors/barometer.cpp` | `getBaroPressure ( )` ( Pa ), `getBaroTemperature ( )` ( deg C ) |
| `src/main/drivers/barometer_icp10111.cpp` | ICP-10111 driver: pressure in Pa, temperature in deg C, as floats |
| `src/main/flight/imu.cpp` | `accSmooth`, `accVelScale`, `netAccMagnitude` |
