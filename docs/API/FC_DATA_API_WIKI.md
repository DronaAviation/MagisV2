# MagisV2 FC-Data Sensor API Wiki

Raw sensor readings for user code: barometric pressure and temperature, and the accelerometer, gyroscope and
magnetometer axes. Everything comes from the two overloads of `Sensor_Get` in `API/FC-Data.h`, which return an
`int32_t`.

API version **1.8.1**. See "Changes in 1.8.1" for what differs from 1.8.0.

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
| `Gyroscope` | `X` / `Y` / `Z` | not verified | Bias-corrected, board-aligned gyro counts divided by 1.64, truncated. 0 while the gyro calibrates after power-up. |
| `Magnetometer` | `X` / `Y` / `Z` | not verified | Calibration-scaled magnetometer value divided by 6.82, truncated. **0 on a board whose compass has never been calibrated in the app** ( the stored scale defaults to 0 ). |
| any other | - | - | 0 |

**Known gaps** ( not yet fixed, do not rely on them ):

- `Gyroscope` or `Magnetometer` with `Net_Acc` is undefined behaviour: the function ends without a `return` on that
  path. Pass only `X`, `Y` or `Z` for these two sensors.
- The gyro and magnetometer units are not confirmed against the ICM-20948 / AK09916 driver scales. The constant
  1.64 suggests deg/s x10 at 16.4 counts per deg/s, but the driver declares 131 counts per deg/s ( an 8x conflict in
  the code itself ), and nothing records where 6.82 comes from. Use them as relative values ( sign, change ) until
  they are documented.
- All axis values are 0 until the first IMU update after power-up ( so also inside `plutoInit` ), and 0 when the
  sensor is not detected.

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

## Changes in 1.8.1

- `Sensor_Get ( Barometer, Temperature )` returns **deg C x100**, as the header always documented. Up to 1.8.0 it
  returned whole degrees ( `32` for 32.70 C ): the float temperature was truncated to an integer and never scaled.
  **User code written against 1.8.0 that expects whole degrees must now divide by 100.**
- `Sensor_Get ( Barometer, Pressure )` is rounded to the nearest Pa instead of truncated ( at most 1 Pa different ).
  The unit is unchanged: Pa, which the header used to call "100*millibar".

## File Architecture

| File | Contents |
|------|----------|
| `src/main/API/FC-Data.h` | `FC_Sensors_e`, `BARO_Data_e`, both `Sensor_Get` declarations |
| `src/main/API-Src/FC-Data-Sensors.cpp` | Both `Sensor_Get` implementations |
| `src/main/sensors/barometer.cpp` | `getBaroPressure ( )` ( Pa ), `getBaroTemperature ( )` ( deg C ) |
| `src/main/drivers/barometer_icp10111.cpp` | ICP-10111 driver: pressure in Pa, temperature in deg C, as floats |
| `src/main/flight/imu.cpp` | `accSmooth`, `accVelScale`, `netAccMagnitude` |
