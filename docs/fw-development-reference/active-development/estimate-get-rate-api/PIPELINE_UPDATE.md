# Estimate_Get Rate API - Pipeline update

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

Staged text for the pipeline docs. Apply at commit ( `pluto-commit` ); do not edit `fw-architecture-pipeline/` before.

## 1. `fw-architecture-pipeline/subsystems/User_Space_API.md`, section "Primary Functions"

**Replace** the bullet that starts `` - `Estimate_Get ( Position | Velocity | Angle | Rate, axis )` `` ( currently lines 27-28 ):

```markdown
- `Estimate_Get ( Position | Velocity | Angle | Rate, axis )` ( `API/FC-Data.h` ): `EstAlt` in cm, the Z velocity in
  cm/s, and the angles.
```

with:

```markdown
- `Estimate_Get ( Rate | Position | Velocity, axis )` ( `API/FC-Data.h`, `int16_t` ): `Rate` is the measured gyro rate
  in deg/s x10, `gyroADC [ axis ] / 1.64` rounded ( 16.4 counts per deg/s, the scale the DCM attitude integrates;
  `FC-Data-Estimate.cpp:74-91` ); X > 0 rolling right, Y > 0 nose down, Z > 0 turning left ( opposite to the heading ).
  `Position Z` is `EstAlt` in cm, `Velocity Z` is `VelocityZ` in cm/s ( climbing positive ); X / Y are the UWB
  position ( cm ) and velocity ( cm/s ), 0 without UWB. `Estimate_Get ( Angle, AG_ROLL | AG_PITCH | AG_YAW )`: roll and
  pitch in deci-degrees ( right side down / nose down positive ), heading in whole degrees 0-359, clockwise.
```

**In the next bullet** ( `` - `Sensor_Get ( Barometer, Pressure | Temperature )` `` ), replace its last sentence:

```markdown
  `Sensor_Get ( Accelerometer | Gyroscope | Magnetometer, axis )`: accelerometer axes in cm/s^2, `Net_Acc` the squared
  magnitude in ( g^2 ) x100. Wiki `docs/API/FC_DATA_API_WIKI.md` ( API 1.8.1 ).
```

with:

```markdown
  `Sensor_Get ( Accelerometer | Gyroscope | Magnetometer, axis )`: accelerometer axes in cm/s^2, `Net_Acc` the squared
  magnitude in ( g^2 ) x100, gyro axes in deg/s x10 ( truncated; the same rate as `Estimate_Get ( Rate )` ). Wiki
  `docs/API/FC_DATA_API_WIKI.md` ( API 1.8.2 ).
```

No other line of the doc changes. No flowchart changes: no call path moved ( `Estimate_Get` reads `gyroADC`, which
the rate loop already reads; nothing new calls it ).

## 2. Other pipeline docs

No change. The attitude, rate-loop and altitude-hold docs are untouched: no firmware path changed, only what the user
API returns.

## 3. Hardware maps

No change: no pin, DMA channel or timer is touched.

## 4. At commit ( for `pluto-commit` )

- `CHANGELOG.md` ( Fixed ): API 1.8.2 is behaviour only: `Estimate_Get ( Rate, X | Y | Z )` returns the measured
  rotation rate in deg/s x10 from the gyro ( was the stored stick-rate setting, `75` on every axis ). Yaw right reads
  negative. Header units and `int16_t` return documented for both overloads; the wiki gains the `Estimate_Get`
  sections and documents `Sensor_Get ( Gyroscope )` as deg/s x10.
- Makefile: `FW_Version` 3.15.4, `API_Version` 1.8.2.
- `tools/warnings-baseline.json`: no entry added or fixed by this topic ( the four `FC-Data-Estimate.cpp` warnings
  left are the baselined Position / Velocity conversions ). The gate's "108 fixed" carries over from earlier commits;
  see `baro-temperature-api/PIPELINE_UPDATE.md` §4.
