# Baro Temperature API - Pipeline update

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

Staged text for the pipeline docs. Apply at commit ( `pluto-commit` ); do not edit `fw-architecture-pipeline/` before.

## 1. `fw-architecture-pipeline/subsystems/User_Space_API.md`, section "Primary Functions"

**Insert** after the bullet that starts `` - `Estimate_Get ( Position | Velocity | Angle | Rate, axis )` `` ( currently lines 27-28 ):

```markdown
- `Sensor_Get ( Barometer, Pressure | Temperature )` ( `API/FC-Data.h` ): pressure in Pa ( = mbar x100 ), temperature
  in deg C x100 ( `3270` = 32.70 C ), both rounded to nearest ( `FC-Data-Sensors.cpp:84-100` ). The same
  `getBaroPressure ( )` / `getBaroTemperature ( )` values altitude hold reads; only the API layer scales them.
  `Sensor_Get ( Accelerometer | Gyroscope | Magnetometer, axis )`: accelerometer axes in cm/s^2, `Net_Acc` the squared
  magnitude in ( g^2 ) x100. Wiki `docs/API/FC_DATA_API_WIKI.md` ( API 1.8.1 ).
```

No other line of the doc changes. No flowchart changes: no call path moved.

## 2. `fw-architecture-pipeline/subsystems/Altitude_Hold_Estimator.md`

No change. `baroTemperature` / `getBaroTemperature ( )` stay in deg C; only `Sensor_Get` scales its return.

## 3. Hardware maps

No change: no pin, DMA channel or timer is touched.

## 4. At commit ( for `pluto-commit` )

- `CHANGELOG.md`: API 1.8.1 is behaviour only: `Sensor_Get ( Barometer, Temperature )` now returns deg C x100
  ( was whole degrees ); pressure rounded instead of truncated. User code reading whole degrees must divide by 100.
  New wiki `docs/API/FC_DATA_API_WIKI.md`.
- Makefile: `FW_Version` 3.15.3, `API_Version` 1.8.1.
- `tools/warnings-baseline.json`: the two `FC-Data-Sensors.cpp -Wfloat-conversion` entries are fixed, and the gate
  reports 108 fixed entries in total ( the rest from earlier commits: `XRanging`, `common/utils.h`, `battery.cpp`,
  `FC-Config-Setpoints.cpp` ). A refresh drops all 108; if done, the commit message must say so. Never refresh to hide
  a new warning.
