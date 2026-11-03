# Baro Temperature API - Changes

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

What changed and how. Why is in [INVESTIGATION.md](INVESTIGATION.md).

## Task 1 ( 3 Nov 2026 ): scale and round the baro returns in `Sensor_Get`

### [FC-Data-Sensors.cpp](../../../../src/main/API-Src/FC-Data-Sensors.cpp)

- New file-local helper `roundToInt32 ( float )` just above `Sensor_Get ( FC_Sensors_e, BARO_Data_e )`:
  `( int32_t ) ( value + ( value >= 0.0f ? 0.5f : -0.5f ) )`, round to nearest, away from zero at .5. It exists
  because `lrintf` needs `<math.h>`, and including `<math.h>` in this translation unit adds a 19th baselined
  `"M_PI" redefined` warning from `flight/pid.h` ( the gate fails on it; the baseline is not to be regenerated ).
- `case Pressure`: `return roundToInt32 ( getBaroPressure ( ) );` ( Pa, same value as before, now rounded instead
  of truncated ). Comment: "Pa ( = mbar x100 )".
- `case Temperature`: `return roundToInt32 ( getBaroTemperature ( ) * 100.0f );` ( deg C x100; was the whole
  degrees of an implicit float -> int32 truncation ). Comment: "deg C x100 ( 3270 = 32.70 C )".
- Banner `Last Modified` set to 3 Nov 2026.

### [FC-Data.h](../../../../src/main/API/FC-Data.h)

- `@return` of the `BARO_Data_e` overload now reads "pressure in Pa ( = mbar x100, e.g. 100470 ), temperature in
  deg C x100 ( e.g. 3270 = 32.70 C ), both rounded to nearest". Signature unchanged.
- Banner `Last Modified` set to 3 Nov 2026.

### Not changed

`barometer.cpp` ( `baroTemperature`, `getBaroTemperature ( )` stay deg C: the altitude-hold temperature
compensation and calibration use them ), the ICP-10111 driver, the warnings baseline.

## Build ( task 1, gate on `PRIMUS_X2_v1` )

| | |
|---|---|
| Flash | 121.1 KB / 256 KB ( 47% ); text 122752 B |
| RAM | 20.5 KB / 40 KB ( 51% ) |
| Warning gate | no new `src/` warning; the two `-Wfloat-conversion` at the old lines 87 / 90 are gone |

## Build gate ( task 2, 3 Nov 2026, `PRIMUS_X2_v1` )

Re-run after the review's comment fix. `driver.sh --gate PRIMUS_X2_v1`: **no new `src/` warnings**, all builds passed.

| | HEAD ( `a7f398d` ) | This change |
|---|---|---|
| text / data / bss ( B ) | 122752 / 1316 / 19728 | 122752 / 1316 / 19728 |
| Flash | 121.1 KB / 256 KB | 121.1 KB / 256 KB |
| RAM | 20.5 KB / 40 KB | 20.5 KB / 40 KB |

The delta is 0 because `Sensor_Get` has no caller in the current `PlutoPilot.cpp` and the link uses
`-Wl,-gc-sections` ( Makefile:447 ): the function is not in the ELF in either build. In the object file the fixed
`Sensor_Get ( FC_Sensors_e, BARO_Data_e )` is 0x48 = 72 B ( `roundToInt32` inlined ); that is what user code that
calls it pays in flash. RAM cost is 0.

Note for task 3: a plain `make` in Git Bash needs the PlutoIDE toolchain on PATH
( `/c/PlutoIDE/tools/ARM GNU ToolChain/bin` ); `driver.sh` adds it itself.

## File index

| File | Change |
|---|---|
| `src/main/API-Src/FC-Data-Sensors.cpp` | `roundToInt32` helper; x100 and rounding in `Sensor_Get ( ..., BARO_Data_e )` |
| `src/main/API/FC-Data.h` | `@return` units corrected |
