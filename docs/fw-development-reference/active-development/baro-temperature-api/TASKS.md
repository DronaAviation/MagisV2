# Baro Temperature API - Tasks

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

## Resume

Read this block first in a new session; read only the task it points to.

| | |
|---|---|
| **Current task** | none: **Closed** in `7a41a37` ( 3 Nov 2026 ) |
| **Next step** | none: committed in `7a41a37` |
| **Open questions** | Defaults taken unattended: x100 rounded half away from zero, API 1.8.1, new `FC_DATA_API_WIKI.md`. Confirm or correct. |
| **Blocked on** | nothing |
| **Last updated** | 2026-11-03 |

## Summary

| | |
|---|---|
| **Mode** | fix |
| **Goal** | `Sensor_Get ( Barometer, Temperature )` returns deg C x100 as its header promises, with no change to the internal deg C path. |
| **Done when** | A bench `Monitor_Print` of the two values shows about 3270 for a 32.7 C room and about 100470 Pa, on `PRIMUS_X2_v1`; build gate clean. |
| **Target** | PRIMUS_X2_v1 ( PRIMUS_V5 built at commit ) |
| **Analysis** | Causal chain proven ( INVESTIGATION.md ): float deg C -> implicit float -> int32 at `FC-Data-Sensors.cpp:90`, no x100. Option ( a ) chosen: scale and round in the API layer only. |
| **Pipeline docs affected** | subsystems/User_Space_API.md ( one line ); new docs/API/FC_DATA_API_WIKI.md; CHANGELOG at commit |
| **Order** | 1 -> 2 -> 3 -> 4 -> 5 -> 6 |

## Index

| # | Title | Status | Depends on | Skills / agent |
|---|---|---|---|---|
| 1 | Scale and round the baro returns in `Sensor_Get`, fix the header comment | done 2026-11-03 | - | pluto-rules |
| 2 | Build gate | done 2026-11-03 | 1 | pluto-build |
| 3 | Bench validation of the two values | done 2026-11-03 | 2 | pluto-flighttest |
| 4 | Architecture & pipeline docs ( PIPELINE_UPDATE.md, FC-Data wiki ) | done 2026-11-03 | 3 | spec-to-code-compliance |
| 5 | Topic review | done 2026-11-03 | 4 | pluto-reviewer agent |
| 6 | Graph refresh & commit ( FW 3.15.3, API 1.8.1 ) | done 2026-11-03 | 5 | graphify, pluto-commit |

Status values: `todo`, `in-progress`, `done YYYY-MM-DD`, `blocked (<why>)`, `dropped (<why>)`.
Serial numbers are never reused or renumbered.

## Tasks

### 1. Scale and round the baro returns in `Sensor_Get`, fix the header comment

**Description.** In `Sensor_Get ( FC_Sensors_e, BARO_Data_e )` replace the two implicit float -> int32 returns with
explicit rounded conversions: `Temperature` returns `roundToInt32 ( getBaroTemperature ( ) * 100.0f )`
( deg C x100 ), `Pressure` returns `roundToInt32 ( getBaroPressure ( ) )` ( Pa, value unchanged ); `roundToInt32` is a
file-local float helper ( see D2 amended ). Update the
in-function comments and the `FC-Data.h` doc block ( lines 79-80 ) to "pressure in Pa ( = mbar x100 )" and
"temperature in deg C x100 ( 3270 = 32.70 C )". Do not touch `getBaroTemperature ( )`, `barometer.cpp` or the
ICP-10111 driver: the altitude-hold compensation needs deg C. Relies on decisions D1-D4.

- **Depends on:** -
- **Skills / agent:** pluto-rules
- **Files:** [FC-Data-Sensors.cpp](../../../../src/main/API-Src/FC-Data-Sensors.cpp), [FC-Data.h](../../../../src/main/API/FC-Data.h)
- **Safety impact:** none ( user-code getter only; no firmware caller )
- **Done when:** `make TARGET=PRIMUS_X2_v1` builds; the two `-Wfloat-conversion` warnings at these lines are gone; no new warning under `src/`.
- **Rollback:** revert the two return lines ( whole degrees again ).
- **Status:** done 2026-11-03
- **Result:** `roundToInt32` helper, x100 temperature, rounded Pa, header comment fixed; gate green on `PRIMUS_X2_v1` ( 121.1 KB flash, 20.5 KB RAM, two baselined `-Wfloat-conversion` gone ); reviewer: no blocking finding, doc wording and one comment fixed.

### 2. Build gate

**Description.** Run `.claude/skills/pluto-build/driver.sh --gate PRIMUS_X2_v1` ( `PROJECT=AltX` so PlutoIDE can
flash the hex ). The baseline lists the two warnings removed by task 1; fewer warnings than the baseline must pass.
Never regenerate the baseline. Record the flash / RAM delta ( expected: a few dozen bytes of flash, 0 RAM ).

- **Depends on:** 1
- **Skills / agent:** pluto-build
- **Files:** -
- **Safety impact:** none
- **Done when:** gate passes with no new `src/` warning; flash and RAM fit; numbers noted in CHANGES.md.
- **Rollback:** n/a
- **Status:** done 2026-11-03
- **Result:** gate green on `PRIMUS_X2_v1`, no new `src/` warning; flash 121.1 KB, RAM 20.5 KB, identical to HEAD ( `Sensor_Get` is uncalled and removed by `-gc-sections`; 72 B when user code calls it ).

### 3. Bench validation of the two values

**Description.** Add a temporary `Monitor_Print` in `plutoLoop ( )` of `PlutoPilot.cpp` printing both integers
( under 130 bytes per tick; print the int, not a double ). Bench, disarmed, app connected: read the two values and
compare against a reference thermometer where available ( within 1 C ) and the earlier 100470 Pa reading
( within 200 Pa ). Record in TESTING.md, then restore `PlutoPilot.cpp` to HEAD.

- **Depends on:** 2
- **Skills / agent:** pluto-flighttest
- **Files:** [PlutoPilot.cpp](../../../../PlutoPilot.cpp) ( temporary )
- **Safety impact:** none ( bench, disarmed )
- **Done when:** TESTING.md shows temperature as a 4-digit x100 value ( e.g. 3270 ) that varies in the hundredths over time, pressure about 100470; `PlutoPilot.cpp` back to HEAD.
- **Rollback:** n/a
- **Status:** done 2026-11-03
- **Result:** log-1 ( 498 records, 53 s ): `Tx100` 3500-3574 in steps of 1, within +/-5 of `degC` x100; `Pa` 101242 +/-1; finger step and thermometer not shown. `PlutoPilot.cpp` restored to HEAD.

### 4. Architecture & pipeline docs ( PIPELINE_UPDATE.md, FC-Data wiki )

**Description.** Write `PIPELINE_UPDATE.md` with the new version of the `Sensor_Get` line for
`fw-architecture-pipeline/subsystems/User_Space_API.md` ( units: Pa; deg C x100 ). Create `docs/API/FC_DATA_API_WIKI.md`
in the layout of the other wikis, covering `Sensor_Get` ( both overloads: the baro overload fully with units and an
example; the axis overload listed with the units found, and the known gaps from the README noted ). Check the wiki
against the code with `spec-to-code-compliance`. CHANGELOG and the Makefile bump are done by task 6. No Mermaid
change: no flow changed.

- **Depends on:** 3
- **Skills / agent:** spec-to-code-compliance
- **Files:** PIPELINE_UPDATE.md ( this folder ), [docs/API/](../../../API/)
- **Safety impact:** none
- **Done when:** PIPELINE_UPDATE.md and the wiki exist; every stated unit matches a source line.
- **Rollback:** n/a
- **Status:** done 2026-11-03
- **Result:** `PIPELINE_UPDATE.md` ( one `User_Space_API.md` bullet, commit notes ) and `docs/API/FC_DATA_API_WIKI.md` written. Four spec-compliance checks: baro, accel axes, Net_Acc implemented; gyro / mag partial ( "raw" wording wrong, mag reads 0 uncalibrated ), wiki corrected. Side findings in README.

### 5. Topic review

**Description.** `pluto-reviewer` agent on the whole topic diff ( code and docs ), with the pluto-rules checklist.
Fix BLOCKING findings and re-run the gate ( task 2 ) before moving on.

- **Depends on:** 4
- **Skills / agent:** pluto-reviewer agent
- **Files:** topic diff
- **Safety impact:** none
- **Done when:** no open BLOCKING finding; re-gate passed if code changed.
- **Rollback:** n/a
- **Status:** done 2026-11-03
- **Result:** `pluto-reviewer` on the whole topic diff: 0 blocking, 2 should-fix ( stale index row; wiki example needed `( float )` cast ), 5 doc nits; all applied. No code changed, so no re-gate. Reviewer confirmed all line references, the wiki rows and the log-1 statistics.

### 6. Graph refresh & commit ( FW 3.15.3, API 1.8.1 )

**Description.** `graphify update .` then `python tools/graph_labels.py`. Then `/pluto-commit`: all-target build,
Makefile bump to FW 3.15.3 / API 1.8.1, promote PIPELINE_UPDATE.md, CHANGELOG entry ( API 1.8.1 is behaviour only:
`Sensor_Get ( Barometer, Temperature )` now deg C x100; user code that read whole degrees must divide by 100 ),
topic set Closed, staged with a drafted message. The user commits.

- **Depends on:** 5
- **Skills / agent:** graphify, pluto-commit
- **Files:** Makefile, CHANGELOG.md, graphify-out/
- **Safety impact:** none
- **Done when:** staged tree ready; topic Closed in both READMEs.
- **Rollback:** n/a
- **Status:** done 2026-11-03
- **Result:** graph refreshed ( 5111 nodes ) and labelled; all three targets built, no new `src/` warnings ( PRIMUS_X2_v1 121.1 KB / 20.5 KB, PRIMUSX2 120.9 / 20.7, PRIMUS_V5 121.1 / 20.5 ); FW 3.15.3, API 1.8.1; `User_Space_API.md` and CHANGELOG updated; staged with a drafted message.

## Decisions log

Newest last. One line each: `YYYY-MM-DD [decision|assumption|out-of-scope|risk] text`.

- 2026-11-03 [decision] Mode `fix`: the user asked to "resolve this issue in the api"; cause proven by the scout from the code, no reproduction task needed.
- 2026-11-03 [assumption] Target `PRIMUS_X2_v1` ( `selected_target` in `plutoide.ini` ); the session ran unattended, so no interview round was answered.
- 2026-11-03 [decision] D1: option ( a ), code matches the header: return deg C x100. Option ( b ) ( document whole degrees ) rejected.
- 2026-11-03 [decision] D2: round to nearest with `lrintf`, single precision ( `100.0f` ).
- 2026-11-03 [decision] D3: pressure stays Pa; header reworded "Pa ( = mbar x100 )"; explicit cast added.
- 2026-11-03 [decision] D4: internal unit of `baroTemperature` / `getBaroTemperature ( )` stays deg C ( altitude-hold compensation and calibration depend on it ).
- 2026-11-03 [decision] API 1.8.1 ( behaviour-only change, precedent API 1.4.1 ), FW 3.15.3.
- 2026-11-03 [decision] Create `docs/API/FC_DATA_API_WIKI.md` ( none exists ), limited to `Sensor_Get`.
- 2026-11-03 [out-of-scope] Axis overload `-Wreturn-type` undefined behaviour, `uint16_t` vs `int16_t` doc drift, gyro / mag units: noted in README for a later topic.
- 2026-11-03 [risk] The two fixed warnings remain listed in `tools/warnings-baseline.json`; the gate must accept fewer warnings than the baseline. If the driver treats a vanished baseline entry as an error, report it rather than regenerate.
- 2026-11-03 [decision] Wiki axis-overload rows state only what the code proves: accel in cm/s^2 and ( g^2 ) x100 confirmed; gyro / mag units left "not verified" ( driver scale conflict 1/131 vs 16.4 ); mag documented as 0 until calibrated.
- 2026-11-03 [out-of-scope] Gyro scale 8x conflict in `accgyro_icm20948.cpp`, uncalibrated mag reads 0, `baroInit` without detection check: recorded in README for a later `check` topic.
- 2026-11-03 [decision] D2 amended: `lrintf` dropped. `#include <math.h>` in `FC-Data-Sensors.cpp` adds a 19th baselined `"M_PI" redefined` warning ( `flight/pid.h` ) and fails the gate. A file-local `roundToInt32 ( float )` ( +/-0.5 then explicit cast ) rounds to nearest instead. The gate confirmed fewer-than-baseline warnings pass ( "fixed in src/" is informational ).
- 2026-11-03 [decision] Closed - pending commit: FW 3.15.2 -> 3.15.3 ( patch, a fix ), API 1.8.0 -> 1.8.1 ( behaviour only ); all targets built; `User_Space_API.md` and CHANGELOG updated; warnings baseline not refreshed.
