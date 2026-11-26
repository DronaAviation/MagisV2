# Estimate_Get Rate API - Tasks

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md)

## Resume

Read this block first in a new session; read only the task it points to.

| | |
|---|---|
| **Current task** | none: **Closed** in `1b43fbe` ( 5 Nov 2026 ) |
| **Next step** | none: committed in `1b43fbe` |
| **Open questions** | - |
| **Blocked on** | - |
| **Last updated** | 2026-11-05 |

## Summary

| | |
|---|---|
| **Mode** | fix |
| **Goal** | `Estimate_Get ( Rate, X | Y | Z )` returns the measured rotation rate in deg/s x10 instead of the stick-rate setting. |
| **Done when** | On the bench, `Estimate_Get ( Rate, axis )` reads about 0 at rest ( within +/-20 ), follows rotation on all three axes with the right sign, and one hand turn per axis integrates to 360 +/- 30 deg; build gate clean. |
| **Target** | PRIMUS_X2_v1 ( PRIMUS_V5 built at commit ) |
| **Analysis** | Causal chain proven ( INVESTIGATION.md ): the `Rate` case returns `currentControlRateProfile->rates []`, a stored setting ( 75 ). Option A chosen: rounded `gyroADC / 1.64f`. |
| **Pipeline docs affected** | subsystems/User_Space_API.md ( the `Estimate_Get` bullet ); docs/API/FC_DATA_API_WIKI.md; CHANGELOG at commit |
| **Order** | 1 -> 2 -> 3 -> 4 -> 5 -> 6 |

## Index

| # | Title | Status | Depends on | Skills / agent |
|---|---|---|---|---|
| 1 | Return the measured gyro rate from `Estimate_Get ( Rate, axis )`, fix the header doc | done 2026-11-04 | - | pluto-rules |
| 2 | Build gate | done 2026-11-04 | 1 | pluto-build |
| 3 | Bench validation: rest, sign and one-turn integral per axis | done 2026-11-05 | 2 | pluto-flighttest |
| 4 | Architecture & pipeline docs ( PIPELINE_UPDATE.md, FC-Data wiki ) | done 2026-11-05 | 3 | spec-to-code-compliance |
| 5 | Topic review | done 2026-11-05 | 4 | pluto-reviewer agent |
| 6 | Graph refresh & commit ( FW 3.15.4, API 1.8.2 ) | done 2026-11-05 | 5 | graphify, pluto-commit |

Status values: `todo`, `in-progress`, `done YYYY-MM-DD`, `blocked (<why>)`, `dropped (<why>)`.
Serial numbers are never reused or renumbered.

## Tasks

### 1. Return the measured gyro rate from `Estimate_Get ( Rate, axis )`, fix the header doc

**Description.** In `Estimate_Get ( FC_Estimate_e, axis_e )` replace the three `currentControlRateProfile->rates []`
returns of the `Rate` case with the measured rate in deg/s x10: `roundToInt16 ( gyroADC [ _axis ] / 1.64f )` for X, Y
and Z ( index 0/1/2 = roll/pitch/yaw, the `axis_e` order ). `roundToInt16` is a file-local float helper ( half away
from zero, clamped to the int16 range, explicit cast ), because `<math.h>` would add a baselined `M_PI` redefinition
warning ( this file includes `flight/pid.h` ). Correct the `FC-Data.h` doc block of both `Estimate_Get` overloads:
`@return int16_t`, and the unit of every case ( Rate deg/s x10; Position Z cm; Velocity Z cm/s; Position / Velocity X,
Y cm and cm/s, 0 without UWB; Angle deci-degrees, yaw whole degrees ). Do not touch the Position, Velocity or Angle
code, or `rates []` itself ( the flip code owns it ). Relies on decisions D1-D3.

- **Depends on:** -
- **Skills / agent:** pluto-rules
- **Files:** [FC-Data-Estimate.cpp](../../../../src/main/API-Src/FC-Data-Estimate.cpp), [FC-Data.h](../../../../src/main/API/FC-Data.h)
- **Safety impact:** none ( user-code getter only; no firmware caller of the Rate case )
- **Done when:** `make TARGET=PRIMUS_X2_v1` builds; no new warning under `src/`.
- **Rollback:** revert the Rate case ( returns the stick-rate setting again ).
- **Status:** done 2026-11-04
- **Result:** Rate = `roundToInt16 ( ( float ) gyroADC [ _axis ] / 1.64f )` ( deg/s x10 ); header `int16_t` and units per case, X / Y warning narrowed to UWB; gate green on `PRIMUS_X2_v1` ( 121.1 KB / 20.5 KB ); reviewer: no blocking, 2 nits ( one applied, the axis sign deferred to task 4 ).

### 2. Build gate

**Description.** Run `.claude/skills/pluto-build/driver.sh --gate PRIMUS_X2_v1` with `PROJECT=AltX` so PlutoIDE can
flash the hex. Never regenerate the baseline. Record the flash / RAM numbers ( expected: about 0, `Estimate_Get` has no
firmware caller and is stripped by `-gc-sections` unless user code calls it ).

- **Depends on:** 1
- **Skills / agent:** pluto-build
- **Files:** -
- **Safety impact:** none
- **Done when:** gate passes with no new `src/` warning; flash and RAM fit; numbers in CHANGES.md.
- **Rollback:** n/a
- **Status:** done 2026-11-04
- **Result:** gate green on `PRIMUS_X2_v1`, no new `src/` warning; flash 121.1 KB, RAM 20.5 KB, identical to HEAD ( `Estimate_Get` uncalled, stripped; 212 B when user code calls it ).

### 3. Bench validation: rest, sign and one-turn integral per axis

**Description.** Temporary `Monitor_Print` in `plutoLoop ( )` of `PlutoPilot.cpp`: `t`, `n`, the three
`Estimate_Get ( Rate, axis )` values and, as a cross-check, the three `Estimate_Get ( Angle, ... )` values ( under
130 bytes per tick ). Bench, disarmed, props off: 20 s still, then by hand one full turn about each axis in turn
( yaw flat on the table; roll and pitch held in the hand ), 10 s still between. Integrate each Rate over `t` per turn
with `tools/flightlog.py` or a script, and check the sign against the Angle change. Record in TESTING.md, then
restore `PlutoPilot.cpp` to HEAD.

- **Depends on:** 2
- **Skills / agent:** pluto-flighttest
- **Files:** [PlutoPilot.cpp](../../../../PlutoPilot.cpp) ( temporary )
- **Safety impact:** none ( bench, disarmed )
- **Done when:** TESTING.md shows each Rate about 0 at rest ( within +/-20 ), one turn integrating to 360 +/- 30 deg per axis ( an 8x scale error would give about 45 ), the sign matching the Angle change; `PlutoPilot.cpp` back to HEAD.
- **Rollback:** n/a
- **Status:** done 2026-11-05
- **Result:** PASS ( log-1 ): rest -0.9 / -1.8 / -0.7 ( all within +/-5 ); one turn = yaw -353.8, roll +386.7, pitch +356.9 deg; roll right and pitch nose down read positive, yaw right reads negative ( opposite to the heading ); `PlutoPilot.cpp` back to HEAD, clean hex rebuilt, gate green, size identical to HEAD.

### 4. Architecture & pipeline docs ( PIPELINE_UPDATE.md, FC-Data wiki )

**Description.** Write `PIPELINE_UPDATE.md` with the new `Estimate_Get` bullet for
`fw-architecture-pipeline/subsystems/User_Space_API.md` ( every case with its unit ). Add an `Estimate_Get` section
to `docs/API/FC_DATA_API_WIKI.md` ( both overloads, units, the X / Y without UWB note, an example per hook, the sign of each axis as measured in task 3 ),
add that sign to the `Rate` line of the `FC-Data.h` doc, correct its gyro "not verified" note ( 16.4 counts per deg/s proven by the flown DCM path; `Sensor_Get ( Gyroscope )` is
deg/s x10 ), and add "Changes in 1.8.2". Check the wiki against the code with `spec-to-code-compliance`. No Mermaid
change: no flow moved.

- **Depends on:** 3
- **Skills / agent:** spec-to-code-compliance
- **Files:** PIPELINE_UPDATE.md ( this folder ), [FC_DATA_API_WIKI.md](../../../API/FC_DATA_API_WIKI.md)
- **Safety impact:** none
- **Done when:** PIPELINE_UPDATE.md and the wiki section exist; every stated unit matches a source line.
- **Rollback:** n/a
- **Status:** done 2026-11-05
- **Result:** PIPELINE_UPDATE.md written ( `User_Space_API.md` Estimate_Get bullet, gyro unit, API 1.8.2 ); wiki gains the `Estimate_Get` and Angle sections, sign table, three examples, "Changes in 1.8.2", gyro unit confirmed; `FC-Data.h` Rate and Angle docs carry the measured signs ( comments only ). Every unit checked against a source line ( CHANGES.md; inline, the compliance skill is not installed ). Review of the header comment change deferred to the task 5 topic review.

### 5. Topic review

**Description.** `pluto-reviewer` agent on the whole topic diff ( code and docs ), with the pluto-rules checklist. Fix
BLOCKING findings and re-run the gate ( task 2 ) if code changed.

- **Depends on:** 4
- **Skills / agent:** pluto-reviewer agent
- **Files:** topic diff
- **Safety impact:** none
- **Done when:** no open BLOCKING finding; re-gate passed if code changed.
- **Rollback:** n/a
- **Status:** done 2026-11-05
- **Result:** pluto-reviewer: 0 blocking, 1 should-fix, 3 nits, all fixed in docs ( no code change, no re-gate ): the wiki turn-count example now zeroes in `plutoInit` ( a Wi-Fi restart re-runs `onLoopStart` about every 5 s ); `gyro.scale` note narrowed ( still read by `pidLuxFloat` and the blackbox header ); CHANGES.md roll / pitch range cites the flown `bodyDcmToEuler` ( `imu.cpp:479-488` ); a U+2212 minus in `active-development/README.md:62` replaced. Code, units, signs, PIPELINE_UPDATE anchors and the gate verified clean.

### 6. Graph refresh & commit ( FW 3.15.4, API 1.8.2 )

**Description.** `graphify update .`, then the labeller with the graphify interpreter in isolated mode
( `GRAPHIFY_VIZ_NODE_LIMIT=6000 "/c/Users/MechAsh/AppData/Roaming/uv/tools/graphifyy/Scripts/python.exe" -I tools/graph_labels.py`;
the plain `python` fails, see baro-temperature-api task 6 ). Then `/pluto-commit`: all-target build, Makefile FW 3.15.4 /
API 1.8.2, promote PIPELINE_UPDATE.md, CHANGELOG ( API 1.8.2 is behaviour only: `Estimate_Get ( Rate, axis )` now
returns the measured rate in deg/s x10 ), topic Closed, staged with a drafted message. The user commits.

- **Depends on:** 5
- **Skills / agent:** graphify, pluto-commit
- **Files:** Makefile, CHANGELOG.md, graphify-out/
- **Safety impact:** none
- **Done when:** staged tree ready; topic Closed in both READMEs.
- **Rollback:** n/a
- **Status:** done 2026-11-05
- **Result:** graph 5112 nodes / 13989 edges / 243 communities, relabelled; all targets built clean ( PRIMUS_X2_v1 121.1 KB / 20.5 KB, PRIMUSX2 120.9 / 20.7, PRIMUS_V5 121.1 / 20.5 ), no new warnings; Makefile FW 3.15.4 / API 1.8.2; PIPELINE_UPDATE promoted into `User_Space_API.md`; CHANGELOG version line, Fixed and Documentation entries; staged, message in `.git/MAGISV2_COMMIT_MSG.txt`.

## Decisions log

Newest last. One line each: `YYYY-MM-DD [decision|assumption|out-of-scope|risk] text`.

- 2026-11-04 [decision] Mode `fix`: the user asked to "check this api and help me fix the issue"; the scout proved the cause from the code ( `FC-Data-Estimate.cpp:77-86` ), so no separate `check` topic.
- 2026-11-04 [assumption] Target `PRIMUS_X2_v1` ( `selected_target` in `plutoide.ini` ); the session ran unattended, so no interview round was answered.
- 2026-11-04 [decision] D1: Rate unit deg/s x10 ( `gyroADC / 1.64f` ), the same scale as `Sensor_Get ( Gyroscope )`; whole deg/s rejected ( loses resolution ).
- 2026-11-04 [decision] D2: raw `gyroADC` ( the value the rate loop uses ), no new filter; a filtered copy in `imu.cpp` rejected ( RAM, loop hook, delay ).
- 2026-11-04 [decision] D3: file-local float rounding helper with an int16 clamp, no `<math.h>` ( `M_PI` clash with `flight/pid.h` ).
- 2026-11-04 [decision] API 1.8.2 ( behaviour-only change ), FW 3.15.4.
- 2026-11-04 [decision] The gyro scale is 16.4 counts per deg/s ( flown DCM path, `imu.cpp:590`, `GYRO_CONFIG_1 = 0x1F` ); the wiki's "not verified" gyro note is corrected in task 4.
- 2026-11-04 [out-of-scope] The stale `gyro->scale = 1 / 131` and its comments in `accgyro_icm20948.cpp`: its own topic.
- 2026-11-04 [out-of-scope] Position / Velocity X and Y read 0 without UWB: documented only.
- 2026-11-04 [risk] The bench integral relies on a hand-held turn; a fixture or a reference mark per turn keeps it within the +/-30 deg window.
- 2026-11-04 [decision] Task 1 review: no blocking. The X / Y `@warning` in `FC-Data.h` narrowed to UWB ( the only X / Y source; Z always valid ). The sign of each Rate axis goes into the header and wiki in task 4, after task 3 measures it.
- 2026-11-04 [decision] Task 3 instrumentation review: no blocking. Analysis method added to TESTING.md: subtract the rest mean per axis, window by motion, integrate over real `t`, redo turns with a START or a `t` gap over 150 ms, unwrap `h` / `r`, expect about 354 for hand-held roll / pitch.
- 2026-11-05 [decision] Task 3 log-1 PASS. Every turn held 3-5 one-record gaps ( 26 START ticks, 40 records lost in transfer, all `n` +2, max 217 ms ), so they were bridged by the trapezoid instead of redoing turns; trapezoid vs sample-hold differs by under 6 deg.
- 2026-11-05 [decision] Measured signs: Rate X > 0 rolling right, Y > 0 pitching nose down ( both match Angle ), Z < 0 turning right ( opposite to `Angle AG_YAW` heading ). Kept raw per D2; task 4 documents it.
- 2026-11-05 [decision] Task 4: the Angle signs and ranges ( roll right / nose down positive, heading 0-359 clockwise ) also documented in the header and wiki, from the same log and `imu.cpp:1185-1197`; the yaw-rate sign is opposite to the heading because `heading` is built from `-anglerad [ AI_YAW ]` ( `imu.cpp:1191` ).
- 2026-11-05 [assumption] `spec-to-code-compliance` is not installed in this session; the doc check was done inline, claim by claim, in CHANGES.md.
- 2026-11-05 [decision] Task 5 review: no blocking; the should-fix ( wiki turn-count example reset by Wi-Fi restarts of `onLoopStart` ) and three nits fixed in docs only.
