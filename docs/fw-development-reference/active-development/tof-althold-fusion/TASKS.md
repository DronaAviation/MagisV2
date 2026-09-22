# ToF Altitude Hold Fusion - Tasks

[README](README.md) · [TASKS](TASKS.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

| | |
|---|---|
| **Goal** | Stable altitude hold on the VL53L0X laser, a jump-free laser/baro handover, and a 2.5 s hold-off before the setpoint follows a sudden object under the craft. |
| **Done when** | `logs/`: 1 m hover within ±5 cm of `AltHold` for 30 s; climb through 180 cm and back with no `EstAlt` step; hand placed/removed under the craft gives no reaction for ~2.5 s, then a ramped climb/descent. |
| **Target** | PRIMUS_X2_v1 ( `LASER_TOF` + `LASER_ALT` ) |
| **Kind** | behaviour change + investigation |
| **Pipeline docs affected** | subsystems/Altitude_Hold_Estimator.md ( sources, time constants, ToF vs Baro, mermaid ); Firmware_Pipeline.md only if the task cadence changes |
| **Order** | 1 → 2 → 12 → 3 → 4 → 13 → 5 → 6 → 7 → 8 → 14 → 9 → 10 → 11 |

## Index

| # | Title | Status | Skills |
|---|---|---|---|
| 1 | Baseline diagnostics: log fields, test plan, `log-1.txt` | done 2026-09-21 | flight-test, magisv2-rules, run-magisv2 |
| 2 | Analyse the baseline log | done 2026-09-21 | flightlog-analyst agent, flight-test |
| 12 | Estimator diagnostics + hand test flight ( `log-2.txt` ) | done 2026-09-21 | flight-test, magisv2-rules, run-magisv2 |
| 3 | Estimator velocity fix + hygiene: one tau, tilt gate, laser lag | done 2026-09-21 | magisv2-rules, dimensional-analysis, cpp-pro agent |
| 4 | Hover check flight ( `log-3.txt` ) | done 2026-09-21 | flight-test, flightlog-analyst agent |
| 13 | Post-take-off settle offset and bias-term wind-up | done 2026-09-22 | magisv2-rules, flight-test, dimensional-analysis |
| 5 | Laser/baro handover with hysteresis and frozen offset | done 2026-09-22 | magisv2-rules, dimensional-analysis |
| 6 | Step detector and 2.5 s hold-off, then re-base and goal ramp | done 2026-09-22 | magisv2-rules, dimensional-analysis, cpp-pro agent |
| 7 | Build gate | done 2026-09-22 | run-magisv2 |
| 8 | Hardware validation ( `log-5.txt` handover, `log-6.txt` object ) | done 2026-09-22 | flight-test, flightlog-analyst agent |
| 14 | Window-based step detection for slow surface changes ( + re-test `log-7.txt` ) | done 2026-09-22 | magisv2-rules, dimensional-analysis, flight-test |
| 9 | Architecture & pipeline docs ( PIPELINE_UPDATE.md ) | done 2026-09-22 | graphify, spec-to-code-compliance |
| 10 | Graph refresh | done 2026-09-22 | graphify |
| 11 | Review & commit | done 2026-09-22 | magisv2-reviewer agent, commit-magisv2 |

Status values: `todo`, `in-progress`, `done YYYY-MM-DD`, `blocked (<why>)`, `dropped (<why>)`.
Serial numbers are never reused or renumbered.

## Tasks

### 1. Baseline diagnostics: log fields, test plan, `log-1.txt`

**Description.** Add a diagnostic `Monitor_Print` line to `PlutoPilot.cpp` with the fields
degC, BaroAlt, ToF, PaI, Arm ( the set `tools/flightlog.py` already parses ) plus EstAlt,
AltHold and VelocityZ, staying under the ~250 byte per tick ceiling. Write `TESTING.md` with
the baseline procedure: indoor, flat matt floor, 1 m hover, no stick input for 30 s, then a
hand placed under the craft for ~5 s and removed. Build on the working target, create
`logs/log-1.txt` and open it in the IDE for the user to paste the capture.

- **Skills:** flight-test, magisv2-rules, run-magisv2
- **Files:** [PlutoPilot.cpp](../../../../PlutoPilot.cpp), [altitudehold.h](../../../../src/main/flight/altitudehold.h), `TESTING.md`
- **Done when:** firmware builds clean; `logs/log-1.txt` exists and is open; TESTING.md has the procedure.
- **Status:** done 2026-09-21
- **Result:** 8-field 10 Hz log in `PlutoPilot.cpp` ( ~125 B/tick ), procedure in TESTING.md test 1, `logs/log-1.txt` created and opened; builds at 103.5 KB / 15.2 KB; gate shows 8 inherited laser-code warnings, none from the log code; reviewer findings ( double promotion, boot-time `ToF:0` ) fixed.

### 2. Analyse the baseline log

**Description.** Run `tools/flightlog.py` and the `flightlog-analyst` agent on `log-1.txt`.
Quantify the bob ( amplitude, period, ToF→EstAlt→AltHold phase ), the laser noise and lag
against baro, source switching if any, and the hand step ( EstAlt slope, climb rate, drop
rate ). Confirm or replace the working hypothesis in `INVESTIGATION.md` and record the
numbers in `TESTING.md`. Adjust tasks 3, 5 and 6 if the evidence says so.

- **Skills:** flightlog-analyst agent, flight-test
- **Files:** `logs/log-1.txt`, `INVESTIGATION.md`, `TESTING.md`
- **Done when:** INVESTIGATION.md has a confirmed root cause with numbers from the log.
- **Status:** done 2026-09-21
- **Result:** Bob is a lightly damped altitude-loop limit cycle ( 30 cm p-p, 4.1 s ), not laser noise ( 1.2 cm ); `Vz` is 0.3-0.4× the real vertical speed and leads it by 0.3-0.5 s, the leading root cause, mechanism open. No hand test in the log → task 12.

### 3. Estimator velocity fix + hygiene: one tau, tilt gate, laser lag

**Description.** First fix the velocity estimate: tasks 2 and 12 found `VelocityZ` at 0.3-0.4× the
real vertical speed in the hover, caused by the 40-count Z deadband in `imuCalculateAcceleration()`
( `flight/imu.cpp:255`, default `accDeadband.z` in `config/config.cpp:535` ). Remove or shrink it
for the altitude estimator's `accSum[Z]`; the mini-grill decides the scope ( estimator-only vs the
profile default, which a saved profile keeps, and `accSumXYZ[Z]`, which other consumers use ).
Recheck the bias term `_accel_correction_hbf_z` afterwards. Also compute the laser error against the
filter's internal `_position_z` instead of the Kalman-filtered integer `EstAlt`. Then stop `correctedWithBaro()` overwriting
`_time_constant_z` so both sources use a single tau ( value from task 2, default the current
`LASER_ALT` 1.5 s ); fix the tilt gate in `checkReading()` so it compares degrees ( or the
cosine ) rather than radians against 25; and review the laser path lag ( `LASER_LPS 0.1` IIR
at 33 ms in `ranging_vl53l0x.cpp` and the tilt cosine ) against the bob measured in task 2.
Any constant change must carry its units.

- **Skills:** magisv2-rules, dimensional-analysis, cpp-pro agent
- **Files:** [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp), [ranging_vl53l0x.cpp](../../../../src/main/drivers/ranging_vl53l0x.cpp)
- **Done when:** a bench or log check shows `Vz` gain 0.8-1.2× d`EstAlt`/dt with under 100 ms lag; builds clean with no new `src/` warnings ( including the 8 laser-code warnings listed in CHANGES.md task 1 ); tau is source-independent; the tilt gate is unit-correct; CHANGES.md lists each change.
- **Status:** done 2026-09-21
- **Result:** Z deadband 0 for the altitude estimator under `LASER_ALT`, one tau 1.5 s, tilt gate in deci-degrees with driver-IIR reseed, laser error against `_position_z`, float IIR; gate passes with zero new warnings ( 103.5 KB / 15.2 KB ), PRIMUS_V5 builds. `Vz` gain check moved to task 4.

### 4. Hover check flight ( `log-3.txt` )

**Description.** Fly the task 3 firmware with the task 12 sequence ( hand-held phase, floor,
hover, hand test, land ); create and open `logs/log-3.txt`. The hand-held and hover phases measure
the `Vz` gain against the laser ( target 0.8-1.2×, under 100 ms lag, moved here from task 3 ). Measure the hover band against the ±5 cm criterion. If the bob remains,
return to task 3 with the new numbers before adding the handover and step logic.

- **Skills:** flight-test, flightlog-analyst agent
- **Files:** `logs/log-3.txt`, `TESTING.md`
- **Done when:** `Vz` gain 0.8-1.2× d`ToF`/dt in the hover; 30 s hover within ±5 cm of `AltHold` in `log-3.txt`, or a documented reason and a follow-up task.
- **Status:** done 2026-09-21
- **Result:** Limit cycle gone ( `EstAlt` sd 6.0 → 2.6 cm, p-p 30 → 9 cm, no 4 s cycle ); `Vz` gain 0.83 → 0.97 on hand-held motion. ±5 cm / 30 s missed by ~3 cm ( best 30 s: max 8 cm ) because of a slow settle from 5-8 cm below after take-off → task 13.

### 5. Laser/baro handover with hysteresis and frozen offset

**Description.** Replace the hard 200 cm switch in `checkReading()` with a source state:
laser → baro above an upper edge ( or out of range for several consecutive samples: `log-3-temp`
shows isolated single dropouts at 60-133 cm that must not trigger a handover ), baro → laser below a lower edge and
in range for a few consecutive samples. Freeze `baro_offset` at the moment of each handover
( it is then a constant, not the running LPF difference ) so `EstAlt` is continuous; the baro
ground datum itself is never re-zeroed in flight. Named constants in cm with units. **The
180/160 cm band must move down**: task 12 saw the VL53L0X drop out at ~172 cm on the test floor;
the mini-grill sets the new edges ( e.g. 150 / 130 ).

- **Skills:** magisv2-rules, dimensional-analysis
- **Files:** [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp), [altitudehold.h](../../../../src/main/flight/altitudehold.h)
- **Done when:** builds clean; bench ( `logs/log-4.txt`, hand-held, disarmed ): `Src` switches 1 → 0 near 160 cm and 0 → 1 near 140 cm, and single laser dropouts do not switch it. `EstAlt` continuity and the setpoint shift cannot be judged disarmed ( the baro zero tracks while disarmed and `AltHold` follows `EstAlt` ); they move to the task 8 flight.
- **Status:** done 2026-09-22
- **Result:** Handover 160 / 140 cm with 120 ms dropout debounce, 2 s baro-offset average, lag-compensated frame shift on return; bench `log-4`: all 6 switches at the edges, single dropouts and hand passes ignored. Builds clean ( 104.0 KB / 15.2 KB ). Flight continuity check in task 8.

### 6. Step detector and 2.5 s hold-off, then re-base and goal ramp

**Description.** Detect a laser step > 30 cm within 0.5 s in either direction, **on the raw
range** ( `RangingMeasurementData.RangeMilliMeter` ) before the driver's `LASER_LPS` IIR, which
spreads a hand step over ~0.6 s ( task 12 ), and start the hold-off within ~100 ms because
`EstAlt` follows a hand ~20 cm within 0.8 s. While a step
is pending ( 2.5 s ), keep the estimator on the baro path with the offset frozen at the
pre-step value, so `EstAlt` and the position loop do not react. If after 2.5 s the laser is
still consistent ( within ±10 cm ) at the new reading, re-base: set the laser offset so
`EstAlt` becomes the new clearance and write `AltHold` as a goal so the setpoint ramps on the
existing `ALT_CMD_*` profile ( from althold-setpoint-shaping ), climbing over an object or
descending after its removal. If the reading returns to the old value, cancel silently.

- **Skills:** magisv2-rules, dimensional-analysis, cpp-pro agent
- **Files:** [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp), [altitudehold.h](../../../../src/main/flight/altitudehold.h)
- **Done when:** builds clean with no new warnings and the review's blocking findings fixed. The detector is armed-and-airborne only ( pilot's choice ), so it cannot be shown on a bench; the behaviour ( `Src` 2 for ≥ 2.5 s, no climb during it, then a ramped return to the old clearance ) is checked in the task 8 object flight ( `log-6.txt` ).
- **Status:** done 2026-09-22
- **Result:** Raw-laser step detector ( > 30 cm from the lag-advanced estimate, armed, airborne, not landing ), baro hold-off ≥ 2.5 s until steady, frame re-base and a goal back to the old clearance; review fixes applied ( lag, landing, active goal ). Clean gate 104.5 KB / 15.2 KB. Flight check in task 8 ( `log-6` ).

### 7. Build gate

**Description.** `run-magisv2 --gate PRIMUS_X2_v1`: no new warnings under `src/`, flash/RAM fits ( last known 99.7 KB / 14.8 KB with the laser off ).

- **Skills:** run-magisv2
- **Done when:** gate passes; numbers recorded in CHANGES.md.
- **Status:** done 2026-09-22
- **Result:** Clean gated builds with full compiler logs: PRIMUS_X2_v1 104.5 KB / 15.2 KB, PRIMUS_V5 102.5 KB / 14.8 KB; no new warnings in `src/`, none in `PlutoPilot.cpp`.

### 8. Hardware validation ( `log-5.txt` handover, `log-6.txt` object )

**Description.** Flight tests from `TESTING.md`: ( a ) 1 m hover 30 s within ±5 cm;
( b ) climb through 180 cm and descend back, no `EstAlt` step ( `log-5.txt` ); ( c ) hand
placed under the craft for 5 s and removed: no reaction for ~2.5 s, then ramped climb and
ramped descent ( `log-6.txt` ). Each log file created and opened before the flight; results
with numbers in `TESTING.md`.

- **Skills:** flight-test, flightlog-analyst agent
- **Done when:** all three criteria met in the logs.
- **Status:** done 2026-09-22 ( object part partial, see task 14 )
- **Result:** Handover passes in flight ( `log-5` ): continuous on the way up, frame shifted together on the way down, quick re-arm OK. Object test ( `log-6` ): 10 re-bases and 7 cancels behave as specified for brisk box moves, but a slow slide-out was missed and the craft dropped by the box height ( 36 cm under the setpoint ), and edge contamination dipped `EstAlt` ~25 cm in the quick passes. Follow-up: task 14.

### 9. Architecture & pipeline docs ( PIPELINE_UPDATE.md )

**Description.** Write the new `Altitude_Hold_Estimator.md` sections into `PIPELINE_UPDATE.md`: sources and time constant, handover state with hysteresis, step hold-off, updated mermaid ( every edge confirmed with `graphify path` or a source line ). Fix the tau drift already noted ( doc says 2 s ). Update `CLAUDE.md` with the handover/step invariants and `docs/API/` only if a public header changed.

- **Skills:** graphify, spec-to-code-compliance
- **Done when:** PIPELINE_UPDATE.md complete and checked against the code.
- **Status:** done 2026-09-22
- **Result:** `PIPELINE_UPDATE.md` stages the full new `Altitude_Hold_Estimator.md` ( new Laser fusion section and flowchart, edited functions, sources and estimator flowchart ), the `CLAUDE.md` paragraphs and the skill-file edits. Checked against the code twice by the reviewer ( 9 findings, then 2, all fixed ); all 4 diagrams render; only this pipeline doc changes.

### 10. Graph refresh

**Description.** `graphify update .` then `python tools/graph_labels.py`.

- **Skills:** graphify
- **Done when:** graph rebuilt at the topic's HEAD.
- **Status:** done 2026-09-22
- **Result:** `graphify update .` + `graph_labels.py`: 3982 nodes, 7494 edges, 396 communities; `altShiftFrame`, `tofWindowMismatch`, `altHoldSource`, `tofRequestReseed` present, `checkReading() → altShiftFrame()` extracted. Re-run once more in task 11 after the diagnostics come out of `PlutoPilot.cpp`.

### 11. Review & commit

**Description.** `magisv2-reviewer` agent on the whole topic diff; fix BLOCKING findings; remove the temporary diagnostics from `PlutoPilot.cpp` unless kept on purpose; then `commit-magisv2` ( all-target build, version bump, promotion of PIPELINE_UPDATE.md and CHANGELOG ). The user commits.

- **Skills:** magisv2-reviewer agent, commit-magisv2
- **Done when:** staged with a drafted message.
- **Status:** done 2026-09-22 ( staged; the user commits )
- **Result:** Diagnostics removed, target.h restored ( laser off by the pilot's choice ), whole-topic review, all targets built ( X2_v1 98.9 KB / 14.8 KB, PRIMUSX2 99.4 / 15.0, V5 98.8 / 14.8 ), FW 3.9.0, docs promoted, graph refreshed, staged with a drafted message.

### 12. Estimator diagnostics + hand test flight ( `log-2.txt` )

**Description.** Added by task 2. Extend the temporary `PlutoPilot.cpp` log with the estimator
internals needed to find why `VelocityZ` is 0.3-0.4× the real speed: vertical acceleration
after `accVelScale` ( cm/s² ), the bias term `_accel_correction_hbf_z`, and `_velocity_z`
before the Kalman smoother, keeping the line under ~250 bytes ( drop `PaI` if needed ). Fly
the task 1 hover with the **hand test** the baseline missed ( hand 30-40 cm under a 1 m hover
for ~5 s, twice ), and at least 20 s disarmed before arming. Analyse which term shrinks the
velocity and record it in INVESTIGATION.md.

- **Skills:** flight-test, magisv2-rules, run-magisv2
- **Files:** [PlutoPilot.cpp](../../../../PlutoPilot.cpp), [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp), `logs/log-2.txt`
- **Done when:** `log-2.txt` has the new fields and a hand test; INVESTIGATION.md names the term that attenuates `Vz` and gives the hand-step numbers.
- **Status:** done 2026-09-21
- **Result:** Root cause is the 40-count accelerometer Z deadband ( ~9.6 cm/s² ): `Vz` gain 0.78 on large hand-held motion vs 0.36 in the hover, matching a deadband model ( 0.80 / 0.30 ); Kalman ruled out. Hand test overshoots +39 cm; the driver IIR spreads a hand step over ~0.6 s; laser drops out at ~172 cm. Log line trimmed to ~115 B after app disconnects.

### 13. Post-take-off settle offset and bias-term wind-up

**Description.** Added by task 4. After take-off the craft sits 5-8 cm below `AltHold` and creeps
up over ~15 s before holding within ±3 cm ( `log-3` 48-64 s ); this alone fails the ±5 cm / 30 s
criterion. Find whether it is the velocity integrator ( `errorVelocityI`, hover trim ), the
take-off goal hand-off from althold-setpoint-shaping, or the estimator bias term
`_accel_correction_hbf_z`, which climbs 5 → 8.5 cm/s² through the hover and winds up to 18 cm/s²
at touchdown ( `Vz` then reads ~20 cm/s on the floor ). Fix the cause, or record it as accepted if
the pilot is satisfied with ±8 cm.

- **Skills:** magisv2-rules, flight-test, dimensional-analysis
- **Files:** [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp)
- **Done when:** a hover log reaches ±5 cm within ~5 s of take-off and holds it for 30 s, or the offset is accepted in the decisions log; `Vz` on the floor after landing under ~5 cm/s.
- **Status:** done 2026-09-22 ( accepted, no code change )
- **Result:** Pilot accepts the hold. `log-3-temp.txt` meets ±5 cm for 30 s ( `EstAlt` −3..+3 cm, laser 99 % within ±5 ). The slow settle in `log-3` is the bias term learning the in-flight accelerometer offset ( −7.3 cm/s² ) from 0 after a fresh power-up. Touchdown wind-up left as a watched risk.

### 14. Window-based step detection for slow surface changes ( + re-test `log-7.txt` )

**Description.** Added by task 8. A box slid out slowly passes the VL53L0X cone edge first, so the
laser changes over ~0.5-0.8 s; the estimate follows each sample and the per-sample 30 cm residual
never fires ( `log-6` ~60 s: the craft dropped by the box height ). Detect as the pilot originally
specified, "more than 30 cm within 0.5 s": compare the raw laser's change over the last 0.5 s with
the inertial displacement over the same window ( integrated `VelocityZ` or the accelerometer ), and
start the hold-off when they disagree by more than 30 cm. While they disagree by more than a smaller
margin, pause the baro-offset average and the laser correction so edge transitions do not
contaminate them. Re-fly TESTING.md test 6 with slow slides as well as brisk moves.

- **Skills:** magisv2-rules, dimensional-analysis, flight-test; `magisv2-reviewer` agent
- **Files:** [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp)
- **Done when:** builds clean; `log-7.txt` shows a hold-off ( `Src` 2 ) for every box in / out including slow slides, no height change beyond ~10 cm during any hold-off, and no false trigger in a plain hover or during take-off and landing.
- **Status:** done 2026-09-22
- **Result:** Window test with suspect coasting, take-off grace and a 20 cm edge/slope line; two reviews applied. `log-7`: every box move including slow slides started a hold-off ( 6-13 cm, once 16 cm ), no miss like `log-6`, no false trigger at take-off or landing. 105.3 KB / 15.5 KB.

## Decisions log

Newest last. One line each: `YYYY-MM-DD [decision|assumption|out-of-scope|risk] text`.

- 2026-09-21 [decision] Behaviour change with a baseline investigation first; target PRIMUS_X2_v1 with LASER_TOF + LASER_ALT ( VL53L0X ).
- 2026-09-21 [decision] Hover done-when: 1 m indoor hover within ±5 cm of AltHold for 30 s.
- 2026-09-21 [decision] Sudden object: hold off 2.5 s, then climb to keep the laser clearance; symmetric 2.5 s hold-off then ramped descent on removal.
- 2026-09-21 [decision] Step threshold > 30 cm within 0.5 s; consistency window ±10 cm.
- 2026-09-21 [decision] Handover: baro above 180 cm, laser below 160 cm, baro offset frozen at each handover.
- 2026-09-21 [assumption] User asked 180/180; a zero-width band would chatter, so 160 cm is the lower edge until corrected.
- 2026-09-21 [assumption] Floor at the test site is matt and light enough for the VL53L0X at 1 m; verify from log-1.
- 2026-09-21 [decision] One estimator time constant for both sources; the always-true radian tilt gate is fixed in task 3.
- 2026-09-21 [decision] Log fields: degC, BaroAlt, ToF, PaI, Arm + EstAlt, AltHold, VelocityZ.
- 2026-09-21 [decision] Whenever a log is expected, create `logs/log-N.txt` in the topic folder and open it in the IDE.
- 2026-09-21 [out-of-scope] Laser landing/touchdown detection; VL53L1X ( same band to be applied later ); PRIMUS_V5.
- 2026-09-21 [risk] VL53L0X getRange() blocks the periodic slot on I2C1 shared with the ICP-10111; watch loop time if the poll rate changes.
- 2026-09-21 [risk] Enabling LASER_TOF/LASER_ALT surfaces 8 warnings in ranging_vl53l0x.cpp and altitudehold.cpp that the laser-off baseline never saw; task 3 cleans them, the baseline is not regenerated.
- 2026-09-21 [decision] `ToF` is logged from the driver ( `NewSensorRange / 10` ), not from `ToF_Height`, which goes stale while the estimator runs on baro.
- 2026-09-21 [decision] Task 2: the bob is an altitude-loop limit cycle ( 30 cm p-p, 4.1 s ), not laser noise; `Vz` at 0.3-0.4× the real speed is the leading root cause, so task 3 now starts with the velocity estimate.
- 2026-09-21 [decision] Task 12 added before task 3: estimator-internal log fields plus the hand test the baseline missed. Log files renumbered by text only: hover check `log-3`, handover `log-4`, object `log-5`.
- 2026-09-21 [risk] Baro noise is 13 cm sd sample to sample and ~8 cm below the laser; the task 5 handover must not pass it straight into `EstAlt`.
- 2026-09-21 [decision] Task 12 test adds a disarmed hand-held phase ( 30 cm up/down at ~4 s ) so the velocity estimate is measured with the altitude loop off; user's sequence ( hand-held 20 s, floor 20 s, 1 min hover, land ) kept, plus the hand test.
- 2026-09-21 [risk] Review of task 12: the 40-count accelerometer Z deadband ( ~9.6 cm/s² ) sits before the estimator and is a strong candidate for the `Vz` loss; test 2 adds a fast hand-held set ( ~2 s cycles ) to tell a deadband from a gain error.
- 2026-09-21 [risk] App disconnects at ~180-223 bytes per tick of `Monitor_Print`: the log burst plus the app's MSP replies overrun the 256-byte TX ring ( `uartWrite ( )` has no room check ). The practical ceiling with the app connected is lower than the ~250 bytes in CLAUDE.md / magisv2-rules; `log-1` ran at ~125-135. Task 9 updates that gotcha.
- 2026-09-21 [decision] Task 12 log trimmed to ~115 bytes: `degC`, `PaI`, `BaroAlt` commented out ( the baro is not part of the velocity question ).
- 2026-09-21 [out-of-scope] `debugPrint ( msg, double, digits )` in `API-Src/Debugging.cpp` subtracts the ASCII code instead of the digit from `remainder`, so every digit after the first decimal prints 0 ( all 1281 two-decimal values in `log-1.txt` end in 0 ). Also no TX-room check in `debugPrint`. Public API bugs for a separate topic.
- 2026-09-21 [decision] Task 12: root cause of the small `Vz` is the 40-count accelerometer Z deadband ( ~9.6 cm/s² ): gain 0.78 on large hand-held motion vs 0.36 in the hover, matching a deadband model; the Kalman smoother is ruled out. Task 3 removes/shrinks it for the altitude estimator.
- 2026-09-21 [risk] `accDeadband.z` is a stored profile setting and also feeds `accSumXYZ[Z]`; a default change does not reach boards with a saved profile. Task 3 mini-grill decides the scope.
- 2026-09-21 [assumption] The 180/160 cm handover band is superseded: the VL53L0X dropped out at ~172 cm on the test floor. New edges set in task 5's mini-grill.
- 2026-09-21 [decision] Task 6 detects steps on the raw laser range, not on the IIR output; the IIR shows a ~34 cm hand as ~21 cm within 0.5 s.
- 2026-09-21 [risk] PlutoMonitor over Wi-Fi can drop and bunch records ( 1.1 s gap in `log-2` ); rates computed across such gaps are artefacts.
- 2026-09-21 [decision] Task 3 mini-grill: the Z deadband change applies to `LASER_ALT` builds only, as a compile-time constant `ALT_EST_ACC_Z_DEADBAND` = 0 counts ( not the profile default, which a saved profile keeps, and no EEPROM version bump, which would wipe calibrations ).
- 2026-09-21 [decision] `LASER_LPS` stays 0.1 in task 3 so the task 4 flight measures the deadband fix alone; laser lag revisited only if the hover still fails.
- 2026-09-21 [decision] Laser samples above 25° tilt are rejected and the baro path is used ( e.g. during a flip ); the old radian gate never rejected.
- 2026-09-21 [decision] Task 3's `Vz` gain check ( 0.8-1.2× ) needs the drone; it moves into the task 4 flight, whose hand-held phase measures it.
- 2026-09-21 [decision] Review of task 3: the driver IIR reseeds after out-of-range samples and after a rejected tilted sample ( `tofRequestReseed ( )` ), so a gap or flip does not leave stale history to step `EstAlt`.
- 2026-09-21 [decision] Task 4: the deadband fix removes the limit cycle ( sd 6.0 → 2.6 cm ); the ±5 cm miss is a slow post-take-off settle, not oscillation. Task 13 added before the handover work; the pilot may accept ±8 cm instead.
- 2026-09-21 [risk] Velocity-gain fits at 10 Hz are biased towards 0 when the craft barely moves ( laser-rate noise ); use smoothed signals and report both regression directions.
- 2026-09-21 [risk] `_accel_correction_hbf_z` winds up to ~18 cm/s² at touchdown and `Vz` reads ~20 cm/s on the floor for ~10 s; a quick re-arm could start with a wrong velocity ( task 13 ).
- 2026-09-22 [decision] Task 13 closed as accepted: the pilot is satisfied with the hold, and `log-3-temp.txt` meets ±5 cm for 30 s.
- 2026-09-22 [decision] Mechanism of the slow post-take-off settle: the accelerometer reads ~−7.3 cm/s² in flight, and the bias term `_accel_correction_hbf_z` has to learn it; from a fresh power-up it starts at ~0 and needs ~15 s. Not changed.
- 2026-09-22 [risk] Touchdown winds the bias term up to ~16-18 cm/s² and `Vz` reads ~15-20 cm/s on the floor for ~10 s; watch a quick re-arm in task 8. Not changed.
- 2026-09-22 [decision] Task 5 handover must debounce laser dropouts: `log-3-temp` has single `ToF` −1 samples at 60-133 cm.
- 2026-09-22 [decision] Task 5 mini-grill: band 160 cm up / 140 cm down ( ~12 cm margin to the ~172 cm dropout seen on the test floor ); hand over after 100 ms without a usable sample ( 3 samples; a timeout, so a silent sensor also hands over ), shorter gaps coast on the accelerometer; on the return to the laser the whole altitude frame ( estimate and setpoint ) shifts by the baro drift so the aircraft does not move.
- 2026-09-22 [decision] The baro offset is a 2 s average of baro minus laser while on the laser, frozen on the baro and continued ( not reseeded ) after a return, so a handover never uses one noisy baro sample.
- 2026-09-22 [decision] Log for task 5+: `Src` ( 1 laser, 0 baro ) and `BaroAlt` added, `VzR` and `AccZ` commented out ( ~120 B/tick ). Bench log is `log-4.txt`; task 8 flights move to `log-5` / `log-6`.
- 2026-09-22 [decision] Review of task 5: after a dropout or tilt handover the laser takes back below 160 cm ( not 140 ), timeout 120 ms, laser reading advanced by `VelocityZ × 0.3 s` ( driver IIR lag ) for the offset and the return shift, `altPreFlipAltHold` shifted with the frame.
- 2026-09-22 [decision] A disarmed bench cannot show `EstAlt` continuity at the handover ( `baroUpdateZero()` pulls the baro to 0 while disarmed ); the bench checks switch points and debounce only, continuity is checked in the task 8 flight.
- 2026-09-22 [decision] Task 6 mini-grill: after the 2.5 s hold-off, re-base only once the raw laser has been steady within ±10 cm for 0.5 s; a moving hand keeps the estimate on the baro. The detector runs only when armed and airborne.
- 2026-09-22 [decision] Task 6 design: step = raw ( pre-IIR, tilt-corrected ) laser more than 30 cm from `_position_z`; hold-off on the baro with the frozen offset ( offset average paused ); cancel after 3 samples within 15 cm; re-base = frame shift by `raw − _position_z`, then `AltHold` set back to its old value so setpoint shaping flies it as a goal ( 60 cm/s up, 30 cm/s down ).
- 2026-09-22 [decision] Log `Src` is three-valued: 1 laser, 0 baro, 2 object hold-off ( `altHoldSource()` replaces `altHoldOnLaser()` ).
- 2026-09-22 [decision] Review of task 6: the step residual uses the estimate advanced by `VelocityZ × 0.3 s`; the detector is off while landing; a re-base keeps an active goal's end point.
- 2026-09-22 [risk] A step whose raw reading lands between 160 cm and the ~172 cm sensor limit is not detected ( the handover owns it ), and a moving hand keeps the craft on the baro alone ( 13 cm sd ) with no timeout; both accepted.
- 2026-09-22 [decision] Task 8 mini-grill: ceiling ≥ 2.5 m, so the handover flight climbs to ~190-200 cm; the object is a box or books 25-35 cm tall, held still, slid under an ~80 cm hover.
- 2026-09-22 [risk] Known limit: after climbing over an object the craft holds its old clearance above it; if removing the object makes the laser read ≥ 160 cm, the handover takes it ( `Src` 0 ) and the craft stays up on the baro instead of stepping down. The object test keeps hover + object below ~150 cm.
- 2026-09-22 [decision] Task 8: the handover passes in flight; the object detector works for brisk moves but misses a slow slide ( the craft dropped by the box height ). Task 14 adds window-based detection ( laser change against inertial displacement over 0.5 s, the pilot's original criterion ) before the pipeline docs.
- 2026-09-22 [risk] Gradual edge transitions contaminate both the estimate and the baro-offset average; a later hold-off then flies the baro with a wrong offset ( `log-6` 127-131 s dipped ~25 cm ).
- 2026-09-22 [decision] Task 14 go-ahead from the pilot after the trade-off explanation. Window test against `_position_base_z` ( the accelerometer-integrated position the laser's position correction does not move ), 0.5 s window, threshold kept at the pilot's 30 cm; offset average paused above 15 cm.
- 2026-09-22 [risk] A box slid over more than ~1 s changes less than 30 cm per 0.5 s and is still followed at once; the threshold is not lowered because take-off already reaches ~22 cm on the offline proxy.
- 2026-09-22 [decision] Review of task 14: suspect state with a frozen window reference and coasting ( laser correction paused ) until the mismatch passes 30 cm ( hold-off ), falls under 7.5 cm, or 1 s passes ( slope, follow ); offset average paused until the window refills, dt clamped, and moved with every frame shift. Simulated: 36-55 cm edges over up to 1 s ( 1.5 s for 45 cm+ ) caught with the estimate moving under 4 cm; slopes over 2 s and more are followed.
- 2026-09-22 [risk] Coasting on the accelerometer for up to 1 s during a suspected edge; bounded by `ALT_TOF_SUSPECT_MAX_MS`.
- 2026-09-22 [decision] Second review of task 14: 1 s take-off grace for the window test; at the 1 s suspect timeout a mismatch above 20 cm is an edge ( hold-off, or the baro if the laser reads ≥ 160 cm ), 20 cm or less a slope ( followed ). Simulated: 30-55 cm edges over up to 1 s are caught with the estimate moving under 4.5 cm.
- 2026-09-22 [decision] Task 14 closed on `log-7`: every box move, slow slides included, started a hold-off; no box-height drop without one.
- 2026-09-22 [risk] Accepted by the pilot, no climb cap: an object held up under the craft re-bases it repeatedly ( `log-7` 148-156 s climbed ~137 cm to ~2 m ). Pilot's choice from the first planning round, confirmed after this flight.
- 2026-09-22 [risk] A slow slide can re-base on a half-way reading when it stays steady for 0.5 s; later triggers correct it ( `log-7` 95.7 s ).
- 2026-09-22 [decision] Task 9: only `Altitude_Hold_Estimator.md` changes among the pipeline docs; the `Monitor_Print` guidance becomes ~130 B/tick with the app connected ( 115-135 B ran clean, ~180 B disconnected; the exact limit is not measured ), staged for `CLAUDE.md` and five skill files.
- 2026-09-22 [risk] A laser that goes silent ( no new samples, not out of range ) keeps correcting towards its last reading until the 120 ms timeout; documented, not changed.
- 2026-09-22 [decision] VL53L1X check ( pilot's question ): `LASER_TOF_L1x` + `LASER_ALT` compiles and links on PRIMUS_X2_v1 ( 113.2 KB / 15.8 KB, target.h switched temporarily and restored ). It gets the deadband fix, the single tau and the laser error against `_position_z`, but keeps the old hard 350 cm switch, the radian tilt gate, no hysteresis, no dropout debounce, no object hold-off; untested in flight. Its gate shows 7 pre-existing warnings in `ranging_vl53l1x.cpp` and 1 conversion in the old L1x branch. The `tofTiltOk` unused warning it exposed was fixed ( guarded by `LASER_TOF` ).
- 2026-09-22 [out-of-scope] Porting the handover, dropout debounce and object hold-off to the VL53L1X branch: a follow-up topic.
- 2026-09-22 [out-of-scope] VL53L1X parity deferred by the pilot ( no board at hand ): planned topic `vl53l1x-althold-parity` records the gap table, the out-of-range bug ( stale reading keeps correcting past the sensor's reach ) and the approach.
- 2026-09-22 [decision] Commit: PRIMUS_X2_v1 ships with `LASER_TOF` / `LASER_ALT` off ( target.h restored ); FW 3.8.1 → 3.9.0 ( minor, new behaviour ), API unchanged 1.3.2.
- 2026-09-22 [decision] Whole-topic review: `LASER_LPS` 0.1f, `altHoldSource()` gated, re-base keeps the flip return as a clearance, landing during a hold-off re-bases at once, tilt comment moved. Accepted: the return shift uses the lag-advanced laser while `correctedWithTof()` uses the IIR reading, a ~9 cm transient returning in a 30 cm/s descent.
