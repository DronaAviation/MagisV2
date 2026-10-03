# Battery SoC Fix - Tasks

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md)

## Resume

Read this block first in a new session; read only the task it points to.

| | |
|---|---|
| **Current task** | **Closed - pending commit** ( 3 Oct 2026 ). All tasks done. |
| **Next step** | The user commits with `.git/MAGISV2_COMMIT_MSG.txt`; the next pluto-commit / pluto-grill run fills in the hash. Follow-ups outside this topic: the app changes in APP_INTEGRATION.md section 0; optional unmask of level 2 in `MSP_ANALOG` while armed; `MSP_VOLTAGE_METER_CONFIG` field order; review notes 4 and 6 ( CHANGES.md "Task 11" ). log-8 charger mAh was not recorded. |
| **Open questions** | none for this topic |
| **Blocked on** | the user's commit |
| **Last updated** | 2026-10-03 |

## Summary

| | |
|---|---|
| **Mode** | fix |
| **Goal** | A remaining-mAh figure and a low-battery warning that can be trusted on 600 and 800 mAh packs. |
| **Done when** | Bench supply: no SoC jump at ≤ 3.0 V, sensible `E` from the curve, warnings at the planned voltages. Flights ( an older 600, a newer 600, an 800 ), full to warning, charger after each: `D` within ±5% of the charger; app remaining at empty ≤ 5% of capacity; warning with ≥ 15% really left. |
| **Target** | PRIMUS_X2_v1 ( one R020 ); all targets built at commit |
| **Analysis** | Causal chain in INVESTIGATION.md §3: wrap ( `battery.cpp:357`, `:209`, `:433` ), late warning ( straight-line voltage SoC in the fusion, `:454-551`; `BMS_Update` every loop, `mw.cpp:442` ), auto-gain −5% ( `:261-286` ), start estimate ( `:184-212` ), counter losses ( `:341-345`, `ina219.c:41-64`, `maths.cpp:381` ). |
| **Pipeline docs affected** | `subsystems/Power_BMS_Pipeline.md` ( rewrite; carries the check topic's drift list ); `docs/API` BMS wiki if `Bms_Get` changes |
| **Order** | 1 → 13 → 14 → 15 → 2 → 3 → 4 → 5 → 6 → 7 → 8 → 16 → 9 → 10 → 11 → 12 |

## Index

| # | Title | Status | Depends on | Skills / agent |
|---|---|---|---|---|
| 1 | Thresholds and LiPo curve from the logs | done 2026-09-28 | - | pluto-flighttest, pluto-log-analyst agent |
| 2 | INA219 driver: raw units and error handling | done 2026-09-28 | - | pluto-rules, pluto-driver, c-pro agent |
| 3 | Counter: signed averaging, no auto-gain, exact dt, saturating remaining | done 2026-09-28 | 2, 15 | pluto-rules, cpp-pro agent |
| 4 | Plug-in estimate: averaged sample, LiPo curve, cell count | done 2026-09-28 | 1, 3 | pluto-rules, cpp-pro agent |
| 5 | SoC model and warnings: count-led with a voltage floor, `BMS_Update` rate | done 2026-09-29 | 1, 4, 14 | pluto-rules, cpp-pro agent |
| 6 | `Bms_Get` and `MSP_ANALOG` values against the new model | done 2026-09-29 | 5 | pluto-rules |
| 7 | Build gate | done 2026-09-29 | 2-6 | pluto-build |
| 8 | Bench supply sweep | done 2026-09-30 | 7 | pluto-flighttest ( user runs ) |
| 9 | Validation flights on three packs | done 2026-10-01 | 8, 16 | pluto-flighttest ( user flies ), pluto-log-analyst agent |
| 10 | Architecture & pipeline docs ( PIPELINE_UPDATE.md ) | done 2026-10-01 | 9 | spec-to-code-compliance, graphify |
| 11 | Topic review | done 2026-10-01 | 10 | pluto-reviewer agent |
| 12 | Remove test code, graph refresh & commit | done 2026-10-03 | 11 | pluto-build, graphify, pluto-commit |
| 13 | Flight: full to empty on a newer 600, 60 s rest after landing | done 2026-09-28 | 1 | pluto-flighttest ( user flies ), pluto-log-analyst agent |
| 14 | Pack survey: 2-3 more packs full to empty, fixed vs per-flight R | done 2026-09-28 | 13 | pluto-flighttest ( user flies ), pluto-log-analyst agent |
| 15 | Current calibration: INA219 against a reference meter | done 2026-09-28 | - | pluto-flighttest ( user measures ) |
| 16 | Auto-land on critical battery instead of the app's in-air disarm | done 2026-10-01 | 7 | pluto-rules, cpp-pro agent, pluto-flighttest ( user flies ) |

Status values: `todo`, `in-progress`, `done YYYY-MM-DD`, `blocked (<why>)`, `dropped (<why>)`.
Serial numbers are never reused or renumbered.

## Tasks

### 1. Thresholds and LiPo curve from the logs

**Description.** The count-led model needs three sets of numbers, derived from the check topic's data:

- **The pack resistance.** From log-6: the voltage step at arming and landing against the current step, and the
  recovery after landing; from log-1 with its current doubled.
- **The current-compensated voltage at 15% and at 5% really left.** From the log-6 table ( charge used = `D` /
  0.95 against the 534 mAh charger figure ). These set the voltage floor for warning and critical.
- **A LiPo resting-voltage curve for plug-in** ( voltage → fraction left, 4.20 → 3.30 V ). A standard 1S LiPo
  table, checked against the three points the logs give: 4.0 V resting was ~77% in log-1, and the full / empty
  ends of log-6.

Write the numbers and how they were derived into INVESTIGATION.md §5. No code.

- **Depends on:** -
- **Skills / agent:** pluto-flighttest; pluto-log-analyst agent for the log-6 extraction if needed
- **Files:** [battery-capacity-estimate/TESTING.md](../battery-capacity-estimate/TESTING.md), [logs](../battery-capacity-estimate/logs/)
- **Safety impact:** none ( numbers only ); they set the warning timing in task 5
- **Done when:** INVESTIGATION.md §5 has the pack resistance, the two floor voltages with the log rows they come from, and a curve table of at least 10 points with its source.
- **Status:** done 2026-09-28
- **Result:** INVESTIGATION.md §5: R = 100 mOhm ( log-6 steady state 87-106; log-1 steps 125-140 ); floor `Vcomp` = bus + I x R at 15% = 3.715 V, at 5% = 3.57 V ( recommended 3.72 / 3.60 ); standard 21-point 1S curve fits log-6 within 4 points. The 15% floor is very sensitive to pack R ( fires at 6-60% for 80-140 mOhm ): open question for task 5.

### 2. INA219 driver: raw units and error handling

**Description.** In [ina219.c](../../../../src/main/drivers/ina219.c) / `.h`, the driver hands on quantities that
are already rounded:

- `shunt_voltage ( )` returns whole mV ( rounded toward zero );
- `bus_voltage ( )` returns 0.1 V steps floored, with a comment saying mV;
- both return `0xFFFF` on an I2C error, which then enters the averages.

Change the driver so the caller gets:

- the shunt in its native 10 uV LSB ( signed );
- the bus in mV ( 4 mV LSB );
- an explicit valid / invalid result, so a failed read is skipped rather than averaged.

Keep the register set-up unchanged ( PGA /4, 12-bit ). Also fix the Makefile's `drivers/ina219.cpp` entry if the
file is really `ina219.c` and the build relies on it: check how it is compiled today. Plain C: delegate to
`c-pro`.

- **Depends on:** -
- **Skills / agent:** pluto-rules, pluto-driver ( I2C1 shared bus: read cost unchanged ), c-pro agent
- **Files:** [ina219.c](../../../../src/main/drivers/ina219.c), [ina219.h](../../../../src/main/drivers/ina219.h), [Makefile](../../../../Makefile)
- **Safety impact:** none directly; every battery figure depends on it
- **Done when:** builds clean on PRIMUS_X2_v1 with no new `src/` warnings; the new functions' units are in their header comments; callers adapted in task 3 ( this task may leave a thin compatibility shim if task 3 is separate ).
- **Rollback:** revert the two files.
- **Status:** done 2026-09-28
- **Result:** `INA219_ReadBus_mV` ( mV ) and `INA219_ReadShunt_10uV` ( signed 10 uV ) with a valid flag; `bus_voltage` / `shunt_voltage` kept as exact-output shims for task 3; Makefile entry fixed to `ina219.c`. Gate clean, flash 99.9 KB, RAM 14.8 KB. Reviewer: 0 blocking; pipeline doc path staged in PIPELINE_UPDATE.md.

### 3. Counter: signed averaging, no auto-gain, exact dt, saturating remaining

**Description.** In [battery.cpp](../../../../src/main/sensors/battery.cpp):

- **Averages:** use signed or 32-bit storage for the 50-sample averages, in raw units ( 10 uV, mV ), with no
  floor. Clamp negative current to 0 once, after averaging.
- **Scale:** mA = shunt uV / 20 mOhm. `INA219_SHUNT_RESISTOR` stays 0.02 ( one R020 is production ).
- **Auto-gain:** remove `ina219_auto_calibrate_current ( )` and `_ina219_current_gain`, or pin the gain at 1.0
  if an extern must stay for the log line.
- **dt:** integrate in microseconds ( `mA x us` accumulator ), so no sub-ms remainder is lost.
- **Remaining:** `mAhRemain` saturates at 0 instead of wrapping.

Log fields must stay meaningful ( `I`, `G`, `D` ). `C++`: delegate to `cpp-pro`.

**Calibration ( task 15 ):** the INA219 is trusted ( ±100 mA, user ); the auto-gain goes with no replacement factor.

- **Depends on:** 2, 15
- **Skills / agent:** pluto-rules, cpp-pro agent
- **Files:** [battery.cpp](../../../../src/main/sensors/battery.cpp), [battery.h](../../../../src/main/sensors/battery.h), [PlutoPilot.cpp](../../../../PlutoPilot.cpp) ( log line )
- **Safety impact:** all battery figures; no flight-control path
- **Done when:** builds clean; the reviewer confirms the units at every boundary ( 10 uV → mA, us → mAh ) and no wrap; a bench check against the supply shows `I` within one 10 mA step.
- **Rollback:** revert battery.cpp / .h ( with task 2's driver kept ).
- **Status:** done 2026-09-28
- **Result:** signed 10 uV average → mA ( 0.5 mA/LSB ), mV bus average ( `vBat_mV` ), auto-gain removed, mA x us counter, `mAhRemain` saturates at 0, shims removed. Gate clean ( 3 old warnings fixed ), flash 99.8 KB, RAM 14.8 KB. Reviewer: 0 blocking; notes carried into tasks 4, 5, 6, 8; the bench check of `I` moves to task 8.

### 4. Plug-in estimate: averaged sample, LiPo curve, cell count

**Description.** Rewrite `handleBatteryConnected ( )`. Keep today's plug-in detection, including the 1.0 V
presence threshold, and the reconnect path, which currently zeroes the thresholds.

- **Averaged sample:** base the estimate on the averaged bus voltage over the first ~0.5 s. No blocking
  `delay ( 40 )`: compute the estimate once enough samples are in.
- **Estimate:** `E` = configured capacity × the task 1 curve's fraction at that voltage.
- **Cell count:** 1S on these targets, computed before the thresholds that use it, or by a rule that cannot give
  2 at 4.2-4.35 V.
- **Wrap:** no wrap below 3.0 V ( saturate at 0 ).

`C++`: `cpp-pro`.

**Carried from the task 3 review:** `VBATT_PRESENT_THRESHOLD_MV` is really 0.1 V units and a strict `>` ( 1.1 V ):
rename it and settle the level. No counter reset on reconnect is needed: removing the battery powers the board down
( user, 28 Sep ), so a new pack always starts from a fresh boot.

- **Depends on:** 1, 3
- **Skills / agent:** pluto-rules, cpp-pro agent
- **Files:** [battery.cpp](../../../../src/main/sensors/battery.cpp)
- **Safety impact:** the starting point of every flight's SoC; boot path ( no blocking )
- **Done when:** builds clean; `E` on the bench supply ( task 8 ) is within ±5% of the curve at every voltage and stable across power-ups at the same voltage; `Cells` 1 at 4.2 and 4.3 V.
- **Rollback:** revert the function.
- **Status:** done 2026-09-28
- **Result:** E = capacity x the §5.5 curve at the 24-sample ( ~0.5 s ) average + 120 mOhm x I, per cell, clamped, no delay; cells = ceil ( mV / 4400 ) 1..3 before the thresholds; presence at 1000 mV; no warning before E is ready. Gate clean ( 5 old warnings fixed ), flash 100.2 KB, RAM 14.8 KB. Reviewer: 0 blocking; threshold clamp and cells = 1 on disconnect fixed; SoC dip and dead-sensor settle carried to task 5. Bench accuracy in task 8.

### 5. SoC model and warnings: count-led with a voltage floor, `BMS_Update` rate

**Description.** Replace the fused SoC ( `fuse_soc_vbatComp_smart ( )`, `soc_linear_from_voltage ( )` ) with the
chosen model:

- **Count-led SoC:** SoC = `mAhRemain` / capacity.
- **Compensated voltage:** bus + `I` × pack resistance ( task 1 ), averaged.
- **The floor:** when the compensated voltage is at or below the task 1 floor, SoC and `mAhRemain` are pulled
  down to what the voltage says, so an aged pack ends at ~0.
- **Warnings:** warning and critical fire on the lower of the two readings, with hysteresis. Their actions stay
  the same ( beeper, `set_FSI`, `BatteryWarningMode` ).
- **SoC slew:** keep the rule that SoC never rises in flight.
- **`BMS_Update` rate:** run it at its real 21 ms interval ( `mw.cpp:442` compares against a constant today ), so
  any slew limit is in real time.

`C++`: `cpp-pro`; flight-safety relevant, so the reviewer pass is mandatory.

**Carried from the task 3 review:** a failed INA219 read keeps the last value with no time limit, and the service
timestamps advance anyway, so a dead sensor is silent ( frozen current integrated forever, even disarmed; the battery
never reads disconnected ). Count consecutive failures, flag the reading stale after ~250 ms, stop integrating while
disarmed and stale, and keep the warnings safe. Switch the `vBatComp` paths from `vBatRaw x 100` to `vBat_mV`.

**Carried from the task 4 review:** until `batteryEstimateReady` ( ~0.5 s after connect ) hold `soc_Fused` and skip
the fusion, or the app and CRSF see a dip ( SoC ~0.7 x the voltage SoC, `mAhRemain` 0 ) that climbs back over ~0.6 s.
If the bus reads keep failing after connect the estimate never completes and the warning stays off: cap the settle
attempts as part of the stale-sensor handling above.

- **Depends on:** 1, 4, 14
- **Skills / agent:** pluto-rules, cpp-pro agent, pluto-reviewer agent
- **Files:** [battery.cpp](../../../../src/main/sensors/battery.cpp), [mw.cpp](../../../../src/main/mw.cpp)
- **Safety impact:** low-battery warning timing and the app's SoC; no auto-land or arming change ( out of scope )
- **Done when:** builds clean; replaying log-6's `V` / `I` / `D` through the new rule offline ( a short script in the scratchpad ) gives the warning with ≥ 15% left and critical at ~5%; no path returns a SoC above the previous value while armed; an empty pack cannot read high.
- **Rollback:** revert the SoC functions and `mw.cpp:442`.
- **Status:** done 2026-09-29
- **Result:** count-led remaining with a per-flight R, `Vcomp` warning 3.745 V / critical 3.60 V per cell or 15% / 5% counted, 1.5 s debounce, latched until power-off, voltage pull-down ≤ 25%, provisional default-R alarms gated at count ≤ 40%, stale-sensor handling, no-current fallback at fixed 3.10 / 3.00 V, `BMS_Update` every 21 ms. Replay: warning at 16.5% / 17.7% left, app ~5% at empty. Gate clean ( 13 old warnings fixed ), flash 101.7 KB, RAM 14.8 KB. Reviewer: 4 rounds, all findings closed.

### 6. `Bms_Get` and `MSP_ANALOG` values against the new model

**Description.** Check every consumer of the changed values:

- **`MSP_ANALOG`** ( layout unchanged ): `vBatComp`, current, `mAhDrawn`, `mAhRemain`, SoC %, warning mode.
- **CRSF:** unchanged in scope, but it must not break.
- **`Bms_Get ( )`:** `Current` returns the post-gain value today; `Estimated_Capacity` changes meaning.

Decide with the user whether `Bms_Get` gains an SoC option. Update the `docs/API` BMS wiki ( create it if none
exists ) and bump `API_Version` if the API changes. If it does not change, log that.

**Carried from the task 3 review:** `Bms_Get ( Current )` returns `mAmpWithGain`, now ~5% above its old value ( no
gain ): note it in the wiki and decide the `API_Version` bump. CRSF current ( `mw.cpp:453`, `mAmpRaw / 10` into a
0.1 A field ) reads 10x high, an older defect; CRSF units are out of scope ( 26 Sep decision ), noted only.

- **Depends on:** 5
- **Skills / agent:** pluto-rules
- **Files:** [serial_msp.cpp](../../../../src/main/io/serial_msp.cpp), [BMS.cpp](../../../../src/main/API-Src/BMS.cpp), [BMS.h](../../../../src/main/API/BMS.h), `docs/API/`
- **Safety impact:** none ( reporting )
- **Done when:** each consumer's value and unit is listed in CHANGES.md; `MSP_ANALOG` bytes identical in layout; the API decision logged and, if changed, the wiki and version done.
- **Rollback:** revert BMS.cpp / .h.
- **Status:** done 2026-09-29
- **Result:** `Bms_Get` gains `SoC`, `Warning_Level`, `Resistance`; `Voltage` exact mV, `Current` without gain; `API_Version` 1.4.0; new `docs/API/BMS_API_WIKI.md`; every consumer's value and unit in CHANGES.md; `MSP_ANALOG` layout unchanged, CRSF untouched. Reviewer: 0 blocking; wiki accuracy and comment findings fixed.

### 7. Build gate

**Description.** Run `.claude/skills/pluto-build/driver.sh --gate PRIMUS_X2_v1`: no new warnings under `src/`, and
flash / RAM within budget. Check `PlutoPilot.cpp` warnings in `build.log`, because the gate does not scan the repo
root. Record flash and RAM against today's 99.7 KB / 14.8 KB.

- **Depends on:** 2, 3, 4, 5, 6
- **Skills / agent:** pluto-build
- **Files:** -
- **Safety impact:** none
- **Done when:** gate passes; numbers in CHANGES.md.
- **Rollback:** -
- **Status:** done 2026-09-29
- **Result:** PRIMUS_X2_v1 gate passes: no new `src/` warnings ( 13 old ones in battery.cpp fixed ), none from `PlutoPilot.cpp`; flash 101.7 KB ( was 99.7 ), RAM 14.8 KB ( unchanged ). Hex 29 Sep 01:12.

### 8. Bench supply sweep

**Description.** Repeat the check topic's supply procedure ( props off, fresh power-up per point ) at 4.2, 3.8,
3.5, 3.2, 3.0 and 2.9 V with the log line. The bench motor sequence can be re-enabled for load, then set back to 0.

Expect:

- `E` from the curve;
- `Cells` 1;
- `I` within one step of the supply;
- **no SoC jump at 3.0 / 2.9 V**;
- warning and critical at the task 1 voltages.

Results into TESTING.md.

**Carried from task 3:** check `I` against the supply's ammeter at idle and with the motors at full ( props off ):
within one 10 mA step at idle.

**Expectations updated 29 Sep:** "warning and critical at the task 1 voltages" no longer applies on the bench: the
voltage check runs only in loaded flight ( task 5 ), so at rest the level follows the count ( `E` ). The plan's table has
the expected `E`, `Cells`, `L` and `S` per voltage. Log line: `t Ph V Vc I R D S L M Arm`.

- **Depends on:** 7
- **Skills / agent:** pluto-flighttest ( user runs )
- **Files:** `logs/log-N.txt`
- **Safety impact:** props off; bench sequence back to 0 afterwards
- **Done when:** TESTING.md table meets every expectation above, or the failures become fix tasks.
- **Rollback:** -
- **Status:** done 2026-09-30
- **Result:** log-5 ( 29 Sep, capacity 800 ): `E` on the curve ( ±1%, 3.8 V +6.6% = 7 mV ), `Cells` 1 everywhere, `E` 0 and `S` 0 at 3.2 / 3.0 / 2.9 V ( no wrap, no jump ), `L` 0 / 1 / 2 by the count, `S` never rises. `I` against the supply: proper ( user, 30 Sep ). Sequence back to 0, flight hex rebuilt 15:29.

### 9. Validation flights on three packs

**Description.** Each flight: full pack, rested 10 min; hover to the app's warning, then on to critical if it is
safe; land; log 60 s at rest; then the charger mAh. Fly three packs:

- an older 600 mAh pack ( the log-6 pack itself is not identified );
- a newer 600 mAh pack;
- an 800 mAh pack, with the capacity set to 800 in the app.

Before each flight, set the capacity in the app to the pack's rating. Results into TESTING.md, analysed against
the done-when.

- **Depends on:** 8, 16 ( the remaining two flights fly the auto-land build )
- **Skills / agent:** pluto-flighttest ( user flies ), pluto-log-analyst agent for the logs
- **Files:** `logs/log-N.txt`
- **Safety impact:** flights to low battery; hover low; land at critical at the latest
- **Done when:** all three flights meet all of these:
  - `D` within ±5% of the charger;
  - app remaining at empty ≤ 5% of capacity;
  - warning with ≥ 15% really left ( by the charger ).
- **Rollback:** fly the FW 3.10.0 build.
- **Status:** done 2026-10-01
- **Result:** 4 flights ( log-3 800, log-4 older 600, log-6 800, log-7 600 ). Empty ≤ 5%: 4/4. Warning ≥ 15% really left: 3/4 ( log-3 12.9%; the same pack 20.2% on log-6 ). `D` vs charger: a steady +5.3-5.5% on healthy packs ( corrected for the charge missing at takeoff ), 8-13% on the high-R 600s; accepted as a known offset on the safe side ( user ), no calibration factor. Both warning levels came from the voltage rule on every flight.

### 10. Architecture & pipeline docs ( PIPELINE_UPDATE.md )

**Description.** Write the new `subsystems/Power_BMS_Pipeline.md` into this topic's `PIPELINE_UPDATE.md`. It
covers:

- the real functions, units and timing, and the count-led SoC with its voltage floor;
- the warning actions;
- the check topic's drift list: `ina219.cpp` naming, cA units, per-cell thresholds, the failsafe claim, 2S/3S.

Draw a Mermaid flowchart whose every edge is a real call or data path. Also update:

- the `docs/API` BMS wiki, if task 6 changed the API;
- a CLAUDE.md one-liner: one R020 = 20 mOhm is production, and a stacked second shunt halves every reading.

- **Depends on:** 9
- **Skills / agent:** spec-to-code-compliance, graphify
- **Files:** `PIPELINE_UPDATE.md`, [Power_BMS_Pipeline.md](../../fw-architecture-pipeline/subsystems/Power_BMS_Pipeline.md) ( read )
- **Safety impact:** none
- **Done when:** PIPELINE_UPDATE.md complete, every flowchart edge cited to a source line; CLAUDE.md line drafted.
- **Rollback:** -
- **Status:** done 2026-10-01
- **Result:** PIPELINE_UPDATE.md rewritten ( Power_BMS_Pipeline replacement with a cited flowchart, edits for Failsafe / MSP / User_Space_API, two CLAUDE.md lines, drift list ); BMS API wiki gains the auto-land section; APP_INTEGRATION.md revised against the current app. No `Command_Land` wiki exists: the change is in the BMS wiki and the staged User_Space_API edit.

### 11. Topic review

**Description.** Run `pluto-reviewer` on the whole topic diff ( `git diff -- src/ PlutoPilot.cpp Makefile` plus
docs ). Fix every BLOCKING finding and re-run the gate.

- **Depends on:** 10
- **Skills / agent:** pluto-reviewer agent
- **Files:** all changed
- **Safety impact:** as the reviewed code
- **Done when:** no BLOCKING findings open; the rest listed in CHANGES.md.
- **Rollback:** -
- **Status:** done 2026-10-01
- **Result:** no BLOCKING on the topic diff; 11 findings in CHANGES.md "Task 11". Fixed: auto-land and `mwArm ( )` act on a confirmed critical only ( `batteryCriticalConfirmed ( )` ); the follow-up review's one BLOCKING ( provisional never promoted by the count ) fixed and re-reviewed clean. Left: PRIMUSX2 voltage-only auto-land ( user ), minor notes 4 and 6, cleanup items moved to task 12. Gate clean on PRIMUS_X2_v1 ( flash 101.9 KB, RAM 14.8 KB ).

### 12. Remove test code, graph refresh & commit

**Description.** Remove the carried-over test code:

- from `PlutoPilot.cpp`: the battery diagnostic line, its `extern "C"` block and the `BENCH_MOTOR_SEQUENCE` block;
- from `mw.cpp` `userCode ( )`: the `DEV_MODE_LINK_GRACE` block, restoring the committed condition.

Then:

1. Re-run the gate.
2. Run `graphify update .` and `python tools/graph_labels.py`.
3. Run `/pluto-commit` for the all-target build, the FW / API version bump, `PIPELINE_UPDATE.md` → pipeline doc
   and the CHANGELOG.

The commit carries both topics' docs ( battery-capacity-estimate was closed with no commit of its own ).

- **Depends on:** 11
- **Skills / agent:** pluto-build, graphify, pluto-commit
- **Files:** [PlutoPilot.cpp](../../../../PlutoPilot.cpp), [mw.cpp](../../../../src/main/mw.cpp)
- **Safety impact:** restores the committed Developer Mode gating
- **Done when:** `git diff -- PlutoPilot.cpp` shows no test code; the `mw.cpp` diff has no `DEV_MODE_LINK_GRACE`; gate clean on all targets; commit staged and drafted; both topics Closed.
- **Rollback:** -
- **Status:** done 2026-10-03
- **Result:** test code removed ( PlutoPilot.cpp back to pre-topic, DEV_MODE_LINK_GRACE gone, `INA219_RegRead` and task-number comments removed ); FW 3.11.0; all three targets clean ( PRIMUS_X2_v1 100.9 KB / 14.8 KB, PRIMUSX2 100.7 / 14.9, PRIMUS_V5 100.9 / 14.8 ); graph refreshed; docs promoted; change staged and message drafted for the user.

### 13. Flight: full to empty on a newer 600, 60 s rest after landing

**Description.** Added 28 Sep 2026 at the user's request, after task 1. Task 1 put the pack resistance at ~100 mOhm
from log-6 but ~130-140 mOhm from the log-1 steps on the same pack, and showed the 15% voltage floor moves from
~6% to ~60% left over that range ( INVESTIGATION.md §5.4 ). A second pack, logged at mV resolution, with a real
rest after landing, settles it before the SoC model is written ( task 5 ).

- Temporary log fields in `PlutoPilot.cpp`: `Vm` ( INA219 bus in mV, one sample per tick ) and `Is` ( shunt current,
  whole mA ); `Ph` and `G` commented out to hold the byte budget. Removed with the rest in task 12.
- Fly a **newer 600** pack from full to empty on the current firmware, then **60 s at rest** with the log running.
- From the log: R from the arming and landing steps ( `Vm` / `Is` ), the recovery curve after landing and the resting
  voltage at 60 s ( the curve's empty end ), charge from the `Is` integral against the charger, the flicker / `Vm`
  points against the standard curve ( as §5.2 ), and when today's warning fired ( baseline for task 5 ).

- **Depends on:** 1
- **Skills / agent:** pluto-flighttest ( user flies ), pluto-log-analyst agent
- **Files:** [PlutoPilot.cpp](../../../../PlutoPilot.cpp), [TESTING.md](TESTING.md), [logs/log-1.txt](logs/log-1.txt)
- **Safety impact:** test code only ( diagnostic fields, two extra I2C reads per 100 ms tick in Dev Mode )
- **Done when:** log-1 analysed into TESTING.md with the pack R ( arming, landing, steady state ), the 60 s recovery,
  the `Is` integral against the charger and the fired warning; INVESTIGATION.md §5 updated if R or the thresholds move.
- **Rollback:** revert the two fields in `PlutoPilot.cpp`.
- **Status:** done 2026-09-28
- **Result:** analysed 2026-09-28 ( TESTING.md log-1, INVESTIGATION.md §5.6 ): this pack ~146 mOhm vs ~100 for log-6; a fixed-R floor would warn at 66% left; count on the rated 600 warns at 10%; per-flight R from the first 30 s matches both packs. Charger 502 mAh: raw INA219 reads 1.053x the charger ( log-6 pack 1.055x ), see task 15.

### 14. Pack survey: 2-3 more packs full to empty, fixed vs per-flight R

**Description.** Added 28 Sep 2026 ( user ). The app's remaining-mAh figure is the purpose: the count gives mAh used
to ~1-2%, but remaining also needs the pack's real capacity ( the task 13 pack holds ~567 of 600 ), and the
R-compensated voltage on the curve is what can correct it in flight. Task 13 found R per pack ( ~146 vs ~100 mOhm ),
so before task 5 picks a method, fly 2-3 more packs with the task 13 build and log line ( no code change ) and replay
offline, on every log ( log-6 of the check topic, log-1 .. log-4 here ):

- **fixed R** = the survey average;
- **per-flight R** from rest before arming to ~30 s of hover ( less the curve's drop );
- **continuous R** re-estimated through the flight ( e.g. from the curve and the count, or rolling );

and for each: the error of the remaining mAh against the curve-anchored truth over the flight, and where the 15% / 5%
warnings would fire. Procedure: TESTING.md "Task 14 plan".

- **Depends on:** 13 ( same build; the charger reading closes 13 )
- **Skills / agent:** pluto-flighttest ( user flies ), pluto-log-analyst agent
- **Files:** [TESTING.md](TESTING.md), [logs/](logs/), [INVESTIGATION.md](INVESTIGATION.md) §5
- **Safety impact:** none ( data only ); it decides the warning design in task 5
- **Done when:** log-2 .. log-4 analysed into TESTING.md ( R per pack, capacity per pack against the charger ); the three R methods compared across all packs in INVESTIGATION.md §5 with a recommendation for task 5.
- **Rollback:** -
- **Status:** done 2026-09-28 ( log-3 / log-4 dropped by the user )
- **Result:** three packs ( ~100 coarse, ~146, ~157 mOhm ); fixed R warns at 39-76% left, per-flight R from the first 30 s at 10-17%; recommendation in INVESTIGATION.md §5.7: per-flight R, warning `Vcomp` 3.745 V, critical 3.60-3.62 V, voltage floor pulls the remaining figure down.

### 15. Current calibration: INA219 against a reference meter

**Description.** Added 28 Sep 2026 ( task 13 charger reading ). On two packs the raw INA219 integral reads **~5.4%
above the charger** ( 1.053 and 1.055 ). Today the auto-gain's -5% hides it; task 3 removes the gain, and `D` would then
read ~5.5% high. Find out which side is wrong before task 3 decides whether a fixed calibration factor goes in:

- Compare `Is` with a reference current meter ( a multimeter on its 10 A range in series with the battery lead, or a
  bench supply's ammeter ) at a few steady currents, ideally one near hover ( ~4 A ). The method and the meter are
  agreed at the start of the task ( mini-grill ).
- If `Is` is ~5% high against the meter: a fixed scale in task 3 ( a board constant, not an auto-gain ). If `Is`
  agrees with the meter: the charger reads low, no factor; the acceptance against the charger in task 9 allows for it.

- **Depends on:** -
- **Skills / agent:** pluto-flighttest ( user measures )
- **Files:** [TESTING.md](TESTING.md), [ina219.c](../../../../src/main/drivers/ina219.c), [battery.cpp](../../../../src/main/sensors/battery.cpp)
- **Safety impact:** none directly; a ~5% count error shifts the remaining figure and the count-based warning by ~5%
- **Done when:** `Is` / reference ratio measured at ≥ 2 currents, recorded in TESTING.md, and the calibration decision written into task 3.
- **Rollback:** -
- **Status:** done 2026-09-28 ( user statement, no new measurement )
- **Result:** user: the current sensor is accurate to ±100 mA. No calibration factor in task 3; the charger is taken to read ~5% low ( `Is` / charger 1.053, 1.055 ), so task 9 compares `D` against the charger with that offset allowed for.

### 16. Auto-land on critical battery instead of the app's in-air disarm

**Description.** Added 29 Sep 2026 ( log-3 ). The app switches its ARM off when the flight status reports the
`LowBattery_inFlight` bit ( firmware `App_LowBattery_inFlight` = 8, app `setFlightStatus` case 8, `swArm.setChecked ( false )`;
bit 7 `App_Low_battery` only sounds, vibrates and toasts ). With task 5, critical now arrives reliably in flight at ~4%
left, so on log-3 the drone dropped from hover 0.2 s after critical. Instead, the firmware lands the drone itself:

- **Trigger:** the battery level reaches critical while armed ( the task 5 rules and debounce, unchanged ).
- **Landing:** reuse the existing `LAND` path in `command/command.cpp` ( 40 counts/s throttle ramp to 1150, touchdown by
  arrested descent / impact / 30 s timeout, then disarm ); check it runs without an app command and in and out of
  ALT_HOLD. Never start during a flip; wait for it to end.
- **Pilot ( user ):** roll, pitch and yaw stay live to steer clear of obstacles; the throttle stick is ignored; the
  landing cannot be cancelled.
- **App:** while the auto-land runs, report `Low_battery` ( bit 7: warning sound ) and not `LowBattery_inFlight`
  ( bit 8 ), and keep the `MSP_ANALOG` level at 1 ( withhold both: the app to be updated later may read the byte ).
  After the touchdown disarm, report bit 8 and level 2 so the app blocks re-arming ( latched until power-off, as now ).
- **Energy:** at critical ~4% ( ~30 mAh on the 800 ) at ~4.5 A leaves ~24 s; the ramp from a ~1710 us hover to 1200 takes
  ~13 s. Measure the descent time on the flight; if too tight, the trigger or the ramp rate is revisited here.

- **Depends on:** 7
- **Skills / agent:** pluto-rules, cpp-pro agent, pluto-flighttest ( user flies )
- **Files:** [battery.cpp](../../../../src/main/sensors/battery.cpp), [command.cpp](../../../../src/main/command/command.cpp), [serial_msp.cpp](../../../../src/main/io/serial_msp.cpp), [mw.cpp](../../../../src/main/mw.cpp); app reference: `android-app-master/.../MainActivity.java` `setFlightStatus` ( temporary copy in the repo root, not firmware )
- **Safety impact:** high. Replaces an in-air motor cut with a controlled descent; a landing that never detects touchdown, or one that goes through the stick limits, keeps the motors running at low battery ( FLIGHT_INVARIANTS: landing must not go through the stick limits; baro datum not re-zeroed )
- **Done when:** a flight to critical: the drone descends and disarms on touchdown by itself; the app does not disarm it in the air; roll / pitch steer during the descent and the throttle stick has no effect; after landing the app shows LOW BATTERY and will not arm; descent time and mAh left at touchdown recorded in TESTING.md. Build gate clean; `pluto-reviewer` on the change.
- **Rollback:** revert the task 16 change: the app's in-air disarm at critical returns.
- **Status:** done 2026-10-01
- **Result:** code in ( CHANGES.md "Task 16" ), gate clean ( flash 101.8 KB, RAM 14.8 KB ), pluto-reviewer: 1 BLOCKING ( user-code throttle in the ALT_HOLD descent ) and 1 SHOULD-FIX ( user commands could displace LAND ) fixed, re-review clean. Flights: log-4 ( 600 ) landed and disarmed by itself 2.84 s after critical ( arrested-descent rule ), user "all ok"; log-6 ( 800 ) 2.03 s after critical, by the crash detector at touchdown ( app CRASHED for ~0.3 s, then LOW BATTERY ). No in-air disarm on either.

## Decisions log

Newest last. One line each: `YYYY-MM-DD [decision|assumption|out-of-scope|risk] text`.

- 2026-09-26 [decision] Planned from the finished check topic battery-capacity-estimate ( not re-scouted ); new fix topic linked back.
- 2026-09-26 [decision] Production shunt is one R020 ( 20 mOhm ): `INA219_SHUNT_RESISTOR` stays 0.02. The stacked second R020 on the test board was a rework and has been removed.
- 2026-09-26 [decision] Scope: empty-pack wrap, late warning, accurate count ( auto-gain removed, dt, round-downs, bad samples ), better starting estimate. Out of scope: CRSF current units, the app's pre-gain current, auto-land, arming block, the Dev Mode link fix as a product feature.
- 2026-09-26 [decision] SoC model: count-led, anchored at plug-in by a LiPo resting-voltage curve on an averaged sample; a current-compensated voltage floor fires the warnings and pulls remaining to ~0 on an aged pack.
- 2026-09-26 [decision] Low-battery actions unchanged ( beeper, app flag ), earlier: warning with ≥ 15% really left, critical ~5%.
- 2026-09-26 [decision] Capacity is the configured rated value ( 600 / 800 / 1200 mAh, set by the user for the pack in use ). The ELRS 800 default in `RxConfig.cpp` stays.
- 2026-09-26 [decision] `MSP_ANALOG` layout unchanged. `Bms_Get ( )` may change, with the `docs/API` wiki and an `API_Version` bump.
- 2026-09-26 [decision] The check topic's temporary code ( diagnostic line, disabled bench sequence, Dev Mode 400 ms link grace ) carries over for the validation flights and is removed in task 12.
- 2026-09-26 [decision] Validation: bench supply sweep first, then flights on the log-6 600 pack, a newer 600 ( 5 available ) and an 800 ( 2 available ), each with the charger.
- 2026-09-26 [risk] With one R020 the INA219 range tops out at 8 A; a heavier pack ( 1200 ) or hard climbs can clip the reading and under-count. Not validated here ( no 1200 flight ).
- 2026-09-26 [assumption] The pack resistance and the voltage floor derived from one pack ( log-6 ) hold for the newer 600 and the 800 packs; the validation flights check it.
- 2026-09-28 [decision] Design numbers ( task 1, INVESTIGATION.md §5 ): R = 100 mOhm; floor `Vcomp` ( bus + I x R, full term ) warning 3.72 V, critical 3.60 V; plug-in curve = the standard 21-point 1S LiPo resting table.
- 2026-09-28 [risk] The 15% floor moves with pack R ( 80-140 mOhm puts it at ~6-60% left ); log-1 steps read ~130-140 mOhm on the same pack as log-6's ~100. Task 5 decides how the floor and the count combine; task 9 measures R per pack.
- 2026-09-28 [decision] Task 13 added ( user ): full-to-empty flight on a newer 600 with 60 s rest, `Vm` / `Is` exact fields added to the log line, before the SoC model is written.
- 2026-09-28 [risk] Task 13: pack R differs per pack ( ~146 mOhm newer 600, ~100 log-6 pack ); the task 1 fixed-R floor would warn at 66% left on the newer pack. The fixed R = 100 mOhm design value is withdrawn; task 5 chooses between a per-flight R, a higher count warning, or both.
- 2026-09-28 [decision] Task 14 added ( user ): 2-3 more packs full to empty, same build, to choose between a fixed average R and a per-flight or continuous R by offline replay; purpose is an accurate remaining mAh in the app. Task 5 now depends on 14.
- 2026-09-28 [risk] The raw INA219 integral reads ~5.4% above the charger on two packs ( 1.053, 1.055 ); the auto-gain was hiding it. Task 15 checks against a meter; task 3 depends on it.
- 2026-09-28 [assumption] The log-6 pack is not identified ( user ). Survey and validation use an older 600 in its place; packs are labelled from task 14 on, and a pack flown twice answers the day-to-day R question.
- 2026-09-28 [decision] The idle hold after arming is dropped: the app takes off on arming ( log-2 ). R comes from the first ~30 s of flight.
- 2026-09-28 [assumption] Charger IR ( 100-124 mOhm ) is the ohmic pack resistance; the bus R in flight is higher by the 20 mOhm shunt, wiring and ~25-40 mOhm polarization. Recorded per pack in task 14 as a cross-check, not used for `Vcomp`.
- 2026-09-28 [decision] Survey stopped at three packs ( user ). Recommendation for task 5: per-flight R from the first ~30 s, warning `Vcomp` 3.745 V, critical 3.60-3.62 V ( INVESTIGATION.md §5.7 ).
- 2026-09-28 [decision] Task 15 closed on the user's word: the INA219 is accurate to ±100 mA. No calibration factor; the ~5.4% gap to the charger is put on the charger, and task 9 allows for it when comparing `D`.
- 2026-09-28 [risk] If the ±100 mA was checked only at low current ( ~0.5 A, as in the check topic's log-5 ), it does not rule out a ~5% gain error at hover ( ~230 mA at 4.3 A ).
- 2026-09-28 [decision] No counter reset on battery reconnect: removing the battery powers the board down ( user ), so every pack starts on a fresh boot.
- 2026-09-28 [decision] The start estimate comes from the plug-in reading only ( averaged ~0.5 s after connect ); no refresh before the first arming ( user ).
- 2026-09-28 [decision] Task 5 design confirmed ( user ): per-flight R from the first ~30 s of loaded flight; warning when `Vcomp` ≤ 3.745 V/cell or count remaining ≤ 15% of capacity, critical at 3.60 V or 5%, whichever first; debounced ~1.5 s, latched until landing. In hover this is ~3.0-3.1 V / ~2.9-2.95 V measured.
- 2026-09-28 [decision] The voltage pulls the remaining mAh down only once it says ≤ 25% is left ( user ); mid-pack the count leads.
- 2026-09-28 [decision] The app's stored warning / minimum cell voltages ( 3.2 / 3.0 V ) no longer trigger anything ( user ); they stay in config for compatibility.
- 2026-09-29 [decision] Task 5 review fixes ( user ): until R is measured the voltage check uses a default 100 mOhm ( errs early ), and the R window accumulates loaded time across armings in one power-up.
- 2026-09-29 [decision] Low battery and critical stay on until power-off ( user ); the voltage conditions and the pull-down run only in loaded flight ( armed, ≥ 1500 mA ), so the post-landing recovery dip cannot trip them.
- 2026-09-29 [decision] Without current sensing ( feature off, or PRIMUSX2 ) the warning falls back to the raw bus voltage against the stored warning / minimum cell voltages ( 3.2 / 3.0 V ), debounced ( user ).
- 2026-09-29 [decision] Warnings raised by the default-R voltage check before R is measured are provisional: they step back to OK once the measured R shows the condition does not hold ( and the count is clear ); the default R never pulls the remaining down. Keeps the user's "early only during the first ~35 s" understanding under the latch-until-power-off rule.
- 2026-09-29 [decision] Re-review ( user ): before R is measured, the default-R voltage check is active only while the count says ≤ 40% of capacity is left, and its alarms are not latched ( they clear on disarm unless the count supports them, and are re-levelled when R is measured ). Supersedes the "from takeoff" part of the earlier default-R decision.
- 2026-09-29 [decision] No-current fallback ( user ): stored defaults become warning 3.0 V and minimum 2.9 V per cell ( loaded, hover-measured; still adjustable in the app; boards keep their stored values until a config reset ); in-flight SoC = curve at the cell voltage + 0.45 V typical hover sag, never rising.
- 2026-09-29 [decision] No-current fallback revised ( user ): fixed firmware constants, warning 3.10 V and critical 3.00 V per cell on the raw loaded voltage ( ~13-27% / ~5-13% left on the two measured packs ), in-flight SoC with a 650 mV hover sag. The app's stored warning / minimum values are not used ( the app rewrites them as 3.2 / 3.0 V with every capacity change ); the config defaults go back to 32 / 30. Supersedes the two earlier fallback decisions.
- 2026-09-29 [decision] API ( user ): `Bms_Get` gains `SoC`, `Warning_Level`, `Resistance`; `Voltage` returns exact mV; `Current` has no gain; `API_Version` 1.4.0; new `docs/API/BMS_API_WIKI.md`.
- 2026-09-29 [decision] log-3 is a task 9 flight ( 800 pack, flown on the bench build; the motor sequence aborted on arming ). The bench sweep moves to log-4.
- 2026-09-29 [risk] 800 pack: warning at ~12.9% of rated really left ( 13.9% of delivered ); pack gave ~742 of 800 mAh and the counter read +3.9%. Both rules fired within 7 s of each other.
- 2026-09-29 [decision] The app disarms on the `LowBattery_inFlight` flight-status bit ( 8 ): app case 8 switches ARM off; bit 7 only warns. This app version parses `MSP_ANALOG` in the old MultiWii layout and does not read the level byte.
- 2026-09-29 [decision] Auto-land on critical ( user ): task 16 added, supersedes "auto-land out of scope" ( 26 Sep ). The pilot steers roll / pitch / yaw, throttle ignored, no abort. The app sees bit 7 and level 1 while landing, bit 8 and level 2 after the touchdown disarm.
- 2026-09-29 [risk] The app in `android-app-master/` reads `MSP_ANALOG` as vbat8 / pMeterSum / rssi / amperage, not the firmware's vBat16 / mA / mAh drawn / mAh remain / SoC / level: its battery display is wrong against this firmware. The copy ( sources and `base.apk` dated 22 Jan 2024 ) predates FW 2.10.0 ( `ff037b6`, 6 Jan 2026 ), which moved `MSP_ANALOG` from vbat8 ( 0.1 V ) to vBat16 ( mV ); the app the user flies shows two decimals ( e.g. 3.75 V ), so it already reads the mV layout. The flown app reads the voltage as mV / 1000 ( user, 29 Sep ): this topic did not change the packet, so neither the firmware nor the flown app's MSP receive changes; only the 2024 copy here is out of date.
- 2026-09-29 [decision] Task 16 code: `batteryCriticalAutoLand ( )` re-asserts LAND every loop ( user commands cannot cancel it ) and `rcData [ THROTTLE ]` is re-pinned to `landThrottle` after `userCode ( )`. The re-pin also applies to a user `Command_Land` ( a user throttle override no longer changes its ALT_HOLD descent rate ): task 10 adds a line to the `Command_Land` API wiki. `BENCH_MOTOR_SEQUENCE` set to 0 for flights ( 1 again for the bench sweep, log-5 ).
- 2026-09-30 [risk] log-4 ( worn 600, R 173 mOhm ): `D` 491 vs charger 449 = +9.4%, outside task 9's ±5%. Ratios so far 1.039 / 1.053 / 1.055 / 1.094 vary by pack, so not a pure INA219 gain error; decide after the last 600 flight whether ±5% against the charger is the right test or a fixed counter scale is needed. Warning ( 16.6% really left ) and empty ( 1% ) passed.
- 2026-10-01 [decision] Task 16 closed: both auto-land flights landed and disarmed on the ground. A touchdown may end through the crash detector ( log-6, app CRASHED for ~0.3 s, then LOW BATTERY ); accepted, no firmware change. The app will label it a low-battery auto-land ( app developer, user ).
- 2026-10-01 [decision] log-4 was an older 600 ( user ). Packs are not labelled; the user rotates packs for variation, so task 9's "older / newer 600" becomes "two different 600 packs". log-7: a different 600 from log-4, ideally one not flown in this topic yet.
- 2026-10-01 [decision] Naming ( user ): the estimator is the **Pluto Fuel Gauge** ( technical: hybrid fuel gauge, coulomb counting with OCV initialisation and an IR-compensated voltage floor, per-flight internal resistance ); the critical-battery landing is **Low-Battery Auto-Land**. Task 10 uses these in PIPELINE_UPDATE.md ( Power_BMS_Pipeline ), the BMS API wiki and the CHANGELOG entry; avoid "Impedance Track" ( TI trademark ).
- 2026-10-01 [decision] Task 9 closed ( user ): the counter reads ~5.3% above the charger on healthy packs; accepted as a known offset ( safe side: the remaining shown is slightly low ), no calibration factor. Supersedes the open counter / charger risk of 30 Sep.
- 2026-10-01 [decision] APP_INTEGRATION.md written for the app developer ( MSP_ANALOG layout, SoC-driven % and icon, voltage max 4.20 / min 3.60 if kept, alert states incl. auto-land and the touchdown crash flag ). Task 10 links it from PIPELINE_UPDATE.md and the BMS API wiki. Open: a dedicated "auto-landing" level ( e.g. 3 while armed ) only if the app developer wants it.
- 2026-10-01 [risk] Current app source ( `android-app-dev_br_login/`, untracked, holds `drona_app.jks` ) checked: `MSP_ANALOG` 10-byte layout and `MSP_FLIGHT_STATUS` ( 255, u16, lowest set bit ) match the firmware; % = SoC byte, mAh = remaining, V = mV / 1000, icon at SoC 70 / 40 / 20. Findings: ( 1 ) with task 16 masking the app never sees level 2 while armed, so its "Auto Landing" voice / status / "Auto Land" flight-log entry never run ( it says "Low Battery, Please Land" ); ( 2 ) case 6 logs a touchdown crash-detect as "Crashed"; ( 3 ) `MSP_VOLTAGE_METER_CONFIG` sends max, min, warning but the app reads max, warning, min and writes max, warning, min, so each capacity change swaps the stored warning / minimum ( unused by the fuel gauge since task 5 ); ( 4 ) legacy path ( protocol != 1 ) still computes its own % with 2.9-4.2 V and a fixed 600 mAh. APP_INTEGRATION.md was written against the 2024 copy: revise in task 10.
- 2026-10-01 [decision] Topic review ( user ): refuse arming while a confirmed critical is latched ( finding 2 ). PRIMUSX2 voltage-only auto-land left as is: no firmware is released for PRIMUSX2 ( finding 3 ).
- 2026-10-01 [decision] Finding 1 ( user ): the auto-land starts only on a confirmed critical; a provisional one beeps and does not land.
