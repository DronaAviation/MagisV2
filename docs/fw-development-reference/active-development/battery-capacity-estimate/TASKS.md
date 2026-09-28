# Battery Capacity Estimate ( BMS check ) - Tasks

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md)

## Resume

Read this block first in a new session; read only the task it points to.

| | |
|---|---|
| **Current task** | none: topic **Closed** ( superseded by [battery-soc-fix](../battery-soc-fix/README.md) ) |
| **Next step** | none here; work continues in battery-soc-fix |
| **Open questions** | none |
| **Blocked on** | nothing |
| **Last updated** | 2026-09-26 |

## Summary

| | |
|---|---|
| **Mode** | check |
| **Goal** | Find why the coulomb counter reports ~300 mAh remaining on an empty stock 600 mAh pack, with evidence, and hand a confirmed cause and a recommendation to the fix topic. |
| **Done when** | A full-discharge log, the charger's recharge mAh and the shunt marking identify the cause; `INVESTIGATION.md` has ranked causes with numbers and a recommendation. ( The +/-10%, 5% stretch, accuracy target is the follow-on fix's done-when, not this topic's. ) |
| **Target** | PRIMUS_X2_v1 |
| **Analysis** | **Cause found ( 25 Sep ):** two R020 in parallel = 10 mOhm, code assumes 20 mOhm → every current reading is half, so the mAh count is half ( ~200 of ~400 mAh in log-1 ). Second error: the plug-in estimate ( label 600, linear curve, bus reads 0.1 V low ) ~100 mAh high. Plus the `mAhRemain` wrap: an empty pack shows 54-58% SoC ( safety ). Code losses -2 to -8%. See INVESTIGATION.md, TESTING.md log-3. |
| **Pipeline docs affected** | `subsystems/Power_BMS_Pipeline.md` ( large drift, correction staged in PIPELINE_UPDATE.md ); `Telemetry_Pipeline.md` and `MSP_Communications_Pipeline.md` noted for the fix topic |
| **Order** | 1 → 2 ( parallel with 1 ) → 3 → 11 → 4 → 12 → 13 → 5 → 6 → 7 → 8 → 9 |

## Index

| # | Title | Status | Depends on | Skills / agent |
|---|---|---|---|---|
| 1 | Pipeline audit and loss budget | done 2026-09-25 | - | pluto-rules ( read-only ), graphify |
| 2 | Shunt marking and current path on the board | done 2026-09-25 | - | user; pluto-driver ( reference ) |
| 3 | Temporary battery diagnostic line in PlutoPilot.cpp | done 2026-09-25 | 1 | pluto-flighttest, pluto-rules, pluto-build |
| 4 | Bench baseline: plug-in map and warning thresholds on the supply | done 2026-09-25 | 3, 11 | pluto-flighttest |
| 5 | Full-discharge flight with charger readback | done 2026-09-26 | 3, 4 | pluto-flighttest ( user flies ) |
| 6 | Discharge log analysis | done 2026-09-26 | 5 | pluto-log-analyst agent, pluto-flighttest |
| 7 | Findings: ranked causes, evidence, recommendation | done 2026-09-26 ( folded into TESTING.md *Task 6* and battery-soc-fix INVESTIGATION §2 ) | 2, 6 | pluto-rules |
| 8 | Power_BMS_Pipeline.md correction ( PIPELINE_UPDATE.md ) | dropped ( moved to battery-soc-fix task 10 ) | 7 | spec-to-code-compliance, graphify |
| 9 | Close: remove the diagnostic line, record findings, commit | dropped ( superseded: test code carried over; removed and committed in battery-soc-fix task 12 ) | 8 | pluto-build, pluto-commit |
| 10 | INA219 PGA /8 A/B discharge ( H6 ) | dropped ( shunt is 10 mOhm: the reading is a scale error, not clipping ) | 6 | pluto-driver, pluto-rules, pluto-build, pluto-flighttest |
| 11 | Temporary: Developer Mode survives short link drops | done 2026-09-25 | 3 | pluto-rules, pluto-build, pluto-reviewer agent |
| 12 | Current ratio sweep on the supply, props off ( H6 vs scale error ) | done 2026-09-25 ( log-3 ) | 4 | pluto-flighttest ( user runs ), pluto-rules, pluto-build |
| 13 | Repeat 4.2 / 3.5 / 3.0 V with the parallel R020 removed ( 20 mOhm ); then `BENCH_MOTOR_SEQUENCE` off | done 2026-09-25 | 4 | pluto-flighttest ( user measures ) |

Status values: `todo`, `in-progress`, `done YYYY-MM-DD`, `blocked (<why>)`, `dropped (<why>)`.
Serial numbers are never reused or renumbered.

## Tasks

### 1. Pipeline audit and loss budget

**Description.** Confirm every claim in [SCOUT.md](SCOUT.md) against the source with file:line, and write the
"Current pipeline" section of [INVESTIGATION.md](INVESTIGATION.md): INA219 register setup and units,
`bus_voltage ( )` / `shunt_voltage ( )` rounding, the 50-sample averages, `mAmpRaw = shunt mV x 50`, the
auto-gain formula and why it converges to 0.95, `dtMs` truncation, `Est` at plug-in, `mAhRemain`, the fused SoC,
and what each consumer ( `MSP_ANALOG`, CRSF, `Bms_Get` ) receives. Quantify each known loss into a loss-budget
table ( expected % under-count at hover current ) so task 6 can subtract the explained part from the measured
gap. Also read `88f594f^` vs `88f594f` for the counter so the "closer before" recollection can be judged. No code
change.

- **Depends on:** -
- **Skills / agent:** pluto-rules ( read-only ), graphify ( `graphify explain "BMS_Update"` if the CLI is available; the graph is stale, do not rely on it alone )
- **Files:** [battery.cpp](../../../../src/main/sensors/battery.cpp), [battery.h](../../../../src/main/sensors/battery.h), [ina219.c](../../../../src/main/drivers/ina219.c), [ina219.h](../../../../src/main/drivers/ina219.h), [serial_msp.cpp](../../../../src/main/io/serial_msp.cpp), [crsf.c](../../../../src/main/rx/crsf.c), [BMS.cpp](../../../../src/main/API-Src/BMS.cpp), [config.cpp](../../../../src/main/config/config.cpp), [RxConfig.cpp](../../../../src/main/API-Src/RxConfig.cpp), [mw.cpp](../../../../src/main/mw.cpp)
- **Safety impact:** none ( reading only )
- **Done when:** INVESTIGATION.md has the pipeline section with every stage cited and a loss-budget table whose total is a single % figure ( expected around -8 to -10% ).
- **Status:** done 2026-09-25
- **Result:** INVESTIGATION.md §3 audited with file:line. Code losses total **-6 to -8%** ( gain 0.95 -5%, dt 0 to -2%, round-downs ~-1% ); integrator itself sound ( uint64 ); Est 550-600 for a full pack. So the pack really delivered ~265-325 mAh to 3.1 V: either the pack ( H2 ) or a new candidate, PGA saturation on PWM current peaks ( H6, task 10 ). `88f594f` made it at most 6-8% worse.

### 2. Shunt marking and current path on the board

**Description.** The user reads the printed value of the INA219 shunt resistor on the PRIMUS_X2_v1 board
( expected `R020` = 20 mOhm; `R010` would explain a 2x under-count on its own ) and checks whether the shunt sits
in the main battery lead so the INA219 sees the whole motor current, or only part of the board. Record the
marking, the implied mA per shunt-mV, and a photo path if one is taken. Any value other than 20 mOhm becomes the
top cause and changes the expected `mAhDrawn / charger` ratio in task 6.

- **Depends on:** - ( can run in parallel with task 1 )
- **Skills / agent:** user action; pluto-driver for the INA219 PGA range check ( +/-160 mV full scale = 8 A at 20 mOhm, 16 A at 10 mOhm )
- **Files:** [ina219.h](../../../../src/main/drivers/ina219.h) ( `INA219_SHUNT_RESISTOR` )
- **Safety impact:** none
- **Done when:** The marking and the current-path answer are written in INVESTIGATION.md and the decisions log.
- **Status:** done 2026-09-25
- **Result:** Marking **R020** = 20 mOhm, matching `INA219_SHUNT_RESISTOR` 0.02 and the x50 scale ( 50 mA per shunt mV, 8 A full scale at +/-160 mV ); the shunt sits between the battery and the entire circuit, so the INA219 sees all the current. H1 ruled out.

### 3. Temporary battery diagnostic line in PlutoPilot.cpp

**Description.** Add a diagnostic `Monitor_Print` line to `plutoLoop ( )` ( 100 ms tick ) so PlutoMonitor records
the battery pipeline, which it does not get from MSP: `V` ( `Bms_Get ( Voltage )` mV ), `Vc` ( `vBatComp` ), `I`
( `mAmpRaw` ), `G` ( gain x 1000 ), `D` ( `Bms_Get ( mAh_Consumed )` ), `S` ( fused SoC % ), `M` ( mean motor command, for the H6 check: current flattening as the motors are driven harder ), `t` ( `millis ( )` ), `Arm`; and `E` / `Cap`
( `Bms_Get ( Estimated_Capacity )` / `Battery_Capicity` ) once, on the first tick. Internals not exposed by `Bms_Get` need an
`extern "C"` declaration in [battery.h](../../../../src/main/sensors/battery.h) or a `Bms_Get` field; prefer
the smallest change. Budget: 114 B realistic, 121 B worst per tick, under the 130 B ceiling ( reviewer's count ). This line is temporary and is
removed in task 9 ( or handed to the fix topic ). Approved by the user on 2026-09-25.

- **Depends on:** 1
- **Skills / agent:** pluto-flighttest ( log field plan ), pluto-rules, pluto-build ( `--gate PRIMUS_X2_v1` )
- **Files:** [PlutoPilot.cpp](../../../../PlutoPilot.cpp), [battery.h](../../../../src/main/sensors/battery.h) ( only if a field must be exposed )
- **Safety impact:** none in flight ( logging only, Developer Mode ); keep the tick under 130 B so the app link stays up
- **Done when:** Build gate clean on PRIMUS_X2_v1 with no new `src/` warnings; a 30 s bench log shows all fields at 10 Hz and the app stays connected.
- **Rollback:** Delete the added lines from `PlutoPilot.cpp` ( and any `extern` added ); rebuild.
- **Status:** done 2026-09-25
- **Result:** Line in place and reviewed; gate clean, 99.7 KB / 14.8 KB. [log-1](logs/log-1.txt): all nine fields at ~10 Hz for 375 s, no overrun. The user flew the pack to empty, which **reproduced the symptom** ( E 500 - D 203 = 297 remaining ) and showed the counter matches an independent integral of `I` within 1.4%: the gap is in the current reading or the pack ( TESTING.md ).

### 4. Bench baseline: plug-in map and warning thresholds on the supply

**Description.** On the bench supply ( leads into the battery connector, props off, 1 A limit ), boot the board
at 4.30, 4.20, 4.15, 4.10, 4.00, 3.90, 3.80, 3.50, 3.20, 3.00 and 2.90 V and record `E`, `Cells` and `V` at
each ( test A ), to check the plug-in formula, the floored single sample, the 2-cell count at >= 4.2 V and the
wrap below 3.0 V. Then walk the supply from 3.80 V down to 3.00 V in 0.05 V steps and note where the app's
warning and critical flags trip ( test C ). Procedure and prediction table: TESTING.md *Task 4 plan*. Replaces
the pack-based runs planned first ( user has a bench supply, 25 Sep ).
- **Depends on:** 3, 11 ( uninterrupted bench log )
- **Skills / agent:** pluto-flighttest
- **Files:** none ( bench procedure )
- **Safety impact:** none ( props off )
- **Done when:** TESTING.md has the measured `E` / `Cells` / `V` against the prediction at every voltage, whether the board boots at 2.9 V, and the SoC / voltage at which the warning and critical flags trip.
- **Status:** done 2026-09-25
- **Result:** [log-3](logs/log-3.txt): `E` follows the formula exactly with the bus reading 0.1 V below the supply; `Cells` 2 at 4.3 V; at 3.0 / 2.9 V the `mAhRemain` wrap makes the fused SoC jump to 54-58% on an empty pack ( safety, H5b ). Test C replaced by the `S` map ( warning between 3.5 and 3.2 V, critical ~3.2 V ). Bonus: the INA219 reads ~half the supply current at DC ( idle and 100% duty ): H6 ruled out, H7 added.

### 5. Full-discharge flight with charger readback

**Description.** The user flies one full hover-to-empty discharge on PRIMUS_X2_v1 with PlutoMonitor logging
( Developer Mode on ): start from a charged pack, hover until the app low-battery indication or 3.1 V under load,
land, keep logging for 60 s at rest and note the resting voltage, then charge the pack on the charger and record
the mAh it puts back. Note the pack's identity and age. A second pack, if available, distinguishes a worn pack
( H2 ) from a scale error ( H1 ). Test plan from `pluto-flighttest`; place logs under the topic's `logs/` or
record the path.

- **Depends on:** 3, 4
- **Skills / agent:** pluto-flighttest ( user flies )
- **Files:** none
- **Safety impact:** flight to low battery: hover only, low altitude, land on the app warning; no auto-land exists ( `failsafeOnLowBattery` is disabled )
- **Done when:** At least one `logs*.txt` from full to rest, the resting voltage after landing, and the charger's recharge mAh are recorded in TESTING.md.
- **Status:** done 2026-09-26
- **Result:** [log-6](logs/log-6.txt): full pack, one R020, 467 s; `D` 528 vs charger **534** ( 0.99 ); pre-gain integral 556 ( 1.04 ); app remaining at empty 22 mAh ( was 297 ). The pack holds ~534 mAh. Warning ~15 s before the end.

### 6. Discharge log analysis

**Description.** Analyse the task 5 log with `tools/flightlog.py` and the `pluto-log-analyst` agent: firmware
`D` at landing vs charger mAh ( the headline ratio ); an independent integral of logged `I` over time vs `D`
( isolates the integration losses of H3 from the scale of H1 ); the `G` trajectory ( expected to reach 0.950
within ~10 s armed ); `V` vs `D` curve shape; `E` at start. Subtract the task 1 loss budget from the measured
gap and state which hypothesis the remainder matches. Results into TESTING.md.

- **Depends on:** 5
- **Skills / agent:** pluto-log-analyst agent, pluto-flighttest
- **Files:** [flightlog.py](../../../../tools/flightlog.py)
- **Safety impact:** none
- **Done when:** TESTING.md has the ratio table ( firmware mAh, integral mAh, charger mAh ), the gain trajectory, and a one-line verdict per hypothesis ( confirmed / contributes X% / ruled out ).
- **Status:** done 2026-09-26
- **Result:** TESTING.md *Task 6*: log-6 `D` / charger 0.99, integral 1.04; gain 0.95 at 28 s and fixed; lost records do not move the integral; fused SoC 21-24% at empty ( late warning ); verdict per hypothesis ( H1 confirmed, H2 ruled out, H3/H4/H5 confirmed, H6 ruled out, H7 = H1 ).

### 7. Findings: ranked causes, evidence, recommendation

**Description.** Write the Findings section of INVESTIGATION.md: the causes ranked with their measured share of
the gap, the evidence for each ( log figures, marking, code lines ), the edge bugs found on the way ( H5 ), and a
recommendation for the fix topic within its constraints: no hardware change, `MSP_ANALOG` layout unchanged,
accuracy target +/-10% ( 5% stretch ) against the charger. Include what the fix should decide: voltage
re-anchoring of the count, fixing the auto-gain units or removing it, `dtUs` accumulation, the CRSF unit,
`BMS_Update` timing, and whether low battery should do more than beep ( the user did not exclude failsafe
changes ). Update README.md status text.

- **Depends on:** 2, 6
- **Skills / agent:** pluto-rules
- **Files:** [INVESTIGATION.md](INVESTIGATION.md), [README.md](README.md)
- **Safety impact:** none
- **Done when:** INVESTIGATION.md Findings has every hypothesis closed with a verdict and a number, and a recommendation the fix grill can start from.
- **Status:** todo
- **Result:**

### 8. Power_BMS_Pipeline.md correction ( PIPELINE_UPDATE.md )

**Description.** `fw-architecture-pipeline/subsystems/Power_BMS_Pipeline.md` describes a BMS that is not in the
code ( `ina219.cpp` / `ina219Init` / `batteryUpdate` names, cA units, per-cell voltage thresholds, a failsafe
action, 2S/3S ). Stage a corrected version in this topic's `PIPELINE_UPDATE.md` describing the committed
behaviour only ( real functions, mA units, SoC thresholds 18/8, beeper/LED/app-flag actions, 1S, the 21 ms
sampling and the every-loop `BMS_Update` ), with a Mermaid flowchart whose every edge is a real call or data path
confirmed by a source line. Note the `Makefile` `drivers/ina219.cpp` vs `ina219.c` discrepancy and the
`Telemetry_Pipeline.md` `vbat` name for the fix topic. Do not edit the pipeline folder directly.

- **Depends on:** 7
- **Skills / agent:** spec-to-code-compliance, graphify ( `graphify path` for the flowchart edges )
- **Files:** [Power_BMS_Pipeline.md](../../fw-architecture-pipeline/subsystems/Power_BMS_Pipeline.md) ( read ), `PIPELINE_UPDATE.md` ( write )
- **Safety impact:** none
- **Done when:** `PIPELINE_UPDATE.md` exists with the full replacement text and every flowchart edge has a cited source line.
- **Status:** todo
- **Result:**

### 9. Close: remove the diagnostic line, record findings, commit

**Description.** Remove the task 3 diagnostic line, the task 12 bench motor sequence ( `BENCH_MOTOR_SEQUENCE` block ),
the task 11 Developer Mode change in `userCode ( )` and the task 10 PGA change if it was flown ( or, if the user goes straight to `/pluto-grill fix battery-capacity-estimate`, log which
of them is kept for the fix topic and skip its removal ), run the build
gate on PRIMUS_X2_v1 to confirm the tree is back to baseline, then `/pluto-commit` to record the findings,
apply `PIPELINE_UPDATE.md` to `Power_BMS_Pipeline.md`, set the topic **Closed** here and in
`active-development/README.md`. No firmware behaviour changes in this topic, so no version bump beyond what
`pluto-commit` decides for a docs-only change.

- **Depends on:** 8
- **Skills / agent:** pluto-build ( `--gate PRIMUS_X2_v1` ), pluto-commit
- **Files:** [PlutoPilot.cpp](../../../../PlutoPilot.cpp), [mw.cpp](../../../../src/main/mw.cpp), [ina219.c](../../../../src/main/drivers/ina219.c)
- **Safety impact:** none ( restores the committed gating )
- **Done when:** `git diff -- src/ PlutoPilot.cpp` is empty ( or what is handed to the fix topic is logged, and `BENCH_MOTOR_SEQUENCE` is gone in any case ), gate clean, commit staged and drafted; topic status Closed.
- **Rollback:** n/a ( docs and a removed diagnostic line )
- **Status:** todo
- **Result:**

### 10. Conditional: INA219 PGA /8 A/B discharge ( H6 )

**Description.** Run only if task 6 leaves a gap the pack does not explain ( charger puts back well over what the
firmware counted after removing the -6 to -8% code losses ). H6: the shunt carries the pulsed 20 kHz brushed-motor
current, and on-phase peaks above the +/-160 mV ( 8 A ) PGA range saturate the ADC so the average reads low. Build
a temporary diagnostic firmware with `INA219_CONFIG_GAIN_8` ( +/-320 mV = 16 A ) in `INA219_Init ( )`, keep
everything else identical ( the shunt register LSB stays 10 uV, so the x50 scale is unchanged ), and fly the same
hover discharge as task 5 with the same pack and charger. If `D` / charger rises toward ~0.93, H6 is confirmed.
Needs the user's go-ahead: it is a temporary driver change in a `check` topic, reverted in task 9.

- **Depends on:** 6
- **Skills / agent:** pluto-driver ( INA219 config ), pluto-rules, pluto-build ( `--gate PRIMUS_X2_v1` ), pluto-flighttest
- **Files:** [ina219.c](../../../../src/main/drivers/ina219.c)
- **Safety impact:** none on flight control ( measurement only ); same low-battery precautions as task 5
- **Done when:** TESTING.md has the /4 vs /8 comparison: `D`, integral of `I`, charger mAh for each, and a verdict on H6.
- **Rollback:** Restore `INA219_CONFIG_GAIN_4`; rebuild.
- **Status:** todo
- **Result:**

### 11. Temporary: Developer Mode always on, survives short link drops

**Description.** For the discharge logs of this topic, user code should run without the AUX switch and should not
be torn down by the every-3-s RC frame gaps seen in log-1 ( TESTING.md, *Link timing* ). In `userCode ( )`
( [mw.cpp](../../../../src/main/mw.cpp) `:1111-1170` ) replace the switch-and-link condition with: run user code
from the first RC frame onwards and keep running while the link was seen less than a grace window ago
( `DEV_MODE_LINK_GRACE_US`, **400 ms**; the 1 s planned first was wrong, see the decisions log ), so a real RC loss
still stops user code before failsafe acts and the existing teardown ( `onLoopFinish ( )`, override clear ) still runs then. Keep `devmode`
( OLED "DEV" ) following the same flag. Mark the block TEMPORARY with the topic name; a single `#define` must
switch it back to the committed behaviour. Reviewed by `pluto-reviewer` against the override-expiry and failsafe
rules. Requested by the user on 2026-09-25 ( "switch always on, survive short link drops; temporary, this topic
only" ).

- **Depends on:** 3
- **Skills / agent:** pluto-rules, pluto-build ( `--gate PRIMUS_X2_v1` ), pluto-reviewer agent
- **Files:** [mw.cpp](../../../../src/main/mw.cpp)
- **Safety impact:** user code and its `RcCommand_Set` overrides keep running for up to ~600 ms after the last RC frame ( today: 200 ms ), while the channels are held anyway; they stop ~400 ms before failsafe lands the craft ( ~1 s ). Reviewer caveats, both irrelevant to this topic's `PlutoPilot.cpp` ( no RC override ): ( a ) a **throttle** override active during a frame gap feeds its own blended output back through the held `rcData` into `rcDataPilot`, so the pilot's throttle share fades over the gap and steps back when the next frame arrives ( real fix: hold `rcDataPilot` at `rx.cpp:528`, out of scope ); ( b ) applied only to the first version: with the switch restored ( 18:13 amendment ) the pilot can stop user code with AUX as before. The pilot cannot switch user code off with AUX while this is in. Temporary.
- **Done when:** Gate clean; a 60 s bench log with the app connected shows no `E` / `Cap` restart marker after the first one, and disconnecting the app stops the log within ~0.6 s.
- **Rollback:** Set the `#define` back ( or revert the block ); rebuild. Removed in task 9.
- **Status:** done 2026-09-25
- **Result:** Gate clean; reviewer: no blocking, wrap-around nit fixed. [log-2](logs/log-2.txt): one restart marker in 51 s ( log-1: one per 2.9 s ), all fields in all 491 records. The disconnect-stop check cannot be observed through the app log and rests on the reviewed code timeline. 12 unmarked one-tick gaps, likely records lost on Wi-Fi ( TESTING.md ).

### 12. Current ratio sweep on the supply, props off ( H6 vs scale error )

**Description.** The flight data ( log-1 + charger ) says the INA219 reads ~60-64% of the true current. With the
supply at 3.80 V, current limit 3 A, props off, **disarmed**, the firmware's bench sequence ( `BENCH_MOTOR_SEQUENCE`
in `PlutoPilot.cpp`, CHANGES.md ) holds the four motors at 1000 / 1250 / 1500 / 1750 / 2000 us for 10 s each when Dev
Mode is switched on; the user reads the supply's current display at each level ( `Ph` in the log ). Brushed-motor PWM makes the
shunt current steady at 100% throttle and pulsed ( ~2x average ) at 50%, so the ratio `I` / supply-current
against throttle separates a constant scale error ( parallel path around the shunt, sense connection, INA219
setup ) from a reading that loses the pulses ( PGA clipping or sampling ). Procedure and decision table:
TESTING.md *Task 12 plan*. Decides whether task 10 runs. The macro stays on for task 13, whose last step sets `BENCH_MOTOR_SEQUENCE` to 0.

- **Depends on:** 4
- **Skills / agent:** pluto-flighttest ( user runs; analysis in the next `/pluto-task next` )
- **Files:** none ( bench procedure; log in `logs/log-4.txt` )
- **Safety impact:** motors driven while disarmed through `motor_disarmed [ ]`, props off, supply current-limited at 3 A; stops on Dev Mode off, link loss, arming or sequence end. **`BENCH_MOTOR_SEQUENCE` must be 0 before task 5's flight.**
- **Done when:** TESTING.md has the ratio at each throttle level with the supply reading, and one of the three verdicts from the decision table.
- **Status:** done 2026-09-25
- **Result:** Answered by log-3 ( the sequence ran in every task 4 run ): INA219 / supply ~0.4-0.5 at idle ( no PWM ) and at 100% duty alike → constant scale error, not clipping ( decision table row 1 ). Cause found by the user: a second R020 in parallel, 10 mOhm vs the 20 mOhm the code assumes.

### 13. Repeat 4.2 / 3.5 / 3.0 V with the parallel R020 removed ( 20 mOhm ); then `BENCH_MOTOR_SEQUENCE` off

**Description.** The user removes the second R020, leaving one 20 mOhm shunt, the value the firmware assumes, and
repeats the log-3 procedure at three supply voltages ( 4.2, 3.5, 3.0 V ): fresh power-up each, app, PlutoMonitor, Dev
Mode on ( the motor sequence provides the load ), supply idle and load current noted. Expected: `I` doubles against
log-3 and now tracks the supply ( idle ~100, motors ~450-500 in 50 mA steps ); `E`, `Cells`, `V` unchanged. This is
the direct confirmation of H1. Caution recorded below on running a single R020 in flight. Last step: set
`BENCH_MOTOR_SEQUENCE` to 0 and rebuild ( gate ).
- **Depends on:** 4
- **Skills / agent:** pluto-flighttest ( user measures; result into TESTING.md and INVESTIGATION.md H7 )
- **Files:** none ( bench measurement ); [PlutoPilot.cpp](../../../../PlutoPilot.cpp) for the macro at the end
- **Safety impact:** motors running props off on a current-limited supply; probe carefully near the motor drivers
- **Done when:** TESTING.md has the three-point table ( supply vs `I`, idle and load ) with one R020 against log-3's two, and `BENCH_MOTOR_SEQUENCE` is 0 with the gate clean.
- **Status:** done 2026-09-25
- **Result:** [log-5](logs/log-5.txt): with one R020 `I` tracks the supply within one 50 mA step ( idle 100-150, load 450-500 ) where log-3 read half: H1 confirmed directly. `BENCH_MOTOR_SEQUENCE` set to 0 and fully compiled out; gate clean ( 20:35 hex ).

## Decisions log

Newest last. One line each: `YYYY-MM-DD [decision|assumption|out-of-scope|risk] text`.

- 2026-09-25 [decision] Mode `check` first; the fix is planned afterwards with `/pluto-grill fix battery-capacity-estimate` ( user confirmed ).
- 2026-09-25 [decision] Target PRIMUS_X2_v1 ( `selected_target` in plutoide.ini ).
- 2026-09-25 [decision] Done-when for this topic is a cause with evidence; the accuracy target +/-10% ( 5% stretch ) against the charger's recharge mAh belongs to the fix topic.
- 2026-09-25 [out-of-scope] No hardware change ( no new shunt or current sensor ). No change to the `MSP_ANALOG` layout or any MSP/CRSF field the app parses. Low-battery behaviour and the `Bms_Get` API were not excluded and may change in the fix topic.
- 2026-09-25 [decision] Symptom: Pluto app battery widget, both `Rx_ESP` ( capacity 600 ) and `Rx_ELRS` ( capacity 800 ), stock 600 mAh 1S pack; ~300 mAh remaining at 3.1 V both under load and at rest after landing.
- 2026-09-25 [decision] Evidence so far is the app display only; the reference instrument is a charger with mAh readback ( no ammeter, no BOM ). The user reads the shunt marking and flies the discharge with PlutoMonitor logging.
- 2026-09-25 [decision] A temporary `Monitor_Print` diagnostic line in `PlutoPilot.cpp` is allowed ( PlutoMonitor does not record MSP battery fields ), under 130 B per tick, removed at close or handed to the fix topic.
- 2026-09-25 [assumption] "Was closer before 278f77d": that commit changed only `CURR_CAL_ALPHA` 0.002 to 0.003 and left the current conversion numerically identical, so the recollection refers to firmware before the BMS rewrite `88f594f` ( 2025-12-31 ), which introduced the auto-gain, the dt truncation and the u16 ring buffers ( ~-8% together ).
- 2026-09-25 [risk] A 2 mOhm shunt is ruled out by arithmetic ( it would show ~540 mAh remaining because the auto-gain gate would never open ) and `PE1206FRE470R02L` reads R02 = 20 mOhm; but any other value ( R010 ) or a partial current path would explain the gap on its own. Task 2 settles it.
- 2026-09-25 [risk] The user's suspicion "pack is worn" cannot be separated from a scale error by the app alone; the charger readback in task 5 is the only way, ideally with two packs.
- 2026-09-25 [decision] `Power_BMS_Pipeline.md` drifts from the code in names, units, thresholds and failsafe; the correction is staged in this topic ( task 8 ), not applied to the pipeline folder during the work.
- 2026-09-25 [decision] Shunt marking is R020 ( 20 mOhm ): the code's current scale is right. H1 is reduced to "the INA219 does not see all the motor current"; the ~2x gap must otherwise come from the pack ( H2 ), the integrator ( H3, task 1 audits the arithmetic ) or the plug-in estimate ( H4, including the 2-cell count at 4.2 V ).
- 2026-09-25 [decision] The shunt sits between the battery and the entire circuit ( user ): the INA219 sees all the current. H1 ruled out; the gap is the pack ( H2 ), the integrator and losses ( H3 ) or the plug-in estimate ( H4 ).
- 2026-09-25 [assumption] Unknown whether the ~300 figure was seen on more than one pack; task 5 flies two packs if two are available.
- 2026-09-25 [decision] Task 1 loss budget: the code under-counts by -6 to -8% ( auto-gain 0.95, dt remainder, round-downs ); the integrator is sound; Est is 550-600 for a full pack. Implied delivery to 3.1 V: ~265-325 mAh.
- 2026-09-25 [risk] New hypothesis H6: INA219 PGA /4 ( 8 A ) saturates on the pulsed 20 kHz motor current peaks, reading the average low. Added task 10 ( conditional PGA /8 A/B discharge ) after task 6, and a throttle field `T` to the task 3 log line.
- 2026-09-25 [decision] Task 3 log line: `t V Vc I G D S M Arm` at 10 Hz ( 114 B realistic ), `E Cap` alone on the first tick after Developer Mode ( reviewer: the two lines in one slot would be 144-153 B ). `M` ( mean motor command ) replaces the planned throttle field because it is what the shunt current follows.
- 2026-09-25 [decision] log-1 ( user flew the pack to empty instead of a 30 s bench check ): symptom reproduced ( 297 remaining at empty ), counter faithful to its own reading ( 202 vs 204.8 mAh integral ), hover `I` flat at ~2.25 A. Task 3 closed; log-1 counts as a partial task 5 run ( pack started at 4.0 V; no charger figure yet ).
- 2026-09-25 [decision] The auto-gain loss measured in flight is smaller than the task 1 budget ( G 1000 → 959, learned only late ) because `sag_obs >= 20` is rarely met at ~45 shunt mV. Code losses in this flight: ~-2 to -3%.
- 2026-09-25 [out-of-scope] `onLoopStart ( )` re-ran 121 times in 375 s with Developer Mode on throughout ( user ): the `Rx_ESP` link-loss timeout is 200 ms ( `DELAY_5_HZ`, `rx.cpp:369` ) and about once every 3 s one RC frame from the app arrives later than that; 113 of 120 drops lasted under one tick. Drone → app delivery was smooth ( wall-clock drift linear, no bunching ), so the debug output rate is not implicated; an A/B with the line halved is untested. Documented in `pluto-flighttest` ( "Developer Mode restarts" ) and the `pluto-log-analyst` checklist; TESTING.md *Link timing*. Candidate for its own `check` ( timeout vs app send rate; user-code state lost at each flicker ).
- 2026-09-25 [decision] `tools/flightlog.py` import failure ( shadowed by `tools/warnings.py` ) and its `t`-as-seconds summary logged as `.claude/TOOLING_BACKLOG.md` item 12; runpy workaround used meanwhile.
- 2026-09-25 [decision] Shunt part confirmed by the user: PE1206FRE470R02L ( Yageo, 0.02 Ohm, 1% ), **one** resistor in the battery path. Parallel shunts ruled out; H1 fully closed.
- 2026-09-25 [decision] Charger put back **430 mAh** into the log-1 pack from 3.5 V resting. With 4.0 V resting taken as 75-80% charge: flown ~317-339 mAh, firmware `D` 202, so the reading is **~60-64% of true**; `E` 500 was ~160-180 mAh too high. The 297 mAh symptom is two errors of about equal size ( estimate + measurement ). The pack holds ~450 mAh, 75% of its label.
- 2026-09-25 [assumption] 4.0 V resting = 75-80% charge and ~5% left below 3.5 V resting ( typical LiPo curve, not measured on this pack ). Task 5 from a full pack removes it.
- 2026-09-25 [decision] Task 10 ( PGA /8 A/B ) is no longer optional: its condition ( charger well above the count ) is met. It still needs the user's go-ahead because it changes the driver temporarily. Order: 5 → 6 → 10 → 7.
- 2026-09-25 [decision] User asked for Developer Mode on by default. Scoped: switch always on and user code survives sub-second link drops ( still stops before failsafe, 1 s ), temporary for this topic only: task 11, reverted in task 9. Product-default behaviour, if wanted later, is its own `/pluto-grill`.
- 2026-09-25 [decision] Task 11 grace is **400 ms**, not 1 s. Armed failsafe commands LAND as soon as `rxLinkState` goes down ( `failsafe.cpp:241-248` ), which is `PERIOD_RXDATA_FAILURE` ( 200 ms ) after the channels stop being valid, and they are held valid for `MAX_INVALID_PULS_TIME` ( 600 ms, `rx.cpp:87` ) after the last *refresh*, and the retained MSP frame refreshes them at 50 Hz while the signal flag is up ( `rx/msp.c:37-41`, `rx.cpp:535-537` ), so until ~780-800 ms: LAND at ~1 s ( reviewer's correction ). `failsafe_delay` ( 1 s ) only sets `rxDataFailurePeriod`, which nothing reads. `rxIsReceivingSignal ( )` lasts 200 ms after the last frame, so user code stops at ~600 ms. Covers every armed gap in log-1 ( longest ~305 ms ).
- 2026-09-25 [risk] Reviewer ( task 11 ): with user code running through a frame gap, a *throttle* `RcCommand_Set` override feeds its own output back via the held `rcData` into `rcDataPilot` ( `rx.cpp:528`, `:564` ), so the pilot's share fades during the gap and steps back after it. Not reachable in this topic ( no override in `PlutoPilot.cpp` ); if Dev Mode ever becomes always-on for real, `rcDataPilot` must be held at the pilot's last stick, not at `rcData`.
- 2026-09-25 [decision] Task 11 closed on log-2: no restarts in 51 s. The "app disconnect stops user code within ~0.6 s" check is not observable through the app log ( the log itself stops ); accepted on the reviewed code timeline.
- 2026-09-25 [risk] log-2 shows 12 one-tick gaps with no restart marker ( one per ~4 s ). Likely debug records lost on the Wi-Fi link in the drone → app direction during the same hiccups that delay RC frames; a main-loop stall cannot be excluded from `t` alone. Task 6 interpolates across gaps with `t`. `pluto-flighttest` and `pluto-log-analyst` corrected: a gap without a marker is ambiguous unless the line carries a tick counter.
- 2026-09-25 [decision] Task 11 amended ( user ): the always-on start sent the one-time `E Cap Cells` line before PlutoMonitor was listening. The Dev AUX switch is back in charge of starting and stopping user code; only the 400 ms link grace is kept ( `DEV_MODE_LINK_GRACE` ). Gate clean; log-2's no-restart result still applies to the link half.
- 2026-09-25 [decision] The user has a bench supply ( 3 A max, 10 mA current display, leads into the battery connector ). Task 4 becomes the plug-in map and warning-threshold sweep on the supply ( exact voltages ). New task 12: current ratio sweep with props off at 3.8 V, using the supply's current display as the reference the topic lacked; its ratio-vs-throttle shape decides whether task 10 ( PGA /8 ) is worth flying. Order: 4 → 12 → 5 → 6 → 10 ( conditional ).
- 2026-09-25 [risk] User: idle current used to read 100 mA on the older BMS and reads 50 mA now. 50 mA is one whole-mV step of the shunt reading ( 20 mOhm x 50 ), so any idle draw from 50 to 99 mA shows as 50 after the two round-downs ( ina219.c:64, maths.cpp:381 ); the pre-rewrite code took one unrounded-average sample. Task 12's disarmed level ( supply display vs `I` ) settles the true idle draw.
- 2026-09-25 [decision] Task 12 automated at the user's request: `PlutoPilot.cpp` bench sequence drives `motor_disarmed [ 0..3 ]` ( mixer's disarmed path ) at 1000 / 1250 / 1500 / 1750 / 2000 / 1000 us, 10 s each, once per Dev Mode start, disarmed only; `Ph` logged. `Motor.cpp`'s API was not used: it writes the same `motor_disarmed [ ]` entries ( M5-M8 map onto motors 3, 2, 0, 1 ) but is gated on `BOXARM` and sets `usingMotorAPI`, so the sequence writes the array directly. `BENCH_MOTOR_SEQUENCE` must be 0 for any flight.
- 2026-09-25 [decision] Reviewer ( bench sequence ): safe on the bench, props off, disarmed; every exit writes 1000. Applied: task 9 removes the sequence and checks `PlutoPilot.cpp` too; task 12 ends by setting `BENCH_MOTOR_SEQUENCE` to 0; `#warning` on every build while it is on; level rewritten every tick. Noted: a Wi-Fi drop over 400 ms re-runs the sweep on reconnect ( split log-4 at each `E:` marker ); `M` lags `Ph` by one row at each change ( drop the first rows of a phase ).
- 2026-09-25 [risk] User: the frame drops may come from the PC software. Lost records towards the PC ( log-2 ) plausibly yes; late RC frames at the drone ( log-1 ) are phone-sent and drone-timed, but PlutoMonitor is a second client on the ESP bridge. Offered test: a drop counter in the temporary mw.cpp block printed in the first line, read after 5 min with the phone only. Not started.
- 2026-09-25 [decision] Task 4 closed on log-3 ( bench supply, 10 voltages ). `E` formula confirmed with the bus reading 0.1 V below the supply; `Cells` 2 at 4.3 V; test C replaced by the logged `S` map.
- 2026-09-25 [risk] **Safety, measured:** at 3.0 V and below the `mAhRemain` wrap makes `soc_from_mAh ( )` return 100% and the fused SoC read 54-58% on an empty pack; the low-battery warning clears ( after landing, or at plug-in of an empty pack ). Must be fixed in the fix topic regardless of the current error.
- 2026-09-25 [decision] log-3 shows the INA219 reading ~0.4-0.5 of the supply current at DC ( idle with motors off; motors at 100% duty ). H6 ( PWM / PGA clipping ) ruled out as the main cause; task 10 expected to be dropped when task 12 closes. New H7 ( board current path: bypass around the shunt or sense routing ) and task 13 ( multimeter across the R020 pads ). `BENCH_MOTOR_SEQUENCE` stays on for task 13; task 13 ends by setting it to 0.
- 2026-09-25 [assumption] The last log-3 run labelled `2.0v` was 2.9 V ( firmware bus read 2.8 V, `E` 65436 ).
- 2026-09-25 [decision] **Cause of the current under-read found ( user ): a second R020 stacked in parallel on the first, 10 mOhm total; the firmware converts as 20 mOhm ( `INA219_SHUNT_RESISTOR` 0.02, shunt mV x 50 ), so every reading is exactly half.** H1 confirmed ( it was "ruled out" on a one-resistor count ); H6 and H7 closed; task 10 dropped; task 12 closed on log-3; task 13 reduced to an optional 5 mV cross-check plus switching the bench sequence off.
- 2026-09-25 [risk] Whether the parallel R020 is on every PRIMUS_X2_v1 ( and PRIMUS_V5 ) or a rework on this board decides the fix: a constant ( 0.01 ) or a per-board calibration. Ask before the fix grill. The old `PE1206FRE470R02L` "2 mOhm" comment and the `0.4` constant before 278f77d suggest the value has been uncertain for a while.
- 2026-09-25 [decision] User removes the parallel R020 and repeats 4.2 / 3.5 / 3.0 V ( task 13, log-5 ): with one 20 mOhm shunt the firmware's scale is right and `I` should double against log-3.
- 2026-09-25 [risk] A single R020 in flight: 20 mOhm x ( 5 A )^2 = 0.5 W in one 1206 ( PE1206 is rated 1 W ), 100 mV drop at 5 A, and the INA219 PGA /4 range ( +/-160 mV ) clips at 8 A instead of 16 A. The pair may be deliberate. Decide in the fix whether the production board keeps two ( then the constant becomes 0.01f ) or one ( code as is ).
- 2026-09-25 [decision] Task 13 closed on log-5: with the second R020 removed the reading tracks the supply ( H1 confirmed directly ). Bench motor sequence off ( 20:35 build ). The board now has **one** R020, matching the firmware.
- 2026-09-25 [risk] Task 5 on this board would fly one R020: ~0.5 W in one 1206 ( PE1206 rated 1 W ) at a 5 A hover-plus, 100 mV drop, INA219 range clips at 8 A. User to decide: fly with one ( firmware correct as is ) or refit the pair and fly the halved reading ( the task 6 analysis then doubles `D` ).
- 2026-09-26 [decision] Task 5 flown with **one R020** ( firmware scale correct as is ) and the **log-1 pack**, full to the app's low-battery warning. Gentle inputs ( INA219 range 8 A with one R020 ). Predicted `D` / charger 0.92-0.97; app remaining at empty ~150-200 mAh from the plug-in estimate alone.
- 2026-09-26 [decision] log-6 ( full pack, one R020, 467 s ): `D` 528, app remaining at empty 22 mAh ( was 297 ), hover `I` 4.15-4.4 A, `G` 0.950 from the start ( `D` = 95% of the 556 mAh pre-gain integral ), one restart in 8 min.
- 2026-09-26 [risk] The low-battery warning ( fused SoC <= 18% ) came at t+453 s of 467, at 3.0 V under load, ~15 s before the end; `D` ended 22 mAh short of `E`, near the H5b wrap. The warning threshold / SoC model needs work in the fix even with the scale right.
- 2026-09-26 [risk] The pack gave ~560 mAh by the INA219 ( which matched the bench supply in log-5 ), not the ~450 inferred from the log-1 charger reading: either the charger reads low or that inference was wrong. The log-6 charger reading decides which reference the fix is accepted against.
- 2026-09-26 [decision] Charger put back **534 mAh** after log-6. With one R020 the firmware count is within 1% ( 528 / 534 ); the pre-gain reading runs ~4% high and the auto-gain's -5% happens to cancel it. The pack is ~534 mAh ( the ~450 estimate came from the halved reading ); log-1's 4.0 V start was ~77% charge, as assumed. Acceptance for the fix: `D` within +/-5% of the charger, with the auto-gain removed.
- 2026-09-26 [decision] Task 6 closed. Log-6 fused SoC runs ~15 points high over the last 30% ( 24% with 10% left, 21% at empty ): the warning's lateness is the SoC model, not the threshold alone.
- 2026-09-26 [decision] Topic closed, superseded by the fix topic [battery-soc-fix](../battery-soc-fix/README.md) ( `/pluto-grill fix`, user ). Findings: TESTING.md *Task 6* verdict table, carried into battery-soc-fix INVESTIGATION §2. Task 8 moved there ( task 10 ); task 9 superseded: the temporary code carries over and is removed in battery-soc-fix task 12, and this topic's docs are committed with that topic ( no separate commit ).
