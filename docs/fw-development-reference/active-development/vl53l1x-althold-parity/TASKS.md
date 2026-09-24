# VL53L1X Altitude Hold Parity - Tasks

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [TESTING](TESTING.md)

## Resume

Read this block first in a new session; read only the task it points to.

| | |
|---|---|
| **Current task** | **Closed - pending commit** ( 24 Sep 2026 ) |
| **Next step** | The user commits with `.git/MAGISV2_COMMIT_MSG.txt`; the next pluto-commit / pluto-grill run fills in the hash |
| **Open questions** | none |
| **Blocked on** | nothing |
| **Last updated** | 2026-09-24 |

## Summary

| | |
|---|---|
| **Mode** | fix |
| **Goal** | The VL53L1X ( `LASER_TOF_L1x` + `LASER_ALT` ) altitude path behaves like the flight-validated VL53L0X path, and hands over to the baro past its reach. |
| **Done when** | Flight logs show: hover ±3 cm for 30 s under 140 cm; 160/140 handover with no `EstAlt` jump; box test with 2.5 s hold-off and re-base; climbing past the sensor's reach hands over to the baro ( `Src` → baro, no pull towards a stale reading ). The L0X build's `altitudehold.o` is byte-identical to the baseline. |
| **Target** | PRIMUS_X2_v1 |
| **Analysis** | Causal chain and findings in [INVESTIGATION.md](INVESTIGATION.md). A stale reading past the reach is proven. The driver freeze ( no `ClearInterruptAndStartMeasurement` ) is suspected and proven in task 2. Refactor: scout option C ( one code path, per-sensor compile-time constants and inline accessors ). |
| **Pipeline docs affected** | `fw-architecture-pipeline/subsystems/Altitude_Hold_Estimator.md` ( Laser fusion ), `PIN_MAP.md`, `dev-guide/HARDWARE_RESOURCES.md`, `dev-guide/FLIGHT_INVARIANTS.md`, root `CLAUDE.md`, `CHANGELOG.md` |
| **Order** | 1 → 2 → 3 → 4 → 14 → 15 → 16 → 17 → 19 → 18 → 5 → 6 → 7 → 8 → 9 → 10 → 11 → 12 → 13 |

## Index

| # | Title | Status | Depends on | Skills / agent |
|---|---|---|---|---|
| 1 | L0X object-code baseline | done 2026-09-23 | - | pluto-build |
| 2 | VL53L1X bring-up and freeze check ( bench ) | done 2026-09-23 | - | pluto-driver, pluto-flighttest |
| 3 | VL53L1X driver fix: handshake, 50 ms cadence, float cleanup | done 2026-09-23 | 2 | pluto-driver, pluto-rules, cpp-pro agent, pluto-reviewer agent |
| 4 | Bench noise and tilt log ( by hand, up to 1.5 m ) | done 2026-09-23 | 3 | pluto-flighttest |
| 5 | Share the fusion helpers across sensors ( L0X byte-identical ) | done 2026-09-24 | 1 | pluto-rules, pluto-build, pluto-reviewer agent |
| 6 | L1x constants and the shared fusion path for the L1x | done 2026-09-24 | 4, 5, 15, 18 | pluto-rules, pluto-build, pluto-reviewer agent |
| 7 | Bench handover and out-of-range log with `LASER_ALT` | done 2026-09-24 | 6 | pluto-flighttest |
| 8 | Build gate | done 2026-09-24 | 6 | pluto-build |
| 9 | Hardware validation ( flights ) | done 2026-09-24 | 7, 8, 14 | pluto-flighttest, pluto-log-analyst agent |
| 10 | Remove diagnostics and restore `target.h` | done 2026-09-24 | 9 | pluto-rules, pluto-build |
| 11 | Architecture & pipeline docs ( PIPELINE_UPDATE.md ) | done 2026-09-24 | 10 | graphify, spec-to-code-compliance |
| 12 | Topic review | done 2026-09-24 | 11 | pluto-reviewer agent |
| 13 | Graph refresh & commit | done 2026-09-24 | 12 | graphify, pluto-commit |
| 14 | Cut the VL53L1X sample-poll I2C cost | done 2026-09-23 | 3 | pluto-driver, pluto-rules, pluto-reviewer agent |
| 15 | Reach flight on baro hold ( 1.5-2.5 m, laser logged only ) | done 2026-09-23 | 4, 14 | pluto-flighttest |
| 16 | Long distance mode build ( A/B switch ) | done 2026-09-24 | 15 | pluto-driver, pluto-rules, pluto-build, pluto-reviewer agent |
| 17 | Long mode bench: noise, tilt, poll cost | done 2026-09-24 | 16 | pluto-flighttest |
| 18 | Long mode reach flight and mode decision | done 2026-09-24 | 17, 19 | pluto-flighttest |
| 19 | Long mode at 10 Hz ( 95 ms / 100 ms ) bench | done 2026-09-24 | 17 | pluto-driver, pluto-rules, pluto-build, pluto-flighttest, pluto-reviewer agent |

Status values: `todo`, `in-progress`, `done YYYY-MM-DD`, `blocked (<why>)`, `dropped (<why>)`.
Serial numbers are never reused or renumbered.

## Tasks

### 1. L0X object-code baseline

**Description.** The L0X board can no longer be flown, so the only proof that the refactor leaves the
flight-validated VL53L0X path unchanged is its object code. Before any source change, build
PRIMUS_X2_v1 with `LASER_TOF` + `LASER_ALT` on ( temporary edit to `target.h` ). Save
`objdump -d` of `altitudehold.o` and the `memory` output into the topic folder as
`baseline/altitudehold-L0X.dis`. Record the exact command, so the same build can be diffed in tasks 5,
6 and 8. Also record the L1x build's flash/RAM at `b1f070b` and a no-laser build's `altitudehold.o`
disassembly as a second baseline. Restore `target.h` afterwards.

- **Depends on:** -
- **Skills / agent:** pluto-build
- **Files:** [target.h](../../../../src/main/target/PRIMUS_X2_v1/target.h), [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp)
- **Safety impact:** none ( no flash )
- **Done when:** `baseline/altitudehold-L0X.dis` and `baseline/altitudehold-nolaser.dis` exist. Building the L0X configuration a second time reproduces the first disassembly exactly, which rules out timestamps or paths in the object file.
- **Rollback:** n/a
- **Status:** done 2026-09-23
- **Result:** Baselines in `baseline/` ( L0X `35812d92…`, no-laser `a8775323…`, L1X `50af04b3…` ); the L0X dump was identical across 3 clean builds; `target.h` restored. Reusable script `baseline/snapshot.sh`; details in [TESTING.md](TESTING.md).

### 2. VL53L1X bring-up and freeze check ( bench )

**Description.** Flash a `LASER_TOF_L1x` build ( **without** `LASER_ALT`, so the estimator is untouched )
and confirm the sensor starts on I2C1 at 0x29 and that the other I2C1 sensors still initialise. Add a
temporary `Monitor_Print` line in `PlutoPilot.cpp` of at most 115 B: raw mm, `Range_Status_L1`,
`StreamCount`, `Global_Status_L1`, `isOutofRange_L1()`, and the time between new samples. Move the
board by hand from about 5 cm to beyond its reach and back. This proves or kills hypothesis H1 ( the
driver freezes without `VL53L1_ClearInterruptAndStartMeasurement()` ). Results go in `TESTING.md` as
log-1.

- **Depends on:** -
- **Skills / agent:** pluto-driver ( I2C bus sharing, init order ), pluto-flighttest ( log fields, log-N.txt handoff )
- **Files:** [ranging_vl53l1x.cpp](../../../../src/main/drivers/ranging_vl53l1x.cpp), [PlutoPilot.cpp](../../../../PlutoPilot.cpp), [target.h](../../../../src/main/target/PRIMUS_X2_v1/target.h)
- **Safety impact:** none ( bench, disarmed, `LASER_ALT` off )
- **Done when:** the log settles H1. Either `StreamCount` freezes or `Global_Status_L1` goes non-zero ( H1 real ), or the distance tracks the hand for 60 s with `StreamCount` advancing ( H1 dead ). The measured sample interval is recorded.
- **Rollback:** reflash `b1f070b` / defines off
- **Status:** done 2026-09-23
- **Result:** log-1: H1 ( freeze ) killed, with 178 s of fresh ranges and `G` 0. Found instead: without the handshake the driver re-reads and flags "new" every 10 ms; `StreamCount` only toggles 0/1, so the sample interval was **not** measurable ( moved to task 3 ); out of range is flagged, but `NewSensorRange_L1` holds the last value; covered window reads `St` 0 at 0-5 mm; possible signal fail from ~1.9 m. Details in TESTING.md Test 1.

### 3. VL53L1X driver fix: handshake, 50 ms cadence, float cleanup

**Description.** In `ranging_vl53l1x.cpp`, call `VL53L1_ClearInterruptAndStartMeasurement()` after
every result read. Task 2 showed no freeze, but without it data-ready never clears, so each 10 ms poll
re-reads the result and sets `isTofDataNewflag_L1`. The goal is exactly one "new" sample per
measurement. Add a driver-side count of data-ready events ( and its timestamp ) for the log-2
interval measurement, because `StreamCount` only toggles 0/1 in this preset. Set the
timing budget to 45 ms ( changed from 33 ms after review, see the decisions log ) and the inter-measurement period to 50 ms, keeping Medium mode ( decision
2026-09-23 ). Make `isOutofRange_L1()` also cover a status error and a stale sample, so the estimator
can treat "no fresh valid sample" the same way as out of range. Replace the double leftovers
( `LASER_LPS 0.75`, unsuffixed literals ) with float. Fix the call sequence in `drivers/` only, never
in `lib/`. Do not rely on the `LaserSensor_L1` constructor, because static constructors never run.
Delegate to `cpp-pro` with this description and the task 2 result. Re-run the task 2 bench log as log-2.

- **Depends on:** 2
- **Skills / agent:** pluto-driver, pluto-rules, cpp-pro agent, pluto-reviewer agent
- **Files:** [ranging_vl53l1x.cpp](../../../../src/main/drivers/ranging_vl53l1x.cpp), its header
- **Safety impact:** none yet ( `LASER_ALT` still off ). The driver becomes the source for the estimator in task 6
- **Done when:** log-2 shows a sample interval of 50 ± 5 ms ( from the driver's data-ready count, not `StreamCount` ), one read per measurement ( no 10 ms re-reads ), ranges fresh for 60 s, `Global_Status_L1` at 0, and out-of-range flagged within one period of going past the reach. The build is clean, and the reviewer reports no BLOCKING findings.
- **Rollback:** revert the driver file
- **Status:** done 2026-09-23
- **Result:** log-2: one sample per measurement ( `N` max 3, mostly 2 ), interval 52.7 ms, fresh throughout ( `A` ≤ 96 ms ), `G` 0 for 230 s, out of range flagged on every `St` 2 sample. Gate clean, L0X identical, reviewer no BLOCKING. Found: the sample poll blocks 5.44 ms ( new task 14 ).

### 4. Bench reach and cadence log

**Description.** ( Narrowed 2026-09-23: the user cannot hold the craft above 1.5 m by hand, so the
reach above 1.5 m moved to task 15, a baro-hold flight. ) On the same indoor matt floor as the L0X
flights, hold the craft level at measured heights up to 1.5 m. Log raw mm, status and
out-of-range. Measure the noise at 50, 100 and 150 cm ( sd over 10 s ) and the height where valid
samples start dropping. This confirms that 160/140 cm sits well inside the reach, or sets new band
edges that the user signs off. Also check the tilt response: tilt the board 30° and confirm the reading
lengthens as `1/cos`. Record as log-3 in `TESTING.md`.

- **Depends on:** 3
- **Skills / agent:** pluto-flighttest
- **Files:** [PlutoPilot.cpp](../../../../PlutoPilot.cpp) ( temporary log )
- **Safety impact:** none ( bench )
- **Done when:** the noise at 50, 100 and 150 cm ( sd over 10 s ) and the tilt response are recorded, and the laser matches the tape within 2 cm at each height. The band is confirmed in task 15.
- **Rollback:** n/a
- **Status:** done 2026-09-23
- **Result:** log-3: sd 4-6 mm hand-held at ~50/100/150 cm, `St` 0 throughout; tilt to 19.6° follows `h / cos` within 1.7 %; >25° not tested ( estimator check in task 7 ). Accuracy against the tape moved to task 15 ( no tape values ). Near-field bias on the ground noted for task 6.

### 5. Share the fusion helpers across sensors ( L0X byte-identical )

**Description.** Scout option C, part 1. Move the ring buffer, `tofWindowMismatch` and
`altShiftFrame` ( `altitudehold.cpp:905-970` ) from `#ifdef LASER_TOF` to `LASER_ALT`. Replace the
direct driver reads in the L0X fusion body ( `:988-1205` ) with inline accessors ( `tofNew()`,
`tofRawCm()`, `tofFiltCm()`, `tofOutOfRange()`, `tofReseed()` ) and sensor constants selected with
`#if defined(LASER_TOF) / #elif defined(LASER_TOF_L1x)`. Under `LASER_TOF` every accessor and
constant must expand to exactly today's expression, keeping the same types, the same literals and the
same order of operations. Add `#error` if both sensors are defined. The L1x branch stays as it is in
this task.

- **Depends on:** 1
- **Skills / agent:** pluto-rules, pluto-build, pluto-reviewer agent
- **Files:** [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp)
- **Safety impact:** ALT_HOLD with the L0X. The object-code proof below makes it zero
- **Done when:** the L0X build's `altitudehold.o` disassembly diffs empty against `baseline/altitudehold-L0X.dis`, and the no-laser build diffs empty against its baseline. The L1x build compiles. Defining both sensors stops the build with the `#error`.
- **Rollback:** revert `altitudehold.cpp`
- **Status:** done 2026-09-24
- **Result:** Accessor macros ( `tofNew / tofClearNew / tofOutOfRange / tofReseed / tofFiltCm / tofRawCm` ) and per-sensor constants; helpers moved to `LASER_ALT`; `#error` for both sensors and for `LASER_ALT` without a laser. L0X, no-laser **and** L1x + ALT `altitudehold.o` identical ( checked twice ); L0X + ALT and L1x + ALT warning sets unchanged ( 32 / 33 ); gate PASS. Reviewer: no BLOCKING; the L1x dropout ( 3P + 25 ), offset clamp ( 1.5 P ) and count asserts were fixed; 2 design points moved into task 6.

### 6. L1x constants and the shared fusion path for the L1x

**Description.** Scout option C, part 2. Delete the old L1x branch ( `altitudehold.cpp:1208-1235`:
the 350 cm hard switch, the radian tilt test, the one-sample `baro_offset` ) and run the shared fusion
body for `LASER_TOF_L1x` as well. The L1x constants are derived from `ALT_TOF_L1X_PERIOD_MS` ( 50 )
as sample counts: dropout 3×T+10 = 160 ms, steady 500/T = 10, ring ≥ 660/T ( 14 ), offset dt clamp
= T. `ALT_TOF_IIR_LAG_S` is 0 ( no IIR ), reseed is a no-op, and the band comes from task 4. The
sensor-independent constants ( 25° tilt, 30 cm step, 2.5 s hold-off, and the others in SCOUT.md ) are
shared. Add an L1x minimum-valid range: log-1 showed a covered window reporting `St` 0 at 0-5 mm, so a blocked sensor must count as invalid, not as ground. The on-ground reading is also biased ( laser about 2.6-2.8 cm up, below the sensor's ~4 cm minimum; log-3 and the user ), so the laser must not give the height reference while landed. Check how the L0X path uses the on-ground reading ( landing re-base, take-off ) and set the L1x minimum-valid range above the ground reading ( for example 40 mm ). Widen **both** `#ifdef LASER_TOF` guards ( `tofTiltOk` ~:1023 and the fusion body ~:1034 ) to include `LASER_TOF_L1x`, and set the L1x `ALT_TOF_HANDOVER_UP/DOWN_CM` ( 160 / 140, decision 2026-09-23 ). **Minimum-valid range goes in the validity test, not in the `ToF_Height > 0` check** ( reviewer ): put it in the driver's status-0 branch or in the L1x `tofOutOfRange ( )`, so that `newGoodSample`, `tofUsable`, the dropout and the return all see it. Then check the knock-on: a landed craft ( ~27 mm ) drops to the baro, so the last ~4 cm of a landing fly on the baro; touchdown detection must still work ( landing re-base ~:1135 needs `tofUsable` ). **Guard the baro → laser return** ( reviewer: every suspect / step path requires `altSourceLaser`, so the return itself has no plausibility check ): count a return sample only if `|tofNowCm − ( Baro_Height − baro_offset )|` is within a bound ( about `ALT_TOF_STEP_CM` plus the baro noise ), L1x only. Check that the baro → laser return ( below 140 cm for the return count ) cannot be taken on a reading far from the current estimate: log-5 had 4 consecutive valid 56 cm readings of ceiling-fan blades while the craft was at 2.5 m. The L0X step / window logic may already cover it; if not, add a guard for the L1x only. After the edit, re-run the task 5 diff: the L0X object code must still be identical.

- **Depends on:** 4, 5
- **Skills / agent:** pluto-rules, pluto-build, pluto-reviewer agent
- **Files:** [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp)
- **Safety impact:** ALT_HOLD, landing and take-off on L1x builds. Keep the landing re-base during hold-off ( `:1088-1096` ) and never re-zero the baro datum
- **Done when:** the L1x build compiles clean ( `--gate` ). The L0X and no-laser `altitudehold.o` still diff empty. The reviewer reports no BLOCKING findings. Every L1x constant appears in `CHANGES.md` with its derivation.
- **Rollback:** revert `altitudehold.cpp` ( and `target.h` defines off, which is baro-only )
- **Status:** done 2026-09-24
- **Result:** The L1x runs the shared L0X body ( both guards widened, old branch deleted ); band 160 / 140; 15 mm minimum in the driver's validity test; L1x return guard ( 50 cm agreement ) with a steady-disagreement exit ( 10 cm for 2.5 s ); dropout 185 ms. L0X and no-laser identical; L1x + ALT 111.0 KB / 16.1 KB, no new warnings; gate PASS; review and re-review: no BLOCKING.

### 7. Bench handover and out-of-range log with `LASER_ALT`

**Description.** Flash the L1x + `LASER_ALT` build with the tof-althold-fusion flight log line
( `EstAlt`, `AltHold`, `Vz`, `ToF`, `Src`, `Arm`, ≤ 115 B ). Disarmed, raise the craft through 160 cm
and lower it through 140 cm, and check that `Src` switches at the band edges with no `EstAlt` step.
Then cover the sensor or carry it past its reach, and check that `Src` drops to the baro within the
160 ms dropout. Last, slide a box under it and check the hold-off. Record as log-4 in `TESTING.md`
( compare with tof-althold-fusion log-4 ).

- **Depends on:** 6
- **Skills / agent:** pluto-flighttest
- **Files:** [PlutoPilot.cpp](../../../../PlutoPilot.cpp) ( temporary log )
- **Safety impact:** none ( bench, disarmed )
- **Done when:** the switch points are within ±5 cm of 160/140, `EstAlt` jumps less than 3 cm at a switch, the dropout reaches the baro in ≤ 200 ms, and the hold-off is visible for the box.
- **Rollback:** n/a
- **Status:** done 2026-09-24
- **Result:** log-9 ( disarmed ): laser → baro at 1605 / 1609 mm with `E` continuous; baro → laser at 1389 / 1392 mm; return frame shift as designed ( large on the bench only because the datum tracks while disarmed ); dropout ~200 ms and return 150 ms seen at pick-up. Covered and >25° tilt not performed; closed by the user, tilt to be seen in task 9.

### 8. Build gate

**Description.** Run `.claude/skills/pluto-build/driver.sh --gate PRIMUS_X2_v1` on the L1x +
`LASER_ALT` configuration and on the shipped configuration ( defines off ). Record flash/RAM against
the task 1 numbers. Repeat the L0X and no-laser object-code diffs one last time.

- **Depends on:** 6
- **Skills / agent:** pluto-build
- **Files:** -
- **Safety impact:** none
- **Done when:** no new `src/` warnings in either configuration, flash and RAM fit, and both object-code diffs are empty.
- **Rollback:** n/a
- **Status:** done 2026-09-24
- **Result:** Gate PASS on L1x + ALT ( 111.0 KB / 16.1 KB ), no-laser ( 98.9 / 14.8, unchanged ) and L0X + ALT ( 101.7 / 15.5, unchanged ); L0X and no-laser `altitudehold.o` identical; the 32 `altitudehold.cpp` warnings are pre-existing ( present in no-laser too ).

### 9. Hardware validation ( flights )

**Description.** ( Added 2026-09-24: the object hold-off and the L1x return guard with its 2.5 s steady exit run only when airborne, so they are checked here, not on the task 7 bench. Guard: on the baro above 160 cm, hold a board ~1 m under the craft ( more than 50 cm from the estimate ), check it is held off and returns after ~2.5 s if held steady.  Also check: the tilt rejection above 25° ( not reached on the bench ), and the take-off from the floor, where 0-12 mm near-field readings may cause a ~0.3 s baro leg with a small frame shift) Fly the L1x + `LASER_ALT` build on PRIMUS_X2_v1 over the matt floor, with the
flight plan from `pluto-flighttest` modelled on tof-althold-fusion log-3/5/7. Log-5: a 30 s hover at
about 100 cm. Log-6: climb through 160 cm and descend through 140 cm. Log-7: box test, sliding a box in
and out under the hover. Log-8: climb past the laser reach ( or cover the sensor in flight ) and check
that the baro takes over with no pull. Large logs go to `pluto-log-analyst`. Results go in
`TESTING.md`.

- **Depends on:** 7, 8
- **Skills / agent:** pluto-flighttest, pluto-log-analyst agent
- **Files:** [PlutoPilot.cpp](../../../../PlutoPilot.cpp) ( temporary log )
- **Safety impact:** ALT_HOLD, handover and landing in flight. Rollback build: defines off ( baro-only ). Fly the reach test with a spotter and throttle ready
- **Done when:** hover ±3 cm for 30 s; handover with less than 3 cm `EstAlt` step and no visible bump; box hold-off and re-base, then a return to the old clearance; beyond reach, `Src` goes to the baro and the craft holds height ( no climb or sink towards a stale value ); touchdown detected normally on every landing.
- **Rollback:** reflash with the laser defines off
- **Status:** done 2026-09-24
- **Result:** Flights log-10..13: hover ±3 cm on 92 % ( p-p 8 cm ), handover 160 → baro with no jump, returns shift `E` and `H` together ( 7-27 cm; 119 / 169 cm after a board-frame baro leg ), 9 box hold-offs with re-base, past-reach baro hold ±5 cm up to 3.1 m, normal touchdowns. The return guard held and exited in log-12; the exit band was widened to 25 cm ( not flight-tested: the board was out of the beam in log-13 ). Tilt above 25° not reached.

### 10. Remove diagnostics and restore `target.h`

**Description.** Restore `PlutoPilot.cpp` to HEAD, removing the temporary log lines. Remove the TEMPORARY poll-cost instrumentation in the driver ( `maxPollUs_L1` and the two `micros ( )` calls in `ranging_vl53l1x.cpp`, the extern in `ranging_vl53l1x.h` ). Set the `target.h`
laser defines to the shipping choice, assumed off as after tof-althold-fusion ( confirm with the user ).
Re-run the gate on the shipped configuration.

- **Depends on:** 9
- **Skills / agent:** pluto-rules, pluto-build
- **Files:** [PlutoPilot.cpp](../../../../PlutoPilot.cpp), [target.h](../../../../src/main/target/PRIMUS_X2_v1/target.h)
- **Safety impact:** none
- **Done when:** `git diff PlutoPilot.cpp` is empty, `target.h` matches the user's choice, and the gate passes.
- **Rollback:** n/a
- **Status:** done 2026-09-24
- **Result:** `PlutoPilot.cpp` and `target.h` at HEAD ( laser defines off, user ); poll timer removed from the driver. Shipped 98.9 KB / 14.8 KB ( = HEAD ), gate PASS, L0X / no-laser identical, L1x + ALT 110.1 KB / 16.1 KB; reviewer: nothing temporary left.

### 11. Architecture & pipeline docs ( PIPELINE_UPDATE.md )

**Description.** Stage the new text in `PIPELINE_UPDATE.md`; never edit the pipeline folder directly.
`Altitude_Hold_Estimator.md` needs: the L1x driver in Source files; the Laser fusion section applying
to both sensors, with a per-sensor constants table and a Mermaid flow where it changed ( every edge a
real call path, checked with `graphify path` ); and the ToF vs Baro note ( scout: `:129`, `:292` ).
`PIN_MAP.md` and `HARDWARE_RESOURCES.md` need a note that there is one down-laser at 0x29 on I2C1,
that L0X and L1x are exclusive, and about the `#error`. `CLAUDE.md` gets the `LASER_ALT` paragraph
rewritten ( drop "the L1X has none of this" ), then that paragraph and the flip paragraph move into
`dev-guide/FLIGHT_INVARIANTS.md`, leaving a one-line rule for each in `CLAUDE.md`. CHANGELOG entry
text. No `docs/API` change. Check the staged Laser fusion text against the code with
`spec-to-code-compliance`.

- **Depends on:** 10
- **Skills / agent:** graphify, spec-to-code-compliance
- **Files:** `PIPELINE_UPDATE.md`, [CLAUDE.md](../../../../CLAUDE.md), [FLIGHT_INVARIANTS.md](../../dev-guide/FLIGHT_INVARIANTS.md), [PIN_MAP.md](../../PIN_MAP.md), [HARDWARE_RESOURCES.md](../../dev-guide/HARDWARE_RESOURCES.md)
- **Safety impact:** none
- **Done when:** PIPELINE_UPDATE.md is complete, the compliance check shows no contradictions, and the CLAUDE.md move is done with one-line rules left.
- **Rollback:** n/a
- **Status:** done 2026-09-24
- **Result:** PIPELINE_UPDATE.md ( Source files, `checkReading ( )`, the whole Laser fusion section with a new Mermaid flow, the ToF-vs-Baro bullet, CHANGELOG draft ); FLIGHT_INVARIANTS.md gains the flip and laser sections, and CLAUDE.md keeps one line each; PIN_MAP and HARDWARE_RESOURCES notes. Single-agent compliance pass ( user, instead of the multi-agent workflow ): 196 / ~202 claims matched; the 4 divergences and 5 omissions were fixed.

### 12. Topic review

**Description.** `pluto-reviewer` on the whole topic diff against `b1f070b`. Fix BLOCKING findings and
re-run the task 8 gate and object-code diffs after any code fix.

- **Depends on:** 11
- **Skills / agent:** pluto-reviewer agent
- **Files:** topic diff
- **Safety impact:** as in the fixes
- **Done when:** no open BLOCKING findings; gate and diffs re-passed after fixes.
- **Rollback:** n/a
- **Status:** done 2026-09-24
- **Result:** Whole-topic review against `b1f070b`: **no BLOCKING, verdict ready to commit**. SHOULD: the warnings-baseline refresh must come from a clean gate build ( written into task 13 ). NITs fixed: README nav / date / task count, PIPELINE_UPDATE wording, "superseded" markers in CHANGES.md.

### 13. Graph refresh & commit

**Description.** `graphify update .` and `python tools/graph_labels.py`, then `/pluto-commit`. **Warnings baseline** ( task 12 review ): refresh it only from a **clean** `driver.sh --gate PRIMUS_X2_v1` build whose "fixed" list is exactly the 12 `-Wreorder` entries of `ranging_vl53l1x.h`; never from an incremental `build.log` ( it listed 101 "fixed", 89 of them missing compile units ). Say so in the commit message. The commit
skill does the all-target build, the version bump ( FW minor; API unchanged ), promotion of
PIPELINE_UPDATE.md and CHANGELOG, and sets the topic Closed. Flight logs stay out of the commit.

- **Depends on:** 12
- **Skills / agent:** graphify, pluto-commit
- **Files:** -
- **Safety impact:** none
- **Done when:** staged, with the drafted message pasted in chat; the user commits.
- **Rollback:** n/a
- **Status:** done 2026-09-24
- **Result:** Graph refreshed ( 4000 nodes / 7522 edges, labels ). pluto-commit: all three targets built ( 98.9 / 99.4 / 98.8 KB ); warnings baseline refreshed from a clean gate ( exactly the 12 `-Wreorder` fixed ); FW 3.10.0 ( API 1.3.2 ); PIPELINE_UPDATE.md promoted and the CHANGELOG merged; staged with a drafted message. The user commits.

### 14. Cut the VL53L1X sample-poll I2C cost

**Description.** log-2 measured the VL53L1X sample poll ( `getRange_L1 ( )`: data-ready + full
results read + `ClearInterruptAndStartMeasurement` ) at 5.44 ms of blocking I2C at 400 kHz, once per
53 ms. That is longer than the 3.5 ms control loop, so one loop in about 15 is stretched to about 9 ms.
First measure the VL53L0X-equivalent cost from its code ( for reference only: the board is gone ),
then cut the L1x poll. Candidates, to be weighed against the ST full-API state machine:
( a ) split the read and the clear across two consecutive 10 ms polls;
( b ) read only the range and status registers and clear the interrupt directly, ULD-style, with the
ST state kept consistent;
( c ) anything the ST API offers to shrink the results read.
Keep one sample per measurement and every task 3 check. Re-run the log-2 bench sequence as log-4 ( `T` field ).

- **Depends on:** 3
- **Skills / agent:** pluto-driver, pluto-rules, pluto-reviewer agent ( cpp-pro if delegated )
- **Files:** [ranging_vl53l1x.cpp](../../../../src/main/drivers/ranging_vl53l1x.cpp)
- **Safety impact:** loop timing in every mode, on L1x builds only. The L0X and no-laser builds are not touched
- **Done when:** worst poll `T` ≤ 2000 µs ( or a lower bound the user accepts, with the reason ), and log-3b still passes the task 3 checks ( interval 50 ± 5 ms, `N` ≤ 3, `A` ≤ 100, `G` 0, out of range flagged ). Gate clean, L0X identical, reviewer no BLOCKING.
- **Rollback:** revert to the task 3 driver
- **Status:** done 2026-09-23
- **Result:** log-4: worst sample poll 784-789 µs ( was 5440 ), interval 51.2 ms, `N` 2-3, `A` ≤ 64 ms, `G` 0, out of range flagged. A 17-byte result read + interrupt clear with a local decode; flash -2.1 KB; gate clean, L0X identical, reviewer no BLOCKING ( limit values corrected to 90 mm / 1.5 MCPS ).

### 15. Reach flight on baro hold ( 1.5-2.5 m, laser logged only )

**Description.** The VL53L1X reach above 1.5 m can only be measured in flight. Fly the task 14 build
with `LASER_TOF_L1x` on and **`LASER_ALT` off**: altitude hold is baro-only, as shipped, and the
laser is only logged, so a bad reading cannot affect control. `BaroAlt` freezes its datum at arm and
then serves as the second height reference. Climb slowly in ALT_HOLD from 1 m to about 2.5 m in
steps, hold about 10 s at each, and descend the same way. The log adds `Arm`. Result: the height where
`St` 0 samples start dropping out ( climbing and descending ), and the laser-minus-baro agreement below
it. This sets or confirms the 160/140 handover band with margin, which task 6 needs.

- **Depends on:** 4, 14
- **Skills / agent:** pluto-flighttest
- **Files:** [PlutoPilot.cpp](../../../../PlutoPilot.cpp) ( temporary log )
- **Safety impact:** a normal baro ALT_HOLD flight on the shipped control path. The laser is not in the loop. Indoor ceiling above 3 m needed, with a spotter
- **Done when:** the valid-reach height is recorded ( climb and descent, `St` 0 fraction per 25 cm band ), and the handover band is confirmed as 160/140 with at least 50 cm of margin, or new values are signed off by the user
- **Rollback:** n/a ( no control change )
- **Status:** done 2026-09-23
- **Result:** log-5: 100 % valid up to 180 cm, 90 % at 180-190, 57 % at 190-200, ~0 above 210 ( 50 % point ~1.95 m ). Band kept at 160/140 ( user ), with 20 cm to the first dropouts. Laser − baro estimate +22 cm, constant ( baro take-off offset ). The fan blades seen from above at 2 m give a false-return risk for task 6.

### 16. Long distance mode build ( A/B switch )

**Description.** Added 2026-09-23 at the user's request, before task 5. Make the VL53L1X distance
mode a compile-time choice in `ranging_vl53l1x.cpp`: `L1X_DISTANCE_MODE`, defaulting to
`VL53L1_DISTANCEMODE_MEDIUM`, so a build can switch between Medium and Long with one line and the other
setting stays as it is. Build the Long variant with the same 45 ms budget / 50 ms period ( user: a clean
A/B of the mode alone, so every later constant stays valid ). Check in `lib/main/VL53L1X_API/` that
the Long preset accepts a 45 ms budget and a 50 ms period, and that the task 14 short read path
( 17-byte result block, interrupt clear, local status decode ) is equally valid in Long mode: the same
register block, and no per-range software step that Long adds. Keep the temporary log of log-5.

- **Depends on:** 15
- **Skills / agent:** pluto-driver, pluto-rules, pluto-build, pluto-reviewer agent
- **Files:** [ranging_vl53l1x.cpp](../../../../src/main/drivers/ranging_vl53l1x.cpp)
- **Safety impact:** none until flown. The laser stays out of the loop ( `LASER_ALT` off ) for tasks 17-18
- **Done when:** the Long build passes the gate, the L0X object code is identical, and the ST constraints for Long at 45/50 are cited ( file:line ). Reviewer: no BLOCKING
- **Rollback:** `L1X_DISTANCE_MODE` back to Medium
- **Status:** done 2026-09-24
- **Result:** `L1X_DISTANCE_MODE` ( default Medium ), with Long set in `target.h` for tasks 17-18. ST accepts Long at 45 / 50 ( 9.2 ms per phase, IMP ≥ MTB + 4 ); the short read path and decode are independent of the mode. Gate PASS, L0X identical, disassembly passes mode 3. Reviewer: 2 NITs, fixed.

### 17. Long mode bench: noise, tilt, poll cost

**Description.** Repeat the log-3 / log-4 bench on the Long build: floor, the ~57 cm box, 100 cm and
150 cm by hand, a tilt at ~100 cm, and the sensor pointed far away. The log is `mm St N O T Tl E Ar`, shared with task 18 so the same image flies without a reflash.
Compare with Medium: noise ( sd ) at each height, the interval, the worst poll `T`, and the floor
reading ( near-field behaviour ). Long mode is expected to be noisier and more sensitive to ambient
light; this measures by how much. Result in `TESTING.md` as log-6.

- **Depends on:** 16
- **Skills / agent:** pluto-flighttest
- **Files:** [PlutoPilot.cpp](../../../../PlutoPilot.cpp) ( temporary log )
- **Safety impact:** none ( bench, disarmed )
- **Done when:** Long mode sd at 50/100/150 cm, interval, `T` and floor reading are tabled against Medium ( log-3 / log-4 )
- **Rollback:** n/a
- **Status:** done 2026-09-24
- **Result:** log-6: Long at 45 / 50 is worse than Medium below 1.8 m: 83-95 % valid level ( 5-17 % `St` 7 wrap-target fail ), **1 % valid at a 20° tilt** ( Medium 100 % ), sd up to 2× ( 8.2 against 4.4 mm at 100 cm ), floor reading 16.5 mm ( Medium 28 mm ). Poll cost and interval unchanged.

### 18. Long mode reach flight and mode decision

**Description.** Updated 2026-09-24 after task 19. Fly the log-5 reach flight on the **Long 95 ms / 100 ms**
build ( the task 19 image, no reflash ): baro ALT_HOLD, `LASER_ALT` off, the laser logged only, climbing in
steps to about 2.5 m and back down. **Stay well clear of the ceiling fan**: log-5 flew above its
blades. Compare the valid fraction per 10 cm band with Medium ( log-5: 100 % to 180 cm, 50 % at
~195 cm ), and the laser-minus-baro offset. Then **decide the mode and timing for this topic** ( user ):
Medium 45 / 50 ( 20 Hz, reach ~1.9 m ) or Long 95 / 100 ( 10 Hz, reach from this flight ). Long at
45 / 50 is out ( log-6 ). Weigh the extra reach against half the sample rate: at 10 Hz task 6 uses a
310 ms dropout, and the 0.5 s step window holds 5 samples. The band stays 160 / 140; 340 / 300 remains a
follow-up. Log the choice and set the driver defaults to it.

- **Depends on:** 17, 19
- **Skills / agent:** pluto-flighttest
- **Files:** [PlutoPilot.cpp](../../../../PlutoPilot.cpp) ( temporary log ), [ranging_vl53l1x.cpp](../../../../src/main/drivers/ranging_vl53l1x.cpp) ( the chosen mode )
- **Safety impact:** a normal baro ALT_HOLD flight; the laser is not in the loop. Ceiling clearance, and keep away from the fan
- **Done when:** the Long valid fraction per band ( climb and descent ) is tabled against log-5, and the mode is chosen and logged with its reason
- **Rollback:** Medium
- **Status:** done 2026-09-24
- **Result:** log-8 ( climb only; the paste was cut at 61 s ): Long 95 / 100 is 100 % valid to 160 cm and 0 / 188 above 200 cm, the same reach as Medium ( ~1.8-2.0 m, `St` 2 signal fail ). **Chosen: Medium 45 / 50** ( user ); the `target.h` Long overrides were removed ( disassembly: mode 2, period 50; gate PASS; L0X identical ). The overridable mode / budget / period stay in the driver.

### 19. Long mode at 10 Hz ( 95 ms / 100 ms ) bench

**Description.** Added 2026-09-24 ( user ). log-6 showed that Long mode at 45 ms / 50 ms rejects 5-17 %
of level samples and about 99 % at a 20° tilt as `St` 7 ( wrap-target fail ). The likely cause is the
short integration, 9.2 ms per phase. Try Long with a 95 ms budget and a 100 ms period ( ST: period ≥
budget + 4 ms ), giving about 34 ms per phase. The budget and the period become overridable like the
mode: `#ifndef` defaults in the driver ( budget ) and the header ( `L1X_SAMPLE_PERIOD_MS` ), with the
temporary overrides in `target.h`. The header change must stay a pure macro, with the L0X
`altitudehold.o` identical. `L1X_STALE_MS` follows the period ( 310 ms at 100 ms ). Then repeat the
log-6 bench as log-7, with the same steps plus a box step, and the tilt steps at about 10°, 20° and 25°.
**If Long at 10 Hz is chosen**, task 6 has to derive its constants from a 100 ms period ( dropout,
window counts ): half the L0X's sample rate. Log that as a risk at the decision.

- **Depends on:** 17
- **Skills / agent:** pluto-driver, pluto-rules, pluto-build, pluto-flighttest, pluto-reviewer agent
- **Files:** [ranging_vl53l1x.cpp](../../../../src/main/drivers/ranging_vl53l1x.cpp), [ranging_vl53l1x.h](../../../../src/main/drivers/ranging_vl53l1x.h), [target.h](../../../../src/main/target/PRIMUS_X2_v1/target.h), [PlutoPilot.cpp](../../../../PlutoPilot.cpp) ( temporary log )
- **Safety impact:** none ( bench, `LASER_ALT` off )
- **Done when:** the gate is clean and the L0X object code identical; log-7 tables the valid fraction ( level and 10° / 20° / 25° tilt ), sd, interval ( ~100 ms, `N` ~1 per tick ), `T` and the floor reading against Medium ( log-3 ) and Long at 50 ms ( log-6 )
- **Rollback:** remove the `target.h` overrides ( back to Medium 45 / 50 )
- **Status:** done 2026-09-24
- **Result:** log-7: Long 95 / 100 is 100 % valid level and at 20° ( 96 % at 35° ), sd 3.2 mm at 100 cm and 5.2 mm at 150 cm, floor 25.6 mm sd 1.2: as clean as Medium 45 / 50 or cleaner, at 10 Hz. The log-6 `St` 7 wraps came from the 9.2 ms per phase. Gate clean, L0X / no-laser identical; reviewer no BLOCKING ( 2 guards added ).

## Decisions log

Newest last. One line each: `YYYY-MM-DD [decision|assumption|out-of-scope|risk] text`.

- 2026-09-23 [decision] Mode fix, planned in place in the existing Planned topic; target PRIMUS_X2_v1 with `LASER_TOF_L1x` + `LASER_ALT`.
- 2026-09-23 [decision] Done when: the same flight bar as the L0X ( hover, handover, box ) plus an out-of-range handover to the baro.
- 2026-09-23 [decision] Scope: refactor + parity + docs, including the CLAUDE.md → FLIGHT_INVARIANTS.md move.
- 2026-09-23 [decision] Refactor is scout option C: one fusion code path, per-sensor compile-time constants and inline accessors, `#error` if both sensors are defined.
- 2026-09-23 [decision] L0X path contract: `altitudehold.o` byte-identical for the L0X build ( `objdump -d` diff ), because the L0X board can no longer be flown.
- 2026-09-23 [decision] The L0X keeps its literal sample-count constants. L1x counts are derived from its period in macros, which reconciles "constants in time" with byte-identical L0X code.
- 2026-09-23 [decision] L1x cadence: 33 ms timing budget, 50 ms inter-measurement period, Medium mode kept.
- 2026-09-23 [decision] Plan starts with bring-up: the VL53L1X has never been powered with this firmware.
- 2026-09-23 [assumption] Wiring is drop-in on the L0X connector ( I2C1, 0x29, PB8/PB9 ): no PIN_MAP pin change, notes only.
- 2026-09-23 [assumption] Same indoor matt floor as the L0X flights. The 160/140 band is expected to hold and is confirmed in task 4.
- 2026-09-23 [assumption] Rollback build is the laser defines off ( baro-only ), as shipped today. `target.h` ships with them off again unless the user says otherwise ( task 10 ).
- 2026-09-23 [risk] ~~The L1x driver has no bench or flight history~~ Corrected by the user: on the old code, before the L0X work, the VL53L1X gave data and held altitude, with the same bob and the same hard reaction to objects as the L0X. The shared fixes from `484226c` ( accelerometer Z deadband 0, one tau ) already reach the L1x build; the object reaction needs the handover and hold-off logic ( task 6 ).
- 2026-09-23 [risk] Traffic on the shared I2C1 bus rises at 20 Hz L1x ranging; watch loop time and the other I2C1 sensors in task 2.
- 2026-09-23 [out-of-scope] Removing the unused XVision / `LaserSensor_L1` class ( 964 B RAM in every build ): follow-up improve topic.
- 2026-09-23 [out-of-scope] Extending the L1x band to 340 cm up / 300 cm down ( user's later plan ). It needs Long mode ( Medium is rated about 2.9 m ) and its own bench reach log: follow-up topic.
- 2026-09-23 [out-of-scope] `opticflow.cpp:355` scales by the L0X `NewSensorRange` and reads 0 on L1x builds ( `OPTIC_FLOW` is off ).
- 2026-09-23 [decision] L0X equivalence is checked on the debug-stripped `objdump -d -r -s` of `altitudehold.o` ( `baseline/snapshot.sh` ), never on the `.hex`, because the Makefile bakes `__BUILD_DATE__` / `__BUILD_TIME__` into the image.
- 2026-09-23 [decision] The task 2 bench build fails the warning gate with 10 pre-existing warnings in `ranging_vl53l1x.cpp/.h`. They appear only because `LASER_TOF_L1x` is now compiled, and none come from the diagnostic code. Accepted for the bench build; task 3 cleans them ( a clean gate is part of its done-when ).
- 2026-09-23 [decision] Temporary diagnostics: `PlutoPilot.cpp` bench log and `LASER_TOF_L1x` in PRIMUS_X2_v1 `target.h`. Both are removed / restored in task 10.
- 2026-09-23 [decision] Task 2 reviewer ( no BLOCKING ): added the read-event field `R` so that "never read" can be told apart from "frozen"; `lastSc` is seeded from the first sample; 0xFF-filled rows after a failed read are ignored; the sample interval is taken as elapsed time / sum of `d` ( `A` is quantised to the 100 ms tick ). The log is now about 97 B per tick ( 109 worst case ).
- 2026-09-23 [decision] H1 ( driver freeze ) killed by log-1. The handshake stays in task 3 for a different reason: data-ready never clears, so the driver re-reads and flags "new" every 10 ms.
- 2026-09-23 [risk] A covered or blocked VL53L1X reports `St` 0 at 0-5 mm. Task 6 needs a minimum-valid range, or a blocked sensor reads as ground.
- 2026-09-23 [risk] Signal fail seen intermittently at ~1.9 m. If confirmed in task 4, the 160/140 band has less margin than assumed.
- 2026-09-23 [risk] The old L1x branch was fed a "new" sample on every 10 ms re-read ( log-1 ), so its laser correction ran at about 100 Hz on duplicates. With one sample per 50 ms ( task 3 ) the effective laser correction strength changes. Check `correctedWithTof` for a per-sample versus per-time gain in task 6.
- 2026-09-23 [decision] L1x timing budget is 45 ms, not the planned 33 ms ( user, after review ). In the full ST API's AUTONOMOUS preset, 33000 µs leaves 3.2 ms per ranging phase against the default's 7.2 ms; 45000 gives 9.2 ms and is the largest the 50 ms period allows ( ≥ budget + 4 ms ).
- 2026-09-23 [decision] `isOutofRange_L1 ( )` also reports a stall: no new result for 160 ms ( 3 × period + 10, the task 6 dropout ).
- 2026-09-23 [risk] A VL53L1X sample poll is about 4.8 ms of blocking I2C ( reviewer estimate ), longer than a 3.5 ms loop, once per 50 ms. Measured in log-2 ( `T` ). If it is confirmed, splitting the read and the clear over two polls is a candidate follow-up.
- 2026-09-23 [decision] Do not fly `LASER_ALT` on the intermediate builds before task 6: the old L1x fusion test still ignores `isOutofRange_L1 ( )` ( `altitudehold.cpp:1219` ).
- 2026-09-23 [decision] Task 3 reviewer: no BLOCKING. Applied the budget change, the staleness check, the `T` poll timing and the header HISTORY row. Deferred to task 6: the new flag is also set on invalid samples, so `tofNew ( )` must pair it with out-of-range.
- 2026-09-23 [decision] Task 3 closed on log-2. The measured interval is about 53 ms ( the ST oscillator calibration ); the L1x counts stay derived from `L1X_SAMPLE_PERIOD_MS` 50 ( within 6 % ).
- 2026-09-23 [decision] New task 14 ( user ): cut the 5.44 ms VL53L1X sample poll before the flights. Placed after task 4 in the Order; task 9 now depends on it.
- 2026-09-23 [assumption] log-1's opening 28 mm was the craft sitting on the floor ( log-2 reads 25-32 mm there ).
- 2026-09-23 [decision] Task 4 split ( user ): the hand bench stays in task 4 ( up to 1.5 m ); the reach above 1.5 m becomes task 15, a baro-hold flight with the laser logged only, after task 14 so that it flies the final driver. Task 6 now depends on 15.
- 2026-09-23 [decision] Task 4 closed without the tape accuracy check ( user ). Accuracy is checked against the baro in the task 15 flight.
- 2026-09-23 [risk] The VL53L1X is biased on the ground ( user; laser about 2.6-2.8 cm up, below its ~4 cm minimum ). Task 6: the laser must not give the height reference while landed; the minimum-valid range goes above the ground reading.
- 2026-09-23 [decision] Task 14 approach ( b ) ( cpp-pro, from its analysis ): read the 17-byte result block ( 0x0089-0x0099 ) and clear only the interrupt; decode the status and range locally with the API's own mapping. The TIMED preset does not need the per-range GENERAL_ONWARDS rewrite; the sigma / signal limits run on the device. Flash -2.1 KB. The LL driver's state is left stale on purpose ( only checked back-to-back ).
- 2026-09-23 [decision] Task 14 reviewer: no BLOCKING. Correction: the VL53L1X limit checks are the Medium preset tuning defaults, **sigma 90 mm / signal 1.5 MCPS**. The preset overwrites DataInit's 18 mm / 0.25 MCPS ( SCOUT.md's figure ), and it always did, so validity is unchanged. The comments are corrected; the pipeline doc must state 90 mm / 1.5 MCPS. Kept as they are ( no behaviour change ).
- 2026-09-23 [decision] Task 14 closed on log-4. A covered window reading `St` 0 at a few mm is the sensor's own behaviour ( the API path does the same ); it is rejected by the task 6 minimum-valid range.
- 2026-09-23 [decision] Handover band stays 160 / 140 cm ( user ). The VL53L1X reach on the flying floor is 100 % valid to 180 cm, with the 50 % point at ~195 cm ( log-5 ); the 50 cm margin in the task 15 done-when was not met and is waived. L0X precedent: ~12 cm.
- 2026-09-23 [risk] False baro → laser return on an object seen from above ( log-5: the fan blades read 56 cm, valid, 4 samples in a row, with the craft at 2.5 m ). Task 6 must guard the return.
- 2026-09-23 [assumption] The in-flight baro estimate reads about 20 cm below the laser, constant from 40 to 140 cm ( a baro take-off offset ). It is absorbed by the frozen handover offset; not addressed in this topic.
- 2026-09-23 [decision] Long mode tested before task 5 ( user; added as tasks 16-18, not a new topic ). Same 45 ms / 50 ms timing as Medium, so the only variable is the mode. The result picks this topic's mode; the band stays 160 / 140; 340 / 300 remains a follow-up. Task 6 now depends on 18.
- 2026-09-24 [decision] Long mode at 45 / 50 fails the "as clean as Medium below 1.8 m" criterion on the bench ( log-6 ). The user's possible extra tilt ( logged 20.8° against Medium's 19.6° ) does not explain it: 5-17 % `St` 7 also occurs level. User: try Long at 10 Hz ( 95 ms / 100 ms ) as new task 19 before task 18.
- 2026-09-24 [risk] If Long at 10 Hz is chosen, the L1x runs at half the L0X's sample rate, so the task 6 counts and the dropout ( 310 ms ) are derived from 100 ms, and the object step detector sees 5 samples per 0.5 s.
- 2026-09-24 [decision] Task 19 reviewer: no BLOCKING. ST accepts Long at 95 / 100 ( 34.2 ms per phase, start check 100 ≥ 98 ). Applied: `ranging_vl53l1x.h` includes `platform.h`, so an `L1X_SAMPLE_PERIOD_MS` override is always seen; `static_assert` period ≥ budget + 5 ms. Noted for task 6: at 100 ms the dropout is 310 ms.
- 2026-09-24 [decision] Long at 45 / 50 is ruled out ( log-6 ). Long at 95 / 100 passes the bench criterion ( log-7 ). Task 18 flies the 95 / 100 image and chooses between Medium 45 / 50 and Long 95 / 100.
- 2026-09-24 [decision] Mode for this topic: **Medium, 45 ms / 50 ms** ( user ). Long 95 / 100 has no extra reach ( log-8: both modes stop at ~1.8-2.0 m ) and runs at half the rate. The `target.h` Long overrides were removed; `L1X_DISTANCE_MODE` / `L1X_TIMING_BUDGET_US` / `L1X_SAMPLE_PERIOD_MS` stay overridable, with defaults Medium / 45000 / 50.
- 2026-09-24 [out-of-scope] Reach beyond ~2 m ( for 340 / 300 ): likely capped by the preset's 1.5 MCPS minimum signal rate ( DataInit's 0.25 MCPS is overwritten ). Follow-up topic: re-apply a lower limit after `SetDistanceMode`, then bench and reach flight ( user ).
- 2026-09-24 [decision] Task 5 reviewer: no BLOCKING. The L1x provisional constants were fixed in task 5: dropout 3P + 25 ( 175 ms, tolerates 2 missed samples plus poll / tick jitter, ≥ the driver's stale time ), offset clamp 1.5 P, and `static_assert`s on the steady count and the ring span. L0X / no-laser / L1x objects are still identical.
- 2026-09-24 [out-of-scope] Pre-existing, L0X only: `LaserSensor::startRanging` ( XRanging side lasers ) sets `isTofDataNewflag = true` ( `ranging_vl53l0x.cpp:378` ), so in an L0X `LASER_ALT` build with side lasers the down-laser sample is consumed twice. The L1x is not affected. Follow-up.
- 2026-09-24 [decision] User target for the L1x: hand over at **270 cm up / 240 cm down**. It is not reachable with the current setup ( 0 % valid above ~2.1 m in both modes ); it needs a valid reach of ~2.9-3.0 m. Order ( user ): finish this topic on Medium 20 Hz at 160 / 140 and commit, then a **new topic** for the reach ( lower the 1.5 MCPS minimum-signal limit, probably Long 95 / 100, reach flights to ~3 m in a higher hall the user has access to ) and the 270 / 240 band. The old 340 / 300 idea is replaced by 270 / 240.
- 2026-09-24 [decision] L1x minimum-valid range: **15 mm** ( user: do not change the landing logic ). It rejects a covered window ( 0-10 mm, `St` 0 ) and keeps the landed reading ( 25-28 mm ) valid, so landing, touchdown and the landing re-base keep the L0X behaviour. This supersedes the '~40 mm' example in the task 6 text.
- 2026-09-24 [decision] Task 6 return guard ( reviewer: no exit, so a >50 cm frame disagreement locks the laser out for the flight ). User: **accept a steady disagreement**, i.e. return candidates steady within `ALT_TOF_STEADY_CM` for `ALT_TOF_STEP_HOLD_MS` ( 2.5 s ) are accepted with the normal frame shift. L1x only.
- 2026-09-24 [decision] Task 6 re-review: the steady test now measures the disagreement ( laser − estimate ), not the raw reading, so the exit works while the craft moves, including during a landing.
- 2026-09-24 [decision] Task 7 bench is disarmed only ( handover, frame shift, dropout, return ). The object hold-off and the return guard need airborne state ( armed, not in ground idle ), so they move to the task 9 flights, not to a props-off bench with the motors running.
- 2026-09-24 [decision] Task 7 review: no BLOCKING. The log is now gated on `LASER_TOF_L1x && LASER_ALT` ( `altHoldSource` exists only with `LASER_ALT` ), and `V` comes from `Estimate_Get ( Velocity, Z )`. The other `LASER_ALT` effects ( absolute 120 cm take-off, accelerometer Z deadband 0, tau 1.5 s ) are recorded in TESTING.md Test 9 for task 9.
- 2026-09-24 [decision] Task 7 closed without the covered-at-1.2 m and >25° tilt steps ( user ). The dropout / return was exercised by near-floor readings; tilt rejection is shared L0X code and is checked in the task 9 logs.
- 2026-09-24 [decision] Flights A / B / C ( log-10..12 ) pass the hover ( ±3 cm 92 %, p-p 8 cm ), handover, return shift, object hold-off, past-reach and touchdown checks. Tilt above 25° was not reached and stays unexercised ( user ).
- 2026-09-24 [decision] Guard exit band widened to **25 cm** ( `ALT_TOF_RETURN_STEADY_CM`, L1x only, user ): in log-12 the baro-path estimate swung 20-40 cm while descending and the 10 cm band delayed the exit to ~10 s. Re-fly as Test 13 ( log-13 ).
- 2026-09-24 [risk] Known limit, inherited from the L0X: a re-base onto a close object followed by the clearance climb can leave the craft above 160 cm in the object's frame when the object is removed; the baro leg then carries a ~1 m frame error until the guard exit or the return corrects it ( log-12 ).
- 2026-09-24 [decision] Task 9 closed on log-10..13 ( user ). The 25 cm guard-exit band is not flight-tested ( board out of beam in log-13 ); accepted on the log-12 behaviour plus review.
- 2026-09-24 [decision] Shipping `target.h`: laser defines off ( user ), as after tof-althold-fusion. The shipped build is byte-identical to HEAD in size ( 98.9 KB / 14.8 KB ) and in `altitudehold.o`.
- 2026-09-24 [decision] Task 11 compliance check was one read-only agent rather than the multi-agent spec-compliance workflow ( user ).
- 2026-09-24 [out-of-scope] A latched VL53L1X error ( any I2C failure at runtime ) is never recovered: laser out, baro hold until a power cycle. Documented in PIPELINE_UPDATE.md; recovery is a follow-up topic ( user ).
- 2026-09-24 [decision] Dropped from the follow-ups ( user ): the XVision / `LaserSensor_L1` removal and the L0X side-laser double-consume. Both stay recorded above as found, not planned.
