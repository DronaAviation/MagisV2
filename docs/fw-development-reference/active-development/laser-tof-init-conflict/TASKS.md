# Laser ToF Init Conflict - Tasks

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md)

## Resume

Read this block first in a new session; read only the task it points to.

| | |
|---|---|
| **Current task** | none: topic closed, handed to [tof-single-pipeline](../tof-single-pipeline/TASKS.md) |
| **Next step** | none here; work continues in `tof-single-pipeline` ( `/pluto-task next` ) |
| **Open questions** | none ( XSHUT pin answered in the fix topic: GPIO_5 ) |
| **Blocked on** | nothing |
| **Last updated** | 2026-10-14 |

## Summary

| | |
|---|---|
| **Mode** | check |
| **Goal** | Explain why `LASER_TOF` + `LASER_ALT` with `initObjectAvoidance ( ... )` / `XRanging.init ( )` leaves every laser dead, and recommend one centralised ToF pipeline |
| **Done when** | INVESTIGATION.md holds both pipelines ( init + per-loop, file:line, flowcharts ), a causal chain for the failure, H1-H5 ranked with evidence, and 2-3 single-pipeline designs with one recommended |
| **Target** | PRIMUS_X2_v1 |
| **Analysis** | Ranked causes ( start: H1 address collision + shared `Global_Status` latch > H2 shared result globals > H3 no reset during `ranging_init` > H5 bus load; H4 L1x out of scope ) |
| **Pipeline docs affected** | none in this topic; drift recorded for the fix topic ( `Main_Initialization.md`, `Altitude_Hold_Estimator.md`, `User_Space_API.md`, `Hardware_Bus_Pipeline.md`, `PIN_MAP.md` ) |
| **Order** | 1 → 2 → 3 → 4 → 5 → 6 → 7 |

## Index

| # | Title | Status | Depends on | Skills / agent |
|---|---|---|---|---|
| 1 | Trace pipeline A: altitude laser | done 2026-10-14 | - | graphify |
| 2 | Trace pipeline B: XRanging / object avoidance | done 2026-10-14 | - | graphify |
| 3 | Collision analysis: verify and rank H1-H5 | done 2026-10-14 | 1, 2 | graphify, spec-to-code-compliance |
| 4 | Single-pipeline design options | done 2026-10-14 | 3 | pluto-rules, pluto-driver |
| 5 | Doc drift list | done 2026-10-14 | 1, 2 | spec-to-code-compliance |
| 6 | Findings and recommendation | done 2026-10-14 | 3, 4, 5 | - |
| 7 | Close | done 2026-10-14 | 6 | pluto-commit |

Status values: `todo`, `in-progress`, `done YYYY-MM-DD`, `blocked (<why>)`, `dropped (<why>)`.
Serial numbers are never reused or renumbered.

## Tasks

### 1. Trace pipeline A: altitude laser

**Description.** Follow the down-laser ( `LASER_TOF`, with `LASER_ALT` ) from boot to its consumers. That covers the
defines in `target.h`, `ranging_init ( )` at `main.cpp:518` ( I2C address, mode, timing budget, calibration steps ),
the periodic `getRange ( )` call from `UPDATE_LASER_TOF_TASK` in `mw.cpp`, and the globals it writes. Then trace who
reads those globals ( `altitudehold.cpp` accessors ). Re-verify every scout line number against the source. Write the
"Pipeline A" section of INVESTIGATION.md with a Mermaid flowchart in which every edge is a real call or data path
( `graphify path` or a source line ). Note every file-level global in `ranging_vl53l0x.cpp` that this pipeline owns or
touches. Fusion internals are out of scope: stop at the accessor boundary.

- **Depends on:** -
- **Skills / agent:** graphify ( `graphify path`, `graphify explain` ), then source reads
- **Files:** [main.cpp](../../../../src/main/main.cpp), [ranging_vl53l0x.cpp](../../../../src/main/drivers/ranging_vl53l0x.cpp), [mw.cpp](../../../../src/main/mw.cpp), [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp), `target/PRIMUS_X2_v1/target.h`
- **Safety impact:** none ( analysis only )
- **Done when:** the Pipeline A section lists each step with file:line, the globals table and a flowchart whose edges are all confirmed
- **Rollback:** n/a
- **Status:** done 2026-10-14
- **Result:** Pipeline A traced ( A1-A16, globals table, flowchart ): the down-laser is identified only by 0x29 in the global `MyDevice`, it is fully configured before `xRangingInit ( )`, `Global_Status` is never cleared, and a latched error looks like a dropout ( baro hold after 120 ms ).

### 2. Trace pipeline B: XRanging / object avoidance

**Description.** Follow both user entry points, `initObjectAvoidance ( ... )` and `XRanging.init ( )`. Show how they
record the enabled lasers ( `isXLaserInit[]` ), how `xRangingInit ( )` at `main.cpp:600` drives the XSHUT pins and
assigns 0x2A+ addresses, and what `LaserSensor::init / setAddress / startRanging` configure ( continuous mode, 20 ms
budget ). Trace the per-loop reads from user `getRange ( )` / `isTriggered ( )` and `applyObjectAvoidance ( )`
( `mw.cpp:532`, every 100 ms ). For every write, record whether it goes to a per-instance field or a shared
file-level global. Write the "Pipeline B" section with a flowchart, and a side-by-side table of the boot order with
pipeline A.

- **Depends on:** -
- **Skills / agent:** graphify, then source reads
- **Files:** [XRanging.cpp](../../../../src/main/API-Src/XRanging.cpp), [XRanging.h](../../../../src/main/API/XRanging.h), [ranging_vl53l0x.cpp](../../../../src/main/drivers/ranging_vl53l0x.cpp), [main.cpp](../../../../src/main/main.cpp), [mw.cpp](../../../../src/main/mw.cpp)
- **Safety impact:** none ( analysis only )
- **Done when:** the Pipeline B section covers both entry points, the XSHUT / address sequence, the per-loop readers, the shared vs per-instance write table and a confirmed flowchart
- **Rollback:** n/a
- **Status:** done 2026-10-14
- **Result:** Pipeline B traced ( entry points, B1-B6, read path, write table, boot-order table, flowchart ): `LaserSensor` keeps only its address separate, while its error and results go to the shared `Global_Status` and the 4 result globals altitude hold reads ( H2 real in code ); `setAddress ( )` is sent to 0x29 and its status is discarded; disabled entries' XSHUT is never driven.

### 3. Collision analysis: verify and rank H1-H5

**Description.** Check each hypothesis against the source, step by step:
- **H1:** both chips obeying `setAddress` ( `vl53l0x_api.c` `SetDeviceAddress` ); the I2C timeout and recovery path in `bus_i2c_stm32f30x.c`; where `Global_Status` latches and that nothing clears it; that `startRanging ( )` skips its work on the latch.
- **H2:** the side-laser writes into the shared result globals and whether the estimator can consume them.
- **H3:** the state of the XSHUT pins before `main.cpp:600`.
- **H5:** the bus time per OA cycle.

Check the sequence against ST's multi-sensor XSHUT bring-up procedure with spec-to-code-compliance. Write the causal
chain ( symptom → mechanism → code ) and rank H1-H5, each with evidence for and against and the bench test that would
confirm it ( recorded for the fix topic, not run here ). H4 gets one line only.

- **Depends on:** 1, 2
- **Skills / agent:** graphify, spec-to-code-compliance ( VL53L0X multi-device procedure )
- **Files:** [ranging_vl53l0x.cpp](../../../../src/main/drivers/ranging_vl53l0x.cpp), `lib/main/VL53L0X/` ( read only ), [bus_i2c_stm32f30x.c](../../../../src/main/drivers/bus_i2c_stm32f30x.c), [XRanging.cpp](../../../../src/main/API-Src/XRanging.cpp)
- **Safety impact:** none ( analysis only ). Note any flight risk found, e.g. side-laser data reaching altitude hold ( H2 )
- **Done when:** the "Where they collide" section holds a proven or refuted causal chain for H1 and ranked H2-H5, each with a confirming test
- **Rollback:** n/a
- **Status:** done 2026-10-14
- **Result:** H1 confirmed in code ( deterministic: down-laser + first enabled side laser end at 0x2A, first `getRange ( )` at 0x29 NACKs 8× → `Global_Status` −20 → all 5 stop ); H3 confirmed for LEFT ( SPI2 SCK idles high on PB13 ) and FRONT ( PA15 reset pull-up ); H2 confirmed latent ( stale `NewSensorRange` replayed + side range in object check once H1 is fixed ); H5 ~4-6 ms / 100 ms ( estimate ); F1-F5 and tests T1-T4 recorded.

### 4. Single-pipeline design options

**Description.** Write 2-3 options for centralising ToF bring-up and polling, starting from the scout's sketches:
- **A:** reserve 0x29 for the down-laser, hold every XSHUT low before `ranging_init ( )`, and give per-device status and result.
- **B:** a table-driven ToF manager with one boot sequence and a non-blocking round-robin poll.
- **C:** the down-laser as an XSHUT-controlled table entry once its port exists.

For each option give: how it works, files touched, flash / RAM / loop-time cost, safety impact ( the estimator only
ever sees down-laser data; one device's error never stops another; no blocking re-init in flight ), API impact
( `XRanging.h`, the `docs/API` wiki, `API_Version` ), and risk. Cover how the API should expose the down-laser's
future XSHUT pin ( a target define, a user-code call, or a table entry ). This is the point the user wants to
discuss. Recommend one option. Each option must keep both `initObjectAvoidance ( ... )` and `XRanging.init ( )`
working.

- **Depends on:** 3
- **Skills / agent:** pluto-rules ( invariants the design must hold ), pluto-driver ( init order, I2C sharing, pin ownership )
- **Files:** INVESTIGATION.md "Single-pipeline design options"
- **Safety impact:** none ( design only ). Each option states its flight-safety implications
- **Done when:** 2-3 options are written with costs and risks, the down-laser XSHUT API question has a proposed answer, and one option is recommended
- **Rollback:** n/a
- **Status:** done 2026-10-14
- **Result:** Requirements R1-R8; one bring-up order ( everything at main.cpp:600, side lasers held low, down-laser moved off 0x29 first, warm-reset probe ); option A ( patch in place, leaves H5 ) and option B ( one ToF manager, paced cached reads ) with costs; C folded into B as an optional XSHUT; three ways to declare the down-laser XSHUT. Recommended: B + `target.h` define. The scout's sketch A was dropped ( breaks R1 ).

### 5. Doc drift list

**Description.** List each statement in the pipeline docs that disagrees with the code traced in tasks 1-2, with
file:line on both sides. Known so far: `Main_Initialization.md:38-40` shows the wrong position of `SYSTEM_STATE_READY`
and leaves out the laser inits; `PIN_MAP.md:58-61` and `Altitude_Hold_Estimator.md:9` say "one laser at 0x29"; neither
`User_Space_API.md` nor `docs/API/` has an XRanging page; `Hardware_Bus_Pipeline.md` has no I2C1 device list. No doc
is edited here. The list feeds the fix topic's `PIPELINE_UPDATE.md`.

- **Depends on:** 1, 2
- **Skills / agent:** spec-to-code-compliance
- **Files:** `fw-architecture-pipeline/` docs listed above, `PIN_MAP.md`
- **Safety impact:** none
- **Done when:** INVESTIGATION.md holds a "Doc drift" table: doc:line, what it says, what the code does, code:line
- **Rollback:** n/a
- **Status:** done 2026-10-14
- **Result:** 12 drift rows ( D1-D12: 2 contradicted, 6 partial, 3 absent, 1 stale line refs ) plus the docs that change with `initAlt`; the worst is Main_Initialization ( READY before the laser and PWM inits, no laser step ) and every "one laser at 0x29" statement ignoring XRanging.

### 6. Findings and recommendation

**Description.** Summarise the topic in a "Findings" section at the top of INVESTIGATION.md: the confirmed causal
chain, the ranked causes, the recommended design with its main trade-off, the bench tests the fix topic should run
first, and the open question on the down-laser XSHUT API. Update README.md with the outcome.

- **Depends on:** 3, 4, 5
- **Skills / agent:** -
- **Files:** INVESTIGATION.md, README.md
- **Safety impact:** none
- **Done when:** the findings read on their own, without the code, and the user has chosen a design or asked for changes
- **Rollback:** n/a
- **Status:** done 2026-10-14
- **Result:** The Findings section heads INVESTIGATION.md: H1 causal chain, ranking, F1-F5, D1-D12, the chosen direction ( `XRanging.initAlt`, one bring-up, per-laser state, paced reads ), T1-T4, open hardware questions; README updated. Design chosen by the user.

### 7. Close

**Description.** Either move on to the fix with `/pluto-grill fix laser-tof-init-conflict` ( new or appended fix
tasks ), or record the findings with `/pluto-commit` ( no firmware build ) and set the topic Closed here and in
`active-development/README.md`. No code changed, so no graph refresh is needed.

- **Depends on:** 6
- **Skills / agent:** pluto-commit, or pluto-grill fix
- **Files:** README.md, `active-development/README.md`
- **Safety impact:** none
- **Done when:** the topic is Closed with a commit hash, or handed to a fix topic
- **Rollback:** n/a
- **Status:** done 2026-10-14
- **Result:** Handed to the fix topic [tof-single-pipeline](../tof-single-pipeline/README.md) ( `/pluto-grill fix` ); the findings are committed with that topic.

## Decisions log

Newest last. One line each: `YYYY-MM-DD [decision|assumption|out-of-scope|risk] text`.

- 2026-10-14 [decision] Mode `check`: analyse both init pipelines before choosing a design; the fix follows via `/pluto-grill fix`.
- 2026-10-14 [decision] Target PRIMUS_X2_v1.
- 2026-10-14 [decision] Done when: causal chain + ranked causes + 2-3 single-pipeline designs with one recommended.
- 2026-10-14 [decision] Code analysis only; no bench test in this topic ( confirming tests are recorded for the fix topic ).
- 2026-10-14 [decision] Both user entry points are covered: `initObjectAvoidance ( ... )` and `XRanging.init ( )`.
- 2026-10-14 [assumption] Hardware: 1 down + 4 side VL53L0X on I2C1; the down-laser has no XSHUT today ( user ).
- 2026-10-14 [assumption] Each pipeline works on its own ( user ).
- 2026-10-14 [out-of-scope] VL53L1X / `LASER_TOF_L1x` ( H4: one line only ), estimator fusion, object-avoidance behaviour, code changes.
- 2026-10-14 [out-of-scope] XRanging pin overlaps with SPI2 flash ( PB12-PB14 ), SWDIO ( PA13 ), WS2811 ( PA15 ): known to the user.
- 2026-10-14 [risk] H2: side-laser ranges may reach the altitude estimator through the shared globals; a flight risk the design must rule out.
- 2026-10-14 [risk] Graph built at `84c326d`, HEAD `c495d6a` ( OLED only ): laser code unaffected; line numbers re-verified in tasks 1-3.
- 2026-10-14 [assumption] A down-laser XSHUT port will be added later; how the API exposes it is discussed in task 4.
- 2026-10-14 [risk] Task 2: side-laser reads write `RangingMeasurementData`, `isTofDataNewflag`, `out_of_range` ( shared ): altitude hold can see a side laser's range as the down-laser's; impact checked in task 3.
- 2026-10-14 [risk] Task 1: a latched VL53L0X `Global_Status` is invisible to altitude hold ( `tofOutOfRange ( )` does not include it, unlike the L1X ); it shows only as a dropout, so baro hold.
- 2026-10-14 [decision] Task 3: ST multi-device procedure checked by hand against DS11555 / UM2039 ( no local copy in the repo, so `spec-to-code-compliance` had no spec file to run on ).
- 2026-10-14 [risk] Task 3: H2 becomes active the moment H1 is fixed if the driver state stays shared ( stale height replayed as good, dropout handover suppressed ): the design must split status and results per device.
- 2026-10-14 [risk] Task 3: H3 for LEFT is caused by SPI2 SCK ( PB13 ) idling high from main.cpp:313; LEFT cannot be held in reset while SPI2 owns PB13 ( a pin overlap the user ruled out of scope, but it limits the design ).
- 2026-10-14 [decision] Task 4: the scout's sketch A ( keep the down-laser at 0x29, side lasers after it ) is dropped: every side chip boots at 0x29, so a down-laser without XSHUT must leave 0x29 first.
- 2026-10-14 [assumption] Task 4: recommendation is option B ( one ToF manager ) with the down-laser XSHUT optional and declared in `target.h`; the user chooses in task 6.
- 2026-10-14 [risk] Task 4: a down-laser without XSHUT keeps its moved address across an MCU reset without power loss; the bring-up must probe the assigned address before 0x29.
- 2026-10-14 [decision] Direction ( user ): option B with XRanging as the only entry point; `XRanging.initAlt ( gpio )` / `initAlt ( )` replaces `#define LASER_TOF` + `LASER_ALT`; one bring-up at main.cpp:600. Supersedes the task-4 `target.h` XSHUT recommendation.
- 2026-10-14 [decision] `initAlt` = laser + altitude-hold fusion; no laser-only mode.
- 2026-10-14 [decision] If the down-laser fails to start after `initAlt`, the fusion runs with a dropout ( today's LASER_ALT behaviour ); its error counts as out of range and is readable as a status.
- 2026-10-14 [decision] Down range readable from user code through a new `laser_e` id ( e.g. `DOWN` ); the 5-entry arrays grow to 6.
- 2026-10-14 [decision] `initAlt` takes effect only in `plutoInit ( )`, latched at boot; without it the firmware behaves exactly like today's baro-only build.
- 2026-10-14 [out-of-scope] `LASER_TOF_L1x` stays a define; `initAlt` is refused when it is compiled in; X1Ranging API is the next topic.
- 2026-10-14 [risk] The user's example `initAlt ( GPIO_5 )` uses FRONT's LED pin ( XRanging.cpp:57 ); the API must refuse pins already used by an enabled laser; GPIO_2 suggested.
- 2026-10-14 [decision] Task 5: drift checked by hand on the ~12 in-scope statements ( code side already traced line by line in tasks 1-3 ); the spec-compliance workflow was not run because it fans out over every requirement in these docs, mostly fusion internals outside this topic.
- 2026-10-14 [decision] Closed: the fix is planned as a new topic, `tof-single-pipeline` ( large: new module + public API ). The down-laser XSHUT is GPIO_5, which supersedes D3's refusal of GPIO_5 ( the FRONT LED is no longer on that pin ).
