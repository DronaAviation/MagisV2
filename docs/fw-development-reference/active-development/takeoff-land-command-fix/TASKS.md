# Take-off / land command fix - Tasks

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md)

## Resume

Read this block first in a new session; read only the task it points to.

| | |
|---|---|
| **Current task** | **Closed - pending commit** ( 16 Oct 2026, FW 3.14.0, API 1.7.0 ). All tasks done. |
| **Next step** | The user commits ( `git commit -F .git/MAGISV2_COMMIT_MSG.txt` ). The next `pluto-commit` / `pluto-grill` run fills in the hash |
| **Open questions** | none |
| **Blocked on** | nothing |
| **Last updated** | 2026-10-16 |

## Summary

| | |
|---|---|
| **Mode** | fix |
| **Goal** | `Command_Arm` + `Command_TakeOff` climbs; take-off / flip cannot break a landing; the default `Command_Land` cannot confirm touchdown at the start |
| **Done when** | The task-6 flights and flight G pass on PRIMUS_X2_v1; gate clean; flash growth < 500 B ( raised from 300 to 400 B after task 11, and to 500 B after tasks 12-15 ) |
| **Target** | PRIMUS_X2_v1 ( all targets at commit ) |
| **Analysis** | Causal chains in [INVESTIGATION.md](INVESTIGATION.md): reported fault flight-proven; D1-D3 from the code ( check topic [takeoff-api-ground-hover](../takeoff-api-ground-hover/README.md) ) |
| **Pipeline docs affected** | `subsystems/User_Space_API.md`, `subsystems/Power_BMS_Pipeline.md` ( landing ), new `docs/API/FC_CONTROL_API_WIKI.md`, CLAUDE.md one-liner |
| **Order** | 1 → 2 → 3 → 4 → 11 → 5 → 6 → 12 → 13 → 15 → 14 → 7 → 8 → 9 → 10 |

## Index

| # | Title | Status | Depends on | Skills / agent |
|---|---|---|---|---|
| 1 | D1: take-off / flip refuse while landing | done 2026-10-16 | - | pluto-rules, pluto-build |
| 2 | Take-off goal on an already-armed drone ( 3 s window or ground idle ) | done 2026-10-16 | 1 | pluto-rules, pluto-build |
| 3 | D2: touchdown confirmed only after 1 s of ramp | done 2026-10-16 | - | pluto-rules, pluto-build |
| 4 | Validation test versions and log line | done 2026-10-16 | 1, 2, 3 | pluto-flighttest, pluto-rules |
| 5 | Build gate | done 2026-10-16 | 1, 2, 3, 4, 11 | pluto-build |
| 6 | Hardware validation ( five flights ) | done 2026-10-16 | 5 | pluto-flighttest, pluto-log-analyst agent |
| 7 | API docs and version | done 2026-10-16 | 6, 14 | spec-to-code-compliance |
| 8 | Architecture & pipeline docs ( PIPELINE_UPDATE.md ) | done 2026-10-16 | 7 | graphify |
| 9 | Topic review | done 2026-10-16 | 8 | pluto-reviewer agent |
| 10 | Graph refresh & commit | done 2026-10-16 | 9 | graphify, pluto-commit |
| 11 | A queued take-off cannot arm the drone later ( 2 s refused-arm drop, Dev Mode off drop ) | done 2026-10-16 | 2 | pluto-rules, pluto-build |
| 12 | `Command_Land` refused during a flip | done 2026-10-16 | 1 | pluto-rules, pluto-build |
| 13 | `Command_TakeOff` arms only with the stick centred | done 2026-10-16 | 11 | pluto-rules, pluto-build |
| 14 | Gate and validation flight for tasks 12-13 | done 2026-10-16 | 12, 13, 15 | pluto-build, pluto-flighttest |
| 15 | Flip watchdog: a flip longer than 8 s ends | done 2026-10-16 | 12 | pluto-rules, pluto-build |

Status values: `todo`, `in-progress`, `done YYYY-MM-DD`, `blocked (<why>)`, `dropped (<why>)`.
Serial numbers are never reused or renumbered.

## Tasks

### 1. D1: take-off / flip refuse while landing

**Description.** In `Command_TakeOff` and `Command_Flip` ( `API-Src/FC-Control-Command.cpp:124-155` ), return without
changing `current_command` while a landing is in progress: `isLanding`, or `current_command == LAND`, which covers the
cycle before `land ( )` first sets `isLanding`. This is the same rule `MSP_SET_COMMAND` already follows
( `serial_msp.cpp:1569` ). It protects the user `Command_Land` and the RC-loss failsafe landing after the link returns.
The battery auto-land is already protected ( `mw.cpp:1250-1275` ). Decision: refuse silently.

- **Depends on:** -
- **Skills / agent:** pluto-rules, pluto-build ( PRIMUS_X2_v1 )
- **Files:** [FC-Control-Command.cpp](../../../../src/main/API-Src/FC-Control-Command.cpp)
- **Safety impact:** landing ( user and failsafe ), flip. Removes a stuck-landing state; no new path can start a motor
- **Done when:** builds with no new warnings; review confirms that no path still replaces LAND from user code
  ( `Command_TakeOff`, `Command_Flip`; check the other `current_command =` writers too )
- **Rollback:** revert the two guards
- **Status:** done 2026-10-16
- **Result:** `landingInProgress ( )` guard in `Command_TakeOff` / `Command_Flip` ( CHANGES.md ). Builds clean, no new warnings; flash 107.8 KB. Review: 0 blocking, 0 should-fix, 3 notes; no lock-out after a landing or a disarm.

### 2. Take-off goal on an already-armed drone ( 3 s window or ground idle )

**Description.** In the armed branch of `takeOff ( )` ( `command/command.cpp:97-103` ): if `! isTakeOffHeightSet`, and
either the drone armed less than 3 s ago ( `arm_time`, `mw.cpp:622` ) or it is at ground idle
( `isThrottleStickArmed` ), then release the idle ( `isThrottleStickArmed = false` ), set the goal and set
`isTakeOffHeightSet`. Then finish as today. Move the goal code ( laser `setDesiredPosition`, baro
`DesiredPosition_setRelative`, `command.cpp:116-123` ) into one static helper used by both branches. Otherwise the
call is a no-op in the air, as today. Name the 3 s window as a constant.

- **Depends on:** 1 ( the take-off no longer runs during a landing, so the guard here does not have to cover it )
- **Skills / agent:** pluto-rules, pluto-build
- **Files:** [command.cpp](../../../../src/main/command/command.cpp); declare `arm_time` extern if needed
- **Safety impact:** arming / take-off. Releasing the idle starts the motors from about 1500 instead of 1000, the same
  lift-off as log-3 ( no hop ). It acts only when user code asks for a take-off
- **Done when:** builds with no new warnings; the review confirms the window math is unsigned and wrap-safe, and that
  `isTakeOffHeightSet` resets at disarm ( `mw.cpp:559` ), giving once per arm
- **Rollback:** revert to the old armed branch ( FINISHED only )
- **Status:** done 2026-10-16
- **Result:** Armed branch sets the goal within 3 s of arming or at ground idle, once per arm and only with the stick centred. `altHoldReleaseGroundIdle ( )` clears both idle flags; the disarmed branch no longer sets a goal on a refused arm ( CHANGES.md ). Build clean, text 109284 B. Review: 1 should-fix ( stick low spent the goal ) fixed and re-checked CLOSED; 0 blocking.

### 3. D2: touchdown confirmed only after 1 s of ramp

**Description.** In `land ( )` ( `command/command.cpp:212-225` ), make `conclusive` also require
`elapsed >= LAND_CONFIRM_MIN_MS` ( new constant, 1000 ms ). Before that, a slow descent cannot confirm touchdown. The
impact check ( `LAND_IMPACT_ACC` ) and the 30 s timeout keep their current behaviour. This closes the default
`Command_Land ( 105 )` case ( start at 1200 ) without changing what `landSpeed` means. The failsafe and battery
landings start at 1300 and reach 1200 only after 2.5 s, so they are not delayed at all. Only `landSpeed` 65 or more
( start at 1240 or below ) changes ( task 3 review ).

- **Depends on:** -
- **Skills / agent:** pluto-rules, pluto-build
- **Files:** [command.cpp](../../../../src/main/command/command.cpp)
- **Safety impact:** landing disarm. With `landSpeed` 65 or more, a soft touchdown in the first second is confirmed
  at 1.3 s at the earliest instead of 0.3 s. The impact check and the timeout are not gated
- **Done when:** builds with no new warnings; the review confirms that every landing type still disarms on the floor
  ( settle, impact, timeout )
- **Rollback:** remove the `elapsed` term
- **Status:** done 2026-10-16
- **Result:** `LAND_CONFIRM_MIN_MS` 1000: `conclusive` also needs `elapsed >= 1000`, and the header comment is corrected ( CHANGES.md ). +4 B, gate clean. Review: 0 blocking, 0 should-fix; closes D2 for every `landSpeed`; failsafe and battery landings unaffected.

### 4. Validation test versions and log line

**Description.** Reuse the check topic's test versions A and B ( `../takeoff-api-ground-hover/test/` ) and their log
line. Add three new versions:

- **D:** `Command_TakeOff ( 100 )`; at 8 s armed `Command_Land ( 5 )`; at 9 s re-send `Command_TakeOff ( 100 )`. It
  must be refused, and the landing must complete.
- **E:** `Command_TakeOff ( 100 )` once in `onLoopStart ( )` **without** the disarmed guard, for an ARM-switch arm
  with the stick low ( D3 ).
- **F:** a default `Command_Land ( )` at 12 s ( D2 ).
- **F2** ( optional ): a default `Command_Land ( )` about 1 s after `Command_TakeOff`, during the climb. Here the
  margin before the 1.3 s confirm window is estimated at only 0.3-0.4 s ( task 3 review note 1 ).

Write the procedures and the expected `F` / `C` / `Zd` / `Ze` per flight into `TESTING.md`. Keep the log line within
130 B.

- **Depends on:** 1, 2, 3
- **Skills / agent:** pluto-flighttest, pluto-rules ( Monitor_Print budget )
- **Files:** `test/` ( this topic ), `TESTING.md`, [PlutoPilot.cpp](../../../../PlutoPilot.cpp)
- **Safety impact:** none in firmware. E arms with the stick low and climbs as soon as Dev Mode comes on
- **Done when:** all five versions compile on PRIMUS_X2_v1; the expected results are written
- **Rollback:** restore the template `PlutoPilot.cpp`
- **Status:** done 2026-10-16
- **Result:** One file `test/PlutoPilot_fix_validation.cpp.txt` with `TEST_CASE` 1-6 ( A, B, D, E, F, F2 ); all compile, 0 warnings; case 1 in the root `PlutoPilot.cpp`. TESTING.md §1 has the procedures, Pass / Fail and the invalid-run rules. Review: 1 blocking ( a re-send after the landing queued a take-off for the next ARM-on ) fixed in the sketch, with the firmware side moved to task 11; 3 should-fix and 5 notes fixed; re-check closed.

### 5. Build gate

**Description.** Run `.claude/skills/pluto-build/driver.sh --gate PRIMUS_X2_v1` on the firmware with tasks 1-3. Record
flash and RAM against the 3.13.0 baseline ( 107.8 KB / 17.6 KB ).

- **Depends on:** 1, 2, 3, 4
- **Skills / agent:** pluto-build
- **Files:** -
- **Safety impact:** none
- **Done when:** no new `src/` warnings; flash growth under 400 B ( was 300 B, raised by the user after task 11 ); RAM unchanged or within a few bytes
- **Rollback:** -
- **Status:** done 2026-10-16
- **Result:** Clean gated build, PRIMUS_X2_v1, template `PlutoPilot.cpp`, no new `src/` warnings. Against 3.13.0 ( `HEAD`, built in a temporary worktree ): text 107820 → 108148 ( **+328 B** ), data 1280 → 1280, bss 16756 → 16760 ( +4 B ). Flash 106.8 KB / 256 KB ( 41% ), RAM 17.6 KB / 40 KB ( 44% ).

### 6. Hardware validation ( five flights )

**Description.** Fly on PRIMUS_X2_v1, ALT HOLD on, one log each ( `log-1.txt` … `log-5.txt` in this folder ):

1. A ( Arm + TakeOff ) must climb to about 100 cm: `F` ends in 1 and `Zd` ramps.
2. B ( TakeOff alone ) must match check-topic log-2.
3. D: the take-off re-sent during the landing is ignored ( `C` stays `21` ), and the drone lands and disarms.
4. E: an ARM-switch arm with the stick low, then Dev Mode. It must leave idle and climb ( third digit of `F` goes from
   1 to 0, `Zd` ramps ).
5. F: the default `Command_Land ( )` from hover disarms on the floor ( `Ze` near the floor at the disarm ), not in the
   air.
6. F2 ( optional ): a default `Command_Land ( )` during the climb, about 1 s after the take-off. It must disarm on
   the floor.

- **Depends on:** 5
- **Skills / agent:** pluto-flighttest; pluto-log-analyst agent for large logs
- **Files:** `log-1.txt` … `log-5.txt`, `TESTING.md`
- **Safety impact:** flight tests. Keep a hand on the ARM switch ( kill switch )
- **Done when:** all five pass with the numbers in `TESTING.md`
- **Rollback:** reflash FW 3.13.0
- **Status:** done 2026-10-16
- **Result:** All six pass ( TESTING.md §2 ): A climbs ( fault fixed ); B unchanged; D re-send ignored, `C` stays 21; E leaves the idle and climbs ( `S:9368` ); F and F2 default land disarm at the floor ( 1.73 s / 1.52 s after `C:21` ). No Wi-Fi restarts happened, so the repeat and drop paths rest on the review.

### 7. API docs and version

**Description.** Update the `FC-Control.h` comments for `Command_TakeOff`:

- height in cm, 100-250;
- arms the drone if it is disarmed and the ARM switch is on;
- when already armed, climbs only within 3 s of arming or at ground idle;
- ignored during a landing.

Do the same for `Command_Land` ( speed meaning, 1305 − speed ) and `Command_Flip` ( ignored during a landing ). Write
a new `docs/API/FC_CONTROL_API_WIKI.md` with the same layout as the existing wikis: arm / take-off / land / flip
patterns and the `onLoopStart ( )` restart caveat. Also the task 11 rules for a queued take-off: it waits while ARM
is off; with ARM on and arming refused it is dropped after 2 s ( app take-offs too ); a user-code take-off is dropped
at a real Dev Mode switch-off. Advise sending it once per boot or once per arm. Tasks 12-13: `Command_Land` is ignored during a flip ( header done, wiki
to do ); a disarmed take-off arms only with the throttle centred, waits otherwise, and is still dropped 2 s after a
refusal ( `OK_TO_ARM` off ) whatever the stick. Update `FC-Control.h:97-100` and the wiki's take-off bullets and drop table. Task 15: a flip now ends after at most 8 s ( wiki,
`Command_Flip` section ). Also state that the stick must be centred ( 1460-1540 ) when `Command_TakeOff`
runs. Otherwise the take-off is a no-op, and user code must call it again ( task 2 review N1 ). Bump `API_Version` 1.6.0 → 1.7.0 in the Makefile; `pluto-commit`
bumps `FW_Version`. Check the wiki against the code with `spec-to-code-compliance`.

- **Depends on:** 6
- **Skills / agent:** spec-to-code-compliance
- **Files:** [FC-Control.h](../../../../src/main/API/FC-Control.h), `docs/API/FC_CONTROL_API_WIKI.md`, [Makefile](../../../../Makefile)
- **Safety impact:** none
- **Done when:** the wiki is written and the compliance check has no open divergence; `API_Version` is 1.7.0
- **Status:** done 2026-10-16
- **Result:** `FC-Control.h` comments; new `docs/API/FC_CONTROL_API_WIKI.md`; `API_Version` 1.7.0. Compliance workflow ( 8 requirements ): 7 hold, 1 partial reworded. A doc-vs-code check confirmed ( a ) to ( e ) and found 8 more, all fixed in the docs. The biggest: with the battery OK arming is allowed right after a landing, so a re-sent take-off re-arms and climbs. Documented ( user ).

### 8. Architecture & pipeline docs ( PIPELINE_UPDATE.md )

**Description.** Stage new text for `subsystems/User_Space_API.md` in this topic's `PIPELINE_UPDATE.md`: take-off
preconditions, landing refusal, and removal of `Control_Command_t`, `Command.set` and `Estimate.get`, which do not
exist ( `:15,20-21` ). Stage the landing confirmation timing for `subsystems/Power_BMS_Pipeline.md`. Confirm every
diagram edge with `graphify path` or a source line. Draft a CLAUDE.md flight-invariants one-liner: user commands
cannot replace a landing. Reword the `batteryCriticalAutoLand ( )` header comment ( `mw.cpp:1252` ): the re-assert is now a backstop ( task 1 review ). The `altitudehold.cpp:413-415` comment describes only the default landing start 1300; `Command_Land` starts at 1305-1050 and is clamped to −100 cm/s ( task 3 review ). Never edit `fw-architecture-pipeline/` directly.

- **Depends on:** 7
- **Skills / agent:** graphify
- **Files:** `PIPELINE_UPDATE.md`, [CLAUDE.md](../../../../CLAUDE.md)
- **Safety impact:** none
- **Done when:** `PIPELINE_UPDATE.md` covers both docs; the CLAUDE.md line is drafted
- **Status:** done 2026-10-16
- **Result:** `PIPELINE_UPDATE.md`: parts A-E ( User_Space_API, Power_BMS, Altitude_Hold, CLAUDE.md / FLIGHT_INVARIANTS lines, drift list ); flowchart node `dropUserTakeOff ( )` confirmed at `mw.cpp:1224-1227`. Two code comments corrected ( `mw.cpp:1252-1265`, `altitudehold.cpp:412-421` ). Gate clean.

### 9. Topic review

**Description.** `pluto-reviewer` on the whole topic diff ( `src/main/API-Src/FC-Control-Command.cpp`,
`src/main/command/command.cpp`, `FC-Control.h`, the Makefile, the docs ). Fix the BLOCKING findings and re-run the
gate.

- **Depends on:** 8
- **Skills / agent:** pluto-reviewer agent
- **Files:** topic diff
- **Safety impact:** as tasks 1-3
- **Done when:** no open BLOCKING findings; the gate is re-run after fixes
- **Status:** done 2026-10-16
- **Result:** Whole-topic review: 0 blocking. It walked full multi-flight sequences across every exit path. Fixed: the refusal-timer clear on FINISHED, the altitudehold HISTORY row, "critical battery" wording, and the `PREVENT_ARMING` justification. Clean re-gate on PRIMUS_X2_v1 ( +456 B ); PRIMUS_V5 and PRIMUSX2 build.

### 10. Graph refresh & commit

**Description.** Restore the template `PlutoPilot.cpp`. Run `graphify update .` and `python tools/graph_labels.py`,
then `/pluto-commit`: all-target build, FW version bump, promotion of `PIPELINE_UPDATE.md`, CHANGELOG, the drafted
message. The user commits.

- **Depends on:** 9
- **Skills / agent:** graphify, pluto-commit
- **Files:** -
- **Safety impact:** none
- **Done when:** staged, with the drafted message shown; the user commits
- **Status:** done 2026-10-16
- **Result:** Graph refreshed and labelled ( 4157 nodes ). All three targets clean-built ( PRIMUS_X2_v1 108276 B text, PRIMUS_V5 108268, PRIMUSX2 108096; no new warnings ). FW 3.13.0 → 3.14.0 ( user ). PIPELINE_UPDATE.md applied; CHANGELOG v4.0.0; staged, message drafted.

### 11. A queued take-off cannot arm the drone later ( 2 s refused-arm drop, Dev Mode off drop )

**Description.** Found in the task 4 review. `Command_TakeOff` sent while the drone is disarmed and cannot arm leaves
TAKE_OFF / RUNNING. Cases: after a landing with the ARM switch still on, because `OK_TO_ARM` is cleared at disarm
( `mw.cpp:539` ); during calibration; on low battery. `takeOff ( )` then retries `mwArm ( )` on every loop
( `command.cpp` disarmed branch ). Nothing clears the command: `mwDisarm ( )` acts only when armed, and
`updateCommandStatus ( )` has no callers. When `OK_TO_ARM` returns ( ARM switch off, `mw.cpp:469-478` ), the next ARM-on
arms the drone and climbs to the take-off height, **even with Dev Mode off and no user code running**. This predates
this topic. The user's `onLoopStart ( )` pattern hits it whenever a Wi-Fi restart comes after a landing. Decide the
drop rule at the start of the task ( mini-grill ): candidates are dropping a queued TAKE_OFF at a real Dev Mode
switch-off, when the ARM switch goes off while disarmed, and / or after a time limit.

- **Depends on:** 2 ( same function )
- **Skills / agent:** pluto-rules, pluto-build
- **Files:** [command.cpp](../../../../src/main/command/command.cpp), [mw.cpp](../../../../src/main/mw.cpp) ( user-code switch-off path, `userCode ( )` )
- **Safety impact:** arming. Removes an arm and climb with no user code running. Must not drop the intended
  `Command_TakeOff`-arms-the-drone path ( ARM switch turned on after the call, check topic log-2 )
- **Done when:** builds with no new warnings; the review confirms that a queued take-off can no longer arm the drone
  after the chosen drop events, and that case B still arms and climbs
- **Rollback:** revert the drop rule
- **Status:** done 2026-10-16
- **Result:** `endTakeOff ( )`, the 2 s refused-arm drop, `takeOffFromUser` and `dropUserTakeOff ( )` at a real Dev Mode off ( CHANGES.md ). Gate clean. Review: 0 blocking; 1 should-fix ( HISTORY rows ) fixed; N2 timer reset fixed; N1 accepted. No disarm-based drop: the hazard arises after the disarm.

### 12. `Command_Land` refused during a flip

**Description.** Found by the task 7 compliance check ( predates this topic ). B_FLIP marks itself FINISHED as soon
as the flip starts ( `command.cpp:324-333` ), and `Command_Land` checks only FINISHED / ABORT and ARMED
( `FC-Control-Command.cpp:151-160` ). So a landing sent during a flip starts at once. Its `isLanding` pin takes the
throttle over the flip's ( `mw.cpp:386-387`, `:1398-1399` run after `flip ( )` at `:1364-1366` ), and its impact
check ( `command.cpp:274-277` ) has no flip exemption. The crash detector and the battery auto-land both skip flips
( `failsafe.cpp:374-377`, `mw.cpp:1274-1277` ). Fix ( user decision ): `Command_Land` is ignored while
`flipActive ( )`. User code sends it again after the flip.

- **Depends on:** 1
- **Skills / agent:** pluto-rules, pluto-build
- **Files:** [FC-Control-Command.cpp](../../../../src/main/API-Src/FC-Control-Command.cpp) ( `flipActive ( )` from `flight/acrobats.h` )
- **Safety impact:** landing, flip. Removes a landing that takes over in the middle of a flip. The failsafe and
  battery landings are unchanged
- **Done when:** builds clean; the review confirms that `flipActive ( )` covers the whole flip ( including the
  post-flip handover ) and that no landing path is newly blocked
- **Rollback:** remove the guard
- **Status:** done 2026-10-16
- **Result:** `Command_Land` returns while `flipState != 0` ( `ENABLE_ACROBAT` ), the same test as the battery auto-land; the header comment says so. Gate clean. Review: 0 blocking. The guard covers every flip ( all start in `executeCommand ( )` B_FLIP ). The stuck-SLOWDOWN gap goes to task 15. Not covered: the failsafe and MSP LAND during a flip.

### 13. `Command_TakeOff` arms only with the stick centred

**Description.** Found by the task 7 compliance check. The disarmed branch of `takeOff ( )` calls `mwArm ( )` whatever
the throttle stick does ( `command.cpp:159-163` ); only the goal is gated by `altHoldStickCentred ( )`. With the stick
off-centre the drone arms with the motors running and no height, and then follows the stick. Fix ( user decision ):
with the stick off-centre, the take-off waits and does not arm, like waiting for the ARM switch. It must not run the
2 s refusal timer. Once the stick is centred it arms and climbs. Keep the stick test the one altitude hold uses
( `rcData`, `alt_hold_deadband` ).

- **Depends on:** 11 ( same branch )
- **Skills / agent:** pluto-rules, pluto-build
- **Files:** [command.cpp](../../../../src/main/command/command.cpp)
- **Safety impact:** arming. Fewer arms: a take-off never arms with the stick off-centre
- **Done when:** builds clean; the review confirms that an off-centre stick neither arms nor starts the refusal
  timer, and that centring then arms and climbs
- **Rollback:** revert the condition
- **Status:** done 2026-10-16
- **Result:** The disarmed branch arms only with the stick centred and waits otherwise. The 2 s refusal timer ( `takeOffRefusedTick ( )` ) also runs whenever ARM is on with `OK_TO_ARM` off, whatever the stick ( review should-fix, re-checked CLOSED ). Gate clean.

### 14. Gate and validation flight for tasks 12-13

**Description.** Re-run the build gate on PRIMUS_X2_v1 against 3.13.0 ( budget 400 B ). Fly one new case, G: with
ALT HOLD on, Dev Mode on, stick held **above** centre ( for example 1600 ) and ARM on, the drone must not arm. Then
centre the stick: it arms and climbs about 100 cm. Task 12 is verified by the code review only, because a flip with a
landing sent mid-flip is not worth a flight. Add case G to the validation file, and record the results in
`TESTING.md`.

- **Depends on:** 12, 13
- **Skills / agent:** pluto-build, pluto-flighttest
- **Files:** `test/PlutoPilot_fix_validation.cpp.txt`, `TESTING.md`, `log-7.txt`
- **Safety impact:** flight test
- **Done when:** gate clean, flash growth under 500 B ( raised from 400 B by the user ); case G passes
- **Rollback:** reflash FW 3.13.0
- **Status:** done 2026-10-16
- **Result:** Gate clean, +448 B flash against 3.13.0 ( budget raised to 500 B ). Flight G passes ( log-7 ): waited disarmed for 12.7 s with `Tp` about 1708, armed with the height set when the stick reached 1540, climbed to 80-98 cm, landed on the floor.

### 15. Flip watchdog: a flip longer than 8 s ends

**Description.** Found in the task 12 review ( predates this topic ). In the flip state machine, state 3
( SLOWDOWNANDEXIT ) has a 2 s timeout only inside its `pitch > 450` branch ( `acrobats.cpp:173` ). With pitch below 0,
or pitch 0-45° and |roll| above 25° ( `:192` ), no branch matches, and `flipState` stays 3 until disarm. While it does,
`Command_Land` ( task 12 ) and the battery auto-land are refused, and the crash detector is off
( `failsafe.cpp:375` ). Fix ( user decision ): in `flip ( )`, if `millis ( ) - flipStartTime` exceeds 8000 ms, end the
flip the way its other timeouts do. The longest legitimate flip is about 7.7 s ( 2.2 + 2 + 2 + 1.5 ). Never seen in a
log ( flip-althold-regression TESTING.md ).

- **Depends on:** 12
- **Skills / agent:** pluto-rules, pluto-build
- **Files:** [acrobats.cpp](../../../../src/main/flight/acrobats.cpp)
- **Safety impact:** flip. The watchdog ends a stuck flip. Check what the existing timeouts do on exit ( some
  disarm on FLIP_BACK ) and choose the safe exit for an upright, stuck drone: hand back to level flight, not a
  disarm in the air
- **Done when:** builds clean; the review confirms the watchdog never fires in a normal flip ( 7.7 s maximum ) and that
  the exit leaves the drone flying
- **Rollback:** remove the watchdog
- **Status:** done 2026-10-16
- **Result:** 8 s watchdog at the top of `flip ( true )`. It hands back within 70° on both axes ( the crash detector's limit ), otherwise disarms. Gate clean. Review: 0 blocking; 1 should-fix ( thresholds, was 90° / 45° ) applied as proposed.

## Decisions log

Newest last. One line each: `YYYY-MM-DD [decision|assumption|out-of-scope|risk] text`.

- 2026-10-16 [decision] New linked fix topic for the check topic `takeoff-api-ground-hover` ( user ).
- 2026-10-16 [decision] A take-off on an already-armed drone counts as on the ground within 3 s of arming or at ground idle; once per arm; no-op in the air ( user ).
- 2026-10-16 [decision] D1: `Command_TakeOff` / `Command_Flip` are ignored silently during a landing ( user ).
- 2026-10-16 [decision] D2: `conclusive` only after the ramp has run at least 1 s ( user signed off 1 s in the playback ).
- 2026-10-16 [decision] Validation: five flights ( A, B, D re-send during landing, E ARM-switch idle, F default land ) ( user ).
- 2026-10-16 [assumption] `isThrottleStickArmed` stands in for the static `altHoldGroundIdle`: it is cleared once the stick goes above 1500 ( `altitudehold.cpp:581-587` ).
- 2026-10-16 [out-of-scope] D5 ( `updateCommandStatus ( )` has no callers ); battery and failsafe landing logic beyond D1.
- 2026-10-16 [decision] Task 1 guard: `isLanding || ( current_command == LAND && command_status != FINISHED )`. A bare `current_command == LAND` would refuse every take-off after the first landing, because LAND is never reset ( D5 ).
- 2026-10-16 [out-of-scope] `MSP_SET_COMMAND` ( `serial_msp.cpp:1569` ) checks only `isLanding`: a one-cycle gap when the failsafe first sets LAND. It heals itself, because failsafe and battery re-assert LAND ( review note ). Not user-code reachable.
- 2026-10-16 [decision] Task 2 extras: `altHoldReleaseGroundIdle ( )` clears both idle flags, because the controller runs before `applyAltHold ( )`; the goal is set only with the stick centred ( `altHoldStickCentred ( )` ), so a stick request cannot spend the once-per-arm flag; the disarmed branch sets the goal only once actually armed ( a refused `mwArm ( )` left no goal for the later arm, an existing defect ).
- 2026-10-16 [out-of-scope] `altHoldStickCentred ( )` matches the default `alt_hold_fast_change` 0 only; with 1 ( CLI only ) the stick test is relative to `initialThrottleHold` ( review N2 ).
- 2026-10-16 [risk] D2 margin is estimated at 0.3-0.4 s when `Command_Land` arrives during the take-off climb ( reviewer estimate, not measured ). Optional flight F2 added to tasks 4 / 6.
- 2026-10-16 [risk] Found in the task 4 review: a TAKE_OFF queued while the drone cannot arm survives a disarm and Dev Mode off. It arms and climbs at the next ARM-on with no user code running ( predates the topic ). Task 11 added before the build gate; the test versions send the take-off once per boot to avoid it.
- 2026-10-16 [decision] Task 11 ( user ): with ARM on and arming refused, retry 2 s and then drop; a take-off queued with ARM off waits, and a user-code take-off is dropped at a real Dev Mode switch-off; the app's take-off is not dropped with Dev Mode ( the 2 s rule applies to it too ).
- 2026-10-16 [decision] Accepted ( review N1 ): a user take-off queued with ARM off ( for example re-sent by a Wi-Fi restart after the pilot switched ARM off ) arms at the next ARM-on while Dev Mode stays on. This is case B's path; the wiki advises sending once per boot or once per arm.
- 2026-10-16 [assumption] Corrected: a calibration refusal does not succeed on a retry. Calibration clears `OK_TO_ARM`, and only ARM off restores it ( `mw.cpp:469-500` ), so the 2 s rule drops such a take-off.
- 2026-10-16 [out-of-scope] Review N5 ( predates the topic ): after `Command_DisArm` while a TAKE_OFF is queued, `current_command` stays TAKE_OFF / FINISHED and blocks a new `Command_TakeOff` until an app command or a Dev Mode off. It belongs with D5.
- 2026-10-16 [decision] Flash budget raised from 300 B to 400 B ( user ): the measured growth is +328 B, and task 11 was added after the budget was set.
- 2026-10-16 [decision] Task 6: all six validation flights pass ( logs 1-6 ). The Wi-Fi-restart repeat path ( case 4 ) and the task 11 drops were not exercised in flight; they are covered by the code reviews.
- 2026-10-16 [decision] The task 7 compliance check ( workflow, 8 requirements, 19 agents ) found 7 hold and 1 partial ( the stick-band wording ), plus 5 undocumented behaviours. The user chose code fixes for two: `Command_Land` refused during a flip ( task 12 ) and `Command_TakeOff` arming only with the stick centred ( task 13 ). The other three are wiki fixes in task 7. Task 12 is verified by review only.
- 2026-10-16 [decision] Task 12: `Command_Land` ignored during a flip. The user chose a flip watchdog ( task 15, 8 s ) for the pre-existing stuck-SLOWDOWN gap, rather than accepting it.
- 2026-10-16 [out-of-scope] The RC-loss failsafe ( `failsafe.cpp:247-251` ) and the app's `MSP_SET_COMMAND` can still start a LAND during a flip ( task 12 review note ). Unchanged.
- 2026-10-16 [decision] Task 13: "refused" means ARM on with `OK_TO_ARM` off, or a centred `mwArm ( )` refusal, whatever the stick. Only ARM off, or off-centre with `OK_TO_ARM` set, is a plain wait ( review finding; keeps the task 11 post-landing drop independent of the stick ).
- 2026-10-16 [out-of-scope] A refusal with `OK_TO_ARM` still set ( battery, `isCalibrated` false ) can be put off by stick jitter at the deadband edge. It cannot arm, because `mwArm ( )` keeps refusing ( review ).
- 2026-10-16 [decision] Task 15 exit: hand back if |roll| and |pitch| are 70° or less ( matches the crash detector, `failsafe.cpp:368` ), otherwise disarm like the existing stuck-flip timeouts. Only FLIP_BACK can run ( `flipDirection` is always 0 ).
- 2026-10-16 [decision] Flash budget raised from 400 B to 500 B ( user ): the measured growth is +448 B after tasks 12, 13 and 15.
- 2026-10-16 [decision] Task 7 doc check: with the battery OK, `BMS_Update ( )` re-enables `OK_TO_ARM` every 21 ms ( `battery.cpp:794-795` ), so arming is allowed again right after a landing with ARM still on. A re-sent `Command_TakeOff` / `Command_Arm` re-arms and climbs. The user chose **document only** ( wiki and header ). The task 11 / 13 "post-landing refusal" premise is corrected in CHANGES.md and the code comments; their code is unchanged ( it still drops real refusals: calibration, low battery ).
- 2026-10-16 [out-of-scope] The same override also defeats `annexCode ( )`'s level-attitude and ARM-cycle rules for the ARM-switch arming path ( a tilted arm; a re-arm at idle after a landing with the stick at the bottom ). The user asked to ignore it unless it is critical for normal use. Judged not critical: with the app the stick sits at 1500 ( the ARM-switch path arms only with the stick at the bottom ) and the drone on the floor is level. **Correction ( topic review ):** `PREVENT_ARMING` after an RC-loss landing is not reached, because `failsafeActivate ( )` has no caller ( `failsafe.cpp:175` ), so after an RC-loss landing the drone can re-arm too. It needs the stick at the bottom or user code re-sending the take-off. Not tracked further.
- 2026-10-16 [decision] Topic review: 0 blocking. Fixed: the refusal timer is also cleared when `command_status == FINISHED` ( a disarm with TAKE_OFF queued ); the altitudehold.cpp HISTORY row; "low battery" corrected to "critical battery" ( `isBatteryLow` is never set, and WARNING does not refuse arming ); the `PREVENT_ARMING` justification corrected.
