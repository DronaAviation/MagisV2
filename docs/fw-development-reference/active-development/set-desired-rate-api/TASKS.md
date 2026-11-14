# Set desired rate API - Tasks

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [TEST_CODE](TEST_CODE.md)

## Resume

Read this block first in a new session; read only the task it points to.

| | |
|---|---|
| **Current task** | none: **Closed** in `3be9093` ( 15 Nov 2026, FW 3.15.9, API 1.8.7 ) |
| **Next step** | The user commits with `git commit -F .git/MAGISV2_COMMIT_MSG.txt`; the next `pluto-commit` or `pluto-grill` run fills in the hash. |
| **Open questions** | none |
| **Blocked on** | the user's commit |
| **Last updated** | 2026-11-15 |

## Summary

| | |
|---|---|
| **Mode** | fix |
| **Goal** | A value set with `setDesiredRate ( )` stays set on every axis, and `getDesiredRate ( )` reads it back. |
| **Done when** | The user's own test shows: the read-back equals the set ( clamped ) value across loops, disarm / re-arm and after a flip; RATE-mode stick response follows the set roll / pitch rate; flips fly as before; with no call, roll / pitch stay 75. |
| **Target** | PRIMUS_X2_v1 |
| **Analysis** | Causal chain proven in code ( INVESTIGATION.md ): `flip ( false )` writes 75 every disarmed loop; flip exits write 75; the getter returns the PID setpoint. Fix: RAM user rate restored by the flip code instead of 75. |
| **Pipeline docs affected** | subsystems/User_Space_API.md, subsystems/Core_PID_Controller.md |
| **Order** | 1 -> 2 -> 3 -> 4 -> 5 -> 6 -> 7 -> 10 -> 8 -> 9 |

## Index

| # | Title | Status | Depends on | Skills / agent |
|---|---|---|---|---|
| 1 | Bench test code and pre-fix baseline | done 2026-11-15 | - | pluto-flighttest |
| 2 | Setter stores a clamped user rate; getter returns it | done 2026-11-15 | 1 | pluto-rules, cpp-pro ( optional ) |
| 3 | Flip code restores the user rate instead of 75 | done 2026-11-15 | 2 | pluto-rules |
| 4 | FC-Config.h doc comment and API_Version 1.8.7 | done 2026-11-15 | 3 | pluto-rules |
| 5 | Build gate | done 2026-11-15 | 2, 3, 4 | pluto-build |
| 6 | Hardware validation | done 2026-11-15 | 5 | pluto-flighttest |
| 7 | Architecture & pipeline docs ( PIPELINE_UPDATE.md ) and API wiki | done 2026-11-15 | 6 | graphify, spec-to-code-compliance |
| 10 | Refuse a roll / pitch rate change during a flip | done 2026-11-15 | 7 | pluto-rules, pluto-build |
| 8 | Topic review | done 2026-11-15 | 7, 10 | pluto-reviewer agent |
| 9 | Graph refresh & commit | done 2026-11-15 | 8 | graphify, pluto-commit |

Status values: `todo`, `in-progress`, `done YYYY-MM-DD`, `blocked (<why>)`, `dropped (<why>)`.
Serial numbers are never reused or renumbered.

## Tasks

### 1. Bench test code and pre-fix baseline

**Description.** The user tests this themselves ( no T16 script is handed over ), so write the test code they will
run, before and after the fix. In `PlutoPilot.cpp`: in `plutoInit ( )` call `setDesiredRate ( AG_ROLL, 40 )`,
`( AG_PITCH, 50 )`, `( AG_YAW, 120 )`; in `plutoLoop ( )` print the three `getDesiredRate ( )` values at a low rate
( well under 130 bytes per tick ). Add a second step that sets an out-of-range value ( e.g. roll 300, yaw -5 ) after a
few seconds, to check the clamp later. Keep the code in a `TEST_CODE.md` so the stock `PlutoPilot.cpp` can be
restored at commit. The user flashes the current firmware and records the baseline log ( expected: values do not
match the set ones ).

- **Depends on:** -
- **Skills / agent:** pluto-flighttest
- **Files:** [PlutoPilot.cpp](../../../../PlutoPilot.cpp) ( repo root ), `TEST_CODE.md`, `TESTING.md`
- **Safety impact:** none on the bench ( disarmed ). Test code only.
- **Done when:** the test code builds, and the user's pre-fix log is saved under `logs/` and summarised in
  `TESTING.md`.
- **Rollback:** restore the stock `PlutoPilot.cpp`.
- **Status:** done 2026-11-15
- **Result:** Test code written and built ( TEST_CODE.md, TESTING.md ); pre-fix baseline skipped by the user's choice: the cause is proven in the code. The same code is used for the post-fix run ( task 6 ).

### 2. Setter stores a clamped user rate; getter returns it

**Description.** In [FC-Config-Setpoints.cpp](../../../../src/main/API-Src/FC-Config-Setpoints.cpp) add a RAM user
rate per axis plus a "set" flag ( 3 + 3 bytes or a bitmask ). `setDesiredRate` clamps with `constrain` ( 0-100 roll
/ pitch, 0-255 yaw ), stores the value, sets the flag, and writes it into `currentControlRateProfile->rates [ ]` so it
takes effect at once, with explicit casts ( removes the 3 baseline `-Wconversion` warnings ). `getDesiredRate`
returns the stored user rate when set, else the live `rates [ ]` value ( 75 roll / pitch in practice ). Expose a small
C-linkage helper ( e.g. `userRollPitchRate ( axis )` returning the user rate or 75 ) for task 3. Nothing is written
to EEPROM. Decisions: getter returns the set rate, clamp, held until reboot / next call.

- **Depends on:** 1
- **Skills / agent:** pluto-rules; cpp-pro if delegated
- **Files:** [FC-Config-Setpoints.cpp](../../../../src/main/API-Src/FC-Config-Setpoints.cpp), a shared header for
  the helper ( API-Utils.h or acrobats.h, decided in the task )
- **Safety impact:** roll / pitch rate only acts in RATE / HORIZON mode and the flip's non-ANGLE states; yaw rate
  scales the yaw stick in every mode. The clamp caps it at the app's own limits.
- **Done when:** builds clean on PRIMUS_X2_v1; the 3 setter warnings are gone from the gate; reviewer 0 blocking.
- **Rollback:** revert the file; the old setter / getter come back.
- **Status:** done 2026-11-15
- **Result:** Setter clamps ( 0-100 / 0-255 ) and holds the rate in the existing `userDesiredRate [ ]`; getter returns it, else the live rate; `userRollPitchRate ( )` added for task 3. Gate green ( flash 122.6 KB, RAM 20.5 KB ), 3 setter `-Wconversion` warnings fixed. Reviewer 0 blocking; its test-code finding ( the out-of-range step could run while armed ) and both comment nits fixed.

### 3. Flip code restores the user rate instead of 75

**Description.** In [acrobats.cpp](../../../../src/main/flight/acrobats.cpp) classify each `rates [ FD_ROLL / FD_PITCH ]
= 75` write ( `:87-88` watchdog, `:116-117` ASCEND, `:240-241` HOLD, `:329-330` HOLDPOS, `:343-349` `flip ( false )` ).
Writes the craft flies with while `flipState != 0` keep 75 / 110 ( flip behaviour unchanged, out of scope ). Writes
that hand roll / pitch back to normal flight ( flip end, watchdog abort, disarmed idle ) use the task-2 helper: the
user rate when set, else 75. Also check the disarm-time flip aborts ( `mwDisarm` paths the scout listed ) end in one
of these restores. Record the classification of every write in `CHANGES.md`.

- **Depends on:** 2
- **Skills / agent:** pluto-rules
- **Files:** [acrobats.cpp](../../../../src/main/flight/acrobats.cpp)
- **Safety impact:** flip path. Flip rates and timing must stay identical; only the post-flip / disarmed resting
  value changes, and only when the user API was called. Flip invariants ( altitude hold bypassed, flip ends within
  8 s ) untouched.
- **Done when:** builds clean; with no `setDesiredRate` call every path still ends at 75 / 75 ( checked by reading
  each path ); reviewer 0 blocking.
- **Rollback:** revert `acrobats.cpp`; flips and the disarmed reset return to the hard 75.
- **Status:** done 2026-11-15
- **Result:** `restoreRestRates ( )` ( `userRollPitchRate ( )` ) replaces the 75 at the watchdog and `flip ( false )`, and is added at the ASCEND timeout, HOLD end and HOLDPOS end; in-flip 75 / 110 untouched. Reviewer confirmed path-by-path equivalence with no user rate set, and that the disarm aborts restore via the next `flip ( false )`. Gate green ( flash 122.6 KB, RAM 20.5 KB ), reviewer 0 blocking.

### 4. FC-Config.h doc comment and API_Version 1.8.7

**Description.** Update the `setDesiredRate` / `getDesiredRate` comments in
[FC-Config.h](../../../../src/main/API/FC-Config.h) ( `:180-194` ): units ( Cleanflight rate ), ranges and clamp,
lifetime ( until reboot or next call, not saved to EEPROM ), that flips fly their own rates and return to the user
value, and that roll / pitch only act outside ANGLE mode. Signatures unchanged. Bump `API_Version` 1.8.6 -> 1.8.7 in
the Makefile ( behaviour change ); `FW_Version` is bumped by `pluto-commit`.

- **Depends on:** 3
- **Skills / agent:** pluto-rules
- **Files:** [FC-Config.h](../../../../src/main/API/FC-Config.h), [Makefile](../../../../Makefile)
- **Safety impact:** none ( comments and version string ).
- **Done when:** builds clean; header comments match the code from tasks 2-3.
- **Rollback:** revert both files.
- **Status:** done 2026-11-15
- **Result:** `getDesiredRate` / `setDesiredRate` doxygen rewritten ( range and clamp, hold until next call or reboot, where each axis acts with the default PID controller, flip return and the timeout caveat, live-settings save / reload caveat ); banner HISTORY row added. The `API_Version` bump moved to task 9 ( branch convention: FW and API bumped together at commit ). Save / reload claims checked against `mw.cpp:262`, `acceleration.cpp`, `compass.cpp:110`, `serial_msp.cpp:1399 / 1624`. Gate green.

### 5. Build gate

**Description.** `pluto-build --gate PRIMUS_X2_v1` with `PROJECT=<plutoide.ini project_name>`. No new `src/`
warnings ( 3 fewer expected ); flash / RAM fit. Never regenerate the baseline.

- **Depends on:** 2, 3, 4
- **Skills / agent:** pluto-build
- **Files:** -
- **Safety impact:** none
- **Done when:** gate green; flash / RAM deltas recorded in `CHANGES.md`.
- **Rollback:** -
- **Status:** done 2026-11-15
- **Result:** Clean build PRIMUS_X2_v1, gate green: no new `src/` warnings, 0 outside `src/` / `lib/` ( test `PlutoPilot.cpp` clean ), 3 setter warnings fixed. Flash 122.6 KB ( text 124324 + data 1316 B ), RAM 20.5 KB ( data 1316 + bss 19768 B ); about +0.1 KB flash over the test-code-only build ( 122.5 KB ), no RAM change.

### 6. Hardware validation

**Description.** Bench test, props off, no flying ( decision 2026-11-15: the app cannot hold Developer Mode on and flip
at the same time, and the log only runs in Developer Mode ). The user runs the task-1 test code on the fixed firmware:
read-backs at S0-S3, the clamp at S2, arm with the ARM switch and disarm to show the value holds, then S4. Then a control
run with `TEST_SET_RATES = false`. Procedure and expected values in `TESTING.md`; results recorded there. The RATE-mode
stick response and the flip restore are not covered and stay on the code reading plus the reviewer's check.

- **Depends on:** 5
- **Skills / agent:** pluto-flighttest; pluto-log-analyst for large logs
- **Files:** `TESTING.md`, `logs/log-2.txt`, `logs/log-3.txt`
- **Safety impact:** none in flight. Props off; the drone is armed on the bench with the throttle down, held by hand.
- **Done when:** `logs/log-2.txt` shows 40 / 50 / 10 on every disarmed line ( no 75 ), 100 / 0 / 255 at S2, the value
  held across arm and disarm, 30 / 40 / 15 after S4 and `F` = 0 throughout; `logs/log-3.txt` shows 75 / 75 / 0. Any
  failure is logged and becomes a new task.
- **Rollback:** flash the previous firmware ( FW 3.15.8 without this topic ).
- **Status:** done 2026-11-15
- **Result:** Bench log-2 PASS: 790 records, no `lR` / `lP` = 75 on any line ( 517 disarmed lines held the set value ), getter = live rate on every line, 300 / -5 / 400 clamped to 100 / 0 / 255, values held across arm and disarm ( S4 set while armed held 25 s after the disarm ), `F` = 0. Control run log-3 skipped ( user ). Not flight-validated: RATE-mode stick and flip restore.

### 7. Architecture & pipeline docs ( PIPELINE_UPDATE.md ) and API wiki

**Description.** Write new `docs/API/FC_CONFIG_API_WIKI.md` covering only `setDesiredRate` and `getDesiredRate`
( units, range, clamp, lifetime, modes where it acts, flip interaction, example in the right hook ). Stage new
versions of `User_Space_API.md` and `Core_PID_Controller.md` ( rate setter / getter semantics, the flip's rate
restore ) in `PIPELINE_UPDATE.md`; never edit the pipeline folder directly. Mermaid only if a flow changes, every
edge a real call path. Check the wiki against the code with `spec-to-code-compliance`. Add a one-line CLAUDE.md flight
invariant only if the user agrees it is a gotcha.

- **Depends on:** 6
- **Skills / agent:** graphify, spec-to-code-compliance
- **Files:** `docs/API/FC_CONFIG_API_WIKI.md`, `PIPELINE_UPDATE.md`
- **Safety impact:** none
- **Done when:** the wiki and staged docs match the code; compliance check has no open drift.
- **Rollback:** -
- **Status:** done 2026-11-15
- **Result:** New `docs/API/FC_CONFIG_API_WIKI.md` ( these two functions only: range and clamp, where each axis acts, lifetime, flips, saving, three examples in the right hooks, pitfalls, "Changes in 1.8.7" ) and `PIPELINE_UPDATE.md` ( new "Stick rates" section in `User_Space_API.md` with a Mermaid diagram of real call paths, "Stick rates and the flip" in `Core_PID_Controller.md`, and three moved line references: `acrobats.cpp:84` -> `:94` twice, `FC-Config.h:95` -> `:97` ). `spec-compliance` workflow: 7 requirements hold, 0 divergences, 1 refuted, and two **undocumented behaviours** it found were verified and fixed in the wiki, the `FC-Config.h` comment and `PIPELINE_UPDATE.md`: a setter call during a flip overrides the flip's rate ( task 10 now guards it ), and under a non-default `pidController` the rates scale that controller's dynamic P / I / D gains in every mode. Four smaller doc problems fixed ( INVESTIGATION scope wording, the `PlutoPilot.cpp` link path, the README claim that the commit restores the stock file, which `pluto-commit` does not do ). Gate green.

### 10. Refuse a roll / pitch rate change during a flip

**Description.** Found by the task-7 compliance check and confirmed in the source: `userCode ( )` ( `mw.cpp:1436` ) runs
after `flip ( true )` ( `:1407` ) and before `pid_controller ( )` ( `:1507` ), and `setDesiredRate` writes the live
profile with no `flipState` check, so a call during a flip replaces the flip's spin rate. The flip writes 110 once, at
the ASCEND transition ( `acrobats.cpp:135-150` ), and PITCHING never rewrites it, so a low user rate can stall the flip
into its 2 s timeout, which calls `mwDisarm ( )` in the air ( `acrobats.cpp:182-185` ) on `BACK_FLIP`, the API's only
direction. Pre-existing, not a regression. Fix: in `setDesiredRate`, for `AG_ROLL` / `AG_PITCH` while `flipState != 0`,
store `userDesiredRate` and set the flag but do **not** write `currentControlRateProfile->rates [ ]`; the flip's exit
`restoreRestRates ( )` then applies it. Yaw is unaffected ( the flip never touches it ). Update the wiki "Flips"
section, its pitfall line and the `FC-Config.h` comment from "do not call during a flip" to what the guard does.

- **Depends on:** 7
- **Skills / agent:** pluto-rules, pluto-build
- **Files:** [FC-Config-Setpoints.cpp](../../../../src/main/API-Src/FC-Config-Setpoints.cpp), [FC-Config.h](../../../../src/main/API/FC-Config.h), `docs/API/FC_CONFIG_API_WIKI.md`, `PIPELINE_UPDATE.md`
- **Safety impact:** removes a mid-air-disarm path reachable from user code. Does not change the flip's own rates or
  timing; with no call during a flip, behaviour is unchanged.
- **Done when:** builds clean on PRIMUS_X2_v1, gate green; a roll / pitch call with `flipState != 0` leaves
  `rates [ ]` untouched and is applied at the flip exit ( read through ); yaw still applies at once; docs match.
- **Rollback:** revert the guard; the setter applies during a flip again, as before 1.8.7.
- **Status:** done 2026-11-15
- **Result:** `setDesiredRate` now holds a roll / pitch value but skips the live write while `flipState != 0` ( `FC-Config-Setpoints.cpp:183`, with `flight/acrobats.h` included ); the flip's exit applies it and `getDesiredRate ( )` returns it at once. Yaw unguarded ( the flip never touches it ). Wiki "Flips", its pitfall, "Changes in 1.8.7", the `FC-Config.h` comment and `PIPELINE_UPDATE.md` ( new bullet, Mermaid edge, and a table correcting the four pipeline refs the include shifted ) updated. Gate green ( flash 122.7 KB, RAM 20.5 KB ). Reviewer 0 blocking, 0 should-fix: it traced every path a held value reaches, found no ISR or atomicity issue, and confirmed a disarmed set cannot be swallowed; both doc nits fixed.

### 8. Topic review

**Description.** `pluto-reviewer` on the whole topic diff; fix BLOCKING findings and re-run the gate.

- **Depends on:** 7
- **Skills / agent:** pluto-reviewer agent
- **Files:** -
- **Safety impact:** none
- **Done when:** 0 blocking findings; gate green after fixes.
- **Rollback:** -
- **Status:** done 2026-11-15
- **Result:** Whole-topic `pluto-reviewer`: **0 blocking**, 2 should-fix and 2 nits, all fixed. It confirmed that with no `setDesiredRate` call every changed path still writes 75 / 75 ( the only no-call change is the getter, which is the fix ), walked the set / arm / disarm / flip / reload sequences against the docs, re-checked every `PIPELINE_UPDATE.md` line reference, and confirmed the test code is flagged for task 9. Fixed from its findings: the save / reload wording in the wiki and `FC-Config.h` ( a profile change saves **and** loads, and roll / pitch are written back by `flip ( false )`, so only yaw keeps a reloaded value ), the `FC-Config.h` flip-timeout sentence ( it contradicted the wiki ), the getter's no-call wording ( during a flip it returns the flip's rate ), `mw.cpp:358` -> `:359`, and the stray `spec-compliance/` output moved into the topic folder. Gate re-run green.

### 9. Graph refresh & commit

**Description.** Restore the stock `PlutoPilot.cpp` ( test code stays in `TEST_CODE.md` ). `graphify update .` then
`python tools/graph_labels.py`, then `/pluto-commit` ( all-target build, version bump, pipeline promotion,
CHANGELOG, staged message ). The user commits. The bump is FW 3.15.8 -> 3.15.9 and API 1.8.6 -> 1.8.7 ( API behaviour change; moved here from task 4 ).

- **Depends on:** 8
- **Skills / agent:** graphify, pluto-commit
- **Files:** -
- **Safety impact:** none
- **Done when:** staged and the message drafted; topic set Closed - pending commit.
- **Rollback:** -
- **Status:** done 2026-11-15
- **Result:** Stock `PlutoPilot.cpp` restored ( copy confirmed identical in `TEST_CODE.md` first ). `graphify update .` ( 5119 nodes, 14018 edges, 247 communities; `userRollPitchRate ( )` and `restoreRestRates ( )` in the graph ) and the labels via the isolated-mode workaround. Previous topic's hash filled in ( `73022c3` ). All three targets clean: PRIMUS_X2_v1 121.5 KB / 20.5 KB, PRIMUSX2 121.3 / 20.7, PRIMUS_V5 121.5 / 20.5; no new warnings on any target, none in the touched files. FW 3.15.8 -> 3.15.9, API 1.8.6 -> 1.8.7 ( user confirmed ). `PIPELINE_UPDATE.md` applied to three pipeline docs after correcting two statements task 8 made stale. CHANGELOG version line, Fixed and Documentation entries. CLAUDE.md flight-invariant line added. Staged; message drafted.

## Decisions log

Newest last. One line each: `YYYY-MM-DD [decision|assumption|out-of-scope|risk] text`.

- 2026-11-15 [decision] Mode `fix`, target PRIMUS_X2_v1.
- 2026-11-15 [decision] The user tests it themselves; the plan supplies test code ( task 1 ) and the checks ( task 6 ).
- 2026-11-15 [decision] `getDesiredRate` returns the set rate ( or the effective rate when none was set ), not the PID setpoint.
- 2026-11-15 [decision] A set rate is held in RAM until reboot or the next call, across disarm / arm; after a flip roll / pitch return to it.
- 2026-11-15 [decision] Out-of-range input is clamped: 0-100 roll / pitch, 0-255 yaw ( MSP_SET_RC_TUNING limits ).
- 2026-11-15 [decision] With no `setDesiredRate` call, roll / pitch stay 75: no behaviour change for existing code.
- 2026-11-15 [decision] Fix shape: RAM user rate restored by the flip code ( scout option A / C ), not a save / restore of the EEPROM profile ( option B would change the default from 75 to 0 ).
- 2026-11-15 [decision] Wiki: new `docs/API/FC_CONFIG_API_WIKI.md` with only these two functions; API 1.8.6 -> 1.8.7.
- 2026-11-15 [out-of-scope] Flip rates ( 75 / 110 ) and timing, `getDesiredAngle`, EEPROM persistence, other FC-Config APIs, the app's MSP_SET_RC_TUNING roll / pitch ( also reset to 75 today ).
- 2026-11-15 [assumption] An app EEPROM save while disarmed stores whatever is in the profile then ( the user rate after this fix, if set ). Persistence is not changed on purpose.
- 2026-11-15 [risk] Graph is stale ( built ebf4e46, HEAD 73022c3 ); grep was used for line numbers. Re-check `acrobats.cpp` lines before editing.
- 2026-11-15 [decision] Test values chosen for flight safety: yaw 10 / 15 ( 1.5x / 1.75x the stock yaw stick; 120 would be ~7x ), roll / pitch 40 / 50 then 30 / 40 ( act only outside ANGLE mode ); the out-of-range step runs disarmed on the bench and is restored 5 s later, before arming.
- 2026-11-15 [decision] Pre-fix baseline log skipped ( user ): the causal chain is proven in the code; log-1 stays empty and the post-fix run uses the next log.
- 2026-11-15 [decision] Task 2 reuses the unused `userDesiredRate [ 3 ]` / `isUserDesiredRate [ 3 ]` ( API-Utils.cpp ) instead of new fields; `STOCK_ROLL_PITCH_RATE` ( 75 ) and `userRollPitchRate ( )` live in API-Utils.
- 2026-11-15 [risk] A profile change ( app / CLI ) or MSP_SET_RC_TUNING after a `setDesiredRate` call rewrites the live yaw rate, and `getDesiredRate` then still returns the user value; roll / pitch are re-applied by the flip code. Out of scope ( other writers unchanged ).
- 2026-11-15 [decision] Reviewer finding fixed in the test code: the out-of-range step is skipped while armed ( clamped yaw 255 is ~8.6x the stock yaw stick ), marker "STEP 2 SKIPPED ARMED".
- 2026-11-15 [decision] The wiki ( tasks 4 / 7 ) must state that the setter writes the live profile, so a later EEPROM save by the app or a stick acc-trim stores the rate, and a profile reload drops the live value while `getDesiredRate` still reports the user one.
- 2026-11-15 [decision] Task 3 mini-grill: restore the user rate only at the flip's normal exits ( watchdog, ASCEND timeout, HOLD / HOLDPOS end, disarmed `flip ( false )` ). The four abnormal 2 s timeouts that set `flipState = 0` without disarming ( SLOWDOWNANDEXIT left, PITCHINGPOS, SLOWDOWNANDEXITPOS both ) stay as they are.
- 2026-11-15 [risk] Separate finding, out of scope: those four abnormal flip timeouts leave the flip axis at 110 with ANGLE mode off until disarm ( today, with or without this fix ). Candidate for its own topic.
- 2026-11-15 [decision] `acrobats.cpp` includes `API/API-Utils.h` rather than forward-declaring the helper ( reviewer nit ): `flight/pid.cpp:43` and `mw.cpp:111` do the same, and the extra `lib/` warning noise never fails the gate.
- 2026-11-15 [decision] `API_Version` 1.8.6 -> 1.8.7 is done at commit ( task 9 ), not in task 4: on this branch FW and API are bumped together by `pluto-commit`, so an early bump would risk a double bump.
- 2026-11-15 [decision] Task 6 is bench only, props off: the app cannot keep Developer Mode on and flip at the same time, and `plutoLoop ( )` only runs in Developer Mode. The bench run covers the reported bug, the clamp and the hold across arming.
- 2026-11-15 [decision] No unit or simulated test: `src/test/` does not build ( CLAUDE.md ), and the two functions cannot be compiled on a host without the STM32 include chain; a host copy of the logic would not test the firmware binary.
- 2026-11-15 [risk] Not flight-validated: the rate reaching the PID ( RATE-mode stick ) and the flip restore. Both rest on the task-3 path reading and the reviewer's equivalence check ( with no API call the code is value-identical to before ). To be stated at commit.
- 2026-11-15 [decision] Control run ( log-3 ) skipped by the user; task 6 closed on bench log-2. The no-call default rests on the task-3 path reading.
- 2026-11-15 [decision] Task 7 compliance findings: the two undocumented behaviours are fixed in the docs, and the flip override also gets a code guard ( task 10, user's choice ) rather than staying a documented footgun.
- 2026-11-15 [decision] Under a non-default `pidController` ( app / CLI ) the rates scale that controller's dynamic P / I / D gains in every mode, ANGLE included. Documented in the wiki and `PIPELINE_UPDATE.md`; no code change ( the default controller `pidRewrite` is unaffected ).
- 2026-11-15 [decision] Only `BACK_FLIP` is reachable: `flipDirection` is written 0 and nothing else ( `command.cpp:342` ). So the four abnormal flip timeouts that leave 110 on the flip axis are dead paths today, which lowers the priority of that earlier finding; recorded in `PIPELINE_UPDATE.md` and the wiki wording.
- 2026-11-15 [decision] Task 8 doc fixes: a profile change ( `MSP_SELECT_SETTING`, `serial_msp.cpp:1405` ) saves the live settings **before** reloading, so it both stores a user rate and reloads others; roll / pitch are then written back by `flip ( false )` and only yaw keeps the reloaded value. Wiki and header corrected.
- 2026-11-15 [decision] The `spec-compliance/` workflow output ( 6 requirement files, task 7 ) is moved to `<topic>/spec-compliance/` so it is not a stray untracked folder at the repo root.
- 2026-11-15 [decision] Closed - pending commit: FW 3.15.8 -> 3.15.9 ( patch, a fix ), API 1.8.6 -> 1.8.7 ( behaviour only ), user confirmed. Committed with the RATE-mode stick response and the restore after a flip not flight-validated ( bench test only, by the user's choice ).
