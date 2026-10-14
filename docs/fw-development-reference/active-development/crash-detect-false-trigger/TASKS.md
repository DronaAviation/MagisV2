# Crash Detect False Trigger - Tasks

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

## Resume

Read this block first in a new session; read only the task it points to.

| | |
|---|---|
| **Current task** | **Closed** in `386599b` ( 13 Oct 2026, FW 3.11.1 ). All tasks done or dropped. |
| **Next step** | none |
| **Open questions** | none |
| **Blocked on** | nothing |
| **Last updated** | 2026-10-13 |

## Summary

| | |
|---|---|
| **Mode** | fix ( planned as check; the user asked for the 15% change up front ) |
| **Goal** | Stop the false crash-disarms on light bumps and landings without missing a real crash. |
| **Done when** | No false crash trigger in the validation flights ( bumps, firm landings, normal flying ), and a deliberate crash still disarms. |
| **Target** | PRIMUS_X2_v1 |
| **Analysis** | Confirmed: the contact spike of a bump ( ≤ 1.32 g raw ) fired the accel terms; tilt and calibration bias ruled out. Flips are caught by the 70° rule. Final limits 5000 / 8000 ( INVESTIGATION.md findings ). |
| **Pipeline docs affected** | subsystems/Failsafe_Subsystem.md ( new crash-detector section ), Error_Failure_Mode.md ( line 4 ) |
| **Order** | 1 → 12 → 2 → ( 3 dropped ) → 4 → 5 → 6 → 7 → 8 → 9 → 10 → 11 |

## Index

| # | Title | Status | Depends on | Skills / agent |
|---|---|---|---|---|
| 1 | Impact limits 15% less sensitive | done 2026-10-08 | - | pluto-rules, pluto-build, pluto-reviewer agent |
| 12 | Impact limits a further 20% less sensitive | done 2026-10-08 | 1 | pluto-rules, pluto-build, pluto-reviewer agent |
| 2 | Temporary crash-trip diagnostics | done 2026-10-12 | 12 | pluto-rules, pluto-flighttest |
| 3 | Bench: accel bias and tilt margin | dropped (bias ruled out by log-1 pre-arm, ~0.8°; bumps at < 14° tilt, so tilt is not the cause) | 2 | pluto-flighttest |
| 4 | Diagnostic flights | done 2026-10-12 | 2 | pluto-flighttest, pluto-log-analyst agent |
| 5 | Findings: are the new limits enough? | done 2026-10-13 | 4 | pluto-flighttest |
| 6 | Remove the diagnostics | done 2026-10-13 | 5 | pluto-rules, pluto-reviewer agent |
| 7 | Build gate | done 2026-10-13 | 6 | pluto-build |
| 8 | Hardware validation | done 2026-10-13 | 7 | pluto-flighttest |
| 9 | Architecture & pipeline docs ( PIPELINE_UPDATE.md ) | done 2026-10-13 | 8 | graphify, spec-to-code-compliance |
| 10 | Topic review | done 2026-10-13 | 9 | pluto-reviewer agent |
| 11 | Graph refresh & commit | done 2026-10-13 ( staged; the user commits ) | 10 | graphify, pluto-commit |

Status values: `todo`, `in-progress`, `done YYYY-MM-DD`, `blocked (<why>)`, `dropped (<why>)`.
Serial numbers are never reused or renumbered.

## Tasks

### 1. Impact limits 15% less sensitive

**Description.** The user asked for this up front. Raise the two lateral-accel crash limits in
`failsafeOnCrash ( )` by 15%: `accSmooth` 1250 → 1438 LSB ( 0.305 → 0.351 g ) and `accADC`
1750 → 2013 LSB ( 0.427 → 0.491 g ). The 70° angle rule, the ANGLE_MODE gate and the flip bypass
stay as they are ( flip logic is out of scope ).

- **Depends on:** -
- **Skills / agent:** pluto-rules, pluto-build ( `--gate` ), pluto-reviewer agent
- **Files:** [failsafe.cpp](../../../../src/main/flight/failsafe.cpp#L366)
- **Safety impact:** Crash failsafe ( disarm ) in ANGLE mode. A real low-speed impact is now slightly
  less likely to disarm, and a touchdown is less likely to end the flight through this path.
- **Done when:** PRIMUS_X2_v1 builds, no new `src/` warnings, the reviewer finds nothing blocking.
- **Rollback:** put 1250 / 1750 back in `failsafe.cpp`.
- **Status:** done 2026-10-08
- **Result:** gate passed ( no new warnings; flash 112.1 KB / 256, RAM 16.0 KB / 40 ). Review: nothing blocking; one fix to the comment wording ( the g figures were next to the old values ), applied. Risk of a missed low-speed crash: low, because the 70° rule is unchanged. Risk at touchdown: low, because `LAND` has its own Z impact check ( `command.cpp:212` ).

### 12. Impact limits a further 20% less sensitive

**Description.** The user asked for this after task 1: lower the sensitivity by a further 20%, on top of
the task 1 values. `accSmooth` 1438 → 1726 LSB ( 0.351 → 0.421 g ) and `accADC` 2013 → 2416 LSB
( 0.491 → 0.590 g ), about 38% above the original 1250 / 1750. Static tilt at which they trip:
~24.9° and ~36.1°. The 70° rule, the ANGLE_MODE gate and the flip bypass are unchanged.

- **Depends on:** 1
- **Skills / agent:** pluto-rules, pluto-build ( `--gate` ), pluto-reviewer agent
- **Files:** [failsafe.cpp](../../../../src/main/flight/failsafe.cpp#L366)
- **Safety impact:** Crash failsafe ( disarm ) in ANGLE mode, as in task 1, but a wider band of lateral impacts no longer disarms.
- **Done when:** PRIMUS_X2_v1 builds, no new `src/` warnings, the reviewer finds nothing blocking.
- **Rollback:** put 1438 / 2013 back ( task 1 values ), or 1250 / 1750 ( original ).
- **Status:** done 2026-10-08
- **Result:** gate passed ( no new warnings; flash 100.9 KB / 256, RAM 14.8 KB / 40, with the uncommitted `target.h` change from session start no longer present ). Review: nothing blocking. One should-fix: the chip's ~5.7 Hz accel filter ( τ ~28 ms ) smooths impact spikes, so a slow drift into a wall at ~0.1-0.2 m/s may now not trip either accel rule. Added to the task 4 / 8 flight tests.

### 2. Temporary crash-trip diagnostics

**Description.** Add a debug-only latch in `failsafeOnCrash ( )` that keeps, on every trip, a code for
the term that fired ( angle / `accSmooth` / `accADC`, and which axis ) and the peak |`accADC` X/Y|,
|`accSmooth` X/Y| and roll / pitch near the trip. Also keep running maxima while armed, so flights that
do not trip still show the margin. Print them through `PlutoPilot.cpp` `Monitor_Print`. `accADC` cannot
be reached through the API, so the latch has to live in the core. The user agreed to this temporary
debug build on 8 Oct; it is removed in task 6.

- **Depends on:** 12
- **Skills / agent:** pluto-rules, pluto-flighttest ( log fields )
- **Files:** [failsafe.cpp](../../../../src/main/flight/failsafe.cpp), [PlutoPilot.cpp](../../../../PlutoPilot.cpp)
- **Safety impact:** none to the trip decision ( read-only latch ). Keep the log line under ~130 B per tick.
- **Done when:** builds clean with the gate; a bench tap or tilt shows the latch fill with the right term.
- **Rollback:** revert the two files ( removed in task 6 regardless ).
- **Status:** done 2026-10-12
- **Result:** works on hardware: log-1 has 517 complete records, `T` and `N` match the code, and peaks and events are captured ( TESTING.md test 1 ). Built by the user in PlutoIDE. The warning gate was not run ( blocked by a permission check ) and is owed in task 7. Review: nothing blocking. Two comment fixes applied: the line is ~105 B on the wire ( ≤ 111 worst case ), not ~65; `T` / `N` are crash events, not disarms. Not a concern: a flip `return` skips the event-flag update, which only matters with the disarm commented out. Limits at this point: 2500 / 3000, set by the user. `set_FSI ( Crash )` / `mwDisarm ( )` are commented out by the user, so there is no crash disarm in this build.

### 3. Bench: accel bias and tilt margin

**Description.** Tests hypotheses 3 and 2. Level and disarmed, read X/Y accel ( `Sensor_Get ( Accelerometer )`
cm/s², or the task 2 latch ). Above ~50 cm/s² points to calibration bias eating the margin. Then, armed at
low throttle in ANGLE mode with the props off or restrained, tilt the craft by hand and record the angle at
which each accel term trips. The prediction for the new limits is ~20.5° for `accSmooth` and ~29.4° for
`accADC`. Note the surface and date of the last accelerometer calibration.

- **Depends on:** 2
- **Skills / agent:** pluto-flighttest
- **Files:** `TESTING.md` ( new )
- **Safety impact:** none ( bench, props off or restrained ).
- **Done when:** bias X/Y and the trip angle of each term are recorded in TESTING.md.
- **Status:** dropped (2026-10-12)
- **Result:** dropped. log-1's pre-arm lines give an offset of ~0.8° ( ~60 LSB ), so bias is ruled out. log-1's bumps happened at < 14° tilt, so the tilt-trip check would not change the outcome.

### 4. Diagnostic flights

**Description.** Fly the task 2 build on PRIMUS_X2_v1 and cover the situations that trip today: light
bumps against a wall or hand, firm and bouncy landings, normal and aggressive ANGLE flying, **a slow
drift into a wall ( ~0.2 m/s ) and a bump at hover height**, and **one deliberate crash as a control**
( it must still disarm ). Logs go to numbered `log-N.txt` in this folder.
Record each trip: the term, the peak value, the ratio to the new limit, and whether it was false.

- **Depends on:** 2 ( 3 dropped )
- **Skills / agent:** pluto-flighttest; pluto-log-analyst agent for large logs
- **Files:** `log-N.txt`, `TESTING.md`
- **Safety impact:** flights with a debug build; crash failsafe active as in task 1.
- **Done when:** at least 5 flights logged with bumps and landings, plus a deliberate crash, all
  tabulated in TESTING.md.
- **Status:** done 2026-10-12
- **Result:** log-1..4: 15 armed segments. Survivable bumps peak at ≤ 5409 raw / 3100 filtered; contacts that flipped the craft are no bigger ( ≤ 4970 ) and were caught by the 70° rule in 0.1-0.35 s; gentle landings ≤ 878. At 5000 / 8000 no bump or landing disarmed, and all 3 flips disarmed. Not shown: firm landings, a craft pinned upright ( TESTING.md test 2 results ).

### 5. Findings: are the new limits enough?

**Description.** From tasks 3 and 4, rank the causes and decide on the next step. One option is to keep
the task 1 + 12 change. Others, if the logs justify them: per-term limits, a persistence or debounce of N
consecutive 3.5 ms loops, or the pre-`49a2fdb` rule ( `accSmooth > 12000` ). The findings must also say
how touchdown ends if the crash path no longer fires on landing: `LAND`'s own rules ( `LAND_IMPACT_ACC`
6000, 300 ms settle ) or a pilot disarm. Landing detection code stays out of scope. If more code is
needed, add it as new tasks with `/pluto-task add` before task 6.

- **Depends on:** 4
- **Skills / agent:** pluto-flighttest
- **Files:** [INVESTIGATION.md](INVESTIGATION.md), `TESTING.md`
- **Safety impact:** decides how sensitive the crash failsafe ends up.
- **Done when:** INVESTIGATION.md has the confirmed cause, the numbers, and the user's chosen next step.
- **Status:** done 2026-10-13
- **Result:** cause confirmed: the contact spike of a bump ( ≤ 1.32 g raw ) fired the accel terms. Final limits 5000 / 8000 ( 1.22 / 1.95 g ), then lowered by the user to **4000 / 7000** ( 0.977 / 1.71 g ; gate passed: no new warnings, 101.7 KB / 14.8 KB ), 70° unchanged; **reverted by the user to 5000 / 8000** before validation ( decision 2026-10-13 ). Gaps ( pinned upright, Z, caught ) accepted and moved to the Planned topic crash-stuck-detection. No new code tasks.

### 6. Remove the diagnostics

**Description.** Remove the task 2 latch and its log line ( plus any extra code tasks added from task 5 ),
leaving only the agreed detector change. Check with `git diff` that `failsafe.cpp` holds only the intended
change and that `PlutoPilot.cpp` is back to its earlier state.

- **Depends on:** 5
- **Skills / agent:** pluto-rules, pluto-reviewer agent
- **Files:** [failsafe.cpp](../../../../src/main/flight/failsafe.cpp), [PlutoPilot.cpp](../../../../PlutoPilot.cpp)
- **Safety impact:** none ( the diagnostics are read-only ).
- **Done when:** the diff contains only the agreed detector change; it builds.
- **Rollback:** -
- **Status:** done 2026-10-13
- **Result:** no `crashDbg*` left; `PlutoPilot.cpp` back to HEAD. The diff against HEAD is limits 4000 / 7000 ( later reverted to 5000 / 8000 by the user ), three named terms and the limits comment ( the stale `>5000` trailing comment was cut, per the review ). Gate: no new warnings, 100.9 KB / 14.8 KB. Review: nothing blocking.

### 7. Build gate

**Description.** `pluto-build --gate PRIMUS_X2_v1` on the final code: no new `src/` warnings, and flash and
RAM fit.

- **Depends on:** 6
- **Skills / agent:** pluto-build
- **Files:** -
- **Safety impact:** none
- **Done when:** gate passes; flash/RAM recorded.
- **Status:** done 2026-10-13
- **Result:** PRIMUS_X2_v1: no new `src/` warnings. Flash 100.9 KB / 256 ( text 102112 + data 1272 B ), RAM 14.8 KB / 40 ( bss 13896 B ). `DEFAULT_PRIMUS_X2_v1_3.11.0.hex`. Only `failsafe.cpp` modified under `src/`; no `crashDbg*` left. Re-run after the user reverted to 5000 / 8000 ( comment fixed to match ): no new warnings, 100.9 KB / 14.8 KB.

### 8. Hardware validation

**Description.** Fly the clean build ( no diagnostics ) through the same situations as task 4. This is the
topic's done-when check.

- **Depends on:** 7
- **Skills / agent:** pluto-flighttest
- **Files:** `log-N.txt`, `TESTING.md`
- **Safety impact:** crash failsafe as shipped.
- **Done when:** no false crash trigger in at least 5 flights with bumps and firm landings, and a
  deliberate crash still disarms.
- **Status:** done 2026-10-13
- **Result:** the user reports test 3 flown and passed at 5000 / 8000: no false trip on bumps or landings, and the deliberate crash disarmed. log-5.txt was left empty, so per-flight detail is not recorded ( TESTING.md test 3 results ).

### 9. Architecture & pipeline docs ( PIPELINE_UPDATE.md )

**Description.** Stage in `PIPELINE_UPDATE.md`:
- a crash-detector section for `subsystems/Failsafe_Subsystem.md`: the conditions and final limits, the
  ANGLE_MODE gate, the flip bypass, disarm, and the ~300 ms flag clear;
- the correction to `Error_Failure_Mode.md:4`: a crash disarms and the flag clears; nothing halts until a
  power cycle.

Add a Mermaid flow only if every edge is a real call path. Add a FLIGHT_INVARIANTS / CLAUDE.md line only
if task 5 found a new gotcha.

- **Depends on:** 8
- **Skills / agent:** graphify, spec-to-code-compliance
- **Files:** `PIPELINE_UPDATE.md`
- **Safety impact:** none
- **Done when:** PIPELINE_UPDATE.md is staged and checked against `failsafe.cpp` and `mw.cpp`.
- **Status:** done 2026-10-13
- **Result:** applied directly at commit time ( no separate PIPELINE_UPDATE.md ): a new "Crash Detector" section with a terms table and a Mermaid flow in `subsystems/Failsafe_Subsystem.md`, every edge checked against `mw.cpp:1424`, `failsafe.cpp:362-380`, `mw.cpp:484-494` and `rc_controls.cpp:183`; the claim in `subsystems/Error_Failure_Mode.md:4` that a crash is a power-cycle halt is corrected. CHANGELOG has a Fixed entry and the version line updated for 3.11.1.

### 10. Topic review

**Description.** `pluto-reviewer` on the whole topic diff. Fix any BLOCKING findings and re-run the gate.

- **Depends on:** 9
- **Skills / agent:** pluto-reviewer agent
- **Files:** -
- **Safety impact:** none
- **Done when:** no BLOCKING findings left open.
- **Status:** done 2026-10-13
- **Result:** no BLOCKING findings. The code is the same on all three targets ( ICM20948, `acc_1G` 4096 everywhere ); no other user of these limits; `LAND` touchdown uses Z only; no API wiki or CLAUDE.md change needed. 2 should-fix and 2 nits, all in topic docs, all fixed.

### 11. Graph refresh & commit

**Description.** `graphify update .` and `python tools/graph_labels.py`, then `/pluto-commit` ( all-target
build, version bump, doc promotion, staged change and drafted message; the user commits ).

- **Depends on:** 10
- **Skills / agent:** graphify, pluto-commit
- **Files:** -
- **Safety impact:** none
- **Done when:** staged, message drafted, topic Closed.
- **Status:** done 2026-10-13
- **Result:** graph refreshed, all targets built at 3.11.1, change staged, commit message drafted in `.git/MAGISV2_COMMIT_MSG.txt`; the user commits.

## Decisions log

Newest last. One line each: `YYYY-MM-DD [decision|assumption|out-of-scope|risk] text`.

- 2026-10-08 [decision] Target PRIMUS_X2_v1.
- 2026-10-08 [decision] Planned as `check`; the user then asked for the impact sensitivity to be reduced by 15% now, so the mode is `fix`, with the investigation as tasks 2-5.
- 2026-10-08 [decision] "Impact sensitivity" means the two lateral-accel terms ( `accSmooth`, `accADC` ); the 70° angle term is unchanged.
- 2026-10-08 [decision] Every landing trip counts as false, as well as light bumps or collisions.
- 2026-10-08 [decision] A temporary debug build with a crash-trip latch is allowed; it is removed in task 6.
- 2026-10-08 [decision] The fix may be anything the logs justify: per-term limits, persistence, or the pre-`49a2fdb` rule.
- 2026-10-08 [out-of-scope] Flip logic, API / MSP / app changes, and landing / touchdown detection code.
- 2026-10-08 [out-of-scope] `FlightStatus_Check ( FS_CRASHED )` maps to the wrong status bit in the API ( `FC-Control.h:30` vs `runtime_config.h:92` ); to be handled in a separate topic.
- 2026-10-08 [assumption] "No false trigger in flights" = 5 flights with bumps and firm landings and no false trip; the user can change the number.
- 2026-10-08 [risk] Less sensitive limits may miss a real low-speed impact ( motors keep spinning ); the deliberate-crash control in tasks 4 and 8 covers this.
- 2026-10-08 [risk] If no landing trips the crash path, touchdown ends only through `LAND`'s rules or a pilot disarm; task 5 checks this.
- 2026-10-08 [decision] The user asked for the sensitivity to drop 20% more; applied on top of the task 1 values ( compounded: 1726 / 2416 LSB, ~38% above the original ), task 12.
- 2026-10-08 [risk] At 1726 / 2416 a slow sideways contact ( ~0.1-0.2 m/s into a wall, no tip past 70° ) may not disarm, so the motors keep running against the wall. Rated low-medium by the review; tested in tasks 4 and 8 with the latch's peak |`accADC` X/Y| against 2416.
- 2026-10-09 [decision] The user set the limits to 2500 / 3000 LSB ( 0.610 / 0.732 g ) by hand, replacing task 12's 1726 / 2416.
- 2026-10-09 [risk] The user commented out `set_FSI ( Crash )` / `mwDisarm ( )`: no crash disarm at all in the working tree. It must be restored before any flight that relies on it, and before commit.
- 2026-10-12 [decision] log-1: events 1-4 were deliberate light bumps ( must not disarm ), lighter than typical in-use bumps; the 40.5 s event was an accidental real crash ( must disarm ).
- 2026-10-12 [assumption] The 4.5x raw-peak gap between bumps and crash in log-1 is an upper bound; task 4 flights need typical-strength bumps.
- 2026-10-12 [decision] Task 3 dropped: bias ruled out by log-1's pre-arm lines, and tilt is not the cause of the bumps.
- 2026-10-12 [decision] Task 4 flights: disarm restored, test-candidate limits 5000 / 8000 ( 1.22 / 1.95 g ), 3 flights with a full mix of bumps and landings, a deliberate crash at the end of flight 3.
- 2026-10-12 [risk] At 5000 / 8000 both accel limits are above 1 g, so tilt alone can never trip them: only an impact or the 70° rule can. These are not caught: craft pinned upright against a wall, ceiling strike or upright drop ( Z is not checked ), craft caught in a curtain or by a hand, craft resting tilted at 20-69°. Task 5 must decide whether a held-tilt term or a debounced lower limit is needed. 5000 / 8000 are test values and must not be committed as they are.
- 2026-10-12 [decision] log-2: flying into the wall; log-3: landings with the LAND command and by hand; log-4: light and hard bumps ( the user's notes ).
- 2026-10-12 [risk] Firm or bouncy landings were not reproduced ( log-3 landings ≤ 0.21 g ); task 8 must include them.
- 2026-10-13 [decision] Final limits `accSmooth` 5000 ( 1.22 g ) / `accADC` 8000 ( 1.95 g ), 70° unchanged ( task 5 ). This replaces the earlier note that 5000 / 8000 were test values not to be committed.
- 2026-10-13 [out-of-scope] Pinned upright, ceiling / upright drop ( Z ), caught in a curtain or hand, resting tilted at 20-69°: accepted; Planned topic crash-stuck-detection.
- 2026-10-13 [decision] The user lowered the final limits to 4000 ( 0.977 g ) / 7000 ( 1.71 g ), accepting 1.29× margin over the hardest survivable bump for an earlier catch of medium hits. Not flown yet; task 8 validates them. Replaces 5000 / 8000.
- 2026-10-13 [decision] Diagnostics removed before task 8, so validation flies the exact shipped build; pass or fail is read from the app's crashed flag and disarm behaviour. The three named trip terms stay.
- 2026-10-13 [decision] The user reverted the final limits to 5000 / 8000 ( 1.22 / 1.95 g ), the values flown in test 2; this replaces 4000 / 7000. The comment was corrected to match. Task 8 validates 5000 / 8000.
- 2026-10-13 [decision] Commit requested: task 8 recorded as passed on the user's report ( notes not filled in ); tasks 9-10 done as part of the commit; FW_Version 3.11.0 → 3.11.1 ( patch ), API unchanged.
- 2026-10-13 [decision] The untracked battery-soc-fix/APP_INTEGRATION.md is left out of this commit.
