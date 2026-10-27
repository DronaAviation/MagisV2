# Payload Tilt On Descent - Tasks

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

## Resume

Read this block first in a new session; read only the task it points to.

| | |
|---|---|
| **Current task** | none: **Closed - pending commit** ( 27 Oct 2026 ) |
| **Next step** | The user commits with `git commit -F .git/MAGISV2_COMMIT_MSG.txt`; the next `pluto-commit` / `pluto-grill` run fills in the hash. |
| **Open questions** | none |
| **Blocked on** | the user's commit |
| **Last updated** | 2026-10-27 |

## Summary

| | |
|---|---|
| **Mode** | fix ( from `check`, 27 Oct 2026 ) |
| **Goal** | A full app stick-down in flight must not zero the attitude rate I-term, so an off-centre payload ( or the airframe's own trim ) no longer leans the craft; every ground and arming reset keeps working. |
| **Done when** | On the fixed build, with the front payload and the log-1 procedure, `IP` stays within about +/-15 counts of its hover value through every full stick-down and the pitch matches a partial stick-down; app arm / take-off, Land button and a stick-down touchdown plus lift-off behave as before ( flip: by review, not loggable from the app ); diagnostics removed; gate clean on PRIMUS_X2_v1, all targets built at commit. |
| **Target** | PRIMUS_X2_v1 |
| **Analysis** | Proven by log-1..5: `mw.cpp:895-897` zeroes `errorGyroI` while raw `Th` < 1100; it is the only integrator holding a CG offset in ANGLE mode, so the craft leans 2-7 deg. Present since at least `v3.0.0`; `2a8d59a` only exposed it. Fix: freeze the `pidRewrite` rate I-term instead of resetting it while `ARMED && throttleRaisedSinceArm && !isThrottleStickArmed` ( INVESTIGATION.md ). |
| **Pipeline docs affected** | subsystems/Core_PID_Controller.md, RC_Input_Pipeline.md, Mixer_Pipeline.md, Altitude_Hold_Estimator.md; dev-guide/FLIGHT_INVARIANTS.md; CLAUDE.md flight invariants |
| **Order** | 1 → 2 → 3 → 7 → 8 → 9 → 10 → 11 → 12 → 13 → 14 → 15 |

## Index

| # | Title | Status | Depends on | Skills / agent |
|---|---|---|---|---|
| 1 | Temporary diagnostic log of throttle, attitude, I-term and motors | done 2026-10-27 | - | pluto-flighttest, pluto-rules, pluto-build |
| 2 | Test plan and flights on HEAD: full vs partial stick-down with payload | done 2026-10-27 | 1 | pluto-flighttest |
| 3 | Control flights on a rebuilt `4657cbf` with the same log | done 2026-10-27 | 1 | pluto-build, pluto-flighttest |
| 4 | Log analysis: lean vs I-term vs throttle, descent time, motor floor | dropped (superseded: per-log reads and summary in TESTING.md; findings in task 8) | 2, 3, 7 | pluto-log-analyst agent, pluto-flighttest |
| 5 | Findings, recommendation and removal of the diagnostics | dropped (superseded by tasks 8 and 12) | 4 | pluto-rules, pluto-build |
| 6 | Close | dropped (superseded by fix closing tasks 13-15) | 5 | pluto-commit |
| 7 | Control flight on release `v3.0.0` with the same log | done 2026-10-27 | 1 | pluto-build, pluto-flighttest |
| 8 | Findings: ranked causes, evidence and the chosen fix | done 2026-10-27 | 2, 3, 7 | pluto-flighttest |
| 9 | Freeze the rate I-term during an airborne stick-down | done 2026-10-27 | 8 | pluto-rules, cpp-pro agent |
| 10 | Build gate | done 2026-10-27 | 9 | pluto-build |
| 11 | Hardware validation: payload stick-down and regression flights | done 2026-10-27 | 10 | pluto-flighttest |
| 12 | Remove the diagnostics and the worktrees, re-gate | done 2026-10-27 | 11 | pluto-rules, pluto-build |
| 13 | Architecture & pipeline docs ( PIPELINE_UPDATE.md ) | done 2026-10-27 | 12 | graphify, spec-to-code-compliance |
| 14 | Topic review | done 2026-10-27 | 13 | pluto-reviewer agent |
| 15 | Graph refresh & commit | done 2026-10-27 | 14 | graphify, pluto-commit |

Status values: `todo`, `in-progress`, `done YYYY-MM-DD`, `blocked (<why>)`, `dropped (<why>)`.
Serial numbers are never reused or renumbered.

## Tasks

### 1. Temporary diagnostic log of throttle, attitude, I-term and motors

**Description.** Add one bounded `Monitor_Print` line in `plutoLoop ( )` of `PlutoPilot.cpp` that
prints the raw throttle ( `rcData[THROTTLE]`, the value the THROTTLE_LOW test uses, not `rcDataPilot` ),
the roll and pitch angle ( decidegrees ), the roll and pitch rate I-term and the four motor outputs.
`errorGyroI[]` is file-static in `pid.cpp` and `axisPID_I[]` is only written under `BLACKBOX`, so a
tiny read-only accessor in `pid.cpp` / `pid.h` ( e.g. `pidGetErrorGyroI ( axis )` ) is allowed as
diagnostic-only code; it is removed in task 5. Rate the print so the line stays well under the 130-byte
per-tick budget ( integers only, no `double` overload ). Keep a copy of the added lines in the topic
( TESTING.md ) so task 3 can port them to the old commit.

- **Depends on:** -
- **Skills / agent:** pluto-flighttest ( log fields ), pluto-rules, pluto-build ( `--gate PRIMUS_X2_v1` )
- **Files:** [PlutoPilot.cpp](../../../../PlutoPilot.cpp), [pid.cpp](../../../../src/main/flight/pid.cpp), [pid.h](../../../../src/main/flight/pid.h)
- **Safety impact:** none ( read-only diagnostics; no control-path change )
- **Done when:** `pluto-build --gate PRIMUS_X2_v1` passes with no new `src/` warning; the line is under 130 bytes per tick; PlutoMonitor shows all fields changing on the bench with the sticks ( props off ).
- **Rollback:** revert `PlutoPilot.cpp`, `pid.cpp`, `pid.h` to HEAD.
- **Status:** done 2026-10-27
- **Result:** `pidGetGyroITerm ( )` added; 11-field line in `plutoLoop ( )` plus a `START` marker; gate clean ( flash 121.9 KB, RAM 20.5 KB ); reviewer: 0 blocking, 0 should-fix, worst case 123 B/tick. Bench check of the fields is folded into the first flight of task 2.

### 2. Test plan and flights on HEAD: full vs partial stick-down with payload

**Description.** Write the test procedure with `pluto-flighttest` ( TESTING.md ) and fly it on HEAD with
the task 1 build: same payload ( under 10 g ) on the front arm; take off from the app, hover 5 s, hold
the stick about one third down for 3 s ( above 1100, no THROTTLE_LOW ), centre 3 s, hold the stick fully
down for 3 s, centre, repeat; then land with the Land button. Repeat once with the payload on the right
arm and once with no payload. One `logs-N.txt` per flight, saved in the topic folder.

- **Depends on:** 1
- **Skills / agent:** pluto-flighttest
- **Files:** `TESTING.md`, `logs-*.txt` in this folder
- **Safety impact:** flights with a leaning craft near the ground; fly in a clear area, keep the full-down holds short ( 3 s ) and start them above 1 m.
- **Done when:** at least three logs on HEAD ( front, right, none ) each containing both a partial and a full stick-down with the raw throttle visible crossing 1100.
- **Rollback:** n/a ( test only )
- **Status:** done 2026-10-27
- **Result:** Three HEAD logs, 19 full stick-downs: in every one `Th` < 1100 pinned `IR` / `IP` at 0 within 100 ms and the craft leaned toward whatever the I-term had been holding ( front payload 2-7 deg nose-down, right payload 4.5-6 deg roll, no payload 2-3 deg roll from the airframe's own trim ). All partial stick-downs ( `Th` down to 1104 ) kept the I-term and stayed level. No motor below 1453. Details: TESTING.md.

### 3. Control flights on a rebuilt `4657cbf` with the same log

**Description.** Check out `4657cbf` in a separate worktree, port the task 1 diagnostic lines ( the
`PlutoPilot.cpp` API at that commit may differ; use only `Monitor_Print` ), build `PRIMUS_X2_v1`, flash
and fly the task 2 procedure with the payload on the front arm. That build descends at up to 100 cm/s on a
full stick, so hold full-down for at most 1.5 s from at least 1.5 m. This shows whether the old build
also zeroed the I-term and leaned during its short drop ( hypothesis 2 ) or stayed level ( a real
regression in `2a8d59a` ).

- **Depends on:** 1
- **Skills / agent:** pluto-build ( plain build, no gate: the warning baseline did not exist then ), pluto-flighttest
- **Files:** worktree of `4657cbf`; `logs-*.txt` in this folder
- **Safety impact:** fast descent on the old build; short holds, clear area.
- **Done when:** one log on `4657cbf` with a partial and a full stick-down, raw throttle crossing 1100, fields identical to the HEAD log; the HEAD worktree is unchanged afterwards.
- **Rollback:** delete the worktree; re-flash the HEAD build.
- **Status:** done 2026-10-27
- **Result:** log-4 on `4657cbf`: the same reset in all eleven airborne full stick-downs ( `IR` / `IP` to 0, pitch +0.7 to +7.3 deg toward the front payload, peaks to +10 deg ). The user saw it only barely: the 100 cm/s drop hid it, and four stick-downs reached the floor. Hypothesis 2 confirmed; `2a8d59a` made the lean visible, it did not cause it. HEAD hex restored.

### 4. Log analysis: lean vs I-term vs throttle, descent time, motor floor

**Description.** Run `tools/flightlog.py` and the `pluto-log-analyst` agent on every log from tasks 2 and
3. For each stick-down: the raw throttle minimum and the time below 1100, the roll / pitch I-term before,
during and after, the peak lean angle and its sign against the payload side, the drift, the descent
duration, and the minimum of each motor ( kills hypothesis 3 if none reaches 1100 ). Compare partial vs
full stick-down, payload vs none, and HEAD vs `4657cbf`.

- **Depends on:** 2, 3, 7
- **Skills / agent:** pluto-log-analyst agent, pluto-flighttest
- **Files:** `TESTING.md` ( results tables )
- **Safety impact:** none
- **Done when:** TESTING.md holds a table per flight with the numbers above and one line per hypothesis saying confirmed / killed / open.
- **Rollback:** n/a
- **Status:** dropped (superseded: per-log reads and summary in TESTING.md; findings in task 8)
- **Result:**

### 5. Findings, recommendation and removal of the diagnostics

**Description.** Write the ranked causes with evidence into INVESTIGATION.md, the recommended fix and
its constraints ( gate the THROTTLE_LOW reset at `mw.cpp:895-897` on not armed, or ground idle, or
`!throttleRaisedSinceArm`; keep the ground / stick-arm reset and the chuck-arm path in
`rc_controls.cpp:183`; landing must not go through the stick limits ), and the doc drift found by the
scout ( Mixer_Pipeline.md minthrottle 1070 vs 1100; Core_PID_Controller.md silent on the reset and on
controller 1 being `pidRewrite` ). Then remove the task 1 diagnostics: `PlutoPilot.cpp`, `pid.cpp` and
`pid.h` back to HEAD, build gate clean. Remove the worktrees `../MagisV2-4657cbf` and `../MagisV2-v3.0.0`
( `git worktree remove --force` ) and make sure `Build/PRIMUS_X2_v1/AltX_PRIMUS_X2_v1_3.15.1.hex` is a HEAD build. Update README status and open items.

- **Depends on:** 4
- **Skills / agent:** pluto-rules, pluto-build ( `--gate PRIMUS_X2_v1` )
- **Files:** `INVESTIGATION.md`, `README.md`, [PlutoPilot.cpp](../../../../PlutoPilot.cpp), [pid.cpp](../../../../src/main/flight/pid.cpp)
- **Safety impact:** none
- **Done when:** `git diff HEAD -- PlutoPilot.cpp src/main` is empty, the gate passes, INVESTIGATION.md names the commit, the code line, the confirmed mechanism and the recommended fix.
- **Rollback:** n/a
- **Status:** dropped (superseded by tasks 8 and 12)
- **Result:**

### 6. Close

**Description.** `/pluto-commit` records the findings ( docs only, no firmware build ) and sets the topic
Closed here and in `active-development/README.md`, unless the user moves on with
`/pluto-grill fix payload-tilt-on-descent`, in which case the fix tasks are appended or a new topic links
back.

- **Depends on:** 5
- **Skills / agent:** pluto-commit
- **Files:** `README.md`, `../README.md`, `CHANGELOG.md`
- **Safety impact:** none
- **Done when:** the findings are staged with a drafted commit message; the user commits.
- **Rollback:** n/a
- **Status:** dropped (superseded by fix closing tasks 13-15)
- **Result:**

### 7. Control flight on release `v3.0.0` with the same log

**Description.** Added at the user's request after task 3. Build the tagged release `v3.0.0`
( `0acc43d`, 10 Feb 2026, FW 3.0.0 ) in a separate worktree with the task 1 diagnostics, and fly the
log-4 procedure with the front payload into `logs/log-5.txt`. `v3.0.0` has the same THROTTLE_LOW reset
( `mw.cpp:821-824` ), the same `pidRewrite` default and the same 100 cm/s unramped stick descent as
`4657cbf`, so the prediction is the same lean hidden by the fast drop. The log shows whether the
behaviour has shipped since at least that release.

- **Depends on:** 1
- **Skills / agent:** pluto-build ( plain build, no gate at that commit ), pluto-flighttest
- **Files:** worktree `E:/08-DRONA-AVIATION/99-Dump/MagisV2-v3.0.0`; `logs/log-5.txt`
- **Safety impact:** fast descent on the old build; full stick-downs of at most 1.5 s from about 2 m.
- **Done when:** one log on `v3.0.0` with three airborne full stick-downs and a partial one, fields identical to the HEAD logs; HEAD hex restored afterwards.
- **Rollback:** restore the HEAD hex from `HEAD-backup_AltX_PRIMUS_X2_v1_3.15.1.hex.bak`; re-flash.
- **Status:** done 2026-10-27
- **Result:** Built: `pid.cpp` / `pid.h` diff applied cleanly; `PlutoPilot.cpp` lines ported by anchor ( its comment block differs ), added lines identical to HEAD's; flash 93.6 KB, RAM 12.5 KB, 0 warnings in `PlutoPilot.cpp`, accessor linked. Staged as `Build/PRIMUS_X2_v1/AltX_PRIMUS_X2_v1_3.15.1.hex` in the main folder. log-5: identical behaviour to `4657cbf` and HEAD ( I-term to 0 in all four full stick-downs, pitch 2.7-6.1 deg toward the front payload, peaks to 8 deg; partial stick-downs level ). The behaviour has shipped since at least `v3.0.0`. HEAD hex restored.

### 8. Findings: ranked causes, evidence and the chosen fix

**Description.** Finish the `check` part: expand the short findings in INVESTIGATION.md into the ranked causes
with their evidence from log-1..5 ( numbers from the TESTING.md tables ), the verdict per hypothesis, the
history ( present since at least `v3.0.0`, exposed by `2a8d59a` ), the workaround until the fix ships ( stop just
above the bottom of the stick ), and the doc drift found ( Mixer_Pipeline.md minthrottle, Core_PID_Controller.md
silent on the reset ). Update the README summary. Docs only.

- **Depends on:** 2, 3, 7
- **Skills / agent:** pluto-flighttest
- **Files:** `INVESTIGATION.md`, `README.md`
- **Safety impact:** none
- **Done when:** INVESTIGATION.md names the cause, the code line, each hypothesis with confirmed / killed and its log evidence, and the chosen fix; README summary current.
- **Rollback:** n/a
- **Status:** done 2026-10-27
- **Result:** INVESTIGATION.md *Findings*: cause confirmed in 34 of 34 full stick-downs across log-1..5, boundary exactly `mincheck` 1100, lean proportional to the I-term removed; hypotheses 1-2 confirmed, 3-4 killed; history, doc drift, workaround and recommendation written.

### 9. Freeze the rate I-term during an airborne stick-down

**Description.** Implement the chosen design ( INVESTIGATION.md, *Fix* ). In `pid.h` / `pid.cpp` add a main-loop
flag ( e.g. `pidSetITermFreeze ( bool )` ) that makes `pidRewrite` skip the `errorGyroI` accumulation on all
three axes while set ( the constrain and `ITerm = errorGyroI >> 13` stay, so the frozen value is still applied ).
In `mw.cpp:892-898` keep `calculateThrottleStatus ( )` unchanged; when THROTTLE_LOW and
`ARMED && throttleRaisedSinceArm && !isThrottleStickArmed` and the active controller is `pidRewrite`, set the
freeze and skip the reset; otherwise clear the freeze and keep today's reset. Clear the freeze whenever the
throttle is not low and at disarm. Do not repurpose `throttleRaisedSinceArm`; no floats; the flag is main-loop
only. Keep the temporary diagnostics in place for task 11. Record the change in CHANGES.md.

- **Depends on:** 8
- **Skills / agent:** pluto-rules ( before and while editing ); `cpp-pro` agent if delegated ( review what comes back ); `pluto-reviewer` on the task diff
- **Files:** [mw.cpp](../../../../src/main/mw.cpp), [pid.cpp](../../../../src/main/flight/pid.cpp), [pid.h](../../../../src/main/flight/pid.h)
- **Safety impact:** attitude PID in every armed state. Ground idle, arming and disarmed must keep the reset; a stick-down touchdown must not wind up.
- **Done when:** builds; the reviewer finds no BLOCKING issue; the table in INVESTIGATION.md *Fix* matches the code case by case ( checked line by line ).
- **Rollback:** revert `mw.cpp`, `pid.cpp`, `pid.h` to the task 1 state ( diagnostics only ) and re-flash.
- **Status:** done 2026-10-27
- **Result:** `pidSetGyroITermFreeze ( )` + `gyroITermFrozen` in `pid.cpp` / `pid.h`; `mw.cpp:897-910` freezes instead of resetting while `THROTTLE_LOW && ARMED && throttleRaisedSinceArm && !isThrottleStickArmed` and the controller is `pidRewrite`; cleared in `mwDisarm`. Gate clean ( flash 122.0 KB, RAM 20.5 KB ). Reviewer: 0 blocking; case table matches the code row by row; 1 should-fix ( pre-stick-down wind-up kept: documented as a residual, added to log-7 ); 3 wording notes applied.

### 10. Build gate

**Description.** `PROJECT=AltX .claude/skills/pluto-build/driver.sh --gate PRIMUS_X2_v1`: no new `src/` warnings,
flash / RAM fit. The hex `Build/PRIMUS_X2_v1/AltX_PRIMUS_X2_v1_3.15.1.hex` is what the user flashes for task 11.

- **Depends on:** 9
- **Skills / agent:** pluto-build
- **Files:** -
- **Safety impact:** none
- **Done when:** gate passes; flash and RAM numbers recorded in TESTING.md.
- **Rollback:** n/a
- **Status:** done 2026-10-27
- **Result:** Gate passed on the task 9 diff ( no new `src/` warnings ); flash 122.0 KB, RAM 20.5 KB; `AltX_PRIMUS_X2_v1_3.15.1.hex` holds the fix and the diagnostic log. Recorded in TESTING.md.

### 11. Hardware validation: payload stick-down and regression flights

**Description.** With `pluto-flighttest`, write the plan into TESTING.md and fly the fixed build with the task 1
log line. **log-6**: front payload, the log-1 procedure ( partial and full stick-downs, climbing back to 1.5 m
after each ). **log-7**: regression: app arm then take-off; a full stick-down held until touchdown, 5 s on the
floor with the stick low, then stick-up to lift off; **the reviewer's residual: a touchdown on a partial stick
( about one third down ), 10 s on the floor, then full stick-down for 3 s, then stick-up to lift off**; an app
flip; the Land button. Compare with log-1.

- **Depends on:** 10
- **Skills / agent:** pluto-flighttest; pluto-log-analyst agent if the logs are large
- **Files:** `TESTING.md`, `logs/log-6.txt`, `logs/log-7.txt`
- **Safety impact:** first flights of a PID change; clear area, start full stick-downs at 1.5 m or higher.
- **Done when:** log-6: in every full stick-down `IP` stays within about +/-15 counts of its hover value and the mean pitch is within 1.5 deg of the partial stick-downs'; log-7: after the full-stick touchdown `IR` / `IP` do not grow while the stick is low and the lift-off is level; the partial-stick touchdown case is measured ( how far `IR` / `IP` wind in 10 s on the floor and the lift-off attitude ) and recorded as pass or as the residual; take-off, flip and Land behave as in log-1.
- **Rollback:** flash the HEAD build without the fix ( rebuild from the task 1 state ).
- **Status:** done 2026-10-27
- **Result:** log-6 ( front payload ): in all three full stick-downs ( up to 6.5 s ) `IR` / `IP` held constant at their hover values and the mean pitch was +0.1 to +0.3 deg ( log-1: up to +4.6 deg ): PASS. log-7 ( no payload ): app take-off as before, 10 s on the floor at full stick with the I-term frozen and a level lift-off, partial-stick touchdown residual not observed ( drift 10-15 counts on a level floor ), Land button clean. Flip not loggable ( app: Dev or Flip button ), covered by review.

### 12. Remove the diagnostics and the worktrees, re-gate

**Description.** Remove the task 1 diagnostics ( `pidGetGyroITerm` in `pid.cpp` / `pid.h`, the log line,
`START` marker and `extern "C"` block in `PlutoPilot.cpp` ) so only the task 9 fix remains. Remove the worktrees
`../MagisV2-4657cbf` and `../MagisV2-v3.0.0` ( `git worktree remove --force` ). Re-run the gate on PRIMUS_X2_v1.

- **Depends on:** 11
- **Skills / agent:** pluto-rules, pluto-build
- **Files:** [PlutoPilot.cpp](../../../../PlutoPilot.cpp), [pid.cpp](../../../../src/main/flight/pid.cpp), [pid.h](../../../../src/main/flight/pid.h)
- **Safety impact:** none ( diagnostics only )
- **Done when:** `git diff HEAD -- PlutoPilot.cpp` is empty; `pid.cpp` / `pid.h` / `mw.cpp` differ from HEAD only by the fix; `git worktree list` shows one tree; gate passes.
- **Rollback:** n/a
- **Status:** done 2026-10-27
- **Result:** `PlutoPilot.cpp` back to HEAD; `pidGetGyroITerm` removed; both worktrees removed ( `git worktree list` shows one tree ). Remaining diff: `mw.cpp`, `pid.cpp`, `pid.h`, 28 lines in and 3 out, identical to the reviewed task 9 fix. Gate clean, flash 121.1 KB, RAM 20.5 KB.

### 13. Architecture & pipeline docs ( PIPELINE_UPDATE.md )

**Description.** Stage the new text in the topic's `PIPELINE_UPDATE.md` ( never edit the pipeline folder ):
Core_PID_Controller.md ( the reset / freeze rule; controller 1 is `pidRewrite` with a P-only angle loop ),
RC_Input_Pipeline.md ( THROTTLE_LOW users ), Mixer_Pipeline.md ( minthrottle 1100, not ~1070 ),
Altitude_Hold_Estimator.md ( a stick-down keeps the attitude trim ). Add the invariant to
`dev-guide/FLIGHT_INVARIANTS.md` and a one-line CLAUDE.md flight-invariants bullet. Any flowchart edge confirmed
with `graphify path` or a source line.

- **Depends on:** 12
- **Skills / agent:** graphify, spec-to-code-compliance
- **Files:** `PIPELINE_UPDATE.md`, `dev-guide/FLIGHT_INVARIANTS.md`, `CLAUDE.md`
- **Safety impact:** none
- **Done when:** every listed doc has staged text that matches the code; spec-to-code check clean.
- **Rollback:** n/a
- **Status:** done 2026-10-27
- **Result:** PIPELINE_UPDATE.md staged: Core_PID_Controller ( 3 function bullets, windup bullet, new section with a flowchart ), RC_Input_Pipeline ( THROTTLE_LOW users ), Mixer_Pipeline ( minthrottle 1100 ), Altitude_Hold_Estimator ( one paragraph ), FLIGHT_INVARIANTS section and CLAUDE.md bullet ( applied at commit, as in flip-althold-regression ), drift list. Every cited line and every replace anchor checked against the source and the docs.

### 14. Topic review

**Description.** `pluto-reviewer` on the whole topic diff; fix its BLOCKING findings ( each fix through
pluto-rules and the gate again ).

- **Depends on:** 13
- **Skills / agent:** pluto-reviewer agent
- **Files:** the topic diff
- **Safety impact:** as task 9
- **Done when:** no open BLOCKING finding.
- **Rollback:** n/a
- **Status:** done 2026-10-27
- **Result:** Reviewer: code diff clean on every checklist item ( no blocking, no should-fix ); 11 doc findings, all applied: the staged invariant scoped to app flights and ALT_HOLD ( THROTTLE mode after an ARM-switch arm keeps the reset ), a CHANGELOG entry and FW 3.15.2 note staged, stale README lines, CHANGES.md anchors, the log-7 lift-off time, the second way into the residual, the rcData caveat, the FrSky THROTTLE_LOW user, number mismatches. Verdict: ready for commit. No rebuild needed.

### 15. Graph refresh & commit

**Description.** After the review fixes: `graphify update .` and `python tools/graph_labels.py`, then
`/pluto-commit` ( all-target build, FW_Version bump, promotion of PIPELINE_UPDATE.md, CHANGELOG, staged change
and drafted message; the user commits ). API unchanged.

- **Depends on:** 14
- **Skills / agent:** graphify, pluto-commit
- **Files:** -
- **Safety impact:** none
- **Done when:** change staged with a drafted message; the user commits.
- **Rollback:** n/a
- **Status:** done 2026-10-27
- **Result:** Graph refreshed with graphifyy 0.9.96 ( 8762 nodes, labelled ); all three targets built at FW 3.15.2 ( PRIMUS_X2_v1 121.1 KB / 20.5 KB, PRIMUSX2 120.9 KB / 20.7 KB, PRIMUS_V5 121.1 KB / 20.5 KB; no new `src/` warnings ); PIPELINE_UPDATE.md promoted into Core_PID_Controller, RC_Input_Pipeline, Mixer_Pipeline, Altitude_Hold_Estimator, FLIGHT_INVARIANTS and CLAUDE.md; CHANGELOG Fixed bullet and version line; change staged with a drafted message.

## Decisions log

Newest last. One line each: `YYYY-MM-DD [decision|assumption|out-of-scope|risk] text`.

- 2026-10-27 [decision] Mode `check`: confirm the cause with a log before any fix. Target PRIMUS_X2_v1.
- 2026-10-27 [decision] Bisect ( user, visual ): `4657cbf` / `d2e900c` level on the same full-stick test, `2a8d59a` leans. `d2e900c` itself is docs-only.
- 2026-10-27 [decision] User observations: lean only on a full stick-down; partial stick-down level; app Land button level; levels again when the stick is centred; steady lean with drift toward the payload; payload under 10 g on an arm.
- 2026-10-27 [decision] Leading hypothesis: THROTTLE_LOW I-term reset at `mw.cpp:895-897` ( raw throttle < 1100 ) made visible by the 30 cm/s shaped descent of `2a8d59a`.
- 2026-10-27 [decision] A temporary read-only I-term accessor in `pid.cpp` and a `Monitor_Print` line in `PlutoPilot.cpp` are allowed as diagnostics; both are removed in task 5.
- 2026-10-27 [decision] A control flight on a rebuilt `4657cbf` with the same log is part of the plan.
- 2026-10-27 [assumption] The app sends a raw throttle below 1100 at full stick-down ( task 2 log shows it ).
- 2026-10-27 [assumption] Flights are in ANGLE + ALT_HOLD ( app AUX3 ), as the user flies now.
- 2026-10-27 [out-of-scope] The firmware fix ( `/pluto-grill fix payload-tilt-on-descent` ), the Land button path, payloads over 30 g, the chuck-arm path.
- 2026-10-27 [risk] The graphify CLI was not on PATH for the scout and the graph is stale ( built at `5e26823` ); the scout used git and grep instead.
- 2026-10-27 [decision] Task 1: the diagnostic line goes in `plutoLoop ( )`, not `onLoopFinish ( )` as first written ( that hook runs once when Developer Mode stops ).
- 2026-10-27 [decision] Task 1 done. The props-off bench check of the fields was not run ( no hardware in this session ); the first flight of task 2 covers it.
- 2026-10-27 [risk] The log line is 123 B worst case against a ~130 B budget: no further field may be added without dropping one.
- 2026-10-27 [decision] Task 2 procedure: per log, 5 s disarmed ( bench check of fields ), hover at 1.5 m, partial stick-down ( ~1/3, ~10 cm/s ) 3 s, centre 5 s, full stick-down 3 s, centre 5 s, repeat once, Land button. log-1 front, log-2 right, log-3 no payload.
- 2026-10-27 [assumption] The board runs `pidController` 1 ( `pidRewrite` ); the log shows it if `IR` / `IP` leave -2..1 in hover with the payload.
- 2026-10-27 [decision] log-1 ( front payload ) read: the THROTTLE_LOW I-term reset is confirmed as the mechanism of the lean on HEAD; partial stick-downs stay level; hypothesis 3 ( motor floor ) killed.
- 2026-10-27 [decision] User notes for log-1: the 10-15 deg swing back after each full stick-down was the pilot's pitch correction; the crash at ~52.5 s was a ground contact after the 4th stick-down.
- 2026-10-27 [decision] log-2 ( right arm ) and log-3 ( no payload ) will still be flown; procedure tightened: climb back to 1.5 m after every stick-down.
- 2026-10-27 [out-of-scope] While disarmed with the app stick centred, the rate I-terms wind to the 256 clamp ( no THROTTLE_LOW with the app ); they are reset at arming in log-1.
- 2026-10-27 [risk] The airframe needs a roll I-term of about +50 counts even with a front payload, so a full stick-down also rolls it 3-6 deg without any side payload. log-3 will show it.
- 2026-10-27 [decision] log-2 ( right arm ) read: same mechanism on roll ( `IR` -57 to -1, 4.5-6 deg roll toward the payload ); a partial stick-down to `Th` 1104 stayed level, so the boundary is exactly `mincheck` 1100.
- 2026-10-27 [assumption] The 15-18 deg swings after centring in log-2 are pilot stick input: the motors drive into the roll. Same as the user's confirmed pitch correction in log-1; not asked again.
- 2026-10-27 [decision] log-3 ( no payload ) read: the bare airframe holds about +35 counts of roll I-term; every full stick-down zeroes it and rolls the craft 2-3 deg. The lean scales with the I-term removed; a payload enlarges an existing effect. Task 2 closed.
- 2026-10-27 [decision] Task 3 build: worktree `../MagisV2-4657cbf` at `4657cbf` with the task 1 diff applied unchanged, `PROJECT=AltX` ( hex `AltX_PRIMUS_X2_v1_3.7.0.hex` ), `plutoide.ini` copied in; flashed from that folder in PlutoIDE so the HEAD hex is not overwritten.
- 2026-10-27 [decision] log-4 procedure: front payload, three full stick-downs of at most 1.5 s from about 2 m ( the old build descends at up to 100 cm/s ).
- 2026-10-27 [decision] The user could not open the worktree in PlutoIDE, so the 4657cbf hex was copied over `Build/PRIMUS_X2_v1/AltX_PRIMUS_X2_v1_3.15.1.hex` in the main folder ( HEAD hex kept as `HEAD-backup_AltX_PRIMUS_X2_v1_3.15.1.hex.bak` ). **Until it is restored or rebuilt, the 3.15.1-named hex in the main folder is the 3.7.0 control build.** Restore before the next HEAD flight.
- 2026-10-27 [decision] log-4 ( `4657cbf`, front payload ) read: the old build zeroes the I-term and leans the same way on a full stick-down; the user saw it only barely because the 100 cm/s drop hid it ( four of eleven stick-downs reached the floor and disarmed ). Hypothesis 2 confirmed; `2a8d59a` exposed the lean, it did not cause it.
- 2026-10-27 [decision] HEAD hex restored over `Build/PRIMUS_X2_v1/AltX_PRIMUS_X2_v1_3.15.1.hex` from the backup; the worktree `../MagisV2-4657cbf` is kept until task 5.
- 2026-10-27 [decision] User asked for one more control build on release `v3.0.0`: added as task 7 ( before task 4 ). Same reset code, same 100 cm/s unramped descent as `4657cbf`.
- 2026-10-27 [decision] The v3.0.0 hex is staged over `Build/PRIMUS_X2_v1/AltX_PRIMUS_X2_v1_3.15.1.hex` ( HEAD hex backed up as `HEAD-backup_AltX_PRIMUS_X2_v1_3.15.1.hex.bak` ). **Until it is restored, the 3.15.1-named hex in the main folder is the v3.0.0 control build. Do not press Build before the flight.**
- 2026-10-27 [decision] log-5 ( `v3.0.0`, front payload ) read: same reset and lean as `4657cbf` and HEAD. The behaviour has shipped since at least `v3.0.0`; this is not a regression of this branch. Task 7 closed.
- 2026-10-27 [decision] HEAD hex restored over `Build/PRIMUS_X2_v1/AltX_PRIMUS_X2_v1_3.15.1.hex` and verified against the HEAD build; the `.bak` deleted. Both worktrees are kept until task 5.
- 2026-10-27 [decision] `/pluto-grill fix payload-tilt-on-descent`: fix tasks appended to this topic ( small fix ). Tasks 4, 5, 6 dropped as superseded; new tasks 8-15.
- 2026-10-27 [decision] Done when: log shows `IP` kept near its hover value and pitch as level as a partial stick-down, plus regression checks ( arm / take-off, Land, stick-down touchdown and lift-off, flip ).
- 2026-10-27 [decision] Scope: airborne in any mode. Design ( user-confirmed ): freeze the `pidRewrite` rate I-term instead of resetting it while THROTTLE_LOW and `ARMED && throttleRaisedSinceArm && !isThrottleStickArmed`; reset everywhere else as today; `throttleStatus` unchanged.
- 2026-10-27 [decision] Rejected: skipping the reset without a freeze ( wind-up after a stick-down touchdown ); gating on BARO_MODE ( wind-up at stick-arm ground idle ).
- 2026-10-27 [out-of-scope] Radio THROTTLE mode after a stick-arm keeps today's reset ( `isThrottleStickArmed` is only cleared by altitude hold, so the floor cannot be told from the air there ). The disarmed I-term wind-up stays out of scope.
- 2026-10-27 [decision] The task 1 diagnostics stay in until the fix is flown ( task 11 ), then task 12 removes them and both worktrees.
- 2026-10-27 [decision] Task 8 done: findings written; 34 of 34 full stick-downs and every partial one ( ten ) across five logs support hypothesis 1.
- 2026-10-27 [decision] Task 9 done. Reviewer should-fix accepted as a documented residual: a wind-up from a tilted partial-stick touchdown is now frozen rather than zeroed by a later full stick-down ( it reached the lift-off before the fix too, unless the pilot went through the bottom of the stick ). Measured in log-7; a ground-contact reset is the follow-up if it shows.
- 2026-10-27 [decision] Case-table wording corrected per review: THROTTLE-mode reset applies to any ARM-switch arm with the throttle low ( radio or app ARM button ); `isThrottleStickArmed` is also cleared by a take-off goal.
- 2026-10-27 [decision] Task 10 done: the gate run from task 9 stands ( no source changed since ); not rebuilt.
- 2026-10-27 [decision] Task 11 plan written: log-6 repeats log-1 with the fix ( pass: `IP` within about +/-15 of hover, mean pitch within 1.5 deg of the partial stick-downs ); log-7 covers take-off, full-stick touchdown and lift-off, the partial-stick touchdown residual ( measured ), a flip and the Land button.
- 2026-10-27 [decision] Task 11 done: fix validated in flight ( log-6 PASS, log-7 PASS ). The reviewer's residual did not show on a level floor; it stays documented in INVESTIGATION.md.
- 2026-10-27 [assumption] The flip is unaffected by the fix: not loggable ( the app offers either the Dev or the Flip button ); the user accepted the review's source check instead of a visual flip.
- 2026-10-27 [decision] Task 12: no separate review of the removal; the remaining diff is byte-identical to the reviewed task 9 fix, and task 14 reviews the whole topic diff.
- 2026-10-27 [decision] Task 13: the FLIGHT_INVARIANTS section and the CLAUDE.md bullet are staged in PIPELINE_UPDATE.md ( section E ) and applied by `pluto-commit`, as flip-althold-regression did; the dev-guide and CLAUDE.md are not edited before the commit.
- 2026-10-27 [assumption] The `spec-to-code-compliance` skill is not available in this session; the staged docs were checked against the source line by line instead ( every cited line, every replace anchor ). The graphify CLI is not on PATH, so flowchart edges rest on source lines.
- 2026-10-27 [decision] `pid_at_min_throttle` default confirmed as 1 ( `config.cpp:369` ); `config.cpp:765` forces 0 only with `retarded_arm`, whose default is 0.
- 2026-10-27 [decision] Task 14 done. The invariant is scoped honestly: in THROTTLE mode after an ARM-switch arm ( radio, or the app ARM button with the stick low ) the reset still runs in flight, because only altitude hold clears `isThrottleStickArmed`. Accepted for this fix ( the user flies ALT_HOLD from the app ); logged as the remaining gap.
- 2026-10-27 [decision] `logs/` ( 7 files, about 2 MB ) stays untracked: `pluto-commit` never stages `log-*.txt`, and no other topic has committed logs.
- 2026-10-27 [decision] Version at commit: FW 3.15.1 → 3.15.2, API 1.8.0 unchanged ( no public API change ).
- 2026-10-27 [decision] graphify was not installed on this machine ( the committed graph was built under `F:\MagisV2` ); the user chose to install it: `uv tool install graphifyy` ( 0.9.96 ). `graphify update .` rebuilt the graph: 8762 nodes, 19643 edges, 331 communities ( was 4230 nodes ); the newer extractor finds about twice the symbols, mostly in `lib/test`.
- 2026-10-27 [decision] User chose to commit the new graph with full labels: `tools/graph_labels.py` run with `GRAPHIFY_VIZ_NODE_LIMIT=10000` through `runpy` ( graph.json 9.0 MB, graph.html 9.2 MB, was 4.0 / 3.8 MB ).
- 2026-10-27 [risk] `python tools/graph_labels.py` fails on every machine: `tools/warnings.py` shadows the standard `warnings` module. Recorded as item 13 in `.claude/TOOLING_BACKLOG.md` with the workaround; not fixed in this topic.
- 2026-10-27 [decision] Closed - pending commit: FW 3.15.1 → 3.15.2 ( patch, a fix ), API 1.8.0 unchanged; all targets built; docs promoted.
- 2026-10-27 [decision] User: the graph only needs `src/main` and `lib/main` ( plus the user file ). `.graphifyignore` extended ( `lib/test`, `.claude`, `.github`, `.agent`, `tools`, `graphify-out`, root md files ); rebuilt and relabelled: 5110 nodes, 13985 edges, 251 communities; graph.json 6.2 MB, graph.html 6.0 MB. graphify's own backup folder inside `graphify-out/` deleted.
