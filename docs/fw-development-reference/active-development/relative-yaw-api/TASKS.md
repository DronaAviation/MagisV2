# Relative Yaw API - Tasks

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md)

## Resume

Read this block first in a new session; read only the task it points to.

| | |
|---|---|
| **Current task** | **Closed - pending commit** ( 15 Oct 2026, FW 3.13.0, API 1.6.0 ). All tasks done ( task 6 partial, task 11 dropped to a follow-up ). |
| **Next step** | The user commits with `git commit -F .git/MAGISV2_COMMIT_MSG.txt`; then fill the hash into this README and the topic table |
| **Open questions** | none |
| **Blocked on** | the user's commit |
| **Last updated** | 2026-10-15 |

## Summary

| | |
|---|---|
| **Mode** | fix |
| **Goal** | `DesiredAngle_setRelative ( AG_YAW, x )` turns the craft x° from its own heading as soon as Developer Mode is on, with no stick input, as `setDesiredAngle` does. |
| **Done when** | In flight with the sticks untouched, one call turns the craft x° within ±5°. Four calls of +90 turn it a further 90° each, cleanly through 0/359. After a pilot yaw it does not snap back. On dev-mode exit it holds its current heading. `setDesiredAngle` is unchanged inside dev mode. |
| **Target** | PRIMUS_X2_v1 ( Rx_ESP ) |
| **Analysis** | Cause not proven. Ranked: H1 heading hold off ( AUX1 not MAG/HEADFREE ) > H2 an unguarded call pattern > H3 a stuck `isMagHoldChanged` latch > H4 a dev re-entry re-base. See [INVESTIGATION.md](INVESTIGATION.md). |
| **Pipeline docs affected** | `subsystems/User_Space_API.md`, `subsystems/Core_PID_Controller.md`; a new `docs/API/` entry for the setpoint API |
| **Order** | 1 → 2 → 3 → 4 → 5 → 6 → 7 → 10 → 8 → 9 |

## Index

| # | Title | Status | Depends on | Skills / agent |
|---|---|---|---|---|
| 1 | Diagnostic flight: prove the cause | done 2026-10-15 | - | pluto-flighttest |
| 2 | Root-cause fix: works on dev-mode entry with no stick | done 2026-10-15 | 1 | pluto-rules, cpp-pro ( optional ) |
| 3 | Relative re-base after a pilot yaw, and the header contract | done 2026-10-15 | 2 | pluto-rules |
| 4 | Dev-mode exit holds the current heading ( both setters ) | done 2026-10-15 | 2 | pluto-rules |
| 5 | Build gate | done 2026-10-15 | 2, 3, 4 | pluto-build |
| 6 | Hardware validation | done 2026-10-15 (partial) | 5 | pluto-flighttest, pluto-log-analyst |
| 7 | Architecture & pipeline docs ( PIPELINE_UPDATE.md, API wiki ) | done 2026-10-15 | 6 | graphify, spec-to-code-compliance |
| 8 | Topic review | done 2026-10-15 | 7 | pluto-reviewer agent |
| 9 | Graph refresh & commit | done 2026-10-15 | 8 | graphify, pluto-commit |
| 10 | `setDesiredAngle ( AG_YAW )` wraps its value into 0..359 | done 2026-10-15 | 7 | pluto-rules, pluto-build, pluto-reviewer agent |
| 11 | `MSP_SET_HEAD` wraps the app heading into 0..359 | dropped (out of scope: pre-existing app path, not made reachable by this topic; follow-up topic) | - | pluto-grill ( new topic ) |

Status values: `todo`, `in-progress`, `done YYYY-MM-DD`, `blocked (<why>)`, `dropped (<why>)`.
Serial numbers are never reused or renumbered.

## Tasks

### 1. Diagnostic flight: prove the cause

**Description.** Write a test `PlutoPilot.cpp` with two parts:

- A one-shot, guarded `DesiredAngle_setRelative ( AG_YAW, 90 )`, fired by a counter a few seconds after dev mode
  comes on.
- Then a `setDesiredAngle ( AG_YAW, <bearing> )` control turn in the same flight.

Log once per user loop, under 130 bytes per tick:

- the heading
- the yaw target ( `getDesiredAngle ( AG_YAW )` or equivalent )
- `FlightMode_Check ( MAGHOLD )` and `FlightMode_Check ( HEADFREE )`
- the yaw stick
- a call counter

Write the procedure with `pluto-flighttest`. It has two runs: AUX1 heading hold explicitly on, and as the user
normally flies. Create `logs/log-1.txt` and open it for the user. The log must settle H1-H4. H1 is shown if heading
hold is off in the failing run and both setters fail.

- **Depends on:** -
- **Skills / agent:** pluto-flighttest ( `pluto-log-analyst` if the log is large )
- **Files:** [PlutoPilot.cpp](../../../../PlutoPilot.cpp) ( test code only, never committed ),
  [mw.cpp](../../../../src/main/mw.cpp#L695-L724)
- **Safety impact:** none ( test code only ). Hover at least 1 m with space to yaw; the pilot can take yaw back with the stick.
- **Done when:** `log-1.txt` shows which hypothesis holds, with the line numbers. The result goes to INVESTIGATION.md
  and the decisions log.
- **Rollback:** n/a ( no firmware change ); restore the template `PlutoPilot.cpp`.
- **Status:** done 2026-10-15
- **Result:** H1 confirmed. log-1 has `Md:0` throughout, and both setters are wiped on the next tick ( the
  `setDesiredAngle` control fails too ). log-2 ( MAGHOLD forced ) turns all four calls with no stick input: 90° in
  about 1.5 s, ±1°, exact queueing. H2 and H3 ruled out; H4 not seen. Details in TESTING.md §1.

### 2. Root-cause fix: works on dev-mode entry with no stick

**Description.** Fix the cause proven in task 1, so that a relative call made in any hook turns the craft with the
sticks centred and dev mode on, with no prior stick input.

- **If H1** ( heading hold off ): ask the user to choose first.
  - Option A: the setter turns MAGHOLD on by itself while dev mode is on, through the `isUserFlightModeSet`
    re-assert in `userCode ( )`.
  - Option B: the API requires heading hold, and the header and wiki say so.

  Log the choice in the decisions log.
- **If H2** ( call pattern ): make the documented contract "one call = one turn" and check the examples.
- **If H3** ( latch ): check the threshold against the gyro scale and fix it for both setters only with the user's
  sign-off, because `setDesiredAngle` is out of scope.

Keep the accumulate-on-`userHeading` rule ( decided ).

- **Depends on:** 1
- **Skills / agent:** pluto-rules; `cpp-pro` agent if the change is self-contained
- **Files:** [FC-Config-Setpoints.cpp](../../../../src/main/API-Src/FC-Config-Setpoints.cpp#L110-L131),
  [FC-Config.h](../../../../src/main/API/FC-Config.h), [mw.cpp](../../../../src/main/mw.cpp#L1115-L1176)
- **Safety impact:** yaw only, while in dev mode. If MAG_MODE gets forced, the flip ( `command.cpp:254` needs
  MAG_MODE ) and HEADFREE stick mapping ( `mw.cpp:418` ) must be re-checked in task 6.
- **Done when:** builds clean on PRIMUS_X2_v1. A bench check ( armed, props off, or on a yaw stand ) shows
  `rcCommand[YAW]` responding to a relative call with no stick input.
- **Rollback:** revert the task's diff. `setDesiredAngle` is untouched by any branch except H3.
- **Status:** done 2026-10-15
- **Result:** Automatic MAG mode for a held user yaw target, plus the review fixes ( latch re-arm keeps the user target,
  `SENSOR_MAG` check, overflow-safe wrap ). Gate clean, 107.5 KB / 17.6 KB, no BLOCKING findings. In log-3 session 1
  ( AUX1 off, no stick ), all 4 calls turned 90° in about 2 s and held ±1°. In HEADFREE ( session 2 ), Ev 1 turned;
  the session then ended in an external collision.

### 3. Relative re-base after a pilot yaw, and the header contract

**Description.** After the pilot yaws and releases the stick, a target that came from the relative setter is re-based
to the nose at release ( `userHeading = heading` ). The craft then does not snap back, and the next relative call is
measured from there.

- Add a flag, set by `DesiredAngle_setRelative` and cleared by `setDesiredAngle`, so the absolute setter keeps its
  return-to-bearing behaviour ( decided ).
- Do the re-base where the stick-release edge is known: the `updateMagHold ( )` latch clear, or a flag from its
  else-branch. Keep pilot-yaw blending itself unchanged ( out of scope ).
- Correct the header text ( it says "stick discards the setpoint" ) to the real behaviour, for both setters.

- **Depends on:** 2
- **Skills / agent:** pluto-rules
- **Files:** [FC-Config-Setpoints.cpp](../../../../src/main/API-Src/FC-Config-Setpoints.cpp),
  [FC-Config.h](../../../../src/main/API/FC-Config.h), [mw.cpp](../../../../src/main/mw.cpp#L695-L724),
  [API-Utils.cpp](../../../../src/main/API-Src/API-Utils.cpp#L124)
- **Safety impact:** yaw only, while in dev mode. The pilot's stick still overrides heading hold at ±300.
- **Done when:** builds clean. A code trace and the bench show:
  - relative target → pilot yaw → release → holds the new nose;
  - absolute target → pilot yaw → release → returns to the bearing ( unchanged ).
- **Rollback:** revert the task's diff; behaviour returns to that of task 2.
- **Status:** done 2026-10-15
- **Result:** Re-base on the armed rising edge of the yaw stick ( ±300 ), applied at the `isMagHoldChanged` re-arm, for
  relative targets only. Flags `isUserHeadingRelative` / `userHeadingStale`. Header stick behaviour corrected for both
  setters. Gate clean ( 107.6 KB / 17.6 KB ). Reviewer: no BLOCKING findings, all 8 traced scenarios OK, 3 notes
  fixed. Flight behaviour is checked in task 6, case 4.

### 4. Dev-mode exit holds the current heading ( both setters )

**Description.** At the dev-mode exit edge in `userCode ( )`, next to `isUserHeadingSet = false` ( `mw.cpp:1170` ),
set `magHold = heading`. Also clear the task-3 relative flag. The craft then holds where it points instead of
finishing a half-done user turn. This applies to both setters ( decided ). The next relative call after re-entry is
measured from the nose, because `isUserHeadingSet` is false.

- **Depends on:** 2
- **Skills / agent:** pluto-rules
- **Files:** [mw.cpp](../../../../src/main/mw.cpp#L1160-L1176)
- **Safety impact:** yaw at the dev-mode exit edge only, which includes an exit caused by RC loss. With heading hold
  off, there is no change in behaviour.
- **Caution ( found in task 1 ):** on Wi-Fi, a single RC frame arriving more than 200 ms late runs the same exit edge
  ( `onLoopFinish` → `onLoopStart` ), about every 3 s. Applying `magHold = heading` there would cut every turn short at
  each flicker. The exit rule must apply only to a real switch-off ( AUX2 out of range ) or to a link loss longer than
  a flicker, not to a `rxIsReceivingSignal ( )` blip. The same flicker also clears `isUserHeadingSet` today, which
  matters for task 2 / H4.
- **Added by the task-2 review:** with AUX1 off, MAG mode is now forced only while `isUserHeadingSet`. A flicker
  clears it, so heading hold drops and `magHold = heading`. The turn stops partway, and the next relative call is
  measured from the nose ( 4 × +90 no longer makes 360 ). This task must also make the user yaw target
  ( `isUserHeadingSet` / `userHeading` ) **survive a flicker**, and clear it only on a real switch-off. The task-3
  flags `userHeadingStale` and `isUserHeadingRelative` must survive with it, and be cleared at a real exit.
- **Done when:** builds clean. In a bench or hover check, switching dev mode off mid-turn stops the turn within one
  loop.
- **Rollback:** delete the added line.
- **Status:** done 2026-10-15
- **Result:** `devSwitchOn` ( AUX2 in range; AUX holds through a signal loss ) separates a real switch-off from a
  flicker. The target and flags are cleared, and `magHold = heading` is set, only on a real switch-off or on the armed
  → disarmed edge. The flicker path no longer clears them, and MAG forcing runs outside `runUserCode` so heading hold
  continues through a flicker. Gate clean ( 107.7 KB / 17.6 KB ). Reviewer: no BLOCKING findings; the disarm-edge
  clear was added from its finding. Flight checks are in task 6, cases 5 and 8.

### 5. Build gate

**Description.** Run `.claude/skills/pluto-build/driver.sh --gate PRIMUS_X2_v1`. There must be no new `src/` warnings
against the baseline, and flash and RAM must fit. Never regenerate the baseline to silence a new warning.

- **Depends on:** 2, 3, 4
- **Skills / agent:** pluto-build
- **Files:** -
- **Safety impact:** none
- **Done when:** the gate passes, and the flash/RAM delta is recorded in the Result.
- **Rollback:** n/a
- **Status:** done 2026-10-15
- **Result:** Clean `--gate PRIMUS_X2_v1`: no new warnings in `src/`. text 109,044 B ( +296 B against the task-1
  build ), bss 16,772 B ( +4 B: the four new one-byte flags ). Flash 107.7 / 256 KB, RAM 17.6 / 40 KB. The 15 "fixed"
  warnings come from earlier topics ( battery, XRanging ).

### 6. Hardware validation

**Description.** Run a test plan with `pluto-flighttest`, results in `TESTING.md`, on PRIMUS_X2_v1. It has six cases:

1. One relative +90 with no stick input, right after dev mode comes on.
2. Four × +90: 360° in total, through 0/359.
3. −90 and a large value ( e.g. +270 ).
4. A pilot yaw mid-sequence: no snap-back, and the next call measured from the nose.
5. Dev mode off mid-turn: holds the heading.
6. A `setDesiredAngle` regression: absolute bearing and return after a pilot yaw.

Task 2 chose option A, so also include:

- an app-flip check while a user target is held ( the flip gate now opens );
- a **full Ev 1-4 sequence in HEADFREE** ( in log-3 only Ev 1 ran before an external collision );
- case 7, **a turn through a Wi-Fi flicker**: an `S:2` marker during a turn must not stop it;
- case 8, **land and disarm with a target held, turn the craft by hand, re-arm with dev mode on**: no yaw at takeoff.

Each new log is a numbered `logs/log-N.txt`.

- **Depends on:** 5
- **Skills / agent:** pluto-flighttest; `pluto-log-analyst` agent for large logs
- **Files:** `TESTING.md`, `logs/`
- **Safety impact:** test flights. Hover at least 1 m in open space; the pilot keeps yaw override and dev-mode switch ready.
- **Done when:** every case passes within ±5° of the commanded heading and settles within 3 s per 90°. Any failure
  becomes a new task.
- **Rollback:** fly the last committed firmware ( `edfbd61` ).
- **Status:** done 2026-10-15 (partial, accepted by the user)
- **Result:** log-4 ( AUX1 off ) passes cases 1-3: 4 × +90 = 360 back to H0 exactly, a clean wrap, ±1° within about
  2 s, and +270 flown the short way. It also passes the absolute turn and the disarm clear ( twice ). Cases 4, 6
  ( hand yaw ), 5, 7 and 8, the flip and flight B ( HEADFREE, log-5 empty ) were not flown; the user accepted. The
  two heading jumps at takeoff with no target are existing behaviour.

### 7. Architecture & pipeline docs ( PIPELINE_UPDATE.md, API wiki )

**Description.** Stage new versions in `PIPELINE_UPDATE.md` ( never edit the pipeline folder directly ):

- `User_Space_API.md`: yaw setpoint semantics for both setters, the relative call, stick and dev-exit behaviour.
- `Core_PID_Controller.md`: the heading-hold path through `updateMagHold ( )`, with a Mermaid flow in which every
  edge is confirmed with `graphify path` or a source line.

Then write the `docs/API/` wiki entry for `DesiredAngle_setRelative` ( create the setpoint API page if none exists ).
State in the wiki that heading hold is automatic for a held user yaw target, and when the target is dropped: at a
real dev-mode switch-off or a disarm, but not on a Wi-Fi flicker or a long RC loss ( the target stays through a
failsafe LAND ). Also state that a yaw setter called inside `onLoopFinish ( )` is wiped at a real switch-off. While one is held,
`FlightMode_Check ( MAGHOLD )` and the app's BOXMAG read true, and the app flip is accepted. Check the code against
the docs with `spec-to-code-compliance`. Add a one-line CLAUDE.md rule if a new gotcha came
up ( e.g. "user yaw needs MAG or HEADFREE" ). The `API_Version` bump to 1.6.0 is done at commit.

- **Depends on:** 6
- **Skills / agent:** graphify, spec-to-code-compliance
- **Files:** `PIPELINE_UPDATE.md`, `docs/API/`, [CLAUDE.md](../../../../CLAUDE.md)
- **Safety impact:** none
- **Done when:** every statement in the docs matches the code ( spec-compliance shows no divergence ).
- **Rollback:** n/a
- **Status:** done 2026-10-15
- **Result:** `docs/API/YAW_HEADING_API_WIKI.md` ( new, API 1.6.0 ) and PIPELINE_UPDATE.md, which stages:
  - `User_Space_API.md`: a Yaw setpoints section and the user-code flowchart tail;
  - `Core_PID_Controller.md`: a Heading hold section and flowchart;
  - a CLAUDE.md invariant and the CHANGELOG.

  Diagram edges were checked against mw.cpp lines. The spec-compliance run ( wf_03ee5f16-449 ) passed all 8 checked
  requirements with no divergence. Its doc findings were fixed:
  - the `RcCommand_Set` claim was qualified in the wiki, header and `mw.cpp` comment;
  - the swing-back after a call made during a stick hold is documented;
  - the stick threshold is stated as 300 / 60 %;
  - the "Developer Mode switch" wording is made explicit;
  - the `rxfail` caveat is noted;
  - the CLAUDE.md line now carries the no-magnetometer exception.

  The docs describe the `setDesiredAngle` yaw wrap that **task 10** adds, so they ship only together with it.

### 8. Topic review

**Description.** Run the `pluto-reviewer` agent on the whole topic diff ( `FC-Config-Setpoints.cpp`, `FC-Config.h`,
`mw.cpp`, `API-Utils.cpp`, docs ). Fix the BLOCKING findings and re-run the gate before moving on.

- **Depends on:** 7
- **Skills / agent:** pluto-reviewer agent ( subagent, never inline )
- **Files:** topic diff
- **Safety impact:** none
- **Done when:** no open BLOCKING findings, and the gate passes after the fixes.
- **Rollback:** n/a
- **Status:** done 2026-10-15
- **Result:** `pluto-reviewer` on the whole diff: no BLOCKING code findings, all cross-task interactions traced clean,
  every Mermaid edge real, gate clean ( 108.0 KB / 17.6 KB, the test `PlutoPilot.cpp` included ). Fixed:
  - the docs no longer claim `FlightMode_Check ( MAGHOLD )` / BOXMAG always read true ( both are sampled before the
    re-force in the same pass );
  - the PIPELINE_UPDATE line references are refreshed;
  - the "disarmed reset" now names its MAG/HEADFREE condition;
  - the header notes the `onLoopStart` re-run after a hiccup.

  **Carried to task 9:** restore the template `PlutoPilot.cpp` ( `git checkout -- PlutoPilot.cpp` ) before staging,
  and the version bump to API 1.6.0 and FW in `pluto-commit`.

### 9. Graph refresh & commit

**Description.** Run `graphify update .` and `python tools/graph_labels.py` after the review fixes, then
`/pluto-commit`, which does the following:

- builds all targets;
- bumps the FW version and `API_Version` 1.5.0 → 1.6.0;
- applies `PIPELINE_UPDATE.md` and adds a `CHANGELOG.md` entry;
- stages the change and drafts the message.

The test `PlutoPilot.cpp` stays out of the commit. Remove or fill the empty `yaw-api-test/` stub folder.

- **Depends on:** 8
- **Skills / agent:** graphify, pluto-commit
- **Files:** Makefile, `CHANGELOG.md`, pipeline docs, `graphify-out/`
- **Safety impact:** none
- **Done when:** all targets build, the change is staged, and the drafted message is shown. The user commits.
- **Rollback:** n/a
- **Status:** done 2026-10-15
- **Result:** Graph refreshed ( 4,124 nodes, `DesiredAngle_setRelative` present ). `PlutoPilot.cpp` was restored after
  the test code was saved to `test/`. All targets built clean ( no new `src/` warnings ). FW 3.12.0 → 3.13.0 and
  API 1.5.0 → 1.6.0. PIPELINE_UPDATE was applied to `User_Space_API.md`, `Core_PID_Controller.md` and CLAUDE.md,
  with the CHANGELOG entries added. The change is staged with a drafted message; the user commits.

### 10. `setDesiredAngle ( AG_YAW )` wraps its value into 0..359

**Description.** Found while writing the task-7 wiki. `setDesiredAngle`'s `AG_YAW` case stores `angle` into
`magHold` / `userHeading` unwrapped. `updateMagHold ( )` corrects the error by 360 only once, so a target of 720 or
more, or −361 or less, never settles and **the drone spins without stopping**. Values from −360 to 719 settle at the
wrapped bearing. This is not new, but task 2 made it reachable without the app's heading-hold switch, and the old
header told users yaw was in deci-degrees ( `900` for 90.0° spins ). The fix stores `angle % 360`, made positive, as
`DesiredAngle_setRelative` already does. In-range values behave identically. The user chose this over documenting
the limit.

- **Depends on:** 7
- **Skills / agent:** pluto-rules; pluto-build `--gate PRIMUS_X2_v1`; pluto-reviewer agent
- **Files:** [FC-Config-Setpoints.cpp:98-105](../../../../src/main/API-Src/FC-Config-Setpoints.cpp#L98-L105)
- **Safety impact:** yaw target value only. It removes a continuous-spin input. Values 0..359 are unchanged.
- **Done when:** builds clean with no new warnings. A code trace shows 900 → 180, 450 → 90, −90 → 270, 359 → 359.
  The wiki and PIPELINE_UPDATE ( already written for the wrapped behaviour ) match the code.
- **Rollback:** revert the three lines.
- **Status:** done 2026-10-15
- **Result:** The `AG_YAW` case wraps `angle % 360` ( made positive ) and stores it with explicit `int16_t` casts.
  The trace: 900 → 180, 450 → 90, −90 → 270, 720 → 0, −361 → 359, INT32 extremes → 127 / 232, and all of them settle.
  0..359 is unchanged. The gate is clean, and 2 baseline warnings are fixed. Reviewer: no BLOCKING or SHOULD-FIX
  findings. Its note about the header `@param` is fixed; its note about `MSP_SET_HEAD` became task 11 ( dropped as out
  of scope ).

### 11. `MSP_SET_HEAD` wraps the app heading into 0..359

**Description.** From the task-10 review. `serial_msp.cpp:1409` stores `magHold = read16 ( )` unwrapped. A value of
720 or more ( or above 32767, which reads negative ) never settles while MAG mode is on and no user heading is held.
This path predates the topic and this topic did not make it more reachable: it acts only with the pilot's AUX1 MAG,
and `mw.cpp:1177` overwrites it whenever user code holds a target.

- **Depends on:** -
- **Skills / agent:** pluto-grill ( a new `fix` topic )
- **Files:** [serial_msp.cpp:1409](../../../../src/main/io/serial_msp.cpp#L1409)
- **Safety impact:** yaw, for app-commanded headings only.
- **Done when:** handled in its own topic.
- **Rollback:** n/a
- **Status:** dropped (out of scope: pre-existing app path, not made reachable by this topic; follow-up topic)
- **Result:** -

## Decisions log

Newest last. One line each: `YYYY-MM-DD [decision|assumption|out-of-scope|risk] text`.

- 2026-10-15 [decision] Mode `fix`; target PRIMUS_X2_v1; the empty `relative-yaw-api/` stub is planned in place.
- 2026-10-15 [decision] Done when: repeated relative turns work ( each +90 a further 90°, clean at 0/359 ) with no stick input after dev mode is on.
- 2026-10-15 [decision] A repeat call before the turn has finished adds to the target ( base = latched `userHeading` ), so turns queue exactly.
- 2026-10-15 [decision] After a pilot yaw, a relative target is re-based to the nose at stick release ( no snap-back ); `setDesiredAngle` keeps returning to its bearing ( a flag records which setter wrote the target ).
- 2026-10-15 [decision] On dev-mode exit `magHold = heading` for both setters: hold where the nose points and drop the target.
- 2026-10-15 [decision] The call may be made from any hook; the user's code times it with a timer or counter.
- 2026-10-15 [out-of-scope] `setDesiredAngle` semantics ( apart from the dev-exit rule ), yaw PID and rate tuning, blending of pilot yaw.
- 2026-10-15 [assumption] `setDesiredAngle` works without a stick input in the user's setup; checked by the control turn in task 1.
- 2026-10-15 [risk] If heading hold ( MAG/HEADFREE ) is the missing piece, forcing MAGHOLD from the setter affects the flip ( `command.cpp:254` ) and HEADFREE stick mapping; the user chooses in task 2.
- 2026-10-15 [decision] Task 1 result: H1 confirmed ( log-1 `Md:0`, both setters wiped; log-2 with MAGHOLD forced, all turns clean ). The relative setter's math is correct; the fix is to get heading hold on for user yaw.
- 2026-10-15 [decision] Task 2 ( the H1 fix ): option A for both setters. While dev mode is on and `isUserHeadingSet`, `userCode ( )` enables `MAG_MODE` on every loop, unless the pilot selected HEADFREE. `processRx ( )` drops it from AUX1 when the target goes. Accepted side effect: the app flip ( it needs MAG_MODE ) works while a user yaw target is held.
- 2026-10-15 [decision] Task-2 review: the `isMagHoldChanged` re-arm restores the user target ( `userHeading` ) instead of the heading when one is held. This applies to both setters and removes a one-loop loss ( or a full loss at fast user loop rates ). Task 3 adds the relative re-base at that point.
- 2026-10-15 [decision] Task 3: the relative re-base is keyed on the rising edge of the yaw stick, only while armed ( a gesture on the ground does not count ). The `isMagHoldChanged` re-arm alone also fires when MAG mode switches on, and must not erase the first turn. `rcData[YAW]` is kept: for yaw it is the pilot's stick, because user code never writes it.
- 2026-10-15 [decision] Task 4: a real switch-off is told from a flicker by the dev AUX range ( AUX channels hold through a signal loss; default `RX_FAILSAFE_MODE_HOLD` ). The user yaw target and flags are dropped, and `magHold = heading` set, only on a real switch-off or on the armed → disarmed edge ( review finding ). A long RC loss keeps the target through the failsafe LAND; this is accepted as yaw only, and matches the AUX1-MAG behaviour.
- 2026-10-15 [decision] Task 6 accepted as partial ( user's choice ): log-4 proves cases 1-3, the absolute turn and the disarm clear.
- 2026-10-15 [risk] Not flight-tested: the hand-yaw re-base and return ( cases 4, 6 ), dev off mid-turn ( 5 ), a turn through a flicker ( 7 ), no yaw at takeoff after a disarm ( 8 ), the app flip while a target is held, and the full sequence in HEADFREE. These rest on the reviewer's code traces; flight-check them before relying on them.
- 2026-10-15 [decision] Task 10 added: `setDesiredAngle ( AG_YAW )` will wrap its value into 0..359 ( the user's choice over documenting it ). Values of 720 or more, or −361 or less, spun the drone, and the auto heading hold from task 2 made that reachable.
- 2026-10-15 [decision] Task 7: the spec-compliance findings were fixed in the docs, not in the code. A relative call made during a stick hold keeps its call-time target ( deliberate, now documented ). `RcCommand_Set` yaw with a held yaw target is documented as "do not mix" rather than blocked in code.
- 2026-10-15 [out-of-scope] Task 11 ( `MSP_SET_HEAD` stores the app heading unwrapped, with the same spin risk ) is dropped from this topic: it is a pre-existing app path that this topic did not make more reachable. Candidate for its own `fix` topic.
- 2026-10-15 [assumption] REFUTED: "`setDesiredAngle` works without a stick input". It fails identically with heading hold off ( log-1 Ev 2 ).
- 2026-10-15 [risk] Wi-Fi RC flickers ( a frame more than 200 ms late, about every 3 s ) run `onLoopFinish` / `onLoopStart` and clear `isUserHeadingSet`. Task 4's exit rule must not fire on a flicker, and H4 may matter more than ranked.
- 2026-10-15 [risk] The `isMagHoldChanged` gyro threshold ( `|gyroADC[YAW]/4| < 70`, ~17°/s at 16.4 LSB/dps ) is unchecked for this IMU.
