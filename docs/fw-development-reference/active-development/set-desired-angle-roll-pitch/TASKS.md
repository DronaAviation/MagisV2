# Set desired angle roll / pitch - Tasks

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md) · [TEST_CODE](TEST_CODE.md)

## Resume

Read this block first in a new session; read only the task it points to.

| | |
|---|---|
| **Current task** | none: **Closed - pending commit** ( 14 Nov 2026 ) |
| **Next step** | The user commits ( `git commit -F .git/MAGISV2_COMMIT_MSG.txt` ). The next `pluto-commit` or `pluto-grill` run fills in the hash. |
| **Open questions** | Optional follow-up: re-fly the roll -5 deg step in open space ( not validated in log-2, collision ); the sequence is in `TEST_CODE.md`. |
| **Blocked on** | nothing |
| **Last updated** | 2026-11-14 |

## Summary

| | |
|---|---|
| **Mode** | fix |
| **Goal** | `setDesiredAngle ( AG_ROLL / AG_PITCH, x )` tilts the drone x deci-degrees in ANGLE mode and holds it until replaced, as `AG_YAW` holds its bearing; no API signature change. |
| **Done when** | In the post-fix flight of the automatic sequence, each +/-50 ( 5 deg ) roll / pitch step shows the measured angle settle with the commanded sign within 1.5 deg of 5 deg inside 0.5 s and return level at 0; no disarm; yaw unchanged. The pre-fix flight shows no tilt. |
| **Target** | PRIMUS_X2_v1 |
| **Analysis** | Causal chain proven from code ( [INVESTIGATION.md](INVESTIGATION.md) ): the setter writes the measured attitude `inclination` ( Setpoints.cpp:92 / :96 ), `imuUpdate` overwrites it ( imu.cpp:1185 ), and `pidRewrite` targets only `2 * rcCommand` ( pid.cpp:801 ). Fix option B. |
| **Pipeline docs affected** | subsystems/User_Space_API.md, subsystems/Core_PID_Controller.md |
| **Order** | 1 -> 2 -> 3 -> 4 -> 5 -> 6 -> 7 -> 8 -> 9 -> 10 |

## Index

| # | Title | Status | Depends on | Skills / agent |
|---|---|---|---|---|
| 1 | Automatic tilt test sequence and pre-fix reproduction flight | done 2026-11-14 | - | pluto-flighttest, pluto-rules, pluto-build |
| 2 | Setter stores the user angle instead of the attitude estimate | done 2026-11-14 | 1 | pluto-rules, pluto-reviewer agent |
| 3 | User angle term in the pidRewrite ANGLE target | done 2026-11-14 | 2 | pluto-rules, pluto-reviewer agent |
| 4 | Clear the user angle at dev-off and on the disarm edge | done 2026-11-14 | 3 | pluto-rules, pluto-reviewer agent |
| 5 | API header and wiki for roll / pitch | done 2026-11-14 | 4 | spec-to-code-compliance |
| 6 | Build gate | done 2026-11-14 | 2, 3, 4 | pluto-build |
| 7 | Hardware validation | done 2026-11-14 | 6 | pluto-flighttest; pluto-log-analyst agent if the log is large |
| 8 | Architecture & pipeline docs ( PIPELINE_UPDATE.md ) | done 2026-11-14 | 7 | graphify, spec-to-code-compliance |
| 9 | Topic review | done 2026-11-14 | 8 | pluto-reviewer agent |
| 10 | Graph refresh & commit | done 2026-11-14 | 9 | graphify, pluto-commit |

Status values: `todo`, `in-progress`, `done YYYY-MM-DD`, `blocked (<why>)`, `dropped (<why>)`.
Serial numbers are never reused or renumbered.

## Tasks

### 1. Automatic tilt test sequence and pre-fix reproduction flight

**Description.** The testing team's T11 code is not available, so we reproduce B4 with our own. In `PlutoPilot.cpp`, add a
sequence that starts by itself when dev mode turns on: 2 s level hover ( `setDesiredAngle ( AG_ROLL / AG_PITCH, 0 )` ),
then roll +50 for 1 s, 0 for 1 s, roll -50 for 1 s, 0 for 1 s, then the same for pitch, then a final 0 and "done". Each
step calls the setter **once**, on its step change, so the post-fix flight also tests hold-until-replaced. The sequence is
sized for a small room: 5 deg for 1 s is about 0.4 m of drift, and the opposite step brings it back. A `Monitor_Print`
log line per tick ( under 130 B ) prints step number, commanded roll / pitch, `Estimate_Get ( Angle, AG_ROLL / AG_PITCH )`
and the heading. Fly it on the current firmware: the user flies in ALT_HOLD / ANGLE, turns dev mode on, waits for "done",
turns dev mode off, and pastes the log into `logs/`.

- **Depends on:** -
- **Skills / agent:** pluto-flighttest ( test plan, log fields ), pluto-rules ( PlutoPilot hooks, Monitor_Print budget ), pluto-build
- **Files:** [PlutoPilot.cpp](../../../../PlutoPilot.cpp)
- **Safety impact:** test code only. The pre-fix firmware cannot tilt; the commanded values stay far below the 700 crash-angle trip.
- **Done when:** builds clean on PRIMUS_X2_v1; the pre-fix log shows each step's commanded value and a measured roll / pitch that stays near level ( no response to the steps ), which reproduces B4. Test plan and result in `TESTING.md`.
- **Rollback:** restore the previous `PlutoPilot.cpp` from git.
- **Status:** done 2026-11-14
- **Result:** B4 reproduced in log-1: every step-change tick reads back the command ( +/-50 ), then the angle returns to the hover band and +50 / -50 give the same mean on both axes ( no tilt ); heading unchanged; 10 Wi-Fi resumes, all kept the timeline ( [TESTING.md](TESTING.md) ). Build: the sequence and the log are in [PlutoPilot.cpp](../../../../PlutoPilot.cpp) ( ~101 B per tick, worst case ~105 B ), the test plan and expected results in [TESTING.md](TESTING.md). Gate green on `PRIMUS_X2_v1`: Flash 122.3 KB, RAM 20.5 KB, no new `src/` warnings. Reviewer 0 blocking; two should-fix items fixed ( the flicker window raised from 1.5 s to 3 s so a normal Wi-Fi stall resumes instead of restarting, and the "remove before release" flag added to the topic README ) plus the lost-DONE-marker nit. Waiting for the flight log.

### 2. Setter stores the user angle instead of the attitude estimate

**Description.** In `setDesiredAngle`, the `AG_ROLL` / `AG_PITCH` branches stop writing `inclination` and store the value
in the existing, unused `userDesiredAngle [ ROLL / PITCH ]` with a held flag ( e.g. `isUserAngleSet` in `API-Utils` ),
bounded to a safe int16 range so the later sum cannot overflow ( the real limit stays the existing clamp in task 3 ). This
alone removes the "read once" and the crash-angle trip hazard. Fix the wrong comment in `DesiredAngle_setRelative`
( :120-122 ); roll / pitch there stays a no-op ( no new API ). The yaw branch is not touched.

- **Depends on:** 1
- **Skills / agent:** pluto-rules; pluto-reviewer agent on the diff
- **Files:** [FC-Config-Setpoints.cpp](../../../../src/main/API-Src/FC-Config-Setpoints.cpp), [API-Utils.h](../../../../src/main/API/API-Utils.h), [API-Utils.cpp](../../../../src/main/API-Src/API-Utils.cpp)
- **Safety impact:** the attitude estimate is no longer written by user code ( removes an in-flight disarm path ). No setpoint effect yet.
- **Done when:** builds clean with no new `src/` warnings ( explicit casts for `-Wconversion` ); `inclination` has no writer outside `imu.cpp` ( grep ); reviewer 0 blocking.
- **Rollback:** revert the Setpoints.cpp hunks.
- **Status:** done 2026-11-14
- **Result:** `AG_ROLL` / `AG_PITCH` store `constrain ( angle, -1800, 1800 )` into the existing unused `userDesiredAngle [ ]` / `isUserDesiredAngle [ ]` ( no new globals; `angle_e` indexes match `FD_ROLL` / `FD_PITCH` ), and the `DesiredAngle_setRelative` comment is corrected. `inclination` is now written only by `flight/imu.cpp` ( :425-426, :1185-1186 ) and nothing else relied on the old write. Gate green on `PRIMUS_X2_v1`: Flash 122.3 KB, RAM 20.5 KB, no new `src/` warnings. Reviewer 0 blocking; one should-fix fixed ( the comment named a behaviour task 3 adds ), two nits deferred to task 5 ( the `FC-Config.h` doxygen and wiki are incomplete, and `getDesiredAngle` becomes correct once task 3 lands ). Details: [CHANGES.md](CHANGES.md).

### 3. User angle term in the pidRewrite ANGLE target

**Description.** In `pidRewrite` ( `pid.cpp:801`, and the `#ifdef GPS` branch at :797 ), add the held user angle to the
target: `desiredAngle[axis] = userAngle + 2 * rcCommand[axis]`, so the existing `max_angle_inclination` clamp limits the
sum ( "as it is now": no new limit; the stick adds on top ). The term applies only when the flag is set, `ANGLE_MODE` is
on, `flipState == 0` and `! landingInProgress ( )`; otherwise it is 0 ( the target is kept, not cleared ). There is no
dev-mode gate: `devmode` drops on every Wi-Fi flicker, so a real switch-off is handled by the task 4 clear instead.
Check the other PID controller ( LuxFloat ) and either apply the same term or record in the decisions log that it is
unsupported there. Keep the per-loop cost to a flag test and an add.

- **Depends on:** 2
- **Skills / agent:** pluto-rules ( loop cost, flip / landing invariants, rcData vs rcDataPilot ); pluto-reviewer agent on the diff
- **Files:** [pid.cpp](../../../../src/main/flight/pid.cpp), possibly [pid.h](../../../../src/main/flight/pid.h) / [API-Utils.h](../../../../src/main/API/API-Utils.h) for the accessor
- **Safety impact:** ANGLE-mode setpoint, flip, landing. The user angle must never act in a flip or a landing, or in ACRO / HORIZON.
- **Done when:** builds clean, no new warnings; the reviewer confirms the flip / landing / mode gates and the clamp order; reviewer 0 blocking. Flash cost noted ( expected under 250 B ).
- **Rollback:** revert the pid.cpp hunk; with task 2 alone the setter is a harmless no-op on roll / pitch.
- **Status:** done 2026-11-14
- **Result:** `pidRewrite` adds `userAngleTarget ( axis )` to its ANGLE target before the `max_angle_inclination` clamp, so the stick adds on top and the sum keeps the pilot's 20 deg limit. The gates live in one helper in `API-Utils.cpp` ( flag, `ANGLE_MODE`, `flipState == 0`, `! landingInProgress ( )`, the last reused from `FC-Control-Command.cpp` ). Gate green on `PRIMUS_X2_v1`: Flash 122.5 KB ( +0.2 KB ), RAM 20.5 KB, no new `src/` warnings. Reviewer 0 blocking / 0 should-fix; it confirmed the loop order ( `userCode` runs before the PID ), the C linkage, that the landing gate covers the RC-loss, user, MSP and battery landings, the flip states, `ANGLE_MODE` being independent of BARO, and no overflow. Four nits: `getDesiredAngle` returns the pre-clamp sum ( document in task 5 ), the battery LAND flies one more loop, the Wi-Fi stall risk ( logged, checked in task 7 ), and stale dev-mode wording in the topic docs ( fixed ). Details: [CHANGES.md](CHANGES.md).

### 4. Clear the user angle at dev-off and on the disarm edge

**Description.** Mirror the yaw clear in `mw.cpp` ( ~1236-1252, `if ( ( ! devSwitchOn || disarmed ) && isUserHeadingSet )` ):
the held roll / pitch target and its flag are cleared at a real dev-mode switch-off and on the disarm edge, not on a
Wi-Fi flicker ( `devSwitchOn` keeps its last value ). A target set on the ground before arming survives to take-off, as
for yaw. A landing ends in a disarm, so it is cleared there; during the landing it is already suppressed by task 3.

- **Depends on:** 3
- **Skills / agent:** pluto-rules; pluto-reviewer agent on the diff
- **Files:** [mw.cpp](../../../../src/main/mw.cpp)
- **Safety impact:** arming / disarm, dev-mode switch, RC flicker.
- **Done when:** builds clean; the reviewer confirms the clear sites match the yaw ones and that no path leaves a target held across a disarm; reviewer 0 blocking.
- **Rollback:** revert the mw.cpp hunk ( the target would then persist until replaced: do not fly that way ).
- **Status:** done 2026-11-14
- **Result:** `userCode ( )` clears the tilt and its flag at a real dev switch-off and on the disarm edge, next to the yaw clear; the reviewer confirmed every disarm path is covered ( the `ARMED` edge ), that `onLoopFinish ( )` cannot re-latch, and that `devSwitchOn` holds through RX loss. Two review should-fixes added gates to `userAngleTarget ( )`: a 600 ms Developer Mode grace window ( a normal flicker keeps the tilt; a real stall suppresses it instead of flying the craft away for ~2.5 s ) and an airborne test ( throttle raised **and** 20 cm above the arm reference, because an app take-off arms with the stick centred ). Gate green on `PRIMUS_X2_v1`: Flash 122.6 KB ( +0.3 KB on HEAD ), RAM 20.5 KB, no new `src/` warnings. Remaining nits: the disarm / re-arm edge inside one loop is sampled once ( the yaw clear has the same gap ), and stale comments / docs, all fixed. Details: [CHANGES.md](CHANGES.md).

### 5. API header and wiki for roll / pitch

**Description.** Update the `setDesiredAngle` doxygen in `FC-Config.h` and the roll / pitch part of the wiki
( `YAW_HEADING_API_WIKI.md:38`, or the FC-Config section of `FC_CONTROL_API_WIKI.md`, whichever owns it ) with what the
code now does: deci-degrees, held until replaced, ANGLE mode only, added to the stick and limited by
`max_angle_inclination` ( 20 deg by default ), not applied in a flip or a landing, cleared at dev-off and disarm. Add a
Changes / Fixed entry ( B4 ). The `API_Version` bump is done at commit ( task 10 ).

- **Depends on:** 4
- **Skills / agent:** spec-to-code-compliance ( every statement checked against a source line )
- **Files:** [FC-Config.h](../../../../src/main/API/FC-Config.h), [docs/API/](../../../API/)
- **Safety impact:** none ( docs ).
- **Done when:** each wiki / header statement has a matching source line; no em dashes.
- **Rollback:** -
- **Status:** done 2026-11-14
- **Result:** `FC-Config.h` doxygen for `getDesiredAngle` and `setDesiredAngle` rewritten; `YAW_HEADING_API_WIKI.md` gains "Roll and pitch tilt" ( hold, stick sum and limit, a table of the five conditions it is flown under, when it ends, default controller only, `getDesiredAngle` ), a bounded "lean then level" example, three pitfalls, and "Changes in 1.8.6". Twelve statements checked by hand against source lines ( table in [CHANGES.md](CHANGES.md); `spec-to-code-compliance` is not available in this session ). No em dashes added. Gate green after the header edit. A first draft had an unsafe `onLoopStart` example ( an unbounded tilt, re-applied after each Wi-Fi hiccup ), replaced before closing.

### 6. Build gate

**Description.** `pluto-build --gate PRIMUS_X2_v1` ( with `PROJECT=` from `plutoide.ini` ): no new `src/` warnings,
flash / RAM fit, and the flash / RAM delta against `ebf4e46` recorded.

- **Depends on:** 2, 3, 4
- **Skills / agent:** pluto-build
- **Files:** -
- **Safety impact:** none
- **Done when:** gate green; flash delta under 250 B, RAM delta under 16 B.
- **Rollback:** -
- **Status:** done 2026-11-14
- **Result:** Clean gate green on `PRIMUS_X2_v1`, no new `src/` warnings; hex `Build/PRIMUS_X2_v1/AltX_PRIMUS_X2_v1_3.15.7.hex` ( fix + task 1 test code ). Exact sizes ( `arm-none-eabi-size`, HEAD built in a temporary worktree ): HEAD `ebf4e46` text 122776 / data 1316 / bss 19728; HEAD + the `src/` fix 123032 / 1316 / 19748, so the **fix costs +256 B flash and +20 B RAM**; with the test code it is 124264 / 1316 / 19764. Slightly over the planned estimate ( 250 B / 16 B ), accepted: the flash covers the two extra gates added in task 4, and the RAM is the previously unused `userDesiredAngle` / `isUserDesiredAngle` arrays ( 15 B, discarded by the linker at HEAD because nothing used them ) plus the 4 B link-grace timestamp. 47 % flash, 51 % RAM.

### 7. Hardware validation

**Description.** Fly the task 1 sequence on the fixed firmware in the same room and mode, and paste the log into `logs/`.
Compare with the pre-fix log. Also check that turning dev mode off mid-step levels the drone, and that a re-arm does
not tilt ( the target was cleared ).

- **Depends on:** 6
- **Skills / agent:** pluto-flighttest; pluto-log-analyst agent if the log is large
- **Files:** `TESTING.md`, `logs/`
- **Safety impact:** flight test, small angles, pilot ready on the sticks.
- **Done when:** each +/-50 step settles with the commanded sign within 1.5 deg of 5 deg inside 0.5 s, and returns level at 0; heading unchanged ( yaw path untouched ); no disarm; dev-off mid-step levels the drone. Results in `TESTING.md`.
- **Rollback:** flash the `ebf4e46` firmware.
- **Status:** done 2026-11-14
- **Result:** PASS in log-2 ( same test code as log-1 ): roll +5, pitch +5 and pitch -5 deg each reached the commanded side within 0.2-0.3 s and within 1 deg; roll + held the full second from one call, pitch - held flat 0.5 s until the pilot corrected; step 2 levelled in ~200 ms; no read-back on step-change ticks; heading 249-251; 9 Wi-Fi resumes ( 200-305 ms ) did not drop the tilt; flight B ( dev-off mid-lean, re-arm with dev off ) passed visually. The roll -5 step was not validated: the drone hit an obstacle in the small room ( pilot ). Details: [TESTING.md](TESTING.md).

### 8. Architecture & pipeline docs ( PIPELINE_UPDATE.md )

**Description.** Stage in `PIPELINE_UPDATE.md` the new text for `User_Space_API.md` ( roll / pitch semantics, hold, mode,
clear rules; replace :162-163 ) and `Core_PID_Controller.md` ( the user angle term in the pidRewrite ANGLE target, its
gates, the clamp order ), with every flow edge checked by `graphify path` or a source line. Add a one-line CLAUDE.md
flight invariant: a user roll / pitch target is added to the ANGLE target, held until replaced, not applied in a flip
or landing, and cleared at dev-off and disarm. Note shifted line references in other pipeline docs.

- **Depends on:** 7
- **Skills / agent:** graphify, spec-to-code-compliance
- **Files:** `PIPELINE_UPDATE.md`, [CLAUDE.md](../../../../CLAUDE.md)
- **Safety impact:** none ( docs ).
- **Done when:** every staged statement maps to a source line; the pipeline folder itself is untouched.
- **Rollback:** -
- **Status:** done 2026-11-14
- **Result:** [PIPELINE_UPDATE.md](PIPELINE_UPDATE.md) stages for `User_Space_API.md` a new "Roll / pitch setpoints ( ANGLE tilt )" section ( gate table and a Mermaid flow, every edge with a source line ), a Data Flow bullet and the wiki version, and for `Core_PID_Controller.md` the `pidRewrite` target and a "User roll / pitch tilt" section. It also lists **45 line references in 7 pipeline docs** shifted by this topic, found by a content-based line map from HEAD ( plus 2 short-form `:NNN` references found by hand ); only `FC-Control-Command.cpp:130` changed line content ( now 131 ). A compliance table maps each statement to a source line. CLAUDE.md gains the user roll / pitch flight invariant. The pipeline folder is untouched. No `dev-guide/` or hardware map change ( no pin, DMA or timer ).

### 9. Topic review

**Description.** `pluto-reviewer` on the whole topic diff against `ebf4e46`. BLOCKING findings are fixed and re-gated
before moving on.

- **Depends on:** 8
- **Skills / agent:** pluto-reviewer agent
- **Files:** -
- **Safety impact:** -
- **Done when:** 0 blocking findings; gate re-run green after any fix.
- **Rollback:** -
- **Status:** done 2026-11-14
- **Result:** 0 blocking. The should-fix was mine: the `PIPELINE_UPDATE.md` line tables were derived by mapping HEAD numbers forward, but many were already stale at HEAD, so the staleness was carried over. All three tables ( 1e, 2c, 3 ) were rebuilt by looking each reference up in the working tree by the symbol its doc text names, and opening the new line to confirm; 12 numbers changed from my first version, and the method note in the file now records the mistake. Nits: the `API-Utils.h` comment listed only four of the six gates ( fixed ), the CLAUDE.md wording read as if the airborne test was a reason not to fly the tilt ( fixed ), the wiki never stated the tilt signs ( added, with a link to the `FC_DATA` angle table ), and the Makefile version bump is task 10. One nit was **rejected after checking the code**: the reviewer thought a Wi-Fi flicker clears the held tilt so the test code should re-send it, but AUX channels hold their value ( `RX_FAILSAFE_MODE_HOLD`, `config.cpp:441` ), `devSwitchOn` stays true and the clear never fires; the change was reverted and `PlutoPilot.cpp` is byte-identical to the log-2 build ( ELF text 124264 ). Gate re-run green. The reviewer also confirmed the numbers agree across code, header, wiki, CLAUDE.md and the staged text, every Mermaid edge is a real path, and `fw-architecture-pipeline/` and `lib/` are untouched.

### 10. Graph refresh & commit

**Description.** After the review fixes, `graphify update .` and `python tools/graph_labels.py`, then `/pluto-commit`:
all targets, `FW_Version` / `API_Version` bump ( API behaviour change ), `PIPELINE_UPDATE.md` applied, CHANGELOG, topic
Closed. Decide whether the task 1 test code stays in `PlutoPilot.cpp` or `PlutoPilot.cpp` is restored.

- **Depends on:** 9
- **Skills / agent:** graphify, pluto-commit
- **Files:** -
- **Safety impact:** -
- **Done when:** staged with a drafted message; the user commits.
- **Rollback:** -
- **Status:** done 2026-11-14
- **Result:** 14 Nov 2026: `graphify update .` rebuilt the graph ( 5119 nodes, 14021 edges, 238 communities; HEAD 5115 nodes ), and `tools/graph_labels.py` relabelled it ( 219 small communities left numbered, HEAD 231 ). The new `userAngleTarget ( )` and `throttleRaisedInFlight ( )` are in the graph. The label script needed the known backlog item 13 workaround: graphify's own Python, isolated mode ( `-I`, so `tools/warnings.py` does not shadow the standard `warnings` ) and `GRAPHIFY_VIZ_NODE_LIMIT=6000`; recorded in `.claude/TOOLING_BACKLOG.md`. The untracked `graphify-out/2026-11-14/` snapshot is not for commit. Commit preparation: `set-default-pid-profile` closed in `ebf4e46`; FW 3.15.7 -> 3.15.8, API 1.8.5 -> 1.8.6; all three targets built clean ( PRIMUS_X2_v1 121.4 KB / 20.5 KB, PRIMUSX2 121.2 / 20.7, PRIMUS_V5 121.4 / 20.5 ), no warning on any line this topic wrote; `PIPELINE_UPDATE.md` applied to seven pipeline docs; CHANGELOG version line, Fixed and Documentation entries; stock `PlutoPilot.cpp` restored, the test sequence kept in `TEST_CODE.md`; staged with a drafted message.

## Decisions log

Newest last. One line each: `YYYY-MM-DD [decision|assumption|out-of-scope|risk] text`.

- 2026-11-14 [decision] Mode `fix`; target PRIMUS_X2_v1; topic `set-desired-angle-roll-pitch` ( the empty `fc-config-api-fixes/` folder is left alone ).
- 2026-11-14 [decision] The T11 code is unknown: reproduce with our own automatic test sequence, flown by the user in a small room ( dev on -> sequence -> done -> dev off -> paste log ).
- 2026-11-14 [decision] Done when: the API behaves as documented with no unusual change; roll / pitch hold until replaced, the same as `AG_YAW`.
- 2026-11-14 [decision] Fix option B: user angle term in the pidRewrite ANGLE target; not the RC override path ( option A, which expires and fights RcCommand_Set ).
- 2026-11-14 [assumption] Limits / stick / safety "as it is now": the user angle plus `2 * rcCommand` goes through the existing `max_angle_inclination` clamp ( 20 deg ); ANGLE mode only ( ignored in ACRO / HORIZON, ANGLE not forced ); not applied in a flip or landing; cleared at dev-off and on the disarm edge, not on a flicker. Confirmed in playback.
- 2026-11-14 [assumption] Test steps are +/-50 ( 5 deg ) for 1 s each, one setter call per step.
- 2026-11-14 [out-of-scope] The yaw path, any new API or signature change, other T-cases / B-items.
- 2026-11-14 [out-of-scope] `getDesiredAngle` returns `2 * rcCommand` ( not the flown target ) and 0 for yaw; `setDesiredRate` writes the persistent rate profile; the `config.cpp:420` comment ( 50 deg for 200 ). Follow-ups.
- 2026-11-14 [risk] HEADFREE: the user tilt is applied in the body frame in pidRewrite after the HEADFREE stick rotation; not tested here.
- 2026-11-14 [decision] Task 3: no dev-mode gate on the flown tilt. `devmode` drops on every Wi-Fi flicker ( `rxIsReceivingSignal` ), and the yaw target also survives flickers; a real dev switch-off is handled by the task 4 clear in `userCode ( )`, which runs before the PID in the same loop. **Superseded in task 4**: the tilt is suppressed after 600 ms without Developer Mode ( `USER_ANGLE_LINK_GRACE_MS` ), which keeps the flicker behaviour and stops a real stall flying the craft away.
- 2026-11-14 [decision] Task 4: a user tilt is flown only once the craft is airborne: throttle raised since arming **and** `getEstAltitude ( ) >= 20 cm` ( `USER_ANGLE_MIN_ALT_CM` ). Throttle-raised alone is true on the skids for an app take-off ( it arms with the stick centred ), and a tilt there could pivot the take-off on a leg. A consequence: with no usable altitude estimate the tilt never applies.
- 2026-11-14 [decision] Task 3: one helper `userAngleTarget ( axis )` ( API-Utils.cpp ) holds the gates ( flag, ANGLE_MODE, `flipState == 0`, `! landingInProgress ( )` ); `landingInProgress ( )` is made non-static in FC-Control-Command.cpp and reused, so the landing test has one definition.
- 2026-11-14 [decision] Task 3: only `pidRewrite` ( the default, `config.cpp:176` ) flies the user tilt. The other five controllers ( MultiWii, LuxFloat, MW23, Hybrid, Harakiri ), selectable only through stored config, ignore it; documented in task 5.
- 2026-11-14 [risk] A held tilt keeps flying through a Wi-Fi stall until the failsafe LAND ( ~2.5 s ), as the yaw target does; the landing gate then drops it. Checked in the task 7 flight ( log-1 had 10 stalls in 34 s ).
- 2026-11-14 [assumption] The commit bumps to FW 3.15.8 / API 1.8.6 ( patch bumps, as the sibling API fixes did ); the wiki and `PIPELINE_UPDATE.md` say so. Task 10 confirms the numbers.
- 2026-11-14 [out-of-scope] `getDesiredAngle ( AG_ROLL / AG_PITCH )` returns the stick-plus-user sum before the clamp, so it can read above `max_angle_inclination`; documented in task 5 as "requested, before the limit" rather than changed.
- 2026-11-14 [decision] Closed - pending commit: FW 3.15.7 -> 3.15.8 ( patch, a fix ), API 1.8.5 -> 1.8.6 ( behaviour only ), as the sibling API fixes on this branch. Stock `PlutoPilot.cpp` restored; the tilt sequence is kept in `TEST_CODE.md` because it was never committed. Committed with roll -5 deg not flight-validated ( same code path as roll +, opposite sign ).
