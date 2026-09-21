# Flip / Altitude Hold Regression - Tasks

[README](README.md) · [TASKS](TASKS.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

| | |
|---|---|
| **Goal** | App back-flip works again on top of the `2a8d59a` altitude-hold setpoint shaping, without removing the shaping from the pilot's sticks. |
| **Done when** | The drone completes a full back-flip and recovers as it did before `2a8d59a`, and ALT_HOLD holds normally afterwards. The validation log shows `flipState` moving past ASCEND with VelocityZ ≥ 100 cm/s. |
| **Target** | PRIMUS_X2_v1 |
| **Kind** | bug fix (regression from `2a8d59a`) |
| **Pipeline docs affected** | subsystems/Altitude_Hold_Estimator.md (new "Flip interaction" section) |
| **Order** | 1 → 2 → 3 → 4 → 10 → 5 → 6 → 11 → 7 → 8 → 9 |

## Index

| # | Title | Status | Skills |
|---|---|---|---|
| 1 | Flip diagnostic log fields | done 2026-09-21 | flight-test |
| 2 | Confirm the ASCEND timeout on hardware | done 2026-09-21 | flight-test, flightlog-analyst agent |
| 3 | Flip bypasses altitude setpoint shaping | done 2026-09-21 | magisv2-rules, cpp-pro agent, dimensional-analysis |
| 4 | Clean handover from flip back to shaped ALT_HOLD | done 2026-09-21 | magisv2-rules |
| 5 | Build gate | done 2026-09-21 | run-magisv2 |
| 6 | Hardware validation | done 2026-09-21 | flight-test, flightlog-analyst agent |
| 7 | Architecture & pipeline docs (PIPELINE_UPDATE.md) | done 2026-09-21 | graphify, spec-to-code-compliance |
| 8 | Graph refresh | done 2026-09-21 | graphify |
| 9 | Review & commit | done 2026-09-21 (staged, user commits) | magisv2-reviewer agent, commit-magisv2 |
| 10 | Return to the pre-flip height after the flip | done 2026-09-21 | magisv2-rules, flight-test |
| 11 | Soften the post-flip braking (dip below target) | done 2026-09-21 | magisv2-rules, run-magisv2, flight-test |

Status values: `todo`, `in-progress`, `done YYYY-MM-DD`, `blocked (<why>)`, `dropped (<why>)`.
Serial numbers are never reused or renumbered.

## Tasks

### 1. Flip diagnostic log fields

**Description.** Add `flipState`, estimated vertical velocity (`getEstVelocity()`), the altitude-hold velocity demand and `EstAlt`/`AltHold` to the diagnostic log in `PlutoPilot.cpp`, so a flip attempt can be followed state by state. Keep the line within the `Monitor_Print` byte budget (well under ~250 bytes per tick). `flipState` may need an accessor or `extern` from `flight/acrobats.h`.

- **Skills:** flight-test
- **Files:** [PlutoPilot.cpp](../../../../PlutoPilot.cpp), [acrobats.cpp](../../../../src/main/flight/acrobats.cpp)
- **Done when:** builds clean on PRIMUS_X2_v1, and one log line holds all fields at under 250 bytes.
- **Status:** done 2026-09-21
- **Result:** 10-field log at 25 Hz (~140 B typical, ≤ 210 B worst). `altholdDebug6/7` carry `setVel`/`altRate`. The warning gate is clean. Review: no blocking findings; the loop-period scale and comments were fixed.

### 2. Confirm the ASCEND timeout on hardware

**Description.** Fly the current firmware (shaping in, no fix) with the task 1 fields and press the app flip button once in ALT_HOLD. The expected result: `flipState` sits in ASCEND, VelocityZ stays near 40 cm/s and never reaches 100, and after ~2.2 s `flipState` drops to 0 without rotating. This confirms the root cause in [INVESTIGATION.md](INVESTIGATION.md) before any control code changes. Can be dropped if the user prefers to skip the extra flight.

- **Skills:** flight-test, flightlog-analyst agent
- **Done when:** the log shows the ASCEND timeout (or disproves it, and a new hypothesis is recorded in INVESTIGATION.md before task 3 starts).
- **Status:** done 2026-09-21
- **Result:** `log-1.txt`: both flips timed out after 2.2 s, with `Vt` pinned at 40, peak `Vz` 34/39 cm/s, `Baro` = 1 and no rotation. Root cause confirmed (TESTING.md test 1).

### 3. Flip bypasses altitude setpoint shaping

**Description.** In `calculateAltHoldThrottleAdjustment()` (`flight/altitudehold.cpp`), while `flipState >= 1` use the pre-`2a8d59a` rate path: `setVelocity = (throttle − 1500) / 4` clamped to −100..+120 cm/s, fed straight to the velocity loop, with no `ALT_MAX_CLIMB_CMS` cap, no `ALT_VEL_ACCEL_CMSS` slew and no position leash. Recover the exact old expression from `git show 2a8d59a^:src/main/flight/altitudehold.cpp`. Check which array the stick path now reads: the flip writes `rcData[THROTTLE]`, so if the shaped path reads `rcDataPilot` the bypass must read `rcData` (flip-owned) and only while the flip is active. Pilot sticks outside a flip stay shaped.

- **Skills:** magisv2-rules, cpp-pro agent, dimensional-analysis
- **Files:** [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp), [acrobats.cpp](../../../../src/main/flight/acrobats.cpp) / `acrobats.h`
- **Done when:** builds clean, and the diff shows the bypass applies only while `flipState >= 1` and matches the pre-`2a8d59a` numbers.
- **Status:** done 2026-09-21
- **Result:** `flipActive()` / `flipVelocitySetpoint()` reproduce the pre-`2a8d59a` controller during a flip, and shaping moved unchanged into `shapedVelocitySetpoint()`. The warning gate is clean. Review: no blocking findings; its match with the old controller was checked against `2a8d59a^`. Two hand-back findings were passed to task 4.

### 4. Clean handover from flip back to shaped ALT_HOLD

**Description.** When the flip ends (`flipState` returns to 0, whether it completed or timed out), the shaped controller must restart from the current state: `AltHold` re-anchored to `EstAlt` (or the flip's own restore target if acrobats sets one), the ramped `altRate` and slewed velocity demand seeded to 0 or the measured velocity, and any goal profile cancelled. This stops a jump or dive at the moment of handover. Do not touch the baro ground datum (`throttleRaisedSinceArm` rule).

*From the task 3 review:* task 3 already keeps `altTarget`, `altRate` and `altGoalActive` following during the flip, but `flipVelocitySetpoint()` leaves `altVelTarget` at the last flip setpoint. HOLD/HOLDPOS write 2000 up to the exit tick, so that is 120 cm/s. The shaped path then clamps `velDemand` to 40 but slews `altVelTarget` down at 150 cm/s², which gives ~0.5 s of extra climb (~40 cm) above the pinned `AltHold`. Fix: on the falling edge of `flipActive()` in `calculateAltHoldThrottleAdjustment()`, seed `altVelTarget` to 0 (or to the clamped `velocity_z`), with `altRate = 0` and `altTarget = AltHold = EstAlt`. Also decide whether to ignore the one stale `setVelocity` sample on the exit tick: HOLD writes 2000 and sets `flipState = 0` in the same call, so `applyMultirotorAltHold()` takes the shaped branch with deflection 500 until the next RX refresh (≤ 20 ms, `altRate` moves by ≤ 2 cm/s).

- **Skills:** magisv2-rules
- **Files:** [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp)
- **Done when:** builds clean, and the code shows a single edge-triggered reseed on flip exit, with no datum re-zero.
- **Status:** done 2026-09-21
- **Result:** Edge detection on `flipActive()` ahead of the tilt return. On exit: `resetAltSetpoint()` (setpoint 0, hold at `EstAlt`) and the pre-flip hover trim is restored. The trim is snapshotted only while BARO_MODE is on (0 otherwise), a review fix for user-code flips started with ALT_HOLD off. The warning gate is clean. Review: no blocking findings.

### 5. Build gate

**Description.** Run `.claude/skills/run-magisv2/driver.sh --gate PRIMUS_X2_v1`.

- **Skills:** run-magisv2
- **Done when:** no new warnings under `src/`, and flash/RAM fit.
- **Status:** done 2026-09-21
- **Result:** The PRIMUS_X2_v1 build (13:47, newer than all sources) has no new warnings under `src/`. Flash 99.7 KB / 256 KB (38%), RAM 14.8 KB / 40 KB (37%).

### 6. Hardware validation

**Description.** Fly the fixed firmware on PRIMUS_X2_v1. Do at least 3 app back-flips in ALT_HOLD, and a hover with stick climbs and descents before and after to confirm the shaping is unchanged. Record the results in `TESTING.md`.

- **Skills:** flight-test, flightlog-analyst agent
- **Done when:** every flip completes the full rotation and recovers, the log shows VelocityZ ≥ 100 cm/s in ASCEND and `flipState` running through to 0, and ALT_HOLD holds within the usual band afterwards.
- **Status:** done 2026-09-21
- **Result:** `log-5.txt`: 4/4 flips completed, and each returned to its exact pre-flip `AH` and settled within about +10 cm (briefly +14). Observed but not blocking: a 15-46 cm dip below target just after exit, and a post-flip `Vz` bias (−13 cm/s for 3-8 s). See TESTING.md test 5.

### 7. Architecture & pipeline docs (PIPELINE_UPDATE.md)

**Description.** Stage the new version of `subsystems/Altitude_Hold_Estimator.md` in `PIPELINE_UPDATE.md`, with a "Flip interaction" section: BOXBARO stays on through AUX3, the flip bypasses shaping, and the handover on exit. Update the Mermaid flow if the rate-source selection is drawn, and confirm every edge with `graphify path` or a source line. Add a CLAUDE.md gotcha: acrobats' `DEACTIVATE_RC_MODE(BOXBARO)` is undone on the next RX frame.

- **Skills:** graphify, spec-to-code-compliance
- **Done when:** `PIPELINE_UPDATE.md` is written, and the CLAUDE.md line is drafted.
- **Status:** done 2026-09-21
- **Result:** `PIPELINE_UPDATE.md` holds the full new `Altitude_Hold_Estimator.md`: the flip in data structures, functions and the rate table, a new Flip interaction section with its flowchart, and flip branches in both existing flowcharts. Every edge was checked against source lines; the graph is stale for these functions. The CLAUDE.md note is drafted. `Firmware_Pipeline.md`, the maps and the API wiki are unchanged.

### 8. Graph refresh

**Description.** Run `graphify update .` then `python tools/graph_labels.py`.

- **Skills:** graphify
- **Done when:** the graph is rebuilt at the new HEAD, with community labels.
- **Status:** done 2026-09-21
- **Result:** `graphify update .` then `tools/graph_labels.py`: 3966 nodes, 7469 edges, 388 communities, built from `b91e6a7` plus the working tree. `flipActive()`, `flipVelocitySetpoint()` and `shapedVelocitySetpoint()` are in the graph. The `android-app-master/` folder was excluded via `.graphifyignore` while it was in the tree; the user later removed the folder and the ignore entry, so it is not part of the commit. Re-run after task 9 removed the diagnostics (3966 nodes, 387 communities).

### 9. Review & commit

**Description.** Run the `magisv2-reviewer` agent on the whole topic diff and fix any BLOCKING findings. Then `commit-magisv2`: all-target build, version bump, promote `PIPELINE_UPDATE.md` and the CHANGELOG, and close the topic.

- **Skills:** magisv2-reviewer agent, commit-magisv2
- **Done when:** committed, and the topic is marked Closed with the hash.
- **Status:** done 2026-09-21: staged, and the commit is left to the user
- **Result:** Diagnostics removed, graph re-run, final review with no blocking findings (doc fixes applied). All three targets pass. `Altitude_Hold_Estimator.md`, CHANGELOG (Fixed) and CLAUDE.md promoted. FW_Version 3.8.0 -> 3.8.1 (patch, at the user's request; API unchanged 1.3.2). Topic Closed - pending commit.

### 10. Return to the pre-flip height after the flip

**Description.** `log-3.txt` (TESTING.md test 3): the flip is stable, but HOLD's fixed 1.5 s of full throttle ends the flip 28-68 cm above the height it started at, and the task 4 hand-back holds that exit height. The user wants the drone back at the height where the flip button was pressed. On the rising edge of `flipActive()` in `calculateAltHoldThrottleAdjustment()`, save `AltHold` (the target ALT_HOLD was holding). On the falling edge, after `resetAltSetpoint()`, write the saved value back into `AltHold`. The shaped path's goal detection (`AltHold != lrintf(altTarget)`) then flies it there on the existing goal profile (`ALT_CMD_MAX_CLIMB_CMS` 60 / `ALT_CMD_MAX_DESCENT_CMS` 30, `ALT_GOAL_DECEL_CMSS` 80), and moving the stick cancels it as it does for take-off. Only save it while BARO_MODE is on; otherwise keep the task 4 behaviour (hold at the exit height). acrobats.cpp is not changed.

- **Skills:** magisv2-rules, flight-test
- **Files:** [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp)
- **Done when:** builds clean. The code sets `AltHold` to the saved pre-flip target on flip exit, with the setpoint reset and trim restore from task 4 kept. The task 6 flight shows each flip settling within ±10 cm of the pre-flip `AH`, reached on the goal profile.
- **Status:** done 2026-09-21 (code). The ±10 cm flight check is part of task 6.
- **Result:** On exit `AltHold` is set to the saved pre-flip target, which is flown as a goal on the goal profile. A 100 ms window after the flip ignores the flip's stale throttle so it cannot cancel the goal. The return is armed only with BARO on, not idle-held, and a pre-flip `AltHold` ≥ 20 cm (review fix: a flip from the floor would fly back down to the floor). The warning gate is clean. Review: no blocking findings.

### 11. Soften the post-flip braking (dip below target)

**Description.** `log-5.txt` (TESTING.md test 5): the flip ends climbing at 116-127 cm/s, because HOLD is still at throttle 2000. The task 4 hand-back brakes to a −30 cm/s setpoint, and the large negative velocity error drives `errorVelocityI` from its restored pre-flip value to −17…−21 counts within ~0.4 s. After the craft stops coasting up, that negative trim drops it at up to −59 cm/s, 15-46 cm below the returning target (37 cm above ground on flip 2). In `calculateAltHoldThrottleAdjustment()`, hold the velocity integrator at its restored value for a short window after the flip's falling edge (~0.5 s, a new `ALT_FLIP_EXIT_I_HOLD_MS`, or until `|Vt − Vz|` is small). The P and D terms still brake the climb, but no negative trim is banked for after it. The other task 4/10 behaviour is unchanged.

- **Skills:** magisv2-rules, run-magisv2, flight-test
- **Files:** [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp)
- **Done when:** builds clean with no new warnings. A validation flight (≥ 3 app flips from ≥ 1.5 m) shows `VI` staying near its pre-flip value through the braking, and the dip below the returning target under ~10 cm, with the return to the pre-flip `AH` still working.
- **Status:** done 2026-09-21
- **Result:** `logs-6.txt` (test 6): `VI` held at 0 through the braking (was −20), and the fall after braking dropped to −40…−51 cm/s (was −61…−73). The remaining 16-42 cm dip is in the altitude estimate re-converging after the flip, not the controller. The user accepted this outcome and deferred estimator work.

## Decisions log

Newest last. One line each: `YYYY-MM-DD [decision|assumption|out-of-scope|risk] text`.

- 2026-09-21 [decision] Kind: regression fix. Flip worked before `2a8d59a` and fails after it. Target PRIMUS_X2_v1.
- 2026-09-21 [decision] Trigger in scope: the Android app flip button → `MSP_SET_COMMAND` = 3 (`BACK_FLIP`), available only in app flight mode 0. That mode holds AUX3 on, which maps to BOXBARO (`RxConfig.cpp:88`), so ALT_HOLD is on during every app flip.
- 2026-09-21 [decision] Fix: while `flipState >= 1`, altitude hold uses the pre-`2a8d59a` raw rate path (~120 cm/s, no cap/slew). Chosen over really disabling BARO during the flip, or lowering `desiredVelocity`.
- 2026-09-21 [decision] The bypass covers the whole flip (ASCEND through HOLD/HOLDPOS), not ASCEND only.
- 2026-09-21 [decision] No failed-flip log exists; add flip fields to the `PlutoPilot.cpp` log (task 1).
- 2026-09-21 [out-of-scope] `Command_Flip()` ignores its direction argument (back flip only). Recorded as known doc drift, not fixed here.
- 2026-09-21 [out-of-scope] Android app / MSP protocol changes.
- 2026-09-21 [assumption] The crash-failsafe bypass while `flipState >= 1` (`failsafe.cpp:370`) stays unchanged.
- 2026-09-21 [risk] If the shaped stick path reads `rcDataPilot`, the flip's `rcData[THROTTLE]=2000` write may not reach altitude hold at all. Checked in task 3.
- 2026-09-21 [decision] Review of task 1 confirmed the flip's `rcData[THROTTLE]` write reaches the shaped path: `applyMultirotorAltHold` reads `rcData` (`altitudehold.cpp:338`), not `rcDataPilot`. `DEACTIVATE_RC_MODE` is an XOR toggle, and `updateActivatedModes` re-derives the mask from AUX, so `Baro` should read 1 through ASCEND.
- 2026-09-21 [decision] At the user's request, Developer Mode is on by default for the test flights (`DEV_MODE_ALWAYS_ON` in `mw.cpp:userCode()`). The AUX switch is bypassed, the live-RC-link requirement is kept. Temporary: removed with the other diagnostics before the commit in task 9.
- 2026-09-21 [decision] Task 2 confirmed the root cause (`log-1.txt`, two flips). A side effect was also seen: a timed-out flip leaves the drone coasting to 70-90 cm above its start height, because `AltHold` keeps climbing at 40 cm/s for 2.2 s. The fix removes the timeout, so this is not tracked separately.
- 2026-09-21 [decision] Task 3: the flip reads `rcData[THROTTLE]` deliberately, because the flip owns it. The raw branch is gated on `flipActive()`, so outside a flip the same `rcData` read goes through shaping and a pilot's 2000 is capped at 40. Landing keeps priority over the flip branch.
- 2026-09-21 [risk] Reading the task 6 log: during a flip `Rt` (`altholdDebug7`) always reads 0 because `flipVelocitySetpoint()` zeroes `altRate`. Read `Vt` (the setpoint) instead.
- 2026-09-21 [risk] `log-2.txt` (task 3 build): the flip completed, then a flyaway into the ceiling ~0.25 s after exit, and crash detection disarmed. Cause: the stale `altVelTarget` (120) plus integrator wind-up during HOLD (TESTING.md test 2).
- 2026-09-21 [decision] Task 4 hand-back: on flip exit, `altVelTarget = 0` and hold at `EstAlt` (`resetAltSetpoint()`), which matches the pre-`2a8d59a` first tick. Chosen over seeding from the measured climb.
- 2026-09-21 [decision] Task 4: snapshot `errorVelocityI` when the flip starts and restore it when it ends (pre-flip hover trim). Chosen over keeping the pre-`2a8d59a` wind-up or freezing the integrator during the flip.
- 2026-09-21 [decision] Task 4 review: the pre-flip trim is snapshotted only when BARO_MODE is on, and 0 otherwise. `Command_Flip()` from user code can start a flip with ALT_HOLD off (`command.cpp` checks only MAG_MODE), and the integrator winds there against an unactuated `AltHold`.
- 2026-09-21 [decision] Task 6 validation flips are flown from a hover of at least 1.5 m (the drop after the rotation was ~100 cm in `log-2.txt`).
- 2026-09-21 [decision] `log-3.txt`: the flip and hand-back are stable (no flyaway, `VI` restored to its pre-flip 0). But each flip ends 28-68 cm above its start, because HOLD runs full throttle for 1.5 s, and the drone holds that exit height. New task 10: return to the pre-flip height.
- 2026-09-21 [decision] Task 10 returns to the pre-flip `AltHold` (the target ALT_HOLD was holding), not the pre-flip `EstAlt`.
- 2026-09-21 [decision] Task 10 flies back on the existing goal profile (60 up / 30 down cm/s, 80 cm/s² braking), not a dedicated faster profile. The stick cancels it.
- 2026-09-21 [out-of-scope] Reducing HOLD's overshoot in `acrobats.cpp` (fixed 1.5 s at throttle 2000). The flip is left as it was before `2a8d59a`; only the return is added.
- 2026-09-21 [decision] Task 10: a 100 ms window after the flip ignores the throttle (`ALT_FLIP_EXIT_IGNORE_MS`). Without it, the flip's stale 2000 on the exit tick reads as full stick and cancels the return goal on its first tick.
- 2026-09-21 [decision] Task 10 review: the return is armed only if BARO_MODE is on, the craft is not idle-held (`altHoldGroundIdle`) and the pre-flip `AltHold` ≥ `ALT_FLIP_RETURN_MIN_CM` (20 cm). Otherwise the drone holds where the flip ends (task 4). This stops a flip sent while armed on the floor from flying back down to the floor with the motors running.
- 2026-09-21 [assumption] Task 10: a flip sent during a take-off or `setAltitude()` goal returns to the target's position at that moment, not to the goal (the "`AltHold` it was holding" decision). A flip that ends with BARO off leaves a harmless unintended 100 ms stick-ignore window on the next BARO entry (review, not fixed). With `alt_hold_fast_change = 1` (default 0) there is effectively no return.
- 2026-09-21 [decision] `log-5.txt` validated tasks 3/4/10 (task 6 done). The user asked for the post-exit dip (15-46 cm below target) to be fixed: new task 11, freezing the velocity integrator briefly after flip exit.
- 2026-09-21 [out-of-scope] Post-flip velocity-estimate bias (`Vz` ≈ −13 cm/s for 3-8 s while level, `EA` 6-14 cm above `AH`). The user chose to ignore it: it stays within about +10 cm and decays by itself.
- 2026-09-21 [decision] Task 11: the post-flip integrator hold is a fixed 500 ms (braking took ~0.4 s in `log-5.txt`). The review noted there is little margin if braking runs longer. The fallback, if the flight shows a dip over 10 cm, is a condition-based release (`error < 0 && Vz > 0`) capped at 500 ms. D8[PIDVEL] = 1 contributes almost nothing to braking; P at its ±300 clamp does the work.
- 2026-09-21 [decision] Task 11 closed on `logs-6.txt`: the integrator hold works. The user accepted the remaining 16-42 cm dip in `EA`, which is the altitude estimate re-converging after the flip (`EA` falls ~2× faster than `∫Vz`).
- 2026-09-21 [out-of-scope] Post-flip altitude-estimator disagreement (baro vs accel after the rotation; candidates: the baro throttle-compensation step at flip exit, and the accel bias). Deferred by the user to a possible later topic; a `LASER_TOF` ground-truth run is the first step if it is picked up.
- 2026-09-21 [decision] Flight logs `log-1.txt` … `logs-6.txt` were deleted from the repo root at the user's request. All results are kept in TESTING.md (tests 1-6); the logs cannot be re-analysed.
- 2026-09-21 [decision] `android-app-master/*` was temporarily in `.graphifyignore` so the graph refresh left out the Java app. The user removed the app folder and both ignore entries before committing.
- 2026-09-21 [out-of-scope] `setUserLoopFrequency()` takes a period in ms, not a frequency in Hz (`FC-Config-PID.cpp:83`, `mw.cpp:1128`), which contradicts its name and the `FC-Config.h` doc. This is existing API drift, not fixed here.
