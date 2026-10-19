# Off-centre payload lean - Tasks

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [TESTING](TESTING.md) · [CHANGES](CHANGES.md) · [SESSION](SESSION.md)

## Resume

Read this block first in a new session; read only the task it points to.

| | |
|---|---|
| **Current task** | none: run /pluto-task next ( task 5 ) |
| **Next step** | Task 5: hover-snapshot seed in `pidResetErrorGyro ( )`; mini-grill the clamp first ( `Ih` ≈ −103 is ~80 % of the planned ±128 ). Also open: task 12's scope ( user ) |
| **Open questions** | Did the pilot give roll / pitch stick during or after the log-1 dips? Task 12 scope. ( Work checkpointed on branch `BugFix-June26-cg-tilt`; handoff in [SESSION.md](SESSION.md) ) |
| **Blocked on** | nothing |
| **Last updated** | 2026-10-19 |

## Summary

| | |
|---|---|
| **Mode** | fix |
| **Goal** | With a 9 g payload off-centre on any side, the app take-off and a full-stick ALT_HOLD descent hold position with no stick input |
| **Done when** | Servo flights ( task 8 ) need no opposite stick on take-off ( first and second flight after power-on ) or full-stick descent; the stock airframe flies as before; a ground-idle arm shows no integrator windup; gate clean, flash growth < 500 B |
| **Target** | PRIMUS_X2_v1 ( all targets at commit ) |
| **Analysis** | Causal chain in [INVESTIGATION.md](INVESTIGATION.md): the rate integrator is zeroed at arm and on stick-low in flight ( `mw.cpp:894` ); `4657cbf` and `2a8d59a` made those phases last ~4× longer, so the lean became visible. H1 confirmed by task 1, H2 by task 3 |
| **Pipeline docs affected** | `subsystems/Core_PID_Controller.md` ( plus drift ), `subsystems/Altitude_Hold_Estimator.md`, `subsystems/RC_Input_Pipeline.md`, `dev-guide/FLIGHT_INVARIANTS.md`, CLAUDE.md one-liner |
| **Order** | 1 → 2 → 3 → 4 → 5 → 6 → ( 12, if kept ) → 7 → 8 → 9 → 10 → 11 |

## Index

| # | Title | Status | Depends on | Skills / agent |
|---|---|---|---|---|
| 1 | Confirming flight: partial vs full stick descent on 3.15.1 | done 2026-10-19 | - | pluto-flighttest |
| 2 | Diagnostic getters and test log ( I-term, reset counter ) | done 2026-10-19 | 1 | pluto-rules, pluto-flighttest, pluto-build |
| 3 | Baseline flight log with the servo | done 2026-10-19 | 2 | pluto-flighttest; pluto-log-analyst agent |
| 4 | E: no stick-low reset while flying in ALT_HOLD | done 2026-10-19 | 3 | pluto-rules, pluto-build |
| 5 | B: hover snapshot seeds the integrator at arm | todo | 4 | pluto-rules, pluto-build |
| 6 | C: faster roll / pitch wind-up during the take-off climb | todo | 5 | pluto-rules, pluto-build |
| 7 | Build gate | todo | 4, 5, 6 | pluto-build |
| 8 | Hardware validation ( servo + stock regression ) | todo | 7 | pluto-flighttest; pluto-log-analyst agent |
| 9 | Architecture & pipeline docs ( PIPELINE_UPDATE.md ) | todo | 8 | graphify, spec-to-code-compliance |
| 10 | Topic review | todo | 9 | pluto-reviewer agent |
| 11 | Graph refresh & commit | todo | 10 | graphify, pluto-commit |
| 12 | Ground idle can drop the motors in the air ( stale `isThrottleStickArmed` ) | todo ( scope: user ) | - | pluto-rules, pluto-build |

Status values: `todo`, `in-progress`, `done YYYY-MM-DD`, `blocked (<why>)`, `dropped (<why>)`.
Serial numbers are never reused or renumbered.

## Tasks

### 1. Confirming flight: partial vs full stick descent on 3.15.1

**Description.** No firmware change. Reflash `Build/PRIMUS_X2_v1/MechAsh!_PRIMUS_X2_v1_3.15.1.hex`; the EEPROM goes
106 → 108, so all settings reset: recalibrate the accelerometer and compass. Servo on the front, ALT_HOLD on, app
take-off, hands off. Hover about 5 s, then descend with the throttle stick about halfway to ¾ down ( raw 1100-1460:
above `mincheck`, below the 40 µs deadband, so no reset ) for 3-4 s. Climb back, then descend with the stick fully
down. Note the lean on take-off as well. Optional: a short `PlutoPilot.cpp` log with the existing API
( `RcData_Get ( RC_THROTTLE )`, roll / pitch from `Estimate_Get`, altitude ) under 130 B, to read the raw stick value.
Video is welcome.

- **Depends on:** -
- **Skills / agent:** pluto-flighttest
- **Files:** `log-1.txt` ( if logged ), `TESTING.md`
- **Safety impact:** flight test; hand on the ARM switch. The full-stick descent drifts by design
- **Done when:** H1 is confirmed ( partial straight, full leans ) or killed. If killed, add bisect tasks
  ( `4657cbf` → `2a8d59a` → later commits, recalibrating at each EEPROM change ) before task 4
- **Rollback:** -
- **Status:** done 2026-10-19
- **Result:** H1 confirmed by the pilot's observation ( no log ): the lean appears only at or just above full stick down ( the `mincheck` 1100 band ); higher partial-stick descents come straight down ( TESTING.md §1 ). No bisect needed.

### 2. Diagnostic getters and test log ( I-term, reset counter )

**Description.** Add `int16_t pidGetGyroITerm ( uint8_t axis )` ( returns `errorGyroI[axis] >> 13` ) and a
`uint16_t` count of `pidResetErrorGyro ( )` calls to the internal header `flight/pid.h` ( already `extern "C"` ). No
public API change. `debug[]` is taken ( `accgyro_mpu.cpp:143`, `altitudehold.cpp:871-873` ). Write a test
`PlutoPilot.cpp` in this topic's `test/` that logs, in the style of `takeoff-land-command-fix/test/`: `Ir`, `Ip`
( I-terms ), `Rs` ( reset count ), `Tp` ( raw stick from `rcDataPilot` ), roll / pitch angle, altitude and the arm /
mode flags, under 130 B per line. The getters stay for the fix's validation ( task 8 ); decide at task 9 whether to
keep them.

- **Depends on:** 1
- **Skills / agent:** pluto-rules ( Monitor_Print budget ), pluto-flighttest, pluto-build
- **Files:** [pid.h](../../../../src/main/flight/pid.h), [pid.cpp](../../../../src/main/flight/pid.cpp), `test/`, [PlutoPilot.cpp](../../../../PlutoPilot.cpp)
- **Safety impact:** none ( read-only getters, a counter increment )
- **Done when:** builds with no new warnings; the log line is under 130 B
- **Rollback:** remove the getters, restore the template `PlutoPilot.cpp`
- **Status:** done 2026-10-19
- **Result:** Getters and reset counter in `pid.{h,cpp}`; sketch `test/PlutoPilot_lean_diag.cpp.txt` installed as the root `PlutoPilot.cpp` ( 9 fields, ~112 B typical, ~125 B worst case ). Gate clean; +1152 B flash ( mostly the sketch ), +8 B RAM against stock `a81c180`. Review: 0 blocking, 0 should-fix, 4 notes ( 1 applied, 2 carried to task 3 ) ( CHANGES.md ).

### 3. Baseline flight log with the servo

**Description.** On the task 2 build, servo on the front: app take-off, hover 10 s, full-stick descent 3 s, centre,
hover, `Command_Land` from the app. Then a second take-off in the same power cycle. The log must show `Ip` at 0 at
lift-off and its build-up ( time constant ~0.5 s expected ), `Ip` falling to 0 and `Rs` counting during the
full-stick descent, and the settled hover `Ip` value. That value sizes task 5's clamp and task 6's gain and window.

- **Depends on:** 2
- **Skills / agent:** pluto-flighttest; pluto-log-analyst agent for a large log
- **Files:** `logs/log-1.txt`, `TESTING.md` §2 ( plan and predictions written 2026-10-19 )
- **Safety impact:** flight test
- **Done when:** H2 is confirmed or killed; the hover `Ip` for 9 g at the front is recorded
- **Rollback:** -
- **Status:** done 2026-10-19
- **Result:** log-1 ( TESTING.md §3 ). H2 confirmed: `Ip` → 0 at every arm, take-off lean +4.8° / +8.7° that starts on the floor and fades in ~0.7 s as `Ip` builds. `Ih` = −102 / −105 pitch, +6 / +7 roll. Full-stick descent: `Ip` held at 0, ~10° standing nose-down lean. Floor windup: `Ip` hits −256 disarmed within ~1 s. New: ±14-20° pitch / roll swings for 3-5 s after each dip recovery ( cause open; checked in task 8 ).

### 4. E: no stick-low reset while flying in ALT_HOLD

**Description.** At [mw.cpp:894-897](../../../../src/main/mw.cpp#L894) skip `pidResetErrorAngle ( )` /
`pidResetErrorGyro ( )` when `ARMING_FLAG ( ARMED ) && FLIGHT_MODE ( BARO_MODE ) && ! altHoldGroundIdle`, the craft is
at least 20 cm up ( `EstAlt` ) and a stick descent has not touched down ( decided 19 Oct 2026, see the decisions log ).
All of it lives in `altHoldKeepRateITerm ( )` in `altitudehold.cpp`, next to the static `altHoldGroundIdle`. Throttle mode ( no BARO ) and the ground idle keep
today's reset. Do not gate on `throttleRaisedSinceArm`: it becomes true right after a centred-stick arm. Check in the
review: the Wi-Fi failsafe writes a low `rxfail` throttle ( `rx.cpp:531,558` ), so the integrator is now kept through
a failsafe landing; the `rcData` hold on an invalid pulse ( `rx.cpp:528` ); a flip writes 2000, not low.

- **Depends on:** 3
- **Skills / agent:** pluto-rules, pluto-build
- **Files:** [mw.cpp](../../../../src/main/mw.cpp), [altitudehold.h](../../../../src/main/flight/altitudehold.h), [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp)
- **Safety impact:** ALT_HOLD descent, failsafe landing ( integrator kept ). Ground behaviour unchanged. Gap: after
  `Command_Arm` with the stick low in BARO mode ( no ground idle ), the integrator winds on the floor, bounded by
  ±256 and later by task 5's clamp
- **Done when:** builds clean; the review confirms the reset still fires on the ground, at ground idle and in
  throttle mode, and never in an ALT_HOLD flight
- **Rollback:** remove the condition
- **Status:** done 2026-10-19
- **Result:** `altHoldKeepRateITerm ( )` gates the stick-low reset in `processRx ( )`: kept while armed in ALT_HOLD, off ground idle, ≥ 20 cm and before a stick-descent touchdown ( `updateStickTouchdown ( )`, 300 ms arrest latch ). Gate clean, +208 B flash, +4 B RAM. Review: round 1 one should-fix ( height floor alone ) fixed; round 2 0 blocking / 0 should-fix ( CHANGES.md ). New task 12 ( pre-existing ground-idle hazard ).

### 5. B: hover snapshot seeds the integrator at arm

**Description.** Keep a low-pass snapshot of `errorGyroI[ROLL/PITCH] >> 13` as `static` in `pid.cpp` ( 8 B RAM, not
EEPROM ). Update it only in settled hover: the `offloadHoverTrim` test ( armed, not ground idle, `setVelocity == 0`,
`altRate == 0`, no goal active, `|VelocityZ| < 10`, [altitudehold.cpp:519-523](../../../../src/main/flight/altitudehold.cpp#L519) ),
plus `flipState == 0` and `EstAlt ≥ 20 cm`. Landing excludes itself ( `setVelocity < 0` ). `pidResetErrorGyro ( )`
loads the seed into roll / pitch instead of 0; yaw stays 0. Clamp ±128 ( half of `GYRO_I_MAX` ), unless task 3 shows
the hover value needs more. A crash disarm ( `failsafe.cpp:380-381` ) clears the seed; a failsafe landing or an ARM
switch off mid-air keep it.

- **Depends on:** 4
- **Skills / agent:** pluto-rules, pluto-build
- **Files:** [pid.cpp](../../../../src/main/flight/pid.cpp), [pid.h](../../../../src/main/flight/pid.h), [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp), [failsafe.cpp](../../../../src/main/flight/failsafe.cpp)
- **Safety impact:** arming. Every arm and stick-low reset starts from the seed, so a wrong seed leans the craft
  until it decays ( ~0.5 s ). The seed never comes from the floor or a landing
- **Done when:** builds clean; the review confirms the snapshot cannot be taken on the floor, during a landing, a
  flip or a take-off climb, and that the clamp holds
- **Rollback:** `pidResetErrorGyro ( )` back to 0
- **Status:** todo
- **Result:**

### 6. C: faster roll / pitch wind-up during the take-off climb

**Description.** Multiply the roll / pitch integration step at [pid.cpp:831](../../../../src/main/flight/pid.cpp#L831)
by k for a window after the take-off. Start values: k = 2, about 1.5 s. Decide at the start of the task
( mini-grill ): when the window starts ( goal set, or lift-off seen by altitude, so it does not wind against the
floor ), when it ends ( time, or goal reached ), and whether it also runs when the seed from task 5 is present. Use
task 3's numbers. No reusable mechanism exists ( `PIDweight` scales P, I and D together; `dynI8` is unused by
`pidRewrite` ).

- **Depends on:** 5
- **Skills / agent:** pluto-rules, pluto-build
- **Files:** [pid.cpp](../../../../src/main/flight/pid.cpp), [command.cpp](../../../../src/main/command/command.cpp) or [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp) ( window start )
- **Safety impact:** take-off. A higher I gain can overshoot or oscillate in the rate loop; it must not run on the
  floor or in a flip
- **Done when:** builds clean; the review confirms the window is bounded, wrap-safe and cannot start on the floor
- **Rollback:** k = 1
- **Status:** todo
- **Result:**

### 7. Build gate

**Description.** `.claude/skills/pluto-build/driver.sh --gate PRIMUS_X2_v1` on tasks 2 and 4-6 with the template
`PlutoPilot.cpp`. Record flash and RAM against `a81c180` ( 3.15.1 ).

- **Depends on:** 4, 5, 6
- **Skills / agent:** pluto-build
- **Files:** -
- **Safety impact:** none
- **Done when:** no new `src/` warnings; flash growth under 500 B; RAM within a few tens of bytes
- **Rollback:** -
- **Status:** todo
- **Result:**

### 8. Hardware validation ( servo + stock regression )

**Description.** PRIMUS_X2_v1, ALT_HOLD on, the task 2 log. With the servo on the front, then on one side: app
take-off as the first flight after power-on and again as the second, hover, full-stick descent, `Command_Land`, all
hands-off. Without the servo: one normal flight ( hover, descent, land ), and an ARM-switch arm at ground idle with
the stick held between 1100 and 1500 for 10 s, then take-off: no lean at lift-off ( no ground windup ). Flip and
failsafe are out of scope beyond what these flights touch. Results in `TESTING.md`. Also check the post-dip recovery
( task 3 found ±14-20° pitch / roll swings for 3-5 s after centring the stick, with the integrator rebuilding from 0 ):
with task 4 it should be gone. If it stays, it has another cause and becomes a new task. Note any roll / pitch
stick input during the descents: the log cannot show it. Fly the full-stick descent from at least 100 cm, so the
recovery is not in ground effect. Add one **stick-down landing** ( stick held low to the floor, then disarm ):
`Rs` must resume counting and `Ip` fall to 0 within about 0.5 s of touchdown ( task 4's touchdown latch; add a getter
for `altStickTouchdown` as a fifth `F` digit ). If the latch is ever seen in the air, raise `ALT_TOUCHDOWN_MIN_REQ_CMS`
from 15 to ~23 ( task 4 review note A ).

- **Depends on:** 7
- **Skills / agent:** pluto-flighttest; pluto-log-analyst agent for large logs
- **Files:** `log-3.txt` … , `TESTING.md`
- **Safety impact:** flight tests; hand on the ARM switch
- **Done when:** no opposite stick needed in any servo case; `Ip` stays near its hover value through the full-stick
  descent; the ground-idle case lifts off level
- **Rollback:** reflash FW 3.15.1 ( `a81c180` )
- **Status:** todo
- **Result:**

### 9. Architecture & pipeline docs ( PIPELINE_UPDATE.md )

**Description.** Stage in `PIPELINE_UPDATE.md`: `Core_PID_Controller.md` ( the real controller `pidRewrite` via the
`pid_controller` pointer, the `GYRO_I_MAX` bound, where the integrator is reset and seeded; fix the drift:
`pidController ( )` / `applyGyroData ( )` do not exist, `:18`, `:27` ), `Altitude_Hold_Estimator.md` ( stick-low
behaviour in ALT_HOLD, the snapshot ), `RC_Input_Pipeline.md` ( who uses THROTTLE_LOW ), `FLIGHT_INVARIANTS.md` and a
CLAUDE.md one-liner ( the integrator is never zeroed in an ALT_HOLD flight; arms start from the hover snapshot ).
Every diagram edge confirmed by `graphify path` or a source line. Decide whether the task 2 getters stay.

- **Depends on:** 8
- **Skills / agent:** graphify, spec-to-code-compliance
- **Files:** `PIPELINE_UPDATE.md`, [CLAUDE.md](../../../../CLAUDE.md)
- **Safety impact:** none
- **Done when:** `PIPELINE_UPDATE.md` covers the four docs; the CLAUDE.md line is drafted
- **Status:** todo
- **Result:**

### 10. Topic review

**Description.** `pluto-reviewer` on the whole topic diff. Fix BLOCKING findings and re-run the gate.

- **Depends on:** 9
- **Skills / agent:** pluto-reviewer agent
- **Files:** topic diff
- **Safety impact:** as tasks 4-6
- **Done when:** no open BLOCKING findings; the gate re-run after fixes
- **Status:** todo
- **Result:**

### 11. Graph refresh & commit

**Description.** Restore the template `PlutoPilot.cpp`. `graphify update .` and `python tools/graph_labels.py`,
then `/pluto-commit`: all-target build, FW version bump, `PIPELINE_UPDATE.md` promotion, CHANGELOG, drafted message.
The user commits.

- **Depends on:** 10
- **Skills / agent:** graphify, pluto-commit
- **Files:** -
- **Safety impact:** none
- **Done when:** staged, with the drafted message shown
- **Status:** todo
- **Result:**

### 12. Ground idle can drop the motors in the air ( stale `isThrottleStickArmed` )

**Description.** Found in the task 4 review ( note 5 ); predates this topic. `isThrottleStickArmed` is set when the ARM
switch arms the craft with the stick low ( `rc_controls.cpp:175` ). It is cleared only inside the alt-hold controller
( `altitudehold.cpp` ground-idle block, `altHoldReleaseGroundIdle ( )` ) or at disarm ( `mw.cpp:586` ). If the craft is
armed that way with BARO off and flown in throttle mode, the flag stays set. Switching BARO on later in flight with the
stick at or below 1500 then takes the ground-idle branch, `rcCommand [ THROTTLE ] = 1000`, so the motors idle in the
air. Candidate fix ( review ): clear `isThrottleStickArmed` on BARO entry once `throttleRaisedSinceArm` is set
( `updateAltHoldState ( )`, `altitudehold.cpp:645` ). Confirm the path at the start of the task.

- **Depends on:** - ( independent of tasks 4-6; place before task 7 if kept in this topic )
- **Skills / agent:** pluto-rules, pluto-build
- **Files:** [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp)
- **Safety impact:** ALT_HOLD entry in flight; removes a motor cut in the air. Must keep the ground idle working for an
  ARM-switch arm with the stick low ( the craft must not spin up on the floor )
- **Done when:** builds clean; the review confirms the flag can no longer survive into a flight, and the ground-idle
  arm still holds the motors at 1000 on the floor
- **Rollback:** remove the clear
- **Status:** todo ( scope: user )
- **Result:**

## Decisions log

Newest last. One line each: `YYYY-MM-DD [decision|assumption|out-of-scope|risk] text`.

- 2026-10-19 [decision] Mode `fix`, target PRIMUS_X2_v1; done when no opposite stick is needed with the servo ( user ).
- 2026-10-19 [decision] Cover both symptoms: stick-down descent and take-off ( user ).
- 2026-10-19 [decision] Take-off: hover snapshot ( B ) plus faster wind-up ( C ), so the first flight after power-on is covered too ( user ).
- 2026-10-19 [decision] Flash budget < 500 B against `a81c180` ( user ).
- 2026-10-19 [decision] Verification: servo flights plus a stock-airframe regression including a ground-idle arm; rollback FW 3.15.1 ( user ).
- 2026-10-19 [assumption] The payload only changes with the battery out, so the RAM snapshot cannot be stale ( user ).
- 2026-10-19 [assumption] Default PIDs ( I8 roll / pitch 10 ); app take-off with the stick centred; ALT_HOLD in every phase ( user ).
- 2026-10-19 [decision] `f60c9ad` flew clean with the payload ( user flight ). The reset code is identical; the regression is the longer exposure from `4657cbf` ( datum re-zero removed ) and `2a8d59a` ( goal profile ). Neither is reverted ( INVESTIGATION.md §4 ).
- 2026-10-19 [decision] A confirming flight on 3.15.1 ( partial vs full stick ) comes first instead of a bisect ( user ).
- 2026-10-19 [out-of-scope] Driving the servo / PWM API; PID retuning; flip and failsafe beyond the validation flights; flying PRIMUS_V5 ( user ). Throttle mode ( no BARO ) keeps today's reset.
- 2026-10-19 [risk] Task 4 keeps the integrator through a Wi-Fi failsafe landing ( low `rxfail` throttle ); judged correct ( it holds the hover trim ) but checked in review. **Corrected by the task 4 review:** the failsafe throttle is 1200 ( `rx.cpp:447` ) or 1050 via `RxConfig.cpp:162`, and `landThrottle` is 1150-1300, so a failsafe or LAND descent normally never reads as stick-low. Task 4 changes nothing there, except during the 600 ms hold of a stick that was already low ( `rx.cpp:526-528` ).
- 2026-10-19 [risk] Doc drift in `Core_PID_Controller.md:18,27` ( nonexistent functions, wrong windup bound ); fixed in task 9.
- 2026-10-19 [decision] Task 1: H1 confirmed in flight ( lean only in the bottom stick band ); the bisect is not needed. The take-off lean ( H2 ) is left to the task 3 log.
- 2026-10-19 [decision] Task 2: internal getters in `flight/pid.h` ( no public API ); the sketch sends no commands ( app take-off ) and drops the `n` counter to stay within 130 B ( review note 3 accepted ).
- 2026-10-19 [risk] The gated build's clean step removed PlutoIDE's `MechAsh!_PRIMUS_X2_v1_3.15.1.hex`. The `Build/` `3.15.1` hex is now the diagnostic build. A stock `a81c180` rollback hex was rebuilt in the scratchpad ( `hex/STOCK_a81c180_…` ); a PlutoIDE rebuild of a clean checkout also restores it.
- 2026-10-19 [decision] Task 3: H2 confirmed ( log-1 ); `Ih` ≈ −103 pitch for 9 g at the front. Positive `P` = nose down. The integrator reaches the −256 rail on the floor while disarmed, so the task 5 seed must come only from airborne settled hover.
- 2026-10-19 [risk] Task 5 clamp: `Ih` −103 is ~80 % of the planned ±128. A heavier or further-out payload would clip the seed ( it still flies; only the seed is short ). Decide the clamp at the start of task 5.
- 2026-10-19 [assumption] Task 6: the take-off lean starts on the floor while the motors spin up ( craft pivots on its front legs ), and the integrator builds in the right direction there, so the faster wind-up window may start at the arm. Settle at the start of task 6.
- 2026-10-19 [decision] Task 4 ground guard ( user, two rounds ): first a 20 cm height floor; the review showed that alone fails after a warm flight ( the baro errs high: `barometer.cpp:373-375`; log-1 read `Ze` 11 cm on the floor ) and on a table ( `EstAlt` is relative to the arm point ). Final: height floor **and** a touchdown latch ( fast descent request ≥ 15 cm/s, the craft seen moving below −8 cm/s, then arrested for 300 ms, as `land ( )` tests it ), latched until the request ends.
- 2026-10-19 [out-of-scope] Task 4 review note 5 ( predates the topic ): with an ARM-switch arm and the stick low while BARO is off, `isThrottleStickArmed` stays true. Switching BARO on later in flight with the stick at 1500 or below sets the ground idle and drops the motors to 1000 in the air. Added as task 12; the user decides whether it stays in this topic.
- 2026-10-19 [risk] Log-1: ±14-20° pitch / roll swings for 3-5 s after each dip recovery ( 30-50 cm high, integrator rebuilding from 0, overshoot to −120 / −137 ). The roll part is unexplained; pilot stick input is not logged. Expected to vanish with task 4; checked in task 8 ( descent from ≥ 100 cm ).
