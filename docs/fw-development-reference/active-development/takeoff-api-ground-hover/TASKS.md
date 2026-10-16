# Take-off API ground hover - Tasks

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [TESTING](TESTING.md)

## Resume

Read this block first in a new session; read only the task it points to.

| | |
|---|---|
| **Current task** | none: topic Closed ( handed over to [takeoff-land-command-fix](../takeoff-land-command-fix/README.md) ) |
| **Next step** | none here; continue in takeoff-land-command-fix |
| **Open questions** | none ( answered in the fix plan: 3 s window or ground idle; ramp ≥ 1 s ) |
| **Blocked on** | nothing |
| **Last updated** | 2026-10-16 |

## Summary

| | |
|---|---|
| **Mode** | check |
| **Goal** | Find out why `Command_Arm ( )` then `Command_TakeOff ( 100 )` in `onLoopStart` skims the floor instead of climbing to 100 cm |
| **Done when** | An A/B flight log shows the cause. Expected: desired Z equals ground Z with Arm + TakeOff, and desired Z is about +100 cm with the craft climbing when TakeOff runs alone |
| **Target** | PRIMUS_X2_v1 |
| **Analysis** | **H1 confirmed in flight** ( log-1 / log-2 ): with `Command_Arm` first, `takeOff ( )` sets no height ( `command.cpp:97-103` ). H2 and H3 ruled out; H4 not tested ( [INVESTIGATION.md](INVESTIGATION.md) ) |
| **Pipeline docs affected** | `subsystems/User_Space_API.md`, `subsystems/Altitude_Hold_Estimator.md` ( only at fix time ) |
| **Order** | 1 → 2 → 3 → 4 → 7 → 5 → 6 |

## Index

| # | Title | Status | Depends on | Skills / agent |
|---|---|---|---|---|
| 1 | A/B diagnostic sketches and log line | done 2026-10-16 | - | pluto-flighttest, pluto-rules, pluto-build |
| 2 | A/B flights: Arm + TakeOff vs TakeOff alone | done 2026-10-16 | 1 | pluto-flighttest, pluto-log-analyst agent |
| 3 | Workaround flight: Arm + `DesiredPosition_setRelative` | done 2026-10-16 | 2 | pluto-flighttest |
| 4 | Fix options compared | done 2026-10-16 | 2 | pluto-rules |
| 5 | Findings | done 2026-10-16 | 2, 3, 4, 7 | - |
| 6 | Close | done 2026-10-16 | 5 | pluto-commit |
| 7 | Check: `Command_TakeOff` re-run on a Dev Mode restart cancels a LAND | done 2026-10-16 | 2 | pluto-rules, pluto-flighttest |

Status values: `todo`, `in-progress`, `done YYYY-MM-DD`, `blocked (<why>)`, `dropped (<why>)`.
Serial numbers are never reused or renumbered.

## Tasks

### 1. A/B diagnostic sketches and log line

**Description.** Write two user-code sketches in the topic's `test/` folder. Sketch A calls `Command_Arm ( )` then
`Command_TakeOff ( 100 )` once in `onLoopStart ( )`, exactly as the user did. Sketch B calls only `Command_TakeOff ( 100 )`.
Both print one diagnostic line per user tick from `plutoLoop ( )`: time, armed flag, desired Z
( `DesiredPosition_get` / `getDesiredPositions ( Z )` ), estimated Z, Z velocity, throttle command, the RC throttle,
the ALT HOLD flag and the command status if the API exposes it. Keep the line under ~130 bytes per tick. Check the
double-overload rule ( print ints ). The root `PlutoPilot.cpp` is restored to the template after the flights.

- **Depends on:** -
- **Skills / agent:** pluto-flighttest ( log fields ), pluto-rules ( Monitor_Print budget ), pluto-build ( PRIMUS_X2_v1 compiles )
- **Files:** `test/PlutoPilot_A_arm_takeoff.cpp.txt`, `test/PlutoPilot_B_takeoff_only.cpp.txt` ( topic folder ), [PlutoPilot.cpp](../../../../PlutoPilot.cpp)
- **Safety impact:** none in firmware; sketch B arms by itself when Dev Mode comes on ( fly with the ARM switch on and in open space )
- **Done when:** both sketches compile on PRIMUS_X2_v1; the log line is ≤ 130 bytes; the test procedure is in `TESTING.md`
- **Rollback:** restore the template `PlutoPilot.cpp`
- **Status:** done 2026-10-16
- **Result:** A and B written ( identical apart from the `Command_Arm` line ); 119 B per tick ( worst case 126 ); both build on PRIMUS_X2_v1, no new warnings, flash 107.8 KB / RAM 17.6 KB. Review: no blocking findings. Both fixes applied: `Command_Land ( 5 )` instead of the default, and the test timers reset in `onLoopStart`. Version A is in the root `PlutoPilot.cpp`.

### 2. A/B flights: Arm + TakeOff vs TakeOff alone

**Description.** The user flies sketch A and then sketch B in the same place and on the same battery, with ALT HOLD on
and the stick centred. Logs go in `log-1.txt` ( A ) and `log-2.txt` ( B ), created in the topic folder and opened in the
IDE. Read them with `tools/flightlog.py` or the `pluto-log-analyst` agent if they are large. This task confirms or
rules out H1, and the logged throttle and ALT HOLD fields settle H2 and H3.

- **Depends on:** 1
- **Skills / agent:** pluto-flighttest; pluto-log-analyst agent for large logs
- **Files:** `log-1.txt`, `log-2.txt`, `TESTING.md`
- **Safety impact:** flight test; land with `Command_Land` or by turning Dev Mode off
- **Done when:** H1 is confirmed ( A: desired Z = ground Z after arming; B: desired Z about +100 cm, climb to about 100 cm in about 3 s ) or ruled out with the numbers; H2 / H3 are settled
- **Status:** done 2026-10-16
- **Result:** H1 confirmed. A: `F` 1100 and `Zd` 0 for 12 s, floor skim at about 1555 µs. B: `F` 1101, `Zd` 0 → 100 in 2.5 s, held 84-94 cm. H2 ruled out ( `Tp` 1500 ); H3 ruled out ( ALT HOLD on ). No restarts ( TESTING.md §2 ).

### 3. Workaround flight: Arm + `DesiredPosition_setRelative`

**Description.** Fly a third sketch: `Command_Arm ( )`, then `DesiredPosition_setRelative ( Z, 100 )` once arming is
confirmed. This checks the path the scout named as the one that works after `Command_Arm`, and shows whether arming
without the ground idle ( `isThrottleStickArmed` not set ) causes a hop or a jolt at lift-off. The result tells users
what to do now and gives the fix options a baseline. Drop this task if the user does not want a third flight.

- **Depends on:** 2
- **Skills / agent:** pluto-flighttest
- **Files:** `test/PlutoPilot_C_arm_setrelative.cpp.txt`, `log-3.txt`, `TESTING.md`
- **Safety impact:** flight test
- **Done when:** the log shows a climb to about 100 cm, or why not; lift-off behaviour noted
- **Status:** done 2026-10-16
- **Result:** Workaround works. `Command_Arm` + `DesiredPosition_setRelative ( Z, 100 )` in one tick: `Zd` 1 → 101 in 2.5 s, held 80-97 cm, the same as B, no lift-off hop ( log-3, TESTING.md §3 ). The ground idle cannot be tested by any API path and is not part of the fault.

### 4. Fix options compared

**Description.** For the user to pick from, write up both options with file:line, without changing code. Option (a):
`takeOff ( )` sets the climb goal even when the drone is already armed. Cover the laser and baro branches, the ground
idle and `isThrottleStickArmed`, `isTakeOffHeightSet`, the stick cancelling the goal, battery auto-land and failsafe,
and the API version bump. Option (b): docs only, covering the `FC-Control.h:98` comment ( "height in units" ), a new
FC-Control section in the `docs/API/` wiki, and `User_Space_API.md`. Note the doc drift the scout found
( `User_Space_API.md:15,20-21` names APIs that do not exist ). Give the flash cost and the safety impact of each option. Also cover the `Command_Land ( )` default ( found in the
task 1 review ). Speed 105 gives `landThrottle` 1200 ( `FC-Control-Command.cpp:145` ), which is already
`LAND_CONFIRM_THROTTLE` ( `command.cpp:155` ), so `conclusive` holds when the landing starts. If the drone's descent
stays slower than 8 cm/s for 300 ms ( `command.cpp:217-224` ), `finishLanding ( )` disarms it in the air. The failsafe
and battery landings start at 1300 and are not affected. Also note: `updateCommandStatus ( )` has no callers, so
`current_command` stays LAND after a landing.

- **Depends on:** 2
- **Skills / agent:** pluto-rules
- **Files:** `INVESTIGATION.md` ( options section )
- **Safety impact:** none ( analysis only )
- **Done when:** both options are written with file:line, cost and risk, and one is recommended
- **Status:** done 2026-10-16
- **Result:** INVESTIGATION.md §Fix options. Recommended: (a) `takeOff ( )` also sets the goal when armed, but only on the ground and only once per arm; it releases the ground idle; about 100 B; API 1.7.0. Plus land fix candidate 1 ( `conclusive` only after the ramp has run ), plus (b)'s docs. New fact: an ARM-switch arm with the stick at 1500 holds the ground idle, which wipes any goal.

### 5. Findings

**Description.** Write the Findings section of `INVESTIGATION.md`: each cause ranked as confirmed or ruled out, with the
log numbers from tasks 2-3; the workaround users can use now; and the recommended option from task 4. Update the
README status and open items.

- **Depends on:** 2, 3, 4
- **Skills / agent:** -
- **Files:** `INVESTIGATION.md`, `README.md`
- **Safety impact:** none
- **Done when:** the findings are written, and the user picks option (a) or (b) or stops here
- **Status:** done 2026-10-16
- **Result:** INVESTIGATION.md §Findings. H1 confirmed in flight; H2 and H3 ruled out; H4 and the ground idle are not factors. Found D1 ( high: TakeOff / Flip during a landing ), D2, D3, D4, D5, plus workarounds. The user chose to plan the fix in a new linked topic.

### 6. Close

**Description.** Either `/pluto-commit` records the findings and sets the topic Closed ( no firmware build ), or the
user continues with `/pluto-grill fix takeoff-api-ground-hover`. That would add the fix tasks to this topic or open a
linked fix topic. Restore the template `PlutoPilot.cpp` before committing.

- **Depends on:** 5
- **Skills / agent:** pluto-commit, or pluto-grill fix
- **Files:** `README.md`, `active-development/README.md`
- **Safety impact:** none
- **Done when:** the topic is Closed or handed to a fix plan
- **Status:** done 2026-10-16
- **Result:** Handed over to the new fix topic [takeoff-land-command-fix](../takeoff-land-command-fix/README.md). The findings are recorded at its commit.

### 7. Check: `Command_TakeOff` re-run on a Dev Mode restart cancels a LAND

**Description.** Found while writing task 1. On `Rx_ESP`, `onLoopStart ( )` runs again after every RC frame gap over
200 ms, about every 3 s ( pluto-flighttest §1 ). User code that calls `Command_TakeOff` there calls it again in flight.
`Command_TakeOff` replaces any command other than TAKE_OFF ( `FC-Control-Command.cpp:126` ), LAND included. `takeOff ( )`
then sees ARMED and finishes ( `command.cpp:97-103` ), so a running `Command_Land` would be dropped and the drone would
hold height. Confirm from the code whether anything protects LAND, and whether the battery auto-land is safe
( `mw.cpp:1252` says user commands are replaced by LAND again ). If the logs from task 2 show a restart during the
landing, use them as evidence. Record the result in `INVESTIGATION.md` as its own finding. No code change.

- **Depends on:** 2
- **Skills / agent:** pluto-rules; pluto-flighttest for the restart markers ( `S:` ) in log-1 / log-2
- **Files:** [FC-Control-Command.cpp](../../../../src/main/API-Src/FC-Control-Command.cpp), [command.cpp](../../../../src/main/command/command.cpp), [mw.cpp](../../../../src/main/mw.cpp)
- **Safety impact:** none ( analysis only ). The hazard itself is to landing
- **Done when:** confirmed or ruled out with file:line; battery auto-land status stated
- **Status:** done 2026-10-16
- **Result:** Confirmed from the code. `Command_TakeOff` or `Command_Flip` during a landing cancels `land ( )` but leaves `isLanding` set. The throttle stays pinned at the frozen ramp value, nothing disarms after touchdown, and the stick is locked out; only the ARM switch ( kill switch ) stops the motors. This hits the user LAND and the RC-loss failsafe once the link returns. Battery auto-land is safe. Fix: refuse those commands while landing ( INVESTIGATION.md ).

## Decisions log

Newest last. One line each: `YYYY-MM-DD [decision|assumption|out-of-scope|risk] text`.

- 2026-10-16 [decision] Mode `check`; target PRIMUS_X2_v1.
- 2026-10-16 [assumption] `Command_Arm ( )` and `Command_TakeOff ( 100 )` were both called once, in `onLoopStart ( )`, in the same tick, with ALT HOLD on and the stick centred ( user ).
- 2026-10-16 [decision] Evidence is an A/B flight with a diagnostic log ( Arm + TakeOff, then TakeOff alone ).
- 2026-10-16 [decision] The findings compare option (a) behaviour change ( API bump ) and option (b) docs only; the user picks.
- 2026-10-16 [out-of-scope] Any firmware change in this topic; the fix goes through `/pluto-grill fix takeoff-api-ground-hover`.
- 2026-10-16 [risk] H1 is proven only from the code; `isThrottleStickArmed` and the ground idle are from the scout brief and are not yet checked line by line.
- 2026-10-16 [decision] Test versions call `Command_Arm` / `Command_TakeOff` only on the first `onLoopStart ( )` while disarmed: identical to the user's first call, without re-calls on a Wi-Fi flicker ( H4 left out on purpose ).
- 2026-10-16 [risk] `Command_Land ( )` default 105 starts at `LAND_CONFIRM_THROTTLE` 1200 and could disarm the drone in the air ( reviewer, `command.cpp:155,217-224`; not proven to fire ). Test versions use `Command_Land ( 5 )`; added to task 4.
- 2026-10-16 [risk] `Command_TakeOff` in `onLoopStart ( )` re-runs on every Wi-Fi flicker and can overwrite a running LAND ( task 7 added ).
- 2026-10-16 [assumption] The ground idle is not a variable in the A/B flights: both arm through `mwArm ( )`, so `isThrottleStickArmed` stays 0 in both. Only task 3 can show the idle part of H1.
- 2026-10-16 [decision] Logging starts with Dev Mode ( user ): no disarmed lines before take-off. The invalid-run check is now the third digit of `F` ( `isThrottleStickArmed` = 1 means the ARM switch armed the drone before Dev Mode ), not armed lines before `S:`.
- 2026-10-16 [decision] Task 2: H1 confirmed in flight ( log-1 / log-2 ); H2 and H3 ruled out.
- 2026-10-16 [decision] Version C calls `DesiredPosition_setRelative ( Z, 100 )` in the same tick as `Command_Arm ( )` ( review: nothing resets the write before the controller takes it as the goal, `altitudehold.cpp:737-740` ).
- 2026-10-16 [assumption] The ground idle is not part of this fault: only the ARM-switch path sets `isThrottleStickArmed` ( `rc_controls.cpp:175` ), no API arming path does, and B climbed without it. Task 3 cannot test the idle ( review ).
- 2026-10-16 [decision] Corrected TESTING.md: API arming does not re-zero the baro reference ( `baroResetGroundLevel` only on a re-arm while armed, from the ARM-switch path ); read climbs relative to the first `Ze`.
- 2026-10-16 [decision] Task 3: the workaround `Command_Arm ( )` + `DesiredPosition_setRelative ( Z, 100 )` is flight-proven ( log-3 ).
- 2026-10-16 [risk] An ARM-switch arm with the stick low keeps the ground idle while the stick is at or below 1500 ( `altitudehold.cpp:581-583,688` ), which wipes any goal. `Command_TakeOff` or setRelative after such an arm would not climb ( from the code, not flown ). Option (a) must release the idle.
- 2026-10-16 [decision] Task 4 recommendation: option (a) with an on-ground guard + land fix candidate 1 + option (b) docs. The user decides at `/pluto-grill fix`.
- 2026-10-16 [risk] Task 7 confirmed from the code, not flown: `Command_TakeOff` / `Command_Flip` during a landing leaves `isLanding` set with `land ( )` stopped. Motors keep running after touchdown, the stick is locked out, only the ARM-switch kill stops them. A later `Command_Land` after 30 s disarms at once, possibly in the air. User LAND and RC-loss failsafe ( after the link returns ) are exposed.
- 2026-10-16 [decision] The scout's "calling Command_TakeOff every loop is harmless" is wrong: it overwrites every landing.
- 2026-10-16 [decision] The user chose: plan the fix with `/pluto-grill fix takeoff-api-ground-hover`, as a **new linked fix topic**. This check topic closes with a link to it.
- 2026-10-16 [decision] Closed, handed over to `takeoff-land-command-fix`. The guard is a 3 s window or ground idle, not 30 cm ( user ).
