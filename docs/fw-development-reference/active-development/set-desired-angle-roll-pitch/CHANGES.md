# Set desired angle roll / pitch - Changes

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

Line numbers are at the working copy after each task.

## Task 1: test sequence ( temporary )

| File | Lines | Change | Why |
|---|---|---|---|
| [PlutoPilot.cpp](../../../../PlutoPilot.cpp) | whole file | Automatic roll / pitch step sequence on Developer Mode on, with one `setDesiredAngle` call per step and a 10 Hz log ( ~101 B per tick ). | Reproduce T11 / B4 and later validate the fix with the same sequence. **Remove before release.** |

## Task 2: the setter stores the user angle

| File | Lines | Change | Why |
|---|---|---|---|
| [FC-Config-Setpoints.cpp](../../../../src/main/API-Src/FC-Config-Setpoints.cpp) | 90-99 | `AG_ROLL` / `AG_PITCH` store `constrain ( angle, -1800, 1800 )` into `userDesiredAngle [ ANGLE ]` and set `isUserDesiredAngle [ ANGLE ]`, instead of writing `inclination.values.roll/pitchDeciDegrees`. | The old write corrupted the attitude estimate for the rest of the loop ( the "read once" ) and could trip the crash-angle disarm ( above 700 ). The +/-180 deg bound only keeps task 3's sum in range; the real limit is the `max_angle_inclination` clamp. |
| same | 122-124 | `DesiredAngle_setRelative` comment: roll / pitch are absolute tilts and a relative call on them is ignored. | The old comment said roll / pitch are re-derived from the sticks, which the setter never relied on. |
| same | 14 | Banner `Last Modified` 14 Nov 2026. | Banner kept current. |

`userDesiredAngle [ 3 ]` and `isUserDesiredAngle [ 3 ]` already existed in [API-Utils.h](../../../../src/main/API/API-Utils.h)
( :111 / :116 ) and [API-Utils.cpp](../../../../src/main/API-Src/API-Utils.cpp) ( :138 / :143 ), unused, so no new
globals were added. They are indexed by `angle_e`: `AG_ROLL` = 0 and `AG_PITCH` = 1 ( `common/axis.h:52` ), the same as the
`pidRewrite` axis index ( `FD_ROLL` = 0, `FD_PITCH` = 1 ).

After task 2 alone, roll / pitch are stored but not yet flown ( task 3 ), and `inclination` is written only by
`flight/imu.cpp` ( :425-426, :1185-1186 ).

## Task 3: the user tilt in the pidRewrite ANGLE target

| File | Lines | Change | Why |
|---|---|---|---|
| [API-Utils.cpp](../../../../src/main/API-Src/API-Utils.cpp) | 187-202 | New `userAngleTarget ( axis )`: returns `userDesiredAngle [ axis ]` for `FD_ROLL` / `FD_PITCH` when `isUserDesiredAngle [ axis ]` is set, ANGLE mode is on, `flipState == 0` and `! landingInProgress ( )`; otherwise 0. | One place for the gates. A flip owns roll / pitch, and a held tilt during a landing would drift the descent. The target stays held while gated off. |
| [API-Utils.h](../../../../src/main/API/API-Utils.h) | 123-129 | Declares `landingInProgress ( )` and `userAngleTarget ( )` ( inside the header's `extern "C"` block ). | Used by `pid.cpp` and `API-Utils.cpp`. |
| [FC-Control-Command.cpp](../../../../src/main/API-Src/FC-Control-Command.cpp) | 130-131 | `landingInProgress ( )` is no longer `static`. | Reused as the landing test, so there is one definition ( user, RC-loss and battery landings ). |
| [pid.cpp](../../../../src/main/flight/pid.cpp) | 797-799, 803-805 | `pidRewrite`: `desiredAngle [ axis ] = 2 * rcCommand [ axis ] + userAngleTarget ( axis )` ( and `+ GPS_angle` in the `#ifdef GPS` branch ), before the existing `max_angle_inclination` clamp. | The fix: the user tilt reaches the ANGLE controller. The stick adds on top and the sum keeps the pilot's limit ( 20 deg by default ). |

Not changed: the other five PID controllers ( MultiWii, LuxFloat, MW23, Hybrid, Harakiri ). They are selectable only through
stored config, the default is `pidRewrite` ( `config.cpp:176` ), and they ignore the user tilt. `GPS` is commented out on
every target, so the `#ifdef GPS` hunk is not compiled.

Cost: Flash 122.3 -> 122.5 KB ( `userAngleTarget` 68 B, `landingInProgress` 40 B now global ), RAM unchanged.

## Task 4: clear at dev-off and on the disarm edge

| File | Lines | Change | Why |
|---|---|---|---|
| [mw.cpp](../../../../src/main/mw.cpp) | 1256-1265 | In `userCode ( )`, right after the yaw clear: `if ( ! devSwitchOn \|\| disarmed )` sets `isUserDesiredAngle [ FD_ROLL..FD_PITCH ] = false` and `userDesiredAngle [ ] = 0`. | Same rules as the yaw target. A Wi-Fi flicker keeps the tilt ( `devSwitchOn` holds through RX loss ), a real dev switch-off levels the craft, and the armed -> disarmed edge drops it, so a re-arm never takes off tilted. A tilt set on the ground before arming survives to takeoff ( edge, not level ). `userCode ( )` runs before `pid_controller` in the same loop, so the PID already sees 0. |
| same | 14, 28 | Banner `Last Modified` 14 Nov 2026 and a HISTORY row. | Banner kept current. |

Two safety gates were added to `userAngleTarget ( )` from the task 4 review, so the helper now also returns 0 when:

| File | Lines | Change | Why |
|---|---|---|---|
| [API-Utils.cpp](../../../../src/main/API-Src/API-Utils.cpp) | 61, 189-224 | Developer Mode has not run for `USER_ANGLE_LINK_GRACE_MS` ( 600 ms ), tracked from `devmode` ( include `io/oled_display.h` ). | During an RC stall user code stops and the pilot's sticks fall back to centre, but a held tilt would keep accelerating the craft for the ~2.5 s until the failsafe landing ( about 5 m at 10 deg ). The window is the pilot's own stick hold, so a normal 200 ms Wi-Fi flicker keeps flying the tilt instead of levelling and stepping back. The target is suppressed, never dropped, and resumes with the link. |
| same | 189-224 | `! throttleRaisedInFlight ( )` or `getEstAltitude ( ) < USER_ANGLE_MIN_ALT_CM` ( 20 cm above the arm reference ). | A tilt set before arming would otherwise be commanded from the arm instant and the take-off could pivot on a leg. "Throttle raised" alone is not airborne: an app take-off arms with the throttle stick centred, so it is true on the skids. |
| [API-Utils.h](../../../../src/main/API/API-Utils.h) | 66-78 | The two constants, with their reasoning. | Tunable in one place. |
| [mw.h](../../../../src/main/mw.h) | 41-46 | Declares `throttleRaisedInFlight ( )`. | `throttleRaisedSinceArm` is file-static in `mw.cpp`. |
| [mw.cpp](../../../../src/main/mw.cpp) | 184-190 | The accessor. | Same flag the baro datum uses. |

## Task 5: API header and wiki

| File | Change |
|---|---|
| [FC-Config.h](../../../../src/main/API/FC-Config.h) | `getDesiredAngle`: roll / pitch is the stick plus the user tilt being flown, before the limit; yaw returns 0. `setDesiredAngle`: the roll / pitch tilt, its hold, the conditions it is flown under, when it ends, and the +/-1800 bound. |
| [YAW_HEADING_API_WIKI.md](../../../API/YAW_HEADING_API_WIKI.md) | New section "Roll and pitch tilt"; example "Lean right for 1 s, then level"; three pitfalls ( HEADFREE frame, a held tilt keeps the drone moving, no tilt on the ground ); "Changes in 1.8.6" ( B4 fix, `getDesiredAngle` ). Header set to API 1.8.6, the version task 10 bumps to. |

Every statement was checked by hand against a source line ( `spec-to-code-compliance` is not available in this
session ):

| Statement | Source |
|---|---|
| Stored and held until replaced, bounded to +/-1800 | `FC-Config-Setpoints.cpp:97-98` |
| Added to the stick ( `2 * rcCommand` ), then limited to `max_angle_inclination` | `pid.cpp:807-808` |
| The limit is 20 deg by default | `config.cpp:420` ( 200 deci-degrees ) |
| Drone tilts until `Estimate_Get ( Angle )` reads the value | `pid.cpp:808` ( error against `inclination_generalised` ), `imu.cpp:1188-1189` ( a copy of `inclination` ), `FC-Data-Estimate.cpp:130-133` |
| Flown only in ANGLE mode, no flip, no landing | `API-Utils.cpp:217` |
| Airborne: throttle raised and 20 cm above the arm reference | `API-Utils.cpp:218`, `API-Utils.h:78` |
| Developer Mode, 0.6 s grace | `API-Utils.cpp:216`, `API-Utils.h:73` |
| Dropped at dev-off and on disarm; kept if set while disarmed | `mw.cpp:1269` ( edge, not level ) |
| Default PID controller only | `pid.cpp:807` ( only `pidRewrite` calls it ), `config.cpp:176` |
| Not rotated by HEADFREE | added in `pid.cpp:807`, after the HEADFREE `rcCommand` rotation in `annexCode` ( `mw.cpp:430` ) |
| `getDesiredAngle` roll / pitch is pre-limit; yaw is 0 | `FC-Config-Setpoints.cpp:84`; `desiredAngle` is written only at `pid.cpp:800 / 807`, roll and pitch only |
| Old behaviour: wrote the attitude; > 700 could disarm | `INVESTIGATION.md`; `failsafe.cpp:368` |

## File index

| File | Tasks |
|---|---|
| `PlutoPilot.cpp` | 1 ( temporary ) |
| `src/main/API-Src/FC-Config-Setpoints.cpp` | 2 |
| `src/main/API-Src/API-Utils.cpp` | 3 |
| `src/main/API/API-Utils.h` | 3 |
| `src/main/API-Src/FC-Control-Command.cpp` | 3 |
| `src/main/flight/pid.cpp` | 3 |
| `src/main/mw.cpp` | 4 |
| `src/main/mw.h` | 4 |
| `src/main/API/FC-Config.h` | 5 |
| `docs/API/YAW_HEADING_API_WIKI.md` | 5 |
