# Set desired angle roll / pitch - Investigation

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md)

## Problem

API test T11 ( testing team, finding B4 ), on `setDesiredAngle ( angle_e ANGLE, int32_t angle )` from `API/FC-Config.h`
( AG_YAW bearing 0..359 deg; roll / pitch deci-deg ):

> T11: AG_YAW OK. AG_ROLL 100: read once, then overwritten, no tilt ( B4 ).

How the testing team ran T11 is not known ( where the call was placed, how the value was read back ). Task 1 reproduces
it with our own test code.

## Evidence ( code, HEAD `ebf4e46` )

| Step | Where | What happens |
|---|---|---|
| 1 | [FC-Config-Setpoints.cpp:92 / :96](../../../../src/main/API-Src/FC-Config-Setpoints.cpp) | `AG_ROLL` / `AG_PITCH` write `inclination.values.rollDeciDegrees` / `pitchDeciDegrees`: the IMU's **measured** attitude, not a setpoint. |
| 2 | [FC-Data-Estimate.cpp:130](../../../../src/main/API-Src/FC-Data-Estimate.cpp) | `Estimate_Get ( Angle, AG_ROLL )` reads `inclination`, so it returns 100 for the rest of that loop: the "read once". |
| 3 | [pid.cpp:801-803](../../../../src/main/flight/pid.cpp) | `pidRewrite` sets `desiredAngle[axis] = 2 * rcCommand[axis]`, clamps it to `max_angle_inclination` ( 200 = 20 deg ) and measures against `inclination_generalised`, which the setter never touches. The user value never reaches the controller: no tilt. |
| 4 | [imu.cpp:1185-1189](../../../../src/main/flight/imu.cpp) | The next `imuUpdate` writes the true attitude back into `inclination`: the "overwritten". |

Loop order ( `mw.cpp` ): `imuUpdate` -> `annexCode` -> `userCode` ( `plutoLoop` ) -> `pid_controller` -> `failsafeOnCrash`.

**Hazard in the current code:** `failsafe.cpp:368` `angleTrip` reads `inclination` later in the same loop. A user value
above 700 ( 70 deg ) while armed in ANGLE mode with crash failsafe on ( the default ) disarms the drone in the air.

**Same bug:** `AG_PITCH` ( :96 ). **Not affected:** `DesiredAngle_setRelative` returns early for roll and pitch
( :123 ), but its comment at :120-122 claims the opposite of what the code did. `userDesiredAngle[3]`
( `API-Utils.h:111`, `API-Utils.cpp:138` ) is declared and defined but nothing uses it.

## Reproduction ( log-1, 14 Nov 2026 )

Our own automatic test sequence ( task 1 ) reproduces B4 on HEAD `ebf4e46`. On each step-change tick,
`Estimate_Get ( Angle )` returns the commanded +/-50 exactly. From the next tick on it is back in the hover band, and
+50 and -50 give the same mean angle on both axes, so the drone does not respond. Heading is unchanged. Details:
[TESTING.md](TESTING.md#log-1-pre-fix-reproduction--task-1-).

## Root cause

The roll / pitch branch of `setDesiredAngle` writes the attitude estimate instead of the ANGLE-mode setpoint. The
controller has no input for a user roll / pitch angle at all.

## Q&A that shaped the plan ( 14 Nov 2026 )

| Question | Answer |
|---|---|
| Mode | `fix`. The user does not know how T11 was produced: reproduce it with our own test first. |
| Target | `PRIMUS_X2_v1`. |
| Done when | The API behaves as documented, with no unusual change: the drone tilts to the commanded angle. |
| Hold rule | Hold until replaced, the same as `AG_YAW` ( calling it every loop also works ). |
| Fix | Option B: a user angle term in the `pidRewrite` ANGLE target ( not the RC override path, option A ). |
| Limits / stick / safety | "As it is now": no new limits. The user angle plus the stick goes through the existing `max_angle_inclination` clamp. ANGLE mode only. Cleared at dev-off and disarm ( the yaw pattern ), not applied during a flip or a landing. |
| Verification | The user flies in a small room. Test code runs automatically when dev mode turns on: a short sequence of small tilts, then done. The user turns dev mode off and pastes the log. |
| Out of scope | The yaw path, any new API, other T-cases / B-items. |
