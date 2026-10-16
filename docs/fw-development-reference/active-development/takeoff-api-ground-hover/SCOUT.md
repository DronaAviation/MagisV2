# Take-off API ground hover - Scout brief

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md)

`pluto-scout` brief from planning ( 16 Oct 2026 ), verbatim. Graph was stale ( built `edfbd615`, HEAD `794002dc` ).

---

## Scout brief: Command_Arm ( ) followed by Command_TakeOff ( 100 ) spins up but stays at ground level

Subsystems: User API commands, altitude hold → `subsystems/User_Space_API.md`, `subsystems/Altitude_Hold_Estimator.md`
Key code:
- `src/main/API-Src/FC-Control-Command.cpp:124` `Command_TakeOff` sets `current_command = TAKE_OFF` and `takeOffHeight = constrain ( h, 100, 250 )` (cm). It does no arming check.
- `src/main/API-Src/FC-Control-Command.cpp:157` `Command_Arm` calls `mwArm ( )` when BOXARM is on and `OK_TO_ARM` is set.
- `src/main/command/command.cpp:93` `takeOff ( )` runs from `executeCommand ( )` at the end of the loop (mw.cpp:1491).
- `src/main/flight/altitudehold.cpp:581` ground idle: only when `isThrottleStickArmed`.
- `src/main/flight/altitudehold.cpp:688` resets the controller when disarmed or idle-held.
- `src/main/io/rc_controls.cpp:169-190` the ARM-switch arming paths (these set `isThrottleStickArmed` and `isTookOff`).
- `src/main/mw.cpp:591` `mwArm`.

Mode: check
Current behaviour: `takeOff ( )` sets the climb goal only in the branch where it does the arming itself (command.cpp:107-123, `DesiredPosition_setRelative ( Z, takeOffHeight )`). If the craft is already armed, it marks `FINISHED` and `isTookOff = true` straight away (command.cpp:97-103). `AltHold` then stays at `EstAlt` from the arm instant (`resetAltSetpoint`, altitudehold.cpp:435).

Analysis:
1. **Most likely, and proven by reading the code: arming first turns the take-off into a no-op.** `Command_Arm` arms in `plutoLoop`, and the next `executeCommand` sees ARMED, so no height is ever set. Because `Command_Arm` does not set `isThrottleStickArmed`, there is no ground idle (altitudehold.cpp:581). ALT_HOLD therefore runs from `initialThrottleHold` 1500 and holds the ground height: the motors sit at hover thrust and the craft skims the floor. To confirm, `Monitor_Print` `getDesiredPositions ( Z )` against `Estimate_Get` Z after the call: the target equals ground height. To kill it, call `Command_TakeOff ( 100 )` alone (it arms by itself) and check whether the craft climbs about 100 cm in about 2.7 s.
2. **The throttle stick is not centred.** `deflection = rcData [ THROTTLE ] - 1500` with deadband 40 (altitudehold.cpp:545). Any `setVelocity != 0` cancels a take-off goal (altitudehold.cpp:741). A low stick asks for -30 cm/s. Evidence against: with the stick low (< mincheck 1100) the ARM switch arms through rc_controls with ground idle, so the motors would idle rather than hover. Test: log `RcData_Get ( RC_THROTTLE )`.
3. **BARO_MODE is off** (no ALT_HOLD in the app, or `FlightMode_Set ( ATLTITUDEHOLD )` not called). Then no height target is flown at all and the throttle is direct. Test: log `FlightMode_Check ( ATLTITUDEHOLD )`.
4. **Calling `Command_Arm` every loop re-zeroes the baro datum** through `mwArm` → `baroResetGroundLevel` while `!throttleRaisedSinceArm` (mw.cpp:597). This is unlikely to matter, because a stick above 1100 latches `throttleRaisedSinceArm` on the next `annexCode` (mw.cpp:399).

Constraints / invariants that apply:
- The baro datum freezes on arm (CLAUDE.md flight invariants).
- Use `rcDataPilot` for the throttle.
- Take-off is a goal on the goal profile, and the stick cancels it (`Altitude_Hold_Estimator.md`:78).
- A battery auto-land replaces `Command_TakeOff`.

Resource conflicts: none

Already answered by the code:
- The height is in cm, clamped to 100-250. The header default is 150, but disarm resets the stored value to 120 (mw.cpp:562).
- User code runs only with Dev Mode on and an RC link. `plutoInit` runs before this, so arming there cannot work.
- `Command_TakeOff` must be called while disarmed, with the ARM switch on and `OK_TO_ARM` set (that flag is only set while ARM is off, mw.cpp:469-478).
- Calling it every loop is harmless: it re-enters TAKE_OFF after it finishes. But a `Command_Land` in the same tick is refused, because the status is RUNNING.
- After `Command_Arm`, the supported way to climb is `DesiredPosition_setRelative ( Z, 100 )`.

Questions for the user:
1. The exact `PlutoPilot.cpp`: which hook, whether both calls run every loop, and any other API calls.
2. Were the app's ARM and ALT HOLD switches on, and where was the throttle stick (centre or low)?
3. Should `Command_TakeOff` work after `Command_Arm` (a behaviour change, API bump), or is a doc/wiki fix enough?
4. Which board was flown (PRIMUS_X2_v1 or V5), and was `XRanging.initAlt` used?
5. Did the motors spin at hover with the craft skating, or did it hop and settle?

Doc drift:
- `FC-Control.h:98` says "height in units" and does not say that the call arms the craft or that it does nothing when already armed.
- `User_Space_API.md:15,20-21` names `Control_Command_t`, `Command.set`, `Estimate.get`, which do not exist in `src/main`.
- There is no `docs/API` wiki for FC-Control.

Prior work: `althold-setpoint-shaping` (2a8d59a added the goal profile and `altHoldGroundIdle`). `flip-althold-regression` (c569c0f). edfbd61 added the laser/baro take-off height split (command.cpp:117). The "already armed → finish" branch dates from 9592044 (old, not a regression).

Suggested skills: pluto-flighttest, pluto-rules; agent pluto-log-analyst
Pipeline docs to update at the end:
- `User_Space_API.md`: take-off preconditions, and remove the nonexistent APIs.
- `Altitude_Hold_Estimator.md`: note that the take-off goal is set only when `Command_TakeOff` does the arming.
