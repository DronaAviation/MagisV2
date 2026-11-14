# Set desired angle roll / pitch - Scout brief

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md)

The `pluto-scout` brief from planning ( 14 Nov 2026, HEAD `ebf4e46` ), kept verbatim apart from symbols written in
ASCII ( `->`, `+/-`, `x` ).

## Scout brief: T11 / B4, `setDesiredAngle ( AG_ROLL, 100 )` read once, overwritten, no tilt

Graph: stale ( built 6d1b5cd9, HEAD ebf4e46 ); no edge setDesiredAngle -> pidRewrite ( 18-hop unrelated path ).

**Subsystems:** User API, IMU, PID ( ANGLE ) -> pipeline docs `fw-architecture-pipeline/subsystems/User_Space_API.md`,
`Core_PID_Controller.md`

**Key code:**
- `src/main/API-Src/FC-Config-Setpoints.cpp:92 / :96` - `setDesiredAngle` writes `inclination.values.roll/pitchDeciDegrees`
  ( the IMU's measured attitude, not a setpoint ).
- `src/main/flight/imu.cpp:1185-1189` `imuUpdate` - rewrites `inclination` + `inclination_generalised` every loop.
- `src/main/flight/pid.cpp:801-803` `pidRewrite` ( active, `pidController = 1`, `config.cpp:176` ) -
  `desiredAngle[axis] = 2*rcCommand[axis]`, clamped to +/-`max_angle_inclination` ( 200 = 20 deg, `config.cpp:420`; comment
  says 50 deg, stale ); error uses `inclination_generalised` ( never touched by the setter ).
- `src/main/mw.cpp:1377 / 1410 / 1416 / 1487 / 1516` - loop order imuUpdate -> annexCode -> userCode -> pid_controller ->
  failsafeOnCrash.
- `src/main/API-Src/FC-Data-Estimate.cpp:130` - `Estimate_Get ( Angle, AG_ROLL )` reads `inclination` -> the "read once".
- `src/main/flight/failsafe.cpp:368` - `angleTrip` reads `inclination` ( > 700 ) later the same loop.
- `src/main/mw.cpp:1140` `applyUserRcOverride` - roll/pitch `rcCommand` cross-fade after annexCode; expiry
  `API-Utils.cpp:196`.
- `src/main/API/API-Utils.h:111` - `userDesiredAngle[3]` declared, defined `API-Utils.cpp:138`, **unused anywhere**.

**Mode:** fix

**Current behaviour:** the roll/pitch branch corrupts the attitude estimate; the PID never reads `inclination`, so it is
never commanded. Later same-loop code sees the fake angle; the next `imuUpdate` overwrites it.

**Analysis ( causal chain, PROVEN from code ):** user code calls the setter in `plutoLoop` ( mw.cpp:1416 ) -> writes the
estimate ( Setpoints.cpp:92 ) -> within that loop `Estimate_Get` / OLED / `failsafeOnCrash` see 100 ( "read once" ) ->
`pidRewrite` builds its setpoint only from `rcCommand` and measures against `inclination_generalised` ( pid.cpp:801-803 ):
the user value never reaches it, nothing tilts -> next loop `imuUpdate` ( imu.cpp:1185 ) writes the true angle back
( "overwritten" ). **Latent hazard:** a value > 700 while armed in ANGLE with `fsCrash` ( default true,
API-Utils.cpp:123 ) trips `angleTrip` failsafe.cpp:368 -> `mwDisarm` in the air. AG_PITCH has the identical bug ( :96 ).
`DesiredAngle_setRelative` returns early for roll/pitch ( Setpoints.cpp:123 ), so it is unaffected, but its comment
:120-122 is wrong. `getDesiredAngle` ( Setpoints.cpp:84 ) returns pidRewrite's unclamped `2*rcCommand`; for AG_YAW
always 0 ( pidRewrite never writes `desiredAngle[2]` ).

**Fix options:**
- **A ( smallest ):** map the angle onto the existing roll/pitch override: `RC_ARRAY[axis] = constrain(angle,
  -max_angle_inclination,+max)/2; userRCassert`. Files: Setpoints.cpp only. ~50 B flash, 0 RAM, 0 loop. Inherits the stick
  cross-fade, expiry ( max(250 ms, 2x user loop) -> must be called every loop, like RcCommand_Set ), the drop on RC
  loss / dev-off ( mw.cpp:1217-1221 ) and the dev-switch-off drop. Only acts in ANGLE ( in ACRO/HORIZON it becomes a
  rate ). Bypasses the HEADFREE rotation ( mw.cpp:423 ) as RcCommand_Set already does. Conflicts with a roll/pitch
  `RcCommand_Set` ( last writer wins ).
- **B ( proper ):** store in `userDesiredAngle[]` + a held flag; in `pidRewrite` `desiredAngle = userAngle +
  2*rcCommand` ( or cross-fade ) only when ANGLE_MODE && dev mode && `flipState == 0` && not landing; clear on dev-off
  and the disarm edge mirroring the yaw clear at mw.cpp:1243-1248. Files: Setpoints.cpp, pid.cpp, mw.cpp, API-Utils.
  ~150-250 B flash, ~8 B RAM, a few cycles per loop.
- **C:** A + fix `getDesiredAngle` to return the flown clamped setpoint ( and `userHeading` for yaw ).
- Safety for A and B: the held target must die on disarm, RC loss ( A free ), landing ( `Command_Land` ) and in a flip
  ( flips drive `rcData` roll/pitch, acrobats.cpp:126-159; a post-annex override would fight the flip ).

**Constraints / invariants that apply:** the user override cross-fades and expires unless re-asserted ( CLAUDE.md flight
invariant ); landing must not go through the stick limits ( a held roll during LAND drifts ); a landing cannot start in
a flip, and a flip owns roll/pitch; the yaw clear pattern ( dev-off + disarm edge, not a flicker ) from relative-yaw-api;
an API behaviour change -> wiki + `API_Version` bump ( now FW 3.15.7 / API 1.8.5 ); `-Wconversion`: cast int32 -> int16
explicitly.

**Resource conflicts:** none.

**Already answered by the code:** units deci-degrees ( FC-Config.h:118, YAW_HEADING wiki:38, already documented );
effective limit +/-200 ( 20 deg ); only ANGLE mode self-levels to an angle; pitch has the same bug; the relative setter
ignores roll/pitch; no doc promises hold, expiry or mode behaviour for roll/pitch.

**Questions for the user:**
1. Hold until replaced, or must it be re-asserted every loop like RcCommand_Set?
2. Pilot stick: cross-fade over it, or add to it?
3. ACRO/HORIZON: ignore the call, or force ANGLE?
4. Clamp: `max_angle_inclination` ( 20 deg ) or a separate user limit?
5. On disarm / landing / flip / failsafe: drop ( proposed: drop on all )?
6. The exact T11 test code: where was the call ( onLoopStart or plutoLoop ), and how was the reading made ( Estimate_Get? )?
7. Should `getDesiredAngle` return the user target or the flown setpoint?
8. HEADFREE in use ( body-frame vs earth-frame tilt )?

**Doc drift:** Setpoints.cpp:120-122 comment ( "re-derived from sticks" ; the setter never touched the setpoint );
config.cpp:420 comment says 50 deg ( value 200 = 20 deg ); User_Space_API.md:162-163 lists roll/pitch deci-degrees as
working without saying they have no effect.

**Prior work:** relative-yaw-api ( 794002d ) is the hold/clear pattern; rc-command-get-api ( e72d098 ) is the override
plumbing; sibling API fixes ebf4e46 / 6d1b5cd / 1b43fbe / 7a41a37. The topic folder `active-development/fc-config-api-fixes/`
exists with only an empty `logs/`. `setDesiredRate` ( Setpoints.cpp:152 ) writes the persistent rate profile: adjacent,
the user may want it in the same topic.

**Suggested skills:** pluto-rules, pluto-build, pluto-flighttest, pluto-commit; the pluto-reviewer agent.

**Pipeline docs to update at the end:** User_Space_API.md ( roll/pitch semantics, hold/expiry, modes, clear rules ),
Core_PID_Controller.md ( the user angle term in pidRewrite, option B ), YAW_HEADING_API_WIKI.md:38 ( or a new FC-Config
section in FC_CONTROL_API_WIKI.md ), the FC-Config.h doxygen, an `API_Version` bump, and a CLAUDE.md flight-invariant
line if B ships ( the user roll/pitch target is dropped on disarm / flip / landing ).
