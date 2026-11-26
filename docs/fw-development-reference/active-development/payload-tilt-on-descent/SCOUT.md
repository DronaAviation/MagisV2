# Payload Tilt On Descent - Scout brief

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

`pluto-scout` brief from planning ( 27 Oct 2026, HEAD `a81c180` ), kept verbatim so no later session
repeats the reconnaissance. The scout was first given both candidate commits; the user's bisect answer
( `4657cbf` good, `2a8d59a` bad ) was sent to it afterwards and its follow-up, if any, is appended at
the end.

---

## Scout brief: payload on one side makes the drone tilt toward it when the app throttle is lowered (ALT_HOLD + ANGLE)

Graph: stale (built 5e268237, HEAD a81c180). The `graphify` CLI is not on PATH (neither as a command nor as a python module), so I used grep and git instead.

**1. Subsystems:** RC input, Core PID, Mixer, Altitude Hold. Pipeline docs: `subsystems/Core_PID_Controller.md`, `Mixer_Pipeline.md`, `RC_Input_Pipeline.md`, `Altitude_Hold_Estimator.md`.

Key code at HEAD:
- `src/main/mw.cpp:892-898`: `calculateThrottleStatus` then `if ( throttleStatus == THROTTLE_LOW ) { pidResetErrorAngle ( ); pidResetErrorGyro ( ); }`. This runs every loop, armed or not, with no BARO or airborne check.
- `src/main/io/rc_controls.cpp:131-139`: THROTTLE_LOW means raw `rcData[THROTTLE] < mincheck` (1100, `config.cpp:472`).
- `rc_controls.cpp:169-180`: with the ARM box on and THROTTLE_LOW, `mwArm ( )` is called every loop. `:183-187`: with the throttle not low and `netAccMagnitude < 11` (free fall, below about 0.33 g), it resets the PID I-terms and calls `mwArm` (the "chuck arm" path, which has no `!ARMED` guard).
- `src/main/flight/pid.cpp:92-101` `pidResetErrorGyro` and `:735-836` `pidRewrite`. The default `pidController = 1` (`config.cpp:176`) is `PID_CONTROLLER_REWRITE` (`pid.h:46-47`). In ANGLE mode the outer loop is P only (`desiredRate = errorAngle*P8[PIDLEVEL]>>4`, `:803`), so **`errorGyroI` is the only integrator that can cancel a CG offset**. `errorAngleI` is not used by this controller.
- `src/main/flight/mixer.cpp:812-825`: with `pid_at_min_throttle = 1` (`config.cpp:369`) and MOTOR_STOP off, the motors keep the PID output below mincheck. Only overshoot above maxthrottle is redistributed; nothing is redistributed at the low end.
- `src/main/flight/altitudehold.cpp:568-606` (stick to `setVelocity`) and `:759-819` (`shapedVelocitySetpoint`).

**2. Current behaviour on throttle-down (app, ALT_HOLD):**
- The stick only drives altitude hold. Inside 1460-1540 the craft holds altitude. Below 1460 the descent rate rises linearly to `ALT_MAX_DESCENT_CMS` 30 cm/s at 1000. The rate ramps at 100 cm/s² and the velocity setpoint slews at 150 cm/s².
- The velocity-loop P term is `120*err/32`, so a 30 cm/s step gives about -112 counts. Motors sit around hover minus about 100, well above minthrottle 1100.
- In the bottom 10 % of stick travel (`rcData < 1100`), `pidResetErrorGyro ( )` clears the roll and pitch rate I-terms every loop. The craft is then held only by the P terms: about `0.78 * errorAngle(decideg)` counts of motor difference (P8 ROLL/PITCH 40, P8 LEVEL 40). A payload needing 40 counts of difference therefore tilts about 5 deg toward the payload, and the craft drifts that way.
- `I8[ROLL/PITCH]` is only 10 and `GYRO_I_MAX` is 256, so after the stick comes back up the I-term rebuilds slowly.
- Landing is not affected: `LAND_THROTTLE_MIN` 1150 is above mincheck, except `Command_Land ( speed > 205 )`, which starts below 1100.

**3. Diffs that matter.** No commit in range touched `pid.cpp`, `mixer.cpp` or `rc_controls.cpp`. The THROTTLE_LOW reset goes back to 9592044/2daff4eb.
- **4657cbf** (`mw.cpp:606-610`): before it, `mwArm ( )`, which runs every THROTTLE_LOW loop in flight, called `baroResetGroundLevel ( )` every loop. That re-zeroed the baro at the current height for as long as the stick was held below 1100. The estimator was pulled toward 0 m and 0 velocity, so the -100 cm/s velocity loop kept cutting throttle: a fast, near-falling drop (suspected, not logged). After 4657cbf the datum stays frozen and the descent is a controlled -100 cm/s.
- **2a8d59a**: full-stick descent went from `setVelocity = (rcData-1500)/4` clamped to -100 cm/s, applied with no ramp, to 30 cm/s (`ALT_MAX_DESCENT_CMS`) with a ramp and slew. A 1 m stick-down now takes 3 s or more instead of about 1 s.
- Both commits make the time spent airborne with THROTTLE_LOW (I-terms zeroed) longer and the motion calmer, which makes a steady payload tilt and drift visible. Neither adds a new attitude mechanism.

**4. Hypotheses, most likely first**
1. **The reset was always there; the commits only made the stick-down descent slow and controlled** (fits either reading of "right after d2e900c": 4657cbf removed the baro-re-zero drop, 2a8d59a slowed the descent to 30 cm/s). The mechanism is `mw.cpp:895-897` zeroing `errorGyroI` while `rcData < 1100`. For: this is the only roll/pitch integrator in ANGLE mode under pidRewrite, and the tilt follows the payload as a P-only offset would. Against: it needs the app to send below 1100 at full stick down (not verified). Test: same payload, hover, then hold the stick at about 1/3 down (1100-1460, no THROTTLE_LOW) versus fully down. A tilt only at full down confirms it. Monitor_Print: `rcData[THROTTLE]`, roll and pitch angle, motor outputs. `errorGyroI` is file-static, and `axisPID_I` exists only under `BLACKBOX`, so a debug export is needed to see the I-term directly.
2. **The old build also tilted but dropped too fast to notice**: the same test on d2e900c and f60c9ad with full stick down. A tilt there points to hypothesis 1, not to a regression.
3. **Low-end motor saturation** (the light-side motors clip at minthrottle). Against: descent commands are smaller now (about -112 versus -300 counts), and the mixer redistributes only at the high end. Test: log the four motor outputs during a descent and check whether any reaches 1100.
4. **Chuck-arm reset (`rc_controls.cpp:183`) during a drop under 0.33 g**: it can only fire in free fall, so it is unlikely at 30 cm/s.

**5. Facts already established in the code:** a props-off bench test cannot show this. The reset happens in any mode, not only ALT_HOLD, but outside ALT_HOLD a stick below 1100 also cuts the throttle. The reset follows raw `rcData`, so a user `RcCommand_Set` throttle below 1100 would trigger it too.

Questions only the user can answer:
- (a) Fully down, or a partial stick-down?
- (b) Does a partial stick-down also tilt?
- (c) After throttle-up, does the tilt stay for a few seconds?
- (d) Does it happen on the app Land button?
- (e) Which build was the last good one exactly: d2e900c or 4657cbf?
- (f) Payload weight and arm position.
- (g) Was the old build also tested with the stick fully down?
- (h) Target is PRIMUS_X2_v1?

**6. Doc drift:**
- `Mixer_Pipeline.md:22` says minthrottle is about 1070; the code is 1100 (`config.cpp:278`).
- `Core_PID_Controller.md` does not mention the THROTTLE_LOW I-term reset or that controller 1 is pidRewrite; its line 27 describes generic I bounds.
- `Altitude_Hold_Estimator.md:78` (30 cm/s) matches the code.
- `FLIGHT_INVARIANTS.md` has nothing on in-flight I-term resets.

Prior work: `active-development/althold-setpoint-shaping` (TESTING.md row 3 tests full stick down at 30 cm/s and does not check attitude), `altitude-hold` (4657cbf), `takeoff-land-command-fix`, `flip-althold-regression`. No topic touches pid.cpp or the THROTTLE_LOW block.

Constraints:
- rcData versus rcDataPilot (pluto-rules).
- Landing must not go through the stick limits.
- `Monitor_Print` must stay under about 130 bytes per tick.
- A possible fix (gate the reset on `!ARMED || altHoldGroundIdle || !throttleRaisedSinceArm`) must keep the ground and stick-arm reset, and must not break the chuck arm.

Resource conflicts: none.

Suggested skills: pluto-flighttest and the pluto-log-analyst agent, pluto-rules, pluto-build (`--gate PRIMUS_X2_v1`), and spec-to-code-compliance for the doc drift.

Pipeline docs to update if this becomes a fix: Core_PID_Controller.md (the I-term reset condition), RC_Input_Pipeline.md (THROTTLE_LOW users), Mixer_Pipeline.md (minthrottle 1100), and FLIGHT_INVARIANTS.md (a new invariant).

---

## User answers given to the scout's questions ( planning interview )

- (a) / (b) Only a full stick-down leans; a partial ( about one third ) stick-down is level.
- (c) The craft levels again as soon as the stick is centred.
- (d) The app Land button is level.
- (e) / (g) `4657cbf` / `d2e900c` was tested with the same full-stick test and stayed level; `2a8d59a` leans.
- (f) Under 10 g, on an arm / frame edge; any one side at a time.
- (h) PRIMUS_X2_v1.

---

## Scout follow-up for the fix ( 27 Oct 2026 )

Asked after the five flight logs proved the mechanism: every reason the THROTTLE_LOW reset exists, what
clears the I-term at arming, the airborne signals, and an evaluation of gates. Line numbers at HEAD `a81c180`
( `pid.cpp` in the working tree is shifted +10 by the diagnostics ).

**1. Why the reset exists, and who uses it today.** `mw.cpp:892-898` computes `throttleStatus` from
`rcData[THROTTLE] < mincheck` 1100 ( `rc_controls.cpp:131-139` ) and zeroes `errorAngleI` / `errorGyroI`
( `pid.cpp:83-101` ). `processRx` runs only on RX frames ( `mw.cpp:1322` ); at line 892 `rcData` was just
rewritten by `calculateRxChannelsAndUpdateFailsafe` ( `mw.cpp:872`, `rx.cpp:542/545` ), so there it equals
`rcDataPilot` ( `rx.cpp:563-565` ), except when `skipRxSamples` returns early ( `rx.cpp:580-585` ).

- **Stick-arm with a radio** ( `rc_controls.cpp:169-180` ): the reset at `mw.cpp:895` runs before
  `processRcStickPositions` ( `mw.cpp:939` ), so the I-terms are clean when `mwArm` arms; the craft then sits
  armed at idle and keeps being reset. **Must stay.**
- **Chuck arm** ( `rc_controls.cpp:183-187` ): does its own reset; independent of line 895.
- **Auto-disarm and beeper** ( `mw.cpp:903-937` ): read `throttleStatus`, not the reset; inactive anyway
  ( FEATURE_MOTOR_STOP is never set ).
- **Switch disarm** ( `rc_controls.cpp:198-200` ): reads `throttleStatus`; independent of the reset.
- **Armed on the ground before take-off**: stick-arm ground idle is `isThrottleStickArmed` /
  `altHoldGroundIdle` ( `altitudehold.cpp:608-614` ); with the stick at the bottom the reset prevents wind-up
  against the floor. **Must stay.**
- **Landing / touchdown / disarm**: `land ( )` writes `landThrottle` into `rcData` after `processRx`
  ( `mw.cpp:390/1410` ) and the next frame overwrites it, so `Command_Land ( speed > 205 )` never reaches
  line 892. After `finishLanding` disarms ( `command.cpp:243-253` ) the reset is the disarmed case. In a battery
  or failsafe LAND it fires only if the pilot's stick is at the bottom; the RX-fail throttle is 1200
  ( `rx.cpp:447` ).
- **Flips**: `flip ( )` writes 2000 after `processRx` ( `acrobats.cpp:121/233/324` ); the reset fires only if
  the app stick itself is below 1100.
- **THROTTLE mode, stick low in flight**: motors fall to minthrottle and the craft drops; the reset is the
  Cleanflight anti-windup for that case.
- **User `RcCommand_Set` throttle below 1100**: written in `userCode` ( `mw.cpp:1202` ), after `processRx`,
  so it does not trigger the reset ( except in a `skipRxSamples` frame ).

**2. What clears the I-terms at arm in log-1.** `mwArm ( )` ( `mw.cpp:602-660` ) does not. With the stick at
1500 the arm came from ( a ) `takeOff ( )` from the app's MSP_SET_COMMAND TAKE_OFF ( `serial_msp.cpp:1568` ),
which runs `pidResetErrorAngle/Gyro` then `mwArm` ( `command.cpp:176-178` ), most likely; ( b ) `Command_Arm`
( `FC-Control-Command.cpp:188-190` ); ( c ) the chuck path; or ( d ) the app briefly sending `Th` below 1100.
( a )-( c ) are outside the block being changed; ( d ) is kept by any gate that keeps "not armed means reset".

**3. Airborne signals.**

- `ARMING_FLAG(ARMED)`: set `mw.cpp:620`, cleared `mw.cpp:549`.
- `throttleRaisedSinceArm` ( static, `mw.cpp:181` ): set at `mw.cpp:403-405` when armed and `rcData > 1100`,
  cleared only at `mw.cpp:571`. Not "airborne": after an app take-off arm at 1500 it is true on the floor
  from the first armed loop; with stick-arm it becomes true once the stick passes 1100, even in ground idle.
- `isThrottleStickArmed` ( `rc_controls.cpp:87`, `.h:265` ): set `rc_controls.cpp:175`, cleared at
  `altitudehold.cpp:612` ( stick above 1500 ), `altitudehold.cpp:1473` ( take-off goal ) and `mw.cpp:585`. In
  BARO mode it equals `altHoldGroundIdle` ( `altitudehold.cpp:454` ).
- `isLanding` ( `command.cpp:260` ), `landingInProgress ( )` ( `FC-Control-Command.cpp:130` ).
- `isTookOff`: set at arm; useless as an airborne signal.
- Laser fusion's local `airborne = ARMED && !altHoldGroundIdle && !isLanding` ( `altitudehold.cpp:1158` ).
- `FLIGHT_MODE(BARO_MODE)`: updated in `updateAltHoldState` after line 895 ( one frame late ); stays on
  through flips.
- No touchdown detector exists for a stick-down landing; touchdown is only detected inside `land ( )`.

**4. The gates.**

- **A. Reset only when `!ARMED || !throttleRaisedSinceArm`**: fixes ALT_HOLD flight, but loses the reset in a
  THROTTLE-mode drop and in stick-arm ground idle after the stick passed 1100. Risk medium.
- **B. Skip the reset while `ARMED && BARO_MODE`**: a stick-arm in ALT_HOLD winds up on the ground at idle
  ( `pid_at_min_throttle = 1` keeps the PID acting, `mixer.cpp:818-824`, `config.cpp:369` ). Risk high.
- **C. Skip while `ARMED && BARO_MODE && throttleRaisedSinceArm && !isThrottleStickArmed`**: ground idle, arm
  and THROTTLE mode keep the reset; residual: after a stick-down touchdown in ALT_HOLD the craft sits armed and
  the I-term integrates against the floor, about 5-7 s to the 256 clamp at a 3 deg ground tilt, then a tilted
  lift-off.
- **D. Recommended: C's condition, but freeze the rate I-terms instead of skipping the reset** ( skip the
  accumulation in `pidRewrite`, `pid.cpp:831`, while an `mw.cpp` flag is set ). In flight the I-term holds the
  hover trim; after a stick-down touchdown it cannot wind up. One bool in `pid.h/.cpp`, a few lines in `mw.cpp`,
  no loop-time impact.

**5. Constraints.** At line 892 `rcData` is the pilot value; anywhere after `userCode` use `rcDataPilot`.
**Do not change `calculateThrottleStatus` or `throttleStatus`** ( stick-arm, switch disarm, the auto-disarm /
beeper block and the stick combos read it; `mixer.cpp:819` reads raw `rcData` separately ). The fix touches
only the PID I-term, not `setVelocity` ( landing must not go through the stick limits ). Leave the `mwArm`
datum gate and `throttleRaisedSinceArm` semantics unchanged: read the flag, do not repurpose it. No new floats
in `pidRewrite`; the flag is main-loop only. Keep `pid.cpp`'s style and banner.

**6. Docs the fix would change.** `Core_PID_Controller.md` ( reset / freeze rule; controller 1 is `pidRewrite`
with a P-only angle loop ), `RC_Input_Pipeline.md` ( THROTTLE_LOW users ), `Mixer_Pipeline.md:22`
( minthrottle 1100 ), `Altitude_Hold_Estimator.md` ( a stick-down keeps the attitude trim ),
`FLIGHT_INVARIANTS.md` and a CLAUDE.md Flight-invariants bullet, `CHANGELOG.md` and the FW_Version bump; the
API is unchanged.

