# Flight invariants

Detail behind the one-line flight rules in the root [`CLAUDE.md`](../../../CLAUDE.md). Each of these has
cost a flight. The review checklist form lives in the `pluto-rules` skill.

## User RC overrides share the axis with the pilot

`RcCommand_Set` (`API/RC-Interface.h`) stores into `RC_ARRAY[]` and latches `userRCflag[]` via
`userRCassert()`; `applyUserRcOverride()` in `mw.cpp` then runs every loop iteration after
`annexCode()` and **cross-fades** the override against the pilot's sticks by deflection
(`final = command × (1−d) + pilot × d`, `d` reaching 1.0 at `USER_RC_STICK_TRAVEL` counts off centre —
see the tuning block in `API/API-Utils.h`).

- Overrides expire unless re-asserted (`resetUserRCflag()`, `max(250 ms, 2 × userLoopFrequency)`), so
  user code hands a channel back by not calling `RcCommand_Set`.
- Pilot input is read from `rcDataPilot[]` (snapshot taken in `rx/rx.cpp` before user code can write
  `rcData`) — never from `rcData` for throttle, which the override path itself writes.
- `applyObjectAvoidance()` asserts with authority `0.0f` because it already blends the pilot in itself.

## Barometer altitude is compensated relative to the arm instant

In `sensors/barometer.cpp`, the ICP-10111 pressure gets a throttle term
(`BARO_COMP_THROTTLE_PA_PER_COUNT`, rotor inflow) and a die-temperature term
(`BARO_COMP_TEMP_PA_PER_DEGC` = 2.1, measured −2.17 to −2.42 Pa/°C on PRIMUS_V5) added back, both zero
at arm and clamped to `BARO_COMP_LIMIT_PA`. Conversion to altitude uses a fixed ISA temperature, and
the ground zero tracks while disarmed and freezes on arm. The ground reference must never be re-zeroed
in flight (`throttleRaisedSinceArm` in `mw.cpp`).

Reasoning, changes, measurements and open items:
[`../active-development/altitude-hold/`](../active-development/altitude-hold/).

## In ALT_HOLD the throttle stick moves the setpoint; it never replaces the position loop

`calculateAltHoldThrottleAdjustment()` in `flight/altitudehold.cpp` moves `AltHold` at a ramped rate
`altRate`, which is fed forward to the velocity loop. The rate comes from one of three sources:

- the stick (`ALT_MAX_CLIMB_CMS` 40 / `ALT_MAX_DESCENT_CMS` 30);
- a goal profile when something writes `AltHold` (take-off, `setAltitude()`, MSP: trapezoid at
  `ALT_CMD_*`);
- landing, which keeps its own `(landThrottle − 1500) / 4` descent.

Landing must not go through the stick limits: at slow descent rates the in-ground-effect baro drift
masks the descent, and touchdown is never detected. The controller is held in reset while disarmed or
idle-held after stick-arming.

Details: [`../fw-architecture-pipeline/subsystems/Altitude_Hold_Estimator.md`](../fw-architecture-pipeline/subsystems/Altitude_Hold_Estimator.md)
and [`../active-development/althold-setpoint-shaping/`](../active-development/althold-setpoint-shaping/).

## Keep `Monitor_Print` under ~130 bytes per tick with the app connected

It writes into the MSP UART's 256-byte TX ring buffer (115200 baud, ~22 ms to drain) and `uartWrite()`
does not check for full, so an overrun overwrites unsent bytes: the start of the log line, or, as seen
in `tof-althold-fusion`, apparently the app's own MSP replies, and the app disconnects (~180 B/tick
did; 115-135 B/tick ran clean; the exact limit is not measured).

Also: the double overload prints 0 for every digit after the first decimal.

## During a flip, altitude hold is bypassed, not switched off

`flip()`'s `DEACTIVATE_RC_MODE(BOXBARO)` is an XOR on `rcModeActivationMask`, and
`updateActivatedModes()` rebuilds the mask from the AUX channels on the next RX frame, so BARO_MODE
stays on while the app holds AUX3. Altitude hold therefore flies the whole flip.

- The flip drives `rcData[THROTTLE]` itself ( 2000 in ASCEND and HOLD ), and while `flipActive()`
  `calculateAltHoldThrottleAdjustment()` flies that throttle as a raw rate ( `flipVelocitySetpoint()`,
  up to 120 cm/s ) instead of through setpoint shaping. ASCEND waits for 100 cm/s; the shaped 40 cm/s
  never gets there, and the flip times out after 2.2 s without rotating.
- On the flip's falling edge the setpoint is reset, the pre-flip velocity integrator is restored and
  held for 500 ms while the exit climb is braked, and, if it was flying in ALT_HOLD at ≥ 20 cm ( not ground
  idle ), the pre-flip `AltHold` becomes a goal, so the craft flies back to the height it held when the
  flip was sent.
- Anything that changes the altitude frame during or after a flip ( the laser re-base, the return
  shift ) must move the saved return height with it ( `altPreFlipAltHold` in `altShiftFrame()` ).

Details: [`../fw-architecture-pipeline/subsystems/Altitude_Hold_Estimator.md`](../fw-architecture-pipeline/subsystems/Altitude_Hold_Estimator.md)
( Flip interaction ) and [`../active-development/flip-althold-regression/`](../active-development/flip-althold-regression/).

## With a down-laser, it corrects the estimator below 160 cm and the baro above it

The laser fusion works with one down-laser:
- the VL53L0X, switched on at run time by `XRanging.initAlt ( pin )` / `initAlt ( )` in `plutoInit ( )` and
  fixed at start-up ( `altLaserFusion ( )`, `altitudehold.cpp` );
- or the VL53L1X, with the build switches `LASER_TOF_L1x` + `LASER_ALT`.

`initAlt` is refused in a VL53L1X build. `LASER_TOF` is gone ( `#error` ), and `LASER_ALT` without
`LASER_TOF_L1x` is an `#error`. Both sensors run the same fusion body in `checkReading()`, reading the sensor
through accessor macros and per-sensor constants. The VL53L0X accessors read only the down-laser's entry in
`drivers/tof_manager`, so a side laser can never reach the estimator.

- **No `initAlt`: the baro build.** Without it, every fusion branch takes the baro-only path ( estimator,
  τ 2 s, accelerometer deadband, take-off height ). Validated in flight against the baro baseline
  ( tof-single-pipeline, hover B ).
- **The laser fails to start after `initAlt`:** the fusion still runs and treats it as a dropout ( baro ),
  so it keeps the laser tuning. `XRanging.getStatus ( DOWN )` reports it.

- **Handover.** The laser corrects the altitude estimator below 160 cm and hands over to the baro above
  it ( back below 140 cm ). A dropout of `ALT_TOF_DROPOUT_MS` ( 120 ms VL53L0X, 185 ms VL53L1X ) also hands
  over, and so does a tilt above 25° that lasts that long ( tilted samples are only unusable ); shorter
  gaps coast on the accelerometer. On the baro the baro-minus-laser
  offset is frozen; the baro ground datum itself is never re-zeroed.
- **The frame moves, the craft does not.** On the return to the laser the whole altitude frame
  ( estimate, setpoint, goal, flip return height ) is shifted by the laser minus the estimate, so
  `EstAlt` and `AltHold` move together. A change of reference must never become a position error.
- **Object under the craft.** A sudden change of surface ( the laser disagreeing with the
  accelerometer by more than 30 cm within 0.5 s, or more than 20 cm building up within ~1 s ) holds the
  estimate on the baro for 2.5 s, then re-bases to the new surface and flies back to the old clearance
  as a goal; smaller or slower changes are followed as terrain. A landing that starts during a hold-off
  re-bases at once, so touchdown is not delayed.
- **VL53L1X return guard.** On the baro, in flight, a return reading more than 50 cm from the estimate
  is held off unless the disagreement stays within 25 cm for 2.5 s. Without it, ceiling-fan blades seen
  from above at 2.5 m would have returned the craft to the laser at a false 56 cm.
- **VL53L1X minimum range.** Ranges under 15 mm are out of range ( a covered window reads 0-10 mm as
  valid ). The landed reading, 25-28 mm, stays valid, so landing and touchdown behave as on the VL53L0X.
- **Accelerometer Z deadband is 0 for the estimator** while the laser fusion is on ( `ALT_EST_ACC_Z_DEADBAND`
  in `imu.cpp` ). The 40-count profile value made hover velocity 0.3-0.4 × real and the craft bobbed.

The stock user code calls no `initAlt` ( baro-only ), and the shipped `target.h` has the VL53L1X switches off.
Details:
[`../fw-architecture-pipeline/subsystems/Altitude_Hold_Estimator.md`](../fw-architecture-pipeline/subsystems/Altitude_Hold_Estimator.md)
( Laser fusion ), [`../active-development/tof-althold-fusion/`](../active-development/tof-althold-fusion/)
( VL53L0X ) and [`../active-development/vl53l1x-althold-parity/`](../active-development/vl53l1x-althold-parity/)
( VL53L1X ).

## Battery current is read across one R020; levels come from the fuel gauge

The INA219 reads the battery current across **one R020 ( 20 mOhm )**, the production fit. A second shunt stacked on
it ( a rework on one test board ) halves every current and mAh reading, and made an empty 600 mAh pack show ~300 mAh.
The range tops out at 8 A. The low-battery levels come from the Pluto Fuel Gauge ( the count, or the compensated
voltage at 3.745 / 3.60 V per cell ), not from the warning / minimum voltages stored in the config.
See [`Power_BMS_Pipeline.md`](../fw-architecture-pipeline/subsystems/Power_BMS_Pipeline.md) and
[`../active-development/battery-soc-fix/`](../active-development/battery-soc-fix/).

## At critical battery the firmware lands; the app must not be told "critical" while armed

The app switches ARM off when it sees `App_LowBattery_inFlight`, so before battery-soc-fix the craft dropped from
hover at critical ( log-3: 0.2 s after critical ). Now a confirmed critical starts `LAND` ( `batteryCriticalAutoLand ( )`,
`mw.cpp` ), and `MSP_FLIGHT_STATUS` / `MSP_ANALOG` report low battery until the landing has disarmed; then critical
is reported and `mwArm ( )` refuses to arm until the pack is changed. A provisional critical ( before the pack
resistance is measured ) beeps but does not land.

## RC link loss lands ~2.5 s after the last frame ( MSP; ~2.4 s on ELRS / PPM ); do not lengthen the stick hold

On the Wi-Fi link ( MSP over TCP ), stalls of 1.2-2.2 s happen on a healthy link. Before FW 3.11.2
`failsafeOnValidDataFailed ( )` ignored `failsafe_delay`, so every such stall ended the flight in `LAND`
( ~1.0 s ). Now the timeline from the last frame is:
- 0-200 ms: the last frame is replayed;
- 200 ms: Developer Mode stops;
- ~800 ms: the stick hold ends; sticks go centred and throttle to 1200;
- **~2.5 s**: `Signal_loss` and `LAND` ( `failsafe_delay` 15, capped at 22 = ~3.2 s by `FAILSAFE_DELAY_MAX` ).

Extend tolerance only through `failsafe_delay`, which lengthens the sticks-centred phase. Never lengthen it
through `MAX_INVALID_PULS_TIME` or `DELAY_5_HZ`: in that phase the craft keeps flying the last stick input.
See [`Failsafe_Subsystem.md`](../fw-architecture-pipeline/subsystems/Failsafe_Subsystem.md) and
[`../active-development/rx-signal-loss-tolerance/`](../active-development/rx-signal-loss-tolerance/).

## User flight commands cannot replace a landing; a landing cannot start in a flip

- **Mechanism.** One command slot ( `current_command`, `command_status` ) is shared by user code, the app
  ( `MSP_SET_COMMAND` ), the RC-loss failsafe and the battery auto-land. A flip marks itself FINISHED as soon as it
  starts, and `isLanding` is cleared only by `finishLanding ( )` or a disarm.
- **What broke.** A `Command_TakeOff` / `Command_Flip` during a landing stopped `land ( )` but left `isLanding` set:
  the throttle stayed pinned and nothing disarmed after touchdown. `Command_Arm` followed by `Command_TakeOff` set no
  climb goal ( [`../active-development/takeoff-api-ground-hover/`](../active-development/takeoff-api-ground-hover/) ).
- **Rules.**
  - Never write `current_command` from user-reachable code without the landing guard ( `landingInProgress ( )` ).
  - A take-off goal on an armed drone must release the ground idle first ( `altHoldReleaseGroundIdle ( )` ).
  - The touchdown test must not confirm before the descent has started ( `LAND_CONFIRM_MIN_MS` ).
  - With the battery OK, `BMS_Update ( )` re-enables `OK_TO_ARM` every update, so arming is allowed right after a
    landing: a re-sent take-off re-arms. User code must send take-offs once per ARM switch-on.

See [`../active-development/takeoff-land-command-fix/`](../active-development/takeoff-land-command-fix/) and
[`docs/API/FC_CONTROL_API_WIKI.md`](../../API/FC_CONTROL_API_WIKI.md).
