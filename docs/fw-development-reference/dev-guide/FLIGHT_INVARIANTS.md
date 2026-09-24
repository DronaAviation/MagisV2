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

## With `LASER_ALT`, the down-laser corrects the estimator below 160 cm and the baro above it

`LASER_ALT` works with one down-laser: the VL53L0X ( `LASER_TOF` ) or the VL53L1X ( `LASER_TOF_L1x` ).
Both sit on I2C1 at 0x29, so never define both; `altitudehold.cpp` stops the build with an `#error` if
both are defined, or if `LASER_ALT` is defined without a laser. Both sensors run the same fusion body in
`checkReading()`, reading the sensor through accessor macros and per-sensor constants.

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
- **Accelerometer Z deadband is 0 for the estimator** in `LASER_ALT` builds ( `ALT_EST_ACC_Z_DEADBAND`
  in `imu.cpp` ). The 40-count profile value made hover velocity 0.3-0.4 × real and the craft bobbed.

The shipped `target.h` has the laser defines off ( baro-only ). Details:
[`../fw-architecture-pipeline/subsystems/Altitude_Hold_Estimator.md`](../fw-architecture-pipeline/subsystems/Altitude_Hold_Estimator.md)
( Laser fusion ), [`../active-development/tof-althold-fusion/`](../active-development/tof-althold-fusion/)
( VL53L0X ) and [`../active-development/vl53l1x-althold-parity/`](../active-development/vl53l1x-althold-parity/)
( VL53L1X ).
