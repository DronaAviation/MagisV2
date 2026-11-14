# Core PID Controller (`pid.cpp`)

## Overview
The PID (Proportional, Integral, Derivative) controller is the stabilizing brain of the drone. It compares the pilot's requested rotation rates (`rcCommand`) against the drone's actual physical rotation rates (from the IMU), and computes a mathematical error. It then outputs a correction value (`axisPID`) that is sent to the Mixer.

## Source Files
- **Controller Logic**: `src/main/flight/pid.cpp`, `src/main/flight/pid.h`
- **Default Gains**: `src/main/flight/pid_defaults.cpp` ( `pidProfileDefaultGains ( )` )
- **Profile Config**: `src/main/config/config_profile.h`
- **User API Hook**: `src/main/API-Src/FC-Config-PID.cpp` ( `get_PIDProfile`, `set_PIDProfile`, `setDefault_PIDProfile` )

## Key Data Structures
- `rcCommand[3]`: The target rates commanded by the user/GCS.
- `gyroADC[3]`: The current actual rotation rates in degrees per second.
- `axisPID[3]`: The final computed correction (-1000 to +1000) for Roll, Pitch, and Yaw.
- `pidProfile_t`: Holds the tuned P, I, and D multipliers.

## Default gains and the API reset
- `pidProfileDefaultGains ( pidProfile_t * )` ( `pid_defaults.cpp:38`, declared `pid.h:111` ) is the single source of the
  default `P8` / `I8` / `D8` for all eleven `pidIndex_e` slots. `resetPidProfile ( )` ( `config.cpp:175` ) calls it on a
  config reset ( `config.cpp:179` ) and sets the rest of the profile itself: `pidController`, the float gains, the cut
  frequencies, `A_level` / `H_level` / `H_sensitivity` and the GTUNE limits.
- `setDefault_PIDProfile ( )` ( `FC-Config-PID.cpp:155` ) restores only the five slots the PID API exposes,
  `apiSlot [] = { PIDROLL, PIDPITCH, PIDYAW, PIDALT, PIDUSER }` ( `FC-Config-PID.cpp:157` ), from a stack copy filled
  by `pidProfileDefaultGains ( )`. `PIDPOS`, `PIDPOSR`, `PIDNAVR`, `PIDLEVEL`, `PIDMAG` and `PIDVEL` are never
  written. The change is RAM only, and gains are read live every loop, so a call while armed steps the tuning in flight
  ( header comment, `FC-Config.h:95` ). Before FW 3.15.7 the function looped over all eleven slots with five-entry
  tables: it set ALT P8 to a stale 100 and overwrote the hidden gains, `PIDLEVEL` and `PIDVEL` included
  ( `active-development/set-default-pid-profile/` ).

## Primary Functions
- `pidController()`: The core execution loop. It iterates through the Roll, Pitch, and Yaw axes, computing the error and multiplying by the PID gains.
- `pidRewrite()` ( `pid.cpp:745` ): the controller in use. `pidController` defaults to 1 ( `config.cpp:176` ),
  which `pidSetController()` maps to `pidRewrite`. In ANGLE mode its angle loop is P only
  ( `desiredRate = errorAngle * P8[PIDLEVEL] >> 4` ), so the rate I-term `errorGyroI` is the only integrator that
  holds a constant moment such as an off-centre CG or a payload.
  In every mode the roll / pitch target is `desiredAngle = 2 * rcCommand + userAngleTarget ( axis )` ( `pid.cpp:807` ),
  clamped to ±`max_angle_inclination` before the error is taken ( `pid.cpp:808` ); `userAngleTarget ( )` is 0 outside
  ANGLE mode, so only ANGLE flies the user tilt. See **User roll / pitch tilt** below.
- `pidResetErrorAngle()` / `pidResetErrorGyro()` ( `pid.cpp:88` / `:97` ): zero the angle and rate I-terms. Called
  by the stick-low reset in `processRx ( )` whenever the craft is not counted as airborne ( see below ), and by
  every arming path ( take-off, `Command_Arm`, chuck ).
- `pidSetGyroITermFreeze ( bool )` ( `pid.cpp:108` ): while set, `pidRewrite` holds `errorGyroI` on all three axes
  instead of integrating ( `pid.cpp:848` ); the clamp and `ITerm = errorGyroI >> 13` still run, so the held value
  keeps acting. Set by `processRx ( )`, cleared there and in `mwDisarm ( )`. Main loop only.
- `applyGyroData()`: Injects the raw IMU data into the PID logic.
- `updateMagHold()` ( `mw.cpp` ): heading hold, run every loop before the PID while a magnetometer is present. In MAG
  or HEADFREE mode, with the yaw stick within ±300, it replaces `rcCommand[YAW]` with the heading error to `magHold`.
  See **Heading hold** below.
- `plutoLoop()` (API): If an API application is active, it may intercept `axisPID` or `rcCommand` to inject autonomous flight corrections before the Mixer.

## Data Flow & Boundaries
- **P-Term Limit**: The Proportional correction is usually clamped to prevent an instantaneous massive error (like a prop strike) from commanding 100% motor output instantly.
- **I-Term Windup**: `pidRewrite` clamps `errorGyroI` to ±`GYRO_I_MAX` 256 motor counts ( `pid.cpp:854` ). On the
  ground the stick-low reset keeps it from integrating against the floor; airborne it is frozen instead of reset
  ( see *Stick-low I-term reset and freeze* below ). Note that `pid_at_min_throttle` is 1 by default
  ( `config.cpp:331`; forced to 0 only with `retarded_arm` ), so the PID keeps acting at idle on the floor.
- **D-Term Filtering**: The Derivative term calculates the *rate of change* of the error. Because differentiation amplifies high-frequency noise, the D-term operates on heavily filtered gyro data or error deltas to prevent motor smoking.

```mermaid
flowchart TD
    Start([Calculate PID]) --> Loop[For each axis: Roll, Pitch, Yaw]
    Loop --> GetTarget[Get Target Rate from rcCommand]
    GetTarget --> GetCurrent[Get Current Rate from Gyro]
    
    GetCurrent --> Error[Error = Target - Current]
    Error --> P_Term[P = Error * P_Gain]
    Error --> I_Term[I = I + Error * I_Gain]
    Error --> D_Term[D = Delta_Error * D_Gain]
    
    P_Term --> Sum[axisPID = P + I + D]
    I_Term --> Sum
    D_Term --> Sum
    
    Sum --> Limit{Exceeds PID Limit?}
    Limit -- Yes --> Clamp[Clamp axisPID to max limit]
    Limit -- No --> NextAxis{More Axes?}
    
    Clamp --> NextAxis
    NextAxis -- Yes --> Loop
    NextAxis -- No --> End([Output axisPID Array])
```

## User roll / pitch tilt ( `userAngleTarget`, API-Utils.cpp )
- `setDesiredAngle ( AG_ROLL | AG_PITCH, x )` stores a held tilt in `userDesiredAngle [ ]` ( deci-degrees, ±1800 ) and
  `isUserDesiredAngle [ ]` ( `FC-Config-Setpoints.cpp:90-98` ). `pidRewrite` adds `userAngleTarget ( axis )`
  ( `API-Utils.cpp:209` ) to its stick target ( `pid.cpp:807` ), before the clamp, so the pilot's stick adds on top and
  the sum keeps `max_angle_inclination`. The helper returns 0, keeping the tilt held, unless: Developer Mode ran within
  the last 0.6 s, ANGLE mode is on, `flipState == 0`, no landing runs ( `landingInProgress ( )` ), and the craft is
  airborne ( `throttleRaisedInFlight ( )` and `getEstAltitude ( ) >= 20` cm ). `userCode ( )` clears it at a real Dev
  Mode switch-off and on the disarm edge ( `mw.cpp:1269` ); it runs before `pid_controller ( )` in the same loop
  ( `mw.cpp:1436` / `:1507` ). Only `pidRewrite` reads it. The ANGLE loop is P only, so the tilt is reached with no
  integrator wind-up: flown at 5° it settled within 0.2-0.3 s and 1° ( log-2 of
  `active-development/set-desired-angle-roll-pitch/` ).

## Stick-low I-term reset and freeze ( `processRx ( )`, mw.cpp )

On every RX pass, `processRx ( )` computes `throttleStatus` from the raw `rcData[THROTTLE]`: THROTTLE_LOW below
`mincheck` 1100 ( `rc_controls.cpp:131-139` ). At that point `rcData` was just rewritten from the receiver, so it is
the pilot's value ( equal to `rcDataPilot` ), except during the 600 ms invalid-pulse hold ( `rx.cpp:528` keeps the
previous `rcData`, which LAND or user code may have written; at worst that picks freeze over reset for a frame ). What happens with THROTTLE_LOW depends on whether the craft is flying:

| State | Condition | Rate I-term |
|---|---|---|
| Airborne: after an app take-off / `Command_Arm`, or after an ARM-switch arm once altitude hold has seen the stick above 1500 or a take-off goal ran | `ARMED && throttleRaisedSinceArm && !isThrottleStickArmed` and the controller is `pidRewrite` | **frozen** at its value ( holds the payload trim ) |
| Anything else: disarmed, arming, ARM-switch ground idle, THROTTLE mode after an ARM-switch arm, other controllers | - | **reset** to 0 ( anti wind-up ) |

Why: in ANGLE mode the rate I-term carries any CG offset. Resetting it in flight on a full stick-down
( the app's bottom position is about 1004 ) left the craft held by the P terms alone, and it leaned 2-7 deg toward a
payload ( 2-3 deg toward its own trim with none ) for as long as the stick stayed down. Frozen, a full stick-down
descends as level as a partial one. See `active-development/payload-tilt-on-descent/`.

`throttleStatus` itself is unchanged: stick-arm, switch disarm, auto-disarm and the beeper still read it.

```mermaid
flowchart TD
    Loop([loop: processRx, mw.cpp:1354]) --> Rx["calculateRxChannelsAndUpdateFailsafe ( ), mw.cpp:881"]
    Rx --> Status["throttleStatus = calculateThrottleStatus ( ), mw.cpp:901"]
    Status --> Airborne{"THROTTLE_LOW and ARMED and throttleRaisedSinceArm<br/>and not isThrottleStickArmed and pidRewrite?<br/>mw.cpp:908-910"}
    Airborne -- yes --> Freeze["pidSetGyroITermFreeze ( true ), mw.cpp:911"]
    Airborne -- no --> Thaw["pidSetGyroITermFreeze ( false ), mw.cpp:911"]
    Thaw --> Low{"THROTTLE_LOW?<br/>mw.cpp:913"}
    Low -- yes --> Reset["pidResetErrorAngle ( ) + pidResetErrorGyro ( ), mw.cpp:914-915"]
    Low -- no --> Sticks
    Reset --> Sticks["processRcStickPositions ( ), mw.cpp:957"]
    Freeze --> Sticks
    Sticks --> PID["pid_controller = pidRewrite, mw.cpp:1507 / pid.cpp:745"]
    PID --> Gate{"gyroITermFrozen?<br/>pid.cpp:848"}
    Gate -- no --> Integrate[errorGyroI += rate error x I8]
    Gate -- yes --> Hold[errorGyroI held]
    Integrate --> Clamp["clamp ±GYRO_I_MAX, ITerm = errorGyroI >> 13, pid.cpp:854-855"]
    Hold --> Clamp
```

`throttleRaisedSinceArm` is set in `annexCode ( )` once the armed throttle is above `mincheck` ( `mw.cpp:412` ) and
cleared in `mwDisarm ( )`, which also clears the freeze ( `mw.cpp:580` ). `isThrottleStickArmed` is set by an ARM-switch
arm with the throttle low ( `rc_controls.cpp:175` ) and cleared by altitude hold when the stick goes above 1500
( `altitudehold.cpp:612` ), by a take-off goal ( `altitudehold.cpp:1473` ) or at disarm ( `mw.cpp:594` ).

**Known residual.** A wind-up that builds *before* the stick goes low is held, not cleared. Two ways in: a tilted
touchdown on a partial stick lets `errorGyroI` integrate against the floor and a later full stick-down freezes it; and
after `Command_Arm` with the app stick centred, `throttleRaisedSinceArm` is set at once on the floor ( `mw.cpp:412`,
1500 > 1100 ) and `isThrottleStickArmed` is not, so a stick-down on the floor freezes any floor wind-up instead of
clearing it ( `setTakeOffGoal ( )` does not reset either ). Not observed in flight on a level floor ( log-7 ); there
is no touchdown detector for a stick-down landing.

## Heading hold ( `updateMagHold`, mw.cpp )

Heading hold turns a target heading into a yaw-rate command. It is not part of the PID: it runs just before it in
the same loop ( `userCode ( )` → `updateMagHold ( )` → `pid_controller ( )` ) and writes `rcCommand[YAW]`, which the
rate PID then flies.

- **Target:** `magHold`, in degrees ( 0..359, like `heading` ). It is set by the pilot ( it follows the heading
  while yaw is commanded directly ), by the app ( `MSP_SET_HEAD` ), by a BOXMAG switch-on ( `magHold = heading` ), or by
  user code ( `setDesiredAngle` / `DesiredAngle_setRelative` on `AG_YAW`, restored from `userHeading` on every
  non-`plutoLoop` loop ). While disarmed with MAG or HEADFREE on, `processRx ( )` resets it to `heading`.
- **Active when** MAG or HEADFREE mode is on and `|rcData[YAW] − 1500| < 300`. Otherwise `magHold = heading` and the
  re-arm latch is set.
- **Command:** `headingError = heading − magHold`, wrapped to ±180, clamped to ±30°, then
  `rcCommand[YAW] = error × P8[PIDMAG]` ( 40 ), clamped to ±400. Full authority is reached at 10° of error.
- **Re-arm latch:** after the stick is released or the mode is switched on, no command is produced until the yaw gyro
  reads under ~17°/s ( `|gyroADC[YAW] / 4| < 70` ). The latch then re-arms on the user target if one is held,
  otherwise on the current heading. A relative user target is re-based to the heading first, but only if the yaw stick
  went out while armed since the target was set ( `userHeadingStale` ).

```mermaid
flowchart TD
    Start(["updateMagHold ( )"]) --> Edge{"Yaw stick crossed ±300<br/>while armed?"}
    Edge -- Yes --> Stale[userHeadingStale = true]
    Edge -- No --> Gate
    Stale --> Gate{"Stick within ±300<br/>and MAG or HEADFREE?"}
    Gate -- No --> Off["magHold = heading<br/>latch set"]
    Gate -- Yes --> Latch{"Latch set and<br/>yaw rate < ~17°/s?"}
    Latch -- Yes --> Rebase{"Stale relative<br/>user target?"}
    Rebase -- Yes --> RB[userHeading = heading]
    Rebase -- No --> Arm
    RB --> Arm["magHold = user target or heading<br/>latch clear"]
    Latch -- No --> Held{Latch clear?}
    Arm --> Held
    Held -- Yes --> Cmd["rcCommand [ YAW ] = clamp ( err, ±30 ) × 40<br/>clamped to ±400"]
    Held -- No --> End([Return])
    Cmd --> End
    Off --> End
```
