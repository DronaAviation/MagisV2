# Core PID Controller (`pid.cpp`)

## Overview
The PID (Proportional, Integral, Derivative) controller is the stabilizing brain of the drone. It compares the pilot's requested rotation rates (`rcCommand`) against the drone's actual physical rotation rates (from the IMU), and computes a mathematical error. It then outputs a correction value (`axisPID`) that is sent to the Mixer.

## Source Files
- **Controller Logic**: `src/main/flight/pid.cpp`, `src/main/flight/pid.h`
- **Profile Config**: `src/main/config/config_profile.h`
- **User API Hook**: `src/main/API-Src/FC-Config-PID.cpp`

## Key Data Structures
- `rcCommand[3]`: The target rates commanded by the user/GCS.
- `gyroADC[3]`: The current actual rotation rates in degrees per second.
- `axisPID[3]`: The final computed correction (-1000 to +1000) for Roll, Pitch, and Yaw.
- `pidProfile_t`: Holds the tuned P, I, and D multipliers.

## Primary Functions
- `pidController()`: The core execution loop. It iterates through the Roll, Pitch, and Yaw axes, computing the error and multiplying by the PID gains.
- `applyGyroData()`: Injects the raw IMU data into the PID logic.
- `updateMagHold()` ( `mw.cpp` ): heading hold, run every loop before the PID while a magnetometer is present. In MAG
  or HEADFREE mode, with the yaw stick within ±300, it replaces `rcCommand[YAW]` with the heading error to `magHold`.
  See **Heading hold** below.
- `plutoLoop()` (API): If an API application is active, it may intercept `axisPID` or `rcCommand` to inject autonomous flight corrections before the Mixer.

## Data Flow & Boundaries
- **P-Term Limit**: The Proportional correction is usually clamped to prevent an instantaneous massive error (like a prop strike) from commanding 100% motor output instantly.
- **I-Term Windup**: To prevent the Integral term from accumulating endlessly when the drone is on the ground or physically constrained (Anti-Windup), it is strictly bounded (e.g., `pidProfile->pid[axis].I`).
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
