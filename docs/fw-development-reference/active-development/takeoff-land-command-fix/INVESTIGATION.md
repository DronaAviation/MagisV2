# Take-off / land command fix - Investigation

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md)

The full investigation, with evidence and the flight logs, is in the check topic:
[../takeoff-api-ground-hover/INVESTIGATION.md](../takeoff-api-ground-hover/INVESTIGATION.md) and
[TESTING.md](../takeoff-api-ground-hover/TESTING.md). This file keeps the causal chains the fix acts on and the Q&A
that shaped it.

## Causal chains

**Reported fault ( flight-proven, log-1 vs log-2 ).**
`Command_Arm ( )` → `mwArm ( )` arms the drone inside `userCode ( )` ( `FC-Control-Command.cpp:157-166`, `mw.cpp:1387` ).
`Command_TakeOff ( 100 )` only queues TAKE_OFF ( `FC-Control-Command.cpp:124-135` ). Then `executeCommand ( )`
( `mw.cpp:1491` ) runs `takeOff ( )`, sees ARMED, and takes the armed branch: FINISHED, no height ( `command.cpp:97-103` ).
The target stays at the floor ( `Zd` 0 ), and the drone skims at about 1555 µs.

**D1 ( from the code ).** `Command_TakeOff` / `Command_Flip` replace LAND ( `FC-Control-Command.cpp:126,151` ), so
`land ( )` stops. `isLanding` stays true ( only `finishLanding ( )` and a disarm clear it ), and `rcData [ THROTTLE ]`
stays pinned to a frozen `landThrottle` ( `mw.cpp:385,1391` ). Touchdown detection, the impact check and the timeout
all live inside `land ( )`, so nothing disarms the drone. Only the ARM-switch kill does.

**D2 ( from the code ).** `Command_Land ( 105 )` gives `landThrottle` 1200 = `LAND_CONFIRM_THROTTLE`, so `conclusive`
holds from the first cycle ( `command.cpp:155,218` ). Any 300 ms before the descent reaches 8 cm/s counts as a
touchdown and disarms the drone.

**D3 ( from the code ).** An ARM-switch arm with the stick low sets `isThrottleStickArmed`. That keeps
`altHoldGroundIdle` on while the stick is at or below 1500, and the controller resets the setpoint on every cycle
( `altitudehold.cpp:581-583,688-689` ). Any goal is wiped.

## Q&A ( 16 Oct 2026 )

| Question | Answer |
|---|---|
| New topic or append to the check topic? | New linked topic |
| What counts as "on the ground" for a take-off on an armed drone? | Within **3 s** of arming, **or** at ground idle ( `isThrottleStickArmed` ). Once per arm. In the air: no-op |
| D1: what should take-off / flip do during a landing? | Be ignored silently; the landing completes |
| D2 fix | `conclusive` counts only after the ramp has run at least **1 s** ( log-2: descent −46 cm/s after 0.6 s ) |
| Validation flights | Arm + TakeOff climbs; TakeOff alone unchanged; take-off re-sent during `Command_Land` is refused; ARM-switch idle + TakeOff climbs; default `Command_Land ( )` disarms on the floor |
