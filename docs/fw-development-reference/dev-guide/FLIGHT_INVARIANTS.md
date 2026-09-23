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
