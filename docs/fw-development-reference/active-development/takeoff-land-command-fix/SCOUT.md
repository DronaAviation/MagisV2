# Take-off / land command fix - Scout brief

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md)

No new scout was run. This fix plan reuses the reconnaissance of the check topic:

- Scout brief: [../takeoff-api-ground-hover/SCOUT.md](../takeoff-api-ground-hover/SCOUT.md). Its claim that "calling
  Command_TakeOff every loop is harmless" is **wrong**; see the check topic's task 7.
- Code facts, all verified against the source during the check ( 16 Oct 2026 ):
  [../takeoff-api-ground-hover/INVESTIGATION.md](../takeoff-api-ground-hover/INVESTIGATION.md), sections *How take-off
  works*, *`Command_TakeOff` during a landing* and *Fix options*.

## Facts added while planning the fix

| Fact | Source |
|---|---|
| `arm_time = millis ( )` is set on the first arm and reset to 0 at disarm. `opticflow.cpp:152` already uses `millis ( ) - arm_time < 2500` as an "armed just now" window | `mw.cpp:166,565,622` |
| `altHoldGroundIdle` is `static` in `altitudehold.cpp`. The public stand-in is `isThrottleStickArmed`: the idle runs while it is set and the stick is at or below 1500, and it is cleared as soon as the stick goes above 1500 | `altitudehold.cpp:427,581-587`, `rc_controls.h:265` |
| `MSP_SET_COMMAND` is already ignored while landing. D1 extends the same rule to the user API | `serial_msp.cpp:1569` |
| The landing is described in the pipeline docs in `User_Space_API.md` and `Power_BMS_Pipeline.md` ( battery auto-land ) | grep of `fw-architecture-pipeline/` |
| Landing constants: `LAND_RAMP_RATE` 40 counts/s, `LAND_CONFIRM_THROTTLE` 1200, `LAND_SETTLE_MS` 300, `LAND_TIMEOUT_MS` 30000 | `command.cpp:153-159` |
