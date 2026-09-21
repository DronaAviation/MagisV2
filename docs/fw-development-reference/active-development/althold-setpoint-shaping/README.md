# Altitude Hold Setpoint Shaping

[README](README.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

| | |
|---|---|
| **Status** | **Closed** in `2a8d59a` ( FW 3.8.0 / API 1.3.2, 18 Sep 2026 ). Stick shaping and landing flown; goal-profile take-off built after the last flight - see Open items |
| **Branch** | `BugFix-June26`, base `d2e900c` |
| **Target** | `PRIMUS_X2_v1` |
| **Last updated** | 18 Sep 2026 |
| **Pipeline doc** | [Altitude_Hold_Estimator.md](../../fw-architecture-pipeline/subsystems/Altitude_Hold_Estimator.md): "Setpoint shaping" section written directly at commit ( no staged `PIPELINE_UPDATE.md` ) |

**Problem.** In `ALT_HOLD` the throttle stick switched the controller between
two modes. In the dead zone the position loop held `AltHold`. Outside it the
position loop was removed and the stick commanded a raw climb rate:
`( stick - 1500 ) / 4`, up to +120 / -100 cm/s. This caused three problems:

- The rate stepped from 0 to 10 cm/s at the dead-zone edge.
- On centring, `AltHold` snapped to `EstAlt` while the craft was still moving,
  so it overshot and came back.
- Armed on the throttle stick with the motors held at idle, the velocity
  integrator wound down against a craft that could not move. After about 11 s it
  reached its -300 count limit, so take-off was late and sluggish.

**Change.** ArduPilot / DJI style setpoint shaping. The stick sets a ramped
climb rate that moves `AltHold`. The position loop is always active, with the
rate fed forward. A commanded altitude (take-off command, `setAltitude ( )`,
MSP) becomes a goal: the target travels there on a trapezoidal profile (ramp
up, cruise, brake to stop exactly on the goal), again with the rate fed
forward. The final velocity setpoint is clamped and slewed. While the craft is
disarmed or held at idle, the whole controller is held in reset.

## Behaviour changes to watch

| Situation | Before | Now |
|---|---|---|
| Full stick up / down | +120 / -100 cm/s, immediately | +40 / -30 cm/s ( 0.4 / 0.3 cm per 10 ms tick ), ramped at 100 cm/s² |
| Leaving the dead zone | Step to +/- 10 cm/s | Starts at 0 and grows with the stick |
| Centring the stick | Target snaps to `EstAlt`, overshoot | Rate ramps to 0 and the target coasts to a stop |
| Take-off command (+120 cm) | Target stepped; P = 1 loop crawled the last 40 cm, ~4.3 s | Goal profile: 60 cm/s cruise, brakes at 80 cm/s², ~2.7 s. (A first version, target stepped with a 40 cm/s cap, felt slow in flight on 18 Sep.) |
| `Command_Land` (`landThrottle` 1300 to 1150 fed as the stick) | About -50 to -87 cm/s | Unchanged: about -50 to -87 cm/s, via its own path ( `isLanding`, clamp `ALT_LAND_MAX_DESCENT_CMS` 100 ) |
| Armed on the stick, waiting at idle | Integrator winds down | Controller held at reset |
| `setAltitude ( )` / MSP far target | Approached at up to 300 cm/s, slow final approach | Same goal profile, 60 up / 30 down. Moving the stick cancels the goal |

## Open items

1. Flight test (see [TESTING.md](TESTING.md)). Stick climb and descent and `Command_Land` were flown on 18 Sep ( landing fixed after the first flight ). The goal-profile take-off ( about 2.7 s to 120 cm ) was built after that flight and is not flown yet.
2. **Landing (fixed 18 Sep 2026, working in flight after the fix).** The first build
   fed `landThrottle` through the stick scaling, which gave a 10-20 cm/s descent.
   Landing felt sluggish, and the craft **hovered a few cm above the floor with
   no touchdown detected**. Likely cause, not yet confirmed from a log: the baro
   drifts low in the craft's own downwash near the floor. At 10-20 cm/s that
   drift alone met the descent demand, so `EstAlt` kept falling while the craft
   held. The touchdown test (descent slower than 8 cm/s for 300 ms) then never
   fired, leaving only the 30 s timeout. Fix: while `isLanding`, `setVelocity`
   = `( landThrottle - 1500 ) / 4` again (-50 to -87 cm/s, the profile tested in
   `f60c9ad`), and the descent clamp is `ALT_LAND_MAX_DESCENT_CMS` (100).
   To confirm the cause, log `EstAlt` against a laser during a landing.
3. The ceiling (`limitAltitude ( )`) only clamps the stick. With a 100 cm/s²
   ramp from 40 cm/s, the target coasts about 8 cm after the clamp. That is
   inside the 25 cm `ALT_CEILING_MARGIN_CM`, but it has not been tested.
4. There is no general landed detector yet. The ground reset only covers the
   stick-armed idle hold. A touchdown and re-take-off without disarming can
   still wind the integrator down.
5. `initialThrottleHold` still survives a disarm (carried over from the
   altitude-hold topic).
6. Tuning constants are first guesses: `ALT_STICK_ACCEL_CMSS`,
   `ALT_VEL_ACCEL_CMSS`, `ALT_TARGET_LEASH_CM`.

## Stashes

`stash@{0}` "AltHold stick climb/descent speed 60/50 cm/s" is the earlier,
simpler attempt: stick scaling only. This topic supersedes it, so it can be dropped.
