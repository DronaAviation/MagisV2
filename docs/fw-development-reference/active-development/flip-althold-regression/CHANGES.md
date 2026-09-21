# Flip / Altitude Hold Regression - Changes

[README](README.md) · [TASKS](TASKS.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md)

## Temporary diagnostics (removed in task 9, 21 Sep 2026)

### Task 1: flip diagnostic log (21 Sep 2026)

- [PlutoPilot.cpp](../../../../PlutoPilot.cpp): `extern "C"` declarations for `flipState`,
  `AltHold`, `altholdDebug6/7`, `rcData`, `getEstAltitude()` and `getEstVelocity()`.
  `onLoopStart()` calls `setUserLoopFrequency(40)`. Despite the name, the argument is the
  period in ms, so this gives 40 ms / 25 Hz. `plutoLoop()` logs one line per tick
  (~140 B typical, ≤ 210 B worst). `Vt`/`Rt` are live only while the craft is level and
  the controller is active. During the rotation states (tilt > 80°) they hold their last
  value.

  | Tag | Source | Unit |
  |---|---|---|
  | `Fs` | `flipState` | 0 idle, 1 ASCEND, 2+ rotate/recover |
  | `Vz` | `getEstVelocity()` (VelocityZ) | cm/s |
  | `Vt` | `altholdDebug6` = slewed velocity setpoint `setVel` | cm/s |
  | `Rt` | `altholdDebug7` = profile rate `altRate` | cm/s |
  | `EA` | `getEstAltitude()` | cm |
  | `AH` | `AltHold` | cm |
  | `Thr` | `rcData[THROTTLE]` | µs-counts |
  | `Pit` | `Estimate_Get(Angle, AG_PITCH)` | deci-degrees |
  | `Baro` | `FlightMode_Check(ATLTITUDEHOLD)` | 0/1 |
  | `Arm` | `FlightStatus_Check(FS_ARMED)` | 0/1 |

- [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp)
  `calculateAltHoldThrottleAdjustment()`: after `setVel` is computed, it writes the
  previously unused `altholdDebug6 = setVel` and `altholdDebug7 = lrintf(altRate)`
  (marked `TEMP flip-althold-regression`). No control effect.

### Developer Mode on by default (21 Sep 2026)

- [mw.cpp](../../../../src/main/mw.cpp) `userCode()`: a new `#define DEV_MODE_ALWAYS_ON 1`
  (marked `TEMP flip-althold-regression`) bypasses the `DevModeAUX` range check, so
  `plutoLoop()` and the diagnostic log run without the Dev-mode switch. The live-RC-link
  condition (`rxIsReceivingSignal() || ppmIsRecievingSignal()`) is kept, so user code
  still stops, and `onLoopFinish()` runs, on link loss. Set the define to 0 or remove the
  block before release.

## Fix

### Task 3: flip bypasses altitude setpoint shaping (21 Sep 2026)

[altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp):

- `#include "flight/acrobats.h"` for `flipState`.
- New block after `resetAltSetpoint()`:
  - `ALT_FLIP_MAX_CLIMB_CMS` 120 and `ALT_FLIP_MAX_DESCENT_CMS` 100, the pre-shaping clamp.
  - `flipActive()`: `flipState != 0`, and always false without `ENABLE_ACROBAT`.
  - `flipVelocitySetpoint()`: the pre-`2a8d59a` controller. Outside the deadband,
    `setVelocity` goes straight to the velocity loop and `AltHold` is pinned to `EstAlt`.
    Inside it, the P-only position loop runs (`P8[PIDALT] × err / 128`, ±300). The shaping
    state (`altTarget`, `altRate`, `altGoalActive`, `altVelTarget`) follows, so shaping
    resumes from the current state when the flip ends.
- `applyMultirotorAltHold()`: a new `flipActive()` branch after `isLanding` (landing keeps
  priority) sets `setVelocity = (rcData[THROTTLE] − 1500) / 4`, clamped −100..+120 outside
  the deadband and 0 inside it. `offloadHoverTrim()` is not called during a flip.
- `calculateAltHoldThrottleAdjustment()`: after the tilt, reset and `ctrlDt` checks it now
  chooses `flipVelocitySetpoint()` or `shapedVelocitySetpoint(ctrlDt)`. The shaping code
  moved unchanged into `shapedVelocitySetpoint()`. The velocity PID tail is unchanged.

### Task 4: clean hand-back from flip to shaped ALT_HOLD (21 Sep 2026)

Driven by the flyaway in `log-2.txt` (TESTING.md test 2).
[altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp)
`calculateAltHoldThrottleAdjustment()` now edge-detects `flipActive()` at the top,
ahead of the >80° tilt early return, so a flip that ends tilted is still handed back:

- **Rising edge** (flip starts): saves `errorVelocityI`, the pre-flip hover trim, in
  `altPreFlipVelocityI`. It saves 0 if BARO_MODE is off, because a user-code flip can
  start with ALT_HOLD off and the integrator would be wound against an unactuated
  `AltHold`. 0 is what BARO entry resets it to.
- **Falling edge** (flip ends, completed or timed out): calls `resetAltSetpoint()`
  (`altRate = 0`, `altVelTarget = 0`, goal cleared, `AltHold = altTarget = EstAlt`) and
  restores `errorVelocityI = altPreFlipVelocityI`. The drone holds where the flip ended,
  and the wind-up from HOLD (+120 demanded while falling at −160) is dropped.
- Known and accepted (superseded by task 10's `ALT_FLIP_EXIT_IGNORE_MS` window): on the exit tick HOLD writes throttle 2000 and clears `flipState`
  in the same call. Until the next RX refresh (≤ 20 ms) the shaped branch sees full
  stick (40 cm/s), so `altRate` moves by at most ~2 cm/s.

Diagnostics: the `VI` field (`errorVelocityI / 8192`, throttle counts) was added to the
`PlutoPilot.cpp` log (~150 B typical, ≤ 230 B worst).

### Task 10: return to the pre-flip height (21 Sep 2026)

Driven by `log-3.txt` / `log-4.txt` (TESTING.md tests 3-4): each flip ended 40-122 cm high.
[altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp):

- The flip edge block in `calculateAltHoldThrottleAdjustment()`:
  - **Rising edge:** also saves `altPreFlipAltHold = AltHold` (the target ALT_HOLD was
    holding) and `altPreFlipReturn = FLIGHT_MODE(BARO_MODE)`.
  - **Falling edge:** after `resetAltSetpoint()` and the trim restore, sets
    `AltHold = altPreFlipAltHold` when `altPreFlipReturn` is set. Goal detection in
    `shapedVelocitySetpoint()` then flies it back on the existing goal profile (60 up /
    30 down cm/s cruise, 80 cm/s² braking). Stick input cancels it, as for take-off.
    The return is armed only if BARO is on, the craft is not idle-held and the pre-flip
    `AltHold` ≥ `ALT_FLIP_RETURN_MIN_CM` (20 cm). Otherwise the drone holds at the exit
    height (task 4 behaviour), so a flip sent while armed on the floor does not fly back
    down to it.
- `applyMultirotorAltHold()`: a new `ALT_FLIP_EXIT_IGNORE_MS` (100 ms) window after the
  flip ends forces `setVelocity = 0`. On its last tick the flip writes 2000 into
  `rcData[THROTTLE]` and clears `flipState` in the same call. Until the next RX frame
  (≤ 20 ms) that would read as full stick and cancel the return goal
  (`setVelocity != 0 → altGoalActive = false`). Landing keeps priority.

### Task 11: soften the post-flip braking (21 Sep 2026)

Driven by the 15-46 cm dip below target in `log-5.txt` (TESTING.md test 5).
[altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp)
`calculateAltHoldThrottleAdjustment()`:

- The flip falling edge also stores `altFlipExitAtMs = millis()`.
- For `ALT_FLIP_EXIT_I_HOLD_MS` (500 ms) after that, the velocity integrator is not
  updated (`holdFlipExitI`). It stays at the restored pre-flip trim while the P and D
  terms brake the ~120 cm/s exit climb, so braking no longer banks −20 counts of
  negative trim that later drops the craft below the returning target. Disarm still
  zeroes the integrator, and the disarm/idle reset also clears the window, so a quick
  re-arm does not run with the integrator held.

## File index

| File | Tasks |
|---|---|
| `PlutoPilot.cpp` | 1, 4 (temp, removed in task 9: no diff against HEAD) |
| `src/main/flight/altitudehold.cpp` | 1 (temp, removed in task 9), 3, 4, 10, 11 |
| `src/main/mw.cpp` | Dev mode default on (temp, removed in task 9: no diff against HEAD) |
