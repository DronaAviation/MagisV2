# Flip / Altitude Hold Regression - Investigation

[README](README.md) · [TASKS](TASKS.md) · [INVESTIGATION](INVESTIGATION.md)

## Problem

After `2a8d59a` ("shape the altitude setpoint from stick and commands"), the app flip
command no longer flips the drone. It does not rotate at all. Before that commit the
flip worked. Target: PRIMUS_X2_v1.

## How a flip is triggered from the app

- Android app flip button (`MainActivity.java:911`) → `commandType = App.BACK_FLIP` (3)
  → `SendRequestMSP_SET_COMMAND(3)`, MSP 217.
- The button is enabled only in app flight mode 0. In that mode the app holds AUX3 on
  (`MainActivity.java:678`). AUX3 is mapped to BOXBARO
  ([RxConfig.cpp:88](../../../../src/main/API-Src/RxConfig.cpp#L88)), so **ALT_HOLD is on
  whenever the app sends a flip**.
- Firmware: `serial_msp.cpp:1565` sets `current_command`. `command.cpp:255` starts
  `flipState = 1` (requires MAG_MODE). `flip(true)` runs every armed loop from
  `mw.cpp:1252-1260`, before `annexCode()`.

## Root cause (confirmed by flight log, task 2, 21 Sep 2026)

`log-1.txt`, two app flips: ASCEND ran 2.2 s each time, with `Vt` pinned at 40 cm/s, peak
`Vz` 34 and 39 cm/s, `Baro` = 1 throughout and no pitch rotation. Both timed out. Details
in [TESTING.md](TESTING.md).

1. ASCEND ([acrobats.cpp:87-124](../../../../src/main/flight/acrobats.cpp#L87)) calls
   `DEACTIVATE_RC_MODE(BOXBARO)`, sets `rcData[THROTTLE] = 2000`, and waits for
   `getEstVelocity() >= desiredVelocity` (100 cm/s, `acrobats.cpp:68`). After 2200 ms it
   gives up and sets `flipState = 0`.
2. `updateActivatedModes()` (`mw.cpp:904`, `rc_controls.cpp:372`) rebuilds the mode mask
   from AUX3 on every RX frame, so BOXBARO comes straight back on and altitude hold keeps
   running through the flip.
3. Before `2a8d59a`: throttle 2000 in ALT_HOLD gave `(2000 − 1500) / 4` = 125, clamped to
   **120 cm/s**, fed straight to the velocity loop, so VelocityZ passed 100.
4. After `2a8d59a`: the stick rate is capped at `ALT_MAX_CLIMB_CMS` = **40 cm/s**
   (`altitudehold.cpp:248`), and the velocity demand is clamped and slewed at
   `ALT_VEL_ACCEL_CMSS` 150 cm/s² (`altitudehold.cpp:255, 490`). VelocityZ can never reach
   100, so ASCEND always times out and there is no rotation. This matches the symptom.
5. Secondary: the recovery states HOLD/HOLDPOS (`acrobats.cpp:212, 303`) also rely on the
   throttle-2000 climb, so recovery would be weaker even if the flip got through.

Not involved: the flip does not write `AltHold`, so the goal trapezoid is not triggered.
`4657cbf` and `f60c9ad` do not touch the ASCEND velocity path.

## Q&A that shaped the plan (2026-09-21)

| Question | Answer |
|---|---|
| Symptom | Does not rotate at all |
| Trigger | Android app flip button (MSP), ALT_HOLD on through AUX3 |
| Done when | Flips like pre-`2a8d59a` |
| Fix approach | Flip bypasses shaping: pre-`2a8d59a` raw rate path while `flipState >= 1` |
| Recovery phases | Same bypass for the whole flip |
| Log | None yet; add fields |
| `Command_Flip` direction | Out of scope |
