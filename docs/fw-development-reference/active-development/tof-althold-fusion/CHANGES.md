# ToF Altitude Hold Fusion - Changes

[README](README.md) · [TASKS](TASKS.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

## Task 1: baseline diagnostics ( temporary )

- [target.h](../../../../src/main/target/PRIMUS_X2_v1/target.h): `LASER_TOF` and `LASER_ALT`
  enabled on `PRIMUS_X2_v1` ( made by the user before the topic started ).
- [PlutoPilot.cpp](../../../../PlutoPilot.cpp): `extern "C"` declarations of the firmware
  internals and a 10 Hz `Monitor_Print` line in `plutoLoop ( )` with `degC`, `BaroAlt`,
  `ToF`, `PaI`, `EstAlt`, `AltHold`, `Vz`, `Arm` ( ~125 bytes per tick ). **Remove before
  release** ( task 11 ).

`ToF` prints `-1` while out of range and before the first sample ( `NewSensorRange` starts at 0 ), so the ground reference is not skewed by boot samples. Review fix: the value is built as a float and promoted once, removing two `-Wdouble-promotion` warnings.

**Build ( `PRIMUS_X2_v1`, gate ).** 103.5 KB flash, 15.2 KB RAM. The gate reports 8 new
warnings under `src/`, and `PlutoPilot.cpp` has none ( checked in `build.log`, since the gate does not scan the repo root; see `.claude/TOOLING_BACKLOG.md` item 10 ): they are in laser code that `LASER_TOF` / `LASER_ALT`
now compile and that the baseline ( recorded with the laser off ) never saw.

| File | Warnings |
|---|---|
| `drivers/ranging_vl53l0x.cpp` | float→`uint16_t`, unused `dataFlag`, unused `SysRangeStatus` |
| `flight/altitudehold.cpp` | `dt` shadows a global, parameter `ToF_Height` shadows the global, float→`int32_t`, 2 × `int32_t`→float |

These are cleaned in task 3, which edits the same functions. The baseline is not regenerated.

## Task 12: estimator-internal log fields ( temporary )

- [PlutoPilot.cpp](../../../../PlutoPilot.cpp): `extern` declarations of `accel_ef_z`,
  `_accel_correction_hbf_z` and `_velocity_z` ( C++ linkage, defined in `altitudehold.cpp` ),
  and three fields `VzR`, `AccZ`, `AccB`; the line is now ~175 bytes per tick. **Remove before
  release** ( task 11 ).
- Build ( `PRIMUS_X2_v1`, gate ): 103.6 KB flash, 15.2 KB RAM; the same 8 inherited laser-code
  warnings, none in `PlutoPilot.cpp`.
- **Trimmed after app disconnects:** the 11-field line ( ~180-223 bytes ) overran the MSP TX ring
  and the app disconnected. `degC`, `PaI`, `BaroAlt` commented out, `AccZ` printed as a whole
  number, `AccB` at 1 decimal: ~115 bytes per tick. 103.5 KB / 15.2 KB, no `PlutoPilot.cpp` warnings.

## Task 3: velocity fix and estimator hygiene ( `LASER_ALT` builds )

- [imu.cpp](../../../../src/main/flight/imu.cpp) `imuCalculateAcceleration()`: under `LASER_ALT`,
  `accSum[Z]` uses `ALT_EST_ACC_Z_DEADBAND` ( **0** counts ) instead of the profile's
  `accDeadband.z` ( 40 counts, ~9.6 cm/s² ). A compile-time constant, so a saved profile cannot
  keep the old value and no EEPROM reset is needed. `accSum[Z]` has one reader, the altitude
  estimator; position estimation and optical flow read X/Y only. Baro-only builds unchanged.
- [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp):
  - `ALT_EST_TAU_S` 1.5 s for both sources: `correctedWithBaro()` no longer switches to 2 s / 5 s
    under `LASER_ALT`.
  - `checkReading()`: the tilt gate compares deci-degrees with `ALT_TOF_MAX_TILT_DECIDEG` 250; a
    sample above 25° is rejected and the baro path is used until an untilted sample arrives
    ( the old gate compared radians with 25 and never rejected ). `baro_offset` is a float
    ( was `int32_t` ), `dt` renamed `baroDt`.
  - `correctedWithTof()`: error against the filter's `_position_z` instead of the Kalman-smoothed
    integer `EstAlt`; parameter renamed `tofHeightCm` ( also in `altitudehold.h` ).
- [ranging_vl53l0x.cpp](../../../../src/main/drivers/ranging_vl53l0x.cpp) `getRange()`: the
  `LASER_LPS` IIR keeps its state in float, seeded from the first valid sample, and rounds into
  `NewSensorRange`. Truncating to `uint16_t` every update had made it stick until the raw range was
  ≥ 10 mm above it. Unused statics `dataFlag`, `SysRangeStatus` removed. `LASER_LPS` stays 0.1.

**Review fixes ( task 3 ).**

- The `LASER_TOF_L1x` branch still passed `dt`, which after the rename bound silently to the PID
  loop's global `dt` ( `pid.cpp:72` ); now `baroDt`. No target defines `LASER_TOF_L1x` today.
- The driver IIR reseeds from the raw sample after any out-of-range sample and whenever the
  estimator rejects a tilted sample ( new `tofRequestReseed ( )` in `ranging_vl53l0x.h` ). Before,
  the first sample after a gap or a flip was blended into stale history ( e.g. 1500 mm history and a
  1000 mm return gave ~145 cm ), which would put a step into `EstAlt` at every handover.
- The `imu.cpp` comment now says `accSum[Z]` also feeds the velocity loop's D term through
  `accZ_tmp`. At the default `d_vel` 1 that term truncates to 0 below 512 counts, so no change at
  default; a profile with a higher `d_vel` would now see sub-40-count acceleration.

**Build.** `PRIMUS_X2_v1` gate **passes**: 103.5 KB flash, 15.2 KB RAM ( after the review fixes ), zero new warnings ( the 8
inherited laser-code warnings from task 1 are gone ). `PRIMUS_V5` ( laser off ) also built:
102.5 KB / 14.8 KB.

## Task 5: laser / baro handover ( `LASER_ALT`, VL53L0X )

- [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp) `checkReading()`: the hard
  200 cm switch is replaced by a source state `altSourceLaser`:
  - laser → baro when a usable sample reads ≥ `ALT_TOF_HANDOVER_UP_CM` 160 cm, or when no usable
    sample ( dropout, tilt > 25°, sensor silent ) has arrived for `ALT_TOF_DROPOUT_MS` 100 ms;
    shorter gaps coast on the accelerometer ( `_position_error_z` = 0 ) instead of re-applying the
    last reading;
  - baro → laser after `ALT_TOF_RETURN_SAMPLES` 3 consecutive usable samples below
    `ALT_TOF_HANDOVER_DOWN_CM` 140 cm ( below 160 cm after a dropout / tilt handover, see review fixes );
  - `baro_offset` is a 2 s ( `ALT_BARO_OFFSET_TAU_S` ) average of `Baro_Height − ToF_Height`
    while on the laser, frozen on the baro, and continued after a return. The baro ground datum is
    untouched.
- New `altShiftFrame()`: on the return to the laser, shifts `_position_base_z`, `_position_z`, the
  15-entry history `buff`, `altHoldFilter.X`, `EstAlt`, `altTarget`, `altGoal` and `AltHold` by
  `ToF − _position_z`, so the estimate and the setpoint move together and the aircraft does not.
  `AltHold` is kept equal to `lrintf ( altTarget )` when it was, so setpoint shaping does not read
  the shift as a new goal.
- New `altHoldOnLaser()` ( declared in [altitudehold.h](../../../../src/main/flight/altitudehold.h) ).
- [PlutoPilot.cpp](../../../../PlutoPilot.cpp) ( temporary ): `Src` and `BaroAlt` logged, `VzR` and
  `AccZ` commented out, ~120 B/tick.

**Review fixes ( task 5 ).**

- **Stuck on the baro:** after a dropout or tilt handover the laser now takes back on 3 good
  samples below the *upper* edge ( 160 cm ); only a handover caused by climbing above 160 cm needs
  the 140 cm edge. Before, a hover or post-flip return between 140 and 160 cm stayed on the baro.
- **Timeout** 100 → 120 ms ( `ALT_TOF_DROPOUT_MS` ): 100 ms tripped on 2 missed samples.
- **Laser lag:** the driver IIR lags ~0.3 s ( `ALT_TOF_IIR_LAG_S` ). The laser reading is advanced
  by `VelocityZ × 0.3 s` for the offset average and the return shift, which otherwise carried
  ~9 cm into `AltHold` on a 30 cm/s descent.
- **Flip:** `altPreFlipAltHold` moved to file scope and shifted with the frame.
- `altShiftFrame()` is under `LASER_TOF` ( no unused-function warning with `LASER_TOF_L1x` ).

**Build.** Clean gated builds with full compiler output: `PRIMUS_X2_v1` 104.0 KB flash, 15.2 KB
RAM, no new warnings in `src/` ( 2692 total, none in `PlutoPilot.cpp` ); `PRIMUS_V5` 102.5 KB /
14.8 KB, no new warnings.

## Task 6: object under the craft ( `LASER_ALT`, VL53L0X )

- [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp) `checkReading()`: each raw laser
  sample ( `RangingMeasurementData.RangeMilliMeter`, before the driver IIR, tilt-corrected ) is
  compared with `_position_z`. More than `ALT_TOF_STEP_CM` 30 cm away while armed, airborne, on the
  laser and below 160 cm sets `altStepPending`:
  - the estimate is corrected by the baro with the frozen offset, and the offset average pauses;
  - 3 samples back within 15 cm cancel it;
  - after `ALT_TOF_STEP_HOLD_MS` 2500 ms, once the raw reading has been steady within
    `ALT_TOF_STEADY_CM` 10 cm for `ALT_TOF_STEADY_SAMPLES` 15 samples, `altShiftFrame ( raw − _position_z )`
    re-bases estimate and setpoint together and `AltHold` is set back to its old value, so setpoint
    shaping flies back to the old clearance as a goal ( 60 cm/s up, 30 cm/s down, stick cancels );
  - the driver IIR is reseeded at detection, cancel and re-base.
- `altHoldSource()` ( 1 laser, 0 baro, 2 object hold-off ) replaces `altHoldOnLaser()` in
  [altitudehold.h](../../../../src/main/flight/altitudehold.h); the temporary `Src` log field uses it.

**Review fixes ( task 6 ).**

- **Lag false-triggers:** the estimate follows the IIR-lagged laser, so in steady motion it trails
  the raw reading by `VelocityZ × 0.3 s` ( ~30 cm in a landing or a flip climb ). The residual, the
  cancel test and the re-base shift now use the estimate advanced by that lag.
- **Landing:** the detector is off while `isLanding`, so a false trigger can never put touchdown
  detection on the drifting near-ground baro.
- **Active goals:** a re-base during a take-off, flip-return or MSP goal keeps the goal's end point
  over the old surface instead of replacing it with the current target.

**Build.** Clean gated builds: `PRIMUS_X2_v1` 104.5 KB flash, 15.2 KB RAM, no new warnings in `src/`,
none in `PlutoPilot.cpp`; `PRIMUS_V5` 102.5 KB / 14.8 KB, no new warnings ( before the review fixes,
which are `LASER_ALT`-only ).

## Task 7: build gate

Clean gated builds on the final task 6 code, 22 Sep 2026, each with a full compiler log
( ~1 MB, 2692 warnings in total ):

| Target | Flash | RAM | New warnings in `src/` | `PlutoPilot.cpp` warnings |
|---|---|---|---|---|
| PRIMUS_X2_v1 ( `LASER_TOF` + `LASER_ALT` ) | 104.5 KB / 256 KB | 15.2 KB / 40 KB | 0 | 0 |
| PRIMUS_V5 ( laser off ) | 102.5 KB / 256 KB | 14.8 KB / 40 KB | 0 | 0 |

Both targets have ample headroom ( ~150 KB flash, ~25 KB RAM free ).

## Task 14: window-based step detection ( `LASER_ALT`, VL53L0X )

- [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp): a 24-entry ring keeps each raw
  laser sample with `_position_base_z` at the same instant. `_position_base_z` is the
  accelerometer-integrated position; the laser's position correction ( `_position_correction_z` )
  does not move it. On every good sample the window mismatch is the raw change minus the
  `_position_base_z` change over ~0.5 s ( `ALT_TOF_WINDOW_MS`; 0.4 s minimum after a reset ).
  - The hold-off now starts on either the per-sample residual ( task 6 ) or a window mismatch above
    `ALT_TOF_STEP_CM` 30 cm, so an edge the wide cone turns into a 0.5-0.8 s ramp ( a slowly slid
    box, or an edge crossed while moving ) is caught.
  - While the mismatch is above `ALT_TOF_SUSPECT_CM` 15 cm the baro-offset average pauses, so an
    edge cannot leak into the offset the next hold-off or handover flies on.
  - The ring resets on detection, cancel, tilt rejection and every frame shift.
- Offline estimate on earlier logs, using the lag-advanced filtered laser and integrated `Vz` as
  proxies: largest mismatch 6.5-11.5 cm in take-off, hover and climbs at 20-38 cm/s, 21.9 cm at one
  lift-off; the missed slow slide in `log-6` reads 24 cm on the proxy, which flattens the ramp, so
  the raw value the firmware sees is expected to be larger. To be confirmed in `log-7`.

**Review fixes ( task 14 ).** The review simulated the real filter and found the plain 0.5 s window
only catches edges faster than ~60 cm/s ( a 45 cm edge over 0.8 s peaked at 29 cm ), and that the
suspect state did not stop the laser correction, so the estimate still absorbed the edge.

- **Suspect state:** above `ALT_TOF_SUSPECT_CM` 15 cm the window reference is frozen at the start of
  the edge, the laser correction pauses ( coast on the accelerometer ) and the mismatch keeps adding
  up. It starts the hold-off above 30 cm, ends when laser and inertial agree again ( < 7.5 cm ), or
  after `ALT_TOF_SUSPECT_MAX_MS` 1000 ms without reaching 30 cm, which is a slope: the correction
  resumes and the craft follows it.
- **Offset protection after a reset:** the offset average also pauses until the window has 400 ms
  of history again.
- **No weight spike after a pause:** the offset average's dt is clamped to 0.05 s.
- **Offset moves with the frame:** `baro_offset -= delta` on the re-base and on the baro → laser
  return, so the next hold-off or handover flies on the right offset.

**Second review fixes ( task 14 ).**

- **Take-off:** no window-based suspicion or hold-off for `ALT_TOF_AIRBORNE_GRACE_MS` 1000 ms after
  becoming airborne ( lift-off read ~22 cm on the offline proxy ); the per-sample test still runs.
- **Edge or slope at the 1 s timeout:** above `ALT_TOF_SUSPECT_ESCALATE_CM` 20 cm it is an edge: the
  next sample starts the hold-off, or, if the laser reads ≥ 160 cm, the baro takes over at once; at
  or below 20 cm it is a slope and is followed. This bounds the coast and removes the 20-30 cm band
  where one noisy sample decided.
- The comment now says `_position_base_z` is not moved by the laser's position ( k1 ) correction but
  is reached, weakly, by its velocity ( k2 ) correction.

Simulation of the estimator ( tau 1.5 s, 33 ms samples, `LASER_LPS` 0.1, 10 ms tick, craft still ):

| Surface change | Spread over | Result | Estimate moved before the hold-off |
|---|---|---|---|
| 30 cm | 0-1.0 s | hold-off at the 1 s timeout ( 1.05-1.55 s after the edge starts ) | 0-4.5 cm |
| 36-55 cm | 0-1.0 s | hold-off, 0.03-0.9 s after the edge starts | 0-3 cm |
| 45-55 cm | 1.5 s | hold-off | 2-4 cm |
| 30-55 cm | 1.5-4 s ( 2 s and more for 45 cm+ ) | followed as a slope | full change + 5-9 cm overshoot |

**Build.** Clean gated builds: `PRIMUS_X2_v1` 105.3 KB flash, 15.5 KB RAM ( +0.3 KB for the ring ),
no new warnings in `src/`, none in `PlutoPilot.cpp`; `PRIMUS_V5` 102.5 KB / 14.8 KB, no new warnings.

## Task 9: docs ( and two code comments )

- [PIPELINE_UPDATE.md](PIPELINE_UPDATE.md) staged: new `Altitude_Hold_Estimator.md`, `CLAUDE.md`
  paragraphs, `Monitor_Print` guidance in five skill files.
- [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp): two comments corrected ( a silent
  sensor keeps correcting until the timeout; the lag advance applies to the offset and the return
  shift only ). `static bool tofTiltOk` moved inside `#ifdef LASER_TOF` ( it was unused, and warned, in a
  `LASER_TOF_L1x` build ). Gate still clean ( 105.3 KB ); citations in `PIPELINE_UPDATE.md` recomputed.

## Task 11: final review fixes

- `LASER_LPS` is `0.1f`, so the driver IIR stays in float maths.
- `altHoldSource()` is declared only under `LASER_ALT` ( its one caller, the diagnostics, is gone ).
- A re-base also moves the flip's return height back to the old clearance, like `AltHold` and an
  active goal.
- Landing that starts during an object hold-off re-bases the frame to the laser at once, so the
  surface step does not reach the estimator as position error and touchdown is not delayed.
- The 30° tilt comment moved into the baro-only branch it describes.
- `PlutoPilot.cpp` and `target.h` restored to HEAD: diagnostics removed, PRIMUS_X2_v1 committed with
  `LASER_TOF` / `LASER_ALT` off.

**Build.** A temporary laser-on `PRIMUS_X2_v1` build: 101.7 KB / 15.5 KB, no new warnings. Shipping
builds ( laser off ): PRIMUS_X2_v1 98.9 KB / 14.8 KB, PRIMUSX2 99.4 / 15.0, PRIMUS_V5 98.8 / 14.8; no
new warnings in files this topic touches ( PRIMUSX2 shows one pre-existing `sensors/battery.cpp`
conversion ).

## File index

| File | Tasks |
|---|---|
| `PlutoPilot.cpp` | 1, 12, 5, 6 ( temporary ) |
| `src/main/target/PRIMUS_X2_v1/target.h` | 1 |
| `src/main/flight/imu.cpp` | 3 |
| `src/main/flight/altitudehold.cpp`, `altitudehold.h` | 3, 5, 6, 14 |
| `src/main/drivers/ranging_vl53l0x.cpp`, `ranging_vl53l0x.h` | 3 |
