# ToF Altitude Hold Fusion - Investigation

[README](README.md) · [TASKS](TASKS.md) · [INVESTIGATION](INVESTIGATION.md) · [TESTING](TESTING.md)

## Problem

On `PRIMUS_X2_v1` with `LASER_TOF` and `LASER_ALT` enabled ( VL53L0X on I2C1 ), altitude
hold over a flat floor bobs slowly ( ~10-30 cm, 1-2 s period ). A hand placed suddenly under
the craft makes it climb hard; removing the hand makes it drop hard, then it settles. The
user reports laser hold was never stable on this drone.

## What the code does today ( scout, 21 Sep 2026 )

- `drivers/ranging_vl53l0x.cpp` `getRange()`: single ranging every 33 ms, blocking start
  + read inside the `UPDATE_LASER_TOF_TASK` periodic slot ( `mw.cpp` ~804-815 ),
  `LASER_LPS 0.1` IIR on `NewSensorRange` ( mm ), `out_of_range` when RangeStatus != 0 or
  > 2000 mm.
- `flight/altitudehold.cpp` `apmCalculateEstimatedAltitude()` ( 100 Hz ): with `LASER_ALT`
  it calls `checkReading()` ( ~843-871 ) instead of `checkBaro()`.
- `checkReading()`: laser used when `0 < ToF_Height < 200` cm and in range, otherwise
  `correctedWithBaro(Baro_Height - baro_offset)` where `baro_offset` = LPF( baro ) −
  `ToF_Height` ( ~866 ). Hard switch at 200 cm, no hysteresis. The tilt gate ( ~861 )
  compares radians against 25, so it is always true.
- `correctedWithTof()` ( ~941-951 ): `_position_error_z = ToF_Height − EstAlt`, tau 1.5 s;
  first read calls `setAltitude(ToF_Height)`. No step or outlier rejection.
- `correctedWithBaro()` ( ~914-938 ): delayed-history error, tau 2 s ( 5 s above 30° ), and
  it overwrites `_time_constant_z`, so alternating sources flips the estimator gains.
- The setpoint-shaping and flip code reads only `EstAlt` / `VelocityZ`, so it is untouched
  by the source; `LASER_TOF` alone is logging-only.

## Working hypothesis ( before the baseline log, superseded below )

1. **Bob**: laser samples ( 33 ms, 0.1 IIR → ~0.3 s lag ) drive the 100 Hz estimator as a raw
   position error with a 1.5 s tau, and the position loop closes on `EstAlt` with the shaped
   40/30 cm/s limits. Lag in the laser path plus gain flips at the source switch give a slow
   limit cycle. To be confirmed from the baseline log ( ToF vs EstAlt vs AltHold phase ).
2. **Hand under**: a 30 cm hand under a 100 cm hover pulls `EstAlt` down ~70 cm within
   ~1.5 s; the position loop climbs at 40 cm/s. On removal `EstAlt` jumps back and it
   descends at 30 cm/s.

## Q&A that shaped the plan

- Work kind: behaviour change with a baseline investigation first.
- Done: 1 m hover within ±5 cm of `AltHold` for 30 s; no `EstAlt` jump through the handover;
  no reaction to a sudden object for ~2.5 s, then a ramped climb/descent.
- Handover band: baro above 180 cm, laser below 160 cm ( user asked 180/180; a zero-width
  band chatters, 160 assumed as the lower edge ). Same band for the VL53L1X later.
- Step: > 30 cm within 0.5 s; hold-off 2.5 s; symmetric on removal.
- Landing/touchdown from the laser: out of scope. VL53L1X and PRIMUS_V5: out of scope.
- Log fields: degC, BaroAlt, ToF, PaI, Arm + EstAlt, AltHold, VelocityZ.

## Baseline findings ( `logs/log-1.txt`, task 2, 21 Sep 2026 )

FW 3.8.1 working tree, `LASER_TOF` + `LASER_ALT`, indoor, `AltHold` fixed at 120 cm from
5.5 s to 65 s, no throttle input. 641 records at 10 Hz over 68 s, no dropped fields. **The log
contains no hand test**: the largest laser change within 0.5 s in the hover is 15 cm.

| Measure | Hover 8-50 s | Calm 50-57.5 s |
|---|---|---|
| `EstAlt` sd / peak-to-peak | 6.0 / 30 cm | 1.1 / 4 cm |
| `ToF` sd / peak-to-peak | 7.6 / 39 cm | 0.9 / 3.6 cm |
| `EstAlt − AltHold` mean / max | −4.1 / 21 cm | −3.1 / 5 cm |
| Bob period ( 9 cycles, `EstAlt` upward crossings ) | 3.4-4.5 s, median 4.1 s | - |
| Laser sample-to-sample sd | 1.2 cm | - |
| Baro sample-to-sample sd | 13.4 cm | - |
| `ToF − EstAlt` sd | 2.8 cm | 0.5 cm |
| `EstAlt` lag behind `ToF` | ~200 ms | - |

**Estimator velocity vs its own position** ( least-squares fit of `Vz` against the central
difference of the position, best lag ):

| Against | `Vz` lag | r | `Vz` gain |
|---|---|---|---|
| d`EstAlt`/dt | leads by 500 ms | 0.78 | 0.42 |
| d`ToF`/dt | leads by 300 ms | 0.79 | 0.30 |

### Conclusions

1. **The laser is not the cause of the bob.** Its noise is 1.2 cm sample to sample, `EstAlt`
   follows it within 2.8 cm, and in a 7.5 s calm window the craft held `EstAlt` within 1 cm sd.
   The bob is real motion of the craft.
2. **The bob is a lightly damped limit cycle in the altitude loop**: ~30 cm peak-to-peak at
   ~4.1 s ( the pilot's 1-2 s estimate was short ). The loop can hold, but once disturbed it rings.
3. **The estimated vertical velocity is too small and phase-shifted.** `VelocityZ` is only
   0.3-0.4× the rate of change of the estimator's own position and runs 0.3-0.5 s ahead of it.
   The velocity loop damps on this signal, so it has less than half the damping it was tuned for.
   This is the leading root cause. The mechanism is **not yet identified**. In the
   complementary filter ( `apmCalculateEstimatedAltitude()` ) the velocity above the ~0.1 Hz
   crossover ( tau 1.5 s ) comes from the accelerometer, so candidates are the accelerometer
   path ( `accel_ef_z × accVelScale`, `accSum` averaging ) or the bias term
   `_accel_correction_hbf_z` ( k3 ) absorbing real motion. The two Kalman smoothers after the
   filter ( `altHoldFilter` Q 0.01 / R 0.5, `velHoldFilter` Q 0.1 / R 1.0 ) settle to ~30-80 ms
   lag and are too light to explain a 60 % loss.
4. **Laser lag inside the loop.** The driver's `LASER_LPS 0.1` IIR at 33 ms has a ~0.3 s time
   constant, and `correctedWithTof()` computes its error against the Kalman-filtered,
   integer-rounded `EstAlt` rather than the filter's internal `_position_z`, so a filtered
   output is fed back into the filter.
5. **Steady offset.** `EstAlt` sits 3-4 cm below `AltHold` on average, even in the calm window.
6. **Baro is very noisy here**: 13 cm sd sample to sample and ~8 cm below the laser. A laser→baro
   handover must not pass that noise straight into `EstAlt` ( relevant to task 5 ).
7. **Tilt gate** confirmed always true ( `tilt` is radians, compared with 25 ); the cosine is
   therefore always applied, which is harmless at hover angles.

### What is still missing

- The hand-under response ( not in the log ).
- **Candidate found in review ( task 12 ):** `imuCalculateAcceleration()` ( `flight/imu.cpp` ~255 )
  applies `applyDeadband ( …, accDeadband->z )` before `accSum`. The default is 40 counts
  ( `config.cpp` ~535 ), which is ~9.6 cm/s² at `acc_1G` 4096. The bob's peak acceleration is only
  ~35 cm/s² ( 15 cm amplitude at 4.1 s ), so the deadband zeroes the slow part of every cycle and
  cuts ~9.6 cm/s² off the rest: a strong candidate for the lost velocity. Test 2's slow and fast
  hand-held sets separate it from a plain gain error.
- The internals needed to identify conclusion 3: accelerometer vertical acceleration in cm/s²,
  the bias term, and the velocity before the Kalman smoother. Added as task 12. **Answered: see
  below.**

## Estimator diagnostics ( `logs/log-2.txt`, task 12, 21 Sep 2026 )

133 s, 1263 records, fields `ToF EstAlt AltHold Vz VzR AccZ AccB Arm` ( ~115 B/tick ). Disarmed
0-37 s: hand-held phase 6-25 s ( four lifts floor → ~85 cm → floor, ~6.5 s per cycle, larger than
planned ), then on the floor. Armed 38-129 s: take-off to `AltHold` 120 cm, hover with several hand
tests ( ~60, ~73, ~76, ~90-93 s ), land. One Wi-Fi delivery gap of 1.1 s at 61.5-62.6 s with
records lost and the rest bunched; samples right after it are not usable for rates.

### Root cause of the small `Vz`: the accelerometer Z deadband

| Phase | Real acceleration ( laser, sine-equivalent amplitude ) | Deadband predicts gain | Measured `Vz` gain vs d`ToF`/dt | vs d`EstAlt`/dt |
|---|---|---|---|---|
| Hand-held, loop off ( 6-25 s ) | ~63 cm/s² | ~0.80 | 0.78 ( r 0.97, leads 200 ms ) | 0.88 |
| Hover, no hand ( 98-127 s ) | ~17 cm/s² | ~0.30 | 0.36 ( r 0.83, leads 300 ms ) | 0.45 |

`imuCalculateAcceleration()` ( [imu.cpp:255](../../../../src/main/flight/imu.cpp#L255) ) applies
`applyDeadband ( …, accDeadband->z )` with the profile default of **40 counts**
( [config.cpp:535](../../../../src/main/config/config.cpp#L535) ), ~9.6 cm/s² at `acc_1G` 4096.
A deadband removes a fixed amount from every sample and zeroes anything smaller, so it is a gain that
**depends on the amplitude**: it costs little on large hand motions and more than half on the gentle
hover bob. The simulated fundamental gain of a 9.6 cm/s² deadband on a sine matches both phases
( table ). `AccZ` reads exactly 0 in 25-32 % of samples. With the loop closed the craft only moves
gently, so the velocity loop sees a third of the real vertical speed and has a third of its damping:
the ~4-5 s limit cycle.

Ruled out:

- **Kalman smoother**: `Vz` and `VzR` differ by at most 0.5 cm/s everywhere.
- **Plain gain error** in `accVelScale`: the gain would be the same in both phases, not 0.8 vs 0.36.

Contributing, secondary:

- **Bias term `AccB` absorbs motion.** It starts at 0 at arm, settles at 6.4 cm/s² mean in the
  hover ( sd 1.1 ), and jumps 3-6 cm/s² on every large height change ( e.g. 5 → 9.8 at 63 s ). Its
  correlation with `ToF` is 0.46 at 0.8 s lag. With tau 1.5 s, `k3` = 0.30 s⁻³; part of the deadband
  loss is being "explained" as bias. Expected to calm once the deadband is fixed; re-check in task 4.
- **`accDeadband.z` is shared**: the same value feeds `accSumXYZ[Z]` ( [imu.cpp:261](../../../../src/main/flight/imu.cpp#L261) )
  for other consumers, and it is a stored profile setting, so a default change does not reach a
  board with a saved profile. Task 3 must choose the scope.

### Hover ( 98-127 s, no hand )

`EstAlt` sd 4.5 cm, p-p 19 cm, max error 12 cm; `ToF` p-p 26 cm. Cycle periods 4-5 s. Same limit
cycle as `log-1`, slightly smaller.

### Hand under the craft

| Event | Laser seen by the log | `EstAlt` | Response |
|---|---|---|---|
| ~60.2 s | 110 → 76 cm over 0.6 s ( −21.5 cm in the steepest 0.5 s ) | 111 → 89 in 0.8 s ( −22 cm ) | Loop climbs; after the hand leaves, `EstAlt` overshoots to 159 ( **+39 cm** over `AltHold` ), settles in ~5 s |
| ~73-76 s | steps of 15-23 cm | −24 / +23 cm | same pattern, smaller |
| ~90-93 s | +34 cm in 0.5 s | −17 / +29 cm | same pattern |

- **The driver IIR hides the step.** `LASER_LPS 0.1` at 33 ms spreads an instant hand step over
  ~0.6 s, so the steepest 0.5 s window of a ~34 cm hand shows only ~21 cm. A "> 30 cm within 0.5 s"
  test on `NewSensorRange` would miss most hand steps. **Task 6 must detect on the raw range**
  ( `RangingMeasurementData.RangeMilliMeter` ) before the IIR.
- **Detection has to be fast.** `EstAlt` moves ~20 cm within 0.8 s, so the hold-off has to start
  within ~100 ms of the step.

### Laser range on this floor

Out-of-range samples at 63.1, 63.3 and 73.0 s, at 172-176 cm. On this floor the VL53L0X drops out
at **~172 cm**, below the planned 180 cm laser → baro handover. **Task 5's band has to move down**
( for example 150 up / 130 down ) or the handover will be triggered by dropouts rather than height.

## Post-take-off settle ( `log-3`, `log-3-temp`, task 13, 22 Sep 2026 )

In the hover the accelerometer reads ~−7.3 cm/s² on average ( `AccZ`, after the zero learned while
disarmed ), and the complementary filter's bias term cancels it ( `AccB` +7.3 cm/s², sum 0.0 in
`log-3-temp` ). The bias term is never reset, so its value at arm depends on history:

| Log | `AccB` at arm | Settle after reaching height |
|---|---|---|
| `log-3` ( fresh power-up ) | ~0 | ~15 s, 5-8 cm below `AltHold` meanwhile |
| `log-3-temp` ( flown after `log-3` ) | 5.2 cm/s² | ~3 s |

A wrong bias reads as a vertical acceleration, so until the term has learned the offset the
estimator's velocity is biased and the craft sits low. The pilot accepted the hold as it is; no
change. At touchdown the offset disappears and the term winds up to ~16-18 cm/s², so `Vz` reads
~15-20 cm/s on the floor for ~10 s.

## Object detector in flight ( `log-6`, task 8, 22 Sep 2026 )

The per-sample test ( raw laser more than 30 cm from the lag-advanced estimate ) catches a box put
in or pulled out briskly: 10 re-bases, holds of 2.4-3.4 s with `EstAlt` moving 4-8 cm, then ramps
at ~55 cm/s up and ~30-37 cm/s down.

It misses a **slow slide**. The VL53L0X has a wide cone, so a box edge leaving it changes the reading
over ~0.5-0.8 s ( 95 → 113 cm over 0.65 s, then +11 cm at ~60 s ). The estimate is corrected towards
every sample, so the gap between raw and estimate never reaches 30 cm; the change is absorbed as if
the craft had climbed, and the position loop then descends by the box height with no hold-off. The
same absorption also feeds the baro-offset average, so a later hold-off flies the baro with a wrong
offset ( ~25 cm dip at 127-131 s ).

A residual against a laser-corrected estimate cannot see changes slower than the estimator's own
correction. The test must compare the laser with a reference the laser does not steer: the inertial
displacement over a short window ( task 14 ).
