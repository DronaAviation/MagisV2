# ToF Altitude Hold Fusion - Testing

[README](README.md) · [TASKS](TASKS.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

## Log fields ( temporary `PlutoPilot.cpp` diagnostics, task 1 )

One record per `plutoLoop ( )` tick, 10 Hz ( default `userLoopFrequency` 100 ms ), about
175 bytes per tick ( 125 before task 12 ) against the ~250 byte `Monitor_Print` ceiling. Runs only in Developer
Mode with a live RC link. Each field arrives on its own line with a PlutoMonitor timestamp.

| Field | Meaning | Unit |
|---|---|---|
| `degC` | ICP-10111 die temperature | °C, 2 decimals |
| `BaroAlt` | Compensated barometric altitude | cm |
| `ToF` | VL53L0X range after the driver's 0.1 IIR ( `NewSensorRange / 10` ), slant range, `-1` out of range | cm |
| `PaI` | ICP-10111 raw pressure | Pa, 2 decimals |
| `EstAlt` | Estimator altitude ( what the position loop flies ) | cm |
| `AltHold` | Altitude setpoint | cm |
| `Vz` | Estimator vertical velocity `VelocityZ` | cm/s |
| `VzR` | Velocity before the Kalman smoother ( `_velocity_z` ), from task 12 | cm/s, 1 decimal |
| `AccZ` | Vertical acceleration after `accVelScale` ( `accel_ef_z` ), 10 ms average at the sample instant, from task 12 | cm/s², 1 decimal |
| `AccB` | Complementary-filter accelerometer bias term ( `_accel_correction_hbf_z` ), from task 12 | cm/s², 2 decimals |
| `Src` | Estimator source: 1 laser, 0 baro, 2 object hold-off ( task 5 on, value 2 from task 6; `VzR` and `AccZ` commented out from task 5 ) | - |
| `Arm` | 1 armed, 0 disarmed; ends the record | - |

`ToF` is read from the driver, not from the estimator's `ToF_Height`, because
`ToF_Height` only refreshes while the estimator is on the laser path.

## Test 1: baseline ( `logs/log-1.txt` )

**Build.** FW 3.8.1 working tree, `PRIMUS_X2_v1`, `LASER_TOF` + `LASER_ALT` on, no
estimator changes. `Build/PRIMUS_X2_v1/DEFAULT_PRIMUS_X2_v1_3.8.1.hex`.

**Site.** Indoor, flat matt floor, no fan or draught. Note the floor surface and lighting.

**Sequence.**

1. Power on, connect PlutoMonitor, turn on Developer Mode, start logging.
2. 20 s disarmed on the floor ( ground `ToF` and baro reference ).
3. Arm, take off in ALT_HOLD, bring it to ~1 m, centre the throttle stick.
4. **Hover 30-60 s** with no throttle input; roll/pitch only to stay over the same spot.
5. **Hand test:** hold a flat hand ( or a box ) ~30-40 cm under the craft for ~5 s, then
   remove it. Wait 10 s. Repeat once.
6. Land, disarm, 20 s disarmed.

**Also note** anything the log cannot show: visible bob size, how hard it climbed and
dropped on the hand test, any bumps or catches.

**What the analysis looks for.**

| Question | Evidence |
|---|---|
| Is the bob in the sensor or in the loop? | `ToF` vs `EstAlt` vs `AltHold` amplitude and phase over the hover |
| How big and how slow is the bob? | Peak-to-peak and period of `EstAlt` and `ToF` in the hover window |
| Does the estimator ever switch to baro at 1 m? | `ToF` dropouts ( `-1` ) or `ToF` > 200 cm |
| Hand test: how fast does `EstAlt` follow the step, how fast does it climb and drop? | `EstAlt` slope after the step, `Vz` peak, `AltHold` change |
| Laser noise vs baro noise | `ToF` and `BaroAlt` standard deviation in the hover window |

**Pass for the baseline:** none. This test records the current behaviour.

**Results ( task 2 ).** Flown 21 Sep 2026, 68 s, 641 records, no dropped fields. Only 3 s
disarmed before arming ( laser reads 3.4 cm on the ground ). `AltHold` 120 cm from 5.5 s, landed
at 65 s. **No hand test in the log** ( largest laser change within 0.5 s: 15 cm ).

| Measure ( hover 8-50 s ) | Value |
|---|---|
| `EstAlt` sd / peak-to-peak | 6.0 / 30 cm |
| Bob period | median 4.1 s ( 3.4-4.5 s, 9 cycles ) |
| `EstAlt − AltHold` mean / max | −4.1 / 21 cm |
| Laser / baro sample-to-sample sd | 1.2 / 13.4 cm |
| `Vz` vs d`EstAlt`/dt | gain 0.42, leads 500 ms, r 0.78 |
| Calm window 50-57.5 s, `EstAlt` sd | 1.1 cm |

Against the ±5 cm done-when: **fails** ( max error 21 cm ). Interpretation in
[INVESTIGATION.md](INVESTIGATION.md#baseline-findings--logslog-1txt-task-2-21-sep-2026-).
Analysis scripts: `tools/flightlog.py summary` / `table`, plus a one-off fit of `Vz` against the
position slope.

## Test 2: estimator diagnostics and hand test ( `logs/log-2.txt`, task 12 )

**Build.** Task 1 firmware plus `VzR`, `AccZ` ( whole cm/s² ), `AccB` ( 1 decimal ); `degC`,
`PaI` and `BaroAlt` commented out, ~115 bytes per tick. No control change.

**Disconnects ( first attempt ).** With all 11 fields ( ~180 bytes typical, ~223 worst ) the app
kept disconnecting while logging. `Monitor_Print` writes each field as its own `$D` frame into the
MSP UART's 256-byte TX ring at 115200 baud ( ~22 ms to drain ), and `uartWrite ( )` never checks for
room, so the log burst plus the app's own MSP replies overran the ring and corrupted those replies.
Trimmed to ~115 bytes, below the ~125-135 bytes of `log-1.txt`, which ran without a disconnect.
`Build/PRIMUS_X2_v1/DEFAULT_PRIMUS_X2_v1_3.8.1.hex`.

**Sequence.** Wait a few seconds after power-on, then turn on Developer Mode and start logging.

1. **Hand-held, disarmed, ~40 s.** Hold the drone level over the floor at about 60-80 cm, laser
   pointing down, motors off. Hold still for 5 s. Then move it **up and down by about 30 cm,
   one full cycle every ~4 s, for 5 cycles**. Hold still for 5 s. Then the same 30 cm but
   **faster, one cycle every ~2 s, for 5 cycles**. Hold still for 5 s. The altitude loop is off,
   so this measures the estimator's velocity against the laser on its own. The slow and fast
   sets have different accelerations ( ~35 vs ~150 cm/s² peak ), which separates a deadband
   from a plain gain error.
2. **On the floor, disarmed, 20 s.** Ground reference.
3. Arm, take off in ALT_HOLD to ~1 m, centre the throttle.
4. **Hover 60 s** with no throttle input.
5. **Hand test, twice.** Flat hand ( or a box ) 30-40 cm under the craft for ~5 s, remove it,
   wait 10 s.
6. Land, disarm, 20 s on the floor.

**Note** what the log cannot show: how far it climbed or dropped on each hand test, the floor
surface, anything that caught your eye.

**What the analysis looks for.**

| Question | Evidence |
|---|---|
| Is `Vz` small even with the loop off? | Hand-held phase: `VzR` and `Vz` against d`ToF`/dt, gain and lag |
| Does the Kalman smoother shrink it? | `Vz` against `VzR` |
| Is the accelerometer path short? | `AccZ` reading exactly 0 during slow motion ( deadband ), and its mean at rest |
| Deadband or gain error? | `VzR` gain against d`ToF`/dt in the slow set vs the fast set: a deadband loses far more in the slow set, a gain error loses the same in both |
| Does the bias term eat real motion? | `AccB` swinging in phase with the motion instead of staying near constant |
| Hand step | `EstAlt` slope after the step, `Vz` peak, height change, recovery time |

**Caveats ( review, task 12 ).** `AccZ` is measured *after* the 40-count accelerometer deadband
( ~9.6 cm/s² with `acc_1G` 4096 ) and is one 10 ms window per 100 ms sample, so do not integrate
it to rebuild velocity. Base the attenuation on `VzR` against the laser slope. `AccB` is never
reset ( not by `AltRst()`, arming or `setAltitude()` ), so read its drift relative to its value at
arm, not its absolute level.

**Results ( task 12 ).** Flown 21 Sep 2026, 133 s, 1263 records, no dropped fields, no app
disconnect at ~115 B/tick. The hand-held phase was four lifts floor → ~85 cm at ~6.5 s per cycle
instead of the two 30 cm sets; the amplitude difference to the hover still separates deadband from
gain error. One 1.1 s Wi-Fi gap at 61.5-62.6 s ( records lost ).

| Measure | Hand-held, loop off | Hover 98-127 s |
|---|---|---|
| Real acceleration amplitude ( laser ) | ~63 cm/s² | ~17 cm/s² |
| `Vz` gain vs d`ToF`/dt | 0.78 ( r 0.97 ) | 0.36 ( r 0.83 ) |
| Deadband model gain | ~0.80 | ~0.30 |
| `AccZ` exactly 0 | 32 % | 25 % |
| `VzR − Vz` max | 0.5 cm/s | 0.5 cm/s |
| `EstAlt` sd / p-p / max error | - | 4.5 / 19 / 12 cm |
| `AccB` mean / sd | - | 6.4 / 1.1 cm/s² |

Hand test: `EstAlt` follows the hand down ~20 cm within 0.8 s, the loop climbs, and after the hand
leaves `EstAlt` overshoots up to **+39 cm** over `AltHold` and settles in ~5 s. Laser out of range at
172-176 cm. Interpretation in [INVESTIGATION.md](INVESTIGATION.md#estimator-diagnostics--logslog-2txt-task-12-21-sep-2026-).


## Test 3: hover check on the task 3 firmware ( `logs/log-3.txt`, task 4 )

**Build.** Task 3 firmware ( Z deadband 0 under `LASER_ALT`, one tau 1.5 s, tilt gate, float laser
IIR with reseed ), same ~115 B/tick log. Same sequence as test 2. Flown 21 Sep 2026: 138 s, 1299
records, no dropped fields. Hand-held 6-21 s, floor 21-39 s, armed 40-130 s, `AltHold` 120 cm from
~42 s, hand tests ~82-108 s, landed ~130 s.

**Velocity gain** ( `Vz` against the laser rate, both smoothed over 7 samples, best lag; the plain
10 Hz central difference is noise-dominated in a quiet hover and biases the fit towards 0, which also
flattered the earlier hover numbers downwards ):

| Phase | `log-2` ( before ) | `log-3` ( after ) |
|---|---|---|
| Hand-held, large motion | 0.83 ( r 0.98 ) | **0.97** ( r 0.98 ) |
| Quiet hover 51-81 s | - | 0.86 geometric, r 0.45: laser-rate sd only 2.1 cm/s, too little motion to measure |
| Hover after hand tests 110-128 s | 0.46 ( 98-127 s, r 0.87 ) | 0.54 ( r 0.84 ), `Vz` leads by 600 ms |

`AccZ` exactly 0: 25-32 % → 2-12 %.

**Hover band.**

| Window | `EstAlt − AltHold` mean / sd / range | within ±5 cm | laser − `AltHold` range |
|---|---|---|---|
| `log-1` 8-50 s ( baseline ) | −4.1 / 6.0 / worst 21 cm | - | p-p 39 cm |
| 51-81 s ( 30 s ) | −2.7 / 2.6 / −8..+1 | 78 % | −9.2..+2.7 |
| 110-128 s | −0.4 / 3.6 / −6..+6 | 89 % | −8.0..+10.1 |

Best 30 s window ( 49.5-79.5 s ): max |error| 8 cm on `EstAlt`, 9 cm on the laser. No periodic
cycle ( the upward-crossing intervals are 7 and 14 s, not 4 s ).

**Verdict.** The limit cycle is gone ( sd 6.0 → 2.6 cm, p-p 30 → 9 cm ). The ±5 cm / 30 s
criterion is **missed by ~3 cm**: after take-off the craft sits 5-8 cm below `AltHold` and creeps up
over ~15 s ( 48 → 64 s ), then holds within ±3 cm. Follow-up: task 13.

**Also seen.**

- `AccB` climbs 5 → 8.5 cm/s² through the hover; its correlation with `ToF` is 0.72 at 1.4 s lag.
- After landing `AccB` winds up to 18 cm/s² and `Vz` reads 15-21 cm/s with the craft still on the
  floor, decaying over ~10 s. A quick re-arm could start with a wrong velocity.
- Hand tests still overshoot to +27..+33 cm over `AltHold` ( task 6 ).

## Test 3b: plain hover, no hand ( `logs/log-3-temp.txt`, task 13 )

Same task 3 firmware. Arm, take off in ALT_HOLD to 120 cm, hover ~60 s, land; below the laser
limit throughout. Flown 21 Sep 2026 after the `log-3` flight without a power cycle: 77 s, 728
records, no dropped fields.

| Window ( 30 s ) | `EstAlt − AltHold` mean / sd / range | within ±5 cm | laser − `AltHold` range | within ±5 cm |
|---|---|---|---|---|
| 10-40 s | −0.9 / 1.8 / −3..+3 cm | 100 % | −4.3..+5.1 cm | 99 % |
| 37-67 s | −1.4 / 1.4 / −6..+1 cm | 99 % | −6.3..+1.7 cm | 96 % |

**The ±5 cm / 30 s criterion is met** ( 10-40 s ). Settled ~3 s after reaching height.

- **Why this hover settled fast and `log-3` did not.** In the hover the accelerometer reads
  −7.3 cm/s² on average ( `AccZ` ) and the bias term cancels it ( `AccB` +7.3, sum 0.0 ). Here
  `AccB` was still 5.2 at arm, carried over from the previous flight. In `log-3` it started at ~0
  after a fresh power-up and took ~15 s to learn the offset, which was the slow creep.
- **Isolated laser dropouts** ( `ToF` −1 ) at 5.6, 8.4, 20.3 and 56.1 s, single samples at
  60-133 cm, so not the range limit. The estimator used the baro path for one sample each without
  a visible step.
- **Take-off overshoot:** the laser reached 144 cm ( +24 cm ) at 7.3 s before settling.
- **Touchdown:** `AccB` winds up to 16.4 cm/s² and `Vz` reads 13-16 cm/s on the floor, as in `log-3`.

## Test 4: handover bench ( `logs/log-4.txt`, task 5 )

**Build.** Task 5 firmware with the review fixes: handover 160 cm up / 140 cm down ( 160 cm after a
dropout ), 120 ms dropout timeout, frame shift on the return. Clean build of 22 Sep 14:53. Log `BaroAlt ToF EstAlt AltHold Vz Src AccB Arm`, ~120 B/tick. **Motors off,
disarmed, throughout.**

**Sequence.** Power on, wait a few seconds, Developer Mode on, start logging. Hold the drone level,
laser pointing down, over the same floor.

1. **20 s on the floor.**
2. **Slow lift to ~190-200 cm** over ~5 s ( above 160 cm; a stool or a raised arm ), hold 5 s.
3. **Slow lower to ~100 cm** over ~5 s ( below 140 cm ), hold 5 s.
4. Repeat 2-3 once.
5. **At ~100 cm, pass a hand quickly across the laser** ( under 0.1 s ) three times, then hold a
   hand under it for ~1 s once.
6. Back on the floor, 10 s.

**Pass.**

| Check | Expected |
|---|---|
| Lift through 160 cm | `Src` 1 → 0 when `ToF` reaches ~160 cm |
| Lower through 140 cm | `Src` 0 → 1 when `ToF` falls below ~140 cm, after 3 samples |
| Quick hand passes | `Src` stays 1 ( the hand is a valid short range; `EstAlt` may follow it, which is task 6 ) |
| Laser dropout ( `ToF` −1 ) | `Src` stays 1 for single −1 samples, switches to 0 only after ~120 ms of them, and returns below 160 cm |

**Limit of this bench.** While disarmed `baroUpdateZero()` keeps pulling the baro altitude to 0 and
`AltHold` follows `EstAlt`, so `EstAlt` on the baro path and the setpoint shift mean nothing here.
Those are checked in flight in task 8.

**Results ( task 5 ).** 22 Sep 2026, 127 s, 1210 records, disarmed throughout. Three lifts to
~190-200 cm, then hand passes at ~115 cm.

| Switch | `ToF` around it | Verdict |
|---|---|---|
| 38.4 s laser → baro | 161.2 → 175.1 cm | at the 160 cm edge |
| 53.8 s baro → laser | 141.2 → 135.5 cm | at the 140 cm edge |
| 63.7 s laser → baro | 156.8 → −1 ( crossed 160 between 10 Hz log samples, then dropped out ) | as designed |
| 73.9 s baro → laser | 138.7 → 136.0 cm | at the 140 cm edge |
| 86.6 s laser → baro | 151.1 → 167.6 cm ( crossed 160 between log samples ) | at the 160 cm edge |
| 93.6 s baro → laser | 140.3 → 136.1 cm | at the 140 cm edge |

- Single `ToF` −1 samples while on the laser ( e.g. 36.5 s at ~130 cm, 38.1 s at ~140 cm ) did not
  switch the source.
- Hand passes at ~115 cm ( 99-114 s ): `Src` stayed 1; `EstAlt` followed the hand ( task 6 ).
- The laser read up to 199.9 cm but dropped out repeatedly above ~175 cm, which confirms 160 cm as
  the upper edge on this floor.
- `EstAlt` jumped 22-42 cm at the returns: the disarmed baro zero tracking, the known limit of this
  bench, not a flight result.

**Pass** for everything this bench can show. `EstAlt` continuity and the setpoint shift: task 8.


## Test 5: handover in flight ( `logs/log-5.txt`, task 8 )

**Build.** Final task 6 firmware, `Build/PRIMUS_X2_v1/DEFAULT_PRIMUS_X2_v1_3.8.1.hex` from the task 7
gate. Log `BaroAlt ToF EstAlt AltHold Vz Src AccB Arm`, ~120 B/tick ( `Src` 1 laser, 0 baro, 2 object
hold-off ). Indoor, ceiling ≥ 2.5 m, same floor as before.

**Sequence.**

1. Power on, wait a few seconds, Developer Mode on, start logging. **20 s on the floor.**
2. Arm, take off in ALT_HOLD to ~120 cm, hover **15 s**.
3. **Stick up slowly** to ~190-200 cm ( past 160 cm, where the laser hands over ), centre, hover
   **15 s** on the baro.
4. **Stick down slowly** to ~100 cm ( past 140 cm, where the laser takes back ), centre, hover **15 s**.
5. Repeat 3-4 once.
6. Land, disarm. **Within ~5 s re-arm**, take off to ~100 cm, hover 10 s, land ( quick re-arm check
   for the touchdown bias wind-up ).
7. 10 s on the floor.

**Note** anything visible at the two switch points: a bump, a sink, a climb.

**Pass.**

| Check | Expected |
|---|---|
| Laser → baro on the climb | `Src` 1 → 0 near 160 cm by `ToF`; `EstAlt` continuous ( no jump beyond the motion within a sample ) |
| Hover on the baro | no runaway; the estimate may drift a few cm |
| Baro → laser on the descent | `Src` 0 → 1 near 140 cm; `EstAlt` and `AltHold` jump by the same amount in the same sample, `ToF` shows no bump in the real height |
| Quick re-arm | no climb or sink at take-off beyond a normal take-off |

**Results ( task 8 ).** 22 Sep 2026, 198 s, 1630 records. Two Wi-Fi gaps, before arming
( 17.8 s ) and in a low hover ( 3.0 s ), do not touch the switches. The pilot climbed to ~245, ~270
and ~310 cm by the baro estimate.

| Switch | Log | Verdict |
|---|---|---|
| 53.8 s laser → baro, climbing 20 cm/s | `ToF` 160 → 162, `EstAlt` 159 → 163 | continuous ( the change is the climb ) |
| 116.7 s laser → baro by dropout, climbing 38 cm/s | `EstAlt` 153 → 158 over 0.5 s | continuous |
| 84.5 s baro → laser, descending 30 cm/s | `EstAlt` 128 → 128, `AltHold` 94 → 93 | shift about 0: the lag-advanced laser matched the baro estimate; the apparent 13 cm gap before it was IIR lag |
| 144.1 s baro → laser, descending 33 cm/s | `EstAlt` 134 → 126 and `AltHold` 91 → 82 in the same sample | shifted together, as designed |

- **Baro-only hover** above the laser range: `EstAlt` within about ±4 cm.
- **Quick re-arm** 6 s after landing, `AccB` wound up to 20 cm/s²: take-off overshoot +14 cm, dip
  −18 cm, settled in ~7 s. No runaway. **Pass.**
- **One stray step trigger** at 146.5 s ( 65 cm hover ): `Src` 1 → 2 → 1 within ~0.1 s, a single
  outlier raw sample, cancelled; no height change.

**Pass.**


## Test 6: object under the craft ( `logs/log-6.txt`, task 8 )

**Keep the hover height plus the box height below ~150 cm.** After the drone climbs over the box it
flies at its old clearance above the box; if pulling the box out makes the laser read ≥ 160 cm, the
handover takes over and the drone stays up on the baro instead of stepping down ( known limit ).

**Sequence.** Box or stack of books 25-35 cm tall, held still.

1. Power on, Developer Mode on, start logging. **20 s on the floor.**
2. Arm, take off in ALT_HOLD, bring the hover to **~80 cm**, centre, hover 10 s.
3. **Slide the box under** the drone and leave it **8 s**. Pull it out and wait **8 s**.
4. Repeat 3 once.
5. **Quick pass:** move the box under and out again within **~1 s**, twice, 8 s apart.
6. Land, disarm, 10 s on the floor.

**Note** where the box was placed each time and how the drone moved.

**Pass.**

| Check | Expected |
|---|---|
| Box in | `Src` 1 → 2 on the first sample; no climb for ≥ 2.5 s ( `ToF` shows the box, `EstAlt` stays near its old value ) |
| After ≥ 2.5 s and 0.5 s steady | `Src` 2 → 1; `EstAlt` drops by the box height while `AltHold` ramps it back up at ≤ 60 cm/s; the drone climbs smoothly to ~80 cm above the box |
| Box out | `Src` 1 → 2; no drop for ≥ 2.5 s; then a ramped descent at ≤ 30 cm/s to ~80 cm above the floor |
| Quick pass | `Src` 1 → 2 → 1 ( cancel ), no height change |

**Results ( task 8 ).** 22 Sep 2026, 176 s, 1675 records. Box ~45-53 cm tall ( from the
re-base shifts ), hover 92 cm, then 68 cm.

| Hold-off start | Hold | Outcome | `EstAlt` moved during hold | Peak `Vz` in the next 4 s |
|---|---|---|---|---|
| 31.6 s box in | 2.5 s | re-base −48 cm | 5 cm | +52 cm/s |
| 39.5 s box out | 3.0 s | re-base +43 cm | 7 cm | −34 cm/s |
| 49.2 s box in | 2.5 s | re-base −53 cm | 4 cm | +56 cm/s |
| **~60 s box out** | **none** | **missed** | follows the laser | - |
| 70.9 s box in | 2.4 s | re-base −51 cm | 4 cm | +61 cm/s |
| 84.9 s box out | 3.4 s | re-base +54 cm | 7 cm | −37 cm/s |
| 97.4 s box in | 2.5 s | re-base −51 cm | 4 cm | +58 cm/s |
| 102.4 s box out | 1.5 s | cancel ( laser back at box level ) | 5 cm | - |
| 108.9 s box out | 2.5 s | re-base +44 cm | 5 cm | −29 cm/s |
| 119.4 s box in | 2.6 s | re-base −46 cm | 5 cm | +55 cm/s |
| 128.8-137.6 s quick passes | 0.3-1.6 s | 4 cancels | 3-16 cm | - |
| 146.7-151.4 s box in, wobbling | 0.6-2.6 s | 2 cancels, then re-base −43 cm | 4-5 cm | +55 cm/s |
| 161.6 s box out | 2.6 s | re-base +50 cm | 8 cm | −34 cm/s |

- **Works as specified** for a box put in or pulled out briskly: no climb or drop during the
  hold-off ( 4-8 cm ), then a ramped climb at ~55 cm/s or descent at ~30-37 cm/s.
- **Fails for a slow slide ( ~60 s ).** The laser rose gradually ( 95 → 113 cm over 0.65 s, then
  +11 cm ) as the box edge left the wide VL53L0X cone. No single raw sample was 30 cm from the
  estimate, because the estimate followed it, so no hold-off started. `AltHold` 92 cm was then
  counted from the floor instead of the box top, and the craft **dropped by the box height at
  once**, the laser bottoming at 56 cm ( 36 cm below the setpoint ). This is the originally
  reported symptom.
- **Edge contamination ( 127-131 s ).** Before the quick passes the estimate followed a gradual
  ~16 cm edge rise and the baro-offset average learned it; during the hold-offs the baro path,
  using that offset, dipped `EstAlt` ~25 cm, and one cancel matched a raw reading against the
  drifted estimate.

**Partial pass.** Follow-up: task 14.


## Test 7: object detector re-test with slow slides ( `logs/log-7.txt`, task 14 )

**Build.** Task 14 firmware ( window test with suspect coasting, 1 s take-off grace, 20 cm
edge/slope line ), `Build/PRIMUS_X2_v1/DEFAULT_PRIMUS_X2_v1_3.8.1.hex`, same log fields ( `Src` 2 = hold-off ). Same box
( ~45-50 cm ), same floor. Keep hover + box below ~150 cm.

**Sequence.**

1. Power on, Developer Mode on, start logging. **20 s on the floor.**
2. Arm, take off in ALT_HOLD, bring the hover to **~80 cm**, centre, **hover 15 s** without touching
   anything ( false-trigger check ).
3. **Brisk:** push the box under in one quick move, leave **8 s**, pull it out quickly, wait **8 s**.
4. **Slow slide:** slide the box under over **about 1 s**, leave **8 s**; slide it out over
   **about 1 s**, wait **8 s**. Do this **twice**.
5. **Moving edge:** with the box on the floor, fly slowly sideways ( ~0.5 m/s ) so the drone passes
   over the box and off the other side, then back. Twice.
6. Land, disarm, 10 s on the floor.

**Note** which move was which and roughly when.

**Pass.**

| Check | Expected |
|---|---|
| Hover, take-off, landing | `Src` never 2 without a box move; in particular none in the first seconds after lift-off |
| Brisk and slow moves ( in and out ) | `Src` 2 on every one; `EstAlt` within ~10 cm during each hold-off; no drop or climb by the box height |
| After the hold-off | ramped climb over the box / descent after it, as in test 6 |
| Moving edge | `Src` 2 at each edge, or, if an edge is soft enough to be followed, no height error beyond ~15 cm |

**Results ( task 14 ).** 22 Sep 2026, 178 s, 1655 records. Wi-Fi gaps of 0.7, 0.8 and 3.5 s
( the last at 131-135 s hides one box-out re-base ). Hover moved from 120 to ~76 cm by stick at 47 s.

| Hold-off start | Hold | Outcome | `EstAlt` moved during hold | Next 4 s |
|---|---|---|---|---|
| 53.3 s box in | 3.0 s | re-base −49 cm | 9 cm | climb, peak +56 cm/s |
| 68.6 s box out | 2.6 s | re-base +47 cm | 13 cm | descent, peak −33 cm/s |
| 77.0 s box in, pulled out early | 1.8 s | cancel | 3 cm | - |
| 83.3 s box in | 2.5 s | re-base −49 cm | 6 cm | climb +55 cm/s |
| 95.7 s box out, slow | 2.4 s | re-base +16 cm ( reading steady half-way ) | 11 cm | descent −29 cm/s |
| 99.8-108.7 s rest of that slide | 0.3-1.6 s | 3 cancels | 2-7 cm | - |
| 109.4 s box out | 2.7 s | re-base +44 cm | 6 cm | descent −29 cm/s |
| 114.3 s box in | 2.6 s | re-base −46 cm | 11 cm | climb +59 cm/s |
| 122.2 s box out | 2.5 s | re-base +45 cm | 8 cm | descent −35 cm/s |
| 125.0 s box back in during that descent | 1.3 s | cancel | ( commanded descent ) | - |
| 126.8 s box in | 2.5 s | re-base −50 cm | 16 cm | climb +56 cm/s |
| 138.9-146.3 s moving edge, over the box and back | many | cancels | `EstAlt` 65-79 cm, `AltHold` 73-76 | no height change |
| 148.9 s and 152.5 s box **held up under the drone** | 2.4-2.5 s | re-base −72 cm, then −65 cm | 5 cm | climbed twice, ~137 cm in all, to ~2 m; baro above 156 s, back on the laser at 169 s |

- **No box move was missed.** Nothing like `log-6` ~60 s, where the craft dropped by the box height
  with no hold-off. Slides, including a slow slide-out ( 95.7-112 s ), all started a hold-off.
- **Take-off and landing:** `Src` stayed 1 for the first 5 s after arming and through the landing.
- **Stray triggers:** single samples at 21.5, 29.6 and 46.8 s started a hold-off that cancelled
  within 0.1-1.6 s with no height change.
- **A slow slide can re-base on a half-way reading** ( 95.7 s, +16 cm ) when the reading stays steady
  for 0.5 s part-way; later triggers correct it.
- **Held-up box:** the craft climbed over it each time it was re-based on, as the pilot chose ( no
  climb cap ). An object that follows the craft can drive it up repeatedly.

**Pass** ( holds 6-13 cm, once 16 cm right after a commanded descent ).

