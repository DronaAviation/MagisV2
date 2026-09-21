# Flip / Altitude Hold Regression - Testing

[README](README.md) · [TASKS](TASKS.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

## Test 1: baseline, current firmware without the fix (task 2)

| | |
|---|---|
| **Date** | 21 Sep 2026 |
| **Build** | FW 3.8.0 at `b91e6a7` + task 1 diagnostics + `DEV_MODE_ALWAYS_ON` |
| **Target** | PRIMUS_X2_v1 |
| **Log** | `log-1.txt` (1145 records, 54.3 s, 25 Hz; armed 8-52 s) |
| **Procedure** | Hover in ALT_HOLD, press the app back-flip button, twice |
| **Expected if the hypothesis holds** | `Fs` 1 for ~2.2 s with `Vz` levelling off near 40 and never reaching 100, `Pit` not rotating, `Baro` 1 throughout, then `Fs` back to 0 |

### Results

| Attempt | `Fs` = 1 (ASCEND) | Duration | `Vt` setpoint | Peak `Vz` | `Pit` range | `Baro` | Altitude `EA` | Outcome |
|---|---|---|---|---|---|---|---|---|
| 1 | 22.57 → 24.81 s | 2.2 s | ramps 9 → 40 in 0.3 s, then pinned at 40 | **34 cm/s** | −1.2 … +2.0° | 1 | 38 → 99 cm (+61) | timeout, `Fs` → 0, no rotation |
| 2 | 43.22 → 45.40 s | 2.2 s | ramps 0 → 40 in 0.35 s, then pinned at 40 | **39 cm/s** | −0.7 … +2.4° | 1 | 42 → 108 cm (+66) | timeout, `Fs` → 0, no rotation |

- `Thr` is 2000 for the whole of ASCEND. The flip's throttle write reaches altitude
  hold, as the review predicted.
- `Rt` (profile rate) climbs at 0.4 cm/s per tick to `ALT_MAX_CLIMB_CMS` = 40, and `Vt`
  (the slewed setpoint) is pinned at 40. `Vz` settles at 32-39 cm/s, never close to the
  ASCEND threshold of `desiredVelocity` = 100.
- `Baro` reads 1 throughout. The flip's `DEACTIVATE_RC_MODE(BOXBARO)` has no lasting
  effect while the app holds AUX3.
- After the timeout the pilot throttle is back at 1500. The profile rate ramps down and
  the drone coasts to a stop 70-90 cm above its starting height: `AH` 47 → 135 cm in
  attempt 1, 42 → 130 cm in attempt 2. A failed flip therefore leaves the drone
  hovering almost a metre higher.

### Conclusion (test 1)

**Root cause confirmed.** On both attempts the shaped stick path caps the climb at
40 cm/s, ASCEND never sees VelocityZ ≥ 100 cm/s, and the flip times out after 2.2 s
without starting the rotation. This matches the hypothesis in
[INVESTIGATION.md](INVESTIGATION.md) and supports the fix planned in task 3.

## Test 2: after task 3 (flip bypass, no hand-back fix)

| | |
|---|---|
| **Date** | 21 Sep 2026 |
| **Build** | Task 3 + task 1 diagnostics + `DEV_MODE_ALWAYS_ON` |
| **Target** | PRIMUS_X2_v1 |
| **Log** | `log-2.txt` (777 records, armed 7-26 s) |
| **Pilot report** | Flipped, then "shot up like a flyaway" after the flip |

### Timeline (one flip)

| t (s) | `Fs` | What happens |
|---|---|---|
| 23.20 | 1 ASCEND | `Vt` = 120 raw (bypass works), `EA` ≈ 30 cm. `Vz` reaches 100 cm/s in **0.69 s** |
| 23.89 | 2 PITCHING | Rotation: `Pit` −1 → −842 → +729 deci-deg in 0.4 s. `EA` peaks at 92 cm |
| 24.34 | 4 HOLD | Throttle 2000, `Vt` = 120, but the craft is falling after the rotation: `Vz` down to **−160 cm/s**. `EA` 79 → **−14 cm** (at or below the arming height) at 24.9 s, then climbs back |
| 25.83 | 0 exit | Exit with `Vz` = +69 cm/s, `EA` 94, `AH` frozen at 88. **`Vt` = 115, slewing down only ~6 cm/s per 40 ms tick (150 cm/s²)** |
| 25.83-26.08 | 0 | **Flyaway:** `Vz` *rises* 69 → 92 cm/s, `EA` 94 → 143 cm in 0.25 s, while `Vt` is still above `Vz` (115 → 84) |
| 26.09 | 0 | `Arm` → 0 (disarmed). The craft tumbles (`Pit` to +72°) and falls to the ground |

### Diagnosis

Two effects add up at the hand-back from the flip to the shaped controller:

1. **Stale velocity setpoint** (the task 3 review, finding 1). `flipVelocitySetpoint()`
   leaves `altVelTarget` = 120 at exit. `shapedVelocitySetpoint()` clamps `velDemand`
   (position error −6 cm → about −5 cm/s) but slews `altVelTarget` down at 150 cm/s²,
   so it takes ~0.8 s to come below `Vz`. During that time the velocity loop keeps
   commanding more climb, not less.
2. **Integrator wind-up during HOLD.** For 1.5 s HOLD demands +120 while the craft
   falls at up to −160, a velocity error of up to +280 cm/s. `errorVelocityI` integrates
   that towards its +300-count clamp, so there is a large extra throttle offset at exit.
   (`errorVelocityI` is not logged. This is inferred from the error history and the
   climb acceleration after exit.)

Before `2a8d59a`, the first deadband tick after the flip snapped `AltHold = EstAlt` and
used the P-only position loop, so `setVel` ≈ 0 and the velocity error was −`Vz` ≈ −70
from the first tick. That immediately fought the wound-up integrator. The shaped
hand-back instead keeps asking for more climb for most of a second, which is the
flyaway.

Also seen: the flip started at only ~30 cm, and the drop after the rotation took the
estimate to −14 cm, so the craft was at or near the ground during HOLD.

## Test 3: after task 4 (flip hand-back)

| | |
|---|---|
| **Date** | 21 Sep 2026 |
| **Build** | Tasks 3 + 4 + diagnostics (`VI` field added) + `DEV_MODE_ALWAYS_ON` |
| **Target** | PRIMUS_X2_v1 |
| **Log** | `log-3.txt` (1584 records, armed 10-65 s, 3 flips) |
| **Pilot report** | Flips worked and were stable, but the drone did not return to the height where the flip was pressed |

### Results

| Flip | Pressed at `EA` / `AH` | ASCEND to 100 cm/s | Lowest `Vz` in HOLD | Exit `AH` (held) | Overshoot after exit | Offset from pre-flip `AH` |
|---|---|---|---|---|---|---|
| 1 (14.7 s) | 101 / 121 cm | 0.38 s | −139 cm/s | 149 cm | `EA` to 167, dips to 97, climbs back | +28 cm |
| 2 (35.4 s) | 60 / 57 cm | 0.34 s | −135 cm/s | 125 cm | `EA` to 145, dips to 73, climbs back | +68 cm |
| 3 (51.8 s) | 128 / 125 cm | 0.36 s | −139 cm/s | 188 cm | `EA` to 208, dips to 141, climbs back | +63 cm |

- **Task 4 works:** no flyaway. At exit `Vt` goes to 0 → −20 and `VI` is restored to its
  pre-flip 0 (from ~60-70 counts of wind-up in HOLD).
- **Why the height is higher:** HOLD runs throttle 2000 (`Vt` 120) for a fixed 1.5 s. The
  craft falls out of the rotation (to −139 cm/s), then climbs at 120-140 cm/s and ends the
  flip 50-65 cm above its start. The hand-back then holds `AltHold = EstAlt` at exit.
- **After exit:** the craft coasts ~20 cm up, drops 40-70 cm below `AH`, then climbs back
  at the shaped 40 cm/s limit over ~2.5 s.

### Conclusion (test 3)

The hand-back is safe. Returning to the pre-flip height is a new requirement: task 10.

## Test 4: task 4 build again, flip heights (log-4.txt)

| | |
|---|---|
| **Date** | 21 Sep 2026 |
| **Build** | Same logic as test 3 (tasks 3 + 4). Task 10 is not implemented yet: both `Build/PRIMUS_X2_v1/*.hex` predate `log-3.txt` |
| **Log** | `log-4.txt` (1961 records, armed 9.2-83.8 s, 4 flips) |
| **Question** | At what height was the flip sent, and what was the height afterwards? |

| Flip | Pressed (t) | Height at press `EA` / target `AH` | Lowest during flip | Height at flip end (new `AH`) | Settled height | Change vs press target |
|---|---|---|---|---|---|---|
| 1 | 20.0 s | 33 / 32 cm | **2 cm** | 154 cm | ~147 cm, pilot on the throttle stick 22.4-30.4 s | **+122 cm** at flip end |
| 2 | 34.6 s | 67 / 67 cm | 28 cm | 128 cm | 132 cm | **+61 cm** |
| 3 | 45.4 s | 136 / 128 cm | 90 cm | 180 cm | 187 cm | **+52 cm** |
| 4 | 66.4 s | 65 / 65 cm | **13 cm** | 105 cm | 105 cm | **+40 cm** (1.6 s log gap during HOLD) |

- Every flip ends 40-122 cm above the target it started from, and the drone then holds
  there. This is the expected task 4 behaviour, which task 10 addresses.
- Flips 1 and 4 were started at 33 and 65 cm and dropped to 2 and 13 cm during the
  recovery, close to the floor. None were flown from the agreed ≥ 1.5 m (the highest was
  136 cm).
- Settled height = mean `EA` from 4 s after the flip ends until the next event.

## Test 5: task 10 build, validation flight (task 6)

| | |
|---|---|
| **Date** | 21 Sep 2026 |
| **Build** | Tasks 3 + 4 + 10 + diagnostics (hex 13:47, newer than all sources; log 14:09) |
| **Target** | PRIMUS_X2_v1 |
| **Log** | `log-5.txt` (1777 records, armed 9.2-75.8 s, 4 flips) |

### Flips

| Flip | Pressed at `EA` / `AH` | ASCEND | Flip end `EA` | Target back at pre-flip `AH` | Dip below target after exit | Settled `EA` (before the next stick input) |
|---|---|---|---|---|---|---|
| 1 (16.2 s) | 126 / 138 cm | ok | 178 cm | 138 at +1.7 s | **92 cm (−46)** | 137-148 (+0…+10) |
| 2 (31.6 s) | 57 / 55 cm | ok | 121 cm | 55 at +2.6 s | **37 cm (−18)** | 58-64 (+3…+9) |
| 3 (48.1 s) | 118 / 118 cm | ok | 186 cm | 118 at +2.5 s | **103 cm (−15)** | 124-130 (+6…+12) |
| 4 (60.9 s) | 190 / 193 cm | ok | 246 cm | 193 at +2.1 s | **168 cm (−25)** | 194-207 (+1…+14), 194-198 after 8 s |

- All 4 flips completed the rotation and recovered. ASCEND reached 100 cm/s each time.
- **Return works:** in every flip `AH` travels back to exactly the pre-flip target on the
  goal profile, with no pilot input needed. (The throttle sample flagged at exit is the
  flip's own 2000. The pilot's real stick inputs came 5-11 s later.)
- **Dip after exit:** the flip ends climbing at 116-127 cm/s (HOLD is still at full
  throttle). The hand-back brakes that climb (`Vt` −30), the craft coasts 20-30 cm up,
  `VI` swings to −17…−21 counts, and the craft then drops at up to −59 cm/s, 15-46 cm
  *below* the returning target, before recovering ~1 s later. On flip 2 (target 55 cm)
  it went down to 37 cm.
- **Offset while settling:** after the return, `Vz` reads −11…−18 cm/s for 3-8 s while
  `EA` is steady, and `Vt` follows it (−10). The velocity estimate is biased after the
  rotation, so `EA` sits 6-14 cm above `AH` until the bias decays. This is inferred from
  `Vz` vs the `EA` slope.

### Conclusion (test 5)

Task 6 **Done when** met: every flip completed and recovered, VelocityZ ≥ 100 cm/s in
ASCEND, `flipState` ran through to 0, and the drone returned to and held the pre-flip
height, within about +10 cm, briefly +14 cm on flip 4. Possible improvements: the
post-exit dip below target, and the post-flip velocity-estimate bias.

## Test 6: task 11 build (post-flip integrator hold)

| | |
|---|---|
| **Date** | 21 Sep 2026 |
| **Build** | Tasks 3 + 4 + 10 + 11 + diagnostics (hex 14:21, newer than all sources; log 14:36) |
| **Target** | PRIMUS_X2_v1 |
| **Log** | `logs-6.txt` (1476 records, armed 11.7-62.2 s, 3 flips) |

### Comparison with test 5 (same metrics, window cut at the next pilot stick input)

| | Test 5 (`log-5.txt`, 4 flips) | Test 6 (`logs-6.txt`, 3 flips) |
|---|---|---|
| `VI` in the first 0.5 s after exit | −21…−17 (min per flip) | **0** on every flip |
| Fastest fall after the braking (`Vz`) | −61…−73 cm/s | **−40…−51 cm/s** |
| Dip below the pre-flip target (`EA − AH`) | −15…−47 cm | −16…−42 cm (−42, −32, −16) |
| Target back at the pre-flip `AH` | 1.6-2.4 s | 0.4-1.7 s |
| Settled, from 4 s after exit | +0…+15 cm | −4…+10 cm |

### What the dip is now

- **The integrator is out of it:** `VI` stays at 0 through the braking, and the drone
  falls more slowly after it (−40…−51 instead of −61…−73 cm/s).
- **The height estimate falls faster than the estimated speed allows.** Flip 1: `EA`
  drops 136 → 76 cm (60 cm) in ~0.7 s while `Vz` peaks at −40 cm/s, which accounts for
  ~28 cm. `EA` then sits at 76-77 for 0.5 s while `Vz` reads +17. Flip 3: `EA` drops
  96 cm in 1.4 s while `∫Vz` ≈ 53 cm. During these falls `Vt` is 0…−4 (flip 1), and
  after the integrator hold `VI` is still near 0. So the controller is not commanding a
  descent; the altitude estimate is re-converging.
- The likely cause is in the estimator: baro and accelerometer disagree right after the
  rotation. One candidate is the baro throttle compensation, 0.0086 Pa/count × the
  throttle step from HOLD's full throttle back to hover, roughly 3 Pa ≈ 25 cm, removed at
  exit. The other is the same post-rotation accel bias seen in test 5 (`Vz` ≈ −13 cm/s
  while level). `rcCommand[THROTTLE]` is not logged, so neither is confirmed.
- Whether the drone physically dips this much cannot be told from the baro estimate. A
  ground-truth run with `LASER_TOF` (ToF logged, not in the loop) would show it.

### Conclusion (test 6)

Task 11's code works as intended: the integrator is held and the fall after braking is
slower. The remaining 16-42 cm dip in `EA` is dominated by the altitude estimate
re-converging after the flip, not by the controller, so the "dip under ~10 cm"
criterion is not met in the estimate. This belongs with the accepted post-flip estimator
bias (test 5).
