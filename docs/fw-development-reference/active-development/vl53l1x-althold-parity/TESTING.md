# VL53L1X Altitude Hold Parity - Testing

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [TESTING](TESTING.md)

## Object-code baseline ( task 1, 23 Sep 2026 )

The VL53L0X board can no longer be flown, so the proof that the refactor ( tasks 5 and 6 ) leaves the
flight-validated L0X path unchanged is its object code: the disassembly of `altitudehold.o` for the
L0X build must stay byte-identical.

**Method.** [baseline/snapshot.sh](baseline/snapshot.sh) `<L0X|L1X|NOLASER> <name>` takes a copy of
the PRIMUS_X2_v1 `target.h`, enables that configuration's defines, and clean-builds with the
pluto-build driver. It then writes `objdump -d -r -s` of `altitudehold.o`, with the debug sections and
`.comment` stripped, to `<name>.dis`, writes the hashes and memory to `<name>.txt`, and restores
`target.h` whatever the build result. From Git Bash:

```bash
cd docs/fw-development-reference/active-development/vl53l1x-althold-parity/baseline
bash snapshot.sh L0X after-task5
diff altitudehold-L0X.dis after-task5.dis    # empty = identical
```

**Baselines at `b1f070b`** ( PRIMUS_X2_v1, Arm GNU Toolchain 14.2.1 ):

| Configuration | Defines | `altitudehold.o` dump sha256 | Flash | RAM |
|---|---|---|---|---|
| L0X | `LASER_TOF` + `LASER_ALT` | `35812d92…aac04f` | 101.7 KB | 15.5 KB |
| No laser ( shipped ) | none | `a8775323…0e3a0` | 98.9 KB | 14.8 KB |
| L1X | `LASER_TOF_L1x` + `LASER_ALT` | `50af04b3…66ae` | 109.5 KB | 15.8 KB |

**Reproducibility.** The L0X configuration was clean-built three times, and the dump was identical
every time. The comparison is therefore not disturbed by paths or timestamps.

**Why not the `.hex`.** The whole image changes on every build, because the Makefile compiles
`__BUILD_DATE__` / `__BUILD_TIME__` into it ( [Makefile:47-48](../../../../Makefile#L47-L48) ). The `.hex`
hash is recorded for information only.

**Limit of the proof.** The dump covers `altitudehold.o` alone: code, literal pools, `.rodata`, `.data`
and relocations. Tasks 5 and 6 change only `altitudehold.cpp`. If they touch any other file that the L0X
build compiles, that file needs its own baseline first.

## Test 1: VL53L1X bring-up and freeze check ( task 2, `logs/log-1.txt` )

**Question.** Does the VL53L1X driver keep producing new measurements, or does it freeze after the
first result because `getRange_L1 ( )` never calls `VL53L1_ClearInterruptAndStartMeasurement ( )`
( hypothesis H1, [INVESTIGATION.md](INVESTIGATION.md) )? Does the rest of I2C1 ( barometer ) still
work with the L1x on the bus?

**Build.** Working tree at `b1f070b`, PRIMUS_X2_v1, `LASER_TOF_L1x` on, **`LASER_ALT` off**, so the
laser is logged but never used by altitude hold. Temporary log in `PlutoPilot.cpp`. Flash 109.8 KB,
RAM 15.8 KB.

**Log fields** ( one record per `plutoLoop ( )` tick, 10 Hz, about 97 bytes per tick, 109 worst case, against the 115 B cap ):

| Field | Meaning | Healthy | Frozen ( H1 real ) | Never read ( init / data-ready problem ) |
|---|---|---|---|---|
| `mm` | Raw range of the last result read | Follows the hand | Stuck on one value | 0 |
| `St` | RangeStatus of that result ( 0 = valid ) | 0 over the floor | - | 0 ( zero-filled, not data ) |
| `R` | 1 if the driver read a result since the last tick | 1 | **1** ( it keeps re-reading the same result ) | **0** |
| `SC` | StreamCount, bumped by the sensor on every new measurement ( wraps 255 → 128 ) | Counts up | Stuck | 0 |
| `d` | New measurements since the previous tick | 0-2 ( beats against the 100 ms tick ) | 0 | 0 |
| `A` | ms since `SC` last changed ( quantised to the 100 ms tick ) | Under about 250 | Climbs into the thousands | Climbs |
| `G` | `Global_Status_L1` ( 0 = ok ) | 0 | 0, or non-zero if the sync check latches | 0 |
| `O` | `isOutofRange_L1 ( )` | 0 over the floor, 1 past the reach | - | 0 |
| `B` | `BaroAlt`, cm | Changes when lifted | - | - |

**Confirms H1:** `R` = 1 while `SC` stays put ( `A` climbing ), or `G` goes non-zero. **Kills H1:** `SC`
advances and `mm` follows the hand for the whole 60 s of step 8. **Neither:** `R` stays 0, meaning the
driver never gets a data-ready flag. That points task 3 at init / start, not at the handshake.

A row with `G` ≠ 0 shows `SC` 255, `mm` -1 and `St` 255. That is the ST API's 0xFF fill after a failed
read, not data.

**Sample interval.** `A` is quantised to the 100 ms `plutoLoop` tick, so the interval is measured as
the elapsed time divided by the sum of `d` over a still window of at least 10 s ( steps 3 and 8 ).

**Setup.** VL53L1X on the laser connector ( I2C1 ), facing down, L0X removed. **Propellers off**:
the craft is never armed in this test. Tape measure. Indoor, the same matt floor as the L0X flights.

**Sequence** ( about 3 minutes; the steps are separated by at least 5 s of stillness so they are easy
to find in the log ):

1. Flash the build from PlutoIDE ( PRIMUS_X2_v1 ). Remove **all** power ( battery and USB ), wait 5 s,
   then power up from the battery. The LEDs must come up as usual. A board with no LEDs and no app
   connection means the sensor init is blocking, which points to wiring or power: stop and report it.
2. Connect the app, switch **Developer Mode on** ( `plutoLoop` only runs then, with a live RC link ),
   start logging in PlutoMonitor. Check that lines appear.
3. **Still at a known height, 30 s.** Rest the craft level on a box or stand, with the laser seeing
   the floor. Measure laser-to-floor with the tape and write it down.
4. **Slow lift, about 30 s.** Pick it up level, raise it slowly from about 5 cm to about 150 cm over
   the floor, and lower it back. Keep the laser pointing straight down.
5. **Past the reach, 10 s.** Point the laser at something more than 4 m away ( down a corridor, or
   across the room at a far wall ), or up at a high ceiling. Then back over the floor.
6. **Covered, 5 s.** Cover the laser window with a finger or a dark cloth, then uncover it.
7. **Tilt, 10 s.** Back over the floor at about 50 cm, tilt about 30° to one side and hold, then level.
8. **Still again, 60 s.** Put it back on the box from step 3 and do not touch it.
9. Stop logging, paste the whole log into `logs/log-1.txt` and save it. Note the tape height from
   step 3, the height you reached in step 4, and anything odd ( LEDs, the app dropping, a value that
   stuck ).

**Stop early** if in step 3 `SC` does not change for 5 s ( note whether `R` is 1 or 0 ) or `G` is not 0. That already answers the
question: save the log as it is and report it.

### Test 1 result ( `logs/log-1.txt`, 23 Sep 2026 15:10-15:13 )

**Build as above. 1576 records over 178.5 s. Complete records: 1574 of 1576.** No log-line overrun.

**Firmware tick is steady.** While `SC` was stuck ( 0-41 s ), `A` is a firmware clock. Tick to tick it
steps by 100-107 ms, and 41.4 s of PC time equals 41.4 s of firmware time. The 16 gaps of 0.3-2.75 s in
the PlutoMonitor timestamps are delivery bursts over the link, not loop stalls. About 6 records were
lost in transit ( `A` steps of 204-207 ms ).

**H1 as stated ( driver freezes ) is killed.**

- `mm` changes on 1458 of 1575 ticks and follows the motion ( 28 mm → 1.9 m → 870 mm → 3 m → 0 mm →
  700 mm → 1080 mm → 30 mm ).
- At rest the readings are fresh noise, not a re-read value: 0-35 s mean 29 mm, sd 2.1 mm, only 46 of
  311 ticks equal to the previous one.
- `G` = 0 for all 178 s. The sync check never latched.

**But the driver re-reads every 10 ms.** `R` = 1 on every tick. Without
`VL53L1_ClearInterruptAndStartMeasurement ( )` the data-ready flag never clears, so `getRange_L1 ( )`
reads the result block on every 10 ms poll. That is up to 10 I2C result reads per 100 ms measurement,
and `isTofDataNewflag_L1` is set on every read. The estimator's "new sample" gate therefore sees about
100 Hz of mostly duplicate samples. The handshake is still needed ( task 3 ), to get one sample per
measurement rather than to stop a freeze.

**`StreamCount` is not a sample counter here.** It only toggles 0 ↔ 1, and only around range-status
changes. `SC`, `d` and `A` therefore cannot measure the sample interval, so this log does not give
one. Task 3 needs a driver-side count of data-ready events.

**Out of range is flagged by the driver.** `O` = 1 on 375 ticks, with `St` 2 ( signal fail ) or 4
( out of bounds ), when pointed far away ( 85-120 s ). Meanwhile `NewSensorRange_L1` keeps its last
valid value. That is the input to bug 1 ( the stale estimator correction ), now seen on hardware.

**Intermittent signal fail at about 1.9 m.** At 40-60 s, readings of about 1.87-1.92 m alternate
between `St` 0 and `St` 2. If that was a real surface, the usable reach on this floor is below 2 m. To be
confirmed in task 4.

**Covered window reads as valid.** 9 samples of 0-5 mm carry `St` 0. The estimator needs a
minimum-valid range, or a covered or blocked sensor becomes "0 cm above the floor".

**Noise.** 29 mm: sd 2.1 mm. 873 mm ( still, 65-80 s ): sd 13.2 mm. 700 mm ( hand-held, tilt step ):
sd 35.7 mm.

**Barometer on the shared bus is live.** `B` varies by ±10 cm and never goes stale. It cannot show the
lift, because the baro datum tracks while disarmed.

## Test 2: driver fix check ( task 3, `logs/log-2.txt` )

**Question.** After the task 3 fix, does the VL53L1X deliver exactly one new sample every 50 ms, keep
ranging for a full minute, and flag out of range within one period?

**Build.** Working tree with the task 3 driver, PRIMUS_X2_v1, `LASER_TOF_L1x` on, **`LASER_ALT`
off**. 45 ms timing budget, 50 ms period. Flash 110.3 KB, RAM 15.8 KB.

**Log fields** ( 10 Hz, about 87 bytes per tick, 101 worst case ):

| Field | Meaning | Expected |
|---|---|---|
| `mm` | Raw range of the last result | Follows the hand |
| `St` | RangeStatus ( 0 = valid ) | 0 over the floor, 2 / 4 past the reach |
| `N` | New measurements since the previous tick ( driver counter ) | **2** most ticks ( 1 or 3 when the 50 ms sensor beats against the 100 ms tick ); **never 5-10** ( that would be the old re-reads ) |
| `A` | ms since the last new measurement | 0-60, never above about 100 |
| `G` | `Global_Status_L1` | 0 throughout |
| `O` | Out of range ( also after 160 ms with no new sample ) | 0 over the floor, 1 within one period of pointing past the reach |
| `T` | Worst `getRange_L1 ( )` poll since the previous tick, µs | Recorded: about 4800 expected on a sample poll ( reviewer estimate ). A loop is 3500 µs |
| `B` | `BaroAlt`, cm | Varies a little ( the datum tracks while disarmed ) |

**Pass ( task 3 done-when ):** over a still window of at least 10 s, elapsed time / sum of `N` =
50 ± 5 ms; worst `T` recorded; `A` never above 100 ms in 60 s of stillness; `G` 0; `O` goes to 1 when pointed past the reach.

**Setup.** As in Test 1: VL53L1X on the laser connector, facing down, **propellers off, never armed**,
tape measure, the same matt floor.

**Sequence** ( about 2 minutes ):

1. Flash from PlutoIDE. Remove all power ( battery and USB ), wait 5 s, and power up from the battery.
   Check that the LEDs come up normally.
2. Connect the app, turn Developer Mode on, start logging, and check that lines appear.
3. **Still on a box, 30 s.** Rest it level on a box so the laser sees the floor at 30-60 cm. Measure
   laser-to-floor with the tape and note it.
4. **Slow lift, about 20 s.** Raise it level to about 150 cm over the floor and back down.
5. **Past the reach, 10 s.** Point it at something more than 4 m away, then back at the floor.
6. **Still again, 60 s.** Back on the same box. Do not touch it.
7. Stop logging, paste the log into `logs/log-2.txt`, and save. Note the tape height from step 3.

**Stop early** if `N` is 5 or more on most ticks ( re-reads are back ), `A` keeps climbing ( the
sensor stopped ), or `G` is not 0. Save the log as it is and report it.

### Test 2 result ( `logs/log-2.txt`, 23 Sep 2026 15:47-15:51 )

**Build as above ( 45 ms budget, 50 ms period ). 2197 records over 229.7 s, all complete, no delivery
gaps.**

| Check ( task 3 done-when ) | Result | Pass |
|---|---|---|
| Sample interval 50 ± 5 ms ( elapsed / sum of `N` ) | 52.7 ms overall. Still windows: 53.7, 52.0, 53.2, 52.7 ms | yes |
| One read per measurement ( `N` never 5-10 ) | `N` = 2 on 1910 ticks, 1 on 158, 3 on 128, 0 once. Max 3 | yes |
| Fresh for 60 s | 180-229 s still on the floor: `A` max 81 ms. Whole log: `A` max 96, never above 100 | yes |
| `G` 0 | 0 on all 2197 ticks | yes |
| Out of range flagged | 150-175 s pointed far: `St` 2 ( signal fail ) on 203 ticks, `O` = 1 on each. `O` never 1 over the floor, except for one `St` 7 ( wrap-around ) sample at 1.2 m | yes |
| Staleness ( 160 ms ) never falsely set | `O` = 1 only on `St` ≠ 0 samples | yes |

**Interval is about 53 ms, not 50.** The ST setter programs the period against the sensor's
oscillator calibration, and the result is consistently 52-54 ms. Task 6 derives the L1x counts from
`L1X_SAMPLE_PERIOD_MS`. At 53 ms each count still covers the same time within 6 %, so no change is
needed, but the dropout ( 3 × 50 + 10 = 160 ms ) is about 3 periods, not more.

**Noise is lower than log-1** ( 45 ms budget against 41 ms, and still on a box ):

| Height | log-2 sd | log-1 sd |
|---|---|---|
| 3 cm ( on the floor ) | 2.1 / 3.5 mm | 2.1 mm |
| 57 cm ( box, 31-70 s ) | 2.7 mm | - |
| 148 cm ( held still, 100-130 s ) | 8.1 mm, `St` 0 throughout | - |
| 87 cm | - | 13.2 mm |

**Reach.** Valid ( `St` 0 ) and steady at 148 cm for 30 s. The ~1.9 m signal fail from log-1 is not
repeated here: nothing between 1.5 m and a far target was measured. Task 4 maps the reach.

**Poll cost ( `T` ).** The worst `getRange_L1 ( )` poll was 5437-5463 µs on **every** tick. That is
the sample poll ( data-ready + results read + interrupt clear ), once per 53 ms. It is longer than the
reviewer's 4.8 ms estimate and longer than one 3.5 ms loop, so about one loop in 15 is stretched to about
9 ms. The pre-fix driver blocked about 3.2 ms every 10 ms. New task 14 cuts this before the flights.

**Floor reading.** 25-32 mm when the craft sits on the floor ( the laser is about 3 cm up ). log-1's
opening 28 mm was the same.

## Test 3: noise, accuracy and tilt up to 1.5 m ( task 4, `logs/log-3.txt` )

**Question.** How noisy and how accurate is the VL53L1X at 50, 100 and 150 cm on the flying floor,
and does a tilted reading lengthen as `h / cos ( tilt )`? ( The reach above 1.5 m is task 15, in
flight. )

**Build.** Task 3 driver ( 45 ms budget / 50 ms period ), `LASER_TOF_L1x` on, **`LASER_ALT` off**.
The log is `mm St N A G O T Tl`, about 89 B per tick. `Tl` replaces `B`, because the baro reads about 0
while disarmed on the bench.

| Field | Meaning |
|---|---|
| `mm`, `St`, `N`, `A`, `G`, `O`, `T` | As in Test 2 |
| `Tl` | Tilt: the larger of \|roll\| and \|pitch\|, deci-degrees ( 300 = 30° ) |

**What each step shows.** At each height: the mean against the tape ( accuracy ) and the sd over
10 s ( noise ). At the tilts: `mm` against `h / cos ( Tl )`, which is 1.035 × h at 15° and 1.155 × h
at 30°.

**Setup.** VL53L1X on the laser connector, facing down, **propellers off, never armed**. Tape
measure. The same matt floor. Something steady at each height, if possible ( stacked boxes, a chair, a
table, a shelf ). Hand-holding works, but a stand halves the noise from your hand. **Measure from the
laser window to the floor.**

**Sequence** ( about 3 minutes; hold still for at least 10 s at each step, with a short pause between
steps ):

1. Flash, remove all power, wait 5 s, and power up from the battery. Connect the app, turn Developer
   Mode on, and start logging.
2. **Floor, 10 s.** Drone flat on the floor.
3. **50 cm, 15 s.** Level, laser 50 cm above the floor ( tape ). Note the exact tape value.
4. **100 cm, 15 s.** Level at 100 cm. Note the tape value.
5. **150 cm, 15 s.** Level at 150 cm. Note the tape value.
6. **Tilt at 100 cm.** Back at 100 cm: level for 10 s, then **15°** for 10 s, then **30°** for 10 s
   ( roll or pitch, whichever is easier ), then level for 10 s. Keep the laser spot on the same matt
   floor.
7. **Floor, 10 s.** Flat on the floor again.
8. Stop logging, paste the log into `logs/log-3.txt`, and **write the tape values from steps 3-5 at the
   top of the file** ( for example `# 50cm=50.5 100cm=99 150cm=151` ).

### Test 3 result ( `logs/log-3.txt`, 23 Sep 2026 16:13-16:18 )

**2130 records over 289.6 s.** The PC's Wi-Fi dropped for 62 s ( 127-189 s ), before the tilt step. The
sensor was unaffected. Over the whole log: `St` 0 on every sample, `G` 0, `O` 0, `N` 1-3 ( one 4 and
one 5 around the reconnection ), `A` ≤ 96 ms, `T` ≤ 5469 µs ( unchanged; task 14 ).

**Noise, hand-held.** The sd includes hand motion. `mm·cos` is the tilt-corrected vertical height.

| Step | mm mean | sd | Tilt | mm·cos mean | sd |
|---|---|---|---|---|---|
| Floor ( start ) | 28.2 | 2.1 | 0.7° | 28.2 | 2.1 |
| ~50 cm | 492.4 | 5.9 | 8.6° | 486.7 | 5.1 |
| ~100 cm | 1002.7 | 4.4 | 2.7° | 1001.6 | 4.5 |
| ~150 cm | 1521.2 | 4.8 | 3.7° | 1517.7 | 5.0 |
| Floor ( end ) | 25.6 | 1.6 | 0.5° | 25.6 | 1.6 |

Noise stays at 4-6 mm up to 1.5 m even hand-held, well inside what the estimator needs ( the L0X had
about 12 mm ).

**Tilt.** Held at about 95 cm. The hand-held tilts were about 10° and 20°, not the planned 15° / 30°.

| Step | mm | Tilt | mm·cos |
|---|---|---|---|
| Level | 944.7 | 2.4° | 943.8 |
| ~10° | 949.4 | 9.5° | 936.3 |
| ~20° | 1018.8 | 19.6° | 959.9 |

At 20° the raw reading grew by 75 mm. After the cosine correction only 16 mm ( 1.7 % ) is left, which
is about the hand-height drift between steps ( the "level" readings moved 910 → 944 mm over 60 s ).
The `h / cos ( tilt )` model therefore holds, and the estimator's cosine correction is the right one.
**Not tested:** above 25°, where the estimator rejects the sample. That is estimator behaviour, checked in
task 7.

**Accuracy against the tape: not measured** ( no tape values ). Decided with the user: the absolute
accuracy is checked against the baro in the task 15 flight. A constant bias is absorbed by the handover
offset anyway.

**Near-field bias on the ground ( user ).** Landed, the laser is about 2.6-2.8 cm above the floor.
That is below the VL53L1X's rated minimum range ( about 4 cm ), and there the reading is biased. It
reads correctly once airborne. The two floor readings here differ by 2.6 mm on the same floor.
Consequence for task 6: an on-ground laser reading must not be used as a height reference.

## Task 14: sample-poll cost ( build only, bench pending )

Before ( log-2 / log-3, `T` ): 5437-5469 µs per sample poll, about 213 bus bytes ( data-ready 5 +
results 137 + GENERAL_ONWARDS write 71 ), so about 25.5 µs per byte including the API's processing.

After ( estimate ): data-ready 5 B + result read 21 B ( address, 2 index, address, 17 data ) +
interrupt clear 4 B = 30 bytes, about 0.7-0.8 ms at the same 25.5 µs/byte ( 22.5 µs/byte is the
400 kHz wire time ). Polls without new data stay at about 0.13 ms. Target: `T` ≤ 2000 µs.

Build: gate PASS on PRIMUS_X2_v1 with `LASER_TOF_L1x`, text 111728 → 109604 B, data 1320 → 1328 B,
bss unchanged. L0X `altitudehold.o` disassembly identical to the baseline.

Bench check ( done, Test 4 below ): `T` ≤ 2000 µs; `N` 2-3 per tick, interval 50 ± 5 ms; `G` 0; `St` 0 on a floor inside
reach with `mm` matching log-3 at the same height; `St` 2 / out of range beyond reach or with the
sensor covered.

## Test 4: poll-cost check ( task 14, `logs/log-4.txt` )

**Build.** Task 14 driver ( 17-byte result read + interrupt clear ), `LASER_TOF_L1x` on, **`LASER_ALT`
off**. Flash 108.3 KB, RAM 15.8 KB. The log is the same as Test 3: `mm St N A G O T Tl`.

**Pass ( task 14 done-when ):** `T` ≤ 2000 µs on every tick; `N` 1-3 ( mostly 2 ), with interval
50 ± 5 ms; `A` ≤ 100 ms; `G` 0; `St` 0 over the floor, with `mm` at the box and the 100 cm step within
about 1 cm of a log-3-style reading at the same height ( checks the local decode against the API's );
`O` = 1 past the reach and when covered.

**Setup.** Props off, never armed, same floor, same box as log-2 ( ~57 cm ) if you still have it.

**Sequence** ( about 2 minutes, at least 10 s still per step ):

1. Flash, remove all power, wait 5 s, power up from the battery. Connect the app, turn Developer
   Mode on, start logging.
2. **Floor, 10 s.**
3. **Box, 20 s** ( the ~57 cm box from log-2, if you have it ).
4. **100 cm by hand, 15 s**, level.
5. **Past the reach, 10 s**: point it at something more than 4 m away, then back at the floor.
6. **Covered, 5 s**: finger or dark cloth over the window.
7. **Still on the box or floor, 60 s.** Do not touch it.
8. Paste the log into `logs/log-4.txt`.

**Stop early** if `N` is 0 for several ticks in a row with `A` climbing past 160 ( ranging stopped
after the short clear ), or `G` is not 0.

### Test 4 result ( `logs/log-4.txt`, 23 Sep 2026 )

**1630 records over 172.8 s**, 1629 complete. Only short delivery gaps ( ≤ 1.4 s ).

| Check ( task 14 done-when ) | Result | Pass |
|---|---|---|
| Worst poll `T` ≤ 2000 µs | **784-789 µs** on every tick ( p99 788 ), down from 5437-5469. The one exception is the first record, 3627 µs: that value covers everything since boot, including the one-time `StartMeasurement` config write before logging started. It never recurs | yes |
| Ranging continues with only the interrupt clear | `N` 2 on 1530 ticks, 3 on 96. The one `N` 9 at 166 s is a paused `plutoLoop` ( app link hiccup, gaps around 165 s ): the driver kept counting through it and `A` stayed low | yes |
| Interval 50 ± 5 ms | **51.2 ms** ( 5-25 s ), 51.1 ms ( 120-155 s ), 51.5 ms overall. Closer to 50 than log-2's 52.7 ms, because the per-range restart no longer delays each cycle | yes |
| `A` ≤ 100 ms | max 64 ms | yes |
| `G` 0 | 0 throughout | yes |
| Out of range flagged | `St` 2 on 60 samples when pointed far ( 80-90 s ), `O` = 1 on each; 2 `St` 7 samples ( wrap target ), as in log-2 | yes |
| Decode matches the API path | Floor 25-28 mm, the same as log-3 ( 25.6-28.2 mm ) with the full API. Box 549 mm ( sd 2.8, tilt 6.1° ) against log-2's 570 mm, which is not comparable: a different placement, and log-2 has no tilt field. The reviewer found the decode bit-identical to the API for every status | yes |

**Covered window:** 4-10 mm with `St` 0, the same as the full API in log-1. My plan wrongly expected
`O` = 1 here. Rejecting it is the task 6 minimum-valid range, not a driver job.

**Poll cost now:** one sample poll of about 0.79 ms per 51 ms; data-ready-only polls are much cheaper.
Against the 3.5 ms loop this is no longer a stretched loop.

## Test 5: reach flight on baro hold ( task 15, `logs/log-5.txt` )

**Question.** Up to what height does the VL53L1X give valid ranges over the flying floor, and does
it agree with the baro below that? This sets the margin of the 160/140 cm handover band.

**Build.** Task 14 driver, `LASER_TOF_L1x` on, **`LASER_ALT` off**. Altitude hold is **baro-only**,
the same control path as the shipped firmware. The laser is only logged and cannot affect the flight.
Flash 108.4 KB, RAM 15.8 KB.

**Log fields** ( 10 Hz, about 93 B per tick ):

| Field | Meaning |
|---|---|
| `mm` | Laser raw range, mm ( slant ) |
| `St` | Range status: 0 valid, 2 signal fail, 4 out of bounds, 7 wrap |
| `O` | Driver out-of-range flag |
| `Tl` | Tilt, deci-degrees |
| `B` | `BaroAlt`, cm: the datum freezes at arm, so this is height above take-off |
| `E` | Estimated altitude the position loop flies, cm ( baro-only here ) |
| `H` | Altitude setpoint, cm |
| `Ar` | 1 armed |

**Pass ( task 15 done-when ):** the valid-reach height is found on the climb and on the descent ( the
`St` 0 fraction per 25 cm band of `E` ), and the band is confirmed as 160/140 with at least 50 cm of
margin ( valid reach ≥ 2.1 m ), or new band values are signed off.

**Site and safety.**
- Indoors, **ceiling above 3 m**, the same matt floor, no draught. A spotter if possible.
- Full battery. Clear floor under the flight path: nobody walking underneath, no boxes ( the laser
  would see them ).
- Stay 1 m or more from walls, so the laser spot stays on the floor.
- The laser is not in the control loop. Baro hold may wander ±10-20 cm, which is normal and fine.
- **Abort** on anything unusual: land with the throttle as you normally would.

**Sequence** ( about 3 minutes of flight ):

1. Flash from PlutoIDE. Remove all power, wait 5 s, and power up from the battery.
2. Connect the app, turn Developer Mode on, start logging, and check that lines appear.
3. **Disarmed on the floor, 15 s.**
4. Arm, take off in **ALT_HOLD**, and bring it to about **1 m**. Centre the throttle and hover **15 s**.
5. **Climb in steps**, holding about **10 s** at each, with the throttle centred while holding. Use `E`
   ( or the app's altitude ) to judge the height:
   - about **1.5 m**
   - about **1.75 m**
   - about **2.0 m**
   - about **2.25 m**
   - about **2.5 m** ( stop earlier if the ceiling is close; leave at least 50 cm below it )
6. **Descend in the same steps**: 2.25 → 2.0 → 1.75 → 1.5 → 1.0 m, about 10 s each.
7. Land normally and disarm. **Stay on the floor 15 s.**
8. Stop logging, paste the log into `logs/log-5.txt`, and note the ceiling height, anything odd, and
   roughly how high it went.

### Test 5 result ( `logs/log-5.txt`, 23 Sep 2026 )

**1271 records over 141 s; armed 13-120 s.** Flown on baro ALT_HOLD: to ~1.2 m, ~1.85 m, ~2.5 m, then
down to ~0.4 m, then back up to ~1.7 m, and landed. Four delivery gaps of about 3 s ( link ). The user
flew above the ceiling-fan blades at the top step.

**Valid reach ( `St` 0 fraction per 10 cm band of the raw laser reading, airborne ):**

| Laser reads | Valid | |
|---|---|---|
| 50-180 cm | 100 % ( 386 / 387 samples ) | clean |
| 180-190 cm | 90 % ( 18 / 20 ) | first dropouts |
| 190-200 cm | 57 % ( 52 / 92 ) | edge |
| 200-210 cm | 40 % ( 12 / 30 ) | |
| 210-290 cm | 0.3 % ( 1 / 404 ) | out of reach: `St` 2 ( signal fail ) |

Climb and descent agree ( descent: 100 % up to 200 cm on the few samples there ). The 50 % point is
about **1.95 m**. Medium mode, 45 ms budget, the user's matt floor.

**Band decision ( user ):** keep **160 / 140 cm**. That leaves 20 cm to the first dropouts and
35 cm to the 50 % point. The planned 50 cm margin is not met. The L0X flew with about 12 cm ( it
dropped at ~172 cm ), the handover happens on clean data, and the 160 ms dropout covers a miss.

**Laser against baro.** Tilt-corrected laser height ( minus the 2.5 cm on-ground reading ) minus the
estimate `E`: median **+22 cm** ( p10 +6, p90 +35, 397 valid samples between 30 and 160 cm ). The
difference is the same at 40 cm ( laser 62, `E` 39 ) and at 120 cm ( laser 141, `E` 121 ), so the
scale agrees and the baro carries a constant offset of about 20 cm in flight. That is the baro's
take-off offset, not a laser error: log-3 matched the nominal bench heights. The handover's frozen offset
absorbs it.

**Ceiling-fan event, 52.3-52.8 s.** At `E` ≈ 255 cm, 4 consecutive **valid** samples read 54-57 cm,
between `St` 2 samples. The laser saw the fan blades about 2.0 m above the floor, below the craft.
With `LASER_ALT` on, this is exactly the pattern that could trigger a baro → laser return at a false
56 cm ( a reading below 140 cm, several samples in a row ). Task 6 must check that the shared return
logic rejects a return reading far from the current estimate, the way the step detector does below
the band.

## Test 6: Long mode bench ( task 17, `logs/log-6.txt` )

**Question.** In Long mode, at the same 45 ms / 50 ms timing, is the VL53L1X as quiet and as well
behaved as Medium below 1.8 m?

**Build.** Task 16 driver with `L1X_DISTANCE_MODE` = **Long** ( temporary, in `target.h` ),
`LASER_TOF_L1x` on, **`LASER_ALT` off**. Flash 108.5 KB, RAM 15.8 KB. **The same image is used for the
task 18 flight**, so there is no reflash between the two.

**Log fields** ( 10 Hz, about 90 B per tick, 98 worst case ): `mm St N O T Tl E Ar`. These are the
Test 3 / 4 fields without `A` and `G` ( `O` also covers staleness and a latched error ), plus `E`
( baro altitude ) and `Ar` for the flight.

**Compare with Medium:**

| | Medium ( log-3 / log-4 ) |
|---|---|
| Floor | 25-28 mm, sd 1.6-2.1 |
| Box ~55-57 cm | sd 2.7-2.8 mm |
| ~100 cm, hand-held | sd 4.4 mm |
| ~150 cm, hand-held | sd 4.8 mm |
| Tilt ~20° | `mm·cos` within 1.7 % of level |
| Interval / `N` | 51.2 ms / 2 |
| `T` | 784-789 µs |

**Sequence** ( props off, never armed, about 3 minutes, at least 10 s still per step ):

1. Flash from PlutoIDE. Remove all power, wait 5 s, power up from the battery. Connect the app, turn
   Developer Mode on, start logging.
2. **Floor, 15 s.**
3. **Box, 20 s** ( the same box as log-2 / log-4 ).
4. **100 cm by hand, 15 s**, level.
5. **150 cm by hand, 15 s**, level.
6. **Tilt at ~100 cm:** level 10 s, then about **20°** for 10 s, then level 10 s.
7. **Past the reach, 10 s**: point it more than 4 m away, then back at the floor.
8. **Floor, 15 s.**
9. Stop logging and paste the log into `logs/log-6.txt`.

**Watch live:** `N` about 2, `T` about 790, `St` 0 over the floor and box. If `St` flickers to 2 / 4 /
7 at the box or at 100 cm, Long mode is noisier here: note it.

### Test 6 result ( `logs/log-6.txt`, 24 Sep 2026 )

**1628 records over 170 s**, all complete. `N` 2-3 ( interval 50 ms ), `T` 785 µs ( p99 789 ), the same
as Medium: the mode does not change the poll cost. There was no box step; the other steps are all present.

| Step | Long: valid | Long: `St` 7 | Long: sd ( valid ) | Medium: valid | Medium: sd |
|---|---|---|---|---|---|
| Floor ( start ) | 100 % | 0 | 1.9 mm ( reads **16.5 mm** ) | 100 % | 2.1 mm ( reads 28.2 mm ) |
| ~100 cm | **83 %** | 33 / 190 | **8.2 mm** | 100 % | 4.4 mm |
| ~150 cm | **92 %** | 16 / 192 | 6.1 mm | 100 % | 4.8 mm |
| ~95 cm level | **85-95 %** | 4-17 | 4.4-10.7 mm | 100 % | 5.4 mm |
| **~20° tilt** | **1 %** | 116 / 117 | - | **100 %** | 6.6 mm |
| Floor ( end ) | **82 %** | 24 / 135 | 2.3 mm | 100 % | 1.6 mm |
| Pointed far | `St` 2 | - | - | `St` 2 | - |

**Long mode at 45 ms / 50 ms is worse than Medium below 1.8 m on every count.**
- 5-17 % of level samples are rejected as `St` 7 ( wrap-target fail, `PHASECONSISTENCY`: the two VCSEL
  phases disagree ).
- A 20° tilt, inside the estimator's 25° limit, makes the laser almost completely invalid.
- Noise is up to twice as high.
- The on-ground reading is further off ( 16.5 mm against Medium's 28 mm ), so the near-field bias is
  worse.

The likely cause is the short integration: Long's longer VCSEL periods ( 15 / 13 ) get the same 9.2 ms
per phase, so the phase estimate that tells the wrap apart is noisier. A longer budget might cure it,
but not at 20 Hz ( ST requires period ≥ budget + 4 ms ).

## Test 7: Long mode at 10 Hz bench ( task 19, `logs/log-7.txt` )

**Question.** Does Long mode with about 34 ms per phase ( 95 ms budget, 100 ms period ) stop the `St` 7
wrap failures that log-6 showed at 9.2 ms per phase, both level and tilted?

**Build.** `L1X_DISTANCE_MODE` Long, `L1X_TIMING_BUDGET_US` 95000, `L1X_SAMPLE_PERIOD_MS` 100 ( all
temporary, in `target.h` ), `LASER_TOF_L1x` on, **`LASER_ALT` off**. Flash 108.5 KB, RAM 15.8 KB.
The log is the same as Test 6: `mm St N O T Tl E Ar`. **Expect `N` ≈ 1 per tick now** ( 100 ms samples ).

**Compare:**

| | Medium 45 / 50 ( log-3 / log-4 ) | Long 45 / 50 ( log-6 ) |
|---|---|---|
| Valid, level 1-1.5 m | 100 % | 83-95 % |
| Valid at ~20° | 100 % | 1 % |
| sd at ~100 cm | 4.4 mm | 8.2 mm |
| Floor reading | 25-28 mm | 16.5 mm |

**Sequence** ( props off, never armed, about 3½ minutes, at least 10 s still per step ):

1. Flash, remove all power, wait 5 s, power up from the battery. Connect the app, turn Developer Mode
   on, start logging.
2. **Floor, 15 s.**
3. **Box, 20 s** ( the same box as before ).
4. **100 cm by hand, 15 s**, level.
5. **150 cm by hand, 15 s**, level.
6. **Tilt at ~100 cm:** level 10 s → **about 10°** 10 s → **about 20°** 10 s → **about 25°** 10 s →
   level 10 s. Tilt one axis only ( roll or pitch ), and watch `Tl` for the angle: 100 = 10°, 200 = 20°,
   250 = 25°.
7. **Past the reach, 10 s**: point it more than 4 m away, then back at the floor.
8. **Floor, 15 s.**
9. Paste the log into `logs/log-7.txt`.

**Watch live:** `N` about 1, `T` about 790, and how often `St` shows 7 at each step.

### Test 7 result ( `logs/log-7.txt`, 24 Sep 2026 )

**3449 records over 359 s** ( about 3 minutes of it on the floor at the start ). `N` 1 on 3346 ticks
( 100 ms samples ), `T` 785 µs ( p99 789 ). No box step; tilt at about 10°, 20° and 35°.

| Step | Long 95 / 100 ( log-7 ) | Long 45 / 50 ( log-6 ) | Medium 45 / 50 ( log-3 ) |
|---|---|---|---|
| Floor | 100 %, **25.6 mm, sd 1.2** | 100 %, 16.5 mm, sd 1.9 | 100 %, 28.2 mm, sd 2.1 |
| ~100 cm | **100 %, sd 3.2 mm** | 83 %, sd 8.2 | 100 %, sd 4.4 |
| ~150 cm | **100 %, sd 5.2 mm** | 92 %, sd 6.1 | 100 %, sd 4.8 |
| ~10° | 100 % | - | - |
| ~20° | **100 %** | 1 % | 100 % |
| ~35° | 96 % ( 4 `St` 7 ) | - | - |
| Pointed far | `St` 2 | `St` 2 | `St` 2 |

**Cosine model**, at ~90 cm: level `mm·cos` 887.8, 10° 896.1, 20° 890.5, 35° 899.9 mm. That is within
1.4 % across the tilts, which is about the hand's height drift.

**Long at 95 ms / 100 ms is as clean as Medium at 45 / 50, or cleaner**: 100 % valid level and at 20°,
the lowest noise at 100 cm, and a floor reading close to Medium's. The `St` 7 failures in log-6 came
from the short integration ( 9.2 ms per phase ), not from the mode itself; at 34 ms per phase they
disappear. The cost is **half the sample rate** ( 10 Hz against 20 Hz ).

## Test 8: Long 10 Hz reach flight ( task 18, `logs/log-8.txt` )

**Question.** How far does Long 95 ms / 100 ms reach over the flying floor, compared with Medium's
100 % valid to 180 cm and 50 % point at ~195 cm ( log-5 )? The answer decides between Medium 45 / 50 and
Long 95 / 100 for the rest of the topic.

**Build.** The task 19 image, already on the drone ( no reflash ): Long, 95 ms / 100 ms, `LASER_TOF_L1x`
on, **`LASER_ALT` off**. Altitude hold is **baro-only**; the laser is only logged. The log is
`mm St N O T Tl E Ar`.

**Pass ( task 18 done-when ):** the valid fraction per 10 cm band on the climb and on the descent,
tabled against log-5, and the laser-minus-`E` offset; then the mode choice with its reason.

**Site and safety.**
- **Stay away from the ceiling fan**: take off at a spot where the craft is at least **1.5 m
  horizontally from the fan-blade tips**, and never climb to the fan's height beside it. log-5 flew
  above the blades.
- Ceiling: keep at least 50 cm below it. A spotter if possible. Full battery. Clear floor under the
  flight path.
- Baro hold may wander ±10-20 cm; that is normal. **Abort** by landing normally.

**Sequence** ( about 3 minutes of flight ):

1. Power up, connect the app, turn Developer Mode on, start logging.
2. **Disarmed on the floor, 15 s.**
3. Arm, take off in **ALT_HOLD** to about **1 m**, and hover **15 s**.
4. **Climb in steps**, holding about **10 s** each with the throttle centred. Watch `E` ( cm ):
   **1.5 → 1.75 → 2.0 → 2.25 → 2.5 m**, and **higher, up to 3 m, if the ceiling allows** ( 50 cm
   clearance ). Long may reach further than Medium's 1.95 m, and the point of this flight is to find
   where it stops.
5. **Descend in the same steps** to 1 m, about 10 s each.
6. Land, disarm, **15 s on the floor**.
7. Paste the log into `logs/log-8.txt` and note the ceiling height, the highest `E`, and anything odd.

**Watch live:** `St` 0 at each step. Note the `E` at which `St` 2 starts to appear.

### Test 8 result ( `logs/log-8.txt`, 24 Sep 2026 )

**575 records over 61 s; armed from 17.5 s.** The pasted log ends at 61 s during the climb ( about 2.1 m ),
so the descent and landing are missing. The user confirms the flight itself was normal.

**Valid fraction per 10 cm band ( airborne ), Long 95 / 100 against Medium 45 / 50 ( log-5 ):**

| Laser reads | Long 95 / 100 | Medium 45 / 50 |
|---|---|---|
| 30-160 cm | 100 % ( 197 / 197 ) | 100 % |
| 170-180 cm | 2 / 3 | 100 % |
| 180-190 cm | 1 / 3 | 90 % |
| 190-200 cm | 0 / 3 | 57 % |
| 200-260 cm | **0 / 188**, all `St` 2 | ~0-40 % |

The climb from 1.2 to 2.0 m took about 5 s, so there are few samples between 1.6 and 2.0 m. Every sample
above 2 m is a signal fail. **Long mode gives no extra reach on this floor**: both modes stop at about
1.8-2.0 m. Laser − `E`: median +15 cm ( p10 +1, p90 +19 ), the same baro take-off offset as in log-5.

**Why the reach is the same in both modes ( hypothesis ).** `St` 2 means the return signal is below the
minimum count-rate limit. For both modes that limit is the preset tuning default of **1.5 MCPS**
( `tp_lite_med/long_min_count_rate_rtn_mcps`, 192 in 9.7 format ); the preset overwrites DataInit's
0.25 MCPS. A limit that strict would cap the reach whatever the integration. Lowering it is the lever
for the 340 / 300 band: a follow-up topic, not tested here.

**Mode decision ( user ): Medium, 45 ms / 50 ms.** It keeps 20 Hz with no loss of reach, and the task 6
constants stay as designed. Long 95 / 100 is only about 1 mm quieter at 1 m, at half the sample rate.

## Test 9: bench handover and dropout with `LASER_ALT` ( task 7, `logs/log-9.txt` )

**Question.** With the VL53L1X now driving the estimator ( task 6 ), does the source switch at the band
edges with no jump in `E`, fall back to the baro when the laser is lost, and come back cleanly?

**Build.** Working tree after task 6: Medium 45 / 50, `LASER_TOF_L1x` **and `LASER_ALT` on**
( temporary ). Flash 111.0 KB, RAM 16.1 KB. Gate: no new warnings in `src/`.

**Log fields** ( 10 Hz, about 95 B per tick ):

| Field | Meaning |
|---|---|
| `mm` | Raw laser range, mm ( slant ) |
| `St` | Range status ( 0 valid ) |
| `O` | Driver out-of-range: invalid, below 15 mm, latched error or stale |
| `S` | Estimator source: **1 laser, 0 baro**, 2 object hold-off |
| `E` | Estimated altitude, cm ( what the position loop flies ) |
| `V` | Estimated vertical speed, cm/s |
| `Tl` | Tilt, deci-degrees |
| `Ar` | 1 armed |

**What this bench can and cannot test.** Disarmed, the handover, the frame shift on return, the dropout
and the return all run. The **object hold-off** and the **return guard** run only when airborne ( armed,
not in ground idle ), so they are checked in the task 9 flights, not here.

**Pass ( task 7 done-when ):** laser → baro when the laser reads 160 ± 5 cm, baro → laser at 140 ± 5 cm;
`E` steps by less than 3 cm at each switch; covered, or tilted past 25°: `S` goes to 0 within about
2 ticks ( 185 ms ), and comes back to 1 within about 3 samples once uncovered or level.

**Other effects of `LASER_ALT` on this target** ( reviewer ), relevant to the task 9 flights:
- The app / API **take-off** goes to an absolute 120 cm ( `command.cpp:116` ) instead of a relative climb.
- The accelerometer Z deadband is 0 ( `imu.cpp:261` ), the fix for the L0X bob.
- `checkReading ( )` replaces `checkBaro ( )` in the estimator ( `altitudehold.cpp:914` ).
- The estimator tau is fixed at 1.5 s: the 2 s / 5 s switch above 30° of tilt is gone.

**Setup.** **Propellers off, never armed.** Tape measure, the same matt floor. A stand or shelf at about
1.8 m helps for step 4.

**First attempt ( 24 Sep 12:33, `logs/log-9-old-firmware.txt` ): not usable.** The log has the tasks
17-19 fields ( `mm St N O T Tl E Ar` ), and `E` stays near 0 while the laser reads 99 cm, so the drone ran
the previous image, not this one ( built 12:27:56 ). Before logging, check that the monitor shows the
**`S:` and `V:`** fields.

**Sequence** ( about 3 minutes, slow movements, at least 10 s still per step ):

1. Flash from PlutoIDE, remove all power, wait 5 s, power up from the battery. Connect the app, turn
   Developer Mode on, start logging.
2. **Floor, 15 s.** Expect `S` 1 and `E` of about 2-3 cm.
3. **Slow lift to about 1.2 m, hold 10 s.** Expect `S` 1 and `E` ≈ `mm` / 10.
4. **Slowly up through 1.6 m to about 1.8 m, hold 10 s.** `S` should go 1 → 0 as `mm` passes ~1600, with
   no jump in `E`.
5. **Slowly down through 1.4 m to about 1.2 m, hold 10 s.** `S` should go 0 → 1 as `mm` passes ~1400,
   with no jump in `E`.
6. **Covered at ~1.2 m:** finger or cloth over the window for 5 s ( expect `O` 1, `S` 0 ), then
   uncover and hold 5 s ( expect `S` 1 ).
7. **Tilt at ~1.2 m:** about 35° for 5 s ( expect `S` 0 ), then level for 5 s ( expect `S` 1 ).
8. **Repeat steps 4-5 once** ( up through 1.6 m, down through 1.4 m ).
9. **Floor, 15 s.** Paste the log into `logs/log-9.txt`.

### Test 9 result ( `logs/log-9.txt`, 24 Sep 2026 12:57 )

**1570 records over 164 s**, all complete, disarmed. `St` 0 throughout.

| Check | Result | Pass |
|---|---|---|
| Laser → baro at 160 cm | `mm` 1605 and 1609; `E` continuous ( 159 → 160 → 161, 157 → 159 → 158 ) | yes |
| Baro → laser at 140 cm | `mm` 1389 and 1392 ( slant; about 138.6 cm vertical at 4° ) | yes |
| `E` at the return | −14 cm and +6 cm: the designed frame shift. It is large here only because the **baro datum keeps re-zeroing while disarmed**: `E` stayed at about 155 on the baro leg while the laser went from 1.9 to 1.4 m. In flight the datum is frozen at arm, the shift equals the small baro drift, and the setpoint moves with it ( checked in task 9 ) | as designed |
| Dropout and return | Seen when the craft was picked up off the floor: `mm` 0 → 12 ( `O` 1, below the 15 mm minimum ) → `S` 0 about 200 ms after the last good sample → `S` 1 three samples ( 150 ms ) later. At touchdown, a single 0 mm sample caused no dropout | yes |
| Covered at ~1.2 m | Not performed | - |
| Tilt past 25° | Not reached: largest `Tl` 182 ( 18.2° ); `S` correctly stayed 1 | - |

**Closed without the covered and tilt steps** ( user ). The dropout / return path was exercised by the
near-floor 0 mm readings. Tilt rejection is shared, L0X-flight-validated code; it is to be seen in the
task 9 flight logs.

**Noted for take-off:** lifting off the floor briefly reads 0-12 mm with `St` 0 ( near field, tilting ).
The 15 mm minimum rejects it, so a take-off can spend about 0.3 s on the baro and return with a frame
shift of a few cm. Watch for it in task 9.

## Build gate ( task 8, 24 Sep 2026 )

`.claude/skills/pluto-build/driver.sh --gate PRIMUS_X2_v1` on three configurations ( `target.h` switched
temporarily, then restored and compared with `cmp` ):

| Configuration | Gate | Flash | RAM | At `b1f070b` ( task 1 ) |
|---|---|---|---|---|
| L1x + `LASER_ALT` ( working tree ) | PASS, no new warnings in `src/` | 111.0 KB | 16.1 KB | 109.5 / 15.8 KB ( old L1x branch ) |
| No laser ( shipped ) | PASS | 98.9 KB | 14.8 KB | 98.9 / 14.8 KB |
| L0X + `LASER_ALT` | PASS | 101.7 KB | 15.5 KB | 101.7 / 15.5 KB |

The 32 `altitudehold.cpp` warnings appear in all three, including the no-laser build: they are
pre-existing baseline entries, not from this topic. The 12 `-Wreorder` entries fixed in task 3 need a
baseline refresh at commit ( pluto-commit ).

**Object code:** L0X and no-laser `altitudehold.o` identical to the task 1 baselines. The L1x grew by
+1.5 KB flash and +0.3 KB RAM over the old branch: the shared fusion path, the return guard and the
driver fixes, less the 2.1 KB saved in task 14.

## Test 10-12: validation flights ( task 9, `logs/log-10.txt` … `log-12.txt` )

**Build.** Working tree after task 8: Medium 45 / 50, `LASER_TOF_L1x` + **`LASER_ALT` on** ( temporary ),
the shared fusion path, the 15 mm minimum, the return guard with its 2.5 s steady exit. 111.0 KB / 16.1 KB.
**The laser is in the control loop.**

**Log fields** ( 10 Hz, ~92 B per tick ): `mm St S E H V Tl Ar`. `S` 1 laser, 0 baro, 2 object hold-off;
`E` the estimate and `H` the setpoint ( cm ); `V` in cm/s; `Tl` tilt in deci-degrees.

**Rollback** if anything misbehaves: land normally. The fallback firmware is the same tree with `LASER_ALT` off
( baro-only, as shipped ).

**Site and safety ( all three flights ).** The same matt floor; **well away from the ceiling fan**; full battery;
a spotter; ALT_HOLD for everything; land normally to abort. `LASER_ALT` changes the app / API take-off to an
absolute 120 cm.

**Flight A: hover and handover ( `log-10` ).**
1. Power up, connect, Developer Mode on, start logging. 15 s disarmed on the floor.
2. Take off in ALT_HOLD to about **1 m**. Centre the throttle, **hands off for 30 s**.
3. Climb to about **1.9 m** ( above the 160 cm handover ), hold 15 s.
4. Descend to about **1.2 m** ( below 140 cm ), hold 15 s.
5. Repeat steps 3-4 once.
6. Descend and **land normally**, disarm, 10 s on the floor. Paste the log into `logs/log-10.txt`.

Checks: hover `E` within ±3 cm for 30 s; at 160 → baro, `E` and `H` without a step; at 140 → laser, `E` and `H`
shift **together** ( the craft does not move ); take-off and landing normal.

**Flight B: object hold-off ( `log-11` ).** Needs a helper and a box about 20-30 cm tall.
1. Take off to about **1 m**, hands off.
2. Helper slides the box **briskly** under the craft, leaves it 5 s, removes it briskly. Wait 10 s.
3. Repeat **slowly** ( slide over about 2 s ). Wait 10 s.
4. Land normally. Paste into `logs/log-11.txt`.

Checks: `S` 2 ( hold-off ) for about 2.5 s after each step, then re-base; the craft does not climb or drop hard; it
returns to its old clearance.

**Flight C: return guard and past the reach ( `log-12` ).** Needs a helper and a flat board.
1. Take off, climb to about **1.9 m** ( `S` 0, on the baro ). Hold 10 s.
2. **Brief object:** the helper passes the board through under the craft at about **1 m** height, quickly
   ( under 1 s ), like the fan blades. Expect `S` to **stay 0**.
3. **Steady object:** the helper holds the board **still** at about 1 m under the craft for **5 s**. Expect `S` 0 for
   about 2.5 s, then `S` 1 ( return accepted ), with `E` and `H` shifting together. Then the helper removes the
   board slowly; expect a hold-off or a handover back to the baro.
4. **Past the reach:** climb to about **2.3 m** ( keep clear of the ceiling ), hold 10 s. Expect `S` 0 with a
   steady hold.
5. Descend to about 1 m ( `S` 1 again ), land normally. Paste into `logs/log-12.txt`.

If you are alone, fly A, then B's and C's steps without the helper parts ( C step 4 still applies ), and say so.

### Test 10-12 results ( 24 Sep 2026, `log-10` A, `log-11` B, `log-12` C )

All three flights ran the task 9 image ( `mm St S E H V Tl Ar` ). All took off, flew and landed normally.

| Done-when item | Evidence | Result |
|---|---|---|
| Hover ±3 cm for 30 s | A, hands-off at ~1.27 m for 25 s: `E` sd 1.8 cm, 92 % within ±3 cm, p-p 8 cm; `E − H` −0.9 ± 1.8 cm; laser ( true ) sd 2.5 cm | pass ( about ±4 cm peak ) |
| Handover with no `E` jump | A: 1 → 0 at `mm` 1608 / 1616, `E` and `H` continuous ( 160 → 161, 159 → 162 ) | pass |
| Return: craft does not move | A: `E` +20 / `H` +24 cm and `E` −27 / `H` −26 cm; C: +119 / +118 and +169 / +167 cm. `E` and `H` shift together | pass |
| Object hold-off, re-base, return to clearance | B: six 2.5 s hold-offs ( `S` 2 ), each followed by a re-base with `E` and `H` together ( e.g. 81 → 26 / 85 → 30 ), then a climb back to the old clearance; brief moves cancelled; no hard climb or drop | pass |
| Past the reach: baro, no pull to a stale value | A: 15 s at ~2.8 m and 10 s at ~3.05 m on the baro ( `St` 2 ), `E` within ±5 cm; C at ~2.1 m the same | pass |
| Touchdown normal | A, B, C all landed and disarmed normally | pass |
| Tilt rejection above 25° | Max `Tl`: A 16.9°, B 8.6°, C 16.1° | not reached |
| Take-off near field | A and B: one ~0.1-0.2 s baro blip at lift-off ( `mm` 0-12 ), with no visible effect | as expected |

**Flight C, board 9 cm under the drone ( not the planned step 3 ).** The hold-off and re-base moved the
frame to "height above the board", and the drone climbed back to its 120 cm clearance above the board
( about 2.3 m true ). The board was then removed, so the laser saw the floor past 160 cm and the drone
went to the baro **carrying the board frame, about 1.1 m off**. That is the known L0X limit
( tof-althold-fusion README ). As the pilot descended, `E` read down to −142 cm while the laser, valid
again, read 0.75-1.4 m. **The return guard held** ( disagreement 110-220 cm ) and the steady exit then
returned with the full shift, `E` and `H` together. The second exit took **~10 s instead of 2.5 s**: during
the descent the baro-path estimate wobbled by more than the 10 cm steady band ( `E` swung 20-40 cm with
`V` ), which restarted the timer until the craft hovered ( 91.1-93.6 s ).

**Early hover in A ( 12-22 s ), after take-off:** `E` 15 cm below the setpoint for ~10 s before it
converged; the same slow first settle that tof-althold-fusion accepted after a fresh power-up.

**Change after the flights ( 24 Sep 2026, user ):** the guard's exit band is now its own L1x constant,
`ALT_TOF_RETURN_STEADY_CM` = **25 cm** ( was the 10 cm object band ). The object hold-off keeps 10 cm.
L0X / no-laser objects identical, gate PASS, 111.0 KB. Tilt above 25° stays not exercised ( user ).

## Test 13: guard exit re-fly ( task 9, `logs/log-13.txt` )

**Question.** With the 25 cm band, does the guard still hold off a brief object, and does the exit
return within about 2.5-4 s even while the craft moves?

**Sequence** ( a helper and a flat board; same safety rules as A-C, away from the fan ):
1. Take off, hover at about **1 m** for 10 s ( `S` 1 ).
2. Climb to about **1.9 m** ( `S` 0 ) and hold 10 s.
3. **Brief object:** the helper sweeps the board through at about **1 m height**, under the drone, in under
   1 s. Expect `S` to **stay 0**. Do it twice, 5 s apart.
4. **Steady object:** the helper holds the board **still at about 1 m height** ( about 90 cm under the drone,
   not close to it ) for **6 s**. Expect `S` 0 for about 2.5 s, then `S` 1, with `E` and `H` shifting
   together. Then the helper removes the board **slowly**.
5. **Descend** slowly to about 1 m, so the laser sees the floor again, and hover 10 s. Note how long `S`
   takes to become 1.
6. Land normally. Paste into `logs/log-13.txt`.

### Test 13 result ( `logs/log-13.txt`, 24 Sep 2026 14:23 )

**1403 records over 153 s; armed 5-137 s.** Two climbs to ~2.1 m ( `S` 0 ), with handover at `mm` 1604 /
1607 and `E` continuous ( 158 → 161, 157 → 161 ); two returns during the descents, shift +7 / −15 cm
with `E` and `H` together; hover at ~1.2 m; three box hold-offs with re-base at ~1 m ( 122-134 s );
normal landing.

**The guard steps were not captured:** during both baro legs every valid reading was the floor
( 1.76-2.09 m ), so the board was never in the laser beam. The 25 cm exit band is therefore not
flight-tested. **Task 9 is closed on the log-12 evidence** ( user ): there the guard held twice and its
exit returned correctly, with `E` and `H` together. It was only slower, which the wider band addresses
by construction. The band change is L1x-only and reviewed.
