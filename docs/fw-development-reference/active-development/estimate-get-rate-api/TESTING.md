# Estimate_Get Rate API - Testing

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

## Test 1: bench rotation check of `Estimate_Get ( Rate, ... )` ( task 3 )

**Build.** `PRIMUS_X2_v1`, the task 1 fix plus the temporary instrumentation in
[PlutoPilot.cpp](../../../../PlutoPilot.cpp). Hex: `Build/PRIMUS_X2_v1/AltX_PRIMUS_X2_v1_3.15.3.hex`, built
4 Nov 2026 at 23:23 ( still named 3.15.3: the version is bumped at commit ).

**Log line** ( about 110 bytes per tick with framing at the default 10 Hz, under the 130-byte budget ):

| Field | Source | Unit |
|---|---|---|
| `t` | `millis ( )` | ms, drone time |
| `n` | tick counter, +1 per `plutoLoop ( )` | count; a step of 2 away from a START is a lost record |
| `x` / `y` / `z` | `Estimate_Get ( Rate, X / Y / Z )` | deg/s x10 ( roll, pitch, yaw ) - the API under test |
| `r` / `p` | `Estimate_Get ( Angle, AG_ROLL / AG_PITCH )` | deci-degrees |
| `h` | `Estimate_Get ( Angle, AG_YAW )` | degrees, 0..359 |
| `START n:` | `onLoopStart ( )`, own tick | Developer Mode ( re )start |

**Sequence.** Props off, disarmed, app connected, Developer Mode on. Turn **slowly and smoothly, about 4 s per full
turn**, so the 10 Hz samples follow the motion. Mark the start of each turn against something fixed ( table edge ).

1. Flat on the table, untouched: **20 s**.
2. **Yaw:** flat on the table, turn it **once to the right ( clockwise seen from above )** a full 360 deg, back to the mark. Still 10 s.
3. **Roll:** hold the board level, roll it **once right side down** a full 360 deg about the front-back axis, back to level. Still 10 s on the table.
4. **Pitch:** hold the board level, pitch it **once nose down** a full 360 deg about the left-right axis, back to level. Still 10 s on the table.
5. Save the log into `logs/log-1.txt` ( its own name, `logs.txt` gets overwritten ). Note anything unusual ( a turn
   that went past the mark, a pause in the middle ).

**Pass if all hold.**

| Check | Expected |
|---|---|
| Still on the table ( steps 1, and the pauses ) | each of `x`, `y`, `z` within +/-20 ( 2 deg/s ) |
| One turn integrates to a full circle | sum of rate x dt / 10 over each turn = 360 +/- 30 deg on that axis ( an 8x scale error gives about 45 ) |
| Only the turned axis moves | the other two rates stay small during each turn ( hand wobble aside ) |
| Sign matches the angle | during the yaw turn `h` changes the same way `z` says; during the first quarter of roll / pitch `r` / `p` move the same way as `x` / `y` |
| Old bug gone | no axis sits at a constant ( 75 or any other ) value |

The sign of each axis is recorded here and goes into the header and wiki in task 4.

**Analysis method** ( from the instrumentation review, so the integral is not biased ):

- **Remove the rest offset first.** The gyro zero is set once at boot, so up to +/-20 ( 2 deg/s ) of offset stays in
  every sample, and 6-10 s of still time inside a window would add 12-20 deg. Subtract each axis's mean over the step 1
  still period before integrating.
- **Window each turn by the motion,** from where the rate leaves the rest band to where it returns, not by the 10 s
  pauses.
- **Integrate over the real `t` steps,** not a fixed 0.1 s ( spacing is 100 ms plus up to one 3.5 ms main loop ).
- **Redo any turn with a gap inside it:** a `START` or a `t` step over about 150 ms means `plutoLoop ( )` did not run
  ( a late Wi-Fi RC frame restarts the user code, `mw.cpp:1170-1176` ) and samples are missing; `n` does not show it.
- **Unwrap the angles** before comparing: `h` wraps between 359 and 0, `r` at +/-1800 halfway through a roll, and `p`
  only goes out to about +/-90 deg and back over a full pitch turn, so use the first quarter for the roll / pitch sign.
- **Expect hand-held roll / pitch slightly under 360:** one gyro axis integrates 360 x cos ( wobble ), about 354 for a
  10 deg wobble off the turn axis. Within the +/-30 window that is not a failure.

**Fails if.** A rate stays constant ( old firmware flashed, check the hex time ), a turn integrates to about 45 deg or
2880 deg ( wrong scale ), or the axes are swapped ( a yaw turn shows on `x` or `y` ).

**Cannot show.** Rates above what a hand can turn ( the int16 range allows +/-1998 deg/s ), or behaviour in flight.

### Results ( log-1, 5 Nov 2026, **PASS** )

Log: [logs/log-1.txt](logs/log-1.txt), 122 s, 1135 records, the 23:23 instrumentation hex. Analysed by script with the
method above ( rest mean subtracted, motion windows, trapezoid over real `t` ).

**Log integrity.** 1068 steps of 100-110 ms; 66 steps of 200-217 ms, every one with `n` +2, so each is exactly one
missing record, not a stall: 26 are the `START` ticks ( Wi-Fi restarts of the user code, about one per 4.7 s ) and 40
are records lost on the way to PlutoMonitor ( 3.5 % ). The longest step is 217 ms. **Deviation from the method:** every
turn holds 3-5 such one-record gaps, so a redo would never get a clean turn; they are bridged by the trapezoid ( linear
over 200 ms ) instead, and the spread between the trapezoid and a sample-hold sum ( below ) bounds that error.

**At rest** ( steps 1, 0-26 s, 249 samples; and all pauses ):

| Axis | Mean | SD | Range, step 1 | Range, all pauses | Limit |
|---|---|---|---|---|---|
| `x` ( roll ) | -0.86 | 0.88 | -3 .. 1 | -4 .. 2 | +/-20 |
| `y` ( pitch ) | -1.77 | 0.93 | -4 .. 1 | -5 .. 1 | +/-20 |
| `z` ( yaw ) | -0.67 | 0.84 | -3 .. 2 | -4 .. 2 | +/-20 |

All within +/-0.5 deg/s. No axis sits at 75 or any other constant: the old bug is gone.

**One turn per axis** ( degrees; window from where the turned axis leaves +/-20 of its rest mean to where it returns ):

| Turn | Window | Samples / gaps | Turned axis, trapezoid | Sample-hold | Other two axes | Peak rate | Pass |
|---|---|---|---|---|---|---|---|
| Yaw, clockwise from above | 27.4-37.0 s ( 9.7 s ) | 91 / 5 | `z` **-353.8** | -358.2 | `x` -1.3, `y` -0.3 | `z` -2225 ( 223 deg/s ) | yes |
| Roll, right side down | 101.6-105.9 s ( 4.3 s ) | 39 / 4 | `x` **+386.7** | +381.0 | `y` +7.6, `z` -12.0 | `x` 2346 ( 235 deg/s ) | yes |
| Pitch, nose down | 112.9-116.9 s ( 4.0 s ) | 37 / 3 | `y` **+356.9** | +355.3 | `x` +13.5, `z` -21.7 | `y` 2495 ( 250 deg/s ) | yes |

All three are within 360 +/- 30. The scale is right: the stale `1 / 131` scale would give about 45 deg, a whole-deg/s
unit 36 deg. The rest offset changes each integral by less than 1 deg. Roll reads 27 deg high, though `r` came back to
its start ( -2 to -3 deci-degrees ), so the craft did turn exactly once. The turns were done faster than planned
( peaks 220-250 deg/s, 1-3 s of real motion ), so 10 Hz samples miss the shape of each burst, and the hand wobble on the
other two axes reached 100-120 deg/s. The rate itself is not suspect; the 10 Hz log is the limit of this check.

**Sign of each axis** ( from the first samples of each turn ):

| Turn | Rate | Angle | Sign |
|---|---|---|---|
| Roll right side down | `x` positive ( 371, 1491, 2346 ) | `r` rises ( -2, 7, 247, 425 ) | `x` > 0 = rolling right, same sign as `Angle AG_ROLL` |
| Pitch nose down | `y` positive ( 596, 971, 1169 ) | `p` rises ( 11, 51, 115, 213 ) | `y` > 0 = pitching nose down, same sign as `Angle AG_PITCH` |
| Yaw clockwise from above | `z` negative ( -38, -68, -216 ) | `h` rises ( 335, 345, 15, 36 ... 339 ) | `z` < 0 = turning right, **opposite** to the heading, which rises clockwise |

Roll right and pitch nose down positive is the Cleanflight stick convention ( `errorAngle = rcCommand - inclination` ).
Yaw is the raw gyro sign ( decision D2 ): a right turn reads negative while `Angle AG_YAW` and
`DesiredAngle_setRelative ( AG_YAW, + )` count clockwise as positive. Task 4 states this in the header and the wiki, so
user code negates `z` when it compares it with the heading.

**Verdict.** Every check in the pass table holds: rest within +/-20, each turn 360 +/- 30 on its own axis, no constant
value, and signs recorded. `PlutoPilot.cpp` restored to HEAD; the clean hex was rebuilt ( gate below ).

**Clean rebuild** ( 5 Nov 2026, `PROJECT=AltX driver.sh --gate PRIMUS_X2_v1` ): no new `src/` warnings; text / data / bss
122752 / 1316 / 19728 B, identical to HEAD; flash 121.1 KB / 256 KB, RAM 20.5 KB / 40 KB.
`Build/PRIMUS_X2_v1/AltX_PRIMUS_X2_v1_3.15.3.hex` ( 14:09 ) has no instrumentation.

**Side observation, not in scope.** With the app on Wi-Fi the user code restarted 26 times in 122 s
( `onLoopFinish` / `onLoopStart` on an RC frame over 200 ms late, `mw.cpp:1170-1176` ), and 3.5 % of the log records
never reached PlutoMonitor. Both were already known and are unchanged by this topic; worth noting for user code that
treats `onLoopStart ( )` as a one-time event.
