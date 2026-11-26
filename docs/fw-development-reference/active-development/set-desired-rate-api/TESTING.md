# Set desired rate API - Testing

[README](README.md) · [TASKS](TASKS.md) · [INVESTIGATION](INVESTIGATION.md) · [TEST_CODE](TEST_CODE.md)

## Method

Test code: [TEST_CODE.md](TEST_CODE.md) ( copied into `PlutoPilot.cpp` during the topic ). It calls `setDesiredRate ( )`
in `plutoInit ( )` and then at fixed times after Developer Mode is first switched on, and logs at 10 Hz
( ~110 B per tick ):

| Field | Meaning |
|---|---|
| `t` | `millis ( )` |
| `gR` / `gP` / `gY` | `getDesiredRate ( AG_ROLL / AG_PITCH / AG_YAW )` |
| `lR` / `lP` / `lY` | live `currentControlRateProfile->rates [ ]`: the rate the PID really uses |
| `A` | 1 = armed |
| `F` | `flipState` ( 0 = no flip ) |

Markers on their own tick: `LOOP START` ( Developer Mode start, also a Wi-Fi restart of the user code ) and `STEP <n>`
( that step's `setDesiredRate` calls ran on that tick; the next data line is the first one after them ), and
`STEP 2 SKIPPED ARMED` ( the out-of-range step is never run while armed; if you see it, you armed before 15 s ).

| Step | Time after first Dev Mode on | Set ( roll / pitch / yaw ) | Craft |
|---|---|---|---|
| S0 | boot ( `plutoInit` ) | 40 / 50 / 10 | disarmed |
| S1 | 5 s | 40 / 50 / 10 | disarmed |
| S2 | 15 s | 300 / -5 / 400 ( out of range ) | disarmed, bench only |
| S3 | 20 s | 40 / 50 / 10 | disarmed |
| S4 | 45 s | 30 / 40 / 15 | armed on the bench ( props off ) |

Stock rates: roll / pitch 75 ( after the disarmed reset ), yaw 0.

## Procedure ( bench, props off, no flying )

Decision ( 2026-11-15 ): the app cannot keep Developer Mode on and flip at the same time, and the log only runs in
Developer Mode, so task 6 is a bench test. It covers the reported bug ( the rate reset to 75 every disarmed loop ), the
clamp and the hold across arming. The two in-flight items are not covered here; see **Not covered** below.

1. **Props off.** Flash `AltX_PRIMUS_X2_v1_3.15.8.hex`. Drone on the bench, PlutoMonitor connected.
2. Switch Developer Mode on and **leave it on**. Do not arm yet.
3. Watch the lines through `STEP 1` ( 5 s ), `STEP 2` ( 15 s, the out-of-range values ) and `STEP 3` ( 20 s ).
4. After `STEP 3`, arm with the ARM switch, throttle down. Hold the drone, wait about 10 s, disarm, wait about 10 s.
5. Wait for `STEP 4` ( 45 s ) and about 5 s more.
6. Developer Mode off. Save the log as `logs/log-2.txt`.
7. **Control run:** set `TEST_SET_RATES` to `false` in `PlutoPilot.cpp`, rebuild, flash, then repeat steps 2-3 and 6 only
   ( about 25 s, never armed ) as `logs/log-3.txt`.

## Expected

`gR gP gY` is `getDesiredRate ( )`, `lR lP lY` the rate the PID uses.

| Point in the log | Pre-fix ( FW 3.15.8 ) | Post-fix ( pass ) |
|---|---|---|
| S0 / S1, disarmed | `lR` `lP` = 75 / 75 on every line; `gR` `gP` `gY` near 0 with sticks centred ( PID setpoint ) | `gR` `gP` `gY` = `lR` `lP` `lY` = 40 / 50 / 10 on every line, no 75 |
| S2, disarmed | `lR` `lP` = 75 / 75; `lY` = 144 ( 400 narrowed to `uint8_t` ) | 100 / 0 / 255 ( clamped ) |
| S3, disarmed | 75 / 75 / 10 | 40 / 50 / 10 |
| Armed ( `A` = 1 ), then disarmed | 75 / 75 after the disarm | 40 / 50 / 10 throughout, and after the disarm |
| S4 | `lR` `lP` back to 75 / 75 at the next disarmed loop | 30 / 40 / 15, held |
| Control run ( `TEST_SET_RATES = false` ) | 75 / 75 / 0 | 75 / 75 / 0, unchanged |

`F` must stay 0 for the whole bench run: no flip is involved.

## Not covered by the bench test

- **The rate reaching the PID** ( RATE-mode stick response ). Verified only by reading `pid.cpp:793` ( yaw, every mode )
  and `:812-813` ( roll / pitch outside ANGLE mode ).
- **The flip restore and flip behaviour.** Verified by the task-3 path-by-path reading and the reviewer: the flip's own
  75 / 110 are untouched, and with no user rate every exit writes the same 75 as before. Not flight-validated.

## Results

### log-1 ( pre-fix baseline, FW 3.15.8 )

Skipped ( 2026-11-15, user's choice ): the causal chain is proven in the code ( INVESTIGATION.md ). `logs/log-1.txt` stays
empty; the post-fix run ( task 6 ) uses `logs/log-2.txt`.

### log-2 ( post-fix bench run, FW 3.15.8 + this topic ): PASS

[logs/log-2.txt](logs/log-2.txt), 15 Nov 2026, bench, props off, PRIMUS_X2_v1. 790 records over 84 s, all complete
( ~110 B per tick fits ), no gap in `t` over 250 ms. 18 `LOOP START` markers: Developer Mode restarts on the Wi-Fi link,
each shorter than one tick; the step timeline carried on through them as designed. No `STEP 2 SKIPPED ARMED`.

| Segment ( from the markers ) | `A` | Lines | `gR gP gY` = `lR lP lY` | `F` |
|---|---|---|---|---|
| S0, after `plutoInit` | 0 | 46 | 40 / 50 / 10 on every line | 0 |
| S1 ( 5 s ) | 0 | 95 | 40 / 50 / 10 | 0 |
| S2 ( 15 s ), set 300 / -5 / 400 | 0 | 48 | **100 / 0 / 255** ( clamped ) | 0 |
| S3 ( 20 s ) | 0 | 186 | 40 / 50 / 10 | 0 |
| S3, armed on the bench | 1 | 47 | 40 / 50 / 10 | 0 |
| S4 ( 45 s ), set while armed | 1 | 128 | 30 / 40 / 15 | 0 |
| S4, after the disarm | 0 | 240 | **30 / 40 / 15**, held 25 s | 0 |

- **The T16 bug is gone:** no line has `lR` or `lP` = 75. Before the fix `flip ( false )` reset both to 75 on every
  disarmed loop; here 517 disarmed lines hold the set value.
- **The getter matches:** `getDesiredRate ( )` equals the live rate on all 790 lines.
- **Clamp:** 300 / -5 / 400 became 100 / 0 / 255.
- **Hold across arm and disarm:** the S3 value held through arming; the S4 value, set while armed, held after the
  disarm. The pilot stayed armed through S4 rather than disarming before it ( the procedure said arm about 10 s ); that
  also covers a set while armed, which the procedure did not.
- `F` = 0 throughout: no flip was involved.

### log-3 ( control run, `TEST_SET_RATES = false` )

Not run ( 15 Nov 2026, user's choice ). The no-call default ( 75 / 75 / 0 ) rests on the task-3 reading: with no user
rate `userRollPitchRate ( )` returns 75, so every path writes what it wrote before ( reviewer, path by path ).
