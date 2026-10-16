# Take-off / land command fix - Testing

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

## 1. Validation versions ( task 4 )

One file, [test/PlutoPilot_fix_validation.cpp.txt](test/PlutoPilot_fix_validation.cpp.txt). Set `#define TEST_CASE`,
copy it over the root `PlutoPilot.cpp`, then build and flash PRIMUS_X2_v1. All six cases compile with no warnings
( 16 Oct 2026 ). Case 1 is in the root `PlutoPilot.cpp` now. **Restore the template before committing**
( `git checkout PlutoPilot.cpp` ).

| `TEST_CASE` | Flight | `onLoopStart ( )` | Test command |
|---|---|---|---|
| 1 | A | `Command_Arm ( )`, then `Command_TakeOff ( 100 )` | `Command_Land ( 5 )` 12 s after arming |
| 2 | B | `Command_TakeOff ( 100 )` | `Command_Land ( 5 )` at 12 s |
| 3 | D | `Command_TakeOff ( 100 )` | `Command_Land ( 5 )` at 8 s, **`Command_TakeOff ( 100 )` re-sent at 9 s** |
| 4 | E | `Command_TakeOff ( 100 )` on **every** `onLoopStart ( )` while armed ( the drone is armed by the switch ) | `Command_Land ( 5 )` 12 s after **Dev Mode** came on |
| 5 | F | `Command_TakeOff ( 100 )` | **default `Command_Land ( )`** at 12 s |
| 6 | F2 | `Command_TakeOff ( 100 )` | **default `Command_Land ( )` at 1 s**, during the climb ( optional ) |

- Cases 1, 2, 3, 5 and 6 send the take-off **once per boot**, while disarmed. Case 4 re-sends it on every restart,
  as the user's original code did: the firmware must ignore the repeats ( once per arm, never in a landing ).
- **Why once per boot.** Before task 11, a `Command_TakeOff` sent while the drone was disarmed and could not arm ( after
  a landing, with the ARM switch still on ) stayed queued and armed at the next ARM-on. Task 11 now drops it after
  2 s of refused arming, or at a real Dev Mode off. A take-off queued with ARM off still waits while Dev Mode is on,
  so the tests keep the once-per-boot send.
- The schedule runs from the firmware's `arm_time`, except case 4, which runs from Dev Mode on. It restarts on every
  arm.

### Log line

The same as the check topic ( [../takeoff-api-ground-hover/TESTING.md](../takeoff-api-ground-hover/TESTING.md) §1 ),
about 119 B per tick: `t n F C Zd Ze Vz Mo Tp`.

- **`F`**: four flag digits: armed, ALT HOLD, ground idle ( `isThrottleStickArmed` ), height set. Values print without
  leading zeros: disarmed with ALT HOLD on is `F:100`.
- **`C`**: `current_command * 10 + status`. Command: 0 NONE, 1 TAKE_OFF, 2 LAND. Status: 0 FINISHED, 1 RUNNING.
  So `C:21` is a landing, `C:20` after it, and `C:11` a queued take-off.

Markers, each on its own tick:

- `S:<ms>`: `onLoopStart ( )` ran, with the time since arming at that moment, or −1 if disarmed. More than one `S`
  means Dev Mode restarted on a Wi-Fi flicker.
- `X:1`: the test sent `Command_Land`. `X:2`: the test re-sent `Command_TakeOff` ( case 3 ). The data line
  printed on the send tick already shows the result, so the `X` line comes **one tick after** its effect.

The log starts when Dev Mode comes on. Read heights relative to the first line's `Ze`. Time landings from the
first `C:21` line.

### Common procedure

On a flat floor in open space: connect the app, turn ALT HOLD on, and start the PlutoMonitor log. Keep a hand on
the ARM switch: it disarms at any throttle ( kill switch ). Hands off the sticks once Dev Mode is on. After the
disarm, keep Dev Mode on for 5 s, then stop the log. Save the log as `log-N.txt` in this folder ( N = the flight
number in the table below ). **Power-cycle the drone between flights.** That also re-arms the once-per-boot send.
If the last lines show `C:11` after the disarm, a take-off is queued: power-cycle before turning ARM on again.

- Cases 1, 2, 3, 5, 6: stick **centred**. ARM on, then Dev Mode on.
- Case 4: stick **low** ( bottom ), then ARM on: the drone arms with the motors at idle. Move the stick back to
  **centre, at or below 1500, and never above it**: the idle stays on. Wait **4-10 s**, so the 3 s window has
  expired and only the idle path can let the take-off through. Then turn Dev Mode on.

### Expected result

| # | Case | Pass | Fail |
|---|---|---|---|
| 1 | A | `S:-1` ( the marker value is taken before `Command_Arm ( )` arms the drone in that pass ). `F:1101` from the first data line. `Zd` ramps to about +100 in about 2.5 s, and `Ze` holds 80-100 cm. Same as check log-2 | `F:1100` with `Zd` flat ( the old fault ) |
| 2 | B | `S:-1`. The same as check log-2: `F:1101`, climb to 84-94 cm | No climb, or a different lift-off |
| 3 | D | `C:21` from the land tick. After `X:2`, **`C` stays `21`**. The drone descends and disarms on the floor ( `F:100`, `C:20` ) | `C:11` on the line before `X:2`, then `C:0`; or the drone sits armed on the floor. **Kill it with the ARM switch if it sits armed on the floor for more than 3 s** |
| 4 | E | `S:` between 4000 and 10000. The first data line shows `F:1101` ( idle released, height set ). `Mo` rises from idle, and `Zd` ramps to about +100. Later `S:` restarts do not move `Zd` | `F:1110` that stays, with `Mo` at idle |
| 5 | F | From the first `C:21` line: a descent of −75 to −87 cm/s, and the disarm at least 1.2 s later, with `Ze` no more than 15 cm above the floor reference. The baro may read up to about 20 cm **below** it in the downwash | `Ze` 30 cm or more above the floor at the disarm. 15-30 cm is inconclusive |
| 6 | F2 | The first `C:21` during the climb. The drone turns round, descends and disarms at least 1.2 s after that line, with `Ze` no more than 15 cm above the floor | `Ze` 30 cm or more above the floor at the disarm. 15-30 cm is inconclusive |

An impact on the floor ( `|accZ| > LAND_IMPACT_ACC` ) disarms the drone at once, with no time floor. In cases 5 and 6,
a disarm earlier than 1.2 s after the first `C:21` line counts as a pass only if `Ze` is at the floor.

**Invalid runs ( repeat the flight ):**

- `Tp` on the first data line is outside 1460-1540 ( case 4: 1460-1500 ). The alt-hold deadband is 40: an
  off-centre stick makes the firmware skip the goal, and that looks exactly like the fault under test.
- Cases 1, 2, 3, 5, 6: `S:` shows anything but −1, meaning the drone was already armed. Another way to see this: the third digit of `F` is 1.
- Case 4: `S:` outside 4000-10000, or the first data line shows `F:1100`. That means the idle had already ended:
  the stick went above 1500 at some point.

**Safety:** case 6 tests the case most likely to disarm in the air: a landing that starts in the climb, when the
reviewer's estimated margin is only 0.3-0.4 s. At that point the drone is about 35-50 cm up. Fly it over a soft
surface. It is optional.

## 2. Results ( task 6, 16 Oct 2026, PRIMUS_X2_v1, FW 3.13.0 + this topic, `Rx_ESP` )

Logs [log-1.txt](log-1.txt) … [log-6.txt](log-6.txt). Every run is valid:

- `Tp` 1499-1500 on every armed line;
- the expected `S:` value ( −1, or 9368 in case 4 );
- one `S:` per log, so no Wi-Fi restarts.

Heights are relative to the first line's `Ze`.

| # | Case | Key numbers | Verdict |
|---|---|---|---|
| 1 | A: Arm + TakeOff | `F:1101` from the first line. `Zd` 3 → 52 → 100 in 2 s, then 102. Hover 86-103 cm. Land at 12.0 s, disarm 2.34 s after the first `C:21` with `Ze` −14 ( on the floor ) | **pass: the reported fault is fixed** |
| 2 | B: TakeOff alone | `F:1101`, `Zd` 1 → 100 in 2 s, hover 80-97 cm ( check log-2: 84-94 ). Disarm 2.34 s after `C:21`, `Ze` −21. 11 records lost on the link ( `n` 42 → 53 ) | **pass: unchanged** |
| 3 | D: re-send during the landing | `X:1` at 7.9 s ( `C:21` ), `X:2` at 8.95 s. **`C` stays `21`**, with no `C:11` or `C:0`. The descent runs on at −66 to −77 cm/s; disarm at 10.27 s ( 2.34 s after `C:21` ), `Ze` −20 | **pass: D1 is fixed** |
| 4 | E: ARM-switch idle | `S:9368` ( Dev Mode 9.4 s after the arm, so the idle path, not the window ). The first line is `F:1101` with `Mo` 1560: the idle was released and the height set. `Zd` ramps 0 → 100 in 3 s, hover 86-100 cm. Disarm 2.74 s after `C:21`, `Ze` −42 | **pass: D3 is fixed** |
| 5 | F: default `Command_Land ( )` | Land at 12.0 s from 95 cm; descent down to −104 cm/s ( estimate ). Disarm 1.73 s after the first `C:21`, last armed `Ze` −12 ( on the floor ) | **pass: D2 holds from hover** |
| 6 | F2: default land in the climb | `X:1` at 0.92 s, `Ze` 19 cm, `Vz` +28. The climb carries on to 31 cm, then the drone turns round and descends. Disarm 1.52 s after `C:21`, last armed `Ze` +9, `Vz` −69 | **pass: disarmed at the floor** |

Notes:

- In cases 1-4, `Command_Land ( 5 )` disarmed 2.34-2.74 s after the first `C:21`, before the ramp can reach
  `LAND_CONFIRM_THROTTLE` ( 2.5 s from 1300 ). So the disarm was the impact check at touchdown, as in the check-topic
  logs. In cases 5 and 6 ( start 1200 ) the disarm came 1.5-1.7 s in, after the 1 s floor, at the floor.
- In case 6 the last armed line shows `Vz` −69 at `Ze` +9 cm. That is a touchdown with speed, which the impact check
  disarms at once. It is not a disarm in the air: the drone was within baro noise of the floor.
- The baro reads 12-42 cm **below** the floor reference at the disarm ( downwash ). That is within the one-sided
  test, and the same effect as in the check-topic logs.
- No Wi-Fi restart happened, so case 4's ignore-the-repeat path and task 11's drops were not exercised in flight.
  They rest on the code review ( tasks 2 and 11 ).

## 3. Flight G: take-off with the stick off-centre ( task 14 )

`TEST_CASE 7` in [test/PlutoPilot_fix_validation.cpp.txt](test/PlutoPilot_fix_validation.cpp.txt). It is the same code
path as case B: `Command_TakeOff ( 100 )` once per boot while disarmed, and `Command_Land ( 5 )` 12 s after arming.
Build ( 16 Oct 2026 ): no warnings; case 7 is in the root `PlutoPilot.cpp`. Log to [log-7.txt](log-7.txt).

**Procedure.**

1. Power-cycle. On the floor, ALT HOLD on. Start the log.
2. Hold the throttle stick **above centre**, about 1600 ( clearly above 1540 ), and keep it there.
3. ARM on, then Dev Mode on. **The drone must stay disarmed** ( motors off ).
4. After about 5 s, centre the stick ( 1460-1540 ). The drone arms and climbs about 100 cm.
5. Hands off; it lands itself 12 s after arming. Keep Dev Mode on for 5 s after the disarm.

**Pass.**

- While the stick is held up: `F:100` ( disarmed ), `C:11` ( take-off queued and running ), `Tp` above 1540.
- When `Tp` comes into 1460-1540: `F:1101` on that line or the next, then `Zd` ramps to about +100.
- The landing disarms on the floor, as in cases 1-4.

**Fail.**

- `F` goes to `1xxx` while `Tp` is above 1540: armed off-centre, the old behaviour.
- `C` goes to `0` before arming: the take-off was dropped.

**Invalid.** `Tp` at or below 1540 on the first lines: the stick was not held up. Repeat the run.

**Gate ( task 14 ).** Clean build with the template user code: text 108268, data 1280, bss 16760. Against 3.13.0 that is
**+448 B** of flash and +4 B of RAM. The budget was raised from 400 to 500 B by the user after tasks 12, 13 and 15
added 120 B.

### Result ( 16 Oct 2026, log-7 )

[log-7.txt](log-7.txt): 332 records, no gaps, one `S:-1`, so the run is valid.

| Phase | Lines | Verdict |
|---|---|---|
| Stick held up, 0-12.7 s | `F:100 C:11`, `Tp` 1707-1710, `Mo` 1000 on every line: disarmed, take-off queued. Not dropped, because `OK_TO_ARM` was set, so it was a plain wait | **pass: no arm off-centre** |
| Stick centred, 12.71 s | First line with `Tp` 1540 ( the deadband edge, which counts as centred ): `F:1101 C:0`, armed with the height set on that line | **pass** |
| Climb and hover | `Zd` 1 → 78 → 101 in about 2 s; `Ze` 63 → 98 cm, hover 80-98 cm | **pass** |
| Landing | `C:21` at 24.7 s, disarm 2.24 s later with `Ze` −15 ( on the floor ) | **pass** |

Task 12 ( `Command_Land` ignored during a flip ) and task 15 ( the flip watchdog ) are verified by code review only, as
planned.
