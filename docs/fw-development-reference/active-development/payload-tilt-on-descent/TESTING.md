# Payload Tilt On Descent - Testing

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

## Diagnostic log ( task 1 )

**Removed in task 12 ( 27 Oct 2026 ).** Kept here as the record of what log-1..7 contain. Printed by `plutoLoop ( )`
in `PlutoPilot.cpp` at the default user loop of 10 Hz, only in Developer Mode with a live RC link.

| Field | Source | Unit / meaning |
|---|---|---|
| `START` | `onLoopStart ( )` flag, printed alone on the next tick | Developer Mode ( re )start; the line has no other field |
| `t` | `millis ( )` | ms since boot |
| `Th` | `RcData_Get ( RC_THROTTLE )` = raw `rcData [ THROTTLE ]` | 1000-2000; THROTTLE_LOW ( I-term reset ) below `mincheck` 1100 |
| `R`, `P` | `Estimate_Get ( Angle, AG_ROLL / AG_PITCH )` | decidegrees ( 50 = 5.0° ) |
| `IR`, `IP` | `pidGetGyroITerm ( 0 / 1 )` = `errorGyroI >> 13` | `pidRewrite` rate I-term, motor counts, bounded by `GYRO_I_MAX` 256 |
| `RR`, `FR`, `RL`, `FL` | `motor [ 0..3 ]` | motor command, `mixerQuadX` order: rear-right, front-right, rear-left, front-left |
| `S` | `FlightStatus_Check ( FS_ARMED )`, `FlightMode_Check ( ATLTITUDEHOLD / ANGLE )` | bits: 1 armed, 2 ALT_HOLD, 4 ANGLE ( 7 = armed, ALT_HOLD, ANGLE ) |

**Byte budget.** Each `Monitor_Print ( tag, int )` costs 5 bytes of framing plus the tag, the sign and the
digits; the closing `Monitor_Println` adds 5 bytes. Reviewer's count against `Debugging.cpp:215-277`:
about 110-121 bytes per tick in a normal flight, **123 bytes worst case** ( 7-digit `t`, every value
negative and at full width ), budget ~130. The `START` tick is 14 bytes. There is no room for another field.

**Reading the log ( reviewer notes, task 1 ).**

- `IR` / `IP` are *pinned near 0*, not exactly 0, while `Th` < 1100: the reset runs inside `processRx ( )`
  ( each RC frame or every 20 ms ), and `pidRewrite` adds to the I-term on every PID cycle in between.
  Values that never leave a few counts while the stick is down are the reset signature.
- `Th` is the raw `rcData` throttle the THROTTLE_LOW test reads, except during a firmware `LAND`, where
  `annexCode ( )` has already replaced it with `landThrottle` before `plutoLoop ( )` runs. Ignore `Th` on
  LAND ticks; the stick-down tests here never use LAND.
- `IR` / `IP` mean the `pidRewrite` I-term only while `pidController` is 1 ( the default ). Confirm it on
  the test board before flying.

**What a confirmation looks like.** During a full stick-down `Th` is below 1100, `IR` / `IP` sit at 0, and
`R` or `P` grows toward the payload side while the motors on that side rise above the others. During a
partial stick-down `Th` stays above 1100, `IR` / `IP` hold a non-zero value and `R` / `P` stay near 0.

### Code to port to `4657cbf` ( task 3 )

`4657cbf` has the same `PlutoPilot.cpp` hooks, `Estimate_Get`, `FlightMode_Check`, `FlightStatus_Check`
and `pidResetErrorGyro ( )` at `pid.cpp:92`. Apply the same three edits there.

`src/main/flight/pid.cpp`, after `pidResetErrorGyro ( )`:

```c
// TEMPORARY DIAGNOSTIC ( active-development/payload-tilt-on-descent, task 1 ): remove before release.
// The pidRewrite rate I-term in motor counts ( ITerm = errorGyroI >> 13, bounded by GYRO_I_MAX ).
int16_t pidGetGyroITerm(uint8_t axis)
{
    if (axis >= 3) {
        return 0;
    }
    return (int16_t)(errorGyroI[axis] >> 13);
}
```

`src/main/flight/pid.h`, after `void pidResetErrorGyro(void);`:

```c
int16_t pidGetGyroITerm(uint8_t axis);    // TEMPORARY DIAGNOSTIC ( payload-tilt-on-descent ), remove before release
```

`PlutoPilot.cpp`, after `#include "PlutoPilot.h"`:

```cpp
// TEMPORARY DIAGNOSTIC ( active-development/payload-tilt-on-descent, task 1 ): remove before release.
// Firmware internals read by the diagnostic line below.
extern "C" {
extern int16_t motor[];                      // flight/mixer.h: [0] REAR_R, [1] FRONT_R, [2] REAR_L, [3] FRONT_L
int16_t pidGetGyroITerm ( uint8_t axis );    // flight/pid.cpp: rate I-term, motor counts
uint32_t millis ( void );
}

static bool diagStartMarker = false;
```

`PlutoPilot.cpp`, in `onLoopStart ( )`:

```cpp
  diagStartMarker = true;    // TEMPORARY DIAGNOSTIC: Developer Mode ( re )start marker
```

`PlutoPilot.cpp`, the body of `plutoLoop ( )`:

```cpp
  // TEMPORARY DIAGNOSTIC ( payload-tilt-on-descent ): ~121 bytes per tick at 10 Hz ( budget 130 ).
  // Th is raw rcData throttle ( THROTTLE_LOW below 1100 resets the I-term ); R / P in decidegrees;
  // IR / IP rate I-term in motor counts; S bits: 1 armed, 2 ALT_HOLD, 4 ANGLE.
  if ( diagStartMarker ) {
    diagStartMarker = false;
    Monitor_Println ( "START" );    // own tick, so restarts are visible
    return;
  }

  int state = 0;
  if ( FlightStatus_Check ( FS_ARMED ) ) {
    state += 1;
  }
  if ( FlightMode_Check ( ATLTITUDEHOLD ) ) {
    state += 2;
  }
  if ( FlightMode_Check ( ANGLE ) ) {
    state += 4;
  }

  Monitor_Print ( "t", ( int ) millis ( ) );
  Monitor_Print ( "Th", RcData_Get ( RC_THROTTLE ) );
  Monitor_Print ( "R", Estimate_Get ( Angle, AG_ROLL ) );
  Monitor_Print ( "P", Estimate_Get ( Angle, AG_PITCH ) );
  Monitor_Print ( "IR", pidGetGyroITerm ( 0 ) );    // FD_ROLL
  Monitor_Print ( "IP", pidGetGyroITerm ( 1 ) );    // FD_PITCH
  Monitor_Print ( "RR", motor [ 0 ] );
  Monitor_Print ( "FR", motor [ 1 ] );
  Monitor_Print ( "RL", motor [ 2 ] );
  Monitor_Print ( "FL", motor [ 3 ] );
  Monitor_Println ( "S", state );
```

## Build ( task 1 )

| | |
|---|---|
| Target | PRIMUS_X2_v1, FW 3.15.1 |
| Warning gate | passed: no new warnings in `src/`; `PlutoPilot.cpp` 0 warnings |
| Flash | 121.9 KB / 256 KB ( 47 % ) |
| RAM | 20.5 KB / 40 KB ( 51 % ) |
| Hex | `Build/PRIMUS_X2_v1/AltX_PRIMUS_X2_v1_3.15.1.hex` |
| Note | PlutoIDE flashes `<project_name>_<TARGET>_<FW_Version>.hex` with `project_name` from `plutoide.ini` ( `AltX` ). Build with `PROJECT=AltX` so the driver's hex carries that name. |

## Flights on HEAD ( task 2 )

**Build:** `Build/PRIMUS_X2_v1/AltX_PRIMUS_X2_v1_3.15.1.hex` ( HEAD `a81c180` + the task 1 diagnostics ).

**What would confirm the leading cause.** In every flight with the payload, the full stick-down has
`Th` below 1100, `IR` / `IP` pinned near 0, and `R` or `P` moving toward the payload side, while the
partial stick-down keeps `Th` above 1100, a non-zero I-term on the payload axis and `R` / `P` near 0.
The no-payload flight shows no lean in either stick-down. **What would refute it:** a lean during the
partial stick-down, a lean with `IR` / `IP` away from 0, or `Th` never going below 1100 at full stick.

| Log | Payload | Expected lean in a full stick-down |
|---|---|---|
| `logs/log-1.txt` | under 10 g on the **front** arm | pitch forward ( `P` changes sign toward nose-down ), `FR` / `FL` above `RR` / `RL` |
| `logs/log-2.txt` | same payload on the **right** arm | roll right ( `R` toward right-down ), `FR` / `RR` above `FL` / `RL` |
| `logs/log-3.txt` | **none** | no lean; the baseline for the I-term and motor spread |

### Procedure ( the same for each log )

Fly in ANGLE + ALT_HOLD exactly as you fly now, in a clear indoor area. Start PlutoMonitor logging
before step 1 and stop it after step 9. Save each flight under its own name ( `logs.txt` gets
overwritten ).

1. Fit the payload ( or none ). Power up, connect the app, turn **Developer Mode on** while still
   disarmed. A `START` line should appear, then the data lines.
2. Stay disarmed **5 s**. Move the throttle stick over its range once: `Th` must follow it. This is the
   bench check of the fields from task 1.
3. Arm, take off, climb to about **1.5 m**. Hover **5 s** with the sticks centred.
4. **Partial stick-down:** hold the throttle about **one third of the way down** from centre for
   **3 s** ( slow descent, about 10 cm/s ). Do not touch roll or pitch.
5. Centre the stick for **5 s**. Climb back to about 1.5 m if needed, then hover **5 s**.
6. **Full stick-down:** pull the throttle **all the way down** for **3 s** ( about 30 cm/s, about 90 cm
   of descent, so start at 1.5 m or higher ). Do not touch roll or pitch.
7. Centre the stick for **5 s** and watch whether it levels. Then climb back to about 1.5 m.
8. Climb back to about 1.5 m and repeat steps 4 to 7 once more.
9. Land with the app **Land button**, stay disarmed **5 s**, stop logging.

**Safety.** If the lean grows past about 15° or the drift heads for an obstacle, centre the stick at
once ( the hypothesis says it levels when the stick leaves the bottom ). If it does not, land.

**Notes to write next to each log** ( the log cannot record these ): payload mass and exact position,
which way it leaned and drifted, roughly how far, whether it levelled when the stick was centred, and
anything else ( wind, bumps, app disconnects ).

**Assumption:** the board runs the default PID controller ( `pidController` 1, `pidRewrite` ). If the
logged `IR` / `IP` never leave -2..1 even in hover with the payload, it does not, and the I-term fields
are meaningless.

### Results

#### log-1: payload on the front arm ( 27 Oct 2026, first read in task 2 )

97 s of Developer Mode data, 967 complete records, no field missing ( the 123 B line fits ). 32
Developer Mode restarts ( Wi-Fi flicker, the known ~3 s pattern ) and three record gaps of 270-310 ms;
none fall inside a stick-down. `S` is 7 ( armed, ALT_HOLD, ANGLE ) for the whole flight. The I-term
values reach +/-50-80 and the clamp of 256, so the board runs `pidRewrite` ( assumption confirmed ).

Segment averages ( `R` / `P` in decidegrees; I-term and motor spreads in motor counts;
front-rear = ( FR + FL ) - ( RR + RL ) ):

| Phase | Time in log | Th | R | P | IR | IP | Front-rear | Lowest motor |
|---|---|---|---|---|---|---|---|---|
| Hover, stick centred | 9.8-13.8 s | 1500 | -3 | -15 | +63 | -68 | +256 | 1526 |
| Partial stick-down | 35.9-43.2 s ( 7.3 s ) | 1238-1458 | -7 | -16 | +55 | -76 | +303 | 1562 |
| Partial stick-down | 84.3-92.7 s ( 8.5 s ) | 1155-1441 | -10 | -19 | +39 | -74 | +307 | 1522 |
| **Full stick-down** | 23.3-24.1 s ( 0.9 s ) | 1004-1087 | **-48** ( to -65 ) | **+46** ( to +67 ) | **0** | **-1** | +245 | 1460 |
| **Full stick-down** | 28.8-30.5 s ( 1.8 s ) | 1003-1034 | **-35** ( to -64 ) | +6 ( to +84 ) | **0** | **-1** | +256 | 1510 |
| **Full stick-down** | 50.8-51.6 s ( 0.9 s ) | 1004 | **-54** ( to -65 ) | **+34** ( to +52 ) | **1** | **-2** | +244 | 1541 |
| **Full stick-down** | 71.5-73.1 s ( 1.6 s ) | 1007 | **-27** ( to -44 ) | **+22** ( to +46 ) | **0** | **-1** | +169 | 1453 |
| **Full stick-down** | 92.8-94.8 s ( 2.1 s ) | 1007-1008 | -2 ( to +10 ) | **+17** ( to +73 ) | **0** | **-1** | +198 | 1547 |

What it shows:

- **The payload is carried by the I-term.** In hover the pitch I-term holds about -70 to -78 counts,
  which is the whole front-heavy moment: the front motors run about 70 counts above the rear ones
  ( front-rear about +290 = 4 x 72 ). The roll I-term also holds about +50 counts, so this airframe
  carries a roll imbalance of its own besides the payload.
- **Every full stick-down zeroes it within one record.** At 23.3 s `Th` goes to 1087 and `IR` / `IP` go
  from +47 / -71 to 0 / -1 in the same 100 ms; they stay pinned at 0 to +1 / -3 to 0 for as long as `Th`
  is below 1100, in all five stick-downs.
- **The lean follows at once.** Within 0.2 s pitch moves from about 0 to +6.7 deg ( toward the payload:
  nose down, as seen ) and roll from -1.5 to -6.5 deg ( the side the roll I-term was holding ). The P term
  alone then holds the moment at a 2-7 deg lean, as the scout estimated ( about 5 deg ).
- **Partial stick-downs stay level.** With `Th` 1155-1458 for 7-8 s each, the I-term keeps its hover
  value and the attitude matches hover. The reset needs `Th` below 1100; descending alone does not lean.
- **The I-term rebuilds in about 1-1.5 s after the stick is centred** ( pitch -1 to -55 in 1.2 s at
  24.2-25.5 s ), matching "levels again when centred".
- **No motor reaches minthrottle during a stick-down**: the lowest motor in the five stick-downs is
  1453. Hypothesis 3 ( low-end saturation ) is killed. The 1100 / 2000 motor values at 27.3-28.1 s
  and 52.5 s are ground contacts ( user ).
- **The swing of 10-15 deg the other way after each full stick-down is the pilot's pitch correction**
  ( user: pulled back to stop the forward drift ). It is not a firmware effect; the log cannot separate
  it because the roll / pitch sticks are not logged ( budget ).
- **The flight ended in a crash at about 52.5 s**: after the 4th full stick-down the craft was low and
  hit the ground ( user ), the motors saturated, it disarmed at 53.6 s and came to rest upside down
  ( roll 180 deg ). Ground contact, not a firmware fault.

#### log-2: payload on the right arm ( 27 Oct 2026, first read in task 2 )

80 s of Developer Mode data, 788 complete records, no gaps over 250 ms, 24 Developer Mode restarts
( Wi-Fi flicker ). `S` 7 for the whole flight; it ended with the Land button ( `Th` 1240-1297 is
`landThrottle`, see the reading notes ) and no crash.

| Phase | Time in log | Th | R | P | IR | IP | Left-right | Lowest motor |
|---|---|---|---|---|---|---|---|---|
| Hover, stick centred | 17.0-23.1 s | 1500 | -16 | -16 | -59 | +11 | -237 | 1488 |
| Partial stick-down | 10.7-16.9 s ( 6.2 s ) | 1231-1421 | -11 | -12 | -49 | +10 | -201 | 1460 |
| Partial stick-down | 53.2-59.5 s ( 6.3 s ) | **1104**-1459 | -12 | -12 | -58 | +16 | -227 | 1552 |
| **Full stick-down** | 29.3-30.4 s ( 1.2 s ) | 1004 | **+31** ( to +51 ) | -26 | **-1** | **0** | -164 | 1478 |
| **Full stick-down** | 37.0-37.9 s ( 1.0 s ) | 1014-1081 | **+34** ( to +46 ) | -25 | **-1** | **0** | -164 | 1513 |
| **Full stick-down** | 43.8-44.4 s ( 0.7 s ) | 1004-1007 | **+40** ( to +53 ) | -19 | **-1** | **0** | -226 | 1544 |
| **Full stick-down** | 71.4-72.5 s ( 1.1 s ) | 1004-1007 | **+44** ( to +60 ) | -30 | **-1** | **0** | -147 | 1572 |

( left-right = ( FL + RL ) - ( FR + RR ); negative means the right motors run higher. )

What it shows:

- **The same mechanism on the roll axis.** In hover the roll I-term holds about -55 to -60 counts and
  the right motors run about 55 counts above the left ones: the right-arm payload. ( In log-1 the roll
  I-term was +50 with the payload in front, so the right-arm payload more than cancels the airframe's
  own roll trim. ) The pitch I-term is small here ( +10 ).
- **Every full stick-down zeroes it within one record** ( 29.3 s: `IR` -57 to -1 as `Th` goes from 1200
  to 1004 ) and the craft rolls **4.5-6 deg toward the right arm** within 0.2-0.3 s ( `R` from about -10
  to +50 ). Pitch shifts by only about 1 deg, matching the small pitch I-term it lost.
- **Partial stick-downs stay level**, including one at 53.2-59.5 s whose lowest throttle was **1104**,
  four counts above the reset threshold: the I-term held -58 and the roll stayed at hover value. The
  boundary is exactly `mincheck` 1100.
- **No motor below 1478** during a stick-down: hypothesis 3 stays killed.
- **The 15-18 deg swings after centring are commanded, not a firmware effect.** At 30.9-31.6 s and
  38.3-39.0 s the craft rolls 16-18 deg left while the right motors run higher than the left, so the
  controller is driving *into* the roll: in ANGLE mode that only happens when the stick asks for it.
  This matches the user's pitch correction in log-1 ( roll / pitch sticks are not logged ).

#### log-3: no payload ( 27 Oct 2026, read in task 2 )

104 s of Developer Mode data, 991 complete records, no gaps over 250 ms, 32 Developer Mode restarts
( Wi-Fi flicker ). `S` 7 for the whole flight; it ended with the Land button and no crash. Ten full
stick-downs ( some held only 0.3-0.5 s ) and four partial ones.

| Phase | Time in log | Th | R | P | IR | IP | Lowest motor |
|---|---|---|---|---|---|---|---|
| Hover, stick centred | 8.5-12.4 s | 1500 | -7 | -11 | +30 | +9 | 1498 |
| Partial stick-down | 12.5-18.2 s ( 5.8 s ) | 1113-1451 | -9 | -11 | +37 | +10 | 1484 |
| Partial stick-down | 88.6-91.3 s ( 2.8 s ) | 1167-1394 | -6 | -6 | +33 | +16 | 1519 |
| **Full stick-down** | 29.0-30.3 s ( 1.4 s ) | 1004 | **-33** ( to -52 ) | -18 | **0** | **0** | 1515 |
| **Full stick-down** | 36.4-38.0 s ( 1.7 s ) | 1007-1011 | **-27** ( to -44 ) | -15 | **0** | **0** | 1494 |
| **Full stick-down** | 41.9-43.9 s ( 2.1 s ) | 1007-1097 | **-25** ( to -42 ) | -22 | **0** | **0** | 1453 |
| **Full stick-down** | 54.9-56.3 s ( 1.4 s ) | 1027-1031 | **-24** ( to -30 ) | -28 | **0** | **0** | 1473 |
| **Full stick-down** | 91.4-93.9 s ( 2.5 s ) | 1014-1017 | **-20** ( to -39 ) | -20 | **0** | **0** | 1528 |

What it shows:

- **The bare airframe carries a roll trim of its own**: about +30 to +37 counts of roll I-term in hover
  ( pitch about +10 ). This is the same roll trim seen under the front payload in log-1 ( +50 there ).
- **Every full stick-down still zeroes it** and the craft rolls about **2-3 deg** ( averages -20 to -33,
  peaks -30 to -52, against -7 in hover ), in the direction the roll I-term was holding.
- **The lean scales with the I-term the reset removes**: about 35 counts here gives 2-3 deg; about 57-72
  counts with a payload ( log-1, log-2 ) gives 4.5-7 deg. A payload makes the existing effect larger; it
  does not create it.
- **The boundary is `mincheck` 1100 again**: the stick-down at 75.3-78.8 s hovered around the threshold
  ( `Th` 1103 kept the I-term; 1096-1099 zeroed it ).
- **No motor below 1453** in any stick-down.

#### Summary of the three HEAD flights

| Log | Payload | I-term held in hover | Full stick-downs | I-term during them | Lean | Partial stick-downs |
|---|---|---|---|---|---|---|
| log-1 | front arm, under 10 g | pitch about -72, roll about +50 | 5 | 0 / -1 | 2-7 deg nose-down, 3-6 deg roll | level |
| log-2 | right arm, under 10 g | roll about -57, pitch about +10 | 4 | -1 / 0 | 4.5-6 deg roll right | level ( one at `Th` 1104 ) |
| log-3 | none | roll about +35, pitch about +10 | 10 | 0 / 0 | 2-3 deg roll | level |

Side finding ( out of scope, logged ): while disarmed with the app stick centred ( `Th` 1500 ),
`IR` / `IP` wind up to the 256 clamp, because THROTTLE_LOW never occurs with the app; they are reset
to 0 at the arming instant ( 2.3 s ), so take-off is not affected in this log.

Safety note for log-2 / log-3: a full stick-down here lasted 0.9-2.1 s, and the craft was not climbed
back to 1.5 m between them. Climb back to 1.5 m after **every** stick-down before the next one.

## Control flights on `4657cbf` ( task 3 )

**Build:** worktree `E:/08-DRONA-AVIATION/99-Dump/MagisV2-4657cbf` ( detached at `4657cbf`, FW 3.7.0 ) with the
task 1 diff applied unchanged ( `git apply`, clean: `PlutoPilot.cpp` there was identical to HEAD's ).
Built with `make PROJECT=AltX TARGET=PRIMUS_X2_v1`: `Build/PRIMUS_X2_v1/AltX_PRIMUS_X2_v1_3.7.0.hex`,
flash 98.6 KB, RAM 14.8 KB, 0 warnings in `PlutoPilot.cpp`, `pidGetGyroITerm` linked. No warning gate:
the baseline did not exist at that commit. `plutoide.ini` ( untracked ) was copied into the worktree so
PlutoIDE recognises it. The code is the reviewed task 1 diff, byte for byte, so no second review.

**How it differs from HEAD on a full stick-down:** the stick sets the descent rate directly,
`( rcData - 1500 ) / 4` clamped to 100 cm/s, with no ramp ( `2a8d59a` added the 30 cm/s shaped
profile ). The THROTTLE_LOW I-term reset in `mw.cpp` is the same code.

**Question it answers:** the user saw this build stay level on a full stick-down. Either the reset
also fires here and the 1 s drop hides the lean ( hypothesis 2 ), or something keeps `Th` above 1100
or the I-term alive on this build.

### Procedure ( log-4, payload on the front arm )

Same as the HEAD procedure, with shorter, higher full stick-downs because this build descends at up to
100 cm/s:

1. Front payload as in log-1. Developer Mode on while disarmed, start logging, 5 s disarmed.
2. Arm, take off, climb to about **2 m**. Hover 5 s.
3. Partial stick-down ( about one third ) for 2 s, centre 5 s, climb back to 2 m.
4. **Full stick-down for at most 1.5 s** ( up to about 1.5 m of descent ), then centre 5 s and watch
   the attitude. Climb back to 2 m.
5. Repeat steps 3-4 twice more ( three full stick-downs in all ).
6. Land with the Land button, 5 s disarmed, stop logging.

Same notes as before: which way it leaned and drifted, and whether you touched roll / pitch.
**Safety:** this build drops fast at full stick; release the stick at once if it gets near the floor.

### Results

#### log-4: `4657cbf` ( FW 3.7.0 ), payload on the front arm ( 27 Oct 2026 )

88 s of Developer Mode data, 859 complete records, two gaps of 270-285 ms ( outside the stick-downs ),
28 Developer Mode restarts. Eleven airborne full stick-downs ( 0.3-1.8 s each ). The craft reached the
floor and disarmed after four of them ( 12 s, 15 s, 17 s and 52 s; user: the fast descent hit the
ground ), and was re-armed each time.

| Phase | Th | R | P | IR | IP | Lowest motor |
|---|---|---|---|---|---|---|
| Hover, stick centred ( 8.4-10.3 s, 28.0-30.0 s, 36.3-38.8 s ) | 1500 | -6 to -10 | -11 to +3 | +46 to +50 | -66 to -71 | 1338-1442 |
| Full stick-downs, 11 airborne ( averages per stick-down ) | 1004-1055 | -9 to -38 | **+7 to +73** ( peaks +27 to +101 ) | **0** | **-2 to 0** | 1264-1533 |

What it shows:

- **The old build has the same mechanism.** In hover the pitch I-term holds -66 to -71 counts against the
  front payload, as on HEAD ( log-1, about -72 ). In every full stick-down `Th` goes to 1004-1055, `IR` /
  `IP` drop to 0 within one record and the craft pitches toward the payload by about as much as on HEAD,
  or more ( averages up to +7.3 deg, peaks up to +10 deg, against +0.6 to +4.6 deg averages on HEAD ).
  The roll trim is lost the same way ( -2 to -4 deg ).
- **Hypothesis 2 is confirmed: the lean was always there, and the fast drop hid it.** The user watched
  for it on this flight and saw it only "barely, it dropped too fast to notice". On this build a full
  stick-down commands 100 cm/s at once, so in 1-1.5 s the craft drops more than 1 m and four of the
  eleven stick-downs ended on the floor. On HEAD the same stick commands a 30 cm/s descent, so the craft
  hangs in the air for 3 s or more, leaning and drifting, in plain sight.
- **`2a8d59a` did not introduce the lean.** It slowed the full-stick descent, which made an existing
  lean visible. The visual bisect pointed at it correctly as the commit where the symptom appeared, but
  the cause is the THROTTLE_LOW I-term reset at `mw.cpp:895-897`, present in both builds.

## Control flight on release `v3.0.0` ( task 7 )

**Build:** worktree `E:/08-DRONA-AVIATION/99-Dump/MagisV2-v3.0.0` ( detached at tag `v3.0.0`, `0acc43d`,
10 Feb 2026, FW 3.0.0 ). The `pid.cpp` / `pid.h` diff applied cleanly. `PlutoPilot.cpp` there has a
different comment block, so the lines were ported by anchor from the HEAD file; the added lines are
identical to HEAD's. `make PROJECT=AltX TARGET=PRIMUS_X2_v1`: flash 93.6 KB, RAM 12.5 KB, 0 warnings in
`PlutoPilot.cpp`, `pidGetGyroITerm` linked. Staged for flashing as
`Build/PRIMUS_X2_v1/AltX_PRIMUS_X2_v1_3.15.1.hex` in the main folder; the app reports FW 3.0.0.

**Same as `4657cbf` on a full stick-down:** the THROTTLE_LOW reset is at `mw.cpp:821-824`, the default
controller is `pidRewrite`, and the stick sets `( rcData - 1500 ) / 4` clamped to -100..120 cm/s with no
ramp ( `altitudehold.cpp:230-232` ).

**Procedure:** the log-4 procedure ( front payload, about 2 m, three full stick-downs of at most 1.5 s,
one partial stick-down, Land button ) into `logs/log-5.txt`.

### Results

#### log-5: release `v3.0.0` ( FW 3.0.0 ), payload on the front arm ( 27 Oct 2026 )

52 s of Developer Mode data, 512 complete records, no gaps, a single `START` ( no Developer Mode restart
in this flight ). Four airborne full stick-downs, two partial ones, ended with the Land button
( `Th` 1300 = `landThrottle` at 48.4-50.1 s ) and a disarm.

| Phase | Time in log | Th | R | P | IR | IP | Lowest motor |
|---|---|---|---|---|---|---|---|
| Hover, stick centred | 11.2-12.8 s, 24.7-25.7 s | 1500 | -6 to -13 | -1 to +16 | +41 to +46 | -69 to -74 | 1343-1351 |
| Partial stick-down | 26.8-27.4 s | 1193-1380 | -10 | -5 | +39 | -70 | 1410 |
| Partial stick-down | 32.5-33.8 s ( 1.4 s ) | 1334-1440 | +11 | -5 | +48 | -84 | 1536 |
| **Full stick-down** | 12.9-13.6 s ( 0.8 s ) | 1004 | **-36** ( to -58 ) | **+49** ( to +60 ) | **0** | **-1** | 1302 |
| **Full stick-down** | 18.9-19.7 s ( 0.9 s ) | 1007 | **-32** ( to -42 ) | **+61** ( to +80 ) | **0** | **-1** | 1421 |
| **Full stick-down** | 39.8-40.6 s ( 0.9 s ) | 1004-1007 | **-43** ( to -65 ) | **+27** ( to +51 ) | **0** | **-1** | 1449 |
| **Full stick-down** | 43.9-45.0 s ( 1.2 s ) | 1004 | -15 ( to -36 ) | **+32** ( to +74 ) | **0** | **-1** | 1100 |

What it shows:

- **The tagged release behaves exactly like `4657cbf` and HEAD.** The pitch I-term carries the front
  payload in hover ( -69 to -74 ), every full stick-down zeroes it within one record, and the craft
  pitches 2.7-6.1 deg toward the payload on average ( peaks to 8 deg ), with the roll trim lost too
  ( 1.5-4.3 deg ). Partial stick-downs keep the I-term and stay level.
- **The behaviour has shipped since at least `v3.0.0` ( 10 Feb 2026 ).** It is not a regression of
  this branch.
- The single 1100 motor value in the 4th stick-down ( 43.9-45.3 s ) is most likely floor contact at
  the end of the fast drop, as in log-4 ( not confirmed with the user ). It does not change the result:
  the lean was already there in the first three stick-downs, which stayed well clear of 1100.

## Build gate on the fix ( task 10 )

| | |
|---|---|
| Source | HEAD `a81c180` + task 9 fix ( `mw.cpp`, `pid.cpp`, `pid.h` ) + task 1 diagnostics |
| Command | `PROJECT=AltX .claude/skills/pluto-build/driver.sh --gate PRIMUS_X2_v1` ( 27 Oct 2026, 14:41 ) |
| Warning gate | passed: no new warnings in `src/`; `PlutoPilot.cpp` 0 warnings ( re-checked with `tools/warnings.py check` ) |
| Flash | 122.0 KB / 256 KB ( 47 % ), +0.1 KB over the diagnostics-only build |
| RAM | 20.5 KB / 40 KB ( 51 % ), unchanged |
| Hex to flash | `Build/PRIMUS_X2_v1/AltX_PRIMUS_X2_v1_3.15.1.hex` ( fix + diagnostic log ) |

The build is newer than every edited source ( sources 14:40, build 14:41 ), and both `pidSetGyroITermFreeze` and
`pidGetGyroITerm` are in the linked ELF.

## Flights on the fixed build ( task 11 )

**Build:** `Build/PRIMUS_X2_v1/AltX_PRIMUS_X2_v1_3.15.1.hex` ( HEAD + task 9 fix + task 1 diagnostic log, the
same log line as log-1..5 ). Fly in ANGLE + ALT_HOLD from the app, as before.

### log-6: front payload, the log-1 procedure

Same payload ( under 10 g ) on the front arm, same steps as log-1 ( *Procedure* under *Flights on HEAD* ):
5 s disarmed with Developer Mode on, take off, hover at 1.5 m, partial stick-down 3 s, centre 5 s, full
stick-down 3 s, centre 5 s, **climb back to 1.5 m after every stick-down**, repeat once, Land button.
Hands off roll and pitch during the stick-downs.

| | Pass | Fail |
|---|---|---|
| `IP` during each full stick-down ( `Th` below 1100 ) | stays within about +/-15 counts of its hover value ( about -72 in log-1 ) | pinned near 0, as in log-1 |
| Mean `P` during each full stick-down | within 1.5 deg of the partial stick-downs' mean | 2-7 deg toward the payload, as in log-1 |
| Visual | no lean, no drift toward the front | leans and drifts forward |

### log-7: regression, no payload needed

Payload optional ( keep it on the front arm if it is easy, so the I-term is large and any wind-up shows ).

1. Developer Mode on while disarmed, start logging, 5 s disarmed.
2. **App take-off:** arm and take off from the app as usual. Hover at 1.5 m for 5 s.
3. **Full-stick touchdown:** hold the stick fully down until it lands; keep it fully down for **5 s** on the
   floor ( motors keep running, the craft stays armed ); then push the stick up to lift off and hover 5 s.
4. **Partial-stick touchdown ( the reviewer's residual ):** hold the stick about **one third down** until it
   lands ( slow, about 10 cm/s ); keep it there for **10 s** on the floor; then pull it **fully down for 3 s**;
   then push it up to lift off and hover 5 s. Watch the lift-off: level or tilted?
5. **Flip:** one app flip from at least 1.5 m, as in the flip tests.
6. **Land button:** land with the app button and stay disarmed 5 s; stop logging.

| | Pass | Fail |
|---|---|---|
| Step 2 take-off | as in log-1: I-terms 0 at arm, level climb | anything different from log-1 |
| Step 3, 5 s on the floor with the stick fully down | `IR` / `IP` constant ( frozen ), lift-off level | `IR` / `IP` grow on the floor, or a tilted lift-off |
| Step 4, 10 s on the floor at partial stick | **measured, not pass / fail**: how far `IR` / `IP` move from their hover value in 10 s, whether they stay frozen during the 3 s full stick-down, and the lift-off attitude ( peak `R` / `P` in the first second ) | - |
| Step 5 flip | completes and returns as in the flip tests | flip refused or timed out |
| Step 6 Land | lands and disarms as before | anything different |

**Safety.** Steps 3 and 4 run the motors on the floor while armed; keep hands and objects clear. If the
lift-off in step 4 is visibly tilted, land at once and note it: that is the residual showing.

Notes to write next to each log: payload and position, what you saw at each step ( especially the lift-off
in steps 3 and 4 ), and any roll / pitch input.

### Results

#### log-6: fixed build, front payload ( 27 Oct 2026 ) - PASS

52 s, 509 complete records, no gaps, 18 Developer Mode restarts. Hover: `IR` about +42-48, `IP` about -70
( the same payload trim as log-1 ). Three full stick-downs, two partial.

| Phase | Time in log | Th | IR | IP | Mean R | Mean P | Lowest motor |
|---|---|---|---|---|---|---|---|
| Hover | 3.0-13.5 s | 1500 | +42 | -70 | -1.7 | +4.7 | 1424 |
| Partial stick-down | 13.6-14.0 s | 1153-1328 | +48 | -71 | -10.8 | 0.0 | 1612 |
| Partial stick-down | 48.2-51.7 s ( 3.6 s ) | 1160-1298 | +55 | -70 | -8.3 | +3.0 | 1480 |
| **Full stick-down** | 14.1-14.9 s ( 0.9 s ) | 1069-1098 | **+48 ( constant )** | **-72 ( constant )** | +15.8 | **+1.0** | 1686 |
| **Full stick-down** | 15.7-18.6 s ( 3.0 s ) | 1004-1030 | **+54 ( constant )** | **-68 ( constant )** | +4.8 | **+1.9** | 1593 |
| **Full stick-down** | 28.1-34.6 s ( 6.5 s ) | 1004-1007 | **+48 ( constant )** | **-70 ( constant )** | -4.8 | **+2.9** | 1570 |

- **The I-term is held, not zeroed.** In every full stick-down `IR` / `IP` sit at their hover values and do not
  change by a single count for the whole hold ( frozen, as designed ), including a 6.5 s hold at the bottom.
  Pass criterion ( within +/-15 of hover ): met with a margin of 2 counts.
- **The craft stays level.** Mean pitch during the full stick-downs is +0.1 to +0.3 deg, against 0.0 to
  +0.3 deg in the partial stick-downs ( criterion: within 1.5 deg ) and against +0.6 to +4.6 deg on the same
  flight profile in log-1. Mean roll is within +/-1.6 deg. The peak excursions ( up to +/-5 deg in the 6.5 s
  hold ) are the normal hover wobble seen in every log, not a lean: they are centred on zero.
- No motor below 1480 in any stick-down. Land button at the end, clean disarm.
- User: no lean, no drift ( visual ).

#### log-7: fixed build, no payload, regression ( 27 Oct 2026 ) - PASS, residual not observed

70 s, 684 complete records, two gaps of 300-375 ms outside the test steps, 20 Developer Mode restarts. Hover:
`IR` about +47, `IP` about +15-20 ( no payload ).

| Step | Time in log | What the log shows | Result |
|---|---|---|---|
| 2. App take-off | 2.2 s | `IR` / `IP` 0 at the first armed record ( from 256 / -256 while disarmed ), level climb | as log-1: **pass** |
| 3. Full-stick touchdown, 10 s on the floor | 32.7-42.7 s, `Th` 1004 | `IR` 49 and `IP` 13 **constant for the whole 10 s** ( 94 records, motors down to 1210 on the floor ); lift-off at 42.7 s into a 10 s full-throttle climb, transient P -6.4 / R -2.6 deg in the first second ( the same size as the first take-off's, -4.3 / +6.5 deg ) | frozen, level lift-off ( user confirms ): **pass** |
| 4. Partial-stick touchdown, then full stick, then lift-off | 56.7-61.4 s at `Th` 1306, 62.0-62.7 s at 1004, then `Th` 1159-1317 until the Land | on the floor at partial stick `IR` 50 to 60 and `IP` 22 to 12..29 ( a drift of at most 10-15 counts in 5 s, nowhere near the 256 clamp ); the full-stick second froze them at 54 / 27; the stick then stayed in the descent band to the Land, so the lift-off itself cannot be picked out of the log ( level, user confirms ) | **residual not observed** on this floor; measured, see note |
| 5. Flip | - | **not loggable**: the app offers either the Dev button or the Flip button | checked by review: `flip ( )` writes its throttle after the reset decision ( `acrobats.cpp`, after `processRx` ), so the fix cannot reach it |
| 6. Land button | about 66-70 s | landed and disarmed at 69.7 s | **pass** |

Note on step 4: the partial-stick floor period was about 5 s, not the planned 10 s, and the floor was
level enough that the I-term drifted by only 10-15 counts; the residual needs a tilted floor to wind up. It is
recorded as not observed, not as impossible. An unrelated transient: both climb-outs were flown at full throttle for 10 s ( 19.3-29.8 s and 42.7-52.8 s ) and
`IP` swung to -75 and -15 before settling back to +15; that is the normal I-term response to a long full-throttle
climb and has nothing to do with the stick-low path.

**Verdict for task 11:** the fix removes the lean ( log-6 ) and the regression cases pass ( log-7 ). The
flip is covered by review only.

## Fix-only build ( task 12 )

After the diagnostics were removed: `PROJECT=AltX .claude/skills/pluto-build/driver.sh --gate PRIMUS_X2_v1`
passed ( no new `src/` warnings ), flash 121.1 KB, RAM 20.5 KB. `Build/PRIMUS_X2_v1/AltX_PRIMUS_X2_v1_3.15.1.hex` is now
the fix without the diagnostic log ( it no longer prints the `t` / `Th` / `IR` / `IP` line ).

