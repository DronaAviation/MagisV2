# RcCommand_Get API - Testing

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

## Test 1: bench check of `RcCommand_Get ( )` against the sticks ( task 3 )

**Build.** `PRIMUS_X2_v1`, the task 1 fix plus the temporary logging in [PlutoPilot.cpp](../../../../PlutoPilot.cpp).
Hex: `Build/PRIMUS_X2_v1/AltX_PRIMUS_X2_v1_3.15.4.hex`, built 5 Nov 2026 at 15:35 ( still named 3.15.4: the version is
bumped at commit ). The build linking is itself the B12 check: user code now calls `RcCommand_Get ( )`.

**Log line** ( 118-122 bytes per tick with framing, at the default 10 Hz, under the 130-byte budget ):

| Field | Source | Unit |
|---|---|---|
| `n` | tick counter, +1 per `plutoLoop ( )` | count; a step of 2 away from a START is a lost record |
| `a0` / `a1` / `a2` | `RcCommand_Get ( ) [ RC_ROLL / RC_PITCH / RC_YAW ]` | 1500 + `rcCommand`, about 1000-2000 - **the API under test** |
| `a3` | `RcCommand_Get ( ) [ RC_THROTTLE ]` | `rcCommand [ THROTTLE ]`, 1100-2000 - **under test** |
| `c0` / `c1` / `c2` | `RcCommand_Get ( RC_ROLL / RC_PITCH / RC_YAW )` | the channel getter, the reference |
| `d3` | `RcData_Get ( RC_THROTTLE )` | throttle stick, 1000-2000, the reference for `a3` |
| `START n:` | `onLoopStart ( )`, own tick | Developer Mode ( re )start |

**Sequence.** Props off, **disarmed**, app connected, Developer Mode on. Hold each position about 3 s:

1. All sticks centred, throttle at the bottom: **5 s**.
2. **Roll:** full right, full left, back to centre.
3. **Pitch:** full forward, full back, back to centre.
4. **Yaw:** full right, full left, back to centre.
5. **Throttle:** bottom, then slowly up to the top over about 5 s, hold, then slowly back to the bottom.
6. Save the log into `logs/log-1.txt` ( its own name, `logs.txt` gets overwritten ), and **press Ctrl+S**. Note anything
   unusual ( a stick that did not reach its end ).

**Pass if all hold.**

| Check | Expected |
|---|---|
| Array equals the channel getter | `a0 = c0`, `a1 = c1`, `a2 = c2` on every record |
| Centred | `a0`, `a1`, `a2` at 1500 +/- a few in step 1 ( `deadband` 0 ) |
| Follows the sticks | each of `a0` / `a1` / `a2` moves to both sides only while its own stick moves. Full stick with the default rates: roll and pitch about **1050 / 1950** ( `rcRate8` 90 ), yaw **1000 / 2000** but **yaw right reads below 1500** ( `yaw_control_direction` 1 ). Half stick only about 1500 +/- 120 ( expo 65 ). The sign of each is recorded |
| Throttle | `a3` = max ( `d3`, 1100 ) within about 2 ( default throttle curve ), so 1100 at the bottom and 2000 at the top; during the sweep `a3` may trail `d3` by one RX frame ( `rcSmoothing` ) |
| Old bug gone | no record shows fixed unrelated numbers ( reading address 0 gave the vector table: the same values whatever the sticks ) |

**Reading the log** ( instrumentation review ): all expected ends assume the default rate, expo and throttle settings;
if they were changed in the app the ends differ, but `a = c` and `a3` tracking `d3` still hold. A gap in `n` is a line
the app dropped, not the firmware.

**Fails if.** The array differs from the channel getter, sits at constant values, or `a3` does not follow `d3`.

**Cannot show.** Behaviour in flight or in altitude hold ( user code runs before altitude hold writes its throttle,
so the value would be the same stick value; see the decisions log ).

### Results ( log-1, 5 Nov 2026, **PASS** )

Log: [logs/log-1.txt](logs/log-1.txt), 563 complete records ( `n` 151-745 ), 21 `START` ticks, 11 lines dropped by the
app. Analysed by script.

| Check | Result | Pass |
|---|---|---|
| Array equals the channel getter | `a0 = c0`, `a1 = c1`, `a2 = c2` on **563 of 563** records | yes |
| Old bug gone | every element moves with the sticks: `a0` 1476-1844, `a1` 1176-1790, `a2` 1040-1960, `a3` 1365-1995; no constant values | yes |
| Centred | `a0` / `a1` 1500 whenever the right stick was centred ( 366 records ); 1476 / 1480 for the first 58 records before the stick was first touched, and `c0` / `c1` show the same, so it is the stick, not the getter; `a2` rests at 1510, the app's yaw centre ( `c2` the same ) | yes |
| Follows the sticks | yaw reached 1040 / 1960 ( ends 1000 / 2000 expected, within the app's steps ); roll and pitch moved together on every input, never one alone ( see below ) | yes, with the note below |
| Throttle | `a3 = max ( d3, 1100 ) - 1` on 561 records, `+ 0` on 2: the default throttle curve, within the expected 2. Same-tick match on every record, so no frame lag showed. `d3` ranged 1366-1996: the app's throttle stick sprang back to 1500 and never reached the bottom, so the 1100 floor was not exercised | yes |

**Roll and pitch moved together.** Every right-stick input moved `a0` and `a1` at once, with a combined deflection of
212-442 ( for example `+310 / -315`, `+332 / +284` ), close to the 450 full-stick value spread over both axes. Either the
app's joystick was moved diagonally, or HEADFREE was on ( it rotates roll and pitch by the heading difference,
`mw.cpp:427-429` ). It does not affect the result: the getter returned exactly what the channel getter and `rcCommand`
held on every record. It means the sign of roll and pitch on their own, and the 1050 / 1950 ends, were not shown by
this log; yaw's sign was not identified either, because the order of the right and left yaw moves is not in the notes.
The signs are the `rcCommand` convention, identical to the channel getter's, so the wiki states them from the code.

**Verdict.** The API under test returns the live values, identical to `RcCommand_Get ( channel )`, with the throttle
equal to the throttle-curve output of the stick. `PlutoPilot.cpp` restored to HEAD ( the diff was 22 added lines, none
removed ); the clean hex was rebuilt: gate green, 122752 / 1316 / 19728 B, identical to HEAD,
`Build/PRIMUS_X2_v1/AltX_PRIMUS_X2_v1_3.15.4.hex` ( 16:05 ) has no logging.
