# Flight status enum map - Testing

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

## Test 1: bench crash disarm with the fixed API ( task 3 )

**Build.** `PRIMUS_X2_v1`, the task 1 fix plus the temporary logging in [PlutoPilot.cpp](../../../../PlutoPilot.cpp).
Hex: `Build/PRIMUS_X2_v1/AltX_PRIMUS_X2_v1_3.15.5.hex`, built 5 Nov 2026 at 23:04 ( still 3.15.5: the version is bumped
at commit ). 123500 / 1316 / 19732 B.

**Log line** ( about 50 bytes per tick at the default 10 Hz, well under the 130-byte budget ):

| Field | Source | Meaning |
|---|---|---|
| `n` | tick counter, +1 per `plutoLoop ( )` | a step of 2 away from a START is a lost record |
| `t` | `millis ( )` | drone time, ms |
| `S` | `FlightStatus_Get ( )` | **under test**: 4 crashed, 2 low battery, 3 critical, 7 OK to arm, 8 armed ... |
| `cr` | `FlightStatus_Check ( FS_CRASHED )` | **under test**, 1 / 0 |
| `lb` | `FlightStatus_Check ( FS_LOW_BATTERY )` | **under test**, 1 / 0 |
| `cb` | `FlightStatus_Check ( FS_INFLIGHT_LOW_BATTERY )` | **under test**, 1 / 0 |
| `f` | `flightIndicatorFlag` | the reference: internal bits, 4 = Crash, 8 = Low_battery, 16 = LowBattery_inFlight, 128 = Ok_to_arm, 64 = Not_ok_to_arm, 256 = Armed |
| `START n:` | `onLoopStart ( )`, own tick | Developer Mode ( re )start |

**Sequence.** **Props off.** App connected, ANGLE mode ( the app default ), Developer Mode on. Crash detection is on
by default ( `fsCrash` ); it only trips when armed, in ANGLE, not in a flip.

1. Drone flat on the bench, disarmed: **5 s** ( expect `S:7` ).
2. **Arm** from the app, leave the throttle at the bottom: **5 s** ( expect `S:8` ).
3. Hold the drone and **tilt it past 70 degrees** ( on its side ) until the motors stop ( crash disarm ). **Keep it on
   its side for 5 s** ( expect `S:4 cr:1` ).
4. **Put it back flat**: 5 s ( expect `S:7` within about 0.3 s; if the app's ARM is still on, switch it off ).
5. Repeat steps 2-4 once.
6. Save the log into `logs/log-1.txt` ( its own name; `logs.txt` gets overwritten ) and **press Ctrl+S**. Note anything
   unusual ( the motors did not stop on the tilt, the app showed something else ).

**Pass if all hold.**

| Check | Expected |
|---|---|
| Crashed, tipped | `S:4`, `cr:1`; `f` has bit 4 ( Crash ) |
| Not reported as battery | `lb` = 1 only when `f` has bit 8, `cb` = 1 only when `f` has bit 16, on every record ( was swapped before ) |
| Every record maps by name | `S` = the public value of the lowest set bit of `f` ( 4 → 4 crashed, 8 → 2, 16 → 3, 128 → 7, 256 → 8 ) |
| Recovery | `S:7` within a few ticks once flat |
| Armed | `S:8` while armed ( unless a battery bit is set, which outranks it ) |

**Fails if.** `S` reads 2 while `f` has the Crash bit, or `cr` / `lb` / `cb` disagree with `f`.

**Cannot show.** In-flight crashes ( a real impact ), a provisional or confirmed critical battery unless the pack is
low, a crash during a flip.

### Results ( log-1, 14 Nov 2026, **PASS** )

Log: [logs/log-1.txt](logs/log-1.txt), Wi-Fi, 90 s of drone time ( `t` 70788-160736 ), 841 complete records ( `n` 2-886 ),
25 `START` ticks ( Wi-Fi restarts, normal ), 20 single records dropped by the app ( no step larger than 2 ). Analysed by
script: every record checked against `f`.

| Check | Result | Pass |
|---|---|---|
| Every record maps by name | `S` = public value of the lowest set bit of `f` on **841 of 841** records | yes |
| `Check` agrees with the flags | `cr` = Crash bit, `lb` = Low_battery bit, `cb` = LowBattery_inFlight bit on **841 of 841** | yes |
| Crashed, tipped | two crash disarms; `S:4 cr:1 lb:0 cb:0` on all 152 crash records ( 94 and 58 records, about 10 s and 6 s on its side ). The old code would have returned 2 on each | yes |
| Not reported as battery | `lb` and `cb` 0 on every record; no battery bit in `f` ( pack partly used, above the warning level ) | yes |
| Recovery | `S:7`, `f:128` on the next record after the crash flag cleared ( 113005 and 153918 ms ) | yes |
| Armed | `S:8`, `f:256` while armed ( 197 and 126 records ) | yes |

**Sequence seen** ( `S`, `f` ): 7 / 128 for 11 s → armed 8 / 256 for 21 s → crash: 4 / 4 ( one tick, just disarmed ), 4 / 132
( Crash + Ok_to_arm ), then 4 / 68 ( Crash + Not_ok_to_arm: on its side, not level ) for 9.7 s → 7 / 128 once flat →
armed again 13 s → crash 4 / 4, 4 / 132, 4 / 68 for 5.9 s → 7 / 128 to the end. Both crash disarms followed the same
pattern.

**Not shown.** The battery statuses set ( `lb` / `cb` = 1 ): the pack stayed above the warning level, so only their 0
side and the crash-vs-battery separation were exercised; the mapping for them is checked against the code and by the
review. The exact clear delay after righting ( no angle field ): `S` went from 4 straight to 7 between consecutive
records, with no stale or wrong value in between. In-flight impacts and crashes in a flip.

**Verdict.** `FlightStatus_Get ( )` returns 4 ( `FS_CRASHED` ) after a crash disarm, and `FlightStatus_Check ( )` tests
the right flag, on every record. `PlutoPilot.cpp` restored to HEAD ( the diff was 20 added lines, none removed ); the
clean hex was rebuilt: gate green, 122752 / 1316 / 19728 B, identical to the task 2 build,
`Build/PRIMUS_X2_v1/AltX_PRIMUS_X2_v1_3.15.5.hex` ( 14 Nov 2026 12:47 ) has no logging.
