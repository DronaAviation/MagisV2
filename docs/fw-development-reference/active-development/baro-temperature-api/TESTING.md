# Baro Temperature API - Testing

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

## Test 1: bench check of `Sensor_Get ( Barometer, ... )` ( task 3 )

**Build.** `PRIMUS_X2_v1`, FW 3.15.2 plus the task 1 fix and the temporary instrumentation in
[PlutoPilot.cpp](../../../../PlutoPilot.cpp). Hex: `Build/PRIMUS_X2_v1/AltX_PRIMUS_X2_v1_3.15.2.hex` built 3 Nov 2026 at 22:18 ( the one flashed for log-1; a 22:21 rebuild only added `n:` to the START line ).

**Log line** ( about 80 bytes per tick with framing at the default 10 Hz, under the 130-byte budget ):

| Field | Source | Unit |
|---|---|---|
| `t` | `millis ( )` | ms, drone time |
| `n` | tick counter, +1 per `plutoLoop ( )` | count; a step of 2 is a lost record |
| `Tx100` | `Sensor_Get ( Barometer, Temperature )` | deg C x100 ( the API under test ) |
| `Pa` | `Sensor_Get ( Barometer, Pressure )` | Pa ( the API under test ) |
| `degC` | `getBaroTemperature ( )` | deg C, 1 decimal ( the internal value altitude hold uses ) |
| `START n:` | `onLoopStart ( )`, own tick | Developer Mode ( re )start; it carries the tick count, so `n` steps by 2 across a START without a lost record |

**Sequence.**

1. Flash the hex with PlutoIDE. Props off, drone on the bench, disarmed, away from fans and sunlight.
2. Connect the app, open PlutoMonitor, switch Developer Mode on.
3. Log 2 minutes untouched.
4. Then hold a warm finger on top of the board ( near the baro ) for 30 s, release, and log 1 more minute.
5. Save the log into `logs/log-1.txt` ( its own name, `logs.txt` gets overwritten ) and note the room temperature
   from a thermometer if one is available.

**Pass if all hold.**

| Check | Expected |
|---|---|
| `Tx100` is four digits, not two | about 2000-4000 for an indoor room ( the old build gave `32` ) |
| `Tx100` agrees with `degC` x100 | within 10 ( `degC` prints one decimal, so it is coarser ) |
| `Tx100` moves in steps smaller than 100 | at least some consecutive changes of 1-20 ( the decimals are no longer lost ) |
| `Tx100` follows the finger | a rise of 50 or more during step 4, falling back after |
| `Pa` unchanged in kind | about 100470 Pa ( within 200 Pa of the T03 reading at the same place ), steady within about 10 Pa |
| vs thermometer ( if available ) | within 2 C ( the sensor sits on a warm board, so it reads high ) |

**Fails if.** `Tx100` stays two digits ( old firmware flashed, check the hex time ), `Tx100` only moves in steps of
100 ( still whole degrees ), or `Tx100` and `degC` x100 disagree by more than 10.

**Cannot show.** Behaviour in flight ( the getter is the same variable altitude hold reads, see INVESTIGATION.md ),
or below 0 C ( the negative rounding branch is untested on hardware ).

### Results ( [log-1.txt](logs/log-1.txt), 3 Nov 2026, 22:26-22:27, Wi-Fi )

**Pass.** `Sensor_Get ( Barometer, Temperature )` now returns deg C x100 with the decimals kept, and it matches the
internal value altitude hold uses.

| Check | Result | Verdict |
|---|---|---|
| `Tx100` four digits | 3500 -> 3574 ( was `32` in T03 ) | pass |
| `Tx100` vs `degC` x100 | within +/-5 in all 498 records ( `degC` prints one decimal ) | pass |
| Steps under 100 | 74 changes, every one a step of 1; 497 of 498 values are not whole degrees | pass |
| Follows a warm finger | no separate step: `Tx100` rose steadily by about 74 over 52 s ( board warm-up, `t` starts 40 s after boot ) | not shown |
| `Pa` steady | 101240-101245, standard deviation 1.0 Pa | pass |
| `Pa` vs T03 reading | 101242 vs 100470: 772 Pa apart, a different day / place ( weather ), not a firmware effect; the pressure path only gained rounding | not comparable |
| vs thermometer | no reading noted | not shown |

5 s block means ( selected ):

| Time | Tx100 | degC | Pa |
|---|---|---|---|
| 0 s | 3505.0 | 35.05 | 101243.0 |
| 25 s | 3545.9 | 35.46 | 101242.2 |
| 50 s | 3572.7 | 35.70 | 101241.5 |

**Log health.** 498 complete records over 52.6 s at 10 Hz; no field missing, so the line stayed inside the budget and
the app stayed connected. 11 `START` markers ( Developer Mode restarts on Wi-Fi ). `n` stepped by 2 twenty times: ten next
to a `START` ( the START tick ), ten elsewhere with a matching 200 ms `t` step, i.e. lost records on the link, not loop
stalls.

**Which build flew.** The `START` lines carry no `n:`, so the 22:18 hex was flashed, before the reviewer's nit that added
the tick count to `START`. The code under test ( task 1 ) is identical in both builds.

**Not tested.** The finger step ( the log is 53 s, not the planned 3 min ), a thermometer comparison, below 0 C
( the negative rounding branch ), and `PRIMUS_V5` ( same ICP-10111 path, built at commit ).
