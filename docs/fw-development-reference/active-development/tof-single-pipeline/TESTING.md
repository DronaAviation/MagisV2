# ToF Single Pipeline - Testing

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [TESTING](TESTING.md)

## Builds kept in `rollback/` ( task 1, 14 Oct 2026 )

Both builds are `PRIMUS_X2_v1`, FW 3.11.3, from HEAD `c495d6a`, with Arm GNU Toolchain 14.2.1.

| File | What | text / data / bss | Flash / RAM |
|---|---|---|---|
| `ROLLBACK_LASER_ALT_PRIMUS_X2_v1_3.11.3_c495d6a.hex` | **The rollback flight build.** `LASER_TOF` + `LASER_ALT` on in `target.h`, stock ( empty ) `PlutoPilot.cpp`, no XRanging. `target.h` was restored afterwards ( `git checkout` ). | 105440 / 1280 / 16672 | 104.2 KB / 17.5 KB |
| `BASELINE_STOCK_PRIMUS_X2_v1_3.11.3_c495d6a.hex` | **The baseline build** for the baro-only hovers ( task 1 now, task 10 hover B later ). No laser defines, no XRanging; `PlutoPilot.cpp` carries the temporary log below. | - | 102.1 KB / 16.8 KB |
| ( reference ) the stock build, empty `PlutoPilot.cpp` | today's shipped firmware | 102540 / 1280 / 15944 | 101.3 KB / 16.8 KB |

**Cost of `LASER_TOF` + `LASER_ALT` today:** +2900 B flash and +728 B RAM. The scout estimated about 2-2.5 KB and
0.4 KB. Tasks 4 and 9 should expect a cost close to this, because the fusion becomes always compiled.

The gate on the baseline build found no new warnings in `src/` and none in `PlutoPilot.cpp`.

## Log fields ( temporary `PlutoPilot.cpp`, task 1 )

One record per `plutoLoop ( )` tick at 10 Hz, about 95-110 bytes per tick ( the budget is ~130 ). The record runs
only in Developer Mode with a live RC link. A `START` line on its own tick marks each `onLoopStart ( )`, so
Developer Mode restarts show up in the log.

| Field | Meaning | Unit |
|---|---|---|
| `t` | `millis ( )` | ms |
| `n` | tick counter; it steps by 2 across a lost record and by 1 across a stall | - |
| `Baro` | `BaroAlt`, compensated baro altitude | cm |
| `Est` | `getEstAltitude ( )`, the altitude the position loop flies | cm |
| `AltH` | `AltHold`, the altitude setpoint | cm |
| `Vz` | `VelocityZ`, the estimator's vertical velocity | cm/s |
| `Thr` | `rcCommand [ THROTTLE ]` | µs |
| `Arm` | 1 armed, 0 disarmed; ends the record | - |

Task 10's hover B flies this same `PlutoPilot.cpp` on the new firmware, so the two logs compare field for field.
Hover A adds the laser fields.

## Test 1: baro-only baseline hover ( `logs/log-1.txt` )

**Build.** `rollback/BASELINE_STOCK_PRIMUS_X2_v1_3.11.3_c495d6a.hex`, which is also in
`Build/PRIMUS_X2_v1/DEFAULT_PRIMUS_X2_v1_3.11.3.hex` right now. No lasers are needed: unplug them or leave them,
since the stock firmware does not touch them.

**Site.** Indoor, no fan or draught. Note the floor surface and the start temperature.

**Sequence ( ground-flight-ground with a long hover ).**

1. Power on and wait a few seconds. Connect PlutoMonitor, turn on Developer Mode and start logging.
2. Leave it 20-30 s disarmed on the floor.
3. Arm, take off in **ALT_HOLD**, bring it to about 1 m and centre the throttle stick.
4. Hover **3 min or more** with no throttle input. Use small pitch / roll corrections only.
5. Land, disarm, and leave it 20-30 s on the floor.
6. Stop logging and paste the whole log into `logs/log-1.txt`.

**Note what the log cannot show:** visible sink or climb, bumps, wind, and anything odd.

**What we read from it ( task 10 compares hover B against it ):**
- the `Est` and `Baro` trend over the hover, in cm/min for several window starts;
- the hold error `Est − AltH` ( mean, spread );
- the hover `Thr`;
- ground closure ( `Baro` after landing minus before arming );
- restarts and gaps.

### Result ( log-1, 14 Oct 2026, 13:10, Wi-Fi `Rx_ESP` )

651 records over 65 s: ground 0-6 s, **armed 6.2-60.6 s ( 54 s )**, ground 61-65 s. No field dropouts.

The hover is **shorter than the planned 3 min**, so the trend below is not reliable: the slope swings by
several cm/min with the window start. What this log does give is the hold error, the wobble, the climb to the
setpoint and the ground closure. Hover B in task 10 must fly **the same sequence and length** ( or both are reflown
longer ) to be comparable.

| Measure ( settled window 20-56 s, 340 records ) | Value |
|---|---|
| Setpoint `AltH` | 120 cm, then 114 cm from 44 s ( it moved during the restart burst, see below ) |
| Hold error `Est − AltH` | mean **−4.3 cm**, sd **5.7 cm**, range −18 … +10 cm |
| `Est` spread / trend | sd 5.3 cm; trend −1.3 cm/min ( 20-56 s ), 0.0 ( 20-42 s ), −0.8 ( 30-56 s ): flat within noise |
| `Baro` spread | sd 16 cm ( raw baro noise; the estimator smooths it to the 5 cm above ) |
| `Vz` spread | sd 6.2 cm/s |
| Climb to setpoint | `Est` within ~10 cm of 120 about 10 s after arming ( error −46 cm at 8 s, −12 cm at 16 s, ≈ −5 cm from 20 s ) |
| Ground closure | `Baro` after landing − before arming = **+3.3 cm** |
| `Thr` | 1499 throughout the hover: this is the throttle **stick** ( centred, no input ), not the motor throttle. The hover throttle is not in this log |

**Developer Mode restarts.** There are 4 `START` markers at 42.4-43.4 s, plus a 1.0 s gap at 44.5 s with `n` stepping
by 1. That means user code was off because the app's RC frames were late; it was not a lost record. At the same
time the setpoint moved from 120 cm to 114 cm. This is the known Wi-Fi hiccup pattern ( pluto-flighttest §1 ) and is
unrelated to this topic. Expect something similar in hover B.

**Baseline for task 10 hover B:** hold error mean −4.3 ± 5.7 cm, `Est` sd ~5 cm, ground closure ~3 cm, climb ~10 s.
Differences under ~8 cm ( the drone's normal wobble ) are not a result.

## Test 2: phase-1 bench check ( task 6, `logs/log-2.txt` )

**Build.** The task-5 firmware plus the bench `PlutoPilot.cpp` below: `builds/BENCH1_phase1_PRIMUS_X2_v1.hex`, which
is also in `Build/PRIMUS_X2_v1/DEFAULT_PRIMUS_X2_v1_3.11.3.hex`. 107.1 KB / 17.5 KB, gate clean.

**Hardware.** Down-laser with its XSHUT on **GPIO_5**, plus the 4 side lasers ( LEFT, RIGHT, FRONT, BACK ). Props
**off**, on the bench.

**What the bench code does.**
- **In `plutoInit ( )`:**
  - `initAlt ( GPIO_16 )`: should be refused, because GPIO_16 is FRONT's XSHUT;
  - `Peripheral_Init ( ADC_1 )` then `initAlt ( GPIO_2 )`: should be refused ( ADC alias in use );
  - `initAlt ( GPIO_5 )`: the real request;
  - `initObjectAvoidance ( 50, LEFT, RIGHT, FRONT, BACK )`, which enables the side lasers ( OA stays off ).
- **In `onLoopStart ( )`:** `initAlt ( GPIO_5 )` again, which should be refused after start-up.
- **Every tick:** `Peripheral_Write ( GPIO_5, STATE_LOW )`. This tests the guard: if the guard failed, DOWN would
  reset and `DS` would turn to 3.

**Log lines.** A `START` line on its own tick, then a cycle every ~10 s:

| Line | Fields | Expected |
|---|---|---|
| refusals ( tick 1 ) | `RSide RAdc RMain RLate` | `0 0 1 0` |
| scan ( tick 2, disarmed ) | `Scan` = bitmask, bit n means a VL53L0X answers at 0x29 + n; `I2Cerr` = I2C error count | `Scan:158` ( 0x2A-0x2D + 0x30, 0x29 empty ). `I2Cerr` grows by 3 per scan ( the 3 empty addresses ) |
| statuses ( tick 3 ) | `SL SR SF SB SE SD` ( 0 not enabled, 1 OK, 2 boot failed, 3 I2C error, 4 out of range ) | `1 1 1 1 0 1` ( or 4 for a side laser seeing nothing within range ) |
| ranges ( other ticks ) | `t L R F B D` ( mm, −100 = no valid sample ), `DS` ( DOWN status ), `TH` ( estimator laser height, cm ), `Src` ( 1 laser, 0 baro ) | each laser gives a believable distance; `DS:1`; `Src:1` while DOWN is in range |

### Sequence

1. **Cold start.** Power off for 10 s, then power on. Connect PlutoMonitor, turn on Developer Mode and start
   logging. Leave it still for ~20 s, which covers two scan / status cycles.
2. **Each laser.** Hold a hand ~10 cm in front of LEFT, then RIGHT, FRONT and BACK, about 3 s each. Only that
   laser's value should drop.
3. **T4.** Hold the drone level with DOWN pointing at the floor, about 30-60 cm up. Then put a hand ~5 cm in front of
   **FRONT** for 5 s, three times. **`D` and `TH` must not move.** Only `F` drops.
4. **DOWN.** Move a hand under the drone, from ~30 cm to ~10 cm and back. `D` follows it, and `F` does not move.
5. **Warm reset.** Without removing the battery, reflash the same hex ( or reboot through the app / USB ). Log again
   for ~20 s. All five lasers must come back with `Scan:158` and every status `1`. This step shows the XSHUT on
   GPIO_5 really resets DOWN's address.
6. **Optional, no wire.** Unplug DOWN's XSHUT wire, change `initAlt ( GPIO_5 )` to `initAlt ( )`, power-cycle and log
   ~20 s. Then do a warm reset ( step 5 ) and log ~20 s more. Expected: `Scan:158` both times, which shows the probe
   at 0x30 works after a warm reset.
7. **Optional, fitted but disabled.** Enable only `LEFT, RIGHT, BACK`, leaving FRONT fitted but disabled.
   - **If FRONT is awake at 0x29** ( PA15 pull-up ): every release is refused, DOWN's included, so all statuses read 2
     ( `BOOT_FAILED`, step `BUS_BUSY` ). `Scan` shows only bit 0 ( 0x29 ). That is the safe outcome: no merged
     readings.
   - **If something else holds PA15 low** ( e.g. the WS2811 status pin ): FRONT stays off, and all enabled lasers come
     up normally.

Paste everything into `logs/log-2.txt`, in the order of the steps above, with a short note per step ( e.g.
`--- step 3 T4 ---` ).

### Result ( log-2, 14 Oct 2026, 14:26, Wi-Fi `Rx_ESP`, cold start )

One continuous session: 1054 records over 110 s and 11 one-shot cycles. The log has no step markers, so the steps
below are read from the ranges.

| Check | Result |
|---|---|
| Refusals `RSide RAdc RMain RLate` | `0 0 1 0` in all 11 cycles: **pass** |
| I2C scan | `Scan:158` in all 11 cycles: DOWN alone at 0x30, the side lasers at 0x2A-0x2D, 0x29 empty. **The H1 collision is gone.** |
| Statuses `SL SR SF SB SE SD` | `1 1 1 1 0 1` in 10 cycles; one `SB:4` ( BACK out of range while held up ): **pass** |
| GPIO_5 guard | `Peripheral_Write ( GPIO_5, LOW )` was called on all 1054 ticks, and `DS` was never 3: **pass** |
| Each side laser | the hand shows on its own laser only: R dips at ~12 s, B at 24-26 s, L at 28 s and 90 s, B at 92 s: **pass** |
| **T4** ( held at ~90 cm, hand at FRONT, 76-82 s ) | F 257 → 67 mm. With F < 300 mm ( 72 records ): `D` 905 ± 11.5 mm, `TH` 89.9 ± 0.4 cm. With no hand ( 213 records ): `D` 907 mm, `TH` 89.9 cm. **FRONT never reaches altitude hold: H2 closed on hardware** |
| Step 4 ( hand under DOWN, 98-100 s ) | `D` 873 → 166 mm with F steady at ~1000 mm: **pass** |
| Step 5, warm reset | **not done** ( cold start only ). Moved to task 8, whose reflash is done without removing the battery |
| Steps 6-7 ( optional ) | not done |

**Notes for later tasks.**

- **First side sample is 8191.** Each side laser's first sample after start-up reads 8191 mm with a valid status. In
  continuous mode, start + read returns a result before the first measurement exists ( F5 ). It does not happen
  again in the run. Task 7's data-ready read removes it.
- **`Src:0` while handling.** From 31.5 s to ~38 s, while the drone was being picked up, `Src` was 0 ( baro ) even
  though `D` was valid ( 40-80 mm, `DS:1` ). Tilt rejection plus the dropout / return logic would explain it. That
  is fusion behaviour, unchanged by this topic. Task 10's hover compares the handover against the `LASER_ALT`
  build.
- **Two unexplained I2C errors.** `I2Cerr` grows by 3 per scan ( the 3 empty addresses ), except twice when it grows
  by 4. The counter covers the whole bus ( IMU, baro, mag ). The log cannot say which device; noted only.

## Test 3: phase-2 bench check ( task 8, `logs/log-3.txt` )

**Build.** The task-7 firmware ( paced side poll, cached getters ) plus the bench `PlutoPilot.cpp` below:
`builds/BENCH2_phase2_PRIMUS_X2_v1.hex`. 107.4 KB / 17.6 KB, gate clean. The user code calls `initAlt ( GPIO_5 )`
and `initObjectAvoidance ( 50, LEFT, RIGHT, FRONT, BACK )`, plus **`enableOA ( )`**, so that OA's cost is measured.
**Props off, disarmed**: OA writes roll / pitch overrides.

**Log lines.** A `START` line on its own tick, then:

| Line | When | Fields | Expected |
|---|---|---|---|
| timing | every second | `TUmax` / `TUavg`: worst / mean µs of the laser task ( DOWN poll + side poll ); `TCalls`: laser-task calls in that second; `OAmax`: worst µs of `applyObjectAvoidance ( )`; `Cyc`: `cycleTime` µs | `OAmax` **< 1000** ( it was ~4000-6000 with blocking reads ); `TUmax` ≈ 2000 or less ( reviewer estimate ) |
| rates | every second | `nL nR nF nB nD`: new samples per laser in the last second ( Hz ) | `nD` ≈ 30 ( 33 ms, unchanged ); each side **≥ 7** ( an update at least every ~150 ms ), likely ~20-25 |
| scan | every ~10 s, disarmed | `Scan` | `158` |
| statuses | every ~10 s | `SL SR SF SB SD` | all `1` ( or `4` for a side laser that sees nothing in range ) |
| ranges | other ticks | `t L R F B D TH Src` | believable distances; no `8191` at start-up |

### Sequence

1. **Warm reset ( from task 6 ).** With the battery connected and the drone on, flash this hex over Wi-Fi / USB
   **without removing the battery**, so the lasers keep power. Do not power-cycle. Connect PlutoMonitor, turn on
   Developer Mode and log ~20 s. Expected: `Scan:158` and every status OK. This shows the bring-up resets lasers
   that kept their addresses.
   - If your flash method always cuts power, flash, power-cycle and log ~20 s. Then reboot from the app ( or
     press reset ) with the battery connected, and log ~20 s more.
2. **Still, ~30 s.** Leave it on the bench. This gives clean timing and rate numbers.
3. **Each side laser.** Hand ~10 cm in front of LEFT, RIGHT, FRONT and BACK in turn, ~3 s each. Only that range
   drops, and it follows the hand without lag you can see.
4. **T4 again.** Hold it level at ~30-60 cm over the floor and put a hand at FRONT three times. `D` and `TH` must
   not move.

Put a `--- step N ---` line before each part in `logs/log-3.txt`.

### Result ( log-3, 14 Oct 2026, 15:25, Wi-Fi `Rx_ESP`, warm boot )

916 range records over 120 s, with 118 timing / rate lines and 12 scan / status cycles. The flash was done **with the
battery connected** ( user ), so this run was a **warm reset**: every laser kept power and its old address.

| Check | Result |
|---|---|
| **Warm reset** | `Scan:158` in all 12 cycles, every status OK: the bring-up reset lasers that kept their addresses. **Pass** ( moved here from task 6 ) |
| OA cost `OAmax` | **29-32 µs** ( it was ~4-6 ms of blocking I2C every 100 ms ): **pass**, < 1 ms |
| Laser task `TUmax` / `TUavg` | worst **1934-2199 µs** ( one DOWN start + read plus one side read in the same call ), mean **13 µs**: **pass**, within the ~2 ms estimate |
| Laser-task calls | ~12 000 a second; the periodic tasks round-robin on every free loop pass. The 10 ms / 33 ms pacing keeps the I2C rate fixed |
| DOWN update rate `nD` | **29-31 Hz** ( 33 ms ), unchanged: **pass** ( R6 ) |
| Side update rates `nL nR nF nB` | **23-26 Hz** each, against the ≥ 7 Hz target: **pass** |
| First sample | no 8191 at start-up ( it was 4 × 8191 in log-2 ): **the data-ready read removes it** |
| Each side laser | the hand shows on that laser only ( L 12 s, R 16-18 s, F 22 s, B 30 s; again at 88-110 s ): **pass** |
| T4 | not repeated cleanly in this log. The per-laser state behind T4 is unchanged since log-2's pass; task 10's hover A checks it again in flight |

**Notes.**

- **One 8190 from BACK** at ~98 s, with a valid status, while BACK pointed at a far target ( median 2023 mm that
  second ). The sensor reports far targets this way. OA ignores it ( beyond the avoid distance ).
- **`Cyc` ( loop cycle ) above 4 ms in 18 of 118 one-second snapshots** ( max 4552 µs; `looptime` 3500 µs ). The
  likely cause is the laser task's ~2 ms peak, when DOWN's 33 ms read and a side read land in the same call. That
  is far below the old 4-6 ms OA burst every 100 ms. No baseline `Cyc` was logged, so task 10's hovers judge it. If
  it matters, a side read can be skipped in a call where DOWN read ( worst case ~1.3 ms ).

The temporary timing code is removed after this run ( task 8 ).

## Build gate ( task 9, 14 Oct 2026 )

Every build includes all task 2-8 code. The root `PlutoPilot.cpp` is the task-1 baseline log ( temporary ). Toolchain:
Arm GNU 14.2.1.

| Build | text / data / bss | Flash / RAM | Warning gate |
|---|---|---|---|
| `PRIMUS_X2_v1` ( default ) | 107892 / 1280 / 16756 | **106.6 KB / 17.6 KB** | no new warnings in `src/` |
| `PRIMUS_V5` | 107884 / 1280 / 16756 | 106.6 KB / 17.6 KB | no new warnings in `src/` |
| `PRIMUSX2` | 107712 / 1280 / 16912 | 106.4 KB / 17.7 KB | no new warnings in `src/` |
| `PRIMUS_X2_v1` + `LASER_TOF_L1x` ( check ) | - | 114.8 KB / 18.4 KB | builds |
| `PRIMUS_X2_v1` + `LASER_TOF_L1x` + `LASER_ALT` ( check ) | - | 114.6 KB / 18.4 KB | builds |
| `PRIMUS_X2_v1` + `#define LASER_TOF` ( guard check ) | - | - | **fails as intended**: `altitudehold.cpp:113: #error "LASER_TOF is replaced by XRanging.initAlt ( ) …"` |

`target.h` was restored after each check build, so it holds only the task-4 comment edits. The flight hex is
`Build/PRIMUS_X2_v1/DEFAULT_PRIMUS_X2_v1_3.11.3.hex` ( default build, gated ).

**Against the start of the topic ( PRIMUS_X2_v1 ):**

| Build | Flash | RAM |
|---|---|---|
| today's stock ( baro, with the baseline log ) | 102.1 KB | 16.8 KB |
| today's `LASER_ALT` ( rollback hex ) | 104.2 KB | 17.5 KB |
| **this topic ( baro or laser chosen at boot, plus the ToF manager )** | **106.6 KB** | **17.6 KB** |

This is +2.4 KB flash and +0.1 KB RAM over today's `LASER_ALT` build. Headroom is 149 KB of flash and 22 KB of RAM.

## Test 4: hover A, laser + OA ( task 10, `logs/log-4.txt` )

**Build.** `builds/HOVER_A_laser_OA_PRIMUS_X2_v1.hex` ( task 2-8 firmware, gated ). Its user code is
`initAlt ( GPIO_5 )`, `initObjectAvoidance ( 50, LEFT, RIGHT, FRONT, BACK )` and `enableOA ( )`: avoid anything
within 50 cm.

**Log** ( ~110 B per tick, 10 Hz ): `t n Est AltH D TH Src F Arm`, plus `START` on each Developer Mode restart.

| Field | Meaning |
|---|---|
| `Est` / `AltH` | estimator altitude / setpoint, cm |
| `D` | DOWN range, mm ( −100 = no valid sample ) |
| `TH` | the estimator's laser height, cm |
| `Src` | 1 laser, 0 baro, 2 object hold-off |
| `F` | FRONT range, mm ( the wall approach ) |

**Site.** Indoor, flat floor, the same room as log-1 if possible. You need a clear wall with ~2 m of free floor in
front of it. Keep the hover **below 140 cm**, so the laser stays in charge ( the handover to the baro is at 160 cm ).

**Rollback.** If anything flies wrong, land and flash `rollback/ROLLBACK_LASER_ALT_PRIMUS_X2_v1_3.11.3_c495d6a.hex`.

### Sequence

1. Power on and wait for the boot LED animation. Connect, turn on Developer Mode and start logging.
2. Leave it 20 s disarmed on the floor.
3. Arm, take off in **ALT_HOLD** to ~1 m, and centre the throttle.
4. **Hover ~60 s**, at least 2 m from the wall, with no throttle input and only small corrections.
5. **Wall approach.** Pitch slowly towards the wall, at a walking pace or slower. At ~50 cm, OA should push back. Let
   it, keep a light forward stick for ~3 s, then pitch away.
6. Land, disarm, and leave it 20 s on the floor. Stop logging.

**Pass criteria.**
- The hold error `Est − AltH` is within ±5 cm for **≥ 95 % of a 30 s window** in step 4. The reference is the
  `LASER_ALT` hover in tof-althold-fusion test 3b: −0.9 ± 1.8 cm, 100 %.
- `Src` stays 1 throughout the hover.
- In step 5, OA reacts before ~40 cm, and `Est` / `AltH` do not jump while FRONT sees the wall ( H2 in flight ).
- No `D:-100` runs and no unexplained `Src:0`.

If the hold error misses, we fly the rollback hex with the same sequence, to tell a regression from the day's
conditions.

## Test 5: hover B, baro only ( task 10, `logs/log-5.txt` )

**Build.** `builds/HOVER_B_baro_PRIMUS_X2_v1.hex`: the task 2-8 firmware with the task-1 baseline user code, so
**no XRanging calls**. The lasers stay fitted but unused, which is the D5 "no `initAlt` = today's baro build" case.

**Log.** The log-1 line: `t n Baro Est AltH Vz Thr Arm`.

### Sequence

The same as log-1:
1. 20-30 s disarmed.
2. Arm, ALT_HOLD to ~1 m, centre the throttle.
3. Hover **~60 s**.
4. Land, then 20-30 s disarmed.

**Pass criteria.** Compared with log-1 ( hold error −4.3 ± 5.7 cm, `Est` sd ~5 cm, ground closure +3.3 cm, climb
~10 s ), every figure must be within ~8 cm. That is the drone's normal wobble.

### Result ( log-4, hover A, 14 Oct 2026, 15:57, analysed by `pluto-log-analyst` )

The flight did not follow the plan. Armed 4.4-199.6 s:
- a wall approach at ~15-34 s;
- a hands-off hover at AltH 120 cm, 39-94 s;
- deliberate throttle moves 87-178 s ( user: on purpose );
- a high excursion to ~160 cm at 183-197 s;
- a two-stage landing.

There was one `START` ( at the start ), one lost-record gap of 0.6 s, and one truncated record.

| Check | Result |
|---|---|
| **H2 in flight**: FRONT at the wall | F 946 → 174 mm ( and a second dip to 235 mm ). AltH stays at 120, with no Est / AltH / TH jump tied to F. The hold error *shrank* ( −10.9 → −5.8 cm ). **Pass** |
| OA push-back | F recovers above 1 m within ~3 s of each minimum. **Works** |
| Down-laser health | 5 isolated `D:-100` records in 195 s armed; no run. `TH` ≈ `D/10` ( ratio 1.00 ). **Pass** |
| `Src` | One 0.5 s object hold-off in the 39-94 s hover ( a real −35 cm laser step, nowhere near the wall ). The other 1↔2 toggles and two `Src:0` spells ( at 140-163 cm, the handover band ) fall in the throttle-moving part. **As designed** |
| **Hold quality** ( 39-94 s, Src 1, AltH 120 ) | Best 30 s window ( 59-89 s ): Est − AltH −0.6 ± **4.2** cm, **82 %** within ±5 cm. Longest Src-1 stretch ( 54-94 s ): sd 5.1 cm, 75 %. **Miss**: the target is ≥ 95 % ( reference −0.9 ± 1.8 cm, 100 % ) |
| The bob | ~8.5 s period, ±10 cm. The raw laser swings with it ( `D/10` sd 12.9 cm against `Est` sd 10.4 cm, in step ), so **the craft really moves**; it is not an estimator artefact. The user felt "bobbing more than usual" |
| Climb | within 10 cm of AltH 13.5 s after arming |

**Verdict.** The pipeline itself passes: every laser is healthy, FRONT stays out of altitude hold, and OA works.
**Altitude hold bobs**, and on this evidence it could be a regression or the day's conditions. As planned, we fly
an A/B against today's `LASER_ALT` build ( tests 6-7 ).

**Tooling note** ( from the analyst ): `tools/flightlog.py` `load ( )` mis-parses logs that start with `START` and
have a field literally named `t`. It silently replaces the time with index × 0.1 s. Added to
`.claude/TOOLING_BACKLOG.md`.

## Tests 6-7: A/B against today's `LASER_ALT` build ( task 10, `logs/log-6.txt` R, `logs/log-7.txt` A2 )

The aim is to tell a regression from the day's conditions behind hover A's bob. Both builds log the same line,
`t n Est AltH Df TH Src Arm`, where `Df` is the down-laser's 0.1 IIR range in mm. **Neither runs object
avoidance**, so OA is out of the comparison.

| Run | Hex | What it is |
|---|---|---|
| **R** → log-6 | `builds/AB_R_head_LASER_ALT_PRIMUS_X2_v1.hex` ( 104.9 KB / 17.5 KB ) | today's HEAD `c495d6a` with `LASER_TOF` + `LASER_ALT`: the **old** pipeline. Built in a scratch worktree, so the topic's working tree is untouched. `Df` = `NewSensorRange` |
| **A2** → log-7 | `builds/AB_A2_new_initAlt_noOA_PRIMUS_X2_v1.hex` ( 106.7 KB / 17.6 KB, gated ) | the **new** pipeline with `initAlt ( GPIO_5 )` only: no side lasers, no OA. `Df` = `tofDownFiltMm ( )` |

**Same conditions for both.** Fly them back to back on the **same battery**, R first, in the same room and spot,
**≥ 2 m from walls**. Note the battery level ( app % or voltage ) before each run.

**Sequence, each run.**
1. 20 s disarmed.
2. Arm, ALT_HOLD to ~1 m, centre the throttle.
3. **60 s hands-off hover**: no throttle input, only small pitch / roll corrections.
4. Land, 20 s disarmed.

Fly hover B ( log-5 ) afterwards if the battery allows.

**How the result is read.** In each run, take the best 30 s Src-1 window, and the `Est` and `Df` swing ( sd, and the
period of the ~8.5 s bob ).

| R | A2 | Means |
|---|---|---|
| bobs too | bobs | conditions ( battery, props, room ): not this topic; hover A is no worse than today |
| clean | bobs | **regression in the new pipeline**: investigate before docs ( a new task ) |
| clean | clean | the bob in hover A came from OA ( it blends roll / pitch ) or from that flight: re-fly hover A with OA |

### Result ( log-6, run R, 14 Oct 2026, 16:20 ): void, and it reproduces the original bug

`Df` = 0 and `TH` = 0 in **all 1345 records**, and `Src` = 0 ( baro ) for the whole armed time ( 9.7-117 s ). Today's
`LASER_ALT` build never got a single down-laser sample. The four side lasers were fitted, and with no XRanging calls
nothing holds them in reset, so they were awake at 0x29 ( FRONT's PA15 pull-up, LEFT's PB13 SPI2 SCK ). The old
`ranging_init ( )` then configured every chip at 0x29 at once, failed, and latched `Global_Status`. This is the
check topic's **H3 / F3 failing on hardware**: fitted lasers break the old down-laser even with no XRanging in user
code.

**Consequences.**
- R is a valid reference only with the **side lasers unplugged**.
- The A2 hex calls only `initAlt`. With the side lasers fitted but not enabled, the new bring-up would find 0x29 busy
  and refuse DOWN ( `BUS_BUSY`, safe ). So A2 is also flown with the side lasers unplugged.
- **For the wiki:** with fitted side lasers, enable them ( `initObjectAvoidance ( … )` / `XRanging.init ( … )` ),
  even without `enableOA ( )`, or `initAlt` finds 0x29 busy.

### A/B, corrected setup ( user, 14 Oct 2026 )

Fly both runs with **only the down-laser fitted** ( the 4 side lasers unplugged, DOWN's XSHUT on GPIO_5 ), back to
back on one battery, with the test 6-7 sequence:

| Run | Hex | Log |
|---|---|---|
| R2 ( old pipeline ) | `builds/AB_R_head_LASER_ALT_PRIMUS_X2_v1.hex` | `logs/log-8.txt` |
| A2 ( new pipeline, `initAlt ( GPIO_5 )` only ) | `builds/AB_A2_new_initAlt_noOA_PRIMUS_X2_v1.hex` | `logs/log-7.txt` |

Check in each log that `Src` is 1 and `Df` > 0 during the hover. Then fly hover B ( log-5 ) if the battery allows.

### Result ( log-8, run R2, today's `LASER_ALT` build, side lasers unplugged, 14 Oct 2026 )

2255 records over 232 s; armed 14.9-218.6 s. Setpoints flown: 120 cm ( 17-84 s ), 134, 81, then 61 cm ( 133-217 s ).
The laser worked this time ( `Src` 1 in 1725 of 1988 armed records ). One `START`, two early lost-record gaps.

| Setpoint, Src 1 | Est − AltH | Within ±5 cm | 30 s windows, % within ±5 cm | `Df/10` sd |
|---|---|---|---|---|
| **120 cm** ( 649 records ) | −2.5 ± **4.7** cm | 77.8 % | min 68, **median 86**, max 93 % | 5.6 cm |
| 61 cm ( 668 records ) | −0.9 ± 4.8 cm | 71.7 % | min 64, median 68, max 94 % | 6.2 cm |

The best 30 s window ( 41-71 s at 120 cm ) has 93.2 % within ±5 cm, sd 3.0 cm, with a ~10 s bob.

**Reading.** **Today's own `LASER_ALT` build misses the ≥ 95 % bar too** ( best 93 %, median 86 % ) and bobs with a
similar period. Hover A ( best 82 %, sd 4.2-5.1 cm ) sits inside R2's spread. So most of hover A's bob is the
day's conditions ( battery, props, room ), not this topic. The documented 100 % reference is from 21 Sep. A2 ( log-7,
the same hardware, no OA ) is the clean comparison that settles it.

### Correction ( 14 Oct 2026, 16:40 ): logs 6 and 8 were flown on **A2**, not R

The user flashed from PlutoIDE, which built the working tree at 16:19:46. `PlutoPilot.cpp` held the **A2** code
( `initAlt ( GPIO_5 )` only ) from 16:16:04, and the built ELF has the new pipeline's symbols
( `tofManagerPollSides`, `tofDownFiltMm` ). Both flights came after that build ( 16:20 and 16:30 ). The R hex
( today's `LASER_ALT` ) has **not been flown**. The two readings above are replaced by these:

- **log-6 = A2 with the side lasers fitted but not enabled.** The new bring-up found 0x29 busy ( the side lasers
  awake ) and refused the down-laser ( `BUS_BUSY` ). The fusion stayed on as D6 says and flew on the baro:
  `Df` = 0 and `Src` = 0 throughout. **This is the designed safe refusal on hardware.** It is not a reproduction of
  the old bug, which this log does not show. The wiki rule stands: with side lasers fitted, enable them.
- **log-8 = A2, down-laser only, no OA:** the new pipeline's clean hover. At 120 cm: Est − AltH −2.5 ± 4.7 cm, 77.8 %
  within ±5 cm; 30 s windows min 68, median **86**, max **93 %**; ~10 s bob. At 61 cm: median 68 %, max 94 %.

**Still needed:** **R** ( `builds/AB_R_head_LASER_ALT_PRIMUS_X2_v1.hex`, today's firmware ) on the same hardware
( down-laser only, no OA ), with the same hover at 120 cm. It must be **flashed as that hex**: a PlutoIDE build of the
working tree is always the new firmware. Compare it with log-8.

### Result ( log-7, R = today's `LASER_ALT`, down-laser only, XSHUT wired to GPIO_5, 14 Oct 2026, 16:39 )

R was really flown this time ( the IDE slot held the R hex; the md5 matches ). `Df` = 0 and `TH` = 0 in **all 1822
records**, and `Src` = 0 throughout: **today's firmware cannot use the down-laser on this board as it is now wired.**
The XSHUT wire added for this topic goes to GPIO_5 = **PB15**, which today's firmware sets up as **SPI2 MOSI**
( main.cpp:313 `spiInit ( SPI2 )`, target.h `SPI2_MOSI_PIN`). Nothing in it drives that line as an XSHUT, so the
down-laser is most likely held in reset, and `ranging_init ( )` gets no answer. Before this topic the down-laser had
no XSHUT wire.

**Consequence.** A same-day reference needs the board **without the XSHUT wire**: the only setup today's firmware
supports. The new firmware then runs its **no-wire path** ( `initAlt ( )`: probe 0x30, else move 0x29 → 0x30 ),
which has not flown yet.

## Tests 9-10: A/B, no XSHUT wire ( task 10 )

**Hardware for both runs.** Down-laser only. **Disconnect its XSHUT wire from GPIO_5** and leave the module's
XSHUT on its own pull-up, as before this topic. The 4 side lasers stay unplugged, and there is no OA. Both builds log
`Ver t n Est AltH Df TH Src Arm`, where **`Ver` 1 = old, 2 = new**, so each log names its firmware.

| Run | Hex | How to flash | Log |
|---|---|---|---|
| **R** ( old pipeline ) | `builds/AB2_R_old_nowire_Ver1_PRIMUS_X2_v1.hex` | already in PlutoIDE's slot: **flash only, do not build** | `logs/log-9.txt` |
| **A3** ( new, `initAlt ( )`, no wire ) | `builds/AB2_A3_new_initAlt_nowire_Ver2_PRIMUS_X2_v1.hex` | a normal PlutoIDE build + flash gives this one ( the working tree is A3 ) | `logs/log-10.txt` |

**Sequence, each run.** 20 s disarmed; arm; ALT_HOLD to **120 cm**; **60 s hands-off**; land; 20 s disarmed. Fly
back to back on one battery, R first. In each log `Src` must be 1 and `Df` > 0 during the hover. If `Df` stays 0 in
R, the XSHUT wire is still holding the chip.

### Result ( logs 9-10, R = today's firmware, 14 Oct 2026, 17:14-17:18 )

Both logs are `Ver:1` ( R ). The side lasers were unplugged, and there was no OA.

- **log-9: R with the XSHUT wire off.** This is the valid same-day reference. 918 records; armed 4-89 s;
  AltH 120 cm, then 104 cm.
- **log-10: R again, without a power cycle, with the XSHUT wire connected to GPIO_5 while running.** `Df` froze
  at 846 mm at 9.7 s, when the wire was connected, and never changed again. The craft flew on the baro throughout
  ( `Src:0` ), and a hand under it got no response ( user ). This **confirms on hardware** that today's firmware
  drives PB15 as SPI2 MOSI: an XSHUT on it resets the laser, and the old `Global_Status` latch keeps it dead. The new
  firmware owns the pin as an XSHUT ( and task 14 switches SPI2 off ).

### A/B verdict: R ( log-9 ) against A2 ( log-8 ), at 120 cm, laser source

| | R: today's firmware | A2: new pipeline |
|---|---|---|
| Raw laser motion `Df/10` sd ( real craft movement ) | **5.53 cm** | **5.56 cm** |
| Hold error sd, median 30 s window | 3.26 cm | 3.41 cm |
| Within ±5 cm, 30 s windows | min 71 / **median 93** / max 95 % ( 23 windows ) | min 68 / **median 86** / max 93 % ( 43 windows ) |
| Within ±5 cm, the whole Src-1 stretch | 81.6 % ( −2.1 ± 4.0 cm ) | 77.8 % ( −2.5 ± 4.7 cm ) |

**No regression in altitude hold.** The craft moves by the same amount on both firmwares, and the error spreads
differ by 0.15 cm, far below the ~8 cm wobble that counts as a result. The higher median % on R comes from a shorter,
single-setpoint flight ( 23 windows against A2's 43, which took in several setpoints ). Neither build reaches the
21 Sep reference ( 100 % ) today: **the day's bob is conditions**. The task-10 pass criterion is therefore read
against the same-day R ( ≥ 95 % is not met by today's build either ).

**Still open:** hover A ( log-4: OA on, all lasers fitted ) moved much more ( `D/10` sd 12.9 cm ). Per the A/B table
( R clean-ish and A2 alike → OA or that flight ), hover A is re-flown on the current firmware ( tasks 2-8 + 14 ) as
test 11. That run also gives task 14 its 5-laser check.

## Test 11: hover A re-flown on the current firmware ( task 10, `logs/log-11.txt` )

**Build.** The working tree, built and flashed from PlutoIDE ( a copy is at `builds/TEST11_hoverA_refly_PRIMUS_X2_v1.hex` ).
This is tasks 2-8 + 14 with `initAlt ( GPIO_5 )`, `initObjectAvoidance ( 50, LEFT, RIGHT, FRONT, BACK )` and
`enableOA ( )`. 107.1 KB / 17.6 KB, gate clean.

**Hardware.** XSHUT wire **on GPIO_5**, and all 4 side lasers **plugged in**. **Power-cycle before flying.**

**Log.**
- First tick: `SPIoff:1 Flash:0` ( task 14 ).
- Second tick: statuses `SL SR SF SB SD`, all 1 expected.
- Then the A/B line `Ver:2 t n Est AltH Df TH Src Arm`.

**Sequence.** 20 s disarmed; arm; ALT_HOLD to **120 cm**; **60 s hands-off**, ≥ 2 m from walls so OA stays quiet;
land; 20 s disarmed.

**Read against.** A2 ( log-8: the same pipeline, no OA, no side lasers ): `Df/10` sd 5.56 cm, window error sd
3.41 cm, median 86 %. Today's firmware R ( log-9 ): 5.53 cm, 3.26 cm, 93 %.
- If test 11 matches them, hover A's larger motion ( log-4, 12.9 cm ) was that flight, not OA.
- If it moves clearly more again, OA's roll / pitch override is the cause, and that goes to an OA topic, not this one.

### Result ( log-11, test 11, 15 Oct 2026, 08:47, `Ver:2` )

1779 records; armed 8-169 s; setpoints 120 cm ( main ), 76 cm and a short climb to ~200 cm. One `START`, no gaps.
- **Task 14:** `SPIoff:1 Flash:0`. **SPI2 released and the flash absent on hardware: pass.**
- **Statuses ( second tick ):** `SL:1 SR:1 SF:3 SB:1 SD:1`. **FRONT was in I2C error** at the start of the run and
  stayed out for the flight ( one-error rule ). It was healthy in all 23 earlier status cycles ( logs 2-3 ). The side
  lasers had been unplugged and replugged for the A/B, so a **loose FRONT connector** is the likely cause ( user ).
  It is re-checked on the bench ( test 12 ). The user keeps the one-error rule; no 3-strike tolerance.

| At 120 cm, laser source | **Test 11**: new + OA, all lasers | A2: new, no OA ( log-8 ) | R: today's, no OA ( log-9 ) |
|---|---|---|---|
| Raw laser motion `Df/10` sd | **8.98 cm** | 5.56 cm | 5.53 cm |
| Error sd, median 30 s window | 3.90 cm | 3.41 cm | 3.26 cm |
| Within ±5 cm, 30 s windows | min 73 / median **81** / max 85 % | 68 / 86 / 93 % | 71 / 93 / 95 % |
| Whole Src-1 stretch | −4.7 ± 7.9 cm, 65.8 % | −2.5 ± 4.7 cm, 77.8 % | −2.1 ± 4.0 cm, 81.6 % |

**Reading.** With OA enabled, the craft moves ~60 % more under the down-laser ( 9.0 against 5.5 cm ), as in hover A
( log-4, 12.9 cm ). The estimator's own error barely changes ( 3.9 against 3.4 cm ). While enabled, OA overrides
roll / pitch with the pilot's stick at 30 % authority ( `USER_OA_BLEND_PUSH` ), so the craft drifts and tilts more and
the down-laser sees more floor variation. **That is OA behaviour, out of this topic's scope**; it is logged for an OA
topic. Without OA, the new pipeline holds altitude like today's firmware ( the A/B ).

## Test 12: FRONT re-check on the bench ( task 10 / task 14, `logs/log-12.txt` )

**Build.** The working tree: the test-11 code, with the status line now repeating every ~10 s. Build + flash from
PlutoIDE. **Props off, disarmed.** XSHUT wire on GPIO_5, all 4 side lasers fitted, and **FRONT's connector
reseated**.

**Sequence.** Power-cycle, connect, turn on Developer Mode and log **~2 min** untouched. Then hold a hand in front of
FRONT a few times.

**Pass.** `SPIoff:1 Flash:0` on the first tick, and every status line `SL:1 SR:1 SF:1 SB:1 SD:1` ( 4 is fine for a side
laser that sees nothing in range ). If FRONT still goes to 3, it is not the connector: the PA15 / WS2811 overlap or a
firmware fault, to investigate before closing.

### Result ( log-12, test 12, 15 Oct 2026, 09:11, FRONT reseated )

2480 records ( `Ver:2` ), one `START`, ~4 min on the bench.
- **`SPIoff:1 Flash:0`.**
- **26 status cycles: no laser ever in error.** Status 3 ( I2C error ) and status 2 ( boot failed ) never occur, on any
  laser. DOWN is 1 in every cycle. The side lasers read 1, or 4 ( out of range ) while facing open space: 6 cycles were
  all 1, and the rest mix 1 and 4.
- **FRONT never failed, so test 11's `SF:3` was the loose connector.**

**Task 14 passes on hardware** with all five lasers.

## Test 5 ( revised ): hover B, baro only, on the final firmware ( task 10, `logs/log-5.txt` )

**Build.** The working tree, with tasks 2-8 + 14 and **no XRanging calls**, built and flashed from PlutoIDE ( a copy
is at `builds/HOVER_B2_baro_task14_PRIMUS_X2_v1.hex` ). It replaces the older `HOVER_B_baro` hex, which predates
task 14. The log is the log-1 line plus `Ver`: `Ver t n Baro Est AltH Vz Thr Arm`.

**First tick:** `SPIoff:0 Flash:<size>`. With no XRanging, SPI2 must stay on and the flash present ( task 14's other
half ).

**Sequence.** As log-1: 20-30 s disarmed; arm; ALT_HOLD ~1 m; **60 s hands-off**; land; 20-30 s disarmed. The lasers
can stay fitted: nothing enables them.

**Pass.** Against log-1 ( hold error −4.3 ± 5.7 cm, `Est` sd ~5 cm, ground closure +3.3 cm, climb ~10 s ), every
figure within ~8 cm.

### Result ( log-5, hover B, 15 Oct 2026, 09:35, `Ver:2`, no XRanging )

929 records; armed 2.7-74.4 s ( 72 s ) at 120 cm; no restarts, no gaps.
- First tick: `SPIoff:0` means SPI2 stays on without XRanging: task 14's other half passes. It also printed
  `Flash:0`. Task 14 removed no line from `flash_m25p16.cpp` or `bus_spi.c` ( only guards, inactive while SPI2 is
  not released ), so the boot-time detection runs exactly as on HEAD. **The flash not being recognised is
  pre-existing**: the chip's JEDEC ID is not in the driver's list, or it does not answer. It is unrelated to this
  topic and noted for a hardware / driver check.

| Settled hover ( 14 s after arming until landing, AltH 120 ) | log-1: today's stock | **log-5: new, no XRanging** |
|---|---|---|
| Hold error `Est − AltH` | −5.9 ± 4.7 cm ( 42 % within ±5 ) | **−3.4 ± 6.1 cm** ( 61 % within ±5 ) |
| `Est` sd | 4.7 cm | 6.1 cm |
| Raw `Baro` sd | 15.9 cm | 21.1 cm |
| `Vz` sd | 4.9 cm/s | 3.7 cm/s |
| Climb to within 10 cm of AltH | 12.0 s | 14.6 s |
| Ground closure | +3.3 cm | 0.0 cm |

**Pass.** Every figure is within the ~8 cm wobble threshold of the baseline: the mean is closer to the setpoint, and
the spread is 1.4 cm wider, tracking the noisier raw baro that day. **"No `initAlt` = today's baro build" ( D5 )
holds in flight.**

## Task 10 verdict ( 15 Oct 2026 )

| Check | Result |
|---|---|
| Hover A pipeline checks ( log-4, test 11 ) | FRONT does not leak into altitude hold ( H2 in flight ); OA reacts; down-laser healthy |
| Hold quality with the laser | No regression against same-day today's firmware ( A/B: `Df/10` sd 5.53 / 5.56 cm, window error sd 3.26 / 3.41 cm ). Neither build reaches the 21 Sep 100 %: the day's conditions |
| OA enabled | The craft moves ~60 % more ( `Df/10` sd 9.0 against 5.5 cm ): OA behaviour, logged for its own topic |
| Hover B, baro only | Matches the baseline ( D5 ) |
| Found on the way | Today's firmware cannot run a down-laser whose XSHUT is wired to GPIO_5 ( PB15 = SPI2 MOSI; logs 7 and 10 ), so the wire needs the new firmware; SPI2 off with XRanging ( task 14 ); FRONT connector; pre-existing `Flash:0` |
