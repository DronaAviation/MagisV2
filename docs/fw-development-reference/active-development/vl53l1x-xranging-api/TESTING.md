# VL53L1X XRanging API - Testing

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [DESIGN](DESIGN.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

## 1. Baseline ( task 1, 16 Oct 2026 )

PRIMUS_X2_v1, FW 3.14.0, HEAD `5e26823`, Arm GNU Toolchain 14.2.1, clean builds through
`.claude/skills/pluto-build/driver.sh`.

| Build | text | data | bss | Flash | RAM |
|---|---:|---:|---:|---|---|
| Stock ( L0X family, no laser define ) | 108276 | 1280 | 16760 | 106.9 KB / 256 KB ( 41% ) | 17.6 KB / 40 KB ( 44% ) |
| `LASER_TOF_L1x` + `LASER_ALT` ( old single-L1X path ) | 116404 | 1320 | 17624 | 114.9 KB / 256 KB ( 44% ) | 18.5 KB / 40 KB ( 46% ) |
| Difference ( L1X − stock ) | +8128 | +40 | +864 | +8.0 KB | +0.9 KB |

Warning gate on the stock build: **no new warnings in `src/`**. 17 warnings are already fixed against the baseline
( battery.cpp, XRanging.cpp, FC-Config-Setpoints.cpp ). The baseline should be refreshed once they are committed,
not in this topic.

RAM symbols that matter for this topic ( `arm-none-eabi-nm -S` ):

| Symbol | Size | Build | Note |
|---|---:|---|---|
| `tofDev` ( L0X manager, 6 slots ) | 0x840 = 2112 B | both | stays in the L0X build |
| `XVision` ( `LaserSensor_L1` object ) | 0x3c4 = 964 B | **both** | removed by task 4: −964 B in every build |
| `MyDevice_L1` ( one `VL53L1_Dev_t` ) | 0x3a4 = 932 B | L1X | one L1X slot = 932 B; 6 slots ≈ 5.6 KB |
| `applyObjectAvoidance` ( flash ) | 0x298 = 664 B | both | moved into the shared core by task 6 |

Budget implied for the L1X build after tasks 2-4: the L1X manager replaces `MyDevice_L1`
( 6 × 932 − 932 = +4660 B ), `XVision` goes ( −964 B ), so about **+3.7 KB bss net**, giving ≈ 22 KB / 40 KB RAM.
The L0X `tofDev` table must not be linked into the L1X build, or the build is another 2.1 KB heavier.

`target.h` was put back to stock after the L1X build ( `git checkout`; `git status` clean under `src/` ).

## 2. Flight procedure: X1Ranging altitude hold + object avoidance ( task 7, flown in task 10 )

**Test file:** [test/PlutoPilot_x1ranging_test.cpp.txt](test/PlutoPilot_x1ranging_test.cpp.txt). It is also the current
repo-root `PlutoPilot.cpp`; restore the template before committing.

**Build and hardware.**

- The normal build: no define ( `LASER_TOF_L1x` and `LASER_ALT` are build errors now ). X1Ranging is chosen by calling it. Flash.
- With `X1Ranging.initAlt ( GPIO_5 )` the VL53L1X fusion copy runs ( `fu:1`, task 17 ): all bench and flight checks apply.
- Down VL53L1X: XSHUT on **GPIO_5**. Side VL53L1X: LEFT **GPIO_7**, RIGHT **GPIO_9**, FRONT **GPIO_16**, BACK **GPIO_15**.
- Object-avoidance distance: **50 cm** ( `OA_DISTANCE_CM` ). Avoidance acts only in Developer Mode.
- Save every log as a numbered `log-N.txt` in this folder.

**Log line** ( one frame: ~112 B per 100 ms tick with realistic values, 123 B worst case, under the 130 B budget ):

`t:… n:… D:… L:… R:… F:… B:… S:DLRFB src:… alt:… fu:… a:… r:… p:…`

| Field | Meaning |
|---|---|
| `t`, `n` | `millis ( )` and the tick count: `n` steps by 2 across a lost record |
| `D L R F B` | range in mm: −100 = no valid sample, −1 = not enabled |
| `S` | one status digit per laser, in the order D L R F B: 0 not enabled, 1 OK, 2 boot failed, 3 I2C error, 4 out of range |
| `src` | altitude source: 1 laser, 0 baro, 2 holding on the baro after an object step |
| `alt` | estimated altitude, cm |
| `fu` | laser fusion latched on |
| `a` | armed |
| `r`, `p` | avoidance roll / pitch override |

`START altInit:1` marks each Developer Mode ( re )start.

**Bench ( disarmed, drone held by hand )**

| # | Do | Pass |
|---|---|---|
| B1 | Power on, Developer Mode on | `START altInit:1`; `S` is all 1 or 4 ( no 2 or 3 ); `fu:1` |
| B2 | Hold a matte card 20 / 50 / 100 cm from each laser in turn | that laser's range is within ±2 cm of a tape measure; the others are unchanged |
| B3 | Card at < 50 cm from LEFT for **under 0.5 s**, sticks centred ( then repeat with nothing in range behind it: see the note below ) | `r` ≈ 25 for ~0.5 s, then `r` ≈ −11 ( brake ) for ~0.5 s, then `r` ≈ 0 ( ±1: the stick term is truncated ); LEFT's LED on while it sees the card. With the card **held**, the cycle repeats: 5 ticks ≈ 25, 5 ticks ≈ −11, 1 tick ≈ 0 |
| B4 | Repeat B3 for RIGHT / FRONT / BACK | push / brake: RIGHT `r` ≈ −25 / +11; FRONT `p` ≈ −25 / +11; BACK `p` ≈ 25 / −11 ( LEFT was 25 / −11 ) |
| B5 | Card at < 50 cm from LEFT and RIGHT together | `r:0` ( the two cancel ), as XRanging does |

**Flight ( app sticks, ALT_HOLD, Developer Mode on )**

| # | Do | Pass | Refutes |
|---|---|---|---|
| F1 | 20 s disarmed, arm, take off, hover at 80-100 cm for 60 s with no throttle input | `src:1`; `alt` follows `D` minus the ground reading; height holds within the usual ±8 cm wobble | `src` stays 0 with a valid `D`; drift or bobbing |
| F2 | Climb slowly to ~200 cm, hold 10 s, descend to ~100 cm | `src` 1 → 0 at about 160 cm and 0 → 1 below 140 cm; `alt` has no jump over ~10 cm at either handover | a height jump or a dip at the handover |
| F3 | Hover at ~100 cm; a helper holds a matte board and walks it slowly toward FRONT, then BACK, LEFT, RIGHT ( or fly slowly at a wall ) | under 500 mm on a side, the `r` / `p` push and brake appear and the craft moves away without contact | no `r` / `p` change, or contact |
| F4 | Switch Developer Mode off while a push is active | the craft stops moving away from the obstacle and follows the sticks. **Judged by eye:** the log ends at the switch, because `plutoLoop ( )` stops | the craft keeps drifting away with the sticks centred |
| F5 | Land, 20 s disarmed | touchdown detected; disarm | landing never detected ( a `src` / `alt` problem near the ground ) |

**Out-of-range handling ( task 14, both APIs ).** A side laser whose sample is out of range ( nothing within reach,
a dark or glossy target, a stale result ) keeps its last distance for up to 2 OA ticks, then clears on the 3rd
tick in a row. A removed obstacle is released within ~300 ms, and the push / brake cycle stops. Check in B3: remove the
card with **nothing** in range behind it. Within ~0.3 s the push stops, and the cycle ends after the current brake. The
first valid sample after a clear ( and at start-up ) is used as-is, so a wall at 2 m reads 2000, not 500.

**Watch.** R1: each side laser updates only about every 50-55 ms. If F3 shows a late push, measure the gap between
consecutive changes of that side's range in the log. Glossy or dark walls shorten the VL53L1X range: use a matte
surface first.

## 3. Hardware runs ( task 10 )

### log-1 ( 19 Oct 2026, 09:58, bench, disarmed, 33.5 s ): wrong build, no VL53L1X code ran

[log-1.txt](log-1.txt): 334 records, one `START`.

| Field | Every record | Meaning |
|---|---|---|
| `START altInit` | 0 | `X1Ranging.initAlt` refused |
| `S` | 00000 | every laser "not enabled" |
| `D L R F B` | −1 | not enabled |
| `fu` / `a` / `r` / `p` | 0 | no fusion, disarmed, no avoidance |
| `alt` | −2 … +2 cm | baro noise |

- **Cause:** the image ( `MechAsh!_PRIMUS_X2_v1_3.14.0.hex`, PlutoIDE, 09:56 ) was built **without** `#define LASER_TOF_L1x`, so
  X1Ranging compiled to its stubs ( −1 / `LASER_NOT_ENABLED` / false ). It is 4.8 KB larger than the 09:52 L1X image,
  which fits the stock build carrying the VL53L0X library. Nothing about the sensors can be read from this log.
- **The log line passes the budget check:** `n` is 1…334 with no gap over Wi-Fi ( no lost record ), and the tick is a
  steady ~101.6 ms.
- **Observation:** `src:1` while `fu:0`. The source flag reads "laser" in a baro-only build, so read `src` only when `fu:1`.
- **Next:** `LASER_TOF_L1x` is now uncommented in the X2_v1 `target.h` ( revert before commit ). Rebuild in PlutoIDE,
  flash, and repeat B1 into `log-2.txt`.

### log-2 ( 19 Oct 2026, 10:18, the old `LASER_TOF_L1x` build, 96.5 s ): hardware OK, OA OK, one altitude jump

[log-2.txt](log-2.txt): 938 records, analysed by the log-analyst agent; the OA window and the return window were
checked by hand.

| Phase | Records | Time |
|---|---|---|
| Disarmed bench | 0-34 | 3.5 s |
| Armed flight | 35-881 | 87.2 s |
| Disarmed | 882-937 | 5.6 s |

- One `START altInit:1` and no Dev Mode restarts.
- One `n` gap of 7 ( t 124.3 → 125.0 s, armed ): about 6 lost records, no restart.

| Laser | OK | Out of range | Boot / I2C errors |
|---|---|---|---|
| D | 91.8 % | 8.2 % | none |
| L | 61.3 % | 38.7 % | none |
| R | 48.4 % | 51.6 % | none |
| F | 46.8 % | 53.2 % | none |
| B | 78.8 % | 21.2 % | none |

Out-of-range sides are normal facing open space. Every laser gives a new value almost every tick.

- **B1 passes:** `START altInit:1`, `fu:1`, `S` only 1 / 4.
- **OA ( BACK, bench, records 0-34 ) passes B3 / B4:** B read 462-477 mm ( < 500 ), and `p` cycled 5 ticks +25 → 5 ticks −11 →
  1 tick 0 repeatedly while the obstacle stayed. That is the push direction and brake value specified for BACK. ( The
  analyst first called this a false FRONT trigger; F was ~2 m, and the trigger was BACK. )
- **OA in flight:** R 118-206 mm gave `r` ≈ −25, the correct sign. The other sides were not cleanly separable from stick input.
- **F1:** `alt` follows D while `src:1` ( e.g. D 828 mm ↔ alt 81 cm ).
- **F2, partly fails:**

  | t | Down range | `src` | `alt` |
  |---|---|---|---|
  | 52.4 s | object step, 905 → 288 mm | 1 → 2 | smooth |
  | 56.5 s | ≈ 160 cm | 2 → 0 | smooth |
  | 73.4 s | ≈ 125 cm | 0 → 1 | **29 → 125 cm in one tick** |

  During the baro leg `alt` was 25-38 cm while D read 121-131 cm: the baro path was ~95 cm low. → **task 18**.
- **Touchdown:** `alt` reached −50 cm around the disarm ( ground-effect baro ). Note for task 18.
- F4 / F5 cannot be judged from this log.

## 4. Build gate ( task 9, 19 Oct 2026 )

PRIMUS_X2_v1, FW 3.15.0, no laser define: one image runs both laser families. `driver.sh --gate PRIMUS_X2_v1`.

| Build | text | data | bss | Flash | RAM | Gate |
|---|---:|---:|---:|---|---|---|
| Task 1 baseline ( HEAD, template ) | 108276 | 1280 | 16760 | 106.9 KB ( 41 % ) | 17.6 KB ( 44 % ) | — |
| **Template `PlutoPilot.cpp`** | 122652 | 1316 | 19728 | 121.0 KB ( 47 % ) | 20.5 KB ( 51 % ) | no new warnings in `src/` |
| Test `PlutoPilot.cpp` | 124196 | 1316 | 19860 | 122.5 KB ( 47 % ) | 20.6 KB ( 51 % ) | no new warnings in `src/` |

Against the baseline the template build is **+14.4 KB flash** and **+3.0 KB RAM**. The flash pays for the VL53L1X ST
library, `tof_manager_l1x`, X1Ranging and the second fusion copy. The RAM is the shared device area ( +3.5 KB ) less the
removed `XVision` ( −964 B ), plus the L1X fusion state. Both fit: 121 / 256 KB and 20.5 / 40 KB, with 19.5 KB left for
the stack. `.init_array` holds only `accSmooth`, and there are no `_ZGV` guards.

Warnings: 2601 in total ( 844 in `src/`, 1757 in `lib/` ), none new. The "fixed" list ( `utils.h`, `XRanging.h`,
`XRanging.cpp` OA casts, `battery.cpp` … ) is for the baseline refresh at commit, from this stock build only.
PRIMUS_V5 and PRIMUSX2 are built at commit ( `pluto-commit` ).

### log-3 ( 19 Oct 2026, 11:29, no-flag build FW 3.15.0, test `PlutoPilot.cpp`, 196 s ): passes except landing

[log-3.txt](log-3.txt): 1803 records, analysed by the log-analyst agent; the landing window was checked by hand. The
user put a hand under the drone on purpose ( surface-step test ).

| Phase | Time |
|---|---|
| Disarmed | 0-3.7 s |
| Armed | 3.8-138 s |
| Disarmed | 138-196 s |

- One `START altInit:1` and no Dev Mode restarts.
- Two short record losses ( 2-3 records ).
- One 11.2 s Wi-Fi gap while disarmed; `n` advances by the same count, so the loop ran.

| Laser | OK | Out of range | Boot / I2C errors |
|---|---|---|---|
| D | 91.2 % | 8.8 % | none |
| L | 55.1 % | 44.9 % | none |
| R | 54.7 % | 45.3 % | none |
| F | 49.0 % | 51.0 % | none |
| B | 61.0 % | 39.0 % | none |

**Checks**

| Check | Result |
|---|---|
| B1 | **Pass:** `altInit:1`, `fu:1`, `S` only 1 / 4. |
| B3 / B4 | **Pass:** in clean episodes LEFT +25 / −11, RIGHT −25 / +11, FRONT −25 / +11, BACK +25 / −11, released ~0.2-0.3 s after the side cleared ( the 3-tick release ). No push without a side under 500 mm. Idle `r` / `p` carry the stick offset ( 0.3 × stick ), as designed. |
| F1 | **Pass:** `src:1`, `alt` follows `D`, height held. |
| F2 | **Pass by the user's eye.** The 160 / 140 cm handovers were flown several times; max `alt` 140 cm. One-tick `alt` changes happen only at a re-base or a return: 10 → 94 at 26.1 s, 107 → 17 at 72 s, and others. These are **frame shifts**: `altShiftFrame<T>` moves the estimate, `AltHold`, `altTarget`, `altGoal` and the filter history by the same amount, so the controller error is unchanged. **The user saw no movement** at those moments. |
| Hand steps | **As designed:** `src:2` with `alt` flat while held; after 2.5 s a re-base onto the hand ( `alt` ≈ height above the hand ); on removal a step back, then the baro / laser return, which shifts the frame back. |
| F3 | **Pass:** pushes away on each side. |
| F4 | Not judgeable from the log. |
| F5 | **Not shown:** the flight ended in a crash, see below. |

**Landing / crash ( 176-188 s ).**

1. The hand under the drone at 62-200 mm gave two re-bases onto the hand ( 178.9 s, 183.4 s ).
2. The hand was removed and `D` read 1.65 m, so the baro took over ( `src:0` ).
3. Then a real drop: `D` 1233 → 852 mm in 0.6 s, with `r` / `p` −80…−130 ( large stick or OA input ).
4. Disarm at 186.25 s.
5. After landing, `alt` read −196 cm on the baro ( hand frame plus ground effect ) while `D` read 22 mm; the return
   after disarm shifted it +183.

The log has no throttle or setpoint field, so it **cannot show what started the drop**. Nothing points at the new code.
User: "accidently crashed".

**Tooling:** `tools/flightlog.py` mis-parses PlutoMonitor lines that carry `HH:MM` timestamps: the inline-field regex
matches the clock, and `t` falls back to index × 0.1. The analyst used a one-off parser. Logged in `.claude/TOOLING_BACKLOG.md`.

**Next: log-4**, a short confirmation flight without the hand.

1. Hover at 80-100 cm.
2. Climb past 160 cm and back.
3. Switch Developer Mode off while a side push is active ( F4 ).
4. Land with the throttle stick in ALT_HOLD ( F5 ), then wait 20 s disarmed.

### log-4 ( 19 Oct 2026, 12:16, the same image as log-3, after the log-3 crash ): every laser failed start-up

[log-4.txt](log-4.txt): 752 records, 76 s, armed for 67 s.

| Field | Every record |
|---|---|
| `S` | 22222: all five `LASER_BOOT_FAILED` |
| `D L R F B` | −100 |
| `src` | 0 |
| `START altInit` | 1 ( the request was accepted ) |

- **The fallback worked as designed.** An accepted `initAlt` whose laser does not start flies as a laser dropout: the
  drone flew 67 s on the baro ( `alt` −53…229 cm ) and landed. No OA ( no side laser ).
- **The cause cannot be read from this log**: `getStatus` does not say which start-up step failed. The image is the one
  that started all five in log-3, so a change from the crash is the lead:
  - a loose or damaged connector, cable or laser power on the laser board;
  - **one broken XSHUT wire.** That laser stays awake at 0x29, so the 0x29 probe finds the bus busy before each release
    and refuses **every** laser, DOWN included ( `TOF_L1X_STEP_BUS_BUSY` ). One fault then looks like five.
- **Diagnostic** ( test `PlutoPilot.cpp` only ): the `START` line now also prints each laser's `failedStep` and
  `lastError`, as `START altInit:1 fs:D,L,R,F,B le:D,L,R,F,B`. Step codes:

  | Code | Step |
  |---|---|
  | 0 | none |
  | 1 | bus busy at 0x29 |
  | 2 | boot wait ( no answer ) |
  | 3 | model id |
  | 4 | set address |
  | 5-10 | init / config |
  | 11 | start ranging |

  Gate clean ( 124100 / 1316 / 19860 ). Next: **log-5**, bench only after a full power-off: check the connectors, then
  power on and turn Developer Mode on.

### log-5 ( 19 Oct 2026, 12:26, the no-flag build with the `fs` / `le` START diagnostic, after a power-off ): confirmation flight passes

[log-5.txt](log-5.txt): 1554 records, 159 s; analysed by the log-analyst agent.

| Phase | Time |
|---|---|
| Disarmed, including a hand-under bench check | 42.5 s |
| Armed | 111.1 s |
| Disarmed | 5.2 s |

`n` has no gap, so no record was lost and the loop never stalled; one `START` and no restarts.

- **All lasers started:** `START altInit:1 fs:0,0,0,0,0 le:0,0,0,0,0`, and there is no status 2 / 3 in 159 s. log-4's
  failure was the crash hardware state, cleared by a full power-off.

| Laser | OK | Out of range |
|---|---|---|
| D | 81.0 % | 19.0 % |
| L | 83.4 % | 16.6 % |
| R | 52.4 % | 47.6 % |
| F | 50.5 % | 49.5 % |
| B | 44.5 % | 55.5 % |

**Checks**

| Check | Result |
|---|---|
| F1 | **Pass:** laser hover, `src:1`. |
| F2 | **Pass, flown twice.** To the baro at D 1583 / 1605 mm ( `alt` 157 → 161, 162 → 172 ); back to the laser at D 1397 / 1381 mm ( 142 → 136, 147 → 134, a by-design frame shift ). No one-tick change outside a source change; max 227 cm. |
| F3 | **Pass:** FRONT 532 → 74 mm, push `r` −25 for ~0.4 s, brake +11…+14 for ~0.4 s, released. |
| F4 | **Not flown** ( no second START ); accepted on code review. |
| F5 | **Pass:** descent ~10.5 cm/s through the OA episode, one correction, then ~50 cm/s final, disarm at `alt` 3 cm on the laser ( `D` below the 15 mm minimum at touchdown ). `|r|`, `|p|` ≤ 28, no crash signature. |

**Task 10 verdict:** pass ( log-3 + log-5 ), F4 by code.
