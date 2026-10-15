# ToF Single Pipeline - Tasks

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

## Resume

Read this block first in a new session; read only the task it points to.

| | |
|---|---|
| **Current task** | **Closed - pending commit** ( 15 Oct 2026, FW 3.12.0, API 1.5.0 ). All tasks done. |
| **Next step** | The user commits ( `git commit -F .git/MAGISV2_COMMIT_MSG.txt` ); the next `pluto-commit` / `pluto-grill` run records the hash here |
| **Open questions** | none for this topic. Follow-ups: an OA-behaviour topic ( roll / pitch override adds motion ); the M25P16 flash not detected on this board ( `Flash:0`, pre-existing ) |
| **Blocked on** | nothing |
| **Last updated** | 2026-10-15 |

## Summary

| | |
|---|---|
| **Mode** | fix |
| **Goal** | Replace the two VL53L0X bring-ups with one ToF pipeline behind `XRanging`: `XRanging.initAlt ( gpio )` / `initAlt ( )` instead of `#define LASER_TOF` + `LASER_ALT`, per-laser state, one bring-up, paced cached side-laser reads |
| **Done when** | Bench: down + 4 side lasers all read; an I2C scan shows DOWN at 0x30 and the side lasers at 0x2A-0x2D; `getStatus ( )` OK for all five; a hand in front of FRONT does not move the laser height ( T4 ). Flight: an ALT_HOLD hover with `initAlt ( GPIO_5 )` + OA on four sides flies like today's `LASER_ALT` build and OA reacts on a wall approach; a baro-only hover ( no XRanging ) matches today's baseline log |
| **Target** | PRIMUS_X2_v1 ( all three targets plus an L1x + `LASER_ALT` check build at the gate and at commit ) |
| **Analysis** | Causal chain confirmed in code by [laser-tof-init-conflict](../laser-tof-init-conflict/INVESTIGATION.md#findings) ( H1 address collision + shared `Global_Status` latch ); fix = option B as chosen there ( D1-D9 ), with the additions in [INVESTIGATION.md](INVESTIGATION.md) |
| **Pipeline docs affected** | `subsystems/Main_Initialization.md`, `subsystems/Altitude_Hold_Estimator.md`, `subsystems/User_Space_API.md`, `subsystems/Hardware_Bus_Pipeline.md`, `PIN_MAP.md`, `dev-guide/HARDWARE_RESOURCES.md`, `dev-guide/FLIGHT_INVARIANTS.md`, CLAUDE.md, new `docs/API/XRANGING_API_WIKI.md` |
| **Order** | 1 → 2 → 3 → 4 → 5 → 6 → 7 → 8 → 9 → 10 → 14 → 11 → 12 → 13 ( 14 can run while task 10 waits for flights ) |

## Index

| # | Title | Status | Depends on | Skills / agent |
|---|---|---|---|---|
| 1 | Baseline: rollback hex and baro-only hover log | done 2026-10-14 | - | pluto-build, pluto-flighttest |
| 2 | ToF manager core ( per-laser state, per-device helpers ) | done 2026-10-14 | - | pluto-rules, pluto-driver, cpp-pro agent ( optional ) |
| 3 | XRanging API: `DOWN`, `initAlt`, `getStatus`, pin checks | done 2026-10-14 | 2 | pluto-rules |
| 4 | Boot-latched altitude-laser flag replaces the `LASER_*` gates | done 2026-10-14 | 2, 3 | pluto-rules |
| 5 | Single bring-up at main.cpp:600 and the DOWN poll | done 2026-10-14 | 2, 3, 4 | pluto-rules, pluto-driver |
| 6 | Bench check, phase 1 ( failure fixed, H2 closed ) | done 2026-10-14 | 5 | pluto-flighttest |
| 7 | Paced round-robin side-laser poll with cached getters | done 2026-10-14 | 6 | pluto-rules, cpp-pro agent ( optional ) |
| 8 | Bench check, phase 2 ( timing and cached reads ) | done 2026-10-14 | 7 | pluto-flighttest |
| 9 | Build gate | done 2026-10-14 | 8 | pluto-build |
| 10 | Hardware validation ( hovers ) | done 2026-10-15 | 9 | pluto-flighttest, pluto-log-analyst agent |
| 11 | Architecture & pipeline docs ( PIPELINE_UPDATE.md, API wiki ) | done 2026-10-15 | 10 | graphify, spec-to-code-compliance |
| 12 | Topic review | done 2026-10-15 | 11 | pluto-reviewer agent |
| 13 | Graph refresh & commit | done 2026-10-15 | 12 | graphify, pluto-commit |
| 14 | SPI2 off whenever XRanging is used ( GPIO_5-8 are its pins ) | done 2026-10-15 | 9 | pluto-rules, pluto-driver |

Status values: `todo`, `in-progress`, `done YYYY-MM-DD`, `blocked (<why>)`, `dropped (<why>)`.
Serial numbers are never reused or renumbered.

## Tasks

### 1. Baseline: rollback hex and baro-only hover log

**Description.** Before any code changes, capture what we compare against and what we fly if the fix misbehaves.
Build today's HEAD ( `c495d6a` ) for `PRIMUS_X2_v1` with `LASER_TOF` + `LASER_ALT` uncommented in `target.h`, copy
the hex to `rollback/` in this topic, then restore `target.h` exactly. Record that build's flash / RAM and the
stock build's ( 101.4 KB / 16.8 KB per the scout ). Then fly one ALT_HOLD hover on the stock build ( no laser
defines, no XRanging in user code ) and save the log as the baro-only baseline for task 10.

- **Depends on:** -
- **Skills / agent:** pluto-build ( working target ), pluto-flighttest ( hover procedure and log fields )
- **Files:** `src/main/target/PRIMUS_X2_v1/target.h` ( temporary, restored ), `rollback/`, `logs/`
- **Safety impact:** none ( no firmware change; a normal hover on the stock build )
- **Done when:** `rollback/` holds the `LASER_ALT` hex with its commit and size noted in TESTING.md; `target.h` shows no diff; a baro-only baseline hover log is saved in `logs/`
- **Rollback:** n/a
- **Status:** done 2026-10-14
- **Result:** Hexes saved: rollback `LASER_ALT` 104.2 KB / 17.5 KB, baseline logging build 102.1 KB / 16.8 KB, gate clean, `target.h` restored. log-1 ( 54 s armed, short of 3 min ): hold error −4.3 ± 5.7 cm, `Est` sd 5 cm, closure +3.3 cm, climb ~10 s ( TESTING.md ).

### 2. ToF manager core ( per-laser state, per-device helpers )

**Description.** Add `drivers/tof_manager.cpp/.h` ( in `DRONA_DRIVERS`, Makefile:276 ). It holds a 6-entry table
indexed by `laser_e` ( LEFT, RIGHT, FRONT, BACK, EXTERNAL, DOWN ): XSHUT pin or none, I2C address, a pointer to the
entry's `VL53L0X_Dev_t` ( the existing 5 `LaserSensor` devices and the global `MyDevice`, so there is no new large
allocation ), status ( OK / NOT_ENABLED / BOOT_FAILED / I2C_ERROR ), cached range, out-of-range flag and new-sample
flag. Rework `ranging_vl53l0x.cpp` into per-device helpers that take the device and report status per call: init
with a configuration ( DOWN = today's single mode, 33 ms, VCSEL 18 / 14, limit checks, R6; side = today's continuous,
20 ms ), set address with the status returned ( F2 ), single-shot start / read, and a data-ready read. Nothing calls
the module yet; the old paths stay in place until task 5. No `Global_Status`-style shared latch exists in the new code.

- **Depends on:** -
- **Skills / agent:** pluto-rules, pluto-driver ( I2C1 sharing, init rules ); `cpp-pro` agent if delegated
- **Files:** new `src/main/drivers/tof_manager.cpp/.h`, [ranging_vl53l0x.cpp](../../../../src/main/drivers/ranging_vl53l0x.cpp), [ranging_vl53l0x.h](../../../../src/main/drivers/ranging_vl53l0x.h), `Makefile`
- **Safety impact:** none yet ( not called ). DOWN's configuration must equal today's `ranging_init ( )` exactly ( R6 )
- **Done when:** `PRIMUS_X2_v1` builds with no new warnings ( `--gate` ); DOWN's helper config is shown line by line to equal today's `ranging_init ( )`
- **Rollback:** revert the task's diff ( the module has no callers )
- **Status:** done 2026-10-14
- **Result:** `tof_manager.cpp/.h` ( 6-entry table, per-entry device / status / results, bring-up, DOWN poll, blocking side read, DOWN accessors ) + per-device helpers in `ranging_vl53l0x.cpp`; DOWN config equal to `ranging_init ( )` step by step ( CHANGES.md ); side config adds the sample-ready interrupt ( review fix ). Not linked yet: 102.1 KB / 16.8 KB, gate clean; module 840 B text, 2.2 KB bss once linked ( net ~+100 B after task 5 ).

### 3. XRanging API: `DOWN`, `initAlt`, `getStatus`, pin checks

**Description.** Extend the public API without changing an existing signature ( R7 ):
- Append `DOWN = 5` to `laser_e` ( EXTERNAL keeps 4 ) and grow `isXLaserInit`, `triggerThreshold`, OA `dFilt` and the
  pin table to 6. Add upper-bound checks to every `laser_e` index ( XRanging.cpp:145-168, :253 ).
- Add `initAlt ( gpio )` / `initAlt ( )`. It only records the request, and only while `plutoInit ( )` runs ( D4 ).
- Refuse the `initAlt` request when the pin is invalid, is an enabled laser's XSHUT, or is in use as ADC / PWM
  ( GPIO_5 = PB15 is also ADC_2 and PWM_4 ). Also refuse it when `LASER_TOF_L1x` is compiled in ( D8 ).
- An XSHUT claim beats an LED: with `initAlt ( GPIO_5 )`, FRONT has no LED, and every LED write skips a pin claimed as
  an XSHUT. This covers bring-up, the blink animation and the OA writes at XRanging.cpp:93-123 and :303-327.
- Add `getStatus ( id )`, returning OK / NOT_ENABLED / BOOT_FAILED / I2C_ERROR / OUT_OF_RANGE from the manager.

Delete the commented-out duplicate at XRanging.cpp:392-733.

- **Depends on:** 2
- **Skills / agent:** pluto-rules ( API header stability, `-Wconversion` )
- **Files:** [XRanging.h](../../../../src/main/API/XRanging.h), [XRanging.cpp](../../../../src/main/API-Src/XRanging.cpp), [API-Utils.h](../../../../src/main/API/API-Utils.h), [API-Utils.cpp](../../../../src/main/API-Src/API-Utils.cpp)
- **Safety impact:** the GPIO_5 LED/XSHUT rule is flight-critical: an LED write LOW on the down-laser's XSHUT resets it and wipes its address in flight
- **Done when:** builds clean; a grep shows no `ledGpio` write that can reach a claimed XSHUT pin; out-of-range ids return safely
- **Rollback:** revert the task's diff
- **Status:** done 2026-10-14
- **Result:** `DOWN`, `laser_status_e`, `initAlt ( gpio )` / `initAlt ( )`, `getStatus ( )`, bounds on every entry point, the latch ( `xRangingAltLatch`, re-checks ADC / PWM ), XSHUT-beats-LED on every LED path, and the claimed pin refused by the public GPIO / ADC / PWM init and write after the latch ( review fix ); the 350-line commented duplicate removed. 102.2 KB / 16.9 KB, gate clean.

### 4. Boot-latched altitude-laser flag replaces the `LASER_*` gates

**Description.** Add one flag, latched once after `plutoInit ( )` and never changed afterwards ( D4 ). It is true when
`initAlt` was accepted, or when the build has `LASER_TOF_L1x` && `LASER_ALT` ( L1x builds are unchanged ).

Convert the gates from SCOUT.md §1:
- **Runtime `if ( flag )`:** altitudehold.cpp:918 and :1339; imu.cpp:261; command.cpp:116; mw.cpp:814.
- **Always compiled:** altitudehold.cpp:969-1336 and :1360; the `altHoldSource` declaration; imu.cpp:51.
- **Accessor macros:** the L0X block at :111-235 becomes the `#else` of L1x, and its macros read the DOWN entry.
  `tofOutOfRange ( )` includes DOWN's status ( R3, D6 ).
- **Time constant:** `_time_constant_z` is set at latch time, followed by `updateGains ( )`.
- **Removed:** the :108 and :229 `#error` rules, the :1048 / :1059 gates, and the laser comments in all three `target.h`.
- **Kept:** L1x-only gates and `LASER_ALT` ( it now means something only together with L1x ).
- **opticflow.cpp:355:** reads the DOWN entry.

With the flag false, each converted gate must take the branch a build without the defines takes ( D5 ).

- **Depends on:** 2, 3
- **Skills / agent:** pluto-rules ( baro datum, altitude-hold setpoint, landing invariants )
- **Files:** [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp), [altitudehold.h](../../../../src/main/flight/altitudehold.h), [imu.cpp](../../../../src/main/flight/imu.cpp), `command.cpp`, [mw.cpp](../../../../src/main/mw.cpp), `opticflow.cpp`, `target/*/target.h`
- **Safety impact:** ALT_HOLD, take-off height, landing and the laser / baro handover on every build. With the flag off the behaviour must equal today's baro build; with it on, today's `LASER_ALT` build
- **Done when:** builds clean; a table in CHANGES.md lists every converted gate with its flag-off branch and the matching pre-change branch
- **Rollback:** revert the task's diff; fly the task-1 hex
- **Status:** done 2026-10-14
- **Result:** `altLaserFusion ( )` / `altLaserFusionLatch ( )` replace the VL53L0X build switches; fusion always compiled, L0X accessors on the DOWN entry; every flag-off gate takes the baro branch ( CHANGES.md table, reviewer-confirmed ); L1x keeps its switch ( L1x alone and L1x + `LASER_ALT` both build ); `#error` on `LASER_TOF` / bare `LASER_ALT`. 105.1 KB / 19.3 KB, gate clean. Latch not called until task 5.

### 5. Single bring-up at main.cpp:600 and the DOWN poll

**Description.** Delete `ranging_init ( )` at main.cpp:516-520 so nothing touches 0x29 before :600 ( R2 ). At :600, one
manager bring-up runs in this order:
1. Drive every enabled XSHUT low ( side lasers, and DOWN's when it has a pin ). Wait 10 ms.
2. DOWN with a pin: release it alone, init it, move it to 0x30. DOWN without a pin: probe 0x30 ( warm reset ), and
   if nothing answers, probe 0x29 and move it to 0x30.
3. Release the side lasers one at a time in table order ( 0x2A + n, as today ), checking every status.
4. A failing laser is marked BOOT_FAILED and never retried ( no re-init in flight ). The others carry on.

`UPDATE_LASER_TOF_TASK` then polls DOWN through the manager when the flag is set, keeping today's 33 ms
start + read. `LaserSensor` reads go to their own entries, never to shared globals ( H2 ). Side reads stay
synchronous until task 7.

- **Depends on:** 2, 3, 4
- **Skills / agent:** pluto-rules, pluto-driver ( init order, XSHUT ownership, PB13 / PB15 taken from SPI2 )
- **Files:** [main.cpp](../../../../src/main/main.cpp), `tof_manager.cpp`, [XRanging.cpp](../../../../src/main/API-Src/XRanging.cpp), [mw.cpp](../../../../src/main/mw.cpp)
- **Safety impact:** the altitude laser's start-up. A DOWN failure must give a fusion dropout ( baro hold, D6 ), never a stuck reading
- **Done when:** builds clean; the old `ranging_init` / `getRange` / `Global_Status` paths have no callers left; the bring-up order matches INVESTIGATION.md step by step
- **Rollback:** revert tasks 5 ( and 4 ) diffs; fly the task-1 hex
- **Status:** done 2026-10-14
- **Result:** One bring-up in `xRangingInit ( )` ( latch → reset → DOWN alone via GPIO_5 / probe 0x30 then 0x29 → side lasers 0x2A + n → LEDs ), every release refused if 0x29 already answers ( review fix, `VL53L0X_STEP_BUS_BUSY` ); old `ranging_init` / `LaserSensor` / shared globals deleted; side reads per entry ( H2 closed ). 106.3 KB / 17.5 KB, gate clean.

### 6. Bench check, phase 1 ( failure fixed, H2 closed )

**Description.** Flash the task-5 build with `initAlt ( GPIO_5 )` and `initObjectAvoidance ( …, LEFT, RIGHT, FRONT, BACK )`
in `plutoInit ( )`. Print once after boot: an I2C scan of 0x29-0x30, the I2C error count, and `getStatus ( )` for all
six ids. Then print the five ranges at a low rate ( keep `Monitor_Print` under ~130 bytes per tick ).

Checks:
- Expected: 0x29 empty, 0x2A-0x2D and 0x30 present, five OK.
- **T4:** a hand in front of FRONT must not move DOWN's range or the estimator's laser height.
- **Warm reset:** reflash or soft-reset without a power cycle; all five must come back.
- **Refusals:** `initAlt ( GPIO_16 )` and `initAlt ( GPIO_5 )` with ADC_2 enabled must be refused.

Record everything in TESTING.md.

- **Depends on:** 5
- **Skills / agent:** pluto-flighttest ( bench procedure, log fields )
- **Files:** `PlutoPilot.cpp` ( test code, not committed ), TESTING.md, `logs/`
- **Safety impact:** bench only, props off
- **Done when:** all five lasers read; the scan matches; T4, warm reset and both refusals pass. Optional: the no-wire warm reset ( XSHUT wire unplugged, `initAlt ( )` ), and a fitted-but-disabled side laser giving `BUS_BUSY` instead of a collision
- **Rollback:** n/a ( bench )
- **Status:** done 2026-10-14
- **Result:** log-2 ( cold start, 110 s ): refusals `0 0 1 0`, `Scan:158` every cycle ( DOWN alone at 0x30, H1 gone ), statuses all OK, GPIO_5 guard held over 1054 LOW writes, **T4 pass** ( hand at FRONT: `D` 905 ± 11.5 mm, `TH` 89.9 ± 0.4 cm, no change; H2 closed ). Warm reset not run, moved to task 8.

### 7. Paced round-robin side-laser poll with cached getters

**Description.** From the same periodic call as DOWN's poll, serve **one** enabled side laser per call in round-robin.
That laser gets a data-ready check, and only when data is ready a result read and an interrupt clear ( no restart;
continuous mode; R5 / F5 ). `XRanging.getRange ( )`, `isTriggered ( )` and `applyObjectAvoidance ( )` ( mw.cpp:532 )
return the cached range with no I2C. A side laser's I2C error marks it I2C_ERROR without touching the others ( R4 ).
Add a temporary `micros ( )` measurement of the periodic call and of `applyObjectAvoidance ( )`, worst case and mean.

- **Depends on:** 6
- **Skills / agent:** pluto-rules ( bounded loop work, no blocking ); `cpp-pro` agent if delegated
- **Files:** `tof_manager.cpp`, [XRanging.cpp](../../../../src/main/API-Src/XRanging.cpp), [mw.cpp](../../../../src/main/mw.cpp)
- **Safety impact:** OA stick response now uses ranges up to ~4 poll periods old; DOWN's timing must not change ( R6 )
- **Done when:** builds clean; no XRanging getter calls I2C ( grep ); the periodic call is shown to touch at most one side laser
- **Rollback:** revert the task's diff ( back to the phase-1 synchronous reads )
- **Status:** done 2026-10-14
- **Result:** `tofManagerPollSides ( )` ( 10 ms pacing, round-robin, one data-ready read per call ), continuous ranging started after the last bring-up, all getters cached ( no I2C outside `xRangingInit` ); review fixes: `RANGE_ERROR` interrupt flag no longer fatal, OA ignores a dead laser instead of pushing on its last distance. Temporary µs timing in mw.cpp for task 8. 107.4 KB / 17.6 KB, gate clean.

### 8. Bench check, phase 2 ( timing and cached reads )

**Description.** On the task-7 build, record the `micros ( )` numbers: the periodic call's worst case, OA's cost
( it should drop from ~4-6 ms to well under 1 ms ), and `cycleTime` against phase 1. Record the update rate of each
side range. Repeat the T4 and status checks from task 6. Remove the temporary timing code once the numbers are in
TESTING.md.

**Warm reset ( moved from task 6 ).** Flash the task-7 build **without removing the battery** ( reflash over Wi-Fi / USB, or reboot from the app ). All five lasers must come back: `Scan:158`, every status OK. On this board every laser has an XSHUT wire, so phase 1 of the bring-up resets each chip to 0x29; this run confirms it on hardware.

- **Depends on:** 7
- **Skills / agent:** pluto-flighttest
- **Files:** TESTING.md, `logs/`
- **Safety impact:** bench only
- **Done when:** after a warm reset all five lasers come back ( `Scan:158` ); OA cost < 1 ms; DOWN still updates every ~33 ms; each side range updates at least every ~150 ms; T4 passes; timing code removed
- **Rollback:** n/a ( bench )
- **Status:** done 2026-10-14
- **Result:** log-3 ( warm boot, battery in ): warm reset pass ( 12 × `Scan:158`, all OK ); OA 29-32 µs ( was 4-6 ms ); laser task worst ~2.0-2.2 ms, mean 13 µs; DOWN 29-31 Hz ( unchanged ), sides 23-26 Hz; no start-up 8191. Temporary timing removed ( review: clean ), `PlutoPilot.cpp` back to the task-1 baseline log. 106.6 KB / 17.6 KB, gate clean.

### 9. Build gate

**Description.** Run `pluto-build --gate PRIMUS_X2_v1`: no new `src/` warnings, and flash / RAM within budget
( expected about +2-2.5 KB flash and +0.4 KB RAM over 101.4 KB / 16.8 KB ). Also build `PRIMUS_V5` and `PRIMUSX2`,
and temporary `PRIMUS_X2_v1` builds with `LASER_TOF_L1x` alone and with `LASER_TOF_L1x` + `LASER_ALT`, to show
both L1x paths still compile.
Restore `target.h` afterwards.

- **Depends on:** 8
- **Skills / agent:** pluto-build ( `--gate` )
- **Files:** -
- **Safety impact:** none
- **Done when:** the gate passes on all three targets and the L1x check build compiles; sizes are recorded in TESTING.md
- **Rollback:** n/a
- **Status:** done 2026-10-14
- **Result:** All three targets gate clean ( X2_v1 106.6 / 17.6 KB, V5 106.6 / 17.6, PRIMUSX2 106.4 / 17.7 ); L1x alone ( 114.8 / 18.4 ) and L1x + `LASER_ALT` ( 114.6 / 18.4 ) build; `#define LASER_TOF` fails on the intended `#error`. +2.4 KB flash, +0.1 KB RAM over today's `LASER_ALT` build.

### 10. Hardware validation ( hovers )

**Description.** Plan with `pluto-flighttest` and fly two hovers:
- **Hover A:** `initAlt ( GPIO_5 )` + OA on all four sides. ALT_HOLD hover, then a slow approach to a wall. Log
  DOWN's range and status, the altitude source and the side ranges.
- **Hover B:** baro only ( no XRanging in user code ). Compare it with the task-1 baseline log.

Pass criteria:
- Hover A holds height as well as today's `LASER_ALT` build ( same handover behaviour ), and OA reacts.
- Hover B matches the baseline within noise.

Use the `pluto-log-analyst` agent for large logs.

- **Depends on:** 9
- **Skills / agent:** pluto-flighttest, pluto-log-analyst agent
- **Files:** TESTING.md, `logs/`
- **Safety impact:** flight test: ALT_HOLD, OA near a wall; rollback hex ready
- **Done when:** both hovers pass with logs saved and the comparison written in TESTING.md
- **Rollback:** fly the task-1 hex
- **Status:** done 2026-10-15
- **Result:** Hover A pipeline checks pass ( H2 in flight, OA, laser health ); the A/B shows no regression against same-day today's firmware ( `Df/10` sd 5.53 / 5.56 cm ); OA adds craft motion ( out of scope, own topic ); hover B matches the baseline ( −3.4 ± 6.1 against −5.9 ± 4.7 cm, closure 0.0 / +3.3 cm ). TESTING.md tests 4-12 and the task-10 verdict.

### 11. Architecture & pipeline docs ( PIPELINE_UPDATE.md, API wiki )

**Description.** Write the corrected text into `PIPELINE_UPDATE.md`; never edit `fw-architecture-pipeline/` directly.
It covers drift rows D1-D12 from the check topic and the docs that move from "build-time define" to "`initAlt` at
boot". Redraw the Main_Initialization flowchart with the single laser bring-up; every edge must be a real call,
confirmed by `graphify path` or a source line.

Also update:
- `PIN_MAP.md`: GPIO_5 = DOWN XSHUT, and addresses 0x29-0x30.
- `dev-guide/HARDWARE_RESOURCES.md` and `FLIGHT_INVARIANTS.md`.
- The CLAUDE.md `LASER_ALT` invariant, rewritten for `XRanging.initAlt ( )`.

Write `docs/API/XRANGING_API_WIKI.md` in the BMS wiki layout, stating the new API version. The `API_Version` /
`FW_Version` bump happens at commit.

- **Depends on:** 10
- **Skills / agent:** graphify, spec-to-code-compliance ( docs against the code )
- **Files:** `PIPELINE_UPDATE.md`, `docs/API/XRANGING_API_WIKI.md`, `PIN_MAP.md`, `dev-guide/*.md`, CLAUDE.md
- **Safety impact:** none
- **Done when:** every D1-D12 row has new text; the wiki covers `initAlt`, `DOWN`, `getStatus`, cached reads and the GPIO_5 / FRONT-LED rule; the compliance check finds no contradiction
- **Rollback:** n/a
- **Status:** done 2026-10-15
- **Result:** `PIPELINE_UPDATE.md` ( Main_Initialization, Hardware_Bus_Pipeline, User_Space_API in full; Altitude_Hold_Estimator as replacement blocks; D1-D12 and the build-define → `initAlt` text; 107 diagram edges cited to source; the CLAUDE.md draft ), `docs/API/XRANGING_API_WIKI.md` ( API 1.5.0 ), PIN_MAP, dev-guide HARDWARE_RESOURCES / FLIGHT_INVARIANTS. Compliance check: ~145 statements, 3 minor contradictions fixed, 105 edges, 0 failed.

### 12. Topic review

**Description.** Run the `pluto-reviewer` agent on the whole topic diff. Ask it explicitly for:
- the flag-off equivalence, gate by gate against task 4's table;
- the LED / XSHUT rule;
- `laser_e` bounds;
- no I2C in getters;
- no re-init in flight.

Fix every BLOCKING finding and re-run the task-9 gate.

- **Depends on:** 11
- **Skills / agent:** pluto-reviewer agent
- **Files:** -
- **Safety impact:** none
- **Done when:** no BLOCKING findings remain; the gate passes after the fixes
- **Rollback:** n/a
- **Status:** done 2026-10-15
- **Result:** Fresh `pluto-reviewer` on the whole topic: no BLOCKING; the six required items clean ( flag-off equivalence re-walked gate by gate ). SHOULD-FIX fixed: `initAlt` refuses RGB-slot and UART2 pins, which those drivers can take after start-up; NITs fixed ( leftover includes, dead code, stale comments, static_asserts ); re-check closed. All three targets gate clean ( 107.0 / 107.0 / 106.9 KB ). Version bump left for task 13.

### 13. Graph refresh & commit

**Description.** After the review fixes, run `graphify update .` and `python tools/graph_labels.py`. Then run
`/pluto-commit`, which covers:
- the all-target build;
- the `FW_Version` and `API_Version` bump;
- applying `PIPELINE_UPDATE.md`, the wiki and `CHANGELOG.md`;
- staging and the drafted message.

Set this topic Closed here and in `active-development/README.md`.

- **Depends on:** 12
- **Skills / agent:** graphify, pluto-commit
- **Files:** -
- **Safety impact:** none
- **Done when:** staged with a drafted message; the topic is Closed with the commit hash after the user commits
- **Rollback:** n/a
- **Status:** done 2026-10-15
- **Result:** Graph refreshed ( 4114 nodes, 7779 edges, 389 communities, labelled ). `/pluto-commit`: all three targets clean ( X2_v1 106.3 / 17.6, PRIMUSX2 106.1 / 17.7, V5 106.2 / 17.6 KB ), FW 3.12.0 / API 1.5.0, PIPELINE_UPDATE.md applied, CHANGELOG, CLAUDE.md, stock `PlutoPilot.cpp` restored, scratch worktree removed; staged with a drafted message for the user.

### 14. SPI2 off whenever XRanging is used ( GPIO_5-8 are its pins )

**Description.** GPIO_5-8 are SPI2's four pins: PB15 MOSI, PB14 MISO, PB13 SCK, PB12 NSS / M25P16 CS. XRanging uses
them as XSHUT and LED pins: LEFT 6 / 7, FRONT's LED 5, BACK's LED 8, and the `initAlt ( GPIO_5 )` XSHUT. Today
`spiInit ( SPI2 )` ( main.cpp:313 ) and the user SPI API ( `Serial-IO-SPI.cpp` ) can drive those pins as SPI. An SPI
line resting low is an XSHUT held low, so a chip in reset ( log-7: today's firmware with the XSHUT wire on PB15 never
saw its laser ). User decision: **whenever XRanging is used at all** ( any accepted `initAlt` / `init` /
`initObjectAvoidance` ), the bring-up de-initialises SPI2 before it touches a pin: `SPI_Cmd ( DISABLE )`,
`SPI_I2S_DeInit`, clock off, so PB12-15 become plain GPIO. Also, **XRanging wins**: from then on the user SPI API
( `SPI_Init`, reads, writes on SPI2 ) does nothing, and the M25P16 flash is marked unavailable ( `BLACKBOX` is off in
target.h ). The same latch-time rule as task 3: the decision is fixed at the bring-up.

- **Depends on:** 9
- **Skills / agent:** pluto-rules, pluto-driver ( SPI / flash driver ownership, PIN_MAP )
- **Files:** [XRanging.cpp](../../../../src/main/API-Src/XRanging.cpp) ( bring-up ), [Serial-IO-SPI.cpp](../../../../src/main/API-Src/Serial-IO-SPI.cpp), [bus_spi.c](../../../../src/main/drivers/bus_spi.c) and the M25P16 / flashfs driver ( how to mark the flash absent ), [main.cpp](../../../../src/main/main.cpp), `PIN_MAP.md`
- **Safety impact:** prevents SPI2 from resetting a laser through its XSHUT ( the altitude laser in flight ). Flight with no XRanging calls is unchanged ( SPI2 and the flash as today )
- **Done when:** builds clean on all three targets ( gate ); a grep shows no SPI2 access path that runs after the XRanging bring-up turned SPI2 off; bench: with `initAlt ( GPIO_5 )` + the 4 side lasers, `Scan:158`, all statuses OK, and a user `SPI_Init ( )` / SPI read is refused ( returns without touching PB12-15 ); without XRanging, SPI2 and the flash still initialise
- **Rollback:** revert the task's diff ( SPI2 stays on, as before )
- **Status:** done 2026-10-15
- **Result:** `spiRelease ( SPI2 )` ( only pins SPI2 still owns become pulled-up inputs ), `m25p16_release ( )`, the user SPI API inert, triggered whenever XRanging is used; review blocking fixed ( user pins kept ). Hardware: `SPIoff:1 Flash:0` ( logs 11, 12 ), 26 status cycles with all five lasers and no error ( log-12 ). All three targets gate clean.

## Decisions log

Newest last. One line each: `YYYY-MM-DD [decision|assumption|out-of-scope|risk] text`.

- 2026-10-14 [decision] Planned with `/pluto-grill fix laser-tof-init-conflict`. This is a new topic, because the fix is large ( new module + public API ). The check topic is closed and points here.
- 2026-10-14 [decision] Direction as chosen in the check topic: option B, D1-D9 ( `initAlt`, one bring-up at :600, per-laser state, paced reads, `DOWN`, L1x stays a define ).
- 2026-10-14 [decision] Target PRIMUS_X2_v1. All three targets and an L1x + `LASER_ALT` check build at the gate and at commit.
- 2026-10-14 [decision] Both phases in this topic ( phase 1: state + bring-up + API; phase 2: paced poll + cached getters ), with a bench check between them.
- 2026-10-14 [decision] No T1 / T2 reproduction before the code: the chain is deterministic in the code. Hardware checks come after the fix.
- 2026-10-14 [decision] Done when: bench ( all five read, scan, T4 ) + hover with `initAlt` + OA + baro-only hover.
- 2026-10-14 [decision] The down-laser XSHUT is wired to **GPIO_5 ( PB15 )**. Only the XSHUT is on that pin now ( the FRONT LED is no longer connected ). This supersedes D3's "GPIO_5 refused".
- 2026-10-14 [decision] An XSHUT claim beats an LED: with `initAlt ( GPIO_5 )`, FRONT has no LED, and LED writes skip claimed XSHUT pins ( OA writes FRONT's LED LOW every 100 ms, XRanging.cpp:319, which would reset the down-laser ).
- 2026-10-14 [decision] `initAlt` refuses: invalid pins, an enabled laser's XSHUT, pins in use as ADC / PWM ( GPIO_5 = ADC_2 / PWM_4; GPIO_2 = ADC_1 ), and any build with `LASER_TOF_L1x` compiled in.
- 2026-10-14 [decision] A failed laser is marked failed and never retried; the others keep working. A failed DOWN gives a fusion dropout. Recovery is a power cycle.
- 2026-10-14 [decision] New `XRanging.getStatus ( id )` returns OK / NOT_ENABLED / BOOT_FAILED / I2C_ERROR / OUT_OF_RANGE.
- 2026-10-14 [decision] L1x builds unchanged: the latched flag is `LASER_TOF_L1x && LASER_ALT` at compile time. `LASER_ALT` survives only for L1x until the X1Ranging topic.
- 2026-10-14 [decision] Flag-off equivalence is shown by a reviewer gate-by-gate check plus a baro-only hover against a baseline log of today's build ( object files will differ ).
- 2026-10-14 [decision] Main hover: `initAlt ( GPIO_5 )` + OA on all four sides, ALT_HOLD hover then a wall approach.
- 2026-10-14 [decision] Rollback flight build: today's HEAD ( `c495d6a` ) with `LASER_TOF` + `LASER_ALT`, hex saved in task 1.
- 2026-10-14 [assumption] DOWN uses address 0x30. The side lasers keep today's table order and 0x2A + n ( EXTERNAL last ).
- 2026-10-14 [assumption] The no-wire `initAlt ( )` path ( probe 0x30 then 0x29 and move ) is checked by build and review only: the board has the XSHUT wire.
- 2026-10-14 [risk] PB15 ( GPIO_5 ) is SPI2_MOSI, and PB13 ( LEFT XSHUT ) is SPI2_SCK. SPI2 flash is unusable while either is driven. This is the known overlap and stays out of scope.
- 2026-10-14 [risk] The fusion code becomes always compiled: about +2-2.5 KB flash and +0.4 KB RAM ( scout estimate ). Headroom is ample.
- 2026-10-14 [risk] OA uses cached side ranges up to ~4 poll periods old instead of fresh blocking reads; the wall approach in task 10 checks the response.
- 2026-10-14 [out-of-scope] F1 ( the I2C driver's missing NACKF check ) gets its own topic. Also out of scope: the VL53L1X / X1Ranging API, fusion tuning, OA thresholds and stick response, and the PA15 WS2811 / FRONT XSHUT overlap.
- 2026-10-14 [decision] Task 1: the baseline log line is `t n Baro Est AltH Vz Thr Arm` plus a `START` restart marker ( ~95-110 B/tick ). Task 10 hover B reuses the same `PlutoPilot.cpp`.
- 2026-10-14 [risk] The real cost of `LASER_TOF` + `LASER_ALT` today is +2900 B flash and +728 B RAM ( measured ), above the scout's 2-2.5 KB / 0.4 KB.
- 2026-10-14 [assumption] The baseline hover log-1 lasted 54 s armed, not 3 min. Task 10 hover B flies the same ~50 s sequence to stay comparable; the trend is not compared, only the hold error, wobble, climb and closure.
- 2026-10-14 [risk] The `Thr` field logs the throttle stick ( `rcCommand`, 1499 in ALT_HOLD ), not the motor throttle; the hover throttle is not in the baseline.
- 2026-10-14 [decision] Task 2 review: side lasers get the new-sample-ready interrupt as the last init step, because the vendored `StaticInit` turns GPIO functionality off ( lib DD change ) and data-ready would never reset. DOWN's config is unchanged ( R6 ).
- 2026-10-14 [risk] Task 7: `VL53L0X_ERROR_INTERRUPT_NOT_CLEARED` from the interrupt clear is not a bus fault ( ST returns it after a third clear attempt ); the helper maps it to success and the poll must not latch I2C_ERROR on it.
- 2026-10-14 [decision] The VL53L0X GPIO1 pin is not wired on these boards ( user ). The sample-ready interrupt from task 2 does not need it: data-ready and the clear go through the interrupt status register over I2C; the pin is an unused open-drain output.
- 2026-10-14 [decision] Task 3: `initAlt` refuses any side laser's XSHUT pin, enabled or not, because the order of calls in `plutoInit ( )` is free. ADC / PWM conflicts are checked again once at the latch.
- 2026-10-14 [decision] Task 3 review: after the latch, the public `Peripheral_Init` / `Peripheral_Write` ( GPIO ) and `Peripheral_Init` ( ADC, PWM ) ignore the down-laser's XSHUT pin and its aliases. Task 5 must call `xRangingAltLatch ( )` first and then drive DOWN's XSHUT only through `xshutPinInit` / `xshutPinWrite`.
- 2026-10-14 [out-of-scope] A user `SPI_Init` taking PB12-15 is not guarded; it is documented in the XRanging wiki ( task 11 ).
- 2026-10-14 [decision] Task 4: `initAlt` accepted but DOWN failing to start keeps the fusion on with a dropout ( D6 ), not the baro build ( D5 is only for no / refused `initAlt` ). Task 5 latches `altLaserFusionLatch ( xRangingAltLatch ( ) )` without gating on DOWN's status.
- 2026-10-14 [assumption] Task 4: the unreachable no-`BARO` branch at altitudehold.cpp:950 is left; all targets define `BARO`.
- 2026-10-14 [decision] Task 5 review: before each release, 0x29 must be silent ( `vl53l0xProbe` ). If it is busy, the laser stays in reset and is marked BOOT_FAILED at `VL53L0X_STEP_BUS_BUSY`. This covers fitted-but-disabled side lasers ( PA15 pull-up, PB13 SPI2 SCK high ).
- 2026-10-14 [risk] Two cases only the documentation can cover ( task 11 wiki ): a no-wire DOWN cannot detect company at 0x29; a down-laser wired to GPIO_5 without `initAlt ( GPIO_5 )` is released together with FRONT by FRONT's LED writes. Rule: every fitted laser enabled, and a wired down-laser always declared with its pin.
- 2026-10-14 [out-of-scope] L1x builds: `ranging_init_L1` ( main.cpp:518 ) leaves the VL53L1X at 0x29 during side-laser bring-ups, unchanged from HEAD; for the X1Ranging topic.
- 2026-10-14 [decision] Task 6 closed on steps 1-4 ( all pass, T4 included ). The warm reset moves to task 8's reflash, done without removing the battery ( the user had cold-started ).
- 2026-10-14 [risk] log-2: during handling the estimator went to the baro ( `Src:0`, 31.5-38 s ) with DOWN valid at 40-80 mm. This is fusion logic, unchanged by this topic; task 10's hover checks the handover against the `LASER_ALT` build.
- 2026-10-14 [decision] Task 7 review: `VL53L0X_ERROR_RANGE_ERROR` from the interrupt status is a sensor flag, not a bus fault; OA treats a laser that is not OK / OUT_OF_RANGE as no obstacle and clears its filter; side lasers start continuous ranging after the last bring-up.
- 2026-10-14 [risk] log-3: the loop cycle was 4.3-4.6 ms in 18 of 118 snapshots ( looptime 3.5 ms ), probably the ~2 ms laser-task peak when DOWN and a side read share a call. Task 10's hovers judge it; the fix if needed is to skip the side read in a call where DOWN read.
- 2026-10-14 [decision] Task 10: hover A is judged against the documented `LASER_ALT` reference ( tof-althold-fusion test 3b: estimate − setpoint −0.9 ± 1.8 cm, 100 % within ±5 cm over 30 s ); pass = ≥ 95 % within ±5 cm over a 30 s window. An A/B with the rollback hex is flown only if hover A misses. Each hover ~60 s, matching log-1.
- 2026-10-14 [risk] Hover A ( log-4 ): pipeline checks pass ( H2 in flight, OA, laser health ), but altitude hold bobs ±10 cm with a ~8.5 s period ( best 30 s: 82 % within ±5 cm, against ≥ 95 % ); the raw laser confirms real motion, and the user felt more bobbing than usual.
- 2026-10-14 [decision] Task 10: A/B on one battery, R ( HEAD + `LASER_ALT`, old pipeline ) against A2 ( new pipeline, `initAlt ( GPIO_5 )` only, no OA ), with the same log line, to split regression / conditions / OA.
- 2026-10-14 [risk] log-6 ( run R, HEAD + `LASER_ALT`, side lasers fitted ): the down-laser never gave a sample ( `Df` 0 throughout, baro only ). This is the original H3 / F3 failure on hardware: fitted side lasers awake at 0x29 break the old `ranging_init ( )` even with no XRanging in user code.
- 2026-10-14 [decision] The A/B is reflown with the **4 side lasers unplugged** ( user ), so the old build can use its laser: R2 → log-8, A2 → log-7, same hexes.
- 2026-10-14 [decision] Wiki ( task 11 ): with side lasers fitted, user code must enable them ( `initObjectAvoidance` / `XRanging.init` ), even without `enableOA ( )`, or `initAlt` finds 0x29 busy and the down-laser is refused ( `BUS_BUSY` ).
- 2026-10-14 [risk] R2 ( log-8, today's `LASER_ALT`, side lasers unplugged ): at 120 cm, 30 s windows median 86 % ( max 93 % ) within ±5 cm, sd 4.7 cm, ~10 s bob. Today's build misses the 95 % bar too, and hover A ( best 82 % ) is inside its spread: conditions explain most of the bob. A2 decides.
- 2026-10-14 [decision] Correction: logs 6 and 8 were flown on **A2** ( a PlutoIDE build of the working tree at 16:19:46 ), not R. log-6 = the designed `BUS_BUSY` refusal ( side lasers fitted, not enabled; baro only ); log-8 = the A2 clean hover ( 120 cm: median 86 %, best 93 % ). The earlier entries reading log-6 as "the original bug reproduced" and log-8 as R2 are withdrawn. R still needs flying, from its hex.
- 2026-10-14 [risk] log-7 ( R, today's `LASER_ALT`, XSHUT wired to GPIO_5 ): `Df` 0 throughout. PB15 is SPI2 MOSI in today's firmware, so the new XSHUT wire most likely holds the down-laser in reset. Today's firmware cannot run this board as it is now wired; a fair reference needs the wire disconnected.
- 2026-10-14 [decision] A/B round 2: no XSHUT wire, side lasers unplugged, no OA. R ( old ) against A3 ( new, `initAlt ( )` no-wire path ), both logging a `Ver` field ( 1 / 2 ) so logs name their firmware. This also flies the no-wire path for the first time.
- 2026-10-14 [decision] New task 14 ( user ): SPI2 ( PB12-15 = GPIO_8-5 ) is de-initialised at the bring-up whenever XRanging is used at all; XRanging wins over the user SPI API, which is refused from then on; the M25P16 flash is marked unavailable. Without XRanging, SPI2 and the flash are unchanged.
- 2026-10-14 [decision] Task 14 review fix: `spiRelease` reconfigures only pins SPI2 still owns ( AF5 ), as pulled-up inputs, and leaves PB12 alone, so user RGB_3 / PWM / ADC / GPIO set in `plutoInit` are kept. Task 14's hardware check rides on the A3 run ( `SPIoff:1 Flash:0` ) and the next 5-laser run.
- 2026-10-14 [decision] A/B verdict: no regression. R ( log-9, today's firmware, wire off ) and A2 ( log-8, new pipeline ) move the craft identically ( `Df/10` sd 5.53 / 5.56 cm ); window error sd 3.26 / 3.41 cm. Today's bob is conditions; the task-10 hold criterion is read against same-day R.
- 2026-10-14 [risk] log-10: connecting the XSHUT wire to GPIO_5 while today's firmware ran froze its laser for good ( PB15 = SPI2 MOSI ). The wiki states that the XSHUT wire needs the new firmware with `initAlt ( GPIO_5 )`.
- 2026-10-14 [decision] Next: re-fly hover A on the current firmware ( tasks 2-8 + 14 ), with all lasers, XSHUT on GPIO_5 and OA, to see whether OA explains log-4's larger motion; it also gives task 14's 5-laser check.
- 2026-10-15 [decision] Test 11 ( log-11 ): task 14 passes on hardware ( `SPIoff:1 Flash:0` ). FRONT came up in I2C error ( `SF:3` ), probably a loose connector after the A/B replugging; re-checked on the bench ( test 12 ). The user keeps the one-error rule ( no 3-strike tolerance ).
- 2026-10-15 [out-of-scope] With OA enabled the craft moves ~60 % more under the down-laser ( `Df/10` sd 9.0 against 5.5 cm ), while the estimator error barely changes. This is OA's roll / pitch override ( pilot at 30 % authority ), an OA-behaviour topic, not this pipeline. Without OA the new pipeline equals today's firmware ( A/B ).
- 2026-10-15 [decision] Test 12 ( log-12 ): 26 status cycles with FRONT reseated, no laser ever in error: test 11's `SF:3` was the connector. Task 14 closed.
- 2026-10-15 [decision] Task 10 closed: hover B ( log-5 ) matches the baseline; with the A/B and tests 11-12, the pipeline is validated in flight. Hold quality is judged against same-day today's firmware ( no regression ), not the 21 Sep reference.
- 2026-10-15 [risk] `Flash:0` on hover B without XRanging: the M25P16 is not recognised at boot on this board. Pre-existing ( task 14 added only inactive guards ); a separate hardware / driver check.
- 2026-10-15 [out-of-scope] Found while writing the docs: `spiTransfer ( )`'s timeout path returns the SPI error count as a `bool`, so a timeout reads as success ( pre-existing, Cleanflight ). A follow-up for the bus driver, with F1.
- 2026-10-15 [decision] Task 11: the CLAUDE.md invariant change stays staged in PIPELINE_UPDATE.md and is applied at commit ( CLAUDE.md is read by live sessions ). The spec check ran as one checker agent over every claim ( ~145 statements ), not the per-requirement workflow; 3 minor contradictions fixed ( init order wording, SPI2 trigger, "all lasers held in reset" ).
- 2026-10-15 [decision] Task 12 review fix: `initAlt` refuses the RGB-LED slot and UART2 pins ( GPIO_1, 3, 6, 10-13, 16-18 ), since those drivers ( and the UWB set-up after the bring-up ) can take a pin without asking XRanging. XSHUT candidates left: GPIO_2, 4, 5, 8.
