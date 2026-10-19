# VL53L1X XRanging API - Tasks

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [DESIGN](DESIGN.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

## Resume

Read this block first in a new session; read only the task it points to.

| | |
|---|---|
| **Current task** | **Closed - pending commit** ( 19 Oct 2026 ) |
| **Next step** | The user commits ( `git commit -F .git/MAGISV2_COMMIT_MSG.txt` ); the next pluto-commit / pluto-grill fills in the hash |
| **Open questions** | none |
| **Blocked on** | nothing |
| **Last updated** | 2026-10-19 |

## Summary

| | |
|---|---|
| **Mode** | feature |
| **Goal** | `X1Ranging`: an XRanging-style user API for an all-VL53L1X drone ( down-laser for altitude hold through `initAlt`, four side lasers with object avoidance ), plus one test that proves both |
| **Done when** | A flight log shows ALT_HOLD holding height on the L1X down-laser below 160 cm with handover to the baro above it, and the craft backing off before a wall on all four sides. The gate passes on PRIMUS_X2_v1 ( one image for both families ) and flash/RAM fit. The `X1RANGING_API_WIKI.md` is written and `API_Version` is bumped |
| **Target** | PRIMUS_X2_v1 |
| **Analysis** | Option B, separate `X1Ranging` class ( [DESIGN.md](DESIGN.md) ) |
| **Pipeline docs affected** | subsystems/User_Space_API.md, Hardware_Bus_Pipeline.md, Altitude_Hold_Estimator.md, Main_Initialization.md; PIN_MAP.md; FLIGHT_INVARIANTS.md |
| **Order** | 1 → 2 → 3 → 4 → 5 → 6 → 7 → 14 → 15 → 16 → 17 → 18 → 8 → 9 → 10 → 11 → 12 → 13 |

## Index

| # | Title | Status | Depends on | Skills / agent |
|---|---|---|---|---|
| 1 | Baseline: build sizes and the L0X avoidance reference | done 2026-10-16 | - | pluto-build |
| 2 | VL53L1X multi-laser manager ( `drivers/tof_manager_l1x` ) | done 2026-10-16 | 1 | pluto-driver, pluto-rules, cpp-pro agent |
| 3 | `X1Ranging` API: header, class, bring-up, refusals | done 2026-10-16 | 2 | pluto-rules, cpp-pro agent |
| 4 | Wire X1Ranging into init and the main loop; remove the old L1X path | done 2026-10-19 | 3 | pluto-driver, pluto-rules |
| 5 | Altitude fusion reads the X1Ranging down-laser | done 2026-10-19 | 4 | pluto-rules |
| 6 | Shared object-avoidance core ( L0X unchanged ) | done 2026-10-19 | 3 | pluto-rules, cpp-pro agent |
| 7 | Single test `PlutoPilot.cpp`: alt hold + 4-side avoidance + log line | done 2026-10-19 | 5, 6 | pluto-flighttest, pluto-rules |
| 8 | `X1RANGING_API_WIKI.md` and API / FW version bump | done 2026-10-19 | 3, 6, 15, 16, 17 | spec-to-code-compliance |
| 9 | Build gate | done 2026-10-19 | 2-7, 14-17 | pluto-build |
| 10 | Hardware validation | done 2026-10-19 | 9 | pluto-flighttest, pluto-log-analyst agent |
| 11 | Architecture & pipeline docs ( PIPELINE_UPDATE.md ) | done 2026-10-19 | 10 | graphify, spec-to-code-compliance |
| 12 | Topic review | done 2026-10-19 | 11 | pluto-reviewer agent |
| 13 | Graph refresh & commit | done 2026-10-19 | 12 | graphify, pluto-commit |
| 14 | OA filter: out of range clears, first sample seeds ( both APIs ) | done 2026-10-19 | 6, 7 | pluto-rules |
| 15 | No build flag: X1Ranging chosen at run time; both used → both refused | done 2026-10-19 | 14 | pluto-rules, pluto-driver, cpp-pro agent |
| 16 | One shared RAM area for the VL53L0X / VL53L1X device tables | done 2026-10-19 | 15 | pluto-rules, c-pro agent |
| 17 | Altitude fusion: two compiled copies, picked by the family `initAlt` chose | done 2026-10-19 | 15 | pluto-rules, cpp-pro agent |
| 18 | Check: 96 cm `alt` jump at the VL53L1X laser return after a baro leg ( log-2 ) | done 2026-10-19 | - | pluto-flighttest, pluto-scout |

Status values: `todo`, `in-progress`, `done YYYY-MM-DD`, `blocked (<why>)`, `dropped (<why>)`.
Serial numbers are never reused or renumbered.

## Tasks

### 1. Baseline: build sizes and the L0X avoidance reference

**Description.** Build PRIMUS_X2_v1 twice: stock ( the L0X family ), then with `LASER_TOF_L1x` + `LASER_ALT`
enabled in `target.h`. Record text / data / bss for each in `TESTING.md`. These are the numbers that tasks 9 and 12
compare against. Also record the L0X avoidance reference that task 6 must keep identical. This is the current
`applyObjectAvoidance ( )` ( `XRanging.cpp:482` ): its constants ( `AVOID_RC_MAX` 25, brake 0.45, 500 / 500 ms
phases, 100 ms period, pilot scale 0.3 at `:567/578` ) and the order in which it writes `RC_ARRAY` and calls
`userRCassert`. Put them in `CHANGES.md` as a checklist.

- **Depends on:** -
- **Skills / agent:** pluto-build
- **Files:** [target.h](../../../../src/main/target/PRIMUS_X2_v1/target.h), [XRanging.cpp](../../../../src/main/API-Src/XRanging.cpp)
- **Safety impact:** none ( measurement only; restore `target.h` afterwards )
- **Done when:** both size rows and the avoidance checklist are written; `target.h` is back to stock
- **Rollback:** n/a
- **Status:** done 2026-10-16
- **Result:** Stock 108276 / 1280 / 16760 ( 106.9 KB, 17.6 KB ), gate clean. L1X + `LASER_ALT` 116404 / 1320 / 17624 ( +8.0 KB flash, +0.9 KB RAM ). `XVision` 964 B and `VL53L1_Dev_t` 932 B confirmed. The 10-step avoidance checklist is in CHANGES.md. `target.h` restored ( TESTING.md §1 ).

### 2. VL53L1X multi-laser manager ( `drivers/tof_manager_l1x` )

**Description.** Add `drivers/tof_manager_l1x.{h,cpp}`, the L1X counterpart of `tof_manager`. It has one
`VL53L1_Dev_t` slot per `laser_e`, each with its own state, address, status and cached sample. Bring-up per laser:
raise XSHUT alone, `WaitDeviceBooted` → probe model ID 0x010F → `DataInit` → `StaticInit` → `SetDeviceAddress`
( DOWN 0x30, sides 0x2A+n, 8-bit form as at `ranging_vl53l1x.cpp:353` ) → Medium mode, 45 ms budget, 50 ms period
→ start. *( As built: the address moves **before** `DataInit`; see CHANGES.md task 2. )* Polling: `tofL1xPollDown ( )` follows the timing and filtering of today's `getRange_L1` ( `:243` ), so the
fusion inputs do not change. `tofL1xPollSides ( )` reads one side laser per call, round-robin. Getters only read the
cache. A bus error stops only that laser. Gated on `LASER_TOF_L1x`; added to the Makefile group.

- **Depends on:** 1
- **Skills / agent:** pluto-driver, pluto-rules; delegate to the cpp-pro agent
- **Files:** new `src/main/drivers/tof_manager_l1x.{h,cpp}`, `src/main/drivers/ranging_vl53l1x.cpp` ( deleted in task 4 ) ( reference ), [Makefile](../../../../Makefile)
- **Safety impact:** none until wired in ( task 4 ). Blocking I2C on the bus the IMU shares: one result read per poll at most
- **Done when:** the L1X build compiles clean with the manager linked ( not yet called ); RAM is within the DESIGN estimate ( ~+3.7 KB net once task 4 removes `XVision` )
- **Rollback:** remove the two files and the Makefile line
- **Status:** done 2026-10-16
- **Result:** `tof_manager_l1x.{h,cpp}` ( 6 slots, `.o` 1694 text / 5765 bss ), Makefile:281. The DOWN path is line-for-line `getRange_L1`. The address moves before `DataInit`. Sides use a 10 ms round-robin and never share a loop with the DOWN poll. Stock build byte-identical, gate clean, no warnings in the new files. Review: 0 blocking; 2 should-fix and 1 nit fixed ( CHANGES.md ).

### 3. `X1Ranging` API: header, class, bring-up, refusals

**Description.** Add `API/X1Ranging.h` and `API-Src/X1Ranging.cpp`. The header has class `X1Ranging_P` and object
`X1Ranging`, reuses `laser_e` / `laser_status_e` from `XRanging.h`, and is included from `PlutoPilot.h`. Its methods
mirror XRanging: `init ( )`, `init ( laser, cm )`, `isTriggered`, `getRange`, `initAlt ( gpio )` / `initAlt ( )`,
`getStatus`, `initObjectAvoidance`, `enableOA`, `disableOA`. Requests are recorded during `plutoInit ( )` and acted
on once by `x1RangingInit ( )`, which latches. It reuses XRanging's pin table and `altPinConflict` rules ( down
XSHUT only GPIO_2/4/5/8, pin reserved after start-up, SPI2 off ). In a build without `LASER_TOF_L1x`, every method is
a stub that returns `-1` / not-enabled ( assumption A1 ). The banner header follows the house style. No static
constructors ( `-nostartfiles` ).

- **Depends on:** 2
- **Skills / agent:** pluto-rules; delegate to the cpp-pro agent
- **Files:** new `src/main/API/X1Ranging.h`, new `src/main/API-Src/X1Ranging.cpp`, [PlutoPilot.h](../../../../PlutoPilot.h), [XRanging.h](../../../../src/main/API/XRanging.h) ( read only ), [Makefile](../../../../Makefile)
- **Safety impact:** reserves the XSHUT pins and turns SPI2 off when used ( same as XRanging )
- **Done when:** both builds ( L0X, L1X ) compile clean with a `PlutoPilot.cpp` that calls every X1Ranging method
- **Rollback:** remove the two files, the include and the Makefile line
- **Status:** done 2026-10-16
- **Result:** `X1Ranging.h` / `X1Ranging.cpp` ( stubs in the L0X build ). The pin table and conflict check are reused from XRanging through 3 small helpers. `x1RangingAltXshut` is OR-ed into the three `xRanging*Claimed` checks, and `xRangingInit` returns early when X1Ranging has requests. `PlutoPilot.h` is at the **repo root**. Warning fixes at source: the `XRanging.h` OA default is `DOWN`, and `cmp16` / `cmp32` have casts ( −15 / −68 warnings ). Stock template byte-identical; gate clean on stock, stock + test and L1X + test; no new `.init_array`. Review: 0 blocking / 0 should-fix, 2 of 3 nits fixed.

### 4. Wire X1Ranging into init and the main loop; remove the old L1X path

**Description.** In `main.cpp`, delete the `ranging_init_L1` call ( `:515-519` ) and call `x1RangingInit ( )`
after `plutoInit ( )`, next to `xRangingInit ( )` ( `:594` ). In `mw.cpp`, add `tofL1xPollDown` /
`tofL1xPollSides` beside the L0X polls ( `:842-843` ), under `LASER_TOF_L1x`. Remove `ranging_init_L1`,
`getRange_L1` and `LaserSensor_L1` / `XVision`; `XVision` holds 964 B of `.bss` in every build. Make `LASER_ALT`
an `#error` that names `X1Ranging.initAlt`. In an L1X build, `XRanging` refuses every request, which replaces the
`altRequest` special case at `XRanging.cpp:316-328`. Update the `target.h` comments ( `:184-185` ).

- **Depends on:** 3
- **Skills / agent:** pluto-driver, pluto-rules
- **Files:** [main.cpp](../../../../src/main/main.cpp), [mw.cpp](../../../../src/main/mw.cpp), `src/main/drivers/ranging_vl53l1x.cpp` ( deleted in task 4 ), `src/main/drivers/ranging_vl53l1x.h` ( deleted in task 4 ), [XRanging.cpp](../../../../src/main/API-Src/XRanging.cpp), target.h
- **Safety impact:** start-up order and loop time. One down-laser only; an L1X build never touches the L0X manager
- **Done when:** both builds compile clean; grep finds no `XVision` / `getRange_L1` users; in the L0X build `.bss` is 964 B smaller than the task 1 baseline and `.text` is within a few hundred bytes of it
- **Rollback:** revert the task's commit hunk; the task 1 baseline build is the fallback
- **Status:** done 2026-10-19
- **Result:** `x1RangingInit ( )` follows `xRangingInit ( )` in main.cpp; the laser task runs `tofL1xPollDown` then `tofL1xPollSides` ( L1X ) or the L0X polls ( else ). `ranging_vl53l1x.{h,cpp}` are deleted. `LASER_ALT` is always an `#error` ( checked ). XRanging refuses everything in an L1X build ( `XRANGING_REFUSED` ). The altitudehold L1X accessors moved here from task 5. Stock 108272 / 1276 / 15796 ( bss −964 ); L1X 106584 / 604 / 19204, with only the L1X `tofDev` linked and no VL53L0X library. Gate clean. Review: 0 blocking / 0 should-fix, 2 nits for task 5.

### 5. Altitude fusion reads the X1Ranging down-laser

**Description.** *( Re-scoped 2026-10-19: the accessor swap was done in task 4, because deleting the old driver
removed the globals the macros read. )* Remove the now-dead `ALT_LASER_FUSION_BUILD` ( `altitudehold.cpp:~240-244`;
always false once `LASER_ALT` is an `#error` ) so that `altLaserFusionLatch ( )` is the only switch for both
families. Also remove the dead `|| defined( LASER_ALT )` at `:~361`, and update the comments at `:~107-111` and
`altitudehold.h:65`. Then prove the fusion is unchanged: no L1X constant changed ( dropout 185 ms, ring 14, IIR lag,
return guard ), and the 160 / 140 cm handover, frame shift and 2.5 s hold-off are untouched. An accepted `initAlt`
whose laser fails to start runs the fusion as a dropout ( `tofL1xDownOutOfRange` is true while DOWN is not OK ).

- **Depends on:** 4
- **Skills / agent:** pluto-rules
- **Files:** [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp)
- **Safety impact:** **ALT_HOLD, landing and touchdown detection**. The baro datum freeze on arm is not touched
- **Done when:** the L1X build is clean; `git diff HEAD -- altitudehold.cpp` ( tasks 4 + 5 ) shows only the include, accessor, `#error`, latch-define and comment lines, and no constant; the L0X object code for `altitudehold.cpp` is unchanged ( compare `.o` sizes or the disassembly against HEAD )
- **Rollback:** revert the accessor lines; without `initAlt`, the build flies the baro
- **Status:** done 2026-10-19
- **Result:** `ALT_LASER_FUSION_BUILD` removed: `altLaserFusionLatch ( )` is the only switch for both families ( fusion off until the latch ). Removed the dead `|| defined( LASER_ALT )` and the dead `#else checkReading` ( review nit ); comments updated. Proof: the stock `altitudehold.o` disassembly with relocations is **identical to HEAD** ( 0 of 2667 lines ), with 7434 / 23 / 764 in both; only debug-macro groups differ. No `ALT_TOF_*` define changed. Gate clean on stock and L1X. Review: 0 blocking, 2 nits ( 1 fixed, 1 corrected my note ).

### 6. Shared object-avoidance core ( L0X unchanged )

**Description.** Move the push / brake state machine out of `applyObjectAvoidance ( )` ( `XRanging.cpp:482` ) into
one core that takes a range getter and the enabled-laser set. XRanging calls it with its L0X cache, and X1Ranging
with the L1X cache; the `mw.cpp:533` call reaches whichever family is built. For the L0X, this is a refactor only.
Every item on the task 1 checklist must still hold: the same constants, the same 100 ms period, the same order of
`RC_ARRAY` writes and `userRCassert ( ROLL / PITCH, 0.0f )` calls, and the 0.3 pilot scale.

- **Depends on:** 3
- **Skills / agent:** pluto-rules; delegate to the cpp-pro agent
- **Files:** [XRanging.cpp](../../../../src/main/API-Src/XRanging.cpp), new X1Ranging.cpp, [mw.cpp](../../../../src/main/mw.cpp), new shared core file ( or a section of an existing one )
- **Safety impact:** **pilot roll / pitch while avoidance is on**. Overrides are re-asserted every 100 ms and expire otherwise. The core reads the pilot blend from `rcData`, as the L0X code always did ( CHANGES.md task 1 observation; no feedback loop: the override mixes into `rcCommand`, mw.cpp:1137 )
- **Done when:** both builds are clean; the pluto-reviewer agent confirms the task 1 checklist item by item on the diff
- **Rollback:** restore the old `applyObjectAvoidance ( )` body ( task 1 reference )
- **Status:** done 2026-10-19
- **Result:** The header-only template core `ObjectAvoidanceCore.h` ( `oaCoreRun < U >`, per-API `oaCoreState < U >` static inline members ). XRanging's `applyObjectAvoidance ( )` and the new `x1RangingApplyObjectAvoidance ( )` are one-line wrappers; mw.cpp calls the one for the built family. The stock `applyObjectAvoidance` has **the same 226 instruction words** as before; only the callee name and 3 literal-pool addresses ( −4 B, `dFilt [ 6 ]` → `filtMm [ 4 ]` ) differ. Stock 108272 / 1276 / 15792, L1X + test 108088 / 612 / 19212; gate clean; no guard variables or new constructors. Review: 0 blocking / 0 should-fix, 3 nits fixed.

### 7. Single test `PlutoPilot.cpp`: alt hold + 4-side avoidance + log line

**Description.** Write one test: `X1Ranging.initAlt ( GPIO_5 )` and `X1Ranging.initObjectAvoidance ( cm, LEFT, RIGHT,
FRONT, BACK )` in `plutoInit ( )`, avoidance enabled in `plutoLoop ( )`. A `Monitor_Print` line of **under 130 B per
tick** shows the 5 ranges, the 5 statuses, the altitude source ( laser / baro ), the estimated height and the
avoidance roll / pitch command. It is flown with the app sticks in ALT_HOLD; the test itself does not take off.
Keep a copy in `test/PlutoPilot_x1ranging_test.cpp.txt` in this folder. Restore the template `PlutoPilot.cpp`
before committing. Write the flight procedure in `TESTING.md`: bench check of all statuses; hover below 160 cm;
climb above 160 cm ( handover ) and back; walk a wall to each of the four sides.

- **Depends on:** 5, 6
- **Skills / agent:** pluto-flighttest, pluto-rules
- **Files:** [PlutoPilot.cpp](../../../../PlutoPilot.cpp), `test/PlutoPilot_x1ranging_test.cpp.txt`, `TESTING.md`
- **Safety impact:** test only. Avoidance pushes roll / pitch; the pilot can abort with Dev Mode off or a disarm
- **Done when:** the L1X build with the test compiles clean; the log line is ≤ 130 B for the widest values; the procedure is written
- **Rollback:** restore the template `PlutoPilot.cpp`
- **Status:** done 2026-10-19
- **Result:** The test is the repo-root `PlutoPilot.cpp` ( copy in `test/PlutoPilot_x1ranging_test.cpp.txt` ): `initAlt ( GPIO_5 )`, OA at **50 cm** ( user's choice ) on LEFT / RIGHT / FRONT / BACK, enabled only in Developer Mode, and one framed log line ( ~112 B per tick, 123 B worst case ). Builds clean in L1X and stock, with no warnings in `PlutoPilot.cpp`. Procedure in TESTING.md §2. Review: 0 blocking, 3 should-fix: two doc fixes applied; the third ( out-of-range hold, an existing XRanging behaviour ) became task 14.

### 8. `X1RANGING_API_WIKI.md` and API / FW version bump

**Description.** Write `docs/API/X1RANGING_API_WIKI.md` in the layout of `XRANGING_API_WIKI.md`. It covers every
method, the return values, the pins, the build switch ( `LASER_TOF_L1x` ), the refusals ( XRanging in an L1X build,
X1Ranging in an L0X build, `LASER_ALT` removed ), the SPI2 / flash loss and a full example. Update
`XRANGING_API_WIKI.md:94`, where the L1X note now points at X1Ranging. Bump `API_Version` ( minor: new API ) and
`FW_Version` ( minor ) in the Makefile. Check the wiki against the code with `spec-to-code-compliance`.

- **Depends on:** 3, 6
- **Skills / agent:** spec-to-code-compliance
- **Files:** new `docs/API/X1RANGING_API_WIKI.md`, [XRANGING_API_WIKI.md](../../../API/XRANGING_API_WIKI.md), [Makefile](../../../../Makefile)
- **Safety impact:** none
- **Done when:** the compliance check finds no contradiction; the versions are bumped
- **Rollback:** n/a
- **Status:** done 2026-10-19
- **Result:** `docs/API/X1RANGING_API_WIKI.md` is new, written for the no-flag design: choosing the laser family, both APIs → both refused, status timing, the L1X return guard, the pins it takes over, OA. `XRANGING_API_WIKI.md` is updated ( 1.8.0, family choice, the OA out-of-range release, the shared files ). Makefile: FW 3.15.0, API 1.8.0. Compliance ( one agent instead of the 30-agent workflow, session size guideline ): first pass 66 match / 3 divergences / 5 gaps, all folded in; re-check after the redesign ~70 match / 1 divergence ( only calls that enable a laser choose the family ), fixed; 5 unverifiable ( chip behaviour, runtime jitter ).

### 9. Build gate

**Description.** Run `.claude/skills/pluto-build/driver.sh --gate PRIMUS_X2_v1` on the stock ( L0X ) build, then on
the L1X build ( `LASER_TOF_L1x` on in `target.h` ) with the task 7 test. No new `src/` warnings in either build.
Record flash / RAM against the task 1 baseline in `TESTING.md`.

- **Depends on:** 2-7
- **Skills / agent:** pluto-build
- **Files:** -
- **Safety impact:** none
- **Done when:** both gates pass; the L1X RAM delta is within ~+3.7 KB net; the L0X build is within a few hundred bytes of the baseline
- **Rollback:** n/a
- **Status:** done 2026-10-19
- **Result:** One image now ( no L1X build ). Template 122652 / 1316 / 19728 ( 121.0 KB / 20.5 KB ); test 124196 / 1316 / 19860. That is +14.4 KB flash and +3.0 KB RAM over task 1; gate clean on both, no constructors. TESTING.md §4.

### 10. Hardware validation

**Description.** Fly the task 7 test on the 5 × VL53L1X drone ( down on GPIO_5, sides on 7 / 9 / 16 / 15 ). Do the
bench check first: all five statuses OK and plausible ranges. Then fly: hover below 160 cm in ALT_HOLD, climb past
160 cm and back below 140 cm, then approach a wall on each of the four sides. Save each log as a numbered
`log-N.txt` in this folder ( see the log handoff convention ). If L0X hardware is available, fly one L0X avoidance
regression with the stock build, because task 6 touched XRanging.

- **Depends on:** 9
- **Skills / agent:** pluto-flighttest; pluto-log-analyst agent for big logs
- **Files:** `TESTING.md`, `log-N.txt`
- **Safety impact:** flight test. Abort with Dev Mode off or a disarm
- **Done when:** the logs show height held on the laser below 160 cm, a clean handover both ways without a height jump, and a back-off from the wall on all four sides; results are in `TESTING.md`
- **Rollback:** fly the stock build
- **Status:** done 2026-10-19
- **Result:** Flown on the no-flag build ( log-3, log-5 ). Lasers OK; laser hover; the 160 / 140 cm handovers flown 2 × in log-5 and several times in log-3, with the user seeing no movement at the frame shifts; OA push / brake with the right sign on all four sides ( log-3 ) and FRONT ( log-5 ); a clean ALT_HOLD landing ( log-5 ). F4 ( Dev Mode off ) is code-verified, not flown. log-3 crash after hand-under tests, cause not visible in the log; log-4 had every laser BOOT_FAILED after the crash, cleared by a power-off ( log-5 `fs:0` ×5 ), and flew safely on the baro dropout path. TESTING.md §3.

### 11. Architecture & pipeline docs ( PIPELINE_UPDATE.md )

**Description.** Stage new versions of `subsystems/User_Space_API.md`, `Hardware_Bus_Pipeline.md`,
`Altitude_Hold_Estimator.md` and `Main_Initialization.md` in `PIPELINE_UPDATE.md`. They cover the init order, the
L1X I2C addresses, the fusion latch, and Mermaid charts where the flow changed, with every edge confirmed by
`graphify path` or a source line. Update `PIN_MAP.md:86`, `FLIGHT_INVARIANTS.md` and the CLAUDE.md laser bullets.
Those bullets now say: X1Ranging for the L1X, `LASER_ALT` removed, XRanging refused in an L1X build.

- **Depends on:** 10
- **Skills / agent:** graphify, spec-to-code-compliance
- **Files:** `PIPELINE_UPDATE.md`, [PIN_MAP.md](../../PIN_MAP.md), [FLIGHT_INVARIANTS.md](../../dev-guide/FLIGHT_INVARIANTS.md), [CLAUDE.md](../../../../CLAUDE.md)
- **Safety impact:** none
- **Done when:** every affected doc is staged or updated; the compliance check is clean
- **Rollback:** n/a
- **Status:** done 2026-10-19
- **Result:** `PIPELINE_UPDATE.md` ( 868 lines ). Full new text for Main_Initialization and Hardware_Bus_Pipeline; replacement blocks U1-U4 for User_Space_API and A1-A10 for Altitude_Hold_Estimator; Firmware_Pipeline needs no change. 137 Mermaid edges, each verified against a source line ( the graph has no nodes for the new functions yet ). In-place updates: CLAUDE.md laser bullets ( plus one new bullet ), PIN_MAP, FLIGHT_INVARIANTS, HARDWARE_RESOURCES, the pluto-flighttest skill. Compliance: ~230 citations and 137 edges checked; 5 divergences fixed ( a line citation, "until the next restart" ×4, SPI2 for both families, the probe count, the `BARO` example ). The pipeline folder itself is untouched. `compile_commands.json` was left alone ( stale, Linux-generated ).

### 12. Topic review

**Description.** Run the pluto-reviewer agent on the whole topic diff. Fix the BLOCKING findings and re-run the
task 9 gate.

- **Depends on:** 11
- **Skills / agent:** pluto-reviewer agent
- **Files:** -
- **Safety impact:** none
- **Done when:** no BLOCKING findings are open; the gate passes again
- **Rollback:** n/a
- **Status:** done 2026-10-19
- **Result:** pluto-reviewer on the whole diff against HEAD ( 24 tracked + 9 new files ): **0 blocking**, no code fix needed in `src/`. Fixed: the README summary still described the flag ( should-fix ); the TASKS done-when; a comment citing the deleted `ranging_vl53l1x.cpp`; `Created Date` in the `tof_manager_l1x` banners. Re-gated clean ( test file 124100 / 1316 / 19860 ). Open, logged: the warm reset with no DOWN XSHUT wire ( task 2 risk; worst case BOOT_FAILED → baro ).

### 13. Graph refresh & commit

**Description.** Run `graphify update .` and `python tools/graph_labels.py` after the review fixes, then
`/pluto-commit`. It builds all targets, applies `PIPELINE_UPDATE.md`, writes the CHANGELOG, stages the change and
drafts the message. The user commits.

- **Depends on:** 12
- **Skills / agent:** graphify, pluto-commit
- **Files:** -
- **Safety impact:** none
- **Done when:** the change is staged with a drafted message; the topic is set to Closed - pending commit
- **Rollback:** n/a
- **Status:** done 2026-10-19
- **Result:** Graph refreshed ( 4230 nodes, 8014 edges; communities relabelled; the new functions are present ). `/pluto-commit`: the template `PlutoPilot.cpp` restored; all three targets built ( PRIMUS_X2_v1 121.0 / 20.5 KB, PRIMUSX2 120.8 / 20.7 KB, PRIMUS_V5 121.0 / 20.5 KB ), no new warnings; `PIPELINE_UPDATE.md` applied; CHANGELOG v4.0.0 updated ( FW 3.15.0, API 1.8.0 ); staged with a drafted message. The user commits.

### 14. OA filter: out of range clears, first sample seeds ( both APIs )

**Description.** *( Added 2026-10-19 from the task 7 review; the user chose to fix both APIs. )* In
`ObjectAvoidanceCore.h`, `oaCoreDistance` treats `LASER_OUT_OF_RANGE` like a failed laser: the side's filter is cleared
and the distance is 0 ( no obstacle ). Today an out-of-range side reads −100 and `oaCoreFilter` keeps the last near
distance, so a removed obstacle stays "seen" and the push / brake cycle repeats. Same function, same cause: after
a clear ( or at start-up, `filtMm` = 0 ) the first valid sample is quartered, ( 0 × 3 + d ) / 4. A wall at 2 m then
reads 500 mm and fires one false push at a 50 cm threshold. Seed the filter with the first valid sample when it is 0.
This is a deliberate behaviour change for XRanging ( VL53L0X ) as well: update the task 1 checklist step 5, the
TESTING.md §2 out-of-range note, and record it for the task 8 wiki ( XRanging and X1Ranging ).

- **Depends on:** 6, 7
- **Skills / agent:** pluto-rules
- **Files:** [ObjectAvoidanceCore.h](../../../../src/main/API-Src/ObjectAvoidanceCore.h), CHANGES.md, TESTING.md
- **Safety impact:** **pilot roll / pitch while avoidance is on**, in both APIs. Avoidance stops pushing as soon as a side reads out of range; a real obstacle still triggers on its next valid sample
- **Done when:** both builds are clean; the stock `applyObjectAvoidance` disassembly differs from task 6 only in `oaCoreDistance` / `oaCoreFilter`; the reviewer confirms that out of range gives 0 and clears, that the first valid sample seeds the filter, and that every other checklist step is unchanged
- **Rollback:** revert the two lines in `ObjectAvoidanceCore.h` ( back to the task 6 behaviour, identical to HEAD's XRanging )
- **Status:** done 2026-10-19
- **Result:** In `ObjectAvoidanceCore.h`, a failed laser clears its filter at once; `LASER_OUT_OF_RANGE` holds it for up to 2 ticks and clears it on the 3rd in a row ( `OA_OOR_CLEAR_TICKS` 3, saturating `oorTicks [ 4 ]` per API ); the first valid sample after a clear seeds the filter. The 3-tick debounce came from the review: with an instant clear, a flickering target lost push ticks while the brake stayed at full strength, a net roll toward the obstacle. The stock `applyObjectAvoidance` is unchanged from task 6 ( 0 of 241 lines ); only `oaCoreDistance` changed ( 0x74 ). Stock + test 109660 / 1276 / 15932, L1X + test 108616 / 604 / 19348; gate clean. Re-review: 0 blocking, 0 should-fix.

### 15. No build flag: X1Ranging chosen at run time; both used → both refused

**Description.** *( Added 2026-10-19: the user does not want the `LASER_TOF_L1x` flag. X1Ranging is to be chosen like
the VL53L0X, by calling it. )*

- Compile `X1Ranging.cpp` and `tof_manager_l1x.cpp` in every build: remove their `LASER_TOF_L1x` gates and the stubs.
- Add one boot-latched **laser family** variable ( none / VL53L0X / VL53L1X ), set after `plutoInit ( )` from which API
  has requests.
- If both XRanging and X1Ranging have requests, **both are refused**: no laser starts, both report
  `LASER_NOT_ENABLED`, and the fusion stays off ( baro ).
- `XRANGING_REFUSED` becomes a run-time test: the family is VL53L1X, or both were requested.
- mw.cpp polls both managers and calls both OA wrappers. Each returns at once unless its family runs, and each keeps
  its PollDown → PollSides order.
- Remove the define from the X2_v1 `target.h` and update both `target.h` comments. `LASER_TOF_L1x` becomes an `#error`,
  like `LASER_TOF`.
- `ALT_TOF_*` stays compile-time until task 17. Until then only the VL53L0X fusion constants exist, so a VL53L1X
  altitude test must not be flown on this build ( ranging and OA are fine ).

- **Depends on:** 14
- **Skills / agent:** pluto-rules, pluto-driver; cpp-pro agent
- **Files:** X1Ranging.cpp / .h, XRanging.cpp, tof_manager_l1x.cpp / .h, main.cpp, mw.cpp, API-Utils.h, both target.h, altitudehold.cpp ( the `#error` ), the test PlutoPilot.cpp header comment, the TESTING.md §2 build step
- **Safety impact:** start-up, and which laser drives altitude hold and OA; the both-used refusal
- **Done when:** the stock build ( no define ) runs X1Ranging from user code; user code that calls both APIs starts nothing; gate clean; XRanging's L0X paths unchanged ( `applyObjectAvoidance` disassembly the same as task 14 ); flash / RAM recorded
- **Rollback:** revert the task; the task 14 state with the flag is the fallback
- **Status:** done 2026-10-19
- **Result:** The flag is gone ( `#error` ), and X1Ranging and `tof_manager_l1x` are in every build. `rangingFamilyLatch ( )` runs before `xRangingInit` and gives NONE / VL53L0X / VL53L1X / BOTH. With BOTH, both APIs drop their requests, nobody latches the fusion, OA is inert, and SPI2 stays on. mw.cpp picks the OA wrapper by family and polls both managers. `applyObjectAvoidance` is identical to task 14. **Temporary:** with the VL53L1X family, `x1RangingInit` latches the fusion **off** until task 17 ( review should-fix ). Stock + test 121496 / 1316 / 21740 ( both device tables until task 16 ); gate clean. Review: 0 blocking, 3 should-fix ( latch fixed, TESTING / test header fixed, wikis → task 8 ), 1 nit ( SPI2 note → task 8 ).

### 16. One shared RAM area for the VL53L0X / VL53L1X device tables

**Description.** *( Added 2026-10-19, user decision. )* After task 15 both managers' device tables are linked in every
build ( VL53L0X 6 × 352 B = 2.1 KB, VL53L1X 6 × 932 B = 5.6 KB ). Only one family can run ( task 15's latch ), so both
tables go into one storage area: a union, or one static buffer that each manager uses as its own type. The manager of
the family that is not latched must never touch it, so check the family before any access. Target: about +3.5 KB
`.bss` over today's stock build instead of +5.6 KB.

- **Depends on:** 15
- **Skills / agent:** pluto-rules; c-pro agent
- **Files:** tof_manager.cpp, tof_manager_l1x.cpp ( and a small shared header )
- **Safety impact:** memory corruption if both managers ever ran; prevented by the family latch
- **Done when:** `nm` shows one area of ~5.6 KB and no second table; both families build and run their bring-up ( the review checks that every access is gated by the family ); gate clean
- **Rollback:** separate tables again ( +2.1 KB )
- **Status:** done 2026-10-19
- **Result:** The new `drivers/tof_dev_area.{h,cpp}` hold `union tofDevArea_u { VL53L0X_Dev_t l0x [ 6 ]; VL53L1_Dev_t l1x [ 6 ]; }` ( 0x15d8 B, .bss ); each manager uses only its own member. `tofManagerBringUp` / `tofL1xBringUp` return false unless `rangingFamily ( )` is their own family; every other access needs status OK, which only that gated bring-up sets. Disassembly against the pre-task objects: 13 / 14 and 15 / 16 functions identical, and only the two bring-ups gained the gate ( +8 B / +32 B ). 124196 / 1316 / 19860: bss −2112. Gate clean. Review: 0 blocking; doc should-fix ( here and in task 11 ) and banner nit fixed.

### 17. Altitude fusion: two compiled copies, picked by the family `initAlt` chose

**Description.** *( Added 2026-10-19, user decision: option (a). )* The fusion body in `altitudehold.cpp` uses
per-sensor constants and accessors ( `ALT_TOF_*`, `tofNew` … `tofRawCm`, the L1X-only return guard ). Today
`#if defined( LASER_TOF_L1x )` chooses them. Turn the sensor-dependent part into a template over a small traits struct
( constants + accessors ) and instantiate it twice, for VL53L0X and for VL53L1X. `altLaserFusionLatch ( family )`
records which copy runs; the family comes from task 15's variable, set when `initAlt` was accepted. No constant changes.
Proof: the VL53L0X copy's code matches today's stock `altitudehold.o` ( disassembly ), and the VL53L1X copy matches the
fusion of the current `LASER_TOF_L1x` build.

- **Depends on:** 15
- **Skills / agent:** pluto-rules; cpp-pro agent
- **Files:** altitudehold.cpp / .h, XRanging.cpp, X1Ranging.cpp ( latch call )
- **Safety impact:** **ALT_HOLD, landing and touchdown detection** on both laser families
- **Done when:** both copies are linked; VL53L0X copy equivalence shown against the task 5 reference, VL53L1X copy against the task 14 `LASER_TOF_L1x` build; flash cost recorded ( ~3-4 KB expected ); gate clean
- **Rollback:** revert to task 16 ( only the VL53L0X fusion reachable at run time )
- **Status:** done 2026-10-19
- **Result:** Traits `AltTofL0x` / `AltTofL1x` and templates `checkReadingFor<T>` ( noinline ), `altShiftFrame<T>`, `tofWindow*<T>`, plus the rings per copy. `checkReading ( )` dispatches on `altLaserFusionL1x`, set by `altLaserFusionLatch ( on, vl53l1x )` ( XRanging `false`, X1Ranging `true`; the task 15 temporary off is gone ). **Both copies are instruction-identical to their references** ( 38 / 39 functions each; only the latch differs by the flag store ): L0X against the task 15 object, L1X against today's file compiled with `-DLASER_TOF_L1x` ( the fusion log-2 flew ). The last `LASER_TOF_L1x` uses are gone ( only the `#error` remains ). Stock + test 124156 / 1316 / 21972 ( +2.7 KB flash, +232 B RAM ); gate clean. Review: 0 blocking, 1 should-fix ( test / TESTING warning now wrong, fixed ), 2 nits fixed.

### 18. Check: 96 cm `alt` jump at the VL53L1X laser return after a baro leg ( log-2 )

**Description.** *( Added 2026-10-19 from log-2. That flight ran on the old `LASER_TOF_L1x` build, so this is the
committed VL53L1X fusion from vl53l1x-althold-parity, not new code. )* What log-2 shows:

- At t ≈ 52.4 s the down range stepped 905 → 288 mm. `src` went 1 → 2 ( held on the baro after an object step ), and `alt` stayed smooth.
- At t ≈ 56.5 s, with D ≈ 160 cm, `src` went 2 → 0 smoothly.
- During the baro leg `alt` drifted to 25-38 cm while D read 121-131 cm.
- At t ≈ 73.4 s ( D ≈ 125 cm, below the 140 cm return ) `src` went 0 → 1 and `alt` went **29 → 125 cm in one tick**.

The return should shift the altitude frame so the craft does not move ( CLAUDE.md flight invariant ). Determine:

1. Whether the estimate jump moved the craft. That needs the user's account and a setpoint field.
2. Which return path ran: the guard's "new floor" exit after 2.5 s steady, or a plain return.
3. Why the baro leg was ~95 cm low: the frozen offset at the object step, or baro drift.

Findings go into INVESTIGATION.md with ranked causes. A fix, if one is needed, gets its own task or a `/pluto-grill fix`.

- **Depends on:** -
- **Skills / agent:** pluto-flighttest ( log fields ), pluto-scout or graphify for the return-guard path
- **Files:** log-2.txt; [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp) ( return guard ~:193-209, ~:1247-1310 )
- **Safety impact:** none ( investigation ). The behaviour concerns ALT_HOLD on the VL53L1X
- **Done when:** the cause is ranked with evidence, and a next step ( a fix task, a logging change, or "by design" ) is agreed with the user
- **Rollback:** n/a
- **Status:** done 2026-10-19
- **Result:** **By design.** The one-tick `alt` changes in log-2 and log-3 are frame shifts: `altShiftFrame<T>` moves the estimate and the setpoints ( `AltHold`, `altTarget`, `altGoal` ) by the same amount, so the craft does not move. The user confirmed no visible movement ( log-3 ), and log-2's step at 52 s was the user's hand too. Sequence: a hand ( or object ) held for 2.5 s re-bases the frame onto it; on removal the step goes to the baro ( `src:2` → `0` ) in that frame; when the laser returns over the floor, the frame shifts back. The ~95 cm "baro low" in log-2 is that frame offset, not baro drift. INVESTIGATION.md.

## Decisions log

Newest last. One line each: `YYYY-MM-DD [decision|assumption|out-of-scope|risk] text`.

- 2026-10-16 [decision] Mode feature; target PRIMUS_X2_v1.
- 2026-10-16 [decision] Hardware: 1 down VL53L1X ( XSHUT GPIO_5 ) + 4 side VL53L1X on the stock XRanging pins LEFT=7, RIGHT=9, FRONT=16, BACK=15.
- 2026-10-16 [decision] Design B: separate `X1Ranging` class ( user's pick over the scout's A ); name `X1Ranging`, header `API/X1Ranging.h`, wiki `X1RANGING_API_WIKI.md`.
- 2026-10-16 [decision] Reuse `laser_e` / `laser_status_e` from XRanging.h; no new enum.
- 2026-10-16 [decision] `LASER_TOF_L1x` means X1Ranging ( all lasers L1X ); the old `ranging_init_L1` / `XVision` / `LASER_ALT` path is removed; `LASER_ALT` becomes `#error`; XRanging is refused in an L1X build.
- 2026-10-16 [decision] All L1X lasers run Medium mode, 45 ms budget / 50 ms period ( same as the down-laser ).
- 2026-10-16 [decision] Avoidance behaviour is the same as XRanging ( push ±25 / brake 0.45 / 500 + 500 ms / pilot 0.3 ), through one shared core; the L0X behaviour is identical ( refactor only ).
- 2026-10-16 [decision] One test file with both checks and a log line; flown with the app sticks in ALT_HOLD.
- 2026-10-16 [decision] Fusion constants, the 160 / 140 handover, the frame shift and the 2.5 s hold-off are unchanged.
- 2026-10-16 [assumption] A1: X1Ranging methods are stubs ( −1 / not-enabled ) in an L0X build, so `PlutoPilot.cpp` builds in both families.
- 2026-10-16 [assumption] A2: avoidance keeps XRanging's current failsafe / landing behaviour.
- 2026-10-16 [risk] R1: 4 sides at 50 ms round-robin, so each side updates ~every 50 ms, which may be slow for avoidance; the test shows it.
- 2026-10-16 [risk] R2: the shared avoidance core touches XRanging.cpp; L0X equivalence by checklist review and an L0X regression flight if hardware is available.
- 2026-10-16 [out-of-scope] Mixed L0X + L1X on one drone; avoidance as a firmware flight mode; a ceiling / up laser; any behaviour change to the L0X XRanging; vl53l1x-error-latch-recovery and vl53l1x-extended-reach stay separate topics.
- 2026-10-16 [decision] Baseline ( task 1 ): the L1X build must stay ≤ ≈ 22 KB RAM; the L0X `tofDev` table must not be linked into the L1X build.
- 2026-10-16 [risk] Observation: `applyObjectAvoidance` reads `rcData [ ROLL / PITCH ]`, not `rcDataPilot`, for the pilot blend. Kept as is for L0X equivalence; possible follow-up topic.
- 2026-10-16 [decision] DESIGN correction: the pilot blend is 0.3 in push and idle, **0.2 in brake** ( `USER_OA_BLEND_BRAKE` ), not 0.3 throughout.
- 2026-10-16 [decision] Task 2: the L1X address moves before `DataInit`. This vendored `VL53L1_SetDeviceAddress` does not update the NVM copy that the FULL config writes back ( `vl53l1_api.c:755`, `vl53l1_register_funcs.c:102` ), so a later move would put the chip back at 0x29 on start.
- 2026-10-16 [decision] Task 2: `tofL1xPollSides` skips any loop in which `tofL1xPollDown` used the bus. Task 4 must call PollDown first, then PollSides, in the same loop.
- 2026-10-16 [decision] Task 2: altitude hold's `tofFiltCm` and `tofRawCm` both map to `tofL1xDownRangeMm` in task 5; `tofL1xDownRawMm` is for diagnostics only.
- 2026-10-16 [decision] Task 3: warnings that a new file brings in from shared headers are fixed at the source, never hidden. The `XRanging.h` / `X1Ranging.h` OA unused-laser default is `DOWN` ( ignored as `-1` was ); `cmp16` / `cmp32` have explicit casts ( same code ).
- 2026-10-16 [decision] Task 3: X1Ranging reuses XRanging's pin table and `altPinConflict` through `xRangingSide*` / conflict helpers ( no copy ). A down XSHUT is reserved through `x1RangingAltXshut ( )` inside the existing `xRanging*Claimed` checks.
- 2026-10-16 [out-of-scope] Task 3 review nit: a refused `initAlt ( GPIO_5 )` still keeps FRONT's LED dark ( `isLedUsable` compares the requested pin, not the latched one ). XRanging does the same ( `XRanging.cpp:124` ), so both stay as they are for parity. Possible follow-up for both APIs.
- 2026-10-19 [decision] Task 4: the altitudehold L1X accessor swap moved from task 5 to task 4. Deleting `ranging_vl53l1x` removes the globals the macros read, so the L1X build would not compile between the tasks. The mapping is the planned one ( review-checked ). Task 5 keeps the latch cleanup and the equivalence proof.
- 2026-10-19 [decision] Task 4: in an L1X build XRanging refuses everything ( `XRANGING_REFUSED` ), so VL53L0X side lasers through XRanging are no longer possible in that build ( HEAD allowed them ). Task 8 wiki note and version bump.
- 2026-10-19 [decision] Task 4: the PRIMUS_V5 `target.h` laser comments were updated too ( comment only ), because `LASER_ALT` is an `#error` on every target.
- 2026-10-19 [risk] Never refresh `tools/warnings-baseline.json` from an L1X build log: its "fixed" list counts code an L1X build does not compile. Only from the stock build at commit.
- 2026-10-19 [decision] Task 5: one fusion switch for both laser families. `altLaserFusionLatch ( )` starts off ( tau 2 s ) and is set once after `plutoInit ( )`. `x1RangingInit ( )` always latches in an L1X build, also with no request ( `X1Ranging.cpp:132-136` ), and `xRangingInit ( )` does so in an L0X build. Nothing reads `altLaserFusion ( )` before `loop ( )` ( review ).
- 2026-10-19 [decision] Task 6: the OA core is header-only templates, a first in this tree. Each API gets its own `oaCoreState < U >` of separate C++17 `static inline` members, all zero-initialised: no guard variables, no `.init_array`. A single state struct changed the L0X codegen ( one base register, `vpush d8` ), so separate members were kept to get identical instructions. Each `U` is instantiated in one file only.
- 2026-10-19 [decision] Task 7: OA distance **50 cm** for the test ( user ). OA is enabled in `onLoopStart` and disabled in `onLoopFinish`, so it acts only in Developer Mode. The log is one framed line ( 4 B framing per `Monitor_Print` call made separate fields ~150 B ).
- 2026-10-19 [decision] Task 7 review → task 14 ( user ): fix the OA "sticky obstacle" in the shared core for **both** APIs. Out of range clears the side's filter, and the first valid sample seeds it ( also removes a one-tick false push at start-up ). This deliberately changes XRanging ( VL53L0X ) behaviour, overriding the "no L0X change" scope for this one bug; task 8 wiki note.
- 2026-10-19 [decision] Task 14: out-of-range is debounced, 3 OA ticks ( 300 ms ). Why: with an instant clear, a dropout at a real obstacle zeroed that tick's push, but the brake ( −last × 0.45 ) ran at full strength, so a flickering target could net-roll the craft **toward** it ( review trace: 50 units of push against 56 of brake ). Now a short dropout keeps the push and a removed obstacle is released within ~300 ms.
- 2026-10-19 [out-of-scope] Task 14 review nit: the OA filter and counters are not reset on `disableOA` / `enableOA`. After a re-enable, the first 1-2 out-of-range ticks can hold a distance from before the disable ( up to 2 push ticks ). The same is true at HEAD; possible follow-up ( clear the state on enable ).
- 2026-10-19 [decision] **Design revision ( user ): no `LASER_TOF_L1x` build flag.** X1Ranging is chosen at run time by calling it, like XRanging. This replaces the 2026-10-16 decision "`LASER_TOF_L1x` = X1Ranging" and assumption A1 ( stubs ). One boot-latched laser-family variable is set from which API was used. Both APIs used → both refused ( user ). Tasks 15-17 added; task 8 waits for them.
- 2026-10-19 [decision] Fusion constants without the flag: two compiled copies of the fusion body ( a template over sensor traits ), picked by the family `initAlt` chose. The VL53L0X copy stays the flown code ( user, option (a) ). The two managers' device tables share one RAM area ( user ).
- 2026-10-19 [risk] Until task 17 lands, a no-flag build has only the VL53L0X fusion constants: do not fly a VL53L1X altitude test on a task 15 / 16 build.
- 2026-10-19 [decision] Task 8 compliance check ( one agent instead of the 30-agent workflow, session size guideline ): 66 statements match, 3 doc divergences and 5 reverse gaps. Applied when the wiki is rewritten: DOWN reads `LASER_OUT_OF_RANGE` until its first sample; "until the next restart"; `initAlt ( GPIO_8 )` also darkens BACK's LED; `EXTERNAL` in `initObjectAvoidance` is never avoided; the L1X return guard; side XSHUT pins are not reserved.
- 2026-10-19 [decision] Task 15 review: until task 17, `x1RangingInit` latches the fusion **off** for the VL53L1X family. Otherwise the fusion would read the idle VL53L0X entry and fly its baro branch ( tau 1.5 s, laser deadband, absolute take-off goal ) while reporting `fu:1`, a mode never flown. Task 17 restores `altLaserFusionLatch ( altOn )`.
- 2026-10-19 [decision] log-2 ( flown on the old `LASER_TOF_L1x` build ): the hardware works ( 5 lasers, no boot / I2C errors ) and OA BACK pushes / brakes as specified. The 96 cm `alt` jump at the laser return became task 18 ( check ).
- 2026-10-19 [decision] Task 17 runs before task 16: it removes the last `LASER_TOF_L1x` uses ( the dead altitudehold branches ) and brings VL53L1X laser altitude back; the user asked again that no flag remain. Task 16 ( shared RAM ) follows.
- 2026-10-19 [decision] Task 17: fusion state placement. The 25 function statics and the window ring are kept per copy; `altSourceLaser`, `altStepPending`, `ToF_Height`, the baro values and the frame values are shared, because only one copy runs per boot. The range accessors return mm as `uint16_t` and the body keeps the `( float ) x / 10.0f`, which is what kept the code identical. The risk logged for tasks 15 / 16 ( no VL53L1X altitude ) is closed.
- 2026-10-19 [decision] Task 16: the drivers in `drivers/` include `API/API-Utils.h` for `rangingFamily ( )`. There is a precedent ( `tof_manager.cpp` already includes `API/Scheduler-Timer.h` ) and no include cycle. Task 11 must stage the shared area and its rule ( only the latched family touches it ) in `PIPELINE_UPDATE.md`.
- 2026-10-19 [decision] Task 18 closed as by design: logged `alt` jumps at a re-base or a laser return are frame shifts ( the estimate and the setpoint move together ); the user saw no movement. log-2's 52 s step was the user's hand.
- 2026-10-19 [risk] log-3 ended in a crash after hand-under tests ( hand ~6 cm under the drone, two re-bases onto the hand, then a real 80 cm drop in 0.6 s with large stick / OA input ). The log cannot show the cause ( no throttle / setpoint field ). Task 10 stays open for a clean landing ( log-4 ). Consider adding the setpoint to the test log line if it repeats.
- 2026-10-19 [decision] log-4: every laser BOOT_FAILED on the log-3 image after the crash; the drone flew safely on the baro ( dropout fallback ). The test START line now prints `failedStep` / `lastError` per laser ( temporary, test file only ) to separate a single broken XSHUT ( BUS_BUSY on all ) from a bus / power fault.
- 2026-10-19 [decision] Task 10 closed on log-3 + log-5. F4 ( Dev Mode off during a push ) was not flown; it is accepted on code review ( Dev Mode off clears `RC_ARRAY` / `userRCflag`, mw.cpp:1204-1210, and `onLoopFinish` calls `disableOA` ). log-4's all-lasers BOOT_FAILED was hardware after the crash, cleared by a power-off.
- 2026-10-19 [risk] Side lasers read out of range 45-55 % of the time in open rooms ( R / F / B in log-5 ). OA only sees obstacles within the Medium-mode reach; already in the wiki.
- 2026-10-19 [decision] Task 11: `compile_commands.json` is not patched. It was generated on a Linux machine ( `/home/mecash/...` ) and is already stale before this topic: it lacks `drivers/tof_manager.cpp` from tof-single-pipeline. Patching in only this topic's files would make it look current. Regenerate it with `bear` on that machine ( tooling, not this topic ).
- 2026-10-16 [risk] Task 2: on a warm reset with no XSHUT wire, an L1X may still be ranging; the old driver had the same gap. If a bench test shows it, add `VL53L1_StopMeasurement` when from == to.
