# VL53L1X XRanging API - Design

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [DESIGN](DESIGN.md)

## Problem

The VL53L0X has a full user API ( `XRanging` ): several lasers on XSHUT pins, a down-laser chosen with
`initAlt ( gpio )` that feeds altitude hold, and object avoidance on the side lasers. The VL53L1X has none of that.
It is a single build-time down-laser ( `LASER_TOF_L1x` + `LASER_ALT` ) fixed at 0x29, with no XSHUT, no status and
no user API. `XRanging.initAlt` is refused in an L1X build ( `XRanging.cpp:316-328` ), and the L0X probe cannot see
an L1X ( `ranging_vl53l0x.cpp:115` ), so the two cannot share the bus today.

Wanted: the same thing for an all-VL53L1X drone, meaning one down-laser for altitude hold and four side lasers for
avoidance, plus one test that proves both.

## Options ( from the scout, SCOUT.md )

| | A. Same `XRanging` API, family at build time | **B. Separate `X1Ranging` class** | C. Runtime sensor type per slot |
|---|---|---|---|
| How | `LASER_TOF_L1x` switches the XRanging backend to an L1X manager | New `API/X1Ranging.h` + `API-Src/X1Ranging.cpp` over a new `drivers/tof_manager_l1x` | One manager, each slot L0X or L1X |
| User code | Unchanged | `XRanging` → `X1Ranging` ( same names via `laser_e` ) | Extra type argument |
| Flash | ~+8 KB, L1X build only | ~+8 KB, L1X build only ( a little more than A ) | ~+8 KB in **every** build |
| RAM | ~+3.5 KB, L1X build only | Same as A | Larger device size in every slot, every build |
| Fusion | Constants unchanged | Constants unchanged | Constants become runtime values ( re-validation ) |
| Risk | Low | Low-medium ( two APIs, shared avoidance code ) | High |

## Choice: B, `X1Ranging` ( user's pick, 16 Oct 2026 )

The user picked B over the scout's recommendation of A. B gives the VL53L1X its own explicit API and wiki, and user
code says which sensor it drives. The details agreed in the interview:

- **Names:** object `X1Ranging`, header `API/X1Ranging.h`, wiki `docs/API/X1RANGING_API_WIKI.md`. It reuses
  `laser_e` and `laser_status_e` from `XRanging.h`, so `X1Ranging.init ( LEFT )` and `X1Ranging.getRange ( FRONT )`
  read the same as XRanging.
- **Methods mirror XRanging:** `init ( )`, `init ( laser, cm = 20 )`, `isTriggered`, `getRange` ( mm; −100 invalid,
  −1 not enabled ), `initAlt ( gpio )` / `initAlt ( )`, `getStatus`, `initObjectAvoidance ( cm, l1..l4 )`,
  `enableOA`, `disableOA`.
- **Build gate:** X1Ranging is live only when `LASER_TOF_L1x` is defined. That build calls itself "all lasers
  VL53L1X". The old single-sensor path ( `ranging_init_L1`, `getRange_L1`, `LaserSensor_L1` / `XVision`,
  `main.cpp:515-519` ) is removed, and `LASER_ALT` becomes an `#error` that points at `X1Ranging.initAlt`. In an L1X
  build, `XRanging` calls are refused; in an L0X build, `X1Ranging` calls are refused. Both headers always compile,
  so user code builds either way ( see assumption A1 ). The L0X build is unchanged.
- **Bring-up:** this mirrors `xRangingInit`. It latches once after `plutoInit ( )`. All XSHUTs go low; DOWN wakes
  alone and moves to 0x30; each side laser then wakes alone and moves to 0x2A+n. The probe checks the L1X model ID
  ( 0x010F ). Each laser has its own state, one bus error stops only that laser, and the getters read cached samples.
- **Ranging:** every laser runs Medium mode, 45 ms budget, 50 ms period, as the flight-validated down-laser does
  today. The side lasers are polled round-robin ( one per tick ), as in `tofManagerPollSides`.
- **Pins:** down XSHUT on GPIO_5 for the test ( still only GPIO_2/4/5/8 are allowed ). Sides use the XRanging pin
  table: LEFT=GPIO_7, RIGHT=GPIO_9, FRONT=GPIO_16, BACK=GPIO_15, EXTERNAL=GPIO_14. SPI2 and the flash are off while
  X1Ranging is used, as with XRanging.
- **Altitude:** the altitudehold L1X accessor macros read X1Ranging's DOWN slot. The L1X constants ( dropout,
  ring, IIR, return guard ), the 160 / 140 cm handover, the frame shift and the 2.5 s hold-off are not changed.
  Fusion is on only if `X1Ranging.initAlt` was accepted, using the same latch as XRanging.
- **Object avoidance:** one shared core ( push ±25 for 500 ms, brake at 0.45 for 500 ms, pilot roll and pitch
  scaled to 0.3 in the push and idle phases and 0.2 in the brake phase, 100 ms re-assert; full list in CHANGES.md ). It takes a range getter, and both APIs call it. For the L0X this is a refactor
  only: the constants, timing and outputs stay identical.

## Cost estimate

- RAM ( L1X build ): 6 × `VL53L1_Dev_t` ( 932 B ) ≈ 5.6 KB, minus 964 B for the removed `XVision`, so about
  +3.7 KB net. RAM is 18 KB of 40 KB today. If the budget is tight, dropping the EXTERNAL slot saves 932 B.
- Flash ( L1X build ): ~+8 KB over L0X, 114.8 KB against 106.6 KB in the parity topic. The L0X build gains a few
  hundred bytes for the stubbed `X1Ranging` and the shared avoidance core.
- Loop: up to ~0.8 ms blocking I2C per tick ( one result read ) on the bus the IMU shares.

## Assumptions and risks

- **A1:** in a build without `LASER_TOF_L1x`, `X1Ranging` methods are stubs that return `-1` / not-enabled, the same
  as XRanging's refusals. User code then builds in both families. ( The interview chose "compiled only with
  `LASER_TOF_L1x`"; stubs keep `PlutoPilot.cpp` buildable. Change this if a link error is preferred. )
- **A2:** avoidance keeps XRanging's current failsafe and landing behaviour; nothing new is added there.
- **R1:** four side L1X plus the down-laser at Medium mode means each side updates only about every 50 ms. That is
  slower avoidance than the L0X; the test will show whether it is enough.
- **R2:** the shared avoidance refactor touches `XRanging.cpp`. L0X equivalence is shown by review ( same constants
  and order of operations ), and by an L0X regression flight if L0X hardware is available.

## Revision 2026-10-19: no build flag ( user )

The user does not want the `LASER_TOF_L1x` flag. X1Ranging is chosen at run time, the way XRanging is, by calling it
from `plutoInit ( )`. What changes against the choice above:

- **Build gate → run-time family.** `X1Ranging.cpp` and `tof_manager_l1x.cpp` are compiled in every build. After
  `plutoInit ( )` one variable is latched: none, VL53L0X or VL53L1X, from which API has requests. **Both used → both
  refused**: no laser starts, and altitude hold uses the baro. `LASER_TOF_L1x` becomes an `#error`, like `LASER_TOF`
  and `LASER_ALT`.
- **Fusion constants ( option a ).** The fusion body becomes a template over sensor traits ( constants + accessors ),
  instantiated for VL53L0X and VL53L1X; the latch picks the copy. The VL53L0X copy stays the flown code ( proved by
  disassembly ). Cost: ~3-4 KB of flash.
- **RAM.** Both device tables are linked in every build, but only one family runs, so they share one area: ~5.6 KB,
  about +3.5 KB over stock ( RAM ~19.4 KB of 40 ).
- **Assumption A1** ( stubs in an L0X build ) no longer applies.

Tasks 15 ( flag removed, family latch ), 16 ( shared RAM ) and 17 ( two fusion copies ) carry it out; the wiki
( task 8 ) follows.
