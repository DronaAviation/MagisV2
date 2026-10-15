# ToF Single Pipeline - Scout

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md)

Implementation brief from `pluto-scout` ( 14 Oct 2026, fix planning ), kept verbatim. The root-cause
reconnaissance is in the check topic's [SCOUT.md](../laser-tof-init-conflict/SCOUT.md) and
[INVESTIGATION.md](../laser-tof-init-conflict/INVESTIGATION.md); it is not repeated here.

---

## Scout follow-up: implementation facts for laser-tof-init-conflict (fix, option B)
Graph: stale (built 84c326d, HEAD c495d6a). I answered from grep and source only. Nothing was edited.

**1. Every gate** (8 files, 23 directives). Targets: `LASER_TOF`, `LASER_TOF_L1x` and `LASER_ALT` are all commented out at PRIMUS_X2_v1 target.h:183-185 and PRIMUS_V5:185-187. PRIMUSX2:182-183 has no L1x line. These comments get deleted.
- **Driver**
  - ranging_vl53l0x.cpp:74-249 gates `ranging_init`, `getRange`, `isTofDataNew` and `isOutofRange`. The new manager replaces them: delete them, or compile them always.
  - vl53l1x.cpp:104-314 stays a compile-time gate.
- **Estimator** (altitudehold.cpp)
  - :108 `#error` for both lasers: drop it. `initAlt` refuses to run when L1x is compiled in.
  - :111-235 holds constants and the `tof*()` accessor macros. They must stay compile-time but become unconditional: `#if L1x … #else` L0X block, with the L0X accessors pointing at the DOWN entry. The :229 `#error` goes.
  - :232/234 `_time_constant_z` (1.5 vs 2.0): change to a runtime set plus `updateGains ( )` at latch time.
  - :332 `BARO||LASER_ALT`: every target defines BARO, so leave it.
  - :918 `checkBaro`/`checkReading`: becomes a runtime `if`.
  - :969-1336 (altHoldSource, ring, altShiftFrame, checkReading): always compiled.
  - :1048/:1059 `LASER_TOF||L1x`: remove.
  - :1224/:1280 L1x: stays compile-time.
  - :1339 tau: runtime `if`.
  - :1360 `correctedWithTof`: always compiled.
  - altitudehold.h:59 `altHoldSource` declaration: always declared. It has no caller outside altitudehold.cpp.
- **imu/command**
  - imu.cpp:51 `#define ALT_EST_ACC_Z_DEADBAND`: make it unconditional.
  - imu.cpp:261: runtime `if`.
  - command.cpp:116 `setDesiredPosition` vs `DesiredPosition_setRelative`: runtime `if`.
- **main/mw**
  - main.cpp:516 `ranging_init`: delete.
  - main.cpp:521 L1x: stays.
  - mw.cpp:814: runtime manager poll.
  - mw.cpp:819 L1x: stays.
- **API**: there are no `LASER_*` gates. opticflow.cpp:355-356 reads `NewSensorRange` (only under `OPTIC_FLOW`, which is off on all targets). It needs a shim or the DOWN entry.

**2. L1x fusion depends on `LASER_ALT`.** The L1x constants live inside `#ifdef LASER_ALT` at :111/:185. `LASER_TOF_L1x` alone only reads the laser (main.cpp:523, mw.cpp:821). To keep L1x builds as they are today: latched flag = `defined(LASER_TOF_L1x) && defined(LASER_ALT)`. So `LASER_ALT` survives as a define that only has meaning together with L1x. The alternative is to make L1x always mean fusion, which drops the "L1x reading-only" mode.

**3. No, the fusion is not compiled today without `LASER_ALT`.** The last builds:
- L0X + ALT: 104.5 KB / 15.2 KB.
- Laser off, same day: about 102.5 KB / 14.8 KB.

That gives about +2-2.5 KB flash and about +0.4 KB RAM. The ring alone is 3×24×4 = 288 B. `altitudehold.o` will no longer be byte-identical, so equivalence for the flag-off case has to be argued another way.

**4. XRanging surface**
- XRanging.h:29-36 `laser_e` = LEFT 0, RIGHT, FRONT, BACK, EXTERNAL 4. Append `DOWN = 5` so EXTERNAL keeps its value.
- XRanging.h:40 private `triggerThreshold[5]` (public header).
- API-Utils.h:106 / API-Utils.cpp:134 `isXLaserInit[5]`.
- XRanging.cpp:210 `dFilt[5]`, with literal indices 0-3 at :301-325.
- XRanging.cpp:54-60 `lasers[]` table (LED, XSHUT): LEFT 6/7, RIGHT 10/9, FRONT 5/16, BACK 8/15, EXTERNAL -1/14. `LASER_COUNT` at :63.
- Indexing without an upper-bound check: :145-146 `init(laser)`, :150, :160-161, :166. :167-168 `lasers[laser]` assumes table order equals enum order. :253 checks only `l<0`.
- XRanging.cpp:392-733 is a commented-out duplicate of the file.
- `DOWN` clash: none. I grepped every `.h` under src/ and lib/, and PlutoPilot.h includes (PlutoPilot.h:26-40).

**5. Init order.** main.cpp:514 sets `SYSTEM_STATE_READY`, then :518 `ranging_init`, then :540 `plutoInit ( )`, then :542-597 (timers, pwm, :595 `OledStartUpInit`), then :599 `APIAdcInit`, then :600 `xRangingInit`. Nothing between :540 and :600 reads laser state; the loop starts after `init()` returns.

**6. `UPDATE_LASER_TOF_TASK` in mw.cpp**
- Defined at mw.cpp:742; `PERIODIC_TASK_COUNT` at :746.
- `executePeriodicTasks` (:748) round-robins one case per `loop()` pass when RX is not due (:1241/:1268). That is free-running, not tied to looptime 3500 µs. On X2_v1 there are 5 cases.
- The case is always compiled, but its body is empty without the defines.
- `getRange` paces itself with `rangePoll.set(33,true)`.
- Separately, `applyObjectAvoidance` (mw.cpp:532, in `annexCode`) blocks on side-laser reads every 100 ms.

**7. Versions and wiki**
- Makefile:31 `FW_Version 3.11.3`, :32 `API_Version 1.4.1`.
- docs/API/ holds `BMS_API_WIKI.md` and `OLED_API_WIKI.md`.
- BMS layout: `# MagisV2 <X> API Wiki`, intro with "API version **x**", `## Quick Start` (PlutoPilot code), `## Options` table, `###` notes (timing, safety), `## Examples` per hook, `## Changes in x`, `## File Architecture` table.

**8. Makefile**
- Makefile:276 `DRONA_DRIVERS` holds ranging_vl53l0x.cpp at :279. It is in `DRONA_SRC`, which all three targets include (:346, :354, :362).
- A new `drivers/tof_manager.cpp` goes in `DRONA_DRIVERS`.
- Existing oddity: :278 `drivers/display_ug2864hsweg01` has no extension.

**9. Size and timing**
- Current PRIMUS_X2_v1 build (Build/PRIMUS_X2_v1, 14 Oct 11:14): text 102540, data 1280, bss 15944. That is 101.4 KB flash and 16.8 KB RAM.
- `cycleTime` (mw.cpp:135, set at :1289, µs between imuUpdates) exists. The API does not expose it.
- No task-level `micros()` timing hook exists.

**10. GPIO**
- XRanging drives XSHUT with `Peripheral_Init`/`Peripheral_Write` (Peripherals.h:153/171; pin map at Peripheral-GPIO.cpp:45-62). There is no ownership registry.
- Claimed pins:
  - The XRanging table above.
  - GPIO_11 by UWB (main.cpp:603).
  - GPIO_10/11 are SWD.
  - GPIO_16 (PA15) is the WS2811 flight-status pin, and it is also FRONT's XSHUT.
  - GPIO_1/3/4/12/13 double as PWM/RGB pins.
- GPIO_2 = PB2 is shared with `ADC_1` (`_isAdcEnable[ADC2_IN12]`, Peripheral-ADC.cpp:60/272). Refuse it if ADC_1 is enabled.

## Suggested task cut
1. Tof manager core (`drivers/tof_manager.cpp`, Makefile): per-device `VL53L0X_Dev_t`, status, results and data-ready, replacing the shared globals. Build only.
2. XRanging API: `DOWN`, arrays 6, bounds checks, `initAlt(gpio)/initAlt()` record plus pin validation, L1x refusal, status getter.
3. Single bring-up at :600 (down-laser first, XSHUT or move off 0x29, then side lasers); delete :516-520. Bench T1-T4.
4. Boot-latched flag; convert the estimator, imu, command and mw gates to runtime and make the L0X fusion unconditional.
5. Paced round-robin poll with data-ready reads; cached XRanging getters; OA reads from cache; `micros()` timing check.
6. Gate build on all three targets, plus an L1x+`LASER_ALT` check build.
7. Hardware flight validation (baro-only and `initAlt`).
8. Docs: XRanging wiki, `API_Version`/`FW_Version` bump, target.h comments, CLAUDE.md invariant, `PIPELINE_UPDATE.md`.

## Risks / for the user
- L1x: keep `LASER_ALT` as a define meaningful only with L1x, or make L1x always mean fusion?
- EXTERNAL's place in the bring-up order and its address.
- Fix the FRONT-XSHUT/WS2811 PA15 overlap here or leave it?
- Is a flag-off build without byte-identical `.o` files acceptable when equivalence is argued by inspection plus a flight?
