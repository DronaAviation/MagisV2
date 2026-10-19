# VL53L1X XRanging API - Scout brief

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [DESIGN](DESIGN.md)

`pluto-scout` brief from planning ( 16 Oct 2026 ), verbatim. Graph was stale ( built 794002dc, HEAD 5e26823 ) but
matched the ranging code read.

---

## Scout brief: VL53L1X version of XRanging (several L1X on XSHUT pins, initAlt for altitude hold, object avoidance) plus one test

**Subsystems:** User API, Hardware Bus (I2C1), Altitude Hold Estimator, Main Initialization. Pipeline docs: `subsystems/User_Space_API.md`, `Hardware_Bus_Pipeline.md`, `Altitude_Hold_Estimator.md`, `Main_Initialization.md`.

**Key code**
- `src/main/API/XRanging.h`: `laser_e {LEFT, RIGHT, FRONT, BACK, EXTERNAL, DOWN}` (unscoped, global names) and `laser_status_e`. The methods are `init()`, `init(laser, cm=20)`, `isTriggered`, `getRange` (mm, -100 invalid, -1 not enabled), `initAlt(gpio)` / `initAlt()`, `getStatus`, `initObjectAvoidance(cm, l1..l4)`, `enableOA`, `disableOA`.
- `src/main/API-Src/XRanging.cpp:192` `xRangingInit()`: latch, then reset all XSHUTs low, then DOWN alone moved to 0x30, then side lasers one at a time moved to 0x2A+n, then LED blink. The pin table is at :61. LEFT=GPIO_7, RIGHT=GPIO_9, FRONT=GPIO_16, BACK=GPIO_15, EXTERNAL=GPIO_14. The down XSHUT comes from `initAlt`, and only GPIO_2/4/5/8 are allowed (`altPinConflict` :108).
- `XRanging.cpp:316-328` `altRequest()`: `#if defined(LASER_TOF_L1x) return false;`. This is the "initAlt refused" check.
- `XRanging.cpp:482` `applyObjectAvoidance()`, called from `mw.cpp:533`. Every 100 ms it pushes ±25 (`AVOID_RC_MAX`) for 500 ms, then brakes for 500 ms at gain 0.45. It writes `RC_ARRAY` and calls `userRCassert(ROLL/PITCH, 0.0f)`.
- `src/main/drivers/tof_manager.{h,cpp}`: 6-slot VL53L0X table (`tofDev[6]` = 0x840 B). `tofManagerPollDown` (33 ms, 0.1 IIR) and `tofManagerPollSides` (one laser per 10 ms, round-robin) are called from `mw.cpp:842-843`.
- `src/main/drivers/ranging_vl53l1x.cpp:203` `ranging_init_L1()`: one device, fixed at 0x29, Medium mode, 45 ms budget, 50 ms TIMED period. `:243 getRange_L1()` polls every 10 ms and does a 17-byte result read plus interrupt clear.
- `src/main/flight/altitudehold.cpp:174-235`: per-sensor constants and accessor macros (`tofNew` / `tofOutOfRange` / `tofFiltCm` …), chosen at compile time. `:238-262` holds `ALT_LASER_FUSION_BUILD` and `altLaserFusionLatch()`.
- `src/main/main.cpp:517` calls `ranging_init_L1` before `plutoInit` (:534). `:594` calls `xRangingInit`.

**Mode:** feature (all-VL53L1X: 1 down + N side, each on its own XSHUT).

**Current behaviour**
- The L1X down-laser is chosen at build time. `LASER_TOF_L1x` alone gives ranging only. Adding `LASER_ALT` makes fusion always on.
- The L1X stays at 0x29 for the whole run. There is no XSHUT handling, no address move, no status, and no user API.
- XRanging and OA support the VL53L0X only.
- `vl53l0xProbe` (`ranging_vl53l0x.cpp:115`) reads the 8-bit register 0xC0 and expects 0xEE. It cannot detect a VL53L1X (16-bit registers, model ID 0x010F = 0xEA). So a VL53L1X at 0x29 is invisible to the "is the bus busy" check. This is the known "L1x + XRanging collide at 0x29" (tof-single-pipeline/README.md:50, TASKS.md:383).

**Analysis: design options**

- **A. Choose the sensor family at build time; keep the same XRanging API (recommended).** `LASER_TOF_L1x` would mean "every laser is a VL53L1X".
  - New `drivers/tof_manager_l1x.cpp` with the same `tofManager*` / `tofDown*` interface. The family is picked by `#if` or a small backend struct.
  - `xRangingInit` gets an L1X probe (0x010F) and an L1X bring-up: `WaitDeviceBooted` → `DataInit` → `StaticInit` → `SetDeviceAddress` → mode / budget.
  - `initAlt` is no longer refused. `LASER_ALT` is removed and the runtime latch is used instead.
  - The altitudehold accessor macros point at the new DOWN accessors. The L0X fusion and its constants stay byte-identical.
  - `ranging_init_L1`, `getRange_L1`, the `LaserSensor_L1` / `XVision` class and `main.cpp:515-519` are deleted.
  - User code is the same for both sensors. `PlutoPilot.cpp` changes are only pin choices.
  - Cost:
    - RAM: `VL53L1_Dev_t` is 0x3a4 = 932 B (map). Six slots is 5.6 KB against today's 2.1 KB for L0X, so about +3.5 KB in the L1X build only. A smaller table (down + 4) would save some of that.
    - Flash: an L1X build was 114.8 KB against 106.6 KB (TASKS.md:235).
    - Loop: about 0.79 ms per L1X result read (parity topic). With round-robin it is at most one result per 10 ms tick, plus a data-ready read.
  - API: behaviour change only, no new symbols. Minor `API_Version` bump and wiki update.
  - Risk: low for L0X builds if the changes are gated.
- **B. A separate `X1Ranging_P X1Ranging` class.** New `API/X1Ranging.h`, a new cpp and an L1X manager.
  - The enum names would collide (`LEFT` etc. are global), so this needs a new enum or must reuse `laser_e`.
  - OA would be duplicated or refactored, there would be two wikis, and the most flash.
  - It only pays off for mixed L0X+L1X drones, which are out of scope.
- **C. One manager where each slot has a runtime sensor type** (e.g. `initAlt(pin, L1X)`).
  - Both ST libraries are linked in every build (about +8 KB flash) and every slot uses the RAM of the larger device.
  - The fusion constants (dropout 185 vs 120 ms, ring 14 vs 24, IIR lag 0 vs 0.3, the L1X-only return guard) would become runtime values. That changes the flight-validated estimator, so this is the highest risk.

**Replace or coexist:** under A, `LASER_TOF_L1x` + `LASER_ALT` can be replaced. `LASER_TOF_L1x` stays as the family switch, and `LASER_ALT` becomes `initAlt`. Add an `#error` on `LASER_ALT`, as was done for `LASER_TOF`.

**Constraints / invariants**
- Fusion is latched once after `plutoInit`. The 160 / 140 cm handover, frame shift and 2.5 s hold-off must stay unchanged (CLAUDE.md flight invariants; FLIGHT_INVARIANTS.md).
- One down-laser only.
- The XSHUT pin is reserved after start-up. SPI2 is switched off whenever XRanging is used.
- An override must be re-asserted or it expires (`max(250 ms, 2×userLoop)`). OA already re-asserts every 100 ms.
- Keep `Monitor_Print` under 130 B per tick in the test.
- `-Wconversion`. New `.cpp` files must be added to the Makefile `COMMON_SRC` group (around :279).
- There is no DMA use on I2C1 (polled, shared with the IMU), so no new DMA or timer conflict.

**Resource conflicts**
- I2C1 on PB8/PB9 is shared with the IMU. Every L1X read is blocking I2C inside the loop.
- XSHUT pins are as in `PIN_MAP.md:66-68`. GPIO_5 = PB15 is also SPI2_MOSI, ADC_2 and PWM_4.

**Already answered by the code**
- The L1X platform layer addresses each device through its own `Dev->I2cDevAddr` (`vl53l1_platform.c:124-228`), so several devices work.
- `VL53L1_SetDeviceAddress` takes the 8-bit address (`ranging_vl53l1x.cpp:353`).
- Today's stock build keeps `XVision` in RAM (964 B, .bss) through a static constructor in `.init_array` that never runs. Deleting it gives that RAM back in every build.
- Latest X2_v1 size: text 108276 + data 1280, bss 16760, which is about 107 KB flash of 252 KB and 18 KB RAM of 40 KB.
- With OA enabled, pilot roll and pitch are always scaled to 0.3 (`XRanging.cpp:567/578`).

**Questions for the user**
1. How many side L1X lasers, and which `laser_e` slots and XSHUT pins? Are the stock pins kept?
2. Which pin is the down XSHUT (GPIO_2/4/5/8)?
3. Is a build-time family switch acceptable? With option A, one firmware build cannot drive both kinds of sensor.
4. Side distance mode and budget: Short mode at 20-33 ms for faster OA, or the down-laser's Medium 45/50 ms?
5. OA distance for the test, and are push ±25 and pilot scaling 0.3 acceptable?
6. Fly the test with the app sticks or with user takeoff / hover commands?
7. Is losing SPI2 and the flash acceptable as with the L0X?

**Doc drift:** none found. Main_Initialization.md:45 and Hardware_Bus_Pipeline.md:46 describe the current L1X path correctly. They will go stale after this change.

**Prior work**
- Closed: tof-single-pipeline (`edfbd61`), which deferred this as "X1Ranging" (TASKS.md:359/368/383); laser-tof-init-conflict (D8); vl53l1x-althold-parity (`7cf2444`).
- Planned: vl53l1x-error-latch-recovery and vl53l1x-extended-reach.

**Suggested skills:** pluto-rules, pluto-driver, pluto-build (gate L0X and L1X builds), pluto-flighttest / pluto-log-analyst, spec-to-code-compliance, pluto-commit. Use the cpp-pro agent for the manager.

**Docs to update at the end**
- The four subsystem docs above: init order, I2C addresses for the L1X, and the fusion latch text.
- `PIN_MAP.md:86`.
- `docs/API/XRANGING_API_WIKI.md:94`, where the L1X refusal goes away.
- FLIGHT_INVARIANTS.md and the CLAUDE.md laser bullet.
- target.h comments :184-185.
- CHANGELOG, plus the `FW_Version` / `API_Version` bump.
