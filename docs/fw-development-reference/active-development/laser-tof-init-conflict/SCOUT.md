# Laser ToF Init Conflict - Scout brief

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md)

The `pluto-scout` brief from planning ( 14 Oct 2026 ), verbatim. Line numbers are as of `c495d6a`.

---

## Scout brief: Down-laser (LASER_TOF/LASER_ALT) and XRanging object-avoidance lasers break each other at init
Graph: stale (built 84c326d9, HEAD c495d6aa; the one newer commit only touches the OLED, so the laser code is unaffected)

Subsystems: Altitude estimator laser, User API XRanging, I2C1 bus, init sequence → pipeline docs: `fw-architecture-pipeline/subsystems/Altitude_Hold_Estimator.md`, `Main_Initialization.md`, `User_Space_API.md`, `Hardware_Bus_Pipeline.md`, `PIN_MAP.md`

Key code:
- `src/main/main.cpp:518` `ranging_init()` (LASER_TOF) / `:523` `ranging_init_L1()` (LASER_TOF_L1x): sets up the down-laser at 0x29. No XSHUT.
- `src/main/main.cpp:540` `plutoInit()`: the user's `initObjectAvoidance` only sets `isXLaserInit[]` (`XRanging.cpp:250-261`).
- `src/main/main.cpp:600` `xRangingInit()` (`XRanging.cpp:79-127`): drives XSHUT low, then brings each side laser up at 0x29 and moves it to 42, 43, ... (0x2A+).
- `src/main/drivers/ranging_vl53l0x.cpp:43-56` holds file-level globals that both pipelines share: `MyDevice`, `Global_Status`, `RangingMeasurementData`, `isTofDataNewflag`, `out_of_range`, `Range_Status`.
- `ranging_vl53l0x.cpp:164` `getRange()`: the alt poll, gated every 33 ms; called from `mw.cpp:812-821` `UPDATE_LASER_TOF_TASK`.
- `ranging_vl53l0x.cpp:252/345/354` `LaserSensor::init/setAddress/startRanging`: the side-laser class (VL53L0X only).
- `mw.cpp:532` `applyObjectAvoidance()`: runs every 100 ms (`XRanging.cpp:278`) and reads up to 4 lasers in turn.
- `altitudehold.cpp:168-184`: alt-hold fusion accessors that read the shared globals (`tofRawCm()` = `RangingMeasurementData.RangeMilliMeter`).

Mode: check

Current behaviour: Two independent pipelines on I2C1 (PB8/PB9, `I2C_DEVICE I2CDEV_1`, target.h:113). They share one default address, 0x29, and one error latch.

**Pipeline A (alt laser).** `#define LASER_TOF` / `LASER_TOF_L1x` / `LASER_ALT` are commented out in `target/PRIMUS_X2_v1/target.h:183-185` and `PRIMUS_V5/target.h:185-187`.
- Init: `ranging_init()` sets `MyDevice.I2cDevAddr=0x29` and runs DataInit, StaticInit, SPAD, RefCal, SINGLE_RANGING mode, a 33 ms budget, and VCSEL 18/14 (`:85-159`).
- Poll: `getRange()` runs StartMeasurement then GetRangingMeasurementData, filters with the IIR (`LASER_LPS 0.1`), and writes `NewSensorRange`, `out_of_range` and `isTofDataNewflag`.
- Use: `altitudehold.cpp` reads these. The L1x path uses `MyDevice_L1` at 0x29 and its own `Global_Status_L1` (`ranging_vl53l1x.cpp:83,198`).

**Pipeline B (XRanging).** The pin table is at `XRanging.cpp:54-60`:

| Laser | LED pin | XSHUT pin |
|---|---|---|
| LEFT | GPIO_6 | GPIO_7 |
| RIGHT | GPIO_10 | GPIO_9 |
| FRONT | GPIO_5 | GPIO_16 |
| BACK | GPIO_8 | GPIO_15 |
| EXTERNAL | none | GPIO_14 |

- Init: `xRangingInit()` puts every enabled side laser in reset (XSHUT low), then releases them one at a time. Each one gets `init()` at 0x29 (CONTINUOUS mode, 20 ms budget) and `setAddress(42++)`.
- Reads: on demand from `getRange()`/`isTriggered()`/OA, each a blocking `startRanging()`.
- There is no `ALL` enum. "all" means `XRanging.init()` (`:133`, 4 lasers) or 4 arguments to `initObjectAvoidance`.

Analysis (hypotheses, ranked):

**H1 (most likely, chain proven from the code; hardware not yet confirmed): the down-laser gets re-addressed along with LEFT, then one I2C error stops every laser.**
1. The down-laser has no XSHUT, so it stays at 0x29 throughout `xRangingInit`.
2. When the first enabled side laser is released (`:103`), two devices answer at 0x29. `LaserSensor::init()` reconfigures both of them, which also switches the down-laser to CONTINUOUS / 20 ms.
3. `setAddress(42)` writes reg 0x8A=0x2A, and both devices obey (`ranging_vl53l0x.cpp:348`, `vl53l0x_api.c:314`).
4. The down-laser now sits at 0x2A, but `MyDevice.I2cDevAddr` is still 0x29. The next `getRange()` gets no answer, the I2C driver times out and recovers the bus (`bus_i2c_stm32f30x.c:123-130`), and `Global_Status` latches non-zero.
5. `Global_Status` is shared and nothing clears it. `getRange()` (`:172,189`) and every `LaserSensor::startRanging()` (`:363,372`) skip their work, so the side lasers return -100 forever. OA's `filterDistance` then returns 0 and no avoidance happens.
6. Result: "nothing works". On top of that, LEFT and the down-laser share 0x2A, so any reads made before the latch are mixed (wired-AND).
- For: the code path is deterministic.
- Against: none found in the code.
- Test: run an I2C address scan (`I2C_Read` probe 0x29..0x2E) after boot, and log `Global_Status`, `Range_Status` and `NewSensorRange` with `Monitor_Print`. Expected: 0x29 empty, 0x2A..0x2D present, `Global_Status` non-zero.

**H2: shared result globals feed side-laser readings into the alt-hold estimator, even with no latch.** `startRanging()` writes the shared `RangingMeasurementData`, `isTofDataNewflag=true` and `out_of_range` (`:375-394`). The estimator then takes a side laser's raw range (`tofRawCm`) and its out-of-range flag as if they came from the down-laser. Only the VL53L0X (LASER_TOF) build is affected.
- Test: hold a hand in front of FRONT with the craft on the bench, and watch the alt-hold raw/source change.

**H3: the side lasers were never in reset at boot, so `ranging_init()` talks to up to 5 devices at once.** XSHUT GPIOs are not set as outputs until `main.cpp:600`, after `ranging_init()` at :518. If the modules pull XSHUT high, every module is at 0x29 during the down-laser's SPAD/RefCal reads, and the calibration reads back mixed (wired-AND).
- Test: scope the XSHUT lines at power-up, or drive the XSHUT pins low before `ranging_init()`.

**H4 (LASER_TOF_L1x variant): VL53L1X at 0x29 plus VL53L0X side-laser init at 0x29.**
- The status latches are separate, so the alt laser does not latch from B. But the VL53L0X init frames (8-bit register index) land on the L1X (16-bit index) as garbage writes, and reads collide. Side-laser init likely fails and latches the L0X `Global_Status`, so OA dies.
- The L1X probably stays at 0x29 (a 2-byte `setAddress` frame is a no-op for it). Whether its configuration gets corrupted is suspected, not proven.
- Test: same address scan, plus `Global_Status` vs `Global_Status_L1`.

**H5 (secondary): load on the shared IMU bus.** OA makes 4 blocking reads every 100 ms. CONTINUOUS mode re-issues StartMeasurement every read and never checks data-ready or clears the interrupt. That means stale data and I2C bursts on the IMU bus, but it does not explain "no data at all".

Constraints / invariants that apply:
- LASER_ALT: only one down-laser, and the 160/140 cm handover (CLAUDE.md flight invariants; `altitudehold.cpp:108-109` `#error`).
- `Global_Status_L1` latch recovery is a planned topic (`vl53l1x-error-latch-recovery`). Do not run a blocking re-init in flight.
- OA overrides go through `userRCassert` and expire; OA reads `rcData` for roll/pitch, not throttle, which is allowed (pluto-rules rcData vs rcDataPilot).
- Keep `loop()` bounded: `VL53L0X_DEFAULT_MAX_LOOP 20000` polling in StartMeasurement (`vl53l0x_def.h:86`).
- A public `XRanging.h` change means a `docs/API` wiki (none exists for XRanging today) and an `API_Version` bump.
- `lib/` is vendored: do not edit the VL53L0X API.

Resource conflicts (PIN_MAP.md:25-46):
- LEFT XSHUT GPIO_7 = PB13 (SPI2_SCK) and LEFT LED GPIO_6 = PB14 (SPI2_MISO), both on the M25P16 flash SPI2 (`target.h:55-60`).
- BACK LED GPIO_8 = PB12 is the flash CS.
- RIGHT LED GPIO_10 = PA13 is SWDIO.
- FRONT XSHUT GPIO_16 = PA15 is the WS2811 pin (TIM8_CH1 / DMA2_Ch3).
- I2C1 is shared with the IMU and baro.

Already answered by the code:
- The init order is `ranging_init` (:518) → `plutoInit` (:540) → `xRangingInit` (:600).
- XRanging supports VL53L0X only.
- Side-laser addresses start at 42 in table order, skipping disabled ones.
- `LASER_ALT` itself is not the trigger. `LASER_TOF` + XRanging is enough.
- `XVision` (an L1X instance) is unused (`main.cpp:377`).

Questions for the user:
1. Does the down-laser connector have an XSHUT line to any MCU pin on PRIMUS_X2_v1 / PRIMUS_V5? If so, which GPIO?
2. Which down-laser is fitted: VL53L0X or VL53L1X? Which target?
3. How many side lasers are fitted, and on which XSHUT pins? Do the modules have XSHUT pull-ups?
4. Exact user code: `XRanging.init()` or `initObjectAvoidance(dist, LEFT, RIGHT, FRONT, BACK)`?
5. Should the down-laser keep 0x29 (side lasers elsewhere), or should everything sit in one table?
6. Is the flash or SPI2 used on your build, given the PB12-PB14 overlap?
7. Is XRanging allowed to run when LASER_ALT is off, or are the two independent features?

Doc drift:
- `Main_Initialization.md:38-40` shows the PlutoAPI init before `SYSTEM_STATE_READY`. In the code, READY is set at `main.cpp:514`, before `ranging_init`/`plutoInit`/`xRangingInit`, and the laser inits are not documented.
- `PIN_MAP.md:58-61` / `Altitude_Hold_Estimator.md:9` say "one laser at 0x29" and never mention XRanging's use of 0x29 → 0x2A-0x2D or its XSHUT/LED pins.
- `User_Space_API.md` has nothing on XRanging.

Prior work: `tof-althold-fusion` (closed, 484226c), `vl53l1x-althold-parity` (closed, 7cf2444), `vl53l1x-error-latch-recovery` (planned, related), commit 0790344 (XRanging and OA added).

Centralisation sketches (for later):
- **A. Reserve the down-laser in XRanging.** Keep the down-laser at 0x29. Before `ranging_init`, force all XSHUT pins low (an early hook), and bring the side lasers up after it, starting at 0x2A. Give `LaserSensor` its own status and result fields instead of the shared globals.
  - Small: about 100 B flash, about 10 B RAM.
  - Works only if the down-laser is the last device at 0x29.
- **B. One ToF manager** (`drivers/tof_manager.c`). A table {role, type L0X/L1X, XSHUT pin or none, address, per-device status/result}, one boot sequence (all XSHUT low → down-laser → sides), and one round-robin non-blocking poll from the periodic task. OA and the alt laser become consumers.
  - About 1-2 KB flash, about 60 B RAM per device. The bus load becomes bounded.
- **C. Down-laser as the EXTERNAL entry with its own XSHUT** (needs a hardware line). It is addressed like the others; `ranging_init` takes its address from the table.

Safety note for all three: the estimator must only ever see down-laser data, and never latch on another device's error.

Suggested skills: pluto-rules, pluto-driver, pluto-build, pluto-flighttest; agent cpp-pro; spec-to-code-compliance (VL53L0X multi-sensor XSHUT procedure).

Pipeline docs to update at the end:
- `Main_Initialization.md`: real order, laser init.
- `PIN_MAP.md`: XSHUT/LED pins, the 0x2A+ address range.
- `Altitude_Hold_Estimator.md`: coexistence with side lasers.
- `User_Space_API.md`, plus a new `docs/API` XRanging wiki.
- `Hardware_Bus_Pipeline.md`: I2C1 device list.
