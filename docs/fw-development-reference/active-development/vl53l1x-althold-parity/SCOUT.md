# VL53L1X Altitude Hold Parity - Scout brief

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md)

`pluto-scout` brief from planning ( 23 Sep 2026, HEAD `b1f070b` ), verbatim.

---

## Scout brief: bring the VL53L1X (`LASER_TOF_L1x` + `LASER_ALT`) altitude path up to the VL53L0X laser-fusion level
Graph: stale (report says built f3260960, HEAD b1f070b). Its nodes already include the 484226c code (`altShiftFrame`, `tofWindow*`), so only the report header is out of date.

Subsystems: Altitude estimator, ToF drivers → pipeline doc: `docs/fw-development-reference/fw-architecture-pipeline/subsystems/Altitude_Hold_Estimator.md` (Laser fusion section)
Key code:
- `src/main/flight/altitudehold.cpp:104-159`: all `ALT_TOF_*` constants (under `LASER_ALT`)
- `altitudehold.cpp:905-970`: ring buffer, `tofWindowMismatch`, `altShiftFrame`. These are **under `#ifdef LASER_TOF` only**, so the L1x branch cannot call them.
- `altitudehold.cpp:988-1205`: the L0X fusion body
- `altitudehold.cpp:1208-1235`: the L1x branch
- `src/main/drivers/ranging_vl53l1x.cpp:74-161`: `ranging_init_L1`, `getRange_L1`, `isOutofRange_L1`
- `src/main/drivers/ranging_vl53l0x.cpp:59,170-230`: `tofRequestReseed`, `getRange` (with the IIR)
- `src/main/main.cpp:516-524` (init), `src/main/mw.cpp:804-814` (`UPDATE_LASER_TOF_TASK`)

Mode: fix (the refactor part is closer to `improve` for L0X, which must stay identical)

Current behaviour: The L1x branch reads a new valid sample and tilt-corrects it. It then corrects the estimator with `ToF_Height` whenever `0 < ToF_Height < 350`, taking `baro_offset = filtered - EstAlt` on every tick; otherwise it uses the baro. `altSourceLaser` never changes, so `altHoldSource()` always returns 1 on L1x.

**Analysis**

*Bug 1, out of range (proven from the code).* Past the sensor's reach the driver sets `out_of_range_L1 = true` (vl53l1x.cpp:143-148) and leaves `NewSensorRange_L1` as it was. `checkReading` updates `ToF_Height` only when `!isOutofRange_L1()` (:1209). The fusion test at :1219 does not check validity or age, so `correctedWithTof(stale)` runs every tick for as long as the stale value is below 350, and the baro never takes over.

*Tilt (proven).* At :1214, `calculateTiltAngle()` returns deci-degrees (:537-538). The code divides by 10, converts to radians (at most about 1.57), then tests `tilt < 25`. That test is always true: the cosine is always applied and no sample is ever rejected.

*Bug 2, the driver may stop updating (suspected, strong; a bench test confirms or rules it out).* `getRange_L1` never calls `VL53L1_ClearInterruptAndStartMeasurement()` after `GetRangingMeasurementData` (:129). The ST library says that call is the "handshake which clears the interrupt" and enables the next range (`lib/main/VL53L1X_API/core/src/vl53l1_api_core.c:2397-2404`). If that holds:
- `GetMeasurementDataReady` stays 1, so the driver re-reads the result block every 10 ms (`RANGE_POLL`).
- `VL53L1_check_ll_driver_rd_state` (`vl53l1_core.c:255,311`) can return `GPH_SYNC_CHECK_FAIL`, which latches `Global_Status_L1` and ends all ranging.
- Either way `ToF_Height` would freeze, and bug 1 then keeps pulling the estimator toward that frozen value.

Test: on the bench, log `Global_Status_L1`, `RangingMeasurementData_L1.StreamCount`, `Range_Status_L1` and the raw mm while moving the board by hand.

*Handover offset (proven).* `baro_offset` is one tick of `filtered` (a 0.75 IIR of the baro, :985) minus the smoothed integer `EstAlt`. The baro path then subtracts it from the raw `Baro_Height` (:1232), so the switch carries IIR lag plus one baro sample of noise (13 cm sd) into the offset.

*Sensor-independent parts of 988-1205:* handover state, dropout timeout, return count, offset average, frame shift, suspect/window/step/hold-off/re-base, and the landing re-base. Parts tied to the VL53L0X: `tofRequestReseed()` (IIR), `ALT_TOF_IIR_LAG_S`, `RangingMeasurementData.RangeMilliMeter` as the raw value and `NewSensorRange` as the filtered one, and every constant given as a sample count.

**Constants that depend on the sensor.** The L1x sample period today is the ST default of 100 ms: `StaticInit` selects the AUTONOMOUS/TIMED preset with a 41 ms budget and a 100 ms inter-measurement period (`vl53l1_api.c:996-1007`), and `SetDistanceMode(MEDIUM)` keeps both (:1027-1100). The L1x values below are derived, not measured.

| Constant | L0X value | What sets the L1x value |
|---|---|---|
| `ALT_TOF_DROPOUT_MS` 120 | 3×33+10 | 3×T+10, so 310 at 100 ms |
| `ALT_TOF_STEADY_SAMPLES` 15 | 0.5 s | 0.5 s / T, so 5 |
| `ALT_TOF_RETURN_SAMPLES` 3 | 3 samples | keep the count or the time |
| `ALT_TOF_RING_LEN` 24 | >0.66 s | ≥ 0.66 s / T |
| `ALT_TOF_IIR_LAG_S` 0.3 | 0.1 IIR | 0 with no IIR, plus about half the budget of latency |
| offset dt clamp 0.05 s (:1166, "one sample period") | 33 ms | about T |
| `ALT_TOF_HANDOVER_UP/DOWN` 160/140 | L0X dropped at ~172 cm | the L1x reach on the user's floor (bench) |
| driver range cut | 2000 mm | 4500 mm |
| reseed | IIR | no-op |

Sensor-independent: the 25° tilt limit, `ALT_TOF_STEP_CM` 30, `SUSPECT` 15, `ESCALATE` 20, `WINDOW_MS` 500/400, `STEP_HOLD_MS` 2500, `STEADY_CM` 10, `ALT_BARO_OFFSET_TAU_S` 2, `AIRBORNE_GRACE` 1000. The step/window values may still want a different cone: L1x ROI 27° by default versus L0X 25°.

**Refactor options**

- **A. Sample struct plus a params struct feeding one `laserFusionStep()`.** Each sensor block fills {new, valid, rawCm, filtCm} and passes a const params table plus a reseed function pointer. Touches only altitudehold.cpp and moves the helpers to `LASER_ALT`. Flash about +150 B, RAM about 0. Risk: the order of operations and the indirection change the L0X object code, so equivalence can only be shown by flight.
- **B. Duplicate the block for L1x.** No L0X risk, roughly +2.5 KB flash in the L1x build only, and two copies to maintain from then on.
- **C (recommended). One code path with per-sensor compile-time constants and accessors.** Selected with `#if LASER_TOF / #elif LASER_TOF_L1x`, using small inline accessors (`tofNew()`, `tofRawCm()`, `tofFiltCm()`, `tofOutOfRange()`, `tofReseed()`). No runtime cost, and L0X equivalence can be proven by diffing `objdump -d` of `altitudehold.o` for the L0X build before and after. Add `#error` if both are defined. The driver fix (clear interrupt, stale/age flag, optionally `SetInterMeasurementPeriod`/timing budget) goes in ranging_vl53l1x.cpp.

Whatever is chosen, the driver freeze must be ruled out first, or everything downstream is tuned to a dead sensor.

**Constraints / invariants that apply**
- Never re-zero the baro in flight. The frame shift must move only the estimator frame, never the `barometer.cpp` datum (FLIGHT_INVARIANTS).
- Landing re-base during a hold-off must stay (:1088-1096), so touchdown detection is not delayed.
- Keep `Monitor_Print` at ≤ about 115-130 B per tick with the app connected.
- Strict `-Wconversion`, float-only maths; the L1x driver has `0.25f` / `LASER_LPS 0.75` (double) leftovers.
- C++ static constructors never run. `_GLOBAL__sub_I_MyDevice_L1` exists (map :29491), so do not rely on the `LaserSensor_L1` constructor.
- `lib/` is upstream: fix the call sequence in `drivers/`, not in the ST library.
- Build gate on both sensors: `.claude/skills/pluto-build/driver.sh --gate PRIMUS_X2_v1`.

**Resource conflicts.** Both sensors use I2C1 at 0x29, 400 kHz, on PB8/PB9 (target.h:112-124, PIN_MAP:47-48), shared with the onboard sensors. They cannot both be fitted. With both defines set, both blocks run and the L1x block overwrites `_position_error_z` and `ToF_Height`. `XRanging` (VL53L0X `LaserSensor` objects, also 0x29 until readdressed) would collide with the L1x. No DMA or timer use.

**Already answered by the code**
- The defines are commented out in PRIMUS_X2_v1 target.h:183-185 and PRIMUS_V5 :185-187. PRIMUSX2 has no `LASER_TOF_L1x` line.
- Both drivers are always compiled: Makefile `DRONA_DRIVERS` :278-279, libs via `RANGING_SRC2` wildcards.
- L1x driver: full ST VL53L1 API (not the ULD), MEDIUM mode, limit checks sigma 18 mm / signal 0.25 MCPS (DataInit), valid only when `RangeStatus == 0 && mm < 4500`, no IIR (commented out at :139), output in mm, poll every 10 ms.
- L0X driver: single ranging, 33 ms budget, VCSEL 18/14, <2000 mm, 0.1 float IIR with reseed.
- No public API exposes the down-laser; `XVision` has no users, yet costs 964 B RAM in every build (map :32541).
- `opticflow.cpp:355` scales by `NewSensorRange` (the L0X value), so it reads 0 with L1x. `OPTIC_FLOW` is off.
- `command.cpp:116`: the `LASER_ALT` take-off sets an absolute height.
- Git history for the L1x driver: be31216 (Nov 2025, integration), later only refactors. There is no flight or bench record of it, and no L1x log in any topic.

**Questions for the user**
1. Has this VL53L1X ever returned changing distances on this firmware? This sets whether bug 2 is known to be real.
2. Board wiring: the same I2C1 connector as the VL53L0X, with XSHUT/GPIO1 left unconnected? Is the L0X removed?
3. Distance mode (Short, Medium or Long) and sample rate: keep 100 ms, or ask for about 33-50 ms to match the L0X tuning?
4. What is the flying floor and lighting? These set the handover band after a bench reach log.
5. Target flying height: stay under about 1.5 m, or use the L1x's longer reach (which moves the band up)?
6. Should the validated VL53L0X flights (log-3/5/7 style: hover, handover, box) be re-run as regression, or is a proven object-code-identical L0X build enough?
7. Should the refactor be allowed to touch the L0X code path at all, or should it stay byte-identical?
8. Should the unused `XVision` / `LaserSensor_L1` class be removed (964 B RAM), or kept for a planned API?

**Diagnostics to reuse (TESTING.md:5-24, 78-88):** `EstAlt`, `AltHold`, `Vz`, `ToF` (read from the driver, -1 when out of range), `Src`, `Arm`, with `AccB`/`VzR`/`AccZ` optional, about 115 B per tick in `PlutoPilot.cpp` (temporary). For L1x, log `NewSensorRange_L1`/`Range_Status_L1` and, on the bench, `StreamCount` and `Global_Status_L1`.

**Doc drift:** the pipeline doc :129 and :292 match the code today; both will need rewriting after the fix. FLIGHT_INVARIANTS.md has no laser or flip text, so the planned move from CLAUDE.md has not been done.

**Prior work:** `active-development/vl53l1x-althold-parity/README.md` (Planned), `tof-althold-fusion/` (Closed, 484226c), be31216.

**Suggested skills:** pluto-rules, pluto-driver (fix to the L1x call sequence), pluto-build (`--gate`, both sensors), pluto-flighttest, pluto-log-analyst agent, pluto-reviewer, pluto-commit.

**Pipeline docs to update at the end**
- `Altitude_Hold_Estimator.md`: Source files (the L1x driver), the Laser fusion section applying to both sensors, a per-sensor constants table, the ToF vs Baro note.
- `CLAUDE.md` `LASER_ALT` paragraph: drop the "L1X has none of this" sentence and move the paragraph into `dev-guide/FLIGHT_INVARIANTS.md`, along with the flip paragraph.
- `PIN_MAP.md` / `HARDWARE_RESOURCES.md`: note that the down-laser uses 0x29 on I2C1 and that only one of the two sensors can be fitted.
- CHANGELOG. No `docs/API` change unless an API is added.
