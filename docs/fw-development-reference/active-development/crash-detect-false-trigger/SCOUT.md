# Crash Detect False Trigger - Scout brief

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md)

`pluto-scout` brief from planning ( 8 Oct 2026 ), verbatim. Planned as `check`; the user then asked for the
15% impact-threshold change up front, so the topic runs as `fix`.

---

## Scout brief: "Reduce crash detection by 5-10%": stop false crash-disarms on PRIMUS_X2_v1 (CHECK mode)

Graph: stale (built 369ea71, HEAD cbd1c72). `failsafe.cpp` has not changed since the build, so the graph is still accurate for this area.

**Subsystems:** Failsafe (crash detector), IMU/accel → pipeline docs: `docs/fw-development-reference/fw-architecture-pipeline/subsystems/Failsafe_Subsystem.md`, `Error_Failure_Mode.md`, `IMU_Sensor_Fusion_Pipeline.md`

**Key code:**
- `src/main/flight/failsafe.cpp:362` `failsafeOnCrash` is the only crash detector.
- `src/main/mw.cpp:1424` calls it on every control loop (`looptime` = 3500 us, `config.cpp:518`).
- `src/main/mw.cpp:484-496` clears the Crash flag about 300 ms after disarm, once the craft is level (`SMALL_ANGLE`).
- `src/main/flight/imu.cpp:1210` `accSmooth` = IIR on `accADC`, `acc_lpf_factor` = 8 (`config.cpp:531`).
- `src/main/drivers/accgyro_icm20948.cpp:76,126`: `acc_1G` = 4096 (±8 g); `ACCEL_CONFIG` = 0x35 (chip DLPF cfg 6, about 5.7 Hz).
- `src/main/API-Src/FC-Config-Setpoints.cpp:173,192`: `Failsafe_enable/disable(CRASH)` sets `fsCrash` (default true, `API-Utils.cpp:123`).

**Mode:** check

**Current behaviour:** When the craft is ARMED, `fsCrash` is set and it is in ANGLE_MODE, a single loop sample with any of these six conditions true calls `set_FSI(Crash)` and `mwDisarm()` at once:
- |roll| or |pitch| > 700 deci-degrees (70°)
- |accSmooth X/Y| > 1250 LSB (0.305 g)
- |accADC X/Y| > 1750 LSB (0.427 g)

There is no debounce. The detector is bypassed while `flipState >= 1` (ENABLE_ACROBAT). In ACRO mode there is no crash detection at all. The app sees the Crash status bit for about 0.3 s. PRIMUS_X2_v1 and PRIMUS_V5 use the same ICM20948 and the same scaling, so the per-target-scaling idea is ruled out.

**Analysis: ranked hypotheses for false triggers**
1. **The raw `accADC` 0.43 g rule trips on sharp stick moves or attitude transients.** This is the most likely cause.
   - For: commit 49a2fdb (5 Jun 2026) tightened the rule about 10x. Before it, the only accel rule was `accSmooth > 12000` (about 2.9 g); it added `accSmooth 1250` and `accADC 1750`. A single 3.5 ms sample is enough to trip it. The IMU sitting away from the centre of gravity reads angular acceleration as lateral g.
   - Against: the chip's 5.7 Hz DLPF removes motor vibration, so this is not a vibration problem.
   - Test: log which condition fires and its peak value during aggressive ANGLE-mode stick inputs.
2. **Gravity on a tilted craft that is touching or resting on something.** On the ground, accX = g·sinθ. The `accSmooth` rule trips at 17.8° of tilt and the `accADC` rule at 25.3°. This covers take-off from uneven ground, bumps and touch-and-go. In free flight thrust is along body Z, so these rules see almost nothing.
   - Test: arm on a 20° slope at low throttle.
3. **Accelerometer calibration bias eats the margin.** Calibrating on a surface 3° off level puts about 214 LSB (17% of 1250) on X or Y.
   - Test: disarmed and level on the bench, read `Sensor_Get(Accelerometer, X/Y)`, which returns cm/s² (`accSmooth` × 0.2394). Readings above about 50 cm/s² support this hypothesis.
4. **The 70° angle rule.** Unlikely: the commanded angle limit is 20° (`max_angle_inclination` = 200, `config.cpp:454`, whose comment wrongly says 50°). Only a real upset reaches 70°. Flip exit is out of scope.

**Not proven:** whether today's triggers are false at all, and which of the six terms fires. The first task must log `accSmooth` and `accADC` X/Y, roll/pitch and the firing term at trip time. `accADC` is not reachable through the API.

**5% / 10% loosening of each knob:**

| Knob | Today | +5% | +10% |
|---|---|---|---|
| Angle | 700 (70°) | 735 (73.5°) | 770 (77°) |
| `accSmooth` | 1250 (0.305 g, 299 cm/s²) | 1313 (314 cm/s²; tilt-equivalent 18.7°) | 1375 (329 cm/s²; 19.6°) |
| `accADC` | 1750 (0.427 g) | 1838 (0.449 g; 26.6°) | 1925 (0.470 g; 28.0°) |

A percentage change only helps if the real peaks sit within 5-10% of a threshold. Because the six conditions are ORed, loosening a knob that is not the one firing changes nothing. The option that is not a percentage is persistence: require N consecutive 3.5 ms loops.

**Constraints / invariants:**
- Landing must not rely on stick limits. The crash detector is one of the ways a touchdown currently ends (battery-soc-fix log-6, accepted). Loosening it shifts touchdowns to `LAND`'s own rules (`LAND_IMPACT_ACC` 6000, 300 ms settle), and landing is out of scope.
- Keep `Monitor_Print` under about 130 B per tick.
- Flip bypass stays as it is (flip-althold-regression TASKS.md:152).
- Loosening thresholds risks a real low-speed impact going undetected, with the motors still spinning on the ground.

**Resource conflicts:** none.

**Already answered by the code:**
- Thresholds are compile-time literals, not CLI or config values.
- The only API knob is on/off: `Failsafe_enable/disable(CRASH)`.
- When the detector fires, the drone disarms; it does not land.
- Re-arming is allowed once the craft is level, and chuck-arm is blocked while the Crash flag is set (`rc_controls.cpp:183`).
- Side finding, outside this topic's scope: `FlightStatus_Check(FS_CRASHED)` maps to internal bit 4 (`LowBattery_inFlight`). Crash is bit 2, which the API calls `FS_LOW_BATTERY` (`FC-Control.h:30` vs `runtime_config.h:92`).

**Questions for the user:**
1. In which situations does it trip: aggressive sticks, take-off, bumping into things, or touchdown?
2. Do touchdown trips count as false triggers, given that landing is out of scope?
3. For the log, may the test add temporary instrumentation (`Sensor_Get` lines in PlutoPilot.cpp, or a debug field showing the firing term and `accADC`)? Is a reverted debug build acceptable?
4. Is a persistence (debounce) recommendation acceptable, or only a 5-10% threshold change?
5. When and on what surface was the accelerometer last calibrated?

**Doc drift:**
- `Failsafe_Subsystem.md` does not describe `failsafeOnCrash` at all.
- `Error_Failure_Mode.md:4` says a hard crash causes a halt that only a power cycle clears. In the code, the crash detector only disarms and the flag clears about 300 ms later (`mw.cpp:484-496`).

**Prior work:**
- Commit 49a2fdb (threshold change), mentioned at `CHANGELOG.md:183`.
- battery-soc-fix: touchdown crash-detect accepted (TASKS.md:493, TESTING.md:492-495).
- flip-althold-regression: a real crash was correctly detected after a flyaway (TASKS.md:159).
- No dedicated topic exists.

**Suggested skills:** pluto-flighttest, pluto-log-analyst, pluto-rules, pluto-build

**Pipeline docs to update at the end:** for a check topic, a findings note only. If a fix follows, `Failsafe_Subsystem.md` needs a crash-detector section (conditions, thresholds, ANGLE gate, flip bypass, flag clear) and `Error_Failure_Mode.md:4` needs correcting.
