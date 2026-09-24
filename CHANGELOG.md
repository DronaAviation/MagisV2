# Changelog

All notable changes to MagisV2 firmware are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [v4.0.0] - 2026-07-21

Consolidates all work merged into `main` since `v3.0.0`. Highlights: a unified
non-blocking OLED subsystem, a WS2812B RGB LED API on a selectable data pin,
ExpressLRS (CRSF) receiver support with battery telemetry, a DMA channel
ownership registry, pilot override of user RC commands, barometer altitude hold
that no longer sinks as the board warms ( throttle and temperature compensation
plus four estimator / controller fixes ), and a large driver/platform cleanup
that removes all legacy STM32F10x support.

### Added

- **OLED**: Unified OLED subsystem with SYSTEM/USER ownership modes: non-blocking
  rendering API (`Oled.cpp`/`Oled.h`), framebuffer drawing primitives, and
  diff-based updates for efficient I2C communication. Ownership control prevents
  system/user rendering conflicts.
- **OLED**: Simple API layer with batched I2C, boxy "eyes", `DrawText`, bitmap
  drawing, and rectangle outlines.
- **OLED**: Inverted (highlighted) text rendering.
- **RGB LED**: WS2812B RGB LED public API with 21 built-in animations.
- **RGB LED**: Selectable data pin with user/system control (`RGB_Init`,
  `RGB_Control`), plus a self-contained flight-status indicator.
- **IO**: LED strip functionality integration.
- **RX**: ExpressLRS (CRSF) receiver support (initially USART2, moved to USART1)
  with acro/angle flight-mode switching.
- **RX**: CRSF battery telemetry: voltage, current, capacity, and remaining %
  (800 mAh default for ELRS).
- **RX**: THROTTLE / ALT HOLD switch on AUX5 with LED feedback in `plutoLoop`.
- **RC**: Pilot override of user RC commands. A channel driven by
  `RcCommand_Set` is now cross-faded against the pilot's own sticks by how far
  they are deflected, so the pilot can manoeuvre mid-manoeuvre instead of being
  locked out: sticks centred gives the commanded value unchanged, part
  deflection gives a proportional mix, and `USER_RC_STICK_TRAVEL` counts off
  centre gives the pilot sole authority. No API change - the behaviour is
  built into `RcCommand_Set`, so existing sketches gain it without edits.
- **RC**: Hold watchdog for user RC overrides. An override now expires after
  `max ( 250 ms, 2 x userLoopFrequency )` unless re-asserted, so a channel is
  handed back by simply no longer calling `RcCommand_Set`, rather than staying
  latched for the rest of the Developer Mode session.
- **RX**: `rcDataPilot []` - a snapshot of the pilot's four primary sticks taken
  in the RX layer before user code can write to `rcData`, giving the override
  path a source of pilot input that its own writes cannot contaminate.
- **Barometer**: Throttle and temperature compensation of the ICP-10111 reading
  (`sensors/barometer.cpp`). Sensed pressure falls by ~8.6 Pa per 1000 throttle
  counts (rotor inflow) and ~2.3 Pa per °C of die temperature. Both are corrected
  relative to the arm instant, so the correction cannot add an absolute offset,
  and the total is clamped (`BARO_COMP_LIMIT_PA`, 25 Pa). Temperature coefficient
  `BARO_COMP_TEMP_PA_PER_DEGC` is 2.1, set just below seven flight measurements
  (−2.17 to −2.42 Pa/°C) so any residual error sinks gently rather than climbs.
- **Barometer**: Ground zero is tracked while disarmed and frozen on arm
  (`getBaroZeroOffset`), so warm-up between boot and takeoff no longer offsets
  the flight.
- **AltitudeHold**: Hover-trim offload. The hover throttle trim is moved out of
  the clamped velocity integrator into the throttle baseline while the craft is
  settled, so the sustainable hover throttle is no longer capped at ~1800 as the
  battery sags. Builds on the `errorVelocityI` fix under Fixed: that fix keeps
  the trim through setpoint changes, this moves it where it cannot saturate.
- **Compass**: Magnetometer calibration progress indicator.
- **Drivers**: DMA channel ownership registry (`dmaClaim`/`dmaRelease`/
  `dmaIsFree`/`dmaGetOwner`) enforcing DMA allocation at runtime.
- **PlutoPilot**: API hooks for receiver configuration and initialization.
- **Tooling**: Graphify toolchain integration for project architecture analysis
  and navigation reports; `tools/graph_labels.py` gives graph communities
  readable names ( `area/file: symbol` ) after each AST-only refresh.
- **Tooling**: `tools/flightlog.py` - PlutoMonitor flight-log analysis
  (`summary`, `table`, `report`: pressure-vs-temperature fit, height hold against
  a laser, applied barometer correction and clamp headroom).
- **Docs**: Hardware resource reference documentation (DMA/timer/pin maps) and
  firmware architecture pipeline docs.
- **Meta**: Development agent definitions ( `embedded-systems`, `cpp-pro`,
  `c-pro`, and the read-only `flightlog-analyst` ) and skills: `run-magisv2`
  ( working target in development, all targets at commit ), `commit-magisv2`
  ( all-target build, version bump, doc promotion ), `flight-test` and
  `add-driver`. Trail of Bits analysis plugins enabled in `.claude/settings.json`.

- **AltitudeHold ( laser )**: With `LASER_ALT` and the VL53L0X ( `LASER_TOF` ), the
  laser hands over to the barometer above 160 cm and takes back below 140 cm; a
  dropout of 120 ms or a tilt above 25° also hands over, shorter gaps coast on the
  accelerometer. The barometer offset is a 2 s average of baro minus laser, frozen
  on the baro, and on the return the whole altitude frame shifts by the baro drift
  so the craft does not move.
- **AltitudeHold ( laser )**: Object under the craft. A change of surface ( the laser
  disagreeing with the accelerometer by more than 30 cm within 0.5 s, or more than
  20 cm building up within ~1 s ) holds the estimate on the baro for at least
  2.5 s; if the new surface is still there and steady, the estimate re-bases to it
  and the craft flies back to its old clearance at up to 60 cm/s up / 30 cm/s down
  ( the stick cancels ). A surface that comes back cancels it; slower changes are
  followed as terrain. On PRIMUS_X2_v1 every box put under or pulled out, slow
  slides and edges flown over included, started a hold-off with the height held
  within 6-13 cm ( once 16 cm ). There is no climb cap: an object held up under the craft lifts
  it each time.
- **AltitudeHold ( laser, VL53L1X )**: Return guard on the baro → laser return. In flight, a laser
  reading more than 50 cm from the current estimate is not taken as the floor unless the disagreement
  stays steady within 25 cm for 2.5 s ( a real new floor, such as a take-off from a table ). A VL53L1X
  looking down past a ceiling fan read the blades as a valid 56 cm for four samples at 2.5 m, which would
  have shifted the altitude frame by about −2 m.

### Changed

- **Firmware version** bumped to 3.10.0 (API 1.3.2) over this release: 3.5.0 for
  the RC pilot override, 3.6.0 for the landing fix, 3.7.0 for the barometer
  compensation and altitude-hold fixes, 3.8.0 for altitude-hold setpoint
  shaping, 3.8.1 for the flip fix under setpoint shaping, 3.9.0 for the laser
  ( VL53L0X ) altitude-hold fusion, 3.10.0 for the VL53L1X ( `LASER_TOF_L1x` )
  altitude-hold fusion. The API patch bumps reflect behaviour changes only: 1.3.1 for
  `RcCommand_Set`'s pilot override, 1.3.2 for Z setpoints
  (`DesiredPosition_set*`, take-off) now being flown at the bounded climb /
  descent rate. No public signature changed, so existing projects compile and
  link untouched.
- **Laser driver ( VL53L1X )**: 45 ms timing budget and 50 ms period in Medium mode ( was the ST
  default 41 ms / 100 ms ). The mode, budget and period can be overridden per target
  ( `L1X_DISTANCE_MODE`, `L1X_TIMING_BUDGET_US`, `L1X_SAMPLE_PERIOD_MS` ), with a compile-time check that
  the period is at least the budget + 5 ms. Each sample is fetched with one 17-byte result read and a
  1-byte interrupt clear, decoded with the ST API's own status mapping: 0.79 ms of blocking I2C per
  sample instead of 5.44 ms, which had stretched one 3.5 ms loop in about 15. Flash −2.1 KB.
- **AltitudeHold ( laser )**: One laser-fusion code path for both sensors. It reads the sensor through
  per-sensor accessors and constants; the VL53L0X object code is byte-identical to before. The build
  stops with an `#error` if both `LASER_TOF` and `LASER_TOF_L1x` are defined ( both are at I2C 0x29 ),
  or if `LASER_ALT` is defined without a laser.
  The shipped `target.h` keeps the laser defines off, so the default build is unchanged
  ( 98.9 KB / 14.8 KB ); a VL53L1X + `LASER_ALT` build is about 110 KB / 16.1 KB.
- **AltitudeHold**: Throttle stick moves the altitude setpoint ( ArduPilot / DJI
  style ) instead of switching the controller to raw velocity control. The
  climb rate scales from 0 at the dead-zone edge to 40 cm/s up / 30 cm/s down at
  full stick (`ALT_MAX_CLIMB_CMS` / `ALT_MAX_DESCENT_CMS`; was up to +120 /
  -100 cm/s), is ramped at 100 cm/s², and moves `AltHold` with the rate fed
  forward to the velocity loop. Centring the stick lets the target coast to a
  stop instead of snapping to `EstAlt`, removing the overshoot. A commanded
  altitude (take-off, `DesiredPosition_set*`, MSP) is no longer stepped: it
  becomes a goal, and the target travels to it on a trapezoidal profile
  (cruise `ALT_CMD_MAX_CLIMB_CMS` 60 / `ALT_CMD_MAX_DESCENT_CMS` 30 cm/s,
  braking at 80 cm/s² to stop on the goal). The 120 cm take-off takes about
  2.7 s instead of about 4.3 s with the old ±300 cm/s step, which had no slow
  final approach cut out. Moving the stick cancels a goal. `Command_Land` keeps its own descent profile
  (`( landThrottle - 1500 ) / 4`, about -50 to -87 cm/s, clamp
  `ALT_LAND_MAX_DESCENT_CMS`). Flown through the stick limits it came down
  at 10-20 cm/s, hovered just above the floor, and never detected touchdown.
- **AltitudeHold**: While disarmed or armed on the throttle stick with the
  motors held at idle, the altitude controller is held in reset, so waiting on
  the ground no longer winds the velocity integrator down and delays take-off.
- **RC**: `RcCommand_Set ( RC_THROTTLE, ... )` deflection is measured from where
  the throttle stick sat when the override latched rather than from mid-stick,
  since throttle does not self-centre - a stick resting at minimum can no longer
  fade an autonomous climb away.
- **BMS**: Improved current-measurement accuracy: current return now uses
  `mAmpWithGain` instead of `mAmpRaw`, the INA219 shunt resistor value corrected
  from 0.4 Ω to 0.02 Ω, and the current-calibration convergence rate
  (`CURR_CAL_ALPHA`) raised from 0.002 to 0.003 for improved responsiveness.
- **OLED**: Coordinate types widened to `int16_t` for extended range.
- **Sensor**: `Sensor_Get` return type changed from `uint32_t` to `int32_t`.
- **Failsafe**: Enhanced failsafe handling and refined crash detection.
- **OLED**: Startup display layout updated; improved readability; standardized
  header block and include order to match project convention.
- **Includes**: Reordered and optimized include statements across modules.
- **Altitude**: Sensor integration updated alongside driver cleanup.
- **Barometer**: Pressure-to-altitude conversion uses the ISA standard atmosphere
  at a fixed 288.15 K instead of scaling by the sensor's self-heating die
  temperature, making the altitude scale identical on every flight.
- **AltitudeHold**: Outer-loop gain `P8[PIDALT]` 100 → 128 (unity).
  `EEPROM_CONF_VERSION` bumped 106 → 107, so **saved settings reset to defaults on
  the first boot after flashing**.
- **AltitudeHold**: Altitude-ceiling stand-off named `ALT_CEILING_MARGIN_CM` and
  reduced from 50 to 25 cm.

### Fixed

- **RC**: Out-of-bounds write in `RcCommand_Set ( CHANNEL, value )`. Channels
  above `RC_THROTTLE` (`RC_AUX1`..`RC_USER3`, indices 4-10) indexed the
  4-element `RC_ARRAY` and `userRCflag` arrays, corrupting adjacent globals.
  Those channels have no override storage and are now rejected.
- **RC**: User RC overrides latched permanently. `userRCflag` was set by
  `RcCommand_Set` and only cleared on leaving Developer Mode, so a single call
  disabled that stick for the rest of the session; `resetUserRCflag()` was
  written to expire them but was never called. It is now wired into the control
  loop and reworked to a per-channel timestamp check, so an override that is
  still being asserted cannot be dropped mid-flight.
- **LED**: Clamped LED default-config `memcpy` that corrupted PID / alt-hold gains.
- **AltitudeHold**: Alt-hold collapsed whenever the throttle stick returned to
  centre after a climb. `errorVelocityI` is the only integrator in the alt-hold
  cascade (`PIDALT I8` is 0) and, because `initialThrottleHold` is pinned at
  1500, it carries the *entire* hover-throttle trim rather than a small
  correction. Zeroing it on a mid-flight setpoint change therefore stepped the
  motor command straight down to 1500, dropping the craft instead of capturing
  the new altitude. The reset now happens only on genuine transitions - BARO
  mode entry, disarm, and the explicit reset - where there is no trim to
  preserve.
- **AltitudeHold**: Landing disarmed in mid-air on larger airframes. Touchdown
  was inferred from the descent stopping, which cannot distinguish resting on
  the floor from being held up by ground effect - both give zero vertical
  velocity. A 110 mm rotor gains roughly 27 % thrust at 3 cm against about 5 %
  for a 55 mm one, enough to turn the fixed 1300 descent throttle into a hover
  throttle just off the ground, so the craft parked there and was disarmed a few
  centimetres up. The descent throttle is now bled down throughout the descent,
  and an arrest only counts as a landing once that ramp has passed below a
  throttle at which hovering is impossible. The ramp doubles as the probe that
  separates the two states. The impact threshold was also lowered from 2.08 G to
  about 1.46 G, which a gentle touchdown can actually reach.
- **AltitudeHold / Flip**: The app back-flip stopped rotating after altitude-hold
  setpoint shaping (see *Changed*). The app keeps ALT_HOLD on through AUX3, so the
  flip's full-throttle climb went through the shaped 40 cm/s stick limit and
  never reached the 100 cm/s the flip waits for; it timed out after 2.2 s. While a
  flip runs, altitude hold now flies the flip's throttle as a raw rate (up to
  120 cm/s, no ramp) as before shaping. When the flip ends, the setpoint is
  restarted at zero, the hover trim from before the flip is restored and held for
  500 ms while the exit climb is braked (it had wound up and caused a flyaway into
  the ceiling), and the drone flies back to the altitude it was holding when the
  flip was sent, on the take-off goal profile (the flip ends 40-120 cm high). The
  return is skipped for a flip sent on the ground, and moving the stick cancels
  it. Validated on PRIMUS_X2_v1 over 7 flips.
- **Barometer**: The ground reference could be re-zeroed in flight. Dropping the
  throttle stick to the bottom while armed called `baroResetGroundLevel`, so a
  throttle chop at 2 m left every later reading 2 m wrong. The reset is now
  limited to the ground.
- **AltitudeHold**: Estimator ran permanently in its slow (~15 s) mode. A tilt
  threshold meant as 30° was written in the wrong units as 3°.
- **AltitudeHold**: The position-error deadband subtracted 5 cm from every error
  instead of ignoring small ones, halving the loop's authority around hover.
  Removed.
- **ICP-10111**: Die temperature was frozen at its power-on average, so the
  sensor's own compensation ran on a stale temperature as the board warmed. It is
  now tracked with a low-pass filter.
- **ICP-10111**: OTP calibration constants were read as unsigned; they are signed
  16-bit.
- **RX**: Set `rc_connected` for serial RX.

- **AltitudeHold ( laser )**: Laser altitude hold bobbed ~30 cm at ~4 s. The
  estimator's vertical velocity read 0.3-0.4 × the real speed in a hover because
  the 40-count accelerometer Z deadband ( ~9.6 cm/s² ) removed most of a hover's
  vertical acceleration, leaving the velocity loop with a third of its damping.
  With `LASER_ALT` the estimator now uses no Z deadband ( a compile-time constant,
  so saved profiles are unaffected ), one 1.5 s time constant for both sources,
  and a tilt gate that works ( it compared radians with 25 and never rejected ).
  On PRIMUS_X2_v1 a 30 s hover holds within ±3 cm by the estimate, ±5 cm by the
  laser ( was 30 cm peak-to-peak ).
  Baro-only builds are unchanged.
- **Laser driver**: the VL53L0X IIR truncated to whole millimetres every update,
  so it stuck until the raw range was ≥ 10 mm above it; it now keeps float state
  and reseeds from the raw sample after a gap, a rejected tilted sample or a
  change of surface.
- **AltitudeHold ( laser, VL53L1X )**: With the VL53L1X ( `LASER_TOF_L1x` ) and `LASER_ALT`, the
  estimator kept correcting towards the last valid laser reading once the craft climbed past the
  sensor's reach ( ~1.9 m ): its branch used the laser while the height was between 0 and 350 cm and
  never checked for out of range, so the craft was pulled towards a stale height. Its tilt test also
  compared radians with 25 and never rejected, and its baro offset was a single sample. The VL53L1X now
  runs the same fusion as the VL53L0X: handover to the baro above 160 cm and back below 140 cm with a
  frozen offset and a frame shift on the return, a 185 ms dropout, tilt rejection above 25°, and the
  object hold-off with re-base. On PRIMUS_X2_v1, over four flights: a hands-off hover within ±3 cm 92 %
  of the time ( 8 cm peak to peak ), handovers with no height step, nine box hold-offs with re-base,
  baro hold within ±5 cm up to 3.1 m past the reach, and normal touchdowns.
- **Laser driver ( VL53L1X )**: The data-ready interrupt was never cleared, so every 10 ms poll re-read
  the same result and flagged it as new ( about 100 Hz of duplicates ). The driver now takes exactly one
  sample per measurement. A covered window reads 0-10 mm as a valid range; ranges under 15 mm now
  count as out of range ( the landed 25-28 mm stays valid, so landing is unchanged ). Out of range also
  covers a latched sensor error and a stall of more than 160 ms. A small negative range no longer wraps
  to about 65 m.

### Removed

- **Drivers/Platform**: Removed all legacy STM32F10x support (driver files,
  platform-specific code).
- **Drivers**: Removed obsolete accelerometer, gyroscope, and barometer drivers,
  and deprecated compass sensor drivers.
- **Sensors**: Removed sonar functionality.
- **PlutoPilot**: Removed OLED and RGB initialization code (now handled by the
  new subsystems).
- **Config**: Removed redundant SPDX identifiers and deprecated GPIO functions.

### Documentation

- Updated README for the OLED Simple API, ELRS (USART1, acro/angle modes,
  battery telemetry), and Primus X2 info.
- Converted the ELRS integration report from HTML to Markdown.
- Reorganized API documentation structure; streamlined and deprecated outdated
  hardware info.
- `run-magisv2`: clarified toolchain path setup for Windows.
- `docs/fw-development-reference/active-development/`: working docs for in-progress
  topics ( README / INVESTIGATION / CHANGES / TESTING / staged PIPELINE_UPDATE ),
  kept separate from `fw-architecture-pipeline/`, which now describes committed
  firmware only. First topic: `altitude-hold`.
- `fw-architecture-pipeline/subsystems/Altitude_Hold_Estimator.md`: barometer
  compensation chain added; function names and source files corrected
  ( `updateZVelocity`, `updateZPosition` and `calculateBaseThrottle` do not exist,
  and the Z estimate lives in `altitudehold.cpp`, not `posEstimate.cpp` ).
- `CLAUDE.md`: barometer compensation, `Monitor_Print` 250-byte budget, build-target
  policy ( working target in development, all targets at commit ), graphify usage.
- `fw-architecture-pipeline/subsystems/Altitude_Hold_Estimator.md`: new **Laser
  fusion** section ( handover, object hold-off, window test ) with its flowchart.
- `CLAUDE.md` and the `magisv2-rules`, `flight-test`, `add-driver` skills: keep
  `Monitor_Print` under ~130 bytes per tick with the app connected ( ~180 B
  disconnected the app ); laser altitude-hold summary.
- `fw-architecture-pipeline/subsystems/Altitude_Hold_Estimator.md`: **Laser fusion** covers both lasers,
  with the sensor accessors, a per-sensor constants table, the VL53L1X return guard, the VL53L1X driver
  and its known limits.
- `CLAUDE.md`: the flip and `LASER_ALT` paragraphs moved into `dev-guide/FLIGHT_INVARIANTS.md`, with one
  line each left in *Flight invariants*.
- `PIN_MAP.md`, `dev-guide/HARDWARE_RESOURCES.md`: one down-laser at 0x29 on I2C1; VL53L0X and VL53L1X
  are mutually exclusive.

## [v3.0.0] - 2026-02-10

Major release focused on new sensor drivers, ranging support, and reliability
of the barometer, IMU, and battery subsystems.

### Added

- **Ranging**: VL53L1X time-of-flight sensor support, plus `XRanging` laser
  functionality with object-avoidance.
- **Drivers**: SC18IS602B SPI bridge and PAW3903 optical-flow drivers.
- **Barometer**: Startup drift detection (`checkBaroDriftDuringStartup`) and a
  dedicated barometer-drift failure mode.
- **PWM**: `Servo_Write` function and revised PWM handling.
- **Battery**: Enhanced battery monitoring, state management, and configurable
  warning thresholds/payload.
- **Protocol**: Extended MSP API with versioning.
- **Build**: Makefile RAM-size configuration, memory-summary output, and a
  build-progress indicator.

### Changed

- **ADC**: Reworked ADC configuration and channel mapping.
- **Barometer**: Improved temperature-measurement precision.
- **System**: Optimized LED blink patterns and simplified error indication.
- **Battery**: Reworked configuration and serialization logic.
- **Drivers**: Improved I2C recovery and gyro initialization.
- **Build**: Reorganized Pluto file layout, firmware naming, config file
  locations, version handling, and LTO flags.

### Fixed

- **ICM20948**: Restore user bank 0 after configuration to prevent incorrect
  data reads (PX4-style bank switching).
- **ICP10111**: Corrected I2C ID and temperature handling.
- **Compass**: Fixed calibration logic and consistency checks.
- **Ranging**: Corrected include filename casing in `ranging_vl53l1x.h`.

## [v2.2.0] - 2025-09-19

Release centred on the public API surface: a consolidated flight-control /
sensor-data API and broad peripheral (GPIO/ADC/PWM/UART/I2C/SPI) support.

### Added

- **BMS**: Battery-management subsystem and public API; switched voltage/current
  sensing to the INA219.
- **API**: `FC-Data` sensor API (with consolidated velocity/position estimate),
  `FC-Control` + `RC-Interface` modules, and flight-command functions.
- **Scheduler**: Task scheduling functionality.
- **Peripherals**: GPIO mapping expansion, ADC restructuring, PWM support, UART
  (Serial-IO), I2C, and SPI integration.

### Changed

- **Flight control**: Modularized PID and setpoint management.
- **Debugging**: Migrated and consolidated debugging code.
- **API**: Standardized function naming and refactored the directory structure.

### Fixed

- **BMS**: Corrected spelling and voltage return value.
- **FC-Control**: Implemented `FlightStatus_Check` to resolve a linker error
  (#36).

## [v2.0.0] - 2025-08-05

Major release adding multi-target support, a Kalman-filtered altitude hold, and
a motor/GPIO API refactor.

### Added

- **Targets**: Support for `PRIMUS_V5` and `PRIMUS_X2_v1` boards.
- **Altitude**: Kalman filtering for altitude-hold stability, plus ground-level
  reset and improved barometer measurement handling.
- **Motor**: Refactored motor API (relocated `Motor.cpp`) and Status-LED control.
- **GPIO**: Peripheral GPIO API for pin management.
- **API**: Camera enable/disable API (RxConfig) and a new ADC port integration.
- **CI**: GitHub Actions workflow to publish firmware metadata / release info.

### Changed

- **Build**: Makefile rework, cleanup, and optimization (#29); build-artifact
  directory restructure; fork renamed to MagisV2.
- **Toolchain**: VL53L0X compatibility fixes for ARM-GCC 14.2 (#31).

## [v1.1.2] - 2025-04-20

Initial tagged baseline of the MagisV2 fork from the PrimusX2 codebase.

### Added

- Base PrimusX2 flight code with OLED integration.
- MagisV2 project rename and ESP GPIO-control refactor.

### Changed

- Improved altitude-hold logic and barometer handling.
- Standardized version identifiers for consistency.

### Fixed

- **Motor**: Resolved DRV8212 sleep/wakeup issue: arm-motor boost (1650) for
  100 ms then idle (1100).

[v4.0.0]: https://github.com/DronaAviation/MagisV2/compare/v3.0.0...v4.0.0
[v3.0.0]: https://github.com/DronaAviation/MagisV2/compare/v2.2.0...v3.0.0
[v2.2.0]: https://github.com/DronaAviation/MagisV2/compare/v2.0.0...v2.2.0
[v2.0.0]: https://github.com/DronaAviation/MagisV2/compare/v1.1.2...v2.0.0
[v1.1.2]: https://github.com/DronaAviation/MagisV2/releases/tag/v1.1.2
