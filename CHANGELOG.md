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

### Changed

- **Firmware version** bumped to 3.8.1 (API 1.3.2) over this release: 3.5.0 for
  the RC pilot override, 3.6.0 for the landing fix, 3.7.0 for the barometer
  compensation and altitude-hold fixes, 3.8.0 for altitude-hold setpoint
  shaping, 3.8.1 for the flip fix under setpoint shaping. The API patch bumps reflect behaviour changes only: 1.3.1 for
  `RcCommand_Set`'s pilot override, 1.3.2 for Z setpoints
  (`DesiredPosition_set*`, take-off) now being flown at the bounded climb /
  descent rate. No public signature changed, so existing projects compile and
  link untouched.
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
