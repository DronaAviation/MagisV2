# Changelog

All notable changes to MagisV2 firmware are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [v4.0.0] - 2026-07-21

Consolidates all work merged into `main` since `v3.0.0`. Highlights: a unified
non-blocking OLED subsystem, a WS2812B RGB LED API on a selectable data pin,
ExpressLRS (CRSF) receiver support with battery telemetry, a DMA channel
ownership registry, pilot override of user RC commands, and a large
driver/platform cleanup that removes all legacy STM32F10x support.

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
- **Compass**: Magnetometer calibration progress indicator.
- **Drivers**: DMA channel ownership registry (`dmaClaim`/`dmaRelease`/
  `dmaIsFree`/`dmaGetOwner`) enforcing DMA allocation at runtime.
- **PlutoPilot**: API hooks for receiver configuration and initialization.
- **Tooling**: Graphify toolchain integration for project architecture analysis
  and navigation reports.
- **Docs**: Hardware resource reference documentation (DMA/timer/pin maps) and
  firmware architecture pipeline docs.
- **Meta**: `embedded-systems` and `cpp-pro` agent definitions.

### Changed

- **Firmware version** bumped to 3.5.0 (API 1.3.1). The API patch bump reflects
  `RcCommand_Set`'s new pilot-override behaviour; no public signature changed,
  so existing projects compile and link untouched.
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
- **AltitudeHold**: Reset `errorVelocityI` on AltHold state change.
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
