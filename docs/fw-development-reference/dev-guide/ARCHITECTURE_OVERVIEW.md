# Architecture overview

Detail behind the architecture pointers in the root [`CLAUDE.md`](../../../CLAUDE.md). For per-subsystem
pipelines see [`../fw-architecture-pipeline/`](../fw-architecture-pipeline/).

## Boot and main loop

`src/main/main.cpp` does hardware/sensor init then enters the scheduler. The real-time control loop is
`loop()` in `src/main/mw.cpp` — it reads RX, runs IMU/attitude estimation (`imuUpdate`), PID, mixer,
and motor output every `looptime`, and dispatches periodic tasks (`executePeriodicTasks`) and GPS on
off-cycles. `mw.cpp`/`mw.h` is the integration hub tying sensors, flight, and IO together.

## User code lives only in `PlutoPilot.cpp`

The firmware calls these hooks:

- `plutoRxConfig()` — select receiver (ESP/ELRS-CRSF/PPM/CAM);
- `plutoInit()` — once at power-up;
- when Developer Mode is toggled: `onLoopStart()` → repeated `plutoLoop()` (invoked from `userCode()`
  in `mw.cpp`) → `onLoopFinish()`.

Everything `PlutoPilot.cpp` needs comes from `PlutoPilot.h`.

## Two-layer API (Drona additions on top of Cleanflight)

- `src/main/API/` — public headers exposed to user code (`FC-Data.h`, `FC-Control.h`, `Motor.h`,
  `Oled.h`, `Peripherals.h`, `Serial-IO.h`, `XRanging.h`, `Scheduler-Timer.h`, `RxConfig.h`,
  `Localisation.h`, `RGB-LED.h`, `Debugging.h`, …).
- `src/main/API-Src/` — their implementations, which wrap the internal Cleanflight subsystems. This API
  layer is the seam between user-facing calls and firmware internals; keep the public headers stable.

User-facing API reference wikis live in `docs/API/` (e.g. `OLED_API_WIKI.md`). When a public API
signature or behaviour changes in `API/`/`API-Src/`, update the matching wiki and bump
`FW_Version`/`API_Version` in the Makefile.

### WS2812B RGB LED API

`API/RGB-LED.h` drives an addressable strip from a **selectable data pin** —
`RGB_Init(RGB_1..RGB_8, led_count)` picks one of 8 vetted `{pin, timer channel, DMA channel}` slots
(LUT `ws2811HwTable[]` in `drivers/light_ws2811strip_stm32f30x.c`; PA15 is the default).
`RGB_Control(RGB_USER|RGB_SYSTEM)` toggles ownership between user code and a self-contained
flight-status indicator (`rgbSystemTick()` in `API-Src/RGB-LED.cpp`, called from `mw.cpp` every loop).
It does **not** use the Cleanflight `LED_STRIP` feature, which stays off because enabling it corrupts
config/BARO. Reference + per-pin conflict matrix: [`../WS2812_RGB.md`](../WS2812_RGB.md).

## Source layout under `src/main/` (Cleanflight heritage)

`drivers/` (MCU peripherals, IMU/ICM20948, baro/ICP10111, compass/AK09916, SPI/I2C, optical-flow
PAW3903, VL53L0X/L1X ToF), `flight/` (`pid`, `imu`, `mixer`, `altitudehold`, `navigation`, plus
Drona's `opticflow`/`posControl`/`posEstimate`/`acrobats`), `sensors/`, `rx/` (protocols incl.
`crsf.c` for ELRS + battery telemetry), `io/`, `telemetry/`, `blackbox/`, `command/`, `config/`,
`vcp/` (USB CDC), and `target/<TARGET>/` (board pin maps, feature `#define`s, linker scripts).

## Build groups

The Makefile composes the source set from named groups: `COMMON_SRC`, `MAIN_*` (Cleanflight core),
`DRONA_*` (`DRONA_FLIGHT`/`DRONA_DRIVERS`/`DRONA_COMMAND`/`DRONA_API` — the Pluto-specific additions),
and `PRIMUSX2_DRIVERS`. A new `.c`/`.cpp` module must be added to the appropriate group — there is no
glob over all sources for the firmware build. Feature compilation is gated by `#define`s in the target
header (`BARO`, `SONAR`, `GPS`, `UWB`, `ENABLE_ACROBAT`, `PRIMUSX2`, …); guard hardware-specific code
accordingly.

## Third-party code

Vendored under `lib/main/` (CMSIS, STM32F30x StdPeriph, USB-FS device driver, VL53L0X/VL53L1X APIs)
and `lib/test/` (GoogleTest). Treat as upstream — don't reformat.
