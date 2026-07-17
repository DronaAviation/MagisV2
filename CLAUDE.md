# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

MagisV2 is bare-metal flight-controller firmware for the Pluto Drone family (Drona Aviation), forked from the Baseflight/Cleanflight stack. It targets STM32F303xC (Cortex-M4, 72 MHz, 256 KB flash / 40 KB RAM) and is cross-compiled with the `arm-none-eabi` GCC toolchain. On top of the Cleanflight core it adds a user-facing C++ API so developers write flight behaviour in `PlutoPilot.cpp` without touching internals.

In normal use the firmware is built and flashed through the **PlutoIDE VS Code extension**, which manages the toolchain. The commands below are the underlying Make targets for working directly in the repo.

## Build & flash

`TARGET` is required for every build. Valid targets: `PRIMUS_X2_v1` (default board), `PRIMUS_V5`, `PRIMUSX2` (legacy). All three currently compile the same source set.

```bash
make TARGET=PRIMUS_X2_v1              # build firmware (.hex + .bin) and print flash/RAM usage
make TARGET=PRIMUS_X2_v1 clean        # remove build artifacts for that target
make TARGET=PRIMUS_X2_v1 memory       # flash/RAM usage bars from the linked ELF
make TARGET=PRIMUS_X2_v1 flash        # flash .hex over serial via stm32flash (triggers bootloader with 'R')
make TARGET=PRIMUS_X2_v1 st-flash     # flash .bin via st-flash (ST-Link)
make TARGET=PRIMUS_X2_v1 cppcheck     # static analysis over all C sources
make help                             # list documented targets
```

Build outputs go to `Build/<TARGET>/`. `SERIAL_DEVICE` defaults to the first `/dev/ttyUSB*`; override on the command line for flashing.

- `BUILD_TYPE=BIN` (default) builds the full firmware binary including `PlutoPilot.cpp`. `BUILD_TYPE=LIB` (`make ... libcreate`) builds a static `.a` library with user code excluded — this is the "Library" project mode where user code links against a precompiled core.
- Compilation is strict: `-Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Wdouble-promotion` on both C (`gnu17`) and C++ (`gnu++17`). Hard-float (`fpv4-sp-d16`), `-Os`, LTO disabled. Expect warnings to matter — avoid introducing implicit conversions.

## Tests

**The `src/test/` GoogleTest suite is unmaintained and does not build — do not use it.** Its Makefile still references `.c` sources that were migrated to `.cpp`, so `make test` fails immediately (`No rule to make target '../main/common/maths.c'`). It was never updated after the C++ migration and is not part of any current workflow. The working verification path for a change is the firmware **build** (see above) — confirm it compiles for all targets and fits in flash/RAM. Don't try to revive these tests unless explicitly asked.

In normal use, the maintainer builds, cleans, selects targets, and flashes (STM32 DFU/bootloader mode) through the **PlutoIDE VS Code extension**, which wraps this Makefile and the toolchain.

## Architecture

**Boot and main loop.** `src/main/main.cpp` does hardware/sensor init then enters the scheduler. The real-time control loop is `loop()` in `src/main/mw.cpp` — it reads RX, runs IMU/attitude estimation (`imuUpdate`), PID, mixer, and motor output every `looptime`, and dispatches periodic tasks (`executePeriodicTasks`) and GPS on off-cycles. `mw.cpp`/`mw.h` is the integration hub tying sensors, flight, and IO together.

**User code lives only in `PlutoPilot.cpp`.** The firmware calls these hooks: `plutoRxConfig()` (select receiver — ESP/ELRS-CRSF/PPM/CAM), `plutoInit()` (once at power-up), and when Developer Mode is toggled: `onLoopStart()` → repeated `plutoLoop()` (invoked from `mw.cpp:1054`) → `onLoopFinish()`. Everything `PlutoPilot.cpp` needs comes from `PlutoPilot.h`.

**Two-layer API (Drona additions on top of Cleanflight).**
- `src/main/API/` — public headers exposed to user code (`FC-Data.h`, `FC-Control.h`, `Motor.h`, `Oled.h`, `Peripherals.h`, `Serial-IO.h`, `XRanging.h`, `Scheduler-Timer.h`, `RxConfig.h`, `Localisation.h`, `RGB-LED.h`, `Debugging.h`, …).
- `src/main/API-Src/` — their implementations, which wrap the internal Cleanflight subsystems. This API layer is the seam between user-facing calls and firmware internals; keep the public headers stable.

User-facing API reference wikis live in `docs/API/` (e.g. `OLED_API_WIKI.md`). When you change a public API signature or behaviour in `API/`/`API-Src/`, update the matching wiki, and bump `FW_Version`/`API_Version` in the Makefile to reflect the change.

The **WS2812B RGB LED API** (`API/RGB-LED.h`) drives an addressable strip from a **selectable data pin** — `RGB_Init(RGB_1..RGB_8, led_count)` picks one of 8 vetted `{pin, timer channel, DMA channel}` slots (LUT `ws2811HwTable[]` in `drivers/light_ws2811strip_stm32f30x.c`; PA15 is the default). `RGB_Control(RGB_USER|RGB_SYSTEM)` toggles ownership between user code and a self-contained flight-status indicator (`rgbSystemTick()` in `API-Src/RGB-LED.cpp`, called from `mw.cpp` every loop — it does **not** use the Cleanflight `LED_STRIP` feature, which stays off because enabling it corrupts config/BARO). Reference + per-pin conflict matrix: `docs/WS2812_RGB.md`.

**Source layout under `src/main/`** (Cleanflight heritage): `drivers/` (MCU peripherals, IMU/ICM20948, baro/ICP10111, compass/AK09916, SPI/I2C, optical-flow PAW3903, VL53L0X/L1X ToF), `flight/` (`pid`, `imu`, `mixer`, `altitudehold`, `navigation`, plus Drona's `opticflow`/`posControl`/`posEstimate`/`acrobats`), `sensors/`, `rx/` (protocols incl. `crsf.c` for ELRS + battery telemetry), `io/`, `telemetry/`, `blackbox/`, `command/`, `config/`, `vcp/` (USB CDC), and `target/<TARGET>/` (board pin maps, feature `#define`s, linker scripts).

**Build groups.** The Makefile composes the source set from named groups: `COMMON_SRC`, `MAIN_*` (Cleanflight core), `DRONA_*` (`DRONA_FLIGHT`/`DRONA_DRIVERS`/`DRONA_COMMAND`/`DRONA_API` — the Pluto-specific additions), and `PRIMUSX2_DRIVERS`. When adding a new `.c`/`.cpp` module you must add it to the appropriate group in the Makefile — there is no glob over all sources for the firmware build. Feature compilation is gated by `#define`s in the target header (`BARO`, `SONAR`, `GPS`, `UWB`, `ENABLE_ACROBAT`, `PRIMUSX2`, …); guard hardware-specific code accordingly.

**Third-party code** is vendored under `lib/main/` (CMSIS, STM32F30x StdPeriph, USB-FS device driver, VL53L0X/VL53L1X APIs) and `lib/test/` (GoogleTest). Treat as upstream — don't reformat.

## Conventions

- A `.clang-format` is present; match the existing style (notably the spaced-paren call style, e.g. `Oled_Text ( 0, 0, "..." )`). Files carry a banner header comment block with author/history — preserve it when editing.
- This is hard-real-time firmware on a 40 KB-RAM MCU: avoid dynamic allocation in the control path, keep `loop()` work bounded, and be mindful of flash/RAM budget (check with `make ... memory`).

## Knowledge graph

A prebuilt codebase knowledge graph is in `graphify-out/` (`graph.html`, `GRAPH_REPORT.md`). The `/graphify` skill answers questions against it — prefer it for "where is X / how does Y connect" exploration.

## Hardware resource reference

`docs/fw-development-reference/` is the standing reference for extending the firmware (target reference: `PRIMUS_X2_v1`). **Consult it before reasoning about DMA/timer/pin allocation, and keep it in sync with the code.**

- `DMA_MAP.md` — STM32F303xC DMA1/DMA2 channel ownership (driver-claimed vs ADC USER-API vs free) and which pins/peripherals can attach to free channels.
- `TIMER_MAP.md` — timer inventory: motors = TIM2 (all 4 channels), `TIM1_CH1`, WS2811 = `TIM8_CH1`, SysTick timebase; free = TIM6/TIM7/TIM16; the user `PWM_1..10` map.
- `PIN_MAP.md` — master per-physical-pin table tying GPIO/ADC/PWM/Serial + DMA + timer together, with the multiplexing conflicts (e.g. PB12–15 = ADC vs SPI2/M25P16 flash, PA8 `PWM_1` vs 5th motor output, PA15 `PWM_10` vs LED strip, PA13/PA14 = SWD debug).
- `datasheets/` — `rm0316-stm32f303xbcde.pdf` (RM0316: DMA request Tables 76/78) and `stm32f303vc.pdf` (DS9118: alternate-function Tables 14/15). Extract text with `pdftotext -layout`.
- `fw-architecture-pipeline/` — firmware architecture and per-subsystem pipeline docs.

DMA ownership is enforced at runtime by `drivers/dma_registry.{h,c}` (`dmaClaim`/`dmaRelease`/`dmaIsFree`/`dmaGetOwner`); ADC DMA is lazy (a channel is claimed only when a `Peripheral_Init(ADC_x)` pin on that ADC is used). When DMA/timer/pin assignments change in `Peripheral-ADC.cpp`, `Peripheral-PWM.cpp`, `Peripheral-GPIO.cpp`, `serial_uart_stm32f30x.c`, `light_ws2811strip_stm32f30x.c`, `timer.cpp`, or `target/<TARGET>/target.h`, update the affected map(s). Doc-to-source links use `../../src/main/...`.
</content>
</invoke>
