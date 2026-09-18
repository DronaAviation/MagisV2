---
name: add-driver
description: Checklist for adding or changing a sensor, peripheral driver or hardware feature in MagisV2 firmware - Makefile groups, target defines, init order, periodic task hooks, I2C/SPI bus sharing, DMA/ADC/timer ownership, and the reference maps to update. Use when adding a new driver (.c/.cpp under src/main/drivers or sensors), wiring a new I2C/SPI/ADC/UART device, enabling a hardware feature in target.h, or touching DMA, timer or ADC configuration.
---

# Adding a driver or peripheral to MagisV2

Work through the sections that apply. Each item is either something this
codebase requires or a mistake that has already cost a flight here.

## 1. Before writing code

- Read the working target's `src/main/target/<TARGET>/target.h` for the bus,
  pins and feature defines. Edit only the working target unless told otherwise.
- Check `docs/fw-development-reference/PIN_MAP.md`, `DMA_MAP.md` and
  `TIMER_MAP.md` for conflicts before choosing a pin, DMA channel or timer.
- Look for an existing driver of the same kind (`drivers/barometer_icp10111.cpp`,
  `drivers/ranging_vl53l0x.cpp`, `drivers/compass_ak09916.cpp`) and copy its
  structure, not a driver from another project. Upstream drivers (Cleanflight,
  INAV) assume a different `baro_t` / bus API and do not drop in.

## 2. Build wiring

- **Add every new `.c` / `.cpp` to a Makefile group by hand** - there is no glob:
  board-level drivers to `PRIMUSX2_DRIVERS`, Drona additions to `DRONA_DRIVERS`,
  sensor layers to `PRIMUSX2_SENSORS` / `MAIN_SENSOR`.
- **Gate it with a `USE_...` define** in `target.h` and wrap the whole source file
  and every call site in `#ifdef`. Other targets must still build.
- **C headers called from C++** need `#ifdef __cplusplus extern "C" { ... }` guards
  ( most of `main.cpp`, `mw.cpp` and `PlutoPilot.cpp` are C++ ).
- Strict flags (`-Wconversion -Wsign-conversion -Wshadow -Wdouble-promotion`):
  cast explicitly, use `f` suffixes, avoid left-shifting negative values
  (multiply instead).

## 3. Init and scheduling

Boot order in `main.cpp`: `ledInit` → `i2cInit` → `sensorsAutodetect*` →
`baroInit` / `baroCalibrate` → `timerStart` → `plutoInit ( )` ( user code,
runs **after** firmware init, so user peripheral setup can reconfigure yours ).

- **Init must never block indefinitely.** Bound every wait with a timeout and
  report failure; a stuck init shows up as a board with no LEDs and no app
  connection.
- **Periodic work** goes in the round-robin `executePeriodicTasks ( )` in
  `mw.cpp` (`UPDATE_BARO_TASK`, `UPDATE_LASER_TOF_TASK`, ...). One task runs per
  loop, so a new task slows every other periodic task; prefer hooking into an
  existing one and rate-limiting inside your update.
- **Keep updates non-blocking:** start a conversion in one call, collect it in a
  later one. The control loop is hard real-time.
- **No dynamic allocation.** 40 KB RAM; check with the build's memory bars.

## 4. Buses and shared hardware

- **I2C1 (PB8 SCL / PB9 SDA) is shared** by the ICP-10111 (0x63), VL53L0X (0x29),
  AK09916 and anything added. `i2cRead ( )` returns false on timeout and resets
  the peripheral; check the return value and keep the last good sample on failure.
  An unpowered or 5 V device on the bus can hold SDA/SCL low and stall every
  sensor at boot.
- **DMA:** claim channels through `drivers/dma_registry.h` (`dmaClaim`) before
  configuring them; check `dmaIsFree` if you only borrow a peripheral.
- **ADC (STM32F303):**
  - ADC1/ADC2 share one common register (clock mode, `TSEN`, `VREFEN`); touching
    it affects the other ADC. The user ADC API (`API-Src/Peripheral-ADC.cpp`)
    claims ADC DMA lazily and reconfigures the ADC from `plutoInit ( )`.
  - **The ADC has no clock out of reset** (`RCC_ADC12PLLCLK_OFF`): set
    `RCC_ADCCLKConfig` / the common clock mode *before* calibration, or
    calibration never finishes.
  - **The internal temperature sensor has a negative slope:** `TS_CAL2`
    (110 °C, `0x1FFFF7C2`) is smaller than `TS_CAL1` (30 °C, `0x1FFFF7B8`).
    Do not sanity-check their order.
- **Datasheets** are in `docs/fw-development-reference/datasheets/`; extract with
  `pdftotext -layout`. Watch for garbled units (µs extracts as a replacement
  character - 2.2 µs is not 2.2 ms).

## 5. Diagnostics while bringing it up

- Expose a **status code**, not just a valid flag, so a failed init tells you
  which step failed (e.g. log `-3` = bus claimed, `-4` = calibration timeout).
  Two flights were lost to a driver that only reported "-1".
- Log through `PlutoPilot.cpp` within the ~250-byte budget (see the
  `flight-test` skill).

## 6. Before it counts as done

- Build the working target with no new warnings (`run-magisv2`).
- Update `PIN_MAP.md` / `DMA_MAP.md` / `TIMER_MAP.md` if any pin, DMA channel,
  timer or ADC assignment changed (links use `../../src/main/...`).
- If user code can reach it: public header in `src/main/API/`, implementation in
  `API-Src/`, wiki in `docs/API/`, and an `API_Version` bump at commit.
- Record the change in the active-development topic's `CHANGES.md`.
- Temporary/diagnostic drivers: note "remove before release" in the topic README.
