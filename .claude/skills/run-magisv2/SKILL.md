---
name: run-magisv2
description: Build, compile-verify, and check the flash/RAM budget of MagisV2 Pluto drone firmware. Use when asked to run, build, compile, smoke-test, or verify MagisV2 / Pluto firmware, or to confirm a firmware change still builds and fits for all board targets (PRIMUS_X2_v1, PRIMUSX2, PRIMUS_V5).
---

# Run / build MagisV2 firmware

MagisV2 is **bare-metal ARM firmware** (STM32F303xC, Cortex-M4) for the
Pluto drone. There is no app to launch in this container — the firmware
only "runs" when flashed onto real hardware. The meaningful verification
for a change here is: **does it still cross-compile for every board
target, produce a flashable `.hex`, and fit in the 256 KB flash / 40 KB
RAM budget.** The driver does exactly that.

Paths below are relative to the repo root (the unit dir).

## Run (agent path) — the driver

```bash
.claude/skills/run-magisv2/driver.sh              # clean-build ALL targets, verify .hex + memory
.claude/skills/run-magisv2/driver.sh PRIMUS_X2_v1 # single target
.claude/skills/run-magisv2/driver.sh --no-clean   # incremental (faster, skips clean)
```

The driver puts the PlutoIDE ARM toolchain on `PATH`, builds each
target, asserts a `.hex` was produced, prints the flash/RAM usage bars,
and exits non-zero if any target fails. Expect `ALL BUILDS PASSED` and
exit 0. A full clean build of all three targets takes ~70s; a single
clean target ~23s.

Build outputs land in `Build/<TARGET>/<PROJECT>_<TARGET>_<FW_Version>.hex`
(e.g. `Build/PRIMUS_X2_v1/DEFAULT_PRIMUS_X2_v1_3.3.1.hex`) plus the
`.elf` (a `statically linked ARM EABI5` executable, not a host binary).

## Prerequisites

The `arm-none-eabi` GCC toolchain plus `make` and a `bash` shell. PlutoIDE
installs the toolchain under `<home>/.pluto-ide/tools/ARM GNU ToolChain/bin`;
the driver probes that location under both `$HOME` (Linux/macOS) and
`$USERPROFILE` (Windows) and prepends the first that exists, falling back to
whatever `arm-none-eabi-g++` is already on `PATH`. If it's missing, install
PlutoIDE or the upstream `gcc-arm-none-eabi` package.

**Platform / shell:** the driver is a bash script and runs natively on
**Linux and macOS**. On **Windows** run it from **Git Bash or WSL** — not
cmd.exe / PowerShell, which can't execute it. Only the Linux path is
verified here; macOS/Windows rely on the identical PlutoIDE layout plus the
PATH fallback. If your toolchain lives elsewhere, just put its `bin` on
`PATH` before invoking and the driver will use it.

## Build (manual, without the driver)

```bash
export PATH="$HOME/.pluto-ide/tools/ARM GNU ToolChain/bin:$PATH"
make TARGET=PRIMUS_X2_v1            # build (.hex) + print memory summary
make TARGET=PRIMUS_X2_v1 clean     # clean that target
make TARGET=PRIMUS_X2_v1 memory    # memory bars from the linked ELF
```

`TARGET` is mandatory and must be one of `PRIMUS_X2_v1`, `PRIMUSX2`,
`PRIMUS_V5` (the build errors out otherwise). All three currently
compile the same source set.

## How the firmware is normally built & flashed (human path)

In day-to-day use the maintainer does **not** drive the Makefile by hand.
Build, clean, target selection, and flashing are all managed by the
**PlutoIDE VS Code extension**, which wraps this same Makefile and the
toolchain. Flashing is done by putting the STM32 into **DFU/bootloader
mode** and using the extension's flash buttons (USB or Wi-Fi) — the
extension handles the bootloader trigger and driver.

The raw `make TARGET=... flash` (serial via `stm32flash`) and
`make ... st-flash` (ST-Link) targets exist but are not the normal
workflow and require a physically connected drone + `SERIAL_DEVICE`.
**None of this is runnable in a headless container** — it is the
human-on-hardware path. In the container, use the driver above to
compile-verify only.

## Gotchas

- **`src/test/` is dead — do not use it.** The host GoogleTest suite is
  unmaintained and not part of any current workflow. Its Makefile still
  references `.c` sources (`common/maths.c`, `sensors/battery.c`, …) that
  were migrated to `.cpp`, so `make test` / `cd src/test && make test`
  fails immediately with `No rule to make target '../main/common/maths.c'`.
  It was never updated after the C++ migration and there is no plan to
  run it. **Treat the firmware build as the only verification path** — do
  not try to fix, run, or extend these tests unless explicitly asked.
- **`TARGET` is required.** A bare `make` errors with "Target '' is not
  valid". Always pass `TARGET=`.
- **No `.bin` by default.** The default `all`/`binary` target produces
  only `.hex` (+ `.elf`). A `.bin` is generated only as a dependency of
  the `st-flash` target. Don't look for a `.bin` after a plain build.
- **Strict warnings are on by design** (`-Wconversion -Wsign-conversion
  -Wshadow -Wdouble-promotion`). The build currently succeeds with
  warnings; a change that turns a warning into an error (or that you're
  asked to keep warning-clean) needs attention to implicit conversions.
- **The `.elf` is an ARM binary, not host-executable.** `file` reports
  `ELF 32-bit ... ARM, EABI5`. Don't try to run it; it only executes on
  the STM32 target.

## Troubleshooting

- `arm-none-eabi-g++: command not found` → toolchain not on PATH; the
  driver handles this, or `export PATH="$HOME/.pluto-ide/tools/ARM GNU ToolChain/bin:$PATH"`.
- `Target '' is not valid, must be one of ...` → you omitted `TARGET=`.
- `No rule to make target '../main/common/maths.c'` → you ran the stale
  unit-test suite; see Gotchas. Use the firmware build instead.
</content>
