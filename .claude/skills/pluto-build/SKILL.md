---
name: pluto-build
description: Build, compile-verify, and check the flash/RAM budget of MagisV2 Pluto drone firmware. Use when asked to run, build, compile, smoke-test, or verify MagisV2 / Pluto firmware, or to confirm a firmware change still builds and fits. During development builds only the session's working target (PRIMUS_V5 or PRIMUS_X2_v1, asked once per session); all targets (PRIMUS_X2_v1, PRIMUSX2, PRIMUS_V5) are built at commit time via pluto-commit.
allowed-tools:
  - Bash(.claude/skills/pluto-build/driver.sh *)
  - Bash(make TARGET=* memory)
  - Bash(python tools/warnings.py check *)
  - Bash(python tools/warnings.py summary *)
---

# Run / build MagisV2 firmware

MagisV2 is **bare-metal ARM firmware** (STM32F303xC, Cortex-M4) for the
Pluto drone. There is no app to launch in this container — the firmware
only "runs" when flashed onto real hardware. The meaningful verification
for a change here is: **does it cross-compile clean, produce a flashable
`.hex`, and fit in the 256 KB flash / 40 KB RAM budget** - for the working
target during development, for every target at commit. The driver does
exactly that.

Paths below are relative to the repo root (the unit dir).

## Which target to build

**During development and testing, build only the target being flown.** A
single-target build is about a third of the time of all three, and the
edit-build-flash loop is where the time goes.

- **Ask once per session**, the first time a build is needed: "Are you working
  on `PRIMUS_V5` or `PRIMUS_X2_v1`?" Offer the `selected_target` from
  `plutoide.ini` (the target selected in PlutoIDE) as the suggested answer.
  Then use that target for every build in the session without asking again,
  unless the user names a different one.
- **All targets** (`driver.sh` with no target) only when:
  - the user says they are committing, or asks for a version bump → use the
    `pluto-commit` skill, which does the full build;
  - the user explicitly asks for all targets;
  - a change touches something target-specific in a way the working target
    would not exercise (another target's `target.h`, a `#ifdef` for a define
    the working target does not set). Say why when doing this.
- When an edit touches a `target.h`, edit the working target's file; do not
  mirror it into the other targets unless asked.

## Run (agent path) — the driver

```bash
.claude/skills/pluto-build/driver.sh PRIMUS_V5    # development: the session's target
.claude/skills/pluto-build/driver.sh --no-clean PRIMUS_V5   # incremental, fastest
.claude/skills/pluto-build/driver.sh              # commit: clean-build ALL targets
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

The `arm-none-eabi` GCC toolchain plus `make` and a `bash` shell. The driver
probes `<home>/.pluto-ide/tools/ARM GNU ToolChain/bin` under both `$HOME`
(Linux/macOS) and `$USERPROFILE` (Windows), prepends the first that exists,
and otherwise falls back to whatever `arm-none-eabi-g++` is already on `PATH`.

PlutoIDE's toolchain location is **not the same on every platform**:

- **Linux / macOS** — under the home dir: `~/.pluto-ide/tools/ARM GNU ToolChain/bin`.
- **Windows** — the installer puts it at `C:\PlutoIDE\tools\ARM GNU ToolChain\bin`
  (a fixed root, *not* under the user profile). The `~/.pluto-ide/...` candidate
  the driver probes does **not** exist on Windows. Verified compiler:
  `C:\PlutoIDE\tools\ARM GNU ToolChain\bin\arm-none-eabi-g++.exe`,
  Arm GNU Toolchain 14.2.Rel1 (14.2.1).

**Important — how PlutoIDE exposes the toolchain on Windows:** the extension
does **NOT** add the toolchain to the persistent system/user PATH. Instead it
spawns its own **terminal with the PATH set for that session only** and runs
build/clean inside it; when that terminal closes the PATH is gone. So a fresh
Git Bash / WSL shell (where this driver runs) will **not** have
`arm-none-eabi-g++` on PATH by default, and the driver's `~/.pluto-ide`
probe won't match either — the build will fail with `command not found` unless
the toolchain `bin` is put on PATH first. Either add it for the session before
invoking the driver (see Troubleshooting), or persist it to your environment
manually. Do not assume the extension has set a system-wide PATH.

If the toolchain is missing entirely, install PlutoIDE or the upstream
`gcc-arm-none-eabi` package. If yours lives somewhere else, just put its `bin`
on `PATH` before invoking and the fallback will use it.

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

## Pre-flight gate — did the change add warnings?

A clean build emits **~2700 warnings**, ~1700 of them from vendored `lib/`
(CMSIS, StdPeriph, VL53L0X) that we do not own and never fix. A warning your
change introduced is invisible in that noise, and the dominant classes —
`-Wconversion` (1149) and `-Wsign-conversion` (903) — are exactly the ones that
produce scale and sign bugs in this tree.

`tools/warnings.py` records a baseline and reports only what a build **added**
over it. Run it before flying a build:

```bash
.claude/skills/pluto-build/driver.sh --gate PRIMUS_X2_v1
```

The driver keeps each build's full output at `Build/<TARGET>/build.log`, runs
the check, and fails the run if anything new appeared under `src/`. By hand:

```bash
make TARGET=PRIMUS_X2_v1 clean && make TARGET=PRIMUS_X2_v1 2>&1 | tee build.log
python tools/warnings.py check   build.log    # exit 1 = new warnings in src/
python tools/warnings.py summary build.log    # counts by flag and file
```

Rules for the gate:

- **New warnings under `src/` fail the gate.** Fix them, don't hide them.
- **New warnings under `lib/` never fail it** — that tree is upstream. They are
  listed only with `-v`.
- **Never run `warnings.py baseline` to silence a warning you introduced.**
  Regenerate the baseline only when warnings were deliberately *removed*, or
  after a toolchain change, and say so in the commit.
- The baseline lives at `tools/warnings-baseline.json` and is committed.
  It was recorded on `PRIMUS_X2_v1` with Arm GNU Toolchain 14.2.1.

**`make cppcheck` is not the analysis path here.** It scans only the 49 `.c`
files (a third of the tree — none of `flight/`, `sensors/` or `API-Src/`, which
are `.cpp`), hardcodes Linux include paths (`--platform=unix64`,
`-I/usr/include`), and needs a `cppcheck` binary that is not installed. The
compiler already analyses 100% of the tree with stricter flags; use the gate.

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

- **Board looks dead after flashing** (no LEDs, app says not connected) → almost
  always the STM32 is still in its DFU bootloader. Remove **all** power (battery
  and USB), wait a few seconds, power up again. Only if it stays dead: check the
  I2C bus (an unpowered or 5 V device on PB8/PB9 can hold it low and stall boot in
  barometer calibration for 35 s+ with no LEDs), then re-flash. Remember
  `plutoLoop ( )` only runs in Developer Mode with a live RC link, so user-code
  changes cannot stop the board from booting.
- `arm-none-eabi-g++: command not found` → toolchain not on PATH. On
  Linux/macOS the driver's `~/.pluto-ide` probe usually handles it, or:
  `export PATH="$HOME/.pluto-ide/tools/ARM GNU ToolChain/bin:$PATH"`.
  On **Windows** PlutoIDE only sets PATH inside its own spawned terminal (gone
  when that terminal closes) and does **not** persist it system-wide, so a
  fresh Git Bash shell won't find the compiler — add it for the session first:
  `export PATH="/c/PlutoIDE/tools/ARM GNU ToolChain/bin:$PATH"`.
- `Target '' is not valid, must be one of ...` → you omitted `TARGET=`.
- `No rule to make target '../main/common/maths.c'` → you ran the stale
  unit-test suite; see Gotchas. Use the firmware build instead.
