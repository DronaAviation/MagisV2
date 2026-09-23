# Build, flash and verify

Detail behind the build and verification rules in the root [`CLAUDE.md`](../../../CLAUDE.md).

In normal use the maintainer builds, cleans, selects targets and flashes (STM32 DFU/bootloader mode)
through the **PlutoIDE VS Code extension**, which wraps the Makefile and manages the toolchain. The
commands below are the underlying Make targets for working directly in the repo.

## Make targets

`TARGET` is required for every build. Valid targets: `PRIMUS_X2_v1` (default board), `PRIMUS_V5`,
`PRIMUSX2` (legacy). All three currently compile the same source set.

```bash
make TARGET=PRIMUS_X2_v1              # build firmware (.hex + .bin) and print flash/RAM usage
make TARGET=PRIMUS_X2_v1 clean        # remove build artifacts for that target
make TARGET=PRIMUS_X2_v1 memory       # flash/RAM usage bars from the linked ELF
make TARGET=PRIMUS_X2_v1 flash        # flash .hex over serial via stm32flash (triggers bootloader with 'R')
make TARGET=PRIMUS_X2_v1 st-flash     # flash .bin via st-flash (ST-Link)
make TARGET=PRIMUS_X2_v1 cppcheck     # legacy; NOT the analysis path — see below
make help                             # list documented targets
```

Build outputs go to `Build/<TARGET>/`. `SERIAL_DEVICE` defaults to the first `/dev/ttyUSB*`; override
it on the command line for flashing.

- `BUILD_TYPE=BIN` (default) builds the full firmware binary including `PlutoPilot.cpp`.
- `BUILD_TYPE=LIB` (`make ... libcreate`) builds a static `.a` library with user code excluded — the
  "Library" project mode, where user code links against a precompiled core.

## Compiler flags

Compilation is strict: `-Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Wdouble-promotion` on
both C (`gnu17`) and C++ (`gnu++17`). Hard-float single-precision FPU (`fpv4-sp-d16`) with
`-fsingle-precision-constant` (bare float literals are single), `-Os`, LTO disabled, `-nostartfiles`
(C++ static constructors never run). Expect warnings
to matter — avoid introducing implicit conversions.

## Tests: the GoogleTest suite does not build

The `src/test/` GoogleTest suite is unmaintained. Its Makefile still references `.c` sources that were
migrated to `.cpp`, so `make test` fails immediately
(`No rule to make target '../main/common/maths.c'`). It was never updated after the C++ migration and
is not part of any current workflow. The working verification path for a change is the firmware
**build**: confirm it compiles clean and fits in flash/RAM for the working target, and for all targets
before committing. Don't try to revive these tests unless explicitly asked.

## Warning gate: the compiler is the static analyser

A clean `PRIMUS_X2_v1` build emits ~2674 warnings, ~1712 of them from vendored `lib/` — so a warning
a change introduced is invisible unless it is compared against a baseline.

- `tools/warnings.py` does that (`baseline` / `check` / `summary`); the baseline is committed at
  `tools/warnings-baseline.json`.
- `.claude/skills/pluto-build/driver.sh --gate <TARGET>` runs it as part of the build and fails on any
  new warning under `src/`.
- Warnings in `lib/` never fail the gate, and the baseline is never regenerated to silence a newly
  introduced warning.

**`make cppcheck` is not the analysis path** — it covers only the 49 `.c` files (none of `flight/`,
`sensors/`, `API-Src/`), hardcodes Linux include paths, and needs a binary that isn't installed.

## Hardware validation by flight log

PlutoMonitor captures (`logs*.txt`) are analysed with `tools/flightlog.py` (`summary` / `table` /
`report`). The `pluto-flighttest` skill covers test plans, logging and how to read results; the
`pluto-log-analyst` agent runs it on large logs.
