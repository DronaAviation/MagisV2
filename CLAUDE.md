# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.
Detail lives in per-topic docs under `docs/fw-development-reference/dev-guide/`; read the linked doc
before working in that area.

## What this is

MagisV2 is bare-metal flight-controller firmware for the Pluto Drone family (Drona Aviation), forked
from the Baseflight/Cleanflight stack. It targets STM32F303xC (Cortex-M4, 72 MHz, 256 KB flash /
40 KB RAM), cross-compiled with `arm-none-eabi` GCC. On top of the Cleanflight core it adds a
user-facing C++ API so developers write flight behaviour in `PlutoPilot.cpp` without touching internals.

In normal use the maintainer builds, cleans, selects targets and flashes through the **PlutoIDE VS
Code extension**, which wraps the Makefile and manages the toolchain.

## Build

- `TARGET` is required for every build: `PRIMUS_X2_v1` (default board), `PRIMUS_V5`, `PRIMUSX2`
  (legacy). `make TARGET=<T>` builds; `make TARGET=<T> memory` shows flash/RAM; `make help` lists targets.
- **Which target to build.** During development and testing, build only the target being flown
  (`PRIMUS_V5` or `PRIMUS_X2_v1`): ask once per session, suggesting `selected_target` from
  `plutoide.ini`, then keep using it. Build all targets only at commit time.
- **Committing.** When the user says they are committing or asks to bump the Makefile version, use the
  `pluto-commit` skill: all-target build, version bump, promotion of the `active-development/` docs
  into the pipeline docs and `CHANGELOG.md`, then staging and a drafted message (template:
  `.claude/skills/pluto-commit/COMMIT_TEMPLATE.md`). **Never run `git commit` or `git push`** — the
  user makes every commit.
- Compilation is strict (`-Wconversion -Wsign-conversion -Wshadow -Wdouble-promotion`, C `gnu17` /
  C++ `gnu++17`, `-Os`): avoid introducing implicit conversions.
- **Single-precision FPU** (`fpv4-sp-d16`): `double` runs in software, so use `sqrtf`/`fabsf`/`atan2f`,
  never `sqrt`/`fabs`/`atan2`. Prefer the float helpers in `common/maths.h`: `sin_approx`/`cos_approx`,
  `constrainf`, `safe_asin`, `M_PIf` (not `M_PI`, which `flight/pid.h` redefines). C++ static
  constructors never run (`-nostartfiles`). ISR, stack and critical-section rules: `pluto-rules` (MCU safety).

Make targets, `BUILD_TYPE=BIN|LIB`, flashing, full flag list: `docs/fw-development-reference/dev-guide/BUILD_AND_VERIFY.md`.

## Verification

- **The `src/test/` GoogleTest suite does not build — do not use it or try to revive it** unless
  explicitly asked. The verification path is the firmware build: compiles clean and fits in
  flash/RAM for the working target, and for all targets before committing.
- **New warnings must be diffed against the baseline.** Run
  `.claude/skills/pluto-build/driver.sh --gate <TARGET>`: it fails on any new warning under `src/`
  (`tools/warnings.py`, baseline `tools/warnings-baseline.json`). `lib/` warnings never fail the gate.
  **Never regenerate the baseline to silence a newly introduced warning.**
- **`make cppcheck` is not the analysis path** (covers only 49 `.c` files, needs a missing binary).
- **Hardware validation is by flight log**: `tools/flightlog.py`, the `pluto-flighttest` skill, and the
  `pluto-log-analyst` agent for large logs.

Why and how: `docs/fw-development-reference/dev-guide/BUILD_AND_VERIFY.md`.

## Workflow

**Plan before working: `/pluto-grill <request>`, and start it yourself when the request isn't typed
as a command.** Modes: `check` (investigate, no code change), `fix`, `feature` (design options first),
`improve` (no behaviour change), `docs`. `/pluto-grill-deep` is the same on Fable 5.1 — only the user
starts it; subagents never use Fable. When the user describes an issue or a change in plain words ("I'm facing…", "X isn't
working", "I want to change…") and no active topic's `TASKS.md` covers it, invoke `pluto-grill`
before reading or editing code. Skip it for pure questions, trivial one-line edits, or when the user
says "no grill" / "just do it". The skill interviews the user (the `pluto-scout` agent does the
graphify/code/doc reconnaissance in its own context), then writes `active-development/<topic>/TASKS.md`:
numbered tasks, each routed to a project skill, ending with the closing tasks in `pluto-grill` §7 (build
gate → hardware validation → staged `PIPELINE_UPDATE.md` → topic review → graph refresh & commit;
`check` topics end with a findings task). Then `/pluto-task next` works it one task per
turn (`status`, `resume` in a new session, `done <n>`, `add <text>`).

**Code review always runs in the `pluto-reviewer` subagent** — any review of a diff, branch, commit
range, PR or task, whether asked for or done as part of a skill. Launch the agent and relay its
findings; never run the review in the main conversation.

**Firmware coding rules and the review checklist live in the `pluto-rules` skill.** Use it both when
writing firmware and when reviewing a diff; generic C/C++ review tools do not know any of it.

**Which tool when** (use these; do not reach for a generic tool that overlaps one of them):

| Task | Use |
| --- | --- |
| Any new issue, change or bug | `pluto-grill` skill (it runs the `pluto-scout` agent) |
| Carrying out a planned topic | `pluto-task` skill (`next`, `status`, `resume`) |
| Writing or changing `src/main` code | `pluto-rules` skill, before and while editing |
| Build, compile check, flash/RAM fit | `pluto-build` skill (`--gate` for the warning diff) |
| Review of any diff / task / branch | `pluto-reviewer` agent — never inline, never `c-review`/`sharp-edges`/`code-simplifier` |
| Flight or bench log (`logs*.txt`), test plan | `pluto-flighttest` skill; `pluto-log-analyst` agent for large logs |
| New sensor, peripheral, DMA/timer/pin change | `pluto-driver` skill |
| "Where is X / how does Y connect" | `graphify` (graph first, then grep) |
| Commit, version bump, closing a topic | `pluto-commit` skill |
| Delegating a self-contained implementation | `c-pro` agent for `.c` drivers, `cpp-pro` agent for `.cpp` |
| Code checked against a pipeline doc or datasheet | `spec-to-code-compliance` skill |
| Security audit of input parsers (MSP, CRSF, serial) | `/c-review:c-review` — only when the user asks |

Not used here: `dimensional-analysis` (units are checked by `pluto-rules` and the reviewer), `sharp-edges`, `claude-md-management`.

Tooling backlog: `.claude/TOOLING_BACKLOG.md`.

## Architecture

- **Main loop:** `loop()` in `src/main/mw.cpp` (RX → IMU → PID → mixer → motors every `looptime`);
  `mw.cpp` is the integration hub.
- **User code lives only in `PlutoPilot.cpp`** (hooks `plutoRxConfig`, `plutoInit`, `onLoopStart`,
  `plutoLoop`, `onLoopFinish`; everything it needs comes from `PlutoPilot.h`).
- **Two-layer API:** public headers in `src/main/API/`, implementations in `src/main/API-Src/`. Keep the
  public headers stable. When a public API signature or behaviour changes, update the matching wiki in
  `docs/API/` and bump `FW_Version`/`API_Version` in the Makefile.
- **Every new `.c`/`.cpp` module must be added to its Makefile group** (`COMMON_SRC`, `MAIN_*`,
  `DRONA_*`, `PRIMUSX2_DRIVERS`) — there is no source glob. Gate hardware-specific code on the target
  header `#define`s (`BARO`, `SONAR`, `GPS`, `UWB`, `ENABLE_ACROBAT`, `PRIMUSX2`, …).
- **Vendored code under `lib/` is upstream — don't reformat it.**
- **RGB LED:** `API/RGB-LED.h` with a selectable data pin; the Cleanflight `LED_STRIP` feature stays
  off (enabling it corrupts config/BARO).
- **OLED:** I2C is polled and shared with the IMU, so the OLED never writes the panel byte by byte from the loop
  (that caused a 5 Hz twitch). It draws into a RAM framebuffer, and `oledFlushStep ( )` sends at most 16 changed
  bytes per loop. `Oled_Update` / `Oled_display_Update` only queue the frame, so their buffers must be `static`.
  Never write the panel directly: the only exception is a full hardware clear followed by `oledPanelCleared ( )`
  inside `oled_display.c` (`active-development/oled-flight-wobble/`).

Hooks, API layout, source tree, build groups, RGB LED: `docs/fw-development-reference/dev-guide/ARCHITECTURE_OVERVIEW.md`.

## Flight invariants

Each of these has cost a flight. Full mechanism and numbers:
`docs/fw-development-reference/dev-guide/FLIGHT_INVARIANTS.md`.

- **Pilot input comes from `rcDataPilot[]`, never from `rcData` for throttle** — the user-override path
  writes `rcData`. User RC overrides (`RcCommand_Set`) cross-fade against the sticks and **expire unless
  re-asserted** (`max(250 ms, 2 × userLoopFrequency)`).
- **The baro ground reference freezes on arm and must never be re-zeroed in flight**
  (`throttleRaisedSinceArm` in `mw.cpp`); throttle and temperature compensation are relative to the arm instant.
- **In ALT_HOLD the throttle stick moves the setpoint; it never replaces the position loop.**
  **Landing must not go through the stick limits** — at slow descent the in-ground-effect baro drift
  masks the descent and touchdown is never detected.
- **Keep `Monitor_Print` under ~130 bytes per tick with the app connected** — `uartWrite()` does not
  check the 256-byte TX ring for full; an overrun corrupts the app's MSP replies and it disconnects.
  The double overload prints 0 for every digit after the first decimal.
- **During a flip, altitude hold is bypassed, not switched off** — BARO_MODE stays on and altitude hold
  flies the flip's raw throttle rate, then flies back to the pre-flip height (if ≥ 20 cm) as a goal
  (`FLIGHT_INVARIANTS.md`).
- **With `XRanging.initAlt ( )` (VL53L0X) or `X1Ranging.initAlt ( )` (VL53L1X), one down-laser corrects the
  estimator below its handover and the baro above it (VL53L0X 160 / 140 cm, VL53L1X 190 / 170 cm)** — latched once after `plutoInit ( )`, which also
  picks the fusion copy for that sensor (`altLaserFusionLatch ( on, vl53l1x )`); without `initAlt` the firmware is
  the baro build. The return
  to the laser and a re-base shift the whole altitude frame so the craft does not move, and a sudden surface change
  is held on the baro for 2.5 s before re-basing. An accepted `initAlt` whose laser fails to start still runs the
  fusion as a dropout (`FLIGHT_INVARIANTS.md`). `LASER_TOF`, `LASER_ALT` and `LASER_TOF_L1x` are build errors.
- **The laser family is chosen at run time, not by a build flag**: `rangingFamilyLatch ( )` (after `plutoInit ( )`)
  picks VL53L0X ( XRanging, `drivers/tof_manager` ) or VL53L1X ( X1Ranging, `drivers/tof_manager_l1x` ) from the API
  that enabled lasers; **both APIs used → both refused** ( no laser, baro ). The two managers share one device RAM area
  ( `drivers/tof_dev_area` ): only the latched family may touch it ( `docs/API/X1RANGING_API_WIKI.md`,
  `active-development/vl53l1x-xranging-api/` ).
- **All VL53L0X lasers go through one owner, `drivers/tof_manager`, started once by `xRangingInit ( )`** (each laser
  its own state; one bus error stops only that laser; getters read cached samples). The down-laser XSHUT may only be
  GPIO_2/4/5/8 and needs `initAlt ( <pin> )`; that pin is reserved after start-up (no LED / `Peripheral_*` / ADC /
  PWM). SPI2 (GPIO_5-8: the flash and the user SPI API) is off whenever XRanging is used. Every fitted side laser
  must be enabled, or the start-up refuses the lasers after it (`BUS_BUSY`; undetectable with no XSHUT wire)
  (`docs/API/XRANGING_API_WIKI.md`, `active-development/tof-single-pipeline/`).
- **Battery current is read across one R020 ( 20 mOhm ): a stacked second shunt halves every current and mAh reading**
  ( range 8 A ). Low-battery levels come from the Pluto Fuel Gauge ( count, or compensated voltage ≤ 3.745 / 3.60 V per
  cell ), not from the stored warning / minimum voltages.
- **At critical battery the firmware lands ( `LAND` ); the app must not be told "critical" while armed** — it switches
  ARM off on that status and the motors stop in the air. Low battery is reported until the landing disarms, then
  `mwArm ( )` refuses to arm until the pack is changed ( `FLIGHT_INVARIANTS.md` ).
- **User commands cannot replace a landing, and a landing cannot start in a flip**: `Command_TakeOff` / `Command_Flip`
  are refused while `landingInProgress ( )`, and `Command_Land` while `flipState != 0` ( flips end within 8 s ). Every
  end of a take-off goes through `endTakeOff ( )`, and a queued user take-off is dropped after 2 s of refused arming
  or at a real Dev Mode off. With the battery OK, arming is allowed again right after a landing
  ( `docs/API/FC_CONTROL_API_WIKI.md`, `active-development/takeoff-land-command-fix/` ).
- **RC link loss lands ~2.5 s after the last frame** ( Wi-Fi / MSP; ~2.4 s on ELRS / PPM; `failsafe_delay` 15, capped at 22 ): Wi-Fi stalls of 1-2 s are normal. Extend tolerance only through `failsafe_delay`, never through the 600 ms stick hold ( `MAX_INVALID_PULS_TIME` ) or `DELAY_5_HZ` ( `FLIGHT_INVARIANTS.md` ).
- **A full stick-down in flight does not zero the attitude I-term**: below `mincheck` 1100 the `pidRewrite` rate I-term
  is frozen while airborne ( `ARMED && throttleRaisedSinceArm && !isThrottleStickArmed`: app take-off, or ALT_HOLD after
  an ARM-switch arm ) and reset otherwise ( ground, arming, ground idle, THROTTLE mode after an ARM-switch arm ).
  Zeroing it in flight leaned the craft 2-7 deg toward a payload ( `FLIGHT_INVARIANTS.md` ).
- **User yaw ( `setDesiredAngle` / `DesiredAngle_setRelative` on `AG_YAW` ) is heading hold, which `userCode ( )`
  engages itself while a target is held** ( MAG mode forced unless HEADFREE or no magnetometer ). Without MAG or HEADFREE,
  `updateMagHold ( )` overwrites `magHold` every loop and the target is lost. The target is dropped only at a real
  dev-mode switch-off or a disarm, never on a Wi-Fi flicker ( `active-development/relative-yaw-api/` ).

## Conventions

- A `.clang-format` is present; match the existing style (notably the spaced-paren call style, e.g.
  `Oled_Text ( 0, 0, "..." )`). Files carry a banner header comment block with author/history —
  preserve it when editing.
- This is hard-real-time firmware on a 40 KB-RAM MCU: avoid dynamic allocation in the control path,
  keep `loop()` work bounded, and be mindful of flash/RAM budget (check with `make ... memory`).

## Hardware resource reference

`docs/fw-development-reference/` is the standing reference (target reference: `PRIMUS_X2_v1`).
**Consult it before reasoning about DMA/timer/pin allocation, and keep it in sync with the code.**

- `DMA_MAP.md`, `TIMER_MAP.md`, `PIN_MAP.md`, `WS2812_RGB.md` — channel/timer/pin ownership and conflicts.
- `fw-architecture-pipeline/` — **describes committed firmware only**: do not edit it for work in progress.
- `active-development/<topic>/` — live work; apply its `PIPELINE_UPDATE.md` only at commit
  (rules: `active-development/README.md`).
- DMA ownership is enforced at runtime by `drivers/dma_registry.{h,c}`. When DMA/timer/pin assignments
  change (ADC/PWM/GPIO peripherals, UART, WS2811, `timer.cpp`, `target.h`), update the affected map(s).

Map contents, datasheets and the full sync list: `docs/fw-development-reference/dev-guide/HARDWARE_RESOURCES.md`.

## Knowledge graph

A prebuilt codebase knowledge graph is in `graphify-out/` (`graph.html`, `GRAPH_REPORT.md`). The
`/graphify` skill answers questions against it — prefer it for "where is X / how does Y connect" exploration.

## graphify

This project has a graphify knowledge graph at graphify-out/.

Rules:
- Before answering architecture or codebase questions, read graphify-out/GRAPH_REPORT.md for god nodes and community structure
- If graphify-out/wiki/index.md exists, navigate it instead of reading raw files
- For cross-module "how does X relate to Y" questions, prefer `graphify query "<question>"`, `graphify path "<A>" "<B>"`, or `graphify explain "<concept>"` over grep — these traverse the graph's EXTRACTED + INFERRED edges instead of scanning files
- After modifying code files in this session, run `graphify update .` to keep the graph current (AST-only, no API cost), then `python tools/graph_labels.py` to give the communities readable names in `graph.html` / `GRAPH_REPORT.md` (the update alone leaves them as "Community N"). `inav-9.1.0/` must stay in `.graphifyignore`, or the graph grows ~14x.
