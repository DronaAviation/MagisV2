---
name: magisv2-rules
description: MagisV2 firmware coding rules and review checklist - the invariants, build wiring, style and doc-sync obligations specific to this bare-metal STM32 tree. Use in two ways - when writing or changing firmware code under src/main (before and while editing), and when reviewing a diff, branch or PR for this repo. Covers Makefile source registration, -Wconversion discipline, rcData vs rcDataPilot, user RC override expiry, Monitor_Print byte budget, DMA registry ownership, barometer datum rules, altitude-hold setpoint rules, target.h gating, API header stability and which docs must move with the change. Prefer this over a generic C/C++ review for MagisV2 firmware.
---

# MagisV2 firmware rules

Two entry points, one rule set.

- **Writing** new or changed firmware → work the checklist in *Before you edit*
  and *While editing*, then *Before you call it done*.
- **Reviewing** a diff, branch or PR → **delegate to the `magisv2-reviewer`
  agent**. It runs *Reviewing a diff* below in its own context. Do not run the
  review in the main conversation.

The rules below are ones this codebase actually enforces, or mistakes that have
already cost a flight or a release here. Generic C/C++ advice is out of scope —
`/code-review`, `c-review` and `sharp-edges` cover that ground and do not know
any of this.

Related skills: `run-magisv2` (build + warning gate), `add-driver` (new
peripherals), `flight-test` (hardware validation), `commit-magisv2` (release),
`grill-magisv2` (planning interview + `TASKS.md` before work starts).

## Before you edit

- **Know the working target.** `PRIMUS_X2_v1` or `PRIMUS_V5`, from
  `plutoide.ini` `selected_target`. Edit that target's `target.h`; do not mirror
  into the others unless asked.
- **Check the reference maps before touching a pin, DMA channel or timer** —
  `docs/fw-development-reference/PIN_MAP.md`, `DMA_MAP.md`, `TIMER_MAP.md`. They
  are the source of truth for what is already claimed.
- **Do not bring in upstream code.** Cleanflight/INAV/Betaflight drivers assume
  a different `baro_t` / bus API and do not drop in. Copy the structure of the
  neighbouring driver in this tree instead.
- **Read the live topic folder** if one exists for this work under
  `docs/fw-development-reference/active-development/<topic>/`.

## While editing

### Build wiring

- **Every new `.c`/`.cpp` must be added to a Makefile group by hand.** There is
  no glob for the firmware build. Groups: `PRIMUSX2_DRIVERS` (board drivers),
  `DRONA_DRIVERS` / `DRONA_FLIGHT` / `DRONA_API` / `DRONA_COMMAND` (Pluto
  additions), `MAIN_*` (Cleanflight core), `PRIMUSX2_SENSORS` / `MAIN_SENSOR`,
  `COMMON_SRC`. A file that compiles fine but was never added simply is not in
  the binary.
- **Gate hardware-specific code with a `USE_…`/feature define** in `target.h`,
  and wrap both the source file and every call site. Every other target must
  still build.
- **C headers included from C++ need `extern "C"` guards.** `main.cpp`,
  `mw.cpp` and `PlutoPilot.cpp` are C++.

### Conversions and warnings

The build runs `-Wall -Wextra -Wconversion -Wsign-conversion -Wshadow
-Wdouble-promotion` on both C (`gnu17`) and C++ (`gnu++17`), and there are
already ~2700 warnings, ~1700 of them in vendored `lib/`. **A new warning is
invisible unless you diff it.**

- Make conversions explicit and deliberate. `-Wconversion` and
  `-Wsign-conversion` are 2052 of those warnings and are exactly the class that
  produced the deci-degree bug (`> 30` meant 3°, not 30°).
- Be explicit about fixed-point scale at every boundary: this tree mixes **Pa,
  cm, cm/s, counts (1000–2000 µs RC), µs, ms and deci-degrees**. Name the unit
  in the variable or a trailing comment when it is not obvious.
- Run the gate before you call the change done (see below). Never regenerate
  `tools/warnings-baseline.json` to silence a warning you introduced.

### Control-path invariants

These are the ones that bite.

- **Pilot stick input is `rcDataPilot[]`, never `rcData[]`.** `rx/rx.cpp:564`
  snapshots the pilot's sticks into `rcDataPilot[]` before user code can write
  `rcData`. Reading `rcData` for throttle reads back the override path's own
  output. See `mw.cpp:1055-1103`.
- **User RC overrides expire and must be re-asserted.** `RcCommand_Set` stores
  into `RC_ARRAY[]` and latches `userRCflag[]`; `resetUserRCflag()` drops it
  after `max(250 ms, 2 × userLoopFrequency)`. User code hands a channel back by
  *not* calling `RcCommand_Set`. `applyUserRcOverride()` cross-fades against
  pilot deflection — do not add a second blend on top unless you pass authority
  `0.0f` as `applyObjectAvoidance()` does.
- **The barometer ground datum must never be re-zeroed in flight.** It tracks
  while disarmed and freezes on arm; `throttleRaisedSinceArm` (`mw.cpp:174`)
  guards it. Re-zeroing mid-flight silently redefines altitude.
- **In ALT_HOLD the throttle stick moves the setpoint, it never replaces the
  position loop.** `calculateAltHoldThrottleAdjustment()` ramps `AltHold` at
  `altRate`. **Landing must not go through the stick rate limits** — at slow
  descent rates in-ground-effect baro drift masks the descent and touchdown is
  never detected.
- **`Monitor_Print` output above ~250 bytes per tick is silently corrupted.**
  It writes into the MSP UART's 256-byte TX ring and `uartWrite()` does not
  check for full, so the *start* of the line is overwritten. Count the bytes
  when adding a diagnostic field.
- **DMA channels go through the registry.** `dmaClaim()` / `dmaRelease()` /
  `dmaIsFree()` / `dmaGetOwner()` in `drivers/dma_registry.h`. ADC DMA is lazy —
  claimed only when a `Peripheral_Init(ADC_x)` pin on that ADC is used.
- **Real-time budget.** No dynamic allocation in the control path, no unbounded
  work in `loop()`, no blocking waits. 40 KB RAM total.

### Style

- Match `.clang-format` and the surrounding file. Most visibly the **spaced-paren
  call style**: `Oled_Text ( 0, 0, "..." )`.
- **Preserve the banner header block** at the top of each file (SPDX, author,
  project, created/modified dates, HISTORY table). Update `Last Modified` /
  `Modified By` rather than dropping it.
- **`lib/main/` is vendored upstream** (CMSIS, StdPeriph, USB-FS, VL53L0X/L1X).
  Do not reformat or clean it.

## Before you call it done

1. **Build the working target and run the warning gate** — `run-magisv2`, or by
   hand:
   ```bash
   make TARGET=<target> clean && make TARGET=<target> 2>&1 | tee build.log
   python3 tools/warnings.py check build.log     # exit 1 = you added warnings
   ```
2. **Check flash/RAM headroom** in the build's memory summary (256 KB / 40 KB).
3. **Docs that must move with the code:**
   - Public API signature or behaviour changed in `API/` or `API-Src/` → update
     the matching wiki in `docs/API/`, and bump `FW_Version` / `API_Version` in
     the Makefile.
   - DMA / timer / pin assignment changed → update `DMA_MAP.md`, `TIMER_MAP.md`,
     `PIN_MAP.md`.
   - Work in progress → record it in
     `active-development/<topic>/`. **Do not edit `fw-architecture-pipeline/`** —
     that describes committed firmware only, and is updated at commit time by
     `commit-magisv2`.
4. **Do not try to run `src/test/`.** The GoogleTest suite does not build and is
   not part of any workflow. The build is the verification; hardware is
   validated by flight log (`flight-test`).

## Reviewing a diff

**Reviews run in the `magisv2-reviewer` subagent, not in the main
conversation.** Launch it with the target (diff, branch, commit range, PR, or
the task from `TASKS.md`), then relay its findings. This section is its
checklist.

Get the diff first (`git diff main...HEAD`, or the PR), then walk these in
order. Report findings with file:line and say which rule each one breaks.

**Blocking — these ship bugs:**

1. New `.c`/`.cpp` file not added to a Makefile group.
2. `rcData[]` read where pilot input was meant (`rcDataPilot[]`).
3. A user RC override assumed sticky, with no re-assert each loop.
4. Barometer ground reference re-zeroed while armed / in flight.
5. Landing or descent routed through the ALT_HOLD stick rate limits.
6. A `Monitor_Print` / diagnostic log addition pushing the tick over ~250 bytes.
7. DMA channel used without `dmaClaim()`, or claimed without release.
8. Dynamic allocation, blocking wait or unbounded loop in the control path.
9. Hardware code not gated by a `target.h` define, breaking another target.
10. New implicit conversion where the scale or sign matters — check units at
    every assignment crossing Pa / cm / counts / µs / deci-degrees.

**Should fix:**

11. Warning gate not run, or baseline regenerated to hide a new warning.
12. Public API changed without the `docs/API/` wiki and version bump.
13. Pin / DMA / timer change without the reference map update.
14. `fw-architecture-pipeline/` edited for work that is not committed yet.
15. Banner header dropped or mangled; formatting that fights `.clang-format`.
16. `lib/main/` reformatted or "cleaned".

**Verify, don't assume.** Before reporting an invariant break, open the file and
confirm — several of these rules have a legitimate exception in the tree
(`applyObjectAvoidance()` deliberately passes authority `0.0f`; the landing path
deliberately keeps its own descent rate).

## References

- `references/invariants.md` — the control-path rules in full, with the file and
  line each one lives at and the failure each one prevents.
- `references/style.md` — formatting, banner headers, units and naming.
