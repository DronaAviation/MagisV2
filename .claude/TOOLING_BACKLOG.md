# Tooling backlog ( skills and agents )

What development tooling exists for MagisV2, what should be added, and
what was evaluated from outside collections. Update the status column as items
are built. Last reviewed 21 Sep 2026. Items 1-5 built 18 Sep 2026, items 6-7
on 21 Sep 2026.

## In place

| Item | Type | Purpose |
|---|---|---|
| `pluto-build` | skill | Build and memory check. Working target during development ( asked once per session ), all targets at commit. |
| `pluto-commit` | skill | Commit preparation: all-target build, version bump on request, promote `active-development/` docs to pipeline + CHANGELOG. |
| `pluto-flighttest` | skill | Test plans, `PlutoPilot.cpp` log budget, and analysis with `tools/flightlog.py`. |
| `pluto-driver` | skill | Checklist for new drivers / peripherals: Makefile groups, init order, buses, DMA/ADC pitfalls, maps. |
| `c-pro`, `cpp-pro` | agents | VoltAgent-derived, rewritten for this tree: bare-metal C / embedded C++17, `ATOMIC_BLOCK`, single-precision FPU, warning gate, "MagisV2 project rules". The `embedded-systems` skill and agent were removed 23 Sep 2026; their MCU-safety content moved to `pluto-rules` ( invariants §11 ) and `pluto-driver`. |
| `pluto-log-analyst` | agent | Read-only log analysis with `tools/flightlog.py`, returns conclusions only. |
| `tools/graph_labels.py` | script | Names graphify communities from their contents ( `area/file: symbol` ) and rewrites `graph.html` + `GRAPH_REPORT.md`; run after every `graphify update .`. |
| `graphify` | skill + CLI | `/graphify` skill in `~/.claude/skills` ( per user: `graphify install` ), CLAUDE.md section and PreToolUse hook ( `graphify claude install --project` ). |
| `tools/flightlog.py` | script | PlutoMonitor log parser: `summary`, `table`, `report` ( temperature fit, height hold vs laser, applied correction and limit headroom ). |
| `pluto-rules` | skill | Firmware coding rules and review checklist in one skill, two modes ( writing / reviewing a diff ). `references/invariants.md` ( control-path rules with file:line ), `references/style.md` ( format, banner, units ). |
| `tools/warnings.py` | script | Compiler-warning baseline and triage. `baseline` / `check` / `summary`; fails only on new warnings under `src/`, never `lib/`. Baseline committed at `tools/warnings-baseline.json`. |
| `pluto-build --gate` | skill + driver | `driver.sh --gate <TARGET>` keeps the build log at `Build/<TARGET>/build.log`, runs the warning check and fails the run on new `src/` warnings. Forces a clean build. |
| Trail of Bits skills | plugins | `dimensional-analysis`, `spec-to-code-compliance`, `c-review`, `sharp-edges` installed via `settings.json` `enabledPlugins`. Not yet exercised on this tree. |

## To build, in priority order

| # | Item | Type | Why | Status |
|---|---|---|---|---|
| 1 | **`pluto-flighttest`** | skill + committed script | Test plans ( ground-flight-ground, cold start, 3+ min hover ), `Monitor_Print` field budget ( ~250 B/tick ), laser ground truth, and a committed `tools/flightlog.py`: parse PlutoMonitor output, segment by `Arm`, fit pressure vs temperature, laser vs `BaroAlt`, back out the applied correction, window-sensitivity check. The alt-hold work re-wrote this analysis ~8 times in a scratch folder. | **Done** 18 Sep 2026 |
| 2 | **`pluto-driver`** | skill | Checklist for a new sensor / peripheral: Makefile group ( no glob ), `target.h` define, `main.cpp` init and `mw.cpp` periodic hook, DMA registry, ADC1/ADC2 shared common register, ADC has no clock out of reset, F3 temp sensor negative slope, shared I2C1 bus, which reference maps to update. Both ADC bugs in the alt-hold diagnostic driver cost a flight each. | **Done** 18 Sep 2026 |
| 3 | **`pluto-log-analyst`** | agent | Read-only, runs item 1's script on large logs ( 15k-37k lines ) and returns conclusions only, keeping raw data out of the main context; can run while the next flight is set up. Depends on 1. | **Done** 18 Sep 2026 |
| 4 | **Trim / merge the generic agents** | agent cleanup | `cpp-pro` and `embedded-systems` tell the agent to read `CMakeLists.txt` and query a "context manager" agent; neither exists here. Either trim those steps or replace all three with one `magisv2-firmware` agent that knows the codebase. | **Done** ( trimmed, not merged ) 18 Sep 2026 |
| 5 | Move "power-cycle after DFU flash" into `pluto-build` troubleshooting | skill edit | A board left in the bootloader looks dead ( no LEDs, app not connecting ). | **Done** 18 Sep 2026 |
| 6 | **`pluto-rules`** | skill | Review and coding-standard rules were the one gap the generic tooling could not fill: `/code-review`, `c-review` and `sharp-edges` know C, not this tree. Encodes the invariants that have cost flights ( `rcData` vs `rcDataPilot`, override expiry, baro datum freeze, landing bypassing stick limits, `Monitor_Print` 250 B, DMA registry, Makefile registration ) plus doc-sync obligations. One skill, two modes, so write-time and review-time rules cannot drift. | **Done** 21 Sep 2026 |
| 7 | **`tools/warnings.py` + `--gate`** | script + skill edit | A clean `PRIMUS_X2_v1` build emits **2674** warnings, **1712** of them in vendored `lib/`. A warning introduced by a change is invisible. Baseline-diff reports only new `src/` warnings. Dominant classes are `-Wconversion` ( 1149 ) and `-Wsign-conversion` ( 903 ) — the scale/sign class that produced the deci-degree bug. Verified: injected 3 warnings into `altitudehold.cpp`, gate reported all 3 and exited 1. | **Done** 21 Sep 2026 |
| 8 | `docs-drift-editor` | agent | Last unharvested item from the VoltAgent collection. Minimal Markdown edits when code moves under a doc; would need adapting to the `active-development/` topic-folder layout. | Open |
| 9 | Exercise the Trail of Bits skills on a known bug | evaluation | All four are installed but none has been run on this tree. 23 Sep 2026: `dimensional-analysis` and `sharp-edges` dropped from routing ( units are covered by `pluto-rules` and the reviewer ); `c-review` only on request. Remaining: run `spec-to-code-compliance` over `fw-architecture-pipeline/` against a bug whose answer is already known before trusting it. | Open |
| 10 | **Warning gate covers repo-root `PlutoPilot.cpp`** | script edit | `tools/warnings.py` `scope_of()` counts only paths under `src/`, so warnings in the repo-root `PlutoPilot.cpp` land in "other" and never fail the gate. Found 21 Sep 2026 ( tof-althold-fusion task 1 ): two `-Wdouble-promotion` warnings passed the gate unseen. Treat `PlutoPilot.cpp` as `src`. Separately, enabling a target feature ( `LASER_TOF` / `LASER_ALT` ) surfaces warnings in code the laser-off baseline never compiled. | Open |
| 11 | **`Monitor_Print` safe under load** | firmware ( API-Src ) | `debugPrint ( )` writes straight into the MSP TX ring with no room check, so a long log line corrupts the app's MSP replies and the app disconnects ( ~180 B/tick did, 21 Sep 2026 ). Drop or defer a frame when `serialTxBytesFree ( )` is short. Same file: the double overload subtracts the ASCII code from `remainder`, so every digit after the first decimal prints 0. Needs its own topic ( public API, version bump ). | Open |

## From external collections

Five collections reviewed. Verdict: **none supplies a capability this repo
lacks.** Bare-metal STM32 firmware is close to unrepresented in all of them; the
only durable yield was the Trail of Bits set, now installed.

| Collection | Size | Verdict |
|---|---|---|
| [VoltAgent/awesome-claude-code-subagents](https://github.com/VoltAgent/awesome-claude-code-subagents) | 161 agents | **Mined.** 3 relevant ( `cpp-pro`, `embedded-systems`, basis for `c-pro` ), all installed and trimmed. Only `docs-drift-editor` left ( item 8 ). No C-only specialist exists in it. |
| [VoltAgent/awesome-agent-skills](https://github.com/VoltAgent/awesome-agent-skills) | ~1500 skills | **Mined.** It is an index, not a library; its value was pointing at Trail of Bits. Essentially zero embedded content. |
| [browser-act/skills](https://github.com/browser-act/skills) | — | **No.** Browser automation, CAPTCHA solving, scraping. Nothing applicable. |
| [alirezarezvani/claude-skills](https://github.com/alirezarezvani/claude-skills) | 388 skills | **No.** Bulk is marketing, C-level personas and compliance boilerplate; the engineering slice is generic web/DevOps. |
| [shanraisshan/claude-code-best-practice](https://github.com/shanraisshan/claude-code-best-practice) | — | **Read, do not install.** Ships no skills or agents — documentation on structuring workflows, subagents and context. Useful for refining the skills here. |

**Installed, not yet exercised** ( item 9 )

| Item | Source | Use here |
|---|---|---|
| `dimensional-analysis` | Trail of Bits skill | Detects unit mismatches. This tree mixes Pa, cm, cm/s, counts, µs, ms and deci-degrees; the alt-hold work found a deci-degree bug ( `> 30` meant 3°, not 30° ). Try it on `flight/` and `sensors/`. |
| `spec-to-code-compliance` | Trail of Bits skill | Checks code against its documentation. The live altitude pipeline doc names functions that do not exist; run it over `fw-architecture-pipeline/` against the source. |
| `c-review` | Trail of Bits skill | C/C++ review with verified coverage. Fits the class of bug found in v4.0.0 ( out-of-bounds write in `RcCommand_Set` ). Security framing, but buffer and bounds findings apply to firmware. |
| `sharp-edges` | Trail of Bits skill | Finds footgun APIs. Candidate: `uartWrite ( )` silently overwriting its TX buffer. |

None of the four knows this tree's invariants — pair them with `pluto-rules`
rather than treating either as sufficient alone.

**Still to harvest**

| Item | Source | Use here |
|---|---|---|
| `docs-drift-editor` | VoltAgent agent | Minimal edits to Markdown that drifted from a code change; refuses large rewrites. Useful for keeping `active-development/` line links current. Would need adapting to the topic-folder layout. |

**Use as a starting template only**

| Item | Source | Note |
|---|---|---|
| `data-analyst` / `data-scientist` | VoltAgent agents | Generic; a base for item 3 at most. |
| `code-reviewer`, `debugger` | VoltAgent agents | Built-in `/code-review` already covers review. |

**Not useful here**

- VoltAgent `iot-engineer`, `build-engineer`, `git-workflow-manager`,
  `test-automator`, `documentation-engineer`: generic or cloud/web oriented; the
  Makefile build, the commit skill and the `active-development/` rules already
  cover these for this repo.
- Trail of Bits `static-analysis` ( CodeQL / Semgrep ): heavy tooling for an
  embedded C tree. The compiler is the analyser here — see item 7.
- `make cppcheck` **is not a usable analysis path** and should not be treated as
  one. It scans `$(CSOURCES)` = the 49 `.c` files only, so none of `flight/`,
  `sensors/` or `API-Src/` ( 89 `.cpp` ) is covered; it hardcodes Linux include
  paths ( `--platform=unix64`, `-I/usr/include` ); and `cppcheck` is not
  installed. Fixing the target would still cover less than the build already
  does with `-Wconversion -Wsign-conversion -Wshadow -Wdouble-promotion` over
  100% of the tree. Use `tools/warnings.py`.
- Test-framework skills ( Playwright, Jest, Pytest, ... ): no host test suite in
  use ( `src/test/` does not build ).

**Caution:** our `cpp-pro` and `embedded-systems` agents started as VoltAgent
templates and needed their generic steps ( `CMakeLists.txt`, "context manager",
JSON protocol blocks ) removed ( item 4 ). Anything imported from these
collections needs the same review before use.

## Installing

- Trail of Bits: `/plugin marketplace add trailofbits/skills`, then install
  single plugins. Try one at a time on a known bug before relying on it.
- VoltAgent agents: copy the single `.md` into `.claude/agents/` and edit it;
  avoid the bulk installers, which pull in dozens of irrelevant agents.
