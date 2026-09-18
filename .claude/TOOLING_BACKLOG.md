# Tooling backlog ( skills and agents )

What development tooling exists for MagisV2, what should be added, and
what was evaluated from outside collections. Update the status column as items
are built. Last reviewed 18 Sep 2026. Items 1-5 built the same day.

## In place

| Item | Type | Purpose |
|---|---|---|
| `run-magisv2` | skill | Build and memory check. Working target during development ( asked once per session ), all targets at commit. |
| `commit-magisv2` | skill | Commit preparation: all-target build, version bump on request, promote `active-development/` docs to pipeline + CHANGELOG. |
| `flight-test` | skill | Test plans, `PlutoPilot.cpp` log budget, and analysis with `tools/flightlog.py`. |
| `add-driver` | skill | Checklist for new drivers / peripherals: Makefile groups, init order, buses, DMA/ADC pitfalls, maps. |
| `embedded-systems` | skill | Generic embedded reference material. |
| `c-pro`, `cpp-pro`, `embedded-systems` | agents | VoltAgent-derived; generic context-manager / CMake / JSON-protocol steps removed, "MagisV2 project rules" appended. |
| `flightlog-analyst` | agent | Read-only log analysis with `tools/flightlog.py`, returns conclusions only. |
| `tools/graph_labels.py` | script | Names graphify communities from their contents ( `area/file: symbol` ) and rewrites `graph.html` + `GRAPH_REPORT.md`; run after every `graphify update .`. |
| `graphify` | skill + CLI | `/graphify` skill in `~/.claude/skills` ( per user: `graphify install` ), CLAUDE.md section and PreToolUse hook ( `graphify claude install --project` ). |
| `tools/flightlog.py` | script | PlutoMonitor log parser: `summary`, `table`, `report` ( temperature fit, height hold vs laser, applied correction and limit headroom ). |

## To build, in priority order

| # | Item | Type | Why | Status |
|---|---|---|---|---|
| 1 | **`flight-test`** | skill + committed script | Test plans ( ground-flight-ground, cold start, 3+ min hover ), `Monitor_Print` field budget ( ~250 B/tick ), laser ground truth, and a committed `tools/flightlog.py`: parse PlutoMonitor output, segment by `Arm`, fit pressure vs temperature, laser vs `BaroAlt`, back out the applied correction, window-sensitivity check. The alt-hold work re-wrote this analysis ~8 times in a scratch folder. | **Done** 18 Sep 2026 |
| 2 | **`add-driver`** | skill | Checklist for a new sensor / peripheral: Makefile group ( no glob ), `target.h` define, `main.cpp` init and `mw.cpp` periodic hook, DMA registry, ADC1/ADC2 shared common register, ADC has no clock out of reset, F3 temp sensor negative slope, shared I2C1 bus, which reference maps to update. Both ADC bugs in the alt-hold diagnostic driver cost a flight each. | **Done** 18 Sep 2026 |
| 3 | **`flightlog-analyst`** | agent | Read-only, runs item 1's script on large logs ( 15k-37k lines ) and returns conclusions only, keeping raw data out of the main context; can run while the next flight is set up. Depends on 1. | **Done** 18 Sep 2026 |
| 4 | **Trim / merge the generic agents** | agent cleanup | `cpp-pro` and `embedded-systems` tell the agent to read `CMakeLists.txt` and query a "context manager" agent; neither exists here. Either trim those steps or replace all three with one `magisv2-firmware` agent that knows the codebase. | **Done** ( trimmed, not merged ) 18 Sep 2026 |
| 5 | Move "power-cycle after DFU flash" into `run-magisv2` troubleshooting | skill edit | A board left in the bootloader looks dead ( no LEDs, app not connecting ). | **Done** 18 Sep 2026 |

## From external collections

Reviewed: [VoltAgent/awesome-claude-code-subagents](https://github.com/VoltAgent/awesome-claude-code-subagents)
( ~150 agents ) and [VoltAgent/awesome-agent-skills](https://github.com/VoltAgent/awesome-agent-skills)
( ~1500 skills, almost none embedded ). The strongest matches came from the
[Trail of Bits skills](https://github.com/trailofbits/skills) listed there.

**Worth trying**

| Item | Source | Use here |
|---|---|---|
| `dimensional-analysis` | Trail of Bits skill | Detects unit mismatches in calculations. The alt-hold work found a deci-degree bug ( `> 30` meant 3°, not 30° ) and mixes Pa, cm, counts, µs and ms throughout. Try it on `flight/` and `sensors/`. |
| `spec-to-code-compliance` | Trail of Bits skill | Checks code against its documentation. The live altitude pipeline doc names functions that do not exist; run it over `fw-architecture-pipeline/` against the source. |
| `c-review` | Trail of Bits skill | C/C++ review with verified coverage. Fits the class of bug found in v4.0.0 ( out-of-bounds write in `RcCommand_Set` ). Security framing, but buffer and bounds findings apply to firmware. |
| `sharp-edges` | Trail of Bits skill | Finds footgun APIs. Candidate: `uartWrite ( )` silently overwriting its TX buffer. |
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
  embedded C tree; `make cppcheck` already exists. Revisit only if cppcheck
  proves insufficient.
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
