# RcCommand_Get API - Tasks

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md)

## Resume

Read this block first in a new session; read only the task it points to.

| | |
|---|---|
| **Current task** | none: **Closed - pending commit** ( 5 Nov 2026 ) |
| **Next step** | The user commits with `.git/MAGISV2_COMMIT_MSG.txt`; the next pluto-commit / pluto-grill run fills in the hash |
| **Open questions** | none |
| **Blocked on** | nothing |
| **Last updated** | 2026-11-05 |

## Summary

| | |
|---|---|
| **Mode** | fix |
| **Goal** | `int16_t *RcCommand_Get ( void )` links and returns the four live channel values. |
| **Done when** | All targets link; on the bench ( props off, app sticks ) the four elements follow the sticks, roll / pitch / yaw equal `RcCommand_Get ( channel )`, throttle is 1100-2000. |
| **Target** | PRIMUS_X2_v1 ( all targets at commit ) |
| **Analysis** | Causal chain proven ( INVESTIGATION.md ): name mismatch ( fixed by the user's rename ), then a returned local array that GCC compiles to `return NULL`. |
| **Pipeline docs affected** | subsystems/User_Space_API.md ( one bullet ); new docs/API/RC_INTERFACE_API_WIKI.md; CHANGELOG at commit |
| **Order** | 1 -> 2 -> 3 -> 4 -> 5 -> 6 |

## Index

| # | Title | Status | Depends on | Skills / agent |
|---|---|---|---|---|
| 1 | Return a static snapshot from `RcCommand_Get ( )`, correct the header comments | done 2026-11-05 | - | pluto-rules |
| 2 | Build gate | done 2026-11-05 | 1 | pluto-build |
| 3 | Bench validation: the four elements against the sticks | done 2026-11-05 | 2 | pluto-flighttest |
| 4 | Architecture & pipeline docs ( PIPELINE_UPDATE.md, RC API wiki ) | done 2026-11-05 | 3 | spec-to-code-compliance |
| 5 | Topic review | done 2026-11-05 | 4 | pluto-reviewer agent |
| 6 | Graph refresh & commit ( FW 3.15.5, API 1.8.3 ) | done 2026-11-05 | 5 | graphify, pluto-commit |

Status values: `todo`, `in-progress`, `done YYYY-MM-DD`, `blocked (<why>)`, `dropped (<why>)`.
Serial numbers are never reused or renumbered.

## Tasks

### 1. Return a static snapshot from `RcCommand_Get ( )`, correct the header comments

**Description.** Keep the user's rename `RcCommand_get` -> `RcCommand_Get`. Replace the local array with a file-static
`int16_t [ 4 ]` filled on each call and return its pointer: roll / pitch / yaw as `1500 + rcCommand [ i ]` ( the
channel getter's scale, D2 ), throttle as `rcCommand [ THROTTLE ]` ( D3 ), with explicit casts for `-Wconversion`. Swap
and rewrite the doc comments of both `RcCommand_Get` overloads in `RC-Interface.h` ( units, snapshot lifetime ). Leave
`RcData_Get ( )` and the channel getter's throttle as they are ( out of scope ).

- **Depends on:** -
- **Skills / agent:** pluto-rules
- **Files:** [RC-Interface.cpp](../../../../src/main/API-Src/RC-Interface.cpp), [RC-Interface.h](../../../../src/main/API/RC-Interface.h)
- **Safety impact:** none ( user-code getter; reads `rcCommand`, writes only its own static copy; no firmware caller )
- **Done when:** builds on PRIMUS_X2_v1; the `rcCommandArray` `-Wreturn-local-addr` warning is gone; no new `src/` warning; disassembly shows a copy, not `return NULL`.
- **Rollback:** `git checkout -- src/main/API-Src/RC-Interface.cpp src/main/API/RC-Interface.h` ( back to the link error )
- **Status:** done 2026-11-05
- **Result:** `RcCommand_Get ( )` fills a file-static `int16_t [ 4 ]` and returns it ( objdump: copy instead of `return NULL` ); gate green, `rcCommandArray` warning fixed, size identical to HEAD. Review: 0 blocking; the should-fix ( throttle is the stick through the curve, not the hold output ) and the HEADFREE nit fixed in the header comments.

### 2. Build gate

**Description.** `PROJECT=AltX .claude/skills/pluto-build/driver.sh --gate PRIMUS_X2_v1`; record flash / RAM against
HEAD `1b43fbe` and the cost of a call from user code.

- **Depends on:** 1
- **Skills / agent:** pluto-build
- **Files:** -
- **Safety impact:** none
- **Done when:** no new `src/` warning; flash / RAM recorded in CHANGES.md.
- **Rollback:** n/a
- **Status:** done 2026-11-05
- **Result:** gate green on PRIMUS_X2_v1, no new `src/` warning, `rcCommandArray` warning fixed; size identical to HEAD ( 121.1 KB / 20.5 KB ); a user-code call costs 44 B flash + 8 B RAM.

### 3. Bench validation: the four elements against the sticks

**Description.** Temporary `Monitor_Print` in `plutoLoop ( )` of `PlutoPilot.cpp`: the four elements of
`RcCommand_Get ( )`, `RcCommand_Get ( RC_ROLL / RC_PITCH / RC_YAW )` and `RcData_Get ( RC_THROTTLE )` ( under 130 bytes
per tick ). Props off, disarmed, app connected: centre, then each stick to both ends. Record in TESTING.md, then
restore `PlutoPilot.cpp` to HEAD and rebuild.

- **Depends on:** 2
- **Skills / agent:** pluto-flighttest
- **Files:** [PlutoPilot.cpp](../../../../PlutoPilot.cpp) ( temporary )
- **Safety impact:** none ( bench, disarmed, props off )
- **Done when:** elements 0-2 equal the channel getter on every record and follow the sticks ( about 1000-2000 ); element 3 follows the throttle stick within 1000-2000; no constant value; `PlutoPilot.cpp` back to HEAD.
- **Rollback:** n/a
- **Status:** done 2026-11-05
- **Result:** PASS ( log-1 ): array = channel getter on 563 / 563 records; `a3 = max ( d3, 1100 ) - 1`; all elements follow the sticks. Roll and pitch always moved together ( diagonal stick or HEADFREE ), so their individual signs and ends were not shown; throttle bottom not reached ( app stick springs to 1500 ). `PlutoPilot.cpp` back to HEAD, clean hex rebuilt.

### 4. Architecture & pipeline docs ( PIPELINE_UPDATE.md, RC API wiki )

**Description.** New `docs/API/RC_INTERFACE_API_WIKI.md` ( `RcData_Get`, `RcCommand_Get`, `RcCommand_Set`: units,
the snapshot lifetime, an example per hook, known gaps: `RcData_Get ( )` returns NULL, the channel throttle reads
2600-3500, and the override expiry rule ). `PIPELINE_UPDATE.md` with a `User_Space_API.md` bullet. Check every unit
against the code.

- **Depends on:** 3
- **Skills / agent:** spec-to-code-compliance
- **Files:** PIPELINE_UPDATE.md ( this folder ), `docs/API/RC_INTERFACE_API_WIKI.md` ( new )
- **Safety impact:** none
- **Done when:** wiki and PIPELINE_UPDATE.md exist; every unit matches a source line.
- **Rollback:** n/a
- **Status:** done 2026-11-05
- **Result:** new `docs/API/RC_INTERFACE_API_WIKI.md` ( 1.8.3: values, ranges, signs, what user code reads, known gaps, examples ) and `PIPELINE_UPDATE.md` ( one `User_Space_API.md` bullet ); every claim checked against a source line ( CHANGES.md, inline: the multi-agent compliance workflow needs the user's opt-in ). No code change.

### 5. Topic review

**Description.** `pluto-reviewer` on the whole topic diff ( code and docs ). Fix BLOCKING findings and re-gate if code
changed.

- **Depends on:** 4
- **Skills / agent:** pluto-reviewer agent
- **Files:** topic diff
- **Safety impact:** none
- **Done when:** no open BLOCKING finding.
- **Rollback:** n/a
- **Status:** done 2026-11-05
- **Result:** pluto-reviewer: 0 blocking, 1 should-fix, 4 nits, all fixed in comments / docs ( no code change ): throttle element documented as the curve of `rcData [ THROTTLE ]` ( landing, altitude cap, failsafe 1200, user throttle override can show ), HEADFREE 860-2140, localisation note dropped ( OPTIC_FLOW only ), `onLoopStart` re-run noted, Summary throttle 1100-2000.

### 6. Graph refresh & commit ( FW 3.15.5, API 1.8.3 )

**Description.** `graphify update .`, then
`GRAPHIFY_VIZ_NODE_LIMIT=6000 "/c/Users/MechAsh/AppData/Roaming/uv/tools/graphifyy/Scripts/python.exe" -I tools/graph_labels.py`.
Then `/pluto-commit`: all targets, Makefile FW 3.15.5 / API 1.8.3, promote PIPELINE_UPDATE.md, CHANGELOG Fixed,
`tools/warnings-baseline.json` note ( one entry fixed ), stage; the user commits.

- **Depends on:** 5
- **Skills / agent:** graphify, pluto-commit
- **Files:** Makefile, CHANGELOG.md, graphify-out/
- **Safety impact:** none
- **Done when:** staged tree ready; topic Closed - pending commit.
- **Rollback:** n/a
- **Status:** done 2026-11-05
- **Result:** graph refreshed ( 5112 nodes ) and relabelled; all targets built clean ( PRIMUS_X2_v1 121.1 KB / 20.5 KB, PRIMUSX2 120.9 / 20.7, PRIMUS_V5 121.1 / 20.5 ), no new warnings; Makefile FW 3.15.5 / API 1.8.3; PIPELINE_UPDATE promoted into `User_Space_API.md`; CHANGELOG version line, Fixed and Documentation entries; staged, message in `.git/MAGISV2_COMMIT_MSG.txt`.

## Decisions log

Newest last. One line each: `YYYY-MM-DD [decision|assumption|out-of-scope|risk] text`.

- 2026-11-05 [decision] Mode `fix`: the user asked whether their rename is proper and ready to commit; it fixes the link error but the function returns NULL ( disassembly ), so it is not ready alone.
- 2026-11-05 [decision] Target PRIMUS_X2_v1 ( this session's working target ).
- 2026-11-05 [decision] D1: file-static `int16_t [ 4 ]` snapshot, refreshed per call; signature unchanged. Caller buffer ( API break ) and a pointer to the live `rcCommand` ( writable by user code ) rejected.
- 2026-11-05 [decision] D2: roll / pitch / yaw elements are `1500 + rcCommand`, the same as `RcCommand_Get ( channel )` and the `RcCommand_Set` range.
- 2026-11-05 [decision] D3: throttle element is `rcCommand [ THROTTLE ]` as is ( MINTHROTTLE..MAXTHROTTLE, the hold loop's output in ALT_HOLD ), not the `rcData` stick value.
- 2026-11-05 [out-of-scope] `RcData_Get ( )` ( same local-array bug, returns NULL ) and `RcCommand_Get ( RC_THROTTLE )` returning 2600-3500: documented as known gaps in the wiki.
- 2026-11-05 [risk] Until the channel throttle is fixed, `RcCommand_Get ( ) [ 3 ]` ( 1100-2000 ) and `RcCommand_Get ( RC_THROTTLE )` ( 2600-3500 ) disagree.
- 2026-11-05 [decision] Bench log validation; header comments plus a new RC API wiki; API 1.8.3 ( behaviour only ), FW 3.15.5.
- 2026-11-05 [assumption] `pluto-scout` not run: one function, recon done inline ( SCOUT.md ).
- 2026-11-05 [decision] The user ran `/pluto-task next` after the playback without corrections: taken as confirmation; topic written and task 1 started in the same turn.
- 2026-11-05 [decision] Task 1 review correction to D3: the throttle element is still `rcCommand [ THROTTLE ]`, but user code sees the stick through the throttle curve ( 1100-2000 ), never the altitude-hold output, because `userCode ( )` runs before `applyAltHold ( )` ( `mw.cpp:1416` vs `:1455` ). Documented so; the channel throttle reads 2600-3500.
- 2026-11-05 [decision] Task 3 instrumentation review: no blocking. Expected values corrected in TESTING.md: roll / pitch full stick about 1050 / 1950, yaw 1000 / 2000 with yaw right below 1500 ( `yaw_control_direction` 1 ), `a3 = max ( d3, 1100 )`; 118-122 B per tick.
- 2026-11-05 [decision] Task 3 log-1 PASS on the API checks ( array = channel getter 563 / 563, throttle = curve output ). Roll / pitch were never moved alone in the log, so their signs and full-stick ends are taken from the code in task 4, not from the log; no re-run asked, since they are the channel getter's values by construction.
- 2026-11-05 [decision] Task 4: wiki documents the yaw inversion ( `yaw_control_direction` 1 ) and that user code reads the pilot's command ( before `applyUserRcOverride` and `applyAltHold` ). The `spec-compliance` workflow was not run ( up to 25 agents, needs the user's opt-in ); the check was done inline.
- 2026-11-05 [decision] Task 5 review: no blocking. The throttle element is the curve of `rcData [ THROTTLE ]`, so it is not always the stick ( landing, altitude cap, failsafe, the user's own throttle override ); documented, behaviour kept.
