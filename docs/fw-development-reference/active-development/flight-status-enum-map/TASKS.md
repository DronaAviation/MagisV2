# Flight status enum map - Tasks

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

## Resume

Read this block first in a new session; read only the task it points to.

| | |
|---|---|
| **Current task** | none: **Closed - pending commit** ( 14 Nov 2026 ) |
| **Next step** | The user commits with `.git/MAGISV2_COMMIT_MSG.txt`; the next pluto-commit / pluto-grill run fills in the hash |
| **Open questions** | none |
| **Blocked on** | nothing |
| **Last updated** | 2026-11-14 |

## Summary

| | |
|---|---|
| **Mode** | `fix` |
| **Goal** | `FlightStatus_Get ( )` returns, and `FlightStatus_Check ( )` tests, the documented public `flightstatus_e` numbers ( a crash reads 4, `FS_CRASHED` ) |
| **Done when** | Bench log on the fixed build, props off, ANGLE, armed, tilted past 70 deg and kept tipped: `FlightStatus_Get ( )` = 4, `Check ( FS_CRASHED )` = 1, `Check ( FS_LOW_BATTERY )` = 0; 7 once righted, 8 once armed; no new `src/` warning |
| **Target** | `PRIMUS_X2_v1` ( all targets at commit ) |
| **Analysis** | Proven: the API casts the internal bit index ( Crash 2, Low_battery 3, LowBattery_inFlight 4 ) into a public enum numbered 2 LOW_BATTERY, 3 INFLIGHT_LOW_BATTERY, 4 CRASHED. MSP maps by name and is correct. [INVESTIGATION.md](INVESTIGATION.md) |
| **Pipeline docs affected** | `subsystems/User_Space_API.md` ( via `PIPELINE_UPDATE.md` ); `docs/API/FC_CONTROL_API_WIKI.md` ( new flight-status section ) |
| **Order** | 1 → 2 → 3 → 4 → 5 → 6 |

## Index

| # | Title | Status | Depends on | Skills / agent |
|---|---|---|---|---|
| 1 | Map internal flight status to the public enum | done 2026-11-05 | - | pluto-rules, pluto-reviewer agent |
| 2 | Build gate | done 2026-11-05 | 1 | pluto-build |
| 3 | Hardware validation: bench crash-disarm log | done 2026-11-14 | 2 | pluto-flighttest |
| 4 | Architecture & pipeline docs ( PIPELINE_UPDATE.md, wiki ) | done 2026-11-14 | 3 | graphify, spec-to-code-compliance |
| 5 | Topic review | done 2026-11-14 | 4 | pluto-reviewer agent |
| 6 | Graph refresh & commit | done 2026-11-14 | 5 | graphify, pluto-commit |

Status values: `todo`, `in-progress`, `done YYYY-MM-DD`, `blocked (<why>)`, `dropped (<why>)`.
Serial numbers are never reused or renumbered.

## Tasks

### 1. Map internal flight status to the public enum

**Description.** In `FC-Control-Status.cpp`, translate between the internal `FlightStatus_e` bit index
( `runtime_config.h:85-95` ) and the public `flightstatus_e` ( `FC-Control.h:27-35` ) by name, with one small
`static const` map used by both functions: `FlightStatus_Get ( )` maps the lowest set bit of `flightIndicatorFlag` to the
public value, and `FlightStatus_Check ( status )` maps the public value to the internal flag before `status_FSI ( )`.
Nothing else changes: not the internal enum ( its bit order is the priority and the raw `MSP_STATUS` bits depend on
it ), not the public numbers, not the MSP output, not the crash-clear timing ( D1-D3 ). With no flag set, `Get` keeps
today's result ( the cast of `leastSignificantBit`'s `-1` ); `Check` with a value outside 0-8 returns false ( D4 ).
Correct the `FC-Control.h` comments of both functions: what is returned, the priority order ( lowest internal bit
wins: calibration > crashed > low battery > critical battery > signal loss > not OK / OK to arm > armed ), and that
`FS_INFLIGHT_LOW_BATTERY` is the critical level, latched until the pack is changed. Map entries by name, so the table
cannot drift if a value moves; explicit casts only ( `-Wconversion` ).

- **Depends on:** -
- **Skills / agent:** pluto-rules; pluto-reviewer agent on the task diff
- **Files:** [FC-Control-Status.cpp](../../../../src/main/API-Src/FC-Control-Status.cpp), [FC-Control.h](../../../../src/main/API/FC-Control.h)
- **Safety impact:** none in flight: read-only status reporting for user code. User code that relied on the wrong
  numbers ( `FlightStatus_Check ( FS_CRASHED )` testing critical battery ) changes behaviour; noted in the wiki.
- **Done when:** builds clean on `PRIMUS_X2_v1`; every internal value maps to the public value of the same name
  ( reviewed against both enums ); reviewer finds nothing blocking.
- **Rollback:** revert `FC-Control-Status.cpp` and `FC-Control.h` to HEAD.
- **Status:** done 2026-11-05
- **Result:** Pair table `flightStatusMap [ ]` maps the nine statuses by name for `Get` and `Check`; header comments give the order, crash and battery rules. Gate green, size identical to HEAD. Review: 0 blocking, 1 should-fix + 3 nits ( comment facts ) fixed.

### 2. Build gate

**Description.** `.claude/skills/pluto-build/driver.sh --gate PRIMUS_X2_v1` with `PROJECT=AltX`. No new warning under
`src/`; flash and RAM recorded against HEAD ( expected: a few bytes of flash for the map, 0 RAM if `static const`
lands in flash ).

- **Depends on:** 1
- **Skills / agent:** pluto-build
- **Files:** -
- **Safety impact:** none
- **Done when:** gate green; flash / RAM delta reported
- **Rollback:** -
- **Status:** done 2026-11-05
- **Result:** Gate green on `PRIMUS_X2_v1`: no new `src/` warning ( none from `FC-Control-Status.cpp` ); 122752 / 1316 / 19728 B ( 121.1 KB / 256 KB flash, 20.5 KB / 40 KB RAM ), identical to HEAD: no firmware code calls the two getters, so the map costs nothing until user code does.

### 3. Hardware validation: bench crash-disarm log

**Description.** Temporary logging in `PlutoPilot.cpp` `plutoLoop ( )`, every tick, under 30 bytes:
`FlightStatus_Get ( )`, `FlightStatus_Check ( FS_CRASHED )`, `FlightStatus_Check ( FS_LOW_BATTERY )`,
`FlightStatus_Check ( FS_INFLIGHT_LOW_BATTERY )`. Bench, **props off**, ANGLE, app connected, Developer Mode on: idle
( expect 7 ), arm ( 8 ), tilt past 70 deg so the crash detector disarms and keep it tipped ( 4 / 1 / 0 / 0 ), right it
( 7 within about 0.3 s ). Confirm first that crash detection ( `fsCrash` ) is on, or the tilt does not disarm. Save as
`logs/log-1.txt`; analyse into `TESTING.md`; restore `PlutoPilot.cpp` to HEAD and rebuild the clean hex.

- **Depends on:** 2
- **Skills / agent:** pluto-flighttest
- **Files:** [PlutoPilot.cpp](../../../../PlutoPilot.cpp) ( temporary )
- **Safety impact:** bench only, props off; arming with props off
- **Done when:** the log shows 4 / 1 / 0 while crashed and tipped, 7 after righting, 8 when armed; `PlutoPilot.cpp`
  back to HEAD
- **Rollback:** restore `PlutoPilot.cpp`
- **Status:** done 2026-11-14
- **Result:** log-1 PASS: `S` and all three `Check` values agree with the flags on 841 / 841 records; both crash disarms read `S:4 cr:1` ( 152 records ), 7 once flat, 8 armed. `PlutoPilot.cpp` restored, clean hex rebuilt ( 122752 / 1316 / 19728 B ). Battery statuses not set in this run ( pack above warning ). [TESTING.md](TESTING.md)

### 4. Architecture & pipeline docs ( PIPELINE_UPDATE.md, wiki )

**Description.** New "Flight status" section in `docs/API/FC_CONTROL_API_WIKI.md`: the nine statuses and what sets
each, the priority order, `FS_INFLIGHT_LOW_BATTERY` = critical and latched, the about 0.3 s crash window ( poll every
loop ), `FlightStatus_Check` vs `Get`, what changed in API 1.8.4 ( values before the fix ) and an example. Staged text for
`subsystems/User_Space_API.md` into `PIPELINE_UPDATE.md` ( never the pipeline folder ); CHANGELOG and version lines for
pluto-commit. Check the wiki against the code ( spec-to-code-compliance inline; the workflow only with the user's opt-in ).

- **Depends on:** 3
- **Skills / agent:** graphify, spec-to-code-compliance
- **Files:** [FC_CONTROL_API_WIKI.md](../../../API/FC_CONTROL_API_WIKI.md), [PIPELINE_UPDATE.md](PIPELINE_UPDATE.md)
- **Safety impact:** none
- **Done when:** wiki section written and checked against the code; `PIPELINE_UPDATE.md` staged
- **Rollback:** -
- **Status:** done 2026-11-14
- **Result:** New "Flight status" section in `FC_CONTROL_API_WIKI.md` ( priority table, value is not priority, crash / battery / signal-loss / app rules, two `plutoLoop` examples, Changes in 1.8.4; wiki now 1.8.4 ); `PIPELINE_UPDATE.md` staged for `User_Space_API.md` and the commit. Checked inline against the code; one wrong sentence ( OK / not OK to arm updated only with the ARM switch off ) removed.

### 5. Topic review

**Description.** `pluto-reviewer` on the whole topic diff ( code, header, docs ). Fix BLOCKING findings and re-run the
gate.

- **Depends on:** 4
- **Skills / agent:** pluto-reviewer agent
- **Files:** the topic diff
- **Safety impact:** none
- **Done when:** no open BLOCKING finding
- **Rollback:** -
- **Status:** done 2026-11-14
- **Result:** 0 blocking, 3 should-fix, 4 nits, all fixed. Wiki corrected for real states seen in log-1: one 0xFFFF read right after a disarm with ARM on ( D4 correction, also in the header and code comment ), `FS_OK_TO_ARM` set with the link up whatever the attitude while ARM is on, `FS_SIGNAL_LOSS` cleared only disarmed with the link back. Nav bars, an ellipsis, a stale line link fixed. Gate green, size unchanged.

### 6. Graph refresh & commit

**Description.** `graphify update .` and `tools/graph_labels.py`, then `/pluto-commit`: all targets, FW 3.15.5 → 3.15.6,
API 1.8.3 → 1.8.4 ( behaviour only ), promote `PIPELINE_UPDATE.md`, CHANGELOG, stage and draft the message. The user
commits.

- **Depends on:** 5
- **Skills / agent:** graphify, pluto-commit
- **Files:** -
- **Safety impact:** none
- **Done when:** staged with a drafted message; topic Closed - pending commit
- **Rollback:** -
- **Status:** done 2026-11-14
- **Result:** Graph refreshed and labelled; all three targets built clean ( X2_v1 122752 / 1316 / 19728, PRIMUSX2 122580 / 1316 / 19884, V5 122744 / 1316 / 19728 B ); FW 3.15.6, API 1.8.4; `User_Space_API.md` promoted, CHANGELOG version line, Fixed and Documentation entries; staged with a drafted message.

## Decisions log

Newest last. One line each: `YYYY-MM-DD [decision|assumption|out-of-scope|risk] text`.

- 2026-11-05 [decision] Cause proven from the code ( enum numbering mismatch ); the user skipped the `check` bench log and went straight to `fix`.
- 2026-11-05 [decision] D1: option A, map internal to public by name in `FC-Control-Status.cpp` for both `Get` and `Check`; public numbers, internal enum, priority and MSP unchanged.
- 2026-11-05 [decision] D2: crash-clear timing unchanged ( about 0.3 s once level, `mw.cpp:489-501` ); documented, poll every loop.
- 2026-11-05 [decision] D3: critical battery reported to user code as is ( `FS_INFLIGHT_LOW_BATTERY` ); MSP's armed softening is for the app only.
- 2026-11-05 [assumption] D4: no flag set → `Get` keeps today's value ( 0xFFFF ); `Check` outside 0-8 → false. The user may correct.
- 2026-11-14 [risk] D4 correction ( topic review ): no flag set does happen, for one read right after a disarm with the ARM switch still on ( Armed cleared, `Ok_to_arm` / `Not_ok_to_arm` not yet set; log-1 `f:4` records ). Kept as decided ( 0xFFFF ), documented in the wiki and header as "not armed".
- 2026-11-05 [decision] Done when: bench log on the fixed build ( 4 / 1 / 0 while crashed and tipped, 7 righted, 8 armed ).
- 2026-11-05 [out-of-scope] Crash detection thresholds and false triggers ( `crash-stuck-detection` ); the MSP mapping.
- 2026-11-05 [decision] Docs: `FC-Control.h` comments, new flight-status section in `FC_CONTROL_API_WIKI.md`, `User_Space_API.md` via `PIPELINE_UPDATE.md`; API 1.8.3 → 1.8.4, FW 3.15.5 → 3.15.6 at commit.
- 2026-11-14 [decision] The 0xFFFF read after a disarm ships as documented ( the user moved on to the commit without the optional `FS_NOT_OK_TO_ARM` change ).
- 2026-11-14 [decision] Closed - pending commit: FW 3.15.5 -> 3.15.6 ( patch, a fix ), API 1.8.3 -> 1.8.4 ( behaviour only ); all targets built; `User_Space_API.md` and CHANGELOG updated; warnings baseline not refreshed.
