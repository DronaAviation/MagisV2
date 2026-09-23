# <Topic title> - Tasks

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [DESIGN](DESIGN.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

( Keep only the links that exist: INVESTIGATION for check / fix, DESIGN for feature; CHANGES and TESTING appear when `/pluto-task` first writes them. )

## Resume

Read this block first in a new session; read only the task it points to.

| | |
|---|---|
| **Current task** | <n - title, or "none: run /pluto-task next"> |
| **Next step** | <one line: what happens next inside that task> |
| **Open questions** | <for the user, or "none"> |
| **Blocked on** | <hardware, a flight, a decision, or "nothing"> |
| **Last updated** | YYYY-MM-DD |

## Summary

| | |
|---|---|
| **Mode** | <check / fix / feature / improve / docs> |
| **Goal** | <one sentence> |
| **Done when** | <observable result: log field, behaviour, number> |
| **Target** | <PRIMUS_V5 / PRIMUS_X2_v1> |
| **Analysis** | <ranked causes / chosen option ( DESIGN.md ) / baseline numbers / drift list> |
| **Pipeline docs affected** | <e.g. subsystems/Altitude_Hold_Estimator.md, or "none"> |
| **Order** | 1 → 2 → 3 … ( update when tasks are added ) |

## Index

| # | Title | Status | Depends on | Skills / agent |
|---|---|---|---|---|
| 1 | <title> | todo | - | pluto-rules |
| 2 | … | todo | 1 | … |
| n-4 | Build gate | todo | code tasks | pluto-build |
| n-3 | Hardware validation | todo | n-4 | pluto-flighttest |
| n-2 | Architecture & pipeline docs ( PIPELINE_UPDATE.md ) | todo | n-3 | graphify, spec-to-code-compliance |
| n-1 | Topic review | todo | n-2 | pluto-reviewer agent |
| n | Graph refresh & commit | todo | n-1 | graphify, pluto-commit |

Status values: `todo`, `in-progress`, `done YYYY-MM-DD`, `blocked (<why>)`, `dropped (<why>)`.
Serial numbers are never reused or renumbered.

## Tasks

### 1. <Title>

**Description.** <2-5 sentences: what changes, why, where ( files / functions ), and which decisions it relies on.>

- **Depends on:** <task numbers, or "-">
- **Skills / agent:** <skills to load; agent to delegate to, if any>
- **Files:** [<file>](../../../../src/main/<path>)
- **Safety impact:** <flight modes / failsafe / arming touched, or "none">
- **Done when:** <check that proves it: builds clean, log shows X, value Y>
- **Rollback:** <how to undo if it misbehaves in flight; code tasks only>
- **Status:** todo
- **Result:** <one line, filled in when done>

### 2. …

## Decisions log

Newest last. One line each: `YYYY-MM-DD [decision|assumption|out-of-scope|risk] text`.

- YYYY-MM-DD [decision] …
