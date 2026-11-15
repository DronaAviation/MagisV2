# Failsafe_enable API - Tasks

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

## Resume

Read this block first in a new session; read only the task it points to.

| | |
|---|---|
| **Current task** | none: **Closed** in `c344387` ( 15 Nov 2026, FW 3.15.10, API 1.8.8 ) |
| **Next step** | Committed in `c344387`. The testing team re-runs T07. |
| **Open questions** | none |
| **Blocked on** | nothing |
| **Last updated** | 2026-11-15 |

## Summary

| | |
|---|---|
| **Mode** | `fix` |
| **Goal** | `Failsafe_enable ( X )` enables only `X` ( all three for `ALL` ) and leaves the other flags unchanged, as the header already specifies. |
| **Done when** | The host truth-table check passes for every enable / disable sequence from all-on and from all-off ( only the named flag changes ), the code-flow trace to both consumers is written up in `TESTING.md`, and the gate is green on `PRIMUS_X2_v1`. |
| **Target** | `PRIMUS_X2_v1` |
| **Analysis** | Cause proven by reading the code: [FC-Config-Setpoints.cpp:213-225](../../../../src/main/API-Src/FC-Config-Setpoints.cpp#L213) assigns each of the three flags `enableAll` OR `( FAILSAFE == X )`, so every call clears the two flags it was not asked about. Safety effect: clearing `fsInFlightLowBattery` ( [battery.cpp:798](../../../../src/main/sensors/battery.cpp#L798) ) removes the low-battery warning, the auto-land at critical and the arm refusal; clearing `fsCrash` ( [failsafe.cpp:365](../../../../src/main/flight/failsafe.cpp#L365) ) removes the crash disarm. Full chain: [INVESTIGATION.md](INVESTIGATION.md). |
| **Pipeline docs affected** | `fw-architecture-pipeline/subsystems/Failsafe_Subsystem.md`, `Power_BMS_Pipeline.md`, `User_Space_API.md` ( via `PIPELINE_UPDATE.md` ); `docs/API/FAILSAFE_API_WIKI.md` ( new ); `docs/API/BMS_API_WIKI.md` ( note ); `CHANGELOG.md` |
| **Order** | 1 → 2 → 3 → 4 → 7 → 5 → 6 |

## Index

| # | Title | Status | Depends on | Skills / agent |
|---|---|---|---|---|
| 1 | Correct `Failsafe_enable ( )` | done 2026-11-15 | - | pluto-rules |
| 2 | Host truth-table check and consumer flow trace | done 2026-11-15 | 1 | pluto-rules |
| 3 | Build gate | done 2026-11-15 | 1 | pluto-build |
| 4 | API wiki and pipeline docs | done 2026-11-15 | 2 | spec-to-code-compliance, graphify |
| 5 | Topic review | done 2026-11-15 | 4, 7 | pluto-reviewer agent |
| 6 | Graph refresh and commit | done 2026-11-15 | 5 | graphify, pluto-commit |
| 7 | Correct the `failsafe_e` comments in `FC-Config.h` | done 2026-11-15 | 4 | pluto-rules, pluto-build |
| - | Hardware validation | dropped ( no hardware test this topic: verification is the host check in task 2; the testing team re-runs T07 on its own procedure ) | - | - |

Status values: `todo`, `in-progress`, `done YYYY-MM-DD`, `blocked (<why>)`, `dropped (<why>)`.
Serial numbers are never reused or renumbered.

## Tasks

### 1. Correct `Failsafe_enable ( )`

**Description.** Rewrite the body of `Failsafe_enable ( failsafe_e FAILSAFE )` in
[FC-Config-Setpoints.cpp:213-225](../../../../src/main/API-Src/FC-Config-Setpoints.cpp#L213) so that it only ever sets
a flag to `true`: each of `fsLowBattery`, `fsInFlightLowBattery` and `fsCrash` is set when it is named or when `ALL` is
passed, and is otherwise left untouched. The scout's form is the one to use
( `if ( enableAll || FAILSAFE == LOW_BATTERY ) fsLowBattery = true;` and so on ); a self-OR assignment is equivalent
but reads worse. `Failsafe_disable` is **not** touched: it already keeps the flags it was not asked to change, and the
decision was the smallest diff. Keep the comment block above the function and the spaced-paren style, and correct the
per-line comments so they describe a set rather than an assignment. No signature or header change.

- **Depends on:** -
- **Skills / agent:** `pluto-rules`
- **Files:** [API-Src/FC-Config-Setpoints.cpp](../../../../src/main/API-Src/FC-Config-Setpoints.cpp)
- **Safety impact:** Restores the low-battery warning, the auto-land at critical battery and the arm refusal for user code that enables any other failsafe, and restores the crash disarm for user code that enables a battery failsafe. No behaviour change for `Failsafe_enable ( ALL )`, for code that never calls the function ( all three flags start `true` ), or for `Failsafe_disable`.
- **Done when:** The function contains no assignment that can write `false`, the firmware compiles for `PRIMUS_X2_v1`, and the behaviour matches the header doc at FC-Config.h:267-276.
- **Rollback:** Revert the one function body; the flags default to `true`, so the pre-fix build is the current released behaviour.
- **Status:** done 2026-11-15
- **Result:** Three `if ( enableAll || ( FAILSAFE == X ) ) { fsX = true; }` sets at :224 / :229 / :234, so no path writes `false`; comments corrected, `Failsafe_disable` untouched. Gate green on `PRIMUS_X2_v1`, no new `src/` warnings, flash 121.5 KB / 256 KB, RAM 20.5 KB / 40 KB. Reviewer: 0 findings; it also confirmed both readers run in the main loop ( `BMS_Update` at mw.cpp:464, `failsafeOnCrash` at mw.cpp:1536 ), so no `volatile` or critical section is needed, and that nothing in `src/`, `PlutoPilot.cpp` or `docs/API` relied on the old clear-the-others behaviour.

### 2. Host truth-table check and consumer flow trace

**Description.** Prove the fix without hardware, in two parts. **( a )** A host check: compile the **real**
`src/main/API-Src/FC-Config-Setpoints.cpp` for the PC against the existing
[tools/sim/stub/](../../../../tools/sim/stub) header tree ( the same arrangement as
[tools/sim/run.sh](../../../../tools/sim/run.sh) from the `set-default-pid-profile` topic, which compiled the real
`FC-Config-PID.cpp` ), with a harness that defines whatever symbols the file references, and drive it through every
sequence: each of the four `failsafe_e` values to `enable` and to `disable`, from all-three-on and from
all-three-off, plus the pairs T07 exercises ( `disable ( ALL )` then `enable ( CRASH )`, and the reverse ). The check
asserts that only the named flag, or all three for `ALL`, changes value, and that the pre-fix body fails the same
check. The harness and its runner stay in the **scratchpad**: nothing is added to the repo ( decision of
15 Nov 2026 ). **( b )** A written trace from each flag to its consumer: `fsInFlightLowBattery` to the
`BATTERY_OK` to `BATTERY_WARNING` step at battery.cpp:798 and on to `batteryCriticalConfirmed ( )`, the auto-land at
mw.cpp:1306 and the arm refusal at mw.cpp:625; `fsCrash` to `failsafeOnCrash ( )` at failsafe.cpp:365; `fsLowBattery`
to nothing ( reader deleted in `88f594f` ). Note the gate's one-way nature: clearing a flag after the battery level
has already risen does not step it back. Write both parts, with the command used and its output, into `TESTING.md`.

- **Depends on:** 1
- **Skills / agent:** `pluto-rules`
- **Files:** `TESTING.md`; harness under the session scratchpad; reads [tools/sim/run.sh](../../../../tools/sim/run.sh), [sensors/battery.cpp](../../../../src/main/sensors/battery.cpp), [flight/failsafe.cpp](../../../../src/main/flight/failsafe.cpp), [mw.cpp](../../../../src/main/mw.cpp)
- **Safety impact:** none ( host only; no firmware file changes )
- **Done when:** The check prints a PASS for every sequence with the fixed file and a FAIL with the pre-fix body, and `TESTING.md` holds the truth table, the trace and the exact commands. If `FC-Config-Setpoints.cpp` cannot be made to compile against the stubs within reasonable effort, say so in `TESTING.md` and fall back to a copy of the two function bodies, marked as a copy.
- **Rollback:** n/a ( no firmware change )
- **Status:** done 2026-11-15
- **Result:** The real `FC-Config-Setpoints.cpp` compiled for the PC ( MinGW g++ 14.3.0, no warnings ) behind a scratchpad declaration overlay ahead of `tools/sim/stub`, so no fallback copy was needed. Fixed file PASS, 4164 checks ( 64 single calls from every state, 4096 three-call sequences, 4 named T07 scenarios ); pre-fix `HEAD` file FAIL 1468 / 4164, reproducing T07 ( `1 1 1` then `enable ( CRASH )` gave `0 0 1` ). Trace confirms `fsInFlightLowBattery` is the only way out of `BATTERY_OK` ( `batteryApplyLevel ( )` runs only after a gated step ), so the pre-fix bug removed the whole low-battery chain. Details: [TESTING.md](TESTING.md).

### 3. Build gate

**Description.** Run `.claude/skills/pluto-build/driver.sh --gate PRIMUS_X2_v1` and confirm no new warning under
`src/` against `tools/warnings-baseline.json`, and that flash and RAM still fit. The change is a few lines in one
function, so the size difference should be nil or a handful of bytes; record both numbers against `HEAD`. Never
regenerate the baseline.

- **Depends on:** 1
- **Skills / agent:** `pluto-build`
- **Files:** -
- **Safety impact:** none
- **Done when:** Gate green on `PRIMUS_X2_v1`, no new `src/` warnings, flash and RAM deltas recorded in `TESTING.md`.
- **Rollback:** n/a
- **Status:** done 2026-11-15
- **Result:** Clean gate green ( no new `src/` warnings ). The ELF is byte-identical in size to a clean build of `HEAD` exported to the scratchpad ( text 123108 / data 1316 / bss 19760; 121.5 KB flash, 20.5 KB RAM ): nothing calls `Failsafe_enable`, so `-gc-sections` drops it from both. In the object file the function shrank from 60 to 56 B, and the disassembly shows every store writes 1 ( `HEAD` stored literal zeros ). Details: [TESTING.md](TESTING.md) ( c ).

### 4. API wiki and pipeline docs

**Description.** Document the corrected semantics and the inert `LOW_BATTERY`. Add a `Failsafe_enable` /
`Failsafe_disable` section to [docs/API/FC_CONFIG_API_WIKI.md](../../../API/FC_CONFIG_API_WIKI.md), which has none
today: the enum values, "only the named failsafe changes; `ALL` changes all three", the flags' default of enabled, that
they survive disarm and Developer Mode off and reset only at reboot, what each one actually gates, and a clear note
that `LOW_BATTERY` has had no reader since `88f594f` and is kept for compatibility. Add one line to
[BMS_API_WIKI.md](../../../API/BMS_API_WIKI.md) near :55-56 saying the same about `LOW_BATTERY`, so a reader of the
battery wiki does not expect it to gate the warning. Stage the new text for
`fw-architecture-pipeline/subsystems/Failsafe_Subsystem.md` in `PIPELINE_UPDATE.md` ( never edit the pipeline folder
directly ): the enable / disable semantics at :85 and the stale citation of `failsafe.cpp:362` ( the function is at
:363, the `fsCrash` check at :365 ). Check the header doc at FC-Config.h:267-276 against the new implementation with
`spec-to-code-compliance`. Confirm any flow statement with `graphify path` or a source line before writing it. Follow
the wiki convention of naming the hook an example belongs in: `Failsafe_enable` goes in `plutoInit ( )`.

- **Depends on:** 2
- **Skills / agent:** `spec-to-code-compliance`, `graphify`
- **Files:** [docs/API/FC_CONFIG_API_WIKI.md](../../../API/FC_CONFIG_API_WIKI.md), [docs/API/BMS_API_WIKI.md](../../../API/BMS_API_WIKI.md), `PIPELINE_UPDATE.md`
- **Safety impact:** none ( documentation )
- **Done when:** The wiki section exists and matches the implementation line for line, the BMS note is in, `PIPELINE_UPDATE.md` holds the full new text of the affected `Failsafe_Subsystem.md` passages, and `spec-to-code-compliance` reports no drift between the FC-Config.h header docs and the implementation.
- **Rollback:** n/a
- **Status:** done 2026-11-15
- **Result:** New page [docs/API/FAILSAFE_API_WIKI.md](../../../API/FAILSAFE_API_WIKI.md) ( API 1.8.8 ), not a section in `FC_CONFIG_API_WIKI.md`, which is the Stick Rate page scoped to two functions. It covers the values, the behaviour, three examples in their hooks, the pitfalls ( `LOW_BATTERY` inert ) and Changes in 1.8.8. Linked from a new sentence in `BMS_API_WIKI.md`. `PIPELINE_UPDATE.md` stages `Failsafe_Subsystem.md` ( a new Key Data Structures bullet; the crash paragraph with `failsafe.cpp:363` / `:365` ), `Power_BMS_Pipeline.md` step 7 and a `User_Space_API.md` bullet. Header check done by hand ( the `spec-to-code-compliance` skill is not available in this session ): the function doc blocks match; drift in the `failsafe_e` value comments ( FC-Config.h:45-48 ) became task 7.

### 5. Topic review

**Description.** Run the `pluto-reviewer` agent on the whole topic diff ( the one function body plus the docs ), with
the `pluto-rules` checklist. Points to put to it explicitly: that no path can now clear a flag unintentionally, that
`Failsafe_disable` was deliberately left as it is and is still correct under the precedence of `||` over the ternary,
that the strict warning set is clean on plain `bool` sets, that the wiki matches the header, and whether the inert
`LOW_BATTERY` flag should instead have been removed from the enum ( it must not be: that would renumber a public enum
and break user code ). Fix every BLOCKING finding and re-run task 3's gate before moving on.

- **Depends on:** 4, 7
- **Skills / agent:** `pluto-reviewer` agent ( always a subagent )
- **Files:** the topic diff
- **Safety impact:** none directly; the review is the last check on a change to the failsafe gates
- **Done when:** No BLOCKING findings remain; each should-fix is either applied or logged in the decisions log with a reason, and the gate is green after any fix.
- **Rollback:** n/a
- **Status:** done 2026-11-15
- **Result:** Reviewer over all 13 files: **no BLOCKING findings**, 1 should-fix and 4 nits, all five applied. It confirmed the five points put to it: no path in `Failsafe_enable` can clear a flag; `Failsafe_disable` is still correct; the strict warnings have nothing to catch on plain `bool` sets; every behavioural claim in the new wiki checks out against the code ( the debounce timers keep running while the flag is off, so switching it back on raises the level at the next 21 ms update; the WARNING to CRITICAL step has no guard, so the critical landing still happens ); and `LOW_BATTERY` must stay in the enum. It also verified every `PIPELINE_UPDATE.md` replace block against the current pipeline text and every insert anchor. Gate re-run green after the fixes, ELF still 123108 / 1316 / 19760.

### 6. Graph refresh and commit

**Description.** After the review fixes: `graphify update .` then `python tools/graph_labels.py` ( the graph was stale
at `73022c32` when the scout ran, so this refresh also catches the sibling topics' commits ). Then `/pluto-commit`:
all-target build ( `PRIMUS_X2_v1`, `PRIMUSX2`, `PRIMUS_V5` ), `API_Version` 1.8.7 to 1.8.8 and `FW_Version` 3.15.9 to
3.15.10 in the Makefile, promotion of `PIPELINE_UPDATE.md` into `Failsafe_Subsystem.md`, a `CHANGELOG.md` entry, the
topic README set to Closed - pending commit, and a drafted message. Add the one-line CLAUDE.md note for the new
gotcha: `Failsafe_enable ( X )` only ever enables, and `LOW_BATTERY` is inert.
**Never run `git commit` or `git push`.**

- **Depends on:** 5
- **Skills / agent:** `graphify`, `pluto-commit`
- **Files:** `Makefile`, `CHANGELOG.md`, `../../fw-architecture-pipeline/subsystems/Failsafe_Subsystem.md`, `../../../../CLAUDE.md`, `README.md`
- **Safety impact:** none ( process )
- **Done when:** All three targets build with no new `src/` warnings, versions bumped, pipeline doc and CHANGELOG updated, the change staged with the drafted message, and the user told it is ready to commit.
- **Rollback:** n/a
- **Status:** done 2026-11-15
- **Result:** Graph refreshed ( 5119 nodes, 230 named communities; `graph_labels.py` needed the `-I` workaround of tooling backlog #13, now noted there ). All three targets clean-built at 3.15.10 with no warnings in the touched files: PRIMUS_X2_v1 121.5 KB / 20.5 KB, PRIMUSX2 121.3 KB / 20.7 KB, PRIMUS_V5 121.5 KB / 20.5 KB. Makefile FW 3.15.10 / API 1.8.8; three pipeline docs promoted; CHANGELOG ( version line, Fixed, Documentation ) and a CLAUDE.md flight invariant added. `set-desired-rate-api` recorded as closed in `3be9093`. Staged with the drafted message; the user commits.

### 7. Correct the `failsafe_e` comments in `FC-Config.h`

**Description.** Found in task 4's header check. The per-value comments of `failsafe_e` at
[API/FC-Config.h:45-48](../../../../src/main/API/FC-Config.h#L45) disagree with the firmware, and they are what a user
reads first ( `PlutoPilot.h` brings the header in ):

- `LOW_BATTERY` says "triggering actions to prevent power loss", but nothing reads its flag.
- `INFLIGHT_LOW_BATTERY` says "specifically when in flight", but it gates every battery level, on the ground too, and
  the critical landing.
- `ALL` says "enable or check", but there is no check function.

Rewrite the three comments to match [FAILSAFE_API_WIKI.md](../../../API/FAILSAFE_API_WIKI.md); the `CRASH` comment can
name the conditions ( armed, ANGLE mode, not in a flip ). Comment-only: no enumerator, value or signature changes. Keep
the header's banner and style.

- **Depends on:** 4
- **Skills / agent:** `pluto-rules`, `pluto-build`
- **Files:** [API/FC-Config.h](../../../../src/main/API/FC-Config.h)
- **Safety impact:** none ( comments; the generated code must be unchanged )
- **Done when:** The four comments match the wiki and the code; the gate is green on `PRIMUS_X2_v1` with the ELF the same size as task 3 ( text 123108 / data 1316 / bss 19760 ); a `pluto-reviewer` pass has no BLOCKING findings.
- **Rollback:** Revert the comment lines.
- **Status:** done 2026-11-15
- **Result:** The four `failsafe_e` comments rewritten to match the firmware and the wiki, plus two HISTORY rows in the banner ( all eight rows 84 columns, file still pure ASCII ). Gate green, ELF text 123108 / data 1316 / bss 19760, identical to task 3. Reviewer: no BLOCKING or should-fix findings; it verified each comment against the source ( `fsLowBattery` has no reader; `battery.cpp:798` is the only way out of `BATTERY_OK` and the count conditions do not test `armed`, so a level can be raised on the ground; `ENABLE_ACROBAT` is defined on all three targets, so "not in a flip" holds everywhere ). Both NITs applied: the task tag is now `( T07 )` and the long comment is shortened.

## Decisions log

Newest last. One line each: `YYYY-MM-DD [decision|assumption|out-of-scope|risk] text`.

- 2026-11-15 [decision] Mode `fix`, not `check`: the cause is proven by reading FC-Config-Setpoints.cpp:213-225, so no investigation phase is needed.
- 2026-11-15 [decision] Working target `PRIMUS_X2_v1` ( `selected_target` in `plutoide.ini` ); all targets at commit.
- 2026-11-15 [decision] Fix only `Failsafe_enable`; `Failsafe_disable` is correct and is left untouched for the smallest diff.
- 2026-11-15 [decision] No hardware test from us. The hardware-validation closing task is dropped and replaced by the host truth-table check and the consumer flow trace in task 2; the testing team re-runs T07 on its own procedure.
- 2026-11-15 [decision] The host harness is not committed: it lives in the session scratchpad and reuses the in-repo `tools/sim/stub/` tree, so the repo gains no files. ( The `set-default-pid-profile` topic did commit its harness under `tools/sim/`; that precedent was declined here. )
- 2026-11-15 [decision] `LOW_BATTERY` keeps the corrected enable / disable semantics but is given no new function: it is documented in the API wiki and the BMS wiki as inert since `88f594f`, kept for compatibility.
- 2026-11-15 [out-of-scope] Wiring `LOW_BATTERY` to a real path ( for instance the warning level and beeper, with `INFLIGHT_LOW_BATTERY` left to gate critical and the auto-land ). It is a battery-safety design decision and needs its own topic.
- 2026-11-15 [out-of-scope] Removing `LOW_BATTERY` from `failsafe_e`: it would renumber a public enum and break user code.
- 2026-11-15 [out-of-scope] The empty `active-development/fc-config-api-fixes/` folder is left alone, as `set-desired-angle-roll-pitch/TASKS.md:217` decided.
- 2026-11-15 [decision] `API_Version` 1.8.7 to 1.8.8 and `FW_Version` 3.15.9 to 3.15.10 at commit, matching the sibling API fix topics ( a documented behaviour change ).
- 2026-11-15 [risk] The gate on `fsInFlightLowBattery` guards only the `BATTERY_OK` to `BATTERY_WARNING` step, so clearing it after the level has risen changes nothing. Any T07 re-run must call the API before the pack falls, or it will not see the defect even on the pre-fix build.
- 2026-11-15 [decision] No `volatile` or `ATOMIC_BLOCK` on the three flags: the reviewer confirmed both readers run in the main loop ( `BMS_Update` at mw.cpp:464, `failsafeOnCrash` at mw.cpp:1536 ), as do the API writers, and each write is a single `bool` store.
- 2026-11-15 [risk] `FC-Config-Setpoints.cpp` may not compile against `tools/sim/stub` without added stub symbols. Fallback, logged in `TESTING.md` if used: test a copy of the two function bodies and mark it as a copy.
- 2026-11-15 [decision] Risk above resolved without the fallback: missing declarations went into a scratchpad overlay placed ahead of `tools/sim/stub` on the include path ( types copied from the real headers ), so the real file was tested and the in-repo stubs stayed unchanged.
- 2026-11-15 [decision] The failsafe wiki is a new page, `docs/API/FAILSAFE_API_WIKI.md`, not a section in `FC_CONFIG_API_WIKI.md` as task 4 first said: that page is the Stick Rate page and says it covers only `setDesiredRate` / `getDesiredRate`. One page per feature matches the other wikis ( Yaw Heading, RC Interface ). `BMS_API_WIKI.md` links to it.
- 2026-11-15 [decision] `PIPELINE_UPDATE.md` also stages `Power_BMS_Pipeline.md` step 7 ( it names the gate without the flag or the inert `LOW_BATTERY` ) and a `User_Space_API.md` bullet ( it lists every API family with its wiki ). Both are small scope additions to task 4.
- 2026-11-15 [assumption] The `spec-to-code-compliance` skill is not available in this session, so the header-vs-code check was done by hand: the `Failsafe_enable` / `Failsafe_disable` doc blocks ( FC-Config.h:268-290 ) match the implementation ( proven over 4164 cases in task 2 ).
- 2026-11-15 [decision] New task 7 ( before the topic review ): the `failsafe_e` value comments at FC-Config.h:45-48 are wrong ( `LOW_BATTERY` "triggering actions", `INFLIGHT_LOW_BATTERY` "specifically when in flight", `ALL` "or check" ). Comment-only, but it touches `src/main`, so it gets its own gate and review.
- 2026-11-15 [decision] Task 7 reviewer NITs applied: the banner task tag reads `( T07 )` to match the row above, and the `INFLIGHT_LOW_BATTERY` comment was shortened from 159 to 119 characters ( `.clang-format` sets `ColumnLimit: 0`, so this is style, not a rule ).
- 2026-11-15 [decision] Topic review fixes: stale line citations updated after the header grew by 2 lines ( `FC-Config.h` declaration 278 to 280, enum 44-49 to 46-51, doc block 267-276 to 270-279 / 267-290 to 270-292; `failsafe.cpp` 363-385 to 363-384 ). The pre-fix code quoted in `INVESTIGATION.md` is now labelled "at `3be9093`" instead of being renumbered, since it shows the old body. `SCOUT.md` and the task sections keep their planning snapshots.
- 2026-11-15 [decision] Review nits applied: the banner HISTORY row no longer opens a parenthesis it does not close; the wiki hook comments say `onLoopStart` / `onLoopFinish` also run on a link drop ( `FC-Config.h:158`, `mw.cpp:1177`, `:1224-1232` ); and the `plutoInit` example no longer implies earlier code could have switched a failsafe off ( all three are on at power-up and only `plutoRxConfig` runs before `plutoInit` ).
- 2026-11-15 [decision] Closed - pending commit: FW 3.15.9 -> 3.15.10 ( patch, a fix ), API 1.8.7 -> 1.8.8 ( behaviour only ), as agreed in planning. The Makefile was bumped before the all-target build, so the build covers the committed version strings. Not hardware-tested: the testing team re-runs T07.
