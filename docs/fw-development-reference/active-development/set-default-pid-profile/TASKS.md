# Set default PID profile - Tasks

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

## Resume

Read this block first in a new session; read only the task it points to.

| | |
|---|---|
| **Current task** | none: **Closed - pending commit** ( 14 Nov 2026 ) |
| **Next step** | The user commits with `git commit -F .git/MAGISV2_COMMIT_MSG.txt`; the next `/pluto-commit` or `/pluto-grill` fills in the hash. |
| **Open questions** | none |
| **Blocked on** | nothing |
| **Last updated** | 2026-11-14 |

## Summary

| | |
|---|---|
| **Mode** | `fix` |
| **Goal** | `setDefault_PIDProfile ( )` restores the firmware default gains for the five API slots and touches nothing else. |
| **Done when** | The host simulation in `tools/sim/` shows `PIDROLL` / `PIDPITCH` 40 / 10 / 30, `PIDYAW` 150 / 70 / 50, `PIDALT` 128 / 0 / 30, `PIDUSER` 60 / 30 / 5, the six hidden slots still holding their marker values, and no sanitizer report; the `PRIMUS_X2_v1` gate is clean and the `-Waggressive-loop-optimizations` baseline entry for `FC-Config-PID.cpp` is gone. |
| **Target** | `PRIMUS_X2_v1` ( all targets at commit ) |
| **Analysis** | Causal chain proven in source: the loop in [FC-Config-PID.cpp](../../../../src/main/API-Src/FC-Config-PID.cpp) ( `setDefault_PIDProfile ( )`, line 158 at HEAD ) runs 0 .. 10 ( `PIDUSER` = 10 ) over five-entry tables. Index 3 is a stale ALT P8 = 100 against 128 in [config.cpp:191](../../../../src/main/config/config.cpp#L191); indexes 4 .. 10 read past the tables ( undefined behaviour, already baselined as a GCC warning ) and overwrite `PIDPOS`, `PIDPOSR`, `PIDNAVR`, `PIDLEVEL`, `PIDMAG`, `PIDVEL`, `PIDUSER`. See [INVESTIGATION.md](INVESTIGATION.md). |
| **Pipeline docs affected** | `subsystems/Core_PID_Controller.md` ( new section plus a line reference ), `subsystems/Mixer_Pipeline.md` and `subsystems/User_Space_API.md` ( one shifted `config.cpp` line reference each ) - all three staged in `PIPELINE_UPDATE.md` |
| **Order** | 1 → 2 → 3 → 4 → 5 ( dropped ) → 6 → 7 → 8 |

## Index

| # | Title | Status | Depends on | Skills / agent |
|---|---|---|---|---|
| 1 | Shared default-gain helper | done 2026-11-14 | - | pluto-rules |
| 2 | Rewrite `setDefault_PIDProfile ( )` and its header comment | done 2026-11-14 | 1 | pluto-rules |
| 3 | Host simulation in `tools/sim/` | done 2026-11-14 | 2 | - |
| 4 | Build gate | done 2026-11-14 | 2 | pluto-build |
| 5 | Hardware validation | dropped ( replaced by task 3; the user declined bench and flight logging ) | 4 | pluto-flighttest |
| 6 | Pipeline docs ( `PIPELINE_UPDATE.md` ) | done 2026-11-14 | 3, 4 | spec-to-code-compliance |
| 7 | Topic review | done 2026-11-14 | 6 | pluto-reviewer agent |
| 8 | Graph refresh & commit | done 2026-11-14 | 7 | graphify, pluto-commit |

Status values: `todo`, `in-progress`, `done YYYY-MM-DD`, `blocked (<why>)`, `dropped (<why>)`.
Serial numbers are never reused or renumbered.

## Tasks

### 1. Shared default-gain helper

**Description.** Move only the `pidProfile->P8 / I8 / D8` assignments out of the `static` `resetPidProfile ( )` in
[config.cpp:175-233](../../../../src/main/config/config.cpp#L175) into a new
`void pidProfileDefaultGains ( pidProfile_t *pidProfile )`, and call it from `resetPidProfile ( )` so the committed
defaults are unchanged. Everything else `resetPidProfile ( )` does ( `pidController`, `yaw_p_limit`, the cut
frequencies, `P_f` / `I_f` / `D_f`, `A_level` / `H_level` / `H_sensitivity`, `pid5_oldyw`, the GTUNE limits ) stays
where it is, because an API gain reset must not touch it. Put the helper in a new
`src/main/flight/pid_defaults.cpp` declared in `flight/pid.h`, so the host harness of task 3 can compile it without
`config.cpp` and its hardware headers; fall back to declaring it in `config/config.h` only if `pid.h` turns out to pull
in too much, and record which was used. Register the new file in the `MAIN_FLIGHT` group of the
[Makefile](../../../../Makefile) ( line 175 ): there is no source glob. Keep the comments that carry the reasoning for
the current values, notably the `PIDALT` P8 = 128 unity note.

- **Depends on:** -
- **Skills / agent:** `pluto-rules`
- **Files:** [config/config.cpp](../../../../src/main/config/config.cpp), [flight/pid.h](../../../../src/main/flight/pid.h), new `src/main/flight/pid_defaults.cpp`, [Makefile](../../../../Makefile)
- **Safety impact:** None intended: a pure move of the default-gain block. These are the values every fresh EEPROM gets, so a typo would mistune every axis on a config reset. Diff the moved block line by line against the original.
- **Done when:** The target builds, and the moved block is value-identical to the original ( `git diff` shows a move, not an edit ); `resetPidProfile ( )` still sets every field it set before.
- **Rollback:** Revert the three files and drop the Makefile line; nothing else depends on the helper yet.
- **Status:** done 2026-11-14
- **Result:** `pidProfileDefaultGains ( )` in new [flight/pid_defaults.cpp](../../../../src/main/flight/pid_defaults.cpp), declared in [pid.h:111](../../../../src/main/flight/pid.h#L111), called from `resetPidProfile ( )`, registered in `MAIN_FLIGHT`. All 31 assignments diffed identical to HEAD and in the same order ( reviewer re-checked independently ); gate green on `PRIMUS_X2_v1`, Flash 121.1 KB / 256 KB, RAM 20.5 KB / 40 KB. Reviewer: 0 blocking, 0 should-fix, 1 nit ( the "shared with `setDefault_PIDProfile ( )`" comments are true only once task 2 lands; left as is ).

### 2. Rewrite `setDefault_PIDProfile ( )` and its header comment

**Description.** Replace the body of `setDefault_PIDProfile ( )` in
[FC-Config-PID.cpp:147-168](../../../../src/main/API-Src/FC-Config-PID.cpp#L147): fill a stack `pidProfile_t` with
`pidProfileDefaultGains ( )` and copy `P8` / `I8` / `D8` for `PIDROLL`, `PIDPITCH`, `PIDYAW`, `PIDALT` and `PIDUSER`
only, then delete the three local tables and the 0 .. 10 loop. This ends the out-of-bounds write, the stale ALT 100 and
the baselined `-Waggressive-loop-optimizations` warning in one go. Mind the `-Wconversion` rule: `P8` and friends are
`uint8_t`, so the copy must not introduce an implicit narrowing warning. Then correct the comment at
[FC-Config.h:73-76](../../../../src/main/API/FC-Config.h#L73), which today claims "all axes": it must name the five
slots it restores, say that the hidden gains ( `PIDLEVEL`, `PIDVEL`, `PIDPOS`, `PIDPOSR`, `PIDNAVR`, `PIDMAG` ) are left
alone, say the change is RAM only and is not saved to EEPROM, and warn that gains are read live every loop so calling it
while armed steps the tuning mid-flight. Note in the comment that with `pidController == 2` the roll / pitch / yaw
float gains are the ones in use, so the restored `P8` values do not apply on those axes. Public API behaviour changes,
so the API version bumps at commit ( task 8 ).

- **Depends on:** 1
- **Skills / agent:** `pluto-rules`
- **Files:** [API-Src/FC-Config-PID.cpp](../../../../src/main/API-Src/FC-Config-PID.cpp), [API/FC-Config.h](../../../../src/main/API/FC-Config.h)
- **Safety impact:** This is the fix for a function that silently retuned the angle and altitude-hold loops. No caller exists in the tree today, so the change cannot affect a flight until user code calls it. Keep the signature unchanged ( public header stability ).
- **Done when:** The function body contains no table and no loop over `pidIndex_e`; the five slots come from `pidProfileDefaultGains ( )`; the target builds clean; the header comment states the five slots, RAM only, the armed caveat and the `pidController == 2` caveat.
- **Rollback:** Revert both files; the function returns to its current ( buggy ) behaviour and the two baseline warnings reappear.
- **Status:** done 2026-11-14
- **Result:** Tables and the 0 .. 10 loop gone; a zero-initialised stack `pidProfile_t` is filled by `pidProfileDefaultGains ( )` and the five `apiSlot` entries are copied ( `uint8_t` to `uint8_t`, no narrowing ). Header comment rewritten with the five values, the untouched internal gains, RAM only, the armed caveat and the LuxFloat caveat. Gate green, +24 B flash ( 122776 B text ), RAM unchanged; the `-Waggressive-loop-optimizations` x1 and `-Wconversion` x3 entries for this file are now reported fixed ( baseline refreshed in task 4 ). Reviewer: 0 blocking, 0 should-fix, 2 nits, both fixed ( the save paths are named, HISTORY rows added ) and the gate re-run.

### 3. Host simulation in `tools/sim/`

**Description.** The user asked for a simulated test instead of a bench check, so this task is the topic's evidence.
Add a small, re-runnable host harness under `tools/sim/` ( a `.cpp` with the checks plus a one-line build or `run.sh` /
`run.ps1` recipe ) that compiles **the real** `setDefault_PIDProfile ( )` and the real `pid_defaults.cpp` against a
minimal stub for `currentProfile` and whatever `pidProfile_t` needs, with no hardware headers. The test sets all eleven
`P8` / `I8` / `D8` entries to a marker value ( for example `0xAA` ), calls `setDefault_PIDProfile ( )`, and asserts
`PIDROLL` and `PIDPITCH` 40 / 10 / 30, `PIDYAW` 150 / 70 / 50, `PIDALT` 128 / 0 / 30, `PIDUSER` 60 / 30 / 5, and that
`PIDPOS`, `PIDPOSR`, `PIDNAVR`, `PIDLEVEL`, `PIDMAG`, `PIDVEL` still hold `0xAA`. Build it with
`-fsanitize=address,undefined` where the host toolchain supports it ( MinGW `g++` 14.3 and clang 18 are both on this
machine; ASan on MinGW may not work, in which case use clang or drop to `-Wall -Wextra` and a bounds-checked stub ) so a
future out-of-bounds write is caught rather than inferred. Keep the stub honest: a real `pidProfile_t`, real
`pidIndex_e`, and no copy of the expected values other than the literals this task's checks assert. This is **not** the
abandoned `src/test/` GoogleTest suite and must not depend on it. Record the command and its output in `TESTING.md`.

- **Depends on:** 2
- **Skills / agent:** -
- **Files:** new `tools/sim/` files, `TESTING.md`
- **Safety impact:** None: host-only, not compiled into the firmware. Nothing under `tools/` may be added to a Makefile source group.
- **Done when:** The harness builds and passes on the host, output pasted into `TESTING.md`; the six hidden slots read `0xAA` after the call; no sanitizer diagnostics ( or a note saying which sanitizer was unavailable and what replaced it ).
- **Rollback:** Delete `tools/sim/`; no firmware impact.
- **Status:** done 2026-11-14
- **Result:** `tools/sim/` holds `run.sh`, `test_set_default_pid.cpp` and 22 stub headers; it builds the real `FC-Config-PID.cpp`, `pid_defaults.cpp` and `maths.cpp` on the host. Fixed code: `PASS: 103 checks, 0 failed`, exit 0, under clang++ `-fsanitize=address,undefined -fno-sanitize-recover=all` and under g++. Control with HEAD's file: UBSan aborts at `index 5 out of bounds for type 'const int[5]'` ( line 160 ), and under g++ `FAIL: 58/103` with `PID_ALT` 100 / 0 / 30 and all six hidden slots overwritten. Reviewer ( who re-ran it ): 0 blocking, 1 should-fix and 3 nits, all fixed ( no-recover plus a stderr check, `memcpy` snapshots, build-warning output, corrected `TESTING.md` quote ). Evidence: [TESTING.md](TESTING.md).

### 4. Build gate

**Description.** Run `.claude/skills/pluto-build/driver.sh --gate PRIMUS_X2_v1` ( pass `PROJECT=` from `plutoide.ini`
if a flashable hex is wanted ) and confirm no new `src/` warning against `tools/warnings-baseline.json`. Two baselined
entries for `FC-Config-PID.cpp` should now be **gone**: the `-Waggressive-loop-optimizations iteration 5 invokes
undefined behavior` ( 1 ) and some or all of the three `-Wconversion int to uint8_t` ( the file also has an unrelated
`-Wfloat-conversion` entry that must stay ). Remove only the entries this fix genuinely eliminated; never regenerate the
baseline to silence a new warning. Record flash / RAM from `make TARGET=PRIMUS_X2_v1 memory` against HEAD: the expected
change is a few bytes of flash and no RAM.

- **Depends on:** 2
- **Skills / agent:** `pluto-build`
- **Files:** [tools/warnings-baseline.json](../../../../tools/warnings-baseline.json)
- **Safety impact:** none
- **Done when:** Gate green on `PRIMUS_X2_v1`; the `-Waggressive-loop-optimizations` entry for `FC-Config-PID.cpp` is removed from the baseline because it no longer occurs; flash / RAM recorded.
- **Rollback:** Restore the baseline file from HEAD.
- **Status:** done 2026-11-14
- **Result:** Gate green on `PRIMUS_X2_v1` ( clean build ), no new `src/` warnings. Baseline edited, not regenerated: the `-Waggressive-loop-optimizations` x1 and `-Wconversion` int to uint8_t x3 entries for `FC-Config-PID.cpp` removed, `total` 2662 to 2658, a three-line diff; `warnings.py check` exits 0 and no longer lists the file. 122776 / 1316 / 19728 B: Flash 121.1 KB / 256 KB, RAM 20.5 KB / 40 KB; +24 B flash against HEAD, RAM unchanged. The 109 other "fixed" entries from earlier commits are left alone.

### 5. Hardware validation

**Description.** Dropped. The user chose a host simulation over a bench or flight check for this topic, and task 3
carries the evidence instead. Nothing in the tree calls `setDefault_PIDProfile ( )`, so no flown code path changes:
the only firmware-visible effects are the moved default-gain block ( verified by diff in task 1 ) and the API function
itself. If a bench check is wanted later, it is `Monitor_Print` of `get_PIDProfile` for all five slots before and after
one call, disarmed.

- **Depends on:** 4
- **Skills / agent:** `pluto-flighttest`
- **Safety impact:** -
- **Done when:** -
- **Status:** dropped ( replaced by task 3; the user declined bench and flight logging )
- **Result:**

### 6. Pipeline docs ( `PIPELINE_UPDATE.md` )

**Description.** Write the new version of
[subsystems/Core_PID_Controller.md](../../fw-architecture-pipeline/subsystems/Core_PID_Controller.md) into this topic's
`PIPELINE_UPDATE.md` ( never edit the pipeline folder directly ): one line stating that the API default reset covers the
five exposed slots from the shared `pidProfileDefaultGains ( )` and leaves the hidden gains alone, and a mention of the
new `flight/pid_defaults.cpp` as the single source of the default gains. Check the statement against the code with
`spec-to-code-compliance`. No new `docs/API/` wiki and no `User_Space_API.md` change were asked for; if the compliance
check shows `User_Space_API.md` already describes this function wrongly, stage that correction too. No flow changed, so
no Mermaid diagram is needed. No `DMA_MAP` / `TIMER_MAP` / `PIN_MAP` impact. No new CLAUDE.md rule: the gotcha is
narrow and now documented in the header.

- **Depends on:** 3, 4
- **Skills / agent:** `spec-to-code-compliance`
- **Files:** new `PIPELINE_UPDATE.md`
- **Safety impact:** none
- **Done when:** `PIPELINE_UPDATE.md` holds the staged `Core_PID_Controller.md` text, and every statement in it matches a source line.
- **Status:** done 2026-11-14
- **Result:** [PIPELINE_UPDATE.md](PIPELINE_UPDATE.md) stages: in `Core_PID_Controller.md` a "Default Gains" source line and a new "Default gains and the API reset" section ( `pidProfileDefaultGains ( )` as the single source; `setDefault_PIDProfile ( )` writes the five API slots only, RAM only, armed caveat ); and three line references shifted by task 1 ( `config.cpp` 369 to 331, 278 to 240 in `Mixer_Pipeline.md`, 479 to 441 in `User_Space_API.md` ), each matched by content to HEAD. `spec-to-code-compliance` is not available in this session: every statement was checked by hand against a source line, with a table in the file. No diagram, map, wiki or CLAUDE.md change, with the reasons recorded.

### 7. Topic review

**Description.** Run the `pluto-reviewer` agent over the whole topic diff ( `config.cpp`, `pid_defaults.cpp`, `pid.h`,
`FC-Config-PID.cpp`, `FC-Config.h`, `Makefile`, `tools/sim/`, baseline, docs ). Point it at the two things most likely
to be wrong: that the moved default-gain block is value-identical to the original, and that the five-slot copy uses the
right `pidIndex_e` members with no implicit narrowing. Fix every BLOCKING finding and re-run the gate afterwards. The
review runs in the subagent, never in the main conversation.

- **Depends on:** 6
- **Skills / agent:** `pluto-reviewer` agent
- **Safety impact:** Catching a mistuned default here is the point of the task.
- **Done when:** Zero BLOCKING findings outstanding; the gate is green again after any fix; findings and resolutions recorded in the README open items.
- **Status:** done 2026-11-14
- **Result:** 0 blocking, 1 should-fix, 5 nits, all fixed. The should-fix mattered: `TASKS.md` named only `Core_PID_Controller.md`, so task 8 would have left `Mixer_Pipeline.md` and `User_Space_API.md` stale; all three are now named in the Summary and in task 8. Nits: three stale `CHANGES.md` line references, the `#L156` link ( the loop is 158 at HEAD ), the `TESTING.md` reproduce comment ( UBSan aborts, not ASan ), "host `g++` simulation" ( clang with sanitizers is the main run ), and `tools/sim/` now documented in `dev-guide/BUILD_AND_VERIFY.md`. Gate and `tools/sim/run.sh` re-run green afterwards.

### 8. Graph refresh & commit

**Description.** After the review fixes: `graphify update .` then `python tools/graph_labels.py`, then `/pluto-commit`.
The commit bumps `FW_Version` to 3.15.7 and `API_Version` to 1.8.5 in the [Makefile](../../../../Makefile) ( following
the recent API fix topics ), builds all three targets ( `PRIMUS_X2_v1`, `PRIMUSX2`, `PRIMUS_V5` ), applies all three
sections of `PIPELINE_UPDATE.md` ( `Core_PID_Controller.md`, `Mixer_Pipeline.md`, `User_Space_API.md`: leaving the
last two out would keep their stale `config.cpp` line references ), updates `CHANGELOG.md`, sets this topic Closed in its README and in
[active-development/README.md](../README.md), and stages everything with a drafted message. The user makes the commit.

- **Depends on:** 7
- **Skills / agent:** `graphify`, `pluto-commit`
- **Safety impact:** none
- **Done when:** All three targets build clean, versions bumped, pipeline doc and CHANGELOG updated, graph refreshed with readable community labels, change staged with a drafted message.
- **Status:** done 2026-11-14
- **Result:** `graphify update .` ( 5115 nodes ) and `tools/graph_labels.py` ( run through `runpy` with `GRAPHIFY_VIZ_NODE_LIMIT=6000`: the known `tools/warnings.py` shadowing and 5000-node HTML limit, `.claude/TOOLING_BACKLOG.md` item 13 ). `/pluto-commit`: all three targets clean, no new `src/` warnings; PRIMUS_X2_v1 122776 / 1316 / 19728 B, PRIMUSX2 122596 / 1316 / 19884 B, PRIMUS_V5 122768 / 1316 / 19728 B. FW 3.15.7 / API 1.8.5; three pipeline docs and the CHANGELOG updated; staged with a drafted message.

## Decisions log

Newest last. One line each: `YYYY-MM-DD [decision|assumption|out-of-scope|risk] text`.

- 2026-11-14 [decision] Mode `fix` with the cause taken as proven from source; no reproduce task.
- 2026-11-14 [decision] `setDefault_PIDProfile ( )` restores only the five slots the API exposes ( `PIDROLL`, `PIDPITCH`, `PIDYAW`, `PIDALT`, `PIDUSER` ); the hidden gains are left untouched.
- 2026-11-14 [decision] One source of truth: the gains come from a shared `pidProfileDefaultGains ( )` extracted from `resetPidProfile ( )`, not from a second table in the API, so a stale value cannot drift back in.
- 2026-11-14 [decision] The helper goes in a new `src/main/flight/pid_defaults.cpp` so the host harness can compile it without `config.cpp`; `config.h` is the fallback location.
- 2026-11-14 [decision] RAM only: no `writeEEPROM`, as today.
- 2026-11-14 [decision] No armed guard; the armed caveat is documented in the header comment instead ( `set_PIDProfile` and `MSP_SET_PID` are unguarded too ).
- 2026-11-14 [decision] Verification is a host simulation kept in `tools/sim/`, re-runnable, built with clang++ and ASan + UBSan where the toolchain allows and g++ as the fallback; the user declined bench and flight logging, so closing task 5 is dropped.
- 2026-11-14 [decision] Docs: the `FC-Config.h` comment plus one line in `Core_PID_Controller.md` through `PIPELINE_UPDATE.md`, and the CHANGELOG at commit. No new `docs/API/` FC-Config wiki.
- 2026-11-14 [out-of-scope] `get_PIDProfile` / `set_PIDProfile` ( index map verified correct ), the mixed `ROLL` vs `PIDROLL` naming, an EEPROM save, an armed guard.
- 2026-11-14 [assumption] `flight/pid.h` and the `pidProfile_t` definition compile on the host behind a small stub; if not, task 3 adds a minimal stub header and says so.
- 2026-11-14 [risk] With `pidController == 2` the roll / pitch / yaw gains come from the float `P_f` / `I_f` / `D_f` set, so the restored `P8` values do not apply on those axes. Default controller is 1; noted in the header comment rather than fixed.
- 2026-11-14 [risk] Task 1 moves the values every fresh EEPROM gets; a transcription error there would mistune all axes on a config reset. Guarded by a line-by-line diff and by the topic review.
- 2026-11-14 [decision] `spec-to-code-compliance` is not installed in this session; task 6 checked every staged statement against a source line by hand ( table in `PIPELINE_UPDATE.md` ).
- 2026-11-14 [decision] Task 6 also stages three pipeline line references that task 1 shifted ( `config.cpp` 369 / 278 / 479 to 331 / 240 / 441 ): drift this topic caused, so it fixes them.
- 2026-11-14 [decision] Task 7 also added a `tools/sim/` paragraph to `dev-guide/BUILD_AND_VERIFY.md` ( review nit ): the dev guide is a standing reference, not the pipeline folder, so it is edited directly rather than staged.
- 2026-11-14 [decision] Closed - pending commit: FW 3.15.6 -> 3.15.7 ( patch, a fix ), API 1.8.4 -> 1.8.5 ( behaviour only ); all targets built; `Core_PID_Controller.md`, `Mixer_Pipeline.md`, `User_Space_API.md` and CHANGELOG updated; baseline refreshed for the four removed `FC-Config-PID.cpp` warnings only.
