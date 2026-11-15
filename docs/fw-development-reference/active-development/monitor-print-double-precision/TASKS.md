# Monitor_Print double precision - Tasks

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

## Resume

Read this block first in a new session; read only the task it points to.

| | |
|---|---|
| **Current task** | none: **Closed - pending commit** ( 15 Nov 2026, FW 3.15.11, API 1.8.9 ) |
| **Next step** | The user commits with `git commit -F .git/MAGISV2_COMMIT_MSG.txt`; the next `pluto-commit` / `pluto-grill` run records the hash. |
| **Open questions** | none |
| **Blocked on** | nothing |
| **Last updated** | 2026-11-15 |

## Summary

| | |
|---|---|
| **Mode** | fix |
| **Goal** | Every `Monitor_Print` / `Monitor_Println` overload in `Debugging.h` prints correct, well-framed output, with no signature change. |
| **Done when** | A bench PlutoMonitor log matches the test table for all six functions ( T01: 3.14159 @ 3 -> `3.142` ), and a short hover with prints running keeps the app connected with loop time unchanged. |
| **Target** | PRIMUS_X2_v1 |
| **Analysis** | Proven cause of B9: the pre-fix fraction loop ( `Debugging.cpp:201-208` at `HEAD`, now `:224-238` ) subtracts the ASCII code ( 48 + d ) from the remainder instead of d; the remainder goes negative and the `double -> uint8_t` cast ( `__aeabi_d2uiz` ) returns 0 for every later digit. Also D1 256+ char hang, D2 length-byte wrap, D3 `INT_MIN`, D4 NaN / inf length byte ( INVESTIGATION.md ). |
| **Pipeline docs affected** | `subsystems/User_Space_API.md` ( one bullet, via `PIPELINE_UPDATE.md` ); `docs/API/` wiki, `dev-guide/FLIGHT_INVARIANTS.md`, CLAUDE.md |
| **Order** | 1 → 2 → 3 → 4 → 5 → 6 → 7 → 8 → 9 |

## Index

| # | Title | Status | Depends on | Skills / agent |
|---|---|---|---|---|
| 1 | Fix the fraction digits ( B9 ) and clamp precision to 4 | done 2026-11-15 | - | pluto-rules |
| 2 | Bound string length and fix the frame length bytes ( D1, D2, D4 ) | done 2026-11-15 | 1 | pluto-rules |
| 3 | Print `INT_MIN` correctly ( D3 ) | done 2026-11-15 | - | pluto-rules |
| 4 | Host check of every formatter against the test table | done 2026-11-15 | 1, 2, 3 | pluto-rules |
| 5 | Build gate | done 2026-11-15 | 1, 2, 3 | pluto-build |
| 6 | Hardware validation ( bench table + short hover ) | done 2026-11-15 | 4, 5 | pluto-flighttest |
| 7 | Docs: Debugging API wiki and drift fixes | done 2026-11-15 | 6 | spec-to-code-compliance |
| 8 | Topic review | done 2026-11-15 | 7 | pluto-reviewer agent |
| 9 | Graph refresh & commit | done 2026-11-15 | 8 | graphify, pluto-commit |

Status values: `todo`, `in-progress`, `done YYYY-MM-DD`, `blocked (<why>)`, `dropped (<why>)`.
Serial numbers are never reused or renumbered.

## Tasks

### 1. Fix the fraction digits ( B9 ) and clamp precision to 4

**Description.** In the fraction loop of `debugPrint ( const char *msg, double number, uint8_t digits )`
( `Debugging.cpp:201-208` ), take the digit as an integer, subtract that digit from the remainder, and only then
convert it to ASCII for the wire and the checksum. Keep rounding half away from 0 ( already done at `:144-147` ) and
`-0.00` for small negatives ( decision ). Change the clamp in both double wrappers ( `:294`, `:318` ) from 0..7 to
0..4, and check that the payload length at `:168` still matches the bytes sent. No new `-Wconversion` warnings; the
`double` parameter stays ( pluto-rules exemption ).

- **Depends on:** -
- **Skills / agent:** pluto-rules
- **Files:** [Debugging.cpp](../../../../src/main/API-Src/Debugging.cpp)
- **Safety impact:** none ( debug output only ); the byte count per call does not grow.
- **Done when:** builds clean on PRIMUS_X2_v1; by hand trace 3.14159 @ 3 -> `3.142`, -2.5 @ 1 -> `-2.5`, 1.99999 @ 2 -> `2.00`, precision 9 -> 4 digits; reviewer clean.
- **Rollback:** revert the `Debugging.cpp` hunk ( output returns to the old zeros, no flight effect ).
- **Status:** done 2026-11-15
- **Result:** The digit is subtracted from the remainder before the ASCII conversion and the clamp is now
  `MONITOR_MAX_PRECISION` 4 in both double wrappers. Gate green on `PRIMUS_X2_v1`, no new `src/` warnings, flash
  121.5 KB / 256 KB, RAM 20.5 KB / 40 KB. Reviewer: nothing blocking; it confirmed the remainder stays in [ 0, 1 ) and
  the payload length still matches the bytes sent. One banner-alignment nit, applied. Hand traces as planned
  ( 3.142 / -2.5 / 2.00 / 4 digits at precision 9 ); hardware check is task 6.

### 2. Bound string length and fix the frame length bytes ( D1, D2, D4 )

**Description.** `serializeString` ( `:50-56` ) loops with a `uint8_t` index against `strlen ( )`, so a string of 256+
characters hangs `loop ( )`. Make the loop index wide enough and stop at 255 characters, and in every `debugPrint`
variant cut the tag so the whole payload ( tag + tab + number ) fits in the one-byte length field ( assumption: the
cut applies to the tag, never to the number ). Count the string once ( no repeated `strlen` per character ). Fix the
NaN ( `:97` ) and inf ( `:111` ) payload lengths, which omit the tab byte, so length and checksum match what is sent.

- **Depends on:** 1 ( same file, same function )
- **Skills / agent:** pluto-rules
- **Files:** [Debugging.cpp](../../../../src/main/API-Src/Debugging.cpp)
- **Safety impact:** removes a possible `loop ( )` hang from user code ( a 256+ character string ); no flight logic touched.
- **Done when:** builds clean; the length byte equals the payload bytes for a 0-, 255- and 300-character tag with each overload and for NaN / inf; no `strlen` inside the per-character loop.
- **Rollback:** revert the hunk.
- **Status:** done 2026-11-15
- **Result:** New `MONITOR_MAX_PAYLOAD` 255 and `boundedLength ( )`; `serializeString` is `static` and takes a length;
  the NaN / inf / overflow branches share `debugPrintTagText ( )`, which counts the tab. Every print computes the
  number length first and cuts the tag to fit. Gate green, no new `src/` warnings, and this file's baselined warnings
  drop from 10 to 4; flash 121.5 KB / 256 KB, RAM 20.5 KB / 40 KB. Reviewer: nothing blocking; it traced the length
  byte against the bytes sent on all six paths at tag lengths 0 / 255 / 300, confirmed no `uint8_t` wrap and that
  `serializeString` had no other user. One nit, no code change: a 255-character payload is a 259-byte frame and so
  cannot fit the 256-byte TX ring ( not a regression ) - recorded for the task 7 wiki.

### 3. Print `INT_MIN` correctly ( D3 )

**Description.** `debugPrint ( msg, int )` negates `number` in `int` ( `:227` ), which overflows for `INT_MIN` and then
prints negative digits as garbage characters ( `:242-244` ). Convert to the magnitude in an unsigned type before
negating ( no signed overflow ), keep the payload length calculation consistent, and do not add `-Wconversion` /
`-Wsign-conversion` warnings.

- **Depends on:** -
- **Skills / agent:** pluto-rules
- **Files:** [Debugging.cpp](../../../../src/main/API-Src/Debugging.cpp)
- **Safety impact:** none.
- **Done when:** builds clean; by hand trace `INT_MIN` -> `-2147483648`, `INT_MAX` -> `2147483647`, 0 -> `0`, -7 -> `-7`.
- **Rollback:** revert the hunk.
- **Status:** done 2026-11-15
- **Result:** The magnitude is taken in `uint32_t` ( `0u - magnitude`, defined for every value ) with a
  `static_assert` that `int` fits, and the digit loop works on it. Gate green, no new `src/` warnings, this file's
  baselined warnings drop from 4 to 3; flash 121.5 KB / 256 KB, RAM 20.5 KB / 40 KB. Reviewer: nothing blocking; it
  traced `INT_MIN` -> `-2147483648`, `INT_MAX`, 0 and -7, and confirmed the length bookkeeping still matches the bytes
  sent. One nit, applied: the banner history now also names the task 2 and 3 fixes.

### 4. Host check of every formatter against the test table

**Description.** Compile the real `Debugging.cpp` for the PC against the `tools/sim/stub/` tree ( as earlier API
topics did ), with `serialize8Debug` capturing bytes, and check every frame for content, length byte and checksum.
Table: 3.14159 @ 3, @ 4, @ 9 ( clamp ), -2.5 @ 1, -0.5 @ 2, -0.0001 @ 2 ( `-0.00` ), 0 @ 0, 1.99999 @ 2, 1e9 @ 4,
5e9 ( overflow ), NaN, +inf, -inf; int 0, -7, `INT_MAX`, `INT_MIN`; strings of 0, 255 and 300 characters; every
Println variant ( second `\n` packet ). Run the same table on the pre-fix file to show T01 reproduces.

- **Depends on:** 1, 2, 3
- **Skills / agent:** pluto-rules
- **Files:** [Debugging.cpp](../../../../src/main/API-Src/Debugging.cpp), `tools/sim/` ( test harness only )
- **Safety impact:** none.
- **Done when:** fixed file passes every row; pre-fix file fails T01 ( `3.100` ); results in `TESTING.md`.
- **Status:** done 2026-11-15
- **Result:** The real `Debugging.cpp` on the PC ( g++ 14.3, `-ftrapv` ) passes 86 / 86: 85 table rows ( every length
  byte and checksum correct ) and a 200 000-value sweep against `snprintf` with 0 wrong ( 2 differ at a rounding tie,
  integer part of 1e8 or more at 4 decimals ). `HEAD` fails 51 / 83 and shows B9, D1, D2, D3 and D4. On the PC, T01
  prints `3.1\n`, not `3.100`: the old out-of-range cast is undefined, and ARM saturates it to 0 while x86 wraps it. The
  harness is in the scratchpad, not committed. Details: TESTING.md.

### 5. Build gate

**Description.** `pluto-build --gate PRIMUS_X2_v1` after the code tasks: no new `src/` warnings against the baseline
( removing some of the 10 baselined `Debugging.cpp` warnings is fine; never regenerate the baseline ), flash and RAM fit.
Record the flash delta of `Debugging.o`.

- **Depends on:** 1, 2, 3
- **Skills / agent:** pluto-build
- **Files:** -
- **Safety impact:** none.
- **Done when:** gate green, flash / RAM numbers recorded in `TESTING.md`.
- **Status:** done 2026-11-15
- **Result:** Clean `--gate PRIMUS_X2_v1` green: no new `src/` warnings, `Debugging.cpp` baselined warnings 10 -> 3,
  flash 121.5 KB / 256 KB, RAM 20.5 KB / 40 KB. `Debugging.o` `.text` 1 255 -> 1 263 B ( +8 ) against `HEAD` built with
  the same command; no RAM change. The flashed image is unchanged today because no user code calls `Monitor_*`, so the
  linker drops the module. Details: TESTING.md.

### 6. Hardware validation ( bench table + short hover )

**Description.** Use `pluto-flighttest` to write a `PlutoPilot.cpp` bench test that prints the task-4 table in
PlutoMonitor, one row per tick and under the ~130 B/tick budget, and switches to a hover line ( `t`, `n`, `cyc`,
`alt`, `ts` ) while armed, so one Developer Mode session covers both halves: table disarmed, hover, table again after
landing. Results in `TESTING.md`. The test code is not committed.

- **Depends on:** 4, 5
- **Skills / agent:** pluto-flighttest
- **Files:** [PlutoPilot.cpp](../../../../PlutoPilot.cpp) ( repo root; test only, reverted )
- **Safety impact:** hover only; normal flight code.
- **Done when:** `logs/log-1.txt` matches every table row ( T01 `3.142` ), `ts` = `t` / 1000 on every hover line, no
  app disconnect, loop time as before, and the table resumes after the landing.
- **Status:** done 2026-11-15
- **Result:** Flown twice, both passing; `logs/log-1.txt` holds the second run ( 95 s: table, 85 s airborne up to
  175 cm, table again ). 0 wrong table rows in either run ( 88 rows over 4 passes here, 167 over 8 in run 1 ),
  including T01 `3.142`, `INT_MIN` and complete NaN / inf lines ( the app accepts the corrected length byte ).
  `ts` = `t` / 1000 on 633 / 633 hover records ( 440 / 440 in run 1 ). `cyc` median 3505 us, 631 / 633 within
  3497-3542. No disconnect; about 1 record in 60 lost, never a partial one. Handling normal ( user ). Two long
  Developer Mode gaps ( 2.4 s, 15.6 s ) were the user's own switch, and printing resumed correctly after both.
  R20 / R22 on one line could not be tested: the app shows every frame on its own line, and the frames are unchanged
  from `HEAD`. Two 23 ms `cycleTime` samples are not from this change ( `uartWrite` does not wait; the print loops are
  bounded ) and are logged for a separate check. Root `PlutoPilot.cpp` restored to `HEAD`; the test version is in the
  scratchpad.

### 7. Docs: Debugging API wiki and drift fixes

**Description.** Write the new `docs/API/DEBUGGING_API_WIKI.md`, covering all six `Monitor_*` functions:

- precision 0..4 with rounding half away from 0, and `-0.00` for small negatives;
- the 255-character cut and the bytes each call costs, including the 5-byte Println newline;
- the ~130 B/tick budget, and that the 255-character cut is a framing limit, not a usable length: a 255-character
  payload is a 259-byte frame, longer than the 256-byte UART TX ring ( `UART1_TX_BUFFER_SIZE` ), so it overwrites its
  own start;
- the overflow, NaN and inf outputs;
- the `Monitor_Print ( "x", 3.7f )` -> int overload pitfall;
- that PlutoMonitor shows every frame as its own line ( task 6 ), so a `Print` + `Println` pair is not joined on screen.

Remove the "prints 0 after the first decimal" text from `dev-guide/FLIGHT_INVARIANTS.md`, and stage the CLAUDE.md
line change for commit. Drop the x100 workaround in `FC_DATA_API_WIKI.md:46-48`. Check the wiki against the code with
`spec-to-code-compliance`. No `PIPELINE_UPDATE.md`: no `fw-architecture-pipeline/` doc covers these prints.

- **Depends on:** 6
- **Skills / agent:** spec-to-code-compliance
- **Files:** `docs/API/DEBUGGING_API_WIKI.md` ( new ), `docs/API/FC_DATA_API_WIKI.md`, `docs/fw-development-reference/dev-guide/FLIGHT_INVARIANTS.md`, `CLAUDE.md`
- **Safety impact:** none.
- **Done when:** the wiki examples print what the bench log showed; compliance check finds no drift.
- **Status:** done 2026-11-15
- **Result:** New `docs/API/DEBUGGING_API_WIKI.md`; `Debugging.h` comments state the limits ( no signature change );
  the stale sentence is gone from FLIGHT_INVARIANTS.md, the FC-Data workaround and backlog item 11. A new
  `PIPELINE_UPDATE.md` stages a `User_Space_API.md` bullet and the CLAUDE.md line. The `spec-to-code-compliance`
  skill is not available in this session, so every claim was checked by hand against a source line or test result
  ( table in CHANGES.md ). Every wiki example compiles clean with the firmware command for `PlutoPilot.cpp`. The
  compile also caught one wrong claim in the draft: a `float` passed as the `double` gives no warning. Corrected.
  Gate green after the header change.

### 8. Topic review

**Description.** `pluto-reviewer` on the whole topic diff ( code and docs ). Fix every BLOCKING finding and run the
gate again before moving on.

- **Depends on:** 7
- **Skills / agent:** pluto-reviewer agent
- **Files:** topic diff
- **Safety impact:** none.
- **Done when:** no open BLOCKING findings; gate green after any fixes.
- **Status:** done 2026-11-15
- **Result:** Nothing blocking, and nothing in the code: 3 should-fix and 4 nits, all in the docs and records, all
  applied ( table in CHANGES.md ). The main one: "one frame per call" was wrong for `Println`, which adds a 5-byte
  `"\n"` frame, in the staged CLAUDE.md line as well. Re-checked after the fixes: gate green, 0 broken links, no
  dashes in added lines, root `PlutoPilot.cpp` equals `HEAD`.

### 9. Graph refresh & commit

**Description.** After the review fixes, run `graphify update .` and then `python tools/graph_labels.py`. Then run
`/pluto-commit`, which does the following:

- builds all targets;
- bumps API 1.8.8 -> 1.8.9 and FW 3.15.10 -> 3.15.11;
- writes the CHANGELOG entry and the CLAUDE.md line;
- stages the files and drafts the message.

The user commits.

- **Depends on:** 8
- **Skills / agent:** graphify, pluto-commit
- **Files:** Makefile, CHANGELOG.md, CLAUDE.md, graphify-out/
- **Safety impact:** none.
- **Done when:** all targets build clean; changes staged with a drafted message.
- **Status:** done 2026-11-15
- **Result:** Graph refreshed ( 5121 nodes, 14025 edges, 244 communities, all labelled; `graph_labels.py` needed the
  backlog #13 workaround and `GRAPHIFY_VIZ_NODE_LIMIT=6000`, now noted in item 13 ). `/pluto-commit`: all three targets
  built clean, FW 3.15.10 -> 3.15.11, API 1.8.8 -> 1.8.9, `PIPELINE_UPDATE.md` applied to `User_Space_API.md` and
  CLAUDE.md, CHANGELOG entries added, changes staged with a drafted message. The user commits.

## Decisions log

Newest last. One line each: `YYYY-MM-DD [decision|assumption|out-of-scope|risk] text`.

- 2026-11-15 [decision] Mode `fix`; target PRIMUS_X2_v1.
- 2026-11-15 [decision] Scope is every function in `Debugging.h`, not only the double overloads.
- 2026-11-15 [decision] Done when: bench PlutoMonitor test table for all six functions, plus a short hover with prints running.
- 2026-11-15 [decision] Round half away from 0 ( T01 expects 3.142 ); the existing `:144-147` rounding is kept.
- 2026-11-15 [decision] Precision clamp 0..4 ( was 0..7 ).
- 2026-11-15 [decision] Proper fix, no API signature change, no heap, no printf float ( `common/printf.cpp` has no `%f` ).
- 2026-11-15 [decision] Strings are cut at 255 characters ( the user chose this over a 64-character cap ).
- 2026-11-15 [decision] Println keeps the newline as a separate 5-byte packet ( no app parser risk ).
- 2026-11-15 [decision] Keep `-0.00` for small negatives.
- 2026-11-15 [decision] New `docs/API/DEBUGGING_API_WIKI.md`; API 1.8.8 -> 1.8.9, FW 3.15.10 -> 3.15.11 at commit.
- 2026-11-15 [assumption] When tag + number exceed the one-byte length field, the tag is cut, never the number.
- 2026-11-15 [risk] User code that asked for 5-7 digits now gets 4.
- 2026-11-15 [out-of-scope] `uartWrite ( )` / `serialWrite ( )` not checking the TX ring for full; the PlutoMonitor app parser; `lib/`.
- 2026-11-15 [out-of-scope] `Monitor_Print ( "x", 3.7f )` resolving to the int overload: documented in the wiki, no signature change.
- 2026-11-15 [risk] A 255-character payload is a 259-byte frame and cannot fit the 256-byte UART TX ring, so the
  longest allowed print still overwrites its own start ( not a regression; found in the task 2 review ). Documented in
  task 7, no code change.
- 2026-11-15 [decision] The task 4 host harness stays in the session scratchpad and is not committed, as in
  `failsafe-enable-api`; TESTING.md gives its path and how to re-run it.
- 2026-11-15 [risk] Known limit, not a defect: the firmware rounds by adding 0.5 / 10^d in `double`, so a value
  within double rounding of an exact half can come out one higher in the last digit than `snprintf` ( 2 in 200 000,
  both with an integer part of 1e8 or more at 4 decimals ).
- 2026-11-15 [decision] Task 6 merged into one run ( user ): disarmed table, hover, table again, one log.
- 2026-11-15 [out-of-scope] Seen in task 6 run 2: two `cycleTime` samples of ~23 ms ( 04:52:48.850, 04:53:56.952 )
  against a 3.5 ms loop, about 1 main loop in 28 being sampled. Not the print code ( `uartWrite` only fills the ring
  and starts DMA, `serial_uart.c:301-313`; the print loops are bounded ). Worth a `check` topic: what takes ~20 ms in
  the main loop roughly once a second.
- 2026-11-15 [out-of-scope] Seen in task 6: on low battery the drone bumped the ground at touchdown and took off
  again ( estimate -71 cm, then 180 cm in 2.5 s before the disarm; user confirms real movement ). Not caused by the
  print code; recommended as a separate `check` topic on landing at low battery.
- 2026-11-15 [assumption] PlutoMonitor shows every debug frame as its own line ( tab and leading space dropped, the
  `"\n"` frame not shown ), so `Print` + `Println` does not join on screen. Seen in task 6; whether older app versions
  joined them is unknown. For the task 7 wiki.
- 2026-11-15 [decision] No `PIPELINE_UPDATE.md`: no `fw-architecture-pipeline/` doc covers these prints; the closing docs task covers wiki, dev-guide and CLAUDE.md.
- 2026-11-15 [decision] Superseded in task 7: `User_Space_API.md` names each API area with its wiki, so the new
  Debugging wiki needs a bullet there. `PIPELINE_UPDATE.md` now stages it, with the CLAUDE.md line.
- 2026-11-15 [decision] Closed - pending commit: FW 3.15.10 -> 3.15.11 ( patch, a fix ), API 1.8.8 -> 1.8.9
  ( behaviour only, no signature change ).
- 2026-11-15 [decision] Task 7 also documents the limits in the `Debugging.h` comments ( the header stated no
  precision range ). Comments only; gate green.
