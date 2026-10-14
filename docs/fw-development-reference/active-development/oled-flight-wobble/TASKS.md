# OLED Flight Wobble - Tasks

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

The fix was written and tested in a separate working copy ( a user project, not a git repo ) and copied into
this repo on 14 Oct 2026. These tasks merge it properly. The code is already in the working tree.

## Resume

Read this block first in a new session; read only the task it points to.

| | |
|---|---|
| **Current task** | **Closed - pending commit** ( 14 Oct 2026, FW 3.11.3, API 1.4.1 ). All tasks done. |
| **Next step** | The user commits; the next pluto-commit or pluto-grill run fills in the hash |
| **Open questions** | none |
| **Blocked on** | nothing |
| **Last updated** | 2026-10-14 |

## Summary

| | |
|---|---|
| **Mode** | fix ( merge and ship a fix that is already written and tested ) |
| **Goal** | Merge the OLED framebuffer and rate-limited flush into MagisV2, so an OLED with live telemetry no longer makes the drone twitch at 5 Hz. |
| **Done when** | The merged build passes the gate on all three targets; the reviewer finds nothing blocking; one confirmation hover on this repo's build with the OLED and telemetry live shows no twitch and the OLED updates; the wiki and docs match the code. |
| **Target** | PRIMUS_X2_v1 ( all three targets at commit ) |
| **Analysis** | Cause ( [INVESTIGATION.md](INVESTIGATION.md), confirmed by the A/B flight ): ~630 blocking I2C transactions every 200 ms ( 45-60 ms ) on I2C1, which the gyro shares, so the PID ran on a stale gyro sample at 5 Hz. Fix: a 1 KB RAM framebuffer, and `oledFlushStep ( )` sends at most 16 changed bytes per gated loop ( ~0.6 ms ). Merge check ( [SCOUT.md](SCOUT.md), merge brief ): same base as `fc6879b`, no reverts, nothing missing except the wiki. |
| **Pipeline docs affected** | subsystems/Hardware_Bus_Pipeline.md, subsystems/User_Space_API.md; `docs/API/OLED_API_WIKI.md`; CHANGELOG; CLAUDE.md |
| **Order** | 1 → 2 → 3 → 4 → 5 → 6 |

## Index

| # | Title | Status | Depends on | Skills / agent |
|---|---|---|---|---|
| 1 | Build gate and memory in this repo | done 2026-10-14 | - | pluto-build |
| 2 | Confirmation hover on the merged build | done 2026-10-14 | 1 | pluto-flighttest |
| 3 | OLED API wiki: copy across and check | done 2026-10-14 | 1 | spec-to-code-compliance |
| 4 | Architecture & pipeline docs ( PIPELINE_UPDATE.md ) | done 2026-10-14 | 2, 3 | graphify, spec-to-code-compliance |
| 5 | Topic review | done 2026-10-14 | 4 | pluto-reviewer agent |
| 6 | Graph refresh & commit | done 2026-10-14 ( staged; the user commits ) | 5 | graphify, pluto-commit |

Status values: `todo`, `in-progress`, `done YYYY-MM-DD`, `blocked (<why>)`, `dropped (<why>)`.
Serial numbers are never reused or renumbered.

## Tasks

### 1. Build gate and memory in this repo

**Description.** The merged code has never been built in this repo. Run `pluto-build --gate PRIMUS_X2_v1`
and record flash and RAM; the figures in TESTING.md ( 102.7 KB, 17.8 KB ) come from the other copy. The
framebuffer `oledSystemBuffer` should add exactly 1024 B of RAM over HEAD ( 14.8 KB ). Any new `src/` warning
is fixed here, not baselined.

- **Depends on:** -
- **Skills / agent:** pluto-build
- **Files:** [oled_display.c](../../../../src/main/io/oled_display.c), [display_ug2864hsweg01.c](../../../../src/main/drivers/display_ug2864hsweg01.c), [Oled.h](../../../../src/main/API/Oled.h)
- **Safety impact:** none ( build only ).
- **Done when:** no new `src/` warnings; RAM up by about 1 KB over 14.8 KB, and flash and RAM recorded.
- **Rollback:** -
- **Status:** done 2026-10-14
- **Result:** PRIMUS_X2_v1, no new `src/` warnings. Flash 101.3 KB ( text 102540 + data 1280 B; +428 B over HEAD ), RAM 16.8 KB / 40 ( bss 15944 B; **+2048 B**, not the expected 1024 ). Cause of the extra 1 KB: at HEAD `oledShadowBuffer` was used only by `Oled_display_Update ( )`, which the stock `PlutoPilot.cpp` never calls, so `-ffunction-sections -fdata-sections -gc-sections` dropped it. The system flush now uses it every loop, so it is always linked: +1024 for `oledSystemBuffer` and +1024 for the shadow. A user project that already called `Oled_display_Update` sees only +1 KB. 23.2 KB of RAM is left for the stack.

### 2. Confirmation hover on the merged build

**Description.** The separate project's tested build did not contain `386599b` ( crash limits ) or `84c326d`
( failsafe delay, `EEPROM_CONF_VERSION` 108 ). Neither touches the OLED code, but the user asked for one
confirmation on this repo's build. Flash the merged build and recalibrate the accelerometer and compass if the
EEPROM version changed since the board was last flashed. Fit the OLED, use a `PlutoPilot.cpp` that calls
`Oled_Init ( )`, then hover 30 s with Dev mode off and switch Dev mode on and off 3 times. The drone must be as
steady as without the OLED, with no 5 Hz twitch, and the OLED must keep updating.

- **Depends on:** 1
- **Skills / agent:** pluto-flighttest
- **Files:** `TESTING.md`, `logs/log-N.txt`
- **Safety impact:** flight with the OLED on the shared I2C1 bus.
- **Done when:** no twitch in the hover or at the Dev mode switches; the telemetry updates; recorded in TESTING.md.
- **Status:** done 2026-10-14
- **Result:** passed on the user's report ( "doing great" ), flown with the Traffic Hazard `PlutoPilot.cpp` that calls `Oled_Init ( )`. logs/log-1.txt was not filled in, so no per-step notes are recorded.

### 3. OLED API wiki: copy across and check

**Description.** `docs/API/OLED_API_WIKI.md` was updated in the separate copy but not brought across. The user
copies it in. This task then checks it against this repo's code:
- `Oled_Update ( )` / `Oled_display_Update ( )` only queue the frame; a full screen reaches the panel in ~225 ms;
- a buffer passed in must stay valid ( `static` );
- `Oled_Mode ( )` no longer clears the hardware;
- there is a timing and flight-safety section.

Lines 57, 152 and 163 of the current file are wrong for the new code.

- **Depends on:** 1
- **Skills / agent:** spec-to-code-compliance
- **Files:** [OLED_API_WIKI.md](../../../API/OLED_API_WIKI.md)
- **Safety impact:** none
- **Done when:** the wiki is in this repo and every statement about timing, buffers and modes matches `oled_display.c`.
- **Status:** done 2026-10-14
- **Result:** the user copied `OLED_API_WIKI.md` across: new "Timing and flight safety" section; lines 56, 57, 152 and 163 rewritten. Checked against the code. The flush is at most 16 bytes per loop ( `OLED_FLUSH_MAX_BYTES` ), so a full screen is ~64 loops ( ~0.22 s at 3.5 ms ). The switch-off rule ( 3 failures in a row, or more than ~1 in 5 ) matches `OLED_FAIL_COST` 4 / `OLED_FAIL_LIMIT` 12 with −1 per good write. `Oled_display_Update` is User mode only and the buffer must stay valid; `Oled_Print` draws into the system framebuffer. Two gaps fixed: `Oled_Update` does nothing in System mode, and `Oled_Mode` starts from a blank frame.

### 4. Architecture & pipeline docs ( PIPELINE_UPDATE.md )

**Description.** Stage in `PIPELINE_UPDATE.md`:
- `subsystems/Hardware_Bus_Pipeline.md`: I2C is polled only, so remove the DMA and "non-blocking" claim at
  :17-18; I2C1 is shared with the OLED, which sends one run of at most 16 bytes per gated loop;
- `subsystems/User_Space_API.md`: `Oled_Update` is deferred and buffers must be static.

Also:
- CHANGELOG: new entry, and correct the v4.0.0 lines ( :11, :36 ) that call the old OLED "non-blocking";
- CLAUDE.md gotcha: `Oled_Update` is deferred, buffers must be static, and any direct panel write must call
  `oledPanelCleared ( )`.

- **Depends on:** 2, 3
- **Skills / agent:** graphify, spec-to-code-compliance
- **Files:** `PIPELINE_UPDATE.md`
- **Safety impact:** none
- **Done when:** PIPELINE_UPDATE.md is staged and checked against `oled_display.c`, the driver and `mw.cpp:1130`.
- **Status:** done 2026-10-14
- **Result:** PIPELINE_UPDATE.md stages `Hardware_Bus_Pipeline.md` ( Overview: I2C is polled and has no DMA, checked at `bus_i2c_stm32f30x.c:261-284`; i2cRead/Write blocking at 400 kHz; new bullets for the shared I2C1 budget and OLED failure ), `User_Space_API.md` ( OLED is deferred; static buffers; User mode only ) and the CHANGELOG at commit ( Fixed entry; the v4.0.0 'non-blocking' lines now point at it; version line ). A CLAUDE.md OLED gotcha was added directly under Architecture.

### 5. Topic review

**Description.** `pluto-reviewer` on the whole topic diff, read with `git diff -w`: about 200 lines of the
driver are clang-format whitespace only and stay. Fix any BLOCKING findings and re-run the gate. The other copy's
review was already done; this one checks the code as merged.

- **Depends on:** 4
- **Skills / agent:** pluto-reviewer agent
- **Files:** -
- **Safety impact:** none
- **Done when:** no BLOCKING findings left open.
- **Status:** done 2026-10-14
- **Result:** no BLOCKING findings. Flush logic, bounds, mode switches, the boot-only splash and the API patch bump are confirmed; the driver reflow is whitespace only ( the only real change is `i2c_OLED_send_run` ). Fixed: the CLAUDE.md / README `oledPanelCleared` rule, which was wrong ( the function is static, and zeroing the shadow is only correct after a full clear ); `Oled_display_Clear` now returns when `! OledInitStatus`; the `Oled.h` timing comment; PIPELINE_UPDATE ( `:165`, the `i2cInit` DMA line ); stale CHANGES cost and user-project sections; HISTORY width. Left as is: see the decisions log.

### 6. Graph refresh & commit

**Description.** `graphify update .` and `python tools/graph_labels.py`, then `/pluto-commit`: all-target build,
**FW 3.11.2 → 3.11.3, API 1.4.0 → 1.4.1** ( behaviour change only ), doc promotion, staged change and drafted
message. The message notes the driver reformatting and the deferred `Oled_Update`. **Restore `PlutoPilot.cpp` to HEAD ( or leave it unstaged ) before the commit build**, so the RAM figure is the stock one ( the review could not tell which `PlutoPilot.cpp` the task 1 build used ). The user commits.

- **Depends on:** 5
- **Skills / agent:** graphify, pluto-commit
- **Files:** -
- **Safety impact:** none
- **Done when:** staged, message drafted, topic Closed.
- **Status:** done 2026-10-14
- **Result:** `PlutoPilot.cpp` restored to HEAD ( the user discarded the test code ); graph refreshed; FW 3.11.3 / API 1.4.1; all three targets build with no new warnings ( PRIMUS_X2_v1 101.3 / 16.8 KB, PRIMUSX2 101.2 / 16.9, PRIMUS_V5 101.3 / 16.8 ); Hardware_Bus_Pipeline and User_Space_API promoted; CHANGELOG Fixed entry, Added cross-reference and version line; staged, message in `.git/MAGISV2_COMMIT_MSG.txt`.

## Decisions log

Newest last. One line each: `YYYY-MM-DD [decision|assumption|out-of-scope|risk] text`.

- 2026-10-10 [decision] Planned in the separate copy as `fix`, design A ( RAM framebuffer and rate-limited diff flush ); telemetry stays live in flight; the TASKS.md flow was skipped there.
- 2026-10-14 [decision] Merge into MagisV2 as `fix`: the code was copied from the separate copy; the merge scout found the same base ( `fc6879b` ) and no reverts.
- 2026-10-14 [decision] The A/B flight and the validation hover passed in the separate copy ( the user's report ). One confirmation hover on this repo's build ( task 2 ).
- 2026-10-14 [decision] The user copies the OLED wiki across from the separate copy; task 3 checks it against the code.
- 2026-10-14 [decision] Keep the clang-format reflow of `display_ug2864hsweg01.c`; review with `git diff -w`.
- 2026-10-14 [decision] Versions at commit: FW 3.11.3, API 1.4.1 ( behaviour change only, signatures unchanged ).
- 2026-10-14 [out-of-scope] The user-loop timer wrap ( `mw.cpp:156` / `:1134`: `plutoLoop` stalls if Dev mode first comes on after ~35.8 min ). It is the Planned topic `user-loop-timer-wrap`.
- 2026-10-14 [out-of-scope] The 2 s start-up splash still writes the panel directly ( boot only, before arming ); `PlutoPilot.cpp` from the user project stays out.
- 2026-10-14 [risk] `Oled_display_Clear` ( `oled_display.c:507` ) does not check `OledInitStatus`. It is reachable only from user code while disarmed; minor.
- 2026-10-14 [decision] The RAM cost in the stock build is +2 KB ( `oledSystemBuffer` plus `oledShadowBuffer`, which is now always linked ); 16.8 / 40 KB is accepted.
- 2026-10-14 [decision] Task 2 passed on the user's report. The test `PlutoPilot.cpp` ( Traffic Hazard ) is in the working tree and must not be committed.
- 2026-10-14 [decision] Kept as is after the review: the failure score does not trip at a steady 1-in-5 failure rate ( a documented design choice ); `i2c_OLED_send_changed_bytes` has no callers left ( the linker drops it ); no HISTORY rows added to `Oled.h` / `oled_display.h` ( the latter was already garbled at HEAD ).
- 2026-10-14 [risk] A connector failing just under 1 write in 5 keeps the OLED on, and each failed write blocks for `I2C_LONG_TIMEOUT` plus an I2C1 re-init ( estimated 10 ms or more ). This is a possible follow-up, for example counting total failures while armed.
- 2026-10-14 [decision] Gate re-run after the review fixes: no new warnings; 102.7 KB / 17.8 KB, because the Traffic Hazard `PlutoPilot.cpp` is in the tree. The stock figures are task 1's ( 101.3 / 16.8 KB, built before the test file went in ). The commit build must use the stock `PlutoPilot.cpp`.
