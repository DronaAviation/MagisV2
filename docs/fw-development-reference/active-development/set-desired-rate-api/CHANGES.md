# Set desired rate API - Changes

[README](README.md) · [TASKS](TASKS.md) · [INVESTIGATION](INVESTIGATION.md) · [TESTING](TESTING.md) · [TEST_CODE](TEST_CODE.md)

## Task 2: setter stores a clamped user rate; getter returns it ( 2026-11-15 )

| File | Line | Change | Why |
|---|---|---|---|
| [FC-Config-Setpoints.cpp](../../../../src/main/API-Src/FC-Config-Setpoints.cpp) | :157 `setDesiredRate` | Clamps with `constrain` to `CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX` ( 100 ) / `CONTROL_RATE_CONFIG_YAW_RATE_MAX` ( 255 ), stores into `userDesiredRate [ ANGLE ]`, sets `isUserDesiredRate [ ANGLE ]`, writes `( uint8_t )` into `currentControlRateProfile->rates [ ANGLE ]`. An unknown `angle_e` returns without change. | The old setter narrowed `int32_t` into `uint8_t` unchecked ( 3 baseline `-Wconversion` warnings, now gone ) and kept no copy, so the flip code's 75 had nothing to restore. Same limits as MSP_SET_RC_TUNING. Not written to EEPROM. |
| [FC-Config-Setpoints.cpp](../../../../src/main/API-Src/FC-Config-Setpoints.cpp) | :149 `getDesiredRate` | Returns `userDesiredRate [ ANGLE ]` when set, else the live `rates [ ANGLE ]`; 0 for an unknown `angle_e`. | It returned the PID's computed rate setpoint `desiredRate [ ]`, a different quantity, so a read-back never matched ( T16 ). |
| [API-Utils.h](../../../../src/main/API/API-Utils.h) | :80-83 | `STOCK_ROLL_PITCH_RATE 75`. | Names the value `flip ( false )` and the flip exits have always written: the effective stock roll / pitch rate ( the config default is 0 ). |
| [API-Utils.h](../../../../src/main/API/API-Utils.h) | :152 | Declares `uint8_t userRollPitchRate ( uint8_t axis )` ( C linkage block ). | For task 3: `acrobats.cpp` restores this instead of 75. |
| [API-Utils.cpp](../../../../src/main/API-Src/API-Utils.cpp) | :230 | `userRollPitchRate`: the held user rate for `FD_ROLL` / `FD_PITCH`, else `STOCK_ROLL_PITCH_RATE`. | Single place for the "user value or 75" rule. |

Reuses the existing, previously unused `userDesiredRate [ 3 ]` / `isUserDesiredRate [ 3 ]` ( API-Utils.cpp:140 / :145 ): no new RAM. Nothing clears the flag, so a set rate holds until reboot or the next call ( decision ).

Build ( PRIMUS_X2_v1 ): gate green, no new `src/` warnings, the 3 setter `int32_t -> uint8_t` warnings fixed. Flash 122.5 KB, RAM 20.5 KB ( unchanged at the summary's resolution ).

Not yet effective on its own: `flip ( false )` still writes 75 on every disarmed loop until task 3.

Reviewer ( 2026-11-15 ): 0 blocking. Fixed from its findings: the test code skips the out-of-range step while armed
( `PlutoPilot.cpp`, marker "STEP 2 SKIPPED ARMED" ), and the `userRollPitchRate ( )` comments no longer describe task 3
as already landed. Left as noted, out of scope: the setter writes the live profile, so an app EEPROM save or a stick
acc-trim persists the rate and a profile reload drops the live value while the getter still reports the user one
( for the wiki, tasks 4 / 7 ).

## Task 3: flip code restores the user rate instead of 75 ( 2026-11-15 )

[acrobats.cpp](../../../../src/main/flight/acrobats.cpp): new `static void restoreRestRates ( )` ( :83 ) writes
`userRollPitchRate ( FD_PITCH / FD_ROLL )`, i.e. the held `setDesiredRate` value or the stock 75. `API/API-Utils.h`
included for it.

Every roll / pitch rate write and every `flipState = 0` in `flip ( )`, classified:

| Place ( line after the change ) | What it is | Before | After |
|---|---|---|---|
| Watchdog, 8 s ( :97 ) | exit to normal flight ( or disarm ) | 75 / 75 | `restoreRestRates ( )` |
| ASCEND 2.2 s timeout ( :121 ) | exit to normal flight | rates left at 75 / 75 from the ASCEND body | `restoreRestRates ( )` added |
| ASCEND body ( :126-127 ) | flown while flipping ( ANGLE on ) | 75 / 75 | unchanged |
| ASCEND -> PITCHING / PITCHINGPOS | flip axis | 110 | unchanged |
| HOLD body ( :250-251 ) | flown while flipping, 1.5 s | 75 / 75 | unchanged |
| HOLD end ( :264 ) | exit to normal flight | rates left at 75 / 75 from the HOLD body | `restoreRestRates ( )` added |
| HOLDPOS body ( :340-341 ) | flown while flipping, 1.5 s | 75 / 75 | unchanged |
| HOLDPOS end ( :350 ) | exit to normal flight | rates left at 75 / 75 | `restoreRestRates ( )` added |
| `flip ( false )` ( :359 ) | every disarmed loop | 75 / 75 | `restoreRestRates ( )` |
| PITCHING 2 s timeout, SLOWDOWNANDEXIT back 2 s timeout | `mwDisarm ( )` | rates as the flip left them, then 75 / 75 from `flip ( false )` next loop | same; `flip ( false )` now restores the user rate |
| SLOWDOWNANDEXIT left, PITCHINGPOS, SLOWDOWNANDEXITPOS ( both ) 2 s timeouts | `flipState = 0` without disarm or rate reset: flip axis stays 110 until disarm | unchanged by decision ( mini-grill ): abnormal-exit behaviour is a separate finding |

With no `setDesiredRate` call, `userRollPitchRate ( )` returns 75, so every path ends exactly as before ( the three
added calls write the 75 the rates already held ). Build ( PRIMUS_X2_v1 ): gate green, no new warnings, flash 122.6 KB,
RAM 20.5 KB.

Reviewer ( 2026-11-15, task 3 ): 0 blocking, 2 nits, nothing changed. It re-read every path and confirmed that with no
user rate each one ends at 75 / 75 as before ( the three added calls write the value the rates already held ), and that
the `mwDisarm ( )` aborts restore through `flip ( false )` on the next loop. Nit kept as is: the `API/API-Utils.h`
include pulls in the laser header chain and adds `lib/` warning noise at `acrobats.cpp:60` ( `lib/` never fails the
gate, and `pid.cpp` / `mw.cpp` include the same header ).

## Task 4: FC-Config.h doc comments ( 2026-11-15 )

[FC-Config.h](../../../../src/main/API/FC-Config.h): the `getDesiredRate` / `setDesiredRate` doxygen is rewritten.
Signatures are unchanged. Banner `Last Modified` updated and a HISTORY row added.

| Statement in the header | Checked against |
|---|---|
| Range 0-100 roll / pitch, 0-255 yaw, clamped | `FC-Config-Setpoints.cpp` `setDesiredRate`; `rc_controls.h:116 / 119` |
| Holds until next call or reboot | nothing clears `isUserDesiredRate [ ]` ( grep ) |
| No call: roll / pitch 75, yaw profile value ( 0 by default ) | `flip ( false )` via `userRollPitchRate ( )`; `config.cpp:319` |
| Yaw acts in every mode; full-stick rate grows with rate + 20 | `pid.cpp:793` ( `pidRewrite`, default controller ) |
| Roll / pitch only outside ANGLE mode; rate + 20 | `pid.cpp:812-813` |
| Flip returns to the value; a timed-out flip keeps its rate until disarm | `acrobats.cpp` `restoreRestRates ( )` exits; the four abnormal timeouts ( task 3 table ) |
| Saved if settings are saved while set | `saveConfigAndNotify ( )`: `mw.cpp:262 / 277`, `acceleration.cpp:128 / 186 / 620`, `compass.cpp:110`; app `MSP_EEPROM_WRITE` |
| App loading settings replaces the live value; getter keeps the set one | `serial_msp.cpp:1399` `MSP_SELECT_SETTING`, `:1624` `MSP_RESET_CONF` -> `readEEPROM ( )` |
| Getter returns 0 for an unknown axis; setter ignores it | `FC-Config-Setpoints.cpp` |

`API_Version` is not bumped here: it moves to the commit task ( 9 ), with `FW_Version`, as on the rest of this branch.
Gate green ( flash 122.6 KB, RAM 20.5 KB ). Comment-only change, so no task reviewer run; the topic review ( task 8 )
covers the header.

## Task 5: build gate ( 2026-11-15 )

Clean build, PRIMUS_X2_v1, `PROJECT=AltX`, with the task-1 test code in `PlutoPilot.cpp`:

| | Value |
|---|---|
| Gate | green: no new warnings under `src/`, 0 outside `src/` / `lib/` |
| Warnings fixed | 3 ( `setDesiredRate` `int32_t -> uint8_t` ) |
| Flash | 122.6 KB / 256 KB ( text 124324 + data 1316 B ) |
| RAM | 20.5 KB / 40 KB ( data 1316 + bss 19768 B ) |
| Delta | about +0.1 KB flash over the test-code-only build of task 1 ( 122.5 KB ); RAM unchanged ( reused `userDesiredRate [ ]` ) |
| Output | `Build/PRIMUS_X2_v1/AltX_PRIMUS_X2_v1_3.15.8.hex` |

Other targets are built at commit ( task 9 ).

## Task 7: API wiki and staged pipeline docs ( 2026-11-15 )

- **New** `docs/API/FC_CONFIG_API_WIKI.md`: `setDesiredRate` / `getDesiredRate` only. Range and clamp, where each axis
  acts ( and that roll / pitch do nothing in ANGLE mode with the default controller ), lifetime, flips, saving, three
  examples in the hooks they belong in, pitfalls, "Changes in 1.8.7".
- **New** `PIPELINE_UPDATE.md`: a "Stick rates" section for `User_Space_API.md` ( with a Mermaid diagram whose every
  edge is a source line ), a "Stick rates and the flip" section for `Core_PID_Controller.md`, and three line references
  this topic moved: `acrobats.cpp:84` -> `:94` in `User_Space_API.md` and `Power_BMS_Pipeline.md`, `FC-Config.h:95` ->
  `:97` in `Core_PID_Controller.md`. References checked as unchanged: `FC-Config-Setpoints.cpp:84 / 90-98`,
  `API-Utils.cpp:209 / 209-220`, `API-Utils.h:73 / 78`, `acrobats.h:44`.
- **`spec-compliance` workflow** ( 310 requirements extracted, 8 checked ): 7 hold, 0 divergences, 1 refuted. Two
  undocumented behaviours were verified in the source and are now documented; both were true before this topic:
  1. a `setDesiredRate ( AG_ROLL | AG_PITCH )` call **during** a flip overrides the flip's spin rate, because
     `userCode ( )` runs after `flip ( true )` and before the PID and the flip writes 110 only once. A low rate can
     trip the PITCHING 2 s timeout, which disarms in the air. Documented, and guarded in code by task 10.
  2. under a non-default `pidController` the rates scale that controller's `dynP8 / dynI8 / dynD8` in every mode
     ( `annexCode`, `mw.cpp:359-378` ), so a high rate weakens the gains at full stick even in ANGLE mode.
- Fixed with it: the `FC-Config.h` comment ( same two points ), the INVESTIGATION scope wording, the `PlutoPilot.cpp`
  link path in TASKS.md ( the file is at the repo root ), and the README claim that the commit restores the stock
  `PlutoPilot.cpp` ( `pluto-commit` has no such step: task 9 does it by hand ).

Gate green after the doc edits ( flash 122.6 KB, RAM 20.5 KB ).

## Task 10: refuse a roll / pitch rate change during a flip ( 2026-11-15 )

| File | Line | Change | Why |
|---|---|---|---|
| [FC-Config-Setpoints.cpp](../../../../src/main/API-Src/FC-Config-Setpoints.cpp) | :81 | `#include "flight/acrobats.h"` ( last include ) | `flipState` for the guard. Eight other modules already include it; its macros do not clash. |
| [FC-Config-Setpoints.cpp](../../../../src/main/API-Src/FC-Config-Setpoints.cpp) | :183 | `if ( ANGLE != AG_YAW && flipState != 0 ) return;` after the hold, before the live write | `userCode ( )` runs after `flip ( true )` and before the PID, and the flip sets its spin rate once, so a live write during a flip replaced it; a low rate could trip the PITCHING 2 s timeout, which disarms in the air. The value is kept and applied by the flip's exit. |
| [FC-Config.h](../../../../src/main/API/FC-Config.h) | setDesiredRate comment | Describes the hold-until-flip-ends behaviour | Was a "do not call during a flip" warning ( task 7 ). |
| `docs/API/FC_CONFIG_API_WIKI.md` | "Flips", Pitfalls, "Changes in 1.8.7" | Same | |
| [PIPELINE_UPDATE.md](PIPELINE_UPDATE.md) | - | New bullet "A running flip keeps its rates", a Mermaid edge for the guard, and a table correcting the four pipeline line references the include shifted ( `FC-Config-Setpoints.cpp:84` -> `:85`, `:90-98` -> `:91-99` ) | The include moved every line below it down by one. |

Reviewer ( 2026-11-15 ): 0 blocking, 0 should-fix, 2 doc nits, both fixed. It traced where a value held during a flip is
applied ( the four normal exits via `restoreRestRates ( )`; the two disarming timeouts via the next `flip ( false )` ),
confirmed `flipState` is written only in the main loop ( no ISR, no atomicity issue ), and that a disarmed set cannot be
swallowed ( `flip ( false )` zeroes `flipState` before `userCode ( )`, and `plutoInit ( )` runs while it is still 0 ).
Nits fixed: the wiki no longer implies a timed-out flip keeps flying at 110, and `PIPELINE_UPDATE.md` records that the
four non-restoring timeouts are unreachable today because `flipDirection` is only ever 0 ( `BACK_FLIP` ).

Gate green: flash 122.7 KB ( +0.1 KB for the guard ), RAM 20.5 KB.

## Task 8: whole-topic review ( 2026-11-15 )

`pluto-reviewer` over the full topic diff against HEAD `73022c3`: **0 blocking**, 2 should-fix, 2 nits, all fixed.

What it verified beyond the per-task reviews: with no `setDesiredRate` call every changed path still writes 75 / 75
( the only no-call difference is the getter, which is the fix ); the set / arm / disarm / flip / reload sequences match
the docs; a set on the tick a flip ends is applied; the four non-restoring flip exits are dead code ( only `FLIP_BACK`
is ever commanded ); every `PIPELINE_UPDATE.md` line reference holds; the temporary `PlutoPilot.cpp` test code is
flagged in the README and task 9, with a copy in `TEST_CODE.md`; no stray debug code in `src/`.

| Finding | Fix |
|---|---|
| SHOULD-FIX: "if the app loads other settings, the drone uses those" is false for roll / pitch, and a profile change *saves* before it loads | `FC-Config.h` and the wiki now say: a profile change saves too; yaw takes the loaded value; roll / pitch are written back from the held value on the next disarmed loop ( `MSP_SELECT_SETTING` -> `writeEEPROM ( )` then `readEEPROM ( )`, `serial_msp.cpp:1405-1406`; `flip ( false )`, `acrobats.cpp:359` ) |
| SHOULD-FIX: `FC-Config.h` still said a timed-out flip keeps its rate until disarm, contradicting the wiki | Header now matches the wiki: the value is handed back at once or at the disarm |
| NIT: `mw.cpp:358` is the `rcCommand` lookup | `:359` in `PIPELINE_UPDATE.md` and `CHANGES.md` |
| NIT: untracked `spec-compliance/` at the repo root | moved to `<topic>/spec-compliance/` |

Also fixed with them: the getter comment said the no-call value is 75, but during a flip it returns the flip's own
rate.

Gate re-run after the edits: green, flash 122.7 KB, RAM 20.5 KB.
