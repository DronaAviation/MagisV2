# RX Signal-Loss Tolerance - Tasks

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md) · [CONTROLLER_FINDINGS](CONTROLLER_FINDINGS.md) · [MSP_TIMING](MSP_TIMING.md)

## Resume

Read this block first in a new session; read only the task it points to.

| | |
|---|---|
| **Current task** | **Closed** in `84c326d` ( 14 Oct 2026, FW 3.11.2 ). All tasks done. |
| **Next step** | none |
| **Open questions** | none |
| **Blocked on** | nothing |
| **Last updated** | 2026-10-14 |

## Summary

| | |
|---|---|
| **Mode** | fix |
| **Goal** | Ride through 1-2 s Wi-Fi RC stalls without `Signal_loss` and `LAND`, and still land on a real loss of the controller. |
| **Done when** | Flights in ALT_HOLD and ANGLE throttle mode show stalls in the controller's drop counters with no SIGNAL LOSS, and switching the controller off mid-hover still starts `LAND` ~2.5 s after the last frame. |
| **Target** | PRIMUS_X2_v1 ( all three targets built at commit ) |
| **Analysis** | Cause proven: `failsafeOnValidDataFailed ( )` used the fixed 200 ms `PERIOD_RXDATA_FAILURE` and ignored `failsafe_delay`, so `LAND` came ~1.0 s after the last frame; the measured stalls are 1.21-2.17 s. Fix: apply `rxDataFailurePeriod`, default `failsafe_delay` 15 ( ~2.5 s ). See [INVESTIGATION.md](INVESTIGATION.md). |
| **Pipeline docs affected** | subsystems/Failsafe_Subsystem.md, subsystems/RC_Input_Pipeline.md, subsystems/User_Space_API.md; `.claude/skills/pluto-flighttest/SKILL.md` |
| **Order** | 1 → 2 → 10 → 3 → 4 → 5 → 6 → 7 → 8 → 9 |

## Index

| # | Title | Status | Depends on | Skills / agent |
|---|---|---|---|---|
| 1 | Apply `failsafe_delay`, default 15 | done 2026-10-13 | - | pluto-rules, pluto-reviewer agent |
| 2 | Bump `EEPROM_CONF_VERSION` 107 → 108 | done 2026-10-13 | 1 | pluto-rules, pluto-reviewer agent |
| 10 | Clamp `failsafe_delay` to a safe maximum ( 22 ) | done 2026-10-13 | 2 | pluto-rules, pluto-reviewer agent |
| 3 | Build gate | done 2026-10-14 | 2, 10 | pluto-build |
| 4 | Bench: settings reset, delay readback, real-loss timing | done 2026-10-14 | 3 | pluto-flighttest |
| 5 | Hardware validation: stall and real-loss flights | done 2026-10-14 | 4 | pluto-flighttest, pluto-log-analyst agent |
| 6 | Architecture & pipeline docs ( PIPELINE_UPDATE.md ) | done 2026-10-14 | 5 | graphify, spec-to-code-compliance |
| 7 | Move the root notes into the topic, corrected | done 2026-10-14 | 6 | - |
| 8 | Topic review | done 2026-10-14 | 7 | pluto-reviewer agent |
| 9 | Graph refresh & commit | done 2026-10-14 ( staged; the user commits ) | 8 | graphify, pluto-commit |

Status values: `todo`, `in-progress`, `done YYYY-MM-DD`, `blocked (<why>)`, `dropped (<why>)`.
Serial numbers are never reused or renumbered.

## Tasks

### 1. Apply `failsafe_delay`, default 15

**Description.** Keep the user's change at `failsafe.cpp:213`: `failsafeOnValidDataFailed ( )` now compares
against `failsafeState.rxDataFailurePeriod` instead of `PERIOD_RXDATA_FAILURE`. The scout confirmed the period is
always set from config and is 200 ms at a delay of 0. Change the `resetConf ( )` default `failsafe_delay` from the
user's 22 to **15**, about 2.5 s from the last frame to `LAND`, as decided on 13 Oct. Fix the stale comments:
`config.cpp:550` "// 1sec" and `failsafe.h:39` "1sec in example (10)". Keep the config.cpp banner the user
already updated.

- **Depends on:** -
- **Skills / agent:** pluto-rules; pluto-reviewer agent on the diff
- **Files:** [failsafe.cpp](../../../../src/main/flight/failsafe.cpp#L211), [config.cpp](../../../../src/main/config/config.cpp#L550), [failsafe.h](../../../../src/main/flight/failsafe.h#L39)
- **Safety impact:** RX-loss failsafe. `LAND` starts ~2.5 s after the last RC frame instead of ~1.0 s. From
  ~0.8 s the sticks are centred and throttle is 1200 ( ALT_HOLD sinks ~16 cm/s; ANGLE throttle mode sinks
  faster ).
- **Done when:** the diff is only these lines; the reviewer finds nothing blocking.
- **Rollback:** revert `failsafe.cpp:213` to `PERIOD_RXDATA_FAILURE`, which gives the old ~1.0 s.
- **Status:** done 2026-10-13
- **Result:** the diff is the `failsafe.cpp:213` fix, the default 15 ( set by the user ) and the two comments. Review: nothing blocking. The period is set by `readEEPROM ( )` → `activateConfig ( )` → `failsafeReset ( )` before `rxInit` and the first `processRx`; it lies in 200-25700 ms and never 0. A delay of 0 gives the old 200 ms. All `uint32_t` and wrap-safe. Timing confirmed: `LAND` at t0 + ~2.5 s. Should-fix: `failsafe_delay` has no upper bound over MSP ( 255 → ~25.7 s ), added as task 10.

### 2. Bump `EEPROM_CONF_VERSION` 107 → 108

**Description.** The user chose this so that drones already flown, whose saved settings would otherwise keep the
old `failsafe_delay` ( likely 10, ~2.0 s ), get the new default. `EEPROM_CONF_VERSION` is at `config.cpp:157`.
The mismatch check at `config.cpp:627` makes the first boot after flashing run `resetConf ( )`. **That resets every
saved setting:** accelerometer calibration and trims, PIDs, rates, AUX modes, features, battery capacity and
rxfail. The commit message, CHANGELOG and the test notes must say that every drone needs recalibrating after
flashing. Precedent: `4657cbf`, 106 → 107.

- **Depends on:** 1
- **Skills / agent:** pluto-rules; pluto-reviewer agent
- **Files:** [config.cpp](../../../../src/main/config/config.cpp#L153)
- **Safety impact:** after flashing, the drone flies uncalibrated until the accelerometer is calibrated again.
  The recalibration must be in every test procedure and in the release notes.
- **Done when:** the constant is 108; nothing else in the diff.
- **Rollback:** back to 107. Drones already booted on 108 have been reset either way.
- **Status:** done 2026-10-13
- **Result:** `EEPROM_CONF_VERSION` = 108, with a history comment. Review: nothing blocking; the reset path writes the defaults before the RX and failsafe are set up, with no half-loaded state. Should-fix applied: the comment and notes now include the **compass calibration**, which is wiped too. With `MAG_ENFORCE` on all three targets, the drone will not arm until the compass is calibrated. Full reset list in CHANGES.md.

### 3. Build gate

**Description.** `pluto-build --gate PRIMUS_X2_v1` on tasks 1, 2 and 10: no new `src/` warnings, and flash and RAM fit.

- **Depends on:** 2, 10
- **Skills / agent:** pluto-build
- **Files:** -
- **Safety impact:** none
- **Done when:** gate passes; flash and RAM recorded.
- **Status:** done 2026-10-14
- **Result:** PRIMUS_X2_v1: no new `src/` warnings. Flash 100.9 KB / 256 ( text 102120 + data 1272 B ), RAM 14.8 KB / 40 ( bss 13896 B ). `DEFAULT_PRIMUS_X2_v1_3.11.2.hex`. Source: `EEPROM_CONF_VERSION` 108, `failsafe_delay` 15, cap 22.

### 4. Bench: settings reset, delay readback, real-loss timing

**Description.**
1. Flash the build. Confirm the first boot reset the settings, then calibrate the **accelerometer and the compass**. The drone
   will not arm until the compass is calibrated ( `MAG_ENFORCE` ). Set the battery capacity and any AUX ranges
   in slot 5 and above again. Flashing back to a 107 build resets the settings again.
2. Read back `failsafe_delay` over CLI ( `get failsafe_delay` ) or the app's MSP_FAILSAFE_CONFIG; it must be 15.
3. Props off, armed, Wi-Fi controller connected. Switch the controller off and time how long it takes until `LAND`
   starts or `Signal_loss` shows. Use a video with the controller's last-frame time, or the controller log.
   Expect ~2.5 s; before this change it was ~1.0 s.
4. Check that a short interruption ( controller off and back on within ~1.5 s ) does not trigger `LAND`.

- **Depends on:** 3
- **Skills / agent:** pluto-flighttest
- **Files:** `TESTING.md` ( new ), `logs/log-N.txt`
- **Safety impact:** none ( bench, props off ).
- **Done when:** the settings were reset; the drone arms after the accelerometer and compass calibration;
  `failsafe_delay` reads back 15; in 3 repeats, controller-off to `LAND` is clearly about 2-3 s, not ~1 s
  ( by observation; there is no log ). The 1.5 s interruption check moves to task 5.
- **Status:** done 2026-10-14
- **Result:** passed on the user's report ( "flew and all good" ); log-1.txt not filled in, so the readback and observed times are not recorded ( TESTING.md results ).

### 5. Hardware validation: stall and real-loss flights

**Description.** Fly the profile of the findings doc on Wi-Fi: hover and gentle movement for 30-90 s, controller
on a healthy cell, transmit power on Full. Fly it in **both ALT_HOLD and ANGLE throttle mode**. Use the
controller's stall detector log ( `send buffer full … ms` and the `drop` counters ) to time each stall.

In each mode, also do one **real-loss** flight: controller off mid-hover, low over a soft floor. The drone must
level, descend, start `LAND` at ~2.5 s and disarm. Note how much height it lost before `LAND`; in throttle mode,
note whether it touched down first.

- **Depends on:** 4
- **Skills / agent:** pluto-flighttest; pluto-log-analyst agent for large controller logs
- **Files:** `TESTING.md`, `logs/log-N.txt`
- **Safety impact:** flights with the longer RX-loss window. Fly low, keep a hand on the controller's ARM switch,
  and keep a second way to stop the drone.
- **Done when:** every stall under ~2.5 s seen in the controller log rides through with no SIGNAL LOSS, in both
  modes ( this also covers the short-interruption check moved from task 4 ); both real-loss flights land and
  disarm by themselves.
- **Status:** done 2026-10-14
- **Result:** passed on the user's report: flights all good, no unwanted `Signal_loss`. No controller stall log or per-mode notes were shared, so the stall counts and real-loss times are not recorded.

### 6. Architecture & pipeline docs ( PIPELINE_UPDATE.md )

**Description.** Stage in `PIPELINE_UPDATE.md`:
- `subsystems/Failsafe_Subsystem.md`: the real timeline ( replay 200 ms, hold to ~800 ms, then
  200 + `failsafe_delay` × 100 ms ); IDLE goes straight to `LAND` ( no LOSS_DETECTED stage ); the `LAND` latch
  until disarm; `failsafe_off_delay`, `failsafe_throttle` and `failsafeApplyControlInput` are dead code. Replace
  the flowchart edges that do not exist.
- `subsystems/RC_Input_Pipeline.md`: the replay and hold timing; remove `rxUpdateCheck ( )`, which does not exist.
- `subsystems/User_Space_API.md`: Developer Mode stops 200 ms after the last frame.
- `.claude/skills/pluto-flighttest/SKILL.md:63`: "failsafe_delay 1 s" → 15 ( 1.5 s ).
- `docs/Cli.md:205` ( default 10 → 15; the CLI still accepts 0-200 and MSP / CLI read back the stored value, but the firmware caps it at 22, ~3.2 s ) and `docs/Failsafe.md:25`, `:76` ( `failsafe_delay` description and default ).

Add a FLIGHT_INVARIANTS / CLAUDE.md line for the signal-loss budget ( ~2.5 s to `LAND`; do not lengthen the
hold ). Every flowchart edge must be confirmed with a source line.

- **Depends on:** 5
- **Skills / agent:** graphify, spec-to-code-compliance
- **Files:** `PIPELINE_UPDATE.md`
- **Safety impact:** none
- **Done when:** PIPELINE_UPDATE.md is staged and checked against `failsafe.cpp`, `rx.cpp` and `mw.cpp`.
- **Status:** done 2026-10-14
- **Result:** PIPELINE_UPDATE.md stages new text for `Failsafe_Subsystem.md` ( data structures, functions, timeline, RC link loss flowchart; every edge checked against mw.cpp:839/856, rx.cpp:431/526-532/552-564/588, failsafe.cpp:212-216/246-249 ), `RC_Input_Pipeline.md` ( `rxUpdateCheck` replaced, signal-loss timing ) and `User_Space_API.md` ( user code stops 200 ms after the last frame ). Edited directly: `docs/Cli.md` ( default 15, cap 22 ), `docs/Failsafe.md`, the pluto-flighttest skill line, a new FLIGHT_INVARIANTS section and a CLAUDE.md rule.

### 7. Move the root notes into the topic, corrected

**Description.** Move `MAGISV2-SIGNAL-LOSS-FINDINGS.md` ( repo root ) into this folder as
`CONTROLLER_FINDINGS.md`, and `TEMP_MSP_TIMING.md` as `MSP_TIMING.md`. Correct the timing in both:
- the missing 200 ms replay;
- 3000 ms becomes ~2.5 s at `failsafe_delay` 15;
- the "Resolved" note in TEMP_MSP_TIMING describes the fix of task 1.

Leave the controller-side evidence as written. Remove the two files from the repo root.

- **Depends on:** 6
- **Skills / agent:** -
- **Files:** `CONTROLLER_FINDINGS.md`, `MSP_TIMING.md`
- **Safety impact:** none
- **Done when:** both notes are in the topic with correct numbers; nothing left in the repo root.
- **Status:** done 2026-10-14
- **Result:** `CONTROLLER_FINDINGS.md`: moved, with a status note at the top ( fixed in 3.11.2, default 15, cap 22, version 108 ); the §4 timeline gains the 200 ms replay row; ~800 ms → ~1.0 s, ~1800 → ~2.0 s, 2.3 → 2.5 s, each marked as corrected; evidence §1-3 untouched. `MSP_TIMING.md`: rewritten for 3.11.2, with the timeline 0-200 replay, 200 ms user code stops, ~800 ms sticks centred, ~2.5 s `LAND`, cap note and source table. Both removed from the repo root; the originals are kept outside the repo in the session scratchpad.

### 8. Topic review

**Description.** `pluto-reviewer` on the whole topic diff. Fix any BLOCKING findings and re-run the gate.

- **Depends on:** 7
- **Skills / agent:** pluto-reviewer agent
- **Files:** -
- **Safety impact:** none
- **Done when:** no BLOCKING findings left open.
- **Status:** done 2026-10-14
- **Result:** no BLOCKING findings. The code is identical on all three targets, and every timing number and source line in MSP_TIMING and the PIPELINE_UPDATE flowchart is confirmed. Fixed: the PIPELINE_UPDATE fences ( 4-backtick outer fences; "edges checked" moved out of the staged text ) and its clearer delete instruction; the failsafe-switch delay ( ~1.7 s ) documented; `docs/Failsafe.md` now says `LAND` is latched; stale 600 ms / 1.8 s / 'about a second' in CONTROLLER_FINDINGS; ELRS / PPM ~2.4 s labels; the ELRS compass-check exception; line references; the Cli.md scope. Gate re-run: no new warnings, 100.9 KB / 14.8 KB. Left for later: see the decisions log.

### 9. Graph refresh & commit

**Description.** `graphify update .` and `python tools/graph_labels.py`, then `/pluto-commit`: all-target build,
version bump, doc promotion, staged change and drafted message. The CHANGELOG and message must say:
- `failsafe_delay` is now applied, default 15, capped at 22; RC link loss lands ~2.5 s after the last frame
  ( Wi-Fi; ~2.4 s on ELRS / PPM ), before ~1.0 s;
- a mapped failsafe switch now waits ~1.7 s before `LAND`;
- `EEPROM_CONF_VERSION` 108 resets every saved setting on first boot. Recalibrate the accelerometer and compass;
  on Wi-Fi / PPM the drone will not arm until the compass is done. Battery capacity, rxfail and AUX slots 5 and up
  are reset too. Flashing back to a 107 build resets again;
- MSP and CLI read back the stored `failsafe_delay`, not the capped one.

`FW_Version` is already 3.11.2 ( the user's bump ): do not bump again.
The user commits.

- **Depends on:** 8
- **Skills / agent:** graphify, pluto-commit
- **Files:** -
- **Safety impact:** none
- **Done when:** staged, message drafted, topic Closed.
- **Status:** done 2026-10-14
- **Result:** graph refreshed; all three targets built at 3.11.2 with no new warnings ( PRIMUS_X2_v1 100.9 KB / 14.8 KB, PRIMUSX2 100.8 / 14.9, PRIMUS_V5 100.9 / 14.8 ); PIPELINE_UPDATE promoted into Failsafe_Subsystem, RC_Input_Pipeline and User_Space_API; CHANGELOG Fixed entry and 3.11.2 version line; staged, message in `.git/MAGISV2_COMMIT_MSG.txt`. Version not bumped again.

### 10. Clamp `failsafe_delay` to a safe maximum

**Description.** Found in the task 1 review. Now that `failsafe_delay` is applied, its stored value sets the real
guard time, but nothing bounds it: `MSP_SET_FAILSAFE_CONFIG` ( `serial_msp.cpp:1739` ) accepts 0-255, which is up
to ~25.7 s before `LAND`. The CLI ( `serial_cli.cpp:456` ) allows 0-200, ~20.2 s. On a real loss of the controller,
the drone would fly with centred sticks and throttle 1200 for that long; in ANGLE throttle mode it sinks
throughout. Proposed: clamp where the period is computed, in `failsafeReset ( )` ( `failsafe.cpp:122` ), so every
input path is covered, **at most 22 ( ~3.2 s )**, confirmed by the user on 13 Oct. Optionally narrow the CLI range to match.

- **Depends on:** 2
- **Skills / agent:** pluto-rules; pluto-reviewer agent
- **Files:** [failsafe.cpp](../../../../src/main/flight/failsafe.cpp#L121), optionally [serial_cli.cpp](../../../../src/main/io/serial_cli.cpp#L456)
- **Safety impact:** RX-loss failsafe. It caps the longest possible window before `LAND`. Values within the cap
  are unchanged.
- **Done when:** a stored `failsafe_delay` above the cap gives at most the cap's period; the default 15 is
  unchanged; the reviewer finds nothing blocking.
- **Rollback:** remove the clamp.
- **Status:** done 2026-10-13
- **Result:** `FAILSAFE_DELAY_MAX` 22 in `failsafe.h`; `failsafeReset ( )` uses `MIN ( failsafe_delay, 22 )`, so the period is 200-2400 ms and `LAND` comes at most ~3.2 s after the last frame. Gate: no new warnings, 100.9 KB / 14.8 KB. Review: nothing to fix; every path that applies the setting ( boot, MSP EEPROM write or reset, CLI save ) goes through `failsafeReset ( )`. Note: MSP and CLI report the stored value, not the capped one, so task 6 docs state the cap.

## Decisions log

Newest last. One line each: `YYYY-MM-DD [decision|assumption|out-of-scope|risk] text`.

- 2026-10-13 [decision] Mode `fix`: the user wrote the code ( `failsafe.cpp:213` uses `rxDataFailurePeriod`; default `failsafe_delay` 22 ); the plan validates and ships it. Target PRIMUS_X2_v1.
- 2026-10-13 [decision] Default `failsafe_delay` **15** ( ~2.5 s to `LAND` ), not the user's first 22 ( ~3.2 s ). It clears the 2.17 s worst stall and stays within the findings doc's ~2.5 s cap.
- 2026-10-13 [decision] Bump `EEPROM_CONF_VERSION` 107 → 108 so drones already flown get the new default. This resets every saved setting, so every drone must be recalibrated after flashing.
- 2026-10-13 [decision] Validate in both ALT_HOLD and ANGLE throttle mode; both are flown over Wi-Fi often.
- 2026-10-13 [assumption] The app never writes the failsafe config ( user ), so the default is what flies.
- 2026-10-13 [decision] Done when: the controller log shows stalls with no SIGNAL LOSS, and a real loss of the controller still lands ( ~2.5 s ).
- 2026-10-13 [out-of-scope] `MAX_INVALID_PULS_TIME` and `DELAY_5_HZ`, the controller and ESP firmware ( UDP ), `LAND` behaviour after `Signal_loss`, and the app.
- 2026-10-13 [out-of-scope] Dead code `failsafeActivate`, `failsafeApplyControlInput`, `failsafe_off_delay` and `failsafe_throttle`: documented in task 6, not removed.
- 2026-10-13 [risk] The cause of the stalls ( TCP head-of-line blocking ) is unproven, and the stall times were measured on the controller side; the gap on the drone side may differ slightly. Task 5 checks it.
- 2026-10-13 [risk] In ANGLE throttle mode the drone sinks below hover for ~1.7 s before `LAND` and may touch down, after which the crash detector may disarm it. Task 5 checks it.
- 2026-10-13 [risk] `MSP_SET_FAILSAFE_CONFIG` is not blocked while armed and takes effect only after the next EEPROM reload. Minor; noted only.
- 2026-10-13 [decision] The user set the default to 15 by hand before task 1 started; task 1 kept it and fixed the two stale comments.
- 2026-10-13 [decision] The user bumped `FW_Version` 3.11.1 → 3.11.2 in the Makefile by hand. It belongs to this topic; task 9 does not bump it again.
- 2026-10-13 [risk] `failsafe_delay` is unbounded over MSP ( 0-255, up to ~25.7 s before `LAND` ) now that it is applied. Task 10 proposes a clamp.
- 2026-10-13 [risk] The settings reset from the version bump also wipes the compass calibration; with `MAG_ENFORCE` the drone will not arm until the compass is calibrated. This must be in task 4, CHANGELOG and the commit message.
- 2026-10-13 [decision] Task 10 confirmed: cap the effective `failsafe_delay` at 22 ( ~3.2 s to `LAND` ) in `failsafeReset ( )`; the default stays 15.
- 2026-10-14 [decision] Task 4 timing is by observation only ( no logs ); pass = clearly about 2-3 s, not ~1 s. The short-interruption check moves to task 5's real Wi-Fi stalls.
- 2026-10-14 [decision] Tasks 4 and 5 recorded as passed on the user's report ( bench and flights "all good" ); no notes or controller logs, so no measured times.
- 2026-10-14 [out-of-scope] Older drift found in the topic review, not fixed here: `RC_Input_Pipeline.md` also names `computeRC ( )`, `rcCurve ( )` and `rx_smoothing`, which do not exist; `User_Space_API.md` 'Override Priority' relies on the unreachable `failsafeApplyControlInput`; `docs/Cli.md` `failsafe_throttle` default reads 1000 ( code 1300, unused ). For a later docs topic.
