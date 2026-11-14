# Scout brief ( verbatim, `pluto-scout`, 14 Nov 2026 )

## Scout brief: setDefault_PIDProfile ( ) reads past its tables and resets wrong gains (T06 / B1)

Graph: stale (built e72d098, HEAD 6d1b5cd; the only change since is FlightStatus, which does not touch this code). Mode: fix. Target: PRIMUS_X2_v1.

**Affected files:** `src/main/API-Src/FC-Config-PID.cpp:147-168`, `src/main/config/config.cpp:175-233` (`resetPidProfile`), `src/main/config/config.h` (only if the reset is exposed), `src/main/API/FC-Config.h:73-76` (doc comment), `Makefile:31-32`.

**Current behaviour (proven):**
- The loop runs `i = ROLL..PIDUSER` (0..10, `flight/pid.h:30-43`) over 5-entry `static const int` tables. Index 3 gives ALT P = 100, which is stale (`config.cpp:191` sets 128, EEPROM version 107 note at `config.cpp:151`). Indexes 4..10 read out of bounds and write PIDPOS, PIDPOSR, NAVR, LEVEL, MAG, VEL and USER.
- GCC already flags this. `tools/warnings-baseline.json:107` baselines "-Waggressive-loop-optimizations iteration 5 invokes undefined behavior", and `:108` baselines 3 int-to-uint8_t `-Wconversion` warnings. Because this is undefined behaviour, the written values are not predictable from the table layout, which explains USER 0/0/40.
- What the wrong values break:
  - LEVEL: the angle loop (`pid.cpp:813`).
  - VEL: the inner alt-hold loop (`altitudehold.cpp:728-751`).
  - USER: also the UWB velocity loop (`posControl.cpp:192-194`), so it is not a user-only slot.
  - MAG: heading hold.

**Facts:**
1. `resetPidProfile` is `static` (`config.cpp:175`) and its only caller is `config.cpp:525`. It also resets `pidController=1`, `yaw_p_limit`, `dterm/pterm/gyro_cut_hz`, `P_f/I_f/D_f`, `A_level/H_level/H_sensitivity`, `pid5_oldyw` and the GTUNE limits. An API "set default" should not touch those. There is no FC-Config wiki in `docs/API/`, and nothing there documents `setDefault_PIDProfile`. The only statement is the header's "all axes" (`FC-Config.h:74`).
2. There are no callers anywhere: not in the root `PlutoPilot.cpp`, not in API-Src, not in MSP. The function changes RAM only and never calls `writeEEPROM`. However, any later save writes the corrupted profile to EEPROM: MSP_EEPROM_WRITE when disarmed (`serial_msp.cpp:1638`), trims via `saveConfigAndNotify` (`mw.cpp:255`), or CLI. The get/set `profileIndex {ROLL,PITCH,YAW,PIDALT,PIDUSER}` mapping is correct (ROLL/PITCH/YAW equal PIDROLL/PIDPITCH/PIDYAW = 0..2), and range checks are done on the enum. The axis/pid enum naming is mixed (cosmetic only). Note that with `pidController==2` the R/P/Y gains come from `P_f`, so the API P8 values would be ignored. The default is 1.
3. No guard exists anywhere: `set_PIDProfile` and `MSP_SET_PID` (`serial_msp.cpp:1441`) both write gains while armed. Gains are read live every loop, so a mid-flight default changes ALT (100 to 128) and USER at once. With the fix, LEVEL/VEL are no longer touched.
4. Precedent from the recent API fix topics:
   - Each bumped FW and API by one patch at commit (estimate-get-rate: FW 3.15.4 / API 1.8.2; flight-status: FW 3.15.6 / API 1.8.4). The current values are 3.15.6 / 1.8.4, so this fix would be 3.15.7 / 1.8.5.
   - Each topic had README / INVESTIGATION / TASKS / TESTING / CHANGES / PIPELINE_UPDATE plus a "Changes in 1.8.x" section in its `docs/API/*_WIKI.md`.
   - The "Txx ( Bn )" sheet is not in the repo. The baro-temperature-api `SCOUT.md:58` records that it is the user's external test sheet.
5. There is no other stale ALT=100 default. `posControl.cpp:93 AltHoldfromPos = 100` is unrelated. The only duplicate of the gains is the API table itself.

**Questions for the user:**
1. Scope: reset only the 5 API slots (ROLL, PITCH, YAW, ALT, USER), or all 11 slots?
2. Should it be refused, or only documented, while armed? (`set_PIDProfile` is not guarded today.)
3. Should it also persist to EEPROM (it is RAM-only today)?
4. Should a new FC-Config wiki be created, or only the header comment updated?

**Docs to update:**
- `docs/fw-development-reference/fw-architecture-pipeline/subsystems/Core_PID_Controller.md:9`: one line on the API default semantics.
- `User_Space_API.md`: optional.
- The header comment in `FC-Config.h`.
- CHANGELOG at commit.
- Gate: expect 4 baselined warnings reported as fixed.

**Skills:** pluto-rules, pluto-build (`--gate PRIMUS_X2_v1`), cpp-pro (optional), pluto-reviewer, pluto-flighttest (bench check: get after set-default), pluto-commit.

**Recommended fix:** option (b) with a single source of truth.
- Make the firmware reset reachable, either by un-static `resetPidProfile` or by adding a `pidProfileDefaults ( pidProfile_t * )` declared in `config.h`.
- In `setDefault_PIDProfile`, fill a stack `pidProfile_t` from it and copy P8/I8/D8 for `PIDROLL, PIDPITCH, PIDYAW, PIDALT, PIDUSER` only. These give 40/10/30, 40/10/30, 150/70/50, 128/0/30 and 60/30/5.
- Delete the local tables. This removes the out-of-bounds write, the stale 100 and the 4 baselined warnings.
- The flash cost is a few bytes, there is no RAM cost, and the loop is not affected.

## Answers given after the brief

1. Scope: the five API slots only.
2. Armed: no guard; documented in the header comment.
3. EEPROM: RAM only, as today.
4. Docs: header comment plus one pipeline line; no new FC-Config wiki.
