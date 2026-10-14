# RX Signal-Loss Tolerance - Scout brief

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

`pluto-scout` brief from planning ( 13 Oct 2026 ), verbatim. The brief covered the user's diff at the time:
`failsafe_delay` applied and the default set to 22. The plan then chose 15; see the TASKS.md decisions log.

---

## Scout brief: validate the user's RX signal-loss tolerance diff (failsafe_delay now applied, default 10 -> 22)
Graph: stale (built cbd1c72, HEAD 386599b). I traced the code by reading source.

Subsystems: Failsafe, RC input (MSP RX), Config/EEPROM, User API (Developer Mode). Pipeline docs: `subsystems/Failsafe_Subsystem.md`, `subsystems/RC_Input_Pipeline.md`, `subsystems/User_Space_API.md`
Key code:
- `src/main/flight/failsafe.cpp:121` failsafeReset: `rxDataFailurePeriod = 200 + failsafe_delay*100`
- `failsafe.cpp:211` failsafeOnValidDataFailed: the diff changes line 213
- `failsafe.cpp:240-253` IDLE+armed+!receiving: `Signal_loss`, `LAND`
- `src/main/rx/rx.cpp:369` `DELAY_5_HZ` (200 ms) MSP signal timeout; `rx.cpp:87` `MAX_INVALID_PULS_TIME` 600
- `rx.cpp:503-566` detectAndApplySignalLossBehaviour: hold, then rxfail, then `rcDataPilot` snapshot
- `src/main/config/config.cpp:645,779` activateConfig/readEEPROM; `:550` the default
- `src/main/io/serial_msp.cpp:1738` MSP_SET_FAILSAFE_CONFIG; `:1630` MSP_EEPROM_WRITE
- `src/main/mw.cpp:1119` userCode Developer Mode gate

Mode: fix

Current behaviour (with the diff): the link is declared down about 3.2 s after the last `MSP_SET_RAW_RC`, not 3.0 s. Before the diff it was about 1.0 s.

Analysis:
1. **The diff is correct.** `failsafeReset()` runs only through `useFailsafeConfig()`, which only `activateConfig()` calls (`config.cpp:667`). `activateConfig()` only runs from `readEEPROM()`: at boot (`main.cpp:178`), MSP_EEPROM_WRITE (refused while armed), MSP_SELECT_SETTING and MSP_RESET_CONF (both refused while armed), and the CLI profile/save. Masterconfig is loaded before boot calls it, so the period is never left at 0.
   - If `failsafe_delay` is 0, the period is 200, which is today's behaviour. So the change can never make failsafe trigger sooner.
   - It can be stale: MSP_SET_FAILSAFE_CONFIG (`serial_msp.cpp:1738`, not blocked while armed) writes only `masterConfig`. The new value takes effect at the next readEEPROM, while MSP_FAILSAFE_CONFIG (`:1259`) already reports it. This is minor.
   - Types: `millis()`, `validRx*At` and `rxDataFailurePeriod` are all `uint32_t` (`failsafe.h:64-66`). Line 213 adds no warning and is wrap-safe. Line 122 (int to uint32) was there before.
2. **The timing chain is not what TEMP_MSP_TIMING.md says.** `rxMspReadRawRC` (`rx/msp.c:37`) keeps returning the last frame while `rxSignalReceived` is true. So the channels count as valid, and `rcInvalidPulsPeriod` keeps being refreshed, until 200 ms.
   - 0-200 ms: the last frame is replayed.
   - 200 ms: the signal flag drops. Developer Mode stops (`onLoopFinish`, `RC_ARRAY` and `userRCflag` cleared, `mw.cpp:1155-1167`). The hold starts.
   - About 800 ms (200 + 600): rxfail values apply and `failsafeOnValidDataFailed` starts.
   - About 3200 ms (800 + 200 + 2200, ±20 ms at the 50 Hz floor): link down; Signal_loss and LAND in the same `processRx`.
   - TEMP_MSP_TIMING.md lines 22-28 and 103-106, and FINDINGS §4/§7 (600/800 ms), leave out the 200 ms replay. The battery-capacity-estimate reviewer timeline (`CHANGES.md:73-77`) already has the same ~800 ms.
   - Budgets by `failsafe_delay`: 0 → ~1.0 s, 10 → ~2.0 s, 15 → ~2.5 s, 22 → ~3.2 s. **22 goes past FINDINGS §9's own ~2.5 s cap. 15 covers the 2.17 s stall.**
3. **Recovery.**
   - Before the link is declared down (e.g. frames return at 1.5 s): nothing latches in failsafe; `rxLinkState` never left UP. The sticks step from rxfail back to live values. Developer Mode reruns `onLoopStart` (it latched at 200 ms). User RC overrides are already gone.
   - After it is declared down: UP needs 200 ms of good frames (`PERIOD_RXDATA_RECOVERY`, `:206`). LAND stays latched: MSP_SET_COMMAND is ignored while `isLanding` (`serial_msp.cpp:1569`), and `Signal_loss` is cleared only once disarmed (`:257`, `mw.cpp:568`). This was already so before the diff.
4. **Nothing else uses `failsafe_delay`** (only `:122`, CLI `serial_cli.cpp:456`, and MSP `:1261`/`:1739`). `failsafe_off_delay` and `failsafe_throttle` are dead: `failsafeActivate()` (`:174`) is never called, so FAILSAFE_LANDING and `failsafeApplyControlInput` cannot be reached.
5. **During the window, from about 800 ms:**
   - Roll, pitch and yaw go to midrc; throttle goes to 1200 (`rx.cpp:439-449`). I verified both. AUX channels are HOLD, so ARM stays on.
   - `rcDataPilot` also gets these values (`rx.cpp:563`).
   - In ALT_HOLD, throttle 1200 gives a deflection of -300, which is ≈16 cm/s descent ((300-40)·30/460, `altitudehold.cpp:545`). That is about 38 cm over 2.4 s (before: ~3 cm).
   - In ANGLE (throttle mode), 1200 is far below hover (`landThrottle`/`failsafe_throttle` 1300 are "slightly below hover"). The drone sinks for 2.4 s, may hit the ground, and the crash detector (ANGLE only) may disarm it.
   - The crash and battery LAND paths do not change.
6. **Real loss (controller off):** LAND starts at about 3.2 s instead of about 1.0 s. The time spent flying the last stick (0-800 ms) is unchanged, so a commanded drift is no worse. The extra 2.2 s is level with throttle 1200.
7. **Causal chain:** a TCP stall of 1.2-2.2 s (unproven head-of-line hypothesis) → no frames → link down at about 1.0 s, because `PERIOD_RXDATA_FAILURE` was used and `failsafe_delay` was ignored (proven, old `:213`) → LAND. After the diff the decision waits for the configured delay. **Not proven:** the cause of the stalls; the drone-side gap length (all figures are controller-side); and that 22 rather than 15 is backed by data.

Constraints / invariants that apply:
- Landing must not go through stick limits (FLIGHT_INVARIANTS). LAND is unchanged.
- User override expiry is unaffected because Developer Mode stops at 200 ms.
- Build gate: `driver.sh --gate PRIMUS_X2_v1`.

Resource conflicts: none.

Already answered by the code:
- **Persistence:** `resetConf` runs only when the EEPROM is invalid (version 107 mismatch, size or checksum, `config.cpp:618-636`) or on MSP_RESET_CONF/CLI defaults. A drone with saved settings keeps its stored `failsafe_delay` (likely 10), so it gets about 2.0 s, not 3.2 s.
- Raising `EEPROM_CONF_VERSION` would wipe every setting: acc trims/calibration, PIDs, rates, AUX modes, features, battery capacity and rxfail.

Questions for the user:
1. Is 22 (3.2 s) intended over 15 (2.5 s)? FINDINGS §9 caps the budget at ~2.5 s.
2. Is ANGLE/throttle mode (no alt hold) flown over Wi-Fi? 2.4 s at throttle 1200 is a real sink.
3. Does the app send MSP_SET_FAILSAFE_CONFIG / MSP_EEPROM_WRITE, and with what delay?
4. Does PlutoIDE flashing mass-erase the config page?
5. Should fielded drones get the new value: version bump (full reset), a one-time migration, or app-written?
6. Is a drone-side gap log available (to fit 15 vs 22)?

Doc drift:
- `Failsafe_Subsystem.md:13` (`failsafe_delay` "e.g. 10", never applied), `:14` and `:17` (`failsafeApplyControlInput`, dead), `:22` (IDLE→LOSS_DETECTED; actually IDLE→LAND directly), `:23` (`failsafe_off_delay`, dead), `:24`.
- `RC_Input_Pipeline.md:18` names `rxUpdateCheck()`, which does not exist.
- `User_Space_API.md:24`: the API actually stops at 200 ms.
- `.claude/skills/pluto-flighttest/SKILL.md:63` says "failsafe_delay 1 s".
- `config.cpp:550` and `failsafe.h:39` comments still say "1sec".

Prior work: battery-capacity-estimate (`CHANGES.md:73-78`, `TASKS.md:316`) noted that `rxDataFailurePeriod` was never read. Commits 2059c63 and 49a2fdb touched failsafe. No topic is open for this.

Suggested skills: pluto-rules, pluto-build, pluto-flighttest, pluto-commit; agent pluto-log-analyst.

Pipeline docs to update at the end:
- Failsafe_Subsystem.md: the real timeline (200/800/800+period), the dead stages and settings, LAND latch.
- RC_Input_Pipeline.md: hold/replay timing.
- User_Space_API.md: the 200 ms stop.
- pluto-flighttest SKILL: delay value.
- CHANGELOG.
- Move the two root .md files into `active-development/rx-signal-loss-tolerance/` as INVESTIGATION.md and NOTES.md, correcting 3000 to 3200 ms.
