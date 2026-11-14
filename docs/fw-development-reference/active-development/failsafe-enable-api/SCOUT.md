# Scout brief - failsafe-enable-api

Reconnaissance from the `pluto-scout` agent, 15 Nov 2026, verbatim. Do not re-run this reconnaissance.

---

## Scout brief: T07 ( B2 ): Failsafe_enable ( X ) switches the other two failsafes off

Graph: stale (built 73022c32, HEAD 3be9093). `graphify explain Failsafe_enable` returns only 2 edges, so grep was used.

Subsystems: user API (FC-Config), battery state machine, crash detector -> pipeline docs: `docs/fw-development-reference/fw-architecture-pipeline/subsystems/Failsafe_Subsystem.md`
Key code:
- `src/main/API-Src/FC-Config-Setpoints.cpp:213-225` Failsafe_enable: the bug
- `src/main/API-Src/FC-Config-Setpoints.cpp:227-243` Failsafe_disable: correct. `||` binds tighter than `?:`, so it parses as `(a||b) ? false : x`
- `src/main/API-Src/API-Utils.cpp:122-124` the flags, all `= true`. Extern at `API/API-Utils.h:111-113`
- `src/main/API/FC-Config.h:44-49` `failsafe_e` {LOW_BATTERY 0, INFLIGHT_LOW_BATTERY 1, CRASH 2, ALL 3}. Doc at :267-276 says "others remain unchanged"
- `src/main/sensors/battery.cpp:798` updateBatteryState: the OK->WARNING step is gated on `fsInFlightLowBattery`
- `src/main/flight/failsafe.cpp:365` failsafeOnCrash: gated on `fsCrash`
- `src/main/mw.cpp:625`, `:1306` arm refusal and auto-land, both through `batteryCriticalConfirmed ( )`

Mode: fix
Current behaviour: each flag is assigned `enableAll || ( FAILSAFE == X )`. Calling `Failsafe_enable ( CRASH )` therefore writes false to fsLowBattery and fsInFlightLowBattery. Failsafe_disable keeps the flags it is not asked to change.

Analysis (causal chain):
1. The user calls `Failsafe_enable ( CRASH )` (or any value other than ALL). At :219/:222 the flags not named are set to false. **Proven by reading the code.**
2. fsInFlightLowBattery is now false, so battery.cpp:798 never leaves BATTERY_OK. That means no `Low_battery` / `LowBattery_inFlight` FSI, no beeper, `BatteryWarningMode` stays 0, `batteryCriticalConfirmed ( )` is never true, there is **no auto-land** (mw.cpp:1306) and **no arm refusal** (mw.cpp:625). This is a safety defect, not only an API one: turning crash detection "on" silently disables the low-battery landing.
3. `Failsafe_enable ( LOW_BATTERY )` sets fsCrash false, so failsafe.cpp:365 never disarms on a crash.
- Gate behaviour (relevant for the test): the gate applies only to the OK->WARNING step. Clearing the flag after WARNING/CRITICAL has been reached does not drop the level back.

Minimal fix (FC-Config-Setpoints.cpp only):
```
if ( enableAll || FAILSAFE == LOW_BATTERY )          fsLowBattery = true;
if ( enableAll || FAILSAFE == INFLIGHT_LOW_BATTERY ) fsInFlightLowBattery = true;
if ( enableAll || FAILSAFE == CRASH )                fsCrash = true;
```
`fsX = fsX || ...` would work the same way. For symmetry, optionally rewrite disable as if-clears too. No signature change.

Secondary defect: **fsLowBattery has no reader.** Its only consumer, `annexCode`'s `else if ( fsLowBattery ) set_FSI ( Low_battery )` (a vbat <= 3.4 V ground warning), was deleted in 88f594f, the battery-management rewrite. That commit moved the gate onto fsInFlightLowBattery. So `Failsafe_disable ( LOW_BATTERY )` does nothing today. Gating it is a design choice, not part of this fix (see Q2).

Constraints / invariants that apply:
- CLAUDE.md: at critical the firmware lands, and mwArm refuses until the pack is changed. The fix restores that path when another failsafe is enabled.
- The doc in API/FC-Config.h:267-276 is the specification. Behaviour that differs from documented API behaviour gets a bump to `API_Version` (1.8.7) / `FW_Version` (3.15.9), the Makefile:31-32 convention used by the sibling fix commits.
- Strict warnings: plain bool assignments, no conversions.

Resource conflicts: none
Already answered by the code:
- The flags are reset only by reboot (`.data` init). Nothing else writes them, on disarm or Dev Mode off.
- `ALL` is handled the same way in both functions.
- MSP and app status follow the FSI flags, not these booleans (serial_msp.cpp:892).

Questions for the user:
1. Should LOW_BATTERY get a function again (for example gating the WARNING level / beeper only, while INFLIGHT_LOW_BATTERY gates critical and the auto-land), or be documented as "no effect, kept for compatibility"?
2. Bump API_Version (to 1.8.8?) and FW_Version for this fix, as the sibling topics did?
3. Topic name: reuse the empty `active-development/fc-config-api-fixes/` (it holds only an empty `logs/`; set-desired-angle-roll-pitch/TASKS.md:217 left it alone deliberately) or create `failsafe-enable-api`?
4. Is there a written T07 test procedure to attach? None is in the repo.
5. Should Failsafe_disable be rewritten for symmetry, or left alone?

Doc drift:
- `docs/API/FC_CONFIG_API_WIKI.md` has **no Failsafe_enable/disable section** at all.
- BMS_API_WIKI.md:55-56 and FC_CONTROL_API_WIKI.md:175 correctly say the battery levels need the in-flight failsafe. Neither says LOW_BATTERY is inert.
- Failsafe_Subsystem.md:85 ("switch it with Failsafe_enable / disable ( CRASH )") is true, but leaves out that enable cleared the other two. It also cites failsafe.cpp:362; the function is at :363 and the check at :365.

Prior work:
- No T07/B2 report anywhere in docs/.
- Sibling fix topics to copy the layout from (README, SCOUT, INVESTIGATION, TASKS, CHANGES, TESTING, PIPELINE_UPDATE, logs/): `active-development/rc-command-get-api/`, `flight-status-enum-map/`, `set-default-pid-profile/`.
- Relevant commits: 88f594f (removed the fsLowBattery consumer), cbd1c72 (auto-land at critical), 386599b (crash limits).

Suggested skills: pluto-rules, pluto-build (`--gate PRIMUS_X2_v1`), pluto-flighttest (bench test: enable CRASH, then confirm the battery warning still fires, and the reverse), pluto-commit; spec-to-code-compliance (header doc vs implementation).

Pipeline docs to update at the end:
- `Failsafe_Subsystem.md`: add a note on the enable/disable semantics and the line-number fix.
- `docs/API/FC_CONFIG_API_WIKI.md`: add a Failsafe_enable/disable section stating "others unchanged" and the LOW_BATTERY status.
- BMS_API_WIKI.md only if LOW_BATTERY is given a meaning.
- CHANGELOG.md.

---

## Answers given after the brief

1. LOW_BATTERY keeps no function: documented as inert, kept for compatibility. Wiring it is a separate topic.
2. Yes, API_Version 1.8.7 -> 1.8.8 and FW_Version 3.15.9 -> 3.15.10 at commit.
3. New folder `failsafe-enable-api`; the empty `fc-config-api-fixes/` is left alone.
4. No written T07 procedure exists. The test team re-runs T07 on their own procedure.
5. `Failsafe_disable` is left alone: smallest diff.
