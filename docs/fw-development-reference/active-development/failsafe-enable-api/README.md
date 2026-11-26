# Failsafe_enable API

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

| | |
|---|---|
| **Status** | **Closed** in `c344387` ( 15 Nov 2026, FW 3.15.10, API 1.8.8 ) |
| **Mode** | `fix` |
| **Branch** | `BugFix-June26` |
| **Target** | `PRIMUS_X2_v1` ( all targets built at commit ) |
| **Last updated** | 15 Nov 2026 |
| **Pipeline docs** | [Failsafe_Subsystem.md](../../fw-architecture-pipeline/subsystems/Failsafe_Subsystem.md), `Power_BMS_Pipeline.md`, `User_Space_API.md` ( via `PIPELINE_UPDATE.md` ); new `docs/API/FAILSAFE_API_WIKI.md`, note in `BMS_API_WIKI.md` |

**Problem.** API test T07 ( B2 ): enabling one failsafe switches the other two off.
[`Failsafe_enable ( failsafe_e )`](../../../../src/main/API-Src/FC-Config-Setpoints.cpp#L213) assigns every flag
`fsX = enableAll || ( FAILSAFE == X )`, so each call writes `false` to the two flags it was not asked about. The header
at [FC-Config.h:270-279](../../../../src/main/API/FC-Config.h#L270) says the others "remain unchanged", and
`Failsafe_disable` does exactly that, so only `enable` is wrong.

This is a flight-safety defect, not only an API one: `Failsafe_enable ( CRASH )` clears `fsInFlightLowBattery`, the gate
on the `BATTERY_OK` -> `BATTERY_WARNING` step at [battery.cpp:798](../../../../src/main/sensors/battery.cpp#L798). The
battery level then never leaves OK, so there is no low-battery warning, no `batteryCriticalConfirmed ( )`, no auto-land
at critical ( `mw.cpp:1306` ) and no refusal to arm on a flat pack ( `mw.cpp:625` ). The reverse also holds:
`Failsafe_enable ( LOW_BATTERY )` clears `fsCrash`, so the crash detector at
[failsafe.cpp:365](../../../../src/main/flight/failsafe.cpp#L365) never disarms. Details:
[INVESTIGATION.md](INVESTIGATION.md).

**Fix.** Set each flag to `true` only when it is named or `ALL` is passed, and never clear a flag in `Failsafe_enable`.
No signature change; `Failsafe_disable` is left as it is. `LOW_BATTERY` is documented as inert ( its reader was deleted
in `88f594f` ) rather than given a new meaning. API 1.8.8, FW 3.15.10.

**Verification** is a host truth-table check plus a code-flow trace, not a flight: the real
`FC-Config-Setpoints.cpp` is compiled for the PC against the existing
[tools/sim/stub/](../../../../tools/sim/stub) tree and driven through every enable / disable sequence. The test team
re-runs T07 on hardware afterwards.

## Open items

- Task 1 done ( 15 Nov 2026 ): `Failsafe_enable ( )` now only ever sets a flag, so a failsafe that is not named keeps
  its state. Gate green on `PRIMUS_X2_v1`, no new `src/` warnings, flash 121.5 KB / 256 KB, RAM 20.5 KB / 40 KB.
  Reviewer: no findings. Details: [CHANGES.md](CHANGES.md).
- Task 2 done ( 15 Nov 2026 ): host check on the real source file, PASS 4164 / 4164 with the fix; the pre-fix file
  fails 1468 / 4164 and reproduces T07. The trace shows `fsInFlightLowBattery` is the only way into the whole
  low-battery chain. Details: [TESTING.md](TESTING.md).
- Task 3 done ( 15 Nov 2026 ): clean gate green. The ELF is the same size as `HEAD` to the byte, because nothing
  calls `Failsafe_enable` and `-gc-sections` drops it; in the object file the function is 4 B smaller and every
  store now writes 1. Details: [TESTING.md](TESTING.md).
- Task 4 done ( 15 Nov 2026 ): new [FAILSAFE_API_WIKI.md](../../../API/FAILSAFE_API_WIKI.md) ( the planned
  `FC_CONFIG_API_WIKI.md` is the Stick Rate page ), a note and link in `BMS_API_WIKI.md`, and
  [PIPELINE_UPDATE.md](PIPELINE_UPDATE.md) staging `Failsafe_Subsystem.md`, `Power_BMS_Pipeline.md` and
  `User_Space_API.md`.
- Task 7 added and done ( 15 Nov 2026 ): the `failsafe_e` value comments in `FC-Config.h` disagreed with the firmware
  and are rewritten to match it and the wiki. Gate green, ELF unchanged, reviewer: nothing blocking ( two nits
  applied ).
- Task 5 done ( 15 Nov 2026 ): topic review over all 13 files found nothing blocking; one should-fix ( stale line
  citations after the header grew ) and four nits, all applied. Gate re-run green, ELF unchanged.
- Task 6 done ( 15 Nov 2026 ): graph refreshed ( `tools/graph_labels.py` needed the backlog #13 workaround ), all three
  targets built clean ( no warnings in the touched files ), FW 3.15.9 -> 3.15.10 and API 1.8.7 -> 1.8.8,
  `PIPELINE_UPDATE.md` applied to `Failsafe_Subsystem.md`, `Power_BMS_Pipeline.md` and `User_Space_API.md`, CHANGELOG
  and CLAUDE.md updated, staged. **Closed** in `c344387`.
- Not hardware-tested by this topic: the testing team re-runs T07. It must call `Failsafe_enable ( )` **before** the pack
  runs low, or even the pre-fix firmware passes ( the battery gate only guards the step out of `BATTERY_OK` ).
- Out of scope, recorded: `LOW_BATTERY` has had no reader since `88f594f`; giving it a function again would be a
  battery-safety design topic of its own.

## Next action

None: committed in `c344387`.
