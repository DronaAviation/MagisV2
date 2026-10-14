# RX Signal-Loss Tolerance

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md) · [CONTROLLER_FINDINGS](CONTROLLER_FINDINGS.md) · [MSP_TIMING](MSP_TIMING.md)

| | |
|---|---|
| **Status** | **Closed** in `84c326d` ( 14 Oct 2026, FW 3.11.2, API 1.4.0, `EEPROM_CONF_VERSION` 108 ) |
| **Branch** | `BugFix-June26` |
| **Target** | `PRIMUS_X2_v1` |
| **Last updated** | 14 Oct 2026 |
| **Pipeline docs** | Promoted at commit: [Failsafe_Subsystem.md](../../fw-architecture-pipeline/subsystems/Failsafe_Subsystem.md) ( data structures, timeline, RC link loss flowchart ), [RC_Input_Pipeline.md](../../fw-architecture-pipeline/subsystems/RC_Input_Pipeline.md), [User_Space_API.md](../../fw-architecture-pipeline/subsystems/User_Space_API.md) |

**Problem.** On the Wi-Fi RC link, 1.2-2.2 s transport stalls on an otherwise healthy link trigger `Signal_loss`
and an automatic landing in flight. `failsafeOnValidDataFailed ( )` ignored the configured `failsafe_delay`, so
`LAND` came ~1.0 s after the last RC frame. Details: [INVESTIGATION.md](INVESTIGATION.md).

**Planned change.**
- Apply `failsafe_delay`. This is the user's fix at `failsafe.cpp:214`.
- Default `failsafe_delay` **15**, about 2.5 s from the last frame to `LAND`.
- Bump `EEPROM_CONF_VERSION` 107 → 108 so every drone gets the new value. **This resets all saved settings:
  every drone must have its accelerometer and compass calibrated after flashing; on Wi-Fi / PPM it will not arm until the compass is ( ELRS skips that check ).**

## Open items

- Task 10 done: the effective `failsafe_delay` is capped at 22 ( ~3.2 s ); MSP and CLI still read back the stored value.
- `FW_Version` already bumped to 3.11.2 by the user.
- The stalls themselves ( TCP head-of-line, unproven ) are out of scope; the real fix is on the ESP or controller side.
- Bench and flights ( tasks 4-5 ): passed on the user's report, with no measured times recorded.
- Input notes moved into the topic ( task 7 ): [CONTROLLER_FINDINGS.md](CONTROLLER_FINDINGS.md) and [MSP_TIMING.md](MSP_TIMING.md), with the timing corrected.

## Next action

None: closed in `84c326d` ( FW 3.11.2 ).
