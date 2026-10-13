# Crash Detect False Trigger

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

| | |
|---|---|
| **Status** | **Closed - pending commit** ( 13 Oct 2026, FW 3.11.1, API 1.4.0 ) |
| **Branch** | `BugFix-June26` |
| **Target** | `PRIMUS_X2_v1` |
| **Last updated** | 13 Oct 2026 |
| **Pipeline docs** | Promoted at commit: new "Crash Detector" section in [Failsafe_Subsystem.md](../../fw-architecture-pipeline/subsystems/Failsafe_Subsystem.md); crash-is-a-halt claim corrected in [Error_Failure_Mode.md](../../fw-architecture-pipeline/subsystems/Error_Failure_Mode.md) |

**Problem.** In most flights the app shows "crashed" and the drone disarms after a light bump or
collision, and on hard or bumpy landings. `failsafeOnCrash ( )` disarms on one 3.5 ms sample of
lateral accel above 0.305 g ( filtered ) or 0.427 g ( raw ). Commit `49a2fdb` made those limits about
10× tighter. Details: [INVESTIGATION.md](INVESTIGATION.md).

**Change so far.** The limits went 1250 / 1750 → 1438 / 2013 ( task 1 ) → 1726 / 2416 ( task 12 ) →
**2500 / 3000** ( the user's edit, 0.610 / 0.732 g ). The temporary diagnostics from task 2 print the peaks
and the term that fired. The disarm is restored. **Final limits: 5000 / 8000** ( 1.22 / 1.95 g, as flown in test 2; 4000 / 7000 was tried and reverted ), 70° unchanged; findings in [INVESTIGATION.md](INVESTIGATION.md).

**First log ( [TESTING.md](TESTING.md) test 1 ).** Four short, low-tilt events with raw peaks of 0.88-1.32 g;
a contact spike, not tilt and not calibration bias. Then a crash with a first raw peak of 5.9 g, passing 70°
within 250 ms. Quiet flight stays under 0.47 g raw.

## Open items

- Test 2 ( log-2..4, TESTING.md ): at 5000 / 8000 no bump or landing disarmed and all 3 flips disarmed ( 70° rule ).
- Test 3 ( task 8 ): reported passed by the user; the per-flight notes ( log-5.txt ) were not filled in.
- Temporary diagnostics: **removed** in task 6 ( 13 Oct ); `PlutoPilot.cpp` is back to HEAD.
- Landing: 8 of 8 gentle landings ( `LAND` and manual ) ended cleanly without the crash path; firm landings were covered by test 3 ( user's report ).
- Accepted gaps ( pinned upright, Z impacts, caught ): Planned topic [crash-stuck-detection](../crash-stuck-detection/README.md).
- Side finding, out of scope: in the API, `FlightStatus_Check ( FS_CRASHED )` maps to the in-flight
  low-battery bit, and `FS_LOW_BATTERY` maps to Crash ( `FC-Control.h:30` vs `runtime_config.h:92` ).

## Next action

None: closed, pending the user's commit ( FW 3.11.1 ).
