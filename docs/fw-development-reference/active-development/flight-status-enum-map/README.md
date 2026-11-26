# Flight status enum map

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

| | |
|---|---|
| **Status** | **Closed** in `6d1b5cd` ( 14 Nov 2026, FW 3.15.6, API 1.8.4 ) |
| **Mode** | `fix` |
| **Branch** | `BugFix-June26` |
| **Target** | `PRIMUS_X2_v1` ( all targets built at commit ) |
| **Last updated** | 14 Nov 2026 |
| **Pipeline docs** | [User_Space_API.md](../../fw-architecture-pipeline/subsystems/User_Space_API.md) ( via `PIPELINE_UPDATE.md` ); [FC_CONTROL_API_WIKI.md](../../../API/FC_CONTROL_API_WIKI.md) |

**Problem.** API test T05 ( B3 ): after a crash disarm `FlightStatus_Get ( )` returned 2 ( `FS_LOW_BATTERY` ), not 4
( `FS_CRASHED` ); 7 and 8 were correct. The MSP status and the API read the same `flightIndicatorFlag` with the same
priority, but [FC-Control-Status.cpp](../../../../src/main/API-Src/FC-Control-Status.cpp) casts the internal bit index
straight into the public enum, and the two are numbered differently at 2-4 ( internal Crash 2, Low_battery 3,
LowBattery_inFlight 4; public LOW_BATTERY 2, INFLIGHT_LOW_BATTERY 3, CRASHED 4 ). MSP maps by name, so the app is
right. `FlightStatus_Check ( )` has the same swap. Details: [INVESTIGATION.md](INVESTIGATION.md).

**Fix.** Map internal to public by name in `FC-Control-Status.cpp` for both functions. Public numbers, internal enum,
priority order, MSP output and crash-clear timing unchanged. Header comments and a new wiki section. API 1.8.4.

## Open items

- Task 1 done ( 5 Nov 2026 ): name-to-name map in `FC-Control-Status.cpp` for `Get` and `Check`; header comments corrected; gate green, size identical to HEAD; review fixes applied ( comments only ). Details: [CHANGES.md](CHANGES.md).
- Task 2 done ( 5 Nov 2026 ): gate green, 122752 / 1316 / 19728 B, identical to HEAD.
- Task 3 done ( 14 Nov 2026 ): bench log-1 PASS, 841 / 841 records map by name, both crash disarms read 4 ( `FS_CRASHED` ); temporary logging removed, `PlutoPilot.cpp` back to HEAD, clean hex rebuilt. Details: [TESTING.md](TESTING.md).
- Task 4 done ( 14 Nov 2026 ): "Flight status" section in [FC_CONTROL_API_WIKI.md](../../../API/FC_CONTROL_API_WIKI.md), [PIPELINE_UPDATE.md](PIPELINE_UPDATE.md) staged.
- Task 5 done ( 14 Nov 2026 ): topic review, 0 blocking; 3 should-fix ( wiki status descriptions, the 0xFFFF read after a disarm ) and 4 nits fixed; gate green.
- Task 6 done ( 14 Nov 2026 ): graph refreshed; all three targets built clean ( no new warnings ); FW 3.15.6 / API 1.8.4; `User_Space_API.md` and CHANGELOG updated.
