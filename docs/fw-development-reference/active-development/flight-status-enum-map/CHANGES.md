# Flight status enum map - Changes

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

## Task 1: map internal flight status to the public enum ( 5 Nov 2026 )

| File | Change | Why |
|---|---|---|
| [FC-Control-Status.cpp](../../../../src/main/API-Src/FC-Control-Status.cpp) | New `static const flightStatusMap [ ]`: nine `{ FlightStatus_e, flightstatus_e }` pairs of the same meaning, written by name | The two enums are numbered differently at 2-4; a pair table cannot drift if a value moves |
| same, `FlightStatus_Get ( )` | Looks up the lowest set bit of `flightIndicatorFlag` in the table and returns its public value; no match ( no flag set, `0xFFFF` ) returns the raw value as before | A crash disarm read 2 ( `FS_LOW_BATTERY` ) instead of 4 ( `FS_CRASHED` ) ( D1, D4 ) |
| same, `FlightStatus_Check ( )` | Looks up the public value and tests its internal flag with `status_FSI ( )`; a value not in the table returns false | `Check ( FS_CRASHED )` tested the critical-battery flag; a value outside the enum could shift past bit 15 ( D1, D4 ) |
| same, includes and banner | `<stddef.h>`, `common/utils.h` ( `ARRAYLEN` ); Last Modified and a HISTORY line | |
| [FC-Control.h](../../../../src/main/API/FC-Control.h) | Comments of `FlightStatus_Get` / `FlightStatus_Check` rewritten: the priority order, when crashed is set and cleared ( about 0.3 s once level ), warning vs critical battery ( a confirmed critical lands and is reported until the battery is changed ), the app difference while armed; `Check` tests one status whatever its priority. Banner updated | The old comments described the cast, not the values |

Unchanged ( D1-D3 ): the internal `FlightStatus_e` and its bit order ( the priority, and the raw `MSP_STATUS` bits ), the
public numbers, `MSP_FLIGHT_STATUS`, the crash-clear timing, critical battery reported as is.

**Build** ( `PRIMUS_X2_v1`, gate ): no new warning under `src/`; 122752 / 1316 / 19728 B, identical to HEAD ( no
firmware code calls these API functions, so `-gc-sections` drops them ).

**Review** ( `pluto-reviewer` ): 0 blocking. All nine table pairs and the `Get` / `Check` logic confirmed; every status
is set somewhere in the firmware. Fixed in the `FC-Control.h` comment: critical battery is reported again after the
landing only once confirmed ( a provisional critical, before the pack resistance is measured, does not land and can
clear when disarmed ) ( should-fix ); crash detection runs only while armed and not during a flip; both battery statuses
need the in-flight low-battery failsafe on ( the default ); the `0xFFFF` no-status value is documented. Gate re-run:
green, size unchanged.

## Task 5: topic review fixes ( 14 Nov 2026 )

| File | Change | Why |
|---|---|---|
| [FC_CONTROL_API_WIKI.md](../../../API/FC_CONTROL_API_WIKI.md) | Priority table rows for `FS_SIGNAL_LOSS`, `FS_NOT_OK_TO_ARM`, `FS_OK_TO_ARM` corrected; new "OK to arm is not a full check" note; signal-loss note; 0xFFFF read right after a disarm | Review: `failsafe.cpp:259` sets `Ok_to_arm` with the link up whatever the attitude ( log-1 `f:132` while on its side ); `Signal_loss` is set again after the disarm until the link is back ( `failsafe.cpp:257-262` ); log-1 `f:4` shows a read with no Armed / OK bit after a disarm |
| [FC-Control.h](../../../../src/main/API/FC-Control.h) | `@return` of `FlightStatus_Get`: when 0xFFFF occurs ( not armed ) | Same ( D4 correction ) |
| [FC-Control-Status.cpp](../../../../src/main/API-Src/FC-Control-Status.cpp) | Comment on the 0xFFFF fallback: when it occurs, and that the out-of-range enum cast is kept by GCC ( no `-fstrict-enums` ) | Review nit |

Gate re-run: no new warning, 122752 / 1316 / 19728 B.
