# Flight status enum map - Investigation

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

## Problem

API test T05 ( B3 ) from the testing team: after a crash disarm, `flightstatus_e FlightStatus_Get ( void )`
( `API/FC-Control.h` ) returned 2 ( `FS_LOW_BATTERY` ), not 4 ( `FS_CRASHED` ). 7 ( `FS_OK_TO_ARM` ) and 8
( `FS_ARMED` ) were correct. Battery at the test: partly used.

The user asked whether the status sent to the app over MSP is the same thing the function reads.

## Answer: same flags, different numbering

Both paths read `flightIndicatorFlag` and take its lowest set bit ( `leastSignificantBit`,
[maths.cpp:276-285](../../../../src/main/common/maths.cpp#L276-L285) ), so they share one priority order. They differ
in how that bit index becomes a status:

- **MSP** ( `MSP_FLIGHT_STATUS`, [serial_msp.cpp:867](../../../../src/main/io/serial_msp.cpp#L867) ) switches on the
  internal names ( `case Crash:` → `1 << App_Crash` ), so the app is told the right status.
- **The API** ( [FC-Control-Status.cpp](../../../../src/main/API-Src/FC-Control-Status.cpp), lines 29-35 before the fix ) casts the
  bit index straight into the public enum, which is numbered differently:

| Bit / value | Internal `FlightStatus_e` ( [runtime_config.h:85-95](../../../../src/main/config/runtime_config.h#L85-L95) ) | Public `flightstatus_e` ( [FC-Control.h:27-35](../../../../src/main/API/FC-Control.h#L27-L35) ) |
|---|---|---|
| 0 | `Accel_Gyro_Calibration` | `FS_ACCEL_GYRO_CALIBRATION` |
| 1 | `Mag_Calibration` | `FS_MAG_CALIBARATION` |
| **2** | **`Crash`** | **`FS_LOW_BATTERY`** |
| **3** | **`Low_battery`** ( warning ) | **`FS_INFLIGHT_LOW_BATTERY`** |
| **4** | **`LowBattery_inFlight`** ( critical, latched ) | **`FS_CRASHED`** |
| 5 | `Signal_loss` | `FS_SIGNAL_LOSS` |
| 6 | `Not_ok_to_arm` | `FS_NOT_OK_TO_ARM` |
| 7 | `Ok_to_arm` | `FS_OK_TO_ARM` |
| 8 | `Armed` | `FS_ARMED` |

So the 2 the testers saw was the crash flag reported under the wrong name. Battery was not involved: a real low or
critical battery is bit 3 or 4, and both rank below Crash.

`FlightStatus_Check ( status )` casts the other way and has the same swap: `FlightStatus_Check ( FS_CRASHED )` tests
the critical-battery bit, `FS_LOW_BATTERY` tests Crash, `FS_INFLIGHT_LOW_BATTERY` tests the battery warning.

## Causal chain

1. `failsafeOnCrash ( )` ( `flight/failsafe.cpp:363-384`, ARMED + ANGLE, angle > 70 deg or an accel spike ) calls
   `set_FSI ( Crash )` ( bit 2 ) and `mwDisarm ( )`.
2. While disarmed, bits 2 ( Crash ) and 7 ( Ok_to_arm ) are set; the lowest is 2.
3. `FlightStatus_Get ( )` returns `( flightstatus_e ) 2` = `FS_LOW_BATTERY`.
4. Crash is cleared about 300 ms after the disarm once the craft is level
   ( [mw.cpp:489-501](../../../../src/main/mw.cpp#L489-L501) ); the getter then returns 7.

## Hypotheses

| # | Hypothesis | Verdict |
|---|---|---|
| 1 | Enum mismatch between internal and public numbering | **Proven from the code** ( table above ) |
| 2 | A real low-battery reading at impact | Killed: low battery is bit 3 / 4, never 2, and ranks below Crash |
| 3 | Crash cleared before the read | Not this bug; matters for the retest ( about 0.3 s window when level ) |

History: the mismatch was logged as an out-of-scope side finding in `crash-detect-false-trigger` ( closed `386599b` )
and never fixed.

## Q&A that shaped the plan ( 5 Nov 2026 )

- Mode: check first, then **go straight to a fix topic** ( the cause is proven from the code ).
- Fix: **map internal to public** ( option A ) in `FC-Control-Status.cpp`, for both `Get` and `Check`. Public numbers,
  internal enum, priority order and MSP output unchanged.
- Crash timing: **left as is** ( about 0.3 s once level, same as the app sees ); documented, poll every loop.
- Critical battery: **reported as is** to user code ( `FS_INFLIGHT_LOW_BATTERY` = critical, latched ); MSP's softening
  while armed is for the app only.
- Done when: **bench log on the fixed build** ( props off, ANGLE, armed, tilted past 70 deg, kept tipped ).
- Scope: status reporting only; crash detection thresholds stay with `crash-stuck-detection`.
