# Flight status enum map - Scout brief

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

Brief from the `pluto-scout` agent, 5 Nov 2026, verbatim ( mode `check` at the time; the user then chose `fix` ).

## Scout brief: FlightStatus_Get ( ) returns 2 (FS_LOW_BATTERY) after a crash disarm instead of 4 (FS_CRASHED)
Graph: stale (built 1b43fbea, HEAD e72d098). Not needed: grep located every use of the status flags.

### Summary
**Proven root cause: the two status enums are numbered differently.** `FlightStatus_Get ( )` returns the internal bit index unchanged, and the internal enum puts Crash at bit 2.

| Value | Public `flightstatus_e` (src/main/API/FC-Control.h:27-35) | Internal `FlightStatus_e` (src/main/config/runtime_config.h:85-95) |
|---|---|---|
| 2 | FS_LOW_BATTERY | `Crash` |
| 3 | FS_INFLIGHT_LOW_BATTERY | `Low_battery` (warning) |
| 4 | FS_CRASHED | `LowBattery_inFlight` (critical) |
| 0, 1, 5, 6, 7, 8 | match | match |

That is why the testers saw 7 and 8 come back correct.

A return of 2 can only mean that the Crash bit is the lowest bit set. Battery is not involved. This was already logged as an out-of-scope side finding in crash-detect-false-trigger (README.md:33, SCOUT.md:69, TASKS.md:249) and never fixed.

### Code paths
- **FlightStatus_Get**, src/main/API-Src/FC-Control-Status.cpp:29-31: `return ( flightstatus_e ) leastSignificantBit ( flightIndicatorFlag );`
  - `leastSignificantBit` (src/main/common/maths.cpp:276-285) scans bits 0..8, so the lowest set bit wins.
  - Internal priority order: AccGyroCal 0 > MagCal 1 > Crash 2 > Low_battery 3 > LowBattery_inFlight 4 > Signal_loss 5 > Not_ok_to_arm 6 > Ok_to_arm 7 > Armed 8. Crash already outranks both battery states.
  - If no bit is set the function returns 65535 (`return -1`). In practice that does not happen: failsafe.cpp:259 sets Ok_to_arm while disarmed with an RC link.
- **FlightStatus_Check**, FC-Control-Status.cpp:33-35: `status_FSI ( ( FlightStatus_e ) status )` has the same mismatch.
  - `FlightStatus_Check ( FS_CRASHED )` tests the critical-battery bit.
  - `FS_LOW_BATTERY` tests Crash.
  - `FS_INFLIGHT_LOW_BATTERY` tests the battery warning.
- **MSP to the app**, src/main/io/serial_msp.cpp:
  - `MSP_FLIGHT_STATUS` (:867-~915) runs the same `leastSignificantBit ( flightIndicatorFlag )`, then a `switch` on the internal names that maps each one to `AppFlightStatus_e` (runtime_config.h:97-107). For example, `Crash` → `1 << App_Crash` (6) at :882-884.
  - LowBattery_inFlight is reported as App_Low_battery while armed (:893-897). This implements the "never tell the app critical while armed" invariant.
  - `MSP_STATUS` (:843-845) sends the raw `flightIndicatorFlag` with the internal bit layout.
- **Crash set**, src/main/flight/failsafe.cpp:363-384 `failsafeOnCrash ( )`, called from mw.cpp:1516.
  - Only runs when ARMED, `fsCrash` and ANGLE_MODE are all true.
  - Trips on angle > 70 deg, accSmooth X/Y > 5000, or raw accADC X/Y > 8000. It is skipped during a flip.
  - On a trip it calls `set_FSI ( Crash ); mwDisarm ( );`.
- **Crash clear**, src/main/mw.cpp:489-501. While disarmed, 300 ms after Crash is seen (`crashTimer.set ( 300, false )`), `crashRecoveryCheck` is set. Crash is then cleared once `STATE ( SMALL_ANGLE )` holds.
  - Crash therefore lasts about 0.3 s if the craft lands level, or until it is righted if not.
  - `mwDisarm` (mw.cpp:580-581) does not clear Crash.
- **Low battery set / clear**, src/main/sensors/battery.cpp:790-876 (`updateBatteryState`, `batteryApplyLevel`). It is driven only by the fuel-gauge warnLow / critLow levels. `mwDisarm` resets LowBattery_inFlight (mw.cpp:580), and the CRITICAL state sets it again on every update (battery.cpp:831). The crash path never touches the battery state.

### Same or different?
- **Same source:** both `FlightStatus_Get ( )` and `MSP_FLIGHT_STATUS` read `flightIndicatorFlag` through `leastSignificantBit`, so they use the same priority.
- **Different output:** MSP maps the result by name (`case Crash:` → `App_Crash`), so the app is told "crashed" correctly. The API casts the number straight across to an enum with a different order, so Crash comes out as 2 (FS_LOW_BATTERY).
- **Only other MSP difference:** the armed critical-battery case is softened to App_Low_battery. The API has no equivalent.

### Ranked hypotheses
1. **Enum mismatch. Proven.**
   - Evidence: the code above. A 2 is only possible from bit 2 = Crash. Values 7 and 8 match in both enums, which fits the report.
   - Test: bench, props off. Arm in ANGLE, tilt past 70 deg, keep it tilted, and `Monitor_Print` `FlightStatus_Get ( )` plus `FlightStatus_Check ( FS_CRASHED / FS_LOW_BATTERY )`. Expect 2 / 0 / 1 (under 30 bytes per tick).
2. **A real low-battery condition at the moment of impact. Killed.**
   - Low battery is bit 3 or bit 4, so it would come back as 3 or 4, never 2. It also ranks below Crash.
3. **Crash cleared before the read. Relevant to the retest, not to this bug.**
   - Crash clears about 300 ms after the disarm once the craft is level (mw.cpp:489-501). After the fix, a tester polling slowly may see 7 (FS_OK_TO_ARM) instead of FS_CRASHED.
   - Test: print every loop for 1 s after the disarm, and compare a level landing with a tipped one.

### Fix options for a later fix topic
- **A. Lookup table, internal → public.** Used in both Get and Check. Keeps the public numbers and the internal priority. Touches FC-Control-Status.cpp only.
- **B. Renumber the public enum to the internal order.** Source compatible, but the numbers the testers see change, and the doc would have to give the new order.
- **C. Do not reorder the internal enum.** The bit order *is* the priority, so reordering would let low battery mask Crash. It also changes the raw `MSP_STATUS` bits the app receives.

Either A or B changes how a public API behaves. Per CLAUDE.md that means bumping `API_Version` in the Makefile and updating the wiki.

### Docs and drift
- No doc describes `FlightStatus_Get ( )` or `FlightStatus_Check ( )`, or the priority order. docs/API/FC_CONTROL_API_WIKI.md only uses `FlightStatus_Check ( FS_ARMED )` (:42).
- FC-Control.h:54-70 comments: "Extracts the least significant bit ... casts" is accurate but leaves out the remapping problem.
- Naming drift: `FS_INFLIGHT_LOW_BATTERY` reads as "low battery in flight", but internal `LowBattery_inFlight` is the CRITICAL level, latched until the pack is changed.
- The pipeline docs were not checked for a flight-status section; none was found by grep under docs/.

### Questions for the user
1. Fix choice: keep the public numbers (option A) or renumber (option B)? Does any app or test script compare the raw numbers?
2. Should `FS_CRASHED` stay visible longer than about 0.3 s after a level crash (e.g. until the next arm) so slow pollers can see it?
3. Should the API also soften critical battery to `FS_LOW_BATTERY` while armed, as MSP does?
4. How did the testers poll (loop rate), and did the app show "crashed" at the same moment?
5. Should the fix include a wiki section for `FlightStatus_Get` / `FlightStatus_Check` with the priority order?

### Related topics
- crash-detect-false-trigger (closed `386599b`, which logged this side finding).
- crash-stuck-detection (planned; it changes when Crash is set).
- File history: `c062202` (FlightStatus_Check added), `88f594f` (battery FSI states).

Suggested skills: pluto-rules, pluto-build, pluto-flighttest, spec-to-code-compliance.
Docs to update at the end: docs/API/FC_CONTROL_API_WIKI.md (new flight-status section), FC-Control.h comments, and the pipeline doc for API status (enum mapping).
