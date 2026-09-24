# VL53L1X Altitude Hold Parity

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

| | |
|---|---|
| **Status** | **Closed - pending commit** ( 24 Sep 2026, FW 3.10.0 ). Pipeline doc, CHANGELOG, CLAUDE.md, FLIGHT_INVARIANTS, PIN_MAP and HARDWARE_RESOURCES promoted |
| **Branch** | `BugFix-June26`, base `b1f070b` |
| **Target** | `PRIMUS_X2_v1` with `LASER_TOF_L1x` + `LASER_ALT` ( VL53L1X drop-in on the L0X connector, I2C1 0x29 ) |
| **Origin** | [tof-althold-fusion](../tof-althold-fusion/README.md), which made the VL53L0X ( `LASER_TOF` ) path flight-ready |
| **Last updated** | 24 Sep 2026 |

**Plan ( 23 Sep 2026 ).** 13 tasks in [TASKS.md](TASKS.md), 19 after additions ( poll cost, reach flight, Long-mode tests ). First an L0X object-code baseline, then
VL53L1X bring-up on the bench and a check for a suspected driver freeze ( a missing
`ClearInterruptAndStartMeasurement` ). Then a driver fix at 45 ms / 50 ms ( budget raised from 33 ms after review ), a bench reach log, and the
shared fusion path ( scout option C, with the L0X `altitudehold.o` kept byte-identical ). Then the L1x
constants, bench and flight validation, and docs. Evidence and Q&A: [INVESTIGATION.md](INVESTIGATION.md).

## Open items

- **Next topics ( planned, not started ):** [vl53l1x-extended-reach](../vl53l1x-extended-reach/README.md) ( 270 / 240 cm band ) and [vl53l1x-error-latch-recovery](../vl53l1x-error-latch-recovery/README.md).
- Follow-ups: band extension ( see above ). Long mode alone does not reach further ( log-8, tasks 16-19 ); the likely lever is the preset's 1.5 MCPS minimum signal rate, a separate topic; `opticflow.cpp` still uses the L0X range.
- `target.h` ships with the laser defines off ( user, task 10 ); enable `LASER_TOF_L1x` + `LASER_ALT` for a VL53L1X.
- Temporary diagnostics: **removed in task 10** ( `PlutoPilot.cpp` and `target.h` at HEAD, laser defines off; poll timer removed ).

## Background ( written when the topic was noted, 22 Sep 2026 )

**Problem.** The `tof-althold-fusion` topic put its laser logic in the VL53L0X block of
`checkReading()` ( `flight/altitudehold.cpp`, `#ifdef LASER_TOF` ) only. A `LASER_TOF_L1x` +
`LASER_ALT` build compiles ( 113.2 KB / 15.8 KB ) and gets the shared fixes ( accelerometer Z deadband
0, one 1.5 s time constant, laser error against `_position_z` ), but its own branch is the old code:

| | VL53L0X ( `LASER_TOF` ) | VL53L1X ( `LASER_TOF_L1x` ) today |
|---|---|---|
| Tilt rejection above 25° | yes | no ( radian test never rejects ) |
| Handover | 160 up / 140 down | hard switch at 350 cm |
| Dropout timeout → baro | 120 ms | none |
| Baro offset | 2 s average, frozen on the baro | one sample of baro minus `EstAlt` |
| Frame shift on return | yes | no |
| Object hold-off, window test, re-base | yes | no |

**Known bug.** The L1x branch chooses the laser while `0 < ToF_Height < 350` and never checks
`isOutofRange_L1()`, and `ToF_Height` only updates on valid samples. Past the sensor's real reach
( Medium mode, ~2.9 m indoors, less in bright light or over dark floors ) the estimator keeps
correcting towards the last reading instead of handing over to the baro. **Until fixed, do not fly
the VL53L1X with `LASER_ALT` above ~2.5 m.**

**Planned approach.**

1. Factor the sensor-independent logic out of the VL53L0X block ( handover state, dropout timeout,
   frame shift, offset average, object and window tests ) so each sensor supplies its raw reading,
   validity, tilt and constants.
2. VL53L1X constants: its driver has no IIR ( no lag advance, no reseed ); band edges from a bench
   log of its reach on the flying floor; sample period from its timing budget.
3. Build gate on both sensors, bench log ( switch points ), flight ( hover, handover, box test ).
4. Pipeline doc: extend the Laser fusion section of `Altitude_Hold_Estimator.md`.
5. CLAUDE.md: once the VL53L1X branch has parity, move the flip and `LASER_ALT` paragraphs from
   the root `CLAUDE.md` "Flight invariants" section into
   `dev-guide/FLIGHT_INVARIANTS.md`, leaving a one-line rule for each in `CLAUDE.md`
   ( deferred from the 23 Sep 2026 CLAUDE.md restructure ).

Run `/pluto-grill` on this topic when a VL53L1X board is available.
