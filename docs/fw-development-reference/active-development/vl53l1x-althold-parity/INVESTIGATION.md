# VL53L1X Altitude Hold Parity - Investigation

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md)

## Problem

The [tof-althold-fusion](../tof-althold-fusion/README.md) topic ( closed in `484226c`, FW 3.9.0 ) made the
VL53L0X ( `LASER_TOF` + `LASER_ALT` ) altitude path flight-ready. All of its laser logic lives in the
`#ifdef LASER_TOF` block of `checkReading()` in
[altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp). The VL53L1X ( `LASER_TOF_L1x` ) branch
still runs the old code, and the sensor on the bench has never been powered with this firmware. The
driver has no bench or flight record since it was integrated in `be31216` ( Nov 2025 ).

## Evidence ( from the code, scout brief 23 Sep 2026 )

| # | Finding | Where | Status |
|---|---|---|---|
| 1 | **Stale reading past reach.** Out of range, the driver sets `out_of_range_L1` and keeps the last range. `ToF_Height` updates only on valid samples, and the fusion test `0 < ToF_Height < 350` checks neither validity nor age. The estimator therefore keeps correcting towards a frozen value and the baro never takes over. | `ranging_vl53l1x.cpp:143-148`, `altitudehold.cpp:1209,1219` | proven ( code ) |
| 2 | **Tilt test never rejects.** `calculateTiltAngle()` returns deci-degrees. The branch converts that to radians ( at most about 1.57 ) and then tests `< 25`, which is always true. | `altitudehold.cpp:537-538,1214` | proven ( code ) |
| 3 | **Driver may freeze after one sample.** `getRange_L1` never calls `VL53L1_ClearInterruptAndStartMeasurement()` after reading the result. The ST API documents that call as the handshake that allows the next range. Without it, the data-ready flag either stays set, so the same result is re-read, or `GPH_SYNC_CHECK_FAIL` latches `Global_Status_L1`. Either way `ToF_Height` freezes, and finding 1 then turns the freeze into a flight hazard. | `ranging_vl53l1x.cpp:129`, `lib/main/VL53L1X_API/core/src/vl53l1_api_core.c:2397-2404`, `vl53l1_core.c:255,311` | **freeze killed by log-1** ( 178 s of fresh ranges, `G` 0 ). The missing handshake instead makes the driver **re-read the result every 10 ms** and flag each read as new ( task 2 ) |
| 4 | **Noisy handover offset.** On every tick `baro_offset` is set to one sample of the 0.75-IIR baro minus the integer `EstAlt`. The switch to the baro therefore carries the IIR lag plus about 13 cm sd of baro noise. | `altitudehold.cpp:985,1232` | proven ( code ) |
| 5 | **No handover logic.** The branch has a hard 350 cm switch with no hysteresis, no dropout timeout, no frame shift and no object hold-off. `altSourceLaser` never changes. | `altitudehold.cpp:1208-1235` | proven ( code ) |
| 6 | **100 ms cadence.** The ST default preset uses a 41 ms budget and a 100 ms period. The L0X logic was tuned for about 33 ms samples, so a 0.5 s window would hold only 5 L1x samples. | `vl53l1_api.c:996-1007,1027-1100` | proven ( code ) |

## Causal chain ( the known bug )

The craft is flown above the sensor's reach, beyond about 2.9 m in Medium mode indoors. The driver
reports out-of-range ( finding 1 ), or has frozen since the first sample ( finding 3 ). `ToF_Height`
keeps its last value, which is below 350, so the branch keeps calling `correctedWithTof(stale)` every
tick. The estimator is pulled towards a height the craft has left, and altitude hold flies the craft
towards it.

## Hypotheses still open

- **H1 ( finding 3 ) is real.** For: the ST handshake is missing, and the ST sample code calls it after
  every read. Against: some ST presets re-arm on their own in timed mode. **Test ( task 2 ):** bench log
  of `StreamCount`, `Global_Status_L1`, `Range_Status_L1` and the raw mm while the board is moved by
  hand. If `StreamCount` stops advancing or `Global_Status_L1` goes non-zero, H1 is real.
- **The 160/140 band suits the L1x on this floor.** Medium mode should reach well past 160 cm on the
  same matt floor. **Test ( task 4 ):** bench reach log.

## Bench findings ( task 2, `logs/log-1.txt` )

- H1 ( freeze ) is dead. The sensor keeps ranging without the handshake.
- The handshake is still missing, so data-ready never clears. Every 10 ms poll re-reads the result and
  sets `isTofDataNewflag_L1`, feeding the estimator duplicates at about 100 Hz.
- `StreamCount` only toggles 0 ↔ 1 in this preset, so it is not usable as a freshness or interval
  measure.
- Out of range is flagged ( `St` 2 / 4 ), but `NewSensorRange_L1` holds the last valid value: bug 1
  confirmed on hardware.
- Covered window: `St` 0 with 0-5 mm, so a minimum-valid range is needed.
- Possible signal fail from about 1.9 m on this floor ( task 4 ).

## Q&A that shaped the plan ( 23 Sep 2026 )

| Question | Answer |
|---|---|
| Mode | fix |
| Target | PRIMUS_X2_v1, `LASER_TOF_L1x` + `LASER_ALT` |
| Done when | Same flight bar as the L0X: hover ±3 cm for 30 s, clean 160/140 handover, box test with hold-off and re-base, and a baro handover past the sensor's reach |
| Scope | Refactor, parity, docs, and the CLAUDE.md → FLIGHT_INVARIANTS.md move |
| Bench state | VL53L1X never powered with this firmware, so the plan starts with bring-up |
| L0X regression | The L0X board is no longer available. The refactor must leave the L0X `altitudehold.o` byte-identical, proven with an `objdump` diff |
| Reach / mode | Keep the driver's Medium mode for now. Later, extend the band to 340 cm up / 300 cm down ( follow-up ) |
| Wiring | Drop-in on the L0X connector: I2C1, 0x29, PB8/PB9 |
| Cadence | 33 ms budget / 50 ms period. L1x sample counts derived from the period |
| XVision / `LaserSensor_L1` ( 964 B RAM, unused ) | Out of scope, follow-up |
| Floor | The same indoor matt floor as the L0X flights |
