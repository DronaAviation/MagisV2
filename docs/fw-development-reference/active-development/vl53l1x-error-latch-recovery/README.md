# VL53L1X Error-Latch Recovery

| | |
|---|---|
| **Status** | **Planned** ( noted 24 Sep 2026; not started ) |
| **Branch** | not started |
| **Target** | `PRIMUS_X2_v1` with `LASER_TOF_L1x` + `LASER_ALT` |
| **Origin** | [vl53l1x-althold-parity](../vl53l1x-althold-parity/README.md) ( FW 3.10.0 ): found by its task 11 compliance pass |

**Problem.** Any failed I2C transaction with the VL53L1X latches `Global_Status_L1` for good. That covers set-up
in `ranging_init_L1 ( )`, and at runtime the data-ready check, the 17-byte result read and the interrupt clear in
`getRange_L1 ( )` ( [ranging_vl53l1x.cpp](../../../../src/main/drivers/ranging_vl53l1x.cpp) ). After that the
driver never talks to the sensor again, `isOutofRange_L1 ( )` stays true, and the estimator stays on the baro
**until a power cycle**. The same was true before FW 3.10.0.

**Where it matters.**

- **In flight:** one I2C glitch loses laser hold for the rest of the battery, with no warning to the pilot.
  Likely causes: a loose connector or vibration on the laser cable, motor EMI, or another I2C1 device ( baro at
  0x63, compass ) timing out. `i2cRead ( )` resets the I2C peripheral on a timeout, which can break a VL53L1X
  transaction in progress. The drone stays safe on baro hold, but the hold quality changes suddenly.
- **At boot:** if the sensor is not ready when `ranging_init_L1 ( )` runs ( slow power-up ), the laser is off
  for the whole session.

**How often so far:** never. `G` ( `Global_Status_L1` ) was 0 in every bench and flight log of the last topic.
This is a robustness gap, not an observed fault.

**Planned approach.**

1. After a latch, retry after a few seconds: `getRange_L1 ( )` clears the status and re-runs set-up.
2. Constraint: the full ST init ( `WaitDeviceBooted`, `DataInit`, `StaticInit`, mode / budget / period,
   `StartMeasurement` ) is tens of milliseconds of blocking I2C, too much for the loop in flight. Either retry
   only while disarmed, or split it into short non-blocking steps across polls. `WaitDeviceBooted` is blocking
   and must not run in flight as it is.
3. Make the latch visible ( log `G`, or a flag ), so a lost laser can be told apart from out of range.
4. Bench test: unplug and re-plug the sensor, disarmed and then armed with props off. Check the recovery time,
   that the loop time stays bounded, and that the estimator returns to the laser cleanly ( frame shift ).
5. Check the interaction with the estimator's dropout / return and the return guard.

Run `/pluto-grill` on this topic to start it.
