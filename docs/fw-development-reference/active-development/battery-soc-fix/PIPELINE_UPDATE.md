# Battery SoC Fix: Pipeline Update ( staged )

[README](README.md) · [TASKS](TASKS.md) · [INVESTIGATION](INVESTIGATION.md) · [TESTING](TESTING.md) · [CHANGES](CHANGES.md)

**In progress.** Task 10 rewrites this file into the full replacement for
[`subsystems/Power_BMS_Pipeline.md`](../../fw-architecture-pipeline/subsystems/Power_BMS_Pipeline.md), which is
copied over it at commit. Until then this is the list of what the rewrite must carry.

## Items collected so far

- **Task 2 ( 28 Sep 2026 ), driver:**
  - the hardware driver is `src/main/drivers/ina219.c` ( C ), not `ina219.cpp` ( pipeline doc line 7 );
  - `INA219_ReadBus_mV ( )`: bus voltage in mV, 4 mV LSB, false on an I2C error or the overflow bit;
  - `INA219_ReadShunt_10uV ( )`: signed shunt voltage in the native 10 uV LSB ( 0.5 mA at 20 mOhm ), false on an I2C
    error;
  - `bus_voltage ( )` / `shunt_voltage ( )` are gone after task 3 ( shims until then ).
- **Task 3 ( 28 Sep 2026 ), counter:**
  - bus voltage averaged in mV ( 50 samples ) into `vBat_mV`; `vBatRaw` = `vBat_mV` / 100 ( 0.1 V, floored ) for its
    existing users; a failed read is skipped;
  - shunt averaged signed in 10 uV ( 50 samples, running sum ), clamped at 0 once, mA = avg x 0.01 mV / 0.02 Ohm
    ( 0.5 mA per LSB ); a failed read keeps the last average;
  - the auto-gain is gone ( no calibration factor: the INA219 is trusted, task 15 ); `mAmpWithGain` = `mAmpRaw`;
  - the counter integrates mA x us in a uint64 every 21 ms call; `mAhDrawn` saturates at 0xFFFF, `mAhRemain` at 0;
  - `vShuntRaw` ( mV ) was kept for the old `vBatComp` line; task 5 removed both.
- **Task 4 ( 28 Sep 2026 ), plug-in estimate:**
  - presence at `vBat_mV` >= 1000 mV; on connect no blocking delay; cells = ceil ( mV / 4400 ), 1..3, before the
    thresholds;
  - E from the average of the 24 bus samples after connect ( ~0.5 s ) plus `mAmpRaw` x 120 mOhm, per cell, on the 1S
    LiPo resting curve ( 21 points, 4.20 V = 100%, 3.27 V = 0% ) times the configured capacity; no warning before E
    is ready.
- **Task 5 ( 28-29 Sep 2026 ), SoC and warnings:**
  - per-flight pack resistance R: rest point frozen at the first arming ( capped at 4.18 V/cell ), load point over
    25-35 s of loaded flight ( armed, ≥ 1.5 A ) summed across hops, less the curve's drop; 80-250 mOhm; once per power-up;
  - `Vcomp` per cell = ( bus + I x R ) / cells, 1 s EMA; R = 100 mOhm until measured; I = 0 while the current is stale;
  - remaining = E − drawn, pulled down to curve ( `Vcomp` ) x capacity once ≤ 25% ( loaded flight, measured R only ),
    never rising; SoC = remaining / capacity;
  - warning at `Vcomp` ≤ 3.745 V/cell or ≤ 15% counted, critical at 3.60 V or 5%; 1.5 s debounce; voltage checks only in
    loaded flight; latched until power-off, flags re-asserted every update; in hover ~3.0-3.1 V / ~2.9-2.95 V measured;
  - before R is measured the voltage check runs only when the count is ≤ 40%, and its alarms are provisional
    ( re-levelled to the count on disarm and when R is measured );
  - without current sensing: raw loaded cell voltage ≤ 3.10 V warning / ≤ 3.00 V critical ( firmware constants ), armed
    only; SoC from the curve at cell voltage + 0.65 V in flight; the stored config voltages are reported, not used;
  - stale sensor after ~250 ms without a good read; `BMS_Update` every 21 ms.
