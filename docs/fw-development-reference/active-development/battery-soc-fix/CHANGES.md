# Battery SoC Fix: Changes

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [TESTING](TESTING.md) · [CHANGES](CHANGES.md)

## Task 13: `Vm` / `Is` diagnostic fields ( 28 Sep 2026, temporary )

- [PlutoPilot.cpp](../../../../PlutoPilot.cpp): `INA219_RegRead` declared in the `extern "C"` block; `inaBusMv ( )`
  ( bus mV, -1 on error or overflow ) and `inaShuntMa ( )` ( shunt mA, whole mA ) read the INA219 once per tick; the
  log line gains `Vm` and `Is`, and `Ph` and `G` are commented out for the byte budget ( ~118 B, ~127 B worst case ).
  **Remove before release** ( task 12 ).

## Task 2: INA219 driver, raw units and a valid flag ( 28 Sep 2026 )

| File | Change | Why |
|---|---|---|
| [ina219.c](../../../../src/main/drivers/ina219.c) `ina219ReadReg ( )` | new static read, false on an I2C error | a failed read is reported apart from the value ( 0xFFFF is a real shunt reading, -10 uV ) |
| ina219.c `INA219_RegRead ( )` | built on `ina219ReadReg`; signature and 0xFFFF on error unchanged | the task 13 diagnostic in `PlutoPilot.cpp` calls it |
| ina219.c `INA219_ReadBus_mV ( )` | new: bus in mV ( bits 15..3 x 4 mV ), false on an I2C error or the overflow bit | no 0.1 V floor for the SoC work |
| ina219.c `INA219_ReadShunt_10uV ( )` | new: signed shunt in the native 10 uV LSB, false on an I2C error | no whole-mV round-down ( 50 mA steps at 20 mOhm ) |
| ina219.c `bus_voltage ( )` / `shunt_voltage ( )` | compatibility shims on the new reads, same outputs as before ( incl. 0xFFFF / 0 on error ); wrong "mV" comment fixed | `battery.cpp` unchanged until task 3, which removes both |
| [ina219.h](../../../../src/main/drivers/ina219.h) | prototypes with units, ranges and failure rules | done-when: units in the header |
| [Makefile](../../../../Makefile) `PRIMUSX2_DRIVERS` | `drivers/ina219.cpp` → `drivers/ina219.c` | names the real file; the object list strips extensions, so the build is unchanged ( always compiled as C ) |

Register set-up unchanged ( PGA /4, 12-bit, 16 V, continuous ); one 2-byte I2C read per call as before. Gate
PRIMUS_X2_v1: no new `src/` warnings; flash 99.9 KB, RAM 14.8 KB ( RAM +0 ).

## Task 3: charge counter ( 28 Sep 2026 )

| File | Change | Why |
|---|---|---|
| [battery.cpp](../../../../src/main/sensors/battery.cpp) `updateINA219Voltage ( )` | reads `INA219_ReadBus_mV`, skips a failed read; 50-sample mean in mV into the new global `vBat_mV`; `vBatRaw` = `vBat_mV` / 100 ( 0.1 V, unit unchanged ) | no per-sample 0.1 V floor, no 0xFFFF in the average; telemetry, OLED and CRSF still read `vBatRaw` in 0.1 V ( `Bms_Get` reads `vBat_mV` since task 6 ) |
| battery.cpp `shuntAvgPush ( )`, `ProcessedINA219Current ( )` | signed 50-sample running average of the raw 10 uV reading ( int16 samples, int32 sum ); clamp at 0 once after averaging; mA = avg x 0.01 mV / 0.02 Ohm ( 0.5 mA per LSB, `INA219_MA_PER_10UV` with a `static_assert` ), rounded; a failed read keeps the last average | no whole-mV round-down ( was 50 mA steps ), negative samples no longer wrap into a uint16 ring |
| battery.cpp auto-gain | `ina219_auto_calibrate_current ( )`, `CURR_CAL_*`, `_ina219_current_gain` removed; `mAmpWithGain` = `mAmpRaw` | the gain always settled at -5% ( L1 ); the INA219 is trusted ( task 15 ); `Bms_Get` returned `mAmpWithGain` until task 6 ( now `mAmpRaw` ) |
| battery.cpp `updateINA219Current ( )` | reads and integrates on every call; `mA x us` in a uint64 ( `mA_us_accum` ), mAh = accum / 3.6e9, saturated at 0xFFFF; `mAhRemain` saturates at 0; `last_ms` → `last_us` | no sub-ms remainder lost; no wrap at empty ( H5b ); a failed read integrates the last good average so no time is lost |
| battery.cpp `vShuntRaw` | now the floored mV of the signed average | only the `vBatComp` line uses it, until task 5 |
| [battery.h](../../../../src/main/sensors/battery.h) | `extern uint16_t vBat_mV`; unit comments on `vBatRaw`, `mAmpRaw`, `mAmpWithGain`; `_ina219_current_gain` extern removed | |
| [ina219.c](../../../../src/main/drivers/ina219.c) / [.h](../../../../src/main/drivers/ina219.h) | `bus_voltage ( )` / `shunt_voltage ( )` shims removed | no callers left |
| [PlutoPilot.cpp](../../../../PlutoPilot.cpp) | `_ina219_current_gain` extern and the commented `G:` print removed; `mAmpRaw` comment updated | test code follows the removed gain |

Side effects: `vBatRaw` can read one 0.1 V step higher than before ( floor of the mV mean rather than the mean of floored
codes ); `mAmpRaw` is 0 when the average is at or below 0 ( it used to keep a stale value ). Gate PRIMUS_X2_v1: no new
`src/` warnings ( 3 old warnings in battery.cpp gone ); flash 99.8 KB, RAM 14.8 KB.

## Task 4: plug-in estimate ( 28 Sep 2026 )

| File | Change | Why |
|---|---|---|
| [battery.cpp](../../../../src/main/sensors/battery.cpp) `lipoRestCurve`, `lipoCellRestFraction ( )` | the INVESTIGATION.md §5.5 1S resting curve ( 21 points, flash ) and a linear interpolation, 0..1 per cell ( `[[maybe_unused]]` until task 5 used it on every target ) | E from the curve, not a straight line; task 5 reuses it |
| battery.cpp `handleBatteryConnected ( )` | no `delay ( 40 )`; sets the state, capacity and per-cell max as before, cells and thresholds from `vBat_mV`, E = 0 until ready | non-blocking boot path |
| battery.cpp `updateINA219Voltage ( )`, `handleBatteryPlugInEstimate ( )` | the 24 good bus samples after connect ( ~0.5 s at 21 ms ) are summed; then cells and thresholds again from that average, rest = avg + `mAmpRaw` x 120 mOhm, per cell, E = capacity x fraction, rounded, clamped 0..capacity | averaged sample ( H4 ), no wrap below 3.0 V ( H5 ) |
| battery.cpp `setCellCountAndThresholds ( )` | cells = ceil ( mV / 4400 ), 1..3, computed before the thresholds that use it | 1 cell at 4.20-4.35 V ( log-3: 2 at 4.3 V ) |
| battery.cpp `VBATT_PRESENT_MV` | 1000 mV against `vBat_mV` ( `>=` ), replacing `VBATT_PRESENT_THRESHOLD_MV` ( 0.1 V units, strict `>`, really 1.1 V ) | task 3 review: name and level |
| battery.cpp `updateBatteryStateSoc ( )` | the OK → WARNING transition also needs `batteryEstimateReady` | E is 0 for the first ~0.5 s; no false warning, beeper or app flag |

Review fixes: the pack thresholds are clamped to uint16 ( 3 cells x a per-cell value over 21.8 V would have wrapped );
`handleBatteryDisconnected ( )` leaves `batteryCellCount` at 1, not 0 ( `telemetry/frsky.c` divides by it ).

E for 600 mAh at ~125 mA idle: 4.30 / 4.20 V → 600, 4.10 → 530, 4.00 → 465, 3.80 → 240, 3.50 → 20, ≤ 3.20 → 0. Cells: 1 at
4.20 and 4.35 V, 2 at 6.0-8.4 V, 3 at 9.0-12.6 V. Gate PRIMUS_X2_v1: no new `src/` warnings ( 5 old ones in battery.cpp
gone ); flash 100.2 KB, RAM 14.8 KB ( +6 B ).

## Task 5: SoC model and warnings ( 28-29 Sep 2026, flight-safety relevant, three review rounds )

| File | Change | Why |
|---|---|---|
| [battery.cpp](../../../../src/main/sensors/battery.cpp) removed | `computeVbatComp_mV ( )` ( throttle sag ), `soc_linear_from_voltage ( )`, `soc_from_mAh ( )`, `fuse_soc_vbatComp_smart ( )`, `updateBatteryStateSoc ( )`; the sag and mAh rings; `vShuntRaw`, `wAh`, `soc_battery_percentage`, `soc_mAh_percentage`; `SYSTEM_R_MOHM`, `VBAT_SAG_*`, `THR_*`, `SOC_*_PCT` | the fused straight-line SoC warned late ( ~3% left, log-6 ) |
| battery.cpp `lipoCellRestVoltage ( )` | inverse of the §5.5 curve ( fraction → cell mV ) | the curve's own drop over the R window |
| battery.cpp `batteryResistanceUpdate ( )`, `batteryResistanceFromPoints ( )` | R = ( rest mV − curve drop − load mV ) / ( load mA − rest mA ); rest point frozen at the first arming, capped at 4180 mV/cell ( surface charge ); load = 25-35 s of loaded flight ( armed, ≥ 1500 mA ), summed across armings; clamped 80-250 mOhm; once per power-up; `batteryResistance_mOhm` exported | R differs per pack ( ~100-170 mOhm, tasks 13 / 14 ) |
| battery.cpp `BMS_Update ( )` `Vcomp` | per cell ( `vBat_mV` + I x R ) / cells, EMA 1 s, I = 0 while the current is stale; R = 100 mOhm until measured; re-seeded when R is measured; `vBatComp` = pack value | resting-voltage equivalent |
| battery.cpp remaining / SoC | remaining = E − drawn, pulled down persistently to curve ( `Vcomp` ) x C once that is ≤ 25%, only in loaded flight and only with a measured R; never rises; `soc_Fused` = remaining / C; held until E is ready | count-led; the voltage corrects an aged pack or a wrong capacity at the end |
| battery.cpp warnings | warning at `Vcomp` ≤ 3745 mV/cell or count ≤ 15% of C, critical at 3600 mV or 5%; 1.5 s debounce per source; voltage conditions only in loaded flight; latched until power-off; WARNING / CRITICAL re-assert their FSI flag every update ( `mwDisarm` resets `LowBattery_inFlight` ); `BatteryWarningMode` 0 / 1 / 2 | INVESTIGATION.md §5.7, user decisions 28-29 Sep |
| battery.cpp default-R alarms | before R is measured the voltage check runs only while the count is ≤ 40% of C; alarms it alone raised are provisional: re-levelled to the count level on disarm and when R is measured ( the measured-R voltage then raises a latched alarm through the 1.5 s debounce ) ( `batterySupportedLevel ( )`, `batteryApplyLevel ( )` ) | no lasting false alarm on a healthy pack; a wrong capacity still gets a voltage warning |
| battery.cpp stale sensor, settle cap | stale after 12 failed reads ( ~250 ms, `batterySensorStale` bits ): voltage timers hold, the R window pauses, the last current is integrated only while armed; the plug-in estimate completes after 96 calls ( ~2 s ) or sets E = 0 | task 3 / 4 reviews |
| battery.cpp no-current fallback | without current sensing: raw loaded cell voltage ≤ 3.10 V warning / ≤ 3.00 V critical ( fixed constants ), armed only, debounced, latched; SoC = curve at cell mV + 650 mV in flight, never rising; the stored config voltages are reported only | user decisions 29 Sep: the app rewrites the stored values with every capacity change |
| [battery.h](../../../../src/main/sensors/battery.h) | externs for `batteryResistance_mOhm`, `batterySensorStale`; removed the deleted globals; config field comments | |
| [mw.cpp](../../../../src/main/mw.cpp) | `BMS_Update` every 21 ms via `bmsLastServiced` ( the old test compared against a constant: every loop, and never between ~35.8 and 71.6 min after boot ) | real-time constants |
| [PlutoPilot.cpp](../../../../PlutoPilot.cpp) | ` R:` field after `Is`; ` M:` commented out ( ~128 B worst case ) | test code |

Readers whose meaning changed: `soc_Fused` ( MSP_ANALOG, CRSF, log `S` ), `mAhRemain` ( MSP, `Bms_Get` ), `vBatComp`
( MSP, log `Vc` ), `BatteryWarningMode` ( MSP ), `getBatteryState ( )` ( ledstrip, hott ). Gate PRIMUS_X2_v1: no new
`src/` warnings ( 13 old ones in battery.cpp gone ); flash 101.7 KB ( +1.5 KB over task 4 ), RAM 14.8 KB ( -44 B ).

## Task 6: `Bms_Get`, MSP and CRSF against the new model ( 29 Sep 2026 )

| File | Change | Why |
|---|---|---|
| [BMS.h](../../../../src/main/API/BMS.h) | `BMS_Option_e` gains `SoC`, `Warning_Level`, `Resistance` ( appended: existing values unchanged ); doc comments give units | user decision 29 Sep |
| [BMS.cpp](../../../../src/main/API-Src/BMS.cpp) | `Voltage` → `vBat_mV` ( exact mV ); `Current` → `mAmpRaw` ( no gain ); `SoC` → `soc_Fused` rounded; `Warning_Level` → `BatteryWarningMode`; `Resistance` → `batteryResistance_mOhm` | user decision 29 Sep |
| [Makefile](../../../../Makefile) | `API_Version` 1.3.2 → 1.4.0 ( `FW_Version` at commit ) | public API addition and behaviour change |
| [docs/API/BMS_API_WIKI.md](../../../../docs/API/BMS_API_WIKI.md) | new: options, units, validity, how the level is decided, examples per hook, 1.3.x → 1.4.0 changes | no BMS wiki existed |

**Every consumer of the changed values** ( value and unit after tasks 3-6 ):

| Consumer | Field | Value now | Unit |
|---|---|---|---|
| MSP_ANALOG ( `serial_msp.cpp`, 10 bytes, layout unchanged ) | u16 #1 | `vBatComp`: bus + I x R ( R 100 mOhm until measured ) | mV, pack |
| | u16 #2 | `mAmpRaw`: averaged current, no gain | mA |
| | u16 #3 | `mAhDrawn` | mAh |
| | u16 #4 | `mAhRemain`: E − drawn, pulled down near empty, never rising | mAh |
| | u8 #5 | `soc_Fused`: remaining / capacity | % 0-100 |
| | u8 #6 | `BatteryWarningMode`: 0 OK, 1 low, 2 critical | - |
| CRSF battery ( `mw.cpp` ) | voltage | `vBatRaw` | 0.1 V ( unchanged ) |
| | current | `mAmpRaw / 10` into a 0.1 A field: reads 10x high | older defect, out of scope ( 26 Sep ) |
| | capacity used | `mAhDrawn` | mAh |
| | remaining % | `soc_Fused` | % |
| FrSky / HoTT / SmartPort telemetry | voltage, current, mAh | `vBatRaw` ( 0.1 V ), `mAmpRaw / 10` ( 10 mA ), `mAhDrawn` | unchanged units |
| OLED, LCD display | voltage | `vBatRaw` | 0.1 V, unchanged |
| ledstrip, HoTT | battery state | `getBatteryState ( )`: new trigger rules, latched | enum |
| `Bms_Get` | all options | see the wiki's option table | |

## Task 7: build gate ( 29 Sep 2026 )

PRIMUS_X2_v1, all topic changes: no new `src/` warnings, 13 old ones in `battery.cpp` fixed ( refresh the baseline at
commit ), no `PlutoPilot.cpp` warnings. Flash 101.7 KB ( 99.7 KB before the topic, +2.0 KB ), RAM 14.8 KB ( unchanged ).

## Task 8: log line for the bench and flights ( 29 Sep 2026, temporary )

- [PlutoPilot.cpp](../../../../PlutoPilot.cpp): the task 13 `Vm` / `Is` fields and their helpers ( `inaBusMv ( )`,
  `inaShuntMa ( )`, the `INA219_RegRead` extern ) removed: `V` is exact mV through `Bms_Get` since task 6. The line is now
  `t [Ph] V Vc I R D S L M Arm` ( `L` = `Bms_Get ( Warning_Level )`, `Ph` only with the bench sequence on ), ~125 B worst
  case on the bench. `BENCH_MOTOR_SEQUENCE` is 1 for task 8 ( **props off** ); back to 0 before task 9. **Remove before
  release** ( task 12 ).

