# Scout brief ( pluto-scout, 25 Sep 2026 )

Verbatim reconnaissance from planning. Two parts: the initial brief and the follow-up on the auto-gain, history,
logging and INA219 setup. Line numbers are as of `HEAD` 7cf2444 on `BugFix-June26`; the graphify CLI was not on
PATH and the graph was stale ( built b1f070bb ), so the scout used grep.

## Part 1: initial brief

Scout brief: BMS reads about 300 mAh remaining when the cell is at 3.1 V. Check the battery pipeline end to end.
Graph: stale (built b1f070bb, HEAD 7cf2444). The `graphify` CLI is not on PATH, so I used grep.

Subsystems: INA219 driver, battery/BMS, MSP, CRSF telemetry, user API. Pipeline docs: `fw-architecture-pipeline/subsystems/Power_BMS_Pipeline.md`, `Telemetry_Pipeline.md`.

Key code:
- `src/main/drivers/ina219.c:49-65`: `INA219_Init`, `bus_voltage()`, `shunt_voltage()`
- `src/main/sensors/battery.cpp:184` `handleBatteryConnected`: works out cell count and the one-time `EstBatteryCapacity`
- `battery.cpp:329` `updateINA219Current`: the coulomb counter, which sets `mAhDrawn` and `mAhRemain`
- `battery.cpp:628` `BMS_Update`: compensated voltage, voltage SoC fused with mAh SoC, battery state
- `src/main/mw.cpp:423-454` (in `annexCode`): scheduling and the CRSF feed
- `src/main/io/serial_msp.cpp:995` `MSP_ANALOG`: what the app receives
- `src/main/API-Src/BMS.cpp:26` `Bms_Get`: the user API
- `src/main/config/config.cpp:298` `resetBatteryConfig`: defaults

Mode: check

Current behaviour:
- **Sensing.** The INA219 sits on I2C1 (PB8/PB9). Voltage comes from `bus_voltage()`, which returns 0.1 V units rounded down, even though its comment says mV (`ina219.c:57`). Current comes from `shunt_voltage()` in whole mV rounded down (`:64`) times 50, i.e. a 0.02 Ohm shunt (`ina219.h:29`). Gain is /4, so the ADC saturates at +/-160 mV, which is 8 A (`ina219.c:51`).
- **Filtering and rate.** Both are 50-sample averages (`battery.cpp:50,54`), read every 21 ms (`mw.cpp:117-119`).
- **Starting capacity.** At plug-in it assumes capacity is linear in voltage: `Est = (V - Vcrit) * cap / (Vmax - Vcrit)` (`battery.cpp:209`). This uses one sample taken before `delay(40)`.
- **Remaining.** `mAhRemain = Est - mAhDrawn` (`:357`). It is pure coulomb counting and is never corrected from voltage. Voltage only feeds the fused % (`:633-637`).
- **Low battery.** Warning is fused SoC <= 18%, critical <= 8% (`battery.cpp:136-138`, `:569`, `:578`). The response is only beeper, LED and the FSI flag sent to the app (`serial_msp.cpp:885-892`). There is no auto-land (`failsafeOnLowBattery` is commented out at `failsafe.cpp:353-359`), and arming is not blocked.

Analysis (most likely first):
1. **The mAh count under-reads the real draw.** Contributing factors:
   - The auto-gain compares shunt mV with `vBatRaw` in 0.1 V units (`battery.cpp:264,268`), so the gain slides to the 0.95 clamp while armed: about -5%.
   - `dtMs = dtUs/1000` throws away the fraction every 21-24.5 ms (`:345`): about -2%.
   - Rounding down twice (shunt to mV, then the average) biases current by about -50 mA.
   - Anything above 8 A is clipped.
   - The shunt value is unverified. Commit 278f77d changed the constant from 0.4 to 0.02 Ohm but kept the same x50 scale.

   Against: the known code errors add up to about 10%, not the ~40% the symptom needs. That points to the shunt or scale, or to H2.
   Test: bench power supply plus an ammeter at 1/3/6/9 A, compared with `Bms_Get(Current)`. Then a full discharge, comparing the final `mAhDrawn` with what the charger puts back.
2. **The capacity setting is larger than the pack's real usable capacity.** `Rx_ELRS` forces 800 mAh (`RxConfig.cpp:172`) against a default of 600 (`config.cpp:312`). The app can write the value through `MSP_SET_VOLTAGE_METER_CONFIG` (`serial_msp.cpp:1704`) and it persists in EEPROM. An aged pack would also fit. Numbers: 800 minus about 500 real mAh gives exactly 300. Test: read `MSP_VOLTAGE_METER_CONFIG` or `Bms_Get(Battery_Capicity/Estimated_Capacity)` at boot.
3. **The estimate is never re-anchored, and the voltage model is wrong.** The linear map overstates a part-charged pack; at 3.8 V resting it says 67% where the real figure is about 50%. It uses one sample at plug-in, rounded down to 0.1 V. Test: log `Estimated_Capacity` at power-on for packs resting at 4.2, 3.9 and 3.8 V.
4. **3.1 V is voltage under load.** With about 100 mOhm and 4 A the resting voltage would be about 3.5 V, so some of the reported remainder is real. Test: land, then read the resting voltage after 60 s.
5. **Minor unit/integer bugs** (these push the other way or only hit edge cases):
   - `mAhRemain` wraps to about 65k once drawn exceeds Est, and `soc_from_mAh` then reports 100% (`:357,433`).
   - A negative `shunt_voltage` becomes 65535 in the unsigned average and over-counts.
   - Cell count at 4.2 V is computed as 2 (`:200`).
   - A replug while on USB zeroes the thresholds (`:216`).

Also: `BMS_Update` runs every loop, because `mw.cpp:442` compares against the constant instead of a last-run timestamp. So the "2% per call" rate limit (`:537`) and the ms "stale" checks (`:392`, which compare us against 200) mean nothing.

Constraints / invariants that apply:
- `Monitor_Print` must stay at 130 bytes or less per tick (CLAUDE.md).
- `MSP_ANALOG` is 10 bytes with a fixed layout the app parses; keep it or coordinate with the app.
- `Bms_Get` is a public API: bump `API_Version` if it changes.
- The warning baseline gate applies to `src/`.
- Battery ring buffers use about 260 B of RAM. The INA219 shares I2C1 with the baro, compass and ToF, so bus time matters.

Resource conflicts: none (I2C1 is already allocated; no DMA/timer involved).

Already answered by the code:
- There is a physical current sensor: INA219 on PRIMUS_X2_v1 and PRIMUS_V5 (`target.h:147-148` / `:148-149`).
- Legacy PRIMUSX2 has no `INA219_Current` (`target.h:143`), so Est=0 and `mAhRemain` wraps.
- The app receives `vBatComp`, raw mA (before gain), `mAhDrawn`, `mAhRemain`, fused %, and warning mode.
- CRSF receives `mAhDrawn` plus %, not the remaining mAh. Its current is sent as `mAmpRaw/10`, which is 10 mA units, but the protocol expects 0.1 A.
- Nothing is settable over CLI: the CLI entries point at fields that are commented out (`serial_cli.cpp:400-408`).

Questions for the user:
1. Where do you see "300 mAh remain": the app, `Bms_Get` in your code, or the ELRS radio?
2. Pack rated capacity, chemistry (LiPo/LiHV), age. Which receiver mode (`Rx_ESP` at `PlutoPilot.cpp:16`, or ELRS = 800)? Has the app ever set a capacity?
3. Was 3.1 V under load in flight or at rest? How many mAh does the charger put back from that point?
4. Board shunt part/value (BOM), and does the INA219 see the full motor current?
5. Do you have PlutoMonitor logs of a full discharge?
6. Should critical battery auto-land or block arming?
7. Can the `MSP_ANALOG` layout change?

Doc drift (`Power_BMS_Pipeline.md` vs code):
- The doc names `ina219.cpp`, `ina219Init`, `batteryUpdate`, `updateBatteryStatus`. The code has `ina219.c:49` `INA219_Init`, `battery.cpp:232,329,628`.
- The doc says "amperage cA". The code works in mA (`battery.cpp:306`).
- The doc says thresholds are per-cell voltage. The code uses SoC % (`:136`).
- The doc says critical "Notify Failsafe". The code has no failsafe action (`failsafe.cpp:353`).
- The doc says 2S/3S. The code has `BRNG_16V` and 1S limits.
- The Makefile lists `drivers/ina219.cpp` (`Makefile:330`), but the file is `ina219.c`.
- `Telemetry_Pipeline.md:14` refers to `vbat`; the code uses `vBatRaw`.

Prior work: no topic in `active-development/`. Relevant commits: 278f77d (shunt 0.4 to 0.02 Ohm, `CURR_CAL_ALPHA`), 88f594f, 0b171fe, b7e8040, 704c079.

Suggested skills: pluto-rules; pluto-driver (INA219 PGA/calibration register); pluto-flighttest plus agent pluto-log-analyst (discharge logs); pluto-build; pluto-commit; agent c-pro.

Pipeline docs to update at the end:
- `Power_BMS_Pipeline.md`: rewrite it (real functions, units, timing, SoC fusion, low-battery actions).
- `MSP_Communications_Pipeline.md`: the `MSP_ANALOG` fields.
- `Telemetry_Pipeline.md`: CRSF battery units.
- `Failsafe_Subsystem.md`: only if auto-land is added.
- New `docs/API/` BMS wiki: only if `Bms_Get` changes.

## Part 2: follow-up ( auto-gain, history, logging, INA219 setup )

Nothing measured; this is from the code and git history only.

**1. Auto-gain**
- The formula is in `src/main/sensors/battery.cpp:268-283`: `gain_err = (vShuntRaw - vBatRaw) / (mAmpRaw * 100 / 1000)`.
  - `vShuntRaw` is shunt mV averaged over 50 samples.
  - `vBatRaw` is in 0.1 V units (about 37).
  - `mAmpRaw * 100 / 1000` is the expected sag in mV from `SYSTEM_R_MOHM` 100.
  - `vShuntRaw` = I/50, so the ratio works out to about 0.2 - 10*V/I, which never exceeds 0.2.
- Clamp: 0.95 to 1.05 (`:96-97`). EMA alpha 0.003 (`:95`).
- When it runs: it is called every 21 ms (`mw.cpp:119,434-438`). It only learns when armed, `mAmpRaw` >= 1000, `vShuntRaw` > `vBatRaw` and observed sag >= 20 (`:264,269`). That means about 2.7-2.9 A and above.
- In practice the gain always drives to **0.95** while flying: time constant about 330 samples, about 7 s at hover current. It is a `static` that is never reset until power-off (`:262`).
- **If the shunt were 2 mOhm:**
  - The x50 scale would read 10x low: 4 A real shows 400 mA.
  - The learning gate (`mAmpRaw` >= 1000) would never open, so the gain stays at 1.0.
  - A real 600 mAh flight would log about 55-60 mAh drawn and about 540 remaining.
  - The user sees about 300, which rules this out.
  - Also, PE1206FRE470R02L means R02 = 0.02 Ohm = 20 mOhm. The removed comment's "2 mOhm" was a label error, and the constant 0.002 was used after `/10`, so it matches 20 mOhm.

**2. History** (`git log` for `battery.cpp`, `ina219.*`, `BMS.cpp`)
- **88f594f** (2025-12-31) is the rewrite, and the only plausible regression. Before it (`88f594f^:battery.cpp:197-202,371-396`):
  - the counter used `millis()` with no carried fraction lost;
  - it took one shunt sample per tick;
  - it had no gain;
  - negative mV was forced to 0.

  After it:
  - `dtMs = dtUs/1000` with `last` stored in us (about -2%);
  - the auto-gain to 0.95 (-5%);
  - the 50-sample u16 ring, which adds another round-down (about -25 mA) and lets negative shunt through as 65535 (`:300-302`);
  - the one-time `Est` is unchanged;
  - the capacity default of 600 was added (`config.cpp:312`).

  Net effect about -8%, not 2x.
- **0b171fe**: SoC warning thresholds (18/8 to 20/10), plus the `BatteryWarningMode` byte in `MSP_ANALOG`. No mAh logic changed.
- **b7e8040**: config/MSP renames. The capacity is now read from `BatteryCapacity`, same value.
- **704c079**: API/MSP naming, plus `BatteryCapacityChanged` (reboot after the EEPROM write). No mAh logic changed.
- **278f77d**: alpha 0.002 to 0.003; `Bms_Get(Current)` now returns `mAmpWithGain`. The conversion is numerically the same, and the change is under 1%.
- No commit changed the current scale. "Closer before 278f77d" cannot come from that commit's code. If the memory is right, the ~8% comes from 88f594f; otherwise it came from config or the pack.

**3. Logging**
- PlutoMonitor records only `Monitor_Print` output from `plutoLoop()`, one `tag:value` per line (`tools/flightlog.py:4-8,30`). It does not record `MSP_ANALOG`, and `flightlog.py` has no battery fields.
- The log runs only in Developer Mode with a live RC link (`pluto-flighttest/SKILL.md:72-73`), every 100 ms (`API-Utils.cpp:101`). `PlutoPilot.cpp:33` currently has an empty `plutoLoop`.
- Minimal line, all `int`, at about 5 B framing plus tag plus value per field:

  `V:`Bms_Get(Voltage) mV, `I:`mAmpRaw, `G:`(int)(_ina219_current_gain*1000), `D:`Bms_Get(mAh_Consumed), `S:`(int)soc_Fused, `Arm:` gives about **58 B/tick**.

  - Log `E:`Bms_Get(Estimated_Capacity) once in `onLoopStart()`.
  - Adding `Vc:`vBatComp makes it about 70 B, still under the 130 B ceiling.
  - `mAmpRaw`, `soc_Fused`, `vBatComp` and `_ina219_current_gain` need `extern "C"` declarations (`battery.h:74,84,92,93`).

**4. INA219 setup** (`ina219.c:49-52`)
- Settings: RST 0, `BRNG_16V`, `GAIN_4` (+/-160 mV, which is 8 A at 20 mOhm), bus ADC 12-bit, shunt ADC 12-bit with one sample (532 us, no averaging), mode 7 (shunt and bus, continuous). The config word is 0x119F.
- The calibration register 0x05 is never written. The driver reads only shunt 0x01 (`:60-65`, `int16` to whole mV, rounded toward zero) and bus 0x02 (`:54-58`, rounded down to 0.1 V; the comment says mV). The current and power registers are unused.
- On an I2C error `RegRead` returns 0xFFFF (`:41`): bus returns 0xFFFF into the average, and shunt returns 0.
- Motor PWM is 20 kHz (`config.cpp:84`), so the 532 us window averages about 10 PWM periods. That is noisy but not biased.

**Implication:** the known code errors explain about 8-10%. The remaining roughly 2x gap points to:
- the real shunt value or board layout (check the marking: R020 would confirm the scale, R010 would explain 2x);
- the INA219 not seeing all the motor current;
- the one-time `Est` taken from a floored single sample.

A full discharge logged with the line above, compared with the charger's mAh readback, separates these.

## Planning note: what commit 278f77d changed ( checked in the main session )

`git log -1 -p 278f77d` on the battery files: `INA219_SHUNT_RESISTOR` 0.4 to 0.02f; the `INA219_SHUNT_RESISTOR_MILLIOHM 0.002f // 2 mOhm for PE1206FRE470R02L` define commented out; the gain EMA now uses `CURR_CAL_ALPHA` ( 0.002 to 0.003 ) instead of the milliohm constant; `mAmpRaw = ( vShuntRaw / 10 ) / ( INA219_SHUNT_RESISTOR / 10 )`, numerically the same as the previous `( vShuntRaw / 10 ) / 0.002`. Both are shunt mV x 50.
