# Battery Capacity Estimate: Investigation

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md)

Why the BMS says ~300 mAh remain when the pack is empty. Readable without the code; line references point to
[battery.cpp](../../../../src/main/sensors/battery.cpp) and [ina219.c](../../../../src/main/drivers/ina219.c)
unless stated.

## 1. Problem

On PRIMUS_X2_v1 with the stock 600 mAh 1S pack, the Pluto app's battery widget shows about 300 mAh remaining
when the cell reaches 3.1 V. The figure is the same in flight ( under load ) and after landing once the pack has
rested, so the pack is genuinely empty at that point and the count is about 2x optimistic. It happens with both
receiver builds ( `Rx_ESP`, capacity 600; `Rx_ELRS`, capacity forced to 800 ), which argues against the capacity
setting being the whole cause. The user recalls the figure being closer on older firmware.

The remaining figure the app shows is `mAhRemain = EstBatteryCapacity - mAhDrawn` ( `battery.cpp:357` ):
a coulomb count from the INA219 shunt voltage, never corrected from the cell voltage.

## 2. Evidence so far

| Source | What it shows |
|---|---|
| App display ( user ) | ~300 mAh remaining at 3.1 V, under load and at rest, both Rx modes, 600 mAh pack |
| Code ( scout, [SCOUT.md](SCOUT.md) ) | Current = shunt mV x 50 ( 20 mOhm assumed ); known integration losses of ~-8 to -10% since `88f594f`; no voltage re-anchoring; plug-in estimate from one floored 0.1 V sample |
| `git log -p 278f77d` | Only `CURR_CAL_ALPHA` 0.002 to 0.003 changed in the current path; the conversion is numerically identical before and after, so the "closer before" recollection is not explained by that commit |
| Shunt marking ( user, 25 Sep 2026 ) | **R020** = 20 mOhm: the code's `shunt mV x 50` is the right scale; the shunt-value half of H1 is ruled out |
| Current path ( user, 25 Sep 2026 ) | The shunt sits between the battery and the entire circuit: the INA219 sees all the current |
| Second look at the board ( user, 25 Sep 2026, evening ) | **A second R020 is stacked in parallel on the first: 10 mOhm total.** The code's x50 is for 20 mOhm |
| log-1 ( 25 Sep 2026 ) | Symptom reproduced ( 297 remaining on an empty pack ); counter within 1.4% of an independent integral of its own reading; hover current reads flat at ~2.25 A |
| Charger ( user, 25 Sep 2026 ) | 430 mAh put back from 3.5 V resting. Shunt part PE1206FRE470R02L ( 0.02 Ohm ), one resistor, no parallel path |
| Not yet available | A run from a full pack; whether more than one pack shows the symptom |

## 3. Current pipeline ( audited 25 Sep 2026, task 1 )

Every stage checked against the source at `HEAD` on `BugFix-June26`. Paths are under `src/main/`.

### 3.1 Sensing ( INA219 on I2C1 )

| Stage | What the code does | Where |
|---|---|---|
| Config | `0x119F`: 16 V bus range, PGA /4 ( +/-160 mV shunt = **8 A** at 20 mOhm ), 12-bit single-sample ADCs ( 532 us ), shunt+bus continuous. Calibration register never written; current/power registers unused. | `drivers/ina219.c:49-52` |
| Init | `INA219_Init ( )` at boot; > 300 ms of delays follow before the loop, so the first sample is a completed conversion | `main.cpp:369`, `:399-401`, `:544` |
| Bus voltage | `( reg >> 3 ) x 4 / 100`: **0.1 V units, floored** ( comment says mV ). Returns `0xFFFF` on an I2C error or when the OVF bit is set. | `drivers/ina219.c:54-58`, `:41` |
| Shunt voltage | `int16 raw x 10 / 1000`: whole mV, truncated toward zero. An I2C error reads as raw `0xFFFF` = -1 LSB → 0 mV. | `drivers/ina219.c:60-65` |

### 3.2 Scheduling

| Stage | What the code does | Where |
|---|---|---|
| Voltage | `updateINA219Voltage ( )` when >= 21 ms ( `VBATINTERVAL` 6 x 3500 us ) since last, from `annexCode ( )` | `mw.cpp:117`, `:424-428` |
| Current | `updateINA219Current ( currentTime, armed )` on the same 21 ms gate ( `IBATINTERVAL` ); runs armed **and disarmed** | `mw.cpp:119`, `:430-439` |
| BMS | `BMS_Update ( )` compares `currentTime` against the **constant** 21000, so it runs on every `annexCode ( )` call ( every loop ), not every 21 ms | `mw.cpp:442-448` |
| CRSF | `crsfSetBatteryTelemetry ( vBatRaw, mAmpRaw / 10, mAhDrawn, soc_Fused )` every loop | `mw.cpp:452-454` |

### 3.3 Voltage path and the plug-in estimate

- `vBatRaw` = 50-sample ring average of `bus_voltage ( )` ( 0.1 V units ). The ring divides by the number of samples
  held, not the buffer size, so the first readings are not diluted ( `battery.cpp:235`, `common/maths.cpp:378-382` ).
- Battery "connected" when `vBatRaw > 10` ( 1.0 V; the constant is named `_MV` but compares 0.1 V units ) →
  `handleBatteryConnected ( )` once ( `battery.cpp:238-241` ).
- In it ( `battery.cpp:189-209` ), with config defaults `vBatMaxVoltage` 42, `vBatMinVoltage` 30,
  `vBatWarningVoltage` 32 ( 0.1 V units ), `BatteryCapacity` 600 ( `config/config.cpp:309-312`; `Rx_ELRS` forces
  800 at `API-Src/RxConfig.cpp:172` ):
  - `batteryMaxVoltage` = 4200 mV; critical = `batteryCellCount` x 3000 mV; warning = `batteryCellCount` x 3200 mV,
    computed **before** the cell count is updated ( initial value 1 ), so 3000 / 3200 mV on a normal boot.
  - `delay ( 40 )` blocks the loop; `vBatRaw` is not re-read after it.
  - Cells = `vBatRaw / 42 + 1`: **2 at 4.2 V**, 1 below. `batteryCellCount` only feeds FrSky telemetry
    ( `telemetry/frsky.c:376,391` ) and the thresholds on a later reconnect, so it does not move the mAh figure.
  - `Est = ( vBatRaw x 100 - 3000 ) x capacity / ( 4200 - 3000 )`: linear in voltage from **one** floored 0.1 V
    sample. Full pack at rest 4.15-4.20 V → 41-42 → **Est 550-600** ( 600 capacity ). A pack reading 4.3 V
    gives 650, more than the capacity. Below 3.0 V the int result is negative and wraps to ~65k in `uint16_t`.
- `handleBatteryDisconnected ( )` zeroes the cell count; a later reconnect without a power cycle ( USB-powered
  board ) computes critical = warning = 0 V and `Est = V x capacity / 4200` ( `battery.cpp:214-222` ).

### 3.4 Current path and the counter

| Stage | What the code does | Where |
|---|---|---|
| Average | `vShuntRaw` = 50-sample u16 ring average of `shunt_voltage ( )`. A negative sample ( <= -1 mV, reverse current >= 50 mA ) becomes ~65535 in the `uint16_t` parameter. | `battery.cpp:300` |
| Scale | `mAmpRaw = ( vShuntRaw / 10 ) / ( 0.02 / 10 )` = **vShuntRaw x 50**, 50 mA per step. Correct for the R020 shunt. | `battery.cpp:306` |
| Auto-gain | `gain_err = ( vShuntRaw - vBatRaw ) / ( mAmpRaw x 100 / 1000 )`: shunt **mV** minus bus voltage in **0.1 V**, over an expected sag in mV. At 5 A: ( 100 - 37 ) / 500 = 0.13, clamped to **0.95**. Learns when armed, `mAmpRaw` >= 1000 and `vShuntRaw > vBatRaw` ( I > ~1.9 A ); EMA alpha 0.003 per 21 ms call → time constant ~333 calls ≈ 7-8 s. The gain is a `static` never reset before power-off, so it also scales disarmed current once learned. | `battery.cpp:261-286`, `:95-97` |
| Output | `mAmpWithGain = mAmpRaw x gain` ( truncated to `uint16_t` ) | `battery.cpp:312-314` |
| dt | `dtMs = dtUs / 1000`, `last_ms = nowUs`: the sub-ms remainder of every interval is dropped. The 21 ms gate guarantees `dtUs >= 21000`. | `battery.cpp:341-346` |
| Integrate | `mA_ms_accum` is `uint64_t`; `mAhDrawn = accum / 3600000`. **No per-step flooring**: the accumulator is sound. | `battery.cpp:350-355`, `:116` |
| Remaining | `mAhRemain = ( uint16_t ) ( Est - mAhDrawn )`: pure coulomb count, never re-anchored to voltage; wraps to ~65k if drawn exceeds `Est` | `battery.cpp:357` |

**Reading `I` in a log** ( reviewer note, task 3 ): `I` moves in 50 mA steps ( whole shunt mV ); when the averaged shunt
is 0, `ProcessedINA219Current ( )` returns before assigning `mAmpRaw` ( `battery.cpp:302` ), so `I` **holds its last
value** instead of reading 0; a negative shunt sample becomes ~65535 in the u16 ring and pushes the average up.
The disarmed quiescent current in task 4 must be read with this in mind.

### 3.5 SoC, low battery and consumers

- `vBatComp = 0.35 x computeVbatComp_mV ( ) + 0.65 x ( vBatRaw x 100 + vShuntRaw )` ( `battery.cpp:631` ).
  In `computeVbatComp_mV ( )` the "fresh" checks compare microsecond timestamps against 200 ( `:392-393` ), so the
  current-sag term is only used on the loop in which the voltage was just serviced.
- SoC from mAh = `mAhRemain / capacity`, 100% when `mAhRemain >= capacity` ( so a wrapped ~65k reads 100% )
  ( `battery.cpp:427-442` ); SoC from voltage is linear between 2.9 V and 4.2 V ( `:454-462` ); fused with a
  20-75% mAh weight and a 2%-per-call drop limit ( `:480-551` ), which is meaningless because the call runs every
  loop.
- Low battery ( `battery.cpp:563-612` ): warning at fused SoC <= 18%, critical <= 8% ( `SOC_WARN_PCT` 20 /
  `SOC_CRIT_PCT` 10 minus 2% hysteresis ), and the OK → WARNING edge also requires `fsInFlightLowBattery`.
  Actions: beeper, `set_FSI` flags and `BatteryWarningMode` for the app. No auto-land and no arming block
  ( `DISABLE_ARMING_FLAG ( PREVENT_ARMING )` clears a *prevent* flag ).
- **What the app gets** ( `MSP_ANALOG`, `io/serial_msp.cpp:995-1002` ): `vBatComp` mV, `mAmpRaw` ( **before** the
  gain, so the app's current reads ~5% higher than what is counted ), `mAhDrawn`, `mAhRemain`, `soc_Fused`,
  `BatteryWarningMode`. The ~300 figure is `mAhRemain`.
- CRSF: `mAhDrawn` and `soc_Fused`; current sent as `mAmpRaw / 10` ( 10 mA units ) where the frame expects 0.1 A.
- `Bms_Get ( )`: `Voltage` = `vBatRaw x 100` mV, `Current` = `mAmpWithGain`, `mAh_Consumed`, `mAh_Remain`,
  `Battery_Capicity`, `Estimated_Capacity` ( `API-Src/BMS.cpp:26-50` ).

### 3.6 History: what `88f594f` changed

Before the rewrite ( `git show 88f594f^:src/main/sensors/battery.cpp` ): same x50 scale ( `/ 0.002` after `/ 10`,
`:202` ), same PGA /4, same plug-in formula ( `:242` ), one shunt sample per tick with no ring, no auto-gain,
`millis ( )` on both ends of dt so no remainder was lost ( `:140`, `:373` ). The rewrite therefore added about
-5% ( gain ) and 0 to -2% ( dt ) and one extra round-down. **Older firmware was at most ~6-8% closer**: with the
same flight it would have shown roughly 275-280 instead of ~300 remaining. The recollection is consistent, but
the rewrite does not explain the gap.

### 3.7 Loss budget ( expected firmware under-count vs the true charge, hover at 4-6 A )

| # | Loss | Size | Basis |
|---|---|---|---|
| L1 | Auto-gain pinned at 0.95 | **-5%** of all charge after the first ~8 s armed | `battery.cpp:276-283` |
| L2 | dt sub-ms remainder dropped | **0 to -2%** ( 0 when the 21 ms gate lands on a 6-loop boundary, ~-2% when it slips to the 7th loop, 500 of 24500 us ) | `battery.cpp:341-345`, `mw.cpp:119` |
| L3 | Two round-downs ( shunt to mV, then the ring average ) | ~-50 mA → **~-1%** at 5 A | `ina219.c:64`, `maths.cpp:381` |
| L4 | 8 A PGA ceiling on the *average* | **~0%** in hover; only hard climbs | `ina219.c:51` |
| | **Total** | **-6 to -8%** | |

Plus the plug-in estimate: **Est 550-600** for a full pack ( up to -50 mAh from flooring, which makes the
remaining figure *lower*, not higher ).

### 3.8 What the numbers imply

With Est 550-600 and a counter that reads 92-94% of the true charge, "300 remaining at 3.1 V" means the
firmware counted 250-300 mAh, so the pack **really delivered about 265-325 mAh** before reaching 3.1 V. For a
600 mAh pack that is 45-55% of its rating. The code losses above cannot produce a 2x gap. Either:

- the pack delivers about half its rating ( H2: worn or over-labelled ), which the charger readback will show as
  ~300-350 mAh put back; or
- something the code does not show under-reads the current by ~2x. The one mechanism found is **H6**: the INA219
  sees the *pulsed* battery current of the brushed-motor PWM ( TIM2, 20 kHz, `config/config.cpp:84` ) because the
  board capacitors are downstream of the shunt. If on-phase peaks exceed the +/-160 mV ( 8 A ) PGA range, the
  ADC saturates during the peaks and the average reads low even though the average current is under 8 A. At
  hover duty ~50% a 5 A average means ~10 A peaks. The charger would then put back ~500-600 mAh.

The charger readback in task 5 separates these two cleanly; the logged `I` vs throttle in task 6 gives a hint
for H6 ( a current that flattens as throttle rises ).

## 4. Hypotheses ( updated after log-1 and the charger readback, 25 Sep 2026 )

**Update after log-3 and the second look at the board:** the current under-read is a scale error, and its cause is found: **two R020 in parallel ( 10 mOhm ) where the code assumes 20 mOhm**, so the reading is exactly half ( H1 confirmed; H6 and H7 closed ). And an empty pack reads 54-58% SoC through the `mAhRemain` wrap ( H5b ).

**Result so far:** the 297 mAh is two errors of about the same size: `E` too high by ~160-180 mAh ( H2 + H4 ) and the
current reading ~36-40% low, ~115-140 mAh ( H6 or another measurement cause ). TESTING.md has the arithmetic.


| # | Hypothesis | For | Against | Test |
|---|---|---|---|---|
| H1 | **The shunt is not 20 mOhm: two R020 in parallel = 10 mOhm** ( user, 25 Sep, second look at the board: a second R020 stacked on the first ). The firmware's `x50` assumes 20 mOhm, so every current reads exactly half | log-3: INA219 / supply = 0.4-0.5 at DC, idle and full; flight ~0.6 within its assumptions; the user's memory of ~100 mA idle on the old BMS ( now 50 ) | - | **Confirmed twice**: log-3 ( two R020, reads half ) and log-5 ( one R020 removed, reads the supply within one 50 mA step ) |
| H2 | The pack delivers well under 600 mAh | **Confirmed in part ( charger, 25 Sep )**: ~450 mAh to 3.5 V resting, 75% of the label. With the linear voltage curve this puts `E` ~160-180 mAh too high at 4.0 V | Explains only about half of the 297 mAh | Task 5 from a full pack gives the capacity directly |
| H3 | Accumulated code losses since `88f594f`: auto-gain pinned at 0.95 ( -5% ), dt remainder dropped ( 0 to -2% ), two round-downs ( ~-1% ) | Confirmed in code ( INVESTIGATION 3.7 ); matches "closer before" the rewrite | Totals **-6 to -8%**, not 2x ( task 1 ) | Task 6: independent integral of logged `I` vs firmware `D` |
| H4 | The plug-in estimate is too high: it uses the labelled 600 mAh and a linear curve from one floored sample | **Confirmed together with H2**: `E` 500 at 4.0 V against ~320-340 mAh really left | For a full pack of true capacity `E` would be right | Task 4: `E` at other plug-in voltages; the fix needs a LiPo curve and the real capacity |
| H5 | Edge bugs: `mAhRemain` wraps to ~65k past `Est`; cell count 2 at 4.2 V; I2C error 0xFFFF into the bus average; `BMS_Update` runs every loop; CRSF current in 10 mA units where 0.1 A is expected | Confirmed in code | None produce a steady 300 | Listed for the fix topic; task 4 checks the cell count |
| H5b | **Wrap at empty makes the SoC jump up** ( measured ): at `E` <= `D` or plug-in <= 3.0 V, `mAhRemain` wraps, `soc_from_mAh ( )` returns 100% and the fused SoC reads 54-58% on an empty pack; the low-battery warning clears | log-3 at 3.0 and 2.9 V | Armed, `S` cannot rise, so it shows after landing and at plug-in | Confirmed; fix topic |
| H6 | ~~PGA /4 ( 8 A ) saturating on PWM peaks~~ **Ruled out as the main cause ( log-3, 25 Sep )**: the INA219 reads about half the current at DC too, at idle with the motors stopped and at 100% duty | - | - | log-3 bench supply |
| H7 | ~~Board current path ( bypass or sense routing )~~ Resolved: it is H1, the parallel R020 | - | - | - |

## 5. Q&A that shaped this

| Question | Answer ( user, 25 Sep 2026 ) |
|---|---|
| Mode | `check` first; fix afterwards |
| Target | PRIMUS_X2_v1 |
| Done when | Estimate matches the charger within +/-10% ( 5% stretch ) - assigned to the fix topic; this topic ends with a cause and evidence |
| Out of scope | Hardware change; app / MSP layout change |
| Where seen | Pluto app battery widget |
| Pack | Stock Pluto 1S, 600 mAh |
| Evidence | App display only, so far |
| Rx mode | Both `Rx_ESP` and `Rx_ELRS` |
| 3.1 V when | Under load and at rest after landing, same figure |
| Bench gear | Charger with mAh readback only |
| History | "Closer before 278f77d" ( see assumption in TASKS.md decisions log ) |
| Suspicions | Current reading too low; starting estimate too high; pack worn |
| Proof wanted | Discharge log vs charger, plus the shunt marking |
| Test run | User flies with PlutoMonitor logging; user reads the shunt marking; temporary `Monitor_Print` line allowed if needed ( it is: PlutoMonitor does not record MSP battery fields ) |

## 6. Findings

( Task 7. )
