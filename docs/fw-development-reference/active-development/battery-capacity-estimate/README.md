# Battery Capacity Estimate ( BMS check )

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md)

| | |
|---|---|
| **Status** | **Closed** 26 Sep 2026: superseded by the fix topic [battery-soc-fix](../battery-soc-fix/README.md); committed with it |
| **Mode** | `check` ( investigation only; the fix follows with `/pluto-grill fix battery-capacity-estimate` ) |
| **Branch** | `BugFix-June26` |
| **Target** | `PRIMUS_X2_v1` ( INA219 current sensor on I2C1; `PRIMUS_V5` has the same sensor ) |
| **Origin** | User report: the app shows ~300 mAh remaining when the stock 600 mAh 1S pack is at 3.1 V, and the figure does not change after landing and resting |

**Problem.** The BMS coulomb counter ( `updateINA219Current ( )` in
[battery.cpp](../../../../src/main/sensors/battery.cpp) ) reports about half the charge the pack actually
delivers. On both `Rx_ESP` ( capacity 600 ) and `Rx_ELRS` ( capacity forced to 800 ) the Pluto app battery
widget ends a flight at ~300 mAh remaining while the cell is at 3.1 V resting, i.e. empty. The user recalls the
figure being closer on older firmware. The remaining figure is `Est - mAhDrawn`: pure coulomb counting from the
INA219 shunt voltage, never corrected from the cell voltage.

**What the code already tells us** ( details in [SCOUT.md](SCOUT.md) and [INVESTIGATION.md](INVESTIGATION.md) ).

- Current is `shunt mV x 50`, which assumes a 20 mOhm shunt ( `INA219_SHUNT_RESISTOR` 0.02 ). The INA219 is
  configured for +/-160 mV ( 8 A full scale ), 12-bit, one sample, and its calibration register is never used.
- The counter has known losses since the BMS rewrite `88f594f`: an auto-gain that always converges to its 0.95
  clamp because it compares mixed units, `dtMs = dtUs / 1000` truncation ( 0 to -2% ), two round-downs ( ~-50 mA ),
  a negative shunt sample read as 65535. Together -6 to -8% ( task 1 budget ): real, but not the ~2x seen.
- `278f77d` ( the commit the user remembers ) changed only the auto-gain rate 0.002 to 0.003; the current scale
  was numerically identical before and after.
- Low battery is a fused SoC threshold ( warning 18%, critical 8% ) that only drives beeper, LED and the app
  flag; no auto-land, arming not blocked. `BMS_Update ( )` runs every loop because its rate check compares
  against a constant.

**Ranked hypotheses** ( after the task 1 audit ). The code under-counts by only **-6 to -8%** ( auto-gain pinned
at 0.95, dt remainder dropped, two round-downs; the uint64 integrator itself is sound ), and a full pack starts at
Est 550-600. So "300 remaining at 3.1 V" means the pack really delivered only ~265-325 mAh. Two explanations
remain: H2 the pack delivers about half its rating; H6 the INA219's +/-160 mV ( 8 A ) range saturates on the
pulsed 20 kHz motor current peaks, so the average reads ~2x low. H1 ( shunt value or path ) is ruled out: R020,
between the battery and the entire circuit. H4 ( plug-in estimate ) and H5 ( edge bugs: wrap, cell count 2 at
4.2 V, 0xFFFF samples, CRSF unit, `BMS_Update` every loop ) go to the fix topic.

**How it is settled.** The shunt marking on the board, and one full hover-to-empty discharge logged through a
temporary `Monitor_Print` line ( PlutoMonitor does not record the MSP battery fields ), compared with the
charger's recharge mAh. The ratio `mAhDrawn / charger mAh` and an independent integral of the logged current
separate H2 from H6 in one test: ~300-350 mAh put back means the pack, ~500-600 means
the measurement ( then task 10 flies a PGA /8 diagnostic build to confirm H6 ).

**Out of scope.** Hardware changes ( no new shunt or sensor ), the `MSP_ANALOG` layout the app parses, and any
fix: the accuracy target ( +/-10%, 5% stretch, against the charger ) belongs to the follow-on `fix` topic.

**Open items.**

- **Temporary Developer Mode gating change in `mw.cpp`** ( task 11: AUX switch as before, user code survives RC frame
  gaps under 400 ms ): remove before release ( task 9 ).
- Bench motor sequence in `PlutoPilot.cpp` ( `BENCH_MOTOR_SEQUENCE` ): **off** since task 13 ( 20:35 build ); the block
  is removed in task 9.
- **Temporary diagnostic in `PlutoPilot.cpp`** ( task 3, battery log line and `extern "C"` block ): remove before
  release ( task 9 ), or hand it to the fix topic. Fields in [CHANGES.md](CHANGES.md).

- Shunt: **two R020 in parallel = 10 mOhm**, between the battery and the entire circuit ( user, 25 Sep 2026 ). The
  code's x50 scale is for 20 mOhm: the reading is half. Open: factory-fitted on every board, or a rework on this one?
- **Go-ahead for task 10** ( temporary PGA /8 build ): needed to find why the current reads ~60-64% of true.
- Whether the ~300 figure was seen on more than one pack ( user not sure; decides how much weight H2 carries ).
- Not this topic: the user code restarts every ~3 s on Wi-Fi ( one late RC frame > 200 ms ), see TESTING.md *Link
  timing*; rule and reference numbers now in `pluto-flighttest`.
- `Power_BMS_Pipeline.md` describes a different BMS from the one in the code ( function names, units, thresholds,
  2S/3S, failsafe ); the correction is staged in this topic's `PIPELINE_UPDATE.md`.

**Latest ( log-1, 25 Sep 2026 ).** Symptom reproduced: `E` 500 ( pack at 4.0 V ) - `D` 203 = 297 remaining on an
empty pack. The counter matches an independent integral of its own current within 1.4%, and hover current reads
flat at ~2.25 A. The charger put back 430 mAh ( one PE1206FRE470R02L shunt, 0.02 Ohm, no parallel path ). So the
297 mAh is **two errors of about equal size**: the plug-in estimate is ~160-180 mAh too high ( 600 mAh label vs
~450 real, linear curve ) and the current reads ~60-64% of true ( ~115-140 mAh under-counted ). See [TESTING.md](TESTING.md).

**Cause found ( 25 Sep, evening ).** The board carries **two R020 in parallel ( 10 mOhm )**; the firmware converts the
shunt voltage as 20 mOhm, so every current reading, and the mAh count, is exactly **half**. log-3 on the bench supply
showed the half at DC ( idle and full motors ) before the second resistor was spotted.
And on an empty pack ( <= 3.0 V ) the `mAhRemain` wrap makes the SoC jump to **54-58%** and the low-battery warning
clears ( safety, H5b ).

**Confirmed ( log-5 ).** With the second R020 removed the reading tracks the supply within one 50 mA step.

**log-6 ( full pack, one R020 ).** App remaining at empty **22 mAh** ( was ~300 ), hover current 4.15-4.4 A. The
warning came ~15 s before the end ( 3.0 V under load ). Charger: **534 mAh**; the firmware counted 528 ( 0.99 ).

**Next action.** Task 6 ( analysis, largely written ) and task 7 ( findings and the fix recommendation ). Task 11 ( user code survives short link drops; Dev switch
kept ) is in and confirmed by log-2: no restarts on Wi-Fi.

**Closed ( 26 Sep 2026 ).** Findings in [TESTING.md](TESTING.md) *Task 6*: the ~297 mAh was mostly a stacked
second R020 on this board ( 10 mOhm read as 20 ); the rest is the plug-in estimate. With one R020 ( production ) the
count matched the charger within 1% ( log-6, 528 / 534 ), with a −5% auto-gain cancelling a +4% reading. Open safety
items: the empty-pack SoC jump and the late warning. The fix is planned in
[battery-soc-fix](../battery-soc-fix/README.md).
