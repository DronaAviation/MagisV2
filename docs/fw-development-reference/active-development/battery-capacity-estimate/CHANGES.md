# Battery Capacity Estimate: Changes

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md)

This is a `check` topic: the only code change is a temporary diagnostic, removed in task 9 ( or handed to the fix
topic ).

## Task 3: battery diagnostic log ( 25 Sep 2026 )

- [PlutoPilot.cpp](../../../../PlutoPilot.cpp): `extern "C"` declarations of `mAmpRaw`, `vBatComp`,
  `_ina219_current_gain`, `soc_Fused` ( `sensors/battery.cpp`, C linkage through `battery.h` ) and `motor []`
  ( `flight/mixer.h` ). `onLoopStart ( )` sets a flag; the first `plutoLoop ( )` tick prints `E` and `Cap` alone ( ~30 B ) and
  returns, because `userCode ( )` runs `onLoopStart ( )` and the first `plutoLoop ( )` in the same 100 ms slot and the two
  lines together would be 144-153 B ( reviewer finding, fixed ). Every later tick prints the data line at 10 Hz:
  **114 B** realistic ( `t` 7 digits, values 4 digits ), 121 B type-bounded worst, under the 130 B ceiling.
  **Remove before release** ( task 9 ).

  | Tag | Source | Unit | Why |
  |---|---|---|---|
  | `E` | `Bms_Get ( Estimated_Capacity )`, once | mAh | H4: the plug-in estimate |
  | `Cap` | `Bms_Get ( Battery_Capicity )`, once | mAh | 600 ( `Rx_ESP` ) or 800 ( `Rx_ELRS` ), or an app-written value |
  | `t` | `millis ( )` | ms | Real time base for the independent current integral ( task 6 ) |
  | `V` | `Bms_Get ( Voltage )` = `vBatRaw x 100` | mV, 0.1 V steps | Bus voltage |
  | `Vc` | `vBatComp` | mV | Sag-compensated voltage ( fix topic: voltage anchoring ) |
  | `I` | `mAmpRaw` = shunt mV x 50 | mA | Current before the auto-gain ( what the app shows ) |
  | `G` | `_ina219_current_gain x 1000` | - | L1: expected to settle at 950 within ~8 s armed |
  | `D` | `Bms_Get ( mAh_Consumed )` = `mAhDrawn` | mAh | The firmware count; compared with the charger |
  | `S` | `soc_Fused` | % | Fused SoC driving the warnings |
  | `M` | mean of `motor [ 0..3 ]` | us | H6: does `I` flatten as the motor command rises |
  | `Arm` | `FlightStatus_Check ( FS_ARMED )` | 0 / 1 | Segments |

  `mAhRemain` ( the app's "remaining" ) is `E - D` and is not logged separately.

## Task 13: bench motor sequence switched off ( 25 Sep 2026 )

- [PlutoPilot.cpp](../../../../PlutoPilot.cpp): `BENCH_MOTOR_SEQUENCE` 0; the `#if` now also encloses the sequence's
  statics and functions, so nothing is left unused. Gate clean, no `PlutoPilot.cpp` warnings, no bench symbols in
  the ELF. `Ph` logs -1. The log line and the task 11 grace stay for task 5.

## Task 12: bench motor sequence ( 25 Sep 2026, TEMPORARY, PROPS OFF )

- [PlutoPilot.cpp](../../../../PlutoPilot.cpp): `BENCH_MOTOR_SEQUENCE` ( 1 ), `BENCH_STEP_S` ( 10 ),
  `benchLevels [] = { 1000, 1250, 1500, 1750, 2000, 1000 }`. `onLoopStart ( )` arms the sequence if the drone is
  disarmed; each `plutoLoop ( )` tick calls `benchMotorsTick ( )`, which writes the level into `motor_disarmed [ 0..3 ]`
  ( the mixer copies it to the motors while disarmed, `flight/mixer.cpp:829-833`, the same path as the
  configurator's motor test ) and advances every `BENCH_STEP_S`. It writes 1000 and stops if the drone is armed,
  when the sequence ends, and in `onLoopFinish ( )` ( Dev Mode off or link lost ). New log field `Ph`: -1 idle /
  not running, 0-5 the level index. Requested by the user so the supply readings are taken at repeatable levels.
- **`BENCH_MOTOR_SEQUENCE` must be set to 0 before any flight test ( task 5 )**: with it on, switching Dev Mode on
  while disarmed spins the motors. Removed in task 9.
- Byte budget ( reviewer ): ~123 B per tick realistic, ~126 B plausible worst; inside the 115-135 B band that ran
  clean. Drop `Vc` first if the app disconnects on the bench.
- Review: no blocking. Applied: `#warning` while the macro is on, level rewritten every tick, task 9 removes the
  block and checks `PlutoPilot.cpp`, task 12 ends by setting the macro to 0. Known: a link drop over 400 ms re-runs
  the sweep on reconnect; `M` lags `Ph` by one row.

## Task 4: `Cells` on the first line ( 25 Sep 2026 )

- [PlutoPilot.cpp](../../../../PlutoPilot.cpp): `extern uint8_t batteryCellCount;` and the one-time line is now
  `E Cap Cells` ( ~41 B, alone on its tick ). Gate clean on PRIMUS_X2_v1, no `PlutoPilot.cpp` warnings. Removed in
  task 9 with the rest of the diagnostic.

## Task 11: Developer Mode survives short link drops, 400 ms grace ( 25 Sep 2026, TEMPORARY )

- [mw.cpp](../../../../src/main/mw.cpp) `userCode ( )`: `DEV_MODE_LINK_GRACE` ( 1 ) and `DEV_MODE_LINK_GRACE_US`
  ( 400000 ) above the function. With the flag on, the Dev AUX switch starts and stops user code as before, and
  user code keeps running while `rxIsReceivingSignal ( ) || ppmIsRecievingSignal ( )` was true less than 400 ms ago
  ( the switch value is held through a gap, so it reads unchanged ). **Amendment, 18:13:** the first version ran
  user code from the first RC frame with no switch ( `DEV_MODE_ALWAYS_ON` ), which sent the one-time `E Cap Cells`
  line before PlutoMonitor was listening; the switch is back so the user starts user code after the monitor is up. The `#else` branch is the committed condition, unchanged. `devmode` ( OLED "DEV" ) follows the same flag;
  the teardown on a real loss ( overrides cleared, `onLoopFinish ( )` ) is unchanged. **Remove before release**
  ( task 9 ).
- Why 400 ms ( timeline confirmed by the reviewer ): after the last MSP frame the signal flag drops at 200 ms
  ( `rx.cpp:369` ); until then the retained frame re-validates the channels at 50 Hz ( `rx/msp.c:37-41`,
  `rx.cpp:535-537` ), so they stay held until ~780-800 ms ( `MAX_INVALID_PULS_TIME` 600, `rx.cpp:87` ); armed failsafe
  commands LAND when the link is declared down 200 ms later, ~1 s ( `failsafe.cpp:211-216`, `:241-248` ). User code
  stops at ~600 ms. `failsafe_delay` only sets `rxDataFailurePeriod`, which nothing reads. PPM / CRSF: signal flag
  100 ms, user code stops ~500 ms, LAND ~900 ms.
- Review ( `pluto-reviewer` ): no blocking. Fixed: `devLinkSeen` is cleared once the grace expires, so a `micros ( )`
  wrap after 71 min without a link cannot re-enable user code for 400 ms. Noted in TASKS.md task 11: a throttle
  override active during a gap would feed back through the held `rcData` into `rcDataPilot` ( not reachable here );
  ( first version only ) the pilot could not switch user code off; the amendment restores the switch.
- Side effects: `isLocalisationOn` follows
  `runUserCode` under `OPTIC_FLOW` ( `mw.cpp:~1420` ); `OPTIC_FLOW` is off on PRIMUS_X2_v1 and PRIMUS_V5, but enabling
  it with this flag on would switch localisation on permanently.
- Banner header not updated: the block is temporary and reverted in task 9.

**Build ( `PRIMUS_X2_v1`, gate ).** 99.7 KB flash, 14.8 KB RAM, no new warnings under `src/`.

## Task 3 review

**Review ( `pluto-reviewer` ).** No blocking findings. Should-fix applied: first-tick overrun ( above ). Nits:
doc sync ( done ), and caveats on reading `I` ( recorded in INVESTIGATION.md §3.4 ).

**Build ( `PRIMUS_X2_v1`, gate, after the fix ).** 99.7 KB flash, 14.8 KB RAM. No new warnings under `src/`; `PlutoPilot.cpp`
has none ( checked in `Build/PRIMUS_X2_v1/build.log`, since the gate does not scan the repo root ).
