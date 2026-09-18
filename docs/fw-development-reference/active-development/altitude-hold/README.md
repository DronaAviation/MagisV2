# Altitude Hold

[README](README.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

| | |
|---|---|
| **Status** | **Closed** in `4657cbf` ( FW 3.7.0, 18 Sep 2026 ). Follow-up tests remain under Open items. |
| **Branch** | `BugFix-June26`, base `f60c9ad` (20 Aug 2026) |
| **Target** | `PRIMUS_V5` from 17 Sep 2026 (August work on `PRIMUS_X2_v1`; same barometer chain) |
| **Last updated** | 18 Sep 2026 ( commit preparation ) |
| **Pipeline doc** | [Altitude_Hold_Estimator.md](../../fw-architecture-pipeline/subsystems/Altitude_Hold_Estimator.md) updated from [PIPELINE_UPDATE.md](PIPELINE_UPDATE.md) at commit |

**Problem:** altitude hold sank while the log reported a steady altitude. The
barometer reading moved during flight and the controller followed it down.

**Outcome so far:** four firmware bugs fixed, throttle and temperature effects on
the barometer compensated. A 4 minute cold-start hover now holds within about
±8 cm with no steady drift, checked against a laser rangefinder.

| Document | Answers |
|---|---|
| [INVESTIGATION.md](INVESTIGATION.md) | **Why**: the problem, root causes, how the August open question was answered. Written for a supervisor. |
| [CHANGES.md](CHANGES.md) | **What and how**: every code change with file and line, and the file index. |
| [TESTING.md](TESTING.md) | **Evidence**: coefficient measurements, laser flight results, log format, how to read the logs. |
| [PIPELINE_UPDATE.md](PIPELINE_UPDATE.md) | The pipeline doc as it should read once this is committed. |

## Where things stand

**Altitude hold works on PRIMUS_V5.** Over a 4 minute hover from a cold board,
checked against a VL53L0X laser as ground truth, the craft holds within about
+/- 8 cm with no steady drift. Before this work the same kind of hover lost up
to 100 cm in 2 minutes while the log reported a steady altitude.

The single biggest remaining error term from the August investigation, a
"1.67 Pa/min unattributed drift", turned out to be **barometer temperature
sensitivity hidden by whole-degree logging**, not room pressure. It is now
corrected ([TESTING.md](TESTING.md)).

| Item | Status |
|---|---|
| In-flight ground re-zero on throttle chop | Fixed |
| Estimator stuck in slow mode ( deci-degree bug ) | Fixed |
| ICP-10111 temperature frozen at power-on | Fixed |
| ICP-10111 calibration constants read unsigned | Fixed |
| Position-error deadband halving loop authority | Fixed |
| Hover trim trapped in a clamped integrator | Fixed ( trim offload ) |
| Throttle effect on sensed pressure | Compensated, coefficient not re-verified |
| **Temperature effect on sensed pressure** | **Compensated, coefficient re-measured 7 times ( Sep )** |
| "Unattributed" 1.67 Pa/min drift | **Explained: thermal, not ambient ( strong evidence, [TESTING.md](TESTING.md) )** |
| Second airframe validation | Not done |
| Diagnostic log in `PlutoPilot.cpp` | Kept out of the commit ( working copy only ); the committed `PlutoPilot.cpp` is the stock template |

## Build and environment

- Toolchain: `arm-none-eabi-gcc` 14.2.1 (Arm GNU Toolchain 14.2.Rel1), installed
  by the **PlutoIDE VS Code extension**.
  - Windows: `C:\PlutoIDE\tools\ARM GNU ToolChain\bin`. PlutoIDE only puts it on
    `PATH` inside its own terminal, so a fresh Git Bash needs
    `export PATH="/c/PlutoIDE/tools/ARM GNU ToolChain/bin:$PATH"` first.
  - Linux / macOS: `~/.pluto-ide/tools/ARM GNU ToolChain/bin`.
- Build: `.claude/skills/run-magisv2/driver.sh PRIMUS_V5` (clean build, checks
  the `.hex`, prints memory), or `make TARGET=PRIMUS_V5`.
- Flashing is done through PlutoIDE (STM32 DFU / bootloader). **After a DFU
  flash, remove all power ( battery and USB ) before booting**: the MCU otherwise
  stays in the bootloader, which looks exactly like a dead board ( no LEDs, app
  not connecting ).
- `src/test/` GoogleTest does not build and is not part of any workflow. The
  build is the verification path.

**Build state, 17 Sep 2026:** `PRIMUS_V5` compiles with no warnings under the
strict flag set. Flash 101.3 KB / 256 KB, RAM 14.8 KB / 40 KB ( `LASER_TOF` off ). Build all three
targets before committing.

> **Flashing note:** `EEPROM_CONF_VERSION` is 107 ( was 106 ). The first boot
> after flashing resets saved settings to defaults. Re-do any bench trims.

## Gotchas

**`Monitor_Print` silently corrupts long lines.** It writes into the MSP UART's
256-byte TX ring buffer, and `uartWrite ( )` ( [serial_uart.c:301](../../../../src/main/drivers/serial_uart.c#L301) )
does not check for full: new bytes overwrite the oldest unsent ones. A ~270 byte
line per 100 ms tick lost its first fields almost every time. Keep a tick under
~250 bytes.

**The BMP280 is not a usable reference on this board.** Mounted on the FC it
shares the board's heat path, has a +2 to +13 Pa/degC temperature coefficient with
a ~4 s lag, and sits in rotor wash. It answered the ambient question only by
contrast with the ICP. A future dual-baro test needs it off-board, in foam, out of
the airflow.

**STM32F3 internal temperature sensor has a negative slope.** `TS_CAL2` ( 110 degC )
is *smaller* than `TS_CAL1` ( 30 degC ). Also, ADC1 has no clock out of reset
( `RCC_ADC12PLLCLK_OFF` ), so calibrating before setting a clock never finishes.
Both bit the diagnostic driver; both matter to anyone using the ADC directly.

**`LASER_ALT` is a hard switch, not a fusion.** Below 200 cm with a valid reading
the barometer contributes nothing, so a build with it on does not test the
barometer path. `LASER_TOF` alone is logging-only. No hysteresis at the 200 cm
boundary.

**The `baro_offset` handover** in `checkReading ( )` aligns the barometer to the
last laser reading on dropout. Well designed, leave it alone.

**`baroSetCalibrationCycles ( )` is inert.** The stick command to re-zero the
barometer does nothing. Low priority.

**Re-calibrating in flight is dangerous.** Ground zeroing removes a constant; the
errors here are slopes, and re-zeroing in the air is bug 1.

**Coefficients are from one airframe each.** The throttle coefficient depends on
where the FC sits relative to the rotors.

**`initialThrottleHold` survives a disarm.** Confirmed in a log: `Arm:0`, `AltH:1`,
`Base:1682`. The next arm starts from the previous flight's trim. Harmless so far,
but a stale value from a different battery changes the takeoff.

## Open items and ideas

1. **`BaroAlt` noise** ( ~+/- 8 cm on the ground, ~2x in flight ). `EstAlt`, which
   the controller uses, is already heavily filtered. Options, best first:
   - open-cell foam over the ICP-10111 port ( only fix for rotor-wash turbulence );
   - ICP `ACCURATE` ( low-noise ) mode instead of `NORMAL`: ~0.8 Pa vs ~1.6 Pa
     noise, 24 ms conversion ( ~40 Hz ). Three `measurment_start ( NORMAL )` calls
     in [barometer.cpp](../../../../src/main/sensors/barometer.cpp) ( lines 100, 258, 265 );
   - a light software low-pass, only after the above, because estimator lag
     caused the original runaway.
2. **Post-takeoff dip** of 10 to 13 cm about 30 s after arm, on several flights.
   Log `EstAlt` and throttle to see whether it is estimator settling or the
   throttle-compensation reference being latched at idle throttle on arm.
3. **Throttle coefficient** never re-verified. Needs throttle in the log.
4. **Second airframe:** one 3+ minute cold-start hover with `degC` + `PaI` logged
   gives the temperature coefficient for that frame.
5. **Outdoor / full-battery flights** not tested. Wind and sun are not covered by
   this correction.
6. **Clamp headroom on a long cold start** ( [TESTING.md](TESTING.md) ): a 4 min cold hover used
   71 % of `BARO_COMP_LIMIT_PA`. A full-battery cold start is the test; watch for
   a sink starting late in the flight.

## Next actions, in order

1. ~~Build all three targets and commit~~ - done at FW 3.7.0: all three targets
   build clean ( PRIMUS_X2_v1 97.7 KB / 14.8 KB, PRIMUSX2 98.2 / 14.9, PRIMUS_V5
   97.7 / 14.8 ); pipeline doc and changelog updated.
2. Full-battery cold-start hover with laser logging ( open items 4 and 6 ),
   ideally on a second airframe too. Re-add the diagnostic log locally for it.
3. Decide on the `BaroAlt` noise items and the post-takeoff dip ( open items 1 and 2 ).

## Logs and history

The September flight logs ( `logs.txt` 16:05 coefficient 2.2, `logs2.txt` 15:18
coefficient 2.2, `logs3.txt` 16:18 cold start coefficient 2.1 ) were working files
and are not kept in the repository; their results are recorded in
[TESTING.md](TESTING.md).

The August 62k-line bench log was overwritten on 17 Sep. A copy survives inside
`stash@{1}` and `stash@{2}` ( "altitude hold work", "AltHold current work" ), which
are otherwise duplicates of the August working tree and can be dropped once the
work is committed. `stash@{0}` ( "MSP Failsafe" ) is unrelated work, keep it.

August reasoning history is in the development-session transcripts from the Linux machine
( `~/.claude/projects/-home-mecash-DronaAviaiton-MagisV2/19368ed1-...jsonl` ).

