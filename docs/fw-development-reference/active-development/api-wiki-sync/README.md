# API wiki sync ( GitHub wiki )

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [FINDINGS](FINDINGS.md) · [CHANGES](CHANGES.md) · [CONVENTIONS](CONVENTIONS.md)

| | |
|---|---|
| **Status** | **Closed - pending commit** ( 16 Nov 2026; wiki repo committed separately ) |
| **Mode** | `docs` |
| **Branch** | `BugFix-June26` |
| **Target** | `PRIMUS_X2_v1` ( used only to compile the wiki examples ) |
| **Last updated** | 16 Nov 2026 |
| **Pipeline docs** | none. This topic edits [`PIN_MAP.md`](../../PIN_MAP.md), [`TIMER_MAP.md`](../../TIMER_MAP.md), [`DMA_MAP.md`](../../DMA_MAP.md), [`WS2812_RGB.md`](../../WS2812_RGB.md) and the separate wiki repo `MagisV2.wiki/`. |

**Problem.** The public GitHub wiki ( `MagisV2.wiki/`, a separate git repo cloned into the tree, last commit
`47e451d` on 21 Jul 2026 ) documents the API as it was before 1.4.0. The firmware is at FW 3.15.12 / API 1.8.10,
after about 25 API changes: the fuel gauge ( `Bms_Get` SoC / Warning_Level / Resistance ), the single VL53L0X
pipeline behind `XRanging.initAlt ( )`, the new `X1Ranging` VL53L1X API, relative yaw and heading hold, the
take-off / landing guards, the `FlightStatus_Get` enum map, `RcCommand_Get`, `Estimate_Get ( Rate )`, the baro
temperature scale, the PID profile default fix, `setDesiredAngle` roll / pitch, `setDesiredRate` hold, the
`Failsafe_enable` fix and the `Monitor_Print` precision fix. Two scouts cross-checked all 18 pages against
`src/main/API/*.h` and `src/main/API-Src/`: every page drifts. Nine pages are high severity, including examples
that are unsafe to run ( `Command_Land` / `Command_DisArm` in `onLoopFinish`, which re-runs after every Wi-Fi
hiccup, so the drone lands or drops with the motors stopped ), examples that silently do nothing ( the RxConfig
page configures channels in a mode where `AuxChangeEnable` is false ), examples that write to the onboard flight
IMU ( the Serial-IO I2C examples use address 0x68 ), and units that are wrong by a factor of 10 ( the XRanging
page says mm, the code is cm ).

**Plan.** Rewrite or correct every page against the headers and the implementation, keeping the existing wiki page
shape and adding a Behaviour / Notes section per page for the facts that now live in `docs/API/*_API_WIKI.md`. Add
a page for `X1Ranging.h`, remove the `API-Utils` page ( an internal header: not included by `PlutoPilot.h`, so its
example cannot compile from user code ) and the dead `Specifiers` links ( `Specifiers.h` was deleted in `2a93948` ).
Every example is compiled in a scratch `PlutoPilot.cpp` on `PRIMUS_X2_v1` before the page is finished. No firmware
or header change: the defects the cross-check found are recorded in [FINDINGS.md](FINDINGS.md) for separate topics.
Tasks: [TASKS.md](TASKS.md).

## Gotchas

- **The wiki is a different git repo.** `MagisV2.wiki/.git` has its own remote
  ( `DronaAviation/MagisV2.wiki` ). Firmware commits never carry wiki changes; the user commits in both repos.
  The clone stays untracked in the firmware repo and is not added to `.gitignore` or `.graphifyignore`
  ( user's decision, 15 Nov 2026 ).
- **GitHub wiki links are page names, not file paths.** A page cannot link to `docs/API/...` in the firmware repo,
  so the pages must be self-contained. `docs/API/*_API_WIKI.md` stays as the in-repo source that topics keep writing.
- **`PlutoPilot.h` is at the repo root**, not in `src/main/API/`. It does not include `API-Utils.h`,
  `Localisation.h` or `io/oled_display.h`, so any example using those needs an explicit include.
- **Hook placement is not cosmetic.** `onLoopStart ( )` and `onLoopFinish ( )` re-run after a Wi-Fi stall
  ( `mw.cpp:1176`, `:1194`, `:1230` ), so an arm, take-off, land, disarm or `Scheduler_Add` placed there repeats in flight.
- **Six documented functions do not work in 3.15.12** ( `SPI_Init ( )` with no arguments does not link,
  `Motor_SetDir ( M5..M8 )` writes through a NULL GPIO pointer, `Set_LED ( STATUS, TOGGLE )` does nothing,
  `RcData_Get ( void )` returns NULL, the `Uart_*` calls dereference NULL in ELRS mode or without `Uart_Init`,
  and BlackBox is commented out in every `target.h` ). They are marked "not available in 3.15" on their pages and
  logged in [FINDINGS.md](FINDINGS.md). The testing team has not reported these, so they are code findings from
  this cross-check, not confirmed field failures.

## Open items

- Task 1 done ( 15 Nov 2026 ): [CONVENTIONS.md](CONVENTIONS.md) fixes the page shape, the markers and the hook rules;
  [wiki_check.py](wiki_check.py) compiles and links every wiki example against the built firmware without touching
  `PlutoPilot.cpp`. Today's wiki: 168 example blocks, 55 fail ( 7 compile, 1 link, 8 warning, 39 outside any hook ).
  The link failure was a new defect, F9: `Oled_Print ( )` could not link from user code. It was fixed straight away
  in its own topic [oled-print-linkage](../oled-print-linkage/README.md) ( FW 3.15.12, API 1.8.10 ), so the Oled page
  documents it as working. Also found: `plutoRxConfig ( )` re-runs on every settings save, not only at start-up.
  Details: [CHANGES.md](CHANGES.md).
- Task 2 done ( 15 Nov 2026 ): `PlutoPilot.md` rewritten. It now says when each hook really runs: `plutoRxConfig ( )`
  also on every settings save, `onLoopStart ( )` / `onLoopFinish ( )` also on every RC link stall ( a frame 200 ms late
  on Wi-Fi, 100 ms on ELRS / PPM ), `plutoLoop ( )` every 100 ms. Flight commands moved out of those two hooks into the
  validated once-per-ARM pattern. 8 / 8 blocks pass. Evidence: [CHANGES.md](CHANGES.md).
- Task 3 done ( 15 Nov 2026 ): `FC-Control.md` rewritten. The five unsafe examples are gone ( land / disarm in
  `onLoopFinish`, arm / take-off in `onLoopStart`, a flip on the floor ). The page has the status priority table, the
  full take-off, landing and flip contracts, and the fact that `FlightMode_Set` overrides the app's switch and is
  kept until power-off. 11 / 11 blocks pass. Also fixed: `setRelativeAltitude` was named as a user call; it is
  `DesiredPosition_setRelative`. Evidence: [CHANGES.md](CHANGES.md).
- Task 4 done ( 15 Nov 2026 ): `FC-Config.md` rewritten. It now gives the loop period in ms, the pitch sign the right way
  round ( positive is nose down, forward ), the tilt and heading rules, the missing `DesiredAngle_setRelative`, the
  stick rates as settings rather than rotations, an altitude target that is flown as a goal with **no height limit**,
  and the failsafe switches. 15 / 15 blocks pass. `wiki_check.py` now checks each block at global scope, which also
  catches name collisions with firmware globals. Evidence: [CHANGES.md](CHANGES.md).
- Task 5 done ( 15 Nov 2026 ): `FC-Data.md` and `BMS.md` rewritten. Every sensor and estimate value now has a unit,
  range and sign; the BMS page has the three fuel-gauge options, the level rules and the auto-land. 9 / 9 blocks
  pass. New finding F10: `Sensor_Get ( Gyroscope | Magnetometer, Net_Acc )` returns an undefined value.
  Evidence: [CHANGES.md](CHANGES.md).
- Task 6 done ( 15 Nov 2026 ): `XRanging.md` rewritten. Thresholds and the avoidance distance are cm, `getRange ( )` is
  mm, and the page now says so up front; `initAlt`, its four usable pins and refusal rules, `getStatus`, the
  pin table and the wiring rules are in. Found: object avoidance acts only while Developer Mode runs, and
  `isTriggered ( )` is always false for a laser enabled only by `initObjectAvoidance ( )`. 10 / 10 blocks pass.
  Evidence: [CHANGES.md](CHANGES.md).
- Task 7 done ( 15 Nov 2026 ): new `X1Ranging.md` for the VL53L1X lasers, written as a pair with the XRanging page:
  family choice, pins, timing, the 190 / 170 cm handover with its return guard, every function, and the reach
  measured in flight ( all valid to 180 cm, almost none above 210 cm ). The XRanging page gained three
  object-avoidance facts from the shared core. 10 / 10 blocks pass. Evidence: [CHANGES.md](CHANGES.md).
- Task 8 done ( 15 Nov 2026 ): `RC-Interface.md` and `RxConfig.md` rewritten. RxConfig now has a channel-map table for
  every receiver mode and says plainly that channel configuration works only with PPM and ELRS ( the old example
  configured ESP mode and did nothing ). RC-Interface has the real `RcCommand_Get` ranges and signs and the full
  override rules. Found: `Rx_CAM` switches the ESP Wi-Fi off, and `Rx_ELRS` forces the battery capacity to 800 mAh
  on every settings save. 13 / 13 blocks pass. Evidence: [CHANGES.md](CHANGES.md).
- Task 9 done ( 15 Nov 2026 ): `Serial-IO.md` rewritten. The I2C examples no longer write to the flight IMU at 0x68: they
  use an external TMP102, and the page lists every onboard I2C address. SPI now has its speed codes ( anything else
  runs at 18 MHz ), the chip-select set-up `SPI_Init` skips, and its fixed bit-7 convention; UART has its traps (
  one `Uart_Init`, never read an empty buffer, no room check on writes ). New findings: F11 ( I2C return value and
  length ), F12 ( `GPIO_15` is PB3, shared by BACK's laser XSHUT and the ESP Wi-Fi enable, to confirm on hardware ).
  8 / 8 blocks pass. Evidence: [CHANGES.md](CHANGES.md).
- Task 10 done ( 15 Nov 2026 ): `Motor.md` and `Peripherals.md` rewritten. Peripherals has a full 18-pin table ( port pin,
  ADC / PWM names, timer, and everything else that uses each pin ); PWM is documented as a pulse of value x 0.2 ms
  ( a percentage only at 50 Hz ), pins on one timer share a rate, the servo has six steps, ADC pins work only if
  enabled in `plutoInit ( )`. Motor warns that M1 / M2 are never stopped by the firmware. F2 is reachable from the
  firmware itself ( a motor test, then the ARM switch ); new F13 ( `Motor_Set` before `Motor_Init` ). 10 / 10 blocks
  pass. Evidence: [CHANGES.md](CHANGES.md).
- Task 11 done ( 15 Nov 2026 ): `Status-LED.md` and `RGB-LED.md` rewritten.
  - Status-LED: `STATUS` is the firmware status pattern. `OFF` stops it, `ON` resumes it, `TOGGLE` is not available
    ( F4 ). While the pattern runs it rewrites the three LEDs every 100-120 ms; nothing resumes it at Developer Mode
    off.
  - RGB-LED: a slot table with the timer and what each slot takes away; `RGB_Release` does not free the pin.
    `RGB_Show` drops a frame while busy, brightness scales only later colours, and nothing returns the strip at
    Developer Mode off.
  - 9 / 9 blocks pass. `WS2812_RGB.md` drift ( 8 vs 64 LEDs ) was added to task 16. Evidence: [CHANGES.md](CHANGES.md).
- Task 12 done ( 15 Nov 2026 ): `Debugging.md` rewritten from the in-repo 1.8.9 page. It covers:
  - precision 0 to 4 and the rounding rule;
  - the ~130-byte per-tick budget, with the byte table;
  - how PlutoMonitor shows each call;
  - the float-with-no-precision pitfall.

  `BlackBox_setVar` is marked not available ( F7 ). 4 / 4 blocks pass.
- Task 13 done ( 15 Nov 2026 ): `Oled.md` rewritten.
  - The display is drawn in RAM and sent at most 16 changed bytes per flight loop ( a full screen in about 0.25 s ).
  - `Oled_Clear` / `Oled_Update` are corrected: RAM only, and queue only, with nothing done in `System` mode.
  - `Oled_Init` works only in `plutoInit`.
  - `Oled_Print` works from 1.8.10, with its guards and the rows it may use.
  - An Advanced note gives the `static` buffer rule and the include.

  10 / 10 blocks pass.
- Task 14 done ( 15 Nov 2026 ): `Scheduler-Timer.md` and `Localisation.md` rewritten.
  - `Interval`: the first `set ( )` fixes the period, and polling rounds it up to `plutoLoop` ticks.
  - Scheduler tasks are added once, in `plutoInit ( )`.
  - Three new findings:
    - **F14:** a 0 ms scheduler interval hangs the flight loop;
    - **F15:** cancelling inside a task reads freed memory;
    - **F16:** `Localisation.get ( )` is always 0, because the UWB reader is compiled out.
  - Localisation's banner is stronger to match F16.

  7 / 7 blocks pass.
- Task 15 done ( 15 Nov 2026 ): the navigation pages were updated.
  - `API-Utils.md` is deleted, and the `Specifiers` links are gone.
  - X1Ranging is in the sidebar, Home and Introduction.
  - Home and Introduction are corrected: loop period in ms, no M5-M8, no Black Box, Localisation marked experimental,
    and the real hook run times.
  - The version line is on Home, Introduction, the sidebar and the footer.

  The new `link_check.py` found 0 dead links in 150.
- Task 16 done ( 15 Nov 2026 ): three hardware maps corrected from the code.
  - `PIN_MAP.md`: PB3 is the ESP enable, PB5 the SPI chip select; new Motor `M1` / `M2` rows; TIM3 / PB5 conflicts.
  - `WS2812_RGB.md`: 64 LEDs, the idle note, the slot conflicts.
  - `TIMER_MAP.md` ( found on the way ): TIM3 is shared with the Motor API.

  The 18 pin rows match the wiki's Peripherals table.
- Task 17 done ( 16 Nov 2026 ): the topic review. Three `pluto-reviewer` slices confirmed about 340 claims and found 5
  blocking issues, all fixed:
  - `FlightMode_Set ( ATLTITUDEHOLD )` can make the drone sink; it is marked not available ( F17 );
  - `I2C_Read` returns the byte count, and the Serial-IO example is fixed;
  - the F2 self-trigger happens only on PRIMUSX2: Motor M5-M8 are documented as flight-motor outputs for projects
    that do not fly ( user decision );
  - the RGB / ADC DMA order ( F18 );
  - the rover example's ARM-switch stop.

  New F19 ( the scheduler never reports out of memory ). Every changed page recompiled; 0 dead links.
- Task 18 done ( 16 Nov 2026 ): closed pending commit. CHANGELOG ( v4.0.0 Documentation ) and a CLAUDE.md line added.
  The firmware-repo part and the wiki repo are staged separately with drafted messages; the user commits both.
  Follow-ups: the defects in [FINDINGS.md](FINDINGS.md) ( F14 and F17 first: both can stop or sink a flying drone )
  and the F12 hardware check.
- Settled in code ( task 9 / 10 ): `GPIO_15` is PB3, the ESP Wi-Fi enable, driven by `RxConfig.cpp:271-305` on every
  target. Still open ( F12 ): whether using it as BACK's laser XSHUT disturbs the Wi-Fi on the boards flown.
- `setUserLoopFrequency ( )`: the header comment says Hz, the code uses the argument as the `plutoLoop` period in
  ms ( 100 gives 100 ms, i.e. 10 Hz ). The wiki will document the real behaviour; the header mismatch is a finding.

## Next action

`/pluto-task next` ( task 11: Status-LED.md and RGB-LED.md ).
