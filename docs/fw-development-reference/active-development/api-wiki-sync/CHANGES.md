# Changes ( api-wiki-sync )

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [FINDINGS](FINDINGS.md) · [CONVENTIONS](CONVENTIONS.md)

What changed, page by page, and the source line behind every statement that is new or corrected. Wiki pages are in
the separate repo `MagisV2.wiki/`; source links are relative to this folder.

## File index

| File | Repo | Task | Change |
|---|---|---|---|
| [CONVENTIONS.md](CONVENTIONS.md) | firmware ( topic ) | 1 | new: page shape, markers, hook rules, example rules |
| [wiki_check.py](wiki_check.py) | firmware ( topic ) | 1 | new: compile and link check for wiki examples |
| [wiki_check_selftest.md](wiki_check_selftest.md) | firmware ( topic ) | 1 | new: fixture for `wiki_check.py` |
| `MagisV2.wiki/PlutoPilot.md` | wiki | 2 | rewritten: hook timing incl. RC-link-stall re-runs, flight-command rule, behaviour and limits, timer-wrap limitation, included and not-included headers, two complete examples |
| `MagisV2.wiki/FC-Control.md` | wiki | 3 | rewritten: unsafe hook examples removed, status priority table, full take-off / land / flip / arm / disarm contracts, `FlightMode_Set` persistence, head-free heading at the next arm, two complete examples |
| `MagisV2.wiki/FC-Config.md` | wiki | 4 | rewritten: loop period in ms, PID gains ( RAM, live ), held tilt with signs and conditions, heading targets incl. `DesiredAngle_setRelative`, stick rates, altitude target ( goal profile, no ceiling ), failsafe switches, how long each setting lasts |
| `MagisV2.wiki/FC-Data.md` | wiki | 5 | rewritten: unit, range and sign tables for every `Sensor_Get` / `Estimate_Get` case, UWB-only X / Y, chip temperature, yaw-rate sign, `Net_Acc` limitation, two complete examples |
| `MagisV2.wiki/BMS.md` | wiki | 5 | rewritten: all nine options with units and valid-from, the low-battery level rules, the auto-land, timing, no-current-sensing boards, two complete examples |
| `MagisV2.wiki/XRanging.md` | wiki | 6 | rewritten: cm vs mm made explicit, family choice, pin table, `initAlt` and its refusal rules, `getStatus`, `getRange` codes, object avoidance ( Developer Mode only, 30 % stick ), SPI2 release, wiring rules, two complete examples |
| `MagisV2.wiki/X1Ranging.md` | wiki | 7 | **new page**: family choice, pin table, timing, `initAlt` with the 190 / 170 cm handover and return guard, every function, OA, wiring rules, measured reach, moving from XRanging, two complete examples |
| `MagisV2.wiki/XRanging.md` | wiki | 7 | three OA facts from the shared core added ( `EXTERNAL` never avoided, LEDs rewritten every 100 ms, side XSHUT pins not reserved ) |
| `MagisV2.wiki/RC-Interface.md` | wiki | 8 | rewritten: ranges and signs of both `RcCommand_Get`, `RcData_Get ( void )` not available ( F5 ), the override rules ( expiry, cross-fade, Developer Mode only, landing, OA ), `App_getAppHeading`, `App_isArmSwitchOn`, two complete examples |
| `MagisV2.wiki/RxConfig.md` | wiki | 8 | rewritten: per-mode channel map table, config calls only in PPM / ELRS, ELRS side effects, range quantisation, internal symbols marked, two complete examples |
| `MagisV2.wiki/Serial-IO.md` | wiki | 9 | rewritten: IMU-writing I2C examples replaced by an external TMP102 example, onboard address table, UART pins and traps ( one init, empty reads, no TX room check ), SPI pins, chip-select set-up, speed codes, bit-7 convention, SPI2 release, `SPI_Init ( )` not available |
| [table_check.py](table_check.py) | firmware ( topic ) | 9 | new: flags Markdown table rows whose cell count differs from the header ( found a stray pipe in the FINDINGS F10 row ) |
| `MagisV2.wiki/Motor.md` | wiki | 10 | rewritten: M1 / M2 pins and timer, 20 kHz, the stop-it-yourself warning, `Motor_Init` first, M5-M8 not available ( F2, F13 ), example that stops in `onLoopFinish` |
| `MagisV2.wiki/Peripherals.md` | wiki | 10 | rewritten: full 18-pin table with port pins, ADC / PWM names, timers and every other user; GPIO, ADC ( `plutoInit` only, DMA sharing ), PWM ( pulse = value x 0.2 ms, shared timer rates, 16 Hz minimum, 1.5 ms start ), servo steps; internal symbols marked |
| [wiki_check.py](wiki_check.py), [wiki_check_selftest.md](wiki_check_selftest.md) | firmware ( topic ) | 4 | blocks now checked at global scope in place of `PlutoPilot.o`; new name-collision fixture |
| `MagisV2.wiki/Status-LED.md` | wiki | 11 | rewritten: `STATUS` is the firmware pattern ( stop / resume, rewrites the LEDs every 100-120 ms ), `TOGGLE` on it not available ( F4 ), pattern table, example that stops and resumes it |
| `MagisV2.wiki/RGB-LED.md` | wiki | 11 | rewritten: slot table with timer and conflicts, `RGB_Release` corrected, pin locked on first init, ownership not returned at Developer Mode off, dropped frames, brightness order, animation tick rounding, status display table |
| `MagisV2.wiki/Debugging.md` | wiki | 12 | rewritten from the in-repo 1.8.9 page: precision table, ~130-byte budget with the byte table, output format, float pitfall, `BlackBox_setVar` not available ( F7 ) |
| `MagisV2.wiki/Oled.md` | wiki | 13 | rewritten: RAM frame and 16-byte flush, `Oled_Clear` / `Oled_Update` corrected, `Oled_Init` only in `plutoInit`, `Oled_Print` since 1.8.10 with its guards, Advanced static-buffer note |
| `MagisV2.wiki/Scheduler-Timer.md` | wiki | 14 | rewritten: `Interval` latching and tick rounding, scheduler add-once in `plutoInit`, skip-missed runs, 0 ms interval and in-task cancel not available ( F14, F15 ) |
| `MagisV2.wiki/Localisation.md` | wiki | 14 | rewritten: "does not work in FW 3.15" banner ( F16 ), explicit include, UART2 / `GPIO_11` taken by `init ( UWB )`, read-only example |
| `MagisV2.wiki/_Sidebar.md`, `Home.md`, `Introduction.md`, `_Footer.md` | wiki | 15 | navigation: `API-Utils` and `Specifiers` entries removed, X1Ranging added ( and Oled in Introduction ), descriptions corrected, version line, Introduction links in page-name style |
| `MagisV2.wiki/API-Utils.md` | wiki | 15 | **deleted** ( internal header ) |
| [link_check.py](link_check.py) | firmware ( topic ) | 15 | new: dead page, file and heading-anchor check for the wiki |
| [PIN_MAP.md](../../PIN_MAP.md) | firmware | 16 | PB3 = ESP Wi-Fi enable; PB5 = SPI API chip select; Motor API `M1` / `M2` rows ( PA6 / PA7 / PB0 / PB1 ), flight motors renamed; RGB slot note; BACK XSHUT on PB3 ( F12 ); conflicts 3, 6 corrected, 8 ( TIM3 ) and 9 ( PB5 ) added; PWM advice corrected |
| [TIMER_MAP.md](../../TIMER_MAP.md) | firmware | 16 | TIM3 owners ( `PWM_8`, `RGB_6`, Motor `M1` / `M2` ); `PWM_8` / `PWM_9` no longer "free pin" |
| [WS2812_RGB.md](../../WS2812_RGB.md) | firmware | 16 | 64 LEDs, 1578-byte DMA buffer, idle note corrected, RGB_6 / RGB_7 / RGB_8 conflicts ( Motor `M1`, SPI chip select, FRONT XSHUT ) |
| 14 wiki pages, the four maps, FINDINGS, CONVENTIONS, `link_check.py` | both | 17 | review fixes ( section below ) |

## Task 1: conventions and the example compile harness ( 15 Nov 2026 )

**Harness design.** The plan said to drop each example into the repo-root `PlutoPilot.cpp`, build, and restore the
file. `wiki_check.py` does better without touching it: it reads the exact `PlutoPilot.cpp` compile line and the ELF
link line from `make TARGET=<T> -n -W PlutoPilot.cpp`, compiles each `cpp` block as its own translation unit inside
`namespace wiki_ex` ( so every block may define `plutoLoop ( )` ), and links that object with the built firmware
objects into `Build/wikicheck/<T>/<page>_L<line>.elf`. A generated `extern "C" wiki_check_entry ( )` calls the block's
hooks and is kept with `-Wl,-u`, so `--gc-sections` cannot drop a call before the linker resolves it. `#line`
directives make every error and warning point at the wiki page line. Tools are resolved to full paths because Windows
`CreateProcess` searches the parent's `PATH`, not the one handed to the child.

**Self-test** ( [wiki_check_selftest.md](wiki_check_selftest.md), `PRIMUS_X2_v1` ): 7 checked, 3 passed ( one
`WARN-OK` ), 4 failed, 1 skipped, exit 1, every block with its expected result: an undeclared call fails `compile`;
`SPI_Init ( )` fails `link` with `undefined reference to SPI_Init()` ( finding F1, so link checking works ); the
unmarked `Monitor_Print ( "v:", 3.7f )` fails `warning` with `-Wfloat-conversion`; the marked one is `WARN-OK`; a
statement outside any hook fails `no hook`; an explicit `#include "API/Localisation.h"` passes. First run 43 s ( the
incremental `make` rebuilt the objects, last built at FW 3.15.10 ); later runs 1.2 s.

**Nothing else written.** `PlutoPilot.cpp` SHA-1 `3506618828cd9088ec20ba6b21c970f8f1a8a6bd` before and after;
`git status` shows only this topic, the topic index and the untracked wiki clone; `Build/wikicheck/` is removed after
each run. The one other file the runs produced is `Build/PRIMUS_X2_v1/AltX_PRIMUS_X2_v1_3.15.11.hex`, from the normal
incremental `make` of the unchanged tree: the same file a PlutoIDE build gives.

**Baseline: the wiki as it is today** ( `47e451d`, all pages, `PRIMUS_X2_v1` ): 168 `cpp` blocks, 113 pass, 55 fail.

| Page | Blocks | Fail | compile | link | warning | no hook |
|---|---|---|---|---|---|---|
| API-Utils.md | 1 | 1 | 1 | 0 | 0 | 0 |
| BMS.md | 3 | 1 | 0 | 0 | 0 | 1 |
| Debugging.md | 9 | 0 | 0 | 0 | 0 | 0 |
| FC-Config.md | 18 | 4 | 0 | 0 | 0 | 4 |
| FC-Control.md | 14 | 3 | 0 | 0 | 0 | 3 |
| FC-Data.md | 9 | 9 | 0 | 0 | 5 | 4 |
| Introduction.md | 1 | 0 | 0 | 0 | 0 | 0 |
| Localisation.md | 2 | 2 | 1 | 0 | 0 | 1 |
| Motor.md | 7 | 2 | 0 | 0 | 0 | 2 |
| Oled.md | 12 | 2 | 0 | 1 | 0 | 1 |
| Peripherals.md | 15 | 6 | 0 | 0 | 1 | 5 |
| PlutoPilot.md | 7 | 1 | 0 | 0 | 0 | 1 |
| RC-Interface.md | 8 | 1 | 0 | 0 | 0 | 1 |
| RGB-LED.md | 6 | 5 | 2 | 0 | 0 | 3 |
| RxConfig.md | 8 | 3 | 0 | 0 | 0 | 3 |
| Scheduler-Timer.md | 14 | 6 | 3 | 0 | 0 | 3 |
| Serial-IO.md | 21 | 6 | 0 | 0 | 2 | 4 |
| Status-LED.md | 4 | 2 | 0 | 0 | 0 | 2 |
| XRanging.md | 9 | 1 | 0 | 0 | 0 | 1 |
| **Total** | **168** | **55** | **7** | **1** | **8** | **39** |

Most `no hook` failures are enum or signature quotes, which get a `skip` marker. Passing does not mean correct:
113 blocks compile, but many are wrong in behaviour ( [SCOUT.md](SCOUT.md) ), for example the FC-Control examples
that land or disarm from `onLoopFinish ( )` compile cleanly. The single link failure was a new defect, F9, fixed on 15 Nov 2026 in topic
[oled-print-linkage](../oled-print-linkage/README.md):
`Oled_Print` is declared with C++ linkage in [Oled.h:94](../../../../src/main/API/Oled.h#L94) but defined in the C file
[oled_display.c:536](../../../../src/main/io/oled_display.c#L536), so the object exported the C symbol `Oled_Print`
and a user call could not link. The declaration is now `extern "C"`, and the same block passes.

**Hook rule source.** [mw.cpp:1176](../../../../src/main/mw.cpp#L1176) runs user code only while RC frames arrive;
a late frame takes the `else` path and runs `onLoopFinish ( )` ( [:1230](../../../../src/main/mw.cpp#L1230) ), and
the next frame runs `onLoopStart ( )` again ( [:1194](../../../../src/main/mw.cpp#L1194) ).
`plutoRxConfig ( )` is called only from `validateAndFixConfig ( )` ( [config.cpp:665](../../../../src/main/config/config.cpp#L665) ),
which `readEEPROM ( )` runs ( [:765](../../../../src/main/config/config.cpp#L765) ). `readEEPROM ( )` runs at boot
( [main.cpp:177](../../../../src/main/main.cpp#L177), before `plutoInit ( )` at [:524](../../../../src/main/main.cpp#L524) )
and again from `saveConfigAndNotify ( )` ( [config.cpp:844](../../../../src/main/config/config.cpp#L844) ),
`changeProfile ( )` ( [:850](../../../../src/main/config/config.cpp#L850) ), `MSP_SELECT_SETTING`, `MSP_RESET_CONF`
and `MSP_EEPROM_WRITE` ( [serial_msp.cpp:1406, 1627, 1644](../../../../src/main/io/serial_msp.cpp#L1406) ) and the CLI.
So `plutoRxConfig ( )` re-runs on every settings save. The PlutoPilot page ( task 2 ) must say so.

## Task 2: PlutoPilot.md ( 15 Nov 2026 )

**Result.** `wiki_check.py MagisV2.wiki/PlutoPilot.md`: 8 checked, 8 passed, 0 failed, 1 skipped ( the
`#include "PlutoPilot.h"` line ). Before: 7 blocks, 1 failed ( that same include line, outside any hook ). Passing
before was not correctness: the old page said `onLoopStart ( )` runs once, `plutoLoop ( )` runs "repeatedly",
configured channels in `Rx_ESP` mode where the call is ignored, enabled the inert `LOW_BATTERY` failsafe, and sent
`Command_TakeOff ( 150 )` on every loop while armed.

A scratch fixture with the two "add it yourself" include lines ( `src/main/API/Localisation.h`,
`src/main/io/oled_display.h` ) and a call into each compiled and linked from user code.

**Claims on the page and their source** ( links relative to this folder ):

| Statement | Source |
|---|---|
| All five hooks must be defined; a missing one fails at link | each is called unconditionally: [config.cpp:665](../../../../src/main/config/config.cpp#L665), [main.cpp:524](../../../../src/main/main.cpp#L524), [mw.cpp:1194, 1199, 1230](../../../../src/main/mw.cpp#L1194) |
| `plutoRxConfig ( )` runs at power-up before `plutoInit ( )`, and again on a settings save, reset or profile change | task 1 section above ( `readEEPROM ( )` at `main.cpp:177`, `saveConfigAndNotify`, `changeProfile`, `MSP_*` ) |
| `Receiver_Config_*` calls are ignored with `Rx_ESP` / `Rx_CAM` | `AuxChangeEnable = false` for those modes ( [RxConfig.cpp:119](../../../../src/main/API-Src/RxConfig.cpp#L119) ); each `Receiver_Config_*` body is guarded by `if ( AuxChangeEnable )` ( `:191-261` ) |
| `plutoInit ( )` runs once, unconditionally, near the end of start-up | directly in `init ( )` ( [main.cpp:524](../../../../src/main/main.cpp#L524) ), after `batteryInit` ( `:490` ) and `SYSTEM_STATE_READY` |
| The laser start-up runs right after `plutoInit ( )` | `rangingFamilyLatch ( )`, `xRangingInit ( )`, `x1RangingInit ( )` at [main.cpp:584-586](../../../../src/main/main.cpp#L584) |
| Developer Mode needs the switch **and** RC frames | [mw.cpp:1176](../../../../src/main/mw.cpp#L1176): the dev switch AND `rxIsReceivingSignal ( )` or `ppmIsRecievingSignal ( )` |
| A frame 200 ms late on the Wi-Fi app, 100 ms on ELRS / PPM, is a stall | `needRxSignalBefore = now + DELAY_5_HZ` for MSP ( [rx.cpp:369](../../../../src/main/rx/rx.cpp#L369) ), `DELAY_10_HZ` for serial RX and PPM ( `:356`, `:381` ); `DELAY_5_HZ` = 200 ms, `DELAY_10_HZ` = 100 ms ( `:91-92` ) |
| A stall runs `onLoopFinish ( )` at once and `onLoopStart ( )` with the next frame | [mw.cpp:1223-1232](../../../../src/main/mw.cpp#L1223) and `:1193-1196` ( `callOnPilotStart` / `callonPilotFinish` ) |
| The firmware drops every RC override just before `onLoopFinish ( )` | `RC_ARRAY [ i ] = 0; userRCflag [ i ] = false` for the four channels ( [mw.cpp:1224-1227](../../../../src/main/mw.cpp#L1224) ) |
| `plutoLoop ( )` runs every 100 ms by default | `userLoopFrequency = 100000` us ( [API-Utils.cpp:102](../../../../src/main/API-Src/API-Utils.cpp#L102) ), used as a period at [mw.cpp:1192](../../../../src/main/mw.cpp#L1192) |
| `setUserLoopFrequency ( x )` sets the period to x ms, clamped 3.5-2000 | `constrainf ( frequency, 3.5, 2000 )`, then x 1000 into the us period ( [FC-Config-PID.cpp:85-93](../../../../src/main/API-Src/FC-Config-PID.cpp#L85) ); finding F8 |
| The three loop hooks run inside the flight loop | `userCode ( )` is called inline from the main loop ( [mw.cpp:1436](../../../../src/main/mw.cpp#L1436) ) |
| Between `plutoLoop ( )` calls the firmware keeps applying overrides, flight modes and the heading | the `else` branch ( `magHold = userHeading`, `FlightMode_Set` ) and `resetUserRCflag ( )` / `applyUserRcOverride ( )` on every iteration ( [mw.cpp:1200-1220](../../../../src/main/mw.cpp#L1200) ) |
| An override expires after max ( 250 ms, 2 periods ) | `pluto-rules` invariant; `USER_RC_HOLD_MIN_US` 250000 ( `API-Utils.h` ) |
| RC loss lands about 2.5 s after the last frame, whatever user code does | CLAUDE.md invariant and `rx-signal-loss-tolerance` ( flight-validated ); `failsafe_e` has no RC-loss switch ( [FC-Config.h:46-51](../../../../src/main/API/FC-Config.h#L46) ) |
| About 130 bytes of prints per tick | CLAUDE.md invariant; `docs/API/DEBUGGING_API_WIKI.md` |
| No C++ static constructors | CLAUDE.md ( `-nostartfiles` ) |
| Timer-wrap limitation, 35.8 / 71.6 min | `userLoopTime` starts at 0 ( [mw.cpp:162](../../../../src/main/mw.cpp#L162) ) and is written only at `:1192`; the gate is `( int32_t ) ( now - userLoopTime ) >= 0` ( `:1190` ), and `onLoopStart ( )` sits inside the same gate ( `:1193` ). 2^31 us = 35.8 min, 2^32 us = 71.6 min. Planned topic `user-loop-timer-wrap` |
| Included headers ( 16, with `X1Ranging.h` since 1.8.0 ) | [PlutoPilot.h:27-42](../../../../PlutoPilot.h#L27) |
| `Localisation.h` and `io/oled_display.h` are not included | absent from `PlutoPilot.h:27-42`; include lines compiled and linked ( fixture above ) |

The take-off example is the `FC_CONTROL_API_WIKI.md` Quick Start, kept identical because it is the flight-validated
pattern from `takeoff-land-command-fix`; only one comment word changed ( "hiccup" -> "stall", to match the page ).

## Task 3: FC-Control.md ( 15 Nov 2026 )

**Result.** `wiki_check.py MagisV2.wiki/FC-Control.md`: 11 checked, 11 passed, 0 failed, 3 skipped ( the three enum
quotes ). Before: 14 blocks, 3 failed ( the enum quotes, outside any hook ); the 11 that compiled included the five
unsafe ones: `Command_Land` and `Command_DisArm` in `onLoopFinish ( )` ( three examples ), `Command_Arm` /
`Command_TakeOff` in `onLoopStart ( )`, and a flip sent the moment the drone armed. A scan of the new page finds no
`Command_*` call inside an `onLoopStart ( )` or `onLoopFinish ( )` body.

Also corrected in this task: the PlutoPilot page and CONVENTIONS.md named `setRelativeAltitude` as a user call. It is
internal; the user call is `DesiredPosition_setRelative ( Z, cm )` ( [FC-Config.h:268](../../../../src/main/API/FC-Config.h#L268) ).

**New examples written for this page** ( not copied from a validated doc ), for the topic review to read closely:

| Example | Safety reasoning |
|---|---|
| `Command_Land ( 5 )` on every tick at the battery warning | a repeated call is ignored while the landing runs ( `command_status` RUNNING, [FC-Control-Command.cpp:153-170](../../../../src/main/API-Src/FC-Control-Command.cpp#L153) ) and once disarmed; the landing sets FINISHED and disarms in the same step ( [command.cpp:245-247](../../../../src/main/command/command.cpp#L245) ), so no tick sees FINISHED while armed |
| Flip once per ARM switch-on | waits 5 s after arming so a take-off ( 2-2.5 s ) has finished, because `Command_Flip` replaces the running command ( `current_command = B_FLIP`, [:173-183](../../../../src/main/API-Src/FC-Control-Command.cpp#L173) ); then needs MAG mode and 100 cm |
| Disarm if armed 5 s and never left the floor | a `leftFloor` latch set above 30 cm means it can never disarm a drone that has flown, so not at the end of a manual landing nor in a low hover with baro noise |
| `FlightMode_Set` in `onLoopStart ( )` | repeat-safe: setting the same mode again changes nothing |

The flight-status, take-off and landing examples, and the once-per-ARM Usage Example, are the reviewed and
flight-validated ones from `docs/API/FC_CONTROL_API_WIKI.md` ( `takeoff-land-command-fix`, `flight-status-enum-map` ).

**Claims on the page and their source:**

| Statement | Source |
|---|---|
| Status priority order, 0xFFFF after a disarm, the app told LOW instead of critical while armed | [FC-Control.h:36-58](../../../../src/main/API/FC-Control.h#L36); `flight-status-enum-map` |
| The value is not the priority; 1.8.4 name mapping | `FC-Control.h` enum values vs the priority list; `flight-status-enum-map` ( `6d1b5cd` ) |
| `FS_CRASHED` clears about 0.3 s after the disarm once level | `FC-Control.h:43-44` |
| `RATE` = ANGLE off, `THROTTLE_MODE` = altitude hold off | [FC-Control-Command.cpp:52-78](../../../../src/main/API-Src/FC-Control-Command.cpp#L52) ( `! FLIGHT_MODE ( ANGLE_MODE )`, `! FLIGHT_MODE ( BARO_MODE )` ) |
| `FlightMode_Set` turns the other mode of its pair off | [:80-120](../../../../src/main/API-Src/FC-Control-Command.cpp#L80) |
| It overrides the app's switch while Developer Mode is on | the switches set the modes in `processRx ( )` ( [mw.cpp:972-1016](../../../../src/main/mw.cpp#L972) ); `userCode ( )` re-applies every `isUserFlightModeSet` mode between `plutoLoop ( )` ticks ( [mw.cpp:1208-1212](../../../../src/main/mw.cpp#L1208) ), inside the Developer Mode branch |
| It is kept until the other mode of the pair or power-off; no hand-back call | `isUserFlightModeSet` is written only in `FlightMode_Set` and zeroed at definition ( [API-Utils.cpp:136](../../../../src/main/API-Src/API-Utils.cpp#L136) ) |
| `Command_TakeOff` contract ( arming, 3 s, ground idle, once per arm, centred 1460-1540, 2 s drop, dev-off drop ) | [FC-Control.h:110-127](../../../../src/main/API/FC-Control.h#L110); `takeoff-land-command-fix` |
| Take-off ignored during a landing; height constrained 100-250; a second take-off while one waits is ignored | [FC-Control-Command.cpp:137, 148, 141](../../../../src/main/API-Src/FC-Control-Command.cpp#L135) |
| A 100 cm climb takes about 2-2.5 s | flight data in `docs/API/FC_CONTROL_API_WIKI.md` ( `takeoff-land-command-fix` ) |
| Land: accepted only armed, no command running, no flip; start 1305 - `landSpeed` | [FC-Control-Command.cpp:153-170](../../../../src/main/API-Src/FC-Control-Command.cpp#L153) |
| Throttle down to 1150; touchdown after 300 ms settled, never before 1 s; 30 s backstop | `LAND_THROTTLE_MIN`, `LAND_SETTLE_MS`, `LAND_CONFIRM_MIN_MS`, `LAND_TIMEOUT_MS` ( [command.cpp:232-238](../../../../src/main/command/command.cpp#L232) ) |
| Descent ( 1500 - throttle ) / 4 cm/s, at most 100 cm/s | [FC-Control.h:133-135](../../../../src/main/API/FC-Control.h#L133) |
| Flip: MAG mode only, ends within 8 s, level within 70 degrees else disarm, ignored in a landing | [FC-Control.h:143-152](../../../../src/main/API/FC-Control.h#L143); [FC-Control-Command.cpp:175](../../../../src/main/API-Src/FC-Control-Command.cpp#L175) |
| `Command_DisArm` disarms unconditionally, in the air too | `mwDisarm ( )` with no condition ( [FC-Control-Command.cpp:197-201](../../../../src/main/API-Src/FC-Control-Command.cpp#L197) ) |
| Head-free heading in degrees, applied at the next arm, else the heading at arm; kept until power-off; the head-adjust switch can re-take it | stored at [FC-Control-Command.cpp:205-209](../../../../src/main/API-Src/FC-Control-Command.cpp#L205); applied at arm ( [mw.cpp:634-638](../../../../src/main/mw.cpp#L634) ); flag never cleared; `degreesToRadians ( heading - headFreeModeHold )` ( [mw.cpp:431](../../../../src/main/mw.cpp#L431) ); head-adjust re-take ( [mw.cpp:1017](../../../../src/main/mw.cpp#L1017) ) |
| `App_isArmSwitchOn ( )` is available from `PlutoPilot.h` | declared in [RC-Interface.h:161](../../../../src/main/API/RC-Interface.h#L161) |

## Task 4: FC-Config.md ( 15 Nov 2026 )

**Harness change first.** `wiki_check.py` used to wrap each block in a namespace and link it next to the firmware's own
`PlutoPilot.o`. That cannot see a user's file-level name colliding with a firmware global declared in a header
`PlutoPilot.h` pulls in, and the complete-file Usage Examples are pasted at global scope. It now checks each block as a
user's `PlutoPilot.cpp`: global scope, empty stubs for the hooks the block lacks, linked **in place of** `PlutoPilot.o`.
The hooks are referenced by the firmware itself, so the generated entry function and `-Wl,-u` are gone. The self-test
gave identical results, plus a new block ( `static bool _isAdcEnable`, which `Peripherals.h:132` declares `extern` ) that
now fails `compile` as it should: 8 checked, 3 passed, 5 failed, 1 skipped. PlutoPilot.md and FC-Control.md re-checked
under the new design: all pass.

**Result.** `wiki_check.py MagisV2.wiki/FC-Config.md`: 15 checked, 15 passed, 0 failed, 4 skipped ( the struct and
enum quotes ). Before: 18 blocks, 4 failed ( the quotes ). What compiled before was wrong in behaviour: "100 Hz" for a
100 ms period, roll and pitch "set every loop", the pitch sign backwards ( -100 called "forward" ), `setDesiredRate
( AG_YAW, 200 )` called a rotation, `Failsafe_enable ( LOW_BATTERY )` used as a protection, and a usage example holding a
15 / 10 degree tilt every loop with no level-out. `DesiredAngle_setRelative` was missing.

**New examples written for this page** ( the tilt, square, rate and failsafe examples come from the reviewed
`docs/API/YAW_HEADING_API_WIKI.md`, `FC_CONFIG_API_WIKI.md` and `FAILSAFE_API_WIKI.md` ):

| Example | Safety reasoning |
|---|---|
| `setDesiredPosition ( Z, 120 )` once per flight | only in ALT HOLD and once above 50 cm, so never on the floor; reset on disarm; 120 cm is a modest target and the page warns there is no ceiling |
| `DesiredPosition_setRelative ( Z, 50 )` once per flight | waits 5 s after arming so a take-off has finished; once per flight because each call adds to the estimate of the moment |
| Fly a square, then lean 5 degrees for 1 s | the reviewed square and lean examples chained; the lean is levelled after 1 s |
| `set_PIDProfile` with roll P 45 | in `plutoInit ( )`, disarmed; a 5-step change from the default 40, labelled as an example |

**Claims on the page and their source:**

| Statement | Source |
|---|---|
| `setUserLoopFrequency ( x )`: period x ms, clamped 3.5-2000, default 100 ms; takes effect after the next call | [FC-Config-PID.cpp:85-93](../../../../src/main/API-Src/FC-Config-PID.cpp#L85); default [API-Utils.cpp:102](../../../../src/main/API-Src/API-Utils.cpp#L102); next due time computed before `plutoLoop ( )` ( [mw.cpp:1192, 1199](../../../../src/main/mw.cpp#L1192) ); finding F8 |
| `get/set_PIDProfile` read and write the live `currentProfile` gains, RAM only | [FC-Config-PID.cpp](../../../../src/main/API-Src/FC-Config-PID.cpp) `currentProfile->pidProfile.P8/I8/D8`; saving and LuxFloat notes as for `setDefault_PIDProfile` ( [FC-Config.h:79-99](../../../../src/main/API/FC-Config.h#L79) ) |
| Default gains per profile | [FC-Config.h:82-84](../../../../src/main/API/FC-Config.h#L82) |
| `PID_ALT` is altitude hold; `PID_USER` is UWB position control's velocity gains | `P8 [ PIDALT ]` in [altitudehold.cpp:505, 814, 903](../../../../src/main/flight/altitudehold.cpp#L505); `P8/I8/D8 [ PIDUSER ]` in [posControl.cpp:192-194](../../../../src/main/flight/posControl.cpp#L192), the only reader |
| `setDefault_PIDProfile` restores the five, not the internal gains; 1.8.5 | `FC-Config.h:79-99`; `set-default-pid-profile` ( `ebf4e46` ) |
| `getDesiredAngle`: stick plus tilt before the limit; yaw 0 | [FC-Config.h:101-113](../../../../src/main/API/FC-Config.h#L101) |
| Tilt: deci-degrees, held, signs, stick adds under `max_angle_inclination` 20 deg, conditions, dropped at dev-off and disarm, 0.6 s | [FC-Config.h:115-145](../../../../src/main/API/FC-Config.h#L115); signs and conditions table in `YAW_HEADING_API_WIKI.md` ( `set-desired-angle-roll-pitch`, `73022c3` ) |
| Yaw target: whole degrees 0-359, wrapped, heading hold engaged, HEADFREE kept, pilot override past 60 % of stick, return after release under ~17 deg/s | `FC-Config.h:133-145`; `YAW_HEADING_API_WIKI.md` ( `relative-yaw-api`, `794002d` ) |
| `DesiredAngle_setRelative`: yaw only, calls add up, first call from the heading, `onLoopStart` pitfall, re-base after a stick release, do not mix with a yaw override | [FC-Config.h:147-182](../../../../src/main/API/FC-Config.h#L147) |
| When a heading target ends | `YAW_HEADING_API_WIKI.md` "When the target ends"; dev-off and disarm handling follows the comment at [mw.cpp:1235-1245](../../../../src/main/mw.cpp#L1235) |
| Stick rates: ranges, clamp, defaults, ANGLE no-effect, `rate + 20`, hold until reboot, flips, saving | [FC-Config.h:184-232](../../../../src/main/API/FC-Config.h#L184); `FC_CONFIG_API_WIKI.md` ( `set-desired-rate-api`, `3be9093` ) |
| `getDesiredPositions ( Z )` returns the altitude target | `getSetAltitude ( )` returns `AltHold` ( [altitudehold.cpp:1466-1468](../../../../src/main/flight/altitudehold.cpp#L1466) ) |
| `setDesiredPosition ( Z, cm )` sets the target only ( the estimator is not touched ) | it calls `setAltitude ( int32_t )`, the only overload `altitudehold.h:77` declares: `AltHold = altitude` ( [altitudehold.cpp:1502-1504](../../../../src/main/flight/altitudehold.cpp#L1502) ). The other overload, `setAltitude ( float )` at `:1450`, rewrites the estimator, but it is file-local and not reachable from the API |
| `DesiredPosition_setRelative ( Z, cm )` is relative to the estimated altitude | `AltHold = EstAlt + altitude` ( [altitudehold.cpp:1506-1508](../../../../src/main/flight/altitudehold.cpp#L1506) ) |
| A written target becomes a goal: up to 60 cm/s up, 30 cm/s down, braking 80 cm/s/s; the stick cancels it | [altitudehold.cpp:418-431, 762-768](../../../../src/main/flight/altitudehold.cpp#L418) ( `ALT_CMD_MAX_CLIMB_CMS`, `ALT_CMD_MAX_DESCENT_CMS`, `ALT_GOAL_DECEL_CMSS`, `setVelocity != 0` cancels ) |
| No height limit | no clamp on `AltHold` anywhere in `altitudehold.cpp`; only the control error is constrained to +/-500 cm ( `:502`, `:804` ) |
| Failsafe table, only the named switch changes, all on at power-up, not saved, no read-back, crash at once, battery only blocks raising a level | [FC-Config.h:46-51, 270-292](../../../../src/main/API/FC-Config.h#L46); `FAILSAFE_API_WIKI.md` ( `failsafe-enable-api`, `c344387` ) |
| How long each setting lasts | the rows above; loop period resets because `userLoopFrequency` is RAM, initialised to 100000 |
| Untested yaw and roll cases | `relative-yaw-api` README ( hand-yaw, mid-turn switch-off, flicker, HEADFREE untested ); `set-desired-angle-roll-pitch` README ( roll - not validated ) |

## Task 5: FC-Data.md and BMS.md ( 15 Nov 2026 )

**Result.** `wiki_check.py MagisV2.wiki/FC-Data.md MagisV2.wiki/BMS.md`: 9 checked, 9 passed, 0 failed, 3 skipped ( the
enum quotes ). Before: FC-Data 9 blocks, 9 failed ( 5 warnings, 4 outside any hook ); BMS 3 blocks, 1 failed ( the
enum quote ). The old FC-Data page said Velocity needs "GPS / optical flow / UWB" ( UWB is the only X / Y source and Z is
always valid ), gave pressure as "100 * mbar" without the Pa name, and gave no unit for the accelerometer, gyroscope,
rate, position or velocity. The old BMS page had none of `SoC`, `Warning_Level`, `Resistance`, computed a percentage by
hand, and did not mention the auto-land.

The content is merged from `docs/API/FC_DATA_API_WIKI.md` ( `baro-temperature-api` `7a41a37`, `estimate-get-rate-api`
`1b43fbe`: bench-validated units and signs ) and `docs/API/BMS_API_WIKI.md` ( `battery-soc-fix` `cbd1c72`: validated on
4 packs ). Internals ( formulas, driver names, file architecture ) are left out of the public pages. Both pages' print
lines stay well inside the ~130-byte budget ( about 65 and 50 bytes per tick ).

**New finding F10:** `Sensor_Get ( Gyroscope | Magnetometer, Net_Acc )` falls off the end of the function
( [FC-Data-Sensors.cpp:63-72](../../../../src/main/API-Src/FC-Data-Sensors.cpp#L63) ). Listed in [FINDINGS.md](FINDINGS.md) and shown
on the page as a known limitation with its workaround.

**Replaced example:** the BMS wiki's "react to the level" example called `RGB_SetColorAll ( )` with no `RGB_Init ( )`, so
it was not self-contained. The page prints a message when the level changes instead.

**Claims on the pages and their source:**

| Statement | Source |
|---|---|
| Barometer: Pa and deg C x100, rounded; 0 for a non-barometer sensor | [FC-Data.h:72-83](../../../../src/main/API/FC-Data.h#L72); [FC-Data-Sensors.cpp:84-95](../../../../src/main/API-Src/FC-Data-Sensors.cpp#L84) ( `roundToInt32` ) |
| Chip temperature, warms after power-on | `FC_DATA_API_WIKI.md`, bench log of `baro-temperature-api` |
| Accelerometer cm/s^2 ( 980 at rest ), `Net_Acc` g^2 x100 ( squared ), gyro deg/s x10, magnetometer relative and 0 if never calibrated | [FC-Data-Sensors.cpp:50-74](../../../../src/main/API-Src/FC-Data-Sensors.cpp#L50) ( `accSmooth * accVelScale`, `netAccMagnitude`, `gyroADC / 1.64`, `magADC / 6.82` ); units and the uncalibrated 0 from `FC_DATA_API_WIKI.md` |
| `Estimate_Get` rate deg/s x10 rounded; position cm, velocity cm/s; X / Y UWB only; signs | [FC-Data.h:85-109](../../../../src/main/API/FC-Data.h#L85); signs bench-measured in `estimate-get-rate-api` |
| Angles: deci-degrees roll / pitch, degrees heading, ranges, signs | [FC-Data.h:111-128](../../../../src/main/API/FC-Data.h#L111); ranges from `FC_DATA_API_WIKI.md` |
| Values read 0 before the first update, so in `plutoInit ( )` | `FC_DATA_API_WIKI.md` |
| `Bms_Get` returns 0 for an unknown option | [BMS.cpp:27-58](../../../../src/main/API-Src/BMS.cpp#L27) ( `default: return 0` ) |
| The nine options, units, valid-from, the level rules ( 15 / 5 %, 3.745 / 3.60 V, 1.5 A, 1.5 s, latched, 35 s, 40 %, provisional ) | [BMS.h:37-47](../../../../src/main/API/BMS.h#L37); `BMS_API_WIKI.md` ( `battery-soc-fix`, validated on 4 packs ) |
| The levels need `INFLIGHT_LOW_BATTERY`; `LOW_BATTERY` has no effect | [FC-Config.h:46-51](../../../../src/main/API/FC-Config.h#L46); `failsafe-enable-api` |
| Auto-land at critical, throttle only, not cancellable, no re-arm until the pack is changed | `BMS_API_WIKI.md`; CLAUDE.md flight invariants |
| One 20 mOhm shunt; a second shunt halves every current and mAh reading | CLAUDE.md flight invariant; `battery-capacity-estimate` |
| No current sensing: voltage levels 3.10 / 3.00 V per cell | `BMS_API_WIKI.md` |

## Task 6: XRanging.md ( 15 Nov 2026 )

**Result.** `wiki_check.py MagisV2.wiki/XRanging.md`: 10 checked, 10 passed, 0 failed, 2 skipped ( the enum quotes ).
Before: 9 blocks, 1 failed ( an enum quote ). The old page gave the trigger threshold and the avoidance distance in
**mm** ( the code multiplies by 10: they are cm ), said `getRange ( )` returns -1 for "no reading" ( -100; -1 is "not
enabled" ), and had no `DOWN`, `initAlt`, `getStatus` or `laser_status_e`.

Merged from `docs/API/XRANGING_API_WIKI.md` ( `tof-single-pipeline` `edfbd61`, `vl53l1x-xranging-api` `019a55c`,
flight-validated ). The internal pipeline doc pointer and the file architecture are left out.

**Found while verifying, now on the page:**

- **Object avoidance acts only while Developer Mode runs.** `oaCoreRun` has no Developer Mode gate, but it acts by
  writing `RC_ARRAY` and `userRCassert ( )` ( [ObjectAvoidanceCore.h:260-270](../../../../src/main/API-Src/ObjectAvoidanceCore.h#L260) ),
  and `RC_ARRAY` is consumed only by `applyUserRcOverride ( )` ( [mw.cpp:1147-1165](../../../../src/main/mw.cpp#L1147) ), called only
  inside the `runUserCode` branch of `userCode ( )` ( [mw.cpp:1220](../../../../src/main/mw.cpp#L1220) ). The 30 % stick scaling comes
  through the same path, so it also applies only then.
- **`isTriggered ( )` is always `false` for a laser enabled only by `initObjectAvoidance ( )`.** That call sets
  `isXLaserInit` but no threshold, and `isTriggered` returns `false` for a threshold below 0
  ( [XRanging.cpp:418-430, 456-466](../../../../src/main/API-Src/XRanging.cpp#L418) ). The `-1` default really is in place at power-up:
  `XRanging` is constant-initialized, see the next point.
- **The constructor wording on the PlutoPilot page and in CONVENTIONS.md was too strong.** `XRanging_P` has a default
  member initializer ( `triggerThreshold [ 6 ] = { -1, ... }` ). `arm-none-eabi-nm` shows `D XRanging`, `objdump -s`
  shows its 12 bytes as `ffff` x 6 in `.data`, and the object has no `.init_array` section: the compiler fills it in at
  build time, so no start-up code is needed. The rule is now "a file-level object whose constructor has to run code is
  left zeroed".

**Claims on the page and their source:**

| Statement | Source |
|---|---|
| Threshold and avoidance distance in cm; `getRange` in mm | `threshold * 10` ( [XRanging.cpp:377](../../../../src/main/API-Src/XRanging.cpp#L377) ), `_avoidDist * 10` ( `:462` ), `rangeMm` ( `:415` ) |
| `getRange`: -1 not enabled, -100 no valid sample | [XRanging.cpp:410-416](../../../../src/main/API-Src/XRanging.cpp#L410) ( `TOF_RANGE_INVALID` ) |
| `init ( )` enables four lasers at 20 cm | `triggerThreshold = 200` mm ( [XRanging.cpp:362-372](../../../../src/main/API-Src/XRanging.cpp#L362) ) |
| `init ( laser, ... )` ignores `DOWN` | `laser >= LASER_COUNT` returns ( `:375` ) |
| `initAlt` refused after `plutoInit ( )`, on a side XSHUT, a reserved pin or an aliased ADC / PWM; the four usable pins | `altRequest ( )`, `altPinConflict ( )`, `altReservedGpio [ ]`, `gpioAlias [ ]` ( [XRanging.cpp:98-150, 380-397](../../../../src/main/API-Src/XRanging.cpp#L98) ); [XRanging.h](../../../../src/main/API/XRanging.h) `initAlt` comment |
| Pin table | `lasers [ ]` ( [XRanging.cpp:67-72](../../../../src/main/API-Src/XRanging.cpp#L67) ) |
| `getStatus` mapping | [XRanging.cpp:399-408](../../../../src/main/API-Src/XRanging.cpp#L399) |
| Read pacing ( 33 ms / 10 ms ), 160 / 140 cm handover, family choice and both-refused, SPI2 release, wiring rules, OA timing ( 100 ms, 0.5 s push / brake, 30 % / 20 %, 0.3 s release ) | header comments; `XRANGING_API_WIKI.md` ( flight-validated ); CLAUDE.md flight invariants; `OA_MAX_PUSH_TIME_MS`, `OA_CLEAR_HOLD_MS`, `USER_OA_BLEND_PUSH` ( [ObjectAvoidanceCore.h:39-43](../../../../src/main/API-Src/ObjectAvoidanceCore.h#L39) ) |
| Object avoidance runs for the XRanging ( or no ) family only | `switch ( rangingFamily ( ) )` ( [mw.cpp:546-551](../../../../src/main/mw.cpp#L546) ) |

**For task 10:** `BACK`'s XSHUT is `GPIO_15`, which has no ADC or PWM alias ( `gpioAlias [ GPIO_15 ] = { -1, -1 }` ). The
scout tentatively tied `GPIO_15` to PB3, the pin `RxConfig` drives as the ESP Wi-Fi enable. If that is true, BACK's
XSHUT and the ESP enable share a pin. Task 10 must settle the `GPIO_15` to port-pin mapping before the Peripherals page
states it.

## Task 7: X1Ranging.md, new page ( 15 Nov 2026 )

**Result.** `wiki_check.py MagisV2.wiki/X1Ranging.md`: 10 checked, 10 passed, 0 failed ( no enum quotes: the page uses
`laser_e` / `laser_status_e` from XRanging.h ). Every function in `X1Ranging.h` has a section: both `init`, both
`initAlt`, `getRange`, `isTriggered`, `getStatus`, `initObjectAvoidance`, `enableOA` / `disableOA`. Shaped like the
task 6 XRanging page so the two read as a pair. Merged from `docs/API/X1RANGING_API_WIKI.md` ( `vl53l1x-xranging-api`
`019a55c`, `a81c180`, flight-validated ).

**XRanging page updated too.** Three object-avoidance facts in the X1 wiki come from the core both APIs share, so they
apply to XRanging as well, and the task 6 page lacked them:

- avoidance reads only `LEFT`, `RIGHT`, `FRONT`, `BACK`: `EXTERNAL` is never avoided
  ( [ObjectAvoidanceCore.h:195-222](../../../../src/main/API-Src/ObjectAvoidanceCore.h#L195) );
- while enabled it writes those four LEDs on every 100 ms run, after the `enabled` check and with no Developer Mode
  check, so it overrides `isTriggered ( )` even outside Developer Mode ( same lines; gate at `:180-181` );
- the side lasers' XSHUT pins are not reserved ( only the down-laser pin is, through `altPinConflict` and the peripheral
  refusals ).

**Reach corrected against flight data.** The X1 wiki says Medium mode "usually reaches about 2.9 m" indoors. The
planned `vl53l1x-extended-reach` topic has the flight measurements for the down-laser looking at a floor ( from
`vl53l1x-althold-parity` log-5 ): 100 % valid up to 180 cm, 90 % at 180-190, 57 % at 190-200, about 0 above 210 cm. The
page states those numbers, and the known limitation says the 190 / 170 cm handover cannot be raised in FW 3.15.

**Claims on the page and their source:**

| Statement | Source |
|---|---|
| Threshold and avoidance distance in cm, `getRange` in mm; -1 / -100 | `threshold * 10`, `_avoidDist * 10` ( [X1Ranging.cpp:261, 334](../../../../src/main/API-Src/X1Ranging.cpp#L261) ); [X1Ranging.cpp:295-299](../../../../src/main/API-Src/X1Ranging.cpp#L295) |
| Family choice, both refused, what each call returns then | [X1Ranging.h](../../../../src/main/API/X1Ranging.h) class comment; `rangingFamilyLatch ( )` ( [main.cpp:584](../../../../src/main/main.cpp#L584) ); CLAUDE.md invariant |
| Medium mode, 45 ms budget, 50 ms samples, DOWN checked every 10 ms, side lasers one per 10 ms | `X1Ranging.h` class and `getRange` comments |
| `LASER_OUT_OF_RANGE` also after 160 ms without a sample | `X1Ranging.h` `getStatus` comment |
| Handover 190 / 170 cm, surface-change hold 2.5 s, return guard 50 cm and 25 cm for 2.5 s, 3 good samples | CLAUDE.md invariant; `X1RANGING_API_WIKI.md` ( `a81c180`, flight-validated ) |
| Same pins and refusal rules as XRanging, `GPIO_8` also loses BACK's LED | `X1Ranging.h` `initAlt` comment ( shared pin checks from `XRanging.cpp` ) |
| OA: steers only in Developer Mode, 100 ms, 0.5 s push / brake, 30 % / 20 %, 0.3 s release | the shared core `oaCoreRun < OA_CORE_X1RANGING >` ( [X1Ranging.cpp:347](../../../../src/main/API-Src/X1Ranging.cpp#L347) ); the `RC_ARRAY` path, task 6 |
| `enableOA` has no effect with XRanging too | `X1Ranging.h`; OA dispatch `switch ( rangingFamily ( ) )` ( [mw.cpp:546-551](../../../../src/main/mw.cpp#L546) ) |
| A laser error or boot failure is not retried until power-off | `laser_status_e` comments ( `XRanging.h` ); `vl53l1x-error-latch-recovery` ( planned ) |

## Task 8: RC-Interface.md and RxConfig.md ( 15 Nov 2026 )

**Result.** `wiki_check.py MagisV2.wiki/RC-Interface.md MagisV2.wiki/RxConfig.md`: 13 checked, 13 passed, 0 failed,
3 skipped ( the enum quotes ). Before: RC-Interface 8 blocks, 1 failed; RxConfig 8 blocks, 3 failed. The old
RC-Interface page gave `RcCommand_Get ( channel )` as -500..+500 ( it is 1500 + the command ) and called the array form
undefined behaviour ( fixed in 1.8.3 ). The old RxConfig page's main example configured channels in `Rx_ESP` mode,
where every `Receiver_Config_*` call is ignored, so it did nothing.

**Two of my own examples were unsafe on first draft and were changed before the check:** the one-channel override
triggered on AUX2 high, but AUX2 is the Developer Mode channel in `Rx_ESP` ( 1450-1550 ), so the code would have
switched itself off; it now holds yaw centred once airborne. The four-channel override could send a centred throttle
from ground idle in ALT HOLD; it is now gated on being airborne, and the page warns that a centred throttle in THROTTLE
mode is half throttle.

**Found while verifying, now on the pages:** `Rx_CAM` switches the ESP Wi-Fi off; `Rx_ELRS` sets no HEADFREE channel
( the stored one stays ) and forces the battery capacity to 800 mAh on every `plutoRxConfig ( )` run, so on every
settings save; the Developer Mode range is not quantised while the other ranges are; object avoidance replaces the
user's roll and pitch overrides ( same `RC_ARRAY` path, task 6 ).

**Claims on the pages and their source:**

| Statement | Source |
|---|---|
| Per-mode channel maps, Developer Mode channels, ESP Wi-Fi on / off | `Receiver_Mode ( )` ( [RxConfig.cpp:109-174](../../../../src/main/API-Src/RxConfig.cpp#L109) ) |
| `Receiver_Config_*` ignored with `Rx_ESP` / `Rx_CAM` | `AuxChangeEnable` false for those, each body under `if ( AuxChangeEnable )` ( [RxConfig.cpp:119, 190-266](../../../../src/main/API-Src/RxConfig.cpp#L190) ) |
| ELRS: USART1 CRSF, USART2 MSP, 885-2115 us, mincheck 1050, capacity 800 mAh, Wi-Fi off | [RxConfig.cpp:151-173](../../../../src/main/API-Src/RxConfig.cpp#L151) |
| Ranges limited to 900-2100 in 25 us steps; Developer Mode range stored as given | `CHANNEL_VALUE_TO_STEP` ( [rc_controls.h:106-110](../../../../src/main/io/rc_controls.h#L106) ) in `Receiver_Aux_Config`; `Receiver_Config_Mode_Dev` stores the raw values ( [RxConfig.cpp:255-266](../../../../src/main/API-Src/RxConfig.cpp#L255) ) |
| `STM_PB3_ESP_IO14_Init ( )` is called by the firmware | [main.cpp:203](../../../../src/main/main.cpp#L203); body drives PB3 for the ESP enable ( [RxConfig.cpp:273-305](../../../../src/main/API-Src/RxConfig.cpp#L273) ) |
| `RcData_Get ( channel )` is the received value; throttle can be rewritten | [RC-Interface.cpp:95-98](../../../../src/main/API-Src/RC-Interface.cpp#L95); throttle writers in `RcCommand_Set`, landing, altitude limit ( `RC_INTERFACE_API_WIKI.md` ) |
| `RcData_Get ( void )` returns NULL | [RC-Interface.cpp:82-88](../../../../src/main/API-Src/RC-Interface.cpp#L82); finding F5 |
| `RcCommand_Get` array and channel values, ranges, signs, curve, shared array, throttle notes | [RC-Interface.h](../../../../src/main/API/RC-Interface.h) comments; `RC_INTERFACE_API_WIKI.md` ( `rc-command-get-api` `e72d098`, bench log of 563 records ) |
| `RcCommand_Set`: clamp, four channels only, latch | [RC-Interface.cpp:125-170](../../../../src/main/API-Src/RC-Interface.cpp#L125) |
| Expiry max ( 250 ms, 2 periods ), cross-fade with the pilot alone at 200 counts | `USER_RC_HOLD_MIN_US`, `USER_RC_STICK_TRAVEL` ( `API-Utils.h` ); `RC-Interface.h` comment; CLAUDE.md invariant |
| Overrides act only in Developer Mode and are dropped on a stall | `applyUserRcOverride ( )` only inside the `runUserCode` branch ( [mw.cpp:1220](../../../../src/main/mw.cpp#L1220) ); reset before `onLoopFinish ( )` ( `:1224-1227` ) |
| Throttle override has no effect in any landing | `BMS_API_WIKI.md` ( `battery-soc-fix` ); `mw.cpp:1438` comment |
| `App_getAppHeading ( )`: last `MSP_APP_HEADING` value, 0 until sent | [serial_msp.cpp:1361-1363](../../../../src/main/io/serial_msp.cpp#L1361); [API-Utils.cpp:99](../../../../src/main/API-Src/API-Utils.cpp#L99) |
| `App_isArmSwitchOn ( )` is the ARM switch, not the armed state | `IS_RC_MODE_ACTIVE ( BOXARM )` ( [RC-Interface.cpp](../../../../src/main/API-Src/RC-Interface.cpp) ) |

## Task 9: Serial-IO.md ( 15 Nov 2026 )

**Result.** `wiki_check.py MagisV2.wiki/Serial-IO.md`: 8 checked, 8 passed, 0 failed, 1 skipped ( the enum quote ).
Before: 21 blocks, 6 failed. **No example now addresses an onboard device**: the old page wrote `PWR_MGMT_1`,
`GYRO_CONFIG` and `ACCEL_CONFIG` of the IMU at 0x68, on the bus the drone flies on. The I2C examples use an external
TMP102 at 0x48, and the page lists every onboard address. The old page also passed `1000000` as the SPI speed ( a
`uint16_t`, and not one of the codes, so it ran at 18 MHz ) and recommended the no-argument `SPI_Init ( )`, which does
not link. No in-repo wiki existed for this API: everything here was traced in the source.

**Found while verifying, now on the page and in [FINDINGS.md](FINDINGS.md):**

- A **second `Uart_Init ( )`** leaves the port NULL ( F6 extended ): `openSerialPort` returns NULL for a port already in
  use ( [serial.cpp:260-262](../../../../src/main/io/serial.cpp#L260) ), and `Uart_Init` stores it over the good pointer. UWB localisation
  opens UART2 at start-up too ( [main.cpp:587-590](../../../../src/main/main.cpp#L587) ).
- **Reading an empty UART buffer corrupts the count:** `uartRead` has no empty check
  ( [serial_uart.c:280-299](../../../../src/main/drivers/serial_uart.c#L280) ).
- **The multi-byte `I2C_Read` returns 1 / 0**, and lengths above 255 wrap ( F11 ).
- **`SPI_Init ( mode, ... )` never makes the chip-select pin an output:** `DISABLE_SPI` / `ENABLE_SPI` write
  `GPIOB` pin 5 ( [Serial-IO-SPI.cpp:30-31](../../../../src/main/API-Src/Serial-IO-SPI.cpp#L30) ), and nothing in the SPI path configures
  PB5. The page tells users to call `Peripheral_Init ( GPIO_13, OUTPUT )`. ( `target.h` names PB5 as a green LED, but
  that block is commented out on both targets; the live status LEDs are on GPIOC. )
- **The SPI register convention is reversed from many sensors:** reads clear bit 7, writes set it
  ( [Serial-IO-SPI.cpp:166, 205](../../../../src/main/API-Src/Serial-IO-SPI.cpp#L166) ).
- **`GPIO_15` is PB3** ( [Peripheral-GPIO.cpp:60](../../../../src/main/API-Src/Peripheral-GPIO.cpp#L60) ), the ESP Wi-Fi enable and
  `BACK`'s laser XSHUT: the task 6 question is answered in the code; whether it matters depends on the board wiring
  ( F12, to confirm on hardware ). Task 10 states it on the Peripherals page, task 16 in `PIN_MAP.md`.

**Claims on the page and their source:**

| Statement | Source |
|---|---|
| UART2 pins `GPIO_18` = PA2 ( TX ), `GPIO_17` = PA3 ( RX ) | [Peripheral-GPIO.cpp:62-63](../../../../src/main/API-Src/Peripheral-GPIO.cpp#L62); USART2 TX / RX are PA2 / PA3 |
| `Uart_Init` opens USART2 with `FUNCTION_UNIBUS`, 8N1 | [Serial-IO-Uart.cpp:69-73](../../../../src/main/API-Src/Serial-IO-Uart.cpp#L69); `SERIAL_NOT_INVERTED` = no inversion, 1 stop bit, no parity |
| Not available with ELRS, UWB, or on a second call | F6; `RxConfig.cpp:157`; `main.cpp:587-590`; `serial.cpp:260-262` |
| `Uart_rxBytesWaiting` / `Uart_txBytesFree` return `bool`, so "at least one" | [Serial-IO-Uart.cpp:112-124](../../../../src/main/API-Src/Serial-IO-Uart.cpp#L112) |
| `Uart_Read16 / 32` little-endian, no availability check | [Serial-IO-Uart.cpp:81-91](../../../../src/main/API-Src/Serial-IO-Uart.cpp#L81) |
| 256-byte TX buffer, no room check | `UART2_TX_BUFFER_SIZE` 256 ( [serial_uart.h:33](../../../../src/main/drivers/serial_uart.h#L33) ); CLAUDE.md invariant on `uartWrite ( )` |
| I2C1 on PB8 / PB9 | [PRIMUS_X2_v1/target.h:112-117](../../../../src/main/target/PRIMUS_X2_v1/target.h#L112) |
| Onboard addresses: 0x0C compass, 0x3C OLED, 0x40 INA219, 0x63 baro, 0x68 IMU, lasers 0x29 / 0x2A+n / 0x30 | `AK8963_MAG_I2C_ADDRESS` ( [compass_ak09916.cpp:59](../../../../src/main/drivers/compass_ak09916.cpp#L59) ), `OLED_address` ( [display_ug2864hsweg01.c:173](../../../../src/main/drivers/display_ug2864hsweg01.c#L173) ), `INA219_I2C_ADDRESS` ( [ina219.h:30](../../../../src/main/drivers/ina219.h#L30) ), `ICP_I2C_ID` ( [barometer_icp10111.cpp:67](../../../../src/main/drivers/barometer_icp10111.cpp#L67) ), `MPU_ADDRESS` ( [accgyro_mpu.cpp:51](../../../../src/main/drivers/accgyro_mpu.cpp#L51) ) used by the ICM-20948 path through `mpuConfiguration`; lasers from `XRANGING_API_WIKI.md` |
| I2C polled, a timeout resets the bus | `pluto-driver` skill section 4; `I2C_SHORT_TIMEOUT` / `I2C_LONG_TIMEOUT` ( [bus_i2c_stm32f30x.c:32-33](../../../../src/main/drivers/bus_i2c_stm32f30x.c#L32) ) |
| SPI2 pins `GPIO_5`-`7` = PB15 / PB14 / PB13; chip select `GPIO_13` = PB5; flash CS `GPIO_8` = PB12 | [Peripheral-GPIO.cpp:50-58](../../../../src/main/API-Src/Peripheral-GPIO.cpp#L50); SPI2 MOSI / MISO / SCK are PB15 / PB14 / PB13 on the STM32F303 |
| SPI2 started at power-up | `spiInit ( SPI2 )` ( [main.cpp:313](../../../../src/main/main.cpp#L313) ) |
| Speed codes and the 18 MHz fallback | `getPrescaler ( )` returns 0 ( `SPI_BaudRatePrescaler_2` ) for any other value ( [Serial-IO-SPI.cpp:44-80](../../../../src/main/API-Src/Serial-IO-SPI.cpp#L44) ); SPI2 runs from the 36 MHz APB1 clock |
| Each read / write blocks at least 350 us | `delayMicroseconds` 50 + 50 + 50 + 200 ( [Serial-IO-SPI.cpp:160-223](../../../../src/main/API-Src/Serial-IO-SPI.cpp#L160) ) |
| Every SPI call is a no-op after XRanging / X1Ranging start-up | `spiIsReleased ( SPI2 )` at the top of each ( same file ); `XRANGING_API_WIKI.md` |
| `SPI_Init ( )` without arguments does not link | F1; the self-test fixture reproduces it |

## Task 10: Motor.md and Peripherals.md ( 15 Nov 2026 )

**Result.** `wiki_check.py MagisV2.wiki/Motor.md MagisV2.wiki/Peripherals.md`: 10 checked, 10 passed, 0 failed,
2 skipped ( the enum quotes ); `table_check.py`: 0 mismatched rows. Before: Motor 7 blocks, 2 failed; Peripherals 15
blocks, 6 failed. The old Peripherals page used `GPIO_1` / `PWM_1` ( PA8, the PPM input ) in examples, told users to call
the internal `APIAdcInit ( )`, and showed `Peripheral_Write` at 1 kHz as a percentage ( any value of 5 or more is 100 %
there ). The old Motor page said `Motor_SetDir` on M5-M8 has "no effect" ( it writes through a NULL pointer ).

**Found while verifying, now on the pages and in [FINDINGS.md](FINDINGS.md):**

- **F2 is reachable from the firmware itself.** `MotroWakeUp ( )` runs every loop and calls the API's
  `Motor_SetDir ( M5 / M6 / M8 )` at an ARM switch-on whenever user code had called `Motor_Set ( M5..M8 )`. The page
  marks M5-M8 not available.
- **F13:** `Motor_Set ( M1 / M2 )` before `Motor_Init ( )` dereferences a NULL pointer.
- **M1 / M2 are never stopped by the firmware** ( `Motor_Set` writes the timer compare register directly, with no arming
  check ). The page says so first and shows a stop in `onLoopFinish ( )`.
- **The ADC is set up once, after `plutoInit ( )`:** `Peripheral_Init ( ADC_x )` only marks the pin; `APIAdcInit ( )`
  configures the converters at [main.cpp:583](../../../../src/main/main.cpp#L583). This confirms the scout's suspicion.
- **`GPIO_15` is PB3, the ESP Wi-Fi enable** ( task 9, F12 ). The pin table says it; whether it disturbs the Wi-Fi when
  the BACK laser uses it stays a hardware question.

**Claims on the pages and their source:**

| Statement | Source |
|---|---|
| M1: PWM PA6 TIM3 CH1, direction PB0 starting LOW; M2: PA7 TIM3 CH2, PB1 starting HIGH; motor stopped at init | [Motor.cpp:76-127](../../../../src/main/API-Src/Motor.cpp#L76) ( `pwmOutConfig ( ..., 0 )` ) |
| M1 / M2 PWM at 20 kHz | `PWM_BRUSHED_TIMER_MHZ` 20 ( [pwm_mapping.h:42](../../../../src/main/drivers/pwm_mapping.h#L42) ), `BRUSHED_MOTORS_PWM_RATE` 20000 ( [config.cpp:84](../../../../src/main/config/config.cpp#L84) ), `BRUSHED_MOTORS` on the target |
| `Motor_Set`: 1000-2000 clamp, linear duty, no arming check | [Motor.cpp:141-156](../../../../src/main/API-Src/Motor.cpp#L141) |
| M5-M8 write `motor_disarmed` while the ARM switch is off, reset to 1000 at the switch-on | [Motor.cpp:158-196](../../../../src/main/API-Src/Motor.cpp#L158) |
| `CLOCK_WISE` drives the direction pin LOW | `Motor_SetDir`: `direction` true ( `ANTICLOCK_WISE` = 1 ) is `digitalHi` ( [Motor.cpp:207-216](../../../../src/main/API-Src/Motor.cpp#L207) ) |
| GPIO number to port pin | `gpioMap [ ]` ( [Peripheral-GPIO.cpp:46-63](../../../../src/main/API-Src/Peripheral-GPIO.cpp#L46) ) |
| ADC names, converters and DMA channels | `adcPinMap [ ]` ( [Peripheral-ADC.cpp:63-73](../../../../src/main/API-Src/Peripheral-ADC.cpp#L63) ); each `ADCn_INm` checked against the STM32F303 pin functions |
| PWM names, timers, pins | `pwmConfig [ ]` ( [Peripheral-PWM.cpp:49-60](../../../../src/main/API-Src/Peripheral-PWM.cpp#L49) ) |
| RGB slots and their DMA channels; PA8 is the PPM input | [RGB-LED.h:110-124](../../../../src/main/API/RGB-LED.h#L110); `pwm_mapping.cpp:45-46` |
| Laser XSHUT / LED pins | `lasers [ ]` ( [XRanging.cpp:67-72](../../../../src/main/API-Src/XRanging.cpp#L67) ) |
| GPIO / ADC / PWM refused on the down-laser pin | `xRangingGpioClaimed`, `xRangingAdcClaimed`, `xRangingPwmClaimed` ( Peripheral-GPIO.cpp:91, Peripheral-ADC.cpp:77, Peripheral-PWM.cpp:72 ) |
| ADC: 12-bit, continuous, set up once after `plutoInit ( )`; a pin whose DMA channel is taken stays off | `ADC_Resolution_12b`, `ADC_ContinuousConvMode_Enable` ( Peripheral-ADC.cpp:208-209 ); `dmaClaim` ( `:82` ); `APIAdcInit ( )` at main.cpp:583 |
| PWM: 1 MHz timer, so pulse = value x 0.2 ms; starts at a 1500 us pulse; one set-up per pin; 16-bit period so at least 16 Hz | `PWM_TIMER_MHZ` 1 ( pwm_mapping.h:40 ); `map_i32 ( value, 0, 100, 0, 20000 )` ( Peripheral-PWM.cpp:113-117 ); `pwmOutConfig ( ..., 1500 )` and `isPwmInit` ( `:96-101` ); `uint16_t period` ( [pwm_output.cpp:95](../../../../src/main/drivers/pwm_output.cpp#L95) ) |
| Pins on one timer share a rate | `configTimeBase ( )` on every `pwmOutConfig ( )` ( [pwm_output.cpp:99](../../../../src/main/drivers/pwm_output.cpp#L99) ) |
| Servo: six 200 us steps, 1500 gives 1400 us | `map_i32 ( value, 1000, 2000, 5, 10 )` with integer division ( [maths.cpp:333-350](../../../../src/main/common/maths.cpp#L333) ) |

## Task 11: Status-LED.md and RGB-LED.md ( 15 Nov 2026 )

**Pages:** `MagisV2.wiki/Status-LED.md`, `MagisV2.wiki/RGB-LED.md`, both rewritten to [CONVENTIONS.md](CONVENTIONS.md).

**Check:** `wiki_check.py`: Status-LED 2 / 2 passed ( 2 skipped, header quotes ); RGB-LED 7 / 7 passed ( 4 skipped ).
`table_check.py`: 0 mismatched rows on both. No typographic dashes or quotes.

**What changed:**

- **Status-LED:** `STATUS` is documented as the firmware status pattern, not an LED. `OFF` stops it and blanks the
  three LEDs, `ON` resumes it, `TOGGLE` is "not available in FW 3.15" ( F4 ). The page now says that while the pattern
  runs it rewrites all three LEDs every 100-120 ms, so a user write lasts at most one step, and that switching
  Developer Mode off does not resume it. The old examples ( `Set_LED ( STATUS, TOGGLE )`, R / G / B writes with the
  pattern running ) did nothing visible; the new one stops the pattern in `onLoopStart ( )` and resumes it in
  `onLoopFinish ( )`. A table gives what the pattern shows.
- **RGB-LED:**
  - `RGB_Release ( )` is corrected: it does not free the pin, timer or DMA channel.
  - The broken `docs/WS2812_RGB.md` link is replaced by a slot table with the timer and everything each slot takes
    away, matching task 10's pin table.
  - New statements, each traced to the source:
    - the first `RGB_Init ( )` fixes the pin, and later calls reset the strip to `RGB_SYSTEM`;
    - `led_count` is clamped to 64;
    - `RGB_Control ( RGB_SYSTEM )` stops the animation;
    - nothing returns the strip at Developer Mode off;
    - `RGB_Show ( )` drops a frame while the previous one is still being sent;
    - `RGB_SetBrightness ( )` scales only colours set after it;
    - animation steps are rounded up to `plutoLoop ( )` ticks;
    - in `RGB_SYSTEM` the status display paints over user colours and animations.
  - The header's em dashes are not carried into the page. The old static-colour example ( statements outside a hook )
    is replaced by two complete examples.

**Claims on the pages and their source:**

| Statement | Source |
|---|---|
| `GREEN` / `BLUE` / `RED` ( 0 / 1 / 2 ) are the driver's `LEDg` / `LEDb` / `LEDr` ( 0 / 1 / 2 ), so `Set_LED` passes them straight through; `TOGGLE` ( 2 ) is `LED_TOGGLE` | [Status-LED.cpp:59](../../../../src/main/API-Src/Status-LED.cpp#L59); [light_led.h:171-179](../../../../src/main/drivers/light_led.h#L171); [light_led_stm32f30x.c:118-160](../../../../src/main/drivers/light_led_stm32f30x.c#L118) |
| `STATUS`: `OFF` clears `LedStatusState` and switches all three off, `ON` sets it, `TOGGLE` falls through to `return` | [Status-LED.cpp:43-55](../../../../src/main/API-Src/Status-LED.cpp#L43) |
| The pattern runs every loop, Developer Mode or not, and rewrites the LEDs every 100-120 ms while `LedStatusState` is true | `flightStatusIndicator ( )` called unconditionally at [mw.cpp:1350](../../../../src/main/mw.cpp#L1350); `delay_time` 100 / 120 and the `LedStatusState` gate at [runtime_config.cpp:98-104](../../../../src/main/config/runtime_config.cpp#L98); `FLIGHT_STATUS_INDICATOR` on all three targets |
| Pattern colours per state | `flightStatusIndicator ( )` cases ( [runtime_config.cpp:106-299](../../../../src/main/config/runtime_config.cpp#L106) ) |
| Nothing resumes the pattern at Developer Mode off | `LedStatusState` is written only in `Set_LED` ( grep: Status-LED.cpp:27/46/53, read at runtime_config.cpp:104 ) |
| RGB slot pins, timers, DMA channels; `RGB_8` is the default slot | `ws2811HwTable [ ]` and `WS2811_DEFAULT_SLOT` ( [light_ws2811strip_stm32f30x.c:59-82](../../../../src/main/drivers/light_ws2811strip_stm32f30x.c#L59) ) |
| What each slot takes away | slot table above against task 10's pin table ( this file ) and the conflict matrix in [WS2812_RGB.md](../../WS2812_RGB.md) section 3; M1 is TIM3 CH1 like `RGB_6` ( Motor.cpp:76-127 ); PA15 is FRONT's XSHUT ( XRanging.cpp:67-72 ); PB5 is the SPI API chip select ( [Serial-IO-SPI.cpp:30-31](../../../../src/main/API-Src/Serial-IO-SPI.cpp#L30) ) |
| The strip's DMA claim wins over the ADC when `RGB_Init ( )` is in `plutoInit ( )` | `dmaClaim ( cfg->dma, DMA_OWNER_LED_STRIP )` in `ws2811LedStripHardwareInit ( )` ( [light_ws2811strip_stm32f30x.c:163](../../../../src/main/drivers/light_ws2811strip_stm32f30x.c#L163) ), before `APIAdcInit ( )` at main.cpp:583 |
| `RGB_Init`: clamp to 64; pin fixed on the first call; every call resets count, brightness 100, animation, buffer, and `RGB_SYSTEM` | [RGB-LED.cpp:192-217](../../../../src/main/API-Src/RGB-LED.cpp#L192) |
| `RGB_Release`: animation off, brightness 100, buffer blank and sent, back to `RGB_SYSTEM`; frees nothing | [RGB-LED.cpp:219-238](../../../../src/main/API-Src/RGB-LED.cpp#L219); no `dmaRelease` or timer de-init anywhere in the RGB layer |
| `RGB_Control ( RGB_SYSTEM )` stops the animation; `RGB_USER` only sets the flag | [RGB-LED.cpp:243-252](../../../../src/main/API-Src/RGB-LED.cpp#L243) |
| The status display runs every loop after `RGB_Init`, only in `RGB_SYSTEM`, about every 100 ms; colours per state | `rgbSystemTick ( )` ( [RGB-LED.cpp:260-357](../../../../src/main/API-Src/RGB-LED.cpp#L260) ), called unconditionally at [mw.cpp:1558](../../../../src/main/mw.cpp#L1558) |
| Nothing returns the strip at Developer Mode off | `rgbUserControl` is written only in `RGB_Init`, `RGB_Release`, `RGB_Control` ( grep ) |
| Index at or above the count ignored; `FillColor` end clamped, start past the end ignored; HSV hue capped at 359 | [RGB-LED.cpp:365-421](../../../../src/main/API-Src/RGB-LED.cpp#L365) |
| Brightness is applied when a colour is written, so it scales only later colours | `RGB_SetColor` scales by `rgbBrightness` before storing ( `:371-375` ); `RGB_SetBrightness` only stores ( `:424-428` ) |
| `RGB_Show ( )` drops the frame while a transfer is in progress; a frame is about 2 ms | `rgbWriteDma ( )` returns when `ws2811LedDataTransferInProgress` ( [RGB-LED.cpp:87-89](../../../../src/main/API-Src/RGB-LED.cpp#L87) ); every frame is `WS2811_DMA_BUFFER_SIZE` = 64 x 24 + 42 = 1578 bit slots ( [light_ws2811strip.h:20-26](../../../../src/main/drivers/light_ws2811strip.h#L20) ) at 1.25 us = 1.97 ms |
| Animation steps only on calls, rounded up to whole ticks; `RGB_UpdateAnimation` sends the frame itself; it does not check the owner | `if ( ( now - animLastMs ) < animSpeedMs ) return; animLastMs = now;` ( [RGB-LED.cpp:484-490](../../../../src/main/API-Src/RGB-LED.cpp#L484) ); no `rgbUserControl` test in the function |
| `RGB_StartAnimation` clears and sends; `RGB_StopAnimation` clears and sends | [RGB-LED.cpp:453-477](../../../../src/main/API-Src/RGB-LED.cpp#L453) |

## Task 12: Debugging.md ( 15 Nov 2026 )

**Page:** `MagisV2.wiki/Debugging.md`, rewritten to [CONVENTIONS.md](CONVENTIONS.md) from
[DEBUGGING_API_WIKI.md](../../../API/DEBUGGING_API_WIKI.md) ( the in-repo page written and tested for API 1.8.9 ).

**Check:** `wiki_check.py`: 4 / 4 passed, 0 skipped. `table_check.py`: 0 mismatched rows. No typographic dashes.

**What changed:**

- The page now carries the precision table ( 0 to 4, 5 or more prints 4 ), the rounding rule, the range and the
  `overflow` / `not a number` / `infinite` texts, the 255-character cut and `NULL` as an empty text.
- **The ~130-byte per-tick budget** is on the page with the per-call byte table and two worked examples. It also says
  the budget was measured at the default 100 ms period ( new: print less at a shorter period ).
- How PlutoMonitor shows the output ( one line per call, no space between tag and number ).
- The float-with-no-precision pitfall is under Known limitations, and the print-in-`onLoopStart` / `onLoopFinish`
  repeat under Behaviour.
- `BlackBox_setVar ( )` is marked "not available in FW 3.15" with its examples removed ( F7 ). The old page's two
  BlackBox examples compiled only because they were wrapped in `#ifdef BLACKBOX`.
- A "Since API 1.8.9" note on the decimal functions. The in-repo page's audit IDs and File Architecture are left out.
- The old examples printed about 60 bytes per tick with no budget stated, and used four-space indent.

**Claims on the page and their source:**

| Statement | Source |
|---|---|
| Text frame = `$`, `D`, length, text, checksum: 4 + length | [Debugging.cpp:104-124](../../../../src/main/API-Src/Debugging.cpp#L104) |
| Tag and number frame: 5 + tag + digits ( the `\t` separator is the 5th byte ) | [Debugging.cpp:130-241](../../../../src/main/API-Src/Debugging.cpp#L130) ( double ), `:248-316` ( int ) |
| `Println` sends a second, 5-byte frame | header comment [Debugging.h](../../../../src/main/API/Debugging.h); `Monitor_Println` bodies `:340-362` |
| Precision clamped to 0..4 | `MONITOR_MAX_PRECISION` 4 ( `:53` ), `constrain ( precision, 0, 4 )` ( `:334`, `:358` ) |
| Cut at 255; the tag is shortened so the number fits | `MONITOR_MAX_PAYLOAD` 255 ( `:57` ); `boundedLength ( msg, 255 - 1 - numLen )` ( `:190`, `:286` ) |
| Rounding, range, NaN and infinity texts, `NULL` as empty | committed and tested in `1f8e78e` ( `monitor-print-double-precision` ); [DEBUGGING_API_WIKI.md](../../../API/DEBUGGING_API_WIKI.md) |
| 130-byte budget, 180 bytes disconnects | the flight invariant in `CLAUDE.md` and the same in-repo page ( measured in that topic ) |
| `BlackBox_setVar` does not exist in this build | declared only under `#ifdef BLACKBOX` ( Debugging.h ); `// #define BLACKBOX` in all three `target.h` ( PRIMUS_X2_v1:154, PRIMUS_V5:155, PRIMUSX2:155 ) |

## Task 13: Oled.md ( 15 Nov 2026 )

**Page:** `MagisV2.wiki/Oled.md`, rewritten to [CONVENTIONS.md](CONVENTIONS.md) with the framebuffer and timing facts
of [OLED_API_WIKI.md](../../../API/OLED_API_WIKI.md).

**Check:** `wiki_check.py`: 10 / 10 passed, 1 skipped ( the enum ). `table_check.py`: 0 mismatched rows. No
typographic dashes.

**What changed:**

- **Corrected:**
  - `Oled_Clear ( )` blanks the RAM frame and sends nothing. It is not "skipped while armed", which is
    `Oled_display_Clear ( )`.
  - `Oled_Update ( )` only queues the frame, and does nothing in `System` mode. The old `Oled_Is_Mode` / `Oled_Print`
    examples called it in `System` mode.
- **New "How the display works" section:**
  - RAM frame, 16 changed bytes per flight loop, about 0.25 s for a full screen;
  - why: the I2C bus is shared with the flight sensors;
  - the same armed or disarmed.
- **`Oled_Init ( )` must be in `plutoInit ( )`:** the display is started once right after it, only if enabled by then.
- **`Oled_Mode ( )`:**
  - starts from a blank frame in both modes;
  - nothing hands the display back at Developer Mode off.
- **`Oled_Print ( )`:**
  - documented as working, with a "Since API 1.8.10" note ( F9 );
  - its `System`-only rule and the `col` / `row` / `NULL` guards;
  - rows 1 to 6 are free in Developer Mode, while rows 2 to 5 are redrawn every 200 ms outside it;
  - the overwrite-not-erase rule ( pad texts ).
- **Other notes:**
  - The frame keeps its drawing until cleared ( start each frame with `Oled_Clear` ).
  - The display is switched off after repeated write failures.
  - There is a 2 s start-up page.
- **Advanced section for `Oled_display_Update ( )`:**
  - the explicit `io/oled_display.h` include;
  - the `static` buffer rule;
  - do not call `Oled_display_Clear ( )`.
- **Examples:**
  - all are complete and draw something visible;
  - the old HUD's `Oled_RollLine ( ... / 10 )` warned under `-Wconversion` and is now cast;
  - the heading is labelled whole degrees, not deg x10 ( the FC-Data page, task 5 ).

**Claims on the page and their source:**

| Statement | Source |
|---|---|
| `Oled_Init` only sets `OledEnable`; the display is started once after `plutoInit ( )`, only if set | [Oled.cpp:55-57](../../../../src/main/API-Src/Oled.cpp#L55); `if ( OledEnable ) OledStartUpInit ( );` at [main.cpp:578](../../../../src/main/main.cpp#L578) |
| `Oled_Clear` is RAM only; drawing calls only write `oledSimpleBuffer` | [Oled.cpp:83-156](../../../../src/main/API-Src/Oled.cpp#L83) |
| `Oled_Update` queues; nothing in `System` mode; a `NULL` buffer ignored | `Oled_display_Update ( )` ( [oled_display.c:523-529](../../../../src/main/io/oled_display.c#L523) ) |
| At most 16 changed bytes per loop, one run per loop, every loop whatever the mode or arming | `OLED_FLUSH_MAX_BYTES` 16 and `oledFlushStep ( )` ( [oled_display.c:129-196](../../../../src/main/io/oled_display.c#L129) ); `OledDisplayData ( )` called every loop from `userCode ( )` before the `runUserCode` test ( [mw.cpp:1186](../../../../src/main/mw.cpp#L1186) ) |
| Display switched off after 3 failures in a row or more than about 1 in 5 | `OLED_FAIL_COST` 4, `OLED_FAIL_LIMIT` 12, -1 per good run ( `:131-136`, `:181-187` ): break-even at a 20 % failure rate |
| `Oled_Mode` starts from a blank frame; `System` redraws on the next loop | `Oled_SystemMode ( )` / `Oled_UserMode ( )` ( [oled_display.c:463-484](../../../../src/main/io/oled_display.c#L463) ) |
| Status screen redraws every 200 ms; rows 2 to 5 only with Developer Mode off | `OledDisplayData ( )` ( `:432-455` ); `if ( ! devmode && ... )` around the `sysText ( ..., 2..5, ... )` calls ( `:393-421` ) |
| `Oled_Print`: `System` only, `NULL`, `col` > 20, `row` outside 1-6 ignored; overwrites cells; cut at the screen edge | [oled_display.c:536-545](../../../../src/main/io/oled_display.c#L536); `sysText ( )` ( `:204-216` ) |
| Pixels outside the screen are not drawn | `Oled_DrawPixel ( )` bounds check ( `:553-556` ) |
| `Oled_display_Update`, `Oled_DrawText`, `Oled_DrawTextColor`, `Oled_FillRect` signatures; C linkage | [oled_display.h:25-26, 126-214](../../../../src/main/io/oled_display.h#L126) |
| Joystick values are `int16_t` from `RcData_Get`, cast for `uint16_t` | [RC-Interface.h:63](../../../../src/main/API/RC-Interface.h#L63) |

## Task 14: Scheduler-Timer.md and Localisation.md ( 15 Nov 2026 )

**Pages:** `MagisV2.wiki/Scheduler-Timer.md`, `MagisV2.wiki/Localisation.md`, both rewritten to
[CONVENTIONS.md](CONVENTIONS.md).

**Check:** `wiki_check.py`: Scheduler-Timer 5 / 5 passed ( 1 skipped ), Localisation 2 / 2 passed ( 1 skipped ),
including the explicit `API/Localisation.h` include. `table_check.py`: 0 on both. No typographic dashes.

**What changed:**

- **`Interval`:**
  - The first `set ( )` fixes the period ( it was "self-configuring" ).
  - A non-repeat interval stays `true` until `reset ( )`.
  - `check ( )` on a never-set interval is always `true`.
  - Polling rounds periods up to `plutoLoop` ticks. The old "every 50 ms" example fired every 100 ms tick.
  - It must be a file-level or `static` object.
- **Scheduler:**
  - Add once in `plutoInit ( )`, not `onLoopStart ( )`, which the old examples used: it re-runs at stalls, so tasks
    were duplicated.
  - Each add allocates; the first run comes one interval after the add.
  - Missed runs are skipped on a fixed rate, at most once per call.
  - Tasks run in reverse add order; cancel removes the last-added copy.
- **New findings:**
  - **F14:** a 0 ms interval hangs the flight loop.
  - **F15:** cancel / clear from inside a task is a use-after-free.

  Both are on the page as a "not available" marker and Known limitations.
- **micros / millis:** wrap times are stated, with the subtract-to-compare rule.
- **Localisation:**
  - The "experimental" banner, made stronger than planned: `get ( )` is always 0, because the UWB reader is compiled
    out, not only the position control ( new **F16** ).
  - The explicit include.
  - `init ( UWB )` takes UART2 at 115200 and `GPIO_11` ( PA14, SWCLK ) after `plutoInit ( )`.
  - Pass only X / Y / Z to `get ( )`.
  - The fly-away example ( roll from a position error, no level-out, statements outside hooks ) is replaced by a
    read-only one using `WHYCON`, which takes no pins.

**Claims on the pages and their source:**

| Statement | Source |
|---|---|
| `Interval::set`: period latched on the first call ( `time == 0` ), clamped 1-5000; repeat restarts from now; non-repeat stays true | [Scheduler-Timer.cpp:127-143](../../../../src/main/API-Src/Scheduler-Timer.cpp#L127) |
| `reset` zeroes both fields; `check` compares against `loopTime`, 0 when never set, so always true | `:146-161` |
| A file-level `Interval` starts zeroed; a local one does not | no constructor in the class ( Scheduler-Timer.h ); file-level objects are in `.bss` |
| `Scheduler_Add`: `NULL` checks, `malloc` per node, `last_run_ms = millis ( )`, inserted at the head | [Scheduler-Timer.cpp:45-59](../../../../src/main/API-Src/Scheduler-Timer.cpp#L45) |
| `Execute_Scheduled`: runs once per call, the `do / while` skips missed periods on a fixed rate; one-shot nodes freed | `:96-124` |
| **F14:** `interval_ms` 0 makes `( now - last ) >= 0` always true for `uint32_t`, so the `do / while` that adds 0 never ends | `:103-106` |
| **F15:** a task that cancels or clears itself frees `cur`, then `cur->repeat` and `cur->next` are read | `:108-122` ( after `cur->task_function ( )` ) with `Scheduler_Cancel` `:62-80` |
| Cancel removes the first match from the head, the last added | `:66-79` |
| `micros` / `millis` are `uint32_t` | Scheduler-Timer.h; 2^32 us = 71.6 min, 2^32 ms = 49.7 days |
| **F16:** positions are never read: the UWB reader is under `#ifdef UWB`, and no target or Makefile defines `UWB` ( the enum value is not a macro ) | `uwbUpdate ( )` at [mw.cpp:220-252](../../../../src/main/mw.cpp#L220), its call `:1425-1428`; grep finds no `#define UWB` or `-DUWB` |
| `get ( )` returns `posX` / `posY` / `posZ`, written only by `uwbUpdate`, and has no `default` | [Localisation.cpp:14-25](../../../../src/main/API-Src/Localisation.cpp#L14); [posEstimate.cpp:63-65](../../../../src/main/flight/posEstimate.cpp#L63) |
| No position control: `isLocalisationOn` is never set | `false` at [API-Utils.cpp:132](../../../../src/main/API-Src/API-Utils.cpp#L132); `startLocalisation ( )` commented out ( Localisation.h, Localisation.cpp ) |
| `init ( UWB )` opens UART2 at 115200 and `GPIO_11` input pull-down after `plutoInit ( )` | [main.cpp:587-590](../../../../src/main/main.cpp#L587) |
| `Localisation.h` is not in `PlutoPilot.h` | PlutoPilot.h include list ( task 2 ) |

## Task 15: navigation and housekeeping ( 15 Nov 2026 )

**Pages:** `_Sidebar.md`, `Home.md`, `Introduction.md`, `_Footer.md` edited; `API-Utils.md` deleted ( not staged:
the user commits the wiki ). `_Header.md` needed no change.

**Check:**

- [link_check.py](link_check.py) on `MagisV2.wiki/`: **150 links in 23 pages, 0 dead, 0 written with `.md`**. Its
  self-test, a two-page fixture in the scratchpad, reported the dead page, the dead anchor and the `.md` link it
  contained, and passed a GitHub-slugged anchor with punctuation.
- `wiki_check.py` on `Introduction.md`: 1 / 1 passed. `table_check.py`: 0 on the four pages.
- **Sidebar against `src/main/API/*.h`:** every header with a page is listed. `API-Utils.h` is left out by decision
  ( decisions log ), and `PlutoPilot.h` lives at the repo root.

**What changed:**

- `API-Utils` and `Specifiers` are removed from the sidebar, Home and Introduction; no other page linked them.
- X1Ranging is added to all three, and `Oled.h` to the Introduction list, where it was missing.
- **Home:**
  - every description is corrected to what the page now covers;
  - Debugging is "printing to the app", no longer "debug output and OLED display";
  - Motor names only M1 / M2;
  - Localisation is marked experimental.
- **Introduction:**
  - the "loop frequencies ( 3.5-2000 Hz )" claim is now a 3.5 to 2000 ms period ( F8 );
  - "Regular Motors ( M5-M8 )" is replaced by the firmware-driven flight motors ( F2 );
  - Black Box is now "not built into FW 3.15" ( F7 );
  - Localisation is marked not working ( F16 );
  - the hook comments give the real run times ( settings re-read, RC link stall re-runs ), with a pointer to the
    PlutoPilot page;
  - the example is re-indented to two spaces;
  - all links use the page-name style.
- The version line "API 1.8.10 / FW 3.15.12" is on Home, Introduction, the sidebar and the footer.

## Task 16: PIN_MAP.md, TIMER_MAP.md and WS2812_RGB.md ( 15 Nov 2026 )

**Files:** `docs/fw-development-reference/PIN_MAP.md`, `TIMER_MAP.md`, `WS2812_RGB.md`. The only firmware-repo docs
this topic changes; no `fw-architecture-pipeline/` doc is touched, so there is no `PIPELINE_UPDATE.md`.

**Check:**

- A script compared the 18 `GPIO_n` rows of `PIN_MAP.md` §1 ( port pin, `ADC_n`, `PWM_n`, timer ) with the pin table
  of `MagisV2.wiki/Peripherals.md`: **18 / 18 identical**.
- `table_check.py`: 0 mismatched rows on the three maps.
- No em or en dash in any added line: the empty cells of the rows touched now use `-`. Older lines keep their style.

**Scope note:** `TIMER_MAP.md` was not in the task. Checking TIM3 showed the same drift: "user `PWM_8` ( CH1 ),
otherwise free", and "PB5 = TIM3_CH2" offered as a free channel, which is Motor `M2`'s. Five lines were fixed here
rather than in a new task ( decisions log ).

**Changes and their source:**

| Change | Source |
|---|---|
| PB3: ESP Wi-Fi enable, push-pull output at boot, LOW = on; no longer "free" | `STM_PB3_ESP_IO14_Init ( )` [RxConfig.cpp:291](../../../../src/main/API-Src/RxConfig.cpp#L291) ( `ESP_WiFi_ON` drives LOW, `:273-276` ), called at [main.cpp:203](../../../../src/main/main.cpp#L203) |
| PB5: SPI API chip select, set up by user code | `DISABLE_SPI` / `ENABLE_SPI` on GPIOB Pin_5 ( [Serial-IO-SPI.cpp:30-31](../../../../src/main/API-Src/Serial-IO-SPI.cpp#L30) ); no GPIO init of PB5 in that file |
| SPI API on PB13-PB15, PB12 is the flash CS | as above, and the task 9 / 10 pin tables |
| Motor API `M1`: PA6 TIM3_CH1, direction PB0 starting LOW; `M2`: PA7 TIM3_CH2, direction PB1 starting HIGH; 20 kHz | [Motor.cpp:92-122](../../../../src/main/API-Src/Motor.cpp#L92); 20 kHz from task 10 |
| Flight motors on TIM2 renamed "flight motor 1-4" | the rows were right; only the name clashed with the Motor API's `M1` / `M2` |
| TIM3 shared by `PWM_8`, `RGB_6` ( both CH1 on PB4 ) and `M1` ( CH1 on PA6 ) / `M2` ( CH2 ) | `pwmConfig [ ]` ( Peripheral-PWM.cpp:49-60 ), `ws2811HwTable [ ]` ( light_ws2811strip_stm32f30x.c:59-80 ), Motor.cpp:94 / 113 |
| Same-timer coupling also covers `PWM_6` ( TIM4 ) and `PWM_7` + `PWM_10` ( TIM8 ) | `pwmConfig [ ]`; task 10 |
| PA15 conflict: `RGB_8` and FRONT XSHUT, not "the LED strip" in general | `WS2811_DEFAULT_SLOT` 7 is used only through `RGB_Init`; `lasers [ ]` ( XRanging.cpp:67-72 ) |
| WS2812: 64 LEDs; DMA buffer 64 x 24 + 42 = 1578 bytes, sent in full ( about 2 ms ) | `RGB_MAX_LEDS` 64 ( RGB-LED.h ), [light_ws2811strip.h:20-26](../../../../src/main/drivers/light_ws2811strip.h#L20), `rgbWriteDma ( )` loops to `WS2811_LED_STRIP_LENGTH` |
| WS2812 idle note: a zero flag paints nothing | `leastSignificantBit ( 0 )` returns -1 as `uint16_t` ( [maths.cpp:276-285](../../../../src/main/common/maths.cpp#L276) ); `default: break;` then `RGB_Show ( )` of the old buffer ( [RGB-LED.cpp:353-357](../../../../src/main/API-Src/RGB-LED.cpp#L353) ) |

## Task 17: topic review ( 15-16 Nov 2026 )

**Review.** Three `pluto-reviewer` slices over the whole topic:

| Slice | Scope | Claims found correct | Findings |
|---|---|---|---|
| A | PlutoPilot, FC-Control, FC-Config, FC-Data, BMS, RC-Interface, RxConfig | about 95 | 1 blocking, 4 should-fix, 3 nits |
| B | XRanging, X1Ranging, Serial-IO, Motor, Peripherals, Status-LED, RGB-LED, PIN_MAP, TIMER_MAP, WS2812_RGB | about 160 | 1 blocking, 7 should-fix, 5 nits |
| C | Debugging, Oled, Scheduler-Timer, Localisation, the navigation pages, the FINDINGS evidence | about 85 | 2 blocking, 5 should-fix, 5 nits |

The two slices disagreed on the multi-byte `I2C_Read ( )` return value. The source settled it:
`return recived;` at `bus_i2c_stm32f30x.c:402` returns the byte count.

**Blocking findings and their fixes:**

- **`FlightMode_Set ( ATLTITUDEHOLD )` ( A1 ):** it starts altitude hold without resetting the target, and the RC path
  switches it off on every frame, so the drone can sink.
  - FC-Control now marks `ATLTITUDEHOLD` / `THROTTLE_MODE` "not available in FW 3.15" and says to switch ALT HOLD from
    the app. The `onLoopStart` example no longer sets it.
  - The override text now says the app's switch wins for one flight loop at each `plutoLoop` tick.
  - Logged as **F17**.
- **`I2C_Read` returns the byte count ( C1 ):**
  - the Serial-IO example tested `== 1`, which a 2-byte read never returns; it now tests `== 2`;
  - the text and the Known limitations are corrected;
  - F11 is rewritten.
- **F2 self-trigger only on `PRIMUSX2` ( C2 ):**
  - `MotroWakeUp ( )` is under `#ifdef PRIMUSX2`, so on PRIMUS_X2_v1 / PRIMUS_V5 `Motor_Set ( M5..M8 )` works and
    drives a flight-motor output of the disarmed drone.
  - User decision ( 16 Nov 2026 ): Motor.md documents M5-M8 as usable for projects that do not fly, at the user's own
    risk. It gets a new section with the output order, the ARM-switch rule, the keep-until-rewritten warning and a
    rover example that stops in `onLoopFinish ( )`.
  - `Motor_SetDir ( M5..M8 )` stays not available, and the PRIMUSX2 crash is under Known limitations.
  - F2 is rewritten.
- **RGB / ADC DMA order ( B1 ):** `Peripheral_Init ( ADC_n )` claims the channel when called, and the strip driver
  ignores a failed claim.
  - RGB-LED, Peripherals and WS2812_RGB now say to call `RGB_Init ( )` first or not to combine them.
  - Logged as **F18**.

**Should-fix and nits applied:**

- **Scheduler-Timer:**
  - `Scheduler_Add ( )` never reports out of memory ( **F19** );
  - the interval is counted from the add, so the first run after Developer Mode on can be immediate;
  - an interval no shorter than the `plutoLoop` period ( catch-up cost );
  - no `Scheduler_Add ( )` from a task ( F15 extended );
  - `Interval.set` / `reset` / `check` headings.
- **Motor:** `Motor_SetDir ( )` before `Motor_Init ( )` is also undefined ( F13 extended ); both function snippets now
  call `Motor_Init ( M1 )` in `plutoInit ( )`.
- **Peripherals:** a late ADC pin reads another pin's value or 0, not always 0.
- **XRanging:** `initAlt ( GPIO_8 )` leaves `BACK` with no LED.
- **Serial-IO:** the conditional `Uart_Init` marker becomes plain text, since its example is valid.
- **Oled:** `Oled_Number` range -32767 to 32767; pitch and roll lines at +90 are off screen.
- **FC-Config:**
  - the roll-lean and relative-yaw snippets act only when armed and above 50 cm;
  - the `onLoopFinish` restores are accepted as an exception ( CONVENTIONS ).
- **FC-Control:**
  - `setheadFreeModeHeading` is also replaced by an ALT HOLD switch-on in flight;
  - the `Command_DisArm` example acts only on an arm it saw happen;
  - a new Known limitations section.
- **RC-Interface:**
  - a `RcCommand_Get` after a `RcCommand_Set` in the same call returns the set value;
  - every landing shows the landing throttle.
- **RxConfig:** the PPM examples put ARM on AUX2, since AUX1 carries MAG / HEADFREE, which flips need.
- **Introduction / Home:**
  - "Translational PIDs" becomes the altitude PID;
  - X / Y estimates read 0 in FW 3.15;
  - "desired altitude", not positions;
  - "user-facing API header";
  - Motor described with M5-M8.
- **Maps:**
  - PIN_MAP: PA4 TIM3_CH2 and PB5 TIM8_CH3N; the `WS2812_RGB.md` link; the VL53L1X down-laser at 0x30; the ellipsis.
  - TIMER_MAP: the sharing note, the free-pin list, the TIM8 row and the advice line.
  - DMA_MAP: the free-pin row.
  - WS2812_RGB: the `dmaClaim` note and every relative link ( `../src` to `../../src`; the map links ).
- **FINDINGS:** F11 / F2 rewritten; F13 / F15 extended; F17-F19 added; the OPTIC_FLOW writer of `isLocalisationOn`
  noted.
- **`link_check.py`:** `.md` is stripped only from page-style links, so links to repo files are checked as files.

**Not confirmed, logged for later:** `pwmOutputPorts [ 12 ]` may have no bound check if nine or more user PWM outputs
are set up together with M1 / M2 and the flight motors ( reviewer B, not verified ).

**Checks after the fixes:**

- `wiki_check.py` on every changed page: FC-Control 11, FC-Config 15, RC-Interface 9, RxConfig 4, Serial-IO 8, RGB-LED
  7, Peripherals 6, XRanging 10, Scheduler-Timer 5, Oled 10, Introduction 1 and Motor 5 blocks, all passed.
- `link_check.py`: wiki 153 links, 0 dead; `docs/fw-development-reference/` 43 links, 0 dead.
- `table_check.py`: 0 everywhere.
- No em dash, en dash, typographic quote or ellipsis in any wiki page.
- `PlutoPilot.cpp` and `src/` unchanged.

**Focused re-review** of the two passages written during the fixes ( Motor M5-M8, `FlightMode_Set` ), by one more
`pluto-reviewer` run: 1 blocking, 2 should-fix and 1 nit, all applied.

- **Blocking, the rover example:** its `Motor_Set` calls sat inside `if ( ! App_isArmSwitchOn ( ) )`, so no call ever
  came with the ARM switch on, and the firmware's one stop ( `Motor.cpp:189-196` ) never fired. The outputs kept running
  through an ARM switch-on, and came back after a disarm. The example now writes 1000 on every tick while the switch is
  on.
- **The keep-the-value text** now names every writer ( user code, a call with the ARM switch on, configurator / CLI ),
  and says `onLoopFinish ( )` does not run at a disarm.
- **`FlightMode_Set`:**
  - the app's switch wins only on a tick that follows an RC frame, and never when `plutoLoop ( )` itself calls
    `FlightMode_Set ( )`;
  - the example moved from `onLoopStart ( )` to the start of `plutoLoop ( )`;
  - the `MAGHOLD` heading-drop consequence is stated;
  - the CONVENTIONS hook table now puts flight modes in `plutoLoop ( )`.

Recompiled: Motor 5 / 5, FC-Control 11 / 11 passed.
