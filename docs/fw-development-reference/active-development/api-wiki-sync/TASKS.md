# API wiki sync ( GitHub wiki ) - Tasks

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [FINDINGS](FINDINGS.md) · [CHANGES](CHANGES.md)

## Resume

Read this block first in a new session; read only the task it points to.

| | |
|---|---|
| **Current task** | **Closed - pending commit** ( 16 Nov 2026 ) |
| **Next step** | The user commits both repos ( messages in `.git/MAGISV2_COMMIT_MSG.txt` and `MagisV2.wiki/.git/MAGISV2_COMMIT_MSG.txt` ). The next `pluto-commit` or `pluto-grill` run fills in the hash. Follow-ups: `/pluto-grill fix` from FINDINGS ( F14, F17 first ); F12 hardware check. |
| **Open questions** | F12 ( `GPIO_15` / ESP enable on one net ) needs the schematic or a bench test; `pwmOutputPorts [ 12 ]` bound ( CHANGES task 17, unverified ). |
| **Blocked on** | nothing |
| **Last updated** | 2026-11-16 |

## Summary

| | |
|---|---|
| **Mode** | `docs` |
| **Goal** | Every page of the public GitHub wiki ( `MagisV2.wiki/` ) describes API 1.8.10 / FW 3.15.12 as the code behaves, with examples that compile and sit in the right `PlutoPilot` hook. |
| **Done when** | All 18 pages corrected, `X1Ranging` page added, `API-Utils` page and `Specifiers` links removed, every example compiled on `PRIMUS_X2_v1`, `pluto-reviewer` finds no wiki statement that disagrees with a header or `API-Src` line, every page carries "Verified against API 1.8.10 / FW 3.15.12". |
| **Target** | `PRIMUS_X2_v1` ( used only to compile the examples ) |
| **Analysis** | Drift list in [SCOUT.md](SCOUT.md): all 18 pages drift. High: FC-Control ( unsafe `Command_Land` / `Command_DisArm` in `onLoopFinish` ), FC-Config ( missing `DesiredAngle_setRelative`, wrong rate / angle semantics, wrong pitch sign ), XRanging ( mm vs cm, missing `initAlt` / `DOWN` / `getStatus` ), RC-Interface, RxConfig ( example is all no-ops ), Serial-IO ( I2C example writes the flight IMU, SPI speed codes ), Peripherals ( PWM duty, `APIAdcInit` ), Status-LED, Debugging ( 130-byte budget, precision 0..4 ). Medium: FC-Data, BMS, Oled, Motor, RGB-LED, Localisation. Low: Scheduler-Timer, navigation. |
| **Pipeline docs affected** | none. This topic touches the separate wiki repo and [`../PIN_MAP.md`](../../PIN_MAP.md) only. |
| **Order** | 1 → 2 → 3 → 4 → 5 → 6 → 7 → 8 → 9 → 10 → 11 → 12 → 13 → 14 → 15 → 16 → 17 → 18 |

## Index

| # | Title | Status | Depends on | Skills / agent |
|---|---|---|---|---|
| 1 | Page conventions and the example compile harness | done 2026-11-15 | - | pluto-build |
| 2 | PlutoPilot.md: the five hooks and what is included | done 2026-11-15 | 1 | pluto-rules |
| 3 | FC-Control.md: status, commands, head-free | done 2026-11-15 | 1, 2 | pluto-rules |
| 4 | FC-Config.md: PID, loop period, angles, rates, failsafe | done 2026-11-15 | 1, 2 | pluto-rules |
| 5 | FC-Data.md and BMS.md: units, signs, fuel gauge | done 2026-11-15 | 1 | pluto-rules |
| 6 | XRanging.md: cm units, `initAlt`, `DOWN`, `getStatus` | done 2026-11-15 | 1 | pluto-driver |
| 7 | X1Ranging.md: new page for the VL53L1X API | done 2026-11-15 | 6 | pluto-driver |
| 8 | RC-Interface.md and RxConfig.md: ranges, override expiry, receiver modes | done 2026-11-15 | 1 | pluto-rules |
| 9 | Serial-IO.md: SPI codes, safe I2C example, UART limits | done 2026-11-15 | 1 | pluto-driver |
| 10 | Motor.md and Peripherals.md: pins, PWM duty, servo steps | done 2026-11-15 | 1 | pluto-driver |
| 11 | Status-LED.md and RGB-LED.md: `STATUS` is not an LED, pin conflicts | done 2026-11-15 | 1, 10 | pluto-driver |
| 12 | Debugging.md: precision, byte budget, BlackBox | done 2026-11-15 | 1 | pluto-rules |
| 13 | Oled.md: framebuffer, queued frames, advanced note | done 2026-11-15 | 1 | pluto-rules |
| 14 | Scheduler-Timer.md and Localisation.md | done 2026-11-15 | 1 | pluto-rules |
| 15 | Navigation and housekeeping | done 2026-11-15 | 2-14 | - |
| 16 | PIN_MAP.md and WS2812_RGB.md correction | done 2026-11-15 | 10, 11 | pluto-driver |
| 17 | Topic review | done 2026-11-16 | 15, 16 | pluto-reviewer agent |
| 18 | Findings, CHANGES and close | done 2026-11-16 | 17 | pluto-commit |

Status values: `todo`, `in-progress`, `done YYYY-MM-DD`, `blocked (<why>)`, `dropped (<why>)`.
Serial numbers are never reused or renumbered.

**Closing tasks.** The `pluto-grill` build gate and hardware-validation tasks are **dropped**: no firmware changes,
and the per-page example compile ( task 1's harness, run inside every page task ) is the verification instead. The
architecture / pipeline task becomes task 16 ( `PIN_MAP.md` only; no `fw-architecture-pipeline/` doc changes, so no
`PIPELINE_UPDATE.md` ). The graph refresh is dropped: no code or structure changes for `graphify` to pick up.

## Tasks

### 1. Page conventions and the example compile harness

**Description.** Write `CONVENTIONS.md` in this folder fixing the shape every page follows, so the 13 page tasks do
not each invent one: the existing wiki shape is kept ( `# <Header>.h`, intro paragraph, `## Enumerations`,
`## Functions` with one hook-wrapped example each, `## Usage Example` ), plus a new `## Behaviour and limits`
section for the facts that now live in `docs/API/*_API_WIKI.md`, an optional `## Known limitations` section, and a
closing `Verified against API 1.8.9 / FW 3.15.11` line. Record the wording for "Since API 1.8.x" notes and for the
"not available in 3.15" marker ( [FINDINGS.md](FINDINGS.md) F1-F7 ). Then set up the compile harness: a copy of the
repo-root `PlutoPilot.cpp` saved to the scratchpad, a script that drops a candidate example into `PlutoPilot.cpp`,
builds `PRIMUS_X2_v1` with `pluto-build`, and restores the original file whatever the result.

- **Depends on:** -
- **Skills / agent:** `pluto-build` ( the driver script and `PROJECT=AltX` ), memory `wiki-example-hook-placement`
- **Files:** [CONVENTIONS.md](CONVENTIONS.md) ( new ), [PlutoPilot.cpp](../../../../PlutoPilot.cpp) ( restored, never committed changed )
- **Safety impact:** none directly; the conventions decide how safely every later example reads.
- **Done when:** `CONVENTIONS.md` exists; the harness builds an unmodified `PlutoPilot.cpp` and one deliberately
  broken example on `PRIMUS_X2_v1` ( pass, then fail ), and `git status` shows `PlutoPilot.cpp` unchanged afterwards.
- **Rollback:** delete `CONVENTIONS.md`; `git checkout PlutoPilot.cpp`.
- **Status:** done 2026-11-15
- **Result:** [CONVENTIONS.md](CONVENTIONS.md) written; [wiki_check.py](wiki_check.py) compiles and links each wiki `cpp` block against the built firmware without touching `PlutoPilot.cpp` ( SHA-1 unchanged ); self-test 7 / 7 as expected; today's wiki baseline 168 blocks, 55 fail; new finding F9 ( `Oled_Print` cannot link ). Details: [CHANGES.md](CHANGES.md).

### 2. PlutoPilot.md: the five hooks and what is included

**Description.** Correct the hook descriptions from `mw.cpp:1170-1233`: `plutoLoop ( )` runs once per
`userLoopFrequency` ( 100 ms by default ), not "repeatedly"; `onLoopStart ( )` and `onLoopFinish ( )` re-run after a
Wi-Fi stall, so anything one-shot placed there repeats; Developer Mode needs a live RX link; `plutoRxConfig ( )` runs
before `plutoInit ( )` and again on every settings save, profile change or settings reset ( task 1, CHANGES.md ).
Fix the "Included APIs"
list ( add `X1Ranging.h`; state that `API-Utils.h`, `Localisation.h` and `io/oled_display.h` are **not** included and
need an explicit include ), and note that `PlutoPilot.h` lives at the repo root. Replace the two bad examples: the
`Failsafe_enable ( LOW_BATTERY )` one ( that value has no reader ) and the `Command_TakeOff ( 150 )`-every-loop one.
Add a Known limitations note for `user-loop-timer-wrap` ( Developer Mode first entered after about 35.8 min of uptime
stalls `plutoLoop` ).

- **Depends on:** 1
- **Skills / agent:** `pluto-rules`, memory `wiki-example-hook-placement`
- **Files:** [PlutoPilot.md](../../../../MagisV2.wiki/PlutoPilot.md), [PlutoPilot.h](../../../../PlutoPilot.h), [mw.cpp](../../../../src/main/mw.cpp#L1170)
- **Safety impact:** high as documentation: this page teaches the hook semantics every other page's examples rely on.
- **Done when:** every hook statement matches a `mw.cpp` line cited in the page task's notes; both examples compile
  through the task 1 harness; the included-APIs list matches `PlutoPilot.h:27-52`.
- **Rollback:** `git -C MagisV2.wiki checkout PlutoPilot.md`.
- **Status:** done 2026-11-15
- **Result:** page rewritten; 8 / 8 blocks pass ( 1 skipped ), every hook statement traced to a source line in [CHANGES.md](CHANGES.md); `plutoRxConfig ( )` re-run and the 200 / 100 ms stall thresholds added; flight commands moved to the validated once-per-ARM pattern.

### 3. FC-Control.md: status, commands, head-free

**Description.** The highest-severity page: three examples put `Command_Land` or `Command_DisArm` in
`onLoopFinish ( )`, which re-runs on a Wi-Fi hiccup, so the drone lands or drops with the motors stopped. Replace
them with the `plutoLoop` once-per-arm pattern from `docs/API/FC_CONTROL_API_WIKI.md`. Document
`FlightStatus_Get ( )` properly ( priority order, the value is not the priority, `0xFFFF` after a disarm, the app is
sent LOW instead of INFLIGHT_LOW while armed, the 1.8.4 enum map ), the full `Command_TakeOff` contract ( queued,
arms when disarmed, throttle centred 1460-1540, once per arm, 3 s or ground-idle window, dropped after 2 s of
refused arming, refused during a landing ), `Command_Land` ( start throttle `1305 - landSpeed`, 1 s minimum before
touchdown, refused in a flip, cannot be interrupted ), `Command_Flip` ( needs MAG mode, ends within 8 s, never on
the floor ), and `setheadFreeModeHeading ( )` ( a heading value, applied at the next arm, `mw.cpp:634-638` ). Say
plainly that `FS_OK_TO_ARM` is not a full arming check.

- **Depends on:** 1, 2
- **Skills / agent:** `pluto-rules`
- **Files:** [FC-Control.md](../../../../MagisV2.wiki/FC-Control.md), [FC-Control.h](../../../../src/main/API/FC-Control.h), [FC_CONTROL_API_WIKI.md](../../../API/FC_CONTROL_API_WIKI.md)
- **Safety impact:** the page currently tells users to write code that stops the motors in flight. Highest priority.
- **Done when:** no example places an arm, take-off, land or disarm in `onLoopStart` / `onLoopFinish`; every
  contract line traces to `FC-Control.h` or `FC-Control-Command.cpp`; all examples compile.
- **Rollback:** `git -C MagisV2.wiki checkout FC-Control.md`.
- **Status:** done 2026-11-15
- **Result:** page rewritten; 11 / 11 blocks pass ( 3 enum quotes skipped ); no flight command left in `onLoopStart` / `onLoopFinish`; full command contracts, status table and `FlightMode_Set` persistence traced in [CHANGES.md](CHANGES.md); three new flight examples flagged there for the topic review.

### 4. FC-Config.md: PID, loop period, angles, rates, failsafe

**Description.** Ten behaviour drifts. Document `setUserLoopFrequency ( )` as it behaves ( the argument is the
`plutoLoop` period in ms: 100 gives 100 ms, i.e. 10 Hz; default 100 ms; range from the 3.5-2000 check in
`FC-Config-PID.cpp:88` ) and note the header still says Hz ( finding F8 ). Add the missing
`DesiredAngle_setRelative ( )` ( yaw only, turns add up, dropped only at a real dev-off or disarm ). Rewrite
`setDesiredAngle` / `getDesiredAngle`: yaw in whole degrees 0..359 and engages heading hold; roll / pitch in
deci-degrees, held ( not set every loop ), flown only airborne at 20 cm or more, in ANGLE mode, outside a flip or
landing, with the 0.6 s Developer Mode grace, under the `max_angle_inclination` clamp; **pitch greater than 0 is
nose-down** ( the page's "-100 = pitch forward" is backwards ); the getter returns the pre-clamp value and 0 for yaw.
Rewrite `get/setDesiredRate` as the stick-rate setting ( 0-100 roll / pitch, 0-255 yaw, default 75, held until
reboot, a roll / pitch call during a flip applied at the flip's exit ), not a rotation command. Correct
`setDefault_PIDProfile ( )` ( five exposed gain sets, RAM only, hidden gains untouched since 1.8.5 ),
`DesiredPosition_setRelative ( Z, cm )` ( the user name of the internal `setRelativeAltitude ( )`; relative to the **estimated** altitude, so calling it every loop walks the target away ),
and `Failsafe_enable / disable ( )` ( since 1.8.8 each call changes only the failsafe it names; all three on at boot;
no read-back; `LOW_BATTERY` has no reader ). Replace the fly-away usage example.

- **Depends on:** 1, 2
- **Skills / agent:** `pluto-rules`
- **Files:** [FC-Config.md](../../../../MagisV2.wiki/FC-Config.md), [FC-Config.h](../../../../src/main/API/FC-Config.h), [FC_CONFIG_API_WIKI.md](../../../API/FC_CONFIG_API_WIKI.md), [YAW_HEADING_API_WIKI.md](../../../API/YAW_HEADING_API_WIKI.md), [FAILSAFE_API_WIKI.md](../../../API/FAILSAFE_API_WIKI.md)
- **Safety impact:** high. A wrong pitch sign, a held tilt with no level-out, or a `Failsafe_enable` that users think
  disables the others all end in a crash.
- **Done when:** every function's units, hold behaviour and preconditions match the header and `API-Src`; the pitch
  sign is stated with the source line; examples compile and none holds a tilt without releasing it.
- **Rollback:** `git -C MagisV2.wiki checkout FC-Config.md`.
- **Status:** done 2026-11-15
- **Result:** page rewritten; 15 / 15 blocks pass ( 4 quotes skipped ); loop period in ms, pitch sign, tilt and heading rules, `DesiredAngle_setRelative`, rates, altitude goal profile with no ceiling, failsafe switches; harness upgraded to check blocks at global scope ( [CHANGES.md](CHANGES.md) ).

### 5. FC-Data.md and BMS.md: units, signs, fuel gauge

**Description.** FC-Data: add the full unit and sign tables from `docs/API/FC_DATA_API_WIKI.md` ( Rate in deg/s x10
with the yaw rate opposite to the heading since 1.8.2; Position in cm; Velocity in cm/s; acceleration in cm/s2, 980
at rest; `Net_Acc` in g squared x100; gyro in deg/s x10; magnetometer 0 if never calibrated; pressure in Pa;
barometer temperature in deg C x100 since 1.8.1 ), state that X / Y Position and Velocity come from UWB only while Z
is always valid, and flag that `Gyro` / `Mag` combined with `Net_Acc` is undefined. BMS: add the three missing
options `SoC`, `Warning_Level` and `Resistance`, correct `Estimated_Capacity` ( the charge at plug-in from the
resting voltage, not a running estimate ), note the roughly 1 s averaging of voltage and current, and replace the
hand-rolled percentage example with `SoC` / `Warning_Level`, including that a confirmed critical level lands the
drone and arming is refused until the pack is changed.

- **Depends on:** 1
- **Skills / agent:** `pluto-rules`
- **Files:** [FC-Data.md](../../../../MagisV2.wiki/FC-Data.md), [BMS.md](../../../../MagisV2.wiki/BMS.md), [FC-Data.h](../../../../src/main/API/FC-Data.h), [BMS.h](../../../../src/main/API/BMS.h), [FC_DATA_API_WIKI.md](../../../API/FC_DATA_API_WIKI.md), [BMS_API_WIKI.md](../../../API/BMS_API_WIKI.md)
- **Safety impact:** none in firmware; wrong units lead to user code that misjudges altitude or battery state.
- **Done when:** every option on both pages has a unit and a validity condition traced to the header; the BMS page
  documents the auto-land; examples compile.
- **Rollback:** `git -C MagisV2.wiki checkout FC-Data.md BMS.md`.
- **Status:** done 2026-11-15
- **Result:** both pages rewritten; 9 / 9 blocks pass ( 3 enum quotes skipped ); every option has a unit, range and validity; `SoC` / `Warning_Level` / `Resistance` and the auto-land documented; new finding F10 ( `Net_Acc` with gyro / mag ). Details: [CHANGES.md](CHANGES.md).

### 6. XRanging.md: cm units, `initAlt`, `DOWN`, `getStatus`

**Description.** The units are wrong by a factor of 10: `threshold` and `avoidDist` are centimetres
( `XRanging.cpp:377`, `:462` multiply by 10 ), not millimetres, so `init ( FRONT, 300 )` is 3 m. Correct the
`getRange ( )` return codes ( -100 no valid sample, -1 laser not enabled ), add `DOWN` to `laser_e`, add
`laser_status_e`, `initAlt ( gpio )` / `initAlt ( )` and `getStatus ( )`, and document the rules from
`docs/API/XRANGING_API_WIKI.md`: the latch after `plutoInit ( )` only, cached reads ( DOWN every 33 ms ), the
160 / 140 cm baro handover, the down-laser XSHUT restricted to GPIO_2 / 4 / 5 / 8 and reserved afterwards ( no LED,
`Peripheral_*`, ADC or PWM on it ), SPI2 and the flash off whenever XRanging is used, every fitted side laser must be
enabled or the lasers are refused with `BUS_BUSY`, the object-avoidance behaviour, and that using `XRanging` and
`X1Ranging` together refuses both.

- **Depends on:** 1
- **Skills / agent:** `pluto-driver`, `pluto-rules`
- **Files:** [XRanging.md](../../../../MagisV2.wiki/XRanging.md), [XRanging.h](../../../../src/main/API/XRanging.h), [XRANGING_API_WIKI.md](../../../API/XRANGING_API_WIKI.md)
- **Safety impact:** none in firmware; a user who reads mm sets avoidance thresholds 10x wrong.
- **Done when:** units, return codes and refusal rules all trace to `XRanging.cpp` or the header; the pin and SPI2
  reservations are stated; examples compile with `initAlt` in `plutoInit`.
- **Rollback:** `git -C MagisV2.wiki checkout XRanging.md`.
- **Status:** done 2026-11-15
- **Result:** page rewritten; 10 / 10 blocks pass ( 2 enum quotes skipped ); cm vs mm explicit, `initAlt` refusal rules, `getStatus`, OA acting only in Developer Mode, `isTriggered` false for OA-only lasers; constructor rule corrected on the PlutoPilot page. Details: [CHANGES.md](CHANGES.md).

### 7. X1Ranging.md: new page for the VL53L1X API

**Description.** New page, written from `src/main/API/X1Ranging.h` and `docs/API/X1RANGING_API_WIKI.md`, following
the task 6 page so the two read as a pair: the run-time family choice ( `rangingFamilyLatch ( )` after
`plutoInit ( )`, both APIs used means both refused, the shared device RAM area ), Quick Start, the laser / XSHUT / LED
table, the same function set as XRanging, the Medium timing budget ( 45 ms budget, 50 ms samples, 0.79 ms poll, 15 mm
minimum ), `LASER_OUT_OF_RANGE` after 160 ms, the 190 / 170 cm handover, the refusal rules, SPI2 off, wiring,
object avoidance, a "Moving from XRanging" section, and the 1.8.0 change note. Add a Known limitations note for
`vl53l1x-error-latch-recovery` ( an I2C error latches the laser out until a power cycle ) and for the
`vl53l1x-extended-reach` band limit.

- **Depends on:** 6
- **Skills / agent:** `pluto-driver`, `pluto-rules`
- **Files:** [X1Ranging.md](../../../../MagisV2.wiki/X1Ranging.md) ( new ), [X1Ranging.h](../../../../src/main/API/X1Ranging.h), [X1RANGING_API_WIKI.md](../../../API/X1RANGING_API_WIKI.md)
- **Safety impact:** none in firmware; the handover numbers and the family latch decide whether altitude hold works.
- **Done when:** every function in `X1Ranging.h` is documented; the page states the family latch and both refusal
  cases; examples compile; the page is reachable from the sidebar after task 15.
- **Rollback:** delete `MagisV2.wiki/X1Ranging.md`.
- **Status:** done 2026-11-15
- **Result:** new page; 10 / 10 blocks pass; every `X1Ranging.h` function documented, family choice and both-refused stated, reach given from flight data; XRanging page gained three shared OA facts. Sidebar link comes in task 15. Details: [CHANGES.md](CHANGES.md).

### 8. RC-Interface.md and RxConfig.md: ranges, override expiry, receiver modes

**Description.** RC-Interface: correct the `RcCommand_Get ( channel )` range ( `1500 + rcCommand`, so about
1000-2000, and 2600-3500 for `RC_THROTTLE`, 0 for channels after THROTTLE ), drop the stale "returns a local array,
undefined behaviour" note for the no-argument form ( fixed in 1.8.3: it is a file-static snapshot ), mark
`RcData_Get ( void )` as returning NULL in 3.15 ( finding F5 ), and document `RcCommand_Set ( )` fully: the
cross-fade against the pilot's sticks with full pilot authority at 200 counts of travel, the expiry after
`max ( 250 ms, 2 x user loop period )` unless re-asserted, channels after THROTTLE ignored, the inverted yaw read,
and that throttle can show a landing, failsafe or override value. RxConfig: state up front that
`Receiver_Config_*` calls are ignored in `Rx_ESP` / `Rx_CAM` mode ( `AuxChangeEnable` false ), so the page's main
example does nothing; document each mode's fixed default map ( ESP: ARM on AUX4 1300-2100, Dev Mode on AUX2
1450-1550 ), the `Rx_ELRS` side effects ( USART1 becomes CRSF, USART2 becomes MSP, mincheck 1050 / maxcheck 1900,
battery capacity forced to 800 mAh, ESP Wi-Fi off ), and that `STM_PB3_ESP_IO14_Init ( )` is already called by the
firmware at `main.cpp:203`.

- **Depends on:** 1
- **Skills / agent:** `pluto-rules`
- **Files:** [RC-Interface.md](../../../../MagisV2.wiki/RC-Interface.md), [RxConfig.md](../../../../MagisV2.wiki/RxConfig.md), [RC-Interface.h](../../../../src/main/API/RC-Interface.h), [RxConfig.h](../../../../src/main/API/RxConfig.h), [RC_INTERFACE_API_WIKI.md](../../../API/RC_INTERFACE_API_WIKI.md)
- **Safety impact:** high as documentation: the override expiry and the `rcData` / `rcDataPilot` split are flight
  invariants, and the RxConfig page currently implies a working channel remap that is a no-op.
- **Done when:** ranges, signs and the expiry match `RC-Interface.cpp`; the RxConfig page names the mode each config
  call needs; examples compile and re-assert the override every loop.
- **Rollback:** `git -C MagisV2.wiki checkout RC-Interface.md RxConfig.md`.
- **Status:** done 2026-11-15
- **Result:** both pages rewritten; 13 / 13 blocks pass ( 3 enum quotes skipped ); ranges and signs, the override rules, the per-mode channel map; `RcData_Get ( void )` marked not available; two unsafe first-draft examples caught and replaced. Details: [CHANGES.md](CHANGES.md).

### 9. Serial-IO.md: SPI codes, safe I2C example, UART limits

**Description.** Replace both I2C examples: they currently write to the onboard flight IMU at 0x68
( `PWR_MGMT_1`, then `GYRO_CONFIG` / `ACCEL_CONFIG` ). New examples use a clearly labelled external device address
plus a table of the onboard addresses that must never be written ( IMU, barometer, magnetometer, OLED ), each read
from its driver, with a note that the bus is polled and shared. Document SPI properly: `speed` is a kHz code from
the fixed set 140 / 281 / 562 / 1125 / 2250 / 4500 / 9000 / 18000 and anything else falls back to 18 MHz; the
chip-select is hardwired to PB5 ( GPIO_13 / PWM_9 / RGB_7 ) and `SPI_Init` does not configure it as an output; every
SPI call is dead when XRanging or X1Ranging is in use, because SPI2 and the flash are released; `SPI_Read` blocks at
least 350 us; and the no-argument `SPI_Init ( )` is not available in 3.15 ( finding F1 ), so only the 3-argument
form is documented. UART: the API is not available in `Rx_ELRS` mode ( USART2 is MSP, so `openSerialPort` returns
NULL and the calls dereference it, finding F6 ), `Uart_Init` must succeed first, `Uart_Read16/32` are little-endian
and do not check availability, `Uart_txBytesFree` means at least one byte, and the pins are PA2 / PA3
( GPIO_18 / 17, ADC_9 / 8 ). The multi-byte `I2C_Read` returns a count or 0, never negative, and length is capped
at 255.

- **Depends on:** 1
- **Skills / agent:** `pluto-driver`, `pluto-rules`
- **Files:** [Serial-IO.md](../../../../MagisV2.wiki/Serial-IO.md), [Serial-IO.h](../../../../src/main/API/Serial-IO.h)
- **Safety impact:** the current I2C example reconfigures the flight gyro and accelerometer on a shared, polled bus.
  Replacing it is the point of this task.
- **Done when:** no example addresses an onboard device; the reserved-address table cites a driver line per entry;
  the SPI speed codes and the ELRS UART limitation are stated; examples compile.
- **Rollback:** `git -C MagisV2.wiki checkout Serial-IO.md`.
- **Status:** done 2026-11-15
- **Result:** page rewritten; 8 / 8 blocks pass ( 1 skipped ); no example touches an onboard I2C device, the onboard address table is in, SPI speed codes / chip-select set-up / bit-7 convention / SPI2 release and the UART traps documented; F6 extended, F11 and F12 added. Details: [CHANGES.md](CHANGES.md).

### 10. Motor.md and Peripherals.md: pins, PWM duty, servo steps

**Description.** Motor: add the pin table ( M1 / M2 on PA6 / PA7 via TIM3_CH1 / CH2, direction pins PB0 / PB1, and
the TIM3_CH1 clash with PWM_8 and RGB_6 ), note that `Motor_Set ( M1 / M2 )` before `Motor_Init ( )` dereferences
NULL, that `Motor_SetDir` supports M1 and M2 only in 3.15 ( finding F2 ), and that M5-M8 map to `motor_disarmed` and
reset to 1000 on the arm edge. Peripherals: state that `Peripheral_Write ( PWM )` maps 0-100 onto a fixed
20000-count period, so it is a true percentage only at 50 Hz and at 1 kHz any value of 5 or more is 100%; that
`Servo_Write ( )` resolves to six 200 us steps, so 1500 outputs 1400 us; that `APIAdcInit ( )` is firmware-internal
and already runs after `plutoInit ( )`, so user code must not call it; that GPIO, ADC and PWM are silently refused on
the XRanging down-laser XSHUT pin; and add the pin-conflict table ( GPIO_1 / PWM_1 = PA8 is also the PPM input and
RGB_1, GPIO_10 / 11 are SWD, GPIO_17 / 18 are UART2, GPIO_5-8 are SPI2 and the flash, GPIO_13 is the SPI chip-select,
GPIO_15 = PB3 drives the ESP enable line ). Mark the header's internal externs and macros as internal. Verify in the
source whether channels on one timer share a frequency ( TIM4 PWM_2 / 3 / 6, TIM15 PWM_4 / 5, TIM8 PWM_7 / 10 ) and
whether PB3 is board-independent; document what is proven and label the rest as a limitation. Also settle which
port pin `GPIO_15` is: it is `BACK`'s XRanging XSHUT, and the scout tentatively tied it to PB3, the ESP enable line
( CHANGES.md task 6 ).

- **Depends on:** 1
- **Skills / agent:** `pluto-driver`, `pluto-rules`
- **Files:** [Motor.md](../../../../MagisV2.wiki/Motor.md), [Peripherals.md](../../../../MagisV2.wiki/Peripherals.md), [Motor.h](../../../../src/main/API/Motor.h), [Peripherals.h](../../../../src/main/API/Peripherals.h), [PIN_MAP.md](../../PIN_MAP.md)
- **Safety impact:** none in firmware; an example on PA8 fights the PPM input, and a wrong duty assumption can drive
  an attached device at 100%.
- **Done when:** every pin claim is traced to a driver line or `PIN_MAP.md`; the PWM duty and servo resolution are
  stated with their source lines; no example uses PA8 or calls `APIAdcInit`; examples compile. The timer-frequency
  and PB3 questions are answered from the source or recorded as assumptions.
- **Rollback:** `git -C MagisV2.wiki checkout Motor.md Peripherals.md`.
- **Status:** done 2026-11-15
- **Result:** both pages rewritten; 10 / 10 blocks pass, tables clean; the 18-pin table with every other user, PWM pulse = value x 0.2 ms, shared timer rates, servo steps, ADC set up only in `plutoInit`; F2 shown reachable from the firmware, F13 added. `GPIO_15` = PB3 settled in code ( F12 stays a hardware check ). Details: [CHANGES.md](CHANGES.md).

### 11. Status-LED.md and RGB-LED.md: `STATUS` is not an LED, pin conflicts

**Description.** Status-LED: rewrite around the fact that `STATUS` is not a colour channel but the firmware status
pattern: `OFF` stops the pattern and blanks R / G / B, `ON` resumes it, and `TOGGLE` does nothing in 3.15
( finding F4 ), so the page's two current examples do nothing. Add that while the pattern runs the firmware
overwrites user writes to RED / GREEN / BLUE, verified in the source before it is stated. RGB-LED: correct
`RGB_Release ( )` ( it stops the animation, blanks the strip, resets brightness to 100 and hands the strip back to
the system; it does not free the pin, timer or DMA ), fix the broken `docs/WS2812_RGB.md` link, and add the conflict
table from `WS2812_RGB.md`: RGB_6 clashes with `Motor_Init ( M1 / M2 )` and PWM_8, RGB_7 is the SPI chip-select plus
PWM_9 and the ADC DMA, RGB_8 ( PA15 ) is GPIO_16 / PWM_10 and the XRanging FRONT XSHUT. Add that `RGB_Init ( )` locks
the data pin on its first call, that `led_count` is clamped, and that `RGB_Control ( RGB_SYSTEM )` stops the
animation. Remove the em dashes the header text carries into the page.

- **Depends on:** 1, 10
- **Skills / agent:** `pluto-driver`, `pluto-rules`
- **Files:** [Status-LED.md](../../../../MagisV2.wiki/Status-LED.md), [RGB-LED.md](../../../../MagisV2.wiki/RGB-LED.md), [Status-LED.h](../../../../src/main/API/Status-LED.h), [RGB-LED.h](../../../../src/main/API/RGB-LED.h), [WS2812_RGB.md](../../WS2812_RGB.md)
- **Safety impact:** none in firmware; an RGB data pin on RGB_6 or RGB_8 silently takes a motor timer or a laser
  XSHUT, which does affect flight.
- **Done when:** the `STATUS` semantics and the overwrite behaviour are traced to source lines; every example does
  something observable; the conflict table matches `WS2812_RGB.md` and task 10's table; no em dashes; examples compile.
- **Rollback:** `git -C MagisV2.wiki checkout Status-LED.md RGB-LED.md`.
- **Status:** done 2026-11-15
- **Result:** Both pages rewritten; `STATUS` documented as the firmware pattern ( stop / resume, rewrites the LEDs every 100-120 ms, `TOGGLE` not available ), RGB slot table with timer and conflicts, `RGB_Release` corrected; 9 / 9 blocks pass, 0 table mismatches. `WS2812_RGB.md` drift added to task 16.

### 12. Debugging.md: precision, byte budget, BlackBox

**Description.** Merge `docs/API/DEBUGGING_API_WIKI.md` ( written for 1.8.9 ) into the page: precision clamped to
0..4 so 5 or more prints 4 digits, the rounding rule, the "overflow" / "not a number" / "infinite" texts, the
255-character cut, and above all **the roughly 130 bytes per `plutoLoop` tick budget with the app connected**, with
the per-call byte table, because an overrun corrupts the app's MSP replies and it disconnects. Add that `Println`
costs an extra 5-byte frame, that `Monitor_Print ( "v:", 3.7f )` selects the integer overload and prints 3, that NULL
counts as an empty string, that each call is its own line in the app, and that `onLoopStart` runs again after a
Wi-Fi stall ( so a header printed there repeats ). Mark the BlackBox section as not built in 3.15 ( finding F7:
`BLACKBOX` is commented out in all three targets ) and remove its examples.

- **Depends on:** 1
- **Skills / agent:** `pluto-rules`
- **Files:** [Debugging.md](../../../../MagisV2.wiki/Debugging.md), [Debugging.h](../../../../src/main/API/Debugging.h), [DEBUGGING_API_WIKI.md](../../../API/DEBUGGING_API_WIKI.md)
- **Safety impact:** none in firmware; the byte budget is a flight invariant for anyone logging from `plutoLoop`.
- **Done when:** the byte budget and the precision clamp are on the page with their numbers; the BlackBox section is
  marked not built; every example stays inside the budget and compiles.
- **Rollback:** `git -C MagisV2.wiki checkout Debugging.md`.
- **Status:** done 2026-11-15
- **Result:** Page rewritten from the in-repo 1.8.9 wiki: precision table, ~130-byte budget with the byte table, output format, BlackBox marked not available ( F7 ); 4 / 4 blocks pass, 0 table mismatches.

### 13. Oled.md: framebuffer, queued frames, advanced note

**Description.** Correct the two framebuffer-era claims: `Oled_Clear ( )` clears the RAM framebuffer and sends
nothing ( it is not "skipped while armed" ), and `Oled_Update ( )` only queues the frame, after which at most 16
changed bytes go out per loop so a full frame takes about 0.25 s; in `System` mode it does nothing, which makes the
page's current example a no-op. Merge the "Timing and flight safety" section from `docs/API/OLED_API_WIKI.md` ( the
I2C bus is shared with the IMU and polled, which is why the panel is never written byte by byte from the loop, and
the OLED switches off after repeated write failures ), note that `Oled_Mode ( )` starts from a blank frame, and add
a short `## Advanced` subsection for `Oled_display_Update ( )`: it needs an explicit `io/oled_display.h` include and
its buffers must be `static`, because only the frame is queued. Document `Oled_Print ( )` as working from API 1.8.10
( finding F9 was fixed in topic [oled-print-linkage](../oled-print-linkage/README.md) on 15 Nov 2026 ), with its
`System`-mode and `col` 0-20 / `row` 1-6 guards and the version note.

- **Depends on:** 1
- **Skills / agent:** `pluto-rules`
- **Files:** [Oled.md](../../../../MagisV2.wiki/Oled.md), [Oled.h](../../../../src/main/API/Oled.h), [OLED_API_WIKI.md](../../../API/OLED_API_WIKI.md)
- **Safety impact:** none in firmware; the "never write the panel from the loop" rule exists because doing so caused
  a 5 Hz flight twitch.
- **Done when:** the queue and flush behaviour are stated with the header lines; the Advanced note carries the
  static-buffer rule and the include; examples compile and none is a no-op.
- **Rollback:** `git -C MagisV2.wiki checkout Oled.md`.
- **Status:** done 2026-11-15
- **Result:** Page rewritten: RAM frame and 16-byte-per-loop flush, `Oled_Clear` / `Oled_Update` corrected, `Oled_Init` only in `plutoInit`, `Oled_Print` working since 1.8.10 with its guards, Advanced `static`-buffer note; 10 / 10 blocks pass.

### 14. Scheduler-Timer.md and Localisation.md

**Description.** Scheduler-Timer: correct `Interval::set ( )` ( it latches its period on the first call and later
values are ignored until `reset ( )`, so it is not "self-configuring" ), add that a non-repeat interval returns true
on every call once elapsed, that `check ( )` on an interval that was never set is always true, that `Scheduler_Add`
calls `malloc` per node and `onLoopStart` re-runs after a stall so adding there duplicates tasks, and that
`Execute_Scheduled` skips missed periods with the first run one interval after the add. Localisation: add the
"experimental, not functional in 3.15" banner ( `isLocalisationOn` is only set by the commented-out
`startLocalisation ( )`, so UWB position control never engages ), state that `Localisation.h` is not included by
`PlutoPilot.h` and needs an explicit include, that `init ( UWB )` claims UART2 at 115200 and GPIO_11, and that
`get ( )` has no default return. Replace the fly-away example with a read-only one.

- **Depends on:** 1
- **Skills / agent:** `pluto-rules`
- **Files:** [Scheduler-Timer.md](../../../../MagisV2.wiki/Scheduler-Timer.md), [Localisation.md](../../../../MagisV2.wiki/Localisation.md), [Scheduler-Timer.h](../../../../src/main/API/Scheduler-Timer.h), [Localisation.h](../../../../src/main/API/Localisation.h)
- **Safety impact:** none in firmware; the Localisation example currently commands roll from a position error with no
  level-out.
- **Done when:** both pages' statements trace to source lines; the Localisation banner and the UART2 / GPIO_11
  reservation are stated; examples compile, including the explicit include.
- **Rollback:** `git -C MagisV2.wiki checkout Scheduler-Timer.md Localisation.md`.
- **Status:** done 2026-11-15
- **Result:** Both pages rewritten; `Interval` latching and polling, scheduler add-once in `plutoInit`; new F14 ( 0 ms interval hangs the loop ), F15 ( cancel inside a task ), F16 ( Localisation never reads a position ); 7 / 7 blocks pass.

### 15. Navigation and housekeeping

**Description.** Last page task, so it reflects the final page set. Delete `API-Utils.md` ( an internal header: not
included by `PlutoPilot.h`, so its example cannot compile from user code, and three of its documented functions were
removed in `2a93948` ); its user-relevant constants are already covered where they act ( override travel and hold in
task 8, the 75 default rate in task 4, the 600 ms grace and 20 cm minimum in task 4 ). Remove the dead `Specifiers`
links from `_Sidebar.md`, `Home.md` and `Introduction.md` ( `Specifiers.h` was deleted in `2a93948` ). Add
`X1Ranging` to the sidebar, Home and Introduction, and add the missing `Oled` entry to the Introduction list. Fix the
stale "Debug output and OLED display" description, the `onLoopStart` "runs once" line, and the Black Box entry in
Introduction. Make the Introduction link style consistent with the sidebar so both render on GitHub, and put
"API 1.8.10 / FW 3.15.12" on Home, Introduction and `_Footer.md`. Then check every link on every page resolves to an
existing page.

- **Depends on:** 2-14
- **Skills / agent:** -
- **Files:** [Home.md](../../../../MagisV2.wiki/Home.md), [Introduction.md](../../../../MagisV2.wiki/Introduction.md), [_Sidebar.md](../../../../MagisV2.wiki/_Sidebar.md), [_Footer.md](../../../../MagisV2.wiki/_Footer.md), `MagisV2.wiki/API-Utils.md` ( deleted )
- **Safety impact:** none.
- **Done when:** a link check over all pages reports 0 dead targets; every header in `src/main/API/` that has a page
  is in the sidebar; `API-Utils.md` and every `Specifiers` reference are gone; the version line is on Home,
  Introduction and the footer.
- **Rollback:** `git -C MagisV2.wiki checkout .` for the navigation pages; `git -C MagisV2.wiki checkout API-Utils.md`
  to bring the page back.
- **Status:** done 2026-11-15
- **Result:** `API-Utils.md` deleted; `Specifiers` gone; X1Ranging ( and Oled ) in the navigation; Home and Introduction corrected ( Hz to ms, M5-M8, Black Box, hooks ); version line on Home, Introduction, sidebar, footer; new `link_check.py`: 150 links, 0 dead.

### 16. PIN_MAP.md and WS2812_RGB.md correction

**Description.** The only firmware-repo doc this topic changes. Correct the three drifts the cross-check found, from
the code, so the wiki pin tables written in tasks 10 and 11 and the standing reference agree: add the SPI chip-select
on PB5 ( `Serial-IO-SPI.cpp:30-31` ), add rows for PA6 / PA7 / PB0 / PB1 ( the M1 / M2 outputs and direction pins ),
and correct PB3 from free to the ESP enable line ( `RxConfig.cpp:271-305`, driven at `main.cpp:203` ). **Added in
task 11:** correct `WS2812_RGB.md` too: 64 LEDs, not 8 ( `RGB_MAX_LEDS`, `WS2811_LED_STRIP_LENGTH` ), a 1578-byte
buffer, the wrong "Idle note" ( a zero status flag paints nothing ), and RGB_6 sharing TIM3 CH1 with motor M1
( [FINDINGS.md](FINDINGS.md) reference-doc drift ). No `fw-architecture-pipeline/` doc changes, so there is no
`PIPELINE_UPDATE.md` for this topic.

- **Depends on:** 10, 11
- **Skills / agent:** `pluto-driver`
- **Files:** [PIN_MAP.md](../../PIN_MAP.md), [WS2812_RGB.md](../../WS2812_RGB.md)
- **Safety impact:** none directly; the map is what later topics reason about before claiming a pin.
- **Done when:** every added or changed row cites the source line that proves it, and the map agrees with the wiki
  pin tables from tasks 10 and 11.
- **Rollback:** `git checkout docs/fw-development-reference/PIN_MAP.md docs/fw-development-reference/WS2812_RGB.md`.
- **Status:** done 2026-11-15
- **Result:** PIN_MAP ( PB3 ESP enable, PB5 SPI CS, Motor `M1` / `M2` rows, TIM3 and PB5 conflicts ), WS2812_RGB ( 64 LEDs, idle note, slot conflicts ) and, found on the way, TIMER_MAP ( TIM3 owners ) corrected; the 18 pin rows match the wiki Peripherals table exactly.

### 17. Topic review

**Description.** Run the `pluto-reviewer` agent over the whole topic diff: the wiki repo's working tree and the
firmware repo's `PIN_MAP.md`, this folder's docs. The agent checks every wiki statement against the header or
`API-Src` line it claims, that no example places an arm, take-off, land, disarm or `Scheduler_Add` in
`onLoopStart` / `onLoopFinish`, that units and signs match the code, that the "not available in FW 3.15" markers
match [FINDINGS.md](FINDINGS.md), that no page contains an em dash, and that `PlutoPilot.cpp` is unchanged in the
firmware repo. Blocking findings are fixed and the affected page's examples recompiled before task 18.

- **Depends on:** 15, 16
- **Skills / agent:** `pluto-reviewer` agent ( always a subagent ), `pluto-rules` for the checklist
- **Files:** all pages in `MagisV2.wiki/`, [PIN_MAP.md](../../PIN_MAP.md), [TIMER_MAP.md](../../TIMER_MAP.md), [WS2812_RGB.md](../../WS2812_RGB.md)
- **Safety impact:** this is the gate that catches an unsafe example before it reaches the public wiki.
- **Done when:** the agent reports nothing blocking, every should-fix is applied or logged with a reason, and any
  page whose examples changed has been recompiled.
- **Rollback:** per page, as in the page tasks.
- **Status:** done 2026-11-16
- **Result:** Three review slices ( about 340 claims confirmed ) and one focused re-review; 5 blocking findings fixed ( `FlightMode_Set` ALT HOLD, `I2C_Read` count, M5-M8 per user decision, RGB / ADC DMA order, rover example ), all should-fix applied; F17-F19 new; every changed page recompiled, 0 dead links.

### 18. Findings, CHANGES and close

**Description.** Finish [FINDINGS.md](FINDINGS.md) with anything the page tasks added, and write `CHANGES.md` with a
per-page index of what changed and why. Update this folder's README to **Closed - pending commit** and add the topic
row to [`../README.md`](../README.md). Then prepare both commits with `pluto-commit`: in the firmware repo only
`PIN_MAP.md` and this topic folder are staged ( no version bump: no firmware or API change ), and the wiki repo's
changes are staged separately with their own drafted message. The user makes both commits.

- **Depends on:** 17
- **Skills / agent:** `pluto-commit`
- **Files:** [README.md](README.md), [FINDINGS.md](FINDINGS.md), [CHANGES.md](CHANGES.md), [`../README.md`](../README.md)
- **Safety impact:** none.
- **Done when:** both repos are staged with drafted messages, the topic row is in the index, `FW_Version` and
  `API_Version` are untouched, and `FINDINGS.md` lists every code defect with its proof line for the next `fix` topic.
- **Rollback:** `git reset` in either repo.
- **Status:** done 2026-11-16
- **Result:** Closed pending commit; CHANGELOG and CLAUDE.md lines added; firmware-repo docs and the wiki repo staged separately with drafted messages; no version bump.

## Decisions log

Newest last. One line each: `YYYY-MM-DD [decision|assumption|out-of-scope|risk] text`.

- 2026-11-15 [decision] Mode is `docs`: the wiki pages change, no firmware or header change. Code defects found while
  verifying go to FINDINGS.md for separate topics.
- 2026-11-15 [decision] Every one of the 18 pages is checked function by function against `src/main/API/` and the
  `API-Src` behaviour, not only the pages whose API changed.
- 2026-11-15 [decision] Add an `X1Ranging` page; remove the dead `Specifiers` links rather than writing a page for a
  deleted header.
- 2026-11-15 [decision] The GitHub wiki absorbs the `docs/API/*_API_WIKI.md` content and must be self-contained
  ( a wiki page cannot link to a repo-relative file ). `docs/API/` stays as the in-repo source that topics keep writing.
- 2026-11-15 [decision] Keep the existing wiki page shape and add a `Behaviour and limits` section per page for the
  `docs/API` facts. No `File Architecture` internals on the public wiki.
- 2026-11-15 [decision] Per-page version stamp "Verified against API 1.8.9 / FW 3.15.11" plus "Since API 1.8.x" notes
  on changed items; Home, Introduction and the footer state the current versions.
- 2026-11-15 [decision] Known limitations are named on the affected pages for the three unfixed Planned topics
  ( `user-loop-timer-wrap`, `vl53l1x-error-latch-recovery`, the untested relative-yaw cases ).
- 2026-11-15 [decision] `setUserLoopFrequency ( )` is documented as it behaves ( the argument is the `plutoLoop`
  period in ms ); the header comment saying Hz is finding F8, not fixed here.
- 2026-11-15 [decision] `API-Utils.md` is removed: an internal header whose example cannot compile from user code.
  `Localisation.md` stays with an "experimental, not functional in 3.15" banner.
- 2026-11-15 [decision] The six functions that do not work in 3.15.11 are marked "not available in 3.15" on their
  pages, with no example, and logged in FINDINGS.md. The testing team has not reported them, so they are
  code-reading findings, not confirmed field failures.
- 2026-11-15 [decision] Every page's examples are compiled in a scratch `PlutoPilot.cpp` on `PRIMUS_X2_v1` before the
  page task closes; `PlutoPilot.cpp` is restored each time. This replaces the dropped build-gate and
  hardware-validation closing tasks.
- 2026-11-15 [decision] The I2C example uses a labelled external device address plus a table of onboard addresses
  that must never be written. The current examples write to the flight IMU at 0x68.
- 2026-11-15 [decision] `PIN_MAP.md` is corrected in this topic ( task 16 ) so the wiki pin tables and the standing
  reference agree.
- 2026-11-15 [decision] The wiki clone stays untracked in the firmware repo and is not added to `.gitignore` or
  `.graphifyignore` ( user's choice ).
- 2026-11-15 [decision] `spec-to-code-compliance` is not installed in this tree, so the per-page check against the
  headers is done inside each page task and verified by the `pluto-reviewer` agent in task 17.
- 2026-11-15 [assumption] PB3 / GPIO_15 is the ESP Wi-Fi enable line on every target, because `RxConfig.cpp:271-305`
  drives it with no target gate. To be confirmed in task 10 or stated on the page as an assumption.
- 2026-11-15 [out-of-scope] Any firmware, header-comment or `docs/API/` change; `fw-architecture-pipeline/` docs;
  committing ( the user commits in both repos ).
- 2026-11-15 [decision] ( task 1 ) The example check does not swap examples into `PlutoPilot.cpp` as first planned:
  `wiki_check.py` compiles each block as its own translation unit with the exact firmware flags and links it against
  the built objects, so `PlutoPilot.cpp`, `Build/<TARGET>/` and the hex are never written, and link errors are caught.
  It lives in this folder ( not the scratchpad ) so later sessions and later wiki updates can rerun it.
- 2026-11-15 [decision] ( task 1 ) Wiki examples use two-space indent and the spaced-paren style of the
  `PlutoPilot.cpp` template and `docs/API/`, replacing the old wiki's four spaces.
- 2026-11-15 [decision] ( task 1 ) Wiki examples must compile warning-free under the firmware flags, since users build
  with them; a block meant to show a warning carries `<!-- wiki-check: warn-ok -->`.
- 2026-11-15 [decision] ( task 1 ) `Oled_Print ( )` joins the "not available in FW 3.15" set ( finding F9, found by the
  harness ): seven functions are now marked, not six.
- 2026-11-15 [decision] F9 is fixed instead of documented as broken: the user chose the one-line `extern "C"` fix in
  `API/Oled.h`, done in its own topic [oled-print-linkage](../oled-print-linkage/README.md) ( FW 3.15.12, API 1.8.10 )
  and committed before the wiki work resumes. Back to six functions marked "not available". The other two defects of
  the same class, F1 ( `SPI_Init ( )` ) and F5 ( `RcData_Get ( void )` ), stay open and stay marked.
- 2026-11-15 [decision] The wiki targets the firmware as committed in `9e6f555`: FW 3.15.12 / API 1.8.10 ( the
  `Oled_Print` linkage fix landed between task 1 and task 2 ). Every version stamp and "Since" note uses those numbers.
- 2026-11-15 [decision] ( task 2 ) The PlutoPilot page does not name `API-Utils.h` at all, although the task text said
  to state it is not included: its page is being removed as firmware-internal ( task 15 ), and naming it invites use.
  Only the two user headers that need an explicit include are listed ( `Localisation.h`, `io/oled_display.h` ).
- 2026-11-15 [decision] ( task 2 ) The PlutoPilot page reuses the flight-validated take-off Quick Start of
  `FC_CONTROL_API_WIKI.md` unchanged, so the FC-Control page ( task 3 ) and this page show one pattern.
- 2026-11-15 [decision] ( task 3 ) `FlightMode_Set ( )` is documented as overriding the app's switch while Developer
  Mode is on and as kept until power-off, with no call to hand the mode back: `isUserFlightModeSet` is never cleared.
  Stated as behaviour, not as a defect; a candidate for a later `feature` ( a release call ).
- 2026-11-15 [decision] ( task 3 ) Calling `Command_Land` on every tick while a condition holds is shown as the safe
  pattern: the call is ignored while the landing runs and once disarmed, and it retries after a flip.
- 2026-11-15 [decision] ( task 4 ) `wiki_check.py` now checks each block at global scope in place of the firmware's
  `PlutoPilot.o`, with empty stubs for the missing hooks, instead of wrapping it in a namespace. It catches file-level
  name collisions with firmware globals, which the namespace hid. Earlier pages re-checked: all pass.
- 2026-11-15 [decision] ( task 4 ) Untested yaw and roll cases are listed by name under FC-Config's Known limitations
  ( negative roll tilt, pilot yaw during a relative turn, dev-off mid-turn, stall mid-turn, HEADFREE with a target ).
- 2026-11-15 [decision] ( task 5 ) Formulas, driver names and file architecture from the in-repo FC-Data and BMS wikis
  stay out of the public pages; units, ranges, signs and validity go in. The magnetometer is documented as a
  relative value, since its scale is unconfirmed.
- 2026-11-15 [decision] ( task 6 ) The constructor rule on the wiki is "a file-level object whose constructor has to run
  code is left zeroed", not "any class with a constructor": `XRanging` is constant-initialized ( `.data`, no
  `.init_array` ).
- 2026-11-15 [decision] ( task 7 ) The X1Ranging page gives the down-laser reach from flight data ( all valid to
  180 cm, almost none above 210 cm ) instead of the in-repo wiki's "about 2.9 m indoors", which does not hold for a
  down-laser over a floor.
- 2026-11-15 [decision] ( task 8 ) Override examples are gated on being armed and above 50 cm, so none can act on the
  floor; the four-channel example also requires ALT HOLD, since a centred throttle in THROTTLE mode climbs.
- 2026-11-15 [decision] ( task 9 ) The I2C examples use a TMP102 at 0x48 as the external device: a common part at an
  address no onboard device uses, so a copied example cannot reach the flight sensors.
- 2026-11-15 [assumption] ( task 9 ) The UART is 8N1, from `SERIAL_NOT_INVERTED` and the serial layer's defaults; not
  measured on hardware.
- 2026-11-15 [decision] ( task 10 ) M5-M8 are marked "not available in FW 3.15" on the Motor page as a whole, not only
  for `Motor_SetDir`: `Motor_Set ( M5..M8 )` arms the NULL write that the firmware performs at the next ARM switch-on.
- 2026-11-15 [assumption] ( task 10 ) The ADC full scale is 3.3 V ( VDDA ); not measured on the board.
- 2026-11-15 [risk] The wiki is public. A page that documents a defect as a limitation is read by customers, so each
  such line states the version and the workaround rather than the cause.
- 2026-11-15 [decision] ( task 11 ) `LedStatusState` is listed as internal on the Status-LED page ( read it, change it
  only through `Set_LED ( STATUS, ... )` ): writing it skips the LED blanking that `Set_LED ( STATUS, OFF )` does.
- 2026-11-15 [decision] ( task 11 ) The `WS2812_RGB.md` drift found while checking the RGB page ( 8 vs 64 LEDs, the
  idle note, RGB_6 vs M1 ) is fixed in task 16 with `PIN_MAP.md`, rather than in a new task: same kind of
  reference-map correction, same files' owner.
- 2026-11-15 [decision] ( task 12 ) The Debugging page adds one statement the in-repo page does not make: the
  130-byte budget was measured at the default 100 ms period, so a shorter `plutoLoop` period should print less. No
  number is given for faster periods, since none was measured.
- 2026-11-15 [decision] ( task 14 ) The Localisation banner says "does not work in FW 3.15" and `get ( )` "always 0",
  stronger than the planned "position control never engages": the UWB reader is compiled out ( F16 ). The example
  uses `WHYCON` so a copied example takes no pins.
- 2026-11-15 [decision] ( task 14 ) Scheduler tasks are added in `plutoInit ( )` on the page, the hook-table rule from
  CONVENTIONS ( no `Scheduler_Add` in `onLoopStart` ). The first run after Developer Mode goes on is then immediate
  if the interval has passed, which is harmless for the examples shown.
- 2026-11-15 [decision] ( task 15 ) The Introduction's architecture text is kept, with only the claims that are wrong
  in 3.15 corrected ( loop Hz, M5-M8, Black Box, Localisation, hook timing ). It is an overview page, not an API
  page, so it carries the version line but no per-function checks.
- 2026-11-15 [decision] ( task 16 ) `TIMER_MAP.md` is corrected in this task too ( five lines, TIM3 owners and the
  `PWM_8` / `PWM_9` notes ): the same drift the wiki pin tables exposed, and CLAUDE.md asks for the maps to stay in
  sync. Logged in FINDINGS and CHANGES rather than as a new task.
- 2026-11-16 [decision] ( task 17, user ) Motor `M5`-`M8` are documented as usable: the flight-motor outputs for
  projects that do not fly ( a rover, a test stand ), unsafe on a drone that flies, at the user's own risk. This
  replaces the task 10 decision to mark them not available, whose reason ( the firmware's own NULL write ) holds only on
  the PRIMUSX2 build. `Motor_SetDir ( M5..M8 )` stays not available.
- 2026-11-16 [decision] ( task 17 ) `FlightMode_Set ( ATLTITUDEHOLD / THROTTLE_MODE )` is marked not available ( F17 ),
  and the task 3 wording "applied again on every loop, the pilot's switch has no effect" is corrected: the app's
  switch wins for one flight loop at each `plutoLoop` tick.
- 2026-11-16 [decision] ( task 17 ) Restoring a setting the user's own code changed may sit in `onLoopFinish ( )` when
  harmless during a stall ( CONVENTIONS exception ), rather than rewriting the FC-Config examples.
- 2026-11-16 [decision] ( task 18 ) No firmware build and no version bump: no `src/` or Makefile change. The firmware
  repo commit holds the topic folder, the four maps, CHANGELOG, CLAUDE.md and the index; the wiki repo is staged
  separately. `__pycache__` in the topic folder is not staged.
