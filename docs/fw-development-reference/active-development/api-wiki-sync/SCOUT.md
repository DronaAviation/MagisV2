# Scout briefs ( api-wiki-sync )

Two `pluto-scout` agents ran in parallel on 15 Nov 2026, each owning half of the wiki pages. Both briefs are kept
verbatim so no later session repeats the reconnaissance. Line numbers are as of FW 3.15.11 / API 1.8.9 ( `1f8e78e` ).

Both scouts noted the knowledge graph was stale ( built at `c344387` ) and checked every finding in the source.

---

## Scout A: PlutoPilot, API-Utils, Specifiers, FC-Control, FC-Config, FC-Data, BMS, XRanging ( + X1Ranging ), Localisation

Graph: stale (built c344387b, HEAD 1f8e78e). It did not matter here: every finding below was checked in the source.

Subsystems: user API layer (`src/main/API`, `src/main/API-Src`), the userCode dispatch in `mw.cpp`. Pipeline docs: none change. The wiki is a separate git repo (`MagisV2.wiki/.git`, last commit 47e451d) and shows as untracked in the firmware repo.
Key code:
- `PlutoPilot.h:27-52` (repo root, not `src/main/API/`): umbrella header plus the 5 hook declarations
- `src/main/mw.cpp:1170-1233` `userCode()`: dev-mode gate, the plutoLoop period, onLoopStart/onLoopFinish re-run
- `src/main/API-Src/FC-Config-PID.cpp:85-93` `setUserLoopFrequency`
- `src/main/flight/altitudehold.cpp:1506` `setRelativeAltitude`: `AltHold = EstAlt + altitude`
- `src/main/API-Src/XRanging.cpp:377,462`: threshold and avoidDist x10, so the units are cm
- `src/main/mw.cpp:634-638`: the head-free heading is applied at arm
- `src/main/main.cpp:587-590`: `Localisation.init(UWB)` takes UART2 at 115200 and GPIO_11
Mode: docs

Current behaviour: the wiki pages date from before API 1.4.0. The headers and `docs/API/*_API_WIKI.md` are current for 1.8.9. No page in my half mentions an API or FW version (checked: my 8 pages, Home, _Header, _Footer).

### Drift summary
| page | header | #undoc | #stale/removed | #behaviour | sev | merge from |
|---|---|---|---|---|---|---|
| PlutoPilot.md | PlutoPilot.h (root) | 1 | 0 | 5 | high | FC_CONTROL Pitfalls, mw.cpp |
| API-Utils.md | API-Utils.h | ~25 | 4 | 2 | med | none (internal header) |
| Specifiers (missing) | none (deleted) | - | 1 | - | low | - |
| FC-Control.md | FC-Control.h | 0 | 0 | 8 | **high (unsafe examples)** | FC_CONTROL, FAILSAFE |
| FC-Config.md | FC-Config.h | 1 | 0 | 10 | **high** | FC_CONFIG, YAW_HEADING, FAILSAFE |
| FC-Data.md | FC-Data.h | 0 | 0 | 4 | med | FC_DATA |
| BMS.md | BMS.h | 3 | 0 | 3 | med | BMS |
| XRanging.md | XRanging.h | 6 | 1 | 4 | **high** | XRANGING |
| (X1Ranging, none) | X1Ranging.h | whole API | - | - | high | X1RANGING |
| Localisation.md | Localisation.h | 0 | 0 | 3 | med | none |

### PlutoPilot.md
- Wiki :48-50 says plutoLoop runs "repeatedly". It actually runs once per `userLoopFrequency` (100 ms by default, mw.cpp:1190-1199). The other loop passes only re-apply flight modes and the heading.
- Wiki :47 says onLoopStart runs "once when Developer Mode is activated". It runs again after every Wi-Fi hiccup, together with onLoopFinish (mw.cpp:1237). Dev mode also needs a live RX link (mw.cpp:1177).
- Wiki :115-124 "Included APIs" is missing X1Ranging.h (PlutoPilot.h:40). It does not say that Localisation.h and API-Utils.h are **not** included.
- Wiki :86-88 example uses `Failsafe_enable(LOW_BATTERY)`. That value has no effect (FC-Config.h `failsafe_e`).
- Wiki :149-152 example calls `Command_TakeOff(150)` on every loop while armed. This is the pattern FC_CONTROL Pitfalls forbids, and more than 3 s after arming it sets no height.
- Minor: the onLoopFinish example `Oled_Clear(); Oled_Mode(System)` only clears RAM (Oled.h:69 says RAM only).

### API-Utils.md
- API-Utils.h is an **internal** header. No public header and not PlutoPilot.h includes it, so the wiki :55-64 example does not compile from user code.
- Wiki :43-45 `getGPIOport/getGPIOpin/getGPIOclock(unibus_e)` were removed. They are commented out at API-Utils.h ~:200, and `unibus_e` was deleted in 2a93948.
- Wiki :39-40 says `resetUser()` "Resets all user variables". It only writes `rcData[AUX2]=1200` (API-Utils.cpp:256-258).
- Undocumented: the override constants (USER_RC_AUTHORITY_DEFAULT, USER_RC_STICK_TRAVEL 200, USER_RC_HOLD_MIN_US 250000, USER_ANGLE_LINK_GRACE_MS 600, USER_ANGLE_MIN_ALT_CM 20, STOCK_ROLL_PITCH_RATE 75), `rangingFamily_e`, the xRanging* and x1Ranging* hooks, `landingInProgress`, `userAngleTarget`, `userRollPitchRate`, and about 20 externs.

### Specifiers
- `Specifiers.h` held only `unibus_e { Pin1..Pin20 }`. It was deleted in 2a93948 (5 May 2026, "remove deprecated GPIO functions"). Nothing in src/ references it.
- **No page is warranted.** Remove the link at `_Sidebar.md:9`. It is also referenced at `Home.md:14` and `Introduction.md:141`, which are the other scout's pages.

### FC-Control.md
- **Unsafe:** the wiki :106-108, :140-142 and :178-181 examples put `Command_Land` and `Command_DisArm` in onLoopFinish. onLoopFinish also runs on every Wi-Fi hiccup, so the drone starts landing, or **drops with the motors stopped** (`Command_DisArm` -> `mwDisarm()`, FC-Control-Command.cpp:197-201). Calling `Command_DisArm` right after the queued Land also defeats the landing.
- Wiki :96-98 and :162-167 call `Command_TakeOff`/`Command_Arm` in onLoopStart. The re-runs re-arm and climb after a landing. The pattern to use is FC_CONTROL Quick Start (plutoLoop, once per ARM switch-on).
- Wiki :63-67 treats FS_OK_TO_ARM as an arming gate. It is "not a full check" (FC_CONTROL :180-184).
- Wiki :51 FlightStatus_Get: missing the priority order (the value is not the priority), the 0xFFFF value after a disarm, and that the app is sent LOW instead of INFLIGHT_LOW while armed (FC-Control.h:56-74).
- Wiki :91-93 Command_TakeOff: missing that it is queued, arms when disarmed, needs the throttle centred (1460-1540), runs once per arm, has a 3 s / ground-idle window, is dropped after 2 s of refused arming, and is ignored during a landing (FC-Control.h:110-127).
- Wiki :101-103 Command_Land: missing the 1305-landSpeed start throttle, the 1 s minimum before touchdown, that it is ignored in a flip, and that it cannot be interrupted (:130-141).
- Wiki :112-122 Command_Flip: missing that it needs MAG mode and that it ends within 8 s. The example flips the moment the drone is armed, i.e. on the floor.
- Wiki :146-151 says `setheadFreeModeHeading(0)` "locks current forward". 0 is a heading value, and it is applied only at the **next arm** (mw.cpp:634-638).
- Merge from the docs: Taking off, When a take-off is dropped, Landing, Flight status table, Pitfalls, Changes 1.7.0 and 1.8.4.

### FC-Config.md
- **Missing:** `DesiredAngle_setRelative` (FC-Config.h:182; yaw only, turns add up, Wi-Fi hiccup pitfall).
- Wiki :57-64 `setUserLoopFrequency(100.0f)` is described as a "100 Hz main control loop". In the code it sets the **plutoLoop period**: the argument x1000 goes into a us period (FC-Config-PID.cpp:90-92, mw.cpp:1192, default 100000). So 100 gives 100 ms (10 Hz). The header :54-60 says Hz, so **header and code disagree**.
- Wiki :110 `setDesiredAngle` says "deci-degrees, set every loop". Yaw is in whole degrees 0..359 (wrapped) and engages heading hold. One call holds the angle.
- Wiki :115 says "-100 = pitch forward". Pitch >0 is nose-down, i.e. forward (YAW_HEADING :113-115), so -100 pitches **back**.
- Missing roll/pitch tilt conditions: ANGLE mode, airborne >= 20 cm, the 0.6 s grace, not during a flip or landing, dropped at dev-off and disarm.
- Wiki :207-209 usage example holds a 15 deg / 10 deg tilt every loop with no level-out. The drone flies away.
- Wiki :100 `getDesiredAngle` omits that the value is before the max-angle clamp and that it returns 0 for yaw.
- Wiki :120 and :130-135 `get/setDesiredRate` are described as a "rotational rate". The value is the stick-rate setting (0-100 roll/pitch, 0-255 yaw, default 75), held until reboot, applied at the end of a flip. `setDesiredRate(AG_YAW,200)` does not rotate anything.
- Wiki :159-163 says "climb 20 cm from current target". It is measured from the **estimated** altitude (altitudehold.cpp:1507). In plutoLoop that call raises the target again every tick. FC_CONTROL says to use it once airborne.
- Wiki :91 says `setDefault_PIDProfile` "resets every profile". It resets the 5 listed gain sets only, in RAM, and leaves the internal gains alone (:80-98).
- Wiki :168-175 and :201 `Failsafe_enable(LOW_BATTERY)` does nothing. Since 1.8.8 each call changes only the failsafe it names, all three are on at boot, and there is no read-back.
- Merge from the docs: FC_CONFIG (rate value, where it acts, how long it holds, flips, saving, 1.8.7), YAW_HEADING (turn flow, stick, end, tilt, 1.6.0/1.8.6), FAILSAFE (table, switch behaviour, 1.8.8).

### FC-Data.md
- Wiki :90-91 says Velocity "needs GPS/optical flow/UWB". In the code X/Y Position and Velocity come from UWB only, and Z is always valid (FC-Data.h:105-107).
- Missing units: Rate in deg/s x10 with signs (yaw rate is opposite to the heading, 1.8.2); Position in cm; Velocity in cm/s; accel in cm/s2 (980 at rest); Net_Acc in g2 x100 (squared); gyro in deg/s x10; mag is 0 if never calibrated; Gyro/Mag with Net_Acc is undefined behaviour.
- Wiki :67 pressure "100*mbar" is correct but should say Pa. Temperature is the chip temperature, rounded (1.8.1).
- Merge the FC_DATA tables, sign table, examples, and Changes 1.8.1/1.8.2.

### BMS.md
- Missing `SoC`, `Warning_Level` and `Resistance` (BMS.h:44-46).
- Wiki :18 calls `Estimated_Capacity` "estimated capacity". It is the charge at plug-in, from the resting voltage.
- Voltage and Current are averaged over ~1 s.
- Wiki :50-60 computes the percentage by hand and warns at 20%. Use `SoC` and `Warning_Level` instead (low at about 15%, critical at about 5%, critical lands the drone).
- Merge from the docs: Options table with "valid from", how the level is decided, auto-land, timing notes, no current sensing, Changes 1.4.0.

### XRanging.md
- **Units wrong:** wiki :6-9, :39 and :72 say threshold and avoidDist are in mm. The code is in **cm** (XRanging.cpp:377 `threshold*10`, :462 `_avoidDist*10`). `init(FRONT,300)` means 3 m.
- Wiki :49 says getRange returns `-1` when there is no reading. It returns -100 for no valid sample and -1 for a laser that is not enabled (XRanging.cpp:410-415).
- Wiki :15-21 `laser_e` lacks `DOWN`. Wiki :71 gives the defaults as `-1`; the header :146 has `DOWN`.
- Missing: `laser_status_e`, `initAlt(gpio)` and `initAlt()`, `getStatus`, the plutoInit-only latch, cached reads (DOWN every 33 ms), the 160/140 cm handover, refusal rules (GPIO_2/4/5/8), pin reservation, SPI2 off, "enable every fitted laser", OA behaviour (stick at 30%, 0.5 s push/brake), and that XRanging and X1Ranging together are both refused.

### X1Ranging (new page needed)
- Needs: family choice, Quick Start, laser/XSHUT/LED table, functions (same as XRanging), Medium mode / 45 ms budget / 50 ms samples, LASER_OUT_OF_RANGE after 160 ms, 190/170 cm handover, refusal rules, SPI2 off, wiring, OA, "Moving from XRanging", Changes 1.8.0.
- Add it to `_Sidebar.md` and to the PlutoPilot included-APIs list.

### Localisation.md
- `Localisation.h` is not in PlutoPilot.h, so the wiki :36-56 example does not compile without an explicit include.
- `get()` has no default return (Localisation.cpp:14-33).
- `isLocalisationOn` is set only by the commented-out `startLocalisation()`, so position control never engages.
- `init(UWB)` in plutoInit takes UART2 and GPIO_11 (main.cpp:587-590).
- The example sets roll from the error with `abs`, with no level-out.

Constraints / invariants that apply: CLAUDE.md flight invariants (onLoopStart/Finish re-run; LOW_BATTERY has no reader; tilt rules; laser family latch; handovers 160/140 and 190/170); memory rule: no em dashes in any file; wiki-example hook placement memory.
Resource conflicts: none (docs only).
Already answered by the code: all of the above. Version is not shown anywhere on these pages. PlutoPilot.h is at the repo root.
Questions for the user:
1. Should `setUserLoopFrequency` be documented as it behaves (period in ms) or as the header says (Hz)? Do you want a separate fix topic for it?
2. API-Utils page: delete it, or keep it marked "internal, not for user code"?
3. Should Localisation stay in the sidebar, given it is not included and not functional, or be marked experimental?
4. Should pages show "API 1.8.9 / FW 3.15.11", e.g. in `_Footer.md`?
5. Should the wiki copy the full `docs/API` content, or a summary that links to it? The wiki cannot link to repo-relative files.
6. Remove the Specifiers sidebar line (and Home/Introduction mentions, other scout)?
Doc drift: header vs code: FC-Config.h:54-60 says "Hz", the code uses a ms period. XRanging.h :71/:139 is correct in cm.
Prior work: closed topics takeoff-land-command-fix, failsafe-enable-api, set-desired-angle-roll-pitch, set-desired-rate-api, relative-yaw-api, estimate-get-rate-api, baro-temperature-api, tof-single-pipeline, vl53l1x-xranging-api, battery fuel gauge (cbd1c72). None of them covers the GitHub wiki.
Suggested skills: spec-to-code-compliance (per-page check of the wiki against the headers), pluto-rules (hook placement and invariants in examples), pluto-reviewer agent (topic review), pluto-commit (the wiki repo is committed separately by the user).
Pipeline docs to update at the end: none. `active-development/README.md` gets an index row for api-wiki-sync.

---

## Scout B: RC-Interface, RxConfig, Serial-IO, Motor, Peripherals, Status-LED, RGB-LED, Debugging, Oled, Scheduler-Timer, navigation pages

Graph: stale (built c344387b, HEAD 1f8e78e; the only difference is the Monitor_Print commit). I checked everything by reading the code directly.

Subsystems: public API (the headers in `src/main/API/` and their code in `src/main/API-Src/`), PIN_MAP, WS2812_RGB -> pipeline docs: none changed (this topic edits only the wiki)
Mode: docs
Current behaviour: The wiki was last committed on 21 Jul 2026. Since then five API commits touch these pages: 868409a (RC override cross-fade/expiry), c495d6a (OLED framebuffer), edfbd61 (XRanging releases SPI2), e72d098 (RcCommand_Get), 1f8e78e (Monitor_Print). None of my pages mention an API or FW version.

### Summary
| Page | Header | #undoc | #stale | #behaviour | Sev | Merge from |
|---|---|---|---|---|---|---|
| RC-Interface | RC-Interface.h | 0 | 1 | 5 | high | RC_INTERFACE_API_WIKI.md |
| RxConfig | RxConfig.h | 0 | 0 | 4 | high | none |
| Serial-IO | Serial-IO.h | 0 | 1 | 8 | high | none |
| Motor | Motor.h | 0 | 0 | 4 | med | none |
| Peripherals | Peripherals.h | 8 internal externs | 0 | 6 | high | none (PIN_MAP table) |
| Status-LED | Status-LED.h | 0 | 0 | 3 | high | none |
| RGB-LED | RGB-LED.h | 0 | 1 | 3 | med | WS2812_RGB.md (conflict matrix) |
| Debugging | Debugging.h | 0 | 0 | 6 | high | DEBUGGING_API_WIKI.md |
| Oled | Oled.h | 0 | 2 | 5 | med | OLED_API_WIKI.md |
| Scheduler-Timer | Scheduler-Timer.h | 0 | 0 | 5 | low-med | none |

### RC-Interface.md
- The wiki says `RcCommand_Get(ch)` returns about -500..+500 (RC-Interface.md:48,53,120). The code returns `1500 + rcCommand`, so about 1000-2000, and 0 for channels after THROTTLE (RC-Interface.cpp:116-123). For `RC_THROTTLE` it returns 2600-3500 (RC-Interface.h:97-98).
- The wiki calls `RcCommand_Get(void)` a pointer to a local array, undefined behaviour (md:58-62). That is stale since 1.8.3: it is now a static four-element array that stays valid (RC-Interface.cpp:100-112).
- `RcData_Get(void)` actually returns NULL (RC_INTERFACE wiki "Known gaps"). The wiki only says "UB".
- `RcCommand_Set` (md:64-85) is missing:
  - the cross-fade against the pilot's sticks, with full pilot authority at `USER_RC_STICK_TRAVEL` 200 (API-Utils.h:59);
  - the expiry after `max(250 ms, 2x user period)`;
  - that channels after THROTTLE are ignored (RC-Interface.cpp:151).
- Also missing: yaw reads inverted, and throttle can show a landing, failsafe or override value.
- Merge from the in-repo wiki: the channel table, the element/range table, the signs, the shared-array note, "Known gaps", and the three examples.

### RxConfig.md
- **The page's main example does nothing.** In Rx_ESP / Rx_CAM mode, `AuxChangeEnable=false` (RxConfig.cpp:119), so every `Receiver_Config_*` call is ignored (RxConfig.cpp:190-266). The example at md:118-134 uses Rx_ESP, so it is all no-ops. Config calls only work after `Receiver_Mode(Rx_PPM|Rx_ELRS)`.
- The fixed default map for each mode is not documented (RxConfig.cpp:122-130, 140-146, 165-170). For ESP: ARM on AUX4 1300-2100, Dev Mode on AUX2 1450-1550.
- The side effects of Rx_ELRS are not documented (RxConfig.cpp:155-174):
  - USART1 becomes CRSF and USART2 becomes MSP;
  - mincheck 1050, maxcheck 1900;
  - BatteryCapacity is set to 800 mAh;
  - ESP Wi-Fi is switched off.
- `STM_PB3_ESP_IO14_Init` (md:99-107) drives PB3 as the ESP Wi-Fi enable line (low = on, RxConfig.cpp:273-305). The firmware already calls it at main.cpp:203, so the user call is redundant.
- `flight_mode` is used only internally.

### Serial-IO.md
- **`SPI_Init()` with no arguments does not link.** It is declared at Serial-IO.h:207, but the code defines `SPI_init` (Serial-IO-SPI.cpp:84). This is the same class of bug as the pre-1.8.3 `RcCommand_get`. The wiki recommends it at md:160-166.
- **`speed` is not in Hz.** It must be one of the codes 140/281/562/1125/2250/4500/9000/18000 (kHz). Any other value falls back to prescaler 2, which is 18 MHz (Serial-IO-SPI.cpp:34-71). The wiki's `SPI_Init(MODE0, 1000000, MSBFIRST)` (md:166, 260) also overflows `uint16_t`, so it runs at 18 MHz. The header says "Hz" at Serial-IO.h:222.
- SPI chip-select is hardcoded to PB5 = GPIO_13 / PWM_9 / RGB_7 (Serial-IO-SPI.cpp:30-31), and `SPI_Init` never configures that pin as an output.
- When XRanging or X1Ranging is used, SPI2 is released and every SPI call returns 0 or does nothing (Serial-IO-SPI.cpp, `spiIsReleased`; XRanging.cpp:258-269). The M25P16 flash shares the same bus.
- Each `SPI_Read` blocks for at least 350 us because of its fixed delays.
- **The I2C examples write to the onboard flight IMU.** Address 0x68 is the IMU (accgyro_mpu.cpp:51). md:142 writes PWR_MGMT_1, and md:152 writes GYRO_CONFIG/ACCEL_CONFIG (high severity). The bus is also shared with the baro, mag and OLED, and it is polled.
- The multi-byte `I2C_Read` returns the byte count or 0, never a negative value. Length is capped at 255 (bus_i2c_stm32f30x.c:331, the `uint8_t len`).
- **UART:**
  - In Rx_ELRS mode USART2 is already MSP, so `openSerialPort` returns NULL (serial.cpp:260-262). Every `Uart_*` call then dereferences NULL (suspected hard fault).
  - The same happens if `Uart_Init` was never called.
  - `Uart_Read16/32` are little-endian and do not check that enough bytes are waiting.
  - `Uart_txBytesFree` only means "at least 1 byte free".
  - Pins: PA2/PA3 (GPIO_18/17, ADC_9/8).

### Motor.md
- Calling `Motor_Set(M1/M2)` before `Motor_Init` dereferences a NULL `userMotor` (Motor.cpp:155).
- `Motor_SetDir(M5..M8)` is not "no effect" (md:37-38). `motors_gpio[2..5]` are zero, so it writes through a NULL GPIO pointer (Motor.cpp:209-215, suspected fault).
- The M1/M2 pins are not documented: PA6/PA7 on TIM3_CH1/CH2, direction pins PB0/PB1. TIM3_CH1 conflicts with PWM_8 and RGB_6 (WS2812_RGB.md:102).
- M5-M8 go to `motor_disarmed[3,2,0,1]`, and all four reset to 1000 on the arm edge (Motor.cpp:190-195).

### Peripherals.md
- **`Peripheral_Write(PWM)` duty is only right at 50 Hz.** It maps 0-100 to 0-20000 timer counts on a 1 MHz timer (Peripheral-PWM.cpp:110-117, pwm_mapping.h:40). At 1 kHz, any value of 5 or more gives 100%, so the md:118,127 example is wrong.
- **`Servo_Write` has only 6 steps.** It uses an integer map to 5..10, giving 1000/1200/.../2000 us. `Servo_Write(PWM_1, 1500)` outputs 1400 us (Peripheral-PWM.cpp:125-131).
- **Do not call `APIAdcInit`.** It is firmware-internal and already runs after `plutoInit` (main.cpp:583); the wiki tells users to call it (md:141-148). An ADC pin initialised after `plutoInit` is probably never started (suspected).
- On the XRanging down-laser XSHUT pin, GPIO/ADC/PWM init and writes are silently refused (Peripheral-GPIO.cpp:93,132; Peripheral-PWM.cpp:72; Peripheral-ADC.cpp:77).
- There is no pin table. Conflicts the page should show:
  - GPIO_1 / PWM_1 = PA8, which is the PPM input and RGB_1 (used in the md:62, 117 examples);
  - GPIO_10/11 are the SWD pins;
  - GPIO_17/18 are UART2;
  - GPIO_5-8 are SPI2/flash;
  - GPIO_13 is the SPI chip-select;
  - GPIO_15 = PB3 is the ESP enable line (?).
- Channels on one timer probably share one frequency: TIM4 (PWM_2/3/6), TIM15 (PWM_4/5), TIM8 (PWM_7/10) (suspected).
- Internal externs and macros in the header are not documented. Mark them internal: `_isAdcEnable`, `_adcDmaIndex`, `_adcNValues`, `Peripheral_ADC_Channel`, `ADC*_CHANNEL_COUNT`.

### Status-LED.md
- **`STATUS` is not an LED.**
  - `Set_LED(STATUS, OFF)` turns off R/G/B and stops the firmware status pattern (`LedStatusState`, runtime_config.cpp:104).
  - `Set_LED(STATUS, ON)` resumes the pattern.
  - `Set_LED(STATUS, TOGGLE)` does nothing (Status-LED.cpp).
  - So the md:46 and md:86-88 examples do nothing.
- While the pattern is running, the firmware overwrites user writes to GREEN/BLUE/RED (suspected).

### RGB-LED.md
- `RGB_Release` does not free the pin, timer or DMA (md:82). It stops the animation, blanks the strip, resets brightness to 100 and hands the strip back to the system (RGB-LED.cpp:219-237).
- RGB_6, the "recommended" slot, conflicts with `Motor_Init(M1/M2)` and PWM_8.
- RGB_7 is the SPI chip-select, PWM_9 and ADC_8/9 (DMA).
- RGB_8 (PA15) is also GPIO_16 / PWM_10 / the XRanging FRONT XSHUT.
- The link `docs/WS2812_RGB.md` (md:32) is wrong: the file is at `docs/fw-development-reference/`.
- Missing: `RGB_Init` can be called more than once and the data pin is locked on the first call; `led_count` is clamped; `RGB_Control(RGB_SYSTEM)` stops the animation (RGB-LED.cpp:245).

### Debugging.md
- Missing from the 1.8.9 DEBUGGING wiki; merge all of it:
  - precision is clamped to 0..4 (5 or more prints 4);
  - rounding rules;
  - "overflow", "not a number" and "infinite" texts;
  - the 255-character cut;
  - **the ~130 bytes per `plutoLoop` budget, or the app disconnects**, with the byte-count table;
  - `Println` adds a 5-byte frame;
  - `Monitor_Print("v:", 3.7f)` calls the int version and prints 3;
  - NULL counts as an empty string;
  - every call shows on its own line in the app;
  - `onLoopStart` runs again after a Wi-Fi stall.
- BlackBox (md:68-124): `BLACKBOX` is commented out in every target.h (PRIMUS_X2_v1/target.h:154), so the function is never built. The `"alt"` literal passed to `char*` also gives a warning.

### Oled.md
- The wiki says `Oled_Clear` "clears the display, skipped while armed" (md:56). It only clears RAM and sends nothing (Oled.h:63-67).
- The wiki says `Oled_Update` "transmits" (md:67). It only queues the frame. At most 16 bytes go out per loop, and a full frame takes about 0.25 s (Oled.h:72-81). In System mode it does nothing, so the md:85-86 example is a no-op.
- Merge from OLED_API_WIKI: "Timing and flight safety" (shared I2C, auto-off after write failures), `Oled_Mode` starting from a blank frame, the `Oled_display_Update` static-buffer rule (needs `io/oled_display.h`, which PlutoPilot.h does not include), and the snippets.

### Scheduler-Timer.md
- `Interval::set` latches `time` on the first call. Later calls with a different value are ignored until `reset()`, so "self-configuring" (md:70) is wrong.
- A non-repeat `Interval` returns true on every call once it has elapsed.
- `check()` on an Interval that was never `set` is always true (loopTime is 0).
- `Scheduler_Add` calls `malloc` for each node. `onLoopStart` runs again after a stall, so the md:120 example (add with no clear) adds duplicate tasks.
- `Execute_Scheduled` skips missed periods, and the first run comes one interval after Add.

### Navigation
- `Specifiers` is linked at _Sidebar.md:8, Home.md:14 and Introduction.md:141, but has no page and no header.
- `X1Ranging.h` has no page and no entry in Sidebar, Home or Intro.
- Oled is missing from the Introduction.md:164-166 list.
- "Debug output and OLED display" (Home.md:36, Introduction.md:165) is stale.
- Introduction.md:139-166 links use `.md` while the sidebar does not; check they render on GitHub.
- Introduction.md:85 lists Black Box, which is disabled on every target.
- Introduction.md:114 says `onLoopStart` "Runs once", but it runs again after a link drop.
- The 3.5-2000 Hz loop range is correct (FC-Config-PID.cpp:88).
- No page states an API version. Home could carry "API 1.8.9 / FW 3.15.11".

Constraints / invariants that apply: no em dashes in any file (user memory; the RGB header text contains them); each example must sit in the right hook (memory `wiki-example-hook-placement`); Monitor_Print ~130 B/tick (CLAUDE.md); XRanging pin reservation and SPI2 release (CLAUDE.md); RC override expiry (CLAUDE.md).
Resource conflicts: none are introduced, but these existing conflicts need documenting: TIM3_CH1 (M1 / PWM_8 / RGB_6), PB5 (SPI chip-select / RGB_7 / PWM_9), PA15 (RGB_8 / FRONT XSHUT), USART2 (Uart API vs ELRS MSP).
Already answered by the code: all the facts above; there are no removed functions on these pages; 3.5-2000 Hz is correct.

Questions for the user:
1. Should the wiki describe today's behaviour, bugs included, with "Known gaps" sections (like RC_INTERFACE)? Or should fix topics come first? The bugs I found are listed below.
2. Copy the docs/API wikis into the GitHub pages in full, or summarise them and link to the repo (which branch)?
3. Specifiers: drop the link or write a page? Who writes an X1Ranging page (it is in neither scout's list)?
4. Should the OLED advanced API (`Oled_display_Update`, `io/oled_display.h`) appear on the public wiki?
5. Document the internal externs (`usingMotorAPI`, `LedStatusState`, `DevMode*`, the `_adc*` arrays) or mark them internal-only?
6. Is PB3 / GPIO_15 the ESP Wi-Fi enable line on every board?
7. Which safe I2C example address should replace 0x68?
8. Add a version line to each page, or only to Home?

Doc drift (reference docs and header comments):
- PIN_MAP.md:74 omits the SPI chip-select on PB5.
- PIN_MAP.md has no rows for PA6/PA7/PB0/PB1 (the M1/M2 pins).
- PIN_MAP.md:38 marks PB3 free, but RxConfig.cpp:271-305 drives it.
- Header comments that are wrong: Peripherals.h:175,181 ("ADC_11"); RGB-LED.h:387 (`RGB_Init(8)`); Serial-IO.h:207 and :222.

Firmware bugs found (for separate fix topics, not this one):
- `SPI_Init()` link error
- `Motor_SetDir(M5-8)` writes through NULL; flight/motor.cpp:108-111 also calls M5 twice and never M7
- `Servo_Write` resolution
- PWM duty only right at 50 Hz
- `Uart_*` NULL dereference with ELRS
- `RcData_Get()` returns NULL
- `Set_LED(STATUS, TOGGLE)` does nothing

Prior work: closed topics rc-command-get-api, monitor-print-double-precision, oled-flight-wobble, tof-single-pipeline, vl53l1x-xranging-api; user-loop-timer-wrap is planned. Commits: 868409a, c495d6a, edfbd61, e72d098, 1f8e78e.
Suggested skills: spec-to-code-compliance (each page against its header), pluto-rules (example hooks and conversions), pluto-commit (closing).
Pipeline docs to update at the end: none in fw-architecture-pipeline. Optional: correct PIN_MAP.md as listed under Doc drift.

Files: `MagisV2.wiki/` (the pages above), `src/main/API/`, `src/main/API-Src/`, `docs/API/`, `docs/fw-development-reference/PIN_MAP.md`, `docs/fw-development-reference/WS2812_RGB.md`

---

## Checks run in the main conversation after the briefs

Verified in the source at `1f8e78e` before planning ( the six "broken function" claims ):

| Claim | Proof |
|---|---|
| `SPI_Init ( )` no-argument form does not link | declared `Serial-IO.h:207`; the only definitions are `SPI_init ( )` at `Serial-IO-SPI.cpp:84` and the 3-argument `SPI_Init` at `:90`. No other definition in `src/`. Confirmed. |
| `Motor_SetDir ( M5..M8 )` | `motors_gpio [ 6 ]` is file-static at `Motor.cpp:74`; only `[ 0 ]` and `[ 1 ]` are ever filled ( `:95-104`, `:114-122` ). The bound check at `:209` passes for 2..5, then `digitalHi / digitalLo` use a NULL `gpio`. Confirmed. |
| `Set_LED ( STATUS, TOGGLE )` | `Status-LED.cpp:43-55`: the `STATUS` branch handles OFF and ON and returns; TOGGLE falls through to the `return`. Confirmed. |
| `RcData_Get ( void )` | `RC-Interface.cpp:82-88` copies into a local `rcDataArray [ 11 ]` and returns it. The channel-taking overload at `:95` is fine. Confirmed. |
| `Uart_*` NULL dereference | `Uart_Init` at `Serial-IO-Uart.cpp:69-73` stores whatever `openSerialPort` returns; `serial.cpp:260-262` returns NULL when the port is taken or unknown, and `Uart_Read8` at `:75-79` calls `serialRead ( uart2 )` with no NULL check. Confirmed. |
| BlackBox never built | `// #define BLACKBOX` in all three targets ( `PRIMUS_X2_v1/target.h:154`, `PRIMUS_V5/target.h:155`, `PRIMUSX2/target.h:155` ). Confirmed. |
