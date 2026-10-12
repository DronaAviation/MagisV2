# VL53L1X Dual XSHUT - Tasks

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [DESIGN](DESIGN.md) · [TESTING](TESTING.md)

## Resume

Read this block first in a new session; read only the task it points to.

| | |
|---|---|
| **Current task** | **Closed - pending commit** ( 5 Oct 2026 ). All tasks done. |
| **Next step** | User commits with `git commit -F .git/MAGISV2_COMMIT_MSG.txt`; next pluto-commit / pluto-grill run fills in the hash. |
| **Open questions** | none |
| **Blocked on** | nothing |
| **Last updated** | 2026-10-05 |

## Summary

| | |
|---|---|
| **Mode** | feature |
| **Goal** | Only the bottom VL53L1X ( XSHUT on `GPIO_5` / PB15 ) is enabled and used; the second ( XSHUT on `GPIO_6` / PB14 ) is held in shutdown from boot, so the two never share 0x29 on I2C1. |
| **Done when** | Bench, both sensors plugged in: `G` ( `Global_Status_L1` ) = 0 and the laser range matches a tape measure; then an ALT_HOLD hover on PRIMUS_X2_v1 where the laser corrects below 160 cm as in the vl53l1x-althold-parity logs. |
| **Target** | PRIMUS_X2_v1 ( flown ); PRIMUS_V5 gets the same defines |
| **Analysis** | Option (a): XSHUT set in `ranging_init_L1` before `WaitDeviceBooted`, pins from `target.h` defines ( [DESIGN.md](DESIGN.md) ) |
| **Pipeline docs affected** | `PIN_MAP.md`, `TIMER_MAP.md`, `DMA_MAP.md`, `subsystems/Altitude_Hold_Estimator.md`, `subsystems/Main_Initialization.md` |
| **Order** | 1 -> 2 -> 3 -> 4 -> 5 -> 6 -> 7 -> 10 -> 8 -> 9 |

## Index

| # | Title | Status | Depends on | Skills / agent |
|---|---|---|---|---|
| 1 | Bench: measure the TOF400C SHUT pin, choose the drive mode | done 2026-10-05 | - | pluto-driver |
| 2 | XSHUT pin defines in PRIMUS_X2_v1 and PRIMUS_V5 `target.h` | done 2026-10-05 | 1 | pluto-driver, pluto-rules |
| 3 | Drive XSHUT in `ranging_init_L1` before the probe | done 2026-10-05 | 2 | pluto-rules, pluto-driver, cpp-pro agent |
| 4 | Note for vl53l1x-error-latch-recovery: re-init must reassert XSHUT | done 2026-10-05 | 3 | - |
| 5 | Build gate | done 2026-10-05 | 3 | pluto-build |
| 6 | Hardware validation ( bench + hover ) | done 2026-10-05 | 5 | pluto-flighttest |
| 7 | Architecture & pipeline docs ( PIPELINE_UPDATE.md ) | done 2026-10-05 | 6 | graphify, spec-to-code-compliance |
| 10 | Keep the M25P16 off the bus when the XSHUT pins are taken | done 2026-10-05 | 7 | pluto-rules, pluto-driver |
| 8 | Topic review | done 2026-10-05 | 7, 10 | pluto-reviewer agent |
| 9 | Graph refresh & commit | done 2026-10-05 | 8 | graphify, pluto-commit |

Status values: `todo`, `in-progress`, `done YYYY-MM-DD`, `blocked (<why>)`, `dropped (<why>)`.
Serial numbers are never reused or renumbered.

## Tasks

### 1. Bench: measure the TOF400C SHUT pin, choose the drive mode

**Description.** It is not known whether the TOF400C module pulls its SHUT ( XSHUT ) pin up on the board, to
which voltage, or whether it tolerates 3.3 V from the FC. That decides how the firmware enables the bottom
sensor. Measure on the bench, with the FC powered and running HEAD firmware ( which leaves PB14 / PB15
floating ): the voltage on each module's SHUT pin, and the resistance from SHUT to VIN / to the 2.8 V rail if
the module has one. Check the module schematic or silkscreen for a level shifter. Decide: pull-up present ->
release the ON pin as an input ( floating ) so the module's own pull-up enables it; no pull-up and 3.3 V
tolerant -> drive HIGH push-pull; neither -> stop and ask the user. The OFF pin is driven LOW in every case.

- **Depends on:** -
- **Skills / agent:** pluto-driver
- **Files:** none ( measurement )
- **Safety impact:** none ( bench, props off )
- **Done when:** the SHUT voltage, pull-up value ( or "none" ) and the chosen drive mode are recorded in
  `TESTING.md` and as a decision in the log below.
- **Rollback:** n/a
- **Status:** done 2026-10-05
- **Result:** SHUT reads 2.7 V with the wire off the FC ( module pull-up to its ~2.8 V rail ); GPIO_5 is released as a floating input to enable, GPIO_6 driven LOW ( [TESTING.md](TESTING.md) ).

### 2. XSHUT pin defines in PRIMUS_X2_v1 and PRIMUS_V5 `target.h`

**Description.** Add two defines next to the laser block of both targets ( e.g. `L1X_XSHUT_ON_GPIO GPIO_5`
and `L1X_XSHUT_OFF_GPIO GPIO_6`; final names chosen here, consistent with the `L1X_*` driver constants ), and
a comment listing what they take away: SPI2 flash ( M25P16 MOSI / MISO ), ADC_2 / ADC_3, PWM_4 / PWM_5,
RGB_3, XRanging FRONT / LEFT, `OPTIC_FLOW`, and the `Serial-IO-SPI` user API ( `spiInit ( SPI2 )` hands both pins back to SPI2 ). On PRIMUS_X2_v1 keep `LASER_TOF_L1x` + `LASER_ALT` enabled ( already
in the working tree ). On PRIMUS_V5 add only the defines; its laser defines stay unchanged. Preserve the banner
headers.

- **Depends on:** 1
- **Skills / agent:** pluto-driver, pluto-rules
- **Files:** [PRIMUS_X2_v1/target.h](../../../../src/main/target/PRIMUS_X2_v1/target.h), [PRIMUS_V5/target.h](../../../../src/main/target/PRIMUS_V5/target.h)
- **Safety impact:** none on its own ( no code reads the defines until task 3 )
- **Done when:** both targets build; the defines and the conflict comment are present.
- **Rollback:** remove the two defines.
- **Status:** done 2026-10-05
- **Result:** `L1X_XSHUT_ON_GPIO/_PIN` ( GPIOB Pin_15 ), `L1X_XSHUT_OFF_GPIO/_PIN` ( GPIOB Pin_14 ), `L1X_XSHUT_PERIPHERAL` in both targets; both build ( X2_v1 112.2 KB / 16.0 KB, V5 100.9 KB / 14.8 KB ); review no BLOCKING. **Do not flash X2_v1 until task 3**: laser is on with no XSHUT handling yet ( and the old 4-line attempt still drives GPIO_5 HIGH ).

### 3. Drive XSHUT in `ranging_init_L1` before the probe

**Description.** At the top of `ranging_init_L1 ( )`, before `VL53L1_WaitDeviceBooted`, under
`#if defined ( L1X_XSHUT_OFF_GPIO )` / `L1X_XSHUT_ON_GPIO`: drive the OFF pin LOW first ( second sensor into
shutdown ), then release the ON pin as a floating input ( no internal pull; task 1: the module pulls SHUT to ~2.8 V, so it must never be driven HIGH ). Both pins must be reconfigured from the SPI2 AF mode `spiInit ( SPI2 )` set at boot. ~~No extra delay ( tBOOT ~1.2 ms, `WaitDeviceBooted`
polls up to 500 ms )~~ ( superseded: the platform waits are stubs, so the driver adds `delay ( 2 )`; see the decisions log ). Decide here whether the driver uses the `Peripheral_*` API ( `Peripheral-GPIO.cpp`, push-pull
only, layering from driver into API ) or the low-level `gpioInit` driver directly; prefer the lower layer if it
avoids a driver -> API include. Spaced-paren style, banner history line. The OFF pin must be held LOW for the
whole session ( nothing in the driver releases it ).

- **Depends on:** 2
- **Skills / agent:** pluto-rules, pluto-driver; delegate to cpp-pro agent if self-contained; pluto-reviewer on the diff
- **Files:** [ranging_vl53l1x.cpp](../../../../src/main/drivers/ranging_vl53l1x.cpp)
- **Safety impact:** laser altitude source ( ALT_HOLD below 160 cm ). A wrong pin or polarity shuts the bottom
  laser off -> `G` != 0 -> baro-only hold, same as no laser fitted.
- **Done when:** PRIMUS_X2_v1 builds with no new warnings; the code is absent from targets without the defines
  ( PRIMUSX2 object unchanged in behaviour ); reviewer has no BLOCKING findings.
- **Rollback:** remove the `#if` block ( or the defines from task 2 ); the driver returns to HEAD behaviour.
- **Status:** done 2026-10-05
- **Result:** `ranging_vl53l1x.cpp:208-224`: OFF pin push-pull LOW, ON pin floating input, `delay ( 2 )` ( platform waits are stubs ); gate clean, 112.2 KB / 16.0 KB; objdump confirms the calls precede `WaitDeviceBooted`; review: 1 BLOCKING ( no boot wait ) fixed, re-check clean.

### 4. Note for vl53l1x-error-latch-recovery: re-init must reassert XSHUT

**Description.** The planned [vl53l1x-error-latch-recovery](../vl53l1x-error-latch-recovery/README.md) topic
will re-run the ST init after an I2C latch. If it calls only part of `ranging_init_L1`, or anything later
touches PB14 / PB15, the second sensor could wake. Add one bullet to that topic's README ( Planned approach ):
re-init must go through the XSHUT block ( or keep the OFF pin LOW ) before `WaitDeviceBooted`.

- **Depends on:** 3
- **Skills / agent:** -
- **Files:** [vl53l1x-error-latch-recovery/README.md](../vl53l1x-error-latch-recovery/README.md)
- **Safety impact:** none
- **Done when:** the bullet is in that README.
- **Rollback:** n/a
- **Status:** done 2026-10-05
- **Result:** Item 6 added to that README's Planned approach: keep the OFF pin LOW, never drive the ON pin HIGH, own tBOOT wait after any XSHUT toggle ( `WaitDeviceBooted` does not wait ); ON-pin pulse as a hard-reset option.

### 5. Build gate

**Description.** `pluto-build --gate PRIMUS_X2_v1`; also a compile of PRIMUS_V5 because its `target.h`
changed. No new `src/` warnings against the baseline; flash and RAM fit. Record flash / RAM deltas against
HEAD ( expected ~+100 B flash, 0 RAM ).

- **Depends on:** 3
- **Skills / agent:** pluto-build
- **Files:** -
- **Safety impact:** none
- **Done when:** gate passes on PRIMUS_X2_v1, PRIMUS_V5 builds, deltas recorded in `CHANGES.md`.
- **Rollback:** n/a
- **Status:** done 2026-10-05
- **Result:** Gate clean on PRIMUS_X2_v1 ( no new `src/` warnings ), V5 builds; +68 B flash, 0 RAM vs HEAD with the laser on ( all in `ranging_init_L1` ); X2_v1 112.2 KB / 16.0 KB.

### 6. Hardware validation ( bench + hover )

**Description.** On PRIMUS_X2_v1 with both TOF400C fitted. Bench, props off: boot, confirm the SHUT pin
voltages ( bottom enabled, second LOW ), log `G` = 0 and laser range against a tape measure at 3 heights
( e.g. 20, 60, 120 cm ). Optional negative check: briefly lift the second sensor's SHUT off GPIO_6 and confirm
the range goes wrong or `G` changes, which proves the hold-off matters. Then an ALT_HOLD hover below 160 cm and
one climb through the 160 / 140 cm handover; compare with the vl53l1x-althold-parity logs. Log fields from
`pluto-flighttest`; large logs to the pluto-log-analyst agent.

- **Depends on:** 5
- **Skills / agent:** pluto-flighttest; pluto-log-analyst agent for big logs
- **Files:** `TESTING.md`, `logs/`
- **Safety impact:** flight test in ALT_HOLD; fallback is baro-only hold if the laser drops.
- **Done when:** bench `G` = 0 and ranges within ~3 cm of tape; hover holds within the parity-topic band and the
  laser source is active below 160 cm; results in `TESTING.md`.
- **Rollback:** fly HEAD firmware ( no XSHUT ) with the second sensor unplugged.
- **Status:** done 2026-10-05
- **Result:** log-1 ( flight, 44 s armed, both TOF400C fitted ): `G` 0 in all records, 20.2 samples/s, `St` 0 in 99.9 % of flight samples, no two-sensor signature, handovers at 160 / 140 cm with `E` - `H` unchanged. PASS. Bench and negative checks skipped by decision; clean hover window not available ( fusion unchanged ).

### 7. Architecture & pipeline docs ( PIPELINE_UPDATE.md )

**Description.** Stage the new text in this topic's `PIPELINE_UPDATE.md` ( never edit `fw-architecture-pipeline/`
directly ): `PIN_MAP.md` PB14 / PB15 rows, down-laser note and conflict section 3.1 ( flash lost, ADC_2/3,
PWM_4/5, RGB_3, XRanging, OPTIC_FLOW ); `TIMER_MAP.md` / `DMA_MAP.md` notes that PWM_4/5 ( TIM15 ) and ADC_3
( DMA2_Ch2 ) are unavailable with the XSHUT defines; `subsystems/Altitude_Hold_Estimator.md` and
`subsystems/Main_Initialization.md`: the XSHUT step in laser init, before `plutoInit`. If a flowchart changes,
every edge must be a real call path. Check with `spec-to-code-compliance`. No API wiki change ( public API
untouched ).

- **Depends on:** 6
- **Skills / agent:** graphify, spec-to-code-compliance
- **Files:** `PIPELINE_UPDATE.md`
- **Safety impact:** none
- **Done when:** `PIPELINE_UPDATE.md` covers every doc listed; compliance check passes.
- **Rollback:** n/a
- **Status:** done 2026-10-05
- **Result:** `PIPELINE_UPDATE.md` stages 3 replacements in `Altitude_Hold_Estimator.md` + CHANGELOG entry; PIN_MAP ( footnote 1, conflict 8 ), TIMER_MAP, DMA_MAP, HARDWARE_RESOURCES edited directly; `Main_Initialization.md` unchanged ( too high-level ). Compliance check ( pluto-reviewer ): 4 misleading + 1 nit, all fixed.

### 10. Keep the M25P16 off the bus when the XSHUT pins are taken

**Description.** Found by the task 7 compliance check. With `L1X_XSHUT_*`, PB14 ( SPI2 MISO, the flash's DO )
is driven LOW push-pull after laser init, but `USE_FLASH_M25P16` / flashfs stay compiled in and
`serial_msp.cpp` still serves MSP dataflash read / erase. An app request would select the flash ( CS PB12, SCK
PB13 still work ) and its DO would drive against PB14: bus contention on the pin that holds the second laser
off, and junk data. Options to decide with the user: (a) undefine `USE_FLASH_M25P16` ( and anything that needs
it ) on targets that define the XSHUT pins, or `#error` if both are set; (b) keep the flash but make the M25P16
driver / flashfs refuse access after laser init; (c) accept and document only ( current state ). Needs a
mini-grill before any code.

- **Depends on:** 7
- **Skills / agent:** pluto-rules, pluto-driver; pluto-reviewer on the diff
- **Files:** `src/main/target/PRIMUS_X2_v1/target.h`, `src/main/target/PRIMUS_V5/target.h`, possibly `drivers/flash_m25p16.cpp`, `io/flashfs`, `serial_msp.cpp`
- **Safety impact:** boot / MSP only; must not touch the laser path. A wrong `#undef` could break the build of other targets.
- **Done when:** the chosen option is in, both targets build with no new warnings, and PIN_MAP conflict 8 / CHANGELOG text match it.
- **Rollback:** revert the target.h change.
- **Status:** done 2026-10-05
- **Result:** `#error` in `ranging_vl53l1x.cpp` if XSHUT + flash defines meet under `LASER_TOF_L1x`; X2_v1 flash defines commented out ( -1.1 KB ); negative test fires; both targets build, gate clean ( 111.2 KB / 15.9 KB ); also fixed the V5 link break from the task 6 log. Review: 3 nits, 2 fixed, 1 noted.

### 8. Topic review

**Description.** `pluto-reviewer` agent on the whole topic diff ( driver, both `target.h`, docs ). Fix
BLOCKING findings and re-run the gate before moving on.

- **Depends on:** 7
- **Skills / agent:** pluto-reviewer agent
- **Files:** -
- **Safety impact:** none
- **Done when:** no open BLOCKING findings; gate re-run if code changed.
- **Rollback:** n/a
- **Status:** done 2026-10-05
- **Result:** pluto-reviewer on the whole topic: no BLOCKING, verdict ready to commit. 1 should-fix ( task 9 must revert `PlutoPilot.cpp` ) and 4 stale-doc nits ( README, DESIGN, CHANGES line range, task 3 text ), all fixed; docs only, no re-gate needed ( last gate clean, 111.2 KB / 15.9 KB ).

### 9. Graph refresh & commit

**Description.** `graphify update .` then `python tools/graph_labels.py`, then `/pluto-commit` ( all-target
build, version bump, PIPELINE_UPDATE promotion, CHANGELOG, staged change and drafted message; the user
commits ). **First revert the temporary diagnostic: `git checkout -- PlutoPilot.cpp`** ( HEAD is the empty
template ), then the all-target build. Set this topic to Closed in its README and in the active-development
index.

- **Depends on:** 8
- **Skills / agent:** graphify, pluto-commit
- **Files:** -
- **Safety impact:** none
- **Done when:** `PlutoPilot.cpp` matches HEAD ( `git diff --quiet -- PlutoPilot.cpp` ); all targets build; the change is staged with a drafted message; the topic is marked Closed - pending commit.
- **Rollback:** n/a
- **Status:** done 2026-10-05
- **Result:** `PlutoPilot.cpp` reverted to HEAD; graph refresh skipped ( graphify not installed ); all targets built ( X2_v1 110.4 KB / 15.9 KB, PRIMUSX2 100.7 / 14.9, V5 100.9 / 14.8 ), no new warnings; `Altitude_Hold_Estimator.md`, CHANGELOG, CLAUDE.md promoted; staged with a drafted message. Version not bumped.

## Decisions log

Newest last. One line each: `YYYY-MM-DD [decision|assumption|out-of-scope|risk] text`.

- 2026-10-05 [decision] Mode feature; working target PRIMUS_X2_v1.
- 2026-10-05 [decision] GPIO_5 ( PB15 ) and GPIO_6 ( PB14 ) are wired to the XSHUT pins of the bottom and the second TOF400C VL53L1X; SDA / SCL / power shared.
- 2026-10-05 [decision] Option (a): XSHUT set in `ranging_init_L1` before `WaitDeviceBooted`; (b) dual addressing deferred, (c) `plutoInit` runs too late ( `main.cpp:540` vs `523` ).
- 2026-10-05 [decision] Second sensor is held off only: no address change, no reads.
- 2026-10-05 [decision] Pins come from `target.h` defines; PRIMUS_X2_v1 and PRIMUS_V5 both get them; on V5 the laser defines stay unchanged.
- 2026-10-05 [decision] Bottom sensor missing at boot behaves as today ( `G` != 0, baro-only hold ); no arming block, no fallback to the second sensor.
- 2026-10-05 [decision] The M25P16 SPI2 flash ( PB14 / PB15 ) is not needed; losing it is accepted and documented.
- 2026-10-05 [decision] Pin conflicts ( XRanging, ADC_2/3, PWM_4/5, RGB_3, OPTIC_FLOW ) are handled with comments and docs only, no runtime guard.
- 2026-10-05 [decision] Done when: bench `G` = 0 with a correct range with both sensors fitted, then an ALT_HOLD hover with the laser active below 160 cm.
- 2026-10-05 [assumption] TOF400C SHUT pull-up and 3.3 V tolerance unknown; task 1 measures it and picks push-pull HIGH or release-as-input.
- 2026-10-05 [risk] User code calling `Peripheral_Init` on GPIO_5 / GPIO_6 / ADC_2 / ADC_3 or XRanging FRONT / LEFT after boot can shut the bottom laser off or wake the second one ( silent bus corruption ). Documented only.
- 2026-10-05 [out-of-scope] Using the second sensor's data; laser fusion tuning ( 160 / 140 cm handover, re-base, hold-off ); the VL53L0X `LASER_TOF` path; implementing error-latch re-init ( only a note, task 4 ).
- 2026-10-05 [out-of-scope] The uncommitted `version.h` change ( `FW_VERSION_LENGTH` 5 -> 6 ) is unrelated to this topic.
- 2026-10-05 [risk] PB14 / PB15 are not idle before laser init: `spiInit ( SPI2 )` ( `main.cpp:313` ) makes PB15 SPI2 MOSI and PB14 MISO, and `m25p16_init ( )` ( `main.cpp:450` ) clocks MOSI, so the bottom sensor's XSHUT toggles and the second sensor's XSHUT floats ( both awake ) until `ranging_init_L1` ( `main.cpp:523` ). Harmless while nothing addresses 0x29 earlier and the driver reconfigures both pins as GPIO before `WaitDeviceBooted`; task 3 must override the SPI AF setting. The `Serial-IO-SPI` API ( `spiInit ( SPI2 )`, `Serial-IO-SPI.cpp:84` ) from user code would hand both pins back to SPI2: add to the conflict list in task 2 / 7.
- 2026-10-05 [decision] Task 1 measures SHUT with the wire disconnected from the FC, because the FC pin is an SPI2 line with the current firmware.
- 2026-10-05 [decision] TOF400C SHUT measured 2.7 V with the wire off the FC ( on-board pull-up to the ~2.8 V rail ). ON pin GPIO_5 is released as a floating input, never driven HIGH; OFF pin GPIO_6 is push-pull LOW. Replaces the [assumption] on drive mode.
- 2026-10-05 [assumption] The second TOF400C is the same board ( not measured ); it is only ever driven LOW, which is safe either way.
- 2026-10-05 [decision] Define names follow the target.h LEDx convention: `L1X_XSHUT_ON_GPIO` / `_PIN`, `L1X_XSHUT_OFF_GPIO` / `_PIN`, one `L1X_XSHUT_PERIPHERAL` ( both on GPIOB ); the driver will use `drivers/gpio.h`, not the `Peripheral_*` API.
- 2026-10-05 [risk] Between task 2 and task 3 the X2_v1 build has `LASER_TOF_L1x` on with no correct XSHUT handling: do not flash it with both sensors fitted.
- 2026-10-05 [risk] From `spiInit ( SPI2 )` ( `main.cpp:313` ) until `ranging_init_L1` ( `main.cpp:523` ) PB15 is SPI2 MOSI, push-pull 3.3 V when high, against the TOF400C SHUT pull-up to ~2.8 V. The task 1 rule ( never drive it HIGH ) therefore holds only from laser init on. Believed harmless ( VL53L1X digital-pin absolute maximum is understood to be 3.6 V; no datasheet in the repo to confirm ). Removing it would need `USE_FLASH_M25P16` / `USE_SPI_DEVICE_2` off on these targets: not in scope, logged for the user.
- 2026-10-05 [decision] `delay ( 2 )` after releasing the bottom sensor: the VL53L1 platform wait functions ( `WaitUs`, `WaitMs`, `WaitValueMaskEx` ) are empty stubs, so `WaitDeviceBooted` does not wait ( planning assumed a 500 ms poll; corrected ). Any future re-init ( error-latch topic ) needs its own boot wait.
- 2026-10-05 [decision] Task 6 log: temporary 10 Hz record in `PlutoPilot.cpp` ( `t mm St G N S E H Ar` + `#START`, ~103 B per tick ); remove before release.
- 2026-10-05 [decision] Task 6 accepted on the flight log alone ( log-1 ): bench tape check, negative check and Test 0 voltages skipped by the user.
- 2026-10-05 [out-of-scope] `setUserLoopFrequency ( f )` stores f x 1000 into `userLoopFrequency`, which the loop uses as a period in us ( requesting 10 Hz gives ~100 Hz ). Pre-existing public-API bug; diagnostic no longer calls it. Needs its own topic ( API wiki + `API_Version` ).
- 2026-10-05 [out-of-scope] `tools/flightlog.py` mis-times logs whose firmware field is named `t` when the first line has no `t:` ( index x 0.1 s fallback ).
- 2026-10-05 [decision] Task 7: `Main_Initialization.md` unchanged ( too high-level to list laser init ); maps ( PIN / TIMER / DMA, HARDWARE_RESOURCES ) edited directly as in earlier topics; only `Altitude_Hold_Estimator.md` is staged. `spec-to-code-compliance` is not available in this session: the compliance check runs in pluto-reviewer instead.
- 2026-10-05 [risk] Task 7 compliance check: with the XSHUT pins taken, an app MSP dataflash read / erase would select the M25P16 and its DO would fight PB14 held LOW. New task 10.
- 2026-10-05 [decision] Task 10 ( user ): `#error` if the XSHUT defines and the flash defines are both set with `LASER_TOF_L1x`; PRIMUS_X2_v1 flash defines commented out. PRIMUS_V5 keeps its flash while its laser is off.
- 2026-10-05 [risk] Found in task 10: the task 6 diagnostic in `PlutoPilot.cpp` broke the PRIMUS_V5 link ( laser symbols ); now gated on `LASER_TOF_L1x` + `LASER_ALT`. Build every target after touching `PlutoPilot.cpp`.
- 2026-10-05 [decision] Task 9 ( user ): `src/main/version.h` ( `FW_VERSION_LENGTH` 5 -> 6 ) is included in this commit. Follow-up topics for `setUserLoopFrequency` and `flightlog.py`: not created now ( left as README open items ).
- 2026-10-05 [risk] Graph refresh skipped: `graphify` is not installed on this machine ( not on PATH, no Python module, no skill ). `graphify-out/` stays at its last build ( 369ea71 ); run `graphify update .` and `python tools/graph_labels.py` once it is installed.
