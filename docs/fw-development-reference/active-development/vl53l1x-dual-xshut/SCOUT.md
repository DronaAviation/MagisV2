# VL53L1X Dual XSHUT - Scout Brief

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [DESIGN](DESIGN.md)

Brief from the `pluto-scout` agent, 5 Oct 2026, verbatim. At that time the working tree held an uncommitted
first attempt ( four `Peripheral_*` lines at the top of `ranging_init_L1` ); by the end of planning those lines
had been removed from the driver.

---

## Scout brief: two VL53L1X lasers, XSHUT on GPIO_5 (bottom, used) and GPIO_6 (second, held off)
Graph: stale (built 369ea71a, HEAD cbd1c72). The `graphify` CLI is not on PATH, so I used grep.

**Subsystems:** VL53L1X driver, boot init, user GPIO API, SPI2 flash pins -> pipeline docs: `subsystems/Altitude_Hold_Estimator.md`, `subsystems/Main_Initialization.md`, `PIN_MAP.md`

**Key code:**
- `src/main/drivers/ranging_vl53l1x.cpp:203` `ranging_init_L1`: ST init at fixed address 0x29, I2C1 PB8/PB9, 400 kHz. It never calls SetDeviceAddress.
- `src/main/main.cpp:523`: `ranging_init_L1()` runs after `baroInit` (~471) and `m25p16_init` (450), before `initOpticFlow` (530), `plutoInit` (540), `APIAdcInit` (599) and `xRangingInit` (600).
- `src/main/API-Src/Peripheral-GPIO.cpp:49-50`: `GPIO_5` = PB15, `GPIO_6` = PB14. The API only offers push-pull `OUTPUT`, no open-drain.
- `src/main/API-Src/XRanging.cpp:55,57`: the XRanging API uses `GPIO_6` as the LEFT status LED and `GPIO_5` as the FRONT status LED.
- `src/main/drivers/opticflow_paw3903.cpp:729-742`: `initOpticFlow` toggles `GPIO_6` (only when `OPTIC_FLOW` is on, which it is not now).
- `src/main/target/PRIMUS_X2_v1/target.h:57-60,126-138`: `USE_FLASH_M25P16` on SPI2, with PB14 = MISO and PB15 = MOSI. PRIMUS_V5 is the same.

**Mode:** feature, partly built already. The working tree has uncommitted code for option (a).

**Current behaviour:** The uncommitted diff adds four lines at the top of `ranging_init_L1` (`ranging_vl53l1x.cpp:205-208`). They set `GPIO_6` and `GPIO_5` to `OUTPUT`, write `GPIO_6` LOW and `GPIO_5` HIGH, then go straight to `VL53L1_WaitDeviceBooted`. It also enables `LASER_TOF_L1x` and `LASER_ALT` in the PRIMUS_X2_v1 `target.h`. The code is unconditional: every target that defines `LASER_TOF_L1x` would drive PB14/PB15. HEAD has no XSHUT handling at all. Nothing in the code detects a second device at 0x29.

**Analysis:**
1. **Without XSHUT control.** The STM32 pins are floating inputs at reset, and breakout boards normally pull XSHUT up, so both sensors boot at 0x29. Nothing addresses 0x29 before `ranging_init_L1`, so baro/mag init is unaffected. After that, every write reaches both sensors, and reads (including the NVM calibration read in `DataInit` and the 17-byte result) combine both replies on the bus (wired-AND). The result is wrong calibration and wrong ranges with no I2C error, so `Global_Status_L1` stays 0. This fails silently: the code cannot detect it.
2. **Option (a), XSHUT in the driver before the probe (what the diff does).**
   - Files: `ranging_vl53l1x.cpp`, plus pin defines in `target.h`, e.g. `L1X_XSHUT_ON_GPIO` / `L1X_XSHUT_OFF_GPIO` (names not in the code yet).
   - Cost: about 100 B flash, 0 RAM, no loop cost (boot only).
   - Timing: when the bottom sensor is released it needs tBOOT ~1.2 ms. `WaitDeviceBooted` polls for up to 500 ms (`vl53l1_platform_user_config.h:77`), so no extra delay is needed.
   - Safety: no effect on ALT_HOLD behaviour.
   - Fixes still needed:
     - gate the pins per target;
     - consider releasing the bottom XSHUT as open-drain or input, not push-pull HIGH, because some carriers (e.g. Pololu) pull XSHUT to 2.8 V without a level shifter (see question 2);
     - match the code style (`OUTPUT)` should be `OUTPUT )`).
3. **Option (b), full dual support.** Hold B low, init A, move A to 0x2A, then optionally bring up B. Files: driver plus the `MyDevice_L1` struct. Costs about 0.5-1 KB flash and ~200 B more RAM if B is ever initialised. The bus keeps working even if someone later releases GPIO_6. This is overkill while B is unused.
4. **Option (c), user code in `plutoInit`.** Unsafe. It runs after `ranging_init_L1` (main.cpp:540 vs 523), so the bottom sensor would already have been initialised with mixed calibration data.

**Constraints / invariants that apply:**
- PIN_MAP §3.1: PB12-PB15 are shared between ADC, GPIO, PWM and SPI2/flash. Driving PB14/PB15 as GPIO cuts off the M25P16 flash after boot. `BLACKBOX` is commented out (`target.h:154`), but MSP dataflash reads would return junk. MISO contention is avoided only while flash CS (PB12) stays high.
- Laser invariant (CLAUDE.md): only one down-laser is fused. The `#error` at `altitudehold.cpp:108` still holds.
- `isOutofRange_L1` falls back to the baro if the bottom sensor fails to boot (`Global_Status_L1`).
- Strict warnings: run the `--gate` build.

**Resource conflicts:**
- PB14 (GPIO_6) is also ADC_3 (ADC4_IN4, DMA2_Ch2), PWM_5 (TIM15_CH1), RGB_3 (`API/RGB-LED.h:119`, not the default; the default is PA15) and SPI2_MISO.
- PB15 (GPIO_5) is also ADC_2, PWM_4 (TIM15_CH2) and SPI2_MOSI.
- Later boot steps can override the pins:
  - `Peripheral_Init(ADC_3)` would float PB14, which re-enables sensor B and corrupts the bus.
  - `XRanging.init(FRONT)` drives `GPIO_5` LOW at `XRanging.cpp:88-89`, which shuts the bottom laser down.
  - `XRanging.init(LEFT)` toggles `GPIO_6`.
  - `OPTIC_FLOW` would toggle `GPIO_6`.
- `pwmInit` does not touch PB15: T15 is not placed in `timerHardware[]` (`timer.cpp:146-150`).

**Already answered by the code:**
- The I2C bus is I2C1 at 400 kHz.
- No address-change path exists for the L1X driver.
- `plutoInit` runs too late to set XSHUT.
- GPIO_5 = PB15 and GPIO_6 = PB14 on both targets.
- The diff makes no delay necessary.

**Questions for the user:**
1. Confirm that GPIO_5 and GPIO_6 are wired to the two sensors' XSHUT pins, not to VIN or GPIO1.
2. Which breakout is it, and is XSHUT pulled up on the board to 2.8 V or 3.3 V? This decides whether to drive the pin push-pull HIGH or release it as open-drain/input.
3. PRIMUS_X2_v1 only, or PRIMUS_V5 too? Should the pins be a `target.h` define?
4. Is the onboard M25P16 flash populated or needed? Taking PB14/PB15 disables it.
5. Should sensor B stay held in reset for the whole session (yes in the current diff), or will it be used later? That decides (a) or (b).
6. Will the XRanging API, ADC_2/ADC_3, PWM_4/PWM_5 or `RGB_3` be used on these boards? Should that be blocked with a guard or a comment?

**Doc drift:** none found in committed docs. After the change, `PIN_MAP.md` (around line 29 and the PB14/PB15 rows) will be wrong.

**Prior work:**
- vl53l1x-althold-parity (closed, `7cf2444`).
- vl53l1x-error-latch-recovery (planned): its re-init must reassert the XSHUT pins before `WaitDeviceBooted`. Coordinate with it.
- vl53l1x-extended-reach (planned): no conflict.

**Suggested skills:** pluto-driver, pluto-rules, pluto-build (`--gate PRIMUS_X2_v1`), pluto-flighttest (bench check: `G`=0 and ranges with B held off and with B released), pluto-commit.

**Pipeline docs to update at the end:**
- `PIN_MAP.md`: PB14/PB15 rows, down-laser note and conflict §3.1.
- `subsystems/Altitude_Hold_Estimator.md` (lines 9, 128, 289): XSHUT set-up step.
- `subsystems/Main_Initialization.md`: laser init before `plutoInit`.
- `TIMER_MAP.md` / `DMA_MAP.md`: note that PWM_4/5 and ADC_2/3 are unavailable when the XSHUT pins are used.
- No API wiki change unless the XRanging pins are touched.
