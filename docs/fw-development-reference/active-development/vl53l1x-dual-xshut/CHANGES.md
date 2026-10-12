# VL53L1X Dual XSHUT - Changes

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [DESIGN](DESIGN.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

## Task 2: XSHUT pin defines ( 5 Oct 2026 )

| File | Lines | Change | Why |
|---|---|---|---|
| [PRIMUS_X2_v1/target.h](../../../../src/main/target/PRIMUS_X2_v1/target.h#L187-L198) | 187-198 | `L1X_XSHUT_ON_GPIO` / `_PIN` ( GPIOB, Pin_15 = GPIO_5 ), `L1X_XSHUT_OFF_GPIO` / `_PIN` ( GPIOB, Pin_14 = GPIO_6 ), `L1X_XSHUT_PERIPHERAL` ( RCC_AHBPeriph_GPIOB ), with a comment on drive mode and what PB14 / PB15 cost | Pins for task 3; follows the `LEDx_GPIO` / `_PIN` / `_PERIPHERAL` convention so the driver can use `drivers/gpio.h` directly |
| [PRIMUS_X2_v1/target.h](../../../../src/main/target/PRIMUS_X2_v1/target.h#L184-L185) | 184-185 | `LASER_TOF_L1x` and `LASER_ALT` enabled ( already in the working tree before the topic ) | Working target flies the VL53L1X |
| [PRIMUS_V5/target.h](../../../../src/main/target/PRIMUS_V5/target.h#L191-L202) | 191-202 | Same XSHUT block; laser defines unchanged ( still commented out ) | Same pins on V5; inert until `LASER_TOF_L1x` is enabled |
| PRIMUS_V5/target.h | 16 | Banner `Last Modified` | Style |

No code reads the defines yet ( task 3 ).

**Review** ( pluto-reviewer, 5 Oct 2026 ): no BLOCKING. Fixed: the conflict comment now says "reconfigures PB14 / PB15
after laser init" ( SPI2 transfers alone no longer reach the pins once they are GPIO ), and notes why there is one
`L1X_XSHUT_PERIPHERAL`. Recorded: an X2_v1 build made before task 3 has the laser on with no XSHUT handling
( both sensors at 0x29 ), so do not flash it.

**Build** ( `driver.sh <T>`, 5 Oct 2026 ): PRIMUS_X2_v1 112.2 KB flash / 16.0 KB RAM; PRIMUS_V5 100.9 KB / 14.8 KB.
Both pass.

## Task 3: XSHUT in `ranging_init_L1` ( 5 Oct 2026 )

| File | Lines | Change | Why |
|---|---|---|---|
| [ranging_vl53l1x.cpp](../../../../src/main/drivers/ranging_vl53l1x.cpp#L215-L231) | 215-231 ( 208-224 before task 10 ) | Under `#if defined( L1X_XSHUT_OFF_GPIO ) && defined( L1X_XSHUT_ON_GPIO )`, before `VL53L1_WaitDeviceBooted`: GPIOB clock on, OFF pin ODR low then push-pull output ( second sensor shut down ), ON pin floating input ( bottom sensor enabled by the module pull-up ), then `delay ( 2 )` | Only the bottom sensor answers at 0x29; both pins taken back from the SPI2 AF mode `spiInit ( SPI2 )` set at boot; never drive the ON pin HIGH ( task 1 ) |
| ranging_vl53l1x.cpp | 16, 22-23 | Banner `Last Modified`, HISTORY line | Style |

Uses `drivers/gpio.h` ( `gpioInit`, `digitalLo` ) and `delay ( )` ( `drivers/system.h`, SysTick running since
`systemInit ( )` ), already included; no `Peripheral_*` call. The earlier 4-line
`Peripheral_*` attempt had been removed from the working tree before this task.

**Build** ( `driver.sh --gate PRIMUS_X2_v1` ): no new warnings in `src/`; 112.2 KB flash / 16.0 KB RAM ( exact delta in
task 5 ). PRIMUS_V5 builds ( 100.9 KB / 14.8 KB; laser off, block not
compiled ). `objdump` of `ranging_init_L1` on X2_v1: `RCC_AHBPeriphClockCmd`, `gpioInit`, `gpioInit`, then
`VL53L1_WaitDeviceBooted`.

**Review** ( pluto-reviewer, 5 Oct 2026 ): one BLOCKING, fixed and re-checked clean. The platform's `VL53L1_WaitUs`,
`VL53L1_WaitMs` and `VL53L1_WaitValueMaskEx` are empty stubs
( [vl53l1_platform.c:258-279](../../../../lib/main/VL53L1X_API/platform/src/vl53l1_platform.c#L258-L279) ), so
`WaitDeviceBooted` does not wait. With MOSI left LOW after the `m25p16_init ( )` RDID, the bottom sensor would be
released only in `ranging_init_L1` and `DataInit` would hit it mid-boot, latching `Global_Status_L1` for the session.
Fix: `delay ( 2 )` after the release ( > tBOOT 1.2 ms ). Gate re-run: no new warnings, 112.2 KB / 16.0 KB.
Checked clean: pin modes ( MODER / PUPDR no-pull ), no glitch on the OFF pin, nothing reconfigures PB14 / PB15
later in the X2_v1 boot, single caller ( `main.cpp:523` ), gating.

## Task 5: build gate ( 5 Oct 2026 )

`driver.sh --gate PRIMUS_X2_v1`: **no new warnings in `src/`**, ALL BUILDS PASSED. PRIMUS_V5 builds.

| Build | text | data | bss | Flash / RAM ( display ) |
|---|---|---|---|---|
| PRIMUS_X2_v1, HEAD `cbd1c72` + `LASER_TOF_L1x` / `LASER_ALT` on ( worktree baseline ) | 113552 | 1312 | 15128 | 112.1 KB / 16.0 KB |
| PRIMUS_X2_v1, topic | 113620 | 1312 | 15128 | 112.2 KB / 16.0 KB |
| **Delta** | **+68 B** | 0 | 0 | |
| PRIMUS_V5, topic ( laser off, XSHUT block not compiled ) | 102092 | 1272 | 13896 | 100.9 KB / 14.8 KB |

The whole delta is `ranging_init_L1`: 120 B -> 188 B ( `nm -S` ). Loop cost 0 ( boot only, plus `delay ( 2 )` = 2 ms
boot time ). Headroom on X2_v1: 143.8 KB flash, 24.0 KB RAM.

## Task 10: M25P16 compiled out with the XSHUT pins ( 5 Oct 2026 )

User decision: build error if both ( not a run-time guard ).

| File | Lines | Change | Why |
|---|---|---|---|
| [ranging_vl53l1x.cpp](../../../../src/main/drivers/ranging_vl53l1x.cpp#L109-L113) | 109-113 | Inside `#ifdef LASER_TOF_L1x`: `#error` if `L1X_XSHUT_OFF_GPIO` and `USE_FLASH_M25P16` or `USE_FLASHFS` are defined | With PB14 held LOW, a flash access ( app MSP dataflash read / erase ) would drive the flash's DO against it |
| ranging_vl53l1x.cpp | banner | HISTORY line | Style |
| [PRIMUS_X2_v1/target.h](../../../../src/main/target/PRIMUS_X2_v1/target.h#L59-L62) | 59-62 | `USE_FLASHFS`, `USE_FLASH_M25P16` commented out, with the reason | One owner per pin; PRIMUS_V5 keeps them ( laser off, guard silent ) |
| [PlutoPilot.cpp](../../../../PlutoPilot.cpp) | | Temporary log wrapped in `#if defined( LASER_TOF_L1x ) && defined( LASER_ALT )`, `#include "platform.h"` | PRIMUS_V5 failed to link ( `altHoldSource`, `sampleCount_L1` undefined without the laser ); found here, introduced in task 6 |

**Build.** PRIMUS_X2_v1 gate clean: 112588 / 1296 / 14988 ( text / data / bss ), 111.2 KB / 15.9 KB; the flash
driver, flashfs, its CLI and MSP handlers are gone ( -1.1 KB text, -140 B bss against task 6 ). PRIMUS_V5 builds
( 100.9 KB / 14.8 KB ). Negative test: flash defines re-enabled on X2_v1 -> build stops at
`ranging_vl53l1x.cpp:112` with the `#error` message; restored.

**Review** ( pluto-reviewer ): no BLOCKING. Fixed: stale "m25p16_init" wording in the driver boot-wait comment and
PIN_MAP conflict 8 ( the idle level comes from `spiInit ( SPI2 )` ); a note above the PRIMUS_V5 flash defines.
Noted only: `io/flashfs.cpp` is in `COMMON_SRC` without a `USE_FLASHFS` guard; X2_v1 links because gc-sections drops it.
With the flash out, the app's `MSP_DATAFLASH_SUMMARY` returns zeros ( no flash ) and dataflash read / erase get an MSP error.

## Not part of this topic, in the working tree

- `src/main/version.h`: `FW_VERSION_LENGTH` 5 -> 6, unrelated.
