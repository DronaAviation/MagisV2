# VL53L1X XRanging API - Changes

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [DESIGN](DESIGN.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

## L0X object-avoidance reference ( task 1 )

The behaviour the shared core ( task 6 ) must keep **exactly** for XRanging, taken from `applyObjectAvoidance ( )`
at [XRanging.cpp:482-592](../../../../src/main/API-Src/XRanging.cpp#L482-L592) on `5e26823`. Its only call site is
[mw.cpp:533](../../../../src/main/mw.cpp#L533), once per loop and not gated. The task 6 review checks every item.

**Constants** ( [XRanging.cpp:377-389](../../../../src/main/API-Src/XRanging.cpp#L377-L389) )

- [ ] `AVOID_RC_MAX` 25: a fixed offset, not proportional to distance
- [ ] `OA_MAX_PUSH_TIME_MS` 500, `OA_CLEAR_HOLD_MS` 500
- [ ] `OA_BRAKE_GAIN` 0.45f
- [ ] `USER_OA_BLEND_PUSH` 0.3f ( push phase **and** idle ), `USER_OA_BLEND_BRAKE` 0.2f ( brake phase )
- [ ] `AVOIDANCE_INTERVAL_MS` 100
- [ ] Threshold `_objectAoidDistance = cm * 10` ( mm ), set by `initObjectAvoidance`

**Order of operations per call**

1. [ ] `now = millis ( )`. Return if `now - lastAvoidanceRunMs < 100`. Then `lastAvoidanceRunMs = now`.
   **The timestamp updates before the `_enOA` check**, so the 100 ms period runs even while OA is off.
2. [ ] Return if `! _enOA`. In that case nothing is written and there is no `userRCassert`, so the override expires.
3. [ ] `userRoll = rcData [ ROLL ] - 1500`, `userPitch = rcData [ PITCH ] - 1500`. This reads **`rcData`, not
   `rcDataPilot`**, as today. Keep it ( behaviour unchanged ); see the observation below.
4. [ ] Lasers in the order LEFT ( filter idx 0, sign +25 ), RIGHT ( idx 1, −25 ), FRONT ( idx 2, −25 ), BACK
   ( idx 3, +25 ). Each runs only if `isXLaserInit [ laser ]`. Roll is the sum of LEFT + RIGHT, pitch the sum of
   FRONT + BACK, so two opposite lasers that both trigger cancel to 0 while `*Sense` stays true.
5. [ ] Per laser: `oaDistance`. A status other than `LASER_OK` / `LASER_OUT_OF_RANGE` resets that filter to 0 and
   returns 0. Otherwise `filterDistance`: d ≤ 0 keeps the previous value; else `( 3 × prev + d ) / 4` in integer
   arithmetic. `mapAvoidance`: d ≤ 0 or d > threshold gives 0, else ±25.
   **Changed by task 14 ( user decision, both APIs ):** `LASER_OUT_OF_RANGE` holds the filter for up to 2 ticks and
   clears it to 0 on the 3rd out-of-range tick in a row ( `OA_OOR_CLEAR_TICKS` ); a filter at 0 is seeded with the
   first valid sample instead of being quartered. All other steps are unchanged.
6. [ ] Per laser: `ledWrite ( laser, o ? HIGH : LOW )`, so the indicator LED follows the trigger.
7. [ ] Sensed: set the start timestamp only if it is 0; `last*OA = offset`, updated every sensed tick.
8. [ ] Per axis, push while `start && now - start < 500`: `user × 0.3 + offset`. Brake while
   `start && now - start < 1000`: `user × 0.2 + ( −last × 0.45 )`. Otherwise reset `start` and `last` to 0, value
   `user × 0.3`. Result in `RC_ARRAY` ( `int32_t`, assignment from float truncates toward zero ).
9. [ ] `constrain ( RC_ARRAY [ ROLL / PITCH ], -500, 500 )`.
10. [ ] `userRCassert ( ROLL, 0.0f )`, `userRCassert ( PITCH, 0.0f )`, every 100 ms tick while enabled ( authority
   0: the pilot is already blended in ).

**`disableOA`** sets `_enOA = false` and drives every enabled laser's LED low.
**`initObjectAvoidance`** marks `isXLaserInit [ l ]` for each argument `< LASER_COUNT` ( DOWN and −1 are ignored ).

**Observation ( no change in this topic ).** Step 3 reads `rcData`. The `pluto-rules` invariant is
pilot input = `rcDataPilot`. Here `rcData [ ROLL / PITCH ]` is the pilot's stick, unless user code also sets roll
or pitch with `RcCommand_Set`. Keep it for L0X equivalence, and log it as a possible follow-up.

## Task 2: VL53L1X multi-laser manager ( 16 Oct 2026 )

| File | Change |
|---|---|
| [tof_manager_l1x.h](../../../../src/main/drivers/tof_manager_l1x.h) | New. Interface parallel to `tof_manager.h`, reusing its `tofId_e` / `tofStatus_e` / `tofEntry_t` / `TOF_*` constants. `tofL1xStep_e` bring-up steps, `TOF_L1X_BOOT_ADDRESS` 0x29, `#ifndef` defaults for `L1X_SAMPLE_PERIOD_MS` / `L1X_TIMING_BUDGET_US` / `L1X_STALE_MS` ( same text as `ranging_vl53l1x.h`, so it survives that file's removal in task 4 ). |
| [tof_manager_l1x.cpp](../../../../src/main/drivers/tof_manager_l1x.cpp) | New, whole body under `LASER_TOF_L1x`. 6 static `VL53L1_Dev_t` slots ( 5592 B ), per-laser table, bounded bring-up, `tofL1xPollDown` / `tofL1xPollSides`, DOWN accessors. |
| [Makefile:281](../../../../Makefile#L281) | `drivers/tof_manager_l1x.cpp` added after `tof_manager.cpp` ( `DRONA_DRIVERS` ). |

**Bring-up ( `tofL1xBringUp ( id, from7, to7 )` ).** A bounded boot poll of `FIRMWARE__SYSTEM_STATUS` ( ≤ 10 reads,
1 ms apart ), needed because this platform's `VL53L1_WaitUs` / `WaitValueMaskEx` are empty, so `WaitDeviceBooted`
never waits. Then: model ID 0xEACC → `SetDeviceAddress ( to7 × 2 )` and a re-probe at the new address → `DataInit` →
`StaticInit` → distance mode ( Medium ) → budget 45 ms → period 50 ms → `TUNING_PROXY_MIN` read-back. Each failure
sets `BOOT_FAILED` with its `TOF_L1X_STEP_*`. Nothing starts ranging in the bring-up.

**Deviation from the task text ( deliberate ).** The address moves *before* `DataInit`. `VL53L1_SetDeviceAddress`
writes only register 0x0001 ( [vl53l1_api.c:755](../../../../lib/main/VL53L1X_API/core/src/vl53l1_api.c#L755) ), and
the FULL config level of `StartMeasurement` re-encodes the NVM copy of that register
( [vl53l1_register_funcs.c:102](../../../../lib/main/VL53L1X_API/core/src/vl53l1_register_funcs.c#L102) ). `DataInit` reads the copy from
the chip, so after the move it holds the new address.

**DOWN poll ( `tofL1xPollDown` ).** It matches `getRange_L1` line for line ( checked by the review ): a 10 ms interval;
on the first poll it starts the measurement; data-ready, then the 17-byte result read and clear-interrupt; valid
from 15 mm to under 4500 mm; the last valid range is kept across invalid results; a read error latches
( `I2C_ERROR` ), and a failed clear still records the sample, then latches. `tofL1xDownOutOfRange` is the old
`isOutofRange_L1` formula, with `status != OK` in place of `Global_Status_L1`.

**Side poll ( `tofL1xPollSides` ).** Starts only after `tofL1xStartSides`. A 10 ms interval, one laser's I2C per
call, round-robin over the running side lasers; it reads only when data is ready, then runs the stale check on every
side ( no bus traffic ).

**Review fixes ( pluto-reviewer, 0 blocking ).**

- *Should-fix:* the DOWN and side polls could both do I2C in one loop ( both 10 ms intervals re-armed from `millis ( )` ).
  Now `tofL1xPollDown` sets `downUsedBus`, and `tofL1xPollSides` skips that loop ( checked before its interval, so
  the two polls settle one loop apart ). **Call order in task 4: PollDown, then PollSides.**
- *Should-fix:* the header now says that `tofFiltCm` and `tofRawCm` both read `tofL1xDownRangeMm` ( as both read
  `NewSensorRange_L1` ), and that `tofL1xDownRawMm` is for diagnostics only.
- *Nit:* the comment on the round-robin `static_assert` says the timing is nominal ( ~55 ms per turn over 5 sides ).

**Sizes.** Stock 108276 / 1280 / 16760, byte-identical to task 1. The L1X build is unchanged too ( the manager is not
called yet, so `-gc-sections` drops it ). `tof_manager_l1x.o` is 1694 text / 1 data / 5765 bss. Gate: no new `src/`
warnings; the only warning in the new files is the vendored CMSIS `__REVSH` one that every file gets through
`platform.h`.

**For the X1Ranging caller ( task 3 ).**

1. At `x1RangingInit`: drive every requested XSHUT low, then call `tofL1xProbe ( 0x29 )`. If it answers, a chip has no
   working XSHUT: `tofL1xMarkFailed ( …, TOF_L1X_STEP_BUS_BUSY )`, as XRanging does.
2. Per laser: raise its XSHUT, wait the boot delay ( as XRanging's `XSHUT_BOOT_MS` ), then
   `tofL1xBringUp ( id, 0x29, DOWN ? 0x30 : 0x2A + n )`. DOWN goes first.
3. After the last bring-up, call `tofL1xStartSides ( )` once.
4. `tofL1xProbe` writes a 16-bit index, which would put 0x0F into register 0x01 of a VL53L0X at that address. That is
   harmless in an all-L1X build ( mixed drones are out of scope ).

## Task 3: `X1Ranging` API ( 16 Oct 2026 )

| File | Change |
|---|---|
| [X1Ranging.h](../../../../src/main/API/X1Ranging.h) | New. `X1Ranging_P` / `X1Ranging`, the same method names and signatures as XRanging, reusing `laser_e` / `laser_status_e`; doxygen per method ( VL53L1X, 50 ms period, refusals ). OA unused-laser defaults are `DOWN`. |
| [X1Ranging.cpp](../../../../src/main/API-Src/X1Ranging.cpp) | New. The L1X implementation under `LASER_TOF_L1x`, stubs otherwise ( −1 / `LASER_NOT_ENABLED` / false / no-op ). `x1RangingInit ( )` latches once. |
| [XRanging.cpp](../../../../src/main/API-Src/XRanging.cpp) | Under `LASER_TOF_L1x` only: `x1RangingAltXshut ( )` OR-ed into `xRangingGpioClaimed` / `AdcClaimed` / `PwmClaimed`; helpers `xRangingSideXshut` / `xRangingSideLed` / the pin-conflict check wrap the existing table; `xRangingInit` returns early when X1Ranging has requests. The stock `XRanging.o` code and data are byte-identical. |
| [API-Utils.h](../../../../src/main/API/API-Utils.h) | `x1RangingInit`, `x1RangingAltXshut` and the shared helpers declared next to `xRangingInit`. |
| [XRanging.h](../../../../src/main/API/XRanging.h) | `initObjectAvoidance` unused-laser default `( laser_e ) -1` → `DOWN`. Both are ≥ `LASER_COUNT` and ignored, so behaviour is the same. Clears 15 baseline warnings and the 3 that the new include would add. |
| [utils.h](../../../../src/main/common/utils.h) | `cmp16` / `cmp32`: explicit `( int16_t )` / `( int32_t )` casts on the wrap-around difference ( same code; callers `mw.cpp:429,439,447` ). Clears 68 baseline warnings and 2 new ones. |
| [PlutoPilot.h](../../../../PlutoPilot.h) | At the **repo root**: `#include` of `X1Ranging.h`, banner. |
| [Makefile:310](../../../../Makefile#L310) | `API-Src/X1Ranging.cpp` in `DRONA_API`, next to `XRanging.cpp`. |

**Bring-up ( `x1RangingInit` )** follows `xRangingInit` step by step: the fusion latch, SPI2 / flash off ( only when a
request survives the latch ), all XSHUTs low, a 0x29 busy probe before every release ( without a wire, 0x30 first
for a warm reset, then 0x29 ), DOWN → 0x30, each requested side → 0x2A + n ( n counts the requested sides, busy ones
included, as XRanging does ), `tofL1xStartSides ( )` once, then the 4 × 75 ms LED blink. Requests go into
X1Ranging's own arrays, never into `isXLaserInit`, so neither `xRangingInit` nor `applyObjectAvoidance` acts on them.

**Init order.** `altLaserFusionLatch` takes only the first call ( `altitudehold.cpp:254` ). XRanging returns early
when X1Ranging has requests, so either order of `xRangingInit` / `x1RangingInit` in main.cpp is safe ( review ).

**OA state for task 6.** `oaThresholdMm` ( cm × 10 ), `oaEnabled`, the enabled sides in `x1LaserInit [ LEFT … BACK ]`, and
range and status through `getRange` / `getStatus`. No avoidance loop runs in an L1X build yet.

**Sizes.** Stock template: 108276 / 1280 / 16760, identical. Stock + API test: 108940 / 1280 / 16764 ( the test's
prints ). L1X + API test: 118088 / 1336 / 17744. With `x1RangingInit` called ( task 4 ) the implementer measured
119504 / 1336 / 23388; the 5592 B device table is now linked. Gate clean on all three builds. The L1X `.init_array`
has only the existing `accSmooth` and `MyDevice_L1` constructors ( the second goes in task 4 ).

**Review ( pluto-reviewer ).** 0 blocking, 0 should-fix. Nits fixed: the `X1Ranging.cpp:47-48` comments cite macro names,
not line numbers; the `XRanging.h` history row is appended below. Nit kept for parity: a refused `initAlt` keeps the
FRONT LED dark ( same as XRanging, decisions log ).

**Open for task 4.**

- Call `x1RangingInit ( )` in main.cpp next to `xRangingInit ( )` ( either order ). In mw.cpp call
  `tofL1xPollDown ( )` and **then** `tofL1xPollSides ( )`.
- Remove `ranging_init_L1` ( `main.cpp:517` ) and the old poll ( `mw.cpp:845` ). They still use 0x29 and would clash
  with the bring-up.
- Refuse XRanging fully in an L1X build. Today, with no X1Ranging requests, `xRangingInit` still runs the L0X manager.
  And XRanging OA enabled with no lasers still writes and re-asserts the RC override ( stick × 0.3 ).

API test used for the compile check: [test/PlutoPilot_task3_api_compile.cpp.txt](test/PlutoPilot_task3_api_compile.cpp.txt).

## Task 4: wiring, and the old L1X path removed ( 19 Oct 2026 )

| File | Change |
|---|---|
| [main.cpp](../../../../src/main/main.cpp) | Removed the `ranging_vl53l1x.h` include, the dead `XVision` TODO comment and the `ranging_init_L1 ( )` call before `plutoInit ( )`. `x1RangingInit ( )` now runs right after `xRangingInit ( )` ( after `plutoInit`, OLED and `APIAdcInit` ). |
| [mw.cpp](../../../../src/main/mw.cpp) | Include `tof_manager_l1x.h`. `UPDATE_LASER_TOF_TASK`: with `LASER_TOF_L1x`, `tofL1xPollDown ( )` and then `tofL1xPollSides ( )`; otherwise the L0X polls, unchanged. `getRange_L1 ( )` is gone. |
| [XRanging.cpp](../../../../src/main/API-Src/XRanging.cpp) | `XRANGING_REFUSED` ( true with `LASER_TOF_L1x` ): `init`, `init ( laser )`, `initObjectAvoidance` and `enableOA` do nothing; `getStatus` returns `NOT_ENABLED` and `getRange` −1; `xRangingInit` returns at once. `_enOA` can never become true, so `applyObjectAvoidance` writes no override. |
| [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp) | Include `tof_manager_l1x.h`; `LASER_ALT` is always an `#error` ( it names both `initAlt` APIs ); the L1X accessors are `tofL1xDownNew` / `tofL1xDownClearNew` / `tofL1xDownOutOfRange` / `tofL1xDownRangeMm` ( filtered **and** raw ). *Moved here from task 5.* |
| `drivers/ranging_vl53l1x.{h,cpp}` | **Deleted** ( `LaserSensor_L1`, `XVision`, `ranging_init_L1`, `getRange_L1`, the `*_L1` globals ). Makefile line removed. |
| [PRIMUS_X2_v1 target.h](../../../../src/main/target/PRIMUS_X2_v1/target.h), [PRIMUS_V5 target.h](../../../../src/main/target/PRIMUS_V5/target.h) | Comments only: `LASER_TOF_L1x` = every laser is a VL53L1X, use X1Ranging; `LASER_ALT` is gone. |

**Sizes ( PRIMUS_X2_v1 ).**

| Build | text / data / bss | Note |
|---|---|---|
| Stock template | 108272 / 1276 / 15796 | bss −964 B ( `XVision` ), text −4 against task 1 |
| L1X template | 106584 / 604 / 19204 | only the L1X `tofDev` ( 5592 B ); the VL53L0X library and the L0X manager drop out; `xRangingInit` is 2 bytes |
| L1X + API test | 107764 / 612 / 19208 | `.init_array` holds only `accSmooth` ( `MyDevice_L1` is gone ) |
| L1X + `LASER_ALT` | build error | the new `#error` at `altitudehold.cpp:117` |

RAM in the L1X build is 19.8 KB of 40 KB, below the 22 KB budget from task 1. The gate is clean on all three builds.

**Review ( pluto-reviewer ).** 0 blocking, 0 should-fix.

- Confirmed: the L0X paths are unchanged.
- Confirmed: `ranging_init_L1` did no bus setup or delay that is now lost.
- Confirmed: the accessors match the old globals line for line ( `tofClearNew ( )` has one use, `:1115`, as a statement ).
- Confirmed: the laser task runs on every non-RX loop, so the 10 ms manager intervals set the pace.
- Nits ( for task 5 ): the `altitudehold.h:65` comment, and the dead `|| defined( LASER_ALT )` at `altitudehold.cpp:361`.

**Leftover references to `LASER_ALT` / `ranging_vl53l1x`, for tasks 8 and 11.**

- Task 8: `docs/API/XRANGING_API_WIKI.md:175` ( the `LASER_ALT` row; add "every XRanging call is refused in a
  `LASER_TOF_L1x` build" ).
- Task 11: `CLAUDE.md:138`; `dev-guide/FLIGHT_INVARIANTS.md:83-85`; `dev-guide/HARDWARE_RESOURCES.md`;
  `.claude/skills/pluto-flighttest/SKILL.md:136`; `fw-architecture-pipeline/subsystems/Main_Initialization.md` and
  `Altitude_Hold_Estimator.md` ( through `PIPELINE_UPDATE.md` ); regenerate `compile_commands.json`.

## Task 5: one fusion switch, equivalence proof ( 19 Oct 2026 )

| File | Change |
|---|---|
| [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp) | `ALT_LASER_FUSION_BUILD` removed. `_time_constant_z = 2.0f` and `altLaserFusionOn = false` to start; `altLaserFusionLatch ( )` sets both, once, for either laser family. The dead `|| defined( LASER_ALT )` and the dead inner `#else checkReading ( )` branch are removed ( every target defines `BARO` ). The fusion comment names both `initAlt` APIs. |
| [altitudehold.h:65](../../../../src/main/flight/altitudehold.h#L65) | Comment: fusion is on with `XRanging.initAlt` ( VL53L0X ) or `X1Ranging.initAlt` ( VL53L1X ). |

**Equivalence ( VL53L0X build ).** HEAD was exported with `git archive` into the scratchpad and built stock there.

| `altitudehold.o` | text / data / bss | `objdump -d -r` |
|---|---|---|
| HEAD | 7434 / 23 / 764 | reference |
| Working tree ( tasks 4 + 5 ) | 7434 / 23 / 764 | **identical** ( 0 of 2667 lines differ ) |

The only section difference is in debug info: the `wm4.*` `.debug_macro` groups of headers HEAD's object included
( `ranging_vl53l1x.h`, `stdarg.h` ). The whole HEAD diff of `altitudehold.cpp` changes no `ALT_TOF_*` constant, only
the six L1X accessor macros ( task 4 ) and the removed switch.

**VL53L1X build: the one behaviour change** against HEAD's `LASER_TOF_L1x` + `LASER_ALT` is that the fusion now
needs `X1Ranging.initAlt ( )`. The order is safe ( review ): main.cpp computes the gains with tau 2 s at `:461`; the
latch runs after `plutoInit ( )` and before `loop ( )` and calls `updateGains ( )` with `ALT_EST_TAU_S`; every reader
of `altLaserFusion ( )` ( `imu.cpp:267`, `command.cpp:125`, `altitudehold.cpp:~946` / `~1364` ) runs in the loop. An
accepted `initAlt` whose DOWN laser fails keeps the fusion on as a dropout: `tofL1xDownOutOfRange ( )` is true, so
the first `checkReading ( )` hands over to `correctedWithBaro ( )`, the same path as the L0X.

**Review ( pluto-reviewer ).** 0 blocking. Nit fixed: the dead inner `#else`. A correction to my note:
`x1RangingInit ( )` latches before its `if ( ! x1RangingRequested ( ) ) return;`, so in an L1X build the latch always
runs once.

**Sizes.** Stock 108272 / 1276 / 15796, L1X 106584 / 604 / 19204, the same as task 4. Gate clean on both builds.

## Task 6: shared object-avoidance core ( 19 Oct 2026 )

| File | Change |
|---|---|
| [ObjectAvoidanceCore.h](../../../../src/main/API-Src/ObjectAvoidanceCore.h) | New, header only ( no Makefile line ). The OA constants, now in one place ( plus `OA_FILTER_COUNT` 4 ); `template < oaCoreUser_e U > struct oaCoreState` with separate `static inline` members ( `lastRunMs`, `filtMm [ 4 ]`, `rollStartTs`, `pitchStartTs`, `lastRollOA`, `lastPitchOA`, all 0 ); `oaCoreFilter` / `oaCoreDistance` ( status gate + `getRange` ) / `oaCoreMap`; `oaCoreRun < U, Ranging > ( enabled&, thresholdMm&, laserInit*, ranging&, ledWrite )` holds checklist steps 1-10. Every original comment is kept, including the authority-0 one. |
| [XRanging.cpp](../../../../src/main/API-Src/XRanging.cpp) | The OA loop is replaced by the wrapper `applyObjectAvoidance ( )` → `oaCoreRun < OA_CORE_XRANGING > ( _enOA, _objectAoidDistance, isXLaserInit, XRanging, ledWrite )`. `enableOA` / `disableOA` / `initObjectAvoidance` are unchanged. |
| [X1Ranging.cpp](../../../../src/main/API-Src/X1Ranging.cpp) | `x1RangingApplyObjectAvoidance ( )` → `oaCoreRun < OA_CORE_X1RANGING > ( oaEnabled, oaThresholdMm, x1LaserInit, X1Ranging, ledWrite )`; a no-op stub in an L0X build. |
| [API-Utils.h](../../../../src/main/API/API-Utils.h) | Declares `x1RangingApplyObjectAvoidance`. The authority-0 and hold-window comments name both wrappers ( review nit ). |
| [mw.cpp](../../../../src/main/mw.cpp) | The call site calls `x1RangingApplyObjectAvoidance ( )` with `LASER_TOF_L1x`, else `applyObjectAvoidance ( )` as before. |

**L0X equivalence ( final stock ELF, before / after ).** `applyObjectAvoidance` is at the same address with the same
226 instruction words. The only differences are the callee name ( `oaDistance` → `oaCoreDistance < … >.constprop.0`,
same address and `bl` encoding ) and three literal-pool data addresses 4 B lower ( `_enOA`, `_objectAoidDistance`,
`isXLaserInit` ). Those moved because the filter array is 4 entries instead of `dFilt [ 6 ]`, whose entries 4-5 were
never used. `XRanging.o`: 731 / 731 instruction lines identical.

**L1X build.** `x1RangingApplyObjectAvoidance` ( 0x298 B, the same instruction sequence ) is linked, and XRanging's
`applyObjectAvoidance` is dropped. There is no `_ZGV` guard, and `.init_array` holds only `accSmooth`.

**Sizes.** Stock 108272 / 1276 / 15792 ( bss −4 ); L1X + API test 108088 / 612 / 19212. Gate clean on both builds;
the six `-Wfloat-conversion` OA warnings in XRanging.cpp are gone ( explicit `( int32_t )` casts, same truncation ).

**Review ( pluto-reviewer ).** 0 blocking, 0 should-fix. Every one of the 10 checklist steps was confirmed at its line in
`ObjectAvoidanceCore.h`, and X1Ranging's status gate, LEDs and inert-when-disabled behaviour were checked. Nits fixed:
the `API-Utils.h` comments; checklist step 8 now says `RC_ARRAY` is `int32_t`; the task 6 safety line now points at
the `rcData` observation ( the override mixes into `rcCommand`, `mw.cpp:1137`, so there is no feedback loop ).

## Task 7: the single test file ( 19 Oct 2026 ) — TEMPORARY, restore the template before committing

| File | Change |
|---|---|
| [PlutoPilot.cpp](../../../../PlutoPilot.cpp) ( repo root ) | The test: `X1Ranging.initAlt ( GPIO_5 )` and `initObjectAvoidance ( 50, LEFT, RIGHT, FRONT, BACK )` in `plutoInit`; `enableOA` in `onLoopStart`, `disableOA` in `onLoopFinish`; a `START altInit:` marker on its own tick; one log line built in a static 128 B buffer and sent as one `Monitor_Println` frame. Copy: [test/PlutoPilot_x1ranging_test.cpp.txt](test/PlutoPilot_x1ranging_test.cpp.txt). The template backup is in the session scratchpad, and git HEAD has it too. |
| [TESTING.md §2](TESTING.md) | Build and hardware setup, log fields, bench B1-B5, flight F1-F5 with pass / refute criteria, the out-of-range note. |

**Why one frame.** Each `Monitor_Print` call is its own `$D` frame ( `'$' 'D' len … checksum`, 4 B,
`Debugging.cpp:62-82` ), and `Monitor_Println` adds a 5 B newline frame. Fourteen separate fields would be ~150 B per
tick, over the budget. One line costs ~112 B with realistic values and 123 B worst case.

**Builds.** L1X + test 108576 / 604 / 19344; stock + test 109620 / 1276 / 15928. Gate clean, no warnings in `PlutoPilot.cpp`.

**Review ( pluto-reviewer ).** 0 blocking.

- Fixed in TESTING.md: B3 describes the push / brake cycle while the card is held and gives the ±1 truncation; the brake
  values are listed per side; F4 is judged by eye, because the log stops with `plutoLoop`.
- Became task 14: an out-of-range side keeps the last near distance in the filter ( an existing XRanging behaviour ).

Also checked: the budget arithmetic, `lineInt` bounds ( `INT32_MIN` included ), the API mapping ( status digits,
`Estimate_Get ( Position, Z )` = EstAlt in cm, ROLL 0 / PITCH 1, C linkage ), the push signs, and that Dev Mode off
clears the override ( `mw.cpp:1204-1210` ).

## Task 14: OA filter, out of range and seeding ( 19 Oct 2026, both APIs, user decision )

| File | Change |
|---|---|
| [ObjectAvoidanceCore.h](../../../../src/main/API-Src/ObjectAvoidanceCore.h) | `OA_OOR_CLEAR_TICKS` 3; `oaCoreState < U >::oorTicks [ 4 ]` ( uint8_t, saturating ). `oaCoreDistance`: `LASER_OK` resets the count and filters; `LASER_OUT_OF_RANGE` holds the filter for up to 2 ticks and clears it on the 3rd in a row; any other status ( failed, not enabled ) clears at once. `oaCoreFilter`: a filter at 0 is seeded with the first valid sample. Comments rewritten ( the filter is an IIR, 3/4 + 1/4 ). |

**Before ( HEAD XRanging, kept by task 6 ).** An out-of-range side read −100; the filter returned the held value
forever, so a removed obstacle stayed "seen" and the push / brake cycle repeated until a valid far sample came. After a
reset, the first sample was quartered: a wall at 2 m read 500 mm and fired one false push at a 50 cm threshold.

**Why the debounce ( review ).** With an instant clear, an out-of-range tick at a real obstacle drops that tick's push,
while the brake phase always applies its full −last × 0.45. Trace with LEFT at 300 mm and out of range on ticks 1, 3 and 4:
push +25 on ticks 0 and 2 only ( 50 units ), then brake −11.25 × 5 ( 56 units ). The net roll goes toward the obstacle.
With the debounce all five push ticks are kept, and a removed obstacle is released on the 3rd out-of-range tick ( ≤ 300 ms ).

**Code proof.** The stock `applyObjectAvoidance` disassembly with relocations is identical to task 6 ( 0 of 241 lines ). Only
`oaCoreDistance` changed ( 0x4c → 0x74 ), plus 4 B of counters per API. The new disassembly was read and matches.

**Sizes.** Stock + test 109660 / 1276 / 15932; L1X + test 108616 / 604 / 19348. Gate clean on both; `.init_array` has
only `accSmooth`, and there are no `_ZGV` guards.

**Docs.** Checklist step 5 ( above ) has the task 14 change; the TESTING.md §2 out-of-range note and B3 are rewritten.
For the task 8 wiki, in **both** XRanging and X1Ranging: out of range releases an obstacle within ~300 ms.

**Review.** First pass: the instant clear plus full brake ( should-fix ), so the debounce was added; the docs had not
been updated ( should-fix, done ); the IIR wording ( nit, done ). Re-review: 0 blocking, 0 should-fix. Open nit: no
state reset on `enableOA` ( the same at HEAD; decisions log ).

## Task 15: no build flag, run-time laser family ( 19 Oct 2026 )

| File | Change |
|---|---|
| [API-Utils.h](../../../../src/main/API/API-Utils.h) | `rangingFamily_e` ( `RANGING_NONE` / `VL53L0X` / `VL53L1X` / `BOTH` ), `rangingFamilyLatch ( )`, `rangingFamily ( )`. |
| [XRanging.cpp](../../../../src/main/API-Src/XRanging.cpp) | The family latch: XRanging counts as used with `altRequested` or any `isXLaserInit`, X1Ranging with `x1RangingRequested ( )`; latched once. `xRangingInit`: for the VL53L1X family or BOTH it clears its requests ( and `altLatched` ) and returns without the fusion latch. NONE / VL53L0X run today's code. `XRANGING_REFUSED` and the `altRequest` `#if` are removed. The pin-claim helpers are unconditional. |
| [X1Ranging.cpp](../../../../src/main/API-Src/X1Ranging.cpp) / [X1Ranging.h](../../../../src/main/API/X1Ranging.h) | The gate and the stub block are removed. `x1RangingInit` runs only for the VL53L1X family, and otherwise clears its requests. **Temporary:** it latches the fusion off ( `altLaserFusionLatch ( false )` ) until task 17. The docs say it is chosen by calling it, and that using both APIs refuses both. |
| [tof_manager_l1x.cpp](../../../../src/main/drivers/tof_manager_l1x.cpp) / `.h` | The gate is removed, so it is compiled in every build. |
| [main.cpp](../../../../src/main/main.cpp) | `rangingFamilyLatch ( )` → `xRangingInit ( )` → `x1RangingInit ( )`. |
| [mw.cpp](../../../../src/main/mw.cpp) | OA: NONE / VL53L0X call `applyObjectAvoidance`, VL53L1X calls `x1RangingApplyObjectAvoidance`, BOTH calls neither. Laser task: both managers are polled every pass ( L0X, then L1X PollDown → PollSides ); each returns at once when nothing is up. |
| [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp) | `#error` for `LASER_TOF_L1x`. The L1X branches are now dead and are left for task 17. |
| Both `target.h` | One comment: VL53L1X needs no define, use X1Ranging. |
| [PlutoPilot.cpp](../../../../PlutoPilot.cpp) ( test ) | The build line, plus the "fusion off until task 17" warning; the copy in `test/` is updated. |

**What both APIs see with BOTH.** No laser starts and no XSHUT is driven. SPI2 and the flash stay on, because both inits
return before releasing them. Nobody latches the fusion ( baro, tau 2 s ), the getters return −1 / `LASER_NOT_ENABLED`, and
OA is inert.

**Equivalence.** `applyObjectAvoidance` is identical to task 14. `xRangingInit` changes only by the family block at the top;
the implementer compiled a copy without the block to check this.

**Sizes.** Stock + test 121496 / 1316 / 21740 ( +11.8 KB flash for the VL53L1X library and manager, +5.8 KB bss for
the second device table, which task 16 shares ). Template 119944 / 1316 / 21604. Both-APIs compile check 120116 / 1324 /
21604. Gate clean; `.init_array` has only `accSmooth`.

**Review ( pluto-reviewer ).** 0 blocking.

- *Should-fix 1:* with the VL53L1X family the fusion was on but read the idle VL53L0X entry, so it flew the fusion's baro
  branch while reporting `fu:1`. Fixed: latched off until task 17.
- *Should-fix 2:* TESTING.md still told the reader to uncomment the define. Fixed, with the until-task-17 warning in the
  test header too.
- *Should-fix 3:* both wikis still describe the flag. Moved to task 8.
- *Nit:* SPI2 stays on with BOTH. Moved to the task 8 wiki.

## Task 17: two compiled fusion copies, picked at boot ( 19 Oct 2026 ) — done before task 16 ( user: no flag left )

| File | Change |
|---|---|
| [altitudehold.cpp](../../../../src/main/flight/altitudehold.cpp) | The `#if defined( LASER_TOF_L1x )` constants / accessor block became traits `AltTofL0x` / `AltTofL1x`: `static constexpr` constants with the macros' names, types and expressions; `always_inline` accessors `tofNew` / `tofClearNew` / `tofOutOfRange` / `tofReseed` / `tofFiltMm` / `tofRawMm`; `RETURN_GUARD`. Templates `checkReadingFor<T>` ( `noinline` ), `altShiftFrame<T>`, `tofWindowReset / Push / Mismatch<T>`, with the window ring as variable templates per copy. The L1X return guard is `if constexpr ( T::RETURN_GUARD )`. `checkReading ( )` ( C linkage ) dispatches on `altLaserFusionL1x`. |
| [altitudehold.h](../../../../src/main/flight/altitudehold.h) | `void altLaserFusionLatch ( bool on, bool vl53l1x )`. |
| [XRanging.cpp](../../../../src/main/API-Src/XRanging.cpp) / [X1Ranging.cpp](../../../../src/main/API-Src/X1Ranging.cpp) | They latch `( altOn, false )` and `( altOn, true )`; task 15's temporary off is gone. |
| [tof_manager_l1x.h](../../../../src/main/drivers/tof_manager_l1x.h) | A comment names the new accessors ( review nit ). |
| Test `PlutoPilot.cpp`, TESTING.md §2 | The "fusion off until task 17" warning is replaced: the VL53L1X fusion copy runs ( review should-fix ). |

**Equivalence.** Both references were compiled with the build's own flags ( `scratchpad/t17/cc.sh` ):

- **VL53L0X:** `altitudehold_t15.o`, the state before this task, whose code is identical to HEAD per task 5.
- **VL53L1X:** today's file compiled alone with `-DLASER_TOF_L1x`, from a copy with only the `#error` removed. That is the
  fusion log-2 flew.

| Copy | Against | Functions identical ( instructions + relocations ) |
|---|---|---|
| VL53L0X | `altitudehold_t15.o` | 38 / 39; `checkReadingFor<AltTofL0x>` = old `checkReading`, 708 lines |
| VL53L1X | `altitudehold_l1x_ref.o` | 38 / 39; `checkReadingFor<AltTofL1x>` = old L1X `checkReading`, 776 lines |

The one differing function in each is `altLaserFusionLatch`, by the store of the family flag. The reviewer also checked the
relocations unmasked: each copy calls only its own manager's accessors.

**State.**

- **Per copy:** the 25 function statics of `checkReadingFor<T>` ( `baro_offset`, `lastGoodTofMs`, `returnCount`,
  `leftOnDropout`, the step / steady / suspect state, and for the L1X only `retSteady*` ) and the 5-variable window ring
  ( 24 / 14 entries ).
- **Shared:** `altSourceLaser`, `altStepPending` ( read by `altHoldSource ( )` ), `ToF_Height`, the baro values and the
  frame values. Only one copy runs per boot, and nothing outside the fusion read the per-copy state.

**Dispatch.** `checkReading ( )` runs only when `altLaserFusion ( )` is true. Before the latch, or latched off, the path
is the baro one ( `checkBaro ( )` ), as before. The latch is first-call-wins. NONE / VL53L0X → `( altOn, false )`;
VL53L1X → `( altOn, true )`; BOTH → no latch.

**Sizes.** Stock + test 124156 / 1316 / 21972: +2660 B flash, +232 B RAM ( the L1X ring and statics ). Gate clean;
`.init_array` has only `accSmooth`. `grep LASER_TOF_L1x src/main` finds only the `#error`, a history line and the
`target.h` notes that it is a build error.

## Task 16: one shared RAM area for both device tables ( 19 Oct 2026 )

| File | Change |
|---|---|
| [tof_dev_area.h](../../../../src/main/drivers/tof_dev_area.h) | New. `union tofDevArea_u { VL53L0X_Dev_t l0x [ TOF_COUNT ]; VL53L1_Dev_t l1x [ TOF_COUNT ]; }`, extern ( C linkage ). Both ST header sets compile together without conflict. |
| [tof_dev_area.cpp](../../../../src/main/drivers/tof_dev_area.cpp) | New. The one definition, with no initialiser ( .bss, no constructor ). |
| [Makefile:282](../../../../Makefile#L282) | `drivers/tof_dev_area.cpp` in `DRONA_DRIVERS`, which all three targets build. |
| [tof_manager.cpp](../../../../src/main/drivers/tof_manager.cpp) | The static `tofDev [ 6 ]` is removed; it uses `tofDevArea.l0x`. `tofManagerBringUp` returns false unless `rangingFamily ( ) == RANGING_VL53L0X`. |
| [tof_manager_l1x.cpp](../../../../src/main/drivers/tof_manager_l1x.cpp) | The static `tofDev [ 6 ]` is removed; it uses `tofDevArea.l1x`. `tofL1xBringUp` returns false unless `rangingFamily ( ) == RANGING_VL53L1X`. |

**The rule: only the latched family touches the area.**

- A device struct is touched in a bring-up, or after it, with the entry at status OK. The status tables stay separate
  ( outside the union ).
- Status OK is set only by that manager's bring-up, and that bring-up is gated on the family.
- `NOT_ENABLED` is 0, so the zeroed table of the manager that never ran stays inert. Its polls ( called every pass ) return
  before any device access.
- Neither reset writes the area.
- With BOTH, both inits return before any reset or bring-up.
- No other code keeps a device pointer: the probes use raw I2C reads.
- There is no cast between the members, so strict aliasing ( on in this build ) holds.

**Equivalence ( disassembly against the pre-task objects, table symbol normalised ).**

| Manager | Identical | Changed |
|---|---|---|
| VL53L0X | 13 / 14, PollDown / PollSides / StartSides included | `tofManagerBringUp`: the family gate, `bl rangingFamily; cmp r0, #1; bne`, +8 B; ~10 cycles per bring-up, nothing in the loop |
| VL53L1X | 15 / 16 | `tofL1xBringUp`: the gate and register reallocation, +32 B |

**Sizes.** 124196 / 1316 / 19860 ( text +40, bss −2112 against task 17 ). `nm`: one `tofDevArea` of 0x15d8 and no
separate 0x840 table. Gate clean; `.init_array` has only `accSmooth`.

**Review ( pluto-reviewer ).** 0 blocking, and no corruption path. Should-fix: the topic docs did not mention the area,
now here, and task 11 stages it in `PIPELINE_UPDATE.md`. Nits: the new banners lacked Created Date / Last Modified
( fixed ); the wrapped HISTORY row in the managers was accepted.

## Task 8: API wikis and version ( 19 Oct 2026 )

| File | Change |
|---|---|
| [X1RANGING_API_WIKI.md](../../../API/X1RANGING_API_WIKI.md) | New. Choosing the laser family ( the API that enables lasers; both → both refused, SPI2 / flash stay on ), the lasers and pins, functions, status timing ( DOWN reads out of range until its first sample, a side laser reads OK for up to 160 ms ), altitude hold incl. the VL53L1X return guard ( 50 cm agreement, or 25 cm steady for 2.5 s; 3 good samples ), the `initAlt` refusals, the pins it takes over ( `GPIO_5` / `GPIO_8` LEDs, side XSHUTs not reserved ), wiring rules, OA ( EXTERNAL not avoided, LED rewrite, the 0.3 s out-of-range release ), examples, changes in 1.8.0. |
| [XRANGING_API_WIKI.md](../../../API/XRANGING_API_WIKI.md) | 1.8.0: the family choice and the both-refused rule, the `initAlt` cancel by an X1Ranging call that enables a laser, the OA out-of-range release, the `DOWN` default, the shared OA core and device area. |
| [Makefile](../../../../Makefile) | `FW_Version` 3.15.0, `API_Version` 1.8.0. |

**Compliance.** One agent checked every statement against the code; the 30-agent workflow was skipped because of the session size guideline. First pass ( flag design ): 66 match, 3 divergences, 5 reverse gaps. Re-check on the no-flag code: ~70 match, 1 divergence, now fixed: only calls that enable a laser choose the family, and reads, `enableOA` and a refused `initAlt` do not. 5 statements cannot be checked from code: Medium reach, VL53L0X period, the warm-reset address, no visible jump at the handover, and the side rate under jitter.

## Task 11: docs outside the pipeline folder ( 19 Oct 2026 )

| File | Change |
|---|---|
| [CLAUDE.md](../../../../CLAUDE.md) | Laser bullet: `X1Ranging.initAlt` for the VL53L1X; the latch picks the fusion copy; `LASER_TOF` / `LASER_ALT` / `LASER_TOF_L1x` are build errors. New bullet: the laser family is chosen at run time; both APIs → both refused; the shared device area. |
| [PIN_MAP.md](../../PIN_MAP.md) | I2C1 lasers: VL53L1X ( X1Ranging ) use the same addresses and XSHUT pins, one family per boot, VL53L1X poll cost. |
| [FLIGHT_INVARIANTS.md](../../dev-guide/FLIGHT_INVARIANTS.md) | The down-laser is chosen by `XRanging.initAlt` or `X1Ranging.initAlt`; both → baro; the two compiled fusion copies. |
| [HARDWARE_RESOURCES.md](../../dev-guide/HARDWARE_RESOURCES.md) | The VL53L1X line: X1Ranging over `tof_manager_l1x`, run-time family, the shared device area. |
| [pluto-flighttest SKILL.md](../../../../.claude/skills/pluto-flighttest/SKILL.md) | Ground truth: `LASER_TOF` is gone; a side laser pointed down ( `init ( EXTERNAL )` ) gives a height reference that never reaches the estimator. |

`compile_commands.json` is not patched: it is a stale, Linux-generated artifact ( decisions log ).
The pipeline docs are staged in [PIPELINE_UPDATE.md](PIPELINE_UPDATE.md) ( applied at commit ).

**Task 11 compliance fixes ( one agent, ~230 citations, 137 edges ).** PIPELINE_UPDATE U2 now cites `mw.cpp:1410`; the
probe count is "at most 7 ( 6 at 0x29, 1 at 0x30 )"; "until the next restart" replaces "power cycle" / "power-off"
( PIPELINE_UPDATE ×3, HARDWARE_RESOURCES ); in HARDWARE_RESOURCES, SPI2 is switched off by the latched family
( XRanging or X1Ranging ); the pluto-flighttest example define is `BARO` ( `LASER_TOF` is an `#error` ).

## Task 12: topic review ( 19 Oct 2026 )

pluto-reviewer on the whole working tree against HEAD: **0 blocking**, nothing in `src/` to fix. Applied: the README
summary was rewritten for the no-flag design ( it still said "live only with `LASER_TOF_L1x`" ); the TASKS done-when now
says one image; the `tof_manager_l1x.cpp` comment no longer cites the deleted `ranging_vl53l1x.cpp`; `Created Date` added
to the `tof_manager_l1x` banners. Gate re-run: clean, 124100 / 1316 / 19860 ( test file ). Open risk kept: the warm reset
with no DOWN XSHUT wire may meet a chip that is still ranging; the worst case found in the code is BOOT_FAILED ( baro ).
