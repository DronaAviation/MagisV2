# MagisV2 control-path invariants

Each entry: the rule, where it lives, and the failure it prevents. Line numbers
were correct at the time of writing — confirm the symbol, not the line.

---

## 1. Pilot stick input is `rcDataPilot[]`

- **Declared:** `src/main/rx/rx.cpp:84`
- **Snapshot taken:** `src/main/rx/rx.cpp:564`, before user code can write `rcData`
- **Consumed:** `src/main/mw.cpp:1055`, `1058`, `1103`; `API-Src/API-Utils.cpp:166`

`rcData[]` is written by the override path itself. Reading it for throttle reads
back your own output, which turns the cross-fade into a feedback loop. Anything
that wants to know *what the pilot is doing* reads `rcDataPilot[]`.

## 2. User RC overrides expire unless re-asserted

- **Assert:** `userRCassert ( channel, pilotAuthority )` — `API-Src/API-Utils.cpp:159`
- **Watchdog:** `resetUserRCflag ( void )` — `API-Src/API-Utils.cpp:194`
- **Timeout:** `max ( 250 ms, 2 × userLoopFrequency )`
- **Public entry:** `RcCommand_Set` — `API/RC-Interface.h`, `API-Src/RC-Interface.cpp:143,166`

`RcCommand_Set` stores into `RC_ARRAY[]` and latches `userRCflag[]`. User code
releases a channel by *not calling it*. Code that sets an override once at init
and expects it to persist will see it silently drop after a quarter second.

## 3. The override already cross-fades against the pilot

- **Blend:** `applyUserRcOverride()` in `src/main/mw.cpp`, every loop after `annexCode()`
- **Formula:** `final = command × (1 − d) + pilot × d`, `d → 1.0` at
  `USER_RC_STICK_TRAVEL` counts off centre
- **Constant:** `USER_RC_STICK_TRAVEL 200` — `API/API-Utils.h:59`
- **Span computed:** `src/main/mw.cpp:1065`

Do not add a second blend on top. A feature that blends the pilot in itself
asserts with authority `0.0f` instead — see `applyObjectAvoidance()`,
`API-Src/XRanging.cpp:380-381`.

## 4. The barometer ground datum freezes on arm

- **Guard:** `throttleRaisedSinceArm` — `src/main/mw.cpp:174` (decl), `396` (set),
  `555` (clear), `592` (test)

The ground zero tracks while disarmed and must freeze on arm. Re-zeroing in
flight silently redefines what "altitude" means mid-flight, and the aircraft
chases the new datum.

## 5. Barometer pressure is compensated relative to the arm instant

- **Constants:** `src/main/sensors/barometer.cpp:384-386`
  - `BARO_COMP_THROTTLE_PA_PER_COUNT 0.0086f` — rotor inflow
  - `BARO_COMP_TEMP_PA_PER_DEGC 2.10f` — die temperature (measured −2.17 to
    −2.42 Pa/°C on PRIMUS_V5)
  - `BARO_COMP_LIMIT_PA 25.0f` — clamp
- **Applied:** `barometer.cpp:410-413`

Both terms are zero at arm and clamped. Altitude conversion uses a fixed ISA
temperature deliberately — do not "fix" it to use measured temperature, the
temperature term is already handled in the pressure domain.

## 6. ALT_HOLD: the stick moves the setpoint, it never replaces the loop

- **Controller:** `calculateAltHoldThrottleAdjustment()` —
  `src/main/flight/altitudehold.cpp:416`, called from `598` and `712`
- **Stick limits:** `ALT_MAX_CLIMB_CMS 40`, `ALT_MAX_DESCENT_CMS 30` —
  `altitudehold.cpp:248-249`, selected at `348`
- **Rate selection:** `altitudehold.cpp:487-488`

`AltHold` moves at a ramped `altRate`, fed forward to the velocity loop. The
rate comes from one of three sources:

| Source | Limits |
|---|---|
| Stick | `ALT_MAX_CLIMB_CMS` / `ALT_MAX_DESCENT_CMS` |
| Goal profile (take-off, `setAltitude()`, MSP) | `ALT_CMD_MAX_CLIMB_CMS` / `ALT_CMD_MAX_DESCENT_CMS` |
| Landing | `ALT_LAND_MAX_DESCENT_CMS`, flown as `(landThrottle − 1500) / 4` |

**Landing must not be routed through the stick limits** (`altitudehold.cpp:260`,
`345`). At slow descent rates the in-ground-effect baro drift masks the descent
and touchdown is never detected — the aircraft hovers a few cm up indefinitely.

The controller is held in reset while disarmed, and idle-held after stick-arming.

## 7. `Monitor_Print`: keep under ~130 bytes/tick with the app connected ( ~180 disconnected the app )

- **API:** `src/main/API/Debugging.h:45,52,78`

Output goes into the MSP UART's 256-byte TX ring. `uartWrite()` does not check
for full, so once a tick exceeds the ring the **oldest unsent bytes — the start
of the line — are overwritten**. The symptom is a log that looks truncated at
the front or interleaved, not a dropped line. Count bytes before adding a field.

## 8. DMA channels are owned through the registry

- **API:** `src/main/drivers/dma_registry.h:52,57,62,67`
  - `bool dmaClaim ( DMA_Channel_TypeDef *channel, dmaOwner_e owner )`
  - `void dmaRelease ( DMA_Channel_TypeDef *channel )`
  - `bool dmaIsFree ( DMA_Channel_TypeDef *channel )`
  - `dmaOwner_e dmaGetOwner ( DMA_Channel_TypeDef *channel )`

ADC DMA is **lazy** — a channel is claimed only when a `Peripheral_Init(ADC_x)`
pin on that ADC is actually used. Reference: `docs/fw-development-reference/DMA_MAP.md`.

Known hazards that have each cost a flight: ADC1/ADC2 share a common register
block, and the ADC has no clock out of reset. The F3 internal temperature sensor
has a **negative** slope.

## 9. Real-time budget

- 72 MHz Cortex-M4, **256 KB flash / 40 KB RAM**, hard-float `fpv4-sp-d16`, `-Os`, LTO off.
- No dynamic allocation anywhere in the control path.
- `loop()` work is bounded; periodic work goes through `executePeriodicTasks()`.
- No blocking waits or busy-spins in the control path.
- Check headroom with `make TARGET=<target> memory` after any sizeable addition.

## 10. WS2812B RGB LED ownership

- `RGB_Init ( RGB_1..RGB_8, led_count )` picks one of 8 vetted
  `{pin, timer channel, DMA channel}` slots — LUT `ws2811HwTable[]` in
  `drivers/light_ws2811strip_stm32f30x.c`. PA15 is the default.
- `RGB_Control ( RGB_USER | RGB_SYSTEM )` switches ownership between user code
  and `rgbSystemTick()` (`API-Src/RGB-LED.cpp`, called from `mw.cpp` every loop).
- The Cleanflight `LED_STRIP` feature **stays off** — enabling it corrupts
  config and BARO.
- Per-pin conflict matrix: `docs/fw-development-reference/WS2812_RGB.md`.

## 11. Interrupts, FPU and stack

There is no RTOS. Concurrency is interrupt handlers pre-empting `loop()`.

### Critical sections

- **Idiom:** `ATOMIC_BLOCK ( NVIC_PRIO_x ) { … }` — `src/main/common/atomic.h:73`.
  It raises BASEPRI (`BASEPRI_MAX`) to the given priority for the block and
  restores it on every exit path. In use: `drivers/timer.cpp:347`, `736`.
  - BASEPRI masks by **group (pre-emption) priority** only: with
    `NVIC_PriorityGroup_2`, `ATOMIC_BLOCK ( NVIC_PRIO_TIMER )` (1,1) also holds
    off UART2/3 TX DMA and USB wake-up (1,0).
  - **Priority 0 masks nothing.** `NVIC_PRIO_I2C_*` are `(0,0)` = 0; use
    `NVIC_PRIO_MAX` (`nvic.h:28`, "can't use 0") when you need everything held off.
  - The block is a `for` statement: `break` inside it leaves the block, not an
    enclosing loop.
- **Priorities:** all in `src/main/drivers/nvic.h` (`NVIC_PriorityGroup_2`).
  Timers and UART1 at `1,1`, USB `2,0`. `NVIC_PRIO_I2C_EV/ER` (`0,0`) are
  defined but unused: I2C is polled (see ISRs below).
- `__disable_irq ( )` appears only in `io/serial_1wire.cpp` (ESC passthrough,
  motors not flying). In flight code it would also hold off the RX capture timer
  (PPM/PWM input), UART/DMA, EXTI data-ready and USB ISRs. Motor PWM itself runs
  in timer hardware with no ISR.

`volatile` stops the compiler caching a value; it does not make a 64-bit value,
a struct or `x |= flag` atomic. A 32-bit aligned load or store is atomic on the
M4; anything more needs the block.

### ISRs

Copy data, set a flag, return. Bus transactions, `Monitor_Print`, float-heavy
maths and waits belong in `loop()` or `executePeriodicTasks()`.

**I2C is polled, not interrupt-driven** (`drivers/bus_i2c_stm32f30x.c` spins on
`I2C_GetFlagStatus`; there is no I2C IRQ handler). Every transaction blocks the
loop for its full duration, so budget it with `micros ( )` like any other work.

### FPU

- `-mfpu=fpv4-sp-d16 -mfloat-abi=hard`: single precision in hardware, **double
  in software** (tens of cycles per operation, more for `sqrt`/trig).
- The build passes `-fsingle-precision-constant` (Makefile `ARCH_FLAGS`), so a
  bare `0.5` is already single precision. Write `0.5f` anyway for clarity; it is
  style, not a correctness finding.
- The real double sources: the C maths calls `sqrt`/`fabs`/`sin`/`atan2`/`pow`
  from `<math.h>` (use `sqrtf`, `fabsf`, `sinf`, `atan2f`, `powf`), `double`
  variables and casts, and passing a float to a variadic function (`printf`
  family). `-Wdouble-promotion` reports most, not all, of these.
- The `Monitor_Print` double overload exists for printing only.
- **Float helpers already exist** in `src/main/common/maths.h` / `maths.cpp`:
  `sin_approx` / `cos_approx` (order-9 polynomial while `FAST_TRIGONOMETRY` is
  defined, else `sinf` / `cosf`), `constrainf`, `safe_asin` (clamps to ±π/2,
  returns 0 for NaN), `degreesToRadians ( int16_t )`, `radians` / `degrees`,
  `M_PIf`, `quickMedianFilter3..9`. Use them rather than a new local version.
- **Use `M_PIf`, not `M_PI`.** `flight/pid.h:89` redefines `M_PI` as a float
  literal, overriding `<math.h>`'s double one, so `M_PI` means different things
  depending on include order. Existing uses (`filter.cpp:43`, `lowpass.cpp:34`)
  cast to `float` and are safe; new code should not rely on that.

### Stack

- `src/main/target/stm32_flash.ld`: `_Min_Heap_Size = 0`, `_Min_Stack_Size = 0x400`.
  The stack grows down from `_estack` (top of the 40 KB RAM) into whatever
  `.data` + `.bss` leave free. The 1 KB figure only makes the link fail if less
  than that remains.
- There is no MPU guard: an overflow silently overwrites the top of `.bss`, which
  shows up as unrelated globals changing value.
- Keep large buffers `static` (they then show in the RAM figure). Local arrays
  over ~128 B in the control path or deep call chains are a finding.

### No watchdog

No IWDG is configured anywhere in `src/main`. That is a choice: a watchdog reset
in flight restarts the board with the motors stopped. Adding one needs a plan
for the armed case, not just a kick in `loop()`.

### Timing

`micros ( )` (`drivers/system.c:104`, also exposed in `API/Scheduler-Timer.h`)
is the timebase. Bracket new work with it to measure cost against `looptime`.
