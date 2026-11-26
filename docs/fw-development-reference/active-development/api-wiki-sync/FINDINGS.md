# Findings: code and header defects found while syncing the wiki

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md)

This topic edits documentation only. Everything below was found by cross-checking the wiki against
`src/main/API/*.h` and `src/main/API-Src/`, and is recorded here as a candidate for its own `fix` topic.
**Nothing here is fixed by this topic.** The testing team has not reported any of these, so each is a
code-reading finding, not a confirmed field failure.

Each row says how the wiki will describe the function in the meantime ( user's decision, 15 Nov 2026:
mark "not available in 3.15", no example, and log the finding ). **F9 was fixed on 15 Nov 2026** before the wiki work
resumed, so the Oled page documents `Oled_Print ( )` as working from API 1.8.10 instead of marking it unavailable.

## Confirmed in the source

| # | Defect | Proof | Wiki wording for 3.15.11 | Candidate topic |
|---|---|---|---|---|
| F1 | `SPI_Init ( )` with no arguments is declared but never defined, so user code that calls it does not link | declared `API/Serial-IO.h:207`; definitions are `SPI_init ( )` at `API-Src/Serial-IO-SPI.cpp:84` and the 3-argument `SPI_Init` at `:90`; no other definition in `src/` | document only the 3-argument form; note the no-argument form is not available in 3.15 | `fix`: same class as the 1.8.3 `RcCommand_get` rename ( `rc-command-get-api` ) |
| F2 | `Motor_SetDir ( M5..M8 )` writes through a NULL GPIO pointer: `motors_gpio [ 2..5 ]` are never set. **On the PRIMUSX2 build only**, the firmware does it by itself: `Motor_Set ( M5..M8 )` with the ARM switch off sets `usingMotorAPI`, and at the next ARM switch-on `MotroWakeUp ( )` calls `Motor_SetDir ( M5 / M6 / M8 )`. That call is under `#ifdef PRIMUSX2` ( corrected in task 17: the task 10 text said every target ). On PRIMUS_X2_v1 and PRIMUS_V5, `Motor_Set ( M5..M8 )` works: it drives a flight-motor output of the disarmed drone. | [Motor.cpp:207-216](../../../../src/main/API-Src/Motor.cpp#L207); `#ifdef PRIMUSX2` around `MotroWakeUp ( )` at [mw.cpp:1332-1334](../../../../src/main/mw.cpp#L1332), body at [flight/motor.cpp:107-112](../../../../src/main/flight/motor.cpp#L107); `TARGET_FLAGS = -D$(TARGET)` ( Makefile:150 ) | `Motor_SetDir ( M5..M8 )` "not available"; `Motor_Set ( M5..M8 )` documented as the flight-motor outputs for projects that do not fly, at the user's own risk ( user decision, 16 Nov 2026 ); PRIMUSX2 crash under Known limitations | `fix`: bound `Motor_SetDir` to M1 / M2 |
| F3 | `flight/motor.cpp:108-111` sets the direction of M5 twice and never M7 | source read ( scout B ) | not user-visible: no wiki text | same topic as F2 |
| F4 | `Set_LED ( STATUS, TOGGLE )` does nothing: the `STATUS` branch handles OFF and ON and returns | `API-Src/Status-LED.cpp:43-55` | document OFF and ON ( stop / resume the firmware status pattern ); TOGGLE listed as having no effect in 3.15 | `fix`, small |
| F5 | `RcData_Get ( void )` returns the address of a local array, which GCC compiles to NULL | `API-Src/RC-Interface.cpp:82-88` | "returns NULL in 3.15; use `RcData_Get ( channel )`" | `fix`, same shape as `rc-command-get-api` |
| F6 | Every `Uart_*` call dereferences NULL when `openSerialPort` fails: ELRS mode holds USART2 as MSP, UWB localisation already opened it ( `main.cpp:587-590` ), `Uart_Init` was never called, or **`Uart_Init` was called a second time** ( the port is then in use, so the second call stores NULL over the good pointer; `onLoopStart ( )` re-runs make this easy ). On STM32 address 0 maps to flash, so this is undefined behaviour rather than a guaranteed fault ( task 9 ) | `API-Src/Serial-IO-Uart.cpp:69-79`; `io/serial.cpp:260-262` returns NULL | "the Uart API is not available in `Rx_ELRS` mode, and `Uart_Init` must succeed before any other call" | `fix`: NULL guard in the five `Uart_*` functions |
| F7 | BlackBox is never built: `// #define BLACKBOX` in all three targets | `target/PRIMUS_X2_v1/target.h:154`, `PRIMUS_V5/target.h:155`, `PRIMUSX2/target.h:155` | the BlackBox section is marked "not built in 3.15" | decision needed: enable, or remove the API |
| F9 ( **fixed** 15 Nov 2026, topic [oled-print-linkage](../oled-print-linkage/README.md), FW 3.15.12 / API 1.8.10 ) | `Oled_Print ( )` could not link from user code: `Oled.h` declares it with C++ linkage, but it is defined in a C file, so the object exports the C symbol `Oled_Print`, not the C++ name the call needs. Found by `wiki_check.py` ( task 1 ). | declared `API/Oled.h:94` ( no `extern "C"` ); defined `io/oled_display.c:536`; `arm-none-eabi-nm` on `oled_display.o` shows `T Oled_Print`; link: `undefined reference to Oled_Print(unsigned char, unsigned char, char const*)` | documented as working from API 1.8.10, with its mode and range guards | done: `extern "C"` on the declaration ( the user chose this over a wrapper in `API-Src/Oled.cpp` ) |
| F10 | `Sensor_Get ( Gyroscope, Net_Acc )` and `Sensor_Get ( Magnetometer, Net_Acc )` returns an undefined value: those two `case` branches `break` for `Net_Acc` and the function falls off its end with no `return`. Found while verifying task 5 ( already listed as a known gap in `docs/API/FC_DATA_API_WIKI.md` ). | [API-Src/FC-Data-Sensors.cpp:63-72](../../../../src/main/API-Src/FC-Data-Sensors.cpp#L63): `if ( _axis != Net_Acc ) return ...; break;` then the `switch` ends, then the function | Known limitations on the FC-Data page: "pass only X, Y or Z with these two sensors" | `fix`: `return 0` after the `switch`, or for `Net_Acc` on those two |
| F11 | The multi-byte `I2C_Read ( )` returns the byte count on success and 0 on failure, never the negative error its header mentions; its `uint32_t length` ( and `I2C_Write`'s ) is passed to a `uint8_t len`, so 256 transfers nothing. Found in task 9; **corrected in task 17**: the first text said it returns 1 / 0, and the Serial-IO example tested `== 1`, which a 2-byte read never returned. | `i2cRead` is `uint8_t` and ends `return recived;` ( [bus_i2c_stm32f30x.c:330-402](../../../../src/main/drivers/bus_i2c_stm32f30x.c#L330) ), 0 from `i2cTimeoutUserCallback` on a timeout ( `:123-131` ); `uint8_t len` ( [bus_i2c.h:33](../../../../src/main/drivers/bus_i2c.h#L33) ) | documented as "the byte count, 0 on failure, 1 to 255 bytes"; the example tests `== 2` | `fix`: header comment; clamp or reject `length` above 255 |
| F12 | **To confirm on hardware:** `GPIO_15` is PB3, which `RxConfig` drives as the ESP Wi-Fi enable ( LOW = on ) at start-up, and is also `BACK`'s XRanging / X1Ranging XSHUT. If the header pad and the ESP enable are one net, starting or resetting the `BACK` laser drives the ESP enable. XRanging was flight-tested with four side lasers over Wi-Fi without a reported link loss, so the effect may be absent on the boards flown. Found in task 9. | `[GPIO_15] = { GPIOB, Pin_3 }` ( [Peripheral-GPIO.cpp:60](../../../../src/main/API-Src/Peripheral-GPIO.cpp#L60) ); `STM_PB3_ESP_IO14_Init ( )` ( [RxConfig.cpp:273-305](../../../../src/main/API-Src/RxConfig.cpp#L273), called at [main.cpp:203](../../../../src/main/main.cpp#L203) ); `BACK` XSHUT `GPIO_15` ( [XRanging.cpp:71](../../../../src/main/API-Src/XRanging.cpp#L71) ) | Peripherals page ( task 10 ) states the shared pin; `PIN_MAP.md` ( task 16 ) records it | `check`: inspect the schematic, then a bench test with a `BACK` laser and the app connected |
| F13 | `Motor_Set ( M1 )` or `Motor_Set ( M2 )` before `Motor_Init ( )` dereferences a NULL `userMotor [ ]`; `Motor_SetDir ( M1 / M2 )` before `Motor_Init ( )` writes through a NULL `motors_gpio [ ].gpio` ( added in task 17, `Motor.cpp:212-215` ). Found in task 10. | `userMotor [ 2 ]` is filled only by `Motor_Init` ( [Motor.cpp:76-127](../../../../src/main/API-Src/Motor.cpp#L76) ); `Motor_Set` writes `*( userMotor [ i ]->ccr )` with no check ( `:151-155` ) | Motor page: call `Motor_Init` first; Known limitations | `fix`: return when `userMotor [ i ]` is NULL |
| F14 | `Scheduler_Add ( t, fn, 0, ... )`: the first `Execute_Scheduled ( )` never returns, so the flight loop stops ( a crash in the air ). Found in task 14. | `( uint32_t ) ( now - last ) >= 0` is always true, and the `do { last += 0; } while ( ... )` never ends ( [Scheduler-Timer.cpp:103-106](../../../../src/main/API-Src/Scheduler-Timer.cpp#L103) ) | "not available in FW 3.15 with `interval_ms` 0" marker and Known limitations | `fix`: reject 0 in `Scheduler_Add`, or clamp to 1 |
| F15 | A task that calls `Scheduler_Cancel ( )` or `Scheduler_Clear ( )` on itself makes `Execute_Scheduled ( )` read freed memory. A `Scheduler_Add ( )` from a one-shot task at the head of the list is lost: after the call, `t->head = cur` ( the old next ) overwrites the new head and leaks the node ( added in task 17, `Scheduler-Timer.cpp:110-118` ). Found in task 14. | `cur->task_function ( )` then `cur->repeat` / `cur->next` on the same node ( `Scheduler-Timer.cpp:108-122` ) | "a task must not call `Scheduler_Cancel` / `Scheduler_Clear`" plus Known limitations | `fix`: read `cur->next` before the call and mark nodes for removal instead of freeing in place |
| F16 | `Localisation.get ( )` returns 0 on every axis in every build: the UWB reader `uwbUpdate ( )` and its call are under `#ifdef UWB`, which nothing defines ( `UWB` is only an enum value ). With `isLocalisationOn` never set, the whole API does nothing except reserve UART2 and `GPIO_11` on `init ( UWB )`. Found in task 14. | [mw.cpp:220-252, 1425-1428](../../../../src/main/mw.cpp#L220); `posX` / `posY` / `posZ` written nowhere else; [main.cpp:587-590](../../../../src/main/main.cpp#L587) | "experimental, does not work in FW 3.15" banner; `get ( )` documented as always 0 | `feature`: decide whether UWB localisation is revived or the API is removed |
| F17 | `FlightMode_Set ( ATLTITUDEHOLD )` only sets `BARO_MODE`: the entry reset ( `resetAltSetpoint ( )`, `initialThrottleHold`, `errorVelocityI` ) runs only on the app-switch path, and with the app's ALT HOLD switch off `updateAltHoldState ( )` clears `BARO_MODE` on every RC frame, while `userCode ( )` re-applies the user mode only between `plutoLoop ( )` ticks. Altitude hold flickers on with a stale target and integrator, so the drone can sink. `THROTTLE_MODE` against an app ALT HOLD on ping-pongs the same way. Found by the task 17 review. | [FC-Control-Command.cpp:108-119](../../../../src/main/API-Src/FC-Control-Command.cpp#L108); [altitudehold.cpp:636-651](../../../../src/main/flight/altitudehold.cpp#L636); [mw.cpp:1188-1212](../../../../src/main/mw.cpp#L1188) | "not available in FW 3.15 for `ATLTITUDEHOLD` and `THROTTLE_MODE`": switch ALT HOLD from the app; the `onLoopStart` example no longer sets it | `fix`: route the user mode through `updateAltHoldState ( )` ( the reset ) and stop the RC path from clearing it while Developer Mode holds a user mode |
| F18 | The WS2812 driver ignores the result of its own `dmaClaim ( )`: with `Peripheral_Init ( ADC_n )` before `RGB_Init ( )` on a shared DMA channel ( `RGB_2` / `RGB_7` with `ADC_8` / `ADC_9`, `RGB_5` with `ADC_4` ), both run on one channel, `APIAdcInit ( )` resets it, and the strip's busy flag never clears, so the strip stays dark. Found by the task 17 review. | `dmaClaim ( cfg->dma, DMA_OWNER_LED_STRIP );` unchecked ( [light_ws2811strip_stm32f30x.c:163](../../../../src/main/drivers/light_ws2811strip_stm32f30x.c#L163) ); the ADC claims at `Peripheral_Init` ( [Peripheral-ADC.cpp:82](../../../../src/main/API-Src/Peripheral-ADC.cpp#L82) ) | RGB-LED and Peripherals pages: call `RGB_Init ( )` first, or do not combine them | `fix`: skip the strip set-up when the claim fails |
| F19 | `Scheduler_Add ( )` never returns `false` for memory: `_sbrk` from `libnosys` has no limit and `_Min_Heap_Size` is 0, so `malloc` grows the heap into the stack instead of failing. Found by the task 17 review. | `malloc` at [Scheduler-Timer.cpp:49](../../../../src/main/API-Src/Scheduler-Timer.cpp#L49); `_Min_Heap_Size = 0` ( `src/main/target/stm32_flash.ld:18` ); the reviewer disassembled the linked `_sbrk` | Scheduler page: "about 24 bytes per add, never checked against the stack"; Known limitations | `fix`: an `_sbrk` that checks against the stack limit, or a static node pool |
| F8 | `setUserLoopFrequency ( )`: the header comment says Hz, the code uses the argument as the `plutoLoop` period in ms ( 100 gives 100 ms = 10 Hz; default 100000 us ) | `API/FC-Config.h:54-60` vs `API-Src/FC-Config-PID.cpp:85-93` and `mw.cpp:1192` | documented as it behaves, with the Hz equivalent spelled out | `fix`: change the header comment, or rename / rescale the argument ( an API change ) |

## Behaviour that is surprising but works as written

Documented on the page with a warning, not a finding to fix:

- `Peripheral_Write ( PWM, ... )` duty is a 0-100 map onto a fixed 20000-count period, so it is only a true
  percentage at 50 Hz ( `API-Src/Peripheral-PWM.cpp:110-117` ).
- `Servo_Write ( )` resolves to 6 steps of 200 us ( `:125-131` ), so 1500 outputs 1400 us.
- `APIAdcInit ( )` is firmware-internal and already runs after `plutoInit ( )` ( `main.cpp:583` ); user code must
  not call it.
- SPI `speed` is a kHz code from a fixed set ( 140 / 281 / 562 / 1125 / 2250 / 4500 / 9000 / 18000 ), not a
  frequency; anything else falls back to 18 MHz ( `API-Src/Serial-IO-SPI.cpp:34-71` ).
- `Interval::set ( )` latches its period on the first call until `reset ( )`.
- `Scheduler_Add ( )` calls `malloc` per node, and `onLoopStart ( )` re-runs after a Wi-Fi stall, so adding tasks
  there duplicates them.
- `Localisation`: `isLocalisationOn` is only set by the commented-out `startLocalisation ( )`, so UWB position
  control never engages. Page carries an "experimental" banner ( user's decision ). Task 14 found it goes further:
  the positions themselves are never read ( F16 ).
  The only other writer, at `mw.cpp:1561-1567`, is under `#ifdef OPTIC_FLOW`, which no target defines ( task 17 ).
- `Peripheral_Read ( ADC_n )` on a pin that was not sampled returns slot 0 of its converter, the first enabled pin of
  that converter, or 0: `_adcDmaIndex [ ]` is only filled by `APIAdcInit ( )` ( `Peripheral-ADC.cpp:97, :168` ).
- `Oled_Number ( x, y, -32768 )` prints a wrong digit ( `-number` wraps ), and `Oled_PitchLine / RollLine ( 90 )`
  draw just off the screen ( `io/oled_display.c:645-654, 846, 852` ).
- `FlightMode_Set ( )` called only from `onLoopStart ( )` leaves a gap: on a `plutoLoop ( )` tick after an RC frame the
  app's switch wins for one flight loop ( `mw.cpp:1188-1212` ); with `MAGHOLD` and the app's MAG switch off, each gap
  drops the held heading to the current nose ( `mw.cpp:735-769` ). Calling it at the start of every tick closes the gap.
- `Motor_Set ( M5..M8 )` values are cleared only by a `Motor_Set ( )` call made while the ARM switch is on
  ( `Motor.cpp:189-196` ), or by the configurator / CLI motor writes; a disarm restores them ( `mixer.cpp:830-832` ).
- The HEADFREE forward direction set by `setheadFreeModeHeading ( )` is also replaced when ALT HOLD is
  switched on in flight ( `mw.cpp:1021-1024` ).
- The onboard status pattern rewrites R / G / B every 100-120 ms while it runs, so `Set_LED ( RED, ON )` without
  `Set_LED ( STATUS, OFF )` shows for one step at most ( `config/runtime_config.cpp:98-104` ). Nothing resumes the
  pattern, and nothing returns the RGB strip to `RGB_SYSTEM`, at Developer Mode off ( task 11 ).
- `RGB_UpdateAnimation ( )` and `RGB_Show ( )` do not check `rgbUserControl`: drawn in `RGB_SYSTEM`, they fight the
  status display, which repaints within about 100 ms ( `API-Src/RGB-LED.cpp:260-264, 484-490` ).
- `RGB_SetBrightness ( )` scales only colours written after it ( `API-Src/RGB-LED.cpp:371-375` ).
- `Oled_Init ( )` works only in `plutoInit ( )`: it sets a flag that `main.cpp:578` reads once ( task 13 ).
- `Oled_Print ( )` text is overwritten every 200 ms on rows 2 to 5 while Developer Mode is off, since the status screen
  redraws those rows ( `io/oled_display.c:393-421` ).

## Reference-doc drift ( fixed by task 16 of this topic, 15 Nov 2026 )

- `PIN_MAP.md:74` omits the SPI chip-select on PB5.
- `PIN_MAP.md` has no rows for PA6 / PA7 / PB0 / PB1 ( the M1 / M2 motor and direction pins ).
- `PIN_MAP.md:38` marks PB3 free, but `API-Src/RxConfig.cpp:271-305` drives it as the ESP enable line.
- `WS2812_RGB.md` ( found in task 11 ):
  - it says `led_count` is 1-8, "Max 8 LEDs" and a 234-byte buffer, but `RGB_MAX_LEDS` and
    `WS2811_LED_STRIP_LENGTH` are 64, a 1578-byte buffer;
  - its "Idle note" says a zero status flag shows the magenta calibration blink, but `leastSignificantBit ( 0 )` returns
    0xFFFF, the `default` branch, so nothing is painted;
  - its RGB_6 row does not say that M1 uses the same TIM3 channel.
- `TIMER_MAP.md` ( found in task 16 ) lists TIM3 as "user `PWM_8`, otherwise free", and offers PB5 = TIM3_CH2 as a
  free channel, but the Motor API runs `M1` / `M2` on TIM3 CH1 / CH2. `PWM_8` and `PWM_9` were marked "free pin".

All of the above are corrected in task 16 ( [CHANGES.md](CHANGES.md) ).

## Header comments that are wrong ( not fixed here )

- `API/Peripherals.h:175,181` refer to "ADC_11".
- `API/RGB-LED.h:387` shows `RGB_Init ( 8 )`.
- `API/Serial-IO.h:207` ( the undefined `SPI_Init ( )` ) and `:222` ( "Hz" ).
- `API/FC-Config.h:54-60` ( "Hz", see F8 ).
