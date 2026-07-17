# WS2812B LED Integration — Report & Root-Cause Analysis

**Board:** PRIMUS_X2_v1 (STM32F303, brushed quad) · **Firmware:** MagisV2 · **Branch:** `ws2812`

---

## TL;DR

We added WS2812B RGB-strip control to the flight firmware. After enabling it, the drone
**flew fine in normal mode but would not climb in BARO / altitude-hold mode.** After a long
hunt, the cause turned out to be **a buffer overflow in the LED default-config code that
silently overwrote the flight controller's PID / altitude-hold gains in memory** — not the
LED hardware at all. A one-line clamp on a `memcpy` fixed it. The strip and BARO alt-hold now
work together.

---

## Update — selectable data pin + System/User control (added after this report)

Since this root-cause writeup, the WS2812B API gained two capabilities. Full reference:
**[WS2812_RGB.md](WS2812_RGB.md)**.

- **Selectable data pin (`RGB_1 .. RGB_8`).** The strip is no longer hard-wired to PA15 —
  `RGB_Init(pin, led_count)` picks one of 8 vetted `{ pin, timer channel, DMA channel }`
  slots. The driver's fixed `WS2811_*` macros were replaced by a `ws2811HwTable[]` LUT in
  [`light_ws2811strip_stm32f30x.c`](../src/main/drivers/light_ws2811strip_stm32f30x.c);
  PA15/TIM8/DMA2_Ch3 is now just the default (`RGB_8`). Each slot's PWM/ADC conflicts are
  tabulated in [WS2812_RGB.md §3](WS2812_RGB.md).
- **`RGB_Control(RGB_USER | RGB_SYSTEM)`.** Toggles who owns the strip. `RGB_SYSTEM` (the
  default after `RGB_Init`) runs a self-contained flight-status indicator — a faithful
  port of `ledStripFlightStatus()` into the RGB layer (`rgbSystemTick()`), driven from
  `mw.cpp` each loop **without** the `LED_STRIP` feature (so it can't re-trigger the BARO
  overflow below). `RGB_USER` hands the strip to your `RGB_*` calls.

> **Signature change:** `RGB_Init(uint8_t led_count)` → `RGB_Init(peripheral_rgb_pin_e pin,
> uint8_t led_count)`. The examples in Part 6 below use the current signature.

---

## Part 1 — The actual bug (a classic buffer overflow)

### The offending line
In `src/main/io/ledstrip.c`, `applyDefaultLedStripConfig()` did:

```c
memcpy ( ledConfigs, &defaultLedStripConfig, sizeof ( defaultLedStripConfig ) );
```

### The mismatch
| Thing | Size |
|---|---|
| `defaultLedStripConfig` (built-in default LED layout) | **28 entries × 4 bytes = 112 bytes** |
| `ledConfigs[MAX_LED_STRIP_LENGTH]` (destination array) | **5 entries × 4 bytes = 20 bytes** |

`MAX_LED_STRIP_LENGTH` had been shrunk to **5** (Cleanflight's default is 32), but the default
layout array still had **28** entries — and `memcpy` copies whatever `sizeof()` says, ignoring
the destination's capacity. So it wrote **112 bytes into a 20-byte box → 92 bytes spilled past
the end.**

### Where the 92 bytes landed
The config lives in one big struct (`master_t`). The fields are laid out back-to-back in memory:

```
            ledConfigs[5]      colors[16]              profile[]  ← PID / ALT-HOLD GAINS
            (20 bytes)         (64 bytes)              (...)
memory:  | ---------------- | ---------------------- | ----------------- ...
             ^                                          ^
             memcpy starts here                         overflow reaches 8 bytes into here
             writes 112 bytes ───────────────────────────►  (corrupts pidProfile)
```

- 20 bytes filled `ledConfigs` (OK)
- next 64 bytes ran into `colors[16]`
- the final **8 bytes ran into `profile[]` → `pidProfile`**, the PID and altitude-hold tuning.

So the LED default config **scribbled garbage onto the flight controller's PID gains.**

### Why the symptoms looked like they did
- **Only with `LED_STRIP` defined.** That `#define` is what makes the LED config arrays exist
  and the default-fill code run (during config reset). `main` doesn't define it for this target,
  so `main` never hit the bug → *"main works perfectly."*
- **Zero LED code needed to run.** The corruption happened in **config initialization**, not in
  any LED animation or hardware path. That's why disabling the strip, the flight-status, the boot
  init, the loop update — none of it helped.
- **Attitude flew, but BARO couldn't climb.** The corrupted bytes hit the altitude-hold gains
  specifically; roll/pitch/yaw survived, so the drone flew level but the altitude controller was
  fed garbage and couldn't hold/climb.

---

## Part 2 — Why it took so long (the red herrings)

Every symptom *screamed* "the LED is interfering with the flight controller," so we chased
hardware/timing theories first:

| Theory we chased | Why we ruled it out |
|---|---|
| LED DMA blocking the main loop | Made the LED push non-blocking; didn't fix it |
| Boot-time `ws2811LedStripInit()` (TIM8/DMA2/PA15) | Skipped it entirely; didn't fix it |
| Timer / DMA / pin conflict with motors or gyro | Motors are TIM2/TIM1, gyro is SPI2 — no overlap |
| RAM / stack overflow | Measured: only 31% RAM used (12.6 KB / 40 KB) |
| Config struct layout corruption on flash | Config version/size check resets cleanly |

The breakthrough was a **clean A/B test**: with the *exact same source*, toggling only
`#define LED_STRIP` flipped BARO from working to broken. That proved it was a **compile-time**
effect with **no LED code executing** — which pointed straight at code that runs at config-init
*because* `LED_STRIP` is defined. That's `applyDefaultLedStripConfig()`.

**Lesson:** when the evidence says a thing is "impossible" (no LED code runs, yet it breaks),
question the premise and isolate with a controlled A/B. Then *measure*, don't guess.

---

## Part 3 — The fix

Clamp the copy so it can never exceed the destination array:

```c
void applyDefaultLedStripConfig ( ledConfig_t *ledConfigs ) {
  memset ( ledConfigs, 0, MAX_LED_STRIP_LENGTH * sizeof ( ledConfig_t ) );

  // Clamp the copy to the destination array size — defaultLedStripConfig has
  // more entries than MAX_LED_STRIP_LENGTH, so copying all of it overflowed
  // into the adjacent PID/alt-hold config and broke BARO mode.
  uint16_t copyBytes = sizeof ( defaultLedStripConfig );
  uint16_t maxBytes  = MAX_LED_STRIP_LENGTH * sizeof ( ledConfig_t );
  if ( copyBytes > maxBytes )
    copyBytes = maxBytes;
  memcpy ( ledConfigs, &defaultLedStripConfig, copyBytes );

  reevalulateLedConfig ( );
}
```

> **Latent issue still there:** `MAX_LED_STRIP_LENGTH = 5` vs a 28-entry default array is a
> mismatch. The clamp makes it *safe* (no overflow). If you ever want the full default layout
> usable, raise `MAX_LED_STRIP_LENGTH` to ≥ 28 (costs ~92 more bytes of config). For the 8-LED
> API use here, it doesn't matter.

---

## Part 4 — Everything we changed

| File | Change | Why |
|---|---|---|
| `src/main/io/ledstrip.c` | **Clamp the `memcpy`** in `applyDefaultLedStripConfig()` | **The real fix** — stops the overflow corrupting PID/alt-hold gains |
| `src/main/io/ledstrip.c` | Moved `rgbUserControl` definition *outside* `#ifdef LED_STRIP` | So the always-compiled RGB API links even when `LED_STRIP` is off |
| `src/main/io/ledstrip.c` | `updateLedStrip()` calls `rgbReleaseFlushTick()` | Cleanly blanks the strip when the user API releases control |
| `src/main/API-Src/RGB-LED.cpp` | `rgbWriteDma()`: `while(busy){}` → `if(busy) return;` | **Non-blocking** — the user API can never stall the flight loop |
| `src/main/API-Src/RGB-LED.cpp` | Added `rgbReleaseFlushTick()` + pending-blank in `RGB_Release()` | Non-blocking, reliable "all-off" on Developer-Mode exit |
| `src/main/io/ledstrip.h` | Declared `rgbReleaseFlushTick()` | Shared between the API and the system LED loop |
| `src/main/target/PRIMUS_X2_v1/target.h` | Disabled `MAG_ENFORCE` | Don't block arming on mag calibration (user choice) |
| `PlutoPilot.cpp` | Steady-glow demo using the `RGB_*` API | Example user program |

**Already on the branch before this work** (the WS2812B integration itself): the public
`RGB-LED` API + 21 animations, the `light_ws2811strip` driver wired to **PA15 / TIM8 / DMA2_Ch3**,
the `ledstrip.c` system integration, and `#define LED_STRIP` in the target.

---

## Part 5 — Concepts a newbie should understand

These are the ideas that made this bug possible (and made it hard to find). Worth knowing for
any embedded / C work.

### 1. Arrays in C have **no bounds checking**
`memcpy`, array indexing, etc. will happily read/write past the end of an array. There's no
safety net — you get **undefined behavior** and silent memory corruption. *Always* make sure a
copy fits the destination (`sizeof(dest)`, not `sizeof(src)`).

### 2. `sizeof` is about the **type/object**, not your intent
`sizeof(defaultLedStripConfig)` = size of the *source array* (112 B). The bug copied the source's
size into a smaller destination. The safe pattern is `min(sizeof(src), sizeof(dest))`.

### 3. Struct fields are laid out **contiguously in memory**
A `struct` packs its members one after another. Overflowing one field writes into the **next
field**. Here, overflowing `ledConfigs` corrupted `colors`, then `profile` (the PID gains). Two
unrelated features sharing one config struct meant an LED bug could break flight control.

### 4. `#define` changes what gets **compiled**, not just runtime behavior
`#define LED_STRIP` pulled extra fields into the config struct and switched on the default-fill
code. So a *compile-time* flag changed *runtime memory behavior* — even though no LED *animation*
ran. When a `#define` flips behavior, suspect everything it gates, including struct layout.

### 5. Blocking vs non-blocking code (critical on a flight controller)
A flight controller runs a tight loop (read sensors → PID → drive motors), thousands of times a
second. **Anything that blocks** (e.g. `while (transferInProgress) {}`) stalls that loop. We made
the LED DMA push *non-blocking* (skip if busy) so LED work can never freeze flight control.

### 6. DMA = hardware moving data without the CPU
WS2812B LEDs need precise 800 kHz bit timing. The MCU sets up a **DMA** transfer (DMA2 channel 3 →
timer TIM8 → pin PA15) and the hardware streams the bits while the CPU does other things. You must
not overwrite the DMA buffer mid-transfer — hence the "is a transfer in progress?" check.

### 7. Config / EEPROM versioning
Saved settings live in flash with a `version` + `size` + checksum. If the firmware's struct
changes size, the check fails and config **resets to defaults**. That's why the bug always
re-appeared on a fresh flash — the reset path re-ran the buggy default-fill.

### 8. Altitude hold is **timing- and tuning-sensitive**
Alt-hold integrates the accelerometer over time and uses PID gains. Corrupt those gains (or jitter
the loop) and it can't hold/climb — while plain attitude flight still works. That asymmetry
("flies but won't climb") is a useful diagnostic clue.

### 9. Debugging method: **isolate, then measure**
- **Isolate** with a clean A/B (change exactly one thing — here, the `#define`).
- **Measure** instead of guessing (we read RAM usage from the build, printed RC values live, etc.).
- Re-question "impossible" results — they mean an assumption is wrong.

---

## Part 6 — Using the WS2812B from `PlutoPilot.cpp`

The strip is controlled with the `RGB_*` API (header: `src/main/API/RGB-LED.h`). It's
**non-blocking** and safe to use in `plutoLoop`. Full reference incl. the pin-selection map
and per-pin conflicts: **[WS2812_RGB.md](WS2812_RGB.md)**.

### Steady glow (current demo)
```c
void onLoopStart ( void ) {
  RGB_Init ( RGB_1, 8 );            // 8-LED strip on PA8 (RGB_1..RGB_8 select the pin)
  RGB_Control ( RGB_USER );         // take over from the flight-status indicator
  RGB_SetBrightness ( 80 );         // 80% brightness
  RGB_SetColorAll ( 0, 180, 255 );  // color (R,G,B) — soft cyan
  RGB_Show ( );                     // push to the strip
}
void plutoLoop   ( void ) { }       // steady — nothing to update
void onLoopFinish( void ) { RGB_Control ( RGB_SYSTEM ); }   // flight status resumes
```

### Breathing glow (pulse)
```c
void onLoopStart ( void ) {
  RGB_Init ( RGB_1, 8 );
  RGB_Control ( RGB_USER );
  RGB_SetBrightness ( 80 );
  RGB_StartAnimation ( RGB_ANIM_BREATHE, 30, RGB_DIR_FORWARD, 0, 180, 255 );
}
void plutoLoop   ( void ) { RGB_UpdateAnimation ( ); }   // advance each loop
void onLoopFinish( void ) { RGB_Control ( RGB_SYSTEM ); }
```

### Handy calls
| Call | Does |
|---|---|
| `RGB_Init(pin, n)` | Init an `n`-LED strip on `pin` (`RGB_1..RGB_8`); starts in `RGB_SYSTEM` |
| `RGB_Control(RGB_USER \| RGB_SYSTEM)` | Choose who drives the strip — you or the flight-status indicator |
| `RGB_SetColorAll(r,g,b)` / `RGB_SetColor(i,r,g,b)` | Set whole strip / one LED (then `RGB_Show()`) |
| `RGB_SetBrightness(percent)` | Global brightness 0–100 |
| `RGB_Show()` | Push the buffer to the strip |
| `RGB_StartAnimation(anim, speed_ms, dir, r,g,b)` | Start one of 21 built-in effects |
| `RGB_UpdateAnimation()` | Advance the running animation (call every loop) |
| `RGB_Release()` | Blank the strip and give it back to the system |

Colors: Red `255,0,0` · Green `0,255,0` · Blue `0,0,255` · White `255,255,255` ·
Yellow `255,255,0` · Cyan `0,255,255` · Purple `128,0,128`.

---

## Part 7 — Final behavior

- **Normal / armed flight (incl. BARO alt-hold):** climbs correctly; strip shows system
  flight-status colors.
- **Developer Mode:** your `plutoLoop` (`RGB_Init` → animations) takes over the strip; on exit
  the system resumes.
- **The LED and BARO alt-hold work together** — the conflict was never real hardware contention,
  just a buffer overflow corrupting flight config.
