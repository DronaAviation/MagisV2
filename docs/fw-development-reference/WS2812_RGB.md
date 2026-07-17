# WS2812 / WS2812B RGB LED API (PRIMUS_X2_v1)

User-facing API for driving a WS2812B addressable LED strip from `PlutoPilot.cpp`.
The strip can be driven from **any one of 8 selectable data pins**, and its ownership
can be handed between the **firmware flight-status indicator** and **your user code**.

- Public header: [`API/RGB-LED.h`](../src/main/API/RGB-LED.h)
- Implementation: [`API-Src/RGB-LED.cpp`](../src/main/API-Src/RGB-LED.cpp)
- Low-level driver: [`drivers/light_ws2811strip_stm32f30x.c`](../src/main/drivers/light_ws2811strip_stm32f30x.c)
- Hardware maps: [`PIN_MAP.md`](fw-development-reference/PIN_MAP.md) · [`DMA_MAP.md`](fw-development-reference/DMA_MAP.md) · [`TIMER_MAP.md`](fw-development-reference/TIMER_MAP.md)

> **One strip at a time.** The data pin is *selectable*, not simultaneous — one WS2812
> output is active per power cycle, chosen on the first `RGB_Init()`.

---

## 1. Quick start

```c
void plutoInit ( void ) {
  RGB_Init ( RGB_1, 3 );        // 3-LED strip on PA8, starts in SYSTEM mode
}

void onLoopStart ( void ) {
  RGB_Control ( RGB_USER );     // take over the strip in Developer Mode
  RGB_SetBrightness ( 80 );
}

void plutoLoop ( void ) {
  RGB_SetColorAll ( 0, 0, 255 );  // all blue
  RGB_Show ();
}

void onLoopFinish ( void ) {
  RGB_Control ( RGB_SYSTEM );   // hand it back — flight status resumes
}
```

`RGB_Init(pin, led_count)` — `pin` is one of `RGB_1..RGB_8` (see §2), `led_count` is
1–8. The strip **defaults to `RGB_SYSTEM`** mode (firmware paints flight status). Call
`RGB_Control(RGB_USER)` to drive it yourself.

---

## 2. Data-pin selection (`RGB_1 .. RGB_8`)

The enum is indexed like `GPIO_n` / `PWM_n`, in physical pin-map order. Each slot is a
vetted `{ pin, timer channel, DMA channel }` combination that can generate the 800 kHz
WS2812 waveform (timer capture/compare DMA request → `CCRx`).

| Enum | Pin | Timer / Ch | AF | DMA channel | Note |
|------|-----|-----------|----|-------------|------|
| `RGB_1` | PA8  | TIM1_CH1  | AF6  | DMA1_Ch2 | shares pin with PPM RC-IN / 5th motor |
| `RGB_2` | PB6  | TIM4_CH1  | AF2  | DMA1_Ch1 | shares DMA with ADC1 |
| `RGB_3` | PB14 | TIM15_CH1 | AF1  | DMA1_Ch5 | ⚠ also SPI2 MISO — disables flash/blackbox |
| `RGB_4` | PA13 | TIM4_CH3  | AF10 | DMA1_Ch5 | ⚠ SWDIO — disables SWD debugging |
| `RGB_5` | PA14 | TIM8_CH2  | AF5  | DMA2_Ch5 | ⚠ SWCLK — disables SWD debugging |
| `RGB_6` | PB4  | TIM3_CH1  | AF2  | DMA1_Ch6 | free pin — recommended alternate ✅ |
| `RGB_7` | PB5  | TIM17_CH1 | AF10 | DMA1_Ch1 | free pin; shares DMA with ADC1 |
| `RGB_8` | PA15 | TIM8_CH1  | AF2  | DMA2_Ch3 | **default** — system flight-status pin |

Source of truth: `ws2811HwTable[]` in
[`light_ws2811strip_stm32f30x.c`](../src/main/drivers/light_ws2811strip_stm32f30x.c).

---

## 3. Conflict matrix — what each slot costs you

Selecting a WS2812 pin consumes **three** resources, so three classes of function
become unavailable:

1. **The pin** — its own GPIO / ADC / PWM / serial role is gone.
2. **The whole timer** — every channel of that timer is frequency-locked to 800 kHz, so
   any *other* `PWM_n` on the same timer is useless as a normal PWM output.
3. **One DMA channel** — any ADC that streams on that channel is blocked (`dmaClaim`
   refuses the second owner).

| Slot | Pin | Timer / DMA | PWM you lose | ADC you lose | Other |
|------|-----|-------------|--------------|--------------|-------|
| **RGB_1** | PA8  | TIM1 / DMA1_Ch2 | **PWM_1** | *(none)* | PPM RC-IN, 5th-motor out |
| **RGB_2** | PB6  | TIM4 / DMA1_Ch1 | **PWM_2**, PWM_3, PWM_6 | **ADC_8, ADC_9** | — |
| **RGB_3** | PB14 | TIM15 / DMA1_Ch5 | **PWM_5**, PWM_4 | **ADC_3** | ⚠ SPI flash/blackbox (MISO) |
| **RGB_4** | PA13 | TIM4 / DMA1_Ch5 | **PWM_6**, PWM_2, PWM_3 | *(none)* | ⚠ SWDIO debug |
| **RGB_5** | PA14 | TIM8 / DMA2_Ch5 | **PWM_7**, PWM_10 | **ADC_4** | ⚠ SWCLK debug |
| **RGB_6** | PB4  | TIM3 / DMA1_Ch6 | **PWM_8** | *(none)* | brushed Motor M1/M2 (TIM3) |
| **RGB_7** | PB5  | TIM17 / DMA1_Ch1 | **PWM_9** | **ADC_8, ADC_9** | — |
| **RGB_8** | PA15 | TIM8 / DMA2_Ch3 | **PWM_10**, PWM_7 | *(none)* | default LED pin |

**Bold PWM** = the selected pin itself; the others are same-timer channels dragged to
800 kHz. Only **RGB_3's ADC_3** is a *pin*-level ADC loss (PB14 *is* ADC_3); every other
ADC loss is a *DMA*-level clash for the whole ADC group.

### Per-slot, in words

- **RGB_1 (PA8):** lose only **PWM_1**. No ADC lost, no other PWM lost — but it steals the
  **PPM receiver pin** and 5th-motor output. Fine on ESP/CRSF, not with a PPM receiver.
- **RGB_2 (PB6):** lose **PWM_2 + PWM_3 + PWM_6** (all TIM4) and **ADC_8 + ADC_9** (DMA1_Ch1).
- **RGB_3 (PB14):** lose **PWM_5 + PWM_4** (TIM15) and **ADC_3** — plus it kills the onboard flash/blackbox.
- **RGB_4 (PA13):** lose **PWM_6 + PWM_2 + PWM_3** (all TIM4), no ADC — costs SWD debug.
- **RGB_5 (PA14):** lose **PWM_7 + PWM_10** (TIM8) and **ADC_4** — costs SWD debug.
- **RGB_6 (PB4):** lose only **PWM_8**. No ADC, no other user PWM — only clashes with the
  brushed-motor API (`Motor_Init(M1/M2)`, TIM3).
- **RGB_7 (PB5):** lose only **PWM_9**, plus **ADC_8 + ADC_9** (DMA1_Ch1).
- **RGB_8 (PA15):** lose **PWM_10 + PWM_7** (TIM8), no ADC. The default.

### Cleanest choices (lose the least)

| If you want to keep… | Best slot |
|---|---|
| **All ADCs + almost all PWM**, no brushed motors | **RGB_6 (PB4)** — costs only PWM_8 |
| **All ADCs**, on ESP/CRSF (no PPM) | **RGB_1 (PA8)** — costs only PWM_1 |
| **All ADCs**, don't need PWM_7 / PWM_10 | **RGB_8 (PA15)** — the default |

> **These bite only if you actually use the other function.** The ADC/DMA clash is
> arbitrated at runtime by `dmaClaim` (first caller wins, loser silently no-ops); the
> same-timer PWM clash means whichever you init *second* drags the first to 800 kHz.

#### Reference: which timer / DMA each `PWM_n` and `ADC_n` uses

| Timer | PWM pins | | DMA channel | ADC pins |
|-------|----------|--|-------------|----------|
| TIM1  | PWM_1 (PA8) | | DMA1_Ch1 | ADC_8 (PA3), ADC_9 (PA2) |
| TIM3  | PWM_8 (PB4) | | DMA2_Ch1 | ADC_1 (PB2), ADC_6 (PA4), ADC_7 (PA5) |
| TIM4  | PWM_2 (PB6), PWM_3 (PB7), PWM_6 (PA13) | | DMA2_Ch2 | ADC_2 (PB15), ADC_3 (PB14), ADC_5 (PB12) |
| TIM8  | PWM_7 (PA14), PWM_10 (PA15) | | DMA2_Ch5 | ADC_4 (PB13) |
| TIM15 | PWM_4 (PB15), PWM_5 (PB14) | | | |
| TIM17 | PWM_9 (PB5) | | | |

---

## 4. Ownership: `RGB_SYSTEM` vs `RGB_USER`

`RGB_Control(mode)` toggles who drives the strip:

| Mode | Behaviour |
|------|-----------|
| `RGB_SYSTEM` | Firmware paints the flight-status indicator automatically every control loop. **Default after `RGB_Init()`.** |
| `RGB_USER`   | Your `RGB_*` calls drive the strip; the status indicator is suppressed. |

This works **without the Cleanflight `LED_STRIP` feature** (which is off on this board and
regresses BARO). The status renderer (`rgbSystemTick()`) is self-contained in the RGB
layer, reads the live `flightIndicatorFlag` / `rc_connected` the firmware already maintains,
and paints on **whatever pin you selected** — called from
[`mw.cpp`](../src/main/mw.cpp) each loop, no-op until `RGB_Init()` runs and only while in
`RGB_SYSTEM` mode.

### Flight-status color map (System mode)

| Status | Colour |
|--------|--------|
| Mag calibration | yellow blink |
| Accel/Gyro calibration | magenta blink |
| OK to arm (RC connected) | solid green |
| OK / Not-ok to arm (no RC) | green → cyan → white → off cycle |
| Not OK to arm (RC connected) | red blink |
| Armed (RC connected) | solid blue |
| Low battery (in flight) | red blink |
| Low battery | solid red |
| Signal loss | blue blink |
| Crash | green / red alternate |

> **Idle note:** when no status flag is set (`flightIndicatorFlag == 0`), the indicator
> shows the accel/gyro-calibration colour (magenta blink) — the firmware sets a real
> status within the first arming checks. This is inherited verbatim from the original
> `ledStripFlightStatus()`.

---

## 5. API reference

| Function | Purpose |
|----------|---------|
| `RGB_Init(pin, led_count)` | Init strip on `pin` (RGB_1..RGB_8), `led_count` LEDs. Starts in `RGB_SYSTEM`. |
| `RGB_Control(mode)` | `RGB_USER` (you draw) or `RGB_SYSTEM` (firmware draws status). |
| `RGB_Release()` | Stop animations, blank the strip, hand back to the system. |
| `RGB_SetColor(index, r, g, b)` | Set one LED. |
| `RGB_SetColorAll(r, g, b)` | Set all LEDs. |
| `RGB_SetColorHSV(index, h, s, v)` | Set one LED from HSV. |
| `RGB_FillColor(start, end, r, g, b)` | Set a range. |
| `RGB_SetBrightness(percent)` | Global 0–100 % scale. |
| `RGB_Clear()` | Buffer to black (call `RGB_Show()` to push). |
| `RGB_Show()` | Push the buffer to the strip (non-blocking). |
| `RGB_StartAnimation(anim, speed_ms, dir, r, g, b)` | Start a built-in effect. |
| `RGB_UpdateAnimation()` | Advance the current animation — call in `plutoLoop()`. |
| `RGB_StopAnimation()` / `RGB_IsAnimating()` | Stop / query animation. |
| `RGB_GetLedCount()` | Configured LED count. |

See [`API/RGB-LED.h`](../src/main/API/RGB-LED.h) for the full animation list and signatures.

---

## 6. Notes & caveats

- **Max 8 LEDs** (`RGB_MAX_LEDS`). Larger counts are clamped.
- **Pin is locked on first `RGB_Init()`** of a power cycle; later calls keep the same pin.
- **`RGB_SetColorAll` respects brightness**; `RGB_SetColorHSV` applies brightness via `v`.
- **Non-blocking:** `RGB_Show()` drops a frame if a DMA transfer is still in flight — the
  flight loop never stalls on the strip.
- **Buffer cost:** 234 bytes per strip (`8×24 + 42`).

---

_Applies to `PRIMUS_X2_v1` (and `PRIMUSX2` / `PRIMUS_V5`, identical LED/timer/DMA set).
Regenerate if the slot table changes in `light_ws2811strip_stm32f30x.c` or the pin maps._
