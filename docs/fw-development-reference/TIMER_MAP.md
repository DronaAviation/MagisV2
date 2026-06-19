# MagisV2 — Timer Map (STM32F303xC)

Reference for which hardware timers are occupied, by whom, and what is free to use.
Target reference: **PRIMUS_X2_v1** (PRIMUSX2 / PRIMUS_V5 share the same timer set —
`USED_TIMERS` and the motor map are identical for all three; see
[`drivers/timer.cpp`](../../src/main/drivers/timer.cpp)).

Unlike DMA, there is **no runtime ownership registry** for timers — allocation is by
convention in the source. The user PWM API self-guards with an `isPwmInit[]` flag
([`Peripheral-PWM.cpp`](../../src/main/API-Src/Peripheral-PWM.cpp)), but nothing stops two
subsystems from being compiled onto the same timer/pin, so this map is the reference
for avoiding clashes.

## Hardware inventory

STM32F303xC has **10 timers + the Cortex SysTick**. All timer kernel clocks run at
**72 MHz** (APB1 bus = 36 MHz ×2, APB2 bus = 72 MHz ×1).

| Timer  | Type      | Bus  | Width  | Channels | Notes |
|--------|-----------|------|--------|----------|-------|
| TIM1   | Advanced  | APB2 | 16-bit | 4 + N/BKIN | complementary outputs, dead-time |
| TIM2   | General   | APB1 | 32-bit | 4 | only 32-bit timer |
| TIM3   | General   | APB1 | 16-bit | 4 | |
| TIM4   | General   | APB1 | 16-bit | 4 | |
| TIM6   | Basic     | APB1 | 16-bit | 0 | no I/O pins; time-base / DAC trigger |
| TIM7   | Basic     | APB1 | 16-bit | 0 | no I/O pins; time-base / DAC trigger |
| TIM8   | Advanced  | APB2 | 16-bit | 4 + N/BKIN | complementary outputs, dead-time |
| TIM15  | General   | APB2 | 16-bit | 2 | |
| TIM16  | General   | APB2 | 16-bit | 1 | |
| TIM17  | General   | APB2 | 16-bit | 1 | |
| SysTick| Core      | —    | 24-bit | — | 1 kHz system tick |

> **Key rule:** all channels of one timer share a single time-base (prescaler + period),
> i.e. one frequency. You cannot set independent PWM frequencies on two channels of the
> same timer — only independent duty cycles.

---

## 1. All timers at a glance

| Timer  | Status            | Owner / use |
|--------|-------------------|-------------|
| TIM1   | Driver + USER API | motor/PWM output CH1 (PA8); user `PWM_1` |
| TIM2   | **Driver (full)** | 4 motor PWM outputs M1–M4 |
| TIM3   | USER API (opt)    | user `PWM_8` (CH1) — otherwise free |
| TIM4   | USER API (opt)    | user `PWM_2/PWM_3/PWM_6` |
| TIM6   | **FREE**          | — (basic timer, no pins) |
| TIM7   | **FREE**          | — (basic timer, no pins) |
| TIM8   | Driver + USER API | WS2811 LED strip (CH1, PA15); user `PWM_7/PWM_10` |
| TIM15  | USER API (opt)    | user `PWM_4/PWM_5` |
| TIM16  | **FREE**          | — (not in `USED_TIMERS`) |
| TIM17  | USER API (opt)    | user `PWM_9` (CH1) |
| SysTick| **Driver**        | system timebase `millis()` / `micros()` |

`USED_TIMERS = TIM1 | TIM2 | TIM3 | TIM4 | TIM8 | TIM15 | TIM17` ([timer.cpp:152](../../src/main/drivers/timer.cpp#L152)).
TIM6, TIM7, TIM16 are outside it.

---

## 2. Claimed by internal drivers (always reserved)

These are configured during boot regardless of user code — do **not** reuse them.

### Motor outputs — **TIM2 (all 4 channels)**
Set up by `pwmInit()` for the mixer ([timer.cpp:121-150](../../src/main/drivers/timer.cpp#L121), `USABLE_TIMER_CHANNEL_COUNT = 5`):

| Motor | Timer / channel | Pin  | AF |
|-------|-----------------|------|----|
| M1    | TIM2_CH1        | PA0  | AF1 |
| M2    | TIM2_CH2        | PA1  | AF1 |
| M3    | TIM2_CH3        | PB10 | AF1 |
| M4    | TIM2_CH4        | PB11 | AF1 |

**TIM2 is fully consumed** — all four channels drive motors. Do not use TIM2 for anything else.

### 5th PWM output — **TIM1_CH1 (PA8)**
`timerHardware[4]` ([timer.cpp:150](../../src/main/drivers/timer.cpp#L150)) — configured as a PWM
output by `pwmInit()`. Reserves **TIM1 channel 1 / PA8 / AF6**. (TIM1 CH2–CH4 remain free.)

### WS2811 LED strip — **TIM8_CH1 (PA15)**
[light_ws2811strip_stm32f30x.c:40-41](../../src/main/drivers/light_ws2811strip_stm32f30x.c#L40):
`WS2811_TIMER = TIM8`, channel 1, PA15, AF2 — DMA-paced (DMA2_Channel3). Reserves
**TIM8 channel 1**. Active when `LED_STRIP` is enabled (it is on PRIMUS_X2_v1).

### System timebase — **SysTick**
[system.c:99-141](../../src/main/drivers/system.c#L99): SysTick at 1 kHz drives `millis()`, and
`micros()` interpolates from `SysTick->VAL`. **No general-purpose timer is used for the
timebase** — so TIM6/TIM7 stay free.

---

## 3. Optionally configured by the USER API — `Peripheral_Init(PWM_x)`

Like the ADC, these timer channels are only set up when you call `Peripheral_Init(PWM_n)`.
The full LUT is `pwmConfig[]` in [Peripheral-PWM.cpp:49-61](../../src/main/API-Src/Peripheral-PWM.cpp#L49):

| API pin | Timer / channel | MCU pin | AF | ⚠ Conflict / note |
|---------|-----------------|---------|----|-------------------|
| `PWM_1`  | TIM1_CH1  | PA8  | AF6  | **same as 5th motor output (TIM1_CH1/PA8)** |
| `PWM_2`  | TIM4_CH1  | PB6  | AF2  | free pin ✅ |
| `PWM_3`  | TIM4_CH2  | PB7  | AF2  | free pin ✅ |
| `PWM_4`  | TIM15_CH2 | PB15 | AF1  | **PB15 = ADC_2** (ADC4_IN5) |
| `PWM_5`  | TIM15_CH1 | PB14 | AF1  | **PB14 = ADC_3** (ADC4_IN4) |
| `PWM_6`  | TIM4_CH3  | PA13 | AF10 | **PA13 = SWDIO (debug)** |
| `PWM_7`  | TIM8_CH2  | PA14 | AF5  | **PA14 = SWCLK (debug)** |
| `PWM_8`  | TIM3_CH1  | PB4  | AF2  | free pin ✅ |
| `PWM_9`  | TIM17_CH1 | PB5  | AF10 | free pin ✅ |
| `PWM_10` | TIM8_CH1  | PA15 | AF2  | **same as WS2811 LED strip (TIM8_CH1/PA15)** |

Notes:
- The "free pin ✅" rows (`PWM_2/3/8/9` on PB6/PB7/PB4/PB5) are the safe user-PWM outputs.
- `PWM_4/PWM_5` and the ADC pins they overlap are mutually exclusive — use one or the other.
- `PWM_6/PWM_7` repurpose the SWD debug pins; using them blocks the debugger.
- `PWM_1` shares TIM1_CH1/PA8 with the firmware's 5th PWM output; `PWM_10` shares TIM8_CH1/PA15
  with the LED strip — don't init these alongside those features.
- Because channels share a time-base, `PWM_2` and `PWM_3` (both TIM4) **run at the same
  frequency**; likewise `PWM_4`+`PWM_5` (TIM15). The `pwmRate` of the last init wins per timer.

---

## 4. Free timers & channels + which pins

### Fully free timers

| Timer  | Channels / pins | Best use |
|--------|-----------------|----------|
| **TIM6** | none (no I/O) | time-base, periodic ISR, DAC/DMA pacing |
| **TIM7** | none (no I/O) | time-base, periodic ISR, DAC/DMA pacing |
| **TIM16**| 1 channel, GPIO-capable | a spare PWM / input-capture / one-shot output |

TIM6 and TIM7 are **basic timers with no output pins** — ideal for a free-running counter,
a periodic interrupt, or pacing a DMA stream (they can trigger DMA via `TIMx_UP`). TIM16 is
a real general-purpose timer with one output channel.

**TIM16 candidate pins** (DS9118 AF table), restricted to the genuinely-free board pins
(PA8, PB3, PB4, PB5, PB6, PB7):

| Signal | Pin | AF |
|--------|-----|----|
| TIM16_CH1  | **PB4** | AF1 |
| TIM16_CH1N | **PB6** | AF1 |
| TIM16_BKIN | **PB5** | AF1 |

(Also TIM16_CH1 on PA6/PB8 — not in the free set.) Note PB4 is also `PWM_8` (TIM3_CH1) and
SPI1_MISO, so pick one function per pin.

### Free channels on partially-used timers

These timers are claimed but have spare channels. Reusing a spare channel means **inheriting
that timer's frequency** (set by the motor/LED/PWM use), so it's only safe when your use can
live with that shared period.

| Timer  | Used channel(s) | Free channel(s) | Reachable free pin(s) |
|--------|-----------------|-----------------|------------------------|
| TIM1   | CH1 (PA8, motor/PWM) | CH2, CH3, CH4 | none on the free pin set (TIM1_CH2/3/4 = PA9/PA10/PA11) |
| TIM3   | CH1 (PB4, only if `PWM_8`) | CH2, CH3, CH4 | **PB5 = TIM3_CH2 (AF2)** |
| TIM4   | CH1/CH2/CH3 (PWM_2/3/6) | CH4 | none on the free set (TIM4_CH4 = PB9) |
| TIM8   | CH1 (LED), CH2 (PWM_7) | CH3, CH4 | **PB5 = TIM8_CH3 (AF10)** |
| TIM17  | CH1 (PB5, only if `PWM_9`) | — (single channel) | — |

> ⚠️ TIM1 and TIM8 are advanced timers shared with motor/LED output — sharing their
> time-base with a custom channel is risky for flight-critical timing. Prefer **TIM6/TIM7/TIM16**
> for anything new.

---

## 5. Recommendation

- Need a **periodic interrupt or a time-base / DMA pacer** → use **TIM6 or TIM7** (no pins consumed).
- Need **one extra PWM / capture pin** → use **TIM16** on **PB4** (or the safe user API outputs
  `PWM_2/PWM_3/PWM_8/PWM_9` on PB6/PB7/PB4/PB5).
- **Never touch TIM2** (all four motors) and avoid adding load to **TIM1/TIM8** time-bases.
- Remember a timer = one frequency across all its channels.

Source: code (`timer.cpp`, `Peripheral-PWM.cpp`, `light_ws2811strip_stm32f30x.c`, `system.c`,
`target.h`) is authoritative for ownership; pin/AF options are from **DS9118 Rev 14, Tables 14–15**.

---

_Generated for the `BugFix-June26` branch. Regenerate if timer assignments change in
`timer.cpp`, `Peripheral-PWM.cpp`, or the WS2811 driver._
