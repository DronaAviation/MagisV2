# MagisV2 — User Pin Capability Map (STM32F303xC / PRIMUS_X2_v1)

What each physically broken-out pin can do through the user-level API
([`API/Peripherals.h`](../../src/main/API/Peripherals.h) for GPIO/PWM/ADC,
[`API/Serial-IO.h`](../../src/main/API/Serial-IO.h) for UART/I2C/SPI), plus which DMA channel
and timer it touches or can borrow.

Companion docs: **[DMA_MAP.md](DMA_MAP.md)** (full DMA detail) and
**[TIMER_MAP.md](TIMER_MAP.md)** (full timer detail). Pin/AF facts are from DS9118
Rev 14; API/ownership facts are from the codebase.

### Legend
- **GPIO / ADC / PWM** columns = the API enum to pass to `Peripheral_Init(...)` for that function.
- **Serial** = fixed-function bus the pin belongs to (UART/I2C/SPI), if any.
- **DMA** = `alloc:` channel taken when that function is initialised · `free:` channel you could
  borrow for a custom DMA peripheral on this pin.
- **Timer** = `alloc:` channel taken when that PWM/feature is used · `free:` a spare channel reachable here.
- ⚠ = a sharing conflict to watch (detailed in §3).

> **One function per pin at a time.** A pin listed under GPIO *and* ADC *and* PWM *and* SPI
> can only be one of those at once — `Peripheral_Init` for one role reconfigures the pin.

---

## 1. User pins (the 18 GPIO pins + the I2C bus pins)

| Physical | GPIO | ADC | PWM | Serial (fixed) | DMA | Timer |
|----------|------|-----|-----|----------------|-----|-------|
| **PA2**  | GPIO_18 | ADC_9 (ADC1_IN3) | — | USART2_TX (AF7) | alloc: DMA1_Ch1 (ADC1, if ADC_9) | free: TIM2_CH3 |
| **PA3**  | GPIO_17 | ADC_8 (ADC1_IN4) | — | USART2_RX (AF7) | alloc: DMA1_Ch1 (ADC1, if ADC_8) | free: TIM2_CH4 |
| **PA4**  | GPIO_9  | ADC_6 (ADC2_IN1) | — | — | alloc: DMA2_Ch1 (ADC2, if ADC_6) | free: TIM3_CH2 |
| **PA5**  | GPIO_14 | ADC_7 (ADC2_IN2) | — | (SPI1_SCK / DAC_OUT2) | alloc: DMA2_Ch1 (ADC2, if ADC_7) | free: TIM2_CH1 |
| **PA8**  | GPIO_1  | — | PWM_1 (TIM1_CH1) | — | free: DMA1_Ch2 (via TIM1_CH1) | ⚠ alloc: TIM1_CH1 (5th motor out **+** PWM_1) |
| **PA13** | GPIO_10 | — | PWM_6 (TIM4_CH3) | — | — | ⚠ alloc: TIM4_CH3 — **SWDIO debug pin** |
| **PA14** | GPIO_11 | — | PWM_7 (TIM8_CH2) | — | — | ⚠ alloc: TIM8_CH2 — **SWCLK debug pin** |
| **PA15** | GPIO_16 | — | PWM_10 (TIM8_CH1) | — | ⚠ alloc: DMA2_Ch3 (WS2811) | ⚠ alloc: TIM8_CH1 (WS2811 LED **+** PWM_10) |
| **PB2**  | GPIO_2  | ADC_1 (ADC2_IN12) | — | — | alloc: DMA2_Ch1 (ADC2, if ADC_1) | — (no usable timer AF) |
| **PB3**  | GPIO_15 | — | — | (SPI1_SCK / USART2_TX, no API) | free: DMA1_Ch7 (USART2_TX) | free: TIM2_CH2 / TIM3_CH1 |
| **PB4**  | GPIO_12 | — | PWM_8 (TIM3_CH1) | (SPI1_MISO / USART2_RX, no API) | free: DMA1_Ch2 (SPI1_RX) / Ch3 (TIM16_CH1) / Ch6 (USART2_RX) | alloc: TIM3_CH1 (PWM_8) · free: TIM16_CH1 |
| **PB5**  | GPIO_13 | — | PWM_9 (TIM17_CH1) | (SPI1_MOSI, no API) | free: DMA1_Ch3 (SPI1_TX) | alloc: TIM17_CH1 (PWM_9) · free: TIM3_CH2 / TIM8_CH3 |
| **PB6**  | GPIO_3  | — | PWM_2 (TIM4_CH1) | (I2C1_SCL alt / USART1_TX, no API) | free: DMA1_Ch6 (I2C1_TX) | alloc: TIM4_CH1 (PWM_2) · free: TIM16_CH1N |
| **PB7**  | GPIO_4  | — | PWM_3 (TIM4_CH2) | (I2C1_SDA alt / USART1_RX, no API) | free: DMA1_Ch5 (USART1_RX) / Ch7 (I2C1_RX) | alloc: TIM4_CH2 (PWM_3) · free: TIM17_CH1N |
| **PB12** | GPIO_8  | ADC_5 (ADC4_IN3) | — | ⚠ SPI2_NSS / flash CS | alloc: DMA2_Ch2 (ADC4, if ADC_5) | — |
| **PB13** | GPIO_7  | ADC_4 (ADC3_IN5) | — | ⚠ SPI2_SCK | alloc: DMA2_Ch5 (ADC3, if ADC_4) | — |
| **PB14** | GPIO_6  | ADC_3 (ADC4_IN4) | PWM_5 (TIM15_CH1) | ⚠ SPI2_MISO | alloc: DMA2_Ch2 (ADC4, if ADC_3) · SPI2_RX would need DMA1_Ch4 (taken) | alloc: TIM15_CH1 (PWM_5) |
| **PB15** | GPIO_5  | ADC_2 (ADC4_IN5) | PWM_4 (TIM15_CH2) | ⚠ SPI2_MOSI | alloc: DMA2_Ch2 (ADC4, if ADC_2) · free: DMA1_Ch5 (SPI2_TX) | alloc: TIM15_CH2 (PWM_4) |
| **PB8**  | — | — | — | **I2C1_SCL** (AF4) — API I2C bus | free: DMA1_Ch6 (I2C1_TX) | (dedicated to I2C) |
| **PB9**  | — | — | — | **I2C1_SDA** (AF4) — API I2C bus | free: DMA1_Ch7 (I2C1_RX) | (dedicated to I2C) |

Notes:
- **Only the ADC API actually uses DMA today.** The UART/I2C/SPI user APIs are
  interrupt/polled (USART2 RX/TX DMA, I2C DMA and SPI DMA are all disabled), and PWM uses
  direct timer compare. The `free:` DMA entries are channels you *could* claim (via the
  [`dma_registry`](../../src/main/drivers/dma_registry.h)) if you build a DMA-driven peripheral on that pin.
- `(… no API)` in Serial = the pin's hardware alternate function exists but no user API drives it.
- The I2C API (`I2C_Read`/`I2C_Write`) targets **I2C1 on PB8/PB9** (`I2C_DEVICE = I2CDEV_1`),
  *not* PB6/PB7. PB6/PB7 are free for PWM/other use.
- The SPI API (`SPI_Init`/`SPI_Read`/`SPI_Write`) targets **SPI2 on PB12–PB15**.

---

## 2. Reserved pins (not exposed by the user API)

| Physical | Function | Timer / DMA |
|----------|----------|-------------|
| PA0  | Motor **M1** | TIM2_CH1 |
| PA1  | Motor **M2** | TIM2_CH2 |
| PB10 | Motor **M3** | TIM2_CH3 |
| PB11 | Motor **M4** | TIM2_CH4 |
| PA9  | USART1_TX (ESP receiver) | DMA1_Ch4 (TX DMA) |
| PA10 | USART1_RX (ESP receiver) | — |
| PB8 / PB9 | I2C1 (also the onboard sensor bus) | — |
| PC13 / PC14 / PC15 | Status LEDs (R/G/B) | — |
| — | System timebase | **SysTick** (millis/micros) |

**TIM2 is fully consumed by the four motors — never reuse it.**

---

## 3. Multiplexing conflicts to watch

1. **PB12–PB15 are quad-purpose: ADC ⇄ GPIO ⇄ PWM ⇄ SPI2/flash.** The onboard **M25P16
   flash** (blackbox storage) lives on SPI2 (PB12 NSS, PB13 SCK, PB14 MISO, PB15 MOSI) and
   is initialised at boot (`m25p16_init()` → [main.cpp:450](../../src/main/main.cpp#L450)). Using
   `ADC_2/3/4/5`, `GPIO_5–8`, `PWM_4/5`, or the user `SPI_Init` on these pins **conflicts
   with the flash / blackbox**. Pick one owner per pin.
2. **PA8** — `PWM_1` (TIM1_CH1) is the *same channel/pin* as the firmware's 5th PWM output
   (`timerHardware[4]`). Don't drive both.
3. **PA15** — `PWM_10` (TIM8_CH1) is the *same channel/pin* as the **WS2811 LED strip**
   (which also owns DMA2_Channel3). Don't use `PWM_10` if the LED strip is enabled.
4. **PA13 / PA14** — `PWM_6` / `PWM_7` repurpose the **SWDIO / SWCLK** debug pins; using
   them blocks SWD debugging.
5. **PA2 / PA3** — `ADC_9` / `ADC_8` share pins with **USART2 TX/RX**. If you use UART2
   (`Uart_Init(UART2, …)`), you cannot also use ADC_8/ADC_9, and vice-versa. (UART2 is used
   when `localisationType == UWB`.)
6. **Same-timer frequency coupling:** `PWM_2`+`PWM_3` (both TIM4) share one frequency;
   `PWM_4`+`PWM_5` (both TIM15) share one frequency. Last `pwmRate` per timer wins.
7. **SPI2 has no free full-duplex DMA:** SPI2_RX request is on DMA1_Channel4, already owned
   by USART1 TX — so a DMA-driven SPI2 could only do TX (DMA1_Ch5).

---

## 4. Quick "what's safest to use" summary

| Want to… | Use |
|----------|-----|
| Spare digital I/O | any GPIO_n not currently used for its other role |
| Extra ADC input | `ADC_1` (PB2), `ADC_6/7` (PA4/PA5), `ADC_8/9` (PA2/PA3) — avoid ADC_2–5 if flash/blackbox is active |
| Extra PWM output | `PWM_2/PWM_3` (PB6/PB7) or `PWM_8/PWM_9` (PB4/PB5) — the conflict-free ones |
| A DMA-driven SPI device | SPI1 on **PB3/PB4/PB5** → DMA1_Ch2 (RX) + Ch3 (TX), both free (see DMA_MAP §4) |
| A timer / DMA pacer with no pin | **TIM6 / TIM7** (see TIMER_MAP §4) |
| A free DMA channel with no contender | **DMA2_Channel4** |

---

_Generated for the `BugFix-June26` branch. Regenerate if pin maps change in
`Peripheral-GPIO.cpp`, `Peripheral-PWM.cpp`, `Peripheral-ADC.cpp`, `target.h`, or `timer.cpp`._
