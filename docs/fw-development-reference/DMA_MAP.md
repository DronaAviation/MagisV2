# MagisV2 — DMA Channel Map (STM32F303xC)

Reference for which DMA channels are occupied, by whom, and what is free to use.
Target reference: **PRIMUS_X2_v1** (PRIMUSX2 / PRIMUS_V5 share the same ADC + LED
assignments). Ownership is enforced at runtime by the DMA registry
([`drivers/dma_registry.h`](../../src/main/drivers/dma_registry.h)) — call `dmaClaim()`
before configuring any channel.

The STM32F303xC has **two DMA controllers**:

| Controller | Channels |
|------------|----------|
| **DMA1**   | 7 (Channel 1–7) |
| **DMA2**   | 5 (Channel 1–5) |
| **Total**  | **12** |

---

## 1. All DMA channels at a glance

| DMA channel    | Status            | Owner / use                       | GPIO pin(s) |
|----------------|-------------------|-----------------------------------|-------------|
| DMA1_Channel1  | ADC (USER API)    | ADC1                              | PA3, PA2 |
| DMA1_Channel2  | **FREE**          | (USART3_TX DMA — code present, disabled) | — |
| DMA1_Channel3  | **FREE**          | (USART3_RX DMA — code present, disabled) | — |
| DMA1_Channel4  | Driver-claimed    | **USART1 TX** (ESP module)        | PA9 (TX) |
| DMA1_Channel5  | **FREE**          | (USART1_RX DMA — code present, disabled) | — |
| DMA1_Channel6  | **FREE**          | (USART2_RX DMA — code present, disabled) | — |
| DMA1_Channel7  | **FREE**          | (USART2_TX DMA — code present, disabled) | — |
| DMA2_Channel1  | ADC (USER API)    | ADC2                              | PB2, PA4, PA5 |
| DMA2_Channel2  | ADC (USER API)    | ADC4                              | PB15, PB14, PB12 |
| DMA2_Channel3  | Driver-claimed    | **WS2811 LED strip** (TIM8_CH1)   | PA15 |
| DMA2_Channel4  | **FREE**          | — (no claimant anywhere)          | — |
| DMA2_Channel5  | ADC (USER API)    | ADC3                              | PB13 |

Summary: **2 always-claimed by drivers**, **4 optionally claimed by the ADC API**,
**6 free** (one of which, DMA2_Channel4, has no other potential in-tree user).

---

## 2. Claimed by internal drivers (always reserved when the feature is on)

These are reserved automatically during firmware init — do **not** reuse them.

| DMA channel    | Driver            | Peripheral | GPIO pin | Registry owner       | Source |
|----------------|-------------------|------------|----------|----------------------|--------|
| DMA1_Channel4  | UART driver       | USART1 TX  | PA9      | `DMA_OWNER_SERIAL_TX`| [serial_uart_stm32f30x.c](../../src/main/drivers/serial_uart_stm32f30x.c) |
| DMA2_Channel3  | WS2811 LED strip  | TIM8_CH1   | PA15     | `DMA_OWNER_LED_STRIP`| [light_ws2811strip_stm32f30x.c](../../src/main/drivers/light_ws2811strip_stm32f30x.c) |

Notes:
- **USART1 TX DMA is unconditional** — `txDMAChannel = DMA1_Channel4` is set whenever
  USART1 is opened (the ESP receiver uses it), so treat DMA1_Channel4 as permanently taken.
- **WS2811** is reserved only when `LED_STRIP` is enabled (it is, on PRIMUS_X2_v1).
- **I2C and SPI do NOT use DMA** in this firmware — those buses are interrupt/polled,
  so they reserve no DMA channels.
- USART2/USART3 TX/RX DMA and the legacy `adc_stm32f30x.c` (which wants DMA1_Channel1)
  are **compiled-out** (`USE_USART*_*_DMA` and `USE_ADC` are disabled), so they reserve nothing.

---

## 3. Optionally configured by the USER API — the ADC

ADC DMA is **lazy**: a channel is reserved **only** when you call
`Peripheral_Init(ADC_x)` for a pin on that ADC. If you never init an ADC pin, its
DMA channel stays **free** for other use. See [Peripheral-ADC.cpp](../../src/main/API-Src/Peripheral-ADC.cpp).

### ADC pin → physical pin → DMA channel

| API pin  | MCU pin | ADC channel | Physical ADC | DMA channel    |
|----------|---------|-------------|--------------|----------------|
| `ADC_1`  | **PB2**  | ADC2_IN12  | ADC2 | DMA2_Channel1 |
| `ADC_6`  | **PA4**  | ADC2_IN1   | ADC2 | DMA2_Channel1 |
| `ADC_7`  | **PA5**  | ADC2_IN2   | ADC2 | DMA2_Channel1 |
| `ADC_2`  | **PB15** | ADC4_IN5   | ADC4 | DMA2_Channel2 |
| `ADC_3`  | **PB14** | ADC4_IN4   | ADC4 | DMA2_Channel2 |
| `ADC_5`  | **PB12** | ADC4_IN3   | ADC4 | DMA2_Channel2 |
| `ADC_4`  | **PB13** | ADC3_IN5   | ADC3 | DMA2_Channel5 |
| `ADC_8`  | **PA3**  | ADC1_IN4   | ADC1 | DMA1_Channel1 |
| `ADC_9`  | **PA2**  | ADC1_IN3   | ADC1 | DMA1_Channel1 |

### Which DMA channel each ADC group locks

| DMA channel    | Freed if you DON'T init… | Reserved if you init any of… |
|----------------|--------------------------|------------------------------|
| DMA1_Channel1  | ADC_8, ADC_9             | ADC_8 / ADC_9 |
| DMA2_Channel1  | ADC_1, ADC_6, ADC_7      | ADC_1 / ADC_6 / ADC_7 |
| DMA2_Channel2  | ADC_2, ADC_3, ADC_5      | ADC_2 / ADC_3 / ADC_5 |
| DMA2_Channel5  | ADC_4                    | ADC_4 |

- Pins **within the same row** share one ADC + DMA channel cooperatively — you can use
  them together (multi-channel scan). They are **not** mutually exclusive.
- The only conflict is **an enabled ADC pin vs. your own custom use of that same DMA
  channel** — the registry refuses the second claimant.

---

## 4. Free channels & which physical pins can attach to them

A DMA channel never attaches to a GPIO pin directly — it attaches to a **peripheral**
(SPI / USART / I2C / timer), and that peripheral's signal is routed to a pin via an
**alternate function (AF)**. So "which pin can use this channel" = "which pin carries a
peripheral signal whose DMA request lands on this channel."

Sources (authoritative): **RM0316 Rev 10, Table 76** (DMA1 requests) and **Table 78**
(DMA2 requests); **DS9118 Rev 14, Tables 14–15** (Port A/B alternate functions). Scoped
to STM32F303xC (xB/C/D/E request set, minus xD/E-only lines).

### 4.1 DMA request set per free channel (STM32F303xC)

| Free channel  | DMA-capable peripheral requests |
|---------------|---------------------------------|
| DMA1_Channel2 | SPI1_RX, USART3_TX, TIM1_CH1, TIM2_UP, TIM3_CH3 |
| DMA1_Channel3 | SPI1_TX, USART3_RX, TIM1_CH2, TIM3_CH4, TIM3_UP, TIM16_CH1/UP, TIM6_UP/DAC1_CH1 |
| DMA1_Channel5 | SPI2_TX, USART1_RX, I2C2_RX, TIM1_UP, TIM2_CH1, TIM4_CH3, TIM15_CH1/UP/TRIG/COM |
| DMA1_Channel6 | USART2_RX, I2C1_TX, TIM1_CH3, TIM3_CH1/TRIG, TIM16_CH1/UP *(remap)* |
| DMA1_Channel7 | USART2_TX, I2C1_RX, TIM2_CH2, TIM2_CH4, TIM4_UP, TIM17_CH1/UP *(remap)* |
| DMA2_Channel4 | ADC4 *(SYSCFG remap)*, TIM7_UP, DAC_CH2 — **no SPI/USART/I2C/timer-channel pin** |

### 4.2 Pin reachability (reference pin set)

Reference broken-out pin set: `PA8, PB2, PB6, PB7, PB15, PB14, PB13, PB12, PA4, PA13,
PA14, PB4, PB5, PA5, PB3, PA15, PA3, PA2`. Pins already assigned on this board:

| Status | Pins |
|--------|------|
| ADC inputs (claimed when inited) | PB2 (ADC_1), PA4 (ADC_6), PA5 (ADC_7), PA3 (ADC_8), PA2 (ADC_9), PB15 (ADC_2), PB14 (ADC_3), PB13 (ADC_4), PB12 (ADC_5) |
| WS2811 LED strip | PA15 (TIM8_CH1) |
| **SWD debug — do not repurpose** | PA13 (SWDIO), PA14 (SWCLK) |
| **Genuinely free** ✅ | **PA8, PB3, PB4, PB5, PB6, PB7** |

Which free channel each free pin can reach:

| Channel        | Reachable pin | Peripheral signal (AF) |
|----------------|---------------|------------------------|
| DMA1_Channel2  | **PB4**       | SPI1_MISO (AF5) → SPI1_RX |
|                | **PA8**       | TIM1_CH1 (AF6) |
| DMA1_Channel3  | **PB5**       | SPI1_MOSI (AF5) → SPI1_TX |
|                | **PB4**       | TIM16_CH1 (AF1) |
| DMA1_Channel5  | **PB7**       | USART1_RX (AF7) |
|                | PB15\*        | SPI2_MOSI (AF5) → SPI2_TX |
|                | PA5\* / PA15\*| TIM2_CH1 (AF1) |
| DMA1_Channel6  | **PB4**       | USART2_RX (AF7) **or** TIM3_CH1 (AF2) |
|                | **PB6 / PB7** | I2C1_SCL / I2C1_SDA (AF4) → I2C1_TX |
|                | PA3\*         | USART2_RX (AF7) |
| DMA1_Channel7  | **PB3**       | USART2_TX (AF7) **or** TIM2_CH2 (AF1) |
|                | **PB6 / PB7** | I2C1 SCL/SDA (AF4) → I2C1_RX |
|                | PA2\* / PA3\* | USART2_TX (AF7) / TIM2_CH4 (AF1) |
| DMA2_Channel4  | — none —      | no comm/timer-channel pin on this channel |

\* pin is already an ADC/LED pin — listed for completeness only.

### 4.3 Practical combos (using only the 6 free pins)

- **SPI1 on PB3 / PB4 / PB5** (SCK/MISO/MOSI, AF5) → full-duplex DMA = **Ch2 (RX) + Ch3
  (TX)**, both free. **Best choice for a new DMA peripheral.**
- **USART2 on PB3 (TX) / PB4 (RX)** (AF7) → **Ch7 (TX) + Ch6 (RX)**, both free.
- **USART1_RX on PB7** (AF7) → **Ch5**. (USART1 **TX** DMA = Ch4 is taken by the ESP, so RX-only.)
- **I2C1 on PB6 / PB7** (SCL/SDA, AF4) → TX = Ch6, RX = Ch7, both free.
- **TIM1_CH1 on PA8** (AF6) → **Ch2** — for a timer-paced DMA pattern (e.g. a second WS2811-style output).

### 4.4 Gotchas

- **SPI2 cannot do full-duplex DMA here:** SPI2_RX is on **DMA1_Channel4**, already owned
  by USART1 TX. Only SPI2_TX (Ch5) is free → TX-only DMA.
- **DMA2_Channel4 has no digital-I/O peripheral** on STM32F303xC — only ADC4 (a SYSCFG
  remap that just relocates ADC4 off Ch2), TIM7_UP (no pin), and DAC_CH2 (analog out on
  PA4/PA5). Use it for **memory-to-memory** or DAC/ADC-paced transfers, not a pin peripheral.
- **TIM16/TIM17 on Ch6/Ch7 require the SYSCFG remap bit** (RM0316 Table 76, footnote 2).
- **SPI_RX and SPI_TX are separate channels** — budget both for full-duplex.
- Remember the **DMA2 OR-ing caution** (RM0316): on a DMA2 channel only one peripheral
  request may be active at a time, since requests are logically ORed before the controller.

---

## 5. How to claim a free DMA channel from your code

```c
#include "drivers/dma_registry.h"

// Reserve a free channel before configuring it:
if ( dmaClaim ( DMA2_Channel4, DMA_OWNER_USER ) ) {
    // success — safe to DMA_Init() / DMA_Cmd() DMA2_Channel4 here
} else {
    // refused — an ADC pin / LED strip / UART already owns it. Do NOT touch it.
}

// Query without claiming:
if ( dmaIsFree ( DMA2_Channel4 ) )     { /* available */ }
dmaOwner_e owner = dmaGetOwner ( DMA2_Channel4 );

// Give it back when done:
dmaRelease ( DMA2_Channel4 );
```

**Ordering rule:** initialise all your ADC pins (`Peripheral_Init(ADC_x)`) *before*
claiming any leftover DMA channel for custom use, so the ADC reserves its channels
first. The registry is a software contract — any code that configures a DMA channel
without calling `dmaClaim()` first bypasses the protection, so always go through it.

---

_Generated for the `BugFix-June26` branch. Regenerate if DMA assignments change in
`Peripheral-ADC.cpp`, `serial_uart_stm32f30x.c`, or `light_ws2811strip_stm32f30x.c`._
