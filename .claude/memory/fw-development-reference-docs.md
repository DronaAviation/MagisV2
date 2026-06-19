---
name: fw-development-reference-docs
description: Location of the firmware hardware-resource reference docs (DMA/timer/pin maps + datasheets) and the rule to keep them in sync with the code
metadata: 
  node_type: memory
  type: project
  originSessionId: 2b4b36b2-6e36-4426-9063-4fa2343da3e5
---

Hardware-resource reference for extending MagisV2 firmware lives in
`docs/fw-development-reference/` (target reference: PRIMUS_X2_v1):

- `DMA_MAP.md` — STM32F303xC DMA1/DMA2 channel ownership (driver-claimed vs ADC USER-API vs free) + which pins/peripherals can attach to free channels.
- `TIMER_MAP.md` — timer inventory: motors=TIM2 (full), TIM1_CH1, WS2811=TIM8_CH1, SysTick timebase; free = TIM6/TIM7/TIM16; user `PWM_1..10` map.
- `PIN_MAP.md` — master per-physical-pin table tying GPIO/ADC/PWM/Serial + DMA + timer together, with conflicts (PB12–15 = ADC vs SPI2/M25P16 flash, PA8 PWM_1 vs motor, PA15 PWM_10 vs LED, PA13/PA14 = SWD).
- `datasheets/` — `rm0316-stm32f303xbcde.pdf` (RM0316 RM, DMA Tables 76/78) and `stm32f303vc.pdf` (DS9118 datasheet, AF Tables 14/15). Extract text with `pdftotext -layout`.
- `fw-architecture-pipeline/` — firmware architecture + subsystem pipeline docs.

**Why:** the user wants these as the standing reference for further firmware development and asked that they be kept current.

**How to apply:** consult these before reasoning about DMA/timer/pin allocation; and when assignments change in `Peripheral-ADC.cpp`, `Peripheral-PWM.cpp`, `Peripheral-GPIO.cpp`, `serial_uart_stm32f30x.c`, `light_ws2811strip_stm32f30x.c`, `timer.cpp`, or `target/<TARGET>/target.h`, update the affected map(s). DMA ownership is enforced at runtime by the new `drivers/dma_registry.{h,c}` (`dmaClaim`/`dmaRelease`/`dmaIsFree`/`dmaGetOwner`); ADC DMA is now lazy (claimed only when a `Peripheral_Init(ADC_x)` pin is used). Doc links to source use `../../src/main/...`. Related: [[windows-pluto-toolchain-path]].
