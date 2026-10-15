# Hardware resource reference

Detail behind the hardware-reference pointer in the root [`CLAUDE.md`](../../../CLAUDE.md).
`docs/fw-development-reference/` is the standing reference for extending the firmware (target
reference: `PRIMUS_X2_v1`). Consult it before reasoning about DMA/timer/pin allocation, and keep it in
sync with the code.

## Maps and sources

- [`DMA_MAP.md`](../DMA_MAP.md) — STM32F303xC DMA1/DMA2 channel ownership (driver-claimed vs ADC
  USER-API vs free) and which pins/peripherals can attach to free channels.
- [`TIMER_MAP.md`](../TIMER_MAP.md) — timer inventory: motors = TIM2 (all 4 channels), `TIM1_CH1`,
  WS2811 = `TIM8_CH1`, SysTick timebase; free = TIM6/TIM7/TIM16; the user `PWM_1..10` map.
- [`PIN_MAP.md`](../PIN_MAP.md) — master per-physical-pin table tying GPIO/ADC/PWM/Serial + DMA + timer
  together, with the multiplexing conflicts (e.g. PB12–15 = ADC vs SPI2/M25P16 flash, PA8 `PWM_1` vs
  5th motor output, PA15 `PWM_10` vs LED strip, PA13/PA14 = SWD debug).
- [`WS2812_RGB.md`](../WS2812_RGB.md) — RGB LED data-pin slots and per-pin conflict matrix.
- `datasheets/` — `rm0316-stm32f303xbcde.pdf` (RM0316: DMA request Tables 76/78) and
  `stm32f303vc.pdf` (DS9118: alternate-function Tables 14/15). Extract text with `pdftotext -layout`.

## I2C1 and the lasers

I2C1 ( PB8 SCL / PB9 SDA, `I2C_DEVICE = I2CDEV_1` in `target.h` ) is the onboard sensor bus and the user
I2C API bus. It also carries the lasers.

- **VL53L0X: a down-laser plus up to five side lasers, all behind `XRanging`.** One owner,
  `drivers/tof_manager`, holds a table entry per laser, each with its own device, status and results.
  - **Start-up:** one sequence, `xRangingInit ( )`, after `plutoInit ( )`. Every chip starts at 0x29. Every enabled
    laser is held in reset; then the down-laser is moved to **0x30** and the side lasers to **0x2A + n**, one at a time. Each
    release first checks that 0x29 is silent, so a fitted but disabled laser gives `BUS_BUSY` instead of two
    chips mixed up.
  - **Polling:** from `UPDATE_LASER_TOF_TASK` in `mw.cpp`. The down-laser is read every 33 ms; one side laser per
    10 ms, only when it has a sample ready. `XRanging` getters and object avoidance read the cache.
  - **Errors:** one bus error takes that laser out until power-off, and the others carry on.
- **VL53L1X down-laser:** `LASER_TOF_L1x`, `drivers/ranging_vl53l1x.cpp`, a build switch at 0x29. It is the
  only down-laser in that build ( `initAlt` is refused ).

No DMA is involved. Whenever XRanging is used, SPI2 ( PB12-15 = `GPIO_8`-`5`, the side lasers' XSHUT / LED
pins ) is switched off at start-up and the M25P16 flash reports size 0 ( `PIN_MAP.md` §3 ).

Bus cost of the VL53L1X: a data-ready poll every 10 ms, and per sample ( every 51-53 ms ) one 17-byte
result read plus a 1-byte interrupt-clear write, about 0.8 ms of blocking I2C at 400 kHz. The full ST
API path it replaced took 5.4 ms and stretched a 3.5 ms loop. Budget any new I2C1 traffic against this.
Details: `fw-architecture-pipeline/subsystems/Altitude_Hold_Estimator.md` ( Laser fusion, VL53L1X driver ).

## Pipeline and in-progress docs

- `fw-architecture-pipeline/` — firmware architecture and per-subsystem pipeline docs. **Describes
  committed firmware only**: do not edit it for work in progress.
- `active-development/` — one folder per in-progress topic (`README`, `INVESTIGATION`, `CHANGES`,
  `TESTING`, and a staged `PIPELINE_UPDATE`). Record work there while it is live; apply
  `PIPELINE_UPDATE.md` to the pipeline docs and mark the topic Closed only when the change is confirmed
  and being committed. Rules: [`../active-development/README.md`](../active-development/README.md).

## DMA registry and map sync

DMA ownership is enforced at runtime by `drivers/dma_registry.{h,c}`
(`dmaClaim`/`dmaRelease`/`dmaIsFree`/`dmaGetOwner`). ADC DMA is lazy: a channel is claimed only when a
`Peripheral_Init(ADC_x)` pin on that ADC is used.

When DMA/timer/pin assignments change in `Peripheral-ADC.cpp`, `Peripheral-PWM.cpp`,
`Peripheral-GPIO.cpp`, `serial_uart_stm32f30x.c`, `light_ws2811strip_stm32f30x.c`, `timer.cpp`, or
`target/<TARGET>/target.h`, update the affected map(s). Doc-to-source links from the maps use
`../../src/main/...`.
