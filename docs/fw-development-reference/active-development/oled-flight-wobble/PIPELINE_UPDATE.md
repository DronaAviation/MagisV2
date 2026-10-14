# OLED Flight Wobble - Pipeline update

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

> **Not live.** This is staged text for `fw-architecture-pipeline/` and the CHANGELOG, applied by `pluto-commit`
> when this topic is committed. Checked against the source on 14 Oct 2026:
> - `bus_i2c_stm32f30x.c:261-284` polls the ISR flags with `I2C_LONG_TIMEOUT`, and has no DMA or I2C interrupt;
> - `oled_display.c:129` sets `OLED_FLUSH_MAX_BYTES` 16; `oledFlushStep ( )` is at `:165`;
> - `OledDisplayData ( )` is called from `mw.cpp:1130`.

---

## 1. `subsystems/Hardware_Bus_Pipeline.md`

In **Overview**, replace the sentence "The pipeline is optimized to use DMA … to avoid blocking the high-frequency
PID control loop." with:

````markdown
I2C is **polled**: every transaction busy-waits on the peripheral's status flags ( `I2C_LONG_TIMEOUT`, then a
bus recovery on timeout ) and blocks the loop until it finishes. There is no I2C DMA or interrupt. Peripherals on
the shared I2C1 bus must therefore budget their bytes per loop.
````

Replace the `i2cInit()` / `spiInit()` bullet under **Primary Functions** ( it claims I2C DMA channels ) and the
`i2cRead()` / `i2cWrite()` bullet with:

````markdown
- `i2cInit()` / `spiInit()`: configure the GPIO alternate functions and clock speeds for the bus. I2C uses no DMA.
- `i2cRead()` / `i2cWrite()` / `i2cWriteBuffer()`: blocking, polled I2C transactions at 400 kHz
  ( `I2C_Timing` 0x00E0257A ). A byte costs ~25 µs on the wire plus per-transaction overhead.
````

Add to **Data Flow & Boundaries**:

````markdown
- **Shared I2C1 budget**: the IMU, barometer, magnetometer and the OLED ( 0x3C ) share I2C1 inside the gated loop.
  The OLED never writes the panel from the loop directly. Drawing goes to a 1 KB RAM framebuffer, and
  `oledFlushStep ( )` ( called from `OledDisplayData ( )`, `mw.cpp:1130` ) sends at most **one run of up to 16 changed
  bytes per loop** ( ~0.6 ms ), and only while the frame differs from what the panel shows. The old per-byte
  writes ( ~630 transactions, 45-60 ms every 200 ms ) made the drone twitch at 5 Hz
  ( [oled-flight-wobble](../../active-development/oled-flight-wobble/README.md) ).
- **OLED failure**: if the panel stops answering ( 3 failed runs in a row, or more than ~1 in 5 failing ), the
  OLED is switched off, so a loose connector cannot keep stalling the loop. The 2 s start-up splash still writes
  the panel directly, at boot only.
````

---

## 2. `subsystems/User_Space_API.md`

Add to **Data Flow & Boundaries**:

````markdown
- **OLED is deferred**: `Oled_Update ( )` / `Oled_display_Update ( )` only queue the frame. The firmware sends the
  changed bytes to the panel at most 16 per loop, so a full screen appears in ~0.25 s. A buffer passed to
  `Oled_display_Update ( )` must stay valid ( `static` or global ). `Oled_Update ( )` works in `User` mode only, and
  `Oled_Mode ( )` starts from a blank frame. Wiki: `docs/API/OLED_API_WIKI.md`.
````

---

## 3. `CHANGELOG.md` ( open v4.0.0 section )

- **Fixed**, new entry: "**OLED**: with an OLED fitted and `Oled_Init ( )` called, the drone twitched about 5 times a
  second in flight. The system telemetry wrote the panel one blocking I2C transaction per byte ( ~630 every 200 ms,
  45-60 ms ) on the bus the IMU uses. Drawing now goes to a RAM framebuffer, and at most 16 changed bytes are sent
  per loop; `Oled_Update` / `Oled_display_Update` only queue the frame ( buffers must stay valid ). The OLED is
  switched off if the panel stops answering. RAM +2 KB."
- Lines 10-11 ( highlights ) and 36-37 ( Added, OLED ): "non-blocking" was not true before this fix. Keep the
  word, and point the Added entry at the Fixed entry, for example "non-blocking rendering ( framebuffer and
  rate-limited flush, see Fixed )".
- Version line: 3.11.3 for the OLED flight fix; API 1.4.1 for the deferred `Oled_Update`.
