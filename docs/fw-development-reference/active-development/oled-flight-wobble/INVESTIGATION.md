# OLED Flight Wobble: investigation

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

**Status:** cause identified from the code and the symptom; fix written, built and reviewed. Not yet
confirmed in flight ( see [TESTING.md](TESTING.md) ).
**Target:** `PRIMUS_X2_v1`, receiver `Rx_ESP`. **Date:** 10 Oct 2026.

Line numbers below are from **before** the fix, unless marked otherwise.

---

## 1. Problem

A user project ( Traffic Hazard Drone ) added an SSD1306 OLED and a buzzer. `PlutoPilot.cpp` calls
`Oled_Init ( )` in `plutoInit`. In Dev mode the project draws a warning screen in User mode; with Dev
mode off the system telemetry screen is shown.

With the OLED fitted, the drone **did not fly stably**:

- It wobbled for the **whole flight**, with Dev mode **on or off**.
- Looked at closely, it was **about 5 twitches per second**: rhythmic kicks, not a smooth buzz.
- The OLED was **firmly mounted** ( screws / standoffs ).
- The same drone without the OLED fitted flew stable.

## 2. Evidence from the code

### 2.1 `Oled_Init ( )` turns on system telemetry in flight

- `Oled_Init ( )` ( `API-Src/Oled.cpp` ) only sets `OledEnable = true`.
- `main.cpp:594-595`: `if ( OledEnable ) OledStartUpInit ( )`. That is the only place `OledInitStatus` is set
  ( `oled_display.c:114` ).
- `mw.cpp:1130`: `userCode ( )` calls `OledDisplayData ( )` on **every** call, Dev mode or not, armed or not.
- `oled_display.c:331-345` `OledDisplayData ( )`: the only gates are `OledInitStatus`, `oledMode == USER` and
  a 200 ms timer. **Nothing checks ARMED.**

The stock `PlutoPilot.cpp` never calls `Oled_Init ( )`, so on a stock build the OLED code does nothing in flight.
That is why the problem appeared only with this project.

### 2.2 The telemetry writes one blocking I2C transaction per font byte

- `display_ug2864hsweg01.c:233-242` `i2c_OLED_send_char ( )`: 5 font bytes + 1 gap byte, each sent with its
  own `i2cWrite ( )`. That is **6 I2C transactions per character**.
- `i2c_OLED_set_xy ( )` adds 3 more transactions per text field.
- The telemetry tick ( `OledDisplayNav ( )` + `OledDisplaySystemData ( )`, `oled_display.c:265-326` ) writes
  about 35 status characters and about 63 characters of P/R/Y and RC values, plus 14 cursor moves.
  That is **about 630 transactions**.
- I2C1 runs at 400 kHz and is **polled** ( `bus_i2c_stm32f30x.c`, `i2cWrite` / `i2cWriteBuffer` poll the
  ISR flags ). One `i2cWrite` costs about 85 µs ( 29 bit-times × 2.5 µs plus polling ).

**Result: about 45-60 ms of blocking I2C every 200 ms.**

### 2.3 The OLED shares the bus with the flight sensors

On `PRIMUS_X2_v1` the **ICM20948 gyro/accelerometer, the ICP10111 barometer and the AK09916 magnetometer** are
all on I2C1 with the OLED. SPI2 is used only for flash. Everything is polled, so while an OLED write runs,
nothing else in `loop ( )` runs and no sensor can be read.

### 2.4 Where it sits in the control loop

`mw.cpp:1281-1395`, inside the gated cycle ( `looptime` = 3500 µs ):

```text
imuUpdate ( )  →  cycleTime measured  →  annexCode ( )  →  userCode ( )  →  pid_controller ( )  →  writeMotors ( )
                                                            └─ OledDisplayData ( )   ← 45-60 ms here, every 200 ms
```

So once every 200 ms:

- The PID runs on a gyro sample that is **about 50 ms old**.
- The next `cycleTime` is **about 54 ms** instead of 3.5 ms: **13-17 control cycles are missed**.
- `pid.cpp:831` ( pidRewrite, the active controller, `config.cpp:172` ): the I-term is multiplied by
  `cycleTime`, so it takes a step **about 15 times** the normal size.
- `pid.cpp:869-871`: the D-term is the raw gyro difference with no division by dt, so 54 ms of gyro change
  arrives as **one D spike**.
- `imu.cpp:349-350` integrates a single gyro sample over the whole `deltaT`, so the attitude estimate jumps.

**This gives a kick 5 times a second ( 1 / 200 ms ) that the pilot sees as a constant wobble.**

### 2.5 Other blocking paths found

| Path | Blocking | When |
|---|---|---|
| System telemetry tick | 45-60 ms | every 200 ms, armed or not |
| Dev-mode toggle → `i2c_OLED_clear_display_quick ( )` | ~24 ms | each toggle |
| User full-screen `Oled_Update ( )` ( `send_changed_bytes`, all pages ) | ~25 ms | e.g. entering Dev mode |
| User message change ( one page ) | ~3 ms | every 3 s in the project |
| Start-up splash page | ~40 ms per loop for 2 s | boot only, before arming |
| `Oled_Update ( )` compare when nothing changed | ~0.1-0.15 ms | every call |

A missing or dead panel makes things worse: `i2cWriteBuffer ( )` does not check NACK, so each transaction spins
the full `I2C_LONG_TIMEOUT` and then resets I2C1 ( `i2cRecoverPeripheral ( )` ), the bus the IMU shares.

## 3. Hypotheses ( ranked )

| # | Hypothesis | For | Against | Test |
|---|---|---|---|---|
| 1 | **I2C blocking in the loop** ( §2 ) | Only the System-mode path runs with Dev mode off, the reported case. The 5 Hz rhythm matches the 200 ms tick. The stock code had no OLED traffic. | None found. | A/B flight, `Oled_Init ( )` out vs in ( TESTING.md §1 ). |
| 2 | Mechanical: OLED mass or a loose mount couples vibration into the IMU | It wobbles only with the OLED fitted. | Firm screw mount. A mechanical cause would give a fast buzz, not a 5 Hz rhythm, and would not depend on `Oled_Init`. | A/B flight: if A ( no `Oled_Init` ) also wobbles, it is this one. |
| 3 | Electrical: the module's pull-ups or capacitance on I2C1, or charge-pump noise on 3.3 V, corrupt gyro reads | Same bus as the IMU. | It would not be periodic at 5 Hz. | I2C error count in MSP_STATUS ( `serial_msp.cpp:846` ); the OLED unplugged with its weight taped on. |
| 4 | Buzzer on GPIO_1 | New hardware. | It is held LOW with Dev mode off, and the wobble happens then too. | - |

**Conclusion:** hypothesis 1. It is proven to exist from the code, and it matches every detail of the symptom.
Flight confirmation is still pending.

## 4. Decisions ( interview with the user, 10 Oct 2026 )

| Question | Answer |
|---|---|
| Mode | `fix` |
| When does it wobble | Whole flight, Dev mode on or off |
| Look of the wobble | About 5 twitches per second |
| OLED mounting | Firm ( screws / standoffs ) |
| What the OLED shows with Dev mode off | **Telemetry always**, live in flight as well |
| Fix scope | **Firmware change OK** ( fix it for every user ) |
| Design | **A: RAM framebuffer + chunked diff flush** |
| Confirm the cause first | Yes: A/B flight ( TESTING.md §1 ) |
| Done when | Stable hover with Dev mode off and on, as steady as without the OLED |

Options that were not chosen:

- **Freeze telemetry while armed** ( ~20 B ): the simplest fix, but the screen stops updating in flight, which
  the user did not want.
- **PlutoPilot.cpp only** ( take the OLED into User mode at start-up ): it fixes only this project, and the
  system screen is lost.
- **Interrupt / DMA I2C**: truly non-blocking, but the OLED would share I2C1 with the polled sensor reads, which
  needs bus arbitration and a DMA channel. Highest effort and risk.

## 5. Fix ( summary )

Nothing writes to the panel directly any more except the boot splash. Everything goes through one bounded
flush per loop.

1. **System telemetry is drawn into RAM.** A new `oledSystemBuffer [ 1024 ]` and `sysText ( )` draw the 6 px
   grid text into it. All the `disp* ( )` functions use it instead of `i2c_OLED_set_xy ( )` /
   `i2c_OLED_send_string ( )`.
2. **One diff flush per loop.** `oledFlushStep ( )` is called from `OledDisplayData ( )` once per gated cycle.
   - It sends **one run of at most 16 changed bytes** ( `OLED_FLUSH_MAX_BYTES` ), compared against the shadow
     buffer: **about 0.6 ms**.
   - It goes idle when the frame matches the panel.
   - A full screen takes about 64 loops ( ~225 ms ).
3. **A cheaper write.** New `i2c_OLED_send_run ( )`: the page and column commands in one transaction ( Co = 1
   per command ), then the data in a second one.
4. **No hardware access from the API calls.**
   - `Oled_Update ( )` / `Oled_display_Update ( )` only queue the frame.
   - `Oled_Mode ( User / System )` only switch the buffer.
   - `Oled_Print ( )` draws into RAM.
   - The Dev-mode toggle clears RAM, not the panel.
5. **A dead panel is switched off.** A leaky failure score ( +4 on fail, −1 on success, limit 12 ) sets
   `OledInitStatus = false`. This covers 3 failures in a row, or more than about 1 run in 5 failing.
6. **The shadow stays in step with the panel.** The two remaining direct hardware clears ( end of the splash,
   `Oled_display_Clear ( )` ) call `oledPanelCleared ( )`.

**Cost:** RAM +1 KB ( 17.8 / 40 KB ), flash +0.3 KB. Per loop: 0.6 ms or less, only while a frame differs
from the panel.

**Files**

| File | Change |
|---|---|
| `src/main/io/oled_display.c` | Framebuffer, flush, `sysText`, modes, update, print, banner history |
| `src/main/drivers/display_ug2864hsweg01.c` | `i2c_OLED_send_run ( )` |
| `src/main/drivers/display_ug2864hsweg01.h` | Its declaration |
| `src/main/io/oled_display.h` | Comments, Last Modified |
| `src/main/API/Oled.h` | Comments ( `Oled_Clear`, `Oled_Update`, `Oled_Print` ), Last Modified |

**Verified:** `PRIMUS_X2_v1` builds; warning gate "no new warnings in src/"; `pluto-reviewer`: no blocking
findings, and the should-fix items and nits were applied.

## 6. Behaviour change for user code ( public API )

The signatures are unchanged, so existing projects compile.

- `Oled_Update ( )` is **deferred**. A line of text appears in a few ms, a full screen in about 0.25 s.
  Full-screen animations are limited to about 4 completely new frames per second.
- A buffer passed to `Oled_display_Update ( )` must stay valid after the call ( `static` ), because the flush
  reads it over the next loops. Possible follow-up: copy the user's buffer into `oledSystemBuffer`, which is
  idle in User mode. That costs about 5 µs per call and lifts this restriction.

This needs an `API_Version` minor bump at commit. The wiki ( `docs/API/OLED_API_WIKI.md`, section "Timing and
flight safety" ) is already updated.

## 7. Open items

1. **The A/B flight and the validation hover** ( [TESTING.md](TESTING.md) ).
2. **At commit:**
   - `API_Version` bump.
   - CHANGELOG entry.
   - Correct the v4.0.0 CHANGELOG line that calls the old OLED rendering "non-blocking".
   - Note in `Hardware_Bus_Pipeline.md` that I2C1 is polled and shared with the OLED.
3. **Not in scope:**
   - The start-up splash ( still blocking, boot only ).
   - The `mw.cpp:1134` user-loop timer: Dev mode can be ignored if first switched on more than 35.8 min
     after boot.
