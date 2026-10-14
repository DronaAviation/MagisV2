# MSP timing requirements

[README](README.md) · [TASKS](TASKS.md) · [INVESTIGATION](INVESTIGATION.md) · [CONTROLLER_FINDINGS](CONTROLLER_FINDINGS.md)

**For:** the joystick / controller side. These are firmware facts for MagisV2 **FW 3.11.2** ( `failsafe_delay`
default 15, capped at 22 ), with MSP over Wi-Fi and `FEATURE_RX_MSP` enabled. Moved here from the repo root
( `TEMP_MSP_TIMING.md` ) on 14 Oct 2026. The timeline is corrected: the old note left out the 200 ms during which
the last frame is replayed, and it assumed a `failsafe_delay` of 22.

---

## Only one command is real time

`MSP_SET_RAW_RC` ( ID 200 ) is the only message with a hard deadline. Everything else is either event driven or
telemetry polling.

**Send it every 20 ms ( 50 Hz ). Keep the gap under 200 ms whenever you can.**

### What happens when frames stop arriving

Measured from the last `MSP_SET_RAW_RC` received:

| Elapsed | Firmware behaviour |
|--------:|--------------------|
| 0 to 200 ms | The last frame is **replayed**: the channels stay valid and nothing changes. |
| **200 ms** | `rxSignalReceived` goes false. **User code ( Developer Mode ) stops** and user RC overrides are cleared. The channels begin **holding** their last values; a drone commanded into a turn keeps turning. |
| **~800 ms** | The hold expires ( `MAX_INVALID_PULS_TIME` 600 ms after the last valid sample ). Sticks centre and throttle drops to 1200. **This is where the drone stops following the joystick.** |
| **~2500 ms** | The link is declared down: `Signal_loss` is set and the drone starts an automatic landing ( `LAND` ). |

The ~2500 ms figure is 800 ms + `PERIOD_RXDATA_FAILURE` 200 ms + `failsafe_delay` 15 × 100 ms. It is ±20 ms at
the 50 Hz RC processing floor. A stored `failsafe_delay` above 22 is capped, so the longest possible budget is
~3200 ms.

Three deadlines matter:

* **200 ms**: user code restarts. This matters only to `PlutoPilot.cpp` code.
* **~800 ms**: the control deadline. Past this the drone ignores the joystick.
* **~2500 ms**: the failsafe deadline. Past this the flight ends in an automatic landing.

If frames come back before ~2500 ms, nothing has latched: the sticks are live again and the flight continues. Once
`LAND` has started it stays on until the drone disarms.

Sending faster than 20 ms gains nothing. The firmware processes RC data with a 50 Hz floor, and the PID loop reads
the same values either way.

---

## Command table

| Command | ID | Direction | Send every | Hard limit |
|---------|---:|-----------|-----------|-----------|
| `MSP_SET_RAW_RC` | 200 | to drone | **20 ms** | **200 ms** ( user code ), ~800 ms ( sticks ), ~2.5 s ( `LAND` ) |
| `MSP_SET_COMMAND` ( takeoff, land ) | 217 | to drone | on event | none |
| `MSP_SET_MAX_ALT` | 218 | to drone | on change | none |
| `MSP_APP_HEADING` | 221 | to drone | 100 ms if used | none |
| `MSP_APP_GPS` | 222 | to drone | 200 ms if used | none |
| `MSP_ATTITUDE` | 108 | from drone | 50 to 100 ms | none |
| `MSP_ALTITUDE` | 109 | from drone | 100 ms | none |
| `MSP_RAW_IMU` | 102 | from drone | 100 ms | none |
| `MSP_STATUS` | 101 | from drone | 100 to 200 ms | none |
| `MSP_FLIGHT_STATUS` | 255 | from drone | 100 to 200 ms | none |
| `MSP_ANALOG` ( battery ) | 110 | from drone | 500 to 1000 ms | none |

Send configuration writes ( `MSP_SET_PID`, `MSP_SET_RC_TUNING`, `MSP_SET_MISC`, `MSP_SET_FEATURE`,
`MSP_SET_FAILSAFE_CONFIG`, `MSP_EEPROM_WRITE` and similar ) only while the drone is on the ground and disarmed.
`MSP_EEPROM_WRITE` performs a flash write and must never be sent in flight. A new `failsafe_delay` takes effect
only after an EEPROM write. Reading it back over MSP gives the stored value, even when the firmware is capping it
at 22.

---

## Two things that matter more than the rates

### Telemetry competes with control

Every telemetry poll shares the link with `MSP_SET_RAW_RC`. On a TCP transport, a stalled segment blocks
everything queued behind it, including the next RC frame.

Guidance:

* poll telemetry at the slowest rate the interface can tolerate;
* never send several telemetry requests in one burst;
* if the link degrades, drop the telemetry rate first and keep RC at 20 ms;
* prefer a fixed schedule over request-and-response round trips.

### 200 ms is not the real safety margin

On a healthy link at 91 percent battery, flight logs recorded transport outages of 1.21, 1.43 and 2.17 seconds.
Wi-Fi stalls of this length are normal, and they are far longer than the 200 ms threshold.

Up to FW 3.11.1 the firmware ignored `failsafe_delay` and landed ~1.0 s after the last frame, so every one of those
stalls ended the flight. From FW 3.11.2, `failsafe_delay` is applied, with a default of 15 and a cap of 22. The
budget is ~2.5 s, which rides through all three measured stalls. See
[CONTROLLER_FINDINGS.md](CONTROLLER_FINDINGS.md) and [INVESTIGATION.md](INVESTIGATION.md).

---

## Source references ( FW 3.11.2 )

| Value | Where |
|-------|-------|
| 200 ms signal timeout ( frame replay ) | `src/main/rx/rx.cpp:369`, `DELAY_5_HZ` at `rx.cpp:92` |
| 600 ms channel hold | `MAX_INVALID_PULS_TIME`, `src/main/rx/rx.cpp:87` |
| rxfail values ( midrc, throttle 1200 ) | `getRxfailValue ( )`, `src/main/rx/rx.cpp:431` |
| 200 ms base link-down delay | `PERIOD_RXDATA_FAILURE`, `src/main/flight/failsafe.h:35` |
| 1500 ms configured guard | `failsafe_delay` = 15, `src/main/config/config.cpp:554` |
| Cap of 22 ( 2200 ms ) | `FAILSAFE_DELAY_MAX`, `src/main/flight/failsafe.h:37` |
| Combined 1700 ms period | `failsafeReset ( )`, `src/main/flight/failsafe.cpp:121` |
| Period applied | `failsafeOnValidDataFailed ( )`, `src/main/flight/failsafe.cpp:212` |
| Automatic landing on signal loss | `src/main/flight/failsafe.cpp:246-249` |
| User code stops at 200 ms | `rxIsReceivingSignal ( )`, `src/main/mw.cpp:1121` |
| 50 Hz processing floor | `shouldProcessRx ( )`, `src/main/rx/rx.cpp:398` |
| Main loop period 3500 µs | `masterConfig.looptime`, `src/main/config/config.cpp` |
| `MSP_SET_RAW_RC` handler | `src/main/io/serial_msp.cpp` |
