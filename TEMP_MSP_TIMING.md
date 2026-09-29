# MSP timing requirements (temporary notes)

For the joystick / controller side. All numbers traced from MagisV2 firmware
source, branch `BugFix-June26`. Transport assumed to be MSP over Wi-Fi with
`FEATURE_RX_MSP` enabled.

---

## Only one command is real time

`MSP_SET_RAW_RC` (ID 200) is the sole message with a hard deadline. Everything
else is either event driven or telemetry polling.

**Send it every 20 ms (50 Hz). Never let the gap exceed 200 ms.**

### What happens when frames stop arriving

Measured from the last `MSP_SET_RAW_RC` received:

| Elapsed | Firmware behaviour |
|--------:|--------------------|
| 200 ms | `rxSignalReceived` goes false. Channels begin **holding** their last values. Nothing visible from outside. |
| 200 to 600 ms | Held values still count as valid, so the failsafe clock keeps being reset. A drone commanded into a turn keeps turning. |
| **600 ms** | Hold expires. Sticks centre, throttle drops to 1200. **This is where the drone stops following the joystick.** |
| **3000 ms** | Link declared down, `Signal_loss` is set, and the drone begins an automatic landing. |

Verified against source on 25 August 2026. The 3000 ms figure is
`600 ms hold + PERIOD_RXDATA_FAILURE 200 ms + failsafe_delay 2200 ms`.

Two deadlines matter, and they are very different:

* **600 ms** is the control deadline. Past this the drone ignores the joystick.
* **3000 ms** is the failsafe deadline. Past this the flight ends in an auto land.

Sending faster than 20 ms gains nothing. The firmware processes RC data driven
with a 50 Hz floor, and the PID loop reads the same values either way.

---

## Command table

| Command | ID | Direction | Send every | Hard limit |
|---------|---:|-----------|-----------|-----------|
| `MSP_SET_RAW_RC` | 200 | to drone | **20 ms** | **200 ms** |
| `MSP_SET_COMMAND` (takeoff, land) | 217 | to drone | on event | none |
| `MSP_SET_MAX_ALT` | 218 | to drone | on change | none |
| `MSP_APP_HEADING` | 221 | to drone | 100 ms if used | none |
| `MSP_APP_GPS` | 222 | to drone | 200 ms if used | none |
| `MSP_ATTITUDE` | 108 | from drone | 50 to 100 ms | none |
| `MSP_ALTITUDE` | 109 | from drone | 100 ms | none |
| `MSP_RAW_IMU` | 102 | from drone | 100 ms | none |
| `MSP_STATUS` | 101 | from drone | 100 to 200 ms | none |
| `MSP_FLIGHT_STATUS` | 255 | from drone | 100 to 200 ms | none |
| `MSP_ANALOG` (battery) | 110 | from drone | 500 to 1000 ms | none |

Configuration writes (`MSP_SET_PID`, `MSP_SET_RC_TUNING`, `MSP_SET_MISC`,
`MSP_SET_FEATURE`, `MSP_EEPROM_WRITE`, and similar) must only be sent while the
drone is on the ground and disarmed. `MSP_EEPROM_WRITE` performs a flash write
and must never be sent in flight.

---

## Two things that matter more than the rates

### Telemetry competes with control

Every telemetry poll shares the link with `MSP_SET_RAW_RC`. On a TCP transport a
stalled segment blocks everything queued behind it, including the next RC frame.

Guidance:

* poll telemetry at the slowest rate the interface can tolerate
* never burst several telemetry requests together
* if the link degrades, drop telemetry rate first and keep RC at 20 ms
* prefer a fixed schedule over request and response round trips

### 200 ms is not the real safety margin

Flight logs on a healthy link at 91 percent battery recorded transport outages of
1.21, 1.43 and 2.17 seconds. Wi-Fi stalls of this length are normal, and they
exceed the 200 ms threshold by a wide margin.

The firmware default `failsafe_delay` is set to 22 (2.2 seconds) to extend the
total budget to roughly 3.0 seconds. See `MAGISV2-SIGNAL-LOSS-FINDINGS.md` in the
repository root for the full analysis.

**Resolved:** that document reports (section 5) that the configured
`failsafe_delay` might not be applied. This has been checked against current
source and it **is** applied. `failsafeOnValidDataFailed()` reads
`failsafeState.rxDataFailurePeriod` at `failsafe.cpp:213`, and the value reaches
it through `activateConfig()` to `useFailsafeConfig()` to `failsafeReset()`. The
3 second budget is real.

---

## Source references

| Value | Where |
|-------|-------|
| 200 ms signal timeout | `src/main/rx/rx.cpp:369`, `DELAY_5_HZ` at `rx.cpp:92` |
| 600 ms channel hold | `MAX_INVALID_PULS_TIME`, `src/main/rx/rx.cpp:87` |
| 200 ms base link down delay | `PERIOD_RXDATA_FAILURE`, `src/main/flight/failsafe.h:35` |
| 2200 ms configured guard | `failsafe_delay` = 22, `src/main/config/config.cpp:544` |
| Combined 2400 ms period | `failsafeReset()`, `src/main/flight/failsafe.cpp:122` |
| Period actually applied | `failsafeOnValidDataFailed()`, `src/main/flight/failsafe.cpp:213` |
| Auto land on signal loss | `src/main/flight/failsafe.cpp:245` |
| 50 Hz processing floor | `shouldProcessRx()`, `src/main/rx/rx.cpp:400` |
| Main loop period 3500 us | `masterConfig.looptime`, `src/main/config/config.cpp:512` |
| `MSP_SET_RAW_RC` handler | `src/main/io/serial_msp.cpp:1406` |
