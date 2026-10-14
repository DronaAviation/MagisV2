# RX Signal-Loss Tolerance - Investigation

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md) · [CONTROLLER_FINDINGS](CONTROLLER_FINDINGS.md) · [MSP_TIMING](MSP_TIMING.md)

## Problem

On the Wi-Fi RC link ( Pluto Controller v3, MSP over TCP at 100 Hz ), a drone flying normally declares
`Signal_loss`, starts `LAND` and disarms. This happened on every one of four consecutive test flights, 9-75 s
after take-off, on a link that was healthy before and after: RSSI −48 to −57 dBm, full transmit power, no
disassociation. The controller measured two-way blackouts of **1.21 s, 1.43 s and 2.17 s**.

Source: the controller-side report [CONTROLLER_FINDINGS.md](CONTROLLER_FINDINGS.md) and the timing note
[MSP_TIMING.md](MSP_TIMING.md). Both were moved here from the repo root in task 7, with the timing corrected.

**Likely cause of the stalls, not proven:** TCP head-of-line blocking. One lost RC frame holds up everything
queued behind it until the retransmission timeout ( ~1 s, doubling on repeat ), and telemetry shares the
socket. The fix for the stalls themselves would be on the ESP or controller side, which is out of scope. This
topic makes the flight controller ride through them.

## Timing chain ( from the last `MSP_SET_RAW_RC` )

Traced in the source by the scout ( [SCOUT.md](SCOUT.md) ). This corrects both input notes, which leave out the
first 200 ms.

| Elapsed | What happens | Where |
|---|---|---|
| 0-200 ms | The last frame is **replayed** while `rxSignalReceived` is still true; channels stay valid | `rx/msp.c:37`, `DELAY_5_HZ` `rx.cpp:369` |
| 200 ms | Signal flag drops; Developer Mode stops ( `onLoopFinish`, user overrides cleared ); stick **hold** starts | `mw.cpp:1121` ( gate ), `:1167-1178` ( finish ) |
| ~800 ms | Hold ends ( `MAX_INVALID_PULS_TIME` 600 ms ): roll / pitch / yaw → midrc, throttle → 1200, also into `rcDataPilot`; AUX held, so ARM stays on; `failsafeOnValidDataFailed ( )` starts | `rx.cpp:439-449`, `:563` |
| ~800 ms + period | Link declared down → `Signal_loss`, `LAND`. Period = 200 + `failsafe_delay` × 100 ms | `failsafe.cpp:121`, `:211-216`, `:240-253` |

**Budget to LAND, by `failsafe_delay`:** 0 → ~1.0 s, 10 → ~2.0 s, **15 → ~2.5 s**, 22 → ~3.2 s ( ±20 ms ).

## Root cause, proven

`failsafeOnValidDataFailed ( )` compared against the fixed `PERIOD_RXDATA_FAILURE` ( 200 ms ) instead of
`failsafeState.rxDataFailurePeriod`, so the configured `failsafe_delay` ( default 10, 1 s ) was never applied.
The link was declared down ~1.0 s after the last frame. Every stall measured ( 1.21-2.17 s ) is longer than that.

**The user's fix ( in the working tree at planning ):** compare against `rxDataFailurePeriod`
( `failsafe.cpp:213` ). The scout confirmed it:
- `failsafeReset ( )` runs from `activateConfig ( )` at boot and after EEPROM reloads, so the period is never 0.
- A delay of 0 gives exactly the old 200 ms, so the change can never make failsafe fire sooner.
- All the times are `uint32_t`, so there is no new conversion.

## What the drone does during the longer window ( from ~0.8 s )

- **ALT_HOLD:** throttle 1200 is a −300 deflection, which lowers the setpoint at ≈16 cm/s
  ( `altitudehold.cpp:545` ). At 15 that is about 27 cm before `LAND`; at 22, about 38 cm.
- **ANGLE throttle mode:** 1200 is well below hover, so the drone sinks for ~1.7 s and may touch down. The crash
  detector ( ANGLE only ) may then disarm it.
- **Recovery before the link is declared down:** nothing latches. The sticks return to live values and
  Developer Mode restarts. After the link is declared down, `LAND` stays latched until disarm, as before.
- **Real loss of the controller:** `LAND` starts at ~2.5 s instead of ~1.0 s. The 0-0.8 s replay-and-hold
  phase ( flying the last stick ) is unchanged.

## Q&A that shaped the plan ( 13 Oct 2026 )

- **Mode:** `fix`; the user wrote the code, and the plan validates and ships it. **Target:** PRIMUS_X2_v1.
- **Delay:** the user first set 22 ( ~3.2 s ). Recommendation: **15 ( ~2.5 s )**. It clears the 2.17 s worst
  stall and stays within the findings doc's ~2.5 s cap, and on a real loss it lands sooner and sinks less.
  The user chose 15.
- **Drones with saved settings** keep their stored `failsafe_delay`, because `resetConf ( )` runs only on an
  invalid EEPROM. The user chose to **bump `EEPROM_CONF_VERSION` ( 107 → 108 )**, which resets every saved
  setting on first boot, accelerometer calibration included.
- **Modes flown over Wi-Fi:** both ALT_HOLD and ANGLE throttle mode, often, so both are tested.
- **The app** never writes the failsafe config.
- **Done when:** controller logs show stalls ( drop counters ) without SIGNAL LOSS, and switching the controller
  off mid-hover still lands.
- **Out of scope:**
  - `MAX_INVALID_PULS_TIME` and `DELAY_5_HZ`;
  - the controller and ESP firmware ( UDP );
  - `LAND` behaviour after `Signal_loss`;
  - the app.
