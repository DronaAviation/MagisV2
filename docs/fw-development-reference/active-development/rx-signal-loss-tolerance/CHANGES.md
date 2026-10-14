# RX Signal-Loss Tolerance - Changes

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md) · [CONTROLLER_FINDINGS](CONTROLLER_FINDINGS.md) · [MSP_TIMING](MSP_TIMING.md)

## Task 1 - Apply `failsafe_delay`, default 15 ( 13 Oct 2026 )

- [failsafe.cpp:214](../../../../src/main/flight/failsafe.cpp#L212) `failsafeOnValidDataFailed ( )`: compares the
  time since the last valid RC data against `failsafeState.rxDataFailurePeriod`
  ( = `PERIOD_RXDATA_FAILURE` + `failsafe_delay` × 100 ms, set in `failsafeReset ( )` ) instead of the fixed
  200 ms `PERIOD_RXDATA_FAILURE`. Written by the user. The configured `failsafe_delay` is applied for the first
  time; at 0 the behaviour is the old 200 ms.
- [config.cpp:554](../../../../src/main/config/config.cpp#L554) `resetConf ( )`: default `failsafe_delay`
  10 → **15**. The user first set 22, then 15 as decided in planning. The comment now gives the effect:
  `LAND` ~2.5 s after the last RC frame. The banner date was updated by the user.
- [failsafe.h:40](../../../../src/main/flight/failsafe.h#L40): field comment "1sec in example (10)" →
  "1.5sec by default (15)".

Rollback: put `PERIOD_RXDATA_FAILURE` back at `failsafe.cpp:214`, which gives `LAND` ~1.0 s after the last frame.

## Task 2 - `EEPROM_CONF_VERSION` 107 → 108 ( 13 Oct 2026 )

[config.cpp:157](../../../../src/main/config/config.cpp#L150): `EEPROM_CONF_VERSION` 107 → **108**, with a history
line above it. On the first boot after flashing, the version check fails, so `resetEEPROM ( )` → `resetConf ( )`
writes the defaults; this happens before `failsafeInit`, `rxInit` and the loop. **Every saved setting is reset:**
- **Calibration:** accelerometer offset, scale and trims; **compass `magZero` / `magScale`. All three targets
  define `MAG_ENFORCE`, so on Wi-Fi / PPM the craft refuses to arm until the compass is calibrated** ( ELRS / CRSF skips
  this check, `mw.cpp:291` ); board alignment.
- **Tuning:** PIDs, rates and expo, filters, alt-hold rates and deadband, `max_angle_inclination`, looptime. All
  profiles become copies of profile 0.
- **RX:** rxfail, channel ranges, rcmap, midrc / mincheck / maxcheck, failsafe config ( delay 15 ),
  `auto_disarm_delay`.
- **Battery:** capacity back to 600 mAh ( a value set from the app is lost ), voltages, meter type.
- **Not lost:** the RX feature and AUX slots 0-4 plus the Developer Mode AUX are set again on every boot by
  `plutoRxConfig ( )` → `Receiver_Mode ( )`. AUX ranges in slot 5 and above that were set from the app or CLI
  are lost.
- No other persistent storage exists: the fuel-gauge state is RAM only.
- Switching between a 107 build and a 108 build resets the settings on every flash, in both directions.

Rollback: back to 107. Drones already booted on 108 have been reset either way.

## Task 10 - Cap `failsafe_delay` at 22 ( 13 Oct 2026 )

- [failsafe.h](../../../../src/main/flight/failsafe.h#L37): new `FAILSAFE_DELAY_MAX` 22 ( deciseconds ).
- [failsafe.cpp:122](../../../../src/main/flight/failsafe.cpp#L121) `failsafeReset ( )`: the period uses
  `MIN ( failsafe_delay, FAILSAFE_DELAY_MAX )`, so it is 200-2400 ms whatever is stored. Before, 255 over MSP gave
  25.7 s and 200 over CLI 20.2 s. The default 15 is unchanged. The CLI range stays 0-200; the docs will state the
  effective cap ( task 6 ).

Gate: no new `src/` warnings, flash 100.9 KB, RAM 14.8 KB.

Rollback: remove the `MIN`.

## File index

| File | Tasks |
|---|---|
| [src/main/flight/failsafe.cpp](../../../../src/main/flight/failsafe.cpp) | 1, 10 |
| [src/main/config/config.cpp](../../../../src/main/config/config.cpp) | 1, 2 |
| [src/main/flight/failsafe.h](../../../../src/main/flight/failsafe.h) | 1, 10 |
