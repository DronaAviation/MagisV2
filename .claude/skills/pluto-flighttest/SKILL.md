---
name: pluto-flighttest
description: Plan, instrument and analyse MagisV2 / Pluto flight tests from PlutoMonitor logs. Use when the user shares a flight or bench log (logs*.txt from PlutoMonitor), asks what a flight showed, asks to add or change fields in the PlutoPilot.cpp diagnostic log, or needs a test procedure to validate a firmware change on hardware (altitude hold, barometer, sensor drift, any tuning constant).
---

# Flight test: plan, log, analyse

Firmware changes on this drone are validated by flying and logging, not by unit
tests. This skill covers the three parts of that loop. The analysis script is
[`tools/flightlog.py`](../../../tools/flightlog.py) (standard-library Python).
Record results in the relevant `docs/fw-development-reference/active-development/<topic>/TESTING.md`.

## 1. Analyse a log

```bash
python tools/flightlog.py summary <log>      # fields, dropouts, gaps, arm segments - run first
python tools/flightlog.py report  <log>      # temperature fit, height hold, applied correction
python tools/flightlog.py table   <log> --step 10   # block-averaged table of every field
```

- **Run `summary` first.** Fields missing from a share of records mean the log
  line overran the `Monitor_Print` budget (see section 3); an app that keeps
  disconnecting while logging is the same overrun. Records
  split on a repeated field name, so field order does not matter.
- **Field names** default to the altitude-hold log (`degC`, `BaroAlt`, `ToF`,
  `PaI`, `Arm`); override with `--temp --alt --tof --pressure --arm`.
- **`report`** gives, per ground segment and flight:
  - pressure vs temperature slope (Pa/°C), with `r` and residual; flagged when
    the temperature span is under 2 °C or the fit is poor;
  - ground closure: pressure after landing minus before arming;
  - height hold: `BaroAlt` trend and, with `ToF`, true-height trend, for several
    window starts (`--skip 20 40 60`), plus true height minus `BaroAlt` in 30 s
    steps;
  - the correction the firmware applied, backed out from `BaroAlt` vs raw
    pressure: its slope identifies which coefficient a build carried, and
    "correction used" is compared with `--limit` (`BARO_COMP_LIMIT_PA`, default 25).
- Large logs (15k+ lines): delegate to the `pluto-log-analyst` agent so the raw
  data stays out of the main conversation.

### Reading the results

- **Trust a trend only if the window rows agree.** Under ~2 min of hover the
  slope swings by ±10 cm/min with the window start. Report the spread, not one
  number.
- **The drone wobbles ±8 cm regardless.** A difference smaller than that between
  two flights is not a result.
- **`ToF` is slant range.** Subtract the on-ground reading (the script uses the
  median of disarmed samples); fine at hover angles.
- **Check which build flew.** If the applied slope matches a previous build's,
  the new firmware was probably not flashed. Timestamps of `Build/<TARGET>/*.hex`
  vs the log help.
- **Say what the log cannot show.** No throttle field means the throttle
  coefficient cannot be checked; a warm start cannot test a cold-start limit.

## 2. Plan a test

State in advance what result would confirm the change and what would refute it.

| Test | Sequence | Shows |
|---|---|---|
| Ground-flight-ground | 20-30 s disarmed, arm, take off, hover, land, 20-30 s disarmed | Ground closure (sensor shift over the flight, no rotor wash), hover trend |
| Cold start | Board unpowered 15+ min first; log from power-on | Largest temperature rise, best coefficient fits, limit headroom |
| Long hover | 3+ min hover, no throttle input (pitch/roll only) | Reliable trend; anything shorter is noise-dominated |
| Bench soak | Motors off, untouched, 15+ min | Drift without airflow or throttle |
| A/B | Same sequence and similar start temperature on build A then build B | Effect of a constant change |

Ask the user to save each log under its own name (`logs.txt` gets overwritten)
and to note anything the log cannot record (visible sink, bumps, wind).

## 3. Instrument the firmware

The diagnostic log lives in `plutoLoop ( )` in `PlutoPilot.cpp` and runs only in
Developer Mode with a live RC link.

- **Budget ~130 bytes per tick with the app connected.** `Monitor_Print` writes
  into the MSP UART's 256-byte TX ring buffer and `uartWrite ( )` does not check
  for full, so an overrun overwrites the start of the line or the app's MSP
  replies, and the app disconnects ( ~180 B/tick did, 115-135 B/tick ran clean ). Each field costs ~5 bytes of
  framing plus tag and value. Comment out fields rather than deleting them.
- **End the line with `Monitor_Println`** on the last field.
- **Use unrounded values:** `Monitor_Print ( " degC:", ( double ) x, 1 )`. Whole-degree
  temperature once hid ~1 °C of warming and led to a wrong coefficient and a
  misattributed "unexplained drift".
- **Firmware internals** are declared `extern "C"` at the top of `PlutoPilot.cpp`
  rather than including internal headers. `platform.h` is included so target
  defines (e.g. `LASER_TOF`) reach `#ifdef`s.
- **Ground truth:** `#define LASER_TOF` in the working target's `target.h` logs the
  VL53L0X as `ToF` without affecting control. `LASER_ALT` puts the laser *in* the
  loop and must stay off when testing the barometer path.
- **All of this is temporary.** Note in the topic README that it must be removed
  before release; `pluto-commit` checks for it.
