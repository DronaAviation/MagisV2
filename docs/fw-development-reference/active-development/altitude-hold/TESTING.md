# Altitude Hold: Testing and Measurements

[README](README.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

All September measurements are from one `PRIMUS_V5`, flown indoors on
17 Sep 2026, with a VL53L0X laser logged as ground truth.

Re-run any of these with [`tools/flightlog.py`](../../../../tools/flightlog.py)
( `python tools/flightlog.py report logs3.txt` ). The numbers below were computed
by hand before the script existed; it agrees with them to within a few
hundredths of a Pa/°C ( block boundaries differ slightly ).
See the `flight-test` skill for how to read the output.

## How the coefficients were measured

Conversion throughout: **1 Pa is about 8.37 cm** at the test site ( ~100 750 Pa ).

### Temperature: -2.1 Pa/degC ( set ), measured -2.17 to -2.42

All on one PRIMUS_V5, 17 Sep 2026, fitting 5 s block averages of raw ICP pressure
( `PaI` ) against its unrounded die temperature ( `degC` ):

| Flight | Condition | Pa/degC | r |
|---|---|---|---|
| 13:38 cold start | ground, before + after flight | -2.42 | -0.97 |
| 13:38 cold start | hover | -2.24 | -0.98 |
| 14:23 | hover, 3 min | -2.17 | -0.98 |
| baseline ( before 15:18, log overwritten ) | hover | -2.38 | -0.94 |
| 15:18 ( `logs2.txt` ) | hover | -2.40 | -0.96 |
| 16:05 ( `logs.txt` ) | hover, 3.7 min | -2.39 | -0.99 |
| 16:18 ( `logs3.txt` ) cold start | hover, 4.1 min / ground closure | -2.27 to -2.36 / -2.40 | -0.99 |

Mean about -2.32. Residual after the fit is ~0.36 Pa ( ~3 cm ). What was checked
and ruled out:

- **Lag:** no delay between temperature and pressure improved the fit.
- **STM32 die temperature** ( ADC1 internal sensor ): fitted worse ( r -0.97,
  0.44 Pa residual ) than the ICP's own temperature, and the MCU-minus-ICP
  gradient explained almost nothing ( r -0.20 ).
- **Ambient room pressure:** a BMP280 logged beside the ICP has a temperature
  coefficient of the *opposite* sign ( +2 Pa/degC ). A single constant room
  pressure fitted both sensors over a 7 minute run, so the room was steady within
  ~1 to 2 Pa. The ICP's drift is its own.

**Why 2.1 and not 2.32:** under-correction sinks gently as the board warms
( ~2 cm per degC at 2.1 ), over-correction climbs. Staying below the mean keeps
any error on the sinking side.

**Why the August figure was -1.9:** the bench soak logged `degC` as a whole
number. The original argument that drift continued "while degC saturated at 36"
cannot stand: nearly a degree of warming is invisible inside one integer step,
and at -2.3 Pa/degC that is ~2 Pa. The "1.67 Pa/min unattributed drift" is almost
certainly the same thermal term.

### Temperature limit: 25 Pa

Measured warming **from the arm instant** ( what the correction references ) and
the total correction the firmware applied, worked out from `BaroAlt` vs `PaI`:

| Flight | Start | Rise from arm | Correction used | Share of 25 Pa |
|---|---|---|---|---|
| `logs2.txt` | warm ( 33.5 degC ) | 3.5 degC | 8.6 Pa | 34 % |
| `logs.txt` | 32.2 degC | 6.0 degC | 13.6 Pa | 54 % |
| `logs3.txt` | **cold ( 29.4 degC )** | **8.0 degC** | **17.8 Pa** | **71 %** |

**The limit is tighter than it first looked.** A 4 minute cold-start hover already
uses 71 %; the ~7 Pa left covers only ~3 degC more warming. A full-battery cold
start may reach it, after which the craft sinks at the full uncorrected rate
( ~18 cm per degC ). The old 15 Pa would have clipped the cold-start flight. If a
longer cold flight shows a late-flight sink, raise the limit to 30 to 35 Pa
( ~2.5 to 3 m fault cap ) rather than lowering the coefficient.

### Throttle: -8.6 Pa per 1000 counts ( August, not re-verified )

Craft held at a fixed 60 cm, throttle swept 1000 to 1816, temperature pinned.
Throttle was not logged in September, so none of those flights could test it.

## Flight results with laser ground truth ( PRIMUS_V5, 17 Sep 2026 )

`ToF` is the VL53L0X slant range minus its on-ground reading ( 3 cm ).

| Flight | Coefficient | Hover | ICP temp | True-height trend | Notes |
|---|---|---|---|---|---|
| 14:23 | 1.9 | 3 min | 34.8 -> 38.1 | -3.9 cm/min | raw baro +16 cm/min |
| baseline ( overwritten ) | 1.9 | 100 s | 35.3 -> 36.9 | -7.6 cm/min | window-sensitive |
| 15:18 `logs2.txt` | 2.2 | 110 s | 35.2 -> 36.9 | -0.1 cm/min | window-sensitive ( -0.1 to -12 ) |
| 16:05 `logs.txt` | 2.2 * | 3.7 min | 34.1 -> 38.1 | -2.5 to -3.7 cm/min | stable across windows |
| 16:18 `logs3.txt` | 2.1 | 4.1 min | 32.2 -> 37.5 | see below | cold start ( 28.1 degC at boot ) |

\* Intended as 2.1 but not re-flashed. Identified from the log: the correction
implied by `BaroAlt` vs `PaI` was +2.41 Pa/degC, matching the known 2.2 flight
( +2.38 ), while the 2.1 flight showed +2.27 ( these include the throttle term ).

**The 2.1 flight** ( ToF minus BaroAlt, 30 s steps after arm ): -1, **-13**, -6,
+1, +5, +13, +4, +8, +2 cm. The fitted "+4 cm/min climb" comes from a dip about
30 s after takeoff that then recovers; after 90 s it holds within +/- 8 cm with no
trend. The same post-takeoff dip appears on the 2.2 flights.

**Lessons for reading these logs:**

- A hover under ~2 minutes gives a slope that swings by +/- 10 cm/min depending
  on where the window starts. Only trust 3+ minute hovers.
- `BaroAlt` vs `ToF` has shown flight-to-flight offsets of 0 to 18 cm that stay
  constant within a flight. Likely set at arm, not drift.

### Log fields ( current `PlutoPilot.cpp` )

| Field | Meaning |
|---|---|
| `degC` | ICP-10111 die temperature, 0.1 degC, the value the compensation uses |
| `BaroAlt` | Compensated barometric altitude, cm, what the estimator corrects towards |
| `ToF` | VL53L0X range, cm, -1 out of range. Only when `LASER_TOF` is defined |
| `PaI` | ICP-10111 raw pressure, Pa, 2 decimals |
| `Arm` | 1 armed / 0 disarmed, ends the line |

Each field arrives on its own line, prefixed with a host timestamp by PlutoMonitor.
Records start at `degC`.
