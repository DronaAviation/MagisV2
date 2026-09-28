# Battery SoC Fix: Investigation

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md)

## 1. Problem

On PRIMUS_X2_v1, the app's remaining-mAh figure and the low-battery warning cannot be trusted:

- on an empty 600 mAh pack the app showed ~300 mAh remaining;
- the warning came seconds before empty;
- an empty pack plugged in reads about half full.

The evidence comes from the check topic
[battery-capacity-estimate](../battery-capacity-estimate/README.md): six logs, two bench supply sweeps and two
charger readings.

## 2. Findings carried over ( ranked )

| # | Finding | Evidence | Size | In this fix |
|---|---|---|---|---|
| 1 | The test board had a second R020 stacked in parallel ( 10 mOhm ) while the code assumes 20 mOhm: the current and the count read half | log-3 ( half at DC ), log-5 ( right with one ), log-6 + charger ( 528 / 534 ) | ~200 of the ~297 mAh | No code change: one R020 is production ( user, 26 Sep ); the rework is removed |
| 2 | **Empty-pack wrap**: `mAhRemain` wraps past `E`, or `E` wraps at plug-in ≤ 3.0 V; `soc_from_mAh ( )` returns 100%, so the SoC reads 54-58% on an empty pack and the warning clears | log-3 and log-5 at 3.0 / 2.9 V | safety | Yes |
| 3 | **Late warning**: fused SoC 24% with 10% really left, 21% at empty; warning ~15 s before empty at 3.0 V under load | log-6, TESTING *Task 6* | safety | Yes |
| 4 | **Auto-gain** always converges to its 0.95 clamp ( mixed-unit comparison ): count −5% in every flight | log-6: `G` 0.950 at 28 s, `D` = 95% of the pre-gain integral | −5% | Yes ( remove ) |
| 5 | **Starting estimate**: one sample floored to 0.1 V, a straight 3.0-4.2 V line, the labelled capacity; `E` 550 or 600 on the same supply voltage; `Cells` 2 at ≥ 4.2 V | log-3, log-5, log-6 | ~30-100 mAh per boot | Yes |
| 6 | Small counter losses: dt remainder, two round-downs, negative shunt as 65535, bus `0xFFFF` on I2C error | code; log-5 ( 20-50 mA below the supply ) | ~−1% | Yes |
| 7 | `BMS_Update ( )` runs every loop ( rate check against a constant ): the SoC slew limit and the "fresh" checks mean nothing | code | - | Yes ( the new model depends on its rate ) |
| 8 | CRSF current in 10 mA units; app current sent before the gain | code | - | Out of scope |

## 3. Causal chain ( file:line as of 26 Sep 2026, `src/main/` )

- **Wrap:**
  - `sensors/battery.cpp:357` `mAhRemain = ( uint16_t ) ( EstBatteryCapacity - mAhDrawn )` wraps once
    `mAhDrawn > EstBatteryCapacity`.
  - `:209` computes `EstBatteryCapacity` from `vBatRaw x 100 - batteryCriticalVoltage`, which is negative
    below 3.0 V and wraps in `uint16_t`.
  - `:433` `soc_from_mAh ( )` returns 100 when `mAhRemain >= batteryCapacity_mAh`.
  - `fuse_soc_vbatComp_smart ( )` ( `:480-551` ) gives the mAh side 20-75% weight, so the fused SoC jumps. Armed,
    the "never increase" clamp ( `:531-541` ) hides this until landing.
- **Late warning:**
  - The voltage side is a straight line from 2.9 to 4.2 V ( `:454-462` ).
  - The loaded voltage stays at 3.3-3.4 V from 40% to 80% used ( log-6 ), so the blend reads ~15 points high over
    the last 30%.
  - The thresholds are fused SoC ≤ 18% / ≤ 8% ( `:136-138`, `:569`, `:578` ).
  - `BMS_Update ( )` is called every loop ( `mw.cpp:442` compares `currentTime` to the constant
    `BMS_UpdateInterval` ).
- **Auto-gain:** `ina219_auto_calibrate_current ( )` ( `:261-286` ) divides shunt mV minus bus voltage in 0.1 V
  units by an expected sag in mV. The result is always below the 0.95 clamp, and a `static` holds it until
  power-off.
- **Starting estimate:** `handleBatteryConnected ( )` ( `:184-212` ):
  - it takes one `vBatRaw` sample and `delay ( 40 )`s without re-reading;
  - it computes cells as `vBatRaw / 42 + 1` after the thresholds, which use the stale count;
  - it applies a straight line to the configured `BatteryCapacity`.
- **Counter losses:**
  - `:341-345` `dtMs = dtUs / 1000` drops the sub-ms remainder;
  - `drivers/ina219.c:64` returns whole mV;
  - `common/maths.cpp:381` floors the ring average;
  - `:300` passes `int16_t` into a `uint16_t` ring;
  - `ina219.c:41,57` return `0xFFFF` on an I2C error.

## 4. Q&A that shaped the fix ( user, 26 Sep 2026 )

| Question | Answer |
|---|---|
| Topic | New fix topic, linked back to the check topic |
| Production shunt | One R020 ( 20 mOhm ); the stacked second one was a rework |
| Scope | Safety ( wrap, late warning ), accurate count, better starting estimate. Not telemetry units or cleanup |
| Done when | `D` within ±5% of the charger with the auto-gain removed; app remaining at empty ≤5% of capacity; warning with ≥15% really left; no SoC jump on an empty pack |
| SoC model | Count-led, anchored at plug-in by a LiPo curve on an averaged sample; a current-compensated voltage floor fires the warnings |
| Low-battery action | Keep beeper and app flag, earlier; no auto-land, no arming block |
| Capacity | The configured rated value: packs are 600, 800 and 1200 mAh rated when new; the user sets it for the pack in use |
| Aged pack | The voltage floor also pulls remaining to ~0 at empty |
| ELRS 800 default | Keep |
| Constraints | `MSP_ANALOG` layout unchanged; `Bms_Get ( )` may change ( wiki + `API_Version` ) |
| Test code | The log line and the Dev Mode link grace carry over, removed at close |
| Validation | Bench supply sweep first; flights on the log-6 600 pack, a newer 600 ( 5 available ) and an 800 ( 2 available ) |

## 5. Design numbers

Task 1, 28 Sep 2026. Source: [log-6](../battery-capacity-estimate/logs/log-6.txt) ( 600 pack charged full, one R020,
flown to 2.8 V under load, charger 534 mAh after ) and [log-1](../battery-capacity-estimate/logs/log-1.txt) ( same pack,
two R020, current doubled ). Scripts in the session scratchpad, standard-library Python; `tools/flightlog.py` could not
be used ( see the tooling note at the end ).

### 5.1 Method

- **Charge left** at each log-6 record = 1 - ( integral of `I` from arming ) / 556.1 mAh, the whole-flight pre-gain
  integral. 100% is the charge between full and the end of log-6 ( what the charger put back, 534 mAh; the INA219
  reads ~4% above the charger, so its own integral is the consistent scale ). One percent is ~5.3 mAh, ~4.4 s of hover.
- **Voltage resolution** in the firmware is 0.1 V ( `bus_voltage ( )` returns 0.1 V codes, `ina219.c:57`; the ring
  average floors them ). The reading flickers between two codes only while the bus sits on a code boundary, so each
  flicker gives an exact point: bus = X.X00 V ( within ~10 mV ) at a known charge and current. Those flicker points
  are used below, not the floored level.
- **Compensated voltage** here is `Vcomp = V_bus + I x R` with the full current term. The current firmware mixes
  only 35% of it into `vBatComp` ( `battery.cpp:631` ), so these thresholds are **not** comparable with the logged `Vc`.

### 5.2 Log-6 flicker points ( bus on a 0.1 V boundary )

| Bus V | `t` from arm | Charge left | `I` | Standard curve at that charge | Implied R = ( curve - bus ) / I | `Vcomp` at 100 mOhm | Curve at `Vcomp` |
|---|---|---|---|---|---|---|---|
| 3.7 | 28-37 s | 93.8% | 4.15 A | 4.140 V | 106 mOhm | 4.115 V | 91% |
| 3.6 | 88-92 s | 82.0% | 4.20 A | 4.044 V | 106 mOhm | 4.020 V | 80% |
| 3.5 | 161-172 s | 65.7% | 4.25 A | 3.916 V | 98 mOhm | 3.925 V | 67% |
| 3.4 | 254-279 s | 44.3% | 4.30 A | 3.817 V | 97 mOhm | 3.830 V | 48% |
| 3.3 | 389-398 s | 16.8% | 4.35 A | 3.717 V | 96 mOhm | 3.735 V | 21% |
| 3.2 | 434.6 s | 7.8% | 4.35 A | 3.655 V | 105 mOhm | 3.635 V | 7% |
| 3.1 | 452.8 s | 3.8% | 4.40 A | 3.528 V | 97 mOhm | 3.540 V | 4% |
| 3.0 | 462.0 s | 1.8% | 4.40 A | 3.392 V | 89 mOhm | 3.440 V | 3% |
| 2.9 | 469.3 s | 0.2% | 4.40 A | 3.284 V | 87 mOhm | 3.340 V | 1% |

The flight ended at t 469.5 s ( disarm ). The implied resistance is flat at 87-106 mOhm from full to empty: the
standard curve has the right shape for this pack, and one resistance compensates the whole discharge.

### 5.3 Pack resistance

| Source | Step | Result |
|---|---|---|
| **log-6, steady state ( §5.2 )** | loaded bus against the curve, 9 points | **~100 mOhm** ( 87-106 ) |
| log-6 arming, t 2.4 s | rest code 4.1 ( 4.10-4.19 V ) → 3.8 at 4.15 A | 72-96 mOhm ( ±24 from the 0.1 V codes ) |
| log-6 landing, t 469.5 s | 2.9 at 4.4 A → 3.4 after 1.4 s, 3.5 after 4.3 s | 114 mOhm fast, 136 mOhm at 4 s ( near empty ) |
| log-1 arming, t 8 s | rest 4.0 → 3.4 at 4.4 A ( doubled ) | ~136 mOhm |
| log-1 hops 2-5, landings and take-offs | 2.9 ↔ 3.5-3.6 at 4.4-4.8 A ( doubled ) | 125-140 mOhm; last landing ( 2.6 → 3.4 ) ~190 |

**Design value: R = 100 mOhm**, the value the firmware already carries ( `SYSTEM_R_MOHM`, `battery.cpp:94` ). It covers
the pack, the connector and the board path up to the INA219. The log-1 steps read ~30% higher on the same pack; that
day's current is known only through the x2 of the two-shunt board ( log-3 / log-5 show x2 to x2.5 ), and pack
temperature is not logged, so the spread is real but its cause is not settled. The validation flights ( task 9 )
measure R on the newer 600 and the 800 from the arming and landing steps; with the task 2 driver ( mV codes ) those
steps resolve to a few mOhm.

### 5.4 Voltage floor: compensated voltage at 15% and 5% left ( log-6, R = 100 mOhm )

| Level | Bus under load ( interpolated between flicker rows ) | `I` | **`Vcomp`** | Standard curve at that charge | Hover left at ~4.4 A |
|---|---|---|---|---|---|
| **Warning, 15% left** | 3.280 V ( rows 3.3 / 3.2: 16.8% / 7.8% ) | 4.35 A | **3.715 V** | 3.71 V | ~68 s ( t ~401 s of 469 ) |
| **Critical, 5% left** | 3.130 V ( rows 3.2 / 3.1: 7.8% / 3.8% ) | 4.40 A | **3.570 V** | 3.61 V | ~22 s ( t ~447 s ) |

Recommended thresholds for task 5: **warning `Vcomp` ≤ 3.72 V, critical `Vcomp` ≤ 3.60 V** ( 3.60 fires at ~6% on
log-6, the safe side of the log's 3.57 and the curve's 3.61 ). Disarmed, the current term is ~10 mV, so the same
numbers read as resting voltages on the curve ( 15% / 5% ).

**Sensitivity: the floor is only as good as R.** For a pack whose real resistance differs from the 100 mOhm the
firmware uses, the resting voltage at which the floor fires moves by `I x ( R_real - 100 mOhm )`:

| Pack R real | Warning ( 3.72 V ) fires at | Critical ( 3.60 V ) fires at |
|---|---|---|
| 80 mOhm ( fresh or larger pack ) | ~6% left | ~3% left |
| 100 mOhm ( log-6 ) | 15% | 6% |
| 120 mOhm | ~30% | ~11% |
| 140 mOhm ( log-1 steps ) | ~60% | ~25% |

( At 4.35 A; the resting voltage `Vcomp + I x ΔR` read on the curve. ) The curve is flat from 25% to 55%
( 3.75-3.85 V ), so a voltage floor cannot place the 15% warning on its own across packs: a lower-resistance pack
warns late, a higher one very early. At 5% the curve is steep and the error is smaller. Consequence for task 5:
**the count ( anchored by the curve at plug-in ) should fire the 15% warning**, with the voltage floor as the backstop
that catches an aged pack or a wrong capacity setting; the floor's critical level is the more trustworthy of the two.
Task 9 checks both on packs with a different R.

### 5.5 LiPo resting-voltage curve for plug-in ( 1S, voltage → charge left )

The widely published 1S LiPo resting table ( RC hobby charts, 5% steps ), used as is:

| V | 4.20 | 4.15 | 4.11 | 4.08 | 4.02 | 3.98 | 3.95 | 3.91 | 3.87 | 3.85 | 3.84 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| **%** | 100 | 95 | 90 | 85 | 80 | 75 | 70 | 65 | 60 | 55 | 50 |

| V | 3.82 | 3.80 | 3.79 | 3.77 | 3.75 | 3.73 | 3.71 | 3.69 | 3.61 | 3.27 |
|---|---|---|---|---|---|---|---|---|---|---|
| **%** | 45 | 40 | 35 | 30 | 25 | 20 | 15 | 10 | 5 | 0 |

21 points; linear between points, 100% above 4.20 V, 0% below 3.27 V.

**Checks against the logs:**

| Check | Log | Curve | Verdict |
|---|---|---|---|
| Shape over a whole discharge | log-6 flicker points ( §5.2 ) | implied R flat at 87-106 mOhm; curve at `Vcomp` within 4 points of the charge left | fits |
| Full | log-6 plug-in 10 min after the charger: code 4.1 ( 4.10-4.19 V ) | 88-99% | fits ( the spread is the code floor, not the pack ) |
| Empty | log-6 end: 3.5 V 8 s after landing and rising ( log-1: 3.5-3.6 V by 6 s ) | 3.5-3.6 V → 3-5% | fits ( the charger's 534 mAh is the pack less ~3-5% left ) |
| 4.0 V resting | log-1 start: code 4.0 ( 4.00-4.09 V ), then ~410 mAh ( doubled ) to a deeper end than log-6 | 77-87%; the count gives ≥ 410 / 556 = 74% | fits within the x2 uncertainty |

- **Not explained:** the charger put back 430 mAh after log-1 but 534 after log-6, though log-1 ended deeper ( 2.6 V
  under load, 3.4 V at rest ). Either that charge stopped early or it was read at a different point. The log-1
  charger figure is not used here.
- **For task 4:** the curve takes the true resting voltage. Today's plug-in sample is one floored 0.1 V code that reads
  up to ~0.1 V low ( log-3 ), worth up to ~10% on the curve above 4.0 V; the task 2 driver ( mV ) and an averaged
  sample remove that. A pack plugged in straight off the charger reads high ( surface charge ) and one plugged in right
  after a flight reads low; the curve assumes a rested pack.
- **Cell count:** a full pack reads up to 4.2 V ( 4.35 V for LiHV ), so the 1S / 2S split must sit well above that
  ( log-3: `Cells` 2 at a 4.3 V supply ).

### 5.6 Second pack ( task 13, 28 Sep 2026 ): R is per pack

The task 13 flight ( [TESTING.md](TESTING.md) "log-1", a newer 600, `Vm` / `Is` at mV resolution ) measured **~146 mOhm**
( 137-152 over the whole flight, bus basis ) against ~100 mOhm for the log-6 pack. The standard curve fits both packs
with each pack's own R, so §5.5 stands; §5.3's single design value does not.

- A fixed R = 100 mOhm floor would have warned at **66%** left on this pack ( critical at 50% ). With its own 146 mOhm
  it warns at 11.5% and goes critical at 4.7%.
- Counting against the rated 600 warns at 10% and never goes critical: the pack holds ~567 mAh on the INA219's scale, ~525 on the charger's ( 502 mAh put back from
  ~4% left ). The two scales differ by ~5.4% on both packs measured so far ( task 15 ).
- **R measured in the first ~30 s of each flight** ( rest `Vm` before arming against hover `Vm` at ~30 s, less the
  curve's drop for the charge used ) gives 150 mOhm here and ~105 on log-6, each matching that pack's steady state.
  The arming step alone ( +1 s ) reads ~20 mOhm low ( polarization still building ).

**R at boot, without flying ( asked 28 Sep ):** the INA219 is quiet enough at rest ( `Vm` ±2 mV, `Is` ±8 mA ), but
the only load big enough for a step is the motors, so a boot measurement means pulsing the props on the ground. A
short step also measures the wrong quantity: the fast resistance is ~100-110 mOhm on this pack ( regression of `Vm`
on the in-flight ripple of `Is` ) and 123-128 mOhm at +1 s after arming, against the ~146 mOhm the floor needs, which
builds over ~20-30 s. The first 30 s of flight measure the right value with no extra action. Until they have, the
floor uses a low R, which errs toward an early warning. An idle hold of ~3 s after arming on a validation flight
would show what the idle step gives on each pack.

Task 5 now has three designs on the table: a per-flight R with the §5.4 floor voltages; a count warning set higher
than 15% of the rating to cover packs holding ~5-10% less; or both, whichever fires first.

### 5.7 Survey result and recommendation for task 5 ( task 14, 28 Sep 2026 )

Three packs, stopped there by the user ( no log-3 / log-4 ): log-6 ~100 mOhm ( 0.1 V codes only, weakest ), task 13
newer 600 ~146, task 14 older 600 ~157 ( bus basis, steady hover ). Charger IR 100-124 mOhm is the ohmic part only
( TESTING.md ). Replay on the two mV logs ( TESTING.md "Warning replay" ):

| R method | Result | Verdict |
|---|---|---|
| Fixed ( 100, or the average ~135 ) | warns at 39-76% left | rejected: packs differ by ±15-25% in R and the curve is flat mid-pack |
| **Per flight, first ~30 s** | warns at 10-17% ( 3.715 V ) | **recommended** |
| Continuous re-estimate | not built | not needed: within a flight the steady R stays within ±5% ( 137-152, 148-163 ) until the last ~5%, and in flight there is no resting voltage to re-estimate against |

**Recommended design for task 5:**

- **R per flight.** Rest voltage and current averaged before arming; hover averages at ~25-35 s after arming; less the
  curve's drop for the mAh used over those 30 s; clamp to a sane range ( e.g. 80-250 mOhm ). Keep it for the rest of
  the power cycle ( same pack ), so a re-arm after a landing does not need a fresh rest. Until it is measured, the
  voltage floor stays off and the count carries the warnings.
- **Thresholds on `Vcomp` = bus + `I` x R:** warning **3.745 V** ( fires at 17.1% / 18.6% on the two packs; 3.715 fired
  at 10.4% / 17.2% ), critical **3.60-3.62 V** ( 4.5-4.8% on log-1; log-2 landed at 9% before reaching it ).
- **Remaining mAh to the app:** count-led from the curve-anchored start; once the floor is active, the remaining figure
  is pulled down to what `Vcomp` on the curve says whenever that is lower ( an aged pack or a wrong capacity setting ).
- The count warning ( 15% of the configured capacity ) stays as the other trigger, whichever fires first.

**Tooling note ( not this topic ):** `python tools/flightlog.py` fails on import. Running a script from `tools/` puts
that folder first on `sys.path`, and `tools/warnings.py` shadows the standard `warnings` module that `statistics`
imports. Already open as item 12 in `.claude/TOOLING_BACKLOG.md`.
