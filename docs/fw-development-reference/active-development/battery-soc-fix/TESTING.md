# Battery SoC Fix: Testing

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [TESTING](TESTING.md)

Logs of the check topic ( log-1 .. log-6 ) are in [../battery-capacity-estimate/logs/](../battery-capacity-estimate/logs/);
this topic's logs start again at log-1 in [logs/](logs/).

## Task 13 plan: full to empty on a newer 600, 60 s rest ( log-1 )

**Why.** Task 1 found the pack resistance at ~100 mOhm in log-6 but ~130-140 mOhm from the log-1 steps on the same
pack, and the 15% voltage floor moves from ~6% to ~60% left over that range ( INVESTIGATION.md §5.4 ). A second
pack at mV resolution with a real rest after landing settles it.

**Set-up.**

- Build: `Build/PRIMUS_X2_v1/DEFAULT_PRIMUS_X2_v1_3.10.0.hex` from **28 Sep 2026, 15:24** ( current firmware plus the `Vm` /
  `Is` fields; the SoC and warning logic is unchanged ).
- Pack: a **newer 600**. Note which one ( label or mark ) so the later validation flights reuse it. App capacity **600**.
- Props on, one R020 board.

**Log line** ( ~120 B/tick at 10 Hz ): first tick `E:… Cap:… Cells:…`, then every tick
`t V Vm Vc I Is D S M Arm`.

| Field | Meaning |
|---|---|
| `V` | firmware battery voltage, floored 0.1 V ( mV units ) |
| `Vm` | **new**: INA219 bus voltage in mV, one raw sample, -1 on an I2C error |
| `Vc` | firmware `vBatComp` ( 35% current-compensated ) |
| `I` | firmware current ( mA, 50 mA steps, 50-sample average, before the gain ) |
| `Is` | **new**: INA219 shunt current in whole mA, one raw sample; an I2C error reads 0 |
| `D` | firmware mAh consumed ( after the 0.95 gain ) |
| `S` | fused SoC ( % ); the warning fires at ≤ 18, critical at ≤ 8 |
| `M` | mean motor command ( us ) |

**For the analysis ( reviewer, 28 Sep ):**

- The INA219 bus register reads VIN- to ground, most likely on the load side of the 20 mOhm shunt, so a step
  dVm / dIs includes the shunt. That is the same path the firmware compensates ( task 1's 100 mOhm was also measured on
  the bus ), so keep R on this basis and state the shunt's 20 mOhm share alongside it.
- The worst-case line is ~127 B/tick once `t` reaches 7 digits ( ~16.7 min after power-on ). If the app disconnects
  late in the log, shorten a tag ( ` Arm:` → ` A:` ).

**Procedure.**

1. Flash the 28 Sep 15:24 build. Power-cycle after the flash.
2. Charge the pack full; note the charger's final voltage. **Rest it 10 min.**
3. Plug in, connect the app, start PlutoMonitor, **then** Dev Mode on. The first line must be `E:… Cap:… Cells:…`.
4. **20 s disarmed on the ground**, untouched ( resting voltage at full ).
5. Arm, take off, hover low ( ~1 m ), ALT_HOLD is fine. **Gentle inputs, no hard climbs** ( one R020: 8 A range ).
6. *Optional, worth it:* at about half the flight ( ~4 min ), land, disarm, **20 s** untouched, re-arm and carry on.
   That gives a third resistance point in the middle of the discharge.
7. Keep flying past the app's low-battery warning ( note the time and the app's remaining mAh when it comes ).
   Land when the drone **starts to struggle to hold height** ( log-6 ended at 2.8 V under load, motors ~75% );
   do not run it into the ground.
8. Land, disarm, **Dev Mode stays on, drone untouched for 60 s** ( 120 s if you can ). Do not unplug or move it.
9. Stop PlutoMonitor; save everything into [logs/log-1.txt](logs/log-1.txt).
10. Touch-check the shunt area ( warm is fine, too hot to touch is not ).
11. Charge the pack; note the charger's **mAh** and that the charge **finished** ( the check topic's 430 vs 534 mAh
    mismatch may be a charge stopped early ).

Note anything the log cannot show: wind, bumps, a visible sink, the app's warning time and remaining figure.

**What it should show.**

| Quantity | From | Prediction | If not |
|---|---|---|---|
| R, arming step | `Vm` rest before arming vs first settled second in hover, over `Is` step | ~75-100 mOhm fast step | |
| R, steady state | `Vm` in hover vs the standard curve at the charge left ( as INVESTIGATION.md §5.2 ) | **~100 mOhm**, flat over the discharge | > ~120 or < ~80: the floor needs per-pack R, or the count must carry the 15% warning ( task 5 ) |
| R, landing step and recovery | `Vm` over the first 1 s and the 60 s after disarm | ~115 mOhm fast, rising to ~150+ over 60 s | |
| Resting voltage 60 s after landing | `Vm` | 3.55-3.65 V ( curve: 4-7% left ) | lower: the curve's empty end is too optimistic |
| Charge | `Is` integral vs charger | ~1.04 of the charger ( log-6 ); ~530-600 mAh | outside 0.95-1.10: the INA219 gain needs a look in task 3 |
| Hover current | `Is` | ~4.1-4.4 A, rising as the pack sags | |
| Today's warning | `S` ≤ 18 | ~15 s before landing ( log-6 ) | the baseline task 5 must beat |
| `V` against `Vm` | | `V` = `Vm` floored, 0-100 mV below | a larger gap: an offset in the bus path |

## log-1: full to empty on a newer 600, rest after landing ( 28 Sep 2026, PRIMUS_X2_v1, flight )

[logs/log-1.txt](logs/log-1.txt), 16:23-16:32, 15:24 build, newer 600 pack charged full, app capacity 600, one R020.
5223 records at 10 Hz; 92 one-tick gaps ( Wi-Fi ), one truncated record ( t 183.6 s ), no `Vm` read errors.

| Segment | `t` | Duration | `Vm` | `Is` | Notes |
|---|---|---|---|---|---|
| Ground | 0-8.2 s | 8 s | 4165 mV | 127 mA | `E` 550 ( code 4.1 ), `Cap` 600, `Cells` 1 |
| **Flight** | 8.3-455.0 s | **447 s** | 3.62 → 2.94 V | 4.1 → 4.3 A, max 5.0 A | `M` median 1690; no mid-flight landing |
| Ground after | 455.0-548.6 s | 94 s | 3.47 → 3.56 V | ~150 mA | two Dev Mode restarts ( gaps of 1.9 s and 6.9 s at +19-26 s ) |

Charge to landing: **`Is` integral 528.4 mAh** ( 532.6 to the end of the log ); firmware `I` integral 525.0, `D` 495 at
landing ( `Is` / `I` = 1.013, the two round-downs; `D` / `Is` = 0.937 with the 0.95 gain ).

**Charger ( user, 28 Sep ): 502 mAh put back, at 1.1 A.**

| Comparison | task 13 pack | log-6 pack ( check topic ) |
|---|---|---|
| Raw INA219 integral / charger | 528.4 / 502 = **1.053** ( 1.061 to the end of the log ) | 556.1 x 1.013 = 563 / 534 = **1.055** ( `I` scaled to `Is` ) |
| Firmware `D` / charger | 495 / 502 = 0.986 | 528 / 534 = 0.99 |
| Capacity, full to 0% on the curve | ~525 mAh ( charger scale ), ~567 ( INA219 scale ) | - |

- **The raw INA219 current reads ~5.4% above the charger on both packs**, the same within 0.2%. A LiPo gives back almost
  all the charge put in, so the two should agree; one of them is off by ~5%. A 20 mOhm shunt reads 5% high if ~1 mOhm
  of copper sits inside its sense connection, or if the part is ~5% low ( rated 1% ); the charger's own accuracy is
  unknown. Task 15 measures it against a reference meter.
- `D` lands within 1-1.5% of the charger only because the auto-gain's -5% cancels this. Task 3 removes the gain, so
  without a calibration `D` would read ~5.5% high, just outside the ±5% target.

Noise in a steady 10 s of hover: `Vm` ±16 mV, `Is` ±106 mA ( single samples, PWM ripple ), `I` ±36 mA ( averaged ).
`V` sits 59-65 mV below `Vm` at rest: the 0.1 V floor, no offset in the bus path.

### Resistance ( bus basis: pack + connector + wiring + the 20 mOhm shunt )

| Measure | Values | R |
|---|---|---|
| Arming step, +0.6-1 s | 4165 mV / 127 mA → 3654-3677 mV / 4092-4107 mA | 123-128 mOhm |
| Arming → +31 s, less the curve's own drop ( 97.5 → 90.6%, 60 mV ) | → 3498 mV / 4171 mA | **150 mOhm** |
| Steady state, `Vm` against the standard curve, 17 windows over the flight | start 97.5% ( rest 4.18 V ), end 4.3% ( rest 3.56 V at 60 s ) | **137-152, mean ~146 mOhm**, flat to the end |
| Landing step, +0.2 s | ~2950 mV / 4300 mA → 3473 mV / 151 mA | 126 mOhm |
| Landing, +60 s ( recovery settled ) | → 3561 mV | 147 mOhm |

Recovery after landing: 3473 ( +0.2 s ), 3491 ( +1 ), 3503 ( +2 ), 3525 ( +5 ), 3543 ( +10 ), 3552 ( +15 ), 3559 ( +30 ),
3561 ( +60 ), 3556 ( +90, 145 mA idle ). **Settled by ~30 s.** The fast step is ~20 mOhm below the settled value: the
last ~90 mV is polarization.

**This pack is ~146 mOhm; the log-6 pack was ~100 mOhm** ( same method, same bus basis ). The standard curve fits both
with one R each: the curve is right, and R is the per-pack quantity. The pack held ~567 mAh from full to 0% on the curve
( 528 flown, ~4% left at landing ).

### Where each warning rule would have fired ( true charge left, curve-anchored )

| Rule | Warning ( 15% ) | Critical ( 5% ) |
|---|---|---|
| **Today's firmware** ( fused `S` ≤ 18 / ≤ 8 ) | t 402 s, 53 s before landing, **15.5%** | at landing, 4.3% |
| Voltage floor, R = 100 mOhm ( task 1 design ) | t 163 s, 292 s before landing, **66%** | 50% |
| Voltage floor, R = 126 mOhm ( fast step ) | 50% | 9.3% |
| Voltage floor, R = 146 mOhm ( this pack's own ) | t 421 s, 34 s before landing, **11.5%** | 4.7% |
| Count against the rated 600, start from the curve ( 585 mAh ) | t 427 s, **10.2%** | never: 58 mAh shown at landing, ~24 real |

- The fixed-R voltage floor from task 1 would have warned with two thirds of the pack left: the §5.4 sensitivity,
  measured. A single R cannot serve both packs.
- Count-led on the rated capacity is late on this pack because it holds ~567 mAh against 600 ( ~5% short ), and it
  never reaches critical.
- With the pack's own R the floor lands near the target ( 11.5% / 4.7% ). **R from the first ~30 s of each flight**
  ( rest `Vm` before arming against hover `Vm` at ~30 s, less the curve's drop for the charge used ) gives 150 mOhm here
  and ~105 on log-6, matching each pack's steady-state value. That makes a per-flight R a practical option for task 5.
- Today's firmware warned on time on this pack ( 15.5% ) but late on log-6 ( ~3% ): its timing depends on the pack.

**Not covered:** the mid-flight landing ( optional ) was not flown, so there is no mid-discharge step. There was 8 s on
the ground before arming, not 20; `Vm` was steady ( 4165 mV ) so the rest reading holds.

## Task 14 plan: 2-3 more packs, full to empty ( log-2 .. log-4 )

**Why.** Pick how task 5 gets the pack resistance for the remaining-mAh figure and the warnings: one fixed average,
per flight from the first ~30 s, or re-estimated through the flight. Every method is replayed offline on every log, so
the flights need no new code: same **28 Sep 15:24 build**, same log line as task 13.

**Packs** ( one log each; the more they differ, the better the survey ):

| Log | Pack | App capacity |
|---|---|---|
| [log-2](logs/log-2.txt) | an **older 600** ( the log-6 pack is not identified; an older pack is the closest match to it ) | 600 |
| [log-3](logs/log-3.txt) | **another newer 600** | 600 |
| [log-4](logs/log-4.txt) | an **800** ( optional ) | **800** |

**Procedure** = task 13's, with these changes:

1. **Label every pack you fly** ( A, B, C … on tape ); the task 13 pack is the first one to mark if you still know it.
   Before each flight note the pack label and the **room temperature** ( R rises in the cold ). The same pack flown
   again later ( e.g. in task 9 ) answers whether R is stable day to day.
2. After plugging in and turning on Dev Mode, **20 s untouched on the ground** before arming.
3. ~~Hold at idle ~3 s after arming~~: dropped, the app takes off on arming ( log-2 ).
4. Hover to empty as before; land when it struggles to hold height.
5. **60 s untouched after landing**, Dev Mode on.
6. Save the log; charge the pack and note the **charger mAh**.

Hand back per flight: the log file, pack, capacity setting, room temperature, charger mAh, **the charger's IR
reading for that pack**, and the app's remaining
mAh and warning time if you saw them.

**What the replay gives per pack:** R ( arming step, idle step, first 30 s, steady state, 60 s recovery ), the real
capacity ( `Is` integral and charger ), and for each R method the remaining-mAh error over the flight and where the
15% / 5% warnings would fire.

## log-2: task 14, an older 600, full to empty ( 28 Sep 2026, PRIMUS_X2_v1, flight )

[logs/log-2.txt](logs/log-2.txt), 18:41-18:49, 15:24 build, an older 600 ( label and room temperature not noted ), app
capacity 600. 4685 records, no restarts, no `Vm` errors, 99 one-tick gaps.

| Segment | `t` | Duration | `Vm` | `Is` |
|---|---|---|---|---|
| Ground | 0-26.8 s | 27 s | 4158 mV | 125 mA |
| **Flight** | 26.9-433.7 s | **407 s** | 3.54 → 2.89 V | 4.15 → 4.39 A, `M` median 1691 |
| Ground after | 433.8-486.2 s | 52 s | 3.52 → 3.65 V | ~145 mA |

- **No idle step:** the first armed record already has `M` 1522 and 4.8 A. The app takes off on arming, so the plan's
  3 s idle hold cannot be flown; dropped from the plan.
- Landed earlier than log-1: 3.65 V at rest after 52 s = **9% left** on the curve ( log-1: 4.3% ).

**Charge.** `Is` integral 479.8 mAh to landing ( 482.0 to the end ). **Charger ( user ): 420 mAh at 1.2 A** → `Is` /
charger = **1.142**, against 1.053 and 1.055 on the other two packs. The INA219 cannot change gain between flights on
the same board, and the curve anchors ( start 97.3% at 4158 mV, end 9.0% ) put this pack at ~543 mAh full to 0% on the
INA219 scale, close to log-1's 567. **So the 420 is the odd one out**: most likely the charge ended before full ( an
older, higher-R pack reaches the charger's cut-off current sooner ) or it was read off early. The check topic's 430
after log-1 was the same kind of low outlier. The charger's mAh is therefore not a reliable reference on its own:
task 15 ( meter ) matters.

**Resistance ( bus basis ):**

| Measure | R |
|---|---|
| First 30 s ( rest 4158 mV / 125 mA → 3428 mV / 4159 mA, less 57 mV curve drop ) | **167 mOhm** |
| Steady state, 13 windows against the curve | **148-163, mean ~157** ( 180 in the last window, 9% left ) |
| Landing +0.2 s / +10 s / +45 s | 151 / 176 / 182 mOhm |
| Regression on the in-flight ripple | 39 → 117 over the flight: not usable |

Recovery after landing: 3524 ( +0.2 s ), 3573 ( +1 ), 3612 ( +5 ), 3631 ( +10 ), 3643 ( +20 ), 3652 ( +30 ), 3655 ( +45 ).

**Today's warning:** `S` ≤ 18 at 22% true left, 59 s before landing; `S` ≤ 8 at landing ( 9% ).

### Warning replay, log-1 and log-2 ( curve-anchored true charge left; seconds before landing )

| R used for `Vcomp` | log-1 ( newer 600 ): warning / critical | log-2 ( older 600 ): warning / critical |
|---|---|---|
| 100 mOhm fixed ( task 1 ) | 66% ( 292 s ) / 50% | 76% ( 308 s ) / 54% |
| 135 mOhm fixed ( average so far ) | 39% ( 163 s ) / 6.4% ( 10 s ) | 43% ( 154 s ) / 18% ( 42 s ) |
| **per flight, first 30 s** ( 150 / 167 ) | **10.4% ( 29 s ) / 4.5% ( 1 s )** | **17.2% ( 37 s )** / not reached ( landed at 9% ) |

Three packs now read ~100 ( log-6 ), ~146 and ~157 mOhm. A fixed average warns at ~40% left on both packs here; the
per-flight R lands both warnings at 10-17%.

**Charger IR ( user, 28 Sep ): 100-124 mOhm** across the packs ( per-pack values not yet matched ). The charger pulses
the pack briefly at its own leads, so it sees the ohmic part only. The drone's fast step at landing, less the 20 mOhm
shunt, is ~106 ( log-1 ) and ~131 mOhm ( log-2 ), in that range; steady hover adds ~25-40 mOhm of polarization, and that
is what the warning voltage needs. log-6's ~100 mOhm on the bus ( a ~60-80 mOhm pack ) does not fit and came from
0.1 V codes only: weakest of the three.

## Task 5: offline replay of the new rule ( 28 Sep 2026 )

The task 5 rule as built in `battery.cpp` ( per-flight R from 25-35 s of loaded flight, `Vcomp` EMA 1 s, warning at
3.745 V/cell or 15% count, critical at 3.60 V or 5%, 1.5 s debounce, voltage pull-down of the remaining below 25% ),
replayed in Python over the two mV logs ( `Vm` / `Is` smoothed over 1 s to match the firmware's 50-sample averages; the
count from the `Is` integral, no gain; true charge left curve-anchored as in the log-1 / log-2 sections ).

| | log-1 ( newer 600 ) | log-2 ( older 600 ) |
|---|---|---|
| E at plug-in ( capacity 600 ) | 589 | 585 |
| R measured ( 35 s after arming ) | 151 mOhm | 168 mOhm |
| **Warning** | **16.5% left**, 58 s before landing, 3.10 V loaded, by voltage | **17.7% left**, 40 s before landing, 3.04 V loaded, by voltage |
| **Critical** | 4.3% left, 2 s after landing ( condition from ~1 s before landing, 1.5 s debounce ) | not reached ( landed at 9% ) |
| **App remaining at landing** | **29 mAh ( 5% )** vs ~24 real; the count alone would show 60 | 23 mAh ( 4% ) vs ~49 real ( reads low: the safe side ) |

Both warnings come with ≥ 15% really left; the empty pack shows ≤ 5% of capacity. Script: `replay5.py` in the session
scratchpad.

### Replay after the review fixes ( 29 Sep 2026 )

Rule as revised: default R 100 mOhm until measured ( alarms it raises are provisional ), the R window accumulates over
armings with the rest point frozen at the first arming ( capped at 4180 mV/cell ), voltage checks and the pull-down only
in loaded flight ( ≥ 1500 mA ), the pull-down only with a measured R, warnings latched until power-off. Script
`replay5b.py`; the "power-up at ~50%" rows put 3 s of synthetic rest ( the curve voltage for the true charge left, less
15 mV ) in front of the rest of the flight.

| Scenario | R measured | Events ( true charge left ) | App remaining at landing |
|---|---|---|---|
| log-1 full ( newer 600 ) | 151 mOhm at 43 s | **warning 16.5%**, 58 s before landing | 29 mAh ( 5% ) vs ~24 real |
| log-2 full ( older 600 ) | 169 mOhm at 62 s | **warning 17.7%**, 39 s before landing | 24 mAh ( 4% ) vs ~49 real |
| log-1, power-up at ~51% | 149 mOhm at 38 s | provisional warning at 5 s and critical at 30 s, **both cleared at 38 s**; warning 16.7% | 29 mAh ( 5% ) |
| log-2, power-up at ~53% | 155 mOhm at 38 s | provisional warning / critical cleared at 38 s; warning 24.5%, critical 10.2% ( early side ) | 0 mAh ( landed at 9% ) |
| log-1 full, **capacity set 800** on this ~567 mAh pack | 153 mOhm | warning **11.0%** by voltage ( the count alone would never warn ) | 43 mAh ( 5% ) vs ~24 real |

- The provisional alarms beep for up to ~35 s after a power-up on a part-used pack, then clear: the price of a voltage
  check from takeoff ( user decision, 29 Sep ).
- Near 15% this pack's loaded voltage is very flat ( 3.10-3.12 V from 20% to 10% ): a 10 mV change in `Vcomp` moves the
  warning by ~25 s ( 16.5% vs 11% between the 600 and 800 runs, R 151 vs 153 mOhm ). Task 9 shows how much it varies.
- After landing, critical cannot fire any more ( loaded flight only ): on log-1 the pilot landed ~1 s after the
  critical condition began, inside the 1.5 s debounce.

### Replay after review round 3 ( 29 Sep 2026 )

The default-R voltage check now needs the count at ≤ 40% and its alarms re-level ( script `replay5c.py` ).

| Scenario | Events ( true charge left ) |
|---|---|
| log-1 / log-2 full | warning at **16.5% / 17.7%** ( unchanged ) |
| power-up at ~51-53% | **no provisional alarm**; warning at 16.7% / 24.5% |
| power-up at ~32% ( log-1 ) | provisional warning 5 s and critical 9 s → re-levelled to **OK** at 38 s ( R 144 ); warning 20.3%, critical 4.5% |
| power-up at ~38% ( log-2 ) | provisional warning / critical → **OK** at 38 s ( R 153 ); warning 27.3%, critical 10.8% |
| log-1 full, capacity 800 on a ~567 mAh pack | warning 11.0% by voltage; app 43 mAh at landing ( ~24 real ) |

## Task 8 plan: bench supply sweep ( log-5; first planned as log-3, which became a flight )

**Build:** `Build/PRIMUS_X2_v1/DEFAULT_PRIMUS_X2_v1_3.10.0.hex`, **29 Sep 14:33**: all topic changes ( tasks 2-6, 16 ) plus
the **bench motor sequence ON**. **PROPS OFF.** With Developer Mode on and the drone disarmed, the motors run idle, 25, 50,
75 and 100% for 10 s each, then stop. Never arm this build. The task 16 / task 9 flights need a rebuild with the sequence
off ( same file name ). At 3.5 V and below `L` is 2 while disarmed, so the app shows "LOW BATTERY" and greys out its ARM
switch; Dev Mode still works ( expected, and a guard on the bench ).

**Log line** ( one per 100 ms tick, ~125 B ): first tick after Dev Mode on `E:… Cap:… Cells:…`, then
`t Ph V Vc I R D S L M Arm`.

| Field | Meaning |
|---|---|
| `E`, `Cap`, `Cells` | plug-in estimate ( mAh ), configured capacity, cell count |
| `Ph` | bench motor phase: 0 idle, 1-4 = 25-100%, -1 when done |
| `V` | battery voltage ( mV, exact, 1 s average ) |
| `Vc` | compensated voltage ( mV ): V + I x 100 mOhm on the bench ( R not measured ) |
| `I` | battery current ( mA, 1 s average ) |
| `R` | pack resistance ( 0 on the bench ) |
| `D` | mAh used |
| `S` | SoC ( % ) |
| `L` | warning level: 0 OK, 1 low battery, 2 critical |
| `M` | mean motor command ( us ) |

**Procedure,** app capacity set to **600** first. For each supply voltage **4.3, 4.2, 3.8, 3.7, 3.5, 3.2, 3.0, 2.9 V**:

1. Set the supply; power the drone up **fresh** ( off, then on at that voltage ).
2. Connect the app, start PlutoMonitor, then **Dev Mode on**. The first line must be `E:… Cap:… Cells:…`.
3. The motor sequence runs ~50 s. Read the **supply's current display** at idle ( first 10 s ) and at 100% ( last 10 s );
   note both with the supply voltage.
4. Keep logging ~10 s after the motors stop; note whether the **beeper** sounds ( low battery / critical pattern ).
5. Dev Mode off, power off, next voltage. Keep appending to the same PlutoMonitor session.

Save everything into [logs/log-5.txt](logs/log-5.txt) and send the notes: per voltage, the supply current at idle
and at 100%, and the beeper.

**What it should show** ( the bus reads ~20-100 mV below the supply on the leads, so judge `E` against the logged `V` ):

| Supply | `E` ( C = 600 ) | `Cells` | `L` after ~2 s | `S` |
|---|---|---|---|---|
| 4.3 V | 600 | **1** ( was 2 ) | 0 | 100 |
| 4.2 V | ~590-600 | **1** | 0 | ~99-100 |
| 3.8 V | ~240 | 1 | 0 | ~40 |
| 3.7 V | ~75 ( ~12% ) | 1 | **1**, count ≤ 15% | ~12 |
| 3.5 V | ~20 ( ~3% ) | 1 | **2**, count ≤ 5% | ~3 |
| 3.2 / 3.0 / 2.9 V | **0**, no wrap ( was 65436 at 2.9 V ) | 1 | **2** | **0, no jump** ( was 54-58% ) |

- `E` = 600 x the §5.5 curve at `V` + `I` x 120 mOhm, within ±5%; the same `E` on a repeat power-up at the same voltage.
- On the ground only the count raises the level ( the voltage check runs in flight only ): so `L` follows `E`, and once
  set it stays set until power-off, including through the motor sequence.
- `I` at idle within ~10 mA of the supply's display; at 100% within ~50 mA ( ~0.5 A props off ).
- `S` never rises after it settles; nothing jumps at 3.0 / 2.9 V.



## log-3: task 9, an 800 pack, full to critical ( 29 Sep 2026, PRIMUS_X2_v1 3.10.0, flight )

Flown instead of the task 8 bench sweep, on the bench build ( `BENCH_MOTOR_SEQUENCE 1` ): the sequence aborted on
arming ( `Ph` 0 -> -1 ), as designed. Capacity 800 in the app. Charger afterwards: **711 mAh** at 1 A ( user ).

| | |
|---|---|
| Plug-in | `E` 791 of 800, `Cells` 1, `V` 4170 mV at 134 mA |
| Flight | armed 590.6 s, hover ~4.48 A mean ( max 4.77 A ), min loaded `V` 3074 mV |
| `R` measured | 108 mOhm, 35 s after arming ( `D` 43 ) |
| Warning ( `L` 1 ) | `Vc` 3743 mV ( voltage rule, 3.745 V ); `D` 664, `S` 15; 56.7 s before disarm. The count rule ( 15% ) would have fired 7 s later ( `D` 671, `Vc` 3738 ) |
| Critical ( `L` 2 ) | `Vc` 3588 mV ( voltage rule, 3.60 V ); `D` 735, `S` 4 ( count alone 7%: the voltage pull-down took it to 4 ); **0.2 s before disarm** |
| Rest | 3539 mV at 149 mA, 60 s after disarm ( ~4% on the rest curve, ~30 mAh ) |
| End | `D` 739, `S` 3, `L` 2 |

**Done-when, this pack:**

| Criterion | Result | |
|---|---|---|
| `D` within ±5% of the charger | 739 vs 711 = **+3.9%** | pass |
| Remaining at empty ≤ 5% of capacity | `S` 4% at critical, 3% at rest ( ~32 / 24 mAh ) | pass ( firmware value; app display not yet confirmed ) |
| Warning with ≥ 15% really left | ~103 mAh left = **12.9% of 800** ( 13.9% of the ~742 mAh the pack really held ) | **fail, marginal** |

"Really left" = charger 711 + ~31 mAh left at rest = ~742 mAh delivered from full; used at the warning = 664 x 711 / 739
= ~639 mAh on the charger's scale. The pack delivers ~7% under its 800 rating and the counter reads 3.9% high, so a
count at 15.9% of rated is ~13% real. Both rules agreed within 7 s, so moving one alone does not fix it.

**Open: the disarm.** The motors went from ~1710 us ( hover, 4.6 A ) to 1000 and `Arm` 0 within 190 ms of critical.
No firmware path disarms on the battery level ( failsafe RX loss, crash, low-throttle auto-disarm, app command, arm
switch only ). A human reacting to the beeper is unlikely that fast: most likely the app disarms on the critical flag,
or it was a user disarm / touchdown. To confirm with the user.

**Answered ( user, 29 Sep ): the app disarms on critical.** It switches its ARM off on the `LowBattery_inFlight` flight
status ( `App_LowBattery_inFlight`, 8 ). Task 16 replaces that with a firmware landing.

## Task 16 plan: auto-land at critical, on a 600 pack ( log-4; also a task 9 flight )

Build: `Build/PRIMUS_X2_v1/DEFAULT_PRIMUS_X2_v1_3.10.0.hex`, **29 Sep 15:29**, `BENCH_MOTOR_SEQUENCE 0` ( rebuilt after
the task 8 bench sweep; the 14:33 hex was the bench build ), ( Dev Mode no
longer spins the motors ). Log line: `t V Vc I R D S L M Ld Arm`, where `Ld` = 1 while the LAND command runs.

**Confirms:** `Ld` goes 0 → 1 within a tick of `L` 2; `M` falls from hover; `Arm` 1 → 0 by itself seconds later ( not
0.2 s ) with `Ld` 1 → 0; the app shows "LOW BATTERY in-flight" during the descent and "LOW BATTERY" after, with
arming refused. **Refutes:** `Arm` 0 within ~0.5 s of `L` 2 with `Ld` 0 ( the app still disarms ); `Ld` 1 for ~30 s
( touchdown not detected, the timeout disarmed ); the craft climbs or holds height at `L` 2.

**Procedure,** a newer or older 600 pack ( labelled ), app capacity **600**:

1. Full pack, rested ~10 min. Power up, connect the app, PlutoMonitor, **Dev Mode on**; first line `E:… Cap:… Cells:…`.
2. Arm and take off in ALT_HOLD as usual. Hover **low ( ~1 m ) over a soft, open area**, pitch / roll only, until the
   low-battery warning; keep hovering. Keep a thumb near the app's disarm in case the descent goes wrong.
3. At critical ( critical beeper ), hands off the throttle: the drone should descend by itself. During the descent give
   one small roll or pitch input ( it should respond ) and one brief throttle-up ( it should be ignored ).
4. After touchdown it should disarm by itself. Check the app status text, then try to arm: the app should refuse.
5. Keep logging ~60 s on the ground, Dev Mode off, power off. Charge and note the charger mAh.

Save into [logs/log-4.txt](logs/log-4.txt), and send: the pack label, the charger mAh, the descent time roughly, whether
steering worked and throttle was ignored, how hard the touchdown was, the app's status text during and after, and
whether re-arming was refused. The bench sweep ( task 8 ) moves to log-5 and needs `BENCH_MOTOR_SEQUENCE 1` again.

**Second flight, the 800 pack ( 30 Sep ): logs/log-6.txt.** Same build and procedure, app capacity **800**. A second
auto-land check on another pack, and a repeat of log-3's warning margin ( ~12.9% really left ) with the charger.

## log-5: task 8, bench supply sweep ( 29 Sep 2026 15:00-15:24, PRIMUS_X2_v1, bench build 14:33, props off )

App capacity was **800** ( not the plan's 600 ): expectations scaled. A fresh power-up per voltage, Dev Mode on, the
motor sequence ran each time ( `Ph` 0-4, `Arm` 0 throughout, `R` 0 ). Medians over each phase ( first 2 s dropped ).

| Supply | `E` | expected `E` ( curve at `V` + `I` x 120 mOhm ) | `Cells` | `L` | `S` | `I` idle / 100% ( mA ) | `V` idle / 100% ( mV ) |
|---|---|---|---|---|---|---|---|
| 4.3 V | 800 | 800 | **1** | 0 | 99 → 98 | 131 / 555 | 4266 / 4185 |
| 4.2 V | 786 | 792 | 1 | 0 | 98 → 97 | 133 / 538 | 4174 / 4101 |
| 3.8 V | 292 | 274 | 1 | 0 | 36 → 35 | 148 / 511 | 3769 / 3707 |
| 3.7 V | 72 | 77 | 1 | **1** ( count 9% ) | 8 → 7 | 153 / 513 | 3665 / 3604 |
| 3.5 V | 26 | 26 | 1 | **2** ( count 3% ) | 3 → 1 | 158 / 500 | 3474 / 3414 |
| 3.2 V | **0** | 0 | 1 | 2 | 0 | 182 / 497 | 3159 / 3107 |
| 3.0 V | **0** | 0 | 1 | 2 | 0 | 198 / 486 | 2963 / 2911 |
| 2.9 V | **0** | 0 | 1 | 2 | 0 | 207 / 491 | 2857 / 2808 |

- **`E` from the curve: pass.** Within ±1% everywhere except 3.8 V ( +18 mAh, +6.6% ). That is 7 mV on the curve's
  steepest segment ( 5% per 20 mV at 3.77-3.79 V ); the plug-in sample is taken ~0.5 s after power-up, before the logged
  idle, so a few mV of difference is expected. Not a fault.
- **`Cells` 1 at every voltage: pass** ( 4.3 V read 2 before the fix ).
- **No wrap and no SoC jump at 3.2 / 3.0 / 2.9 V: pass.** `E` 0 ( was 65436 at 2.9 V ), `S` 0 throughout ( was 54-58% ).
- **`L` follows the count: pass.** 0 at 4.3-3.8 V, low battery at 3.7 V ( 9% ≤ 15% ), critical from 3.5 V ( ≤ 5% ),
  raised on the first tick and held through the motor sequence. `S` never rose in any section.
- **`I` vs the supply's ammeter: pass** ( user, 30 Sep: the readings were proper; not recorded per voltage ). The board's idle draw rises as the voltage falls ( 131 → 207 mA, ~0.56-0.59 W ): a constant-power load, as
  expected from the regulators.
- The bus reads ~50-80 mV lower under the ~0.5 A motor load than at idle ( lead and connector drop ), consistent with
  `Vc` adding `I` x 100 mOhm on the bench.

## log-4: task 16 auto-land + task 9, a 600 pack, full to critical ( 29-30 Sep 2026, PRIMUS_X2_v1 15:29 flight build )

User: "all ok" ( landed and disarmed by itself ). **Charger: 449 mAh at 1.2 A** ( user, 30 Sep ). An older 600 ( user, 1 Oct ).

| | |
|---|---|
| Plug-in | `E` 587 of 600, `Cells` 1, `V` 4161 mV at 135 mA ( curve: ~585 ) |
| Flight | armed 412 s, hover ~4.25 A mean |
| `R` measured | **173 mOhm**, 35 s after arming ( `D` 41 ); the highest pack R so far ( others 100-157 ) |
| Warning ( `L` 1 ) | `Vc` 3742 mV ( voltage rule ), `V` 2990 mV loaded, `D` 423, `S` 22 ( count 27%, pulled down ); 50.6 s before critical |
| Critical ( `L` 2 ) | `Vc` 3587 mV ( voltage rule ), `V` 2820 mV loaded, `D` 484, count 17%, `S` 1 ( pull-down ) |
| **Auto-land** | `Ld` 0 → 1 **in the same tick** as `L` 2; `Arm` 1 → 0 and `Ld` 1 → 0 together **2.84 s later** ( touchdown disarm ) |
| Rest | 3653 mV at 150 mA, 90 s after landing ( ~7.7% on the rest curve ) |
| End | `D` 491, `S` 0, `L` 2 |

**Task 16 checks:**

- **Landing started by the firmware at critical: pass.** `Ld` 1 in the `L` 2 tick; no disarm at 0.2 s as in log-3, so the
  app did not cut the motors ( it saw `App_Low_battery` while armed ).
- **Touchdown disarm: pass.** 2.84 s = the earliest the rule allows ( ramp 1300 → 1200 in 2.5 s, then 0.3 s settle ): the
  estimator saw the descent stop by 2.5 s, as expected from ~1 m at 50-75 cm/s. `M` stayed at hover level ( ~1750 us,
  ~4.2 A ) through the descent: in ALT_HOLD `landThrottle` is a descent rate, and a steady descent needs ~hover thrust.
- **Steering, throttle ignored, app status, re-arm refused:** user "all ok"; confirm the details.
- **Energy:** critical left ~8% by the rest curve ( ~45 mAh ); the landing used ~3 mAh.

**Pack notes:** R 173 mOhm sags the bus to 2.82 V at critical in hover ( `Vc` 3.59 V ). The voltage rule fired both levels
while the count still said 27% / 17%: on this pack the count alone would have warned ~60 s late. `D` 491 of 600 rated.

**Task 9 done-when, this pack** ( charger 449 mAh; ~37 mAh left at rest by the curve, so ~486 mAh delivered from full ):

| Criterion | Result | |
|---|---|---|
| `D` within ±5% of the charger | 491 vs 449 = **+9.4%** | **fail** |
| Remaining at empty ≤ 5% of capacity | `S` 1% at critical, 0% at rest | pass |
| Warning with ≥ 15% really left | ~100 mAh = **16.6% of 600** ( 20.5% of delivered ) | pass |

Critical came with ~44 mAh ( 7.3% of 600 ) really left. The counter reads above the charger on every pack so far:
+3.9% ( log-3, 800, 1 A charge ), +5.3 / +5.5% ( task 13 / 14 packs ), **+9.4%** here ( the worn 173 mOhm pack, 1.2 A ).
A gain error in the INA219 would give the same ratio on every pack; a spread of 4-9% points at the charger side
( termination before full on a high-R pack, and the pack not at 100% when flown: `E` 587 ) as well as, possibly, a
common ~4% counter offset. The count did not decide any warning here ( the voltage rule fired both ), and the pull-down
corrected the reported remaining.

## log-6: task 16 auto-land + task 9 repeat, the 800 pack, full to critical ( 1 Oct 2026, PRIMUS_X2_v1 15:29 flight build )

**Charger: 730 mAh at 1.2 A** ( user, 1 Oct ). `R` 109 mOhm, as log-3's 108: most likely the same pack.

| | |
|---|---|
| Plug-in | `E` 797 of 800, `Cells` 1, `V` 4180 mV at 131 mA |
| Flight | armed 607 s, hover ~4.51 A mean |
| `R` measured | 109 mOhm, 35 s after arming ( `D` 44 ) |
| Warning ( `L` 1 ) | `Vc` 3744 mV ( voltage rule ), `V` 3245 mV loaded, `D` 634, `S` 20 ( count 20.4% ); **98.5 s before critical** ( log-3: 57 s ) |
| Critical ( `L` 2 ) | `Vc` 3606 mV ( the 1.5 s debounced condition; the logged `Vc` is the 1 s EMA ), `D` 759, count 4.8%, `S` 4 |
| **Auto-land** | `Ld` 0 → 1 in the `L` 2 tick; `Arm` 1 → 0 with `Ld` 1 → 0 **2.03 s later** |
| Rest | 3591 mV at 151 mA, 60-90 s after landing ( ~4.7% on the rest curve, ~37 mAh ) |
| End | `D` 766, `S` 3, `L` 2 |

**The 2.03 s disarm is shorter than the arrested-descent rule allows** ( ramp to 1200 takes 2.5 s, + 0.3 s settle ), so it
was the firm-contact rule ( `|accADC [ 2 ]|` > ~1.46 G at touchdown ), the crash detector ( `failsafeOnCrash`, X/Y
acceleration in ANGLE mode; the app would show CRASHED ) or a disarm from the app. The log cannot tell them apart; `M`
was at hover level ( ~1750 us ) until the disarm tick. **User ( 1 Oct ): the app showed CRASHED**: the crash detector
disarmed it at touchdown ( the landing jolt, X/Y acceleration ). The Crash flag clears ~300 ms later once level
( `mw.cpp:483-495` ), and the flight status then shows the critical flag ( app: LOW BATTERY, arming blocked ). A touchdown
disarm either way; the app will show it as a low-battery auto-land ( app developer, later ).

**Against log-3 ( same pack ):** the warning came at `D` 634 instead of 664 and 98 s instead of 57 s before critical,
with the same R; critical at `D` 759 instead of 735. The charger reading decides the really-left margin.

**Task 9 done-when, this pack** ( charger 730 mAh; ~36 mAh left at rest by the curve, so ~766 mAh delivered from full ):

| Criterion | Result | |
|---|---|---|
| `D` within ±5% of the charger | 766 vs 730 = **+4.9%** | pass ( at the edge ) |
| Remaining at empty ≤ 5% of capacity | `S` 4% at critical, 3% at rest | pass |
| Warning with ≥ 15% really left | ~162 mAh = **20.2% of 800** ( 21.1% of delivered ) | pass ( log-3, same pack: 12.9% ) |

Critical came with ~43 mAh ( 5.3% of 800 ) really left. Counter / charger so far: 1.039 ( log-3 ), 1.049 ( log-6, same pack ),
1.053 / 1.055 ( task 13 / 14 ), 1.094 ( log-4, worn 600 ).

## log-7: task 9 + task 16, a second 600 pack ( 1 Oct 2026, PRIMUS_X2_v1 15:29 flight build )

**First attempt ( 11:50 ):** the pasted log was cut at 170 s ( armed, `D` 200 ); `R` 149 mOhm; charger after it 488 mAh at
1.2 A. Not usable for task 9. **Re-flown ( 12:41 ) on the same pack after that charge;** log-7.txt now holds the re-fly.
**Charger after the re-fly: 497 mAh at 1 A** ( user, 1 Oct ).

| | |
|---|---|
| Plug-in | `E` **560** of 600 ( 93% ), `Cells` 1, `V` 4119 mV at 129 mA: the pack was not at 100% at takeoff |
| Flight | armed 416 s, hover ~4.29 A mean |
| `R` measured | **146 mOhm**, 35 s after arming ( `D` 43 ); 149 on the first attempt: repeatable on a pack within a day |
| Warning ( `L` 1 ) | `Vc` 3744 mV ( voltage rule ), `V` 3106 mV loaded, `D` 411, `S` 22 ( count 24.8%, pulled down ); 68.8 s before critical |
| Critical ( `L` 2 ) | `Vc` 3589 mV ( voltage rule ), `V` 2938 mV loaded, `D` 494, count 11%, `S` 4 ( pull-down ) |
| **Auto-land** | `Ld` 0 → 1 in the `L` 2 tick; `Arm` 1 → 0 with `Ld` 1 → 0 **2.14 s later** ( below the 2.8 s arrested rule: impact or crash detect at touchdown, as log-6 ) |
| Rest | 3576 mV at ~150 mA, 30-80 s after landing ( ~4.5% on the rest curve, ~27 mAh ) |
| End | `D` 500, `S` 3, `L` 2 |

The charger refill will include the ~40 mAh missing before takeoff ( `E` 560 ): compare `D` against charger minus that
deficit.

**Task 9 done-when, this pack** ( charger 497 mAh; ~4.5% left at rest, so ~520 mAh from full ). The pack started at
`E` 560 ( 93% on the curve ), so two readings of the charger:

| Criterion | A: full at takeoff ( as the other flights ) | B: started at 93% | |
|---|---|---|---|
| `D` within ±5% of the charger | 500 vs 497 = **+0.6%** | 500 vs ~462 = **+8.2%** | pass ( A ) / fail ( B ) |
| Remaining at empty ≤ 5% | `S` 4% at critical, 3% at rest | same | pass |
| Warning with ≥ 15% really left | ~112 mAh = **18.6% of 600** | ~106 mAh = **17.6%** | pass |

Critical came with ~29 mAh ( 4.8% of 600 ) really left.

**Counter / charger across the topic, corrected for the charge missing at takeoff ( `E` below capacity ):**

| Flight | Pack | R ( mOhm ) | raw | corrected |
|---|---|---|---|---|
| task 13 / 14 | 600s | 146 / 157 | 1.053 / 1.055 | - |
| log-3 | 800 | 108 | 1.039 | **1.053** |
| log-6 | 800 ( same ) | 109 | 1.049 | **1.054** |
| log-7 | 600 | 146 | 1.006 | 1.082 |
| log-4 | 600, worn | 173 | 1.094 | 1.126 |

On the healthier packs the counter reads a steady **+5.3-5.5%** above the charger. The two high-R 600s read 8-13% above:
consistent with a charger ending its CV phase early on a high-R pack ( less goes back in ), on top of the same offset.
A steady 5.3% is larger than the ±100 mA ( ±2.3% at hover ) the INA219 was checked to ( task 15 ), so either the
charger reads ~5% low or the INA219 reads ~3-5% high at hover current; the check in task 15 may have been at low current.

## log-8: confirmation flight on the post-review build ( 3 Oct 2026, PRIMUS_X2_v1 17:18 build, a 600 pack )

Build with `batteryCriticalConfirmed ( )`: auto-land on a confirmed critical only, `mwArm ( )` refusing at a confirmed
critical. The user crashed on purpose during the flight to see what a normal user would get. **Charger: ~628 mAh,
on a different charger** from the other flights ( user, 3 Oct ).

| | |
|---|---|
| Plug-in | `E` 600 of 600, `Cells` 1, `V` 4184 mV at 133 mA |
| `R` measured | **101 mOhm**, 35 s after the first arming ( `D` 45 ); kept across the later armings |
| Armings | 4: disarmed at 92 s, 194 s and 529 s straight from hover ( `M` ~1500-1650 to 1000 in one tick: the crashes / user disarms ), re-armed each time |
| Warning ( `L` 1 ) | `Vc` 3744 mV ( voltage rule ), `D` 457, count 23.8%, `S` 22; armed |
| Re-arm with the warning on | allowed, as designed ( only critical refuses ): armed again at 10:09:35 with `L` 1 |
| Critical ( `L` 2 ) | `Vc` 3594 mV ( voltage rule, measured R: **confirmed** ), `D` 550, count 8.3%, `S` 1 |
| **Auto-land** | `Ld` 0 → 1 in the `L` 2 tick; disarmed **2.45 s** later on touchdown ( impact / crash detect, below the 2.8 s arrested rule ) |
| After landing | **no further arming** in the 94 s logged; `L` 2 latched |
| Rest | 3619 mV at ~155 mA, 60-90 s after landing ( ~5% on the rest curve ) |
| End | `D` 557, `S` 0 |

- **R across crash / re-arm cycles:** measured once in the first flight and kept, as designed ( once per power-up ).
- **`Ld` 1 again for 2.9 s while disarmed** ( 10:11:07.7-10:11:10.6 ): a `LAND` command arriving after the disarm. Once
  disarmed the firmware reports level 2, and the app's `setNewBatteryLevel ( )` sends its own `AutoLand ( )` LAND while
  its `isArmed` still lags. `land ( )` then runs disarmed, pins `rcData [ THROTTLE ]` to `landThrottle` ( motors stay
  off ) and ends through the arrested rule with `mwDisarm ( )`. Harmless, and the same as an app LAND sent while
  disarmed before this topic; it also means the app records its "Auto Land" entry after all.
- **Arming refusal:** the log cannot show a refused attempt; whether the app / RC tried and was refused is the user's
  observation.

**Charger result ( 628 mAh, different charger ).** The rest point after landing ( 3619 mV, ~5.6% on the curve ) puts
the pack at ~665 mAh from full by the charger, or ~594 mAh by the drone's own count ( 557 + ~37 left ):

| | by the charger ( 628 ) | by the drone's count ( 557 ) |
|---|---|---|
| Counter / charger | **0.887** ( count 11% below the charger ) | - |
| Really left at the warning ( `D` 457 ) | ~150 mAh = **25.0% of 600** | ~137 mAh = 22.8% |
| Really left at critical ( `D` 550 ) | ~45 mAh = **7.5%** | ~44 mAh = 7.3% |

Warning and critical pass either way. The counter result is the opposite sign to every flight on the first charger
( +3.9% to +9.4% ): the two chargers disagree by ~16% with each other, so neither is a precise reference for the count.
A meter in series at hover current would settle the counter's real error; not done ( accepted offset, task 9 ).
