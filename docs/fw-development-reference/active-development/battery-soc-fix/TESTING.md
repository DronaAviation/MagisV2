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

## Task 8 plan: bench supply sweep ( log-3 )

**Build:** `Build/PRIMUS_X2_v1/DEFAULT_PRIMUS_X2_v1_3.10.0.hex`, **29 Sep 01:14**: all topic changes ( tasks 2-6 ) plus the
**bench motor sequence ON**. **PROPS OFF.** With Developer Mode on and the drone disarmed, the motors run idle, 25, 50,
75 and 100% for 10 s each, then stop. Never arm this build. Task 9 gets a build with the sequence off.

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

Save everything into [logs/log-3.txt](logs/log-3.txt) and send the notes: per voltage, the supply current at idle
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

