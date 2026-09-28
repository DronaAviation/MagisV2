# Battery Capacity Estimate: Testing

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

## log-1: first log with the task 3 line ( 25 Sep 2026, PRIMUS_X2_v1, FW 3.10.0 + diagnostic )

[logs/log-1.txt](logs/log-1.txt), PlutoMonitor over Wi-Fi, 16:34:14 to 16:40:30. Planned as the 30 s task 3 bench
check; the user flew the pack to empty instead, so it also gives a first ( partial ) task 5 data set.

**Tooling note.** `python tools/flightlog.py` fails on import ( `tools/warnings.py` shadows the standard-library
`warnings` module that `statistics` imports ). Workaround: `python -c "import runpy,sys; sys.argv=['flightlog.py',
'summary','<log>']; runpy.run_path('tools/flightlog.py', run_name='__main__')"` from the repo root. Its summary also
reads the `t` field ( ms ) as seconds, so its gap and segment times are wrong for this log; the numbers below come
from a direct parse using `t`.

### Task 3 check: the log line

| Check | Result |
|---|---|
| Fields | All nine present in 3417-3418 of 3418 records ( first record partial ) |
| Rate | `t` step median 102 ms ( 100-102 ms ); 202 steps over 150 ms: 121 are the ticks spent printing `E` / `Cap`, the rest are Developer Mode drop-outs ( max 6.1 s ) |
| App connection | Continuous data for 375 s; no sign of an overrun ( no truncated or merged fields ) |
| `E` / `Cap` | `E` 500, `Cap` 600, printed alone on its tick as designed |
| Unexpected | `onLoopStart ( )` ran **121 times**, median every 2.9 s, with Developer Mode switched on the whole time ( user ). Cause: the firmware's RC-link timeout, see *Link timing* below. Outside this topic. |

### Link timing ( log-1 ): Developer Mode restarts

`plutoLoop ( )` runs only while `rxIsReceivingSignal ( )` is true ( `userCode ( )`, `mw.cpp:1111-1170` ). With
`Rx_ESP` that flag falls **200 ms after the last `MSP_SET_RAW_RC` frame** ( `DELAY_5_HZ`, `rx/rx.cpp:369` ). On
the drop the firmware runs `onLoopFinish ( )`, clears the user RC overrides and `isUserHeadingSet`, and re-runs
`onLoopStart ( )` when the next frame arrives ( `mw.cpp:1160-1169` ). Failsafe does not react ( `failsafe_delay`
1 s ). So every `E` / `Cap` line in this log marks one RC frame that arrived more than 200 ms after the previous.

| Measure | Value |
|---|---|
| Restarts | 121 in 375 s; interval min 1.4 s, median 2.85 s, max 12.4 s; 109 while armed ( 88% of the log is armed ) |
| Length of each drop ( drone-time gap around the marker ) | **113 of 120 are exactly 2 ticks ( 202-207 ms )**: user code was off for under one tick, i.e. a single late frame |
| Longer drops | 6 of 211-305 ms ( all armed, hovering, M 1706-1736 ), 1 of 6.1 s ( disarmed, at t+288 s ) |
| Gap without a restart | 1 ( 304 ms at t+286 s ): the main loop itself paused, not the link |
| Drone → app delivery | wall-clock step median 94 ms vs `t` step 102 ms; 5 of 3418 steps bunched ( < 30 ms ); wall-minus-`t` drift +1.3 s over 375 s, max excursion +0.35 s above the trend |
| Debug output rate | 114 B per 100 ms = 1.1 kB/s, ~10% of the 115200 baud UART |

**Reading.** The drone → app direction was smooth: no queuing, no bunching, a linear clock drift. The drops are
in the app → drone direction: about once every 3 s one RC frame is late by more than 200 ms, and nearly always
only one. Nothing in this log ties the drops to the drone's output rate ( the outages do not cluster with any
change in the line, and the line never changed ). Whether the rate matters at all is untested: an A/B with the
line halved or off, on the same app and phone, would show it. Until then the marker in `onLoopStart ( )` makes
the flicker visible in every log; the rule is now in `pluto-flighttest`.

**Effect on this topic:** none on the numbers ( the counter runs in `annexCode ( )`, not in user code ); one tick
of data replaced by `E` / `Cap` per restart. **Effect on user code in general:** anything set in
`onLoopStart ( )`, every `RcCommand_Set` override and the user heading are reset every few seconds on Wi-Fi.

### Flight segments

Pack resting at **4.0 V** at the start ( not full ): `E` = ( 4000 - 3000 ) x 600 / 1200 = **500**, exactly the
formula.

| Segment | Duration | V start → end ( 0.1 V steps ) | D start → end | `I` median / max | M median / max | G end |
|---|---|---|---|---|---|---|
| Ground | 8 s | 4.0 → 4.0 | 1 → 1 | 50 / 50 mA | 1000 | 1000 |
| **Flight 1** | **273 s** | 3.9 → 2.9 | 1 → 171 | **2250 / 2700 mA** | 1711 / 1971 | 965 |
| Ground ( rest ) | 5 s | 2.9 → 3.6 | 172 | | | |
| Hop 2 | 13 s | 3.5 → 2.9 | 172 → 180 | 2300 / 2550 | 1737 | 962 |
| Hop 3 | 5 s | 3.6 → 2.9 | 180 → 183 | 2300 / 2500 | 1727 | 961 |
| Hop 4 | 8 s | 3.5 → 2.8 | 183 → 188 | 2200 / 2400 | 1767 | 961 |
| Hop 5 | 26 s | 3.5 → 2.6 | 188 → 203 | 2150 / 2350 | 1778 | 959 |
| Ground ( end ) | 12 s | 2.6 → 3.5 | 203 | 50 | 1000 | 959 |

**The symptom is reproduced:** the app's remaining figure at the end is `E - D` = 500 - 203 = **297 mAh** on a
pack that sags to 2.6 V under load and rests at 3.5 V.

First flight in 30 s blocks ( `t` from arm ):

| t s | M median | I median mA | V median | G | D mAh |
|---|---|---|---|---|---|
| 0 | 1648 | 2200 | 3.3 | 1000 | 21 |
| 64 | 1704 | 2250 | 3.1 | 1000 | 60 |
| 129 | 1727 | 2250 | 3.1 | 1000 | 101 |
| 194 | 1735 | 2300 | 3.0 | 985 | 142 |
| 259 | 1773 | 2250 | 2.9 | 965 | 171 |

### What this log shows

1. **The counter is faithful to its own current reading.** Independent integral of `I` ( before gain ) over `t`:
   **204.8 mAh**; firmware `D` rose by **202 mAh**. The -1.4% difference is the gain ( 1000 → 959, learned only
   late in the flight ) plus the dt remainder. The code losses in this flight are **~-2 to -3%**, below the
   task 1 budget, because the auto-gain's `sag_obs >= 20` gate is rarely met at ~45 mV of shunt voltage.
2. **So the ~2x gap is in the current reading or in the pack**, nothing in between.
3. **The current reading is suspiciously flat:** 2200-2350 mA ( 44-47 shunt mV ) through the whole flight, and
   2200-2300 mA in every motor-command bin from 1500 to 1950. Over flight 1 the motor command rose 1648 → 1773
   and the voltage fell 3.3 → 2.9 V, which for constant thrust should raise battery current by roughly 12-15%;
   the reading rose ~5%. Suggestive of a measurement ceiling, not proof: high commands mostly happened late at
   low voltage, so command and voltage are confounded here.
4. **Plausibility:** 2.25 A at ~3.1 V is ~7 W to hover. The pack, starting at 4.0 V resting ( ~75% charge ), gave
   273 s of hover plus ~50 s of hops. If the reading were right, the pack held only ~200 mAh from 4.0 V to empty,
   i.e. ~270 mAh when full: a very worn 600 mAh pack.
5. **Disarmed current** reads 50 mA ( one step, 1 mV ); readings above that just after landing are the 50-sample
   ring average decaying.

### Charger readback and shunt part ( user, 25 Sep 2026 )

- The charger put back **430 mAh** into the log-1 pack ( from 3.5 V resting to full ).
- Shunt part **PE1206FRE470R02L**: Yageo PE series, 1206, 1%, R02 = 0.02 Ohm. **One** resistor in the battery
  path, so no parallel shunt; the firmware's x50 scale is right.

**Splitting the 297 mAh error.** The flights started at 4.0 V resting, not full, so part of the 430 mAh was never
flown. Assumptions: 4.0 V resting is 75-80% charge on a LiPo, and ~5% is left below 3.5 V resting.

| Share of the pack above 4.0 V | Pack total | Flown ( true ) | Firmware `D` / true | True hover current | `E` too high by | Count too low by |
|---|---|---|---|---|---|---|
| 20% | ~453 mAh | ~339 mAh | 0.60 | ~3.8 A | +161 mAh | +137 mAh |
| 25% | ~453 mAh | ~317 mAh | 0.64 | ~3.5 A | +183 mAh | +115 mAh |

So the 297 mAh the app shows on an empty pack is **two errors of about the same size**:

1. **The starting estimate is too high by ~160-180 mAh.** `E` = 500 assumes a 600 mAh pack and a linear voltage
   curve; this pack holds ~450 mAh, and at 4.0 V resting only ~320-340 mAh was left, not 500.
2. **The count is too low by ~115-140 mAh.** The firmware counted 202 mAh against ~320-340 mAh really drawn: the
   current reads **~60-64% of the true value** ( ~2.25 A shown against a real hover of ~3.5-3.8 A ). The counting
   code loses only 2-3%, so nearly all of that is in the INA219 reading itself.

The pack is worn or under-rated ( ~450 mAh to 3.5 V resting against 600 labelled ), but that alone does not
explain the symptom. A flight from a full pack ( task 5 ) removes the 20-25% assumption and gives the ratio
directly.

### Still needed ( task 5 )

- ~~The charger's recharge mAh for this pack~~: 430 mAh, see above.
- A run from a **full** pack ( 4.2 V ) so `E` = 600 and the whole capacity is in view.

## log-2: task 11 bench check, Developer Mode always on ( 25 Sep 2026, PRIMUS_X2_v1, props off, disarmed )

[logs/log-2.txt](logs/log-2.txt), PlutoMonitor over Wi-Fi, 17:48:38 to 17:49:29, no Dev Mode switch used.

| Check | Result |
|---|---|
| Restart markers ( `E` / `Cap` ) | **1**, at the start only, in 51 s ( log-1 on the same link: one every 2.9 s ) |
| Records | 491, all nine fields in every record; `t` step median 102 ms, wall-clock step median 95 ms |
| Ticks missing with no marker | **12** ( `t` step 203-204 ms, one tick skipped ), about one every 4 s |
| Disconnect stops user code within ~0.6 s | **Not observable this way**: the log reaches the app over the same link, so it stops when the app does. Covered by the reviewed code timeline ( CHANGES.md task 11 ), not by a measurement |
| Plug-in estimate | `E` 550 with the pack settling at 4.0 V: the single plug-in sample read 4.1 V ( log-1: `E` 500 at 4.0 V ). H4's one-sample weakness, seen directly |

**The 12 unmarked gaps.** A missing record can mean the drone skipped the tick ( main-loop stall ) or printed it and
the record was lost on the way to the app; `t` looks the same in both cases. The rate ( one per ~4 s ) matches
the Wi-Fi hiccups of log-1, and in log-1 any record lost during a hiccup was hidden inside the restart gap. So the
likely reading is that the Wi-Fi hiccups drop data in **both** directions: RC frames to the drone ( log-1 restarts )
and debug records to the app ( here ). A per-tick counter in the log line would settle it ( steps by 2 = printed and
lost, by 1 = the loop stalled ). Not needed for this topic's numbers: the counter and the mAh integration run in
`annexCode ( )`, not in user code, and one lost record per ~4 s changes an independent 10 Hz integral of `I` by
under 3% if it is not interpolated ( task 6 should interpolate across gaps using `t` ).

## Task 4 plan: plug-in map on the bench supply ( log-3 )

Bench supply ( 3 A max, 10 mA display ), leads into the battery connector, **props off**, never armed with props
on a supply. The supply replaces the pack runs planned first: it gives an exact plug-in voltage.

**Hook-up.** Supply off, set the voltage, current limit **1 A**. Mind polarity at the connector. Board fully off
( no USB ) between steps: `E` and `Cells` are computed once, at the moment the firmware first sees the battery.

**Test A: plug-in map.** For each voltage: supply on, connect the app, start PlutoMonitor, **then** Dev Mode on,
log ~15 s, Dev Mode off, supply off. Put a line `# psu <voltage>` before each run in
[logs/log-3.txt](logs/log-3.txt).

| Supply V | `E` predicted | `Cells` predicted | `V` predicted ( floored 0.1 V ) | What it tests |
|---|---|---|---|---|
| 4.30 | 650 | 2 | 4300 | `E` above the capacity; cell count 2 |
| 4.20 | 600 | 2 | 4200 | `vBatRaw / 42 + 1` gives 2 cells on a full pack |
| 4.15 | 550 | 1 | 4100 | floor of the single sample |
| 4.10 | 550 | 1 | 4100 | |
| 4.00 | 500 | 1 | 4000 | log-1 / log-2 cross-check |
| 3.90 | 450 | 1 | 3900 | |
| 3.80 | 400 | 1 | 3800 | |
| 3.50 | 250 | 1 | 3500 | |
| 3.20 | 100 | 1 | 3200 | |
| 3.00 | 0 | 1 | 3000 | edge of the formula |
| 2.90 | **65486** ( wrap ) | 1 | 2900 | `mAhRemain` wrap edge; does the board still boot and the app connect? Stop here if the ESP does not come up |

Real charge left on the ~450 mAh pack at these resting voltages ( typical LiPo ): 4.2 V ~450, 4.1 ~400, 4.0 ~340,
3.9 ~270, 3.8 ~200, 3.5 ~45.

**Test C: warning thresholds ( same hook-up ).** Boot at 3.80 V, Dev Mode on, then lower the supply in 0.05 V
steps to 3.00 V, ~20 s per step, and note on the app where the low-battery warning and the critical flag appear.
The firmware trips them on the fused SoC ( 18% / 8% ), not on a voltage; `S` in the log shows the SoC. Then raise
the supply back to 3.8 V and note whether the warning clears ( hysteresis 2%, and the SoC can only rise 1% per
call when disarmed ).

**Note on `Vc` and `G`.** A supply has no internal resistance, so the sag compensation ( `SYSTEM_R_MOHM` 100 )
and the auto-gain see a stiffer source than a pack; `Vc` will sit closer to `V` than in flight.

## Task 12 plan: current ratio sweep on the supply ( log-4 )

**Question.** The firmware reads ~60-64% of the true current in flight. Is that a constant scale error, or does the
reading lose the pulses of the 20 kHz motor PWM? Brushed motors at 100% throttle draw a steady current through
the shunt; at ~50% throttle the current comes in pulses of about twice the average. The supply's current display
is the reference.

**Hook-up.** Supply at **3.80 V**, current limit at its **maximum ( 3 A )**, **props off**, drone on the bench,
**disarmed**. The firmware runs the sequence itself ( `BENCH_MOTOR_SEQUENCE` in `PlutoPilot.cpp` ): switch Dev
Mode on with PlutoMonitor listening and it holds the four motors at 1000 ( idle ), 1250, 1500, 1750, 2000 us for
10 s each, then stops. The log field `Ph` shows the level ( 0 idle … 4 full, 5 stop, -1 not running ). During
each level read the supply's current display and note it: `# Ph<n> <amps>` lines in the log, or a list in the
chat. Props-off motors draw about 1-2 A in total at full. Switching Dev Mode off at any point stops the motors.
Repeat the whole sequence twice for consistency ( Dev Mode off, wait, on again ). A Wi-Fi drop longer than
400 ms stops the motors and, on reconnect, re-runs the sweep from idle by itself: the analysis splits the log at
every `E:` marker. `M` lags `Ph` by one row at each level change; the first rows of a level are dropped anyway
for settling. **When log-4 is captured, set `BENCH_MOTOR_SEQUENCE` to 0 and rebuild before doing anything else.**

**Reading.** For each level: firmware `I` ( median over the level ) / supply current.

| Result | Meaning | Next |
|---|---|---|
| Ratio ~0.6 at every level, 100% included | A constant scale error: a parallel copper path around the shunt, the sense connection, or the INA219 setup. Not PWM. | Task 10 is pointless; the findings point at the board / driver setup instead |
| Ratio near 1.0 at 100%, falling toward 50% | The reading loses the pulses: PGA clipping or the 532 us sampling | Task 10 ( PGA /8 A/B ) decides between those |
| Ratio near 1.0 everywhere | The bench does not reproduce the flight error ( amplitude too low ) | Task 5 in flight, then task 10 |

Also read: `V` vs the supply voltage at each level ( supply droop ), `G` ( should stay 1000: the auto-gain gate
needs `mAmpRaw >= 1000` and `vShuntRaw > vBatRaw` ), and `I` at `Ph` 0 ( 50 mA step; the supply shows the true idle
draw, which the user remembers as ~100 mA on the older BMS ).

## log-3: plug-in map and motor sweep on the bench supply ( 25 Sep 2026, PRIMUS_X2_v1, props off, disarmed )

[logs/log-3.txt](logs/log-3.txt), ten runs, each a fresh power-up at one supply voltage, Dev Mode on ( the task 12
motor sequence ran in every run ). The user read the supply's current display: "ideal" = idle, "load" = the peak
while the motors ran. The last run is labelled `2.0v`; the firmware's `E` 65436 means the bus read 2.8 V, which
fits a **2.9 V** setting ( assumed ).

### Plug-in map ( test A )

| Supply | `E` predicted | `E` logged | `Cells` logged | `V` idle ( bus, floored ) | `S` idle |
|---|---|---|---|---|---|
| 4.3 V | 650 | **600** | **2** | 4.2 | 97 |
| 4.2 V | 600 | **550** | 1 | 4.1 | 91 |
| 4.1 V | 550 | 550 | 1 | 4.0 | 89 |
| 4.0 V | 500 | **450** | 1 | 3.9 | 75 |
| 3.9 V | 450 | 450 | 1 | 3.8 | 72 |
| 3.8 V | 400 | 400 | 1 | 3.7 | 64 |
| 3.5 V | 250 | **200** | 1 | 3.4 | 35 |
| 3.2 V | 100 | **50** | 1 | 3.0 | 13 → 7 |
| 3.0 V | 0 | 0 | 1 | 2.9 | 0 → **54-58** |
| 2.9 V | 65486 | **65436** | 1 | 2.8 | **54** |

- `E` follows `( floor ( V_bus ) - 3.0 ) / 1.2 x 600` exactly. The plug-in sample reads **0.1 V below the supply**
  in 6 of 10 runs ( the floor plus ~20-100 mV between the supply terminals and the INA219 ); the settled idle `V`
  reads 0.1 V low in every run. A charged pack at 4.2 V therefore starts at `E` 550, 1 cell; at 4.3 V ( a LiHV
  pack, or a fresh 4.2 pack reading high ) `Cells` is **2**.
- **Wrap at empty, measured.** At 3.0 V `E` = 0, so the first mAh counted makes `mAhRemain` = 65535; at 2.9 V `E`
  itself wraps. `soc_from_mAh ( )` returns 100% for anything above the capacity, and the fused SoC `S` **jumps to
  54-58%** on an empty pack. The low-battery warning ( `S` <= 18 ) clears. Armed, the "never increase in flight"
  clamp holds `S` down; after landing it climbs back at 1% per call. Safety-relevant for the fix.
- Warning thresholds ( test C, replaced by the `S` map ): `S` crosses the warning level ( <= 18% ) between 3.5 and
  3.2 V supply and the critical level ( <= 8% ) at ~3.2 V; the flags themselves were not observed ( not logged ).
- `Vc` equals `V` at idle and sits 100 mV below under load: the sag compensation adds `mAmpRaw x 100 mOhm` = ~20 mV
  on these small currents, and the supply drops ~100 mV into the leads at ~0.5 A.

### Current: supply vs INA219 ( first task 12 data )

| Supply | Idle: supply / `I` | Motors 1250-2000 us: supply peak / `I` median |
|---|---|---|
| 4.3 V | 100-120 / 50 mA | 510-530 / 200-250 mA |
| 4.2 V | 100-120 / 50 | 500-520 / 200 |
| 4.1 - 3.8 V | 100-120 / 50 | 480-510 / 200 |
| 3.5 V | 100-120 / 50 | 465-480 / 200 |
| 3.2 V | 120-160 / 50 | 450-465 / 200 |
| 3.0 V | 120-180 / 50 | 450-465 / 200 |
| 2.9 V | 150-200 / 50 | 450-465 / 200 |

- `I` moves in 50 mA steps ( whole shunt mV ): 50 means the INA219 saw 50-99 mA, 200 means 200-249 mA.
- **Idle, motors stopped:** the supply shows 100-120 mA, the INA219 sees 50-99 mA. There is no motor PWM here.
- **Motors at 2000 us ( 100% duty ):** the current through the shunt is steady DC; the INA219 sees 200-249 mA of
  ~450-530 mA.
- `I` is flat from 1250 to 2000 us ( the user noted one peak figure per run; whether the supply current also
  stayed flat across the levels is to be confirmed ).
- Ratio INA219 / supply: **~0.4-0.5 under load, ~0.4-0.9 at idle** ( the idle range is set by the 50 mA step ). In
  flight ( log-1 + charger ) it was ~0.60-0.64, with a 20-25% assumption and a charger of unknown accuracy.

**Verdict ( task 12 decision table, first row ):** the INA219 reads roughly half the current **at DC** ( idle, no PWM;
100% duty, no pulses ). This is a **constant scale error**, not the PWM pulses: H6 ( PGA clipping ) is ruled out as
the main cause and task 10 is pointless. The remaining candidates are in the board current path: part of the
current returning around the shunt ( e.g. a ground pour joining both ends of a low-side shunt ), or the INA219
sense connections not taken at the shunt pads. The INA219 set-up itself is unlikely: the shunt register LSB is
10 uV at every PGA setting and the driver reads it correctly.

**Settled ( user, after this log ):** there is a **second R020 stacked in parallel** on the first, so the shunt is
**10 mOhm**; the code's x50 conversion is for 20 mOhm, hence exactly half. Re-read with 10 mOhm: idle 100-198 mA
( supply 100-120 ), motors at full 400-498 mA ( supply 450-530 ). The optional task 13 cross-check now expects
~5 mV across the pads at 0.5 A ( 10 mV/A ).

## Task 13 plan: one R020 ( 20 mOhm ), three points ( log-5 )

Same procedure as log-3 ( fresh power-up per point, app, PlutoMonitor, Dev Mode on, motor sequence, supply idle and
load current noted ) at **4.2, 3.5 and 3.0 V**, after the second R020 has been removed.

| Supply | Expected `I` idle ( supply ~100-120 ) | Expected `I` motors full ( supply ~450-500 ) | `E` / `Cells` |
|---|---|---|---|
| 4.2 V | 100 ( was 50 ) | 450-500 ( was 200-250 ) | 550 / 1 as log-3 |
| 3.5 V | 100 | 450 | 200 / 1 |
| 3.0 V | 100-150 | 450 | 0 / 1, `S` jumps on the wrap |

`I` still moves in 50 mA steps ( one shunt mV ). If it matches the supply within one step, H1 is confirmed by direct
comparison. If it stays at half, the parallel part was not the cause and the sense wiring is next.

**Caution for flight with one R020:** 0.5 W in one 1206 at 5 A, 100 mV drop, and the INA219 range clips at 8 A. Fine
on the bench; the fix decides which board configuration is the production one.

## log-5: one R020 ( 20 mOhm ), three points ( 25 Sep 2026, PRIMUS_X2_v1, props off, disarmed )

[logs/log-5.txt](logs/log-5.txt). The user removed the second R020; same 18:58 build and procedure as log-3.

| Supply | Idle: supply / `I` log-3 → **log-5** | Motors at 2000 us: supply / `I` log-3 → **log-5** | `E` / `Cells` | `V` idle |
|---|---|---|---|---|
| 4.2 V | 120-125 / 50 → **100** | 520-535 / 200 → **500** | 600 / 2 ( log-3: 550 / 1 ) | 4.1 |
| 3.5 V | 150-170 / 50 → **100** | 480-495 / 200 → **450** | 200 / 1 | 3.4 |
| 3.0 V | 180-200 / 50 → **150** | 470-485 / 200 → **450** | 0 / 1, `S` 54-58 ( wrap ) | 2.9 |

- **H1 confirmed by direct comparison:** with one 20 mOhm shunt the INA219 reading tracks the supply within one
  50 mA step, at idle and under load, at every voltage. With two in parallel ( log-3 ) it read half.
- The 20-50 mA still missing is the two round-downs ( whole mV, then the 50-sample average ): the known L3 loss.
- The voltage path is unchanged ( same `V` as log-3 ), as expected: it does not pass through the shunt.
- `E` at 4.2 V was 600 / 2 cells here and 550 / 1 in log-3: the single plug-in sample sits on the 4.1 / 4.2 floor
  boundary ( H4 ). At 3.0 V the SoC jumps to 54-58% again ( H5b ).
- `I` at 1250 us runs 250-500 ( settling into the level ); from 1500 us on it is flat at 450-500, as is the supply.

## Task 5 plan: full pack to the low-battery warning, one R020 ( log-6 )

**Set-up.** Board with **one R020** ( 20 mOhm, the value the firmware assumes; log-5 ). The **log-1 pack** ( charger put
back 430 mAh from 3.5 V resting ). Build `Build/PRIMUS_X2_v1/DEFAULT_PRIMUS_X2_v1_3.10.0.hex` from **20:35, 25 Sep**
( bench motor sequence compiled out; log line and task 11 link grace in ).

**Procedure.**

1. Flash the 20:35 build. Props on.
2. Charge the pack full on the charger; rest it **10 min**. Note the charger's final voltage.
3. Plug in, connect the app, start PlutoMonitor, **then** Dev Mode on. The first line must be `E:… Cap:… Cells:…`.
4. Arm, take off, hover low ( ~1 m ), ALT_HOLD is fine. **Gentle inputs, no hard climbs**: with one R020 the INA219
   range tops out at 8 A, and the single 1206 carries ~0.5 W at 5 A.
5. Fly until the **app's low-battery warning** ( or ~3.3 V under load if it never comes ). Note the app's
   "remaining mAh" at that moment ( screenshot if easy ).
6. Land, disarm, **keep logging 60 s** at rest. Stop PlutoMonitor; save everything into [logs/log-6.txt](logs/log-6.txt).
7. Touch-check the shunt area after landing ( warm is fine, too hot to touch is not ).
8. Charge the pack and note the charger's **mAh**.

**What it should show.**

| Quantity | Prediction | Why |
|---|---|---|
| `E` at plug-in | 550 or 600 | single floored sample at the 4.1 / 4.2 boundary ( log-3 vs log-5 ) |
| Hover `I` | ~3.5-3.8 A ( log-1 read 2.25 A with two R020 ) | log-1 + charger estimate of the true hover current |
| Flight time to the warning | ~5-7 min | ~450 mAh pack at ~3.6 A |
| `D` / charger mAh | **0.92-0.97** | code losses only: gain -5% when it engages, dt 0 to -2%, round-downs ~-1% |
| App remaining at empty ( `E` - `D` ) | ~150-200 mAh, no longer ~300 | what is left is the plug-in estimate error ( 600 label vs ~450 real ) |
| `G` | 1000 → ~950 within ~10 s once current > ~2 A | the auto-gain unit mismatch ( L1 ) |

This flight gives the fix its **acceptance number**: `D` / charger with the scale right, and the size of the remaining
plug-in estimate error.

## log-6: full pack to the warning, one R020 ( 26 Sep 2026, PRIMUS_X2_v1, flight )

[logs/log-6.txt](logs/log-6.txt), 02:15-02:23, the log-1 pack charged full, one R020 ( 20 mOhm ), 20:35 build. 4599
records, one restart marker ( the start ), 93 one-tick gaps ( lost on Wi-Fi, one per ~5 s ).

| Segment | Duration | V ( bus ) | `D` | `I` median / max | M median | `G` | `S` |
|---|---|---|---|---|---|---|---|
| Ground | 2 s | 4.1 | 0 | 100 mA | 1000 | 1000 | 91 |
| **Flight** | **467 s** | 4.0 → 2.8 | 0 → **527** | **4300 / 4500 mA** | 1636 | 1000 → **950** | 90 → 3 |
| Ground after | 8 s | 2.9 → 3.5 | 528 | 100 | 1000 | 950 | 4 → 33 |

`E` 550, `Cap` 600, `Cells` 1 ( bus read 4.1 at plug-in ).

Flight in 60 s blocks:

| t s | M | `I` mA | V | `Vc` | `D` | `S` |
|---|---|---|---|---|---|---|
| 0 | 1561 | 4150 | 3.6 | 3750 | 69 | 72 |
| 124 | 1611 | 4250 | 3.5 | 3645 | 207 | 53 |
| 249 | 1648 | 4300 | 3.3 | 3451 | 348 | 37 |
| 373 | 1682 | 4350 | 3.2 | 3352 | 491 | 22 |
| 436 | 1739 | 4400 | 3.0 | 3154 | 527 | 3 |

**What it shows.**

1. **The symptom is gone with the scale right.** App remaining at the end = `E` - `D` = 550 - 528 = **22 mAh**
   ( log-1: 297 ). Hover current reads 4.15-4.4 A, rising as the pack sags ( log-1 with two R020: 2.25 A flat ).
2. **The auto-gain now always engages:** `G` reaches 0.950 within seconds of arming ( at ~86 shunt mV its mixed-unit
   gate is met ). `D` is exactly 95% of the pre-gain integral: 528 vs **556.1 mAh**. L1 is a full -5% in every flight.
3. **The pack delivered ~560 mAh** ( pre-gain integral 556, plus ~1% round-down ). The earlier ~450 mAh estimate came
   from the log-1 charger figure ( 430 mAh from 3.5 V ) under a 20-25% assumption; log-5 showed the INA219 within one
   50 mA step of the bench supply. The charger reading for this flight decides whether the charger reads low or the
   estimate was wrong.
4. **`E` was close by luck:** 550 ( one floored sample at 4.1 V ) against ~560 real. With the 600 label and a straight
   line the error depends on the pack and on which side of the 0.1 V floor the sample lands.
5. **The warning comes late.** Fused `S` crossed the warning level ( <= 18% ) at **t+453 s**, `D` 508, 3.0 V under load,
   and critical ( <= 8% ) at t+469 s, 2.8 V: about **15 s before the end of the flight**. `M` was already 1739 ( motors
   at 74% to hold height ). And `D` finished only 22 mAh short of `E`: a little longer and `mAhRemain` would have
   wrapped ( H5b ).
6. **Link:** one restart marker in 8 minutes ( log-1: 121 in 6 ): the task 11 grace works in flight.
7. **At rest:** only 8 s logged after landing ( 3.5 V and rising ), not the 60 s planned; the resting voltage is not
   settled.

**Charger ( user, 26 Sep ): 534 mAh put back.**

| Comparison | Value |
|---|---|
| Firmware `D` / charger | 528 / 534 = **0.99** |
| Pre-gain integral of `I` / charger | 556 / 534 = **1.04** |
| App remaining at the end ( `E` - `D` ) vs real ( ~0 ) | 22 mAh |

- With the scale right the count lands within **1%** of the charger, inside the +/-5% stretch target. That is partly
  luck: the auto-gain's -5% cancels a reading that runs ~4% above the charger ( the INA219's own gain error, the
  charger's accuracy, or both; at 4 A the round-downs are only ~-1% ). The fix should remove the gain and accept
  against the charger at +/-5%.
- **Cross-check with log-1:** its pre-gain integral ( 204.8 mAh with two R020 ) doubles to ~410 mAh drawn from
  4.0 V resting. With the pack holding ~534 mAh, 4.0 V resting was ~77% charge, which matches the 20-25% assumption
  made on 25 Sep. The pack is ~534 mAh, not ~450: that estimate came from the halved reading.
- `E` 550 against 534 real: the plug-in estimate was right to ~3% here, but by luck ( one floored sample and the
  600 label ).

## Task 6: discharge analysis ( log-1, log-3, log-5, log-6 + charger )

### Counts against the charger

| Flight | Shunt | Start | Firmware `D` | Pre-gain integral | Charger | `D` / charger | Integral / charger |
|---|---|---|---|---|---|---|---|
| log-1 | two R020 ( 10 mOhm ) | 4.0 V resting | 202 | 204.8 ( x2 = ~410 ) | 430 ( from 3.5 V ) | 0.47 ( 0.60-0.64 of the flown part ) | ~0.95 of the flown part, once doubled |
| **log-6** | **one R020 ( 20 mOhm )** | **full** | **528** | **556.1** | **534** | **0.99** | **1.04** |

Gaps: log-6 lost 93 records ( 19.8 s ) on Wi-Fi; holding the last value across a gap and interpolating give the
same 556.1 mAh, so lost records do not affect the integral at this rate.

### Auto-gain trajectory ( log-6 )

`G` 0.99 at 2 s after arming, 0.97 at 6.7 s, 0.96 at 11.2 s, **0.950 at 28 s**, then fixed there for the flight and
after landing ( a `static`, reset only at power-off ). With the correct shunt the gain's gate ( `vShuntRaw >
vBatRaw` in mixed units, sag >= 20 ) is met at any hover current, so L1 is a full -5% in every flight.

### Voltage and fused SoC against charge used ( log-6, charge used = `D` / 0.95, 534 mAh = 100% )

| Used | `V` under load | `Vc` | `I` mA | Fused `S` | True charge left |
|---|---|---|---|---|---|
| 0% | 3.8 | 3943 | 4100 | 85 | 100% |
| 10% | 3.6 | 3745 | 4150 | 74 | 90% |
| 20% | 3.5 | 3647 | 4200 | 66 | 80% |
| 30% | 3.5 | 3646 | 4250 | 61 | 70% |
| 40% | 3.4 | 3547 | 4250 | 53 | 60% |
| 50% | 3.4 | 3547 | 4300 | 49 | 50% |
| 60% | 3.3 | 3448 | 4300 | 41 | 40% |
| 70% | 3.3 | 3448 | 4350 | 37 | 30% |
| 80% | 3.3 | 3448 | 4350 | 32 | 20% |
| 90% | 3.2 | 3348 | 4350 | **24** | 10% |
| ~100% | 3.1 | 3248 | 4400 | **21** | ~0% |

- The fused SoC reads ~15% low early ( 85 at full ), about right in the middle, and **~15-20 points high at the end**:
  24% with 10% left, 21% with almost nothing. The 18% warning therefore fires with ~3% left ( t+453 s of 467 ).
- The loaded voltage is flat at 3.3-3.4 V from 40% to 80% used ( 0.1 V steps ), so the straight-line voltage SoC
  ( 2.9-4.2 V ) cannot place the end; the mAh side has to carry the last 30%, and it is only as good as `E`.
- `Vc` ( sag-compensated ) sits ~150 mV above `V`: the compensation adds `I x 100 mOhm` ( ~430 mV at 4.3 A ) weighted
  0.35 plus the shunt drop.

### Verdict per hypothesis

| # | Hypothesis | Verdict | Share of the ~297 mAh ( log-1 ) |
|---|---|---|---|
| H1 | Current scale wrong | **Confirmed**: two R020 in parallel ( 10 mOhm ) vs 20 mOhm in the code; log-3 half, log-5 and log-6 right with one | ~200 mAh ( the count was half ) |
| H2 | Pack well under its label | **Ruled out as a cause**: ~534 mAh ( 89% of 600 ); the ~450 estimate came from the halved reading | ~65 mAh via the 600 label in `E` |
| H3 | Code losses in the counter | **Confirmed, small**: gain -5% ( always engaged ), dt ~0 at a steady 21 ms, round-downs ~-1%; offset here by a +4% reading | -5 to -6% |
| H4 | Plug-in estimate | **Confirmed weakness**: one floored sample ( 550 vs 600 on the same supply voltage ), straight line, labelled capacity; right to ~3% in log-6 by luck | ~30-100 mAh depending on the boot |
| H5 | Edge bugs | **Confirmed**: `mAhRemain` wrap ( SoC 54-58% on an empty pack, H5b ), `Cells` 2 at >= 4.2 V; CRSF unit, `BMS_Update` every loop, 0xFFFF samples from code | safety, not size |
| H6 | PGA clipping on PWM peaks | **Ruled out** ( log-3: half at DC too ) | 0 |
| H7 | Board current path | **Resolved as H1** | - |
| new | Late warning | **Confirmed**: fused SoC 21-24% at the end, warning ~15 s before empty | safety |
