# Off-centre payload lean - Testing

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [TESTING](TESTING.md)

## 1. Task 1: confirming flight, partial vs full stick descent ( 19 Oct 2026 )

| | |
|---|---|
| **Build** | FW 3.15.1 ( `a81c180` ), PRIMUS_X2_v1, settings reset by the reflash ( EEPROM 106 → 108 ), accelerometer and compass recalibrated |
| **Payload** | 9 g servo, dead weight, front |
| **Mode** | ALT_HOLD on, app take-off, no stick except the throttle descents |
| **Log** | none ( observation by the pilot ) |

**Procedure.** Hover, descend with the throttle stick partway down, climb back, descend with the stick fully down.

**Prediction ( H1 ).** The stick-low reset fires only while the raw throttle is below `mincheck` 1100
( [rc_controls.cpp:135](../../../../src/main/io/rc_controls.cpp#L135) ), the bottom ~10 % of a 1000-2000 stick. A
descent with the stick above that keeps the hover integrator and comes straight down; one in that band leans
toward the payload.

**Result ( user report ).** The lean appears **only with the stick fully down, or a little above fully down**. Higher
partial-stick descents come straight down.

**Verdict.** H1 **confirmed**: the lean is tied to the bottom band of the stick, not to descending as such. No
bisect is needed. Not covered by this flight: the take-off lean ( H2 ), which task 3 logs.

## 2. Task 3: baseline log with the servo ( plan )

| | |
|---|---|
| **Build** | diagnostic: scratchpad `hex/DIAG_task2_PRIMUS_X2_v1_3.15.1.hex` ( `a81c180` + task 2 getters + [test/PlutoPilot_lean_diag.cpp.txt](test/PlutoPilot_lean_diag.cpp.txt) ). EEPROM 108, as the stock 3.15.1: no settings reset |
| **Payload** | 9 g servo, dead weight, **front** ( same spot as task 1 ) |
| **Mode** | ALT_HOLD on, app take-off and land, no roll / pitch / yaw stick |
| **Log** | [logs/log-1.txt](logs/log-1.txt) ( PlutoMonitor, Developer Mode on, app connected ) |

**Sequence** ( one power cycle, one log ):

1. Power on, servo fitted. Developer Mode **on** before arming ( the log runs only in Dev Mode with a live RC link );
   an `S:` line marks it. Leave it disarmed on the floor about 10 s.
2. **Take-off A**: app take-off button. Hands off. Hover 10-15 s.
3. **Full-stick descent**: throttle stick fully down for about 3 s ( −30 cm/s, about 90 cm from the 120 cm goal: stop
   before the floor ), then centre it. Hover 5 s.
4. **Land** with the app land command. Stay disarmed about 5 s.
5. **Take-off B** ( same power cycle ): app take-off, hover 10 s, app land.
6. Note by eye or video: lean direction and roughly how long it lasted at each take-off and in the descent.

**What each phase should show ( prediction from H1 / H2 ):**

| Phase | `Rs` ( read as the step between lines ) | `Ip` | `P` ( pitch, decideg ) | `Tp` |
|---|---|---|---|---|
| Disarmed on the floor | flat | drifts ( the PID runs disarmed ) | ~0 | 1500 |
| Arm ( take-off A, B ) | **+1** | **→ 0** | ~0 | 1500 |
| First ~1-1.5 s after lift-off ( `Ze` rising ) | flat | builds from 0 toward `Ih` ( τ ~0.5 s ) | **lean toward the front, decaying** | 1500 |
| Settled hover | flat | steady at `Ih` ( est. 50-100 ) | ~0 | 1500 |
| Full-stick descent | **+~50 per second** | **≈ 0 throughout** | **standing lean toward the front** | < 1100 |
| After centring | flat | rebuilds to `Ih` | lean decays | 1500 |
| App land | flat | stays ≈ `Ih` to touchdown | ~0 | 1500 |

`F` digits: armed, ALT HOLD, `isLanding`, ARM-switch ground idle. `Ir` should stay small with a front payload, unless the
servo is also off the centre line.

**Numbers to extract** ( they size tasks 5 and 6 ):

- `Ih`: settled hover `Ip` ( and `Ir` ), mean and spread → task 5's clamp ( planned ±128 ).
- Peak `P` and how long it takes to settle after each take-off and after centring → task 6's k and window.
- Arm to lift-off time ( `Ze` starts rising ) → whether task 6's window can start at the arm or must wait for lift-off.
- Standing `P` during the full-stick descent → the size of the lean the fix removes.

**H2 is confirmed** if `Ip` is 0 at lift-off and the take-off lean decays as `Ip` builds; **killed** if `Ip` is already
near `Ih` at lift-off while the craft still leans.

## 3. Task 3: results ( [logs/log-1.txt](logs/log-1.txt), 19 Oct 2026 )

Analysed by `pluto-log-analyst`; the key numbers re-checked from the raw log ( window aggregates and the take-off / recovery
records ). 643 records, 65 s, 9.9 Hz; no missing fields, no `t` gap over 250 ms, no Dev Mode restart in flight.
Angles in **decidegrees** ( 48 = 4.8° ). **Positive `P` = nose down, toward the front payload**: with `Ip` held at 0 in
the full-stick descent B the craft stands at `P` +9.7° ( below ).

**Flown vs plan.** Two short flights instead of one long hover: 3 s on the floor before arm A ( not 10 s ); flight A had
the planned full-stick descent ( A1, 2.6 s ) plus a second, unplanned dip ( A2, 0.8 s ); flight B one dip ( 1.1 s ). All
dips ended 30-55 cm above the floor.

| Event | s after log start | Notes |
|---|---|---|
| Arm A | 3.15 | `Rs` +1, `Ip` −256 → −13 |
| Lift-off A | ~3.9 | `Ze` leaves 0 |
| Stick-low A1 / A2 | 23.5-26.1 / 32.6-33.4 | `Tp` 1000-1093 |
| Land A ( flag → disarm ) | 38.8 → 39.6 | |
| Arm B | 46.45 | `Rs` +1, `Ip` +2 → 0 |
| Lift-off B | ~46.9 | |
| Stick-low B | 55.5-56.6 | |
| Land B | 61.0 → 62.9 | |

**Settled hover ( `Ih` ).** A: `Ip` −101.7 ± 4.1, `Ir` +5.9 ± 4.7; B: `Ip` −105.1 ± 7.2, `Ir` +7.0 ± 2.6. `P` 3.5 ± 17.5,
`R` −0.7 ± 6.3 ( level within ±2° noise ). The 9 g servo at the front needs about **−103 of the ±256** pitch integrator,
40 % of `GYRO_I_MAX`, and about 80 % of the ±128 clamp planned for task 5.

**Take-off ( H2 ).** Confirmed. At each arm `Rs` steps by 1 and `Ip` drops to about 0, including arm B, 7 s after a
hover at −105: the learned trim is thrown away between flights. The motors spin up with the craft still on the floor
and the integrator builds there: `Ip` −58 at 0.5 s after arm A, while `P` rises to **+4.8°** with `Ze` still about 0.
The craft pivots on its front legs before it lifts. At lift-off `Ip` is −74 ( A, 0.8 s ) and −42 ( B, 0.4 s ); it
reaches 90 % of `Ih` about 1.7 s after the arm. Peak lean +4.8° ( A ) and +8.7° ( B ), back to level in about 0.7 s,
followed by an overshoot to −3.5°. The analyst also reported a second, larger pitch swing about 4 s after arm A
( +13.9° ) while `Ip` was already settled. It is not caused by the integrator; possibly the end of the climb. Not
investigated further: re-check it in task 8.

**Full-stick descent ( H1 ).** `Rs` climbs about 60 per second, `Ip` stays at −1 to −3 throughout, and `Ir` at 0. In
dip B ( clean ) the craft stands **nose down at +9.7° ± 1.4°** ( 5.9-11.1° ) toward the payload, which is the drift the
pilot feels. Dip A1 swings ±10° around +2.7°. Descent rate about 29 cm/s, matching `ALT_MAX_DESCENT_CMS` 30.

**Recovery after a dip ( new observation ).** After each dip, `Ip` rebuilds from 0 to −80 in about 0.8 s and overshoots
to −120 / −137. Meanwhile pitch and roll swing hard: after A1, `P` −13.8° → +19.6° → −14.2° with `R` up to +17.8°,
settling about 4.8 s after the stick is centred; after B, `P` ±17-18° and `R` −13.6°, with `Ir` up to 63 ( hover 7 ). The
pitch swing fits the integrator rebuilding with the craft already moving forward and 30-50 cm above the floor
( ground effect ). The roll swing ( `Ir` small in A1 ) does not. The log has no roll / pitch stick field, so pilot
correction cannot be ruled out. With task 4 the integrator is kept through the dip, so this transient should not occur.
Task 8 checks it.

**Floor windup.** Disarmed on the floor before arm A, `Ip` ran from −153 to the **−256 rail** within about 1 s and stayed
there. `P` read +1.1° on the floor ( floor or calibration offset ), and the disarmed PID integrates that error with no
feedback. Between the flights `Ip` swung −115 → +64 → +2. This confirms that a raw carry-over across an arm is unsafe.
The task 5 seed must come only from airborne hover.

**Numbers for later tasks.**

| For | Value |
|---|---|
| Task 5 seed clamp | `Ih` ≈ −103 ( 9 g at the front ); planned ±128 leaves 25 % margin: decide at task 5 |
| Task 6 window | arm → lift-off 0.4-0.9 s with the motors spinning and the integrator already building ( in the right direction ); the take-off lean starts on the floor, so the window can start at the arm |
| Task 6 target | rise to 90 % of `Ih` takes ~1.7 s at k = 1; lean peak 4.8-8.7°, ~0.7 s |
| Fix size | the full-stick standing lean the fix removes: ~10° |
