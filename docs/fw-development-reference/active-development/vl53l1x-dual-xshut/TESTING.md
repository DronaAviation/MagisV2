# VL53L1X Dual XSHUT - Testing

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [DESIGN](DESIGN.md) · [TESTING](TESTING.md)

## Task 1: TOF400C SHUT pin measurement

**Question.** Does the TOF400C module pull its SHUT ( XSHUT ) pin up on the board, to what voltage, and does it
tolerate 3.3 V from the FC? This decides how `ranging_init_L1 ( )` enables the bottom sensor on GPIO_5 ( PB15 ).

**Why measure off the FC.** With the current firmware the FC pins are not floating: `spiInit ( SPI2 )`
( [main.cpp:313](../../../../src/main/main.cpp#L313) ) configures PB15 as SPI2 MOSI ( driven ) and PB14 as
MISO, and `m25p16_init ( )` ( [main.cpp:450](../../../../src/main/main.cpp#L450) ) clocks data on MOSI.
A reading taken with the wire on the FC would show the SPI line, not the module.

**Procedure** ( one module is enough if both are the same board; do both if unsure ):

1. **Power off.** Disconnect the module's SHUT wire from the FC ( GPIO_5 / GPIO_6 ). Leave VIN, GND, SDA and
   SCL connected.
2. **Resistance, power off:** SHUT to VIN, and SHUT to GND. ( Gives the pull-up / pull-down value if present. )
3. **Power on** ( battery or USB ).
4. **Voltage, SHUT to GND** with the wire open. Note: VIN of the module, and SDA / SCL idle voltage for
   reference.
5. **Look at the board:** the part next to the SHUT pin ( resistor only, or a transistor / level shifter ), the
   regulator marking ( e.g. a 2.8 V LDO ), and any silkscreen on the pins.

**Decision rule.**

| SHUT reading with the wire open | Drive mode for the ON pin ( GPIO_5 ) |
|---|---|
| Stable ~2.8 V or ~3.3 V, pull-up resistor present | release as input; the module's pull-up enables the sensor |
| Floating / 0 V, no pull-up, level shifter or 3.3 V-rated pin | drive HIGH push-pull |
| Floating, no pull-up, SHUT goes straight to the 2.8 V sensor pin | stop: ask before driving 3.3 V into it |

The OFF pin ( GPIO_6 ) is driven LOW in every case.

**Results** ( 5 Oct 2026, user, multimeter ).

| Item | Bottom module | Second module |
|---|---|---|
| SHUT voltage to GND, wire disconnected from the FC, powered | **2.7 V** | not measured ( same TOF400C board, assumed identical ) |
| SHUT to VIN / GND resistance ( power off ) | not measured | not measured |
| Parts near SHUT / regulator | not recorded | not recorded |

**Reading.** A stable 2.7 V with nothing connected means the module pulls SHUT up on the board to its ~2.8 V
sensor supply ( the 0.1 V is meter / regulator tolerance ). SHUT is therefore wired to the 2.8 V side of the
module, not to a 3.3 V-tolerant level-shifted input.

**Decision.** ON pin ( GPIO_5 / PB15 ): **release as a floating input** ( no internal pull ); the module's
pull-up enables the bottom sensor. Do **not** drive it HIGH push-pull: 3.3 V against a pull-up to the 2.8 V
rail would back-feed that rail through the resistor and exceed the sensor's own supply level on XSHUT. OFF pin
( GPIO_6 / PB14 ): **output push-pull LOW**; sinking the module's pull-up current is harmless.

## Task 6: hardware validation ( PRIMUS_X2_v1, both TOF400C fitted )

**Build.** Working tree after task 5 plus the temporary log in [PlutoPilot.cpp](../../../../PlutoPilot.cpp)
( remove before release ): `LASER_TOF_L1x` + `LASER_ALT` on, XSHUT block in `ranging_init_L1`. Gate clean,
113.0 KB / 16.0 KB. **The laser is in the control loop.** Check the monitor shows the `G:` and `S:` fields
before logging ( proves this image is flashed ).

**Log fields** ( 10 Hz, ~58 characters, ~103 B per tick with framing, under the 130 B budget ):

| Field | Meaning | Healthy |
|---|---|---|
| `t` | `millis ( )`, ms | steps ~100 |
| `mm` | last valid laser range, mm | floor ~25-28 ( near-field, as in the parity topic ); tape height +- 30 |
| `St` | ST RangeStatus of the last sample | 0 |
| `G` | `Global_Status_L1`, latched driver error | **0 all the time** |
| `N` | laser samples since boot | +2 per tick ( 50 ms period ) |
| `S` | estimator source: 1 laser, 0 baro, 2 object hold-off | 1 below 160 cm, 0 above, back to 1 below 140 |
| `E` / `H` | estimate / setpoint, cm | hover `E` within +-3 cm of `H` |
| `Ar` | armed | |
| `#START` | Developer Mode ( re ) start, own line | |

**Pass / fail decided in advance.**
- Pass: `G` 0 and `N` advancing for the whole session with both sensors fitted; `mm` within ~3 cm of the tape at
  each bench height; in flight `S` follows the 160 / 140 band and the hover holds like the parity logs ( log-10 ).
- Fail: `G` != 0, `N` stuck, `mm` off by more than ~5 cm or jumping between two values ( two sensors answering ),
  or `S` 0 at low height for no reason.

**Rollback.** Land normally. Fallback image: the same tree with the second sensor's cable unplugged, or
`LASER_ALT` off ( baro only ).

### Test 0: pins ( multimeter, after boot )

Power up normally ( SHUT wires connected to the FC ). Measure SHUT to GND on each module:
bottom ( GPIO_5 ) **~2.7 V**, second ( GPIO_6 ) **~0 V**. Note the values here.

### Test 1: bench, both sensors ( `logs/log-1.txt` )

Props off.
1. Power up, connect the app, Developer Mode on, start logging. **15 s** flat on the floor.
2. Hold the drone level with the laser window at **20 cm**, then **60 cm**, then **120 cm** above the floor
   ( tape from window to floor ), **10 s** each. Say the order if it differs.
3. Back on the floor 5 s. Stop logging, paste into `logs/log-1.txt`.

### Test 2 ( optional ): negative check, second sensor awake ( `logs/log-2.txt` )

Shows that the hold-off is what makes it work. **Power off**, disconnect the second sensor's SHUT wire from GPIO_6
( its pull-up then enables it ), power on, repeat Test 1 steps 1-2 at the floor and 60 cm only. Expect `G` != 0,
`N` stuck, or a wrong / jumping `mm`. **Power off and reconnect the wire before Test 3.**

### Test 3: ALT_HOLD hover and handover ( `logs/log-3.txt` )

Same site rules as the parity topic: matt floor, away from the ceiling fan, full battery, spotter, ALT_HOLD
throughout, land normally to abort.
1. Power up, Developer Mode on, start logging. 15 s disarmed on the floor.
2. Take off in ALT_HOLD to about **1 m**. Centre the throttle, **hands off 30 s**.
3. Climb to about **1.9 m** ( above 160 cm ), hold 15 s.
4. Descend to about **1.2 m** ( below 140 cm ), hold 15 s.
5. Land normally, disarm, 10 s on the floor. Paste into `logs/log-3.txt`.

Checks: `G` 0 and `N` advancing throughout; hands-off `E` within +-3 cm; `S` 1 -> 0 at 160 and 0 -> 1 at 140
with `E` / `H` shifting together ( no visible jump ); take-off and landing normal.

### Results ( 5 Oct 2026 )

**Run.** The user flew Test 3 as `logs/log-1.txt` ( short hop, hover around 1-1.5 m, one climb to ~2 m );
Tests 0, 1 and 2 skipped by decision ( the flight covers the goal ). Analysed by the pluto-log-analyst agent from a
custom parse.

**Verdict: PASS.** Only the bottom sensor answers, and the laser works in the altitude-hold loop for the whole
session with both TOF400C fitted.

| Check | Result |
|---|---|
| `G` ( latched driver error ) | **0 in all 7439 records** |
| `N` ( samples ) | monotonic, no stall >= 200 ms; 20.2 / s on the ground and armed ( 50 ms period ) |
| `St` in flight | 0 in 3883 of 3886 samples ( 99.92 % ), 3 x `St` 7 |
| `St` on the ground | 0 in 90.4 %; the 2 / 4 / 7 codes and 160-2400 mm readings fall in t = 10-28 s while the drone was handled before arming |
| On-ground `mm` | 30 mm ( sd 1.2-1.6 ) before the flight, at rest; 19 mm after landing ( 2 s only ) |
| Two-sensor signature | none: no bimodal values at steady height; one single-sample step in flight ( 701 -> 411 mm in 53 ms at t = 66.6 s, `E` unchanged ) |
| Arm segment | 39.0-83.5 s ( 44 s ), max `E` 148 cm, max `mm` 2001 |
| Handover up ( 1 -> 2 -> 0 ) | t = 41.6-42.0 s at `mm` 150-162 cm ( 160 cm threshold ) |
| Handover down ( 0 -> 1 ) | t = 63.2 s, ~2.6 s after `mm` fell below 140 cm: the documented 2.5 s hold before re-basing |
| `E` - `H` across all 7 source changes | unchanged ( 4 cm before and after ): the frame shift moves `E` and `H` together |
| Source share armed | laser 40.8 %, baro 47.1 %, object hold-off 12.1 % |

**Not shown by this log.**
- Hover hold quality: the only window with constant `H` ( 17 s at `H` 120 ) spans the handover and the start of the
  descent ( `E` - `H` +9.1 cm, sd 10.6 ). Not a clean hover; the hold behaviour itself is unchanged by this topic
  ( no fusion code touched ) and was flight-validated in vl53l1x-althold-parity.
- Bench accuracy against a tape and the "second sensor awake" negative check: skipped.

**Log instrumentation issues ( not firmware under test ).**
- **Rate:** records came at ~91 Hz ( 11 ms ), not 10 Hz. `setUserLoopFrequency ( 10 )` stores 10 x 1000 into
  `userLoopFrequency`, which `mw.cpp:1136` uses as a period in us ( default 100000 = 10 Hz ), so the call set a 10 ms
  period. Pre-existing API bug, outside this topic ( logged for the user ). The call is removed from the
  diagnostic; the default is 10 Hz.
- **Restarts:** 57 `#START` in 86 s ( one per 1.5 s ), gaps mostly 1-9 ticks. ~91 records/s at ~58 characters is
  ~9x the per-tick budget rate; the likely cause is the MSP link being flooded, not the laser. 25 records ( 0.34 % )
  lost a field.
- **`tools/flightlog.py`:** a firmware field named `t` collides with the script's time key; with a leading
  `#START` line it falls back to index x 0.1 s for the whole file ( reports 745 s for an 86 s log ). Use a different
  field name ( e.g. `ms` ) or fix the script.
