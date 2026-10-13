# Crash Detect False Trigger - Investigation

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

## Problem

In most flights on PRIMUS_X2_v1, the app shows **crashed** and the drone disarms ( the motors stop ) after a
**light bump or collision** that it could have flown through, and on **hard or bumpy landings**. The only
evidence so far is the "crashed" flag in the controller app; there are no logs yet.

## How the detector works

`failsafeOnCrash ( )` ( [failsafe.cpp:362](../../../../src/main/flight/failsafe.cpp#L362) ) runs on every
control loop ( 3.5 ms, called from [mw.cpp:1424](../../../../src/main/mw.cpp#L1424) ). While the drone is
ARMED, `fsCrash` is on ( default ) and it is in ANGLE_MODE, **one sample** with any of six conditions true
sets the `Crash` status and calls `mwDisarm ( )`:

| Term | Before | Final ( task 5 ) | Meaning |
|---|---|---|---|
| \|roll\| or \|pitch\| | 700 dd ( 70° ) | unchanged | attitude upset |
| \|`accSmooth` X/Y\| | 1250 LSB ( 0.305 g ) | 5000 LSB ( 1.22 g ) | filtered lateral accel ( IIR, factor 8 ) |
| \|`accADC` X/Y\| | 1750 LSB ( 0.427 g ) | 8000 LSB ( 1.95 g ) | raw lateral accel ( chip DLPF ~5.7 Hz ) |

`acc_1G` = 4096 LSB ( ICM20948, ±8 g ). There is no debounce. The detector is bypassed during a flip
( `flipState >= 1` ) and is not active in ACRO. The flag clears ~300 ms after disarm once the craft is level
( [mw.cpp:484-496](../../../../src/main/mw.cpp#L484) ).

History: commit `49a2fdb` ( 5 Jun 2026 ) added the two accel limits at 1250 / 1750. Before it, the only
accel rule was `accSmooth > 12000` ( ~2.9 g ), so the detector became about 10× more sensitive to lateral g.

## Hypotheses ( ranked )

1. **One raw `accADC` sample over 0.43 g on contact.** A bump or a firm touchdown is a short lateral spike;
   one 3.5 ms sample is enough. *Test:* the diagnostic build latches the term that fired and its peak.
2. **Gravity while tilted and touching something.** Resting on or pushing against something, X/Y reads
   g·sinθ: the old `accSmooth` limit was reached at ~17.8° of tilt, the `accADC` one at ~25.3°. At the final
   1.22 / 1.95 g, tilt alone can never reach them. This fits bumps and bouncy landings. *Test:* bench tilt at low
   throttle; diagnostic log at trip.
3. **Accelerometer calibration bias.** A 3° calibration error is ~214 LSB on X or Y, 17% of the old 1250
   margin. *Test:* level and disarmed on the bench, `Sensor_Get ( Accelerometer, X/Y )` above ~50 cm/s²
   supports it.
4. **The 70° angle rule.** Unlikely: commanded angle is limited to 20°.

Not proven yet: which term fires, and how far over the threshold the peaks go. A percentage change only
helps if the peaks sit within that percentage of the threshold that is firing.

## Q&A that shaped the plan ( 8 Oct 2026 )

- **Mode:** planned as `check`, but the user wants the impact sensitivity reduced by **15%** now ( task 1 ),
  with the investigation planned around it, so the topic is `fix`. ( The first request said 5-10%, by feel. )
- **Target:** PRIMUS_X2_v1.
- **When it trips:** light bumps or collisions, and hard or bumpy landings, in most flights.
- **Does it disarm?** Yes, every time the flag shows.
- **Which landing trips count as false:** all of them.
- **Done when:** no false trigger in flights.
- **Instrumentation:** a temporary debug build is allowed and is reverted before close.
- **Fix scope:** anything the logs justify ( per-term thresholds, persistence or debounce, restoring the pre-`49a2fdb` rule ).
- **Out of scope:** flip logic, API, MSP or app changes, and landing or touchdown detection code.

## Findings ( task 5, 13 Oct 2026 )

Evidence: [TESTING.md](TESTING.md), tests 1 and 2. That is log-1..4, 15 armed segments, PRIMUS_X2_v1.

**Cause, confirmed.** The false disarms are the lateral accel terms firing on the **contact spike** of a bump:
one sample of 0.9-1.3 g raw against a limit of 0.43 g raw / 0.305 g filtered.

| Hypothesis | Verdict | Evidence |
|---|---|---|
| 1. Contact spike over the accel limit | **confirmed** | log-1: all 4 bumps fired the raw term ( T 4 ) at 3616-5409 LSB, with a tilt of ≤ 14° |
| 2. Tilt under gravity while touching | ruled out | ≤ 14° tilt gives ≤ 990 LSB, far below the peaks |
| 3. Calibration bias | ruled out | ~60 LSB ( 0.8° ) level before arming |
| 4. 70° rule | not a false-trip cause | it fired only on real flips ( log-2 ×2, log-4 ×1, log-1 crash ) |

**Why percentage tuning failed.** Survivable bumps peak at **≤ 5409 raw / 3100 filtered**, which is 3.1× / 2.5×
the original limits. Every step tried, +15%, a further +20% and the user's 2500 / 3000, stayed below that, so
every bump still disarmed.

**The key observation.** The contacts that flipped the craft peaked no higher ( ≤ 4970 raw ) than the bumps it
survived. The accel peak cannot tell the two apart; the outcome can. The 70° rule caught every flip within
0.1-0.35 s. So the accel terms only need to catch a **hard** hit fast, before the tumble. log-1's crash read
24125 raw / 12522 filtered on its first line.

**Decision ( user, 13 Oct ).** Final limits **`accSmooth` 5000 ( 1.22 g ) / `accADC` 8000 ( 1.95 g )**, 70°
unchanged, which are the values flown in test 2. The user tried 4000 / 7000 ( 0.977 / 1.71 g, margin 1.29× )
on 13 Oct and then reverted to 5000 / 8000 by hand before validation:
- margin over the hardest survivable bump ( 3100 filtered / 5409 raw ): **1.61× / 1.48×**;
- the only hard crash ( first line 12522 / 24125 ) is 2.5× / 3.0× over;
- flown in log-2..4 with no false trip and all 3 flips disarmed;
- both limits are above 1 g ( 4096 ), so tilt alone can never trip them.

**Landing.** 8 of 8 landings in log-3, by `LAND` and by hand, ended cleanly without the crash path. They were all
gentle ( ≤ 0.21 g ), so firm or bouncy landings, the original complaint, are still untested; task 8 must
include them.

**Accepted gaps ( user, 13 Oct ).** The following are not caught by any accel limit or by the 70° rule:
- a craft pinned upright against a wall;
- a ceiling hit or an upright drop ( Z is not checked );
- a craft caught in a curtain or by a hand;
- a craft resting tilted at 20-69°.

The pilot's ARM switch covers them, as it does outside ANGLE mode. Follow-up: Planned topic
[crash-stuck-detection](../crash-stuck-detection/README.md).
