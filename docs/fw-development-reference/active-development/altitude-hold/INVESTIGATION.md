# Altitude Hold: Investigation

[README](README.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

**Platform:** MagisV2 firmware, Pluto. August work on PRIMUS_X2_v1, September validation on PRIMUS_V5. Branch `BugFix-June26`.
**Period:** 19 August to 17 September 2026
**Status:** Altitude hold working. All identified causes fixed or compensated. Validated on one airframe. Committed as firmware 3.7.0.

---

## Summary

Altitude hold was sinking: the drone descended steadily while the flight log
reported a stable altitude. The control loop was never the fault. The
barometer's reading moved during flight and the controller faithfully followed
it down.

The investigation found **six causes**. Four were firmware bugs and are fixed.
Two were sensor effects, throttle and temperature, and are compensated. The one
cause left open in August, an "unexplained" steady drift, turned out to be the
same temperature effect, hidden by how it was being logged.

**Result:** in a 4 minute hover from a cold start, checked against a laser
rangefinder, the drone now holds its height within about **±8 cm with no steady
drift**. Before, a 2 minute hover lost up to 100 cm.

---

## Before and after

### August, before the fixes ( laser as ground truth )

| Time | Reported altitude | Actual height (laser) | Error |
|-----:|------------------:|----------------------:|------:|
| 8 s   | 109 cm | **96 cm**  | 13 cm |
| 40 s  | 114 cm | **80 cm**  | 34 cm |
| 72 s  | 120 cm | **75 cm**  | 45 cm |
| 120 s | 118 cm | **18 cm**  | 100 cm |

About **50 cm per minute** of sink, invisible to the aircraft.

### September, after the fixes ( PRIMUS_V5, laser as ground truth )

| Flight | Hover | Board warming during hover | Height trend |
|---|---|---|---|
| Temperature correction 1.9 ( August value ) | 3 min | +3.2 °C | −3.9 cm/min |
| Correction 2.2 | 3.7 min | +4.0 °C | −2.5 to −3.7 cm/min |
| **Correction 2.1 ( final ), cold start** | **4.1 min** | **+5.2 °C** ( +8.0 °C from arming ) | **no steady trend after settling, ±8 cm** |

Without the correction, the barometer on its own drifts about **+16 cm per
minute** as the board warms.

---

## Root causes

### 1. In-flight reference reset ( firmware bug, fixed )

Pulling the throttle stick to the bottom while armed silently re-zeroed "ground"
to the drone's current altitude. A throttle chop at 2 m left every later reading
2 m wrong. Confirmed in logs as a +255 cm jump. The most likely explanation for
field reports of altitude hold behaving erratically, since dropping the throttle
is a normal pilot action.

### 2. Estimator running 2.5× too slow ( firmware bug, fixed )

A 30 degree tilt threshold was written as 3 degrees ( a units error ), so the
altitude estimator ran permanently in its slow mode: about 15 s to settle instead
of about 6. A slow estimate lagging above a descending aircraft asks for more
descent.

### 3. Controller losing half its authority ( firmware bug, fixed )

A deadband meant to ignore tiny errors actually subtracted 5 cm from every error,
so a 10 cm sag was seen as 5 cm. Together with a low outer-loop gain, small sags
were never corrected. The gain was also raised to its intended value.

### 4. Hover throttle trapped in a limited integrator ( firmware bug, fixed )

The throttle needed to hover was stored in an integrator capped at a fixed range.
As the battery sagged the drone needed more throttle than the cap allowed and
sank with the controller at its limit. The hover throttle now moves into the
baseline gradually, keeping the integrator free for control.

### 5. Throttle dependence ( measured, compensated )

The flight controller sits in the rotor airflow: more thrust lowers the pressure
it reads, by **8.6 Pa per 1000 throttle counts** ( ~14 cm over a typical flight ).
Compensated. Measured in August; not re-verified in September.

### 6. Temperature dependence ( re-measured, compensated )

The barometer reads lower pressure as the board warms from motor current and its
own heating: **−2.17 to −2.42 Pa per °C**, seven independent measurements, about
**18 cm of apparent climb per °C**. Flights so far warmed the board by 3.5 to 8 °C from arming.

August measured this as −1.9 Pa per °C and could not explain a further steady
drift. The temperature was then being logged in **whole degrees**, which hides up
to a degree of warming, about 2 Pa. With unrounded temperature logged, that
"unexplained drift" matched the temperature effect.

---

## The August open question, answered

August asked whether the leftover drift came from **room pressure changes** or
from **the barometer itself**, and proposed adding a second barometer to find out.

This was done. A BMP280 was logged alongside the ICP-10111:

* The two sensors react to temperature in **opposite directions** ( ICP −2.4 Pa/°C,
  BMP +2 Pa/°C ). One steady room pressure explained both sensors at once over
  7 minutes, which would not happen if the room pressure had been moving.
* **Conclusion: the drift is in the barometer and caused by temperature, not the
  room.** That makes it correctable in firmware, which has now been done. No
  hardware change or per-unit factory calibration is needed for this.

The STM32's internal temperature sensor was also tested as an alternative
reference. The barometer's own temperature reading predicted the drift better,
so the STM32 sensor was not adopted. Both test sensors have been removed from the
firmware.

---

## Changes made

| Change | File | Effect |
|--------|------|--------|
| Guard the ground re-zero so it cannot fire in flight | `mw.cpp` | Removes metre-scale step errors |
| Tilt threshold 3° → 30° | `flight/altitudehold.cpp` | Estimator settles in ~6 s instead of ~15 s |
| Remove subtracting deadband, outer-loop gain to unity | `flight/altitudehold.cpp`, `config/config.cpp` | Small sags now corrected |
| Hover throttle moved out of the capped integrator | `flight/altitudehold.cpp` | Holds height as the battery sags |
| Throttle and temperature compensation ( 2.1 Pa/°C, 25 Pa limit ) | `sensors/barometer.cpp` | Removes ~16 cm/min of thermal drift |
| Fixed-scale pressure-to-altitude conversion, zero kept current until arm | `sensors/barometer.cpp` | Same height on every flight |
| Barometer temperature no longer frozen at power-on | `drivers/barometer_icp10111.cpp` | Sensor's own correction stays valid |
| Calibration constants read as signed | `drivers/barometer_icp10111.cpp` | Correctness fix |

Builds with no new compiler warnings. Flash 101.3 KB of 256 KB, RAM 14.8 KB of
40 KB. **Saved settings reset on the first boot after flashing** ( configuration
version change ).

The temperature correction is set slightly below the measured average on purpose:
if it is off, the drone sinks gently rather than climbs.

---

## What is not yet proven

* **One airframe.** All September numbers come from a single PRIMUS_V5. The
  throttle effect in particular depends on where the board sits relative to the
  rotors. One flight on a second drone confirms or corrects both.
* **Indoors only, flights up to about 4 minutes.** Outdoor wind and sun, and full
  battery-length flights, are untested.
* **The throttle coefficient** was not re-checked in September.
* **The correction's safety limit may be too tight for long cold flights.** The
  correction is capped at 25 Pa so a sensor fault cannot move the altitude
  reference more than about 2 m. The 4 minute cold-start flight already used
  71 % of that. A full-battery cold start could reach the cap, after which the
  drone would start sinking again. If a longer test shows this, the cap should be
  raised to 30 to 35 Pa.

---

## Remaining improvements ( not blocking )

* A **10 to 13 cm dip about 30 s after takeoff** recurs across flights, then
  recovers. Cause not yet identified.
* The barometer reading is **noisy** ( about ±8 cm, doubled in flight by rotor
  airflow ). The controller works from a filtered estimate, so this is not the
  main issue, but a foam cover over the sensor and the sensor's low-noise mode
  would reduce it.
* A low-priority user-facing issue: the stick command meant to re-zero the
  barometer does nothing.

---

## Status at commit

* Committed as firmware 3.7.0 after a clean build of all three board targets.
  The diagnostic logging used for testing was kept out of the commit.
