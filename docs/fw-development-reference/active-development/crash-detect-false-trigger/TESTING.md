# Crash Detect False Trigger - Testing

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

## Test 1 - log-1: first flight with the diagnostic build ( 12 Oct 2026 )

**Build.** Task 2 diagnostics, built by the user in PlutoIDE ( PRIMUS_X2_v1 ). Limits 2500 / 3000 LSB
( `accSmooth` / `accADC`, 0.610 / 0.732 g ) and 70°. **`set_FSI ( Crash )` and `mwDisarm ( )` were
commented out**, so every event below is a "would have disarmed" and nothing actually disarmed.

**Log.** [logs/log-1.txt](logs/log-1.txt): 517 records, one every ~105 ms, 52.6 s; armed from 9.9 s to
42.9 s. Fields: `aS` / `aA` = largest |X|, |Y| since the previous line ( LSB, 4096 = 1 g ); `R` / `P` =
roll / pitch at the print ( deci-degrees ); `T` = terms of the last event ( 1 angle, 2 `accSmooth`, 4 `accADC` );
`N` = events since power-up. Parsed with a throwaway script; it left no line unparsed.

**What the user did** ( confirmed 12 Oct ): events 1-4 were **deliberate light bumps, lighter than the bumps
that trip in normal use**, so they must not disarm. At 40.5 s there was an **accidental real crash**, which must disarm.

### Before arming ( level, disarmed, 0-9.9 s )

| | X | Y |
|---|---|---|
| `accSmooth` | 53-65 LSB ( 0.014 g ) | 47-59 LSB ( 0.012 g ) |
| `accADC` | 56-77 LSB | 46-118 LSB |
| roll / pitch | 0.7° | 0.8° |

Calibration offset is about 0.8°, about 60 LSB, which is 2-5% of any limit in use. **Hypothesis 3
( calibration bias ) is ruled out.**

### Events 1-4: isolated, short, at low tilt ( "false" candidates )

| # | Time | T | `aS` X,Y | `aA` X,Y | R / P at the print | Length |
|---|---|---|---|---|---|---|
| 1 | 20.05 s | 4 ( raw ) | 2000, 1737 | 3616, 2640 | −10.5° / −8.4° | 2 lines ( ~200 ms ), quiet before and after |
| 2 | 26.17 s | 4 | 2323, 1207 | 4569, 2289 | −6.0° / −9.6° | 2 lines |
| 3 | 31.59 s | 4 | 2275, 1332 | 4070, 2177 | −0.6° / 3.3° | 2 lines |
| 4 | 37.74 s | 4 ( then `aS` 3080 on the next line ) | 2285, 893 | 5409, 2513 | −0.2° / −3.2° | 2 lines |

- Peak raw `accADC`: **3616-5409 LSB ( 0.88-1.32 g )**, mostly on X. Peak filtered `accSmooth`: **2000-3080 LSB
  ( 0.49-0.75 g )**.
- Tilt stays under ~14°. Gravity at that tilt gives ≤ 990 LSB, far below the peaks, so **hypothesis 2
  ( tilt under gravity ) does not explain these events. Hypothesis 1 ( a short contact spike ) does.**
- Against the earlier limits:
  - original 1250 / 1750: both terms trip on every event;
  - task 12's 1726 / 2416: both trip;
  - the current 2500 / 3000: the raw term trips on all four, and the filtered term on event 4.

### Normal flight between the events

Quiet armed flight, excluding the two lines of each event: `aS` ≤ 1044 LSB ( 0.25 g ), `aA` ≤ 1917 LSB
( 0.47 g ), tilt ≤ 15°. The busiest quiet line, at 17.51 s, is `aS` 1044, 795 and `aA` 1917, 1294.

### Events 5-18: crash and tumble ( 40.5-42.9 s, then disarm )

- First line, 40.52 s: `aS` Y 12522, `aA` Y 24125 ( 5.9 g ), R 64°.
- Within 250 ms: R −81°, P −73° ( angle term, T 7 ). Largest values: `aA` Y 30138 ( 7.4 g ), `aS` Y 15648
  ( 3.8 g ), roll 157°.
- N counts 14 events in 2.4 s, because the craft keeps re-crossing the limits as it tumbles. With the disarm
  in, the first one would have ended the flight.
- After disarm ( 42.9 s onward ): `aS` up to 4490 and tilt up to 88°. The craft is being handled; the detector
  is not active because the craft is disarmed.

### Separation in this log

| | Raw `accADC` peak | Filtered `accSmooth` peak | Tilt |
|---|---|---|---|
| Quiet flight | ≤ 1917 ( 0.47 g ) | ≤ 1044 ( 0.25 g ) | ≤ 15° |
| Events 1-4 | 3616-5409 ( 0.88-1.32 g ) | 2000-3080 ( 0.49-0.75 g ) | ≤ 14° |
| Crash, first line | 24125 ( 5.9 g ) | 12522 ( 3.1 g ) | 64° → 81° within 250 ms |

This is one log, with four bumps and one crash. The bumps and the crash are separated by a factor of ~4.5 in
raw peak. **That is an upper bound:** these bumps were lighter than typical ones, so real-world bumps sit
above 1.32 g raw and the real gap is narrower. Task 4 needs typical-strength bumps before task 5 picks a limit.

## Test 2 - task 4 plan: typical bumps with the disarm on ( 12 Oct 2026 )

**Build.** Task 2 diagnostics, **disarm restored**, test-candidate limits **`accSmooth` 5000 ( 1.22 g ) /
`accADC` 8000 ( 1.95 g )**, 70° unchanged. Gate: no new `src/` warnings, flash 101.7 KB, RAM 14.8 KB.
Flash this build in PlutoIDE ( PRIMUS_X2_v1 ) before flying.

**Expected, if 5000 / 8000 is a workable limit:**
- **Confirms it:** typical bumps stay under 5000 / 8000 and do not disarm ( `N` does not move ). The
  deliberate crash disarms on its first line.
- **Refutes it:** a typical bump disarms ( `N` goes up, `Arm` drops to 0 ), or the deliberate crash does not
  disarm.
- **Narrows the window:** typical bumps land within ~25% of the limit ( raw above ~6000 ). There is then too
  little margin, and task 5 has to weigh a debounce instead.

**Safety conditions for these flights** ( from the review of the 5000 / 8000 limits ):
- Keep a hand on the ARM switch the whole time.
- ANGLE mode only, with prop guards, over a soft floor, and nobody within reach.
- No ceiling work: vertical impacts are not checked at all.
- Abort rule: if the craft is pinned or rubbing against the wall for more than ~1 s without disarming,
  disarm by hand. That non-trip is itself a result to note.

**Sequence, per flight** ( Developer Mode on, app connected, ANGLE mode ):

1. 20 s disarmed, level ( pre-arm offset ).
2. Arm, take off, hover ~20 s with no input.
3. 3 front bumps into a wall, **as hard as typical in-use bumps**, about 6 s apart.
4. 2 side bumps ( left or right side into the wall ), about 6 s apart.
5. 1 slow drift into the wall ( ~0.2 m/s ). Hold it there ~2 s, then back off. If it does not disarm and
   stays pressed, disarm with the switch.
6. 2 firm landings or touch-and-go.
7. Land, then 20 s disarmed.

**Flight 3 only, at the end:** one **deliberate crash**, from a low height over a soft surface ( push it
hard into the wall or tip it over ). It must disarm by itself.

**Save each flight to its own file:** [log-2.txt](logs/log-2.txt), [log-3.txt](logs/log-3.txt),
[log-4.txt](logs/log-4.txt). Note anything the log cannot record: how long each step took, which bump was
side or front, anything that disarmed unexpectedly.

## Test 2 results - log-2..4 ( 12 Oct 2026 )

**Build:** 5000 / 8000, disarm on, 70°. **What the user flew:** log-2, flying into the wall; log-3, landings
with the LAND command and by hand; log-4, light and hard bumps. Parsed with the same script as test 1:
log-2 had 2 lines that could not be parsed, the others none.

### Per arm segment

| Log | Segment | Max raw `aA` | Max filtered `aS` | How it ended |
|---|---|---|---|---|
| log-2 | 2.9-28.6 s | 4970 ( 1.21 g ) at 28.24 s | 3100 ( 0.76 g ) | **crash, angle**: wall contact at 28.24 s, pitch 43° → 66°, then roll −145° at 28.58 s ( 0.34 s after contact ) |
| log-2 | 38.7-51.0 s | 4711 ( 1.15 g ) at 50.71 s | 2815 ( 0.69 g ) | **crash, angle**: wall contact at 50.71 s, then roll 148° at 51.02 s ( 0.31 s ) |
| log-3 | 8 segments, 7.7-20.8 s each | ≤ 878 ( 0.21 g ) | ≤ 388 ( 0.09 g ) | 8 clean landings ( LAND or manual ), **no crash event** |
| log-4 | 2.9-23.4 s | 4690 ( 1.15 g ) at 19.59 s | 2588 ( 0.63 g ) | **crash, angle**: bumps at 12.9 s ( light, 1720 ) and 19.6 s, survived; a contact at 23.29 s ( 3104 ) then pitch 85° at 23.39 s ( 0.1 s ) |
| log-4 | 33.6-48.6 s | 4301 ( 1.05 g ) at 47.54 s | 2667 ( 0.65 g ) | bump at 47.54 s survived; **manual disarm** 1 s later |

No line in any armed segment went over 5000 raw or over 5000 filtered, so **no accel term fired in log-2..4**.
Every crash disarm came from the 70° rule, 0.1-0.35 s after a contact that flipped the craft.

### All contacts so far ( log-1..4 )

| | Raw `accADC` peak | Filtered `accSmooth` peak | Result |
|---|---|---|---|
| Quiet flight | ≤ 1917 ( 0.47 g ) | ≤ 1044 ( 0.25 g ) | - |
| Gentle landings ( log-3 ) | ≤ 878 ( 0.21 g ) | ≤ 388 ( 0.09 g ) | no trip, even at the original limits |
| Bumps survived ( log-1 ×4, log-4 ×3 ) | 1720-5409 ( ≤ 1.32 g ) | 2000-3080 | must not disarm: none did at 5000 / 8000 |
| Contacts that flipped the craft ( log-2 ×2, log-4 ×1 ) | 3104-4970 ( ≤ 1.21 g ) | ≤ 3100 | disarmed by the 70° rule within 0.35 s |
| Hard crash ( log-1 ) | 24125 ( 5.9 g ) on the first line | 12522 | accel trip on the first line at 5000 / 8000; 70° within 0.25 s |

**What this shows:**
- **The size of the contact does not tell a survivable bump from one that flips the craft.** Both peak at about
  1-1.3 g raw. What separates them is the outcome, the tilt, and that is what the 70° rule watches. An accel
  limit under ~1.3 g raw, which includes every limit from the original 0.43 g up to 3000 / 0.73 g, fires on
  survivable bumps. One at or above ~1.3 g lets them through, and the flips are still caught by angle.
- **Margins at 5000 / 8000:** the raw limit is 1.48× the largest survivable bump ( 5409 ) and the filtered limit
  1.61× ( 3100 ). The only hard crash seen ( 24125 raw ) is 3.0× over the raw limit.
- **Landing:** the LAND command and manual landings both end cleanly without the crash path ( 8 of 8 ).

**What these logs cannot show:**
- **Firm or bouncy landings** ( the original complaint ). All 8 landings were gentle. Task 8 must include firm
  landings.
- **A drone pinned upright against the wall** ( the review's risk ). Every wall contact here bounced or flipped.
- **A deliberate crash with nothing but the accel term to catch it.** Every flip was caught by angle. The
  accel term at 8000 is shown only by log-1's hard crash.

## Test 3 - task 8 plan: validation of the shipped build ( 13 Oct 2026 )

**Build.** The final code, with no diagnostics: limits **5000 / 8000** ( 1.22 / 1.95 g ), 70°, disarm on.
`DEFAULT_PRIMUS_X2_v1_3.11.0.hex`, gate passed ( no new warnings, flash 100.9 KB, RAM 14.8 KB ). Flash it from
PlutoIDE before flying. `PlutoPilot.cpp` is back to HEAD, so **there is no PlutoMonitor log**. The evidence is
the app's "crashed" flag, whether the motors stopped, and the user's notes.

**Focus.** Test 2 already covers wall contacts, flips and gentle landings at these values. This test adds what
it did not show:
1. **Firm or bouncy landings and touch-and-go**: the original complaint, never reproduced so far.
2. **Side ( left or right ) bumps**: hardly tested; the Y axis has the least data.
3. **Typical hard front bumps**, as in normal use.
4. **One deliberate crash** at the end, low over a soft surface. It must disarm by itself.

**Pass ( topic done-when ):** across **at least 5 flights**, no "crashed" flag or disarm on any bump or landing
the drone could have flown through, and the deliberate crash disarms by itself.
**Fail:** any false trip. Note which kind ( landing, side bump, front bump ) and the conditions; the task 2
diagnostics can go back in for one flight to measure it.

**Safety conditions:** as in test 2. Hand on the ARM switch, ANGLE mode only, prop guards, a soft floor,
nobody within reach, no ceiling work. If the drone is pinned against a wall for more than ~1 s, disarm by hand.

**Per flight:**
1. Arm and hover for ~10 s.
2. 2 hard front bumps, then 2 side bumps ( one left, one right ).
3. 2 firm landings, at least one with a bounce, and 1 touch-and-go.
4. Land and disarm.

**Flight 5 only:** finish with the deliberate crash.

**Notes:** one row per flight in [logs/log-5.txt](logs/log-5.txt).

## Test 3 results ( 13 Oct 2026 )

**Reported by the user: flown, all passed.** At 5000 / 8000 there was no "crashed" flag or disarm on any bump or
landing, and the deliberate crash disarmed by itself. The per-flight notes file
[logs/log-5.txt](logs/log-5.txt) was not filled in, so the number of flights and the exact mix of firm landings,
bounces and side bumps are not recorded.
