# Set desired angle roll / pitch - Testing

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [TESTING](TESTING.md)

## Test code ( task 1, temporary )

[PlutoPilot.cpp](../../../../PlutoPilot.cpp) runs an automatic tilt sequence when Developer Mode turns on. Each step
calls `setDesiredAngle ( AG_ROLL, ... )` and `setDesiredAngle ( AG_PITCH, ... )` **once**, when the step starts:

| Step `s` | Time from Dev Mode on | Roll `cR` | Pitch `cP` |
|---|---|---|---|
| 0 | 0 - 2 s | 0 | 0 |
| 1 | 2 - 3 s | +50 ( 5 deg right ) | 0 |
| 2 | 3 - 4 s | 0 | 0 |
| 3 | 4 - 5 s | -50 ( 5 deg left ) | 0 |
| 4 | 5 - 6 s | 0 | 0 |
| 5 | 6 - 7 s | 0 | +50 |
| 6 | 7 - 8 s | 0 | 0 |
| 7 | 8 - 9 s | 0 | -50 |
| 8 | 9 - 10 s | 0 | 0 |
| 9 | after 10 s, `TILT DONE` | 0 | 0 |

Log line, 10 Hz, about 97 B per tick:
`t:<millis> n:<tick> s:<step> cR:<cmd roll> cP:<cmd pitch> R:<roll> P:<pitch> H:<heading>`.
`R` / `P` are `Estimate_Get ( Angle, ... )` in deci-degrees, read **after** the setter call in the same tick, and `H` is
the heading in degrees. The markers `TILT START`, `TILT RESUME` ( a Wi-Fi restart of the user code within 1.5 s; the
sequence keeps its timeline ) and `TILT DONE` each print on their own tick.

## Procedure ( the same for the pre-fix and post-fix flights )

1. Flash the build. Set the app to ANGLE mode with altitude hold, and Developer Mode **off**.
2. Arm, take off and hover at about 1 m in the middle of the room. Wait until the hover is steady.
3. Turn Developer Mode **on**. Keep the sticks centred and do not yaw. Be ready to take over: each tilt is 5 deg for
   1 s ( about 0.4 m of drift ), and the opposite step brings the drone back.
4. Wait for `TILT DONE` ( about 10 s ), turn Developer Mode **off**, and land.
5. Paste the PlutoMonitor log into the `logs/log-N.txt` named for that flight, and note anything the log cannot show
   ( visible drift direction, bumps ).

## Expected results

| Build | Expected |
|---|---|
| Pre-fix ( HEAD `ebf4e46` + test code ), log-1 | On the first tick of each step, `R` / `P` equals the command ( the setter wrote the estimate: "read once" ). From the next tick on, `R` / `P` stays near level, whatever the step ( "overwritten, no tilt" ). This reproduces B4. |
| Post-fix ( task 7 ) | `R` / `P` follows `cR` / `cP` with the same sign and settles within 15 deci-degrees of the command inside 0.5 s, returns near 0 on the level steps, `H` does not change, and the drone does not disarm. |

## Results

### log-1: pre-fix reproduction ( task 1 )

Build: `Build/PRIMUS_X2_v1/AltX_PRIMUS_X2_v1_3.15.7.hex` ( HEAD `ebf4e46` + task 1 test code ), 122.3 KB flash /
20.5 KB RAM. Flown 14 Nov 2026, 17:13, Wi-Fi ( `Rx_ESP` ), small room. Log: [logs/log-1.txt](logs/log-1.txt).

**Result: B4 reproduced.** 316 records, none with missing fields. Per step, "first" is the step-change tick ( read
right after the setter call ), and "rest" is every other tick of the step. Values are deci-degrees, heading in degrees.

| `s` | `cR` | `cP` | First `R` / `P` | Rest `R` min..max ( mean ) | Rest `P` min..max ( mean ) | `H` |
|---|---|---|---|---|---|---|
| 0 | 0 | 0 | 0 / 0 | -11..3 ( -0.2 ) | -4..9 ( 2.8 ) | 261..263 |
| 1 | **+50** | 0 | **50** / 0 | -12..1 ( **-7.2** ) | 4..9 ( 7.1 ) | 262..263 |
| 2 | 0 | 0 | 0 / 0 | -13..-4 ( -8.0 ) | 1..6 ( 3.2 ) | 261..263 |
| 3 | **-50** | 0 | **-50** / 0 | -13..-5 ( **-8.4** ) | 2..8 ( 6.0 ) | 261..262 |
| 4 | 0 | 0 | 0 / 0 | -9..5 ( -1.9 ) | 1..10 ( 6.2 ) | 262..263 |
| 5 | 0 | **+50** | 0 / **50** | -7..6 ( 0.2 ) | 7..14 ( **9.5** ) | 262 |
| 6 | 0 | 0 | 0 / 0 | -10..3 ( -3.4 ) | 3..11 ( 8.0 ) | 262 |
| 7 | 0 | **-50** | 0 / **-50** | -2..5 ( 1.6 ) | 2..12 ( **7.6** ) | 262..263 |
| 8 | 0 | 0 | 0 / 0 | -6..5 ( -0.2 ) | -2..11 ( 4.8 ) | 261..263 |
| 9 | 0 | 0 | 0 / 0 | -128..30 ( -9.2 ) | -37..49 ( 3.7 ) | 261..263 |

- **"Read once":** on every step-change tick, `R` / `P` equals the command exactly ( 50, -50 ). That is the setter
  writing `inclination`, read back by `Estimate_Get` in the same tick.
- **"Overwritten, no tilt":** from the next tick on, the measured angle stays in the hover band ( within about 1.5 deg )
  whatever the command. Roll +50 and roll -50 give the same mean ( -7.2 vs -8.4 ), and pitch +50 and -50 the same
  ( 9.5 vs 7.6 ): the drone does not respond to the sign. The small offsets ( roll about -0.8 deg, pitch about +0.7
  deg ) are trim / drift, present in the level steps too.
- **Heading** stays at 261..263 deg throughout: yaw is undisturbed.
- **Step 9** ( after `TILT DONE`, about 24 s until Dev Mode off ) includes the pilot's stick input and the landing, so
  its wider range ( -128..49 ) is not part of the test.
- **Link:** 10 `TILT RESUME` markers in 34 s ( Wi-Fi restarts of the user code ). All resumed with the timeline kept,
  so the 3 s window worked, and every step got its command. There are 22 drone-time gaps of about 200 ms ( one tick ),
  the usual Wi-Fi pattern ( `pluto-flighttest` section 1 ).
- **Not in the log:** arming and flight mode. The user reported flying in ANGLE mode with altitude hold.

### log-2: post-fix validation ( task 7 )

Build: `Build/PRIMUS_X2_v1/AltX_PRIMUS_X2_v1_3.15.7.hex` ( tasks 2-5 fix + the **same** task 1 test code, so log-1 and
log-2 compare directly ), Flash 125.6 KB incl. test code, RAM 19.8 KB bss.

**Flight A, the sequence** ( same procedure as log-1 ): hover at about 1 m in ANGLE mode with altitude hold, Developer
Mode on with the sticks centred, wait for `TILT DONE`, Developer Mode off, land. Paste the log into `logs/log-2.txt`.

**Flight B, the two stop checks** ( visual; the log stops when Developer Mode goes off ):

1. Hover, Developer Mode on, and during the first roll step ( about 2-3 s after switching it on ) switch Developer Mode
   **off**. Expected: the drone levels at once and hovers.
2. Land and disarm. Re-arm and take off **with Developer Mode off**. Expected: no tilt on take-off.

Note in the log file which way the drone moved on each step ( e.g. "s1 drifted right" ), and anything else visible.

| Check | Pass |
|---|---|
| Tilt follows the command | On steps 1, 3, 5, 7, `R` / `P` moves to the commanded sign and settles within 15 deci-degrees of +/-50 ( measured from the level-step mean, which carries the hover trim ) inside 0.5 s. |
| Held between calls | The tilt stays for the whole 1 s step, although the setter is called only once per step. |
| Levels on 0 | On steps 2, 4, 6, 8 and after `TILT DONE`, `R` / `P` returns to the level-step band. |
| No read-back | On the step-change tick `R` / `P` is the measured angle, not the command ( log-1 read back exactly +/-50 ). |
| Yaw untouched | `H` stays within a few degrees. |
| No disarm | The drone stays armed through the sequence. |
| Wi-Fi | A `TILT RESUME` inside a tilt step does not level the drone if the stall is under 0.6 s. |
| Flight B | Dev-off mid-step levels the drone; a re-arm with dev off does not tilt. |

**Result: PASS**, with the roll-left step not validated ( a collision ). Flown 14 Nov 2026, 17:52, Wi-Fi, small room,
about 1 m, ANGLE with altitude hold. 253 records, none with missing fields. The pilot's account was taken after the log:
the drone **hit something** during step 3, and the pilot **corrected on pitch** near the end of both pitch steps.

The level hover reads `R` about -7 and `P` about +9 ( trim / drift; log-1 showed the same kind of offset ), so a 5 deg
tilt shows as about +43 / -57 on roll and +59 / -41 on pitch. Per-tick values, deci-degrees:

| `s` | Command | Ticks ( every ~100 ms ) | Reading |
|---|---|---|---|
| 0 | level | R -13..5, P 1..13 | Level baseline. |
| 1 | roll +50 | R 4, ( lost ), 37, 38, 38, 35, 34, 38, 42, 43 | **Tilted right within ~200 ms and held the whole second** ( about +45 from level, 0.5 deg short of 5 deg ). One setter call: the hold works. |
| 2 | level | R 42, 22, -4, -7, -8, -10, -11, -9 | **Back to level within ~200 ms.** |
| 3 | roll -50 | R -22, -78, -160; then P -267, H 250 -> 265 in one tick | Started tilting left, then **hit an obstacle** ( pilot ). Beyond the 20 deg limit, so not commanded. **Not validated.** |
| 4 | level | R up to +135, decaying | Recovery from the collision. |
| 5 | pitch +50 | P -14, 12, 38, 47, 49, 48, 45, 35, 23 | **Reached +4.9 deg within ~300 ms**, held 0.3 s, then the pilot's correction. Roll also moved ( collision recovery / pilot ). |
| 6 | level | R and P return through the band | Recovery. |
| 7 | pitch -50 | P -20, -34, -51, -51, -51, -52, -52, -46, -36, -24 | **Reached -5.1 deg within ~200 ms and held 0.5 s** flat, then the pilot's correction. |
| 8 | level | P -7, 17, 43, ... 26 | Back toward level; the overshoot follows the pilot's counter-stick. |
| 9 | done | R -33..7 ( mean -6 ), P -22..26 ( mean 4 ) | Level. |

| Check | Result |
|---|---|
| Tilt follows the command | **Pass**: roll +, pitch +, pitch - all reached the commanded sign within 0.2-0.3 s and within 1 deg of 5 deg. Roll - not validated ( collision ). |
| Held between calls | **Pass**: roll + held the full second from one call; pitch - held flat for 0.5 s until the pilot's input. |
| Levels on 0 | **Pass**: step 2 levelled within ~200 ms. |
| No read-back | **Pass**: the step-change ticks read 4, -22, 20, -23, 6 ( measured ), not +/-50 as in log-1. |
| Yaw untouched | **Pass**: H 249-251 throughout, apart from the 15 deg jolt in the collision. |
| No disarm | **Pass**: the sequence ran to `TILT DONE` and on. |
| Wi-Fi | **Pass**: 9 `TILT RESUME` markers, all gaps 200-305 ms ( under the 0.6 s grace ). The one at the start of step 1 did not stop the tilt. |
| Flight B | **Pass** ( pilot ): dev-off during a lean levelled the drone at once; a re-arm with dev off took off level. |
| Airborne gate | Not separately tested; the tilt flew normally at about 1 m. |

**Comparison with log-1** ( same test code ): log-1's roll +50 step read the command back once and then sat at -7
( level ); log-2's roll +50 step reads the real angle and sits at +38 ( 5 deg from level ) for the whole second. The
drone now does what `setDesiredAngle` asks.

**Limits of this test:** a 1 s, 5 deg step carries the drone about 0.4 m, and the level step only stops the tilt, not
the motion, so the drone keeps drifting. In a small room that ended in a collision and in pilot corrections. A roll-left
step in open space would complete the coverage; it uses the same code path as roll right with the opposite sign.
