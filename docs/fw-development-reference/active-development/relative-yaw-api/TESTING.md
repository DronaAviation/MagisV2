# Relative Yaw API - Testing

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

## 1. Diagnostic flight ( task 1 ): why the relative setter waits for a stick nudge

**Build.** PRIMUS_X2_v1, FW 3.12.0 + the uncommitted `DesiredAngle_setRelative`. The test code is in
[PlutoPilot.cpp](../../../../PlutoPilot.cpp) ( temporary, never committed ). It builds clean: flash 107.4 KB,
RAM 17.6 KB.

**What the test code does.** It makes one-shot calls with the sticks untouched. Times count from Developer Mode on:

| Time | `Ev` | Call | Expected target ( `Mh` ) |
|---|---|---|---|
| +5 s | 1 | `DesiredAngle_setRelative ( AG_YAW, +90 )` | `H0 + 90` |
| +13 s | 2 | `setDesiredAngle ( AG_YAW, H0 )` ( control, absolute ) | `H0` |
| +21 s | 3 | `DesiredAngle_setRelative ( AG_YAW, +90 )` | `H0 + 90` |
| +29 s | 4 | `DesiredAngle_setRelative ( AG_YAW, -90 )` | `H0` |

`H0` is the heading at dev-mode switch-on. It is printed on the marker line `S:1 H0:<deg>`.

**Log fields** ( ~105 B per tick, 10 Hz ):

| Field | Meaning |
|---|---|
| `t` | `millis ( )` |
| `n` | tick count |
| `Hd` | measured heading, in degrees |
| `Mh` | `magHold`, the heading-hold target |
| `Us` | `isUserHeadingSet` |
| `Md` | heading-hold mode: 0 none, 1 MAG, 2 HEADFREE |
| `Yw` | yaw stick − 1500 |
| `Yc` | `rcCommand[YAW]` sent to the PID ( ±400 max ) |
| `Ev` | event fired on this tick |

A line with `S:1` marks a real dev-mode switch-on ( the sequence restarts ). A line with `S:2` marks a Wi-Fi RC
flicker that re-ran `onLoopStart ( )` ( the sequence continues ).

### Procedure

1. **Run 1: as you normally fly** ( `#define FORCE_MAGHOLD 0`, the build as it is now ).
   1. Flash it, connect the app and start PlutoMonitor logging.
   2. Note the app's heading-hold / headfree setting.
   3. Arm and take off to a hover of at least 1 m, in open space with room to turn.
   4. With the sticks centred, switch **Developer Mode on** and keep your hands off yaw for **40 s**.
   5. Switch dev mode off, then land.
   6. Optional, if nothing turned: after 35 s, nudge yaw once, let go, and note the time.
   7. Save the log as `logs/log-1.txt`.
2. **Run 2: heading hold forced on** ( set `#define FORCE_MAGHOLD 1` at the top of `PlutoPilot.cpp`, then build and
   flash ). Repeat run 1 exactly and save the log as `logs/log-2.txt`.

**Safety.** The test commands ±90° yaw turns only; throttle and altitude are yours. To take over, deflect yaw past
±300 or switch dev mode off.

### What each hypothesis predicts

| Hypothesis | Run 1 shows | Run 2 shows |
|---|---|---|
| **H1** heading hold off | `Md:0` throughout, `Mh` follows `Hd`, `Yc` ≈ 0 at **both** Ev 1 and Ev 2. Both setters fail. | `Md:1`, and Ev 1-4 all turn. |
| **H2** the user's call pattern | Ev 1-4 all turn ( the one-shot calls work ). The user's own code needs checking. | same |
| **H3** stuck `isMagHoldChanged` latch | `Md` ≥ 1 and `Mh` = target, but `Yc` = 0 while `Hd` ≠ `Mh` for over 1 s | same as run 1 |
| **H4** re-base on a flicker | `S:2` followed by `Us:0`, and the next relative call measured from `Hd` instead of the last target | same |

A turn "works" when `Yc` goes to its limit straight after the event and `Hd` reaches `Mh` ±5° within about 3 s.

### Results ( 15 Oct 2026 )

**Verdict: H1 is confirmed. Heading hold was off in the normal setup, and both setters fail without it.** With
MAGHOLD on, the relative setter works with no stick input. Both logs were parsed per record, with every 9-field
record complete.

**What the two runs looked like**

| | log-1 ( `FORCE_MAGHOLD 0` ) | log-2 ( `FORCE_MAGHOLD 1` ) |
|---|---|---|
| Records / span | 690 / 70.7 s | 646 / 65.7 s |
| Restart markers | one `S:1` only, no `S:2` flicker | one `S:1` only, no `S:2` flicker |
| Heading-hold mode `Md` | **0 on all 690 records** | **1 on all 646 records** |
| Yaw stick `Yw` | 0 through all 4 events; pilot yaw at +50.6 s and +55.5 s ( ±350 ) | 0-3 throughout ( hands off ) |

**What each call did**

| Event | log-1 ( `FORCE_MAGHOLD 0` ) | log-2 ( `FORCE_MAGHOLD 1` ) |
|---|---|---|
| Ev 1: relative +90 ( H0 138 / 147 ) | `Mh` 226 on the call tick, then **136 = `Hd` on the next tick**. No turn. | `Mh` 237. `Hd` 147 → 231 at +1.4 s → 237 at +1.9 s. |
| Ev 2: absolute `setDesiredAngle ( H0 )` | `Mh` 138, then 131 = `Hd`. **No turn: the control fails too.** | `Hd` 236 → 148 at +1.5 s → 147. |
| Ev 3: relative +90 | `Mh` 228, then wiped. No turn. | `Hd` 147 → 235 at +1.6 s → 237 at +3 s. |
| Ev 4: relative −90 | `Mh` 138, then wiped. No turn. | `Hd` 236 → 148 at +1.5 s → 147. |
| Hold between the events | `Hd` wanders 131-134 ( no hold, `Mh` follows `Hd` ) | ±1° of the target for 8 s after each turn, and 147 ±1 for 35 s after Ev 4 |
| What the stick did | Turn only **while the stick is held** ( +50.6 s: `Hd` 134 → 193; +55.5 s: 189 → 128 ). Nothing after release. | - |

- **Turn rate in log-2:** 90° in about 1.5 s, about 60°/s, set by the ±30° error clamp × `P8[PIDMAG]`. Each turn
  ended within 1° of the target with no overshoot beyond 1°, which is within the planned ±5° / 3 s.
- **Repeated calls queue exactly:** Ev 2 → Ev 3 composed on `userHeading` ( 147 + 90 = 237 ), and Ev 4 went back to
  147 with no drift.
- **"It only works with stick input"** is the pilot's own yaw. In log-1 the craft turns only while the stick is past
  ±300, and the target follows the heading. Nothing user code commanded ever took effect.

**Limits of the log**

- `Yc` ( `rcCommand[YAW]` ) is sampled in `plutoLoop ( )`, before `updateMagHold ( )` runs in the same loop
  ( `mw.cpp:1320` → `1344` ). It shows only the stick part ( `Yc` = −`Yw` ), not the heading-hold output. The turns
  in log-2 are proven by `Hd` reaching `Mh`, not by `Yc`.
- log-1 has one gap of 6 ticks ( `n` 698 → 704 ) at about +68 s, during the landing, after all the events. It does
  not affect the result.
- With no flicker in either flight, H4 ( a re-base on a flicker ) was neither seen nor ruled out. It stays a design
  concern for tasks 2 and 4.

| Hypothesis | Verdict |
|---|---|
| H1 heading hold off | **Confirmed.** `Md:0` throughout log-1, and both setters wiped on the next tick. Forcing MAGHOLD ( log-2 ) fixes it. |
| H2 call pattern | Ruled out for these one-shot calls, which work in log-2. The user's own code was not part of this test. |
| H3 stuck `isMagHoldChanged` latch | Ruled out. log-2 turns start on the very next tick after the call. |
| H4 re-base on a flicker | Not seen ( no `S:2` ). Still open as a design concern. |

## 2. Fix check ( task 2 ): automatic heading hold, no app switch

**Build.** PRIMUS_X2_v1 with the task-2 change in `mw.cpp`. The test code is the same as in §1, with
`#define FORCE_MAGHOLD 0`, so user code does not set any flight mode.

**Procedure.** Repeat §1 run 1 exactly:

- the app's heading-hold / headfree setting as you normally fly;
- hover, dev mode on, hands off yaw for 40 s;
- dev mode off, then land.

Save the log as `logs/log-3.txt`.

**Pass**

- `Md:0` before Ev 1 ( no target yet ), and `Md:1` from the tick after Ev 1 to the end of dev mode.
- All four events turn as in log-2: `Hd` reaches `Mh` within ±5° in under 3 s, and holds it.
- After dev mode is switched off, `Md` follows AUX1 again. This shows in the app or in the next flight, because
  user code is no longer printing.

**Fail**

- `Md` stays 0 after Ev 1, or `Mh` snaps back to `Hd` on the next tick ( as in log-1 ).

### Results ( log-3, 15 Oct 2026 )

**Verdict: the fix works.** With AUX1 off and no stick input, every relative and absolute call turned and held.
log-3 contains two Developer Mode sessions ( two `S:1` markers, no `S:2` flicker ). There were 620 records, all
complete, with one lost record ( an `n` step of 2 ).

**Session 1: AUX1 as the user normally flies ( heading hold off ), 0-45 s**

| Event | Target `Mh` | `Hd` |
|---|---|---|
| Ev 1 relative +90 ( H0 128 ) | 218 | 128 → 188 at +1 s → 217 at +1.9 s → 218 |
| Ev 2 absolute `setDesiredAngle ( H0 )` | 131 | 218 → 154 at +1 s → 131 at +2 s |
| Ev 3 relative +90 | 221 | 131 → 192 at +1.1 s → 220 at +2.1 s → 221 |
| Ev 4 relative −90 | 131 | 221 → 158 at +1 s → 132 at +2 s → 131 |
| Hold after each turn | | ±1° of the target ( 131 ±1 for 16 s after Ev 4 ) |

- Compared with log-1 ( the same setup before the fix ), the target is no longer wiped on the next tick. Every turn
  matches log-2 ( MAGHOLD forced from user code ): 90° in about 2 s, with no overshoot beyond 1°.
- **About the `Md:0` samples after Ev 1** ( 127 of the 620 samples are 0 ): this is a sampling artefact.
  `plutoLoop ( )` prints before the new enable block in the same `userCode ( )` call. `processRx ( )` re-derives
  `MAG_MODE` from AUX1 at about 50 Hz, so on loops where it has just run, the print sees 0 while `updateMagHold ( )`
  sees 1. The turns and the steady hold show that heading hold was on whenever it was evaluated. The §2 pass
  criterion "`Md:1` on every tick" was written without this ordering in mind. The real criterion, "the target is not
  wiped", is met.

**Session 2: HEADFREE selected on purpose ( `Md:2` ), from 99 s**

- Ev 1 relative +90 ( H0 154 ): `Mh` 244, `Hd` 154 → 177 at +0.4 s → 229 at +1.3 s → 244 at +2.3 s, then held ±1° for
  7 s. **This shows the relative setter works in HEADFREE**, where the task-2 block does not act because HEADFREE
  already holds heading.
- At +11.2 s **the craft hit an obstacle** ( the user confirmed ). The heading swings several hundred degrees per
  second with the sticks centred. From +12.5 s, `Mh` flips between `Hd` and the target, which is the disarmed reset
  in `processRx ( )` ( `mw.cpp:942` ) racing the restore in `userCode ( )`: the craft was down. Ev 2 at +13 s was
  called on the ground, and the spin at +15.7 s is handling. The log ends at +17.3 s.
- The crash was external, and none of the task-2 code acts in HEADFREE with the stick centred. Ev 2-4 in HEADFREE
  were not tested; they are added to task 6.

## 3. Validation flights ( task 6 ): cases 1-8

**Build.** PRIMUS_X2_v1 with tasks 2-4 ( gate clean, 107.7 KB / 17.6 KB ). The validation `PlutoPilot.cpp` builds
with no warnings, at 108.0 KB.

**Test code.** It is a step sequence. Each step fires a fixed time after the previous one, except the two pilot
steps, which wait for you to yaw by hand and release.

| `St` | When | Action | Case | Pass |
|---|---|---|---|---|
| 0 | +5 s | relative +90 | 1 | Turns 90° with no stick input. `Hd` reaches `Mh` ±5° in < 3 s and holds. |
| 1-3 | +5 s each | relative +90 ×3 | 2 | Each turns a further 90°. After St 3, `Mh` = `H0` exactly. The wrap through 0/359 is clean. |
| 4 | +5 s | relative −90 | 3 | `Mh` = H0 − 90. |
| 5 | +5 s | relative +270 | 3 | `Mh` = H0 + 180. The craft flies the short way ( −90 ). |
| 6 | +5 s | `setDesiredAngle ( H0 )` | 6 ( setup ) | A 180° turn back to H0. `F` bit 2 ( relative ) = 0. |
| 7 | **pilot** | yaw by hand ~90°, then let go | 6 | After release and settling, the craft **returns to H0** ( absolute target kept ). |
| 8 | +6 s after release | relative +90 | | `Mh` = H0 + 90 ( measured from the absolute target ). |
| 9 | **pilot** | yaw by hand ~90°, then let go | 4 | After release, the craft **holds where you left it** ( no swing back ). `Mh` = `Hd` at settle, and `F` bit 4 ( stale ) clears. |
| 10 | +6 s after release | relative +90 | 4 | `Mh` = the nose you left + 90. |
| 11 | +5 s | relative +180 | 5 | **Switch dev mode OFF as soon as this turn starts.** The craft stops the turn and holds. The new session's `H0` should be within about 20° of the heading at the switch-off, not the +180 target. |
| 12 | dev mode on again, then +5 s | relative +90, then hold | 8 | See the procedure below. |

**Other checks**

- **Case 7 ( a flicker ):** any `S:2` marker that falls inside a turn must not stop it. The turn continues, `F`
  bit 1 stays 1 and `Mh` is unchanged across the marker. Flickers cannot be forced, so this is read from whatever
  occurs.
- **Flip ( flight A only, optional ):** with a target held at St 12, press the app's flip button. The app flip now
  works while a user yaw target is held, because MAG mode is forced. After the flip the heading returns to the target.

**Log fields** ( about 95 B per tick )

| Field | Meaning |
|---|---|
| `t`, `n` | time and tick count |
| `Hd` | heading |
| `Mh` | heading-hold target |
| `Md` | 1 MAG, 2 HEADFREE |
| `Yw` | yaw stick − 1500 |
| `F` | flags: 1 `isUserHeadingSet`, 2 relative, 4 stale, 8 armed |
| `St` | current step |
| `Ev` | step + 1 on the tick it fires |
| `S:1` / `S:2` | session markers, also printing `St` and `H0` |

### Procedure

Save each flight under its own name.

**Flight A: AUX1 as you normally fly ( heading hold off ) → `logs/log-4.txt`**

1. Flash, connect the app and start PlutoMonitor.
2. Arm and take off to a hover at least 1 m up, with room to turn.
3. Switch dev mode on and keep your hands off yaw.
4. St 0-6 run by themselves ( about 35 s ).
5. When the craft has been still for about 5 s after the 180° turn, **yaw about 90° by hand and let go** ( St 7 ).
   Watch it return.
6. About 6 s later it turns +90 ( St 8 ). Wait about 5 s, then **yaw about 90° by hand again and let go** ( St 9 ).
   It must stay where you left it.
7. 6 s later it turns +90 ( St 10 ). 5 s after that it starts a 180° turn ( St 11 ): **switch dev mode off as soon
   as it starts turning**.
8. Wait about 3 s, then switch dev mode **on** again. 5 s later it turns +90 ( St 12 ) and holds.
9. Optional: press the app's flip button once and watch the heading afterwards.
10. Land and disarm, but keep dev mode on. **Pick the drone up, turn it about 90° by hand, and put it down.**
11. Re-arm and take off. **It must not yaw by itself on takeoff** ( case 8 ). Hover about 5 s, then land.

**Flight B: HEADFREE on ( the app's headfree switch ) → `logs/log-5.txt`.** The same procedure, without the flip.

**Safety.** Yaw only; you keep throttle and position. To take over, push yaw past about a third of its travel or
switch dev mode off. Fly in open space: the HEADFREE run in log-3 ended in a collision.

### Results ( log-4, 15 Oct 2026 )

**Verdict: partial.** Cases 1-3, the absolute turn and the disarm clear pass. The hand-yaw cases ( 4, 6 ), the
mid-turn switch-off ( 5 ), the flicker in a turn ( 7 ), the no-yaw-at-takeoff check ( 8 ) and flight B ( HEADFREE )
were **not flown**. The user chose to accept this and move on. These cases rest on the reviewer's code traces
( CHANGES.md tasks 3-4 ).

log-4: 1,048 records, all complete. There were three lost records ( an `n` step of 2 ) and one stall or gap of
6 ticks. Markers: S:1 at 0 s, S:2 ( flicker ) at 72.7 s, and S:1 at 88.5 s and 98.4 s ( the pilot switched dev
mode off and on ). log-5 is empty.

| Step / case | Result |
|---|---|
| St 0 relative +90 ( case 1 ) | 143 → 205 at +1 s → 232 at +2 s → 233. Clean, with no stick input. |
| St 1-3 relative +90 ×3 ( case 2 ) | 233 → 323 → 53 → 143. Back to the start heading **exactly** after 360°, and the wrap through 0/359 is clean ( 323 → 53 ). Each turn settled in about 2 s, within ±1°. |
| St 4 relative −90 ( case 3 ) | 143 → 53, error 1° at +4.5 s. |
| St 5 relative +270 ( case 3 ) | Target 323 ( H0 + 180 ), flown the short way: 53 → 348 at +1 s → 323. |
| St 6 `setDesiredAngle ( H0 = 149 )` | 323 → 149 ( 174° ) in 3 s. `F` = 9: the relative flag is 0 for the absolute target. |
| St 7 hand yaw ( case 6 ) | **Not flown.** `Yw` = 0 for the whole flight. The sequence waited at St 7 until the landing. |
| St 9 / St 10 ( case 4 ) | **Not reached.** |
| Disarm clears the target ( task-4 review fix ) | **Passes.** At 50.5 s and again at 107.1 s, `F` drops from 9 / 11 to 0 on the disarm. `isUserHeadingSet` is cleared, and from then on `Mh` follows `Hd`. |
| Case 5 ( dev mode off mid-turn ) | **Inconclusive.** Dev mode was switched off at about 95 s, with the turn 96% done ( 353 of 357 ). The new session's H0 of 355 cannot separate "held" from "finished". |
| Case 7 ( flicker in a turn ) | **Not tested.** The one S:2 ( 72.7 s ) came with no target held. The step stayed at 7 through it, as designed. |
| Case 8 ( no yaw at takeoff ) | **Not done as planned.** After the 107.1 s disarm, the sequence fired St 1 on the ground ( 108.6 s, `F` = 3, disarmed ). That is a test-code timing artefact: by design, a target set before arming survives. |
| Re-runs after the real restarts | St 0 at 93.6 s: 267 → 357. St 0 at 103.5 s: 0 → 90 in 1.5 s. Both clean. |

**Heading jumps at takeoff ( observation, out of scope ).** At 66.6 s ( 147 → 234 ) and 82.0 s ( 196 → 294 ) the
craft was armed with no user target, heading hold off ( `Md` 0, `Mh` following `Hd` ) and the stick centred. The user
confirmed these were **takeoffs**. Nothing was holding heading, so the craft yawed as it lifted. This is existing
behaviour and unrelated to this topic: no user target was held, so neither setter nor heading hold could act.
