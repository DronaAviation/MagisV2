# Off-centre payload leans the craft on take-off and stick-down

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [TESTING](TESTING.md) · [CHANGES](CHANGES.md) · [SESSION](SESSION.md)

| | |
|---|---|
| **Status** | **Active** ( opened 19 Oct 2026 ) |
| **Mode** | `fix` |
| **Branch** | `BugFix-June26-cg-tilt` ( from `BugFix-June26` at `a81c180` ) |
| **Target** | `PRIMUS_X2_v1` ( all targets at commit ) |
| **Baseline** | `a81c180`, FW 3.15.1 ( the failing build ); known-good comparison `f60c9ad` ( 20 Aug 2026 ) |

**Problem.** With a 9 g servo fitted off-centre as a dead-weight payload, the craft leans toward the payload side and
drifts that way during the app take-off and during an ALT_HOLD descent with the throttle stick fully down. It does
not lean in hover, on throttle-up or during `Command_Land`. The pilot has to hold the opposite stick. The same
payload on `f60c9ad` showed no lean ( user flight, 19 Oct 2026 ).

**Cause ( from the code, to be confirmed in flight by task 1 ).** The rate integrator `errorGyroI` is the only part
of `pidRewrite` that carries a steady off-centre torque. It is zeroed at every arm and on every RX frame while the
raw throttle stick is below `mincheck` 1100 ( `mw.cpp:894-897` ), even in flight. That code is unchanged since
`f60c9ad`. What changed is the exposure: `4657cbf` removed the in-flight baro datum re-zero that turned a full-stick
descent into a ~1 s drop, and `2a8d59a` replaced the 120 cm take-off step with a 60 cm/s goal profile. The untrimmed
phases now last several seconds instead of about one, so the drift grows roughly 16× ( see INVESTIGATION.md ).

**Agreed fix.**

1. **E**: skip the stick-low I-term reset while armed and flying in ALT_HOLD ( not at ground idle ).
2. **B**: `pidResetErrorGyro ( )` loads a roll / pitch **hover snapshot** instead of 0; snapshot taken in settled
   hover, clamped to ±128, cleared by a crash disarm, RAM only ( lost at power-off ).
3. **C**: about 2× roll / pitch I gain for about 1.5 s of the take-off climb, so the first flight after power-on
   also levels quickly.

**Gotchas.**

- Flashing `f60c9ad` ( EEPROM 106 ) and back to 3.15.1 ( 108 ) resets all settings each time: recalibrate the
  accelerometer and compass before flying.
- The PID runs even while disarmed, so the integrator winds against the floor. Never carry the raw integrator
  across an arm; only the hover snapshot.

**Open items.**

1. ~~Confirming flight on 3.15.1 ( task 1 )~~. Done: the lean appears only in the bottom stick band ( H1 confirmed ).
2. ~~Diagnostic getters and the test log ( task 2 )~~. Done.
3. ~~Baseline log ( task 3 )~~. Done: H2 confirmed, `Ih` ≈ −103 pitch, ~10° lean in a full-stick descent. Open:
   ±14-20° swings after a dip recovery ( re-checked in task 8 ).
4. ~~Task 4 ( no stick-low reset in an ALT_HOLD flight )~~. Done: guarded by ground idle, a 20 cm floor and a
   stick-descent touchdown latch. Next: the hover-snapshot seed ( task 5 ). Task 12 ( a pre-existing ground-idle
   motor cut in the air ) needs the user's scope decision.
5. The test `PlutoPilot.cpp` and the task 2 log line are temporary: remove them before release.
