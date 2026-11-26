# Off-centre payload lean - Session handoff

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [TESTING](TESTING.md) · [CHANGES](CHANGES.md) · [SESSION](SESSION.md)

Written 19 Oct 2026 at the end of the first working session, so development can continue on another PC. It records
how the topic got where it is, what is decided, what is open, and what did **not** travel with the branch. The topic
files remain the source of truth; this page is the reading guide and the conversation record.

## 1. Resume on the other PC

1. `git fetch` and `git switch BugFix-June26-cg-tilt` ( parent `BugFix-June26` at `a81c180` ).
2. PlutoIDE: select `PRIMUS_X2_v1` ( the board flown in this topic ). Build. The root `PlutoPilot.cpp` is the
   **diagnostic sketch** on purpose ( section 5 ).
3. Start the development session in the repo and run `/pluto-task resume`. It reads the [TASKS.md](TASKS.md) Resume
   block. Then `/pluto-task next` continues with **task 5**.
4. Reading order if you want the background first: this page → [README](README.md) → [INVESTIGATION](INVESTIGATION.md)
   §4-5 → [TESTING](TESTING.md) §3 → [CHANGES](CHANGES.md) → the [TASKS](TASKS.md) decisions log.

## 2. What happened in this session

| Step | Outcome |
|---|---|
| Report | With a 9 g servo ( dead weight ) mounted off-centre, the craft leans toward it and drifts on the app take-off and on an ALT_HOLD descent with the stick fully down. No lean in hover, on throttle-up or on `Command_Land`. Any side: front → pitch, right → roll. The pilot had to hold the opposite stick |
| Planning ( `/pluto-grill-deep`, mode `fix` ) | Interview: PRIMUS_X2_v1; app take-off button; ALT_HOLD in every phase; default PIDs; stick fully down on the bad descents; both symptoms in scope; done when no opposite stick is needed. Out of scope: driving the servo, PID retuning, flip / failsafe beyond regression, flying PRIMUS_V5 |
| Code reconnaissance ( 3 scout briefs, [SCOUT.md](SCOUT.md) ) | `pidRewrite`'s rate integrator `errorGyroI` is the only state that holds an off-centre torque; it is zeroed at every arm and on every RX frame with the raw stick below `mincheck` 1100, even in flight ( `mw.cpp` `processRx ( )` ) |
| Known-good test | The user flew `f60c9ad` ( 20 Aug ) with the same payload: no lean. The reset code is identical; `4657cbf` ( no in-flight baro datum re-zero ) and `2a8d59a` ( goal-profile take-off, −30 cm/s stick descent ) made the zero-integrator phases ~4× longer, so the drift became visible ( ~16× ). Neither is reverted. ( A `c495d6a` hex was also built for a comparison but not flown. ) |
| Task 1 ( flight, 3.15.1 ) | The lean appears only at or just above full stick down ( the `mincheck` band ); higher descents are straight. Cause H1 confirmed, no bisect |
| Task 2 ( code ) | Diagnostic getters `pidGetGyroITerm ( )`, `pidGetGyroResetCount ( )` + the log sketch |
| Task 3 ( flight, log-1 ) | H2 confirmed: `Ip` → 0 at every arm ( even after hovering at −105 ); take-off lean +4.8° / +8.7° that starts on the floor while the motors spin up; hover trim `Ih` ≈ **−103** pitch ( of ±256 ); full-stick descent stands ~10° nose down with `Ip` at 0; disarmed on the floor `Ip` rails to −256 within ~1 s. New: ±14-20° pitch / roll swings for 3-5 s after each dip recovery ( cause open ) |
| Task 4 ( code ) | The stick-low reset is skipped in an ALT_HOLD flight, guarded by ground idle, a 20 cm height floor and a stick-descent touchdown latch. Two review rounds; the user chose the guards in two rounds ( decisions log ) |
| Checkpoint | All three targets build; branch `BugFix-June26-cg-tilt` created; this page written; changes staged for the user's commit |

## 3. Decisions that shape the remaining work

The full list with dates is in the [TASKS.md](TASKS.md) decisions log. The ones the next tasks depend on:

- **Fix design** ( user ): E ( task 4, done ) + B, a hover-snapshot seed at arm ( task 5 ) + C, a faster roll / pitch
  wind-up during the take-off climb ( task 6 ), so that the first flight after power-on is covered too.
- **The raw integrator never crosses an arm.** It winds to the rail on the floor ( log-1 ). Only a snapshot from
  settled airborne hover may seed it.
- **The payload only changes with the battery out** ( user ), so a RAM-only snapshot cannot go stale.
- **Flash budget** < 500 B against `a81c180` ( user ). Used so far: +1152 B with the test sketch, of which the
  firmware share is task 2's getters plus task 4's +208 B.
- **The task 4 ground guard** has both a height floor and a touchdown latch. The height alone failed review: the
  baro errs high after a warm flight, and a table is above the arm point.

## 4. Open items

| Item | Where | Who |
|---|---|---|
| **Task 5 clamp**: `Ih` −103 is ~80 % of the planned ±128 | first step of task 5 ( mini-grill ) | user decides |
| **Task 6 window**: the take-off lean starts on the floor, so the faster wind-up may start at the arm | first step of task 6 | user decides |
| **Task 12 scope**: a pre-existing hazard. ARM-switch arm with the stick low while BARO is off leaves `isThrottleStickArmed` set; BARO switched on later in flight with the stick ≤ 1500 drops the motors to idle **in the air** | [TASKS.md](TASKS.md) task 12 | user: this topic or a new one |
| **Pilot stick during log-1 dips**: did roll / pitch stick follow the full-stick descents? It would explain the roll part of the recovery swings | answer in the next session; checked again in task 8 | user |
| Recovery swings after a dip ( ±14-20° ) | task 8 checks they are gone with task 4 ( descent from ≥ 100 cm ) | flight |
| Stick-down landing timing of the touchdown latch | task 8 ( add an `F` digit for the latch ) | flight |

## 5. What did not travel with the branch

| Item | On this PC | On the other PC |
|---|---|---|
| Diagnostic hex | built from this branch | rebuild the branch ( `PRIMUS_X2_v1` ); the root `PlutoPilot.cpp` is already the sketch |
| Stock rollback hex ( `a81c180`, FW 3.15.1 ) | a scratch copy only | build `BugFix-June26` ( or check out `a81c180` ) with the template `PlutoPilot.cpp` |
| `logs/log-1.txt` ( raw task 3 log ) | in the topic folder, not committed ( logs are never staged ) | copy it by hand if a re-analysis is wanted; the results are in [TESTING.md](TESTING.md) §3 |
| A git worktree used to build `c495d6a` / `a81c180` | registered in this clone only | nothing to do; `git worktree prune` here when done |
| The session's standing preferences | the assistant's local memory on this PC | already in `CLAUDE.md` and the skills: never run `git commit` / `git push` ( stage and draft only ); no AI attribution in commits or docs; show the full drafted commit message and open `.git/MAGISV2_COMMIT_MSG.txt`; reviews run in the `pluto-reviewer` agent; when a log is expected, create a numbered `logs/log-N.txt` in the topic and open it |

## 6. Flight notes carried forward

- Flashing between builds with different `EEPROM_CONF_VERSION` resets all settings: `f60c9ad` is 106, this branch
  108. After any such flash, recalibrate the accelerometer and compass before flying.
- The diagnostic log runs only in **Developer Mode** with the app connected; switch it on before arming. Fields:
  `t F Ir Ip Rs Tp R P Ze` ( [test/PlutoPilot_lean_diag.cpp.txt](test/PlutoPilot_lean_diag.cpp.txt) header ).
  `Rs` is read as the step between lines. Positive `P` = nose down.
- The root `PlutoPilot.cpp` must be restored to the template before the release commit ( task 11; `pluto-commit`
  checks it ).
