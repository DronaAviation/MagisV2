# Set default PID profile

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

| | |
|---|---|
| **Status** | **Closed - pending commit** ( 14 Nov 2026, FW 3.15.7, API 1.8.5 ) |
| **Mode** | `fix` |
| **Branch** | `BugFix-June26` |
| **Target** | `PRIMUS_X2_v1` ( all targets built at commit ) |
| **Last updated** | 14 Nov 2026 |
| **Pipeline docs** | [subsystems/Core_PID_Controller.md](../../fw-architecture-pipeline/subsystems/Core_PID_Controller.md); line references in [Mixer_Pipeline.md](../../fw-architecture-pipeline/subsystems/Mixer_Pipeline.md) and [User_Space_API.md](../../fw-architecture-pipeline/subsystems/User_Space_API.md) ( all via `PIPELINE_UPDATE.md` ) |

**Problem.** API test T06 ( B1 ): after `setDefault_PIDProfile ( )`, `get_PIDProfile ( PID_ALT, ... )` read 100 instead
of the firmware default 128, and `PID_USER` read 0 / 0 / 40 instead of 60 / 30 / 5. The loop in
[FC-Config-PID.cpp](../../../../src/main/API-Src/FC-Config-PID.cpp) ( line 158 at HEAD ) runs `i = ROLL .. PIDUSER` ( 0 .. 10 ) over
five-entry tables, so index 3 takes a stale ALT value and indexes 4 .. 10 read past the tables and overwrite the hidden
gains PIDPOS, PIDPOSR, PIDNAVR, PIDLEVEL, PIDMAG, PIDVEL and PIDUSER. That is undefined behaviour, which GCC already
reports and which is baselined in `tools/warnings-baseline.json`. PIDLEVEL feeds the angle loop and PIDVEL the inner
altitude-hold loop, so a single API call silently retunes the flight controller. Details:
[INVESTIGATION.md](INVESTIGATION.md).

**Fix.** One source of truth for the default gains: the `P8` / `I8` / `D8` assignments move out of the `static`
`resetPidProfile ( )` in [config.cpp](../../../../src/main/config/config.cpp) into a small shared
`pidProfileDefaultGains ( pidProfile_t * )`. `setDefault_PIDProfile ( )` fills a stack copy from it and writes only the
five slots the API exposes ( `PIDROLL`, `PIDPITCH`, `PIDYAW`, `PIDALT`, `PIDUSER` ), leaving the hidden gains untouched;
its local tables are deleted. The function stays RAM-only ( no `writeEEPROM` ) and keeps no armed guard, both stated in
the header comment. Verified with a host simulation in `tools/sim/` ( clang++ with ASan and UBSan, g++ as the fallback ) instead of a bench
log, by the user's choice.

## Open items

- Task 1 done ( 14 Nov 2026 ): the default gains moved into `flight/pid_defaults.cpp` as
  `pidProfileDefaultGains ( )`, called from `resetPidProfile ( )`; values diffed identical to HEAD. Gate green on
  `PRIMUS_X2_v1` ( Flash 121.1 KB, RAM 20.5 KB ), reviewer 0 blocking / 0 should-fix. Details:
  [CHANGES.md](CHANGES.md).
- Task 2 done ( 14 Nov 2026 ): the tables and the 0 .. 10 loop are gone; the five API slots are copied from
  `pidProfileDefaultGains ( )`, and the `FC-Config.h` comment now states the five restored values, the untouched
  internal gains, RAM only, the armed caveat and the LuxFloat caveat. Gate green, +24 B flash, RAM unchanged; the four
  baselined `FC-Config-PID.cpp` warnings are gone. Reviewer 0 blocking / 0 should-fix, 2 nits fixed. Details:
  [CHANGES.md](CHANGES.md).
- Task 3 done ( 14 Nov 2026 ): host simulation in [tools/sim/](../../../../tools/sim/) builds the real API code on the
  PC. Fixed code `PASS: 103 checks, 0 failed` under clang with ASan and UBSan and under g++; HEAD's file through the
  same harness aborts on the out-of-bounds read and fails 58 of 103 checks. Reviewer 0 blocking, 1 should-fix and 3
  nits, all fixed. Evidence: [TESTING.md](TESTING.md).
- Task 4 done ( 14 Nov 2026 ): gate green on `PRIMUS_X2_v1`, Flash 121.1 KB / RAM 20.5 KB ( +24 B flash against HEAD ). The
  four fixed `FC-Config-PID.cpp` entries are removed from `tools/warnings-baseline.json` ( a three-line diff, total 2658 ).
- Task 5 dropped: the user chose the host simulation ( task 3 ) over a bench or flight log.
- Task 6 done ( 14 Nov 2026 ): [PIPELINE_UPDATE.md](PIPELINE_UPDATE.md) stages a "Default gains and the API reset" section
  for `Core_PID_Controller.md` and three line references task 1 shifted ( `config.cpp` 369 / 278 / 479 to 331 / 240 / 441 ).
  Every statement checked by hand against a source line ( `spec-to-code-compliance` is not available in this session ).
- Task 7 done ( 14 Nov 2026 ): topic review, 0 blocking. One should-fix ( task 8 would have applied only one of the three
  staged pipeline docs ) and 5 nits, all fixed; `tools/sim/` is now described in
  [dev-guide/BUILD_AND_VERIFY.md](../../dev-guide/BUILD_AND_VERIFY.md). Gate and simulation re-run green.
- Task 8 done ( 14 Nov 2026 ): graph refreshed and labelled; all three targets built clean ( no new `src/` warnings ),
  FW 3.15.7 / API 1.8.5; the three `PIPELINE_UPDATE.md` sections applied ( `Core_PID_Controller.md`,
  `Mixer_Pipeline.md`, `User_Space_API.md` ); CHANGELOG updated ( version line, Fixed, Documentation ); staged with a
  drafted message. The user makes the commit.
