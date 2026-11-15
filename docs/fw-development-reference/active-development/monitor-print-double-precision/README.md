# Monitor_Print double precision

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

| | |
|---|---|
| **Status** | **Closed - pending commit** ( 15 Nov 2026, FW 3.15.11, API 1.8.9 ) |
| **Mode** | `fix` |
| **Branch** | `BugFix-June26` |
| **Target** | `PRIMUS_X2_v1` ( all targets built at commit ) |
| **Last updated** | 15 Nov 2026 |
| **Pipeline docs** | `subsystems/User_Space_API.md` ( one bullet, via [PIPELINE_UPDATE.md](PIPELINE_UPDATE.md) ); new `docs/API/DEBUGGING_API_WIKI.md`, `FC_DATA_API_WIKI.md`, `dev-guide/FLIGHT_INVARIANTS.md`, CLAUDE.md |

**Problem.** API audit B9 ( Sr 84, T01 ): `Monitor_Print / Monitor_Println ( tag, double, precision )` print 0 for
every digit after the first decimal ( 3.14159 @ 3 -> `3.100` ). The fraction loop
( [Debugging.cpp:224-238](../../../../src/main/API-Src/Debugging.cpp#L224), `:201-208` before the fix ) subtracted the
ASCII code ( 48 + d ) instead of the digit, so the remainder went negative and the `double -> uint8_t` cast returned 0
from then on.

**Plan.** Fix the digit loop ( rounding half away from 0, precision clamped to 4 ), and the other `Debugging.h` defects
found by the audit: the 256+ character hang, the one-byte length wrap, `INT_MIN`, and the NaN / inf length byte. No
signature change. A host check of the formatter, then a bench test table in PlutoMonitor and a short hover. New
Debugging API wiki; API 1.8.9. Details: [INVESTIGATION.md](INVESTIGATION.md), tasks: [TASKS.md](TASKS.md).

## Open items

- Task 1 done ( 15 Nov 2026 ): the fraction loop subtracts the digit before the ASCII conversion, so B9 is fixed, and
  the precision clamp is `MONITOR_MAX_PRECISION` 4. Gate green on `PRIMUS_X2_v1`, no new `src/` warnings, flash
  121.5 KB / 256 KB, RAM 20.5 KB / 40 KB. Reviewer: nothing blocking ( one banner nit, applied ). Details:
  [CHANGES.md](CHANGES.md).
- Task 2 done ( 15 Nov 2026 ): strings are counted with a bounded helper and cut at 255 characters ( a 256-character
  string used to hang `loop ( )` ), every print cuts the tag so tag + tab + number fit the one-byte length field, and
  the NaN / inf / overflow branches share one helper that counts the tab. Gate green, this file's baselined warnings
  drop from 10 to 4, flash 121.5 KB / 256 KB, RAM 20.5 KB / 40 KB. Reviewer: nothing blocking. Details:
  [CHANGES.md](CHANGES.md).
- Task 3 done ( 15 Nov 2026 ): the int print takes the number's magnitude in `uint32_t`, so `INT_MIN` prints
  `-2147483648` instead of garbage characters. Gate green, this file's baselined warnings drop from 4 to 3, flash
  121.5 KB / 256 KB, RAM 20.5 KB / 40 KB. Reviewer: nothing blocking ( one banner nit, applied ). Details:
  [CHANGES.md](CHANGES.md).
- Task 4 done ( 15 Nov 2026 ): host check of the real `Debugging.cpp` on the PC: 86 / 86 pass, covering every
  length byte and checksum, plus 200 000 random values against `snprintf` with 0 wrong ( 2 near-ties, above 1e8 at
  4 decimals ). `HEAD` fails 51 / 83 and shows every planned bug. Details: [TESTING.md](TESTING.md).
- Task 5 done ( 15 Nov 2026 ): clean gate on `PRIMUS_X2_v1` green, no new `src/` warnings, `Debugging.cpp` baselined
  warnings 10 -> 3, flash 121.5 KB / 256 KB, RAM 20.5 KB / 40 KB. `Debugging.o` is +8 B against `HEAD`; the flashed
  image is unchanged until user code calls a `Monitor_*` function. Details: [TESTING.md](TESTING.md).
- Task 6 done ( 15 Nov 2026 ): flown twice on hardware, both passing; `logs/log-1.txt` is the second run ( table,
  85 s hover up to 175 cm, table again ). 0 wrong table rows ( T01 `3.142`; the NaN / inf lines arrive whole );
  `ts` = `t` / 1000 on 633 / 633 hover records; loop time median 3505 us; no disconnect; handling normal. The
  repo-root `PlutoPilot.cpp` is restored to `HEAD`. Details: [TESTING.md](TESTING.md).
- Task 7 done ( 15 Nov 2026 ): new [DEBUGGING_API_WIKI.md](../../../API/DEBUGGING_API_WIKI.md); `Debugging.h`
  comments state the limits ( no signature change ); the stale "0 after the first decimal" text is gone from
  FLIGHT_INVARIANTS.md, the FC-Data wiki and backlog item 11; [PIPELINE_UPDATE.md](PIPELINE_UPDATE.md) stages the
  `User_Space_API.md` bullet and the CLAUDE.md line. Every claim checked against a source line or test, and every wiki
  example compiled with the firmware command ( CHANGES.md ).
- Task 8 done ( 15 Nov 2026 ): topic review found nothing blocking and nothing in the code; 3 should-fix and 4 nits,
  all in the docs, all applied ( see [CHANGES.md](CHANGES.md) ). Gate re-run green, 0 broken links.
- Task 9 done ( 15 Nov 2026 ): graph refreshed and labelled; all three targets built clean ( `PRIMUS_X2_v1` 121.5 KB /
  20.5 KB, `PRIMUSX2` 121.3 KB / 20.7 KB, `PRIMUS_V5` 121.5 KB / 20.5 KB ); FW 3.15.10 -> 3.15.11 and API 1.8.8 ->
  1.8.9; `PIPELINE_UPDATE.md` applied to `User_Space_API.md` and CLAUDE.md; CHANGELOG updated; staged.
  **Closed - pending commit**: the user commits.
- Observations, out of scope ( neither caused by the print code, each worth its own `check` topic ): two ~23 ms
  `cycleTime` samples against a 3.5 ms loop; and, in the first run, the drone bumping the ground at touchdown on low
  battery and taking off again before the disarm.
- Risk: user code that asked for 5-7 digits now gets 4.
- Risk: a 255-character payload is a 259-byte frame, longer than the 256-byte UART TX ring, so the longest allowed
  print overwrites its own start. Not a regression; the task 7 wiki says the cap is a framing limit, not a usable
  length.

## Next action

None: staged with `.git/MAGISV2_COMMIT_MSG.txt`; the user commits, and the next `pluto-commit` / `pluto-grill` run
fills in the hash.
