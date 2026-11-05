# RcCommand_Get API

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md)

| | |
|---|---|
| **Status** | **Closed - pending commit** ( 5 Nov 2026, FW 3.15.5, API 1.8.3 ) |
| **Mode** | `fix` |
| **Branch** | `BugFix-June26` |
| **Target** | `PRIMUS_X2_v1` ( all targets built at commit ) |
| **Last updated** | 5 Nov 2026 |
| **Pipeline docs** | [User_Space_API.md](../../fw-architecture-pipeline/subsystems/User_Space_API.md) ( via `PIPELINE_UPDATE.md` ); new `docs/API/RC_INTERFACE_API_WIKI.md` |

**Problem.** API test B12 ( build ): `int16_t *RcCommand_Get ( void )` from `RC-Interface.h` is an undefined
reference, because [RC-Interface.cpp](../../../../src/main/API-Src/RC-Interface.cpp) defined `RcCommand_get`. The user
renamed it to `RcCommand_Get`, which links. Under the name, the body copies `rcCommand [ 0..3 ]` into a local array and
returns its address: GCC flags it ( `-Wreturn-local-addr`, baselined ) and compiles the function to `return NULL`
( `movs r0, #0` ), so user code reads address 0, the vector table, not the sticks. Details:
[INVESTIGATION.md](INVESTIGATION.md).

**Fix.** Keep the rename. Copy into a file-static `int16_t [ 4 ]` on each call and return its pointer ( signature
unchanged, 8 B RAM, valid until the next call ). Roll / pitch / yaw as `1500 + rcCommand` ( the scale of
`RcCommand_Get ( channel )` and `RcCommand_Set` ), throttle as `rcCommand [ THROTTLE ]` unchanged ( the curve of `rcData [ THROTTLE ]`, 1100-2000: normally the stick; user code
runs before altitude hold writes its output there ). Correct the header
comments of both `RcCommand_Get` overloads; new RC API wiki. API 1.8.3, FW 3.15.5.

## Open items

- Tasks 1-2 done ( 5 Nov 2026 ): static snapshot, header comments corrected ( review: throttle is the stick through the
  curve, not the hold output ); gate green, size identical to HEAD; a user call costs 44 B flash + 8 B RAM.
- Task 3 done ( 5 Nov 2026 ): bench log-1 PASS: array = channel getter on 563 / 563 records, throttle = curve output of
  the stick; `PlutoPilot.cpp` restored, clean hex rebuilt. Details: [TESTING.md](TESTING.md).
- Task 4 done ( 5 Nov 2026 ): new [RC_INTERFACE_API_WIKI.md](../../../API/RC_INTERFACE_API_WIKI.md), [PIPELINE_UPDATE.md](PIPELINE_UPDATE.md) staged.
- Task 5 done ( 5 Nov 2026 ): topic review found nothing blocking; one should-fix ( what the throttle element shows )
  and four nits fixed in comments and docs.
- Task 6 done ( 5 Nov 2026 ): graph refreshed; all three targets built clean ( no new warnings ); FW 3.15.5 / API 1.8.3; `User_Space_API.md` and CHANGELOG updated.
- Out of scope, documented as known gaps in the wiki: `RcData_Get ( )` has the same local-array bug ( returns NULL );
  `RcCommand_Get ( RC_THROTTLE )` returns `1500 + rcCommand [ THROTTLE ]`, 2600-3500. Until that is fixed the array
  throttle ( 1100-2000 ) and the channel throttle disagree.

## Next action

None: staged with `.git/MAGISV2_COMMIT_MSG.txt`; the user commits, and the next `pluto-commit` / `pluto-grill` run fills in the hash.
