# Monitor_Print double precision - Changes

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md)

## Task 1: fraction digits ( B9 ) and precision clamp 0..4

[src/main/API-Src/Debugging.cpp](../../../../src/main/API-Src/Debugging.cpp)

| Where | Change | Why |
|---|---|---|
| `debugPrint ( msg, double, digits )`, fraction loop ( `:224-238` ) | The digit is taken as `uint8_t digit` ( 0-9, guarded at 9 ), `remainder -= ( double ) digit` is applied **before** the ASCII conversion `( uint8_t ) ( '0' + digit )`; the dead `'A' - 10` branch is gone. | The old loop subtracted the character ( 48 + d ), so the remainder went negative and `__aeabi_d2uiz` returned 0 for every later digit ( B9: 3.14159 @ 3 -> `3.100` ). |
| New `#define MONITOR_MAX_PRECISION 4` after the includes | One constant for the clamp. | Decision: precision 0..4 ( was 0..7 ). |
| `Monitor_Print ( tag, double, precision )`, `Monitor_Println ( tag, double, precision )` | `constrain ( precision, 0, MONITOR_MAX_PRECISION )`. | Same. |
| Banner | `Last Modified` 15 Nov 2026, HISTORY line ( right edge realigned after the review nit ). | House style. |

Unchanged: rounding half away from 0 ( `rounding = 0.5 / 10^digits` before the integer split ), sign handling ( `-0.00` kept ),
payload length ( already counts `digits + 1` for `.` and the digits, so it still matches the bytes sent ).

Hand trace ( after the fix ):

| Call | Rounded value | Output |
|---|---|---|
| 3.14159 @ 3 | 3.14209 | `3.142` |
| -2.5 @ 1 | 2.55, sign `-` | `-2.5` |
| 1.99999 @ 2 | 2.00499 | `2.00` |
| x @ 9 | clamped to 4 digits | 4 digits |

Gate: `driver.sh --gate PRIMUS_X2_v1` green, no new `src/` warnings ( `Debugging.cpp` still has its 10 baselined
warnings ), flash 121.5 KB / 256 KB, RAM 20.5 KB / 40 KB.

## Task 2: bounded strings and correct length bytes ( D1, D2, D4 )

[src/main/API-Src/Debugging.cpp](../../../../src/main/API-Src/Debugging.cpp)

| Where | Change | Why |
|---|---|---|
| New `#define MONITOR_MAX_PAYLOAD 255` | Largest payload the one-byte length field carries. | D2: a payload over 255 bytes wrapped the length byte. |
| New `static boundedLength ( s, max )` | Counts a string up to `max` characters; a NULL string counts as empty. | D1: replaces `strlen ( )` calls, so no string can wrap the length or stall the loop. |
| `serializeString ( msg, len )` | Now `static`, sends exactly `len` characters ( no `strlen` per character ). | D1: the `uint8_t` index against `strlen ( )` never ended for 256+ characters. No user outside this file. |
| New `static debugPrintTagText ( msg, text )` | Sends `tag <tab> text` with the tab counted in the length. Used by the NaN, inf and overflow branches. | D4: NaN and inf left the tab out of the length, so the frame length and checksum were wrong. |
| `debugPrint ( msg )` | Message cut at 255 characters. | D1 / D2. |
| `debugPrint ( msg, double, digits )`, `debugPrint ( msg, int )` | The number length is computed first, then the tag is cut to `255 - 1 - numLen`; `data_size = msgLen + 1 + numLen`. | D2: the tag is cut, never the number ( decision ). |

Worst cases: a double payload is at most tag + 1 + 16 bytes ( sign, 10 integer digits, `.`, 4 digits ), so the tag
keeps at least 238 characters; an int payload keeps at least 243.

Gate: green on `PRIMUS_X2_v1`, no new `src/` warnings; `Debugging.cpp` warnings went from 10 to 4 ( the six `strlen`
length conversions are gone ). Flash 121.5 KB / 256 KB, RAM 20.5 KB / 40 KB.

## Task 3: `INT_MIN` prints correctly ( D3 )

[src/main/API-Src/Debugging.cpp](../../../../src/main/API-Src/Debugging.cpp), `debugPrint ( msg, int )`

| Where | Change | Why |
|---|---|---|
| Sign handling | `uint32_t magnitude = ( uint32_t ) number`, and for a negative number `magnitude = 0u - magnitude`; a `static_assert` checks that `int` fits `uint32_t`. | D3: `number = -number` overflowed an `int` for `INT_MIN` ( undefined behaviour ) and stayed negative. |
| Digit loop | Works on `magnitude`: `uint8_t digit = magnitude % 10u`, `*--str = ( char ) ( '0' + digit )`; the dead `'A' - 10` branch is gone. | The negative remainders of `INT_MIN` printed as garbage characters. |
| Buffer | Sized from `sizeof ( magnitude )`: 33 bytes, as before. | Unchanged size. |

Hand trace: `INT_MIN` -> `-2147483648`, `INT_MAX` -> `2147483647`, 0 -> `0`, -7 -> `-7`. The length code from task 2
is unchanged and still counts the sign and the digits.

Gate: green on `PRIMUS_X2_v1`, no new `src/` warnings; `Debugging.cpp` warnings went from 4 to 3 ( the `int` to `char`
digit conversion is gone ). Flash 121.5 KB / 256 KB, RAM 20.5 KB / 40 KB.

## Task 7: documentation

| File | Change |
|---|---|
| `docs/API/DEBUGGING_API_WIKI.md` ( new ) | All six functions with an example each in its lifecycle hook; precision and rounding; range, NaN and infinity; how PlutoMonitor shows the output; bytes per call and the ~130 B/tick budget; the 255 cut as a framing limit; pitfalls; Changes in 1.8.9. |
| [src/main/API/Debugging.h](../../../../src/main/API/Debugging.h) | Comments only: precision 0..4 and rounding, overflow / NaN / infinity, the 255 cut, NULL, the 130 B budget, the `3.7f` pitfall. Banner updated. No signature change. |
| `docs/fw-development-reference/dev-guide/FLIGHT_INVARIANTS.md` | The "prints 0 after the first decimal" sentence replaced by the frame limit and a pointer to the wiki. |
| `docs/API/FC_DATA_API_WIKI.md` | The "print the x100 integer" workaround replaced by a decimal print. |
| `.claude/TOOLING_BACKLOG.md` | Item 11: the digit bug marked fixed; the TX-ring room check stays open. |
| `PIPELINE_UPDATE.md` ( new ) | Staged for commit: a "Debug output" bullet for `User_Space_API.md`, and the CLAUDE.md line that still states the old bug. |

Not changed: records of closed topics that mention the old bug ( `baro-temperature-api`, `tof-althold-fusion` ); they
describe the firmware of their time.

### Claim check ( in place of `spec-to-code-compliance`, not available in this session )

Every statement in the wiki and the header, against the line or test that shows it:

| Claim | Evidence |
|---|---|
| `precision` 0..4; 5 or more prints 4 | `Debugging.cpp:53` ( `MONITOR_MAX_PRECISION 4` ), `:334`, `:358`; host rows @ 7 and @ 9; hardware R03 |
| Rounded half away from zero | `:155-165`: the sign is removed, then 0.5 / 10^d is added before the cut; host rows 2.5 @ 0 -> `3`, -2.5 @ 0 -> `-3`, 1.99999 @ 2 -> `2.00` |
| Small negatives keep the sign ( `-0.00`, `-0.50` ) | `:155-158`; host rows; hardware R05, R06 |
| Beyond +/-4294967040 prints `overflow` | `:149-151`; host rows +/-5e9; hardware R13 |
| NaN -> `not a number`, infinity -> `infinite` | `:137-146`; host rows; hardware R14-R16 |
| Every `int` prints in full | `:259-262`; host `INT_MIN` / `INT_MAX`; hardware R17, R18 |
| At most 255 characters; the tag is cut, never the number | `:57`, `:63-70`, `:86-92`, and each variant's `msgLen` line; host long-tag rows |
| `NULL` counts as empty | `:65`; host NULL rows |
| Bytes per call: 4 + payload; + 5 for `Println` | Frame `'$' 'D' len payload chk`; host byte counts ( `"x\t3.142"` 11 B, its `Println` 16 B ) |
| A 255-character call is 259 B, more than the 256-byte ring | `drivers/serial_uart.h:31`; host 259-byte frames |
| ~130 B per loop; ~180 B disconnected the app | `dev-guide/FLIGHT_INVARIANTS.md:49-54` |
| One line per frame in PlutoMonitor; tab and leading space dropped; no line for `"\n"` | Task 6 log ( `TESTING.md` ) |
| `Monitor_Print ( "v:", 3.7f )` prints 3 and warns `-Wfloat-conversion` | Compiled with the firmware command for `PlutoPilot.cpp`: `conversion from 'float' to 'int' changes value from '3.70000005e+0f' to '3'` |
| A `float` passed as the `double` needs no cast and gives no warning | Same compile: no warning. ( The first draft said it warned `-Wdouble-promotion`; corrected. ) |
| Every wiki example compiles clean | Same compile of all examples: no warning, no error |
| `plutoLoop ( )` every 100 ms by default | `API-Utils.cpp:102` ( `userLoopFrequency = 100000` ), `mw.cpp:1192` |
| `Bms_Get ( Voltage )` is in mV | `API/BMS.h:38` |
| `onLoopStart ( )` runs again after a link drop | `User_Space_API.md` "Link drop stops user code" |
| Changes in 1.8.9: digits, hang, length wrap, `INT_MIN`, NaN / inf length | Task 4 pre-fix run ( each reproduced ) and fixed run; task 6 hardware |

Gate after the header change: green on `PRIMUS_X2_v1`, no new `src/` warnings, flash 121.5 KB / 256 KB, RAM 20.5 KB / 40 KB.

## Task 8: topic review fixes

`pluto-reviewer` over the whole topic diff found nothing blocking in the code. All seven findings were in the docs
and records, and all are applied:

| # | Finding | Fix |
|---|---|---|
| 1 | "one frame per call" was wrong for `Println`, which sends a second 5-byte `"\n"` frame | Reworded in `PIPELINE_UPDATE.md` ( both the `User_Space_API.md` bullet and the staged CLAUDE.md line ) and in `dev-guide/FLIGHT_INVARIANTS.md` |
| 2 | Two stale README open items ( "not yet shown on hardware", "task 7 may add the precision range" ) | Deleted; both were done |
| 3 | `TASKS.md` task 6 linked `src/main/PlutoPilot.cpp`, which does not exist | Now `../../../../PlutoPilot.cpp` ( repo root ) |
| 4 | Task 8's status line said `todo` while the index said `in-progress` | Both now `done` |
| 5 | Stale line citations for the fraction loop ( `:207-221`, `#L203` ) | `CHANGES.md` and `README.md` cite `:224-238`; `TASKS.md` and `INVESTIGATION.md` say which numbers are pre-fix |
| 6 | The wiki said "about 46 bytes per call" for a three-call example | Now "for the whole loop" |
| 7 | `Monitor_Println ( msg )` lacked the 255-character and NULL note its `Print` twin has | Added, with the second-frame note |

Re-checked after the fixes: gate green on `PRIMUS_X2_v1` ( no new `src/` warnings, flash 121.5 KB / 256 KB, RAM
20.5 KB / 40 KB ), every relative link in the topic docs and the new wiki resolves ( 0 broken ), no em or en dash in
any added line, and the repo-root `PlutoPilot.cpp` equals `HEAD`.

The reviewer also re-derived the hover check from the log itself ( 633 / 633 exact ) and confirmed the byte formulas,
the 255 / 259 framing, the clamp, the rounding examples and the +/-4294967040 bound against the code.
