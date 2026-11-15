# Monitor_Print double precision - Testing

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md)

## Task 4: host check of every formatter ( 15 Nov 2026 )

**Result: the fixed file passes 86 / 86 checks; the `HEAD` file fails 51 / 83 and reproduces every planned bug.**

### What is compiled

The **real** [Debugging.cpp](../../../../src/main/API-Src/Debugging.cpp) and the real `src/main/common/maths.cpp`
( for `constrain` ), built for the PC with MinGW g++ 14.3.0, `-std=gnu++17 -O1 -Wall -Wextra -ftrapv`.

- `-ftrapv` aborts on any signed overflow, so a pass also shows that the new `INT_MIN` path has no signed overflow.
- Headers come from a scratchpad overlay first, then the real `src/main`:
  - `io/serial_msp.h` declares only `serialize8Debug ( uint8_t )`, with the firmware signature ( `io/serial_msp.h:38` ).
  - `platform.h`, the five `drivers/` headers, `blackbox/blackbox.h` and `API/API-Utils.h` are empty.
  - `API/Debugging.h`, `common/maths.h` and `common/axis.h` are the real files.
- `BLACKBOX` is not defined, so `BlackBox_setVar` is not built.
- The only build warning is host-only: `maths.h:55` redefines the host `float.h` `FLT_EPSILON`.

The test replaces `serialize8Debug ( )` with a capture buffer and parses the captured bytes as debug frames:
`'$' 'D' len payload checksum`, where checksum = len XOR every payload byte. Each case checks all of these:

- the frame header;
- that the length byte matches the bytes actually sent;
- the checksum;
- the payload text, against the expected value.

A print that sends more than 100 000 bytes is stopped and reported as a runaway: that is the old 256-character hang.

**Why the PC result carries over to the drone.** x86-64 SSE2 `double` and the ARM soft-float `double` helpers in
libgcc are both correctly rounded IEEE 754 binary64, so the fixed arithmetic gives the same digits on both. The fix
also removes the one platform-dependent step: the fraction digit is now cast from a value in [ 0, 10 ), which is
defined everywhere.

### Test table ( fixed file, 85 rows: every row run through both `Print` and `Println` )

| Group | Cases | Result |
|---|---|---|
| double, tag `"x"` | T01 3.14159 @ 3 -> `3.142`; @ 4 -> `3.1416`; @ 7 and @ 9 -> `3.1416` ( clamp 4 ); -2.5 @ 1; -0.5 @ 2 -> `-0.50`; -0.0001 @ 2 -> `-0.00`; 0 @ 0; 2.5 @ 0 -> `3`; -2.5 @ 0 -> `-3`; 1.99999 @ 2 -> `2.00`; 0.999996 @ 4 -> `1.0000`; 123.456 @ 2 -> `123.46`; 1e9 @ 4; 4294967040 @ 0; +/-5e9 -> `overflow`; NaN -> `not a number`; +/-inf -> `infinite` | 40 / 40 |
| int, tag `"n"` | 0, 7, -7, 123456, `INT_MAX` -> `2147483647`, `INT_MIN` -> `-2147483648` | 12 / 12 |
| string only | `""`, `"hello"`, 255 characters, 300 characters ( cut to 255 ) | 8 / 8 |
| long tags | empty tag; 255 / 300-character tag with 3.14159 @ 3 ( tag cut to 249 ), with -4294967040 @ 4, the longest number ( 238 ), NaN ( 242 ), inf and overflow ( 246 ), `INT_MIN` ( 243 ), 7 ( 253 ) | 22 / 22 |
| NULL tag | `Print ( NULL )`, `( NULL, 3.14159, 3 )`, `( NULL, -7 )`: sent as an empty tag | 3 / 3 |

Every `Println` sends its second frame, `"\n"`, correctly framed. The longest frame is 259 bytes ( 255 payload + 4 ),
matching the task 2 review note: it does not fit the 256-byte TX ring ( decisions log ).

The first run reported 2 failures, in `( a<300>, 7 )`. The test expected the tag cut to 252 characters; the firmware
cut it to 253, which is correct ( 255 - tab - one digit ). The expectation was fixed, the firmware was not changed.

### Sweep against `snprintf`

200 000 random values: magnitudes from 1e-6 to 4.29e9, both signs, precision 0..7 ( fixed seed 20261115 ). Each value
was compared with `snprintf ( "%.*f", min ( p, 4 ), v )`.

| Outcome | Count |
|---|---|
| Identical | 199 998 |
| Differs only at a rounding tie | 2 |
| Wrong | **0** |

The two ties: `149794035.61205 @ 4` -> firmware `.6121`, `snprintf` `.6120`; `-246371048.30655 @ 6` ( clamped to 4 )
-> `.3066` vs `.3065`. Each value lies within double rounding of an exact half. `snprintf` rounds the exact binary
value; the firmware adds 0.5 / 10^d in `double` first, and that addition can tip a near-tie up by one in the last
digit. It needs about 13 significant digits ( integer part of 1e8 or more, with 4 decimals ), so it never affects
a sensor value printed on this drone. Recorded as a known limit, not a defect.

### Pre-fix comparison ( `HEAD c344387`, `-DPREFIX -fwrapv` )

The same harness on `git show HEAD:src/main/API-Src/Debugging.cpp`. `-fwrapv` lets the `INT_MIN` overflow wrap
instead of trapping, so the old output can be seen. `-DPREFIX` skips the NULL rows, which crash the old
`strlen ( NULL )`. Result: **32 / 83 pass**, and every planned defect shows:

| Defect | Pre-fix output |
|---|---|
| **B9 / T01**: digits after the first | 3.14159 @ 3 -> `3.1\n` ( a backslash and `n` ); 123.456 @ 2 -> `123.4^`; 1.99999 @ 2 -> `2.0X`. The sweep: 103 567 of 200 000 wrong. One-digit cases ( -2.5 @ 1, 2.5 @ 0 ) pass, as reported. |
| D1: 256+ character hang | Every 300-character string or tag: runaway, the loop never ends. |
| D2: length byte wrap | 255-character tag with any number: checksum mismatch, length byte 5 ( 261 wrapped ) or 11 ( `INT_MIN` ). |
| D3: `INT_MIN` | `-./,),(-*,(` instead of `-2147483648`. |
| D4: NaN / inf length | Checksum mismatch, length byte 13 ( NaN ) / 9 ( inf ): one short. |

**Why the PC shows `3.1\n` where the drone showed `3.100`.** After the first digit, the old loop leaves the
remainder near -47, and casting a negative `double` to `uint8_t` is undefined behaviour:

- **On the STM32**, the cast is `__aeabi_d2uiz`, which saturates to 0, so every later digit prints `0` ( the T01
  report, confirmed in `Debugging.o` by the scout ).
- **On x86**, the cast wraps. -475.8 becomes 37, and 37 + `'A'` - 10 is `\`.

Same root cause, different wrong characters. The fixed loop never casts a negative value.

### How to re-run

The harness is in the session scratchpad and is not committed. A reboot can clear it:

`C:\Users\MechAsh\AppData\Local\Temp\claude\9df01950423a7843\f110ae4e-1d04-491d-960e-a1a7b9bc05cf\scratchpad\mp\`

It holds `test_monitor_print.cpp`, `run.sh` and `overlay/`. Run from the repo root:

```
<scratchpad>/mp/run.sh                                              # fixed file: RESULT PASS, exit 0
git show HEAD:src/main/API-Src/Debugging.cpp > <scratchpad>/mp/Debugging_HEAD.cpp
<scratchpad>/mp/run.sh <scratchpad>/mp/Debugging_HEAD.cpp -DPREFIX -fwrapv   # pre-fix: RESULT FAIL
```

## Task 5: build gate ( 15 Nov 2026 )

`PROJECT=AltX .claude/skills/pluto-build/driver.sh --gate PRIMUS_X2_v1`: a clean build ( the driver cleans first by
default ), exit 0, `ALL BUILDS PASSED`, `Build/PRIMUS_X2_v1/AltX_PRIMUS_X2_v1_3.15.10.hex` written.

| Check | Result |
|---|---|
| New warnings under `src/` | **none** |
| `Debugging.cpp` baselined warnings | 10 -> 3 ( the 7 removed are listed by the gate as "fixed in src/"; the baseline is refreshed at commit, never to hide a warning ) |
| Flash | 121.5 KB / 256 KB ( ELF text 123 108 B + data 1 316 B ) |
| RAM | 20.5 KB / 40 KB |

**`Debugging.o` against `HEAD`.** The `HEAD` file was compiled with the exact firmware command line ( taken from
`make -n -B` ) into the scratchpad:

| Function | `HEAD` ( B ) | Now ( B ) |
|---|---|---|
| `debugPrint ( msg, double, digits )` | 664 | 544 |
| `debugPrint ( msg, int )` | 180 | 188 |
| `debugPrint ( msg )` | 68 | 64 |
| `serializeString` | 48 | 36 ( now `static` ) |
| `boundedLength` ( new, `static` ) | - | 24 |
| `debugPrintTagText` ( new, `static` ) | - | 96 |
| The six `Monitor_*` wrappers | 164 | 164 |
| **Object `.text`** | **1 255** | **1 263 ( +8 B )** |

`.bss` is the 1-byte `checksum` in both, so there is no RAM change. Stack use is unchanged: the number buffer is still
33 bytes. **The flashed image does not change size today:** the current `PlutoPilot.cpp` calls no `Monitor_*`
function, so `-gc-sections` drops the whole module, and no `debugPrint` or `Monitor_*` symbol is in the ELF. User
code that prints pays the +8 B.

## Task 6: hardware validation ( plan, 15 Nov 2026 )

**Build under test:** `Build/PRIMUS_X2_v1/AltX_PRIMUS_X2_v1_3.15.10.hex` ( built 15 Nov 2026 with the task 1-3 fixes and
the temporary test code in the repo-root `PlutoPilot.cpp` ). Flash 126.3 KB / 256 KB with the print module linked,
RAM 20.5 KB / 40 KB. Receiver `Rx_ESP` ( Wi-Fi app ).

**Test code** ( temporary, restored with `git checkout -- PlutoPilot.cpp` before commit ):

- **Disarmed:** prints the bench table one row per tick at 10 Hz and repeats it. A pass is 23 ticks, about 2.3 s. Every
  tag carries the expected value before the colon, so each line checks itself: `T01 3.142:` must be followed by
  `3.142`. The rows:
  - the T01 case and the clamp at 9 digits;
  - -2.5, -0.50, -0.00, 0, 2.5 @ 0 -> 3, 1.99999 @ 2, 123.456 @ 2, 0.0123 @ 4 and 1e9 @ 4;
  - overflow, NaN, +inf and -inf;
  - `INT_MAX`, `INT_MIN` and -7;
  - one line built from `Print ( double )`, `Print ( int )` and `Println ( double )`;
  - one plain-text line, and one line of two strings.
- **Armed:** `t:` ( `millis`, ms ) `n:` ( tick count ) `cyc:` ( `cycleTime`, us ) `alt:` ( estimate, cm ) `ts:`
  ( `t / 1000`, 3 decimals ). `ts` must equal `t` / 1000 to the millisecond on every line, which tests the double
  path with live values.
- **Every Developer Mode ( re )start** prints `== START n:` on its own tick.
- **Bytes per tick:** at most 47 B on the bench ( the R20 line ) and about 78 B in the hover. Both are well inside
  the ~130 B budget.

The values are written as `( double ) 3.14159` and so on. With `-fsingle-precision-constant` they are float values
widened to double, the same as an unsuffixed literal in user code. The expected text accounts for that: each value
prints the same at its precision.

### One run: bench table, then hover, then table again -> `logs/log-1.txt`

The test code switches on the arm state, so a single Developer Mode session gives both halves and the whole thing is
one log. Props on throughout; the drone is simply disarmed for the first part, so the motors are off.

1. Props on, a fresh pack. Power on, connect the app over Wi-Fi, open PlutoMonitor.
2. **Developer Mode on, still disarmed: wait at least 30 s** ( about 13 table passes ). **Read the table on screen
   before arming**: if the values are wrong or the app drops out here, there is no reason to fly.
3. Arm, take off to about 1 m, **hover 60 s** with no stick input. The lines switch to the hover format by themselves.
4. Land and disarm, then leave it another **10 s**: the table comes back, which also shows the app survived the flight.
5. Save the whole PlutoMonitor output as `logs/log-1.txt`. Note anything the log cannot show ( a bump, wind, a visible
   sink ).

`logs/log-2.txt` is no longer used.

| Confirms the fix | Refutes it |
|---|---|
| Every table row prints the value written before its colon ( T01 `3.142`, R18 `-2147483648` ) | Any row with a trailing `0` run ( `3.100` ) or garbage characters |
| R14-R16 ( NaN / inf ) appear as complete lines | R14-R16 missing or cut in every pass: the app parser disagrees with the corrected length byte |
| R20 and R22 each come out on one line | A line break inside them |
| `ts` = `t` / 1000 exactly on every hover line | Any line where they differ |
| `cyc` median near the 3500 us `looptime`; max reported | `cyc` regularly far above 3500 us while printing |
| The table resumes after the landing, and the app never disconnects ( at most the usual Wi-Fi loss, about 1 record in 40, where `n` steps by 2 ) | A disconnect, or fields missing from many records ( a budget overrun ) |
| The drone flies as on any other build | Any change in handling ( the print code is the only change ) |

### Task 6 result: `logs/log-1.txt` ( 15 Nov 2026, PRIMUS_X2_v1, Wi-Fi app )

**PASS.** The session was flown twice; the second run is the one in `logs/log-1.txt` ( it overwrote the first, which
is no longer on disk ). Both passed every criterion. The numbers below are the second run, analysed with the
scratchpad scripts `mp/analyse_log.py` and `mp/steps.py`.

**Run 2 ( the saved log ):** 3 295 lines over 95.5 s:

- 04:52:29-04:52:35: table, disarmed;
- 04:52:35-04:54:01: armed, 85 s of it airborne at up to 175 cm;
- 04:54:01-04:54:05: table again after the disarm.

| Criterion | Result |
|---|---|
| Every table row prints the value before its colon | **Pass: 0 wrong in 88 rows** over 4 passes ( run 1: 0 wrong in 167 rows over 8 passes ). Every ID is seen 3-5 times, including T01 `3.142`, R03 ( clamp 9 -> `3.1416` ), R06 `-0.00`, R12 `1000000000.0000` and R18 `-2147483648`. |
| R13-R16 ( overflow, NaN, inf ) arrive as complete lines | **Pass:** 4 / 3 / 4 / 4 times, never cut. The app accepts the corrected NaN / inf length byte. |
| `ts` = `t` / 1000 on every hover line | **Pass: 633 / 633** ( run 1: 440 / 440 ). Every hover record arrived with all five fields. |
| `cyc` near the 3500 us `looptime` | **Pass:** min 3497, median 3505, p95 3511; 631 of 633 samples are 3497-3542 us. Two isolated samples read 23 060 and 22 790 us; see below. |
| No disconnect; only the usual Wi-Fi loss | **Pass.** One connect message, no disconnect, and the table resumed after the disarm. 10 hover records were lost ( `n` stepped by 2 or 3 with no `START` marker ), about 1 in 60. No record ever arrived partly, so the TX ring did not overrun. |
| Flies as on any other build | **Pass** ( the user: handling normal ). |
| R20 and R22 each on one line | **Not testable with this app.** PlutoMonitor shows every frame on its own line, drops the tab and the leading space, and shows no line for the `"\n"` frame, so R20 came out as `R20 1.5:1.5`, `42:42`, `2.25:2.25`. The firmware sends the same three frames as `HEAD` ( task 4 host check ), so this change did not cause the split. Whether older app versions joined them is not known. |

**Developer Mode restarts:** 25 `== START` markers in 95 s. Most take one tick ( the usual late Wi-Fi RC frame, as in
the battery-capacity-estimate reference ), which is why `n` steps by 2 after them; those are not lost records. Two were
much longer and were **the user switching Developer Mode off and on in flight**: 2.4 s at 04:52:59 ( climbing through
56 cm ) and 15.6 s at 04:53:27 ( hovering at 166 cm, back at 135 cm ). The drone flew on normally with user code off,
and printing resumed correctly both times, which also exercises the restart path.

**Two 23 ms `cycleTime` samples ( not caused by this change ).** At 04:52:48.850 ( `cyc` 23 060 ) and 04:53:56.952
( `cyc` 22 790 ) the main loop took about 23 ms instead of 3.5 ms. `cycleTime` is the previous main loop, and the user
loop samples it only once per tick, about 1 main loop in 28, so a rate of roughly 1 Hz would show up this often. Run 1
sampled 440 loops and caught none, which is consistent with the same rate.

This is not the print code: `uartWrite ( )` only writes a byte into the ring and starts DMA if it is idle
( `drivers/serial_uart.c:301-313` ), with no wait, and every loop in `Debugging.cpp` is bounded by 255 characters or
4 digits. The whole hover line is about 80 bytes. Recorded as an observation for a separate `check` topic: what the
main loop does for ~20 ms, roughly once a second.

**Observation outside this topic ( run 1 ).** At the end of run 1 the altitude estimate read -71 cm at touchdown and
then 180 cm 2.5 s later, just before the disarm. The user reports real movement: on low battery the drone bumped the
ground and took off again. Not caused by the print code; worth a separate `check` ( landing at low battery ).
