# Monitor_Print double precision - Scout brief

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md)

`pluto-scout` brief from planning ( 15 Nov 2026 ), verbatim.

---

## Scout brief: Monitor_Print / Monitor_Println print 0 after the first decimal (B9), plus an audit of every Monitor_* function
Graph: stale (built 3be9093e, HEAD c344387b). Only `Debugging.cpp` changed in scope, and the call path is short, so I traced it by reading the source and confirmed it in the `Build/PRIMUS_X2_v1` object file.

**Subsystems:** user API Debugging, MSP serial TX. No pipeline doc covers it (`fw-architecture-pipeline/` has no match for Monitor/debugPrint).

**Affected files / key code**
- `src/main/API/Debugging.h:45,52` are the C-linkage `Monitor_Print/Println(const char*)`. Lines `:78,83,88,93` are the C++ int and double overloads.
- `src/main/API-Src/Debugging.cpp:50` `serializeString` sends each character and XORs it into the checksum.
- `Debugging.cpp:62` `debugPrint(msg)`, `:85` `debugPrint(msg,double,digits)`, `:215` `debugPrint(msg,int)`. These build the frame `'$' 'D' len payload chk`.
- `Debugging.cpp:280-323` are the six `Monitor_*` wrappers. Each Println sends a second packet, `debugPrint("\n")`.
- `src/main/io/serial_msp.cpp:449` `serialize8Debug`: `serialWrite(mspSerialPort,a)`, which does not check whether the TX ring is full.

**Mode:** fix

**Current behaviour:** The double path clamps precision to 0..7 (`:294`, `:318`), removes the sign (`:138`), adds 0.5/10^digits for rounding (`:144-147`), and prints the integer part through a reversed buffer (`buf[33]`). It then writes the fraction one digit at a time (`:201-208`). Rounding is already the intended behaviour.

**Causal chain (proven)**
```
203  uint8_t toPrint = ( uint8_t ) ( remainder );
204  toPrint = toPrint < 10 ? toPrint + '0' : ...;   // toPrint is now ASCII
207  remainder -= toPrint;                           // subtracts 48+d, not d
```
Worked example, 3.14159 at precision 3:
- After rounding the number is 3.14209 and the remainder is 0.14209.
- Digit 1: x10 = 1.42, so it prints '1'. Then remainder = 1.42 - 49 = -47.58.
- Digit 2: x10 = -475.8. The cast at `:203` compiles to `__aeabi_d2uiz` (4 calls in `Debugging.o`), which returns 0 for negative input. So it prints '0', and every later digit is '0' too. Output: "3.100".

Precision 1 works because only one digit goes through the loop. -2.5 works because the sign is removed before the cast. The minimal fix is in `:203-207`: subtract the digit before turning it into ASCII. The output length is unchanged (`data_size` at `:168` already counts the digits), the flash change is about 0, and there is no signature change.

**Per-function audit** (bytes = whole frame: 4 framing bytes + payload; T = strlen(tag))

| Function | Impl | Worst bytes | Defects |
|---|---|---|---|
| `Monitor_Print(msg)` .h:45 | .cpp:280 -> :62 | T+4 | (a) `:52` `for(uint8_t i...; i<strlen(msg)...)`: a string of 256 characters or more makes `i` wrap, the loop never ends and the loop() hangs. (b) `:72` length is sent as one byte, so it wraps at 256. (c) No length cap, so one call can break the ~130 B budget. |
| `Monitor_Println(msg)` .h:52 | :300 | T+9 | Same as above. The newline is a separate 5-byte packet `$D 01 '\n' 0B`. |
| `Monitor_Print(tag,int)` .h:78 | :286 -> :215 | T+16 | INT_MIN: `:227` `number=-number` overflows (undefined behaviour) and stays negative. `:242` `%10` then gives negative digits, so `:244` prints `"-./,),(-*,("` instead of the number. Also defects (a)-(c) on the tag. |
| `Monitor_Println(tag,int)` .h:88 | :308 | T+21 | Same as the int Print. |
| `Monitor_Print(tag,double,p)` .h:83 | :292 -> :85 | T+24 | **B9** at `:203-207`. NaN length short by one byte at `:97` (no +1 for the tab, unlike `:125`); inf has the same bug at `:111`, so the app gets a bad length/checksum (whether it then drops the line depends on the app parser). Precision above 7 is clamped without any notice (`:294`). Numbers beyond +/-4294967040 print "overflow". Values in (-1,0) keep their sign (-0.5 prints "-0.5"). -0.0001 at p=2 prints "-0.00" (cosmetic). |
| `Monitor_Println(tag,double,p)` .h:93 | :316 | T+29 | Same as the double Print. |

Overload pitfall: `Monitor_Print("x", 3.7f)` with two arguments can only match the `int` overload (`.h:78`), so it prints 3. User code gets a `-Wfloat-conversion` warning, but nothing at run time.

**Constraints / invariants**
- Monitor_Print budget ~130 B per tick (FLIGHT_INVARIANTS.md:48). The B9 fix adds no bytes, but Println costs 5 extra bytes per call.
- `pluto-rules` SKILL.md:201-203 exempts this file's double overloads from the no-double rule. Keep the `double` parameter; it is software double, and the helpers `dmul/ddiv/d2uiz/i2d` are already linked. `debugPrint(double)` is 0x298 = 664 B of flash.
- `-Wconversion`: `tools/warnings-baseline.json:103-106` already lists 10 warnings for this file. The fix must not add any, and removing some is allowed.
- Stack: `buf[33]` is fine.
- `common/printf.cpp` has no `%f`, so it is not an alternative.
- Behaviour fix: CLAUDE.md asks for an `API_Version`/`FW_Version` bump in the Makefile and a wiki update.

**Resource conflicts:** none.

**Already answered by the code:** rounding (not truncation) is intended (`:144`). The sign is handled for (-1,0). Precision 0 prints no '.'. Maximum precision is 7. The module is registered in Makefile:309.

**Questions for the user**
1. Fix the bugs beyond B9 in this topic: the 256+ character hang, INT_MIN, NaN/inf length? Or keep this topic to B9 only?
2. Cap tag/msg length (for example 64 characters, then truncate) to protect the TX budget, or leave it unbounded?
3. Keep the silent clamp to 7 digits, or flag precision above 7?
4. Keep the newline as a separate packet (+5 B), or is the app parser fixed?
5. Should "-0.00" print as "0.00"?
6. Create `docs/API/DEBUGGING_API_WIKI.md`? None exists today.
7. Which API_Version bump?

**Doc drift / docs that change**
- `CLAUDE.md:134` and `docs/fw-development-reference/dev-guide/FLIGHT_INVARIANTS.md:56`: delete the "prints 0 after first decimal" sentence.
- `docs/API/FC_DATA_API_WIKI.md:46-48`: the "print the x100 integer" workaround is no longer needed.
- `pluto-rules` SKILL.md:201-203: the exemption still applies.

**History:** `5043a65` (debugging consolidation; the `remainder -= toPrint` line dates from this commit), then `b06ea6d` (OLED split). No active-development topic covers this.

**Suggested skills:** pluto-rules, pluto-build (`--gate PRIMUS_X2_v1`), pluto-flighttest (bench PlutoMonitor table: 3.14159/3, -2.5/1, -0.5/2, 0/0, 1e9/7, NaN, INT_MIN, INT_MAX, 255- and 256-character strings, Println variants), pluto-commit.

**Pipeline docs to update at the end:** none in `fw-architecture-pipeline/`. Update the dev-guide, CLAUDE.md and the docs/API wiki as listed above.
