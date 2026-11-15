# Monitor_Print double precision - Investigation

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md)

## Problem

API audit B9 ( Sr 84, test T01 ): the `double` overloads of `Monitor_Print` and `Monitor_Println`
( [Debugging.h:83,93](../../../../src/main/API/Debugging.h#L83) ) print 0 for every digit after the first decimal.
T01: `3.14159` with 3 digits prints `3.100`; it should print `3.142`. With 1 digit the output is right ( `-2.5` ).

## Root cause ( proven by reading the code and the object file )

Every line number below is the **pre-fix** file at `HEAD c344387`, so the links point at lines that have since moved.
The fixed positions are in [CHANGES.md](CHANGES.md).

The fraction loop at [Debugging.cpp:201-208](../../../../src/main/API-Src/Debugging.cpp#L201) converts the digit to
ASCII **before** it subtracts the digit from the remainder:

```
remainder *= 10.0;
uint8_t toPrint = ( uint8_t ) ( remainder );    // :203  digit d
toPrint = toPrint < 10 ? toPrint + '0' : ...;    // :204  now '0' + d = 48 + d
serialize8Debug ( toPrint );
checksum ^= toPrint;
remainder -= toPrint;                            // :207  subtracts 48 + d, not d
```

For 3.14159 @ 3: the rounded remainder is 0.14209 -> digit `1`, then remainder = 1.42 - 49 = -47.58. Next pass
x10 = -475.8; the `double -> uint8_t` cast compiles to `__aeabi_d2uiz`, which returns 0 for negative input, so every
later digit is `0`. One digit works because the bug only bites from the second pass on.

## Other defects found by the audit ( in scope: all Debugging.h prints )

| ID | Where | Defect |
|---|---|---|
| D1 | [Debugging.cpp:52](../../../../src/main/API-Src/Debugging.cpp#L52) | `serializeString` loops with a `uint8_t` index against `strlen ( )`: a string of 256+ characters never ends the loop and hangs `loop ( )`. |
| D2 | `:72` and the other length bytes | The payload length is one byte; a tag + number payload over 255 bytes wraps it and the frame is corrupt. |
| D3 | [Debugging.cpp:227](../../../../src/main/API-Src/Debugging.cpp#L227) | `INT_MIN`: `number = -number` overflows ( undefined ) and stays negative; `% 10` gives negative digits, so garbage characters are printed. |
| D4 | `:97`, `:111` | NaN / inf payload length is one byte short ( the tab is not counted, unlike `:125` ), so length and checksum disagree with the bytes sent. |
| D5 | `:294`, `:318` | Precision is clamped to 7 silently. Changed by decision to 4. |
| D6 | `Debugging.h:78` | `Monitor_Print ( "x", 3.7f )` picks the `int` overload and prints 3. API pitfall, documented only ( no signature change ). |

Not defects: values in ( -1, 0 ) keep their sign; precision 0 prints no `.`; rounding is intended ( `:144-147` ).

## Q&A that shaped the plan ( 15 Nov 2026 )

| Question | Answer |
|---|---|
| Mode | `fix` |
| Target | `PRIMUS_X2_v1` |
| Done when | A bench PlutoMonitor log prints a fixed test table correctly for every overload, plus a short hover with prints running ( app stays connected, loop time unchanged ). |
| Scope | Every function in `Debugging.h`, not only the double overloads. |
| Rounding | Round half away from 0 ( T01 expects 3.142 ). |
| Max digits | Clamp precision to 4 ( was 7 ). |
| Fix size | Proper fix, no API signature change, no heap, no printf float. |
| String length | Truncate at 255 characters ( the user chose this over a 64-character cap ). |
| Newline | Keep the separate 5-byte packet. |
| Negative zero | Keep `-0.00`. |
| Docs / version | New `docs/API/DEBUGGING_API_WIKI.md`; API 1.8.8 -> 1.8.9, FW 3.15.10 -> 3.15.11 at commit. |
