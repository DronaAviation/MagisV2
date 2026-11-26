# Changes ( oled-print-linkage )

[README](README.md) · [CHANGES](CHANGES.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

## File index

| File | Change |
|---|---|
| [src/main/API/Oled.h](../../../../src/main/API/Oled.h#L94) | `Oled_Print ( )` declared `extern "C"`; `@note` on why; two en dashes in its `@param` lines replaced with ASCII; banner `Last Modified` and a HISTORY row |
| [docs/API/OLED_API_WIKI.md](../../../API/OLED_API_WIKI.md) | `Oled_Print` section states the mode and range guards and the version it works from; new `Changes in 1.8.10` section |
| [PIPELINE_UPDATE.md](PIPELINE_UPDATE.md) | staged bullet for `fw-architecture-pipeline/subsystems/User_Space_API.md` |
| `Makefile` | `FW_Version` 3.15.11 -> 3.15.12, `API_Version` 1.8.9 -> 1.8.10 ( at commit, by `pluto-commit` ) |

## The one code change

`src/main/API/Oled.h:94`:

```diff
-void Oled_Print ( uint8_t col, uint8_t row, const char *text );
+extern "C" void Oled_Print ( uint8_t col, uint8_t row, const char *text );
```

**Why it is correct.** The definition at
[oled_display.c:536](../../../../src/main/io/oled_display.c#L536) is compiled as C and exports the plain symbol, which
`arm-none-eabi-nm Build/PRIMUS_X2_v1/bin/io/oled_display.o` shows as `T Oled_Print`. Parameter types and order match
the header exactly ( `uint8_t col, uint8_t row, const char *text` ).

**Why no `#ifdef __cplusplus` guard.** `Oled.h` cannot be compiled as C: `Oled_Text ( )` at `:112` takes a C++
default argument ( `bool inverted = false` ). The only translation unit in the tree that includes it is the C++
[API-Src/Oled.cpp](../../../../src/main/API-Src/Oled.cpp), plus user code through the repo-root `PlutoPilot.h:42`,
which is also C++.

**Why no double declaration.** `io/oled_display.h` declares the `Oled_Draw*` helpers inside its own `extern "C"`
block but not `Oled_Print`, so `API-Src/Oled.cpp`, which includes both headers, sees exactly one declaration.

## Verification

| Check | Result |
|---|---|
| Warning gate, clean build, `PRIMUS_X2_v1` | `no new warnings in src/`, `ALL BUILDS PASSED` |
| Flash / RAM | 121.5 KB / 256 KB, 20.5 KB / 40 KB ( unchanged: `text` 123108, `data` 1316, `bss` 19760 ) |
| The defect, before the fix | user code calling `Oled_Print ( 0, 1, "Battery OK" )` in `plutoLoop ( )` failed with `undefined reference to Oled_Print(unsigned char, unsigned char, char const*)` |
| The same code, after the fix | compiles and links. Every example of the public wiki's `Oled.md` compiled and linked against the built firmware: 12 blocks, 11 pass, 1 fails ( an enum quote, not a link ); it was 10 pass / 1 link failure / 1 enum quote |
| Harness self-test unchanged | 7 checked, 3 passed, 4 failed, 1 skipped, as expected |
| Flight behaviour | unchanged. Nothing in `src/` references `Oled_Print` except the declaration and the definition, so the linked image is the same; the only effect is that a user call now resolves |

## Behaviour the wiki now states, with its source line

| Statement | Source |
|---|---|
| Does nothing unless the OLED is in `System` mode | `oled_display.c:537` |
| Ignores a NULL string | `:539` |
| Ignores `col > 20`, `row < 1`, `row > 6` ( rows 0 and 7 are the status rows ) | `:541` |
| Draws into the system framebuffer and marks it dirty; the paced flush sends it, at most 16 bytes per loop | `:543-544`, `Oled.h:72-81` |
| Works from user code from FW 3.15.12 / API 1.8.10 | this change |
