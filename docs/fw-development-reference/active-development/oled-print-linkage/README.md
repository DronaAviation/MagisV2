# Oled_Print linkage

[README](README.md) · [CHANGES](CHANGES.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

| | |
|---|---|
| **Status** | **Closed - pending commit** ( 15 Nov 2026, FW 3.15.12, API 1.8.10 ) |
| **Mode** | `fix` |
| **Branch** | `BugFix-June26` |
| **Target** | `PRIMUS_X2_v1` ( all targets built at commit ) |
| **Last updated** | 15 Nov 2026 |
| **Pipeline docs** | [User_Space_API.md](../../fw-architecture-pipeline/subsystems/User_Space_API.md) ( one bullet, via [PIPELINE_UPDATE.md](PIPELINE_UPDATE.md) ); `docs/API/OLED_API_WIKI.md` |

**Problem.** `Oled_Print ( )` could not be called from user code: the link failed with
`undefined reference to Oled_Print(unsigned char, unsigned char, char const*)`. The public header
[Oled.h](../../../../src/main/API/Oled.h#L94) is C++ only ( no `extern "C"`, and `Oled_Text ( )` at `:112` takes a C++
default argument ), so the declaration had C++ linkage and every call site asked the linker for a mangled name. The
definition lives in [oled_display.c:536](../../../../src/main/io/oled_display.c#L536), which is compiled as C, so the
object exports the plain symbol `Oled_Print` ( `arm-none-eabi-nm` on `oled_display.o`: `T Oled_Print` ). It is the only
function in `Oled.h` whose body sits in a C file; the other 20 are C++ wrappers in
[API-Src/Oled.cpp](../../../../src/main/API-Src/Oled.cpp) over the `Oled_Draw*` helpers. No firmware code calls
`Oled_Print`, so no firmware link ever failed and the mismatch survived since the function was added in `c443a65`
( 3 Apr 2026 ).

Found on 15 Nov 2026 while cross-checking the public GitHub wiki against the API, by a harness that compiles and
links each wiki example against the built firmware: the `Oled_Print` example was the one block that would not link.
`docs/API/OLED_API_WIKI.md` had documented the call as working since 14 Oct 2026. That wiki work is a separate topic,
not yet committed; this fix stands on its own.

**Fix.** Declare that one function `extern "C"` in `Oled.h`, so the declaration matches the C definition. The
alternative, moving the body behind a C++ wrapper in `API-Src/Oled.cpp` like its 20 siblings, was offered and the user
chose the one-line header fix. Nothing else changes: no signature change, no behaviour change for any existing caller,
and the linked image is unchanged because nothing in the firmware calls it. Details: [CHANGES.md](CHANGES.md).

## Gotchas

- **`Oled.h` cannot be compiled as C**, so the bare `extern "C"` needs no `#ifdef __cplusplus` guard: the header
  already uses a C++ default argument at `:112`, and the only translation unit that includes it is the C++
  `API-Src/Oled.cpp`. A guard would be dead code here.
- **`io/oled_display.h` does not declare `Oled_Print`**, which is why no translation unit ends up with two
  declarations of different linkage ( `API-Src/Oled.cpp` includes `API/Oled.h` and `io/oled_display.h` together ).
- **`Oled_Print ( )` is `System` mode only** and silently does nothing otherwise, and it rejects `col > 20`,
  `row < 1`, `row > 6` and a NULL string ( `oled_display.c:537-541` ). That is now on the wiki.

## Open items

- Fix done ( 15 Nov 2026 ): `extern "C"` on the declaration, a `@note` saying why, banner and HISTORY updated, and
  the two en dashes in that function's `@param` lines replaced with ASCII. Clean gate on `PRIMUS_X2_v1` green, no new
  `src/` warnings, flash 121.5 KB / 256 KB, RAM 20.5 KB / 40 KB. Verified by linking the wiki's own `Oled_Print`
  example: `MagisV2.wiki/Oled.md` goes from 1 link failure to 11 of 12 blocks passing ( the last is an enum quote ).
- Out of scope, left as it was: `Oled.h` still contains 22 pre-existing en dashes in other functions' doc comments
  ( lines ~110-255 ), and the same declared-but-unusable class of defect is still open elsewhere: `SPI_Init ( )`
  with no arguments is declared in `API/Serial-IO.h` and never defined, and `RcData_Get ( void )` in
  `API-Src/RC-Interface.cpp` returns the address of a local array, which GCC compiles to `return NULL`. Neither is
  fixed here.
- Not hardware-validated, and it does not need to be: the change is a declaration, the image is unchanged, and the
  function's behaviour on the panel is unchanged. A user calling it from `PlutoPilot.cpp` is the test.

## Next action

None: closed pending commit, which `pluto-commit` stages. The user commits.
