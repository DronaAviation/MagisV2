# Pipeline update ( oled-print-linkage )

[README](README.md) · [CHANGES](CHANGES.md)

Staged text, applied at commit by `pluto-commit`. Do not edit `fw-architecture-pipeline/` directly.

## `fw-architecture-pipeline/subsystems/User_Space_API.md`

In the deferred-OLED bullet at line 97, after the sentence about `Oled_Mode ( )` starting from a blank frame, add:

> `Oled_Print ( col, row, text )` is the one `Oled.h` function whose body is in a C file
> ( `io/oled_display.c` ), so its declaration is `extern "C"`; before FW 3.15.12 it was declared with C++ linkage
> and user code calling it could not link. It draws on the 21 x 8 text grid of the `System` screen and does nothing
> in `User` mode or outside `col` 0-20 / `row` 1-6.

## `CHANGELOG.md`

Under the next release's `### Fixed`:

> - **OLED ( user API )**: `Oled_Print ( col, row, text )` can be called from user code. It was declared with C++
>   linkage in `API/Oled.h` while its body is in the C file `io/oled_display.c`, so a call from `PlutoPilot.cpp`
>   failed to link with `undefined reference to Oled_Print(unsigned char, unsigned char, char const*)`. Nothing in the
>   firmware calls it, so no firmware build ever caught it. The declaration is now `extern "C"`; no signature or
>   behaviour change, and the flashed image is unchanged. It remains `System` mode only, with `col` 0-20 and `row` 1-6
>   ( rows 0 and 7 are the status rows ). Wiki: `docs/API/OLED_API_WIKI.md`.
