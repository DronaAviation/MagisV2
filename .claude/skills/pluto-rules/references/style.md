# MagisV2 style and conventions

## Formatting

A `.clang-format` is present at the repo root. Match it, and match the
surrounding file where the two disagree.

The most visible convention is the **spaced-paren call style**:

```c
Oled_Text ( 0, 0, "Altitude" );
userRCassert ( ROLL, 0.0f );
const int32_t maxRate = ( deflection > 0 ) ? ALT_MAX_CLIMB_CMS : ALT_MAX_DESCENT_CMS;
```

Other settings worth knowing (from `.clang-format`):

| Setting | Value |
|---|---|
| `AlignAfterOpenBracket` | `AlwaysBreak` |
| `AlignConsecutiveAssignments` | on, not across empty lines or comments |
| `AlignConsecutiveMacros` | on, across empty lines |
| `AlignTrailingComments` | always |
| `AllowShortIfStatementsOnASingleLine` | `WithoutElse` |
| `AllowShortLoopsOnASingleLine` | true |
| `AllowShortFunctionsOnASingleLine` | `Empty` only |

## Banner headers

Every source file carries a banner block. **Preserve it when editing** — update
`Last Modified` and `Modified By`, and add a HISTORY row for a substantive
change. Do not delete or regenerate it.

```c
/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Cleanflight & Drona Aviation                  #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\flight\altitudehold.cpp                                    #
 #  Created Date: Sat, 22nd Feb 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Wed, 19th Aug 2026                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/
```

A new file gets the same block with its own path, author and created date.

## Units and scaling

This tree mixes several fixed-point scales in the same functions. Most bugs of
the "it was off by 10×" kind come from crossing one of these boundaries without
noticing.

| Quantity | Unit in code | Notes |
|---|---|---|
| Pressure | Pa (float) | `BARO_COMP_*` constants are Pa or Pa per unit |
| Altitude, position | cm (int32) | not metres |
| Velocity | cm/s (int32) | |
| RC channels | counts, 1000–2000 µs | mid is `masterConfig.rxConfig.midrc`, nominally 1500 |
| Angles | **deci-degrees** (int16) | `300` is 30°, `> 30` means 3° — this exact confusion was a real bug |
| Loop timing | µs | `looptime`, `dTime`; convert with `1e-6f` |
| Task periods | ms | `executePeriodicTasks()` |
| Temperature | °C (float) | F3 internal sensor slope is **negative** |

Name the unit when it is not structurally obvious — `altRate_cms`,
`dTime_us`, or a trailing comment. The compiler will not catch a scale error;
`-Wconversion` only catches the ones that also change type.

## Naming

- Cleanflight-heritage files keep Cleanflight naming (`lowerCamelCase`
  functions, `snake_case_t` / `_e` types).
- Drona additions under `API/` use `Pascal_Snake` public entry points
  (`RcCommand_Set`, `Oled_Text`, `Monitor_Print`, `RGB_Init`).
- Public API headers in `src/main/API/` are the stable surface. Keep signatures
  stable; changing one means updating `docs/API/<NAME>_API_WIKI.md` and bumping
  `FW_Version` / `API_Version` in the Makefile.

## Language and files

- C compiles as `gnu17`, C++ as `gnu++17`. `main.cpp`, `mw.cpp` and
  `PlutoPilot.cpp` are C++.
- A C header included from C++ needs `#ifdef __cplusplus extern "C" { … }`.
- `src/main/` is 49 `.c` and 89 `.cpp` files — new code is usually `.cpp`.
- **`lib/main/` is vendored upstream** (CMSIS, STM32F30x StdPeriph, USB-FS,
  VL53L0X/VL53L1X). Never reformat, tidy or "fix warnings" in it. It accounts
  for ~1700 of the build's warnings and that is expected.
- **`src/test/` does not build** and is not part of any workflow. Do not revive
  it unless explicitly asked.

## Comments

Match the density of the surrounding file. The flight and sensor code carries
substantial explanatory comments where a constant was derived from measurement
(see `sensors/barometer.cpp` and `flight/altitudehold.cpp`) — when you change
such a constant, change the comment that justifies it too, or the next reader
tunes against a stale number.
