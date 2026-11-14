# Set default PID profile - Changes

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md)

## Task 1: shared default-gain helper ( 14 Nov 2026 )

| File | Change | Why |
|---|---|---|
| [flight/pid_defaults.cpp](../../../../src/main/flight/pid_defaults.cpp) ( new ) | `pidProfileDefaultGains ( pidProfile_t * )`: the `P8` / `I8` / `D8` assignments for all eleven `pidIndex_e` slots, moved verbatim from `resetPidProfile ( )` with their comments ( the `PIDALT` P8 = 128 unity note included ). Includes `platform.h` so `pidProfile_t` has the same layout ( `GTUNE` ) as in every other unit. | One source of truth for the default gains, compilable without `config.cpp` ( host simulation, task 3 ). |
| [flight/pid.h:111](../../../../src/main/flight/pid.h#L111) | Declaration of `pidProfileDefaultGains ( )` inside the existing `extern "C"` block. | Reachable from `config.cpp` and the API layer. |
| [config/config.cpp:175-181](../../../../src/main/config/config.cpp#L175) | `resetPidProfile ( )` calls `pidProfileDefaultGains ( pidProfile )` in place of the 40-line gain block; `pidController`, `yaw_p_limit`, the cut frequencies, the float gains, `A_level` / `H_level` / `H_sensitivity`, `pid5_oldyw` and GTUNE stay here. Banner `Last Modified` updated. | Behaviour identical: same values, same order relative to the other fields ( no field reads a gain ). |
| [Makefile:178](../../../../Makefile#L178) | `flight/pid_defaults.cpp` added to `MAIN_FLIGHT`. | No source glob. |

Verification: the moved block diffed against the new function body ( blank lines aside ): identical. Gate on
`PRIMUS_X2_v1`: no new `src/` warnings, Flash 121.1 KB, RAM 20.5 KB.

## Task 2: `setDefault_PIDProfile ( )` rewrite ( 14 Nov 2026 )

| File | Change | Why |
|---|---|---|
| [API-Src/FC-Config-PID.cpp:146-169](../../../../src/main/API-Src/FC-Config-PID.cpp#L146) | Body replaced. A zero-initialised stack `pidProfile_t` ( 88 B ) is filled by `pidProfileDefaultGains ( )`, and `P8` / `I8` / `D8` are copied for the five slots in `apiSlot [] = { PIDROLL, PIDPITCH, PIDYAW, PIDALT, PIDUSER }` only. The three `static const int` tables and the `ROLL .. PIDUSER` ( 0 .. 10 ) loop are gone. Doc comment rewritten. | Ends the read past the tables, the writes into the six hidden slots and the stale ALT 100. `uint8_t` to `uint8_t` copy: no narrowing. |
| [API-Src/FC-Config-PID.cpp:25](../../../../src/main/API-Src/FC-Config-PID.cpp#L25) | The duplicate `#include <stdint.h>` became `#include <stddef.h>`. | `size_t` for the loop index. |
| [API/FC-Config.h:75-95](../../../../src/main/API/FC-Config.h#L75) | The comment "all axes" is replaced by: the five restored values, the internal gains left untouched, RAM only ( not saved ), the armed caveat ( call from `plutoInit ( )` or disarmed ), and the LuxFloat ( `pidController` 2 ) caveat. Signature unchanged. | The public promise now matches the behaviour. |
| Banners | `Last Modified` updated in both files. | |

Gate on `PRIMUS_X2_v1`: no new `src/` warnings. Now fixed and reported by the gate: `FC-Config-PID.cpp
-Waggressive-loop-optimizations` x1 and `-Wconversion` x3; the baseline is refreshed in task 4. Size 122776 / 1316 /
19728 B ( text / data / bss ) against 122752 / 1316 / 19728 at HEAD: +24 B flash, RAM unchanged.

Review fixes ( both nits, same task ): the `FC-Config.h` note on saving now names the paths that persist the change
( app save, stick trim, accelerometer or compass calibration, profile change, which stores the gains into the profile
being left ), and both files gained a HISTORY row. Gate re-run after the fixes: green, size unchanged.

## Task 3: host simulation ( 14 Nov 2026 )

| File | Change | Why |
|---|---|---|
| [tools/sim/test_set_default_pid.cpp](../../../../tools/sim/test_set_default_pid.cpp) ( new ) | 103 checks over four tests: the five API slots against the firmware defaults ( directly and through `get_PIDProfile ( )` ), the six hidden slots still holding the marker, the rest of the profile byte-identical by `memcpy` + `memcmp`, the values equal to `pidProfileDefaultGains ( )`, and a tune-then-restore round trip run twice. Markers `0xAA` and `0x55`. | The topic's evidence in place of a bench log. |
| [tools/sim/run.sh](../../../../tools/sim/run.sh) ( new ) | Builds the real `FC-Config-PID.cpp` ( overridable with `PID_API_SRC=` ), `pid_defaults.cpp` and `maths.cpp` with `-Itools/sim/stub -Isrc/main`. clang++ with `-fsanitize=address,undefined -fno-sanitize-recover=all`, falling back to g++. Fails the run on any sanitizer report; prints build warnings. | Re-runnable, and a sanitizer fault cannot be missed. |
| [tools/sim/stub/](../../../../tools/sim/stub/) ( new, 22 headers ) | Shadows only the hardware headers `FC-Config-PID.cpp` includes directly. Four have content: a reduced `profile_t`, `extern currentProfile`, `extern userLoopFrequency`, and the real `rc_alias_e` head. `pidProfile_t`, `pidIndex_e`, `pid_profile_e` and `axis.h` stay real. | Compiles firmware code on the PC without hardware headers. `tools/*` is in `.graphifyignore`, and the firmware Makefile includes only `src/main`. |

Results ( full detail and the control run in [TESTING.md](TESTING.md) ): fixed code `PASS: 103 checks, 0 failed`,
exit 0, under clang with both sanitizers and under g++. HEAD's `FC-Config-PID.cpp` through the same harness:
UBSan aborts with `index 5 out of bounds for type 'const int[5]'` at line 160, exit 1; under g++ without sanitizers,
`FAIL: 103 checks, 58 failed`, including `PID_ALT` 100 / 0 / 30 ( the T06 symptom ) and all six hidden slots
overwritten.

Review fixes ( 1 should-fix, 3 nits, all applied ): `-fno-sanitize-recover=all` plus a stderr check, so a UBSan-only
fault can no longer pass; `memcpy` instead of struct copies for the before / after snapshots; build warnings shown and
clang's first error reported when the g++ fallback is used; the sanitizer output quoted in `TESTING.md` corrected to
what the tool actually prints. Re-run after the fixes: PASS 103/103 ( clang and g++ ), HEAD still fails.

## Task 4: build gate and baseline ( 14 Nov 2026 )

| File | Change | Why |
|---|---|---|
| [tools/warnings-baseline.json](../../../../tools/warnings-baseline.json) | Removed `FC-Config-PID.cpp -Waggressive-loop-optimizations "iteration 5 invokes undefined behavior"` ( 1 ) and `FC-Config-PID.cpp -Wconversion "int to uint8_t"` ( 3 ); `total` 2662 to 2658. Written with the same `json.dump ( indent=1, sort_keys=True )` as `warnings.py`, CRLF kept: the diff is these three lines only. | Both warnings were removed deliberately by task 2 ( the out-of-bounds loop and its `int` tables ). The baseline was edited, not regenerated, so nothing else moved. The unrelated `-Wfloat-conversion` in `setUserLoopFrequency ( )` stays. |

Gate on `PRIMUS_X2_v1` ( clean build ): `ALL BUILDS PASSED`, no new `src/` warnings; `warnings.py check` against the
edited baseline exits 0 and no longer lists `FC-Config-PID.cpp`. Size 122776 / 1316 / 19728 B ( text / data / bss ),
Flash 121.1 KB / 256 KB ( 47 % ), RAM 20.5 KB / 40 KB ( 51 % ); against HEAD +24 B flash, RAM unchanged.

Not touched: the gate still lists 109 other "fixed" entries ( `utils.h`, `battery.cpp`, `XRanging`, ... ) left by
earlier commits that never refreshed the baseline. They belong to those topics, not this one.

## Task 7: topic review ( 14 Nov 2026 )

`pluto-reviewer` over the whole topic diff ( 7 modified, 31 untracked files ), with the gate and the host simulation
re-run inside the review: **0 blocking**, 1 should-fix, 5 nits. All fixed:

| Finding | Fix |
|---|---|
| SHOULD-FIX: `TASKS.md` named only `Core_PID_Controller.md`, so task 8 would have left `Mixer_Pipeline.md` and `User_Space_API.md` with stale `config.cpp` line references | Both the Summary row and task 8 now name all three sections of `PIPELINE_UPDATE.md`, and task 8 says why leaving the last two out is wrong. |
| NIT: `CHANGES.md` line references shifted by the HISTORY rows | `FC-Config-PID.cpp:143-166` to `146-169`, `:22` to `:25`, `FC-Config.h:73-91` to `75-95`. |
| NIT: `#L156` was never the loop ( it is 158 at HEAD, and 156 is now a comment line ) | The link points at the file, with "line 158 at HEAD" in the text, in `README.md` and `TASKS.md`. |
| NIT: `TESTING.md` reproduce block said "expect the ASan overflow" | Corrected to the UBSan abort, which is what `-fno-sanitize-recover=all` produces. |
| NIT: "host `g++` simulation" understated the main run | `README.md` and the decisions log now say clang++ with ASan and UBSan, g++ as the fallback. |
| NIT: `tools/sim/` was undocumented outside this topic | A paragraph in [dev-guide/BUILD_AND_VERIFY.md](../../dev-guide/BUILD_AND_VERIFY.md), under the GoogleTest section, explaining what it is, how to run it, `PID_API_SRC=`, and that it replaces neither the gate nor flight validation. |

After the fixes: `tools/sim/run.sh` `PASS: 103 checks, 0 failed`; gate on `PRIMUS_X2_v1` green, no new `src/`
warnings, Flash 121.1 KB / RAM 20.5 KB unchanged. The reviewer independently confirmed the 31 moved gain assignments
are identical to HEAD, every `PIPELINE_UPDATE.md` line reference is correct, each "Replace" target exists verbatim,
the baseline diff is exactly the four removed entries, and `MAIN_FLIGHT` reaches all three targets with the same
`pidProfile_t` layout ( GTUNE off everywhere ).
