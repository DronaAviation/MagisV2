# Wiki page conventions ( api-wiki-sync )

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [FINDINGS](FINDINGS.md) · [CHANGES](CHANGES.md)

The rules every page task ( 2-15 ) follows, so the 18 pages read as one wiki. The wiki is public: customers read it
and copy its examples into their `PlutoPilot.cpp`. Decisions behind these rules are in the [TASKS.md](TASKS.md)
decisions log.

## 1. Page shape

The existing wiki shape is kept, with one new section and a closing stamp:

````
# <Header>.h

<1-3 sentences: what the API is for.>
<One line on including it: "Included by PlutoPilot.h." or the explicit #include line to add.>

## Enumerations                      ( only if the header has any )

### `<type_e>`
<One line: what it selects.>

<!-- wiki-check: skip -->
```cpp
<the enum as declared in the header, with a unit comment per value>
```

## Functions

### `<signature exactly as in the header>`
<What it does; units, range and return value; when it takes effect; what it needs first.>

```cpp
<one small example, inside the hook it belongs in ( section 3 )>
```

> **Since API 1.8.4:** <one sentence: how it behaves now>.        ( only on items that changed )

## Behaviour and limits

- <Preconditions, timing, modes, pins or buses it takes, byte budgets, interaction with failsafe / landing / flips.>

## Known limitations                 ( only if there are any )

- <Condition, effect in user terms, workaround.>

## Usage Example

<Combined examples, one per concern ( for example: read and print / react to a value ), not one merged program.>

---

*Verified against API 1.8.10 / FW 3.15.12.*
````

- **Signatures** in `###` headings are copied from the header: same types, names and defaults. Pages for C++ classes
  ( `XRanging`, `X1Ranging`, `Localisation`, `Interval` ) use `Object.method ( ... )` headings.
- **Every return value has a unit** ( mV, cm, deg x10, ... ), a range, and what it reads when the data is not valid.
- **No internals on a public page**: no `File Architecture`, no `src/main/...` paths, no topic names, no `file:line`.
  Header names ( `FC-Config.h` ) are fine. The evidence for each claim goes in [CHANGES.md](CHANGES.md) instead.
- **Links between pages** use the page name with no `.md`: `[FC-Config.h](FC-Config)`. In-page anchors are GitHub's
  lower-case form.

## 2. Markers and fixed wording

| Case | Wording | Rule |
|---|---|---|
| Changed since the old wiki | `> **Since API 1.x.y:** <how it behaves now>.` | Only for changes from API 1.4.0 on. Say what it does now, not the old bug. |
| Function that does not work in this firmware ( [FINDINGS.md](FINDINGS.md) F1-F7, F9 ) | `> **Not available in FW 3.15:** <what happens if called, in user terms>. <What to use instead>.` | Keep the `###` entry so a reader searching for it finds the warning. **No example** under it. |
| Known limitation ( unfixed Planned topic ) | A bullet under `## Known limitations`: condition, effect, workaround. | No topic names. State the version: "In FW 3.15, ...". |
| Experimental API ( Localisation ) | `> **Experimental:** <what does not work yet>. <What still works>.` directly under the `#` title. | |
| Internal symbols a header exposes | `> **Internal:** used by the firmware; do not use from user code.` | One line listing them; no per-symbol entry. |

## 3. Examples

**Every `cpp` block is a complete, standalone unit.** It compiles with only `PlutoPilot.h` ( implicit ) plus any
`#include` it shows, and every statement sits inside one of the five hooks. A block that quotes the header ( an enum,
a signature ) is marked `<!-- wiki-check: skip -->`. An example meant to show a compiler warning is marked
`<!-- wiki-check: warn-ok -->`. `wiki_check.py` fails any other block that does not compile, link, or compile
warning-free under the firmware flags ( `-Wconversion -Wsign-conversion -Wshadow -Wdouble-promotion` ), because
users build their `PlutoPilot.cpp` with the same flags.

**Hook placement.** `onLoopStart ( )` and `onLoopFinish ( )` also run when the RC link stalls: `userCode ( )` runs user
code only while RC frames arrive ( `mw.cpp:1176` ), so a late frame runs `onLoopFinish ( )` ( `:1230` ) and the next
frame runs `onLoopStart ( )` again ( `:1194` ). A Wi-Fi stall of 1-2 s is normal, so both can run in flight.

| Hook | Runs | Put here | Never here |
|---|---|---|---|
| `plutoRxConfig ( )` | at start-up before `plutoInit`, **and again each time the settings are re-read from flash**: every settings save ( for example from the app ), a profile change, a settings reset | `Receiver_Mode`, `Receiver_Config_*`: they only set values, so repeating them is harmless | anything else, above all anything that starts hardware or allocates |
| `plutoInit ( )` | once, at power-up, Developer Mode or not | hardware set-up: pins, `Uart_Init`, `Motor_Init`, `RGB_Init`, `XRanging.init` / `initAlt`, `Oled_Init` | flight commands; prints ( the app is not connected yet, so they are lost ) |
| `onLoopStart ( )` | when Developer Mode turns on, **and again after every RC link stall** | one-time set-up that is harmless to repeat: enable / disable, a printed header, resetting user state ( flight modes go in `plutoLoop ( )`, task 17 ) | `Command_*` ( arm, take-off, land, disarm, flip ), `Scheduler_Add`, `DesiredAngle_setRelative`, `DesiredPosition_setRelative`: each would repeat in flight |
| `plutoLoop ( )` | once per user loop period ( 100 ms by default, `setUserLoopFrequency` ) while Developer Mode is on | reads, prints, setpoints, RC overrides and `FlightMode_Set` ( re-asserted every tick ), one-shot commands guarded by a `static` flag or an edge | blocking waits, prints over the byte budget |
| `onLoopFinish ( )` | when Developer Mode turns off, **and on every RC link stall** | cleanup that is harmless to repeat in flight: release overrides, stop an animation, clear a user state | `Command_Land`, `Command_DisArm`, `Command_Flip`, anything that changes how the drone flies |

**Accepted exception ( task 17 ):** restoring a setting your own code changed ( a stick rate, a failsafe switch ) may
sit in `onLoopFinish ( )` when the restore is harmless during an RC link stall, as on the FC-Config page.

**One-shot actions** go in `plutoLoop ( )` behind a function-scope `static` flag, re-armed by a condition the
example states ( for example the ARM switch going off ). Never re-send a command every tick.

**Flight-affecting examples end safely.** An example that tilts, turns, climbs or overrides a stick also shows how
it stops ( a level-out, a released override, a bounded climb ). No example leaves a held tilt running.

**Print budget.** With the app connected, everything one `plutoLoop ( )` tick prints stays under about 130 bytes
( Debugging page ). Examples on other pages print one or two short values.

**No file-level objects whose constructor must run code.** C++ start-up code never runs in this firmware
( `-nostartfiles` ), so such an object is left zeroed, not constructed. An object the compiler can fill in at build time
is fine: `XRanging_P` has a default member initializer, yet `XRanging` sits in `.data` with its `-1` values and has no
`.init_array` entry ( checked with `arm-none-eabi-nm` / `objdump`, task 6 ). Plain types are always safe.

**Style.** Two-space indent, the spaced-paren call style of the firmware and the `PlutoPilot.cpp` template
( `Monitor_Print ( "mV: ", mv );`, `void plutoLoop ( void ) {` ), fixed-width types from the header ( `int16_t`,
`uint16_t` ), explicit casts where a conversion narrows, `f` suffixes on float literals.

## 4. Text

- Plain ASCII punctuation: no em or en dashes ( use `-`, `:` or a new sentence ), no typographic quotes. Emojis in
  the existing navigation pages may stay.
- Units on every number. Signs stated where they matter ( pitch, yaw rate, roll ).
- State a behaviour as a fact the reader can rely on, with the number. Facts that are "probably" true are verified in
  the source first or left out.
- Short sentences; the reader may not be a native English speaker.

## 5. Checking a page ( every page task )

```
python -I docs/fw-development-reference/active-development/api-wiki-sync/wiki_check.py MagisV2.wiki/<Page>.md
```

- The page is finished when the run reports **0 failed**. Skipped blocks are header quotes only.
- `wiki_check.py` checks each block as a user's `PlutoPilot.cpp`: compiled at global scope with the firmware's exact
  `PlutoPilot.cpp` compile command ( read from `make -n` ), with empty definitions added for the hooks the block does
  not define, then linked in place of the firmware's own `PlutoPilot.o` into a throwaway ELF under `Build/wikicheck/`.
  A missing function and a file-level name that collides with a firmware global both fail, as they would for a user. It never writes `PlutoPilot.cpp`, `Build/<TARGET>/` or the `.hex`; its first
  step is the normal incremental `make` ( `--no-make` skips it ). Target: `selected_target` in `plutoide.ini`.
- After a change to `wiki_check.py`, run [wiki_check_selftest.md](wiki_check_selftest.md) through it: expect 7
  checked, 3 passed, 5 failed, 1 skipped.
- After a page set change, run [link_check.py](link_check.py) on `MagisV2.wiki/`: expect `0 dead`.
- Also run [table_check.py](table_check.py) on the page: GitHub splits a table cell on every unescaped `|`, even
  inside backticks, so a stray pipe shifts the rest of the row. Expect `mismatched table rows: 0`.
- Each page task adds a section to [CHANGES.md](CHANGES.md): the `wiki_check.py` result, and a table of every
  behaviour or number the page states that is new or changed, with the header or source line that proves it.
