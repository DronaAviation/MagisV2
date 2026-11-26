# Monitor_Print double precision - Pipeline update

[README](README.md) · [TASKS](TASKS.md) · [SCOUT](SCOUT.md) · [INVESTIGATION](INVESTIGATION.md) · [CHANGES](CHANGES.md) · [TESTING](TESTING.md) · [PIPELINE_UPDATE](PIPELINE_UPDATE.md)

Staged text. Apply at commit ( `pluto-commit` ); do not edit `fw-architecture-pipeline/` or CLAUDE.md before.

## 1. `fw-architecture-pipeline/subsystems/User_Space_API.md`

### Section "Primary Functions"

**Insert** after the bullet that starts `- **OLED is deferred**:` and ends `` Wiki: `docs/API/OLED_API_WIKI.md`. ``
( currently lines 97-100, before the `- **Laser ranging**:` bullet ):

```markdown
- **Debug output**: `Monitor_Print / Monitor_Println ( msg )`, `( tag, int )` and `( tag, double, precision )`
  ( `API/Debugging.h` ) send a `$D` frame to the app through `serialize8Debug ( )`, which writes into the
  same 256-byte TX ring as the MSP replies with no room check. A `Print` is one frame; a `Println` adds a second,
  5-byte `"\n"` frame. `precision` is 0 to 4 ( more prints 4 ), rounded half
  away from zero; beyond +/-4294967040 the number prints as `overflow`, NaN as `not a number`, infinity as `infinite`.
  A frame carries at most 255 payload bytes: the tag is cut so the number fits ( API 1.8.9; before, the double
  overload printed 0 after the first decimal, a 256-character string hung the main loop, and `INT_MIN` printed
  garbage ). Keep one `plutoLoop ( )` under ~130 bytes with the app connected. Wiki `docs/API/DEBUGGING_API_WIKI.md`
  ( API 1.8.9 ).
```

Every edge stated here is a source line:

- `Monitor_*` -> `debugPrint ( )`: `API-Src/Debugging.cpp:320-363`.
- `debugPrint ( )` -> `serialize8Debug ( )`: `Debugging.cpp`.
- `serialize8Debug ( )` -> `serialWrite ( mspSerialPort, ... )`: `io/serial_msp.cpp:449-451`.
- `uartWrite ( )` has no room check: `drivers/serial_uart.c:301-313`.

## 2. `CLAUDE.md`, section "Flight invariants"

**Replace** the third line of the `Monitor_Print` bullet ( currently line 134 ):

```markdown
  The double overload prints 0 for every digit after the first decimal.
```

**with:**

```markdown
  A `Print` is one frame of at most 255 payload bytes ( the tag is cut ), a `Println` adds a 5-byte `"\n"` frame;
  `precision` 0..4 ( `docs/API/DEBUGGING_API_WIKI.md` ).
```

Reason: the sentence it replaces is no longer true after API 1.8.9, and every future session reads it.
