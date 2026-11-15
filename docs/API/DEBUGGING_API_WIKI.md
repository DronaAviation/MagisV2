# MagisV2 Debugging API Wiki

Print text and numbers from user code to the PlutoMonitor window of the Pluto app with `Monitor_Print ( )` and
`Monitor_Println ( )`. Both are declared in `API/Debugging.h` and available from `PlutoPilot.h`. This page covers the
six print functions.

API version **1.8.9**. See "Changes in 1.8.9".

## Quick Start

```cpp
#include "PlutoPilot.h"

void plutoRxConfig ( void ) {
  Receiver_Mode ( Rx_ESP );
}

void plutoInit ( void ) {
}

// Runs once each time Developer Mode is switched on, and again after a link drop.
void onLoopStart ( void ) {
  Monitor_Println ( "Developer Mode on" );
}

// Runs again and again while Developer Mode is on ( every 100 ms by default ).
void plutoLoop ( void ) {
  float volts = ( float ) Bms_Get ( Voltage ) / 1000.0f;    // mV to V

  Monitor_Print ( "Alt cm:", ( int ) Estimate_Get ( Position, Z ) );
  Monitor_Println ( " Batt V:", ( double ) volts, 2 );
}

void onLoopFinish ( void ) {
}
```

PlutoMonitor then shows two lines ten times a second, for example `Alt cm:57` and `Batt V:3.95`.

---

## Functions

| Function | Prints |
|---|---|
| `Monitor_Print ( msg )` | A text. |
| `Monitor_Println ( msg )` | A text, then a line end. |
| `Monitor_Print ( tag, number )` | A tag and a whole number ( `int` ). |
| `Monitor_Println ( tag, number )` | A tag and a whole number, then a line end. |
| `Monitor_Print ( tag, number, precision )` | A tag and a decimal number ( `double` ) with `precision` digits after the point. |
| `Monitor_Println ( tag, number, precision )` | A tag and a decimal number, then a line end. |

The two text functions can be called from C and C++. The four number functions are C++ only, which is fine in
`PlutoPilot.cpp`.

### `Monitor_Print ( msg )` / `Monitor_Println ( msg )`

```cpp
void onLoopStart ( void ) {
  Monitor_Println ( "Test 3: hover at 1 m" );
}
```

The text must end with a `'\0'`, as every string literal does. A `NULL` pointer counts as an empty text.

### `Monitor_Print ( tag, number )` / `Monitor_Println ( tag, number )`

```cpp
void plutoLoop ( void ) {
  Monitor_Println ( "Roll ddeg:", ( int ) Estimate_Get ( Angle, AG_ROLL ) );
}
```

Every `int` prints in full, from `-2147483648` to `2147483647`. Cast other integer types to `int` ( `( int ) millis ( )` ):
an unsigned value above `2147483647` comes out negative.

### `Monitor_Print ( tag, number, precision )` / `Monitor_Println ( tag, number, precision )`

```cpp
void plutoLoop ( void ) {
  float degC = ( float ) Sensor_Get ( Barometer, Temperature ) / 100.0f;    // deg C x100 to deg C
  Monitor_Println ( "Baro degC:", ( double ) degC, 2 );                      // Baro degC:31.47
}
```

| `precision` | Prints | Example ( 3.14159 ) |
|---|---|---|
| 0 | No decimal point | `3` |
| 1 to 4 | That many digits after the point | `3.1`, `3.14`, `3.142`, `3.1416` |
| 5 or more | 4 digits, the same as 4 | `3.1416` |

- **Rounding.** The last digit is rounded half away from zero: `2.5` at 0 digits prints `3`, `-2.5` prints `-3`,
  `1.99999` at 2 digits prints `2.00`.
- **Small negative values keep their sign.** `-0.0001` at 2 digits prints `-0.00`; `-0.5` prints `-0.50`.
- **Range.** Values from `-4294967040` to `4294967040` print as numbers. Beyond that the number is replaced by the word
  `overflow`.
- **Not a number and infinity.** A NaN prints `not a number`; plus or minus infinity prints `infinite`.

A `float` value works as it is: it converts to `double` on the call, with no warning. Writing `( double ) x` is
optional and only makes the conversion visible. A `float` holds about 7 significant digits, so for very large values
the last of the 4 decimals is not meaningful.

---

## How the output looks in PlutoMonitor

Each call is sent to the app as one small message. In the current PlutoMonitor app:

- **Every call shows on its own line**, with the time it arrived. `Monitor_Print ( "a:", 1 )` followed by
  `Monitor_Println ( " b:", 2 )` shows two lines, `a:1` and `b:2`.
- The tag and the number are joined with no space: `Monitor_Print ( "Alt:", 57 )` shows `Alt:57`. Leading spaces in a
  tag are not shown.
- The line end sent by `Monitor_Println` does not show as an empty line.

So one value per call, with the name in the tag, reads best.

---

## How much you can print

The text and numbers travel over the same link to the app that carries the app's own messages, through a 256-byte
send buffer. If user code prints more than the link can take, the buffer overflows: the start of a line is lost, or
the app's own messages are corrupted and **the app disconnects**.

**Keep each `plutoLoop ( )` call under about 130 bytes** with the app connected ( about 180 bytes per loop
disconnected the app in a test; 115-135 bytes ran clean ). Count the bytes of each call:

| Call | Bytes |
|---|---|
| `Monitor_Print ( msg )` | 4 + length of `msg` |
| `Monitor_Print ( tag, number )` | 5 + length of `tag` + digits ( + 1 for a minus sign ) |
| `Monitor_Print ( tag, number, precision )` | 5 + length of `tag` + digits before the point + `precision` + 1 for the point if `precision` > 0 ( + 1 for a minus sign ) |
| Any `Monitor_Println` | the same as its `Monitor_Print`, + 5 for the line end |

Examples: `Monitor_Print ( "Alt cm:", 57 )` is 5 + 7 + 2 = **14 bytes**; `Monitor_Println ( " Batt V:", 3.95, 2 )` is
5 + 8 + 1 + 2 + 1 = 17, + 5 = **22 bytes**.

**Long texts are cut.** One call sends at most 255 characters of tag, separator and number. A longer tag is shortened
so the whole number still fits; the number is never cut. This is a limit of the message format, not a safe length:
a 255-character call is 259 bytes, more than the 256-byte buffer, so it overwrites its own start. Keep tags short.

---

## Examples

### Print once when Developer Mode starts ( `onLoopStart` )

```cpp
void onLoopStart ( void ) {
  Monitor_Println ( "== START" );
}
```

`onLoopStart ( )` also runs again after a short Wi-Fi drop, so this line also shows when the link stutters. That makes
it a useful marker in a log.

### A telemetry line every loop ( `plutoLoop` )

```cpp
void plutoLoop ( void ) {
  Monitor_Print ( "t:", ( int ) millis ( ) );                            // 5 + 2 + up to 7 = 14
  Monitor_Print ( " alt:", ( int ) Estimate_Get ( Position, Z ) );      // 5 + 5 + up to 4 = 14
  Monitor_Println ( " vz:", ( int ) Estimate_Get ( Velocity, Z ) );     // 5 + 4 + up to 4 + 5 = 18
}
```

About 46 bytes for the whole loop, well inside the budget.

### Print a value with decimals ( `plutoLoop` )

```cpp
void plutoLoop ( void ) {
  float seconds = ( float ) millis ( ) / 1000.0f;
  Monitor_Println ( "Uptime s:", ( double ) seconds, 3 );                // Uptime s:12.345
}
```

### Print once when Developer Mode ends ( `onLoopFinish` )

```cpp
void onLoopFinish ( void ) {
  Monitor_Println ( "== STOP" );
}
```

---

## Pitfalls

- **A decimal number with no `precision` prints as a whole number.** `Monitor_Print ( "v:", 3.7f )` has only two
  arguments, so it calls the `int` version and prints `v:3`. The build warns ( `-Wfloat-conversion` ) but the drone
  prints the wrong value. Always pass the precision for a decimal: `Monitor_Print ( "v:", ( double ) 3.7f, 1 )`.
- **Too many bytes per loop disconnects the app.** See "How much you can print". The limit counts every call in one
  `plutoLoop ( )` together.
- **More than 4 decimals are not printed.** A `precision` of 5 or more prints 4.
- **Very long tags.** A tag of hundreds of characters is cut, and even the longest allowed call overflows the send
  buffer. Keep tags to a few characters.
- **Printing in `onLoopStart ( )` or `onLoopFinish ( )` repeats at every Wi-Fi stutter**, because both run again
  after a link drop.

---

## Changes in 1.8.9

- **Fixed:** the decimal versions printed `0` for every digit after the first decimal ( API audit B9, test T01 ):
  `Monitor_Println ( "pi:", 3.14159, 3 )` printed `3.100`; it now prints `3.142`. One digit was already right.
- **Changed:** `precision` is limited to 4 ( it was 7 ). A request for 5 to 7 digits now prints 4.
- **Fixed:** a text or tag of 256 characters or more stopped the drone's main loop for good. Texts are now cut at 255.
- **Fixed:** a tag and number of more than 255 characters together sent a broken message. The tag is now shortened
  to fit.
- **Fixed:** `-2147483648` ( the smallest `int` ) printed as garbage characters; it now prints correctly.
- **Fixed:** the not-a-number and infinity messages had a wrong length byte, so the app could drop them.
- **Defined:** a `NULL` text or tag now counts as empty.
- Unchanged: every function name and argument, how messages are sent, and what normal numbers and texts look like.

## File Architecture

| File | Contents |
|------|----------|
| `src/main/API/Debugging.h` | The six `Monitor_Print` / `Monitor_Println` declarations |
| `src/main/API-Src/Debugging.cpp` | Their implementations: number formatting, the 255-character cut, the message framing |
| `src/main/io/serial_msp.cpp` | `serialize8Debug ( )`: puts each byte into the app link's send buffer |
