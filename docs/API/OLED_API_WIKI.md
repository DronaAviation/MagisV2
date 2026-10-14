# MagisV2 OLED API Wiki

## Quick Start

```cpp
#include "PlutoPilot.h"

void plutoInit ( void ) { 
    Oled_Init(); 
}

void onLoopStart ( void ) {
    // Take control of the screen at the start of the loop
    Oled_Mode(User);
}

void plutoLoop ( void ) {
    // 1. Clear the internal buffer for the new frame
    Oled_Clear();
    
    // 2. Draw your content
    Oled_Text(10, 20, "Hello Pluto!");
    Oled_Circle(64, 40, 10);
    
    // 3. Send the buffer to the display
    Oled_Update();
}

void onLoopFinish ( void ) { 
    // Return control to the system for telemetry
    Oled_Mode(System); 
}
```

---

## Screen Technical Specs

*   **Resolution**: 128 x 64 pixels.
*   **Coordinate System**: Origin (0,0) is at the **top-left**.
*   **Constants**: `OLED_WIDTH` = 128, `OLED_HEIGHT` = 64.

---

## Simple API (Recommended)

The Simple API uses an internal 1024-byte framebuffer. You don't need to manage memory; just clear, draw, and update.

### Frame & Mode Control

| Function | Description |
|----------|-------------|
| `Oled_Init()` | Enables the OLED subsystem. Call once in `plutoInit()`. |
| `Oled_Mode(mode)` | Switches between `System` and `User` rendering modes. The screen starts from a blank frame: in `System` mode the telemetry is redrawn on the next loop, and in `User` mode it stays blank until your next `Oled_Update()`. The panel itself is not cleared directly. |
| `Oled_Is_Mode(mode)` | Checks if the display is in a specific mode. Returns `bool`. |
| `Oled_Clear()` | Wipes the internal buffer to start a fresh frame (RAM only, nothing is sent). |
| `Oled_Update()` | Queues the current buffer for the display ( `User` mode only; in `System` mode it does nothing ). Non-blocking: see *Timing and flight safety*. |

### Text & Numbers

| Function | Description |
|----------|-------------|
| `Oled_Text(x, y, "text")` | Draws text at `(x, y)`. Each char is 6x7 pixels. |
| `Oled_Text(x, y, "text", inverted)` | Same, but `inverted = true` draws the glyphs in **black** (cleared pixels) instead of white — useful for labelling text on top of a filled shape. No background is drawn; the caller controls what sits behind the text. `inverted` defaults to `false`. |
| `Oled_Number(x, y, val)` | Draws a signed 16-bit integer. |

### Drawing Primitives (Filled)

| Function | Description |
|----------|-------------|
| `Oled_Pixel(x, y)` | Draws a single pixel. |
| `Oled_Line(x0, y0, x1, y1)` | Draws a line between two points. |
| `Oled_Rect(x, y, w, h)` | Draws a filled rectangle. |
| `Oled_RoundRect(x, y, w, h, r)` | Draws a filled rounded rectangle. `r` is radius. |
| `Oled_Circle(cx, cy, r)` | Draws a filled circle. |

### Drawing Primitives (Outline Only)

| Function | Description |
|----------|-------------|
| `Oled_RectOutline(x, y, w, h)` | Draws a rectangle border. |
| `Oled_RoundRectOutline(x, y, w, h, r)` | Draws a rounded rectangle border. |
| `Oled_CircleOutline(cx, cy, r)` | Draws a circle border. |

### Expressive Helpers (Eyes)

These functions draw two eyes automatically centered on the screen.

| Function | Description |
|----------|-------------|
| `Oled_RobotEyes(px, py)` | Two boxy rounded eyes. `px/py` are pupil offsets (-5 to +5). |
| `Oled_RoundEyes(px, py)` | Two round cartoon eyes. `px/py` are pupil offsets (-5 to +5). |
| `Oled_DeadEyes()` | Two X-shaped eyes for error/crash states. |

### HUD & Telemetry

| Function | Description |
|----------|-------------|
| `Oled_Joysticks(t, y, r, p)` | Draws live dual RC stick indicators. Values 1000–2000. |
| `Oled_PitchLine(angle)` | Draws a horizontal pitch bar. Angle -90 to +90. |
| `Oled_RollLine(angle)` | Draws a vertical roll bar. Angle -90 to +90. |

### Utility

| Function | Description |
|----------|-------------|
| `Oled_Erase(x, y, w, h)` | Clears (draws black) a specific rectangular area. |

---

## Live Code Snippets

### Animated Eyes (Joystick Control)

```cpp
  // Map RC sticks (1000-2000) to pupil offset (-5 to +5)
  int8_t px = (int8_t)((RcData_Get(RC_YAW) - 1500) / 100);
  int8_t py = (int8_t)(-(RcData_Get(RC_PITCH) - 1500) / 100);
  
  Oled_RoundEyes(px, py);
```

### Attitude Indicators

```cpp
  int16_t pitch = (int16_t)(Estimate_Get(Angle, AG_PITCH) / 10);
  int16_t roll  = (int16_t)(Estimate_Get(Angle, AG_ROLL) / 10);
  
  Oled_PitchLine(pitch);
  Oled_RollLine(roll);
  
  Oled_Text(0, 56, "P:"); Oled_Number(14, 56, pitch);
  Oled_Text(50, 56, "R:"); Oled_Number(64, 56, roll);
```

### Inverted Text (Label on a Filled Shape)

```cpp
  // Draw a filled white pill, then punch the label out in black.
  Oled_RoundRect(10, 10, 60, 16, 4);
  Oled_Text(16, 14, "ARMED", true);   // inverted = true → black glyphs
```

---

## Advanced API

For power users who need full control over external buffers or direct grid printing:

### Direct Grid Text (System Mode)

`Oled_Print` writes text on the 21x8 grid of the system screen. It draws into the system framebuffer, which the non-blocking flush sends to the panel.

```cpp
  // col 0-20, row 1-6
  Oled_Print(0, 2, "SYSTEM ARMED");
```

### External Framebuffers

For colour-controlled text into your own buffer, use `Oled_DrawTextColor(screen, x, y, "text", on)` — `on = true` sets glyph pixels (white), `on = false` clears them (black). `Oled_DrawText(...)` is the white-only shorthand.

You can use the lower-level `Oled_display_Update` to show your own 1024-byte buffer (User mode only). It only queues the buffer: the flush reads it over the next loops, so the buffer must stay valid after the call. Make it `static` or global, never a local array.

```cpp
static uint8_t myBuffer[1024];

void plutoLoop() {
    memset(myBuffer, 0, 1024);
    // Use Oled_Draw* / Oled_Fill* functions from io/oled_display.h
    Oled_display_Update(myBuffer);
}
```

---

## Timing and flight safety

The OLED shares the I2C1 bus with the IMU, barometer and magnetometer, and I2C
is polled: every byte sent to the display holds up the flight loop. So nothing
draws straight to the panel.

*   Drawing (`Oled_Text`, `Oled_Rect`, ...), `Oled_Clear` and the system
    telemetry screen only change a framebuffer in RAM.
*   `Oled_Update` only queues the frame. Once per flight loop the firmware sends
    the next run of **changed** bytes, at most 16 bytes (about 0.6 ms) per loop.
*   A line of text that changes reaches the panel in a few loops. A full-screen
    change takes about 64 loops, about 0.25 s.
*   This is the same armed or disarmed, in `System` and `User` mode, so the
    OLED can stay live in flight.
*   If the display stops answering (3 failed writes in a row, or more than about
    1 write in 5 failing), the OLED is switched off, so a loose connector cannot
    keep holding up the loop.

The 2 s start-up splash page still writes the panel directly; it runs only at
power-on, before arming.

---

## File Architecture

| File | Description |
|------|-------------|
| `src/main/API/Oled.h` | **The Main Header**. Start here. |
| `src/main/API-Src/Oled.cpp` | Simple API implementation and buffer management. |
| `src/main/io/oled_display.h` | Advanced pixel-drawing primitives. |
| `src/main/io/oled_display.c` | Core display logic and system telemetry. |
| `src/main/drivers/display_ug2864hsweg01.c` | Hardware-level I2C driver (SSD1306). |
