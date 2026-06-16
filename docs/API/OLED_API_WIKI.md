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
| `Oled_Mode(mode)` | Switches between `System` and `User` rendering modes. |
| `Oled_Is_Mode(mode)` | Checks if the display is in a specific mode. Returns `bool`. |
| `Oled_Clear()` | Wipes the internal buffer to start a fresh frame. |
| `Oled_Update()` | Flushes the current buffer to the display hardware. |

### Text & Numbers

| Function | Description |
|----------|-------------|
| `Oled_Text(x, y, "text")` | Draws text at `(x, y)`. Each char is 6x7 pixels. |
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

---

## Advanced API

For power users who need full control over external buffers or direct grid printing:

### Direct Grid Text (System Mode)

`Oled_Print` is used for high-performance text on the 21x8 grid. It bypasses the pixel buffer.

```cpp
  // col 0-20, row 1-6
  Oled_Print(0, 2, "SYSTEM ARMED");
```

### External Framebuffers

You can use the lower-level `Oled_display_Update` to flush your own 1024-byte buffer.

```cpp
static uint8_t myBuffer[1024];

void plutoLoop() {
    memset(myBuffer, 0, 1024);
    // Use Oled_Draw* / Oled_Fill* functions from io/oled_display.h
    Oled_display_Update(myBuffer);
}
```

---

## File Architecture

| File | Description |
|------|-------------|
| `src/main/API/Oled.h` | **The Main Header**. Start here. |
| `src/main/API-Src/Oled.cpp` | Simple API implementation and buffer management. |
| `src/main/io/oled_display.h` | Advanced pixel-drawing primitives. |
| `src/main/io/oled_display.c` | Core display logic and system telemetry. |
| `src/main/drivers/display_ug2864hsweg01.c` | Hardware-level I2C driver (SSD1306). |
