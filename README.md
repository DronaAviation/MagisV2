# MagisV2 OLED API

128x64 monochrome OLED display API for the PlutoX2 nano drone.

## Quick Start

```cpp
void plutoInit ( void ) { Oled_Init(); }

void plutoLoop ( void ) {
  Oled_Begin();
  Oled_Text(10, 20, "Hello Pluto!");
  Oled_RobotEyes(0, 0);
  Oled_End();
}

void onLoopFinish ( void ) { Oled_SystemMode(); }
```

No buffers. No memset. No mode switching. Just draw.

---

## How It Works

**SYSTEM mode** (default) — firmware shows battery, attitude, RC data, flight status automatically. Just call `Oled_Init()` in `plutoInit()`.

**USER mode** — you draw anything. Call `Oled_Begin()` at the start of your frame, draw with `Oled_*` functions, call `Oled_End()` to send to display. Call `Oled_SystemMode()` in `onLoopFinish()` to return to telemetry.

---

## Simple API

### Frame

| Function | Description |
|----------|-------------|
| `Oled_Begin()` | Start frame (clears buffer, auto enters USER mode) |
| `Oled_End()` | Send frame to display (only changed pixels) |

### Text

| Function | Description |
|----------|-------------|
| `Oled_Text(x, y, "string")` | Draw text. 6px/char, 7px tall. |
| `Oled_Number(x, y, 123)` | Draw integer (handles negatives) |

### Shapes

| Function | Description |
|----------|-------------|
| `Oled_Pixel(x, y)` | Single pixel |
| `Oled_Line(x0, y0, x1, y1)` | Line between two points |
| `Oled_Rect(x, y, w, h)` | Filled rectangle |
| `Oled_RoundRect(x, y, w, h, r)` | Filled rounded rectangle |
| `Oled_Circle(cx, cy, r)` | Filled circle |
| `Oled_RectOutline(x, y, w, h)` | Rectangle border |
| `Oled_RoundRectOutline(x, y, w, h, r)` | Rounded rectangle border |
| `Oled_CircleOutline(cx, cy, r)` | Circle border |
| `Oled_Erase(x, y, w, h)` | Black out a region |

### Eyes

| Function | Description |
|----------|-------------|
| `Oled_RobotEyes(pupilX, pupilY)` | Two boxy robot eyes. pupilX/Y: -5 to +5 |
| `Oled_RoundEyes(pupilX, pupilY)` | Two round cartoon eyes |
| `Oled_DeadEyes()` | Two X-shaped crash eyes |

### HUD

| Function | Description |
|----------|-------------|
| `Oled_Joysticks(throttle, yaw, roll, pitch)` | Dual RC joystick display (1000–2000) |
| `Oled_PitchLine(pitch)` | Pitch indicator (-90 to +90 degrees) |
| `Oled_RollLine(roll)` | Roll indicator (-90 to +90 degrees) |

### Setup

| Function | Description |
|----------|-------------|
| `Oled_Init()` | Enable OLED. Call once in `plutoInit()`. |
| `Oled_SystemMode()` | Return to firmware telemetry. Call in `onLoopFinish()`. |
| `Oled_IsUserMode()` | Returns true if you own the screen |
| `Oled_IsSystemMode()` | Returns true if firmware owns the screen |

### Constants

| Constant | Value |
|----------|-------|
| `OLED_WIDTH` | 128 |
| `OLED_HEIGHT` | 64 |

---

## Examples

### Robot Eyes That Track RC Sticks

```cpp
void plutoLoop ( void ) {
  Oled_Begin();
  int px = (RcData_Get(RC_YAW) - 1500) / 100;
  int py = -(RcData_Get(RC_PITCH) - 1500) / 100;
  Oled_RobotEyes(px, py);
  Oled_End();
}
```

### Tall Eyes That React to Roll

```cpp
void plutoLoop ( void ) {
  Oled_Begin();
  int roll = Estimate_Get(Angle, AG_ROLL) / 10;
  int fx = roll / 5;
  if (fx >  8) fx =  8;
  if (fx < -8) fx = -8;
  int leftH  = 36 + fx;
  int rightH = 36 - fx;
  if (leftH  < 6) leftH  = 6;
  if (rightH < 6) rightH = 6;
  Oled_RoundRect(19, 32 - leftH/2, 38, leftH, 10);
  Oled_RoundRect(69, 32 - rightH/2, 38, rightH, 10);
  Oled_End();
}
```

### Show Sensor Data

```cpp
void plutoLoop ( void ) {
  Oled_Begin();
  Oled_Text(0, 0, "Pitch:");
  Oled_Number(40, 0, Estimate_Get(Angle, AG_PITCH) / 10);
  Oled_Text(0, 10, "Roll:");
  Oled_Number(36, 10, Estimate_Get(Angle, AG_ROLL) / 10);
  Oled_Text(0, 20, "Yaw:");
  Oled_Number(30, 20, RcData_Get(RC_YAW));
  Oled_End();
}
```

---

## Advanced API

For users who need full control (custom buffers, erase individual pixels):

```cpp
static uint8_t screen[1024];

void plutoLoop ( void ) {
  memset(screen, 0, 1024);
  Oled_DrawCircle(screen, 64, 32, 20, true);
  Oled_DrawPixel(screen, 50, 50, false);   // erase a pixel
  Oled_Update(screen);
}
```

All `Oled_Draw*` / `Oled_Fill*` functions accept a framebuffer pointer, x/y coordinates, and an `on` flag (true=white, false=black). See [Oled.h](src/main/API/Oled.h) for full documentation with VS Code hover hints.

---

## Documentation

- [OLED API Wiki](docs/OLED_API_WIKI.md) — test snippets for every function, full examples
- [Oled.h](src/main/API/Oled.h) — complete API with doxygen `@param` hints

---

## Hardware

| Property | Value |
|----------|-------|
| Display | SSD1306 128x64 monochrome OLED |
| Interface | I2C @ 0x3C |
| Pins | PB8 (SCL), PB9 (SDA) |
| MCU | STM32F303xC (72 MHz, 40 KB RAM) |
| RAM usage | ~2 KB (shadow buffer + simple buffer) |
| Update rate | 5 Hz system, up to loop rate in USER mode |

---

## File Structure

```
src/main/API/Oled.h                      — Public API (Simple + Advanced)
src/main/API-Src/Oled.cpp                — Implementation
src/main/io/oled_display.c               — System telemetry display
src/main/drivers/display_ug2864hsweg01.c — SSD1306 hardware driver
PlutoPilot.cpp                           — User code entry point
docs/OLED_API_WIKI.md                    — Wiki with all test snippets
```

---

## Credits

- **Omkar Dandekar** — OLED API design, framebuffer engine, simple layer, boxy eyes
- **Ashish Jaiswal (MechAsh)** — System telemetry display, SSD1306 driver integration
- **Dharna Nar** — Original OledEyes reference design
- **Drona Aviation** — MagisV2 firmware platform
