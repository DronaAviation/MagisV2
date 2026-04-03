# MagisV2 OLED API Wiki

## Quick Start

```cpp
#include "PlutoPilot.h"

void plutoInit ( void ) { Oled_Init(); }

void plutoLoop ( void ) {
  Oled_Begin();
  Oled_Text(10, 20, "Hello Pluto!");
  Oled_Circle(64, 40, 10);
  Oled_End();
}

void onLoopFinish ( void ) { Oled_SystemMode(); }
```

That's it. No buffers, no memset, no mode switching.

---

## Screen

128 x 64 pixels. `OLED_WIDTH` = 128, `OLED_HEIGHT` = 64. Origin (0,0) = top-left.

---

## Simple API (recommended)

All simple functions use an internal buffer. Just call `Oled_Begin()` → draw → `Oled_End()`.

### Frame Control

| Function | Description |
|----------|-------------|
| `Oled_Begin()` | Start frame. Clears buffer, auto enters USER mode. |
| `Oled_End()` | Send frame to display. Only changed pixels sent. |

### Text

| Function | Description |
|----------|-------------|
| `Oled_Text(x, y, "string")` | Draw text at pixel position. 6px/char, 7px tall. |
| `Oled_Number(x, y, 123)` | Draw integer. Handles negatives. |

### Shapes (filled)

| Function | Description |
|----------|-------------|
| `Oled_Pixel(x, y)` | Single pixel |
| `Oled_Line(x0, y0, x1, y1)` | Line between two points |
| `Oled_Rect(x, y, w, h)` | Filled rectangle |
| `Oled_RoundRect(x, y, w, h, r)` | Filled rounded rectangle. r = corner radius. |
| `Oled_Circle(cx, cy, r)` | Filled circle |

### Shapes (outline only)

| Function | Description |
|----------|-------------|
| `Oled_RectOutline(x, y, w, h)` | Rectangle border |
| `Oled_RoundRectOutline(x, y, w, h, r)` | Rounded rectangle border |
| `Oled_CircleOutline(cx, cy, r)` | Circle border |

### Eyes (one call = two eyes, auto-centered)

| Function | Description |
|----------|-------------|
| `Oled_RobotEyes(pupilX, pupilY)` | Two boxy rounded-rect eyes. pupilX/Y: -5 to +5. |
| `Oled_RoundEyes(pupilX, pupilY)` | Two round cartoon eyes. pupilX/Y: -5 to +5. |
| `Oled_DeadEyes()` | Two X-shaped dead/crash eyes. |

### HUD

| Function | Description |
|----------|-------------|
| `Oled_Joysticks(throttle, yaw, roll, pitch)` | Dual joystick display. Values 1000–2000. |
| `Oled_PitchLine(pitch)` | Horizontal line moves with pitch. Degrees -90 to +90. |
| `Oled_RollLine(roll)` | Vertical line moves with roll. Degrees -90 to +90. |

### Erase

| Function | Description |
|----------|-------------|
| `Oled_Erase(x, y, w, h)` | Black out a rectangular area |

### Setup

| Function | Description |
|----------|-------------|
| `Oled_Init()` | Enable OLED. Call once in `plutoInit()`. |
| `Oled_SystemMode()` | Return OLED to firmware telemetry. Call in `onLoopFinish()`. |
| `Oled_IsUserMode()` | Returns true if you own the screen. |
| `Oled_IsSystemMode()` | Returns true if firmware owns the screen. |

---

## Test Snippets

Each goes inside `plutoLoop()` between `Oled_Begin()` and `Oled_End()`.

### Text & Numbers

```cpp
  Oled_Text(4, 0, "DRONA AVIATION");
  Oled_Text(4, 14, "PlutoX2 Nano");
  Oled_Number(4, 28, 12345);
  Oled_Number(4, 42, -99);
```

### Pixel

```cpp
  Oled_Pixel(0, 0);
  Oled_Pixel(127, 0);
  Oled_Pixel(0, 63);
  Oled_Pixel(127, 63);
```

### Lines

```cpp
  Oled_Line(0, 0, 127, 63);       // diagonal
  Oled_Line(127, 0, 0, 63);       // other diagonal
```

### Rectangles

```cpp
  Oled_Rect(10, 10, 40, 25);               // filled
  Oled_RectOutline(60, 10, 40, 25);        // outline
  Oled_RoundRect(10, 40, 45, 20, 6);       // filled rounded
  Oled_RoundRectOutline(60, 40, 45, 20, 6); // outline rounded
```

### Circles

```cpp
  Oled_Circle(30, 32, 18);           // filled
  Oled_CircleOutline(90, 32, 18);    // outline
```

### Robot Eyes (boxy, animated with RC)

```cpp
  int px = (RcData_Get(RC_YAW) - 1500) / 100;
  int py = -(RcData_Get(RC_PITCH) - 1500) / 100;
  Oled_RobotEyes(px, py);
```

### Round Eyes (animated with RC)

```cpp
  int px = (RcData_Get(RC_YAW) - 1500) / 100;
  int py = -(RcData_Get(RC_PITCH) - 1500) / 100;
  Oled_RoundEyes(px, py);
```

### Dead Eyes

```cpp
  Oled_DeadEyes();
```

### Tall Eyes Without Pupils (roll-reactive)

```cpp
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
```

### RC Joystick HUD (live)

```cpp
  Oled_Joysticks(
    RcData_Get(RC_THROTTLE),
    RcData_Get(RC_YAW),
    RcData_Get(RC_ROLL),
    RcData_Get(RC_PITCH)
  );
```

### Attitude Indicators (live)

```cpp
  int pitch = Estimate_Get(Angle, AG_PITCH) / 10;
  int roll  = Estimate_Get(Angle, AG_ROLL) / 10;
  Oled_PitchLine(pitch);
  Oled_RollLine(roll);
  Oled_Line(60, 32, 68, 32);    // crosshair
  Oled_Line(64, 28, 64, 36);
  Oled_Text(0, 56, "P:");
  Oled_Number(14, 56, pitch);
  Oled_Text(50, 56, "R:");
  Oled_Number(64, 56, roll);
```

---

## Full Examples (copy entire file)

### Tall Rounded-Rect Eyes (no pupils, roll-reactive, blink)

```cpp
#include "PlutoPilot.h"

static int  blinkTimer = 0;
static bool blinking   = false;

void plutoRxConfig ( void ) { Receiver_Mode(Rx_ESP); }
void plutoInit ( void )     { Oled_Init(); }
void onLoopStart ( void )   { blinkTimer = 0; blinking = false; }
void onLoopFinish ( void )  { Oled_SystemMode(); }

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
  if (leftH  > 50) leftH  = 50;
  if (rightH > 50) rightH = 50;

  blinkTimer++;
  if (blinkTimer >= 30) { blinking = true; blinkTimer = 0; }
  if (blinking) { leftH = rightH = 4; if (blinkTimer >= 2) blinking = false; }

  if (leftH > 5)  Oled_RoundRect(19, 32 - leftH/2, 38, leftH, 10);
  else             Oled_Line(19, 32, 57, 32);

  if (rightH > 5) Oled_RoundRect(69, 32 - rightH/2, 38, rightH, 10);
  else             Oled_Line(69, 32, 107, 32);

  Oled_End();
}
```

### Emo Robot Face (eyes + eyebrows + mouth)

```cpp
#include "PlutoPilot.h"

static int  blinkTimer = 0;
static bool blinking   = false;

void plutoRxConfig ( void ) { Receiver_Mode(Rx_ESP); }
void plutoInit ( void )     { Oled_Init(); }
void onLoopStart ( void )   { blinkTimer = 0; blinking = false; }
void onLoopFinish ( void )  { Oled_SystemMode(); }

void plutoLoop ( void ) {
  Oled_Begin();

  int px = (RcData_Get(RC_YAW) - 1500) / 80;
  int py = -(RcData_Get(RC_PITCH) - 1500) / 80;
  int throt = RcData_Get(RC_THROTTLE);
  int roll  = Estimate_Get(Angle, AG_ROLL) / 10;
  int fx = roll / 6;
  if (fx >  5) fx =  5;
  if (fx < -5) fx = -5;
  int leftH  = 26 + fx;
  int rightH = 26 - fx;
  if (leftH  < 8) leftH  = 8;
  if (rightH < 8) rightH = 8;

  blinkTimer++;
  if (blinkTimer >= 30) { blinking = true; blinkTimer = 0; }
  if (blinking) { leftH = rightH = 4; if (blinkTimer >= 2) blinking = false; }

  // Eyes (using advanced API with internal buffer trick — or use Oled_RobotEyes for simple)
  Oled_RobotEyes(px, py);

  // Eyebrows
  Oled_Line(20, 14 - fx, 56, 14 - fx);
  Oled_Line(72, 14 + fx, 108, 14 + fx);

  // Mouth
  if (throt < 1300)
    Oled_Line(56, 52, 72, 52);
  else if (throt < 1600)
    Oled_RoundRectOutline(54, 50, 20, 5, 2);
  else
    Oled_RoundRect(52, 48, 24, 10, 3);

  Oled_End();
}
```

### Full 10-Page Test Suite

```cpp
#include "PlutoPilot.h"

static int page = 0, frame = 0;

void plutoRxConfig ( void ) { Receiver_Mode(Rx_ESP); }
void plutoInit ( void )     { Oled_Init(); }
void onLoopStart ( void )   { page = 0; frame = 0; }
void onLoopFinish ( void )  { Oled_SystemMode(); }

void plutoLoop ( void ) {
  Oled_Begin();
  switch (page) {
  case 0:
    Oled_Text(4, 0, "1/10 Text+Number");
    Oled_Text(4, 14, "Hello Pluto!");
    Oled_Number(4, 28, 12345);
    Oled_Number(4, 42, -99);
    break;
  case 1:
    Oled_Text(4, 0, "2/10 Lines");
    Oled_Pixel(0, 63); Oled_Pixel(127, 63);
    Oled_Line(0, 12, 127, 63);
    Oled_Line(127, 12, 0, 63);
    break;
  case 2:
    Oled_Text(4, 0, "3/10 Rects");
    Oled_RectOutline(4, 14, 35, 22);
    Oled_Rect(44, 14, 35, 22);
    Oled_RoundRectOutline(4, 40, 45, 20, 6);
    Oled_RoundRect(54, 40, 45, 20, 6);
    break;
  case 3:
    Oled_Text(4, 0, "4/10 Circles");
    Oled_CircleOutline(30, 40, 18);
    Oled_Circle(80, 40, 16);
    break;
  case 4: {
    Oled_Text(10, 0, "5/10 Robot Eyes");
    int px = (frame % 10) - 5;
    Oled_RobotEyes(px, 0);
    break; }
  case 5: {
    Oled_Text(10, 0, "6/10 Round Eyes");
    int px = (frame % 10) - 5;
    Oled_RoundEyes(px, 0);
    break; }
  case 6:
    Oled_Text(10, 0, "7/10 Dead Eyes");
    Oled_DeadEyes();
    break;
  case 7: {
    Oled_Text(4, 0, "8/10 Tall Eyes");
    int roll = Estimate_Get(Angle, AG_ROLL) / 10;
    int fx = roll / 5;
    if (fx >  8) fx =  8;
    if (fx < -8) fx = -8;
    int lh = 36 + fx, rh = 36 - fx;
    if (lh < 6) lh = 6; if (rh < 6) rh = 6;
    Oled_RoundRect(19, 32 - lh/2, 38, lh, 10);
    Oled_RoundRect(69, 32 - rh/2, 38, rh, 10);
    break; }
  case 8:
    Oled_Text(14, 0, "9/10 RC HUD");
    Oled_Joysticks(RcData_Get(RC_THROTTLE), RcData_Get(RC_YAW),
                   RcData_Get(RC_ROLL), RcData_Get(RC_PITCH));
    break;
  case 9: {
    Oled_Text(0, 0, "10/10 Attitude");
    int p = Estimate_Get(Angle, AG_PITCH)/10, r = Estimate_Get(Angle, AG_ROLL)/10;
    Oled_PitchLine(p); Oled_RollLine(r);
    Oled_Line(60,32,68,32); Oled_Line(64,28,64,36);
    Oled_Text(0,56,"P:"); Oled_Number(14,56,p);
    Oled_Text(50,56,"R:"); Oled_Number(64,56,r);
    break; }
  }
  Oled_End();

  frame++;
  if (frame >= 30) { frame = 0; page++; if (page >= 10) page = 0; }
}
```

---

## Advanced API

For power users who need full control (custom framebuffers, erase pixels, multiple buffers):

```cpp
static uint8_t screen[1024];

void plutoLoop() {
  memset(screen, 0, 1024);
  Oled_DrawCircle(screen, 64, 32, 20, true);   // outline
  Oled_FillRect(screen, 10, 10, 30, 20, true); // filled
  Oled_DrawPixel(screen, 50, 50, false);        // erase pixel
  Oled_Update(screen);
}
```

See `Oled.h` for all `Oled_Draw*` / `Oled_Fill*` functions with full `@param` documentation.

---

## File Map

| File | Purpose |
|------|---------|
| `src/main/API/Oled.h` | Public API — all declarations with VS Code hints |
| `src/main/API-Src/Oled.cpp` | Implementation |
| `src/main/io/oled_display.c` | Firmware telemetry (SYSTEM mode) |
| `src/main/drivers/display_ug2864hsweg01.c` | SSD1306 hardware driver |
| `PlutoPilot.cpp` | User code entry point |
| `docs/OLED_API_WIKI.md` | This file |
