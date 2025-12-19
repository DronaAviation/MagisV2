# MagisV2 OLED API

A unified, deterministic OLED rendering API for **MagisV2**, designed to support both **system-level telemetry UI** and **user-defined graphics / animations** without conflicts.

This API sits between the low-level OLED driver and user firmware, enforcing clear ownership rules and efficient rendering.

---

## Key Design Goals

* Clear separation between **SYSTEM** and **USER** rendering
* Non-blocking, deterministic execution
* Framebuffer-based graphics for animations
* Diff-based OLED updates to minimize I2C bandwidth
* Safe-by-default APIs (out-of-range and invalid-mode calls are ignored)

---

## OLED Ownership Model

The OLED can be owned by **only one layer at a time**.

### SYSTEM Mode (Default)

* Used by firmware for telemetry, status, debug text
* Text-grid based rendering
* User drawing APIs are ignored

### USER Mode

* Exclusive access for user code
* Framebuffer-based drawing only
* System text rendering is disabled

Ownership is controlled via:

```c
Oled_SetMode_System();
Oled_SetMode_User();
```

---

## Initialization Flow

```c
void plutoInit(void)
{
    Oled_Init();            // Enable OLED subsystem
    Oled_SetMode_System();  // Default (optional, SYSTEM is default)
}
```

> Hardware initialization of the OLED is handled by the platform and **not** by this API.

---

## SYSTEM Mode APIs (Text Rendering)

### Oled_Print

```c
Oled_Print(uint8_t col, uint8_t row, const char *text);
```

**Parameters**

* `col`: 0–20 (text column)
* `row`: 1–6  (text row)
* `text`: null-terminated string

---

### Oled_Clear

```c
Oled_Clear();
```

Clears the OLED immediately using a fast hardware command.

Valid in both SYSTEM and USER modes.

---

## USER Mode APIs (Graphics Rendering)

### Framebuffer

All USER-mode drawing operates on a framebuffer:

```c
uint8_t buffer[1024];  // 128 x 64 OLED
```

The buffer must persist (global or static).

---

### Oled_Update

```c
Oled_Update(uint8_t *buffer);
```

* Pushes only changed bytes to the OLED
* Uses an internal shadow buffer for diff-based updates
* Ignored unless OLED is in USER mode

---

## Drawing Primitives

### Pixel

```c
Oled_DrawPixel(buffer, x, y, true);
```

* `x`: 0–127
* `y`: 0–63

---

### Lines

```c
Oled_DrawHLine(buffer, x, y, length, true);
Oled_DrawVLine(buffer, x, y, length, true);
Oled_DrawLine(buffer, x0, y0, x1, y1, true);
```

---

### Rectangles

```c
Oled_DrawRect(buffer, x, y, w, h, true);
Oled_FillRect(buffer, x, y, w, h, true);
```

---

### Rounded Rectangles

```c
Oled_DrawRoundedRect(buffer, x, y, w, h, r, true);
Oled_FillRoundedRect(buffer, x, y, w, h, r, true);
```

> Note: Filled rounded rectangles currently ignore radius.

---

### Circles

```c
Oled_DrawCircle(buffer, cx, cy, r, true);
Oled_FillCircle(buffer, cx, cy, r, true);
```

---

## Flight & RC Visuals

### RC Joysticks

```c
Oled_DrawRCJoysticks(buffer, throttle, yaw, roll, pitch);
```

* RC input range: `1000–2000`
* Natural inverted-Y behavior applied internally

---

### Pitch Indicator

```c
Oled_DrawPitchIndicator(buffer, pitch);
```

* Pitch range: `-90° to +90°`

---

### Roll Indicator

```c
Oled_DrawRollIndicator(buffer, roll);
```

* Roll range: `-90° to +90°`

---

## Expressive / Eye APIs

### Filled Eye with Pupil

```c
Oled_DrawEyeWithPupil(buffer, cx, cy, radius, dx, dy);
```

---

### Outline Eye with Pupil

```c
Oled_DrawEyeOutlineWithPupil(buffer, cx, cy, radius, dx, dy);
```

---

### X Eye (Error / Dead State)

```c
Oled_DrawXEye(buffer, cx, cy, size);
```

---

## Utility Helpers

### Clamp

```c
int Oled_Clamp(int v, int min, int max);
```

### Map

```c
int Oled_Map(int v, int inMin, int inMax, int outMin, int outMax);
```

Safe alternatives to raw math for OLED coordinate mapping.

---

## Minimal USER Mode Example

```c
static uint8_t fb[1024];

void plutoInit(void)
{
    Oled_Init();
    Oled_SetMode_User();
}

void plutoLoop(void)
{
    memset(fb, 0, sizeof(fb));
    Oled_DrawCircle(fb, 64, 32, 20, true);
    Oled_Update(fb);
}
```

---

---

## Intended Usage

* SYSTEM mode → telemetry, debug, status UI
* USER mode → animations, eyes, HUDs, visual demos
