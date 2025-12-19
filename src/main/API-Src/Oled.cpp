/*******************************************************************************
 *  Copyright (c) 2025 Drona Aviation
 *  All rights reserved.
 *  ---------------------------------------------------------------------------
 *  Author: Omkar Dandekar
 *  Project: MagisV2
 *  File: \src\main\API\Oled.cpp
 *  Created Date: Thu, 18th Dec 2025
 *  ---------------------------------------------------------------------------
 *  @brief
 *  OLED subsystem implementation for MagisV2.
 *
 *  This module implements the public OLED API defined in Oled.h.
 *
 *  Features:
 *   - SYSTEM mode: immediate text rendering for telemetry/debug UI
 *   - USER mode  : framebuffer-based graphics (eyes, HUDs, animations)
 *
 *  Design principles:
 *   - Non-blocking rendering
 *   - Deterministic execution
 *   - Diff-based OLED updates to minimize I2C bandwidth
 *   - Clear ownership separation between system firmware and user code
 *
 *  Ownership arbitration is controlled via the global `oledMode`.
 *
 *  ---------------------------------------------------------------------------
 *  HISTORY:
 *  Date        By              Comments
 *  ----------  --------------  ----------------------------------------------
 *  2025-12-18  Omkar Dandekar   Introduced unified OLED rendering layer with
 *                              SYSTEM / USER ownership, framebuffer drawing
 *                              primitives, expressive eye helpers, and
 *                              diff-based OLED update mechanism.
 ******************************************************************************/

#include "API/Oled.h"
#include "API/API-Utils.h"
#include "drivers/display_ug2864hsweg01.h"

#include <stdlib.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * INTERNAL CONSTANTS
 * ============================================================================
 */

#define OLED_W        128
#define OLED_H         64
#define OLED_PAGES   (OLED_H / 8)
#define JOY_RADIUS     18
#define LEFT_JOY_X     32
#define RIGHT_JOY_X    96
#define JOY_Y          32
#define STICK_RADIUS    3
/* ============================================================================
 * INTERNAL STATE
 * ============================================================================
 */

/**
 * @brief Current OLED ownership mode.
 *
 * Shared across system firmware and user graphics layer.
 */
oled_mode_e oledMode = OLED_MODE_SYSTEM;

/**
 * @brief Shadow framebuffer used for diff-based OLED updates.
 *
 * Stores the last-sent OLED state so only changed bytes are transmitted.
 */
static uint8_t oledShadowBuffer[1024];

/* ============================================================================
 * CORE OLED CONTROL (SYSTEM + USER)
 * ============================================================================
 */

/**
 * @brief Initialize OLED subsystem.
 *
 * Enables logical OLED usage. Hardware initialization is assumed
 * to be performed elsewhere during platform startup.
 */
void Oled_Init(void)
{
    OledEnable = true;
}

/**
 * @brief Clear OLED display immediately.
 *
 * Uses a fast hardware clear command.
 * Valid in both SYSTEM and USER modes.
 */
void Oled_Clear(void)
{
    if (!OledEnable) return;

    i2c_OLED_clear_display_quick();
}

/**
 * @brief Print text using OLED text-grid.
 *
 * This API is intended for SYSTEM mode only.
 * Calls are ignored when OLED is owned by USER code.
 *
 * @param col     Text column (0–20)
 * @param row     Text row (1–6)
 * @param string  Null-terminated string to print
 */
void Oled_Print(uint8_t col, uint8_t row, const char *string)
{
    if (!OledEnable) return;
    if (oledMode != OLED_MODE_SYSTEM) return;

    if (col > 20 || row < 1 || row > 6) return;

    i2c_OLED_set_xy(col, row);
    i2c_OLED_send_string(string);
}

/**
 * @brief Return OLED ownership to system firmware.
 *
 * After calling:
 *  - System telemetry and UI rendering resumes
 *  - User graphics are cleared
 */
void Oled_SetMode_System(void)
{
    oledMode = OLED_MODE_SYSTEM;
    i2c_OLED_clear_display_quick();
}

/**
 * @brief Grant exclusive OLED ownership to user code.
 *
 * After calling:
 *  - System rendering is disabled
 *  - Only framebuffer-based drawing is allowed
 */
void Oled_SetMode_User(void)
{
    oledMode = OLED_MODE_USER;
    i2c_OLED_clear_display_quick();
}


/* ============================================================================
 * MATH HELPERS
 * ============================================================================
 */

/**
 * @brief Clamp an integer value between a minimum and maximum.
 *
 * If v < min → returns min  
 * If v > max → returns max  
 * Otherwise  → returns v
 *
 * @param v    Input value
 * @param min  Lower bound
 * @param max  Upper bound
 * @return     Clamped value
 */
int Oled_Clamp(int v, int min, int max)
{
    if (v < min) return min;
    if (v > max) return max;
    return v;
}

/**
 * @brief Linearly map a value from one range to another.
 *
 * Equivalent to Arduino's map(), but:
 *  - Uses clamping to prevent overflow
 *  - Safe for OLED coordinate math
 *
 * Formula:
 *   out = (v - inMin) * (outMax - outMin)
 *         -------------------------------- + outMin
 *                 (inMax - inMin)
 *
 * @param v        Input value
 * @param inMin    Input range minimum
 * @param inMax    Input range maximum
 * @param outMin   Output range minimum
 * @param outMax   Output range maximum
 * @return         Mapped value
 */
int Oled_Map(int v, int inMin, int inMax, int outMin, int outMax)
{
    /* Prevent division by zero */
    if (inMax == inMin)
        return outMin;

    /* Clamp input to expected range */
    v = Oled_Clamp(v, inMin, inMax);

    return (v - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

/* ============================================================================
 * FRAMEBUFFER DRAWING PRIMITIVES
 * ============================================================================
 */

/**
 * @brief Set or clear a single pixel in the framebuffer.
 *
 * Performs bounds checking before writing.
 *
 * @param buf Framebuffer pointer
 * @param x   X coordinate (0–127)
 * @param y   Y coordinate (0–63)
 * @param on  true = set pixel, false = clear pixel
 */
void Oled_DrawPixel(uint8_t *buf, int x, int y, bool on)
{
    if (!buf) return;
    if (x < 0 || x >= OLED_W || y < 0 || y >= OLED_H) return;

    int index = x + (y >> 3) * OLED_W;
    uint8_t mask = (1U << (y & 7));

    if (on) buf[index] |= mask;
    else    buf[index] &= ~mask;
}

/* ---------------- Line primitives ---------------- */

void Oled_DrawHLine(uint8_t *buf, int x, int y, int length, bool on)
{
    for (int i = 0; i < length; i++) {
        Oled_DrawPixel(buf, x + i, y, on);
    }
}

void Oled_DrawVLine(uint8_t *buf, int x, int y, int length, bool on)
{
    for (int i = 0; i < length; i++) {
        Oled_DrawPixel(buf, x, y + i, on);
    }
}

/**
 * @brief Draw a line using Bresenham’s algorithm.
 */
void Oled_DrawLine(uint8_t *buf, int x0, int y0, int x1, int y1, bool on)
{
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (1) {
        Oled_DrawPixel(buf, x0, y0, on);
        if (x0 == x1 && y0 == y1) break;

        int e2 = err << 1;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}


/* ---------------- RC control primitives ---------------- */


/**
 * @brief Map RC value to joystick offset.
 */
static int mapStick(int v, int radius)
{
    v = Oled_Clamp(v, 1000, 2000);
    return (v - 1500) * radius / 500;
}

/**
 * @brief Draw a single joystick (internal).
 */
static void drawJoystick(
    uint8_t *buf,
    int cx,
    int cy,
    int xValue,
    int yValue,
    bool invertY
)
{
    /* Outer boundary */
    Oled_DrawCircle(buf, cx, cy, JOY_RADIUS, true);

    /* Center cross */
    Oled_DrawHLine(buf, cx - 4, cy, 8, true);
    Oled_DrawVLine(buf, cx, cy - 4, 8, true);

    /* Stick offset */
    int dx = mapStick(xValue, JOY_RADIUS - 4);
    int dy = mapStick(yValue, JOY_RADIUS - 4);

    if (invertY) dy = -dy;

    /* Stick position */
    Oled_FillCircle(
        buf,
        cx + dx,
        cy + dy,
        STICK_RADIUS,
        true
    );
}

/* ============================================================================
 * PUBLIC API
 * ============================================================================
 */

void Oled_DrawRCJoysticks(
    uint8_t *buf,
    int throttle,
    int yaw,
    int roll,
    int pitch
)
{
    /* Left joystick: Yaw (X) + Throttle (Y) */
    drawJoystick(
        buf,
        LEFT_JOY_X,
        JOY_Y,
        yaw,
        throttle,
        true        /* throttle inverted */
    );

    /* Right joystick: Roll (X) + Pitch (Y) */
    drawJoystick(
        buf,
        RIGHT_JOY_X,
        JOY_Y,
        roll,
        pitch,
        true        /* pitch inverted for natural feel */
    );
}


void Oled_DrawPitchIndicator(uint8_t *buf, int pitch)
{
    int centerY = OLED_H / 2;
    int pitchOffset = (pitch * (OLED_H / 2)) / 90; // Assuming pitch is in range -90 to +90
    Oled_DrawLine(buf, 0, centerY + pitchOffset, OLED_W - 1, centerY + pitchOffset, true);
}   
void Oled_DrawRollIndicator(uint8_t *buf, int roll)
{
    int centerX = OLED_W / 2;
    int rollOffset = (roll * (OLED_W / 2)) / 90; // Assuming roll is in range -90 to +90
    Oled_DrawLine(buf, centerX + rollOffset, 0, centerX + rollOffset, OLED_H - 1, true);
}


/* ---------------- Rectangle primitives ---------------- */

void Oled_DrawRect(uint8_t *buf, int x, int y, int w, int h, bool on)
{
    Oled_DrawHLine(buf, x, y, w, on);
    Oled_DrawHLine(buf, x, y + h - 1, w, on);
    Oled_DrawVLine(buf, x, y, h, on);
    Oled_DrawVLine(buf, x + w - 1, y, h, on);
}

void Oled_FillRect(uint8_t *buf, int x, int y, int w, int h, bool on)
{
    for (int i = 0; i < h; i++) {
        Oled_DrawHLine(buf, x, y + i, w, on);
    }
}

/* ---------------- Rounded rectangles ---------------- */

void Oled_DrawRoundedRect(uint8_t *buf, int x, int y, int w, int h, int r, bool on)
{
    if (r <= 0) {
        Oled_DrawRect(buf, x, y, w, h, on);
        return;
    }

    Oled_DrawHLine(buf, x + r, y, w - 2 * r, on);
    Oled_DrawHLine(buf, x + r, y + h - 1, w - 2 * r, on);
    Oled_DrawVLine(buf, x, y + r, h - 2 * r, on);
    Oled_DrawVLine(buf, x + w - 1, y + r, h - 2 * r, on);
}

void Oled_FillRoundedRect(uint8_t *buf, int x, int y, int w, int h, int r, bool on)
{
    (void)r;  /* Radius currently unused (fallback fill) */
    Oled_FillRect(buf, x, y, w, h, on);
}

/* ---------------- Circle primitives ---------------- */

void Oled_DrawCircle(uint8_t *buf, int cx, int cy, int r, bool on)
{
    int x = r, y = 0, err = 0;

    while (x >= y) {
        Oled_DrawPixel(buf, cx + x, cy + y, on);
        Oled_DrawPixel(buf, cx + y, cy + x, on);
        Oled_DrawPixel(buf, cx - y, cy + x, on);
        Oled_DrawPixel(buf, cx - x, cy + y, on);
        Oled_DrawPixel(buf, cx - x, cy - y, on);
        Oled_DrawPixel(buf, cx - y, cy - x, on);
        Oled_DrawPixel(buf, cx + y, cy - x, on);
        Oled_DrawPixel(buf, cx + x, cy - y, on);

        y++;
        err += 1 + 2 * y;
        if (2 * (err - x) + 1 > 0) {
            x--;
            err += 1 - 2 * x;
        }
    }
}

void Oled_FillCircle(uint8_t *buf, int cx, int cy, int r, bool on)
{
    for (int y = -r; y <= r; y++) {
        for (int x = -r; x <= r; x++) {
            if (x * x + y * y <= r * r) {
                Oled_DrawPixel(buf, cx + x, cy + y, on);
            }
        }
    }
}

/* ============================================================================
 * DIFF-BASED OLED UPDATE (USER MODE)
 * ============================================================================
 */

/**
 * @brief Push framebuffer changes to OLED hardware.
 *
 * Only modified bytes are transmitted over I2C.
 * This function is ignored unless oledMode == OLED_MODE_USER.
 *
 * @param buffer Pointer to framebuffer
 */
void Oled_Update(uint8_t *buffer)
{
    if (!OledEnable) return;
    if (oledMode != OLED_MODE_USER) return;
    if (!buffer) return;

    i2c_OLED_send_changed_bytes(
        buffer,
        oledShadowBuffer,
        sizeof(oledShadowBuffer)
    );
}

/* ============================================================================
 * EYES / EXPRESSIVE HELPERS
 * ============================================================================
 */

/**
 * @brief Clamp pupil offset so it stays inside the eye.
 */
static int clampPupil(int v, int max)
{
    if (v >  max) return  max;
    if (v < -max) return -max;
    return v;
}

/**
 * @brief Draw a filled eye with a moving pupil.
 */
void Oled_DrawEyeWithPupil(
    uint8_t *buf,
    int cx,
    int cy,
    int radius,
    int pupilOffsetX,
    int pupilOffsetY
)
{
    Oled_FillCircle(buf, cx, cy, radius, true);

    int pupilRadius = radius / 3;
    int maxOffset   = radius - pupilRadius - 1;

    pupilOffsetX = clampPupil(pupilOffsetX, maxOffset);
    pupilOffsetY = clampPupil(pupilOffsetY, maxOffset);

    Oled_FillCircle(
        buf,
        cx + pupilOffsetX,
        cy + pupilOffsetY,
        pupilRadius,
        false
    );
}

/**
 * @brief Draw an outline eye with a filled pupil.
 */
void Oled_DrawEyeOutlineWithPupil(
    uint8_t *buf,
    int cx,
    int cy,
    int radius,
    int pupilOffsetX,
    int pupilOffsetY
)
{
    Oled_DrawCircle(buf, cx, cy, radius, true);

    int pupilRadius = radius / 3;
    int maxOffset   = radius - pupilRadius - 1;

    pupilOffsetX = clampPupil(pupilOffsetX, maxOffset);
    pupilOffsetY = clampPupil(pupilOffsetY, maxOffset);

    Oled_FillCircle(
        buf,
        cx + pupilOffsetX,
        cy + pupilOffsetY,
        pupilRadius,
        true
    );
}

/**
 * @brief Draw an X-shaped eye (error / dead state).
 */
void Oled_DrawXEye(uint8_t *buf, int cx, int cy, int size)
{
    int half = size / 2;

    Oled_DrawLine(
        buf,
        cx - half, cy - half,
        cx + half, cy + half,
        true
    );

    Oled_DrawLine(
        buf,
        cx - half, cy + half,
        cx + half, cy - half,
        true
    );
}
/* ============================================================================
 * Draw eye
 * ============================================================================
 */
void Oled_DrawEye(
    uint8_t *buf,
    int cx,
    int cy,
    int radius,
    int pupilOffsetX,
    int pupilOffsetY,
    bool filled
)
{
    if (filled) {
        Oled_DrawEyeWithPupil(
            buf, cx, cy, radius, pupilOffsetX, pupilOffsetY
        );
    } else {
        Oled_DrawEyeOutlineWithPupil(
            buf, cx, cy, radius, pupilOffsetX, pupilOffsetY
        );
    }
}



void Oled_DrawArrow(
    uint8_t *buf,
    int cx,
    int cy,
    int size,
    oled_arrow_dir_t dir,
    bool on
)
{
    if (!buf || size <= 0) return;

    int half = size / 2;
    int head = size / 3;   // arrow head size

    switch (dir)
    {
        case OLED_ARROW_UP:
            // Shaft
            Oled_DrawLine(buf, cx, cy + half, cx, cy - half, on);
            // Head
            Oled_DrawLine(buf, cx, cy - half, cx - head, cy - half + head, on);
            Oled_DrawLine(buf, cx, cy - half, cx + head, cy - half + head, on);
            break;

        case OLED_ARROW_DOWN:
            // Shaft
            Oled_DrawLine(buf, cx, cy - half, cx, cy + half, on);
            // Head
            Oled_DrawLine(buf, cx, cy + half, cx - head, cy + half - head, on);
            Oled_DrawLine(buf, cx, cy + half, cx + head, cy + half - head, on);
            break;

        case OLED_ARROW_LEFT:
            // Shaft
            Oled_DrawLine(buf, cx + half, cy, cx - half, cy, on);
            // Head
            Oled_DrawLine(buf, cx - half, cy, cx - half + head, cy - head, on);
            Oled_DrawLine(buf, cx - half, cy, cx - half + head, cy + head, on);
            break;

        case OLED_ARROW_RIGHT:
            // Shaft
            Oled_DrawLine(buf, cx - half, cy, cx + half, cy, on);
            // Head
            Oled_DrawLine(buf, cx + half, cy, cx + half - head, cy - head, on);
            Oled_DrawLine(buf, cx + half, cy, cx + half - head, cy + head, on);
            break;

        default:
            break;
    }
}


#ifdef __cplusplus
}
#endif