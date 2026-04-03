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
#include "config/runtime_config.h"

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
 * @brief Clear OLED display.
 *
 * Uses batch I2C page writes (8 transactions instead of 2048).
 * Skipped if drone is armed to avoid any I2C contention during flight.
 * Valid in both SYSTEM and USER modes.
 */
void Oled_Clear(void)
{
    if (!OledEnable) return;
    if (ARMING_FLAG(ARMED)) return;

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
void Oled_Print(uint8_t col, uint8_t row, const char *text)
{
    if (!OledEnable) return;
    if (oledMode != OLED_MODE_SYSTEM) return;
    if (!text) return;

    if (col > 20 || row < 1 || row > 6) return;

    i2c_OLED_set_xy(col, row);
    i2c_OLED_send_string(text);
}

/**
 * @brief Draw text into framebuffer at pixel coordinates.
 *
 * Renders using the built-in 5x7 font. Each character is 6px wide
 * (5px glyph + 1px gap). Works in USER mode — text lives in the
 * framebuffer alongside shapes, eyes, etc.
 *
 * @param screen  Framebuffer pointer
 * @param x       Pixel X position (0–127)
 * @param y       Pixel Y position (0–63)
 * @param text    Null-terminated string
 */
void Oled_DrawText(uint8_t *screen, int x, int y, const char *text)
{
    if (!screen || !text) return;

    while (*text) {
        uint8_t ch = (uint8_t)*text;
        if (ch < 32) ch = 32;     // unprintable → space
        uint8_t idx = ch - 32;

        // Draw 5 columns of the glyph
        for (int col = 0; col < 5; col++) {
            uint8_t bits = multiWiiFont[idx][col];
            for (int row = 0; row < 7; row++) {
                if (bits & (1 << row)) {
                    Oled_DrawPixel(screen, x + col, y + row, true);
                }
            }
        }
        // 6th column = gap (already clear from memset)

        x += 6;                    // advance cursor
        if (x > 127 - 5) break;   // stop at screen edge
        text++;
    }
}

/**
 * @brief Draw an integer number into framebuffer.
 */
void Oled_DrawNumber(uint8_t *screen, int x, int y, int number)
{
    char buf[12];    // enough for -2147483648
    int  i = 0;
    bool neg = false;

    if (number < 0) {
        neg = true;
        number = -number;
    }

    // Build digits in reverse
    do {
        buf[i++] = '0' + (number % 10);
        number /= 10;
    } while (number > 0);

    if (neg) buf[i++] = '-';

    // Reverse into output
    char out[12];
    for (int j = 0; j < i; j++) {
        out[j] = buf[i - 1 - j];
    }
    out[i] = '\0';

    Oled_DrawText(screen, x, y, out);
}

/**
 * @brief Check if OLED is in USER mode.
 */
bool Oled_IsUserMode(void)
{
    return oledMode == OLED_MODE_USER;
}

/**
 * @brief Check if OLED is in SYSTEM mode.
 */
bool Oled_IsSystemMode(void)
{
    return oledMode == OLED_MODE_SYSTEM;
}

/**
 * @brief Return OLED ownership to system firmware.
 *
 * After calling:
 *  - System telemetry and UI rendering resumes
 *  - User graphics are cleared
 *
 * Skipped if armed — mode switch involves a display clear.
 */
void Oled_SystemMode(void)
{
    if (ARMING_FLAG(ARMED)) {
        oledMode = OLED_MODE_SYSTEM;
        return;
    }
    oledMode = OLED_MODE_SYSTEM;
    i2c_OLED_clear_display_quick();
}

/**
 * @brief Grant exclusive OLED ownership to user code.
 *
 * After calling:
 *  - System rendering is disabled
 *  - Only framebuffer-based drawing is allowed
 *
 * Resets shadow buffer so first Oled_Update() diffs correctly.
 * Skipped clear if armed.
 */
void Oled_UserMode(void)
{
    oledMode = OLED_MODE_USER;
    memset(oledShadowBuffer, 0, sizeof(oledShadowBuffer));
    if (!ARMING_FLAG(ARMED)) {
        i2c_OLED_clear_display_quick();
    }
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
static int Oled_Clamp(int v, int min, int max)
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
static int Oled_Map(int v, int inMin, int inMax, int outMin, int outMax)
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

/**
 * @brief Helper: draw quarter-circle arcs for rounded rect corners.
 */
static void drawCornerArcs(uint8_t *buf, int x, int y, int w, int h, int r, bool on)
{
    int cx, cy;
    int px = r, py = 0, err = 0;

    while (px >= py) {
        // Top-right corner
        cx = x + w - 1 - r; cy = y + r;
        Oled_DrawPixel(buf, cx + px, cy - py, on);
        Oled_DrawPixel(buf, cx + py, cy - px, on);
        // Top-left corner
        cx = x + r; cy = y + r;
        Oled_DrawPixel(buf, cx - px, cy - py, on);
        Oled_DrawPixel(buf, cx - py, cy - px, on);
        // Bottom-right corner
        cx = x + w - 1 - r; cy = y + h - 1 - r;
        Oled_DrawPixel(buf, cx + px, cy + py, on);
        Oled_DrawPixel(buf, cx + py, cy + px, on);
        // Bottom-left corner
        cx = x + r; cy = y + h - 1 - r;
        Oled_DrawPixel(buf, cx - px, cy + py, on);
        Oled_DrawPixel(buf, cx - py, cy + px, on);

        py++;
        err += 1 + 2 * py;
        if (2 * (err - px) + 1 > 0) {
            px--;
            err += 1 - 2 * px;
        }
    }
}

void Oled_DrawRoundedRect(uint8_t *buf, int x, int y, int w, int h, int r, bool on)
{
    if (r <= 0) {
        Oled_DrawRect(buf, x, y, w, h, on);
        return;
    }
    if (r > w / 2) r = w / 2;
    if (r > h / 2) r = h / 2;

    // Straight edges (shortened by radius)
    Oled_DrawHLine(buf, x + r, y, w - 2 * r, on);
    Oled_DrawHLine(buf, x + r, y + h - 1, w - 2 * r, on);
    Oled_DrawVLine(buf, x, y + r, h - 2 * r, on);
    Oled_DrawVLine(buf, x + w - 1, y + r, h - 2 * r, on);

    // Corner arcs
    drawCornerArcs(buf, x, y, w, h, r, on);
}

void Oled_FillRoundedRect(uint8_t *buf, int x, int y, int w, int h, int r, bool on)
{
    if (r <= 0 || r > w / 2 || r > h / 2) {
        Oled_FillRect(buf, x, y, w, h, on);
        return;
    }
    // Fill center rectangle
    Oled_FillRect(buf, x + r, y, w - 2 * r, h, on);
    // Fill left and right side strips
    Oled_FillRect(buf, x, y + r, r, h - 2 * r, on);
    Oled_FillRect(buf, x + w - r, y + r, r, h - 2 * r, on);
    // Fill corner circles
    int px = r, py = 0, err = 0;
    while (px >= py) {
        Oled_DrawHLine(buf, x + r - px, y + r - py, px, on);               // top-left
        Oled_DrawHLine(buf, x + w - r, y + r - py, px, on);                // top-right
        Oled_DrawHLine(buf, x + r - px, y + h - 1 - r + py, px, on);      // bottom-left
        Oled_DrawHLine(buf, x + w - r, y + h - 1 - r + py, px, on);       // bottom-right
        Oled_DrawHLine(buf, x + r - py, y + r - px, py, on);
        Oled_DrawHLine(buf, x + w - r, y + r - px, py, on);
        Oled_DrawHLine(buf, x + r - py, y + h - 1 - r + px, py, on);
        Oled_DrawHLine(buf, x + w - r, y + h - 1 - r + px, py, on);

        py++;
        err += 1 + 2 * py;
        if (2 * (err - px) + 1 > 0) {
            px--;
            err += 1 - 2 * px;
        }
    }
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
    // Scanline fill using midpoint circle — O(r) scanlines instead of O(r^2) pixels
    int x = r, y = 0, err = 0;

    // Draw center horizontal line first
    Oled_DrawHLine(buf, cx - r, cy, 2 * r + 1, on);

    while (x >= y) {
        y++;
        err += 1 + 2 * y;
        if (2 * (err - x) + 1 > 0) {
            x--;
            err += 1 - 2 * x;
        }
        // Draw horizontal spans for each octant pair
        Oled_DrawHLine(buf, cx - x, cy + y, 2 * x + 1, on);
        Oled_DrawHLine(buf, cx - x, cy - y, 2 * x + 1, on);
        Oled_DrawHLine(buf, cx - y, cy + x, 2 * y + 1, on);
        Oled_DrawHLine(buf, cx - y, cy - x, 2 * y + 1, on);
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
static void Oled_DrawEyeWithPupil(
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
static void Oled_DrawEyeOutlineWithPupil(
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



/* ============================================================================
 * BOXY EYES (Rounded Rectangle)
 * ============================================================================
 */

/**
 * @brief Draw a filled boxy eye with a rounded-rect pupil hole.
 *
 * Outer eye = filled rounded rectangle (white on OLED).
 * Pupil     = smaller filled rounded rectangle erased inside (black).
 * Pupil tracks with offset, clamped to stay inside the eye.
 */
static void Oled_DrawBoxyEyeFilled(
    uint8_t *buf,
    int cx, int cy,
    int w, int h,
    int r,
    int pupilOffsetX,
    int pupilOffsetY
)
{
    // Draw outer eye (filled rounded rect)
    int ex = cx - w / 2;
    int ey = cy - h / 2;
    Oled_FillRoundedRect(buf, ex, ey, w, h, r, true);

    // Pupil dimensions (1/3 of eye size)
    int pw = w / 3;
    int ph = h / 3;
    int pr = (r > 1) ? r / 2 : 1;

    // Clamp pupil offset so it stays inside the eye
    int maxOffX = (w - pw) / 2 - 1;
    int maxOffY = (h - ph) / 2 - 1;
    pupilOffsetX = clampPupil(pupilOffsetX, maxOffX);
    pupilOffsetY = clampPupil(pupilOffsetY, maxOffY);

    // Erase pupil (draw black rounded rect inside)
    int px = cx + pupilOffsetX - pw / 2;
    int py = cy + pupilOffsetY - ph / 2;
    Oled_FillRoundedRect(buf, px, py, pw, ph, pr, false);
}

/**
 * @brief Draw a boxy outline eye with a filled rounded-rect pupil.
 *
 * Outer eye = rounded rectangle border only.
 * Pupil     = small filled rounded rectangle inside.
 */
static void Oled_DrawBoxyEyeOutline(
    uint8_t *buf,
    int cx, int cy,
    int w, int h,
    int r,
    int pupilOffsetX,
    int pupilOffsetY
)
{
    // Draw outer eye border
    int ex = cx - w / 2;
    int ey = cy - h / 2;
    Oled_DrawRoundedRect(buf, ex, ey, w, h, r, true);

    // Pupil dimensions
    int pw = w / 3;
    int ph = h / 3;
    int pr = (r > 1) ? r / 2 : 1;

    // Clamp
    int maxOffX = (w - pw) / 2 - 1;
    int maxOffY = (h - ph) / 2 - 1;
    pupilOffsetX = clampPupil(pupilOffsetX, maxOffX);
    pupilOffsetY = clampPupil(pupilOffsetY, maxOffY);

    // Draw filled pupil
    int px = cx + pupilOffsetX - pw / 2;
    int py = cy + pupilOffsetY - ph / 2;
    Oled_FillRoundedRect(buf, px, py, pw, ph, pr, true);
}

/**
 * @brief Draw a boxy eye — convenience wrapper.
 *
 * @param filled  true = solid eye with hollow pupil, false = outline with solid pupil
 */
void Oled_DrawBoxyEye(
    uint8_t *buf,
    int cx, int cy,
    int w, int h,
    int r,
    int pupilOffsetX,
    int pupilOffsetY,
    bool filled
)
{
    if (filled) {
        Oled_DrawBoxyEyeFilled(buf, cx, cy, w, h, r, pupilOffsetX, pupilOffsetY);
    } else {
        Oled_DrawBoxyEyeOutline(buf, cx, cy, w, h, r, pupilOffsetX, pupilOffsetY);
    }
}

/**
 * @brief Draw a boxy X-eye (error/dead state) inside a rounded rect.
 */
void Oled_DrawBoxyXEye(uint8_t *buf, int cx, int cy, int w, int h, int r)
{
    int ex = cx - w / 2;
    int ey = cy - h / 2;
    Oled_DrawRoundedRect(buf, ex, ey, w, h, r, true);

    int margin = 4;
    Oled_DrawLine(buf, ex + margin, ey + margin, ex + w - margin, ey + h - margin, true);
    Oled_DrawLine(buf, ex + w - margin, ey + margin, ex + margin, ey + h - margin, true);
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


/* ============================================================================
 * SIMPLE API — internal framebuffer, no user management needed
 * ============================================================================
 */

static uint8_t oledSimpleBuffer[1024];

void Oled_Begin(void)
{
    if (!Oled_IsUserMode()) Oled_UserMode();
    memset(oledSimpleBuffer, 0, sizeof(oledSimpleBuffer));
}

void Oled_End(void)
{
    Oled_Update(oledSimpleBuffer);
}

/* --- Simple text --- */
void Oled_Text(int x, int y, const char *text)     { Oled_DrawText(oledSimpleBuffer, x, y, text); }
void Oled_Number(int x, int y, int number)          { Oled_DrawNumber(oledSimpleBuffer, x, y, number); }

/* --- Simple shapes (filled) --- */
void Oled_Pixel(int x, int y)                       { Oled_DrawPixel(oledSimpleBuffer, x, y, true); }
void Oled_Line(int x0, int y0, int x1, int y1)      { Oled_DrawLine(oledSimpleBuffer, x0, y0, x1, y1, true); }
void Oled_Rect(int x, int y, int w, int h)           { Oled_FillRect(oledSimpleBuffer, x, y, w, h, true); }
void Oled_RoundRect(int x, int y, int w, int h, int r) { Oled_FillRoundedRect(oledSimpleBuffer, x, y, w, h, r, true); }
void Oled_Circle(int cx, int cy, int r)              { Oled_FillCircle(oledSimpleBuffer, cx, cy, r, true); }

/* --- Simple outlines --- */
void Oled_RectOutline(int x, int y, int w, int h)           { Oled_DrawRect(oledSimpleBuffer, x, y, w, h, true); }
void Oled_RoundRectOutline(int x, int y, int w, int h, int r) { Oled_DrawRoundedRect(oledSimpleBuffer, x, y, w, h, r, true); }
void Oled_CircleOutline(int cx, int cy, int r)               { Oled_DrawCircle(oledSimpleBuffer, cx, cy, r, true); }

/* --- Simple HUD --- */
void Oled_Joysticks(int throttle, int yaw, int roll, int pitch) { Oled_DrawRCJoysticks(oledSimpleBuffer, throttle, yaw, roll, pitch); }
void Oled_PitchLine(int pitch)    { Oled_DrawPitchIndicator(oledSimpleBuffer, pitch); }
void Oled_RollLine(int roll)      { Oled_DrawRollIndicator(oledSimpleBuffer, roll); }

/* --- Erase --- */
void Oled_Erase(int x, int y, int w, int h)         { Oled_FillRect(oledSimpleBuffer, x, y, w, h, false); }

/* --- Eye presets --- */
void Oled_RobotEyes(int pupilX, int pupilY)
{
    Oled_DrawBoxyEye(oledSimpleBuffer, 38, 32, 40, 30, 6, pupilX, pupilY, true);
    Oled_DrawBoxyEye(oledSimpleBuffer, 90, 32, 40, 30, 6, pupilX, pupilY, true);
}

void Oled_RoundEyes(int pupilX, int pupilY)
{
    Oled_DrawEye(oledSimpleBuffer, 38, 32, 18, pupilX, pupilY, true);
    Oled_DrawEye(oledSimpleBuffer, 90, 32, 18, pupilX, pupilY, true);
}

void Oled_DeadEyes(void)
{
    Oled_DrawBoxyXEye(oledSimpleBuffer, 38, 32, 36, 28, 6);
    Oled_DrawBoxyXEye(oledSimpleBuffer, 90, 32, 36, 28, 6);
}


#ifdef __cplusplus
}
#endif