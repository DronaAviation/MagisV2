/*******************************************************************************
 *  Copyright (c) 2025 Drona Aviation
 *  All rights reserved.
 *  ---------------------------------------------------------------------------
 *  Author: Omkar Dandekar
 *  Project: MagisV2
 *  File: \src\main\API\Oled.h
 *  Created Date: Thu, 18th Dec 2025
 *  ---------------------------------------------------------------------------
 *  @brief
 *  Public OLED API for MagisV2.
 *
 *  This header defines a safe, non-blocking OLED rendering interface intended
 *  for both system firmware and user applications.
 *
 *  Design goals:
 *   - Prevent conflicts between system UI and user graphics
 *   - Enable expressive framebuffer-based drawing (eyes, HUDs, animations)
 *   - Keep rendering deterministic and non-blocking
 *
 *  Operating modes:
 *   - SYSTEM mode : firmware-controlled telemetry / status UI
 *   - USER mode   : user-controlled framebuffer graphics
 *
 *  All user graphics are drawn into a 128x64 framebuffer and explicitly
 *  flushed to hardware using a diff-based update mechanism.
 *
 *  ---------------------------------------------------------------------------
 *  HISTORY:
 *  Date        By              Comments
 *  ----------  --------------  ----------------------------------------------
 *  2025-12-18  Omkar Dandekar   Introduced unified OLED public API with
 *                              explicit ownership control (SYSTEM / USER),
 *                              framebuffer drawing primitives, expressive
 *                              eye helpers, and diff-based update support.
 ******************************************************************************/

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * OLED OWNERSHIP MODE
 * ============================================================================
 */

/**
 * @brief OLED ownership state.
 *
 * This enum defines who is allowed to render on the OLED at any given time.
 *
 * - OLED_MODE_SYSTEM:
 *   Firmware controls the OLED (telemetry, status, system UI).
 *   Text-grid APIs are allowed.
 *
 * - OLED_MODE_USER:
 *   User code exclusively controls the OLED.
 *   System rendering is disabled.
 *   Only framebuffer-based APIs are expected.
 */
typedef enum {
    OLED_MODE_SYSTEM = 0,
    OLED_MODE_USER
} oled_mode_e;

/* ============================================================================
 * CORE OLED CONTROL
 * ============================================================================
 */

/**
 * @brief Initialize the OLED subsystem.
 *
 * Must be called once before using any OLED API.
 * This enables logical OLED usage; hardware initialization is handled
 * elsewhere during platform startup.
 */
void Oled_Init(void);

/**
 * @brief Clear the OLED display immediately.
 *
 * Clears the screen using a fast hardware command.
 * Works in both SYSTEM and USER modes.
 */
void Oled_Clear(void);

/**
 * @brief Flush framebuffer changes to OLED hardware.
 *
 * Performs a diff-based update, sending only modified bytes over I2C.
 * This function is valid only when the OLED is in USER mode.
 *
 * @param buffer Pointer to a 1024-byte framebuffer (128x64, SSD1306 layout)
 */
void Oled_Update(uint8_t *buffer);

/**
 * @brief Return OLED ownership to system firmware.
 *
 * After calling this:
 *  - System telemetry and UI rendering resumes
 *  - User graphics are cleared
 */
void Oled_SetMode_System(void);

/**
 * @brief Grant exclusive OLED ownership to user code.
 *
 * After calling this:
 *  - System rendering is disabled
 *  - User code may freely draw using framebuffer APIs
 */
void Oled_SetMode_User(void);



/** 
 * @brief Print a string at a specific column and row.
 *
 * Works only in SYSTEM mode.
 *
 * @param col Column (0–20)
 * @param row Row (1–6)
 * @param string String to print
 */
void Oled_Print(uint8_t col, uint8_t row, const char *string);

/* ============================================================================
 * DRAWING PRIMITIVES (FRAMEBUFFER)
 * ============================================================================
 */

/**
 * @brief Set or clear a single pixel in the framebuffer.
 *
 * @param buf Framebuffer pointer
 * @param x   X coordinate (0–127)
 * @param y   Y coordinate (0–63)
 * @param on  true = set pixel, false = clear pixel
 */
void Oled_DrawPixel(uint8_t *buf, int x, int y, bool on);

/* ---------------- Line primitives ---------------- */

/**
 * @brief Draw a horizontal line.
 */
void Oled_DrawHLine(uint8_t *buf, int x, int y, int length, bool on);

/**
 * @brief Draw a vertical line.
 */
void Oled_DrawVLine(uint8_t *buf, int x, int y, int length, bool on);

/**
 * @brief Draw a line between two points (Bresenham).
 */
void Oled_DrawLine(uint8_t *buf, int x0, int y0, int x1, int y1, bool on);

/* ---------------- Rectangle primitives ---------------- */

void Oled_DrawRect(uint8_t *buf, int x, int y, int w, int h, bool on);
void Oled_FillRect(uint8_t *buf, int x, int y, int w, int h, bool on);

/* ---------------- Rounded rectangles ---------------- */

void Oled_DrawRoundedRect(
    uint8_t *buf,
    int x,
    int y,
    int w,
    int h,
    int radius,
    bool on
);

void Oled_FillRoundedRect(
    uint8_t *buf,
    int x,
    int y,
    int w,
    int h,
    int radius,
    bool on
);

/* ---------------- Circle primitives ---------------- */

void Oled_DrawCircle(uint8_t *buf, int cx, int cy, int r, bool on);
void Oled_FillCircle(uint8_t *buf, int cx, int cy, int r, bool on);

/* ============================================================================
 * ICONS / COMPOSED DRAWINGS
 * ============================================================================
 */

/**
 * @brief Arrow directions for HUD indicators.
 */
typedef enum {
    OLED_ARROW_UP,
    OLED_ARROW_DOWN,
    OLED_ARROW_LEFT,
    OLED_ARROW_RIGHT
} oled_arrow_dir_t;

/**
 * @brief Draw a directional arrow.
 * @param buf   Framebuffer pointer
 * @param cx    Center X coordinate
 * @param cy    Center Y coordinate
 * @param size  Size of the arrow (length of shaft)
 * @param dir   Direction of the arrow
 * @param on    true = draw arrow, false = erase arrow 
 * 
 */
void Oled_DrawArrow(
    uint8_t *buf,
    int cx,
    int cy,
    int size,
    oled_arrow_dir_t dir,
    bool on
);

/* ============================================================================
 * EYES / EXPRESSIVE SHAPES
 * ============================================================================
 */

/**
 * @brief Draw a basic eye (filled or outline) with optional pupil offset.
 */
void Oled_DrawEye(
    uint8_t *buf,
    int cx,
    int cy,
    int radius,
    int pupilOffsetX,
    int pupilOffsetY,
    bool filled
);

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
);

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
);

/**
 * @brief Draw an X-shaped eye (error / dead state).
 */
void Oled_DrawXEye(
    uint8_t *buf,
    int cx,
    int cy,
    int size
);

/* ============================================================================
 * HUD HELPERS
 * ============================================================================
 */

/**
 * @brief Draw roll direction indicator.
 */
void Oled_DrawRollIndicator(uint8_t *buf, int roll);

/**
 * @brief Draw pitch direction indicator.
 */
void Oled_DrawPitchIndicator(uint8_t *buf, int pitch);

/**
 * @brief Draw joystick
 */
/**
 * @brief Draw RC joystick HUD.
 *
 * RC ranges expected: 1000–2000
 *
 * @param buf       Framebuffer (pass NULL to use internal OLED buffer)
 * @param throttle  RC throttle value
 * @param yaw       RC yaw value
 * @param roll      RC roll value
 * @param pitch     RC pitch value
 */
void Oled_DrawRCJoysticks(
    uint8_t *buf,
    int throttle,
    int yaw,
    int roll,
    int pitch
);

/* ============================================================================
 * MATH HELPERS
 * ============================================================================
 */

int Oled_Clamp(int v, int min, int max);
int Oled_Map(int v, int inMin, int inMax, int outMin, int outMax);

/* ============================================================================
 * GLOBAL STATE (SHARED)
 * ============================================================================
 */

/**
 * @brief Current OLED ownership mode.
 *
 * Defined in Oled.cpp.
 * Read by system renderer and user graphics layer.
 */
extern oled_mode_e oledMode;

#ifdef __cplusplus
}
#endif
