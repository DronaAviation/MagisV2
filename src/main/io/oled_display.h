/*******************************************************************************
 #  Copyright (c) 2025 Drona Aviatino                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \ssd1306.h                                                           #
 #  Created Date: Sat, 25th Jan 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Fri, 8th May 2026                                           #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments # #  ----------	---
 ---------------------------------------------------------  # #  2026-05-08   AG
 Added comprehensive Doxygen documentation.                 #
 *******************************************************************************/
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * GLOBAL FLAGS & DATA
 * ============================================================================
 */

extern bool devmode;    /**< Development mode flag. */
extern bool OledEnable; /**< Global OLED subsystem enable flag. */

/**
 * @brief OLED operation modes.
 */
typedef enum {
  OLED_MODE_SYSTEM = 0, /**< System mode: displays firmware telemetry. */
  OLED_MODE_USER        /**< User mode: exclusive access for user drawings. */
} oled_mode_e;

extern oled_mode_e oledMode; /**< Current OLED ownership mode. */

/* ============================================================================
 * STARTUP & SYSTEM RENDERING
 * ============================================================================
 */

/**
 * @brief Initialize OLED hardware and record startup time.
 */
void OledStartUpInit ( void );

/**
 * @brief Display the startup splash screen for a set duration.
 */
void OledStartUpPage ( void );

/**
 * @brief Render detailed system telemetry (PID, RC data, IMU) to the display.
 */
void OledDisplaySystemData ( void );

/**
 * @brief Render the standard navigation/status HUD.
 */
void OledDisplayNav ( void );

/**
 * @brief Main entry point for system-mode rendering.
 *
 * Checks mode and timing before calling Nav and System rendering functions.
 */
void OledDisplayData ( void );

/* ============================================================================
 * MODE & OWNERSHIP CONTROL
 * ============================================================================
 */

/**
 * @brief Return OLED ownership to the system. Clears display if not armed.
 */
void Oled_SystemMode ( void );

/**
 * @brief Grant OLED ownership to user code. Clears display if not armed.
 */
void Oled_UserMode ( void );

/**
 * @brief Check if user code currently owns the OLED.
 * @return true if in USER mode.
 */
bool Oled_IsUserMode ( void );

/**
 * @brief Check if the system firmware currently owns the OLED.
 * @return true if in SYSTEM mode.
 */
bool Oled_IsSystemMode ( void );

/* ============================================================================
 * HARDWARE / BUFFER CONTROL
 * ============================================================================
 */

/**
 * @brief Send a hardware clear command to the display.
 */
void Oled_display_Clear ( void );

/**
 * @brief Push the contents of a 1024-byte framebuffer to the display.
 * @param buf Pointer to the source framebuffer.
 */
void Oled_display_Update ( uint8_t *buf );

/* ============================================================================
 * BASIC DRAWING PRIMITIVES
 * ============================================================================
 */

/**
 * @brief Set or clear a single pixel in the framebuffer.
 * @param screen Target buffer.
 * @param x Pixel X coordinate.
 * @param y Pixel Y coordinate.
 * @param on True to set, false to clear.
 */
void Oled_DrawPixel ( uint8_t *screen, int16_t x, int16_t y, bool on );

/**
 * @brief Draw a horizontal line.
 * @param screen Target buffer.
 * @param x Start X coordinate.
 * @param y Y coordinate.
 * @param length Line length in pixels.
 * @param on True to set, false to clear.
 */
void Oled_DrawHLine ( uint8_t *screen, int16_t x, int16_t y, int16_t length, bool on );

/**
 * @brief Draw a vertical line.
 * @param screen Target buffer.
 * @param x X coordinate.
 * @param y Start Y coordinate.
 * @param length Line length in pixels.
 * @param on True to set, false to clear.
 */
void Oled_DrawVLine ( uint8_t *screen, int16_t x, int16_t y, int16_t length, bool on );

/**
 * @brief Draw a line between two points using Bresenham's algorithm.
 * @param screen Target buffer.
 * @param x0 Start X.
 * @param y0 Start Y.
 * @param x1 End X.
 * @param y1 End Y.
 * @param on True to set, false to clear.
 */
void Oled_DrawLine ( uint8_t *screen, int16_t x0, int16_t y0, int16_t x1, int16_t y1, bool on );

/**
 * @brief Draw text using a 5x7 bitmap font.
 * @param screen Target buffer.
 * @param x X coordinate.
 * @param y Y coordinate.
 * @param text Null-terminated string.
 */
void Oled_DrawText ( uint8_t *screen, int16_t x, int16_t y, const char *text );

/**
 * @brief Draw an integer as text.
 * @param screen Target buffer.
 * @param x X coordinate.
 * @param y Y coordinate.
 * @param number Value to display.
 */
void Oled_DrawNumber ( uint8_t *screen, int16_t x, int16_t y, int16_t number );

/* ============================================================================
 * SHAPES (OUTLINE & FILLED)
 * ============================================================================
 */

/**
 * @brief Draw a rectangle outline.
 */
void Oled_DrawRect ( uint8_t *screen, int16_t x, int16_t y, int16_t w, int16_t h, bool on );

/**
 * @brief Draw a filled rectangle.
 */
void Oled_FillRect ( uint8_t *screen, int16_t x, int16_t y, int16_t w, int16_t h, bool on );

/**
 * @brief Draw a rounded rectangle outline.
 */
void Oled_DrawRoundedRect ( uint8_t *screen, int16_t x, int16_t y, int16_t w, int16_t h, int16_t radius, bool on );

/**
 * @brief Draw a filled rounded rectangle.
 */
void Oled_FillRoundedRect ( uint8_t *screen, int16_t x, int16_t y, int16_t w, int16_t h, int16_t radius, bool on );

/**
 * @brief Draw a circle outline using midpoint circle algorithm.
 */
void Oled_DrawCircle ( uint8_t *screen, int16_t cx, int16_t cy, int16_t radius, bool on );

/**
 * @brief Draw a filled circle.
 */
void Oled_FillCircle ( uint8_t *screen, int16_t cx, int16_t cy, int16_t radius, bool on );

/* ============================================================================
 * HUD & SPECIALIZED GRAPHICS
 * ============================================================================
 */

/**
 * @brief Draw a dual joystick visualization for RC input.
 */
void Oled_DrawRCJoysticks ( uint8_t *screen, uint16_t throttle, uint16_t yaw, uint16_t roll, uint16_t pitch );

/**
 * @brief Draw a horizontal horizon line indicating pitch.
 */
void Oled_DrawPitchIndicator ( uint8_t *buf, int16_t pitch );

/**
 * @brief Draw a vertical line indicating roll.
 */
void Oled_DrawRollIndicator ( uint8_t *buf, int16_t roll );

/**
 * @brief Directional arrow types.
 */
typedef enum {
  OLED_ARROW_UP,
  OLED_ARROW_DOWN,
  OLED_ARROW_LEFT,
  OLED_ARROW_RIGHT
} oled_arrow_dir_t;

/**
 * @brief Draw a directional arrow.
 */
void Oled_DrawArrow ( uint8_t *screen, int16_t cx, int16_t cy, int16_t size, oled_arrow_dir_t dir, bool on );

/* ============================================================================
 * EYE GRAPHICS PRESETS
 * ============================================================================
 */

/**
 * @brief Draw a circular eye with a movable pupil.
 */
void Oled_DrawEye ( uint8_t *buf, int16_t cx, int16_t cy, int16_t radius, int8_t pupilOffsetX, int8_t pupilOffsetY, bool filled );

/**
 * @brief Draw an X-shaped eye.
 */
void Oled_DrawXEye ( uint8_t *screen, int16_t cx, int16_t cy, int16_t size );

/**
 * @brief Draw a boxy (rounded rect) eye with a movable pupil.
 */
void Oled_DrawBoxyEye ( uint8_t *buf, int16_t cx, int16_t cy, int16_t w, int16_t h, int16_t r, int8_t pupilOffsetX, int8_t pupilOffsetY, bool filled );

/**
 * @brief Draw a boxy eye containing an X.
 */
void Oled_DrawBoxyXEye ( uint8_t *screen, int16_t cx, int16_t cy, int16_t w, int16_t h, int16_t r );

#ifdef __cplusplus
}
#endif
