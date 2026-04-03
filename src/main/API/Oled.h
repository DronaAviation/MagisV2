/*******************************************************************************
 *  Copyright (c) 2025 Drona Aviation
 *  All rights reserved.
 *  ---------------------------------------------------------------------------
 *  Author: Omkar Dandekar
 *  Project: MagisV2
 *  File: \src\main\API\Oled.h
 *  Created Date: Thu, 18th Dec 2025
 *  ---------------------------------------------------------------------------
 *  @brief  OLED API for MagisV2 — 128x64 SSD1306 display over I2C.
 *
 *  Simple usage (recommended):
 *  @code
 *    void plutoInit() { Oled_Init(); }
 *    void plutoLoop() {
 *      Oled_Begin();
 *      Oled_Text(10, 20, "Hello!");
 *      Oled_RobotEyes(0, 0);
 *      Oled_End();
 *    }
 *    void onLoopFinish() { Oled_SystemMode(); }
 *  @endcode
 *
 *  ---------------------------------------------------------------------------
 *  HISTORY:
 *  Date        By              Comments
 *  ----------  --------------  ----------------------------------------------
 *  2025-12-18  Omkar Dandekar  Initial API
 *  2026-04-03  Omkar Dandekar  Simple API layer, boxy eyes, DrawText/Number
 ******************************************************************************/

#pragma once

#include <stdint.h>
#include <stdbool.h>

/** @brief Screen width in pixels */
#define OLED_WIDTH   128
/** @brief Screen height in pixels */
#define OLED_HEIGHT   64

#ifdef __cplusplus
extern "C" {
#endif


/* ============================================================================
 *  SIMPLE API — start here, no buffer management needed
 * ============================================================================ */

/**
 * @brief Start a new OLED frame.
 * Clears the internal buffer and enters USER mode automatically.
 * Call this at the beginning of plutoLoop().
 */
void Oled_Begin ( void );

/**
 * @brief Finish the frame and send to display.
 * Only changed pixels are sent over I2C (non-blocking).
 * Call this at the end of plutoLoop().
 */
void Oled_End ( void );

/**
 * @brief Draw text at a pixel position.
 * Each character is 6px wide and 7px tall. Max ~21 chars per line.
 * @param x     X pixel position (0–127)
 * @param y     Y pixel position (0–63)
 * @param text  Null-terminated string
 */
void Oled_Text ( int x, int y, const char *text );

/**
 * @brief Draw an integer number at a pixel position.
 * Handles negative numbers. Each digit is 6px wide.
 * @param x       X pixel position (0–127)
 * @param y       Y pixel position (0–63)
 * @param number  Integer value to display
 */
void Oled_Number ( int x, int y, int number );

/**
 * @brief Draw a single pixel.
 * @param x  X position (0–127)
 * @param y  Y position (0–63)
 */
void Oled_Pixel ( int x, int y );

/**
 * @brief Draw a line between two points.
 * @param x0  Start X (0–127)
 * @param y0  Start Y (0–63)
 * @param x1  End X (0–127)
 * @param y1  End Y (0–63)
 */
void Oled_Line ( int x0, int y0, int x1, int y1 );

/**
 * @brief Draw a filled rectangle.
 * @param x  Top-left X (0–127)
 * @param y  Top-left Y (0–63)
 * @param w  Width in pixels
 * @param h  Height in pixels
 */
void Oled_Rect ( int x, int y, int w, int h );

/**
 * @brief Draw a filled rounded rectangle.
 * @param x  Top-left X (0–127)
 * @param y  Top-left Y (0–63)
 * @param w  Width in pixels
 * @param h  Height in pixels
 * @param r  Corner radius (0 = sharp, 6–10 = smooth)
 */
void Oled_RoundRect ( int x, int y, int w, int h, int r );

/**
 * @brief Draw a filled circle.
 * @param cx  Center X (0–127)
 * @param cy  Center Y (0–63)
 * @param r   Radius in pixels
 */
void Oled_Circle ( int cx, int cy, int r );

/**
 * @brief Draw a rectangle outline (not filled).
 * @param x  Top-left X
 * @param y  Top-left Y
 * @param w  Width
 * @param h  Height
 */
void Oled_RectOutline ( int x, int y, int w, int h );

/**
 * @brief Draw a rounded rectangle outline (not filled).
 * @param x  Top-left X
 * @param y  Top-left Y
 * @param w  Width
 * @param h  Height
 * @param r  Corner radius
 */
void Oled_RoundRectOutline ( int x, int y, int w, int h, int r );

/**
 * @brief Draw a circle outline (not filled).
 * @param cx  Center X
 * @param cy  Center Y
 * @param r   Radius
 */
void Oled_CircleOutline ( int cx, int cy, int r );

/**
 * @brief Draw two robot-style boxy eyes (centered on screen).
 * Pupils track with the given offset. Auto-clamped to stay inside.
 * @param pupilX  Horizontal pupil offset (-5 to +5, 0 = center)
 * @param pupilY  Vertical pupil offset (-5 to +5, 0 = center)
 */
void Oled_RobotEyes ( int pupilX, int pupilY );

/**
 * @brief Draw two round cartoon eyes (centered on screen).
 * Pupils track with the given offset. Auto-clamped to stay inside.
 * @param pupilX  Horizontal pupil offset (-5 to +5, 0 = center)
 * @param pupilY  Vertical pupil offset (-5 to +5, 0 = center)
 */
void Oled_RoundEyes ( int pupilX, int pupilY );

/**
 * @brief Draw two X-shaped dead eyes (crash / error state).
 */
void Oled_DeadEyes ( void );

/**
 * @brief Draw dual RC joystick visualization.
 * Left stick: yaw (X) + throttle (Y). Right stick: roll (X) + pitch (Y).
 * @param throttle  Throttle RC value (1000–2000)
 * @param yaw       Yaw RC value (1000–2000)
 * @param roll      Roll RC value (1000–2000)
 * @param pitch     Pitch RC value (1000–2000)
 */
void Oled_Joysticks ( int throttle, int yaw, int roll, int pitch );

/**
 * @brief Draw a horizontal pitch indicator line.
 * Line moves up/down based on pitch angle.
 * @param pitch  Pitch angle in degrees (-90 to +90)
 */
void Oled_PitchLine ( int pitch );

/**
 * @brief Draw a vertical roll indicator line.
 * Line moves left/right based on roll angle.
 * @param roll  Roll angle in degrees (-90 to +90)
 */
void Oled_RollLine ( int roll );

/**
 * @brief Erase (draw black) a rectangular area.
 * @param x  Top-left X
 * @param y  Top-left Y
 * @param w  Width
 * @param h  Height
 */
void Oled_Erase ( int x, int y, int w, int h );


/* ============================================================================
 *  SETUP & MODE CONTROL
 * ============================================================================ */

/** @brief Enable OLED subsystem. Call once in plutoInit(). */
void Oled_Init ( void );

/** @brief Clear the entire display. Skipped automatically if drone is armed. */
void Oled_Clear ( void );

/** @brief Return OLED to firmware telemetry. Call in onLoopFinish(). */
void Oled_SystemMode ( void );

/** @brief Take OLED for custom drawing. System rendering stops. */
void Oled_UserMode ( void );

/**
 * @brief Flush a custom framebuffer to the display.
 * Only changed pixels are sent. Works only in USER mode.
 * @param screen  Pointer to 1024-byte framebuffer
 */
void Oled_Update ( uint8_t *screen );

/** @brief Returns true if OLED is in USER mode (you own the screen). */
bool Oled_IsUserMode ( void );

/** @brief Returns true if OLED is in SYSTEM mode (firmware owns the screen). */
bool Oled_IsSystemMode ( void );


/* ============================================================================
 *  ADVANCED DRAWING — pass your own framebuffer
 *
 *  Use these if you need full control (multiple buffers, erase pixels, etc).
 *  For most users, the Simple API above is easier.
 * ============================================================================ */

/** @brief Print text on OLED grid. SYSTEM mode only.
 *  @param col   Column 0–20
 *  @param row   Row 1–6 (rows 0,7 are status bar)
 *  @param text  String to print */
void Oled_Print ( uint8_t col, uint8_t row, const char *text );

/** @brief Draw text into framebuffer at pixel position.
 *  @param screen  Framebuffer pointer
 *  @param x       X pixel position (0–127)
 *  @param y       Y pixel position (0–63)
 *  @param text    String to draw */
void Oled_DrawText ( uint8_t *screen, int x, int y, const char *text );

/** @brief Draw integer into framebuffer.
 *  @param screen  Framebuffer pointer
 *  @param x       X pixel position
 *  @param y       Y pixel position
 *  @param number  Integer (handles negative) */
void Oled_DrawNumber ( uint8_t *screen, int x, int y, int number );

/** @brief Set or clear a pixel.
 *  @param on  true = white, false = black */
void Oled_DrawPixel ( uint8_t *screen, int x, int y, bool on );

/** @brief Horizontal line. @param length  Pixels to the right */
void Oled_DrawHLine ( uint8_t *screen, int x, int y, int length, bool on );

/** @brief Vertical line. @param length  Pixels downward */
void Oled_DrawVLine ( uint8_t *screen, int x, int y, int length, bool on );

/** @brief Line between two points (Bresenham). */
void Oled_DrawLine ( uint8_t *screen, int x0, int y0, int x1, int y1, bool on );

/** @brief Rectangle outline. */
void Oled_DrawRect ( uint8_t *screen, int x, int y, int w, int h, bool on );

/** @brief Filled rectangle. */
void Oled_FillRect ( uint8_t *screen, int x, int y, int w, int h, bool on );

/** @brief Rounded rectangle outline. @param radius  Corner roundness */
void Oled_DrawRoundedRect ( uint8_t *screen, int x, int y, int w, int h, int radius, bool on );

/** @brief Filled rounded rectangle. @param radius  Corner roundness */
void Oled_FillRoundedRect ( uint8_t *screen, int x, int y, int w, int h, int radius, bool on );

/** @brief Circle outline. @param radius  Circle radius */
void Oled_DrawCircle ( uint8_t *screen, int cx, int cy, int radius, bool on );

/** @brief Filled circle. @param radius  Circle radius */
void Oled_FillCircle ( uint8_t *screen, int cx, int cy, int radius, bool on );

/** @brief Arrow directions */
typedef enum {
    OLED_ARROW_UP,     /**< Pointing up */
    OLED_ARROW_DOWN,   /**< Pointing down */
    OLED_ARROW_LEFT,   /**< Pointing left */
    OLED_ARROW_RIGHT   /**< Pointing right */
} oled_arrow_dir_t;

/** @brief Draw a directional arrow.
 *  @param size  Length of arrow shaft
 *  @param dir   OLED_ARROW_UP / DOWN / LEFT / RIGHT */
void Oled_DrawArrow ( uint8_t *screen, int cx, int cy, int size, oled_arrow_dir_t dir, bool on );

/** @brief Draw a round eye with pupil.
 *  @param radius          Eye radius (14–18 typical)
 *  @param pupilOffsetX    Horizontal pupil offset (-5 to +5)
 *  @param pupilOffsetY    Vertical pupil offset (-5 to +5)
 *  @param filled          true = solid eye, false = outline */
void Oled_DrawEye ( uint8_t *screen, int cx, int cy, int radius, int pupilOffsetX, int pupilOffsetY, bool filled );

/** @brief Draw X-shaped dead eye. @param size  Size of the X */
void Oled_DrawXEye ( uint8_t *screen, int cx, int cy, int size );

/** @brief Draw a boxy (rounded rect) eye with pupil.
 *  @param w    Eye width (36–40 typical)
 *  @param h    Eye height (28–30 typical)
 *  @param r    Corner radius (6 typical)
 *  @param pupilOffsetX  Horizontal pupil offset (-5 to +5)
 *  @param pupilOffsetY  Vertical pupil offset (-5 to +5)
 *  @param filled        true = solid eye, false = outline */
void Oled_DrawBoxyEye ( uint8_t *screen, int cx, int cy, int w, int h, int r, int pupilOffsetX, int pupilOffsetY, bool filled );

/** @brief Draw boxy X-shaped dead eye.
 *  @param r  Corner radius */
void Oled_DrawBoxyXEye ( uint8_t *screen, int cx, int cy, int w, int h, int r );

/** @brief Draw dual RC joystick HUD.
 *  @param throttle  RC value 1000–2000
 *  @param yaw       RC value 1000–2000
 *  @param roll      RC value 1000–2000
 *  @param pitch     RC value 1000–2000 */
void Oled_DrawRCJoysticks ( uint8_t *screen, int throttle, int yaw, int roll, int pitch );

/** @brief Pitch indicator line. @param pitch  Degrees -90 to +90 */
void Oled_DrawPitchIndicator ( uint8_t *screen, int pitch );

/** @brief Roll indicator line. @param roll  Degrees -90 to +90 */
void Oled_DrawRollIndicator ( uint8_t *screen, int roll );


/* ============================================================================
 *  INTERNAL — firmware only, not for user code
 * ============================================================================ */

typedef enum {
    OLED_MODE_SYSTEM = 0,
    OLED_MODE_USER
} oled_mode_e;

extern oled_mode_e oledMode;

#ifdef __cplusplus
}
#endif
