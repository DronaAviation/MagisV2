/*******************************************************************************
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Omkar Dandekar                                                     #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API\Oled.h                                                 #
 #  Created Date: Thu, 18th Dec 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Tue, 5th May 2026                                           #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
 *******************************************************************************/

#ifndef OLED_API_H
#define OLED_API_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  System,
  User
} Oled_mode_e;

/* ============================================================================
 *  SETUP & MODE CONTROL
 * ============================================================================ */

/**
 * @brief Enables the OLED subsystem.
 *
 * This function should be called once during the application initialization
 * (typically in `plutoInit`) to enable the OLED display logic.
 */

void Oled_Init ( void );

/**
 * @brief Sets the current OLED operation mode.
 *
 * Switches between `System` mode (firmware telemetry) and `User` mode
 * (custom drawing).
 *
 * @param mode The desired OLED mode (`Oled_mode_e`).
 */

void Oled_Mode ( Oled_mode_e mode );

/**
 * @brief Checks if the OLED is currently in a specific mode.
 *
 * @param mode The mode to check against.
 * @return `true` if the OLED is in the specified mode, `false` otherwise.
 */

bool Oled_Is_Mode ( Oled_mode_e mode );

/**
 * @brief Clears the entire OLED display.
 *
 * This operation is automatically skipped if the drone is currently armed
 * to prevent I2C contention.
 */

void Oled_Clear ( void );

/**
 * @brief Finalizes the current frame and updates the display.
 *
 * Transmits only the changed pixels over I2C to the display hardware.
 * This should be called at the end of every loop (typically in `plutoLoop`).
 */

void Oled_Update ( void );

/**
 * @brief Prints text directly to the OLED grid.
 *
 * This function is intended for `System` mode only.
 *
 * @param col Target text column (0–20).
 * @param row Target text row (1–6; rows 0 and 7 are reserved for status).
 * @param text The null-terminated string to print.
 */

void Oled_Print ( uint8_t col, uint8_t row, const char *text );

/**
 * @brief Draws text at a specific pixel position.
 *
 * Each character is approximately 6px wide and 7px tall.
 * Max capacity is around 21 characters per line.
 *
 * @param x X pixel coordinate (0–127).
 * @param y Y pixel coordinate (0–63).
 * @param text The null-terminated string to be displayed.
 */

void Oled_Text ( int16_t x, int16_t y, const char *text );

/**
 * @brief Draws a signed integer number at a pixel position.
 *
 * Automatically handles negative signs. Each digit is 6px wide.
 *
 * @param x X pixel coordinate (0–127).
 * @param y Y pixel coordinate (0–63).
 * @param number The 16-bit signed integer value to display.
 */

void Oled_Number ( int16_t x, int16_t y, int16_t number );

/**
 * @brief Draws a single pixel on the screen.
 *
 * @param x X pixel coordinate (0–127).
 * @param y Y pixel coordinate (0–63).
 */

void Oled_Pixel ( int16_t x, int16_t y );

/**
 * @brief Draws a line between two specified points.
 *
 * @param x0 Start X coordinate (0–127).
 * @param y0 Start Y coordinate (0–63).
 * @param x1 End X coordinate (0–127).
 * @param y1 End Y coordinate (0–63).
 */

void Oled_Line ( int16_t x0, int16_t y0, int16_t x1, int16_t y1 );

/**
 * @brief Draws a filled rectangle.
 *
 * @param x Top-left X coordinate (0–127).
 * @param y Top-left Y coordinate (0–63).
 * @param w Width of the rectangle in pixels.
 * @param h Height of the rectangle in pixels.
 */

void Oled_Rect ( int16_t x, int16_t y, int16_t w, int16_t h );

/**
 * @brief Draws a filled rectangle with rounded corners.
 *
 * @param x Top-left X coordinate (0–127).
 * @param y Top-left Y coordinate (0–63).
 * @param w Width of the rectangle in pixels.
 * @param h Height of the rectangle in pixels.
 * @param r Corner radius in pixels (0 for sharp, 6–10 recommended for smooth).
 */

void Oled_RoundRect ( int16_t x, int16_t y, int16_t w, int16_t h, int16_t r );

/**
 * @brief Draws a filled circle.
 *
 * @param cx Center X coordinate (0–127).
 * @param cy Center Y coordinate (0–63).
 * @param r Radius of the circle in pixels.
 */

void Oled_Circle ( int16_t cx, int16_t cy, int16_t r );

/**
 * @brief Draws the outline of a rectangle.
 *
 * @param x Top-left X coordinate.
 * @param y Top-left Y coordinate.
 * @param w Width in pixels.
 * @param h Height in pixels.
 */

void Oled_RectOutline ( int16_t x, int16_t y, int16_t w, int16_t h );

/**
 * @brief Draws the outline of a rounded rectangle.
 *
 * @param x Top-left X coordinate.
 * @param y Top-left Y coordinate.
 * @param w Width in pixels.
 * @param h Height in pixels.
 * @param r Corner radius in pixels.
 */

void Oled_RoundRectOutline ( int16_t x, int16_t y, int16_t w, int16_t h, int16_t r );

/**
 * @brief Draws the outline of a circle.
 *
 * @param cx Center X coordinate.
 * @param cy Center Y coordinate.
 * @param r Radius in pixels.
 */

void Oled_CircleOutline ( int16_t cx, int16_t cy, int16_t r );

/**
 * @brief Draws two robot-style boxy eyes centered on the screen.
 *
 * Pupils track with the provided offset and are auto-clamped to the eye bounds.
 *
 * @param pupilX Horizontal pupil offset (-5 to +5, 0 = center).
 * @param pupilY Vertical pupil offset (-5 to +5, 0 = center).
 */

void Oled_RobotEyes ( int8_t pupilX, int8_t pupilY );

/**
 * @brief Draws two round cartoon-style eyes centered on the screen.
 *
 * Pupils track with the provided offset and are auto-clamped to the eye bounds.
 *
 * @param pupilX Horizontal pupil offset (-5 to +5, 0 = center).
 * @param pupilY Vertical pupil offset (-5 to +5, 0 = center).
 */

void Oled_RoundEyes ( int8_t pupilX, int8_t pupilY );

/**
 * @brief Draws two X-shaped eyes to indicate a "dead" or error state.
 */

void Oled_DeadEyes ( void );

/**
 * @brief Draws a visualization of dual RC joysticks.
 *
 * Left stick displays yaw (X) and throttle (Y).
 * Right stick displays roll (X) and pitch (Y).
 *
 * @param throttle Current throttle RC value (1000–2000).
 * @param yaw Current yaw RC value (1000–2000).
 * @param roll Current roll RC value (1000–2000).
 * @param pitch Current pitch RC value (1000–2000).
 */

void Oled_Joysticks ( uint16_t throttle, uint16_t yaw, uint16_t roll, uint16_t pitch );

/**
 * @brief Draws a horizontal line indicating the current pitch.
 *
 * The line shifts vertically based on the pitch angle.
 *
 * @param pitch Current pitch angle in degrees (-90 to +90).
 */

void Oled_PitchLine ( int16_t pitch );

/**
 * @brief Draws a vertical line indicating the current roll.
 *
 * The line shifts horizontally based on the roll angle.
 *
 * @param roll Current roll angle in degrees (-90 to +90).
 */

void Oled_RollLine ( int16_t roll );

/**
 * @brief Erases a specific rectangular area by drawing it black.
 *
 * @param x Top-left X coordinate.
 * @param y Top-left Y coordinate.
 * @param w Width of the area to erase.
 * @param h Height of the area to erase.
 */

void Oled_Erase ( int16_t x, int16_t y, int16_t w, int16_t h );

#endif
