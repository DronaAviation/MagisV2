/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Omkar Dandekar                                                     #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API-Src\Oled.cpp                                           #
 #  Created Date: Thu, 18th Dec 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Fri, 8th May 2026                                           #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
 *******************************************************************************/

#include <stdint.h>
#include <string.h>

#include "API/Oled.h"
#include "io/oled_display.h"

/* ============================================================================
 * INTERNAL CONSTANTS
 * ============================================================================
 */

#define OLED_W     128
#define OLED_H     64
#define OLED_PAGES ( OLED_H / 8 )

/* ============================================================================
 * INTERNAL STATE
 * ============================================================================
 */

/* ============================================================================
 * SIMPLE API — internal framebuffer, no user management needed
 * ============================================================================
 */

static uint8_t oledSimpleBuffer [ 1024 ];

/**
 * @brief Initialize OLED subsystem.
 *
 * Enables logical OLED usage. Hardware initialization is assumed
 * to be performed elsewhere during platform startup.
 */
void Oled_Init ( void ) {
  OledEnable = true;
}

void Oled_Mode ( Oled_mode_e mode ) {
  switch ( mode ) {
    case System:
      Oled_SystemMode ( );
      break;
    case User:
      Oled_UserMode ( );
      break;
    default:
      break;
  }
}

bool Oled_Is_Mode ( Oled_mode_e mode ) {
  switch ( mode ) {
    case System:
      return Oled_IsSystemMode ( );
    case User:
      return Oled_IsUserMode ( );
    default:
      return false;
  }
}

void Oled_Clear ( void ) {
  memset ( oledSimpleBuffer, 0, sizeof ( oledSimpleBuffer ) );
  // Oled_display_Clear ( );
}

void Oled_Update ( void ) {
  Oled_display_Update ( oledSimpleBuffer );
}

/* --- Simple text --- */
void Oled_Text ( int16_t x, int16_t y, const char *text ) {
  Oled_DrawText ( oledSimpleBuffer, x, y, text );
}
void Oled_Number ( int16_t x, int16_t y, int16_t number ) {
  Oled_DrawNumber ( oledSimpleBuffer, x, y, number );
}

/* --- Simple shapes (filled) --- */
void Oled_Pixel ( int16_t x, int16_t y ) {
  Oled_DrawPixel ( oledSimpleBuffer, x, y, true );
}
void Oled_Line ( int16_t x0, int16_t y0, int16_t x1, int16_t y1 ) {
  Oled_DrawLine ( oledSimpleBuffer, x0, y0, x1, y1, true );
}
void Oled_Rect ( int16_t x, int16_t y, int16_t w, int16_t h ) {
  Oled_FillRect ( oledSimpleBuffer, x, y, w, h, true );
}
void Oled_RoundRect ( int16_t x, int16_t y, int16_t w, int16_t h, int16_t r ) {
  Oled_FillRoundedRect ( oledSimpleBuffer, x, y, w, h, r, true );
}
void Oled_Circle ( int16_t cx, int16_t cy, int16_t r ) {
  Oled_FillCircle ( oledSimpleBuffer, cx, cy, r, true );
}

/* --- Simple outlines --- */
void Oled_RectOutline ( int16_t x, int16_t y, int16_t w, int16_t h ) {
  Oled_DrawRect ( oledSimpleBuffer, x, y, w, h, true );
}
void Oled_RoundRectOutline ( int16_t x, int16_t y, int16_t w, int16_t h, int16_t r ) {
  Oled_DrawRoundedRect ( oledSimpleBuffer, x, y, w, h, r, true );
}
void Oled_CircleOutline ( int16_t cx, int16_t cy, int16_t r ) {
  Oled_DrawCircle ( oledSimpleBuffer, cx, cy, r, true );
}

/* --- Eye presets --- */
void Oled_RobotEyes ( int8_t pupilX, int8_t pupilY ) {
  Oled_DrawBoxyEye ( oledSimpleBuffer, 38, 32, 40, 30, 6, pupilX, pupilY, true );
  Oled_DrawBoxyEye ( oledSimpleBuffer, 90, 32, 40, 30, 6, pupilX, pupilY, true );
}
void Oled_RoundEyes ( int8_t pupilX, int8_t pupilY ) {
  Oled_DrawEye ( oledSimpleBuffer, 38, 32, 18, pupilX, pupilY, true );
  Oled_DrawEye ( oledSimpleBuffer, 90, 32, 18, pupilX, pupilY, true );
}
void Oled_DeadEyes ( void ) {
  Oled_DrawBoxyXEye ( oledSimpleBuffer, 38, 32, 36, 28, 6 );
  Oled_DrawBoxyXEye ( oledSimpleBuffer, 90, 32, 36, 28, 6 );
}

/* --- Simple HUD --- */
void Oled_Joysticks ( uint16_t throttle, uint16_t yaw, uint16_t roll, uint16_t pitch ) {
  Oled_DrawRCJoysticks ( oledSimpleBuffer, throttle, yaw, roll, pitch );
}
void Oled_PitchLine ( int16_t pitch ) {
  Oled_DrawPitchIndicator ( oledSimpleBuffer, pitch );
}
void Oled_RollLine ( int16_t roll ) {
  Oled_DrawRollIndicator ( oledSimpleBuffer, roll );
}

/* --- Erase --- */
void Oled_Erase ( int16_t x, int16_t y, int16_t w, int16_t h ) {
  Oled_FillRect ( oledSimpleBuffer, x, y, w, h, false );
}