/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API\Debugging.h                                            #
 #  Created Date: Sun, 24th Aug 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Sun, 15th Nov 2026                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date         By                Comments                                    #
 #  ----------   ----------------  ------------------------------------------  #
 #  2025-09-XX   Omkar Dandekar     Remove Oled functions,check out            #
                                    Oled.h / Oled.cpp                          #
 #  2026-11-15   AJ                Documented precision 0..4, the 255-char     #
                                    cut and the print limits ( API 1.8.9 )     #
*******************************************************************************/

#ifndef DEBUGGING_H
#define DEBUGGING_H

#include <stdint.h>

/* -------------------------------------------------------------------------- */
/*  C / C++ compatibility                                                      */
/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/*  Monitor (Debug Serial) — C-safe APIs                                       */
/* -------------------------------------------------------------------------- */

/**
 * @brief Prints a message to the monitor.
 *
 * @param msg The null-terminated string to be printed; cut at 255 characters.
 *            NULL counts as an empty string.
 */
void Monitor_Print(const char *msg);

/**
 * @brief Prints a message followed by a new line to the monitor.
 *
 * @param msg The null-terminated string to be printed; cut at 255 characters.
 *            NULL counts as an empty string.
 *
 * The new line is sent as a second 5-byte message.
 */
void Monitor_Println(const char *msg);
 

#ifdef BLACKBOX
/**
 * @brief Associates a variable name with a reference to an integer,
 *        using the Black Box mechanism.
 *
 * @param varName A pointer to a null-terminated string.
 * @param reference Integer reference to expose to BlackBox.
 */
void BlackBox_setVar(char *varName, int32_t &reference);
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

/* -------------------------------------------------------------------------- */
/*  C++ ONLY Monitor overloads                                                 */
/*                                                                            */
/*  A text and its number are sent together as one message of at most 255     */
/*  characters; a longer tag is cut so the number always fits. Keep all the   */
/*  prints of one plutoLoop ( ) under ~130 bytes with the app connected, or   */
/*  the app disconnects. Byte counts: docs/API/DEBUGGING_API_WIKI.md.         */
/* -------------------------------------------------------------------------- */
#ifdef __cplusplus

/**
 * @brief Prints a tag followed by an integer number to the monitor.
 *
 * Every int prints in full, -2147483648 to 2147483647.
 * Pass a decimal number with a precision: Monitor_Print ( "v:", 3.7f ) has
 * two arguments, so it calls this version and prints 3.
 */
void Monitor_Print(const char *tag, int number);

/**
 * @brief Prints a tag followed by a double number with precision.
 *
 * @param precision Digits after the decimal point, 0 to 4; 5 or more prints 4.
 *                  The last digit is rounded half away from zero.
 *
 * Beyond +/-4294967040 the number prints as "overflow"; a NaN as
 * "not a number"; +/-infinity as "infinite".
 */
void Monitor_Print(const char *tag, double number, uint8_t precision);

/**
 * @brief Prints a tag and integer followed by a new line.
 */
void Monitor_Println(const char *tag, int number);

/**
 * @brief Prints a tag and double followed by a new line.
 *
 * @param precision Digits after the decimal point, 0 to 4; 5 or more prints 4.
 */
void Monitor_Println(const char *tag, double number, uint8_t precision);

#endif /* __cplusplus */

#endif /* DEBUGGING_H */
