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
 #  Last Modified: Sun, 24th Aug 2025                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/

#ifndef DEBUGGING_H
#define DEBUGGING_H

#include <stdint.h>

/**
 * @brief Prints a message to the monitor.
 *
 * @param msg The null-terminated string to be printed.
 */
void Monitor_Print ( const char *msg );

/**
 * @brief Prints a tag followed by an integer number to the monitor.
 *
 * @param tag A descriptive label for the number.
 * @param number The integer value to be printed.
 */
void Monitor_Print ( const char *tag, int number );

/**
 * @brief Prints a tag followed by a double number with specified precision to the monitor.
 *
 * @param tag A descriptive label for the number.
 * @param number The double value to be printed.
 * @param precision The number of decimal places for the double value.
 */
void Monitor_Print ( const char *tag, double number, uint8_t precision );

/**
 * @brief Prints a message followed by a new line to the monitor.
 *
 * @param msg The null-terminated string to be printed.
 */
void Monitor_Println ( const char *msg );

/**
 * @brief Prints a tag and an integer number followed by a new line to the monitor.
 *
 * @param tag A descriptive label for the number.
 * @param number The integer value to be printed.
 */
void Monitor_Println ( const char *tag, int number );

/**
 * @brief Prints a tag and a double number with specified precision followed by a new line to the monitor.
 *
 * @param tag A descriptive label for the number.
 * @param number The double value to be printed.
 * @param precision The number of decimal places for the double value.
 */
void Monitor_Println ( const char *tag, double number, uint8_t precision );

/**
 * @brief Initializes the OLED display for use.
 */
void Oled_Init ( void );

/**
 * @brief Clears the OLED display.
 */
void Oled_Clear ( void );

/**
 * @brief Prints a string at specified column and row on the OLED display.
 *
 * @param col The column position starting from 0.
 * @param row The row position starting from 0.
 * @param string The null-terminated string to be displayed.
 */
void Oled_Print ( uint8_t col, uint8_t row, const char *string );

#ifdef BLACKBOX
/**
 * @brief Associates a variable name with a reference to an integer,
 *        using the Black Box mechanism.
 *
 * This function sets a user-defined field in the Black Box by linking
 * the provided variable name with a reference to an integer value.
 * This can be useful for tracking or logging purposes.
 *
 * @param varName A pointer to a null-terminated string representing
 *                the name of the variable to be set in the Black Box.
 * @param reference A reference to the integer value that will be
 *                  associated with the variable name in the Black Box.
 */
void BlackBox_setVar ( char *varName, int32_t &reference );
#endif

#endif