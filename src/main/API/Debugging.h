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
 #  Date         By                Comments                                    #
 #  ----------   ----------------  ------------------------------------------  #
 #  2025-09-XX   Omkar Dandekar     Remove Oled functions,check out            #
                                    Oled.h / Oled.cpp                          #
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
 * @param msg The null-terminated string to be printed.
 */
void Monitor_Print(const char *msg);

/**
 * @brief Prints a message followed by a new line to the monitor.
 *
 * @param msg The null-terminated string to be printed.
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
/* -------------------------------------------------------------------------- */
#ifdef __cplusplus

/**
 * @brief Prints a tag followed by an integer number to the monitor.
 */
void Monitor_Print(const char *tag, int number);

/**
 * @brief Prints a tag followed by a double number with precision.
 */
void Monitor_Print(const char *tag, double number, uint8_t precision);

/**
 * @brief Prints a tag and integer followed by a new line.
 */
void Monitor_Println(const char *tag, int number);

/**
 * @brief Prints a tag and double followed by a new line.
 */
void Monitor_Println(const char *tag, double number, uint8_t precision);

#endif /* __cplusplus */

#endif /* DEBUGGING_H */
