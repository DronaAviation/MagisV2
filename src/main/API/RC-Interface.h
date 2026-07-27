/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API\RC-Interface.h                                         #
 #  Created Date: Sat, 23rd Aug 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Thu, 18th Sep 2025                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/
#ifndef RC_INTERFACE_H
#define RC_INTERFACE_H

#include <stdint.h>

// Define an enumeration for remote control channels
typedef enum {

  RC_ROLL,        // Roll channel index
  RC_PITCH,       // Pitch channel index
  RC_YAW,         // Yaw channel index
  RC_THROTTLE,    // Throttle channel index
  RC_AUX1,        // Auxiliary channel 1 index
  RC_AUX2,        // Auxiliary channel 2 index
  RC_AUX3,        // Auxiliary channel 3 index
  RC_AUX4,        // Auxiliary channel 4 index
  RC_USER1,       // User-defined channel 1 index
  RC_USER2,       // User-defined channel 2 index
  RC_USER3        // User-defined channel 3 index

} rc_channel_e;

/**
 * @brief Copies the global rcData array to a local array.
 *
 * Creates a local copy of `rcData`. Note: Returning this local pointer
 * leads to undefined behavior due to scope.
 *
 * @return Pointer to the local copied array.
 */
int16_t *RcData_Get ( void );

/**
 * @brief Gets a channel's value from rcData.
 *
 * Returns the value at the `CHANNEL` index in `rcData`, identified by
 * `rc_channel_e`.
 *
 * @param CHANNEL The `rc_channel_e` type channel index.
 *
 * @return Value from the specified `CHANNEL` in `rcData`.
 */
int16_t RcData_Get ( rc_channel_e CHANNEL );

/**
 * @brief Gets a channel's offset value from rcCommand.
 *
 * Returns the value at `CHANNEL` index in `rcCommand`, offset by 1500,
 * if `CHANNEL` is valid. Otherwise, returns 0.
 *
 * @param CHANNEL The `rc_channel_e` type channel index.
 *
 * @return Offset value from `rcCommand` or 0 if invalid.
 */

int16_t *RcCommand_Get ( void );

/**
 * @brief Copies first four elements of the global rcCommand array.
 *
 * Copies values from `rcCommand` to a local array.
 * Returning this local array's address causes undefined behavior.
 *
 * @return Pointer to an `int16_t` array, leading to undefined behavior.
 */

int16_t RcCommand_Get ( rc_channel_e CHANNEL );

/**
 * @brief Sets `rcCommand` and `rcData` from input RC values.
 *
 * Processes first four elements of `rcValueArray`, constraining between 1000-2000.
 * Adjusts first three by subtracting 1500 for `rcCommand`; assigns fourth to `rcData`.
 * Values stored in `RC_ARRAY`, sets `userRCflag` to `true`.
 *
 * @param rcValueArray Pointer to an array of `int16_t` with raw RC values.
 */

void RcCommand_Set ( int16_t *rcValueArray );

/**
 * @brief Sets RC command for a channel.
 *
 * Constrains `rcValue` between 1000-2000. If below `RC_THROTTLE`, adjusts by subtracting
 * 1500 and stores in `rcCommand`. Stores final value in `RC_ARRAY`, sets flag. If
 * `RC_THROTTLE`, assigns directly to `rcData`.
 *
 * The override never locks the pilot out. It is cross-faded against the
 * pilot's own sticks by how far they are deflected:
 *
 *   - sticks centred        — the commanded value, exactly as asked for
 *   - stick part deflected  — proportional mix of command and pilot
 *   - stick half deflected  — the pilot alone, full manual authority
 *
 * Full authority is reached at `USER_RC_STICK_TRAVEL` counts off centre (half
 * stick), so a moderate command is already flying the other way before then.
 *
 * Throttle deflection is measured from wherever the stick sat when the
 * override latched, since it does not self-centre.
 *
 * The override is also dropped automatically if it stops being re-asserted,
 * so call this every `plutoLoop()` for as long as you want the channel held
 * and simply stop calling it to hand the channel back.
 *
 * Channels above `RC_THROTTLE` have no override storage and are ignored.
 *
 * @param CHANNEL The RC channel (`rc_channel_e` type), `RC_ROLL`..`RC_THROTTLE`.
 * @param rcValue Raw RC value for the channel.
 */

void RcCommand_Set ( rc_channel_e CHANNEL, int16_t rcValue );

/**
 * @brief Gets the application's current heading.
 * Returns `appHeading` indicating the orientation in degrees.
 * @return Current heading as `int16_t`.
 */

int16_t App_getAppHeading ( void );

/**
 * @brief Checks if the arm switch is active.
 * Evaluates `BOXARM` to determine status.
 * @return `true` if active, `false` otherwise.
 */

bool App_isArmSwitchOn ( void );

#endif
