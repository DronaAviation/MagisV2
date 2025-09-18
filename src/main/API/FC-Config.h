/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API\FC-Config.h                                            #
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
#ifndef FC_CONFIG_H
#define FC_CONFIG_H

// Define a structure named PID to hold PID controller parameters
typedef struct {
  uint8_t p;    // Proportional gain
  uint8_t i;    // Integral gain
  uint8_t d;    // Derivative gain
} PID;

// Define an enumeration for different types of PID profiles
typedef enum {
  PID_ROLL,     // PID profile for roll control
  PID_PITCH,    // PID profile for pitch control
  PID_YAW,      // PID profile for yaw control
  PID_ALT,      // PID profile for altitude control
  PID_USER      // User-defined PID profile
} pid_profile_e;

typedef enum {
  LOW_BATTERY,          // Indicates a low battery condition, triggering actions to prevent power loss.
  INFLIGHT_LOW_BATTERY, // Indicates a low battery condition specifically when in flight, requiring immediate attention.
  CRASH,                // Represents a crash condition, prompting emergency procedures to minimize damage.
  ALL                   // Represents all failsafe conditions, used to enable or check all failsafes simultaneously.
} failsafe_e;


/**
 * @brief Sets the loop frequency within 3.5 to 2000 Hz.
 *
 * Constrains and converts the input frequency from Hz to milliHz
 * for precision, updating the global variable.
 *
 * @param frequency Loop frequency in Hz.
 */
void setUserLoopFrequency ( float frequency );

/**
 * @brief Sets PID gains for a specified profile into the provided PID structure.
 *
 * @param PROFILE The PID profile type to retrieve gains from.
 * @param pid Pointer to the PID structure to be updated.
 */
void get_PIDProfile ( pid_profile_e PROFILE, PID *pid );

/**
 * @brief Updates the PID profile with gains from the provided PID structure.
 *
 * @param PROFILE The PID profile type to update.
 * @param pid Pointer to the PID structure containing new gain values.
 */
void set_PIDProfile ( pid_profile_e PROFILE, PID *pid );

/**
 * @brief Initializes the PID profiles with default gain values for all axes.
 */
void setDefault_PIDProfile ( void );

/**
 * @brief Retrieves the desired angle for a given angle type.
 *
 * @param ANGLE An enumerated type specifying the angle type.
 * @return int32_t The angle value associated with the provided angle type.
 */
int32_t getDesiredAngle ( angle_e ANGLE );

/**
 * @brief Sets the desired angle for a given angle type.
 *
 * This function updates the relevant angle value based on
 * the specified angle type using a switch statement. It supports
 * setting roll, pitch, and yaw angles.
 *
 * @param ANGLE An enumerated type specifying the angle type.
 * @param angle The angle value to be set in deci-degrees.
 */
void setDesiredAngle ( angle_e ANGLE, int32_t angle );

/**
 * @brief Retrieves the desired rate for a given angle type.
 *
 * @param ANGLE An enumerated type specifying the angle type.
 * @return The desired rate value for the specified angle type.
 */
int32_t getDesiredRate ( angle_e ANGLE );

/**
 * @brief Sets the desired control rate for a specific angle.
 *
 * @param ANGLE An enumerated type specifying the angle type (e.g., roll, pitch, yaw).
 * @param rate The desired rate value to set for the specified angle type.
 */
void setDesiredRate ( angle_e ANGLE, int32_t rate );

/**
 * @brief Retrieves the desired position for a specific axis.
 *
 * This function returns the desired position based on the specified
 * axis. If the axis is Z, it returns the currently set altitude;
 * otherwise, it returns 0 as a default value.
 *
 * @param AXIS An enumerated type specifying the axis (e.g., X, Y, Z).
 * @return The desired position for the specified axis or 0 if not Z.
 */
int32_t getDesiredPositions ( axis_e AXIS );

/**
 * @brief Sets the desired position for a specific axis.
 *
 * This inline function sets the desired position for the given axis.
 * If the specified axis is Z, it updates the set altitude using the
 * provided position. For other axes, no action is taken.
 *
 * @param AXIS An enumerated type specifying the axis (e.g., X, Y, Z).
 * @param position The position value to set for the specified axis.
 */
void setDesiredPosition ( axis_e AXIS, int32_t position );

/**
 * @brief Sets the desired position relative to a specific axis.
 *
 * This inline function adjusts the desired position relative to the given
 * axis. If the specified axis is Z, it modifies the relative altitude
 * using the provided position value. For other axes, no action is executed.
 *
 * @param AXIS An enumerated type specifying the axis (e.g., X, Y, Z).
 * @param position The position value to adjust for the specified axis.
 */
void DesiredPosition_setRelative ( axis_e AXIS, int32_t position );

/**
 * @brief Enables specified failsafe conditions.
 *
 * This function enables specific failsafe mechanisms based on the provided
 * FAILSAFE type. If the FAILSAFE type is ALL, all failsafes are enabled.
 * Otherwise, only the specified failsafe will be enabled while others
 * remain unchanged.
 *
 * @param FAILSAFE The failsafe condition to enable.
 */
void Failsafe_enable ( failsafe_e FAILSAFE );

/**
 * @brief Disables specified failsafe conditions.
 *
 * This function disables specific failsafe mechanisms based on the provided
 * FAILSAFE type. If the FAILSAFE type is ALL, all failsafes are disabled.
 * Otherwise, only the specified failsafe will be disabled while others
 * retain their current state.
 *
 * @param FAILSAFE The failsafe condition to disable.
 */
void Failsafe_disable ( failsafe_e FAILSAFE );
#endif
