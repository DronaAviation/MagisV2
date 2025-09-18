/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API\FC-Control.h                                           #
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
#ifndef FC_CONTROL_H
#define FC_CONTROL_H

// Define an enumeration for various flight statuses.
typedef enum {
  FS_ACCEL_GYRO_CALIBRATION = 0,    // Indicates that the accelerometer and gyroscope are being calibrated.
  FS_MAG_CALIBARATION,              // Indicates that the magnetometer is being calibrated.
  FS_LOW_BATTERY,                   // Indicates a low battery condition.
  FS_INFLIGHT_LOW_BATTERY,          // Indicates a low battery condition while in flight.
  FS_CRASHED,                       // Indicates that the aircraft has crashed.
  FS_SIGNAL_LOSS,                   // Indicates a loss of control signal.
  FS_NOT_OK_TO_ARM,                 // Indicates that it's not safe to arm the aircraft.
  FS_OK_TO_ARM,                     // Indicates that it's safe to arm the aircraft.
  FS_ARMED                          // Indicates that the aircraft is armed.
} flightstatus_e;

// Define an enumeration for different flight modes
typedef enum {
  ANGLE,            // Angle mode: stabilizes the craft to maintain level flight
  RATE,             // Rate mode: allows manual control of the craft's rate of rotation
  MAGHOLD,          // Magnetic hold mode: maintains the current heading using a compass
  HEADFREE,         // Head-free mode: controls are independent of the craft's orientation
  ATLTITUDEHOLD,    // Altitude hold mode: maintains a set altitude automatically
  THROTTLE_MODE     // Throttle mode: defines how throttle input is handled
} flight_mode_e;

// Define an enumeration for flip directions
typedef enum {
  BACK_FLIP    // Back flip direction: performs a back flip maneuver
} flip_direction_e;

/**
 * @brief Gets the current flight status.
 *
 * Extracts the least significant bit from `flightIndicatorFlag` and casts to `flightstatus_e`.
 *
 * @return Current `flightstatus_e`.
 */
flightstatus_e FlightStatus_Get ( void );

/**
 * @brief Evaluates a flight status.
 *
 * Casts the input to `FlightStatus_e` and uses `status_FSI` to check validity.
 *
 * @param status Flight status of type `flightstatus_e`.
 * @return True if the status meets criteria, otherwise false.
 */
bool FlightStatus_Check ( flightstatus_e status );

/**
 * @brief Checks if the specified flight mode is active.
 *
 * This function evaluates the given flight mode by comparing it against various predefined modes,
 * returning a boolean value indicating its activation status.
 *
 * @param MODE The flight mode to check, of type `flight_mode_e`.
 * @return True if the specified mode is active, false otherwise.
 */
bool FlightMode_Check ( flight_mode_e MODE );

/**
 * @brief Sets the specified flight mode and updates user flight mode states.
 *
 * This function activates or deactivates specific flight modes based on the input,
 * while also updating an array that tracks which user-defined flight modes are currently set.
 *
 * @param MODE The flight mode to set, of type `flight_mode_e`.
 */
void FlightMode_Set ( flight_mode_e MODE );

/**
 * @brief Initiates a take-off command to a specified height.
 *
 * This function sets the current command to TAKE_OFF, updates the command status,
 * constrains the target height within a specified range, and sets the take-off height.
 *
 * @param height The desired take-off height in units, constrained between 100 and 250.
 */
void Command_TakeOff ( uint16_t height = 150 );

/**
 * @brief Initiates a landing sequence with a specified speed.
 *
 * This function sets the current command to LAND and updates the command status,
 * adjusting the throttle based on the provided landing speed, only if certain conditions are met.
 *
 * @param landSpeed The speed at which the craft should land.
 */
void Command_Land ( uint8_t landSpeed = 105 );

/**
 * @brief Initiates a flip maneuver in a specified direction.
 *
 * This function sets the current command to B_FLIP but does not perform additional actions.
 *
 * @param direction The direction in which to perform the flip, of type `flip_direction_e`.
 */
void Command_Flip ( flip_direction_e direction );

/**
 * @brief Attempts to arm the system.
 *
 * This function checks if arming conditions are met, resets PID errors, and arms the system.
 * It returns the arming status.
 *
 * @return True if the system is armed, false otherwise.
 */
bool Command_Arm ( void );

/**
 * @brief Disarms the system.
 *
 * This function disarms the system regardless of its current state.
 * It returns the negation of the arming status.
 *
 * @return True if the system is disarmed, false otherwise.
 */
bool Command_DisArm ( void );

/**
 * @brief Sets the head-free mode heading.
 *
 * Assigns the provided heading to userHeadFreeHoldHeading
 * and updates the flag to indicate it's set.
 *
 * @param heading The desired heading as int16_t.
 */
void setheadFreeModeHeading ( int16_t heading );

#endif